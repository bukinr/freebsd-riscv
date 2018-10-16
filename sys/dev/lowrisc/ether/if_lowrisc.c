/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2008-2012 Juli Mallett <jmallett@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include "opt_inet.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/mbuf.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/callout.h>

#include <net/bpf.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_var.h>
#include <net/if_vlan_var.h>

#ifdef INET
#include <netinet/in.h>
#include <netinet/if_ether.h>
#endif

#include <riscv/include/bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/lowrisc/ether/lowrisc_reg.h>

struct lowrisc_softc {
	struct ifnet *sc_ifp;
	device_t sc_dev;
	unsigned sc_port;
	int sc_flags;
	struct ifmedia sc_ifmedia;
	struct resource *sc_intr, *sc_mem;
	void *sc_intr_cookie;
	struct mtx sc_mtx;
};

#define	LOWRISC_ETHER_LOCK(sc)	mtx_lock(&(sc)->sc_mtx)
#define	LOWRISC_ETHER_UNLOCK(sc)	mtx_unlock(&(sc)->sc_mtx)

static int	lowrisc_probe(device_t);
static int	lowrisc_attach(device_t);
static int	lowrisc_detach(device_t);
static int	lowrisc_shutdown(device_t);

static void	lowrisc_init(void *);
static int	lowrisc_transmit(struct ifnet *, struct mbuf *);

static int	lowrisc_medchange(struct ifnet *);
static void	lowrisc_medstat(struct ifnet *, struct ifmediareq *);

static int	lowrisc_ioctl(struct ifnet *, u_long, caddr_t);

static void	lowrisc_rx_intr(void *);

static int polltime;
static struct callout net_callout;

static device_method_t lowrisc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		lowrisc_probe),
	DEVMETHOD(device_attach,	lowrisc_attach),
	DEVMETHOD(device_detach,	lowrisc_detach),
	DEVMETHOD(device_shutdown,	lowrisc_shutdown),

	{ 0, 0 }
};

static driver_t lowrisc_driver = {
	"eth",
	lowrisc_methods,
	sizeof (struct lowrisc_softc),
};

static devclass_t lowrisc_devclass;

DRIVER_MODULE(lowrisc_eth, simplebus, lowrisc_driver, lowrisc_devclass, 0, 0);

#ifdef DEBUG_LOWRISC_ETHER
static uint64_t dbg_generic_bs_r_8(device_t dev, uint64_t (*f)(void *, bus_space_handle_t, bus_size_t), void *c, bus_space_handle_t b, bus_size_t o)
{
uint64_t res = f(c, b, o);
device_printf(dev, "dbg_generic_bs_r_8(0x%lx) => 0x%lx\n", o, res);
return res;
}

static void dbg_generic_bs_w_8(device_t dev, void (*f)(void *, bus_space_handle_t, bus_size_t, uint64_t), void *c, bus_space_handle_t b, bus_size_t o, uint64_t v)
{
device_printf(dev, "dbg_generic_bs_w_8(0x%lx,0x%lx)\n", o, v);
f(c, b, o, v) ;
}
#endif

static int
lowrisc_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

        if (!ofw_bus_is_compatible(dev, "lowrisc-eth"))
                return (ENXIO);

	device_set_desc(dev, "LowRISC test Ethernet");

	return (BUS_PROBE_DEFAULT);
}

static void
net_timeout(void *arg)
{
        struct lowrisc_softc *sc = arg;
        struct ifnet *ifp = sc->sc_ifp;

        if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == IFF_DRV_RUNNING)
	{
		if (0) device_printf(sc->sc_dev, "net_timer\n");
	    	lowrisc_rx_intr(sc);
	}

        callout_reset(&net_callout, polltime, net_timeout, arg);
}

static int
lowrisc_attach(device_t dev)
{
        uint64_t status;
	struct ifnet *ifp;
	struct lowrisc_softc *sc;
	uint8_t mac[6];
	uint32_t macaddr_lo, macaddr_hi;
	int error;
	int rid;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	sc->sc_port = device_get_unit(dev);

	device_printf(dev, "Try to allocate eth mem\n");

	/* Allocate and establish memory.  */
	rid = 0;
	sc->sc_mem = bus_alloc_resource_any(sc->sc_dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_mem == NULL) {
		device_printf(dev, "unable to allocate memory.\n");
		return (ENXIO);
	}

	rid = 0;
	sc->sc_intr = bus_alloc_resource_any(sc->sc_dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->sc_intr == NULL) {
		device_printf(dev, "unable to allocate IRQ.\n");
		return (ENXIO);
	}

	device_printf(dev, "Try to allocate eth mac address\n");

	/* Read MAC address.  */
        macaddr_lo = __bswap32(GETREG(sc, MACLO_OFFSET));
        macaddr_hi = __bswap16(GETREG(sc, MACHI_OFFSET) & MACHI_MACADDR_MASK);
        memcpy (mac+2, &macaddr_lo, sizeof(uint32_t));
        memcpy (mac+0, &macaddr_hi, sizeof(uint16_t));

	/* discard pending packets from boot loader */
	status = GETREG(sc, RSR_OFFSET);
        while (status & RSR_RECV_DONE_MASK) {
                uint32_t buf = status & RSR_RECV_FIRST_MASK;
		uint32_t off = RPLR_OFFSET+((buf&7)<<3);
		uint32_t length = GETREG(sc, off) & RPLR_LENGTH_MASK;
                device_printf(sc->sc_dev, "Discarded buffer %d of length %d (off=%x)\n",
			buf, length, off);
		SETREG(sc, RSR_OFFSET, buf+1);
                status = GETREG(sc, RSR_OFFSET);
 		}

	error = bus_setup_intr(sc->sc_dev, sc->sc_intr, INTR_TYPE_NET, NULL,
	    lowrisc_rx_intr, sc, &sc->sc_intr_cookie);
	if (error != 0) {
		device_printf(dev, "unable to setup interrupt.\n");
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->sc_intr);
		return (ENXIO);
	}

	bus_describe_intr(sc->sc_dev, sc->sc_intr, sc->sc_intr_cookie, "rx");

	ifp = if_alloc(IFT_ETHER);
	if (ifp == NULL) {
		device_printf(dev, "cannot allocate ifnet.\n");
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->sc_intr);
		return (ENOMEM);
	}

	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_mtu = ETHERMTU;
	ifp->if_init = lowrisc_init;
	ifp->if_softc = sc;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST | IFF_ALLMULTI;
	ifp->if_ioctl = lowrisc_ioctl;

	sc->sc_ifp = ifp;
	sc->sc_flags = ifp->if_flags;

	ifmedia_init(&sc->sc_ifmedia, 0, lowrisc_medchange, lowrisc_medstat);

	ifmedia_add(&sc->sc_ifmedia, IFM_ETHER | IFM_AUTO, 0, NULL);
	ifmedia_set(&sc->sc_ifmedia, IFM_ETHER | IFM_AUTO);

	mtx_init(&sc->sc_mtx, "LowRISC Ethernet", NULL, MTX_DEF);

	ether_ifattach(ifp, mac);

	ifp->if_transmit = lowrisc_transmit;

	polltime = 1;
        callout_init(&net_callout, 1);
        callout_reset(&net_callout, polltime, net_timeout, sc);

	return (bus_generic_attach(dev));
}

static int
lowrisc_detach(device_t dev)
{
	struct lowrisc_softc *sc;

	sc = device_get_softc(dev);

	bus_release_resource(dev, SYS_RES_IRQ, 0, sc->sc_intr);
	/* XXX Incomplete.  */

	return (0);
}

static int
lowrisc_shutdown(device_t dev)
{
	return (lowrisc_detach(dev));
}

static void
lowrisc_init(void *arg)
{
	struct ifnet *ifp;
	struct lowrisc_softc *sc;

	sc = arg;
	ifp = sc->sc_ifp;

	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) != 0)
		ifp->if_drv_flags &= ~IFF_DRV_RUNNING;

	ifp->if_drv_flags |= IFF_DRV_RUNNING;

	device_printf(sc->sc_dev, "lowrisc_init called\n");

	/*
        SETREG(sc, MACHI_OFFSET, MACHI_IRQ_EN | GETREG(sc, MACHI_OFFSET));
	*/
}

static int
lowrisc_transmit(struct ifnet *ifp, struct mbuf *m)
{
	struct lowrisc_softc *sc;
	uint64_t *alloc;
	int i, words;

	sc = ifp->if_softc;

	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) != IFF_DRV_RUNNING) {
		m_freem(m);
		return (0);
	}

	LOWRISC_ETHER_LOCK(sc);
	words = (((m->m_pkthdr.len-1)|7)+1)/8;
	alloc = (uint64_t *)(m->m_data);
        for (i = 0; i < words; i++)
          {
            SETREG(sc, TXBUFF_OFFSET+(i<<3), alloc[i]);
          }

	SETREG(sc, TPLR_OFFSET, m->m_pkthdr.len);

	LOWRISC_ETHER_UNLOCK(sc);

	ETHER_BPF_MTAP(ifp, m);

	if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);
	if_inc_counter(ifp, IFCOUNTER_OBYTES, m->m_pkthdr.len);

	m_freem(m);

	return (0);
}

static int
lowrisc_medchange(struct ifnet *ifp)
{
	return (ENOTSUP);
}

static void
lowrisc_medstat(struct ifnet *ifp, struct ifmediareq *ifm)
{
/*
	struct lowrisc_softc *sc;

	sc = ifp->if_softc;
*/

	/* TODO: attach and interrogate PHY status */
	ifm->ifm_status = IFM_AVALID | IFM_ACTIVE;
	ifm->ifm_active = IFT_ETHER | IFM_100_T | IFM_FDX;
}

static int
lowrisc_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct lowrisc_softc *sc;
	struct ifreq *ifr;
#ifdef INET
	struct ifaddr *ifa;
#endif
	int error;

	sc = ifp->if_softc;
	ifr = (struct ifreq *)data;
#ifdef INET
	ifa = (struct ifaddr *)data;
#endif

	switch (cmd) {
	case SIOCSIFADDR:
#ifdef INET
		/*
		 * Avoid reinitialization unless it's necessary.
		 */
		if (ifa->ifa_addr->sa_family == AF_INET) {
			ifp->if_flags |= IFF_UP;
			if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
				lowrisc_init(sc);
			arp_ifinit(ifp, ifa);

			return (0);
		}
#endif
		error = ether_ioctl(ifp, cmd, data);
		if (error != 0)
			return (error);
		return (0);

	case SIOCSIFFLAGS:
		if (ifp->if_flags == sc->sc_flags)
			return (0);
		if ((ifp->if_flags & IFF_UP) != 0) {
			lowrisc_init(sc);
		} else {
			if ((ifp->if_drv_flags & IFF_DRV_RUNNING) != 0) {
				ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
			}
		}
		sc->sc_flags = ifp->if_flags;
		if (sc->sc_flags & IFF_PROMISC)
			SETREG(sc, MACHI_OFFSET, MACHI_ALLPKTS_MASK | GETREG(sc, MACHI_OFFSET));
		else
			SETREG(sc, MACHI_OFFSET, (~MACHI_ALLPKTS_MASK) & GETREG(sc, MACHI_OFFSET));
		return (0);

	case SIOCGIFFLAGS:
		if (MACHI_ALLPKTS_MASK & GETREG(sc, MACHI_OFFSET))
			sc->sc_flags |= IFF_PROMISC;
		else
			sc->sc_flags &= ~IFF_PROMISC;
		ifp->if_flags = sc->sc_flags;
		return (0);

	case SIOCSIFMTU:
		if (ifr->ifr_mtu + ifp->if_hdrlen > 1536)
			return (ENOTSUP);
		return (0);

	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		error = ifmedia_ioctl(ifp, ifr, &sc->sc_ifmedia, cmd);
		if (error != 0)
			return (error);
		return (0);
	
	default:
		error = ether_ioctl(ifp, cmd, data);
		if (error != 0)
			return (error);
		return (0);
	}
}

static void
lowrisc_rx_intr(void *arg)
{
	uint32_t status;
	struct lowrisc_softc *sc = arg;

	LOWRISC_ETHER_LOCK(sc);
	status = GETREG(sc, RSR_OFFSET);
	if (0) device_printf(sc->sc_dev, "Receive interrupt status %x\n", status);
	while (status & RSR_RECV_DONE_MASK) {
		uint32_t length, buf, errs, start, rnd;
		struct mbuf *m;
		uint64_t *alloc;
		int i;

		/*
		 * XXX
		 * Limit number of packets received at once?
		 */
		buf = status & RSR_RECV_FIRST_MASK;
		errs = GETREG(sc, RBAD_OFFSET);
		length = GETREG(sc, RPLR_OFFSET+((buf&7)<<3)) & RPLR_LENGTH_MASK;

		if (0) device_printf(sc->sc_dev, "Receive interrupt loop %d, %d, %d\n", buf, errs, length);

		if ((length > MCLBYTES - ETHER_ALIGN) || ((0x101<<(buf&7)) & errs))
		{
			device_printf(sc->sc_dev, "Receive discarded\n");
			if_inc_counter(sc->sc_ifp, IFCOUNTER_IERRORS, 1);
			SETREG(sc, RSR_OFFSET, buf+1);
			status = GETREG(sc, RSR_OFFSET);
			continue;
		}

		m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
		if (m == NULL) {
			device_printf(sc->sc_dev, "no memory for receive mbuf.\n");
			if_inc_counter(sc->sc_ifp, IFCOUNTER_IQDROPS, 1);
			LOWRISC_ETHER_UNLOCK(sc);
			return;
		}

		/* Align incoming frame so IP headers are aligned.  */
		m->m_data += ETHER_ALIGN;

		start = RXBUFF_OFFSET + ((buf&7)<<11);

		rnd = ((length-1)|7)+1; /* round to a multiple of 8 */
		alloc = (uint64_t *)(m->m_data);
                for (i = 0; i < rnd/8; i++)
                  {
                     alloc[i] = GETREG(sc, start+(i << 3));
                  }

		SETREG(sc, RSR_OFFSET, buf+1);

		m->m_pkthdr.rcvif = sc->sc_ifp;
		m->m_pkthdr.len = m->m_len = length;

		if_inc_counter(sc->sc_ifp, IFCOUNTER_IPACKETS, 1);

		LOWRISC_ETHER_UNLOCK(sc);

		(*sc->sc_ifp->if_input)(sc->sc_ifp, m);

		LOWRISC_ETHER_LOCK(sc);
		status = GETREG(sc, RSR_OFFSET);
	}

	LOWRISC_ETHER_UNLOCK(sc);

}
