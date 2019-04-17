/*-
 * Copyright (c) 2019 Ruslan Bukin <br@bsdpad.com>
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_var.h>

#include <machine/bus.h>

#include <dev/xilinx/if_xaereg.h>
#include <dev/xilinx/if_xaevar.h>
#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include <dev/mii/tiphy.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#ifdef EXT_RESOURCES
#include <dev/extres/clk/clk.h>
#include <dev/extres/hwreset/hwreset.h>
#endif

#include "miibus_if.h"

#define	READ4(_sc, _reg) \
	bus_read_4((_sc)->res[0], _reg)
#define	WRITE4(_sc, _reg, _val) \
	bus_write_4((_sc)->res[0], _reg, _val)

#define	STATS_HARVEST_INTERVAL	2

#define	MDIO_CLK_DIV_DEFAULT	29

#define	XAE_LOCK(sc)			mtx_lock(&(sc)->mtx)
#define	XAE_UNLOCK(sc)			mtx_unlock(&(sc)->mtx)
#define	XAE_ASSERT_LOCKED(sc)		mtx_assert(&(sc)->mtx, MA_OWNED)
#define	XAE_ASSERT_UNLOCKED(sc)		mtx_assert(&(sc)->mtx, MA_NOTOWNED)

#define XAE_DEBUG
#undef XAE_DEBUG

#ifdef XAE_DEBUG
#define dprintf(fmt, ...)  printf(fmt, ##__VA_ARGS__)
#else
#define dprintf(fmt, ...)
#endif

#define	RX_QUEUE_SIZE		64
#define	TX_QUEUE_SIZE		64
#define	NUM_RX_MBUF		16
#define	BUFRING_SIZE		8192

#define	PHY1_RD(sc, _r)		\
	xae_miibus_read_reg(sc->dev, 1, _r)
#define	PHY1_WR(sc, _r, _v)	\
	xae_miibus_write_reg(sc->dev, 1, _r, _v)

#define	PHY_RD(sc, _r)		\
	xae_miibus_read_reg(sc->dev, sc->phy_addr, _r)
#define	PHY_WR(sc, _r, _v)	\
	xae_miibus_write_reg(sc->dev, sc->phy_addr, _r, _v)

/* Use this macro to access regs > 0x1f */
#define WRITE_TI_EREG(sc, reg, data) {			\
	PHY_WR(sc, MII_MMDACR, MMDACR_DADDRMASK);	\
	PHY_WR(sc, MII_MMDAADR, reg);		\
	PHY_WR(sc, MII_MMDACR, MMDACR_DADDRMASK | MMDACR_FN_DATANPI);\
	PHY_WR(sc, MII_MMDAADR, data);		\
}

/* Not documented, Xilinx VCU118 workaround */
#define	 CFG4_SGMII_TMR			0x160 /* bits 8:7 MUST be '10' */
#define	DP83867_SGMIICTL1		0xD3 /* not documented register */
#define	 SGMIICTL1_SGMII_6W		(1 << 14) /* no idea what it is */

static struct resource_spec xae_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, 0 }
};

static void xae_stop_locked(struct xae_softc *sc);
static void xae_setup_rxfilter(struct xae_softc *sc);

static int
xae_rx_enqueue(struct xae_softc *sc, uint32_t n)
{
	struct mbuf *m;
	int i;

	for (i = 0; i < n; i++) {
		m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
		if (m == NULL) {
			device_printf(sc->dev,
			    "%s: Can't alloc rx mbuf\n", __func__);
			return (-1);
		}

		m->m_pkthdr.len = m->m_len = m->m_ext.ext_size;
		xdma_enqueue_mbuf(sc->xchan_rx, &m, 0, 4, 4, XDMA_DEV_TO_MEM);
	}

	return (0);
}

static int
xae_get_phyaddr(phandle_t node, int *phy_addr)
{
	phandle_t phy_node;
	pcell_t phy_handle, phy_reg;

	if (OF_getencprop(node, "phy-handle", (void *)&phy_handle,
	    sizeof(phy_handle)) <= 0)
		return (ENXIO);

	phy_node = OF_node_from_xref(phy_handle);

	if (OF_getencprop(phy_node, "reg", (void *)&phy_reg,
	    sizeof(phy_reg)) <= 0)
		return (ENXIO);

	*phy_addr = phy_reg;

	return (0);
}

static int
xae_xdma_tx_intr(void *arg, xdma_transfer_status_t *status)
{
	xdma_transfer_status_t st;
	struct xae_softc *sc;
	struct ifnet *ifp;
	struct mbuf *m;
	int err;

	sc = arg;

	XAE_LOCK(sc);

	ifp = sc->ifp;

	for (;;) {
		err = xdma_dequeue_mbuf(sc->xchan_tx, &m, &st);
		if (err != 0) {
			break;
		}

		if (st.error != 0) {
			if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
		}

		m_freem(m);
	}

	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	XAE_UNLOCK(sc);

	return (0);
}

static int
xae_xdma_rx_intr(void *arg, xdma_transfer_status_t *status)
{
	xdma_transfer_status_t st;
	struct xae_softc *sc;
	struct ifnet *ifp;
	struct mbuf *m;
	int err;
	uint32_t cnt_processed;

	sc = arg;

	dprintf("%s\n", __func__);

	XAE_LOCK(sc);

	ifp = sc->ifp;

	cnt_processed = 0;
	for (;;) {
		err = xdma_dequeue_mbuf(sc->xchan_rx, &m, &st);
		if (err != 0) {
			break;
		}
		cnt_processed++;

		if (st.error != 0) {
			if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
			m_freem(m);
			continue;
		}

		m->m_pkthdr.len = m->m_len = st.transferred;
		m->m_pkthdr.rcvif = ifp;
		XAE_UNLOCK(sc);
		(*ifp->if_input)(ifp, m);
		XAE_LOCK(sc);
	}

	xae_rx_enqueue(sc, cnt_processed);

	XAE_UNLOCK(sc);

	return (0);
}

static void
xae_qflush(struct ifnet *ifp)
{
	struct xae_softc *sc;

	sc = ifp->if_softc;

	printf("%s\n", __func__);
}

static int
xae_transmit_locked(struct ifnet *ifp)
{
	struct xae_softc *sc;
	struct mbuf *m;
	struct buf_ring *br;
	int error;
	int enq;

	dprintf("%s\n", __func__);

	sc = ifp->if_softc;
	br = sc->br;

	enq = 0;

	while ((m = drbr_peek(ifp, br)) != NULL) {
		error = xdma_enqueue_mbuf(sc->xchan_tx,
		    &m, 0, 4, 4, XDMA_MEM_TO_DEV);
		if (error != 0) {
			/* No space in request queue available yet. */
			drbr_putback(ifp, br, m);
			break;
		}

		drbr_advance(ifp, br);

		enq++;

		/* If anyone is interested give them a copy. */
		ETHER_BPF_MTAP(ifp, m);
        }

	if (enq > 0)
		xdma_queue_submit(sc->xchan_tx);

	return (0);
}

static int
xae_transmit(struct ifnet *ifp, struct mbuf *m)
{
	struct xae_softc *sc;
	struct buf_ring *br;
	int error;

	dprintf("%s\n", __func__);

	sc = ifp->if_softc;
	br = sc->br;

	XAE_LOCK(sc);

	mtx_lock(&sc->br_mtx);

	if ((ifp->if_drv_flags & (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) !=
	    IFF_DRV_RUNNING) {
		error = drbr_enqueue(ifp, sc->br, m);
		mtx_unlock(&sc->br_mtx);
		XAE_UNLOCK(sc);
		return (error);
	}

	//if ((sc->atse_flags & DWC_FLAGS_LINK) == 0) {
	if (!sc->link_is_up) {
		error = drbr_enqueue(ifp, sc->br, m);
		mtx_unlock(&sc->br_mtx);
		XAE_UNLOCK(sc);
		return (error);
	}

	error = drbr_enqueue(ifp, br, m);
	if (error) {
		mtx_unlock(&sc->br_mtx);
		XAE_UNLOCK(sc);
		return (error);
	}
	error = xae_transmit_locked(ifp);

	mtx_unlock(&sc->br_mtx);
	XAE_UNLOCK(sc);

	return (error);
}

static void
xae_stop_locked(struct xae_softc *sc)
{
	struct ifnet *ifp;

	XAE_ASSERT_LOCKED(sc);

	ifp = sc->ifp;
	ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);
	sc->tx_watchdog_count = 0;
	sc->stats_harvest_count = 0;

	callout_stop(&sc->xae_callout);

#if 0
	/* Stop DMA TX */
	reg = READ4(sc, OPERATION_MODE);
	reg &= ~(MODE_ST);
	WRITE4(sc, OPERATION_MODE, reg);

	/* Flush TX */
	reg = READ4(sc, OPERATION_MODE);
	reg |= (MODE_FTF);
	WRITE4(sc, OPERATION_MODE, reg);

	/* Stop transmitters */
	reg = READ4(sc, MAC_CONFIGURATION);
	reg &= ~(CONF_TE | CONF_RE);
	WRITE4(sc, MAC_CONFIGURATION, reg);

	/* Stop DMA RX */
	reg = READ4(sc, OPERATION_MODE);
	reg &= ~(MODE_SR);
	WRITE4(sc, OPERATION_MODE, reg);
#endif
}

static void xae_clear_stats(struct xae_softc *sc)
{
#if 0
	uint32_t reg;

	reg = READ4(sc, MMC_CONTROL);
	reg |= (MMC_CONTROL_CNTRST);
	WRITE4(sc, MMC_CONTROL, reg);
#endif
}

static void
xae_harvest_stats(struct xae_softc *sc)
{
	struct ifnet *ifp;

	/* We don't need to harvest too often. */
	if (++sc->stats_harvest_count < STATS_HARVEST_INTERVAL)
		return;

	sc->stats_harvest_count = 0;
	ifp = sc->ifp;

#if 0
	if_inc_counter(ifp, IFCOUNTER_IPACKETS, READ4(sc, RXFRAMECOUNT_GB));
	if_inc_counter(ifp, IFCOUNTER_IMCASTS, READ4(sc, RXMULTICASTFRAMES_G));
	if_inc_counter(ifp, IFCOUNTER_IERRORS,
	    READ4(sc, RXOVERSIZE_G) + READ4(sc, RXUNDERSIZE_G) +
	    READ4(sc, RXCRCERROR) + READ4(sc, RXALIGNMENTERROR) +
	    READ4(sc, RXRUNTERROR) + READ4(sc, RXJABBERERROR) +
	    READ4(sc, RXLENGTHERROR));

	if_inc_counter(ifp, IFCOUNTER_OPACKETS, READ4(sc, TXFRAMECOUNT_G));
	if_inc_counter(ifp, IFCOUNTER_OMCASTS, READ4(sc, TXMULTICASTFRAMES_G));
	if_inc_counter(ifp, IFCOUNTER_OERRORS,
	    READ4(sc, TXOVERSIZE_G) + READ4(sc, TXEXCESSDEF) +
	    READ4(sc, TXCARRIERERR) + READ4(sc, TXUNDERFLOWERROR));

	if_inc_counter(ifp, IFCOUNTER_COLLISIONS,
	    READ4(sc, TXEXESSCOL) + READ4(sc, TXLATECOL));
#endif

	xae_clear_stats(sc);
}

static void
xae_tick(void *arg)
{
	struct xae_softc *sc;
	struct ifnet *ifp;
	int link_was_up;

	sc = arg;

	XAE_ASSERT_LOCKED(sc);

	ifp = sc->ifp;

	if (!(ifp->if_drv_flags & IFF_DRV_RUNNING))
		return;

	/*
	 * Typical tx watchdog.  If this fires it indicates that we enqueued
	 * packets for output and never got a txdone interrupt for them.  Maybe
	 * it's a missed interrupt somehow, just pretend we got one.
	 */
#if 0
	if (sc->tx_watchdog_count > 0) {
		if (--sc->tx_watchdog_count == 0) {
			xae_txfinish_locked(sc);
		}
	}
#endif

	/* Gather stats from hardware counters. */
	xae_harvest_stats(sc);

	/* Check the media status. */
	link_was_up = sc->link_is_up;
	mii_tick(sc->mii_softc);
	if (sc->link_is_up && !link_was_up)
		xae_transmit_locked(sc->ifp);

	/* Schedule another check one second from now. */
	callout_reset(&sc->xae_callout, hz, xae_tick, sc);
}

static void
xae_init_locked(struct xae_softc *sc)
{
	struct ifnet *ifp;

	XAE_ASSERT_LOCKED(sc);

	printf("%s\n", __func__);

	ifp = sc->ifp;
	if (ifp->if_drv_flags & IFF_DRV_RUNNING)
		return;

	ifp->if_drv_flags |= IFF_DRV_RUNNING;

#if 0
	xae_setup_rxfilter(sc);

	/* Initializa DMA and enable transmitters */
	reg = READ4(sc, OPERATION_MODE);
	reg |= (MODE_TSF | MODE_OSF | MODE_FUF);
	reg &= ~(MODE_RSF);
	reg |= (MODE_RTC_LEV32 << MODE_RTC_SHIFT);
	WRITE4(sc, OPERATION_MODE, reg);

	WRITE4(sc, INTERRUPT_ENABLE, INT_EN_DEFAULT);

	/* Start DMA */
	reg = READ4(sc, OPERATION_MODE);
	reg |= (MODE_ST | MODE_SR);
	WRITE4(sc, OPERATION_MODE, reg);

	/* Enable transmitters */
	reg = READ4(sc, MAC_CONFIGURATION);
	reg |= (CONF_JD | CONF_ACS | CONF_BE);
	reg |= (CONF_TE | CONF_RE);
	WRITE4(sc, MAC_CONFIGURATION, reg);
#endif

	/* Enable the transmitter */
	WRITE4(sc, XAE_TC, TC_TX);

	/* Enable the receiver. */
	WRITE4(sc, XAE_RCW1, RCW1_RX);

	/*
	 * Call mii_mediachg() which will call back into xae_miibus_statchg()
	 * to set up the remaining config registers based on current media.
	 */
	mii_mediachg(sc->mii_softc);
	callout_reset(&sc->xae_callout, hz, xae_tick, sc);
}

static void
xae_init(void *arg)
{
	struct xae_softc *sc;

	sc = arg;

	XAE_LOCK(sc);
	xae_init_locked(sc);
	XAE_UNLOCK(sc);
}

static void
xae_media_status(struct ifnet * ifp, struct ifmediareq *ifmr)
{
	struct xae_softc *sc;
	struct mii_data *mii;

	sc = ifp->if_softc;
	mii = sc->mii_softc;
	XAE_LOCK(sc);
	mii_pollstat(mii);
	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;
	XAE_UNLOCK(sc);
}

static int
xae_media_change_locked(struct xae_softc *sc)
{

	printf("%s\n", __func__);

	return (mii_mediachg(sc->mii_softc));
}

static int
xae_media_change(struct ifnet * ifp)
{
	struct xae_softc *sc;
	int error;

	sc = ifp->if_softc;

	XAE_LOCK(sc);
	error = xae_media_change_locked(sc);
	XAE_UNLOCK(sc);
	return (error);
}

static const uint8_t nibbletab[] = {
	/* 0x0 0000 -> 0000 */  0x0,
	/* 0x1 0001 -> 1000 */  0x8,
	/* 0x2 0010 -> 0100 */  0x4,
	/* 0x3 0011 -> 1100 */  0xc,
	/* 0x4 0100 -> 0010 */  0x2,
	/* 0x5 0101 -> 1010 */  0xa,
	/* 0x6 0110 -> 0110 */  0x6,
	/* 0x7 0111 -> 1110 */  0xe,
	/* 0x8 1000 -> 0001 */  0x1,
	/* 0x9 1001 -> 1001 */  0x9,
	/* 0xa 1010 -> 0101 */  0x5,
	/* 0xb 1011 -> 1101 */  0xd,
	/* 0xc 1100 -> 0011 */  0x3,
	/* 0xd 1101 -> 1011 */  0xb,
	/* 0xe 1110 -> 0111 */  0x7,
	/* 0xf 1111 -> 1111 */  0xf, };

static uint8_t
bitreverse(uint8_t x)
{

	return (nibbletab[x & 0xf] << 4) | nibbletab[x >> 4];
}

static void
xae_setup_rxfilter(struct xae_softc *sc)
{
	struct ifmultiaddr *ifma;
	struct ifnet *ifp;
	uint8_t *eaddr, val;
	//uint32_t crc, ffval, hashbit, hashreg, hi, lo, hash[8];
	uint32_t crc, hashbit, hashreg, hi, lo, hash[8];
	int nhash, i;

	XAE_ASSERT_LOCKED(sc);

	ifp = sc->ifp;
	nhash = sc->mactype == 0;//DWC_GMAC_ALT_DESC ? 2 : 8;

	/*
	 * Set the multicast (group) filter hash.
	 */
	if ((ifp->if_flags & IFF_ALLMULTI) != 0) {
#if 0
		ffval = (FRAME_FILTER_PM);
#endif
		for (i = 0; i < nhash; i++)
			hash[i] = ~0;
	} else {
#if 0
		ffval = (FRAME_FILTER_HMC);
#endif
		for (i = 0; i < nhash; i++)
			hash[i] = 0;
		if_maddr_rlock(ifp);
		CK_STAILQ_FOREACH(ifma, &sc->ifp->if_multiaddrs, ifma_link) {
			if (ifma->ifma_addr->sa_family != AF_LINK)
				continue;
			crc = ether_crc32_le(LLADDR((struct sockaddr_dl *)
				ifma->ifma_addr), ETHER_ADDR_LEN);

			/* Take lower 8 bits and reverse it */
			val = bitreverse(~crc & 0xff);
#if 0
			if (sc->mactype == DWC_GMAC_ALT_DESC)
				val >>= nhash; /* Only need lower 6 bits */
#endif
			hashreg = (val >> 5);
			hashbit = (val & 31);
			hash[hashreg] |= (1 << hashbit);
		}
		if_maddr_runlock(ifp);
	}

	/*
	 * Set the individual address filter hash.
	 */
#if 0
	if (ifp->if_flags & IFF_PROMISC)
		ffval |= (FRAME_FILTER_PR);
#endif

	/*
	 * Set the primary address.
	 */
	eaddr = IF_LLADDR(ifp);
	lo = eaddr[0] | (eaddr[1] << 8) | (eaddr[2] << 16) |
	    (eaddr[3] << 24);
	hi = eaddr[4] | (eaddr[5] << 8);
#if 0
	WRITE4(sc, MAC_ADDRESS_LOW(0), lo);
	WRITE4(sc, MAC_ADDRESS_HIGH(0), hi);
	WRITE4(sc, MAC_FRAME_FILTER, ffval);
	if (sc->mactype == DWC_GMAC_ALT_DESC) {
		WRITE4(sc, GMAC_MAC_HTLOW, hash[0]);
		WRITE4(sc, GMAC_MAC_HTHIGH, hash[1]);
	} else {
		for (i = 0; i < nhash; i++)
			WRITE4(sc, HASH_TABLE_REG(i), hash[i]);
	}
#endif
}

static int
xae_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct xae_softc *sc;
	struct mii_data *mii;
	struct ifreq *ifr;
	int mask, error;

	sc = ifp->if_softc;
	ifr = (struct ifreq *)data;

	error = 0;
	switch (cmd) {
	case SIOCSIFFLAGS:
		XAE_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
				if ((ifp->if_flags ^ sc->if_flags) &
				    (IFF_PROMISC | IFF_ALLMULTI))
					xae_setup_rxfilter(sc);
			} else {
				if (!sc->is_detaching)
					xae_init_locked(sc);
			}
		} else {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING)
				xae_stop_locked(sc);
		}
		sc->if_flags = ifp->if_flags;
		XAE_UNLOCK(sc);
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
			XAE_LOCK(sc);
			xae_setup_rxfilter(sc);
			XAE_UNLOCK(sc);
		}
		break;
	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		mii = sc->mii_softc;
		error = ifmedia_ioctl(ifp, ifr, &mii->mii_media, cmd);
		break;
	case SIOCSIFCAP:
		mask = ifp->if_capenable ^ ifr->ifr_reqcap;
		if (mask & IFCAP_VLAN_MTU) {
			/* No work to do except acknowledge the change took */
			ifp->if_capenable ^= IFCAP_VLAN_MTU;
		}
		break;

	default:
		error = ether_ioctl(ifp, cmd, data);
		break;
	}

	return (error);
}

static void
xae_intr(void *arg)
{

	printf("%s\n", __func__);

#if 0
	struct xae_softc *sc;
	uint32_t reg;

	sc = arg;

	XAE_LOCK(sc);

	reg = READ4(sc, INTERRUPT_STATUS);
	if (reg)
		READ4(sc, SGMII_RGMII_SMII_CTRL_STATUS);

	reg = READ4(sc, DMA_STATUS);
	if (reg & DMA_STATUS_NIS) {
		if (reg & DMA_STATUS_RI)
			xae_rxfinish_locked(sc);

		if (reg & DMA_STATUS_TI) {
			xae_txfinish_locked(sc);
			xae_txstart_locked(sc);
		}
	}

	if (reg & DMA_STATUS_AIS) {
		if (reg & DMA_STATUS_FBI) {
			/* Fatal bus error */
			device_printf(sc->dev,
			    "Ethernet DMA error, restarting controller.\n");
			xae_stop_locked(sc);
			xae_init_locked(sc);
		}
	}

	WRITE4(sc, DMA_STATUS, reg & DMA_STATUS_INTR_MASK);
	XAE_UNLOCK(sc);
#endif
}

static int __unused
xae_get_hwaddr(struct xae_softc *sc, uint8_t *hwaddr)
{
	uint32_t hi, lo, rnd;

	/*
	 * Try to recover a MAC address from the running hardware. If there's
	 * something non-zero there, assume the bootloader did the right thing
	 * and just use it.
	 *
	 * Otherwise, set the address to a convenient locally assigned address,
	 * 'bsd' + random 24 low-order bits.  'b' is 0x62, which has the locally
	 * assigned bit set, and the broadcast/multicast bit clear.
	 */
#if 0
	lo = READ4(sc, MAC_ADDRESS_LOW(0));
	hi = READ4(sc, MAC_ADDRESS_HIGH(0)) & 0xffff;
#else
	lo = 0;
	hi = 0;
#endif
	if ((lo != 0xffffffff) || (hi != 0xffff)) {
		hwaddr[0] = (lo >>  0) & 0xff;
		hwaddr[1] = (lo >>  8) & 0xff;
		hwaddr[2] = (lo >> 16) & 0xff;
		hwaddr[3] = (lo >> 24) & 0xff;
		hwaddr[4] = (hi >>  0) & 0xff;
		hwaddr[5] = (hi >>  8) & 0xff;
	} else {
		rnd = arc4random() & 0x00ffffff;
		hwaddr[0] = 'b';
		hwaddr[1] = 's';
		hwaddr[2] = 'd';
		hwaddr[3] = rnd >> 16;
		hwaddr[4] = rnd >>  8;
		hwaddr[5] = rnd >>  0;
	}

	return (0);
}

static int __unused
xae_reset(device_t dev)
{

	return (0);
}

#ifdef EXT_RESOURCES
static int
xae_clock_init(device_t dev)
{
	hwreset_t rst;
	clk_t clk;
	int error;

	/* Enable clock */
	if (clk_get_by_ofw_name(dev, 0, "stmmaceth", &clk) == 0) {
		error = clk_enable(clk);
		if (error != 0) {
			device_printf(dev, "could not enable main clock\n");
			return (error);
		}
	}

	/* De-assert reset */
	if (hwreset_get_by_ofw_name(dev, 0, "stmmaceth", &rst) == 0) {
		error = hwreset_deassert(rst);
		if (error != 0) {
			device_printf(dev, "could not de-assert reset\n");
			return (error);
		}
	}

	return (0);
}
#endif

static int
mdio_wait(struct xae_softc *sc)
{
	uint32_t reg;
	int timeout;

	timeout = 200;

	do {
		reg = READ4(sc, XAE_MDIO_CTRL);
		if (reg & MDIO_CTRL_READY)
			break;
		DELAY(1);
	} while (timeout--);

	if (timeout <= 0) {
		printf("Failed to get MDIO ready\n");
		return (1);
	}

	return (0);
}

static int
xae_miibus_read_reg(device_t dev, int phy, int reg)
{
	struct xae_softc *sc;
	uint32_t mii;
	int rv;

	sc = device_get_softc(dev);

	if (mdio_wait(sc))
		return (0);

	mii = MDIO_CTRL_TX_OP_READ | MDIO_CTRL_INITIATE;
	mii |= (reg << MDIO_TX_REGAD_S);
	mii |= (phy << MDIO_TX_PHYAD_S);

	WRITE4(sc, XAE_MDIO_CTRL, mii);

	if (mdio_wait(sc))
		return (0);

	rv = READ4(sc, XAE_MDIO_READ);

	return (rv);
}

static int
xae_miibus_write_reg(device_t dev, int phy, int reg, int val)
{
	struct xae_softc *sc;
	uint32_t mii;

	sc = device_get_softc(dev);

	if (mdio_wait(sc))
		return (1);

	mii = MDIO_CTRL_TX_OP_WRITE | MDIO_CTRL_INITIATE;
	mii |= (reg << MDIO_TX_REGAD_S);
	mii |= (phy << MDIO_TX_PHYAD_S);

	WRITE4(sc, XAE_MDIO_WRITE, val);
	WRITE4(sc, XAE_MDIO_CTRL, mii);

	if (mdio_wait(sc))
		return (1);

	return (0);
}

static void
fixup(struct xae_softc *sc)
{
	uint32_t reg;
	device_t dev;

	dev = sc->dev;

	do {
		WRITE_TI_EREG(sc, DP83867_SGMIICTL1, SGMIICTL1_SGMII_6W);
		PHY_WR(sc, DP83867_PHYCR, PHYCR_SGMII_EN);

		reg = PHY_RD(sc, DP83867_CFG2);
		reg &= ~CFG2_SPEED_OPT_ATTEMPT_CNT_M;
		reg |= (CFG2_SPEED_OPT_ATTEMPT_CNT_4);
		reg |= CFG2_INTERRUPT_POLARITY;
		reg |= CFG2_SPEED_OPT_ENHANCED_EN;
		reg |= CFG2_SPEED_OPT_10M_EN;
		PHY_WR(sc, DP83867_CFG2, reg);

		WRITE_TI_EREG(sc, DP83867_CFG4, CFG4_SGMII_TMR);
		PHY_WR(sc, MII_BMCR,
		    BMCR_AUTOEN | BMCR_FDX | BMCR_SPEED1 | BMCR_RESET);
	} while (PHY1_RD(sc, MII_BMCR) == 0x0ffff);

	do {
		PHY1_WR(sc, MII_BMCR,
		    BMCR_AUTOEN | BMCR_FDX | BMCR_SPEED1 | BMCR_STARTNEG);
		DELAY(40000);
	} while ((PHY1_RD(sc, MII_BMSR) & BMSR_ACOMP) == 0);
}

static int
xae_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "xlnx,axi-ethernet-1.00.a"))
		return (ENXIO);

	device_set_desc(dev, "Xilinx AXI Ethernet");

	return (BUS_PROBE_DEFAULT);
}

static int
xae_attach(device_t dev)
{
	uint8_t macaddr[ETHER_ADDR_LEN];
	struct xae_softc *sc;
	struct ifnet *ifp;
	phandle_t node;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	vmem_t *vmem;
	vmem = xdma_get_memory(dev);

	/* Get xDMA controller */   
	sc->xdma_tx = xdma_ofw_get(sc->dev, "tx");
	if (sc->xdma_tx == NULL) {
		device_printf(dev, "Could not find DMA controller.\n");
		return (ENXIO);
	}

	sc->xdma_rx = xdma_ofw_get(sc->dev, "rx");
	if (sc->xdma_rx == NULL) {
		device_printf(dev, "Could not find DMA controller.\n");
		return (ENXIO);
	}

	int caps;
	caps = 0;
	/* Alloc xDMA TX virtual channel. */
	sc->xchan_tx = xdma_channel_alloc(sc->xdma_tx, caps);
	if (sc->xchan_tx == NULL) {
		device_printf(dev, "Can't alloc virtual DMA TX channel.\n");
		return (ENXIO);
	}
	sc->xchan_tx->vmem = vmem;

	/* Setup interrupt handler. */
	error = xdma_setup_intr(sc->xchan_tx,
	    xae_xdma_tx_intr, sc, &sc->ih_tx);
	if (error) {
		device_printf(sc->dev,
		    "Can't setup xDMA TX interrupt handler.\n");
		return (ENXIO);
	}

	/* Alloc xDMA RX virtual channel. */
	sc->xchan_rx = xdma_channel_alloc(sc->xdma_rx, caps);
	if (sc->xchan_rx == NULL) {
		device_printf(dev, "Can't alloc virtual DMA RX channel.\n");
		return (ENXIO);
	}
	sc->xchan_rx->vmem = vmem;

	/* Setup interrupt handler. */
	error = xdma_setup_intr(sc->xchan_rx,
	    xae_xdma_rx_intr, sc, &sc->ih_rx);
	if (error) {
		device_printf(sc->dev,
		    "Can't setup xDMA RX interrupt handler.\n");
		return (ENXIO);
	}

	xdma_prep_sg(sc->xchan_tx,
	    TX_QUEUE_SIZE,	/* xchan requests queue size */
	    MCLBYTES,	/* maxsegsize */
	    8,		/* maxnsegs */
	    16,		/* alignment */
	    0,		/* boundary */
	    BUS_SPACE_MAXADDR_32BIT,
	    BUS_SPACE_MAXADDR);

	xdma_prep_sg(sc->xchan_rx,
	    RX_QUEUE_SIZE,	/* xchan requests queue size */
	    MCLBYTES,	/* maxsegsize */
	    1,		/* maxnsegs */
	    16,		/* alignment */
	    0,		/* boundary */
	    BUS_SPACE_MAXADDR_32BIT,
	    BUS_SPACE_MAXADDR);

	mtx_init(&sc->br_mtx, "buf ring mtx", NULL, MTX_DEF);
	sc->br = buf_ring_alloc(BUFRING_SIZE, M_DEVBUF,
	    M_NOWAIT, &sc->br_mtx);
	if (sc->br == NULL)
		return (ENOMEM);

#ifdef EXT_RESOURCES
	if (xae_clock_init(dev) != 0)
		return (ENXIO);
#endif

	if (bus_alloc_resources(dev, xae_spec, sc->res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	/* Memory interface */
	sc->bst = rman_get_bustag(sc->res[0]);
	sc->bsh = rman_get_bushandle(sc->res[0]);

	printf("ID: %x\n", READ4(sc, XAE_IDENT));

#if 0
	/* Read MAC before reset */
	if (xae_get_hwaddr(sc, macaddr)) {
		device_printf(sc->dev, "can't get mac\n");
		return (ENXIO);
	}

	/* Reset the PHY if needed */
	if (xae_reset(dev) != 0) {
		device_printf(dev, "Can't reset the PHY\n");
		return (ENXIO);
	}

	/* Reset */
	reg = READ4(sc, BUS_MODE);
	reg |= (BUS_MODE_SWR);
	WRITE4(sc, BUS_MODE, reg);

	for (i = 0; i < MAC_RESET_TIMEOUT; i++) {
		if ((READ4(sc, BUS_MODE) & BUS_MODE_SWR) == 0)
			break;
		DELAY(10);
	}
	if (i >= MAC_RESET_TIMEOUT) {
		device_printf(sc->dev, "Can't reset DWC.\n");
		return (ENXIO);
	}

	if (sc->mactype == DWC_GMAC_ALT_DESC) {
		reg = BUS_MODE_FIXEDBURST;
		reg |= (BUS_MODE_PRIORXTX_41 << BUS_MODE_PRIORXTX_SHIFT);
	} else
		reg = (BUS_MODE_EIGHTXPBL);
	reg |= (BUS_MODE_PBL_BEATS_8 << BUS_MODE_PBL_SHIFT);
	WRITE4(sc, BUS_MODE, reg);

	/*
	 * DMA must be stop while changing descriptor list addresses.
	 */
	reg = READ4(sc, OPERATION_MODE);
	reg &= ~(MODE_ST | MODE_SR);
	WRITE4(sc, OPERATION_MODE, reg);

	if (setup_dma(sc))
	        return (ENXIO);

	/* Setup addresses */
	WRITE4(sc, RX_DESCR_LIST_ADDR, sc->rxdesc_ring_paddr);
	WRITE4(sc, TX_DESCR_LIST_ADDR, sc->txdesc_ring_paddr);
#endif

	mtx_init(&sc->mtx, device_get_nameunit(sc->dev),
	    MTX_NETWORK_LOCK, MTX_DEF);

	callout_init_mtx(&sc->xae_callout, &sc->mtx, 0);

	/* Setup interrupt handler. */
	error = bus_setup_intr(dev, sc->res[1], INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, xae_intr, sc, &sc->intr_cookie);
	if (error != 0) {
		device_printf(dev, "could not setup interrupt handler.\n");
		return (ENXIO);
	}

	/* Set up the ethernet interface. */
	sc->ifp = ifp = if_alloc(IFT_ETHER);
	/* if ifp == NULL */

	ifp->if_softc = sc;
	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_capabilities = IFCAP_VLAN_MTU;
	ifp->if_capenable = ifp->if_capabilities;
	ifp->if_transmit = xae_transmit;
	ifp->if_qflush = xae_qflush;
	ifp->if_ioctl = xae_ioctl;
	ifp->if_init = xae_init;
	IFQ_SET_MAXLEN(&ifp->if_snd, TX_DESC_COUNT - 1);
	ifp->if_snd.ifq_drv_maxlen = TX_DESC_COUNT - 1;
	IFQ_SET_READY(&ifp->if_snd);

	uint32_t reg;
	/* Enable MII clock */
	reg = (MDIO_CLK_DIV_DEFAULT << MDIO_SETUP_CLK_DIV_S);
	reg |= MDIO_SETUP_ENABLE;
	WRITE4(sc, XAE_MDIO_SETUP, reg);
	if (mdio_wait(sc))
		return (ENXIO);

	//xae_miibus_write_reg(dev, 0x1, 0x0, 0x1340);

	macaddr[0] = 0x00;
	macaddr[1] = 0x0a;
	macaddr[2] = 0x35;
	macaddr[3] = 0x04;
	macaddr[4] = 0xdb;
	macaddr[5] = 0x5a;

	reg = macaddr[0] | (macaddr[1] << 8);
	reg |= (macaddr[2] << 16) | (macaddr[3] << 24);
	WRITE4(sc, XAE_UAWL, reg);
	reg = macaddr[4] | (macaddr[5] << 8);
	WRITE4(sc, XAE_UAWU, reg);

	if (xae_get_phyaddr(node, &sc->phy_addr) != 0)
		return (ENXIO);

	/* Attach the mii driver. */
	error = mii_attach(dev, &sc->miibus, ifp, xae_media_change,
	    xae_media_status, BMSR_DEFCAPMASK, sc->phy_addr,
	    MII_OFFSET_ANY, 0);

	if (error != 0) {
		device_printf(dev, "PHY attach failed\n");
		return (ENXIO);
	}
	sc->mii_softc = device_get_softc(sc->miibus);

	fixup(sc);

	/* All ready to run, attach the ethernet interface. */
	ether_ifattach(ifp, macaddr);
	sc->is_attached = true;

	xae_rx_enqueue(sc, NUM_RX_MBUF);
	xdma_queue_submit(sc->xchan_rx);

	return (0);
}

static void
xae_miibus_statchg(device_t dev)
{
	struct xae_softc *sc;
	struct mii_data *mii;
	uint32_t reg;

	/*
	 * Called by the MII bus driver when the PHY establishes
	 * link to set the MAC interface registers.
	 */

	printf("%s\n", __func__);

	sc = device_get_softc(dev);

	XAE_ASSERT_LOCKED(sc);

	mii = sc->mii_softc;

	if (mii->mii_media_status & IFM_ACTIVE)
		sc->link_is_up = true;
	else
		sc->link_is_up = false;

#if 0
	printf("link_is_up %d\n", sc->link_is_up);
	printf("%s: IFM_SUBTYPE(mii->mii_media_active) %d\n",
	    __func__, IFM_SUBTYPE(mii->mii_media_active));
	printf("%s: options %x\n",
	    __func__, IFM_OPTIONS(mii->mii_media_active));
#endif

	switch (IFM_SUBTYPE(mii->mii_media_active)) {
	case IFM_1000_T:
	case IFM_1000_SX:
		reg = SPEED_1000;
		break;
	case IFM_100_TX:
		reg = SPEED_100;
		break;
	case IFM_10_T:
		reg = SPEED_10;
		break;
	case IFM_NONE:
		sc->link_is_up = false;
		return;
	default:
		sc->link_is_up = false;
		device_printf(dev, "Unsupported media %u\n",
		    IFM_SUBTYPE(mii->mii_media_active));
		return;
	}

	WRITE4(sc, XAE_SPEED, reg);
	DELAY(1);

#if 0
	if ((IFM_OPTIONS(mii->mii_media_active) & IFM_FDX) != 0)
		reg |= (CONF_DM);
	else
		reg &= ~(CONF_DM);
	WRITE4(sc, MAC_CONFIGURATION, reg);
#endif
}

static device_method_t xae_methods[] = {
	DEVMETHOD(device_probe,		xae_probe),
	DEVMETHOD(device_attach,	xae_attach),

	/* MII Interface */
	DEVMETHOD(miibus_readreg,	xae_miibus_read_reg),
	DEVMETHOD(miibus_writereg,	xae_miibus_write_reg),
	DEVMETHOD(miibus_statchg,	xae_miibus_statchg),

	{ 0, 0 }
};

driver_t xae_driver = {
	"xae",
	xae_methods,
	sizeof(struct xae_softc),
};

static devclass_t xae_devclass;

DRIVER_MODULE(xae, simplebus, xae_driver, xae_devclass, 0, 0);
DRIVER_MODULE(miibus, xae, miibus_driver, miibus_devclass, 0, 0);

MODULE_DEPEND(xae, ether, 1, 1, 1);
MODULE_DEPEND(xae, miibus, 1, 1, 1);
