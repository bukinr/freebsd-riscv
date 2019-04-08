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

#include <dev/xilinx/if_axi.h>
#include <dev/xilinx/if_axivar.h>
#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#ifdef EXT_RESOURCES
#include <dev/extres/clk/clk.h>
#include <dev/extres/hwreset/hwreset.h>
#endif

#include "if_axi_if.h"
#include "miibus_if.h"

#define	READ4(_sc, _reg) \
	bus_read_4((_sc)->res[0], _reg)
#define	WRITE4(_sc, _reg, _val) \
	bus_write_4((_sc)->res[0], _reg, _val)

#define	MAC_RESET_TIMEOUT	100
#define	WATCHDOG_TIMEOUT_SECS	5
#define	STATS_HARVEST_INTERVAL	2

#define	MDIO_CLK_DIV_DEFAULT	29

#define	DWC_LOCK(sc)			mtx_lock(&(sc)->mtx)
#define	DWC_UNLOCK(sc)			mtx_unlock(&(sc)->mtx)
#define	DWC_ASSERT_LOCKED(sc)		mtx_assert(&(sc)->mtx, MA_OWNED)
#define	DWC_ASSERT_UNLOCKED(sc)		mtx_assert(&(sc)->mtx, MA_NOTOWNED)

#define	DDESC_TDES0_OWN			(1U << 31)
#define	DDESC_TDES0_TXINT		(1U << 30)
#define	DDESC_TDES0_TXLAST		(1U << 29)
#define	DDESC_TDES0_TXFIRST		(1U << 28)
#define	DDESC_TDES0_TXCRCDIS		(1U << 27)
#define	DDESC_TDES0_TXRINGEND		(1U << 21)
#define	DDESC_TDES0_TXCHAIN		(1U << 20)

#define	DDESC_RDES0_OWN			(1U << 31)
#define	DDESC_RDES0_FL_MASK		0x3fff
#define	DDESC_RDES0_FL_SHIFT		16	/* Frame Length */
#define	DDESC_RDES1_CHAINED		(1U << 14)

/* Alt descriptor bits. */
#define	DDESC_CNTL_TXINT		(1U << 31)
#define	DDESC_CNTL_TXLAST		(1U << 30)
#define	DDESC_CNTL_TXFIRST		(1U << 29)
#define	DDESC_CNTL_TXCRCDIS		(1U << 26)
#define	DDESC_CNTL_TXRINGEND		(1U << 25)
#define	DDESC_CNTL_TXCHAIN		(1U << 24)

#define	DDESC_CNTL_CHAINED		(1U << 24)

#define	RX_QUEUE_SIZE		4096
#define	TX_QUEUE_SIZE		4096
#define	NUM_RX_MBUF		512
#define	BUFRING_SIZE		8192

/*
 * A hardware buffer descriptor.  Rx and Tx buffers have the same descriptor
 * layout, but the bits in the fields have different meanings.
 */
struct axi_hwdesc
{
#if 1
	uint32_t tdes0;		/* status for alt layout */
	uint32_t tdes1;		/* cntl for alt layout */
	uint32_t addr;		/* pointer to buffer data */
	uint32_t addr_next;	/* link to next descriptor */
#endif
	uint32_t nxtdesc;
	uint32_t reserved1;
	uint32_t phys;
	uint32_t reserved2;
	uint32_t reserved3;
	uint32_t reserved4;
	uint32_t control;
	uint32_t status;
	uint32_t app0;
	uint32_t app1;
	uint32_t app2;
	uint32_t app3;
	uint32_t app4;
	uint32_t sw_id_offset;
	uint32_t reserved5;
	uint32_t reserved6;
};

/*
 * The hardware imposes alignment restrictions on various objects involved in
 * DMA transfers.  These values are expressed in bytes (not bits).
 */
#define	DWC_DESC_RING_ALIGN		2048

static struct resource_spec axi_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, 0 }
};

static void axi_txfinish_locked(struct axi_softc *sc);
static void axi_rxfinish_locked(struct axi_softc *sc);
static void axi_stop_locked(struct axi_softc *sc);
static void axi_setup_rxfilter(struct axi_softc *sc);

static inline uint32_t
next_rxidx(struct axi_softc *sc, uint32_t curidx)
{

	return ((curidx + 1) % RX_DESC_COUNT);
}

static inline uint32_t
next_txidx(struct axi_softc *sc, uint32_t curidx)
{

	return ((curidx + 1) % TX_DESC_COUNT);
}

static void
axi_get1paddr(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{

	if (error != 0)
		return;
	*(bus_addr_t *)arg = segs[0].ds_addr;
}

inline static uint32_t
axi_setup_txdesc(struct axi_softc *sc, int idx, bus_addr_t paddr,
    uint32_t len)
{
#if 0
	uint32_t flags;
	uint32_t nidx;

	nidx = next_txidx(sc, idx);

	/* Addr/len 0 means we're clearing the descriptor after xmit done. */
	if (paddr == 0 || len == 0) {
		flags = 0;
		--sc->txcount;
	} else {
		if (sc->mactype == DWC_GMAC_ALT_DESC)
			flags = DDESC_CNTL_TXCHAIN | DDESC_CNTL_TXFIRST
			    | DDESC_CNTL_TXLAST | DDESC_CNTL_TXINT;
		else
			flags = DDESC_TDES0_TXCHAIN | DDESC_TDES0_TXFIRST
			    | DDESC_TDES0_TXLAST | DDESC_TDES0_TXINT;
		++sc->txcount;
	}

	sc->txdesc_ring[idx].addr = (uint32_t)(paddr);
	if (sc->mactype == DWC_GMAC_ALT_DESC) {
		sc->txdesc_ring[idx].tdes0 = 0;
		sc->txdesc_ring[idx].tdes1 = flags | len;
	} else {
		sc->txdesc_ring[idx].tdes0 = flags;
		sc->txdesc_ring[idx].tdes1 = len;
	}

	if (paddr && len) {
		wmb();
		sc->txdesc_ring[idx].tdes0 |= DDESC_TDES0_OWN;
		wmb();
	}

	return (nidx);
#endif
	return (0);
}

#if 0
static int
axi_setup_txbuf(struct axi_softc *sc, int idx, struct mbuf **mp)
{
	struct bus_dma_segment seg;
	int error, nsegs;
	struct mbuf * m;

	if ((m = m_defrag(*mp, M_NOWAIT)) == NULL)
		return (ENOMEM);
	*mp = m;

	error = bus_dmamap_load_mbuf_sg(sc->txbuf_tag, sc->txbuf_map[idx].map,
	    m, &seg, &nsegs, 0);
	if (error != 0) {
		return (ENOMEM);
	}

	KASSERT(nsegs == 1, ("%s: %d segments returned!", __func__, nsegs));

	bus_dmamap_sync(sc->txbuf_tag, sc->txbuf_map[idx].map,
	    BUS_DMASYNC_PREWRITE);

	sc->txbuf_map[idx].mbuf = m;

	axi_setup_txdesc(sc, idx, seg.ds_addr, seg.ds_len);

	return (0);
}
#endif

static int
axi_xdma_tx_intr(void *arg, xdma_transfer_status_t *status)
{
	xdma_transfer_status_t st;
	struct axi_softc *sc;
	struct ifnet *ifp;
	struct mbuf *m;
	int err;

	sc = arg;

	DWC_LOCK(sc);

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
		sc->txcount--;
	}

	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	DWC_UNLOCK(sc);

	return (0);
}

#if 0
static void
axi_txstart_locked(struct axi_softc *sc)
{
	struct ifnet *ifp;
	struct mbuf *m;
	int enqueued;

	DWC_ASSERT_LOCKED(sc);

	printf("%s\n", __func__);

	if (!sc->link_is_up)
		return;

	ifp = sc->ifp;

	if (ifp->if_drv_flags & IFF_DRV_OACTIVE)
		return;

	enqueued = 0;

	for (;;) {
		if (sc->txcount == (TX_DESC_COUNT-1)) {
			ifp->if_drv_flags |= IFF_DRV_OACTIVE;
			break;
		}

		IFQ_DRV_DEQUEUE(&ifp->if_snd, m);
		if (m == NULL)
			break;
		if (axi_setup_txbuf(sc, sc->tx_idx_head, &m) != 0) {
			IFQ_DRV_PREPEND(&ifp->if_snd, m);
			break;
		}
		BPF_MTAP(ifp, m);
		sc->tx_idx_head = next_txidx(sc, sc->tx_idx_head);
		++enqueued;
	}

	if (enqueued != 0) {
#if 0
		WRITE4(sc, TRANSMIT_POLL_DEMAND, 0x1);
		sc->tx_watchdog_count = WATCHDOG_TIMEOUT_SECS;
#endif
	}
}

static void
axi_txstart(struct ifnet *ifp)
{
	struct axi_softc *sc = ifp->if_softc;

	DWC_LOCK(sc);
	axi_txstart_locked(sc);
	DWC_UNLOCK(sc);
}
#endif

static void
axi_qflush(struct ifnet *ifp)
{
	struct atse_softc *sc;

	sc = ifp->if_softc;

	printf("%s\n", __func__);
}

static int
axi_transmit_locked(struct ifnet *ifp)
{
	struct axi_softc *sc;
	struct mbuf *m;
	struct buf_ring *br;
	int error;
	int enq;

	printf("%s\n", __func__);

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

		sc->txcount++;
		enq++;

		/* If anyone is interested give them a copy. */
		ETHER_BPF_MTAP(ifp, m);
        }

	if (enq > 0)
		xdma_queue_submit(sc->xchan_tx);

	return (0);
}

static int
axi_transmit(struct ifnet *ifp, struct mbuf *m)
{
	struct axi_softc *sc;
	struct buf_ring *br;
	int error;

	printf("%s\n", __func__);

	sc = ifp->if_softc;
	br = sc->br;

	DWC_LOCK(sc);

	mtx_lock(&sc->br_mtx);

	if ((ifp->if_drv_flags & (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) !=
	    IFF_DRV_RUNNING) {
		error = drbr_enqueue(ifp, sc->br, m);
		mtx_unlock(&sc->br_mtx);
		DWC_UNLOCK(sc);
		return (error);
	}

	//if ((sc->atse_flags & DWC_FLAGS_LINK) == 0) {
	if (!sc->link_is_up) {
		error = drbr_enqueue(ifp, sc->br, m);
		mtx_unlock(&sc->br_mtx);
		DWC_UNLOCK(sc);
		return (error);
	}

	error = drbr_enqueue(ifp, br, m);
	if (error) {
		mtx_unlock(&sc->br_mtx);
		DWC_UNLOCK(sc);
		return (error);
	}
	error = axi_transmit_locked(ifp);

	mtx_unlock(&sc->br_mtx);
	DWC_UNLOCK(sc);

	return (error);
}

static void
axi_stop_locked(struct axi_softc *sc)
{
#if 0
	struct ifnet *ifp;
	uint32_t reg;

	DWC_ASSERT_LOCKED(sc);

	ifp = sc->ifp;
	ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);
	sc->tx_watchdog_count = 0;
	sc->stats_harvest_count = 0;

	callout_stop(&sc->axi_callout);

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

static void axi_clear_stats(struct axi_softc *sc)
{
#if 0
	uint32_t reg;

	reg = READ4(sc, MMC_CONTROL);
	reg |= (MMC_CONTROL_CNTRST);
	WRITE4(sc, MMC_CONTROL, reg);
#endif
}

static void
axi_harvest_stats(struct axi_softc *sc)
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

	axi_clear_stats(sc);
}

static void
axi_tick(void *arg)
{
	struct axi_softc *sc;
	struct ifnet *ifp;
	int link_was_up;

	sc = arg;

	DWC_ASSERT_LOCKED(sc);

	ifp = sc->ifp;

	if (!(ifp->if_drv_flags & IFF_DRV_RUNNING))
		return;

	/*
	 * Typical tx watchdog.  If this fires it indicates that we enqueued
	 * packets for output and never got a txdone interrupt for them.  Maybe
	 * it's a missed interrupt somehow, just pretend we got one.
	 */
	if (sc->tx_watchdog_count > 0) {
		if (--sc->tx_watchdog_count == 0) {
			axi_txfinish_locked(sc);
		}
	}

	/* Gather stats from hardware counters. */
	axi_harvest_stats(sc);

	/* Check the media status. */
	link_was_up = sc->link_is_up;
	mii_tick(sc->mii_softc);
	if (sc->link_is_up && !link_was_up)
		axi_transmit_locked(sc->ifp);

	/* Schedule another check one second from now. */
	callout_reset(&sc->axi_callout, hz, axi_tick, sc);
}

static void
axi_init_locked(struct axi_softc *sc)
{
	struct ifnet *ifp;

	DWC_ASSERT_LOCKED(sc);

	ifp = sc->ifp;
	if (ifp->if_drv_flags & IFF_DRV_RUNNING)
		return;

	ifp->if_drv_flags |= IFF_DRV_RUNNING;

#if 0
	axi_setup_rxfilter(sc);

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

	/*
	 * Call mii_mediachg() which will call back into axi_miibus_statchg()
	 * to set up the remaining config registers based on current media.
	 */
	mii_mediachg(sc->mii_softc);
	callout_reset(&sc->axi_callout, hz, axi_tick, sc);
}

static void
axi_init(void *if_softc)
{
	struct axi_softc *sc = if_softc;

	DWC_LOCK(sc);
	axi_init_locked(sc);
	DWC_UNLOCK(sc);
}

inline static uint32_t
axi_setup_rxdesc(struct axi_softc *sc, int idx, bus_addr_t paddr)
{
#if 0
	uint32_t nidx;

	sc->rxdesc_ring[idx].addr = (uint32_t)paddr;
	nidx = next_rxidx(sc, idx);
	sc->rxdesc_ring[idx].addr_next = sc->rxdesc_ring_paddr +	\
	    (nidx * sizeof(struct axi_hwdesc));
	if (sc->mactype == DWC_GMAC_ALT_DESC)
		sc->rxdesc_ring[idx].tdes1 = DDESC_CNTL_CHAINED | RX_MAX_PACKET;
	else
		sc->rxdesc_ring[idx].tdes1 = DDESC_RDES1_CHAINED | MCLBYTES;

	wmb();
	sc->rxdesc_ring[idx].tdes0 = DDESC_RDES0_OWN;
	wmb();

	return (nidx);
#endif
	return (0);
}

static int
axi_setup_rxbuf(struct axi_softc *sc, int idx, struct mbuf *m)
{
	struct bus_dma_segment seg;
	int error, nsegs;

	m_adj(m, ETHER_ALIGN);

	error = bus_dmamap_load_mbuf_sg(sc->rxbuf_tag, sc->rxbuf_map[idx].map,
	    m, &seg, &nsegs, 0);
	if (error != 0) {
		return (error);
	}

	KASSERT(nsegs == 1, ("%s: %d segments returned!", __func__, nsegs));

	bus_dmamap_sync(sc->rxbuf_tag, sc->rxbuf_map[idx].map,
	    BUS_DMASYNC_PREREAD);

	sc->rxbuf_map[idx].mbuf = m;
	axi_setup_rxdesc(sc, idx, seg.ds_addr);

	return (0);
}

static struct mbuf *
axi_alloc_mbufcl(struct axi_softc *sc)
{
	struct mbuf *m;

	m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
	if (m != NULL)
		m->m_pkthdr.len = m->m_len = m->m_ext.ext_size;

	return (m);
}

static void
axi_media_status(struct ifnet * ifp, struct ifmediareq *ifmr)
{
	struct axi_softc *sc;
	struct mii_data *mii;

	sc = ifp->if_softc;
	mii = sc->mii_softc;
	DWC_LOCK(sc);
	mii_pollstat(mii);
	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;
	DWC_UNLOCK(sc);
}

static int
axi_media_change_locked(struct axi_softc *sc)
{

	printf("%s\n", __func__);

	return (mii_mediachg(sc->mii_softc));
}

static int
axi_media_change(struct ifnet * ifp)
{
	struct axi_softc *sc;
	int error;

	sc = ifp->if_softc;

	DWC_LOCK(sc);
	error = axi_media_change_locked(sc);
	DWC_UNLOCK(sc);
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
axi_setup_rxfilter(struct axi_softc *sc)
{
	struct ifmultiaddr *ifma;
	struct ifnet *ifp;
	uint8_t *eaddr, val;
	//uint32_t crc, ffval, hashbit, hashreg, hi, lo, hash[8];
	uint32_t crc, hashbit, hashreg, hi, lo, hash[8];
	int nhash, i;

	DWC_ASSERT_LOCKED(sc);

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
axi_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct axi_softc *sc;
	struct mii_data *mii;
	struct ifreq *ifr;
	int mask, error;

	sc = ifp->if_softc;
	ifr = (struct ifreq *)data;

	error = 0;
	switch (cmd) {
	case SIOCSIFFLAGS:
		DWC_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
				if ((ifp->if_flags ^ sc->if_flags) &
				    (IFF_PROMISC | IFF_ALLMULTI))
					axi_setup_rxfilter(sc);
			} else {
				if (!sc->is_detaching)
					axi_init_locked(sc);
			}
		} else {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING)
				axi_stop_locked(sc);
		}
		sc->if_flags = ifp->if_flags;
		DWC_UNLOCK(sc);
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
			DWC_LOCK(sc);
			axi_setup_rxfilter(sc);
			DWC_UNLOCK(sc);
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
axi_txfinish_locked(struct axi_softc *sc)
{
	struct axi_bufmap *bmap;
	struct axi_hwdesc *desc;
	struct ifnet *ifp;

	DWC_ASSERT_LOCKED(sc);

	ifp = sc->ifp;
	while (sc->tx_idx_tail != sc->tx_idx_head) {
		desc = &sc->txdesc_ring[sc->tx_idx_tail];
		if ((desc->tdes0 & DDESC_TDES0_OWN) != 0)
			break;
		bmap = &sc->txbuf_map[sc->tx_idx_tail];
		bus_dmamap_sync(sc->txbuf_tag, bmap->map,
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->txbuf_tag, bmap->map);
		m_freem(bmap->mbuf);
		bmap->mbuf = NULL;
		axi_setup_txdesc(sc, sc->tx_idx_tail, 0, 0);
		sc->tx_idx_tail = next_txidx(sc, sc->tx_idx_tail);
		ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
		if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);
	}

	/* If there are no buffers outstanding, muzzle the watchdog. */
	if (sc->tx_idx_tail == sc->tx_idx_head) {
		sc->tx_watchdog_count = 0;
	}
}

static void __unused
axi_rxfinish_locked(struct axi_softc *sc)
{
	struct ifnet *ifp;
	struct mbuf *m0;
	struct mbuf *m;
	int error, idx, len;
	uint32_t rdes0;

	ifp = sc->ifp;

	for (;;) {
		idx = sc->rx_idx;

		rdes0 = sc->rxdesc_ring[idx].tdes0;
		if ((rdes0 & DDESC_RDES0_OWN) != 0)
			break;

		bus_dmamap_sync(sc->rxbuf_tag, sc->rxbuf_map[idx].map,
		    BUS_DMASYNC_POSTREAD);
		bus_dmamap_unload(sc->rxbuf_tag, sc->rxbuf_map[idx].map);

		len = (rdes0 >> DDESC_RDES0_FL_SHIFT) & DDESC_RDES0_FL_MASK;
		if (len != 0) {
			m = sc->rxbuf_map[idx].mbuf;
			m->m_pkthdr.rcvif = ifp;
			m->m_pkthdr.len = len;
			m->m_len = len;
			if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);

			/* Remove trailing FCS */
			m_adj(m, -ETHER_CRC_LEN);

			DWC_UNLOCK(sc);
			(*ifp->if_input)(ifp, m);
			DWC_LOCK(sc);
		} else {
			/* XXX Zero-length packet ? */
		}

		if ((m0 = axi_alloc_mbufcl(sc)) != NULL) {
			if ((error = axi_setup_rxbuf(sc, idx, m0)) != 0) {
				/*
				 * XXX Now what?
				 * We've got a hole in the rx ring.
				 */
			}
		} else
			if_inc_counter(sc->ifp, IFCOUNTER_IQDROPS, 1);

		sc->rx_idx = next_rxidx(sc, sc->rx_idx);
	}
}

static void
axi_intr(void *arg)
{
#if 0
	struct axi_softc *sc;
	uint32_t reg;

	sc = arg;

	DWC_LOCK(sc);

	reg = READ4(sc, INTERRUPT_STATUS);
	if (reg)
		READ4(sc, SGMII_RGMII_SMII_CTRL_STATUS);

	reg = READ4(sc, DMA_STATUS);
	if (reg & DMA_STATUS_NIS) {
		if (reg & DMA_STATUS_RI)
			axi_rxfinish_locked(sc);

		if (reg & DMA_STATUS_TI) {
			axi_txfinish_locked(sc);
			axi_txstart_locked(sc);
		}
	}

	if (reg & DMA_STATUS_AIS) {
		if (reg & DMA_STATUS_FBI) {
			/* Fatal bus error */
			device_printf(sc->dev,
			    "Ethernet DMA error, restarting controller.\n");
			axi_stop_locked(sc);
			axi_init_locked(sc);
		}
	}

	WRITE4(sc, DMA_STATUS, reg & DMA_STATUS_INTR_MASK);
	DWC_UNLOCK(sc);
#endif
}

static int __unused
setup_dma(struct axi_softc *sc)
{
	struct mbuf *m;
	int error;
	int nidx;
	int idx;

	/*
	 * Set up TX descriptor ring, descriptors, and dma maps.
	 */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),	/* Parent tag. */
	    DWC_DESC_RING_ALIGN, 0,	/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    TX_DESC_SIZE, 1, 		/* maxsize, nsegments */
	    TX_DESC_SIZE,		/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->txdesc_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create TX ring DMA tag.\n");
		goto out;
	}

	error = bus_dmamem_alloc(sc->txdesc_tag, (void**)&sc->txdesc_ring,
	    BUS_DMA_COHERENT | BUS_DMA_WAITOK | BUS_DMA_ZERO,
	    &sc->txdesc_map);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not allocate TX descriptor ring.\n");
		goto out;
	}

	error = bus_dmamap_load(sc->txdesc_tag, sc->txdesc_map,
	    sc->txdesc_ring, TX_DESC_SIZE, axi_get1paddr,
	    &sc->txdesc_ring_paddr, 0);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not load TX descriptor ring map.\n");
		goto out;
	}

	for (idx = 0; idx < TX_DESC_COUNT; idx++) {
		nidx = next_txidx(sc, idx);
		sc->txdesc_ring[idx].addr_next = sc->txdesc_ring_paddr +
		    (nidx * sizeof(struct axi_hwdesc));
	}

	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),	/* Parent tag. */
	    1, 0,			/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    MCLBYTES, 1, 		/* maxsize, nsegments */
	    MCLBYTES,			/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->txbuf_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create TX ring DMA tag.\n");
		goto out;
	}

	for (idx = 0; idx < TX_DESC_COUNT; idx++) {
		error = bus_dmamap_create(sc->txbuf_tag, BUS_DMA_COHERENT,
		    &sc->txbuf_map[idx].map);
		if (error != 0) {
			device_printf(sc->dev,
			    "could not create TX buffer DMA map.\n");
			goto out;
		}
		axi_setup_txdesc(sc, idx, 0, 0);
	}

	/*
	 * Set up RX descriptor ring, descriptors, dma maps, and mbufs.
	 */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),	/* Parent tag. */
	    DWC_DESC_RING_ALIGN, 0,	/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    RX_DESC_SIZE, 1, 		/* maxsize, nsegments */
	    RX_DESC_SIZE,		/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->rxdesc_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create RX ring DMA tag.\n");
		goto out;
	}

	error = bus_dmamem_alloc(sc->rxdesc_tag, (void **)&sc->rxdesc_ring,
	    BUS_DMA_COHERENT | BUS_DMA_WAITOK | BUS_DMA_ZERO,
	    &sc->rxdesc_map);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not allocate RX descriptor ring.\n");
		goto out;
	}

	error = bus_dmamap_load(sc->rxdesc_tag, sc->rxdesc_map,
	    sc->rxdesc_ring, RX_DESC_SIZE, axi_get1paddr,
	    &sc->rxdesc_ring_paddr, 0);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not load RX descriptor ring map.\n");
		goto out;
	}

	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),	/* Parent tag. */
	    1, 0,			/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    MCLBYTES, 1, 		/* maxsize, nsegments */
	    MCLBYTES,			/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->rxbuf_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create RX buf DMA tag.\n");
		goto out;
	}

	for (idx = 0; idx < RX_DESC_COUNT; idx++) {
		error = bus_dmamap_create(sc->rxbuf_tag, BUS_DMA_COHERENT,
		    &sc->rxbuf_map[idx].map);
		if (error != 0) {
			device_printf(sc->dev,
			    "could not create RX buffer DMA map.\n");
			goto out;
		}
		if ((m = axi_alloc_mbufcl(sc)) == NULL) {
			device_printf(sc->dev, "Could not alloc mbuf\n");
			error = ENOMEM;
			goto out;
		}
		if ((error = axi_setup_rxbuf(sc, idx, m)) != 0) {
			device_printf(sc->dev,
			    "could not create new RX buffer.\n");
			goto out;
		}
	}

out:
	if (error != 0)
		return (ENXIO);

	return (0);
}

static int __unused
axi_get_hwaddr(struct axi_softc *sc, uint8_t *hwaddr)
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

#define	GPIO_ACTIVE_LOW 1

static int __unused
axi_reset(device_t dev)
{

	return (0);
}

#ifdef EXT_RESOURCES
static int
axi_clock_init(device_t dev)
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
axi_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "xlnx,axi-ethernet-1.00.a"))
		return (ENXIO);

	device_set_desc(dev, "Xilinx AXI Ethernet");
	return (BUS_PROBE_DEFAULT);
}

static int
axi_attach(device_t dev)
{
	uint8_t macaddr[ETHER_ADDR_LEN];
	struct axi_softc *sc;
	struct ifnet *ifp;
	int error; // i;
	//uint32_t reg;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->rx_idx = 0;

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
	/* Alloc xDMA virtual channel. */
	sc->xchan_tx = xdma_channel_alloc(sc->xdma_tx, caps);
	if (sc->xchan_tx == NULL) {
		device_printf(dev, "Can't alloc virtual DMA channel.\n");
		return (ENXIO);
	}

	/* Setup interrupt handler. */
	error = xdma_setup_intr(sc->xchan_tx,
	    axi_xdma_tx_intr, sc, &sc->ih_tx);
	if (error) {
		device_printf(sc->dev,
		    "Can't setup xDMA interrupt handler.\n");
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


	mtx_init(&sc->br_mtx, "buf ring mtx", NULL, MTX_DEF);
	sc->br = buf_ring_alloc(BUFRING_SIZE, M_DEVBUF,
	    M_NOWAIT, &sc->br_mtx);
	if (sc->br == NULL)
		return (ENOMEM);

#if 0
	sc->txcount = TX_DESC_COUNT;
	sc->mii_clk = IF_DWC_MII_CLK(dev);
	sc->mactype = IF_DWC_MAC_TYPE(dev);

	if (IF_DWC_INIT(dev) != 0)
		return (ENXIO);
#endif

#ifdef EXT_RESOURCES
	if (axi_clock_init(dev) != 0)
		return (ENXIO);
#endif

	if (bus_alloc_resources(dev, axi_spec, sc->res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	/* Memory interface */
	sc->bst = rman_get_bustag(sc->res[0]);
	sc->bsh = rman_get_bushandle(sc->res[0]);

	printf("ID: %x\n", READ4(sc, AXI_IDENT));

	uint32_t reg;

	reg = (MDIO_CLK_DIV_DEFAULT << MDIO_SETUP_CLK_DIV_S);
	reg |= MDIO_SETUP_ENABLE;
	WRITE4(sc, AXI_MDIO_SETUP, reg);

#if 0
	/* Read MAC before reset */
	if (axi_get_hwaddr(sc, macaddr)) {
		device_printf(sc->dev, "can't get mac\n");
		return (ENXIO);
	}

	/* Reset the PHY if needed */
	if (axi_reset(dev) != 0) {
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

	callout_init_mtx(&sc->axi_callout, &sc->mtx, 0);

	/* Setup interrupt handler. */
	error = bus_setup_intr(dev, sc->res[1], INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, axi_intr, sc, &sc->intr_cookie);
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
	ifp->if_transmit = axi_transmit;
	ifp->if_qflush = axi_qflush;
	ifp->if_ioctl = axi_ioctl;
	ifp->if_init = axi_init;
	IFQ_SET_MAXLEN(&ifp->if_snd, TX_DESC_COUNT - 1);
	ifp->if_snd.ifq_drv_maxlen = TX_DESC_COUNT - 1;
	IFQ_SET_READY(&ifp->if_snd);

	/* Attach the mii driver. */
	error = mii_attach(dev, &sc->miibus, ifp, axi_media_change,
	    axi_media_status, BMSR_DEFCAPMASK, 3, //MII_PHY_ANY,
	    MII_OFFSET_ANY, 0);

	if (error != 0) {
		device_printf(dev, "PHY attach failed\n");
		return (ENXIO);
	}
	sc->mii_softc = device_get_softc(sc->miibus);

	/* All ready to run, attach the ethernet interface. */
	macaddr[0] = 0x00;
	macaddr[1] = 0x0a;
	macaddr[2] = 0x35;
	macaddr[3] = 0x04;
	macaddr[4] = 0xdb;
	macaddr[5] = 0x5a;
	ether_ifattach(ifp, macaddr);
	sc->is_attached = true;

	/* Enable the transmitter */
	printf("%s: axi_tc %x\n", __func__, READ4(sc, AXI_TC));
	WRITE4(sc, AXI_TC, TC_TX);

	return (0);
}

static int
mdio_wait(struct axi_softc *sc)
{
	uint32_t reg;
	int timeout;

	timeout = 200;

	do {
		reg = READ4(sc, AXI_MDIO_CTRL);
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
axi_miibus_read_reg(device_t dev, int phy, int reg)
{
	struct axi_softc *sc;
	uint32_t mii;
	int rv;

	//printf("%s: phy %d reg %x\n", __func__, phy, reg);

	sc = device_get_softc(dev);

	if (mdio_wait(sc))
		return (0);

	mii = MDIO_CTRL_TX_OP_READ | MDIO_CTRL_INITIATE;
	mii |= (reg << MDIO_TX_REGAD_S);
	mii |= (phy << MDIO_TX_PHYAD_S);

	WRITE4(sc, AXI_MDIO_CTRL, mii);

	if (mdio_wait(sc))
		return (0);

	rv = READ4(sc, AXI_MDIO_READ);
	//printf("%s: phy %d reg %x, rv %x\n", __func__, phy, reg, rv);

	return (rv);
}

static int
axi_miibus_write_reg(device_t dev, int phy, int reg, int val)
{
	struct axi_softc *sc;
	uint32_t mii;

	sc = device_get_softc(dev);

	//printf("%s: phy %d reg %x val %x\n", __func__, phy, reg, val);

	if (mdio_wait(sc))
		return (1);

	mii = MDIO_CTRL_TX_OP_WRITE | MDIO_CTRL_INITIATE;
	mii |= (reg << MDIO_TX_REGAD_S);
	mii |= (phy << MDIO_TX_PHYAD_S);

	WRITE4(sc, AXI_MDIO_WRITE, val);
	WRITE4(sc, AXI_MDIO_CTRL, mii);

	if (mdio_wait(sc))
		return (1);

	//printf("%s: phy %d reg %x val %x, success\n", __func__, phy, reg, val);

	return (0);
}

static void
axi_miibus_statchg(device_t dev)
{
	struct axi_softc *sc;
	struct mii_data *mii;
	uint32_t reg;

	/*
	 * Called by the MII bus driver when the PHY establishes
	 * link to set the MAC interface registers.
	 */

	printf("%s\n", __func__);

	sc = device_get_softc(dev);

	DWC_ASSERT_LOCKED(sc);

	mii = sc->mii_softc;

	if (mii->mii_media_status & IFM_ACTIVE)
		sc->link_is_up = true;
	else
		sc->link_is_up = false;

	printf("link_is_up %d\n", sc->link_is_up);
	printf("%s: IFM_SUBTYPE(mii->mii_media_active) %d\n",
	    __func__, IFM_SUBTYPE(mii->mii_media_active));
	printf("%s: options %x\n",
	    __func__, IFM_OPTIONS(mii->mii_media_active));
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

	WRITE4(sc, AXI_SPEED, reg);
	DELAY(1);

#if 0
	if ((IFM_OPTIONS(mii->mii_media_active) & IFM_FDX) != 0)
		reg |= (CONF_DM);
	else
		reg &= ~(CONF_DM);
	WRITE4(sc, MAC_CONFIGURATION, reg);
#endif
}

static device_method_t axi_methods[] = {
	DEVMETHOD(device_probe,		axi_probe),
	DEVMETHOD(device_attach,	axi_attach),

	/* MII Interface */
	DEVMETHOD(miibus_readreg,	axi_miibus_read_reg),
	DEVMETHOD(miibus_writereg,	axi_miibus_write_reg),
	DEVMETHOD(miibus_statchg,	axi_miibus_statchg),

	{ 0, 0 }
};

driver_t axi_driver = {
	"axi",
	axi_methods,
	sizeof(struct axi_softc),
};

static devclass_t axi_devclass;

DRIVER_MODULE(axi, simplebus, axi_driver, axi_devclass, 0, 0);
DRIVER_MODULE(miibus, axi, miibus_driver, miibus_devclass, 0, 0);

MODULE_DEPEND(axi, ether, 1, 1, 1);
MODULE_DEPEND(axi, miibus, 1, 1, 1);
