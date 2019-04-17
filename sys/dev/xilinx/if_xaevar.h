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
 *
 * $FreeBSD$
 */

#ifndef	_DEV_XILINX_IF_XAEVAR_H_
#define	_DEV_XILINX_IF_XAEVAR_H_

#include <dev/xdma/xdma.h>

/*
 * Driver data and defines.
 */
#define	RX_MAX_PACKET	0x7ff
#define	RX_DESC_COUNT	1024
#define	RX_DESC_SIZE	(sizeof(struct xae_hwdesc) * RX_DESC_COUNT)
#define	TX_DESC_COUNT	1024
#define	TX_DESC_SIZE	(sizeof(struct xae_hwdesc) * TX_DESC_COUNT)

struct xae_bufmap {
	bus_dmamap_t		map;
	struct mbuf		*mbuf;
};

struct xae_softc {
	struct resource		*res[2];
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	device_t		dev;
	int			mactype;
	int			mii_clk;
	device_t		miibus;
	struct mii_data *	mii_softc;
	struct ifnet		*ifp;
	int			if_flags;
	struct mtx		mtx;
	void *			intr_cookie;
	struct callout		xae_callout;
	boolean_t		link_is_up;
	boolean_t		is_attached;
	boolean_t		is_detaching;
	int			tx_watchdog_count;
	int			stats_harvest_count;
	int			phy_addr;

	/* xDMA */
	xdma_controller_t	*xdma_tx;
	xdma_channel_t		*xchan_tx;
	void			*ih_tx;

	xdma_controller_t	*xdma_rx;
	xdma_channel_t		*xchan_rx;
	void			*ih_rx;

	struct buf_ring		*br;
	struct mtx		br_mtx;

	/* RX */
	bus_dma_tag_t		rxdesc_tag;
	bus_dmamap_t		rxdesc_map;
	struct xae_hwdesc	*rxdesc_ring;
	bus_addr_t		rxdesc_ring_paddr;
	bus_dma_tag_t		rxbuf_tag;
	struct xae_bufmap	rxbuf_map[RX_DESC_COUNT];
	uint32_t		rx_idx;

	/* TX */
	bus_dma_tag_t		txdesc_tag;
	bus_dmamap_t		txdesc_map;
	struct xae_hwdesc	*txdesc_ring;
	bus_addr_t		txdesc_ring_paddr;
	bus_dma_tag_t		txbuf_tag;
	struct xae_bufmap	txbuf_map[TX_DESC_COUNT];
	uint32_t		tx_idx_head;
	uint32_t		tx_idx_tail;
	int			txcount;
};

#endif	/* _DEV_XILINX_IF_XAEVAR_H_ */
