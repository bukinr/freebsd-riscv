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

#ifndef _DEV_XDMA_CONTROLLER_PL330_H_
#define _DEV_XDMA_CONTROLLER_PL330_H_

#define	AXI_DMACR		0x00 /* MM2S DMA Control register */
#define	 DMACR_RS		(1 << 0) /* Run / Stop. */
#define	 DMACR_RESET		(1 << 2) /* Soft reset the AXI DMA core. */
#define	 DMACR_IOC_IRQEN	(1 << 12) /* Interrupt on Complete (IOC) Interrupt Enable. */
#define	 DMACR_DLY_IRQEN	(1 << 13) /* Interrupt on Delay Timer Interrupt Enable. */
#define	 DMACR_ERR_IRQEN	(1 << 14) /* Interrupt on Error Interrupt Enable. */
#define	AXI_DMASR		0x04 /* MM2S DMA Status register */
#define	 DMACR_HALTED		(1 << 0) /* Halted. */
#define	AXI_CURDESC		0x08 /* MM2S Current Descriptor Pointer. Lower 32 bits of the address. */
#define	AXI_CURDESC_MSB		0x0C /* MM2S Current Descriptor Pointer. Upper 32 bits of address. */
#define	AXI_TAILDESC		0x10 /* MM2S Tail Descriptor Pointer. Lower 32 bits. */
#define	AXI_TAILDESC_MSB	0x14 /* MM2S Tail Descriptor Pointer. Upper 32 bits of address. */
#define	SG_CTL			0x2C /* Scatter/Gather User and Cache */

#define	READ4(_sc, _reg)	\
	bus_space_read_4(_sc->bst, _sc->bsh, _reg)
#define	WRITE4(_sc, _reg, _val)	\
	bus_space_write_4(_sc->bst, _sc->bsh, _reg, _val)

struct axidma_desc {
	uint32_t next;
	uint32_t reserved1;
	uint32_t phys;
	uint32_t reserved2;
	uint32_t reserved3;
	uint32_t reserved4;
	uint32_t control;
#define	CONTROL_TXSOF	(1 << 27) /* Start of Frame. */
#define	CONTROL_TXEOF	(1 << 26) /* End of Frame. */
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

#endif /* !_DEV_XDMA_CONTROLLER_PL330_H_ */
