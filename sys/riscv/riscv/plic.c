/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2018 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * This software was developed by SRI International and the University of
 * Cambridge Computer Laboratory under DARPA/AFRL contract FA8750-10-C-0237
 * ("CTSRD"), as part of the DARPA CRASH research programme.
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
#include <sys/ktr.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "pic_if.h"

#ifdef  DEBUG
#define dprintf(fmt, args...) printf(fmt, ##args)
#else
#define dprintf(fmt, args...)
#endif

#define	PLIC_NIRQS		32

#define	PRIORITY_BASE		0
#define	PRIORITY_PER_ID		4
#define	ENABLE_BASE		0x2000
#define	ENABLE_PER_HART		0x80
#define	CONTEXT_BASE		0x200000
#define	CONTEXT_PER_HART	0x1000
#define	CONTEXT_THRESHOLD	0
#define	CONTEXT_CLAIM		4

#define	IRQ_ENABLE(n, h)	(ENABLE_BASE + (ENABLE_PER_HART * (h)) + ((n) / 32))
#define	IRQ_PRIORITY(n)		(PRIORITY_BASE + (n) * PRIORITY_PER_ID)
#define	IRQ_CLAIM(h)		(CONTEXT_BASE + CONTEXT_PER_HART * (h) + CONTEXT_CLAIM)
#define	IRQ_THRESHOLD(h)	(CONTEXT_BASE + CONTEXT_PER_HART * (h) + CONTEXT_THRESHOLD)

struct plic_irqsrc {
	struct intr_irqsrc	isrc;
	u_int			irq;
};

struct plic_softc {
	device_t		dev;
	struct resource *	intc_res;
	struct plic_irqsrc	isrcs[PLIC_NIRQS];
};

#define	RD4(sc, reg)				\
    bus_read_4(sc->intc_res, (reg))
#define	WR4(sc, reg, val)			\
    bus_write_4(sc->intc_res, (reg), (val))

static inline void
plic_irq_dispatch(struct plic_softc *sc, u_int irq,
    struct trapframe *tf)
{
	struct plic_irqsrc *src;

	src = &sc->isrcs[irq];

	if (intr_isrc_dispatch(&src->isrc, tf) != 0)
		device_printf(sc->dev, "Stray irq %u detected\n", irq);
}

static int
plic_intr(void *arg)
{
	struct plic_softc *sc;
	struct trapframe *tf;
	uint32_t pending;
	uint32_t cpu;

	sc = arg;
	cpu = PCPU_GET(cpuid);

	pending = RD4(sc, IRQ_CLAIM(cpu));
	if (pending) {
		tf = curthread->td_intr_frame;
		plic_irq_dispatch(sc, pending, tf);
		WR4(sc, IRQ_CLAIM(cpu), pending);
	}

	return (FILTER_HANDLED);
}

static void
plic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct plic_softc *sc;
	struct plic_irqsrc *src;
	uint32_t reg;

	sc = device_get_softc(dev);
	src = (struct plic_irqsrc *)isrc;

	reg = RD4(sc, IRQ_ENABLE(src->irq, 0));
	reg &= ~(1 << (src->irq % 32));
	WR4(sc, IRQ_ENABLE(src->irq, 0), reg);
}

static void
plic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct plic_softc *sc;
	struct plic_irqsrc *src;
	uint32_t reg;

	sc = device_get_softc(dev);
	src = (struct plic_irqsrc *)isrc;

	WR4(sc, IRQ_PRIORITY(src->irq), 1);

	reg = RD4(sc, IRQ_ENABLE(src->irq, 0));
	reg |= (1 << (src->irq % 32));
	WR4(sc, IRQ_ENABLE(src->irq, 0), reg);

	csr_set(sie, SIE_SEIE);
}

static int
plic_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	struct intr_map_data_fdt *daf;
	struct plic_softc *sc;

	sc = device_get_softc(dev);

	if (data->type != INTR_MAP_DATA_FDT)
		return (ENOTSUP);

	daf = (struct intr_map_data_fdt *)data;
	if (daf->ncells != 1 || daf->cells[0] >= PLIC_NIRQS)
		return (EINVAL);

	*isrcp = &sc->isrcs[daf->cells[0]].isrc;

	return (0);
}

static int
plic_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "riscv,plic0"))
		return (ENXIO);

	device_set_desc(dev, "RISC-V PLIC");

	return (BUS_PROBE_DEFAULT);
}

static int
plic_attach(device_t dev)
{
	struct plic_irqsrc *isrcs;
	struct plic_softc *sc;
	struct intr_pic *pic;
	uint32_t irq;
	const char *name;
	phandle_t xref;
	int error;
	int rid;

	sc = device_get_softc(dev);

	sc->dev = dev;

	/* Request memory resources */
	rid = 0;
	sc->intc_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->intc_res == NULL) {
		device_printf(dev,
		    "Error: could not allocate memory resources\n");
		return (ENXIO);
	}

	isrcs = sc->isrcs;
	name = device_get_nameunit(sc->dev);
	for (irq = 0; irq < PLIC_NIRQS; irq++) {
		isrcs[irq].irq = irq;
		error = intr_isrc_register(&isrcs[irq].isrc, sc->dev,
		    0, "%s,%u", name, irq);
		if (error != 0)
			return (error);

		WR4(sc, IRQ_ENABLE(irq, 0), 0);
	}
	WR4(sc, IRQ_THRESHOLD(0), 0);

	xref = OF_xref_from_node(ofw_bus_get_node(sc->dev));
	pic = intr_pic_register(sc->dev, xref);
	if (pic == NULL)
		return (ENXIO);

	return (intr_pic_claim_root(sc->dev, xref, plic_intr, sc, 0));
}

static device_method_t plic_methods[] = {
	DEVMETHOD(device_probe,		plic_probe),
	DEVMETHOD(device_attach,	plic_attach),

	DEVMETHOD(pic_disable_intr,	plic_disable_intr),
	DEVMETHOD(pic_enable_intr,	plic_enable_intr),
	DEVMETHOD(pic_map_intr,		plic_map_intr),

	DEVMETHOD_END
};

static driver_t plic_driver = {
	"plic",
	plic_methods,
	sizeof(struct plic_softc),
};

static devclass_t plic_devclass;

EARLY_DRIVER_MODULE(plic, simplebus, plic_driver, plic_devclass,
    0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
