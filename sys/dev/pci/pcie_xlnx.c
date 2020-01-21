/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 Ruslan Bukin <br@bsdpad.com>
 *
 * This software was developed by SRI International and the University of
 * Cambridge Computer Laboratory (Department of Computer Science and
 * Technology) under DARPA contract HR0011-18-C-0016 ("ECATS"), as part of the
 * DARPA SSITH research programme.
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

#include "opt_platform.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/types.h>
#include <sys/sysctl.h>
#include <sys/kernel.h>
#include <sys/rman.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/cpuset.h>
#include <sys/mutex.h>
#include <sys/proc.h>

#include <machine/intr.h>
#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pci_host_generic.h>
#include <dev/pci/pci_host_generic_fdt.h>
#include <dev/pci/pcib_private.h>

#include "pcie_xlnx.h"

#include "ofw_bus_if.h"
#include "msi_if.h"
#include "pcib_if.h"
#include "pic_if.h"

/* Assembling ECAM Configuration Address */
#define	PCIE_BUS_SHIFT		20
#define	PCIE_SLOT_SHIFT		15
#define	PCIE_FUNC_SHIFT		12
#define	PCIE_BUS_MASK		0xFF
#define	PCIE_SLOT_MASK		0x1F
#define	PCIE_FUNC_MASK		0x07
#define	PCIE_REG_MASK		0xFFF

#define	PCIE_ADDR_OFFSET(bus, slot, func, reg)			\
	((((bus) & PCIE_BUS_MASK) << PCIE_BUS_SHIFT)	|	\
	(((slot) & PCIE_SLOT_MASK) << PCIE_SLOT_SHIFT)	|	\
	(((func) & PCIE_FUNC_MASK) << PCIE_FUNC_SHIFT)	|	\
	((reg) & PCIE_REG_MASK))

static int xlnx_pcie_fdt_attach(device_t);
static int xlnx_pcie_fdt_probe(device_t);
static int xlnx_pcie_fdt_get_id(device_t, device_t, enum pci_id_type,
    uintptr_t *);

#define	XLNX_PCIB_MAX_MSI	64

static struct resource_spec xlnx_pcie_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE | RF_SHAREABLE},
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		1,	RF_ACTIVE },
	{ SYS_RES_IRQ,		2,	RF_ACTIVE },
	{ -1, 0 }
};

struct xlnx_pcie_irqsrc {
	struct intr_irqsrc	isrc;
	u_int			irq;
#define	XLNX_IRQ_FLAG_USED	(1 << 0)
	u_int			flags;
};

static void
xlnx_pcie_intr(void *arg)
{ 
	struct generic_pcie_fdt_softc *fdt_sc;
	struct generic_pcie_core_softc *sc;
	uint32_t val, mask, status;

	fdt_sc = arg;
	sc = &fdt_sc->base;

	val = bus_read_4(sc->res, XLNX_PCIE_IDR);
	mask = bus_read_4(sc->res, XLNX_PCIE_IMR);

	printf("%s: val %x mask %x\n", __func__, val, mask);

	status = val & mask;
	if (!status)
		printf("%s: stray interrupt\n", __func__);

	printf("%s: status %x\n", __func__, status);

	bus_write_4(sc->res, XLNX_PCIE_IDR, val);
}

static void
xlnx_pcie_handle_intr(void *arg, int msireg)
{ 
	struct generic_pcie_fdt_softc *fdt_sc;
	struct generic_pcie_core_softc *sc;
	struct xlnx_pcie_irqsrc *xi;
	struct trapframe *tf;
	int irq;
	int reg;
	int i;

	fdt_sc = arg;
	sc = &fdt_sc->base;
	tf = curthread->td_intr_frame;

	do {
		reg = bus_read_4(sc->res, msireg);

		printf("%s: status %x\n", __func__, reg);

		for (i = 0; i < 32; i++) {
			if (reg & (1 << i)) {
				bus_write_4(sc->res, msireg, (1 << i));

				irq = i;
				if (msireg == XLNX_PCIE_RPMSIID2)
					irq += 32;

				printf("%s: irq %d\n", __func__, irq);

				xi = &fdt_sc->isrcs[irq];
				if (intr_isrc_dispatch(&xi->isrc, tf) != 0) {
					/* Disable stray. */
					//xlnx_pcie_isrc_mask(sc, xi, 0);
					device_printf(sc->dev,
					    "Stray irq %u disabled\n", irq);
        	                }
			}
		}
	} while (reg != 0);
}

static void
xlnx_pcie_msi0_intr(void *arg)
{ 

	printf("%s\n", __func__);

	xlnx_pcie_handle_intr(arg, XLNX_PCIE_RPMSIID1);
}

static void
xlnx_pcie_msi1_intr(void *arg)
{ 

	printf("%s\n", __func__);

	xlnx_pcie_handle_intr(arg, XLNX_PCIE_RPMSIID2);
}

static int
xlnx_pcie_fdt_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_is_compatible(dev, "xlnx,xdma-host-3.00")) {
		device_set_desc(dev, "Xilinx PCI/PCI-E Controller");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
xlnx_pcie_fdt_attach(device_t dev)
{
	struct generic_pcie_fdt_softc *sc;
	int error;

	sc = device_get_softc(dev);
	sc->base.coherent = 1;
	sc->dev = dev;

	mtx_init(&sc->mtx, "msi_mtx", NULL, MTX_DEF);

	if (bus_alloc_resources(dev, xlnx_pcie_spec, sc->res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	/* Setup MISC interrupt handler. */
	error = bus_setup_intr(dev, sc->res[1], INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, xlnx_pcie_intr, sc, &sc->intr_cookie[0]);
	if (error != 0) {
		device_printf(dev, "could not setup interrupt handler.\n");
		return (ENXIO);
	}

	/* Setup MSI0 interrupt handler. */
	error = bus_setup_intr(dev, sc->res[2], INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, xlnx_pcie_msi0_intr, sc, &sc->intr_cookie[1]);
	if (error != 0) {
		device_printf(dev, "could not setup interrupt handler.\n");
		return (ENXIO);
	}

	/* Setup MSI1 interrupt handler. */
	error = bus_setup_intr(dev, sc->res[3], INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, xlnx_pcie_msi1_intr, sc, &sc->intr_cookie[2]);
	if (error != 0) {
		device_printf(dev, "could not setup interrupt handler.\n");
		return (ENXIO);
	}

#if 1
	int reg;

	/* Disable interrupts */
	bus_write_4(sc->res[0], XLNX_PCIE_IMR, 0);

	/* Clear pending */
	reg = bus_read_4(sc->res[0], XLNX_PCIE_IDR);
	printf("idr %x\n", reg);
	reg &= 0x1FF30FED;
	bus_write_4(sc->res[0], XLNX_PCIE_IDR, reg);
	reg = bus_read_4(sc->res[0], XLNX_PCIE_IDR);
	printf("idr %x\n", reg);


	bus_write_4(sc->res[0], XLNX_PCIE_IMR, 0x1FF30FED);
	bus_write_4(sc->res[0], XLNX_PCIE_RPID2_MASK, (0xf << 16));

	/* MSI */
	bus_write_4(sc->res[0], 0x178, 0xffffffff);
	bus_write_4(sc->res[0], 0x17C, 0xffffffff);

	reg = bus_read_4(sc->res[0], XLNX_PCIE_RPSCR);
	printf("old RPSCR %x\n", reg);
	reg |= RPSCR_BE;
	bus_write_4(sc->res[0], XLNX_PCIE_RPSCR, reg);

	reg = bus_read_4(sc->res[0], XLNX_PCIE_RPSCR);
	printf("new RPSCR %x\n", reg);

	reg = bus_read_4(sc->res[0], XLNX_PCIE_BIR);
	printf("BIR %x\n", reg);
	reg = bus_read_4(sc->res[0], XLNX_PCIE_VSEC);
	printf("VSEC %x\n", reg);

	bus_addr_t addr;

	sc->msi_page = kmem_alloc_contig(PAGE_SIZE, M_WAITOK, 0,
	    BUS_SPACE_MAXADDR, PAGE_SIZE, 0, VM_MEMATTR_DEFAULT);
	addr = vtophys(sc->msi_page);
	bus_write_4(sc->res[0], XLNX_PCIE_RPMSIBR1, (addr >> 32));
	bus_write_4(sc->res[0], XLNX_PCIE_RPMSIBR2, (addr >>  0));

	bus_release_resources(dev, xlnx_pcie_spec, sc->res);
#endif

	const char *name;
	int irq;

	sc->isrcs = malloc(sizeof(*sc->isrcs) * XLNX_PCIB_MAX_MSI, M_DEVBUF,
	    M_WAITOK | M_ZERO);

	name = device_get_nameunit(sc->dev);

	for (irq = 0; irq < XLNX_PCIB_MAX_MSI; irq++) {
		sc->isrcs[irq].irq = irq;
		error = intr_isrc_register(&sc->isrcs[irq].isrc,
		    sc->dev, 0, "%s,%u", name, irq);
		if (error != 0)
			return (error); /* XXX deregister ISRCs */
	}

	if (intr_msi_register(sc->dev,
	    OF_xref_from_node(ofw_bus_get_node(sc->dev))) != 0)
		return (ENXIO);

	return (pci_host_generic_attach(dev));
}

static int
xlnx_pcie_fdt_get_id(device_t pci, device_t child, enum pci_id_type type,
    uintptr_t *id)
{
	phandle_t node;
	int bsf;

	printf("%s\n", __func__);

	if (type != PCI_ID_MSI)
		return (pcib_get_id(pci, child, type, id));

	node = ofw_bus_get_node(pci);
	if (OF_hasprop(node, "msi-map"))
		return (generic_pcie_get_id(pci, child, type, id));

	bsf = pci_get_rid(child);
	*id = (pci_get_domain(child) << PCI_RID_DOMAIN_SHIFT) | bsf;

	return (0);
}

static uint32_t
xlnx_pcie_read_config(device_t dev, u_int bus, u_int slot,
    u_int func, u_int reg, int bytes)
{
	struct generic_pcie_fdt_softc *fdt_sc;
	struct generic_pcie_core_softc *sc;
	bus_space_handle_t h;
	bus_space_tag_t	t;
	uint64_t offset;
	uint32_t data;

	fdt_sc = device_get_softc(dev);
	sc = &fdt_sc->base;

	device_printf(dev, "%s: %d/%d/%d reg %x bytes %d\n",
	    __func__, bus, slot, func, reg, bytes);

	if ((bus < sc->bus_start) || (bus > sc->bus_end)) {
		printf("failed 1\n");
		return (~0U);
	}

	if ((slot > PCI_SLOTMAX) || (func > PCI_FUNCMAX) ||
	    (reg > PCIE_REGMAX)) {
		printf("failed 2\n");
		return (~0U);
	}

	if (bus == 0 && slot > 0) {
		printf("failed to read %d/%d/%d\n", bus, slot, func);
		return (~0U);
	}

	//printf("%s: bus %d bus_start %d\n", __func__, bus, sc->bus_start);
	offset = PCIE_ADDR_OFFSET(bus - sc->bus_start, slot, func, reg);
	t = sc->bst;
	h = sc->bsh;

	uint32_t v;
	v = bus_space_read_4(t, h, XLNX_PCIE_PHYSCR);
	if ((v & PHYSCR_LINK_UP) == 0) {
		printf("%s: link down\n", __func__);
		return (~0U);
	}

	data = 0;

	switch (bytes) {
	case 1:
		data = bus_space_read_1(t, h, offset);
		data &= 0x000000ff;
		printf("data %x\n", data);
		break;
	case 2:
		data = le16toh(bus_space_read_2(t, h, offset));
		data &= 0x0000ffff;
		break;
	case 4:
		data = le32toh(bus_space_read_4(t, h, offset));
		break;
	default:
		return (~0U);
	}

	device_printf(dev, "%s: %d/%d/%d reg %x bytes %d, offset %x, val %x\n",
	    __func__, bus, slot, func, reg, bytes, offset, data);

	return (data);
}

static void
xlnx_pcie_write_config(device_t dev, u_int bus, u_int slot,
    u_int func, u_int reg, uint32_t val, int bytes)
{
	struct generic_pcie_fdt_softc *fdt_sc;
	struct generic_pcie_core_softc *sc;
	bus_space_handle_t h;
	bus_space_tag_t t;
	uint64_t offset;

	fdt_sc = device_get_softc(dev);
	sc = &fdt_sc->base;

	if ((bus < sc->bus_start) || (bus > sc->bus_end))
		return;
	if ((slot > PCI_SLOTMAX) || (func > PCI_FUNCMAX) ||
	    (reg > PCIE_REGMAX))
		return;

	if (bus == 0 && slot > 0) {
		printf("failed to write %d/%d/%d\n", bus, slot, func);
		return;
	}

	//printf("%s: bus %d bus_start %d\n", __func__, bus, sc->bus_start);
	offset = PCIE_ADDR_OFFSET(bus - sc->bus_start, slot, func, reg);

	t = sc->bst;
	h = sc->bsh;

	uint32_t v;
	v = bus_space_read_4(t, h, XLNX_PCIE_PHYSCR);
	if ((v & PHYSCR_LINK_UP) == 0) {
		printf("%s: link down\n", __func__);
		return;
	}

	switch (bytes) {
	case 1:
		bus_space_write_1(t, h, offset, val & 0xff);
		break;
	case 2:
		bus_space_write_2(t, h, offset, htole16(val) & 0xffff);
		break;
	case 4:
		bus_space_write_4(t, h, offset, htole32(val));
		break;
	default:
		return;
	}

	device_printf(dev, "%s: %d/%d/%d reg %x bytes %d, offset %x, val %x\n",
	    __func__, bus, slot, func, reg, bytes, offset, val);
}

static int
xlnx_pcie_alloc_msi(device_t pci, device_t child, int count, int maxcount,
    int *irqs)
{
	phandle_t msi_parent;

	printf("%s: count %d\n", __func__, count);

	ofw_bus_msimap(ofw_bus_get_node(pci), pci_get_rid(child), &msi_parent,
	    NULL);
	msi_parent = OF_xref_from_node(ofw_bus_get_node(pci));
	return (intr_alloc_msi(pci, child, msi_parent, count, maxcount,
	    irqs));
}

static int
xlnx_pcie_release_msi(device_t pci, device_t child, int count, int *irqs)
{
	phandle_t msi_parent;

	printf("%s: count %d\n", __func__, count);

	ofw_bus_msimap(ofw_bus_get_node(pci), pci_get_rid(child), &msi_parent,
	    NULL);
	msi_parent = OF_xref_from_node(ofw_bus_get_node(pci));
	return (intr_release_msi(pci, child, msi_parent, count, irqs));
}

static int
xlnx_pcie_map_msi(device_t pci, device_t child, int irq, uint64_t *addr,
    uint32_t *data)
{
	phandle_t msi_parent;

	printf("%s: irq %d\n", __func__, irq);

	ofw_bus_msimap(ofw_bus_get_node(pci), pci_get_rid(child), &msi_parent,
	    NULL);
	msi_parent = OF_xref_from_node(ofw_bus_get_node(pci));
	return (intr_map_msi(pci, child, msi_parent, irq, addr, data));
}

static int
xlnx_pcie_msi_alloc_msi(device_t dev, device_t child, int count, int maxcount,
    device_t *pic, struct intr_irqsrc **srcs)
{
	struct generic_pcie_fdt_softc *sc;
	int irq, end_irq, i;
	bool found;

	printf("%s: count %d\n", __func__, count);

	sc = device_get_softc(dev);

	mtx_lock(&sc->mtx);

	found = false;

	for (irq = 0; (irq + count - 1) < XLNX_PCIB_MAX_MSI; irq++) {

		/* Assume the range is valid. */
		found = true;

		/* Check this range is valid. */
		for (end_irq = irq; end_irq < irq + count; end_irq++) {
			if (sc->isrcs[end_irq].flags & XLNX_IRQ_FLAG_USED) {
				/* This is already used. */
				found = false;
				break;
			}
		}

		if (found)
			break;
	}

	if (!found || irq == (XLNX_PCIB_MAX_MSI - 1)) {
		/* Not enough interrupts were found. */
		mtx_unlock(&sc->mtx);
		return (ENXIO);
	}

	/* Mark the interrupt as used. */
	for (i = 0; i < count; i++)
		sc->isrcs[irq + i].flags |= XLNX_IRQ_FLAG_USED;

	mtx_unlock(&sc->mtx);

	for (i = 0; i < count; i++) 
		srcs[i] = (struct intr_irqsrc *)&sc->isrcs[irq + i];

	*pic = device_get_parent(dev);

	return (0);
}

static int
xlnx_pcie_msi_release_msi(device_t dev, device_t child, int count,
    struct intr_irqsrc **isrc)
{
	struct generic_pcie_fdt_softc *sc;
	struct xlnx_pcie_irqsrc *xi;
	int i;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
	for (i = 0; i < count; i++) {
		xi = (struct xlnx_pcie_irqsrc *)isrc[i];

		KASSERT(xi->flags & XLNX_IRQ_FLAG_USED,
		    ("%s: Releasing an unused MSI interrupt", __func__));

		xi->flags &= ~XLNX_IRQ_FLAG_USED;
	}

	mtx_unlock(&sc->mtx);
	return (0);
}

static int
xlnx_pcie_msi_map_msi(device_t dev, device_t child, struct intr_irqsrc *isrc,
    uint64_t *addr, uint32_t *data)
{
	struct generic_pcie_fdt_softc *sc;
	struct xlnx_pcie_irqsrc *xi;

	printf("%s\n", __func__);

	sc = device_get_softc(dev);
	xi = (struct xlnx_pcie_irqsrc *)isrc;

	*addr = vtophys(sc->msi_page);
	*data = xi->irq;

	return (0);
}

static void
xlnx_pcie_msi_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{

	printf("%s\n", __func__);
}
        
static void
xlnx_pcie_msi_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{

	printf("%s\n", __func__);
}
        
static void
xlnx_pcie_msi_post_filter(device_t dev, struct intr_irqsrc *isrc)
{

	printf("%s\n", __func__);
} 

static void
xlnx_pcie_msi_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{

	printf("%s\n", __func__);
}

static void
xlnx_pcie_msi_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{

	printf("%s\n", __func__);
}

static int
xlnx_pcie_msi_setup_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{

	printf("%s\n", __func__);

	return (0);
}

static int
xlnx_pcie_msi_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{

	printf("%s\n", __func__);

	return (0);
}

static device_method_t xlnx_pcie_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		xlnx_pcie_fdt_probe),
	DEVMETHOD(device_attach,	xlnx_pcie_fdt_attach),

	/* pcib interface */
	DEVMETHOD(pcib_get_id,		xlnx_pcie_fdt_get_id),
	DEVMETHOD(pcib_read_config,	xlnx_pcie_read_config),
	DEVMETHOD(pcib_write_config,	xlnx_pcie_write_config),
	DEVMETHOD(pcib_alloc_msi,	xlnx_pcie_alloc_msi),
	DEVMETHOD(pcib_release_msi,	xlnx_pcie_release_msi),
	DEVMETHOD(pcib_map_msi,		xlnx_pcie_map_msi),

	/* MSI interface */
	DEVMETHOD(msi_alloc_msi,		xlnx_pcie_msi_alloc_msi),
	DEVMETHOD(msi_release_msi,		xlnx_pcie_msi_release_msi),
	DEVMETHOD(msi_map_msi,			xlnx_pcie_msi_map_msi),

	/* Interrupt controller interface */
	DEVMETHOD(pic_disable_intr,		xlnx_pcie_msi_disable_intr),
	DEVMETHOD(pic_enable_intr,		xlnx_pcie_msi_enable_intr),
	DEVMETHOD(pic_setup_intr,		xlnx_pcie_msi_setup_intr),
	DEVMETHOD(pic_teardown_intr,		xlnx_pcie_msi_teardown_intr),
	DEVMETHOD(pic_post_filter,		xlnx_pcie_msi_post_filter),
	DEVMETHOD(pic_post_ithread,		xlnx_pcie_msi_post_ithread),
	DEVMETHOD(pic_pre_ithread,		xlnx_pcie_msi_pre_ithread),

	/* End */
	DEVMETHOD_END
};

DEFINE_CLASS_1(pcib, xlnx_pcie_fdt_driver, xlnx_pcie_fdt_methods,
    sizeof(struct generic_pcie_fdt_softc), generic_pcie_fdt_driver);

static devclass_t xlnx_pcie_fdt_devclass;

DRIVER_MODULE(xlnx_pcib, simplebus, xlnx_pcie_fdt_driver,
    xlnx_pcie_fdt_devclass, 0, 0);
DRIVER_MODULE(xlnx_pcib, ofwbus, xlnx_pcie_fdt_driver,
    xlnx_pcie_fdt_devclass, 0, 0);
