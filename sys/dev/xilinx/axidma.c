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

/* Xilinx AXI DMA driver. */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_platform.h"
#include <sys/param.h>
#include <sys/endian.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/sglist.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/rwlock.h>
#include <sys/sx.h>

#include <machine/bus.h>
//#include <machine/fdt.h>
//#include <machine/cache.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_pager.h>

#ifdef FDT
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

#include <dev/xdma/xdma.h>
#include "xdma_if.h"

#include <dev/xilinx/axidma.h>

#define AXIDMA_DEBUG
//#undef AXIDMA_DEBUG

#ifdef AXIDMA_DEBUG
#define dprintf(fmt, ...)  printf(fmt, ##__VA_ARGS__)
#else
#define dprintf(fmt, ...)
#endif

#define	AXIDMA_NCHANNELS	2

extern struct bus_space memmap_bus;

struct axidma_fdt_data {
	int id;
};

struct axidma_channel {
	struct axidma_softc	*sc;
	struct mtx		mtx;
	xdma_channel_t		*xchan;
	struct proc		*p;
	int			used;
	int			index;
	int			idx_head;
	int			idx_tail;

	struct axidma_desc	**descs;
	bus_dma_segment_t	*descs_phys;
	uint32_t		descs_num;
	bus_dma_tag_t		dma_tag;
	bus_dmamap_t		*dma_map;
	uint32_t		map_descr;
	uint8_t			map_err;
	uint32_t		descs_used_count;
};

struct axidma_softc {
	device_t		dev;
	struct resource		*res[3];
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	void			*ih;
	void			*ih2;
	struct axidma_desc	desc;
	struct axidma_channel	channels[AXIDMA_NCHANNELS];
};

static struct resource_spec axidma_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		1,	RF_ACTIVE },
	{ -1, 0 }
};

#define	HWTYPE_NONE	0
#define	HWTYPE_STD	1

static struct ofw_compat_data compat_data[] = {
	{ "xlnx,eth-dma",	HWTYPE_STD },
	{ NULL,			HWTYPE_NONE },
};

static int axidma_probe(device_t dev);
static int axidma_attach(device_t dev);
static int axidma_detach(device_t dev);

static inline uint32_t
axidma_next_desc(struct axidma_channel *chan, uint32_t curidx)
{

	return ((curidx + 1) % chan->descs_num);
}

static void
axidma_intr_rx(void *arg)
{

	printf("%s\n", __func__);
}

static void
axidma_intr_tx(void *arg)
{
	xdma_controller_t *xdma;
	xdma_transfer_status_t status;
	struct axidma_fdt_data *data;
	struct xdma_transfer_status st;
	struct axidma_desc *desc;
	struct axidma_channel *chan;
	struct xdma_channel *xchan;
	struct axidma_softc *sc;
	uint32_t tot_copied;
	int pending;

	printf("%s\n", __func__);

	sc = arg;
	chan = &sc->channels[0];
	xchan = chan->xchan;
	xdma = xchan->xdma;
	data = xdma->data;

	pending = READ4(sc, AXI_DMASR(data->id));
	WRITE4(sc, AXI_DMASR(data->id), pending);

	printf("%s: AXI_DMASR %x\n", __func__,
	    READ4(sc, AXI_DMASR(data->id)));
	printf("%s: AXI_CURDESC %x\n", __func__,
	    READ4(sc, AXI_CURDESC(data->id)));
	printf("%s: AXI_TAILDESC %x\n", __func__,
	    READ4(sc, AXI_TAILDESC(data->id)));

#if 0
	dprintf("%s(%d): status 0x%08x next_descr 0x%08x, control 0x%08x\n",
	    __func__, data->id,
		READ4_DESC(sc, PF_STATUS),
		READ4_DESC(sc, PF_NEXT_LO),
		READ4_DESC(sc, PF_CONTROL));
#endif

	tot_copied = 0;


	while (chan->idx_tail != chan->idx_head) {
		dprintf("%s: idx_tail %d idx_head %d\n", __func__,
		    chan->idx_tail, chan->idx_head);
#if 0
		bus_dmamap_sync(chan->dma_tag, chan->dma_map[chan->idx_tail],
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
#endif

		desc = chan->descs[chan->idx_tail];
		printf("%s: desc%d status %x (transferred %d)\n", __func__,
		    chan->idx_tail, desc->status,
		    (desc->status & BD_STATUS_TRANSFERRED_M));
		if ((desc->status & BD_STATUS_CMPLT) == 0)
			break;
#if 0
		if ((le32toh(desc->control) & BD_CONTROL_OWN) != 0)
			break;
#endif

		tot_copied += desc->control & BD_CONTROL_LEN_M;
		st.error = 0;
		st.transferred = desc->control & BD_CONTROL_LEN_M;
		xchan_seg_done(xchan, &st);

		chan->idx_tail = axidma_next_desc(chan, chan->idx_tail);
		atomic_subtract_int(&chan->descs_used_count, 1);
	}

#if 0
	WRITE4_DESC(sc, PF_STATUS, PF_STATUS_IRQ);
#endif

	/* Finish operation */
	status.error = 0;
	status.transferred = tot_copied;
	xdma_callback(chan->xchan, &status);
}

static int
axidma_reset(struct axidma_softc *sc, int chan_id)
{
	int timeout;

#if 0
	dprintf("%s: read status: %x\n", __func__, READ4(sc, 0x00));
	dprintf("%s: read control: %x\n", __func__, READ4(sc, 0x04));
	dprintf("%s: read 1: %x\n", __func__, READ4(sc, 0x08));
	dprintf("%s: read 2: %x\n", __func__, READ4(sc, 0x0C));
#endif

	WRITE4(sc, AXI_DMACR(chan_id), DMACR_RESET);

	timeout = 100;
	do {
		if ((READ4(sc, AXI_DMACR(chan_id)) & DMACR_RESET) == 0)
			break;
	} while (timeout--);

	dprintf("timeout %d\n", timeout);

	if (timeout == 0)
		return (-1);

	dprintf("%s: read control after reset: %x\n",
	    __func__, READ4(sc, AXI_DMACR(chan_id)));

	return (0);
}

static int
axidma_probe(device_t dev)
{
	int hwtype;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	hwtype = ofw_bus_search_compatible(dev, compat_data)->ocd_data;
	if (hwtype == HWTYPE_NONE)
		return (ENXIO);

	device_set_desc(dev, "Xilinx AXI DMA");

	return (BUS_PROBE_DEFAULT);
}

static int
axidma_attach(device_t dev)
{
	struct axidma_softc *sc;
	phandle_t xref, node;
	int err;

	sc = device_get_softc(dev);
	sc->dev = dev;

	if (bus_alloc_resources(dev, axidma_spec, sc->res)) {
		device_printf(dev, "could not allocate resources for device\n");
		return (ENXIO);
	}

	/* CSR memory interface */
	sc->bst = rman_get_bustag(sc->res[0]);
	sc->bsh = rman_get_bushandle(sc->res[0]);

	/* Setup interrupt handler */
	err = bus_setup_intr(dev, sc->res[1], INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, axidma_intr_tx, sc, &sc->ih);
	if (err) {
		device_printf(dev, "Unable to alloc interrupt resource.\n");
		return (ENXIO);
	}

	/* Setup interrupt handler */
	err = bus_setup_intr(dev, sc->res[2], INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, axidma_intr_rx, sc, &sc->ih2);
	if (err) {
		device_printf(dev, "Unable to alloc interrupt resource.\n");
		return (ENXIO);
	}

	node = ofw_bus_get_node(dev);
	xref = OF_xref_from_node(node);
	OF_device_register_xref(xref, dev);

	if (axidma_reset(sc, 0) != 0)
		return (-1);

#if 0
	WRITE4(sc, DMA_CONTROL, CONTROL_GIEM);
#endif

	return (0);
}

static int
axidma_detach(device_t dev)
{
	struct axidma_softc *sc;

	sc = device_get_softc(dev);

	return (0);
}

static void
axidma_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int err)
{
	struct axidma_channel *chan;

	chan = (struct axidma_channel *)arg;
	KASSERT(chan != NULL, ("xchan is NULL"));

	if (err) {
		chan->map_err = 1;
		return;
	}

	chan->descs_phys[chan->map_descr].ds_addr = segs[0].ds_addr;
	chan->descs_phys[chan->map_descr].ds_len = segs[0].ds_len;

	dprintf("map desc %d: descs phys %lx len %ld\n",
	    chan->map_descr, segs[0].ds_addr, segs[0].ds_len);
}

static int
axidma_desc_free(struct axidma_softc *sc, struct axidma_channel *chan)
{
	struct axidma_desc *desc;
	int nsegments;
	int i;

	nsegments = chan->descs_num;

	for (i = 0; i < nsegments; i++) {
		desc = chan->descs[i];
		bus_dmamap_unload(chan->dma_tag, chan->dma_map[i]);
		bus_dmamem_free(chan->dma_tag, desc, chan->dma_map[i]);
	}

	bus_dma_tag_destroy(chan->dma_tag);
	free(chan->descs, M_DEVBUF);
	free(chan->dma_map, M_DEVBUF);
	free(chan->descs_phys, M_DEVBUF);

	return (0);
}

static int
axidma_desc_alloc(struct axidma_softc *sc, struct xdma_channel *xchan,
    uint32_t desc_size, uint32_t align)
{
	struct axidma_channel *chan;
	int nsegments;
	int err;
	int i;

	chan = (struct axidma_channel *)xchan->chan;

	nsegments = chan->descs_num;

	dprintf("%s: nseg %d, desc_size %d\n",
	    __func__, nsegments, desc_size);

	dprintf("%s: creating tag\n", __func__);
	err = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),
	    align, 0,			/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    desc_size, 1,		/* maxsize, nsegments*/
	    desc_size, 0,		/* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &chan->dma_tag);
	if (err) {
		device_printf(sc->dev,
		    "%s: Can't create bus_dma tag.\n", __func__);
		return (-1);
	}

	dprintf("%s: allocating descriptors\n", __func__);
	/* Descriptors. */
	chan->descs = malloc(nsegments * sizeof(struct axidma_desc *),
	    M_DEVBUF, (M_WAITOK | M_ZERO));
	if (chan->descs == NULL) {
		device_printf(sc->dev,
		    "%s: Can't allocate memory.\n", __func__);
		return (-1);
	}
	dprintf("%s: allocating dma_map\n", __func__);
	chan->dma_map = malloc(nsegments * sizeof(bus_dmamap_t),
	    M_DEVBUF, (M_WAITOK | M_ZERO));
	dprintf("%s: allocating descs_phys\n", __func__);
	chan->descs_phys = malloc(nsegments * sizeof(bus_dma_segment_t),
	    M_DEVBUF, (M_WAITOK | M_ZERO));

	int size;
	vm_offset_t paddr;
	vm_offset_t vaddr;
	size = desc_size * nsegments;
	if (vmem_alloc(xchan->vmem, size, M_FIRSTFIT, &paddr)) {
		printf("failed to alloc memory\n");
	}
	vaddr = kva_alloc(size);
	pmap_kenter_device(vaddr, size, paddr);

	for (i = 0; i < nsegments; i++) {
		chan->descs[i] = (struct axidma_desc *)((uint64_t)vaddr + desc_size * i);
		chan->descs_phys[i].ds_addr = paddr + desc_size * i;
		chan->descs_phys[i].ds_len = desc_size;
	}
	return (0);

	bus_size_t psize;

	psize = desc_size * nsegments;
	bus_space_map(&memmap_bus, 0x80000000, psize, 0, &vaddr);
	printf("%s: vaddr %x\n", __func__, vaddr);
	printf("%s: paddr %x\n", __func__, vtophys(vaddr));

	for (i = 0; i < nsegments; i++) {
		chan->descs[i] = (struct axidma_desc *)((uint64_t)vaddr + desc_size * i);
		chan->descs_phys[i].ds_addr = 0x80000000 + desc_size * i;
		chan->descs_phys[i].ds_len = desc_size;
	}

	return (0);

	/* Allocate bus_dma memory for each descriptor. */
	for (i = 0; i < nsegments; i++) {
		dprintf("%s: bus_dmamem_alloc\n", __func__);
		err = bus_dmamem_alloc(chan->dma_tag, (void **)&chan->descs[i],
		    BUS_DMA_WAITOK | BUS_DMA_ZERO, &chan->dma_map[i]);
		if (err) {
			device_printf(sc->dev,
			    "%s: Can't allocate memory for descriptors.\n",
			    __func__);
			return (-1);
		}

		chan->map_err = 0;
		chan->map_descr = i;
		dprintf("%s: bus_dmamap_load\n", __func__);
		err = bus_dmamap_load(chan->dma_tag, chan->dma_map[i],
		    chan->descs[i], desc_size, axidma_dmamap_cb, chan,
		    BUS_DMA_WAITOK);
		if (err) {
			device_printf(sc->dev,
			    "%s: Can't load DMA map.\n", __func__);
			return (-1);
		}

		if (chan->map_err != 0) {
			device_printf(sc->dev,
			    "%s: Can't load DMA map.\n", __func__);
			return (-1);
		}
	}

	return (0);
}


static int
axidma_channel_alloc(device_t dev, struct xdma_channel *xchan)
{
	struct axidma_channel *chan;
	struct axidma_softc *sc;
	int i;

	sc = device_get_softc(dev);

	for (i = 0; i < AXIDMA_NCHANNELS; i++) {
		chan = &sc->channels[i];
		if (chan->used == 0) {
			chan->xchan = xchan;
			xchan->chan = (void *)chan;
			//xchan->caps |= XCHAN_CAP_BUSDMA;
			chan->index = i;
			chan->sc = sc;
			chan->used = 1;
			chan->idx_head = 0;
			chan->idx_tail = 0;
			chan->descs_used_count = 0;
			chan->descs_num = 128;

			return (0);
		}
	}

	return (-1);
}

static int
axidma_channel_free(device_t dev, struct xdma_channel *xchan)
{
	struct axidma_channel *chan;
	struct axidma_softc *sc;

	sc = device_get_softc(dev);

	chan = (struct axidma_channel *)xchan->chan;

	axidma_desc_free(sc, chan);

	chan->used = 0;

	return (0);
}

static int
axidma_channel_capacity(device_t dev, xdma_channel_t *xchan,
    uint32_t *capacity)
{
	struct axidma_channel *chan;
	uint32_t c;

	chan = (struct axidma_channel *)xchan->chan;

	/* At least one descriptor must be left empty. */
	c = (chan->descs_num - chan->descs_used_count - 1);

	*capacity = c;

	return (0);
}

static int
axidma_channel_submit_sg(device_t dev, struct xdma_channel *xchan,
    struct xdma_sglist *sg, uint32_t sg_n)
{
	xdma_controller_t *xdma;
	struct axidma_fdt_data *data;
	struct axidma_channel *chan;
	struct axidma_desc *desc;
	struct axidma_softc *sc;
	uint32_t src_addr_lo;
	uint32_t dst_addr_lo;
	uint32_t len;
	uint32_t tmp;
	int i;
	int tail;

	printf("%s: sg_n %d\n", __func__, sg_n);

	sc = device_get_softc(dev);

	chan = (struct axidma_channel *)xchan->chan;
	xdma = xchan->xdma;
	data = xdma->data;

	tail = chan->idx_head;

	tmp = 0;
	if (sg_n == 0)
		panic("here");

	for (i = 0; i < sg_n; i++) {
		src_addr_lo = (uint32_t)sg[i].src_addr;
		dst_addr_lo = (uint32_t)sg[i].dst_addr;
		len = (uint32_t)sg[i].len;

		dprintf("%s(%d): src %x dst %x len %d\n", __func__,
		    data->id, src_addr_lo, dst_addr_lo, len);

		desc = chan->descs[chan->idx_head];
		if (sg[i].direction == XDMA_MEM_TO_DEV)
			desc->phys = src_addr_lo;
		else
			desc->phys = dst_addr_lo;
		dprintf("%s(%d): desc->phys %x\n",
		    __func__, data->id, desc->phys);
		desc->status = 0;
		desc->control = len;
		if (sg[i].first == 1)
			desc->control |= BD_CONTROL_TXSOF;
		if (sg[i].last == 1)
			desc->control |= BD_CONTROL_TXEOF;
		dprintf("%s(%d): desc->control %x\n",
		    __func__, data->id, desc->control);

		tmp = chan->idx_head;

		atomic_add_int(&chan->descs_used_count, 1);
		chan->idx_head = axidma_next_desc(chan, chan->idx_head);

#if 0
		desc->control |= (CONTROL_OWN | CONTROL_GO);

		bus_dmamap_sync(chan->dma_tag, chan->dma_map[tmp],
		    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
#endif
	}

	uint32_t addr;

	printf("%s(%d): _curdesc %x\n", __func__, data->id,
	    READ8(sc, AXI_CURDESC(data->id)));
	printf("%s(%d): _curdesc %x\n", __func__, data->id,
	    READ8(sc, AXI_CURDESC(data->id)));
	printf("%s(%d): status %x\n", __func__, data->id,
	    READ4(sc, AXI_DMASR(data->id)));

	sfence_vma();
	addr = chan->descs_phys[tail].ds_addr;
	addr = chan->descs_phys[chan->idx_head].ds_addr;
	addr = chan->descs_phys[tmp].ds_addr;
	WRITE8(sc, AXI_TAILDESC(data->id), addr);

	printf("%s(%d): taildesc %x %x\n", __func__,
	    data->id, addr, READ8(sc, AXI_TAILDESC(data->id)));
	printf("%s(%d): curdesc %x\n", __func__, data->id,
	    READ8(sc, AXI_CURDESC(data->id)));
	printf("%s(%d): status %x\n", __func__,
	    data->id, READ4(sc, AXI_DMASR(data->id)));

	return (0);
}

static int
axidma_channel_prep_sg(device_t dev, struct xdma_channel *xchan)
{
	xdma_controller_t *xdma;
	struct axidma_fdt_data *data;
	struct axidma_channel *chan;
	struct axidma_desc *desc;
	struct axidma_softc *sc;
	uint32_t addr;
	//uint32_t reg;
	int ret;
	int i;

	sc = device_get_softc(dev);

	chan = (struct axidma_channel *)xchan->chan;
	xdma = xchan->xdma;
	data = xdma->data;

	dprintf("%s(%d)\n", __func__, data->id);

	ret = axidma_desc_alloc(sc, xchan, sizeof(struct axidma_desc), 16);
	if (ret != 0) {
		device_printf(sc->dev,
		    "%s: Can't allocate descriptors.\n", __func__);
		return (-1);
	}

	for (i = 0; i < chan->descs_num; i++) {
		desc = chan->descs[i];
		bzero(desc, sizeof(struct axidma_desc));

		if (i == (chan->descs_num - 1))
			desc->next = chan->descs_phys[0].ds_addr;
		else
			desc->next = chan->descs_phys[i+1].ds_addr;
		desc->status = 0;
		desc->control = 0;

		dprintf("%s(%d): desc %d vaddr %lx next paddr %x\n", __func__,
		    data->id, i, (uint64_t)desc, le32toh(desc->next));
	}

	addr = chan->descs_phys[0].ds_addr;
	printf("%s(%d): curdesc %x\n", __func__, data->id, addr);
	WRITE8(sc, AXI_CURDESC(data->id), addr);

	uint32_t reg;
	reg = READ4(sc, AXI_DMACR(data->id));
	reg |= DMACR_IOC_IRQEN | DMACR_DLY_IRQEN | DMACR_ERR_IRQEN;
	WRITE4(sc, AXI_DMACR(data->id), reg);
	reg |= DMACR_RS;
	WRITE4(sc, AXI_DMACR(data->id), reg);

	return (0);
}

static int
axidma_channel_control(device_t dev, xdma_channel_t *xchan, int cmd)
{
	struct axidma_channel *chan;
	struct axidma_softc *sc;

	sc = device_get_softc(dev);

	chan = (struct axidma_channel *)xchan->chan;

	switch (cmd) {
	case XDMA_CMD_BEGIN:
	case XDMA_CMD_TERMINATE:
	case XDMA_CMD_PAUSE:
		/* TODO: implement me */
		return (-1);
	}

	return (0);
}

#ifdef FDT
static int
axidma_ofw_md_data(device_t dev, pcell_t *cells, int ncells, void **ptr)
{
	struct axidma_fdt_data *data;

	if (ncells != 1)
		return (-1);

	data = malloc(sizeof(struct axidma_fdt_data),
	    M_DEVBUF, (M_WAITOK | M_ZERO));
	data->id = cells[0];

	printf("%s: id %d\n", __func__, data->id);

	*ptr = data;

	return (0);
}
#endif

static device_method_t axidma_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			axidma_probe),
	DEVMETHOD(device_attach,		axidma_attach),
	DEVMETHOD(device_detach,		axidma_detach),

	/* xDMA Interface */
	DEVMETHOD(xdma_channel_alloc,		axidma_channel_alloc),
	DEVMETHOD(xdma_channel_free,		axidma_channel_free),
	DEVMETHOD(xdma_channel_control,		axidma_channel_control),

	/* xDMA SG Interface */
	DEVMETHOD(xdma_channel_capacity,	axidma_channel_capacity),
	DEVMETHOD(xdma_channel_prep_sg,		axidma_channel_prep_sg),
	DEVMETHOD(xdma_channel_submit_sg,	axidma_channel_submit_sg),

#ifdef FDT
	DEVMETHOD(xdma_ofw_md_data,		axidma_ofw_md_data),
#endif

	DEVMETHOD_END
};

static driver_t axidma_driver = {
	"axidma",
	axidma_methods,
	sizeof(struct axidma_softc),
};

static devclass_t axidma_devclass;

EARLY_DRIVER_MODULE(axidma, simplebus, axidma_driver, axidma_devclass, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
