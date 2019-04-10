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

/*
 * DP83867IR/CR Robust, High Immunity 10/100/1000
 * Ethernet Physical Layer Transceiver
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/errno.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/malloc.h>

#include <machine/bus.h>

#include <net/if.h>
#include <net/if_media.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include "miidevs.h"

#include "miibus_if.h"

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#if 0
#define	MII_KSZPHY_EXTREG			0x0b
#define	 KSZPHY_EXTREG_WRITE			(1 << 15)
#define	MII_KSZPHY_EXTREG_WRITE			0x0c
#define	MII_KSZPHY_EXTREG_READ			0x0d
#define	MII_KSZPHY_CLK_CONTROL_PAD_SKEW		0x104
#define	MII_KSZPHY_RX_DATA_PAD_SKEW		0x105
#define	MII_KSZPHY_TX_DATA_PAD_SKEW		0x106
/* KSZ9031 */
#define	MII_KSZ9031_MMD_ACCESS_CTRL		0x0d
#define	MII_KSZ9031_MMD_ACCESS_DATA		0x0e
#define	 MII_KSZ9031_MMD_DATA_NOINC		(1 << 14)
#define	MII_KSZ9031_CONTROL_PAD_SKEW		0x4
#define	MII_KSZ9031_RX_DATA_PAD_SKEW		0x5
#define	MII_KSZ9031_TX_DATA_PAD_SKEW		0x6
#define	MII_KSZ9031_CLOCK_PAD_SKEW		0x8

#define	MII_KSZ8081_PHYCTL2			0x1f

#define	PS_TO_REG(p)	((p) / 200)
#else

/* TI DP83867 */
#define DP83867_DEVADDR         0x1f 
        
#define MII_DP83867_PHYCTRL     0x10
#define MII_DP83867_MICR        0x12
#define MII_DP83867_CFG2        0x14
#define MII_DP83867_BISCR       0x16
#define DP83867_CTRL            0x1f
        
/* Extended Registers */
#define DP83867_CFG4            0x0031
#define DP83867_RGMIICTL        0x0032
#define DP83867_STRAP_STS1      0x006E
#define DP83867_RGMIIDCTL       0x0086
#define DP83867_IO_MUX_CFG      0x0170
                
#define DP83867_SW_RESET        (1 << 15)
#define DP83867_SW_RESTART      (1 << 14)

/* CFG2 bits */
#define MII_DP83867_CFG2_SPEEDOPT_10EN          0x0040
#define MII_DP83867_CFG2_SGMII_AUTONEGEN        0x0080
#define MII_DP83867_CFG2_SPEEDOPT_ENH           0x0100
#define MII_DP83867_CFG2_SPEEDOPT_CNT           0x0800
#define MII_DP83867_CFG2_SPEEDOPT_INTLOW        0x2000
#define MII_DP83867_CFG2_MASK                   0x003F

/* PHY CTRL bits */
#define DP83867_PHYCR_FIFO_DEPTH_SHIFT          14
#define DP83867_PHYCR_RESERVED_MASK     BIT(11)
#define DP83867_MDI_CROSSOVER           5
#define DP83867_MDI_CROSSOVER_AUTO      2
#define DP83867_MDI_CROSSOVER_MDIX      2
#define DP83867_PHYCTRL_SGMIIEN                 0x0800
#define DP83867_PHYCTRL_RXFIFO_SHIFT    12
#define DP83867_PHYCTRL_TXFIFO_SHIFT    14
                          
/* RGMIIDCTL bits */
#define DP83867_RGMII_TX_CLK_DELAY_SHIFT        4

#endif

#define	TI_REGCR		0xd	/* Register Control Register */
#define	 REGCR_FUNC_S		14	/* Function */
#define	 REGCR_FUNC_ADDR	0x0 /* Address */
#define	 REGCR_FUNC_DATA	0x1 /* Data, no post increment */
#define	TI_ADDAR		0xe	/* Address or Data Register */

static int tiphy_probe(device_t);
static int tiphy_attach(device_t);
static void tiphy_reset(struct mii_softc *);
static int tiphy_service(struct mii_softc *, struct mii_data *, int);

static device_method_t tiphy_methods[] = {
	/* device interface */
	DEVMETHOD(device_probe,		tiphy_probe),
	DEVMETHOD(device_attach,	tiphy_attach),
	DEVMETHOD(device_detach,	mii_phy_detach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),
	DEVMETHOD_END
};

static devclass_t tiphy_devclass;

static driver_t tiphy_driver = {
	"tiphy",
	tiphy_methods,
	sizeof(struct mii_softc)
};

DRIVER_MODULE(tiphy, miibus, tiphy_driver, tiphy_devclass, 0, 0);

static const struct mii_phydesc tiphys[] = {
	MII_PHY_DESC(TI1, DP83867),
	MII_PHY_END
};

static const struct mii_phy_funcs tiphy_funcs = {
	tiphy_service,
	ukphy_status,
	tiphy_reset
};

static uint32_t __unused
ti_read(struct mii_softc *sc, uint32_t devaddr, uint32_t reg)
{
	uint32_t val;

	PHY_WRITE(sc, TI_REGCR, devaddr);
	PHY_WRITE(sc, TI_ADDAR, reg);
	PHY_WRITE(sc, TI_REGCR, devaddr | REGCR_FUNC_DATA);

	val = PHY_READ(sc, TI_ADDAR);

	return (val);
}

static void __unused
ti_write(struct mii_softc *sc, uint32_t devaddr, uint32_t reg,
	uint32_t val)
{

	PHY_WRITE(sc, TI_REGCR, devaddr);
	PHY_WRITE(sc, TI_ADDAR, reg);
	PHY_WRITE(sc, TI_REGCR, devaddr | REGCR_FUNC_DATA);
	PHY_WRITE(sc, TI_ADDAR, val);
}

#if 0
static void
ksz90x1_load_values(struct mii_softc *sc, phandle_t node,
    uint32_t dev, uint32_t reg, char *field1, uint32_t f1mask, int f1off,
    char *field2, uint32_t f2mask, int f2off, char *field3, uint32_t f3mask,
    int f3off, char *field4, uint32_t f4mask, int f4off)
{
	pcell_t dts_value[1];
	int len;
	int val;

	val = ti_read(sc, dev, reg);

	if ((len = OF_getproplen(node, field1)) > 0) {
		OF_getencprop(node, field1, dts_value, len);
		val &= ~(f1mask << f1off);
		val |= (PS_TO_REG(dts_value[0]) & f1mask) << f1off;
	}

	if (field2 != NULL && (len = OF_getproplen(node, field2)) > 0) {
		OF_getencprop(node, field2, dts_value, len);
		val &= ~(f2mask << f2off);
		val |= (PS_TO_REG(dts_value[0]) & f2mask) << f2off;
	}

	if (field3 != NULL && (len = OF_getproplen(node, field3)) > 0) {
		OF_getencprop(node, field3, dts_value, len);
		val &= ~(f3mask << f3off);
		val |= (PS_TO_REG(dts_value[0]) & f3mask) << f3off;
	}

	if (field4 != NULL && (len = OF_getproplen(node, field4)) > 0) {
		OF_getencprop(node, field4, dts_value, len);
		val &= ~(f4mask << f4off);
		val |= (PS_TO_REG(dts_value[0]) & f4mask) << f4off;
	}

	ti_write(sc, dev, reg, val);
}
#endif

static void
ti_load_values(struct mii_softc *sc, phandle_t node)
{

#if 0
	ksz90x1_load_values(sc, node, 2, MII_KSZ9031_CONTROL_PAD_SKEW,
	    "txen-skew-ps", 0xf, 0, "rxdv-skew-ps", 0xf, 4,
	    NULL, 0, 0, NULL, 0, 0);
#endif
}

static int
tiphy_probe(device_t dev)
{

	return (mii_phy_dev_probe(dev, tiphys, BUS_PROBE_DEFAULT));
}

static int
tiphy_attach(device_t dev)
{
	struct mii_softc *sc;
	phandle_t node;
	device_t miibus;
	device_t parent;

	sc = device_get_softc(dev);

	mii_phy_dev_attach(dev, MIIF_NOMANPAUSE, &tiphy_funcs, 1);
	mii_phy_setmedia(sc);

	/* Nothing further to configure for 8081 model. */
	if (sc->mii_mpd_model == MII_MODEL_MICREL_KSZ8081)
		return (0);

	miibus = device_get_parent(dev);
	parent = device_get_parent(miibus);

	if ((node = ofw_bus_get_node(parent)) == -1)
		return (ENXIO);

	ti_load_values(sc, node);

	return (0);
}

static void
tiphy_reset(struct mii_softc *sc)
{
	int reg;
	uint32_t cfg2;

	printf("%s\n", __func__);

	//mii_phy_reset(sc);

	reg = PHY_READ(sc, DP83867_CTRL);
	printf("%s: control register %x\n", __func__, reg);

	PHY_WRITE(sc, DP83867_CTRL, reg | DP83867_SW_RESTART);

	reg = PHY_READ(sc, DP83867_CTRL);
	printf("%s: control register %x\n", __func__, reg);
	reg = PHY_READ(sc, DP83867_CTRL);
	printf("%s: control register %x\n", __func__, reg);
	reg = PHY_READ(sc, DP83867_CTRL);
	printf("%s: control register %x\n", __func__, reg);
	reg = PHY_READ(sc, DP83867_CTRL);
	printf("%s: control register %x\n", __func__, reg);

	/* SGMII */
	PHY_WRITE(sc, MII_BMCR,
		(BMCR_AUTOEN | BMCR_FDX | BMCR_S1000));
	cfg2 = PHY_READ(sc, MII_DP83867_CFG2);
	cfg2 &= MII_DP83867_CFG2_MASK;
	cfg2 |= (MII_DP83867_CFG2_SPEEDOPT_10EN |
                         MII_DP83867_CFG2_SGMII_AUTONEGEN |
                         MII_DP83867_CFG2_SPEEDOPT_ENH |
                         MII_DP83867_CFG2_SPEEDOPT_CNT |
                         MII_DP83867_CFG2_SPEEDOPT_INTLOW);
	PHY_WRITE(sc, MII_DP83867_CFG2, cfg2);
	ti_write(sc, DP83867_DEVADDR, DP83867_RGMIICTL, 0);

	PHY_WRITE(sc, MII_DP83867_PHYCTRL,
		DP83867_PHYCTRL_SGMIIEN |
		(DP83867_MDI_CROSSOVER_MDIX <<
		DP83867_MDI_CROSSOVER) |
		(1 << DP83867_PHYCTRL_RXFIFO_SHIFT) |
		(1 << DP83867_PHYCTRL_TXFIFO_SHIFT));

	PHY_WRITE(sc, MII_DP83867_BISCR, 0);

	//mii_phy_auto(sc);
	mii_phy_reset(sc);
#if 0
	/*
	 * The 8081 has no "sticky bits" that survive a soft reset; several bits
	 * in the Phy Control Register 2 must be preserved across the reset.
	 * These bits are set up by the bootloader; they control how the phy
	 * interfaces to the board (such as clock frequency and LED behavior).
	 */
	if (sc->mii_mpd_model == MII_MODEL_MICREL_KSZ8081)
		reg = PHY_READ(sc, MII_KSZ8081_PHYCTL2);
	mii_phy_reset(sc);
	if (sc->mii_mpd_model == MII_MODEL_MICREL_KSZ8081)
		PHY_WRITE(sc, MII_KSZ8081_PHYCTL2, reg);
#endif
}

static int
tiphy_service(struct mii_softc *sc, struct mii_data *mii, int cmd)
{

	switch (cmd) {
	case MII_POLLSTAT:
		break;

	case MII_MEDIACHG:
		mii_phy_setmedia(sc);
		break;

	case MII_TICK:
		if (mii_phy_tick(sc) == EJUSTRETURN)
			return (0);
		break;
	}

	/* Update the media status. */
	PHY_STATUS(sc);

	/* Callback if something changed. */
	mii_phy_update(sc, cmd);
	return (0);
}
