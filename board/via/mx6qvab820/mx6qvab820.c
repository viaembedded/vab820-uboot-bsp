/*
 * Copyright (C) 2012-2014 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 * Author: Jason Liu <r64343@freescale.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#if CONFIG_I2C_MXC
#include <i2c.h>
#include <asm/imx-common/mxc_i2c.h>
#endif
#include <mmc.h>
#include <fsl_esdhc.h>
#include <micrel.h> /* steven: ksz90x1 */
#include <miiphy.h>
#include <netdev.h>

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
#include <lcd.h>
#include <mxc_epdc_fb.h>
#endif
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>

#ifdef CONFIG_FASTBOOT
#include <fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FASTBOOT*/

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |               \
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)
/* steven: sabrelite: PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED | */
#define SPI_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)
/* steven */
#if 0
#define EPDC_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_SPEED_MED |	\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS)
#endif /* steven: if 0 */

/* steven */
#ifdef VAB820_MX6Q
 #define PIN_WRAPPER(pinDef) MX6Q_##pinDef
#else
 #define PIN_WRAPPER(pinDef) MX6DL_##pinDef
#endif

#if CONFIG_I2C_MXC
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

enum {
 MX6Q_PAD_CSI0_DAT9__I2C1_SCL		= IOMUX_PAD(0x064C, 0x027C, 20, 0x0898, 1, 0),
 MX6Q_PAD_CSI0_DAT9__GPIO_5_27		= IOMUX_PAD(0x064C, 0x027C, 5, 0x0000, 0, 0),
 MX6Q_PAD_CSI0_DAT8__I2C1_SDA		= IOMUX_PAD(0x0648, 0x0278, 20, 0x089C, 1, 0),
 MX6Q_PAD_CSI0_DAT8__GPIO_5_26		= IOMUX_PAD(0x0648, 0x0278, 5, 0x0000, 0, 0),
 MX6Q_PAD_KEY_COL3__I2C2_SCL		= IOMUX_PAD(0x05E0, 0x0210, 20, 0x08A0, 1, 0),
 MX6Q_PAD_KEY_COL3__GPIO_4_12		= IOMUX_PAD(0x05E0, 0x0210, 5, 0x0000, 0, 0),
 MX6Q_PAD_KEY_ROW3__I2C2_SDA		= IOMUX_PAD(0x05E4, 0x0214, 20, 0x08A4, 1, 0),
 MX6Q_PAD_KEY_ROW3__GPIO_4_13		= IOMUX_PAD(0x05E4, 0x0214, 5, 0x0000, 0, 0),
 MX6Q_PAD_GPIO_3__I2C3_SCL		= IOMUX_PAD(0x05FC, 0x022C, 18, 0x08A8, 1, 0),
 MX6Q_PAD_GPIO_3__GPIO_1_3		= IOMUX_PAD(0x05FC, 0x022C, 5, 0x0000, 0, 0),
 MX6Q_PAD_GPIO_6__I2C3_SDA		= IOMUX_PAD(0x0600, 0x0230, 18, 0x08AC, 1, 0),
 MX6Q_PAD_GPIO_6__GPIO_1_6		= IOMUX_PAD(0x0600, 0x0230, 5, 0x0000, 0, 0),

 MX6DL_PAD_CSI0_DAT9__I2C1_SCL = IOMUX_PAD(0x039C, 0x0088, 20, 0x0868, 0, 0),
 MX6DL_PAD_CSI0_DAT9__GPIO_5_27 = IOMUX_PAD(0x039C, 0x0088, 5, 0x0000, 0, 0),
 MX6DL_PAD_CSI0_DAT8__I2C1_SDA = IOMUX_PAD(0x0398, 0x0084, 20, 0x086C, 0, 0),
 MX6DL_PAD_CSI0_DAT8__GPIO_5_26 = IOMUX_PAD(0x0398, 0x0084, 5, 0x0000, 0, 0),
 MX6DL_PAD_KEY_COL3__I2C2_SCL = IOMUX_PAD(0x0638, 0x0250, 20, 0x0870, 1, 0),
 MX6DL_PAD_KEY_COL3__GPIO_4_12 = IOMUX_PAD(0x0638, 0x0250, 5, 0x0000, 0, 0),
 MX6DL_PAD_KEY_ROW3__I2C2_SDA = IOMUX_PAD(0x064C, 0x0264, 20, 0x0874, 1, 0),
 MX6DL_PAD_KEY_ROW3__GPIO_4_13 = IOMUX_PAD(0x064C, 0x0264, 5, 0x0000, 0, 0),
 MX6DL_PAD_GPIO_3__I2C3_SCL = IOMUX_PAD(0x05F8, 0x0228, 18, 0x0878, 1, 0),
 MX6DL_PAD_GPIO_3__GPIO_1_3 = IOMUX_PAD(0x05F8, 0x0228, 5, 0x0000, 0, 0),
 MX6DL_PAD_GPIO_6__I2C3_SDA = IOMUX_PAD(0x0604, 0x0234, 18, 0x087C, 2, 0),
 MX6DL_PAD_GPIO_6__GPIO_1_6 = IOMUX_PAD(0x0604, 0x0234, 5, 0x0000, 0, 0),
};

/* steven: adv7180=0x42 */
struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_CSI0_DAT9__GPIO_5_27 | PC,
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_CSI0_DAT8__GPIO_5_26 | PC,
		.gp = IMX_GPIO_NR(5, 26)
	}
};

/* steven: hdmi */
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO_4_12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | PC,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO_4_13 | PC,
		.gp = IMX_GPIO_NR(4, 13)
	}
};

/* steven: lvds, pcie, sgtl5000=0x0a */
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO_3__I2C3_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO_3__GPIO_1_3 | PC,
		.gp = IMX_GPIO_NR(1, 3)
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO_6__I2C3_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO_6__GPIO_1_6 | PC,
		.gp = IMX_GPIO_NR(1, 6)
	}
};
#endif

int cpu_is_mx6q(void);
/* steven: choose iomux_cfg by SoC */
#define get_iomux_cfg_by_cpu(pMx6q, pMx6dl) (cpu_is_mx6q() ? (pMx6q) : (pMx6dl))


#define HW_ANADIG_PLL_SYS	(0x00000000)
#define HW_ANADIG_PLL_SYS_SET	(0x00000004)
#define HW_ANADIG_PLL_SYS_CLR	(0x00000008)
#define HW_ANADIG_PLL_528	(0x00000030)
#define HW_ANADIG_PLL_528_SET	(0x00000034)
#define HW_ANADIG_PLL_528_CLR	(0x00000038)

#define REG_RD(base, reg) \
	(*(volatile unsigned int *)((base) + (reg)))
#define REG_WR(base, reg, value) \
	((*(volatile unsigned int *)((base) + (reg))) = (value))
#define REG_SET(base, reg, value) \
	((*(volatile unsigned int *)((base) + (reg ## _SET))) = (value))
#define REG_CLR(base, reg, value) \
	((*(volatile unsigned int *)((base) + (reg ## _CLR))) = (value))

enum {
	CPU_CLK = 0,
	PERIPH_CLK,
	AHB_CLK,
	IPG_CLK,
	IPG_PERCLK,
	UART_CLK,
	CSPI_CLK,
	DDR_CLK,
	NFC_CLK,
	ALL_CLK,
};

enum pll_clocks {
	CPU_PLL1,	/* System PLL */
	BUS_PLL2,	/* System Bus PLL*/
	USBOTG_PLL3,    /* OTG USB PLL */
	AUD_PLL4,	/* Audio PLL */
	VID_PLL5,	/* Video PLL */
#ifndef CONFIG_MX6SL
	MLB_PLL6,	/* MLB PLL */
	USBHOST_PLL7,   /* Host USB PLL */
#endif
	ENET_PLL8,      /* ENET PLL */
};
#define PLL1_FREQ_MAX	1300000000
#define PLL1_FREQ_MIN	650000000
#define PLL2_FREQ_MAX	528000000
#define PLL2_FREQ_MIN	480000000
#define SZ_DEC_1M       1000000
#define CONFIG_MX6_HCLK_FREQ	24000000
#define CONFIG_REF_CLK_FREQ CONFIG_MX6_HCLK_FREQ

/*!
 * This is to calculate divider based on reference clock and
 * targeted clock based on the equation for each PLL.
 *
 * @param pll		pll number
 * @param ref       reference clock freq in Hz
 * @param target    targeted clock in Hz
 *
 * @return          divider if successful; -1 otherwise.
 */
static int calc_pll_divider(enum pll_clocks pll, u32 ref, u32 target)
{
	int i, div;

	switch (pll) {
	case CPU_PLL1:
		if (target < PLL1_FREQ_MIN || target > PLL1_FREQ_MAX) {
			printf("PLL1 frequency should be"
			"within [%d - %d] MHz\n", PLL1_FREQ_MIN / SZ_DEC_1M,
				PLL1_FREQ_MAX / SZ_DEC_1M);
			return -1;
		}
		for (i = 54, div = i; i < 109; i++) {
			if ((ref * (i >> 1)) > target)
				break;
			div = i;
		}
		break;
	case BUS_PLL2:
		if (target < PLL2_FREQ_MIN || target > PLL2_FREQ_MAX) {
			printf("PLL2 frequency should be"
			"within [%d - %d] MHz\n", PLL2_FREQ_MIN / SZ_DEC_1M,
				PLL2_FREQ_MAX / SZ_DEC_1M);
			return -1;
		}
		for (i = 0, div = i; i < 2; i++) {
			if (ref * (20 + (i << 1)) > target)
				break;
			div = i;
		}
		break;
	default:
		printf("Changing this PLL not supported\n");
		return -1;
		break;
	}

	return div;
}

static int config_pll_clk(enum pll_clocks pll, u32 divider)
{
	u32 ccsr = readl(CCM_BASE_ADDR + CLKCTL_CCSR);

	switch (pll) {
	case CPU_PLL1:
		/* Switch ARM to PLL2 clock */
		writel(ccsr | 0x4, CCM_BASE_ADDR + CLKCTL_CCSR);

		REG_CLR(ANATOP_BASE_ADDR, HW_ANADIG_PLL_SYS,
			BM_ANADIG_PLL_SYS_DIV_SELECT);
		REG_SET(ANATOP_BASE_ADDR, HW_ANADIG_PLL_SYS,
			BF_ANADIG_PLL_SYS_DIV_SELECT(divider));
		/* Enable CPU PLL1 */
		REG_SET(ANATOP_BASE_ADDR, HW_ANADIG_PLL_SYS,
			BM_ANADIG_PLL_SYS_ENABLE);
		/* Wait for PLL lock */
		while (REG_RD(ANATOP_BASE_ADDR, HW_ANADIG_PLL_SYS) &
			BM_ANADIG_PLL_SYS_LOCK)
			udelay(10);
		/* Clear bypass bit */
		REG_CLR(ANATOP_BASE_ADDR, HW_ANADIG_PLL_SYS,
			BM_ANADIG_PLL_SYS_BYPASS);

		/* Switch back */
		writel(ccsr & ~0x4, CCM_BASE_ADDR + CLKCTL_CCSR);
		break;
	case BUS_PLL2:
		/* Switch to pll2 bypass clock */
		writel(ccsr | 0x2, CCM_BASE_ADDR + CLKCTL_CCSR);

		REG_CLR(ANATOP_BASE_ADDR, HW_ANADIG_PLL_528,
			BM_ANADIG_PLL_528_DIV_SELECT);
		REG_SET(ANATOP_BASE_ADDR, HW_ANADIG_PLL_528,
			divider);
		/* Enable BUS PLL2 */
		REG_SET(ANATOP_BASE_ADDR, HW_ANADIG_PLL_528,
			BM_ANADIG_PLL_528_ENABLE);
		/* Wait for PLL lock */
		while (REG_RD(ANATOP_BASE_ADDR, HW_ANADIG_PLL_528) &
			BM_ANADIG_PLL_528_LOCK)
			udelay(10);
		/* Clear bypass bit */
		REG_CLR(ANATOP_BASE_ADDR, HW_ANADIG_PLL_528,
			BM_ANADIG_PLL_528_BYPASS);

		/* Switch back */
		writel(ccsr & ~0x2, CCM_BASE_ADDR + CLKCTL_CCSR);
		break;
	default:
		return -1;
	}

	return 0;
}

static int config_core_clk(u32 ref, u32 freq)
{
	int div = calc_pll_divider(CPU_PLL1, ref, freq);
	if (div < 0) {
		printf("Can't find pll parameters\n");
		return div;
	}

	return config_pll_clk(CPU_PLL1, div);
}

/*!
 * This function assumes the expected core clock has to be changed by
 * modifying the PLL. This is NOT true always but for most of the times,
 * it is. So it assumes the PLL output freq is the same as the expected
 * core clock (arm_podf=0) unless the core clock is less than PLL_FREQ_MIN.
 *
 * @param ref       pll input reference clock (24MHz)
 * @param freq		targeted freq in Hz
 * @param clk_type  clock type, e.g CPU_CLK, DDR_CLK, etc.
 * @return          0 if successful; non-zero otherwise
 */
int clk_config(u32 ref, u32 freq, u32 clk_type)
{
	freq *= SZ_DEC_1M;

	switch (clk_type) {
	case CPU_CLK:
		if (config_core_clk(ref, freq))
			return -1;
		break;
/* steven: CPU_CLK only */
/*
	case PERIPH_CLK:
		if (config_periph_clk(ref, freq))
			return -1;
		break;
	case DDR_CLK:
		if (config_ddr_clk(freq))
			return -1;
		break;
	case NFC_CLK:
		if (config_nfc_clk(freq))
			return -1;
		break;
*/
	default:
		printf("Unsupported or invalid clock type! :(\n");
		return -1;
	}

	return 0;
}

int dram_init(void)
{
	/* Steven */
	ulong nRamSize = (1u * 1024 * 1024 * 1024);
	u32 reg = readl(GPIO2_BASE_ADDR + GPIO_PSR);

 switch ( reg&0x03 ) {
  case 1:
   /* steven: 3.75GB */
   nRamSize = (3840u * 1024 * 1024) - 4096;
   /* steven: 3.5GB */
   /* nRamSize = (3584u * 1024 * 1024); */
   break;
  case 2: nRamSize = (2u * 1024 * 1024 * 1024); break;
  default: nRamSize = (1u * 1024 * 1024 * 1024); break;
 }
	gd->ram_size = nRamSize;

	return 0;
}

/* steven */
static void setup_iomux_uart(void)
{
enum {
 MX6Q_PAD_SD3_DAT6__UART1_RXD = IOMUX_PAD(0x0694, 0x02AC, 1, 0x0920, 3, 0),
 MX6Q_PAD_SD3_DAT7__UART1_TXD = IOMUX_PAD(0x0690, 0x02A8, 1, 0x0000, 0, 0),
 MX6Q_PAD_EIM_D26__UART2_TXD  = IOMUX_PAD(0x03D0, 0x00BC, 4, 0x0000, 0, 0),
 MX6Q_PAD_EIM_D27__UART2_RXD  = IOMUX_PAD(0x03D4, 0x00C0, 4, 0x0928, 1, 0),
 MX6DL_PAD_SD3_DAT6__UART1_RXD = IOMUX_PAD(0x0714, 0x032C, 1, 0x08FC, 2, 0),
 MX6DL_PAD_SD3_DAT7__UART1_TXD = IOMUX_PAD(0x0718, 0x0330, 1, 0x0000, 0, 0),
 MX6DL_PAD_EIM_D26__UART2_TXD = IOMUX_PAD(0x053C, 0x016C, 4, 0x0000, 0, 0),
 MX6DL_PAD_EIM_D27__UART2_RXD = IOMUX_PAD(0x0540, 0x0170, 4, 0x0904, 1, 0),
};
iomux_v3_cfg_t const mx6q_pads[] = {
 MX6Q_PAD_SD3_DAT6__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
 MX6Q_PAD_SD3_DAT7__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
 MX6Q_PAD_EIM_D26__UART2_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
 MX6Q_PAD_EIM_D27__UART2_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};
iomux_v3_cfg_t const mx6dl_pads[] = {
 MX6DL_PAD_SD3_DAT6__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
 MX6DL_PAD_SD3_DAT7__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
 MX6DL_PAD_EIM_D26__UART2_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
 MX6DL_PAD_EIM_D27__UART2_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};
 iomux_v3_cfg_t const * p = get_iomux_cfg_by_cpu(mx6q_pads, mx6dl_pads);
	imx_iomux_v3_setup_multiple_pads(p, ARRAY_SIZE(mx6q_pads));
}

enum {
 MX6Q_PAD_ENET_MDIO__ENET_MDIO		= IOMUX_PAD(0x04E4, 0x01D0, 1, 0x0840, 0, 0),
 MX6Q_PAD_ENET_MDC__ENET_MDC		= IOMUX_PAD(0x0508, 0x01F4, 1, 0x0000, 0, 0),
 MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC	= IOMUX_PAD(0x036C, 0x0058, 1, 0x0000, 0, 0),
 MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0	= IOMUX_PAD(0x0370, 0x005C, 1, 0x0000, 0, 0),
 MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1	= IOMUX_PAD(0x0374, 0x0060, 1, 0x0000, 0, 0),
 MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2	= IOMUX_PAD(0x0378, 0x0064, 1, 0x0000, 0, 0),
 MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3	= IOMUX_PAD(0x037C, 0x0068, 1, 0x0000, 0, 0),
 MX6Q_PAD_RGMII_TX_CTL__RGMII_TX_CTL	= IOMUX_PAD(0x0388, 0x0074, 1, 0x0000, 0, 0),
 MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK	= IOMUX_PAD(0x04E8, 0x01D4, 1, 0x0000, 0, 0),
 MX6Q_PAD_RGMII_RXC__GPIO_6_30		= IOMUX_PAD(0x0398, 0x0084, 5, 0x0000, 0, 0),
 MX6Q_PAD_RGMII_RD0__GPIO_6_25		= IOMUX_PAD(0x0384, 0x0070, 5, 0x0000, 0, 0),
 MX6Q_PAD_RGMII_RD1__GPIO_6_27		= IOMUX_PAD(0x038C, 0x0078, 5, 0x0000, 0, 0),
 MX6Q_PAD_RGMII_RD2__GPIO_6_28		= IOMUX_PAD(0x0390, 0x007C, 5, 0x0000, 0, 0),
 MX6Q_PAD_RGMII_RD3__GPIO_6_29		= IOMUX_PAD(0x0394, 0x0080, 5, 0x0000, 0, 0),
 MX6Q_PAD_RGMII_RX_CTL__GPIO_6_24	= IOMUX_PAD(0x0380, 0x006C, 5, 0x0000, 0, 0),
 MX6Q_PAD_ENET_CRS_DV__GPIO_1_25	= IOMUX_PAD(0x04F0, 0x01DC, 5, 0x0000, 0, 0),
 MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC	= IOMUX_PAD(0x0398, 0x0084, 1, 0x0844, 0, 0),
 MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0	= IOMUX_PAD(0x0384, 0x0070, 1, 0x0848, 0, 0),
 MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1	= IOMUX_PAD(0x038C, 0x0078, 1, 0x084C, 0, 0),
 MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2	= IOMUX_PAD(0x0390, 0x007C, 1, 0x0850, 0, 0),
 MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3	= IOMUX_PAD(0x0394, 0x0080, 1, 0x0854, 0, 0),
 MX6Q_PAD_RGMII_RX_CTL__RGMII_RX_CTL	= IOMUX_PAD(0x0380, 0x006C, 1, 0x0858, 0, 0),

 MX6DL_PAD_ENET_MDIO__ENET_MDIO = IOMUX_PAD(0x05BC, 0x01EC, 1, 0x0810, 0, 0),
 MX6DL_PAD_ENET_MDC__ENET_MDC = IOMUX_PAD(0x05B8, 0x01E8, 1, 0x0000, 0,  0),
 MX6DL_PAD_RGMII_TXC__ENET_RGMII_TXC = IOMUX_PAD(0x06C0, 0x02D8, 1, 0x0000, 0, 0),
 MX6DL_PAD_RGMII_TD0__ENET_RGMII_TD0 = IOMUX_PAD(0x06AC, 0x02C4, 1, 0x0000, 0, 0),
 MX6DL_PAD_RGMII_TD1__ENET_RGMII_TD1 = IOMUX_PAD(0x06B0, 0x02C8, 1, 0x0000, 0, 0),
 MX6DL_PAD_RGMII_TD2__ENET_RGMII_TD2 = IOMUX_PAD(0x06B4, 0x02CC, 1, 0x0000, 0, 0),
 MX6DL_PAD_RGMII_TD3__ENET_RGMII_TD3 = IOMUX_PAD(0x06B8, 0x02D0, 1, 0x0000, 0, 0),
 MX6DL_PAD_RGMII_TX_CTL__RGMII_TX_CTL = IOMUX_PAD(0x06BC, 0x02D4, 1, 0x0000, 0, 0),
 MX6DL_PAD_ENET_REF_CLK__ENET_TX_CLK = IOMUX_PAD(0x05C0, 0x01F0, 1, 0x0000, 0, 0),
 MX6DL_PAD_RGMII_RXC__GPIO_6_30 = IOMUX_PAD(0x06A8, 0x02C0, 5, 0x0000, 0, 0),
 MX6DL_PAD_RGMII_RD0__GPIO_6_25 = IOMUX_PAD(0x0694, 0x02AC, 5, 0x0000, 0, 0),
 MX6DL_PAD_RGMII_RD1__GPIO_6_27 = IOMUX_PAD(0x0698, 0x02B0, 5, 0x0000, 0, 0),
 MX6DL_PAD_RGMII_RD2__GPIO_6_28 = IOMUX_PAD(0x069C, 0x02B4, 5, 0x0000, 0, 0),
 MX6DL_PAD_RGMII_RD3__GPIO_6_29 = IOMUX_PAD(0x06A0, 0x02B8, 5, 0x0000, 0, 0),
 MX6DL_PAD_RGMII_RX_CTL__GPIO_6_24 = IOMUX_PAD(0x06A4, 0x02BC, 5, 0x0000, 0, 0),
 MX6DL_PAD_ENET_CRS_DV__GPIO_1_25 = IOMUX_PAD(0x05B4, 0x01E4, 5, 0x0000, 0, 0),
 MX6DL_PAD_RGMII_RXC__ENET_RGMII_RXC = IOMUX_PAD(0x06A8, 0x02C0, 1, 0x0814, 1, 0),
 MX6DL_PAD_RGMII_RD0__ENET_RGMII_RD0 = IOMUX_PAD(0x0694, 0x02AC, 1, 0x0818, 1, 0),
 MX6DL_PAD_RGMII_RD1__ENET_RGMII_RD1 = IOMUX_PAD(0x0698, 0x02B0, 1, 0x081C, 1, 0),
 MX6DL_PAD_RGMII_RD2__ENET_RGMII_RD2 = IOMUX_PAD(0x069C, 0x02B4, 1, 0x0820, 1, 0),
 MX6DL_PAD_RGMII_RD3__ENET_RGMII_RD3 = IOMUX_PAD(0x06A0, 0x02B8, 1, 0x0824, 1, 0),
 MX6DL_PAD_RGMII_RX_CTL__RGMII_RX_CTL = IOMUX_PAD(0x06A4, 0x02BC, 1, 0x0828, 1, 0),
};

iomux_v3_cfg_t const mx6q_enet_pads[] = {
	MX6Q_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* pin 35 - 1 (PHY_AD2) on reset */
	MX6Q_PAD_RGMII_RXC__GPIO_6_30		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 32 - 1 - (MODE0) all */
	MX6Q_PAD_RGMII_RD0__GPIO_6_25		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 31 - 1 - (MODE1) all */
	MX6Q_PAD_RGMII_RD1__GPIO_6_27		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 28 - 1 - (MODE2) all */
	MX6Q_PAD_RGMII_RD2__GPIO_6_28		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 27 - 1 - (MODE3) all */
	MX6Q_PAD_RGMII_RD3__GPIO_6_29		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
	MX6Q_PAD_RGMII_RX_CTL__GPIO_6_24	| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* PHY nRST */ /* steven */
	MX6Q_PAD_ENET_CRS_DV__GPIO_1_25		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const mx6dl_enet_pads[] = {
	MX6DL_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6DL_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6DL_PAD_RGMII_TXC__ENET_RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6DL_PAD_RGMII_TD0__ENET_RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6DL_PAD_RGMII_TD1__ENET_RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6DL_PAD_RGMII_TD2__ENET_RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6DL_PAD_RGMII_TD3__ENET_RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6DL_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6DL_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* pin 35 - 1 (PHY_AD2) on reset */
	MX6DL_PAD_RGMII_RXC__GPIO_6_30		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 32 - 1 - (MODE0) all */
	MX6DL_PAD_RGMII_RD0__GPIO_6_25		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 31 - 1 - (MODE1) all */
	MX6DL_PAD_RGMII_RD1__GPIO_6_27		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 28 - 1 - (MODE2) all */
	MX6DL_PAD_RGMII_RD2__GPIO_6_28		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 27 - 1 - (MODE3) all */
	MX6DL_PAD_RGMII_RD3__GPIO_6_29		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
	MX6DL_PAD_RGMII_RX_CTL__GPIO_6_24	| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* PHY nRST */ /* steven */
	MX6DL_PAD_ENET_CRS_DV__GPIO_1_25		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t mx6q_enet_pads_final[] = {
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
 MX6Q_PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL)
};
iomux_v3_cfg_t mx6dl_enet_pads_final[] = {
	MX6DL_PAD_RGMII_RXC__ENET_RGMII_RXC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6DL_PAD_RGMII_RD0__ENET_RGMII_RD0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6DL_PAD_RGMII_RD1__ENET_RGMII_RD1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6DL_PAD_RGMII_RD2__ENET_RGMII_RD2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6DL_PAD_RGMII_RD3__ENET_RGMII_RD3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
 MX6DL_PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL)
};

/* steven */
static void setup_iomux_enet(void)
{
 iomux_v3_cfg_t const  * p;
 
	gpio_direction_output(IMX_GPIO_NR(1, 25), 0); /* reset pin = gpio1_io25 = low */
	gpio_direction_output(IMX_GPIO_NR(6, 30), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 25), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 27), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 28), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 29), 1);
 p = get_iomux_cfg_by_cpu(mx6q_enet_pads, mx6dl_enet_pads);
 imx_iomux_v3_setup_multiple_pads(p, ARRAY_SIZE(mx6q_enet_pads));
	gpio_direction_output(IMX_GPIO_NR(6, 24), 1);

	/* steven: Reset PHY, gpio1_io25 = high */
	udelay(500);
	/* gpio_direction_output(IMX_GPIO_NR(1, 25) , 1); */
 gpio_set_value(IMX_GPIO_NR(1, 25), 1);

	p = get_iomux_cfg_by_cpu(mx6q_enet_pads_final, mx6dl_enet_pads_final);
	imx_iomux_v3_setup_multiple_pads(p, ARRAY_SIZE(mx6q_enet_pads_final));
}

enum {
 MX6Q_PAD_EIM_D19__GPIO_3_19		= IOMUX_PAD(0x03B0, 0x009C, 5, 0x0000, 0, 0),
 MX6Q_PAD_EIM_D17__ECSPI1_MISO		= IOMUX_PAD(0x03A8, 0x0094, 1, 0x07F8, 0, 0),
 MX6Q_PAD_EIM_D18__ECSPI1_MOSI		= IOMUX_PAD(0x03AC, 0x0098, 1, 0x07FC, 0, 0),
 MX6Q_PAD_EIM_D16__ECSPI1_SCLK		= IOMUX_PAD(0x03A4, 0x0090, 1, 0x07F4, 0, 0),

 MX6DL_PAD_EIM_D19__GPIO_3_19 = IOMUX_PAD(0x0520, 0x0150, 5, 0x0000, 0, 0),
 MX6DL_PAD_EIM_D17__ECSPI1_MISO = IOMUX_PAD(0x0518, 0x0148, 1, 0x07DC, 2, 0),
 MX6DL_PAD_EIM_D18__ECSPI1_MOSI = IOMUX_PAD(0x051C, 0x014C, 1, 0x07E0, 2, 0),
 MX6DL_PAD_EIM_D16__ECSPI1_SCLK = IOMUX_PAD(0x0514, 0x0144, 1, 0x07D8, 2, 0),
};

iomux_v3_cfg_t const mx6q_ecspi1_pads[] = {
	MX6Q_PAD_EIM_D19__GPIO_3_19   | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6Q_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6Q_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6Q_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

iomux_v3_cfg_t const mx6dl_ecspi1_pads[] = {
	MX6DL_PAD_EIM_D19__GPIO_3_19   | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6DL_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6DL_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6DL_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

void setup_spinor(void)
{
 iomux_v3_cfg_t const * p = get_iomux_cfg_by_cpu(mx6q_ecspi1_pads, mx6dl_ecspi1_pads);
	imx_iomux_v3_setup_multiple_pads(p,
					 ARRAY_SIZE(mx6q_ecspi1_pads));
 /* steven: CONFIG_SF_DEFAULT_CS = GPIO3_19 */
 /* gpio_direction_output(IMX_GPIO_NR(4, 9), 0); */
 gpio_direction_output(CONFIG_SF_DEFAULT_CS, 1);
}

enum {
 MX6Q_PAD_SD2_CLK__USDHC2_CLK		= IOMUX_PAD(0x073C, 0x0354, 0, 0x0000, 0, 0),
 MX6Q_PAD_SD2_CMD__USDHC2_CMD		= IOMUX_PAD(0x0740, 0x0358, 16, 0x0000, 0, 0),
 MX6Q_PAD_SD2_DAT0__USDHC2_DAT0		= IOMUX_PAD(0x0368, 0x0054, 0, 0x0000, 0, 0),
 MX6Q_PAD_SD2_DAT1__USDHC2_DAT1		= IOMUX_PAD(0x0360, 0x004C, 0, 0x0000, 0, 0),
 MX6Q_PAD_SD2_DAT2__USDHC2_DAT2		= IOMUX_PAD(0x0364, 0x0050, 0, 0x0000, 0, 0),
 MX6Q_PAD_SD2_DAT3__USDHC2_DAT3		= IOMUX_PAD(0x0744, 0x035C, 0, 0x0000, 0, 0),
 MX6Q_PAD_NANDF_D2__GPIO_2_2		= IOMUX_PAD(0x06EC, 0x0304, 5, 0x0000, 0, 0),
 MX6Q_PAD_SD4_CLK__USDHC4_CLK		= IOMUX_PAD(0x06E0, 0x02F8, 0, 0x0000, 0, 0),
 MX6Q_PAD_SD4_CMD__USDHC4_CMD		= IOMUX_PAD(0x06DC, 0x02F4, 16, 0x0000, 0, 0),
 MX6Q_PAD_SD4_DAT0__USDHC4_DAT0		= IOMUX_PAD(0x0704, 0x031C, 1, 0x0000, 0, 0),
 MX6Q_PAD_SD4_DAT1__USDHC4_DAT1		= IOMUX_PAD(0x0708, 0x0320, 1, 0x0000, 0, 0),
 MX6Q_PAD_SD4_DAT2__USDHC4_DAT2		= IOMUX_PAD(0x070C, 0x0324, 1, 0x0000, 0, 0),
 MX6Q_PAD_SD4_DAT3__USDHC4_DAT3		= IOMUX_PAD(0x0710, 0x0328, 1, 0x0000, 0, 0),
	MX6Q_PAD_SD4_DAT4__USDHC4_DAT4		= IOMUX_PAD(0x0714, 0x032C, 1, 0x0000, 0, 0),
	MX6Q_PAD_SD4_DAT5__USDHC4_DAT5		= IOMUX_PAD(0x0718, 0x0330, 1, 0x0000, 0, 0),
	MX6Q_PAD_SD4_DAT6__USDHC4_DAT6		= IOMUX_PAD(0x071C, 0x0334, 1, 0x0000, 0, 0),
	MX6Q_PAD_SD4_DAT7__USDHC4_DAT7		= IOMUX_PAD(0x0720, 0x0338, 1, 0x0000, 0, 0),

 MX6DL_PAD_SD2_CLK__USDHC2_CLK = IOMUX_PAD(0x06DC, 0x02F4, 0, 0x0930, 1, 0),
 MX6DL_PAD_SD2_CMD__USDHC2_CMD = IOMUX_PAD(0x06E0, 0x02F8, 16, 0x0000, 0, 0),
 MX6DL_PAD_SD2_DAT0__USDHC2_DAT0 = IOMUX_PAD(0x06E4, 0x02FC, 0, 0x0000, 0, 0),
 MX6DL_PAD_SD2_DAT1__USDHC2_DAT1 = IOMUX_PAD(0x06E8, 0x0300, 0, 0x0000, 0, 0),
 MX6DL_PAD_SD2_DAT2__USDHC2_DAT2 = IOMUX_PAD(0x06EC, 0x0304, 0, 0x0000, 0, 0),
 MX6DL_PAD_SD2_DAT3__USDHC2_DAT3 = IOMUX_PAD(0x06F0, 0x0308, 0, 0x0000, 0, 0),
 MX6DL_PAD_NANDF_D2__GPIO_2_2 = IOMUX_PAD(0x0674, 0x028C, 5, 0x0000, 0, 0),
 MX6DL_PAD_SD4_CLK__USDHC4_CLK = IOMUX_PAD(0x0720, 0x0338, 0, 0x0938, 1, 0),
 MX6DL_PAD_SD4_CMD__USDHC4_CMD = IOMUX_PAD(0x0724, 0x033C, 16, 0x0000, 0, 0),
 MX6DL_PAD_SD4_DAT0__USDHC4_DAT0 = IOMUX_PAD(0x0728, 0x0340, 1, 0x0000, 0, 0),
 MX6DL_PAD_SD4_DAT1__USDHC4_DAT1 = IOMUX_PAD(0x072C, 0x0344, 1, 0x0000, 0, 0),
 MX6DL_PAD_SD4_DAT2__USDHC4_DAT2 = IOMUX_PAD(0x0730, 0x0348, 1, 0x0000, 0, 0),
 MX6DL_PAD_SD4_DAT3__USDHC4_DAT3 = IOMUX_PAD(0x0734, 0x034C, 1, 0x0000, 0, 0),
	MX6DL_PAD_SD4_DAT4__USDHC4_DAT4 = IOMUX_PAD(0x0738, 0x0350, 1, 0x0000, 0, 0),
	MX6DL_PAD_SD4_DAT5__USDHC4_DAT5 = IOMUX_PAD(0x073C, 0x0354, 1, 0x0000, 0, 0),
	MX6DL_PAD_SD4_DAT6__USDHC4_DAT6 = IOMUX_PAD(0x0740, 0x0358, 1, 0x0000, 0, 0),
	MX6DL_PAD_SD4_DAT7__USDHC4_DAT7 = IOMUX_PAD(0x0744, 0x035C, 1, 0x0000, 0, 0),
};

iomux_v3_cfg_t const mx6q_usdhc2_pads[] = {
	MX6Q_PAD_SD2_CLK__USDHC2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6Q_PAD_SD2_CMD__USDHC2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6Q_PAD_SD2_DAT0__USDHC2_DAT0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6Q_PAD_SD2_DAT1__USDHC2_DAT1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6Q_PAD_SD2_DAT2__USDHC2_DAT2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6Q_PAD_SD2_DAT3__USDHC2_DAT3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6Q_PAD_NANDF_D2__GPIO_2_2	| MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

iomux_v3_cfg_t const mx6dl_usdhc2_pads[] = {
	MX6DL_PAD_SD2_CLK__USDHC2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6DL_PAD_SD2_CMD__USDHC2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6DL_PAD_SD2_DAT0__USDHC2_DAT0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6DL_PAD_SD2_DAT1__USDHC2_DAT1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6DL_PAD_SD2_DAT2__USDHC2_DAT2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6DL_PAD_SD2_DAT3__USDHC2_DAT3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6DL_PAD_NANDF_D2__GPIO_2_2	| MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

/* steven: no usdhc3 */
#if 0
iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__USDHC3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__USDHC3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__USDHC3_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__USDHC3_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__USDHC3_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__USDHC3_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT4__USDHC3_DAT4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT5__USDHC3_DAT5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT6__USDHC3_DAT6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT7__USDHC3_DAT7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D0__GPIO_2_0    | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};
#endif /* if 0 */

iomux_v3_cfg_t const mx6q_usdhc4_pads[] = {
	MX6Q_PAD_SD4_CLK__USDHC4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6Q_PAD_SD4_CMD__USDHC4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6Q_PAD_SD4_DAT0__USDHC4_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6Q_PAD_SD4_DAT1__USDHC4_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6Q_PAD_SD4_DAT2__USDHC4_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6Q_PAD_SD4_DAT3__USDHC4_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6Q_PAD_SD4_DAT4__USDHC4_DAT4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6Q_PAD_SD4_DAT5__USDHC4_DAT5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6Q_PAD_SD4_DAT6__USDHC4_DAT6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6Q_PAD_SD4_DAT7__USDHC4_DAT7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

iomux_v3_cfg_t const mx6dl_usdhc4_pads[] = {
	MX6DL_PAD_SD4_CLK__USDHC4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6DL_PAD_SD4_CMD__USDHC4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6DL_PAD_SD4_DAT0__USDHC4_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6DL_PAD_SD4_DAT1__USDHC4_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6DL_PAD_SD4_DAT2__USDHC4_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6DL_PAD_SD4_DAT3__USDHC4_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6DL_PAD_SD4_DAT4__USDHC4_DAT4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6DL_PAD_SD4_DAT5__USDHC4_DAT5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6DL_PAD_SD4_DAT6__USDHC4_DAT6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6DL_PAD_SD4_DAT7__USDHC4_DAT7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

#ifdef CONFIG_I2C_MXC
/* steven: no pmic */
#if 0
static int setup_pmic_voltages(void)
{
	unsigned char value, rev_id = 0 ;

	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
	if (!i2c_probe(0x8)) {
		if (i2c_read(0x8, 0, 1, &value, 1)) {
			printf("Read device ID error!\n");
			return -1;
		}
		if (i2c_read(0x8, 3, 1, &rev_id, 1)) {
			printf("Read Rev ID error!\n");
			return -1;
		}
		printf("Found PFUZE100! deviceid=%x,revid=%x\n", value, rev_id);
		/*For camera streaks issue,swap VGEN5 and VGEN3 to power camera.
		*sperate VDDHIGH_IN and camera 2.8V power supply, after switch:
		*VGEN5 for VDDHIGH_IN and increase to 3V to align with datasheet
		*VGEN3 for camera 2.8V power supply
		*/
		/*increase VGEN3 from 2.5 to 2.8V*/
		if (i2c_read(0x8, 0x6e, 1, &value, 1)) {
			printf("Read VGEN3 error!\n");
			return -1;
		}
		value &= ~0xf;
		value |= 0xa;
		if (i2c_write(0x8, 0x6e, 1, &value, 1)) {
			printf("Set VGEN3 error!\n");
			return -1;
		}
		/*increase VGEN5 from 2.8 to 3V*/
		if (i2c_read(0x8, 0x70, 1, &value, 1)) {
			printf("Read VGEN5 error!\n");
			return -1;
		}
		value &= ~0xf;
		value |= 0xc;
		if (i2c_write(0x8, 0x70, 1, &value, 1)) {
			printf("Set VGEN5 error!\n");
			return -1;
		}
		/* set SW1AB staby volatage 0.975V*/
		if (i2c_read(0x8, 0x21, 1, &value, 1)) {
			printf("Read SW1ABSTBY error!\n");
			return -1;
		}
		value &= ~0x3f;
		value |= 0x1b;
		if (i2c_write(0x8, 0x21, 1, &value, 1)) {
			printf("Set SW1ABSTBY error!\n");
			return -1;
		}
		/* set SW1AB/VDDARM step ramp up time from 16us to 4us/25mV */
		if (i2c_read(0x8, 0x24, 1, &value, 1)) {
			printf("Read SW1ABSTBY error!\n");
			return -1;
		}
		value &= ~0xc0;
		value |= 0x40;
		if (i2c_write(0x8, 0x24, 1, &value, 1)) {
			printf("Set SW1ABSTBY error!\n");
			return -1;
		}

		/* set SW1C staby volatage 0.975V*/
		if (i2c_read(0x8, 0x2f, 1, &value, 1)) {
			printf("Read SW1CSTBY error!\n");
			return -1;
		}
		value &= ~0x3f;
		value |= 0x1b;
		if (i2c_write(0x8, 0x2f, 1, &value, 1)) {
			printf("Set SW1CSTBY error!\n");
			return -1;
		}

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		if (i2c_read(0x8, 0x32, 1, &value, 1)) {
			printf("Read SW1ABSTBY error!\n");
			return -1;
		}
		value &= ~0xc0;
		value |= 0x40;
		if (i2c_write(0x8, 0x32, 1, &value, 1)) {
			printf("Set SW1ABSTBY error!\n");
			return -1;
		}
	}

	return 0;
}
#endif /* steven: if 0, no pmic */

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
 /* steven: no pmic */
}
#endif
#endif


#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC2_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

int mmc_get_env_devno(void)
{
	u32 soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	u32 dev_no;

	/* BOOT_CFG2[3] and BOOT_CFG2[4] */
	dev_no = (soc_sbmr & 0x00001800) >> 11;

	/* steven: emmc=sd4, microsd=sd2(default) */
	switch( dev_no ) {
		case 3: return 1;
		default: return 0;
	}

	return dev_no;
}


#define USDHC2_CD_GPIO	IMX_GPIO_NR(2, 2)
#define USDHC3_CD_GPIO	IMX_GPIO_NR(2, 0)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		ret = !gpio_get_value(USDHC2_CD_GPIO);
		break;
	/* steven: no usdhc3 
	case USDHC3_BASE_ADDR:
		ret = !gpio_get_value(USDHC3_CD_GPIO);
		break;
		* */
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int i;
 iomux_v3_cfg_t const * p;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SD2
	 * mmc1                    SD3 // steven: no sd3
	 * mmc2                    eMMC
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			p = get_iomux_cfg_by_cpu(mx6q_usdhc2_pads, mx6dl_usdhc2_pads);
			imx_iomux_v3_setup_multiple_pads(
				p, ARRAY_SIZE(mx6q_usdhc2_pads));
			gpio_direction_input(USDHC2_CD_GPIO);
			usdhc_cfg[i].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
   /* steven: sd2=4bit */
   usdhc_cfg[i].max_bus_width = 4;
			break;
/* steven: no sd3 */
#if 0
		case 1:
			p = get_iomux_cfg_by_cpu(mx6q_usdhc3_pads, mx6dl_usdhc3_pads);
			imx_iomux_v3_setup_multiple_pads(
				p, ARRAY_SIZE(mx6q_usdhc3_pads));
			gpio_direction_input(USDHC3_CD_GPIO);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
#endif /* steven: if 0, no sd3 */
		case 1:
			p = get_iomux_cfg_by_cpu(mx6q_usdhc4_pads, mx6dl_usdhc4_pads);
			imx_iomux_v3_setup_multiple_pads(
				p, ARRAY_SIZE(mx6q_usdhc4_pads));
			usdhc_cfg[i].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return 0;
		}

		if (fsl_esdhc_initialize(bis, &usdhc_cfg[i]))
			printf("Warning: failed to initialize mmc dev %d\n", i);
	}

	return 0;
}

void board_late_mmc_env_init(void)
{
	char cmd[32];
	u32 dev_no = mmc_get_env_devno();

	setenv_ulong("mmcdev", dev_no);

	sprintf(cmd, "mmc dev %d", dev_no);
	run_command(cmd, 0);
}
#endif

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
#ifdef CONFIG_SPLASH_SCREEN
extern int mmc_get_env_devno(void);
int setup_splash_img(void)
{
#ifdef CONFIG_SPLASH_IS_IN_MMC
	int mmc_dev = mmc_get_env_devno();
	ulong offset = CONFIG_SPLASH_IMG_OFFSET;
	ulong size = CONFIG_SPLASH_IMG_SIZE;
	ulong addr = 0;
	char *s = NULL;
	struct mmc *mmc = find_mmc_device(mmc_dev);
	uint blk_start, blk_cnt, n;

	s = getenv("splashimage");

	if (NULL == s) {
		puts("env splashimage not found!\n");
		return -1;
	}
	addr = simple_strtoul(s, NULL, 16);

	if (!mmc) {
		printf("MMC Device %d not found\n", mmc_dev);
		return -1;
	}

	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return -1;
	}

	blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
	blk_cnt   = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;
	n = mmc->block_dev.block_read(mmc_dev, blk_start,
					blk_cnt, (u_char *)addr);
	flush_cache((ulong)addr, blk_cnt * mmc->read_bl_len);

	return (n == blk_cnt) ? 0 : -1;
#endif

	return 0;
}
#endif

int setup_waveform_file(void)
{
#ifdef CONFIG_WAVEFORM_FILE_IN_MMC
	int mmc_dev = mmc_get_env_devno();
	ulong offset = CONFIG_WAVEFORM_FILE_OFFSET;
	ulong size = CONFIG_WAVEFORM_FILE_SIZE;
	ulong addr = CONFIG_WAVEFORM_BUF_ADDR;
	struct mmc *mmc = find_mmc_device(mmc_dev);
	uint blk_start, blk_cnt, n;

	if (!mmc) {
		printf("MMC Device %d not found\n", mmc_dev);
		return -1;
	}

	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return -1;
	}

	blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
	blk_cnt   = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;
	n = mmc->block_dev.block_read(mmc_dev, blk_start,
				      blk_cnt, (u_char *)addr);
	flush_cache((ulong)addr, blk_cnt * mmc->read_bl_len);

	return (n == blk_cnt) ? 0 : -1;
#else
	return -1;
#endif
}

#endif

#ifdef CONFIG_CMD_SATA
int setup_sata(void)
{
	struct iomuxc_base_regs *const iomuxc_regs
		= (struct iomuxc_base_regs *) IOMUXC_BASE_ADDR;
	int ret = enable_sata_clock();
	if (ret)
		return ret;

	clrsetbits_le32(&iomuxc_regs->gpr[13],
			IOMUXC_GPR13_SATA_MASK,
			IOMUXC_GPR13_SATA_PHY_8_RXEQ_3P0DB
			|IOMUXC_GPR13_SATA_PHY_7_SATA2M
			|IOMUXC_GPR13_SATA_SPEED_3G
			|(3<<IOMUXC_GPR13_SATA_PHY_6_SHIFT)
			|IOMUXC_GPR13_SATA_SATA_PHY_5_SS_DISABLED
			|IOMUXC_GPR13_SATA_SATA_PHY_4_ATTEN_9_16
			|IOMUXC_GPR13_SATA_PHY_3_TXBOOST_0P00_DB
			|IOMUXC_GPR13_SATA_PHY_2_TX_1P104V
			|IOMUXC_GPR13_SATA_PHY_1_SLOW);

	return 0;
}
#endif

int mx6_rgmii_rework(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev); /**/
#if 0
	/* min rx data delay */
	ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_RX_DATA_SKEW, 0x0);
	/* min tx data delay */
	ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_TX_DATA_SKEW, 0x0);
	/* max rx/tx clock delay, min rx/tx control */
	ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_CLOCK_SKEW, 0xf0f0);
#endif
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)
struct display_info_t {
	int	bus;
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev);
	struct	fb_videomode mode;
};

static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

/* steven */
static int detect_hdmi(struct display_info_t const *dev)
{
	struct hdmi_regs *hdmi	= (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
	return readb(&hdmi->phy_stat0) & HDMI_DVI_STAT;
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}

static struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= NULL,
	.mode	= {
		.name           = "Hannstar-XGA",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	/*.detect	= detect_hdmi, */
	.detect	= NULL, /* steven: for disable HDMI */
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 640,
		.yres           = 480,
		.pixclock       = 39721,
		.left_margin    = 48,
		.right_margin   = 16,
		.upper_margin   = 33,
		.lower_margin   = 10,
		.hsync_len      = 96,
		.vsync_len      = 2,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} } };

int board_video_skip(void)
{
	int i;
	int ret;
	char const *panel = getenv("panel");
	if (!panel) {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			struct display_info_t const *dev = displays+i;
			if (dev->detect && dev->detect(dev)) {
				panel = dev->mode.name;
				printf("auto-detected panel %s\n", panel);
				break;
			}
		}
		if (!panel) {
			i = 0; /* steven: default display, 0: enable lvds, 1: enable hdmi */
			panel = displays[i].mode.name;
			printf("No panel detected: default to %s\n", panel);
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			if (!strcmp(panel, displays[i].mode.name))
				break;
		}
	}
	if (i < ARRAY_SIZE(displays)) {
		ret = ipuv3_fb_init(&displays[i].mode, 0,
				    displays[i].pixfmt);
		if (!ret) {
			if (displays[i].enable)
				displays[i].enable(displays+i);
			printf("Display: %s (%ux%u)\n",
			       displays[i].mode.name,
			       displays[i].mode.xres,
			       displays[i].mode.yres);
		} else
			printf("LCD %s cannot be configured: %d\n",
			       displays[i].mode.name, ret);
	} else {
		printf("unsupported panel %s\n", panel);
		return -EINVAL;
	}

	return 0;
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
			| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_eth_init(bd_t *bis)
{
	int ret;

	setup_iomux_enet();

	ret = cpu_eth_init(bis);
	if (ret)
		printf("FEC MXC: %s:failed\n", __func__);

	return 0;
}

int board_early_init_f(void)
{
	setup_iomux_uart();

	/* steven: Switch to 1GHZ */
	clk_config(CONFIG_REF_CLK_FREQ, 1000, CPU_CLK);
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

	setup_spinor();

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif
	return 0;
}
/* steven */
int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd2",	 MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
/* steven: no sd3 */
/*	{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)}, */
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_I2C_MXC
	if(!cpu_is_mx6q()) {
		struct i2c_pin_ctrl* pscl = &(i2c_pad_info0.scl);
		struct i2c_pin_ctrl* psda = &(i2c_pad_info0.sda);
		pscl->i2c_mode = MX6DL_PAD_CSI0_DAT9__I2C1_SCL | PC;
		pscl->gpio_mode = MX6DL_PAD_CSI0_DAT9__GPIO_5_27 | PC;
		psda->i2c_mode = MX6DL_PAD_CSI0_DAT8__I2C1_SDA | PC,
		psda->gpio_mode = MX6DL_PAD_CSI0_DAT8__GPIO_5_26 | PC,
		pscl = &(i2c_pad_info1.scl);
		psda = &(i2c_pad_info1.sda);
		pscl->i2c_mode = MX6DL_PAD_KEY_COL3__I2C2_SCL | PC;
		pscl->gpio_mode = MX6DL_PAD_KEY_COL3__GPIO_4_12 | PC;
		psda->i2c_mode = MX6DL_PAD_KEY_ROW3__I2C2_SDA | PC;
		psda->gpio_mode = MX6DL_PAD_KEY_ROW3__GPIO_4_13 | PC;
		pscl = &(i2c_pad_info2.scl);
		psda = &(i2c_pad_info2.sda);
		pscl->i2c_mode = MX6_PAD_GPIO_3__I2C3_SCL | PC;
		pscl->gpio_mode = MX6_PAD_GPIO_3__GPIO_1_3 | PC;
		psda->i2c_mode = MX6_PAD_GPIO_6__I2C3_SDA | PC;
		psda->gpio_mode = MX6_PAD_GPIO_6__GPIO_1_6 | PC;
	}
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
 /* steven_debug: setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x42, &i2c_pad_info0); */
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
 /* steven_debug: setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x0a, &i2c_pad_info2); */
 /* steven: no pmic */
 /* if (setup_pmic_voltages()) return -1; */
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	return 0;
}

#ifdef CONFIG_FASTBOOT

void board_fastboot_setup(void)
{
	switch (get_boot_device()) {
#if defined(CONFIG_FASTBOOT_STORAGE_SATA)
	case SATA_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "sata");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "booti sata");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_SATA*/
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD2_BOOT:
	case MMC2_BOOT:
	    if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc0");
	    if (!getenv("bootcmd"))
			setenv("bootcmd", "booti mmc0");
	    break;
	case SD3_BOOT:
	case MMC3_BOOT:
	    if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc1");
	    if (!getenv("bootcmd"))
			setenv("bootcmd", "booti mmc1");
	    break;
	case MMC4_BOOT:
	    if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc2");
	    if (!getenv("bootcmd"))
			setenv("bootcmd", "booti mmc2");
	    break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("unsupported boot devices\n");
		break;
	}

}

#ifdef CONFIG_ANDROID_RECOVERY

#define GPIO_VOL_DN_KEY IMX_GPIO_NR(1, 5)
iomux_v3_cfg_t const recovery_key_pads[] = {
	(MX6_PAD_GPIO_5__GPIO_1_5 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

int check_recovery_cmd_file(void)
{
    int button_pressed = 0;
    int recovery_mode = 0;

    recovery_mode = recovery_check_and_clean_flag();

    /* Check Recovery Combo Button press or not. */
	imx_iomux_v3_setup_multiple_pads(recovery_key_pads,
			ARRAY_SIZE(recovery_key_pads));

    gpio_direction_input(GPIO_VOL_DN_KEY);

    if (gpio_get_value(GPIO_VOL_DN_KEY) == 0) { /* VOL_DN key is low assert */
		button_pressed = 1;
		printf("Recovery key pressed\n");
    }

    return recovery_mode || button_pressed;
}

void board_recovery_setup(void)
{
	int bootdev = get_boot_device();

	switch (bootdev) {
#if defined(CONFIG_FASTBOOT_STORAGE_SATA)
	case SATA_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"booti sata recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_SATA*/
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD2_BOOT:
	case MMC2_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"booti mmc0 recovery");
		break;
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"booti mmc1 recovery");
		break;
	case MMC4_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"booti mmc2 recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("Unsupported bootup device for recovery: dev: %d\n",
			bootdev);
		return;
	}

	printf("setup env for recovery..\n");
	setenv("bootcmd", "run bootcmd_android_recovery");
}

#endif /*CONFIG_ANDROID_RECOVERY*/

#endif /*CONFIG_FASTBOOT*/

/* steven */
int checkboard(void)
{
	printf("Board: %s-VAB820\n", cpu_is_mx6q() ? "MX6Q" : "MX6DL");

	return 0;
}

enum {
 MX6Q_PAD_ENET_RX_ER__ANATOP_USBOTG_ID	= IOMUX_PAD(0x04EC, 0x01D8, 0, 0x0000, 0, 0),
 MX6Q_PAD_EIM_D22__USBOH3_USBOTG_PWR	= IOMUX_PAD(0x03BC, 0x00A8, 4, 0x0000, 0, 0),
 MX6Q_PAD_EIM_DA0__GPIO_3_0		= IOMUX_PAD(0x0428, 0x0114, 5, 0x0000, 0, 0),

 MX6DL_PAD_ENET_RX_ER__ANATOP_USBOTG_ID = IOMUX_PAD(0x05C4, 0x01F4, 0, 0x0790, 0, 0),
 MX6DL_PAD_EIM_D22__USBOH3_USBOTG_PWR = IOMUX_PAD(0x052C, 0x015C, 4, 0x0000, 0, 0),
 MX6DL_PAD_EIM_DA0__GPIO_3_0 = IOMUX_PAD(0x0554, 0x0184, 5, 0x0000, 0, 0),
};

/* steven: no CONFIG_IMX_UDC */
#ifdef CONFIG_IMX_UDC

iomux_v3_cfg_t const mx6q_otg_udc_pads[] = {
	(MX6Q_PAD_ENET_RX_ER__ANATOP_USBOTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

iomux_v3_cfg_t const mx6dl_otg_udc_pads[] = {
	(MX6DL_PAD_ENET_RX_ER__ANATOP_USBOTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

void udc_pins_setting(void)
{
	iomux_v3_cfg_t const * p = get_iomux_cfg_by_cpu(mx6q_otg_udc_pads, mx6dl_otg_udc_pads);
	imx_iomux_v3_setup_multiple_pads(p,
			ARRAY_SIZE(mx6q_otg_udc_pads));

	/*set daisy chain for otg_pin_id on 6q. for 6dl, this bit is reserved*/
    mxc_iomux_set_gpr_register(1, 13, 1, 0);
}

#endif /*CONFIG_IMX_UDC*/

#ifdef CONFIG_USB_EHCI_MX6
iomux_v3_cfg_t const mx6q_usb_otg_pads[] = {
	MX6Q_PAD_EIM_D22__USBOH3_USBOTG_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6Q_PAD_ENET_RX_ER__ANATOP_USBOTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const mx6dl_usb_otg_pads[] = {
	MX6DL_PAD_EIM_D22__USBOH3_USBOTG_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6DL_PAD_ENET_RX_ER__ANATOP_USBOTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL),
};


iomux_v3_cfg_t const mx6q_usb_hc1_pads[] = {
	MX6Q_PAD_EIM_DA0__GPIO_3_0 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const mx6dl_usb_hc1_pads[] = {
	MX6DL_PAD_EIM_DA0__GPIO_3_0 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

int board_ehci_hcd_init(int port)
{
	iomux_v3_cfg_t const * p;
	switch (port) {
	case 0:
		p = get_iomux_cfg_by_cpu(mx6q_usb_otg_pads, mx6dl_usb_otg_pads);
		imx_iomux_v3_setup_multiple_pads(p,
			ARRAY_SIZE(mx6q_usb_otg_pads));

		/*set daisy chain for otg_pin_id on 6q. for 6dl, this bit is reserved*/
		mxc_iomux_set_gpr_register(1, 13, 1, 0);
		break;
	case 1:
		p = get_iomux_cfg_by_cpu(mx6q_usb_hc1_pads, mx6dl_usb_hc1_pads);
		imx_iomux_v3_setup_multiple_pads(p,
			ARRAY_SIZE(mx6q_usb_hc1_pads));
		/* steven: Reset USB hub: USB_HUB_RESET_B=GPIO_18=GPIO7_IO13 */
		gpio_direction_output(IMX_GPIO_NR(7, 13), 0);
		mdelay(2);
		gpio_set_value(IMX_GPIO_NR(7, 13), 1);
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return 1;
	}
	return 0;
}

/* steven: no power control, use uboot.default.board_ehci_power() */
#if 1
int board_ehci_power(int port, int on)
{
	switch (port) {
	case 0:
		break;
	case 1:
		if (on)
/* steven:			gpio_direction_output(IMX_GPIO_NR(1, 29), 1); */
			gpio_direction_output(IMX_GPIO_NR(7, 13), 1);
		else
/* steven:			gpio_direction_output(IMX_GPIO_NR(1, 29), 0); */
			gpio_direction_output(IMX_GPIO_NR(7, 13), 0);
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return 1;
	}
	return 0;
}
#endif /* steven: #if 0 */

#endif

/* steven: add a command */
static int cpu_detect(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	return cpu_is_mx6q() ? 0 : 1;
}

U_BOOT_CMD(
	cpudetect, CONFIG_SYS_MAXARGS, 0, cpu_detect,
	"detect cpu type",
	"Returns 0: iMX6Q, 1: iMX6DL"
);


#if defined(CONFIG_CMD_SAVEENV) && !defined(CONFIG_ENV_IS_NOWHERE)
#include <environment.h>

static int bDestroyEnv = 0;
int in_destroyenv(void) {
 return bDestroyEnv;
}

int do_destroyenv(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int nResult;

	bDestroyEnv = 1;
	/* 	printf("invalidate the CRC\n"); */
	printf("write invalidate enviroment data to storage\n");
	nResult = saveenv() ? 1 : 0;
	bDestroyEnv = 0;
	return nResult;
}
#endif

#if defined(CONFIG_CMD_SAVEENV) && !defined(CONFIG_ENV_IS_NOWHERE)
U_BOOT_CMD(
	destroyenv, CONFIG_SYS_MAXARGS, 0,	do_destroyenv,
	"destroy enviroment variables stored in medium",
	"\n    - destroy all environment variables in medium"
	"\n      after reset the default settings will be used"
);
#endif

#include "viatools.c"

void adv_power(void) {
	/* CSI0_PWN=SD1_DAT0=GPIO_1_16 */
	gpio_direction_output(IMX_GPIO_NR(1, 16), 0);
}

void adv_reset(void) {
	/* CSI0_RST_B=SD1_DAT1=GPIO_1_17 */
	gpio_direction_output(IMX_GPIO_NR(1, 17), 0);
	udelay(500);
	gpio_direction_output(IMX_GPIO_NR(1, 17), 1);
}

int do_adv7180(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc > 1) {
		if (strcmp(argv[1], "power") == 0) { adv_power(); }
		else if (strcmp(argv[1], "reset") == 0) { adv_reset(); }
	}
	return 0;
}

/*
U_BOOT_CMD(
	adv7180, CONFIG_SYS_MAXARGS, 0, do_adv7180,
	"control adv7180",
	"{power|reset}"
	"\n   power = power on adv7180"
	"\n   reset = reset adv7180"
);
*/
