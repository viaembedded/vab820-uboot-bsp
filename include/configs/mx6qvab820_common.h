/*
 * Copyright (C) 2012-2014 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.         See the
 * GNU General Public License for more details.
 */

#ifndef __MX6QSABRE_COMMON_CONFIG_H
#define __MX6QSABRE_COMMON_CONFIG_H

#define CONFIG_MX6

#define CONFIG_SYS_L2_PL310
#define CONFIG_SYS_PL310_BASE		0x00A02000
#define CONFIG_SYS_CACHELINE_SIZE	32


/* steven: DRAM, uncomment for PLUGIN mode support */
#define CONFIG_USE_PLUGIN

#define CONFIG_ARCH_MISC_INIT

/* uncomment for SECURE mode support */
/* #define CONFIG_SECURE_BOOT */

#include "mx6_common.h"
#include <asm/sizes.h>

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#include <asm/arch/imx-regs.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART

/* OCOTP Config */
#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR      0

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
/* steven: */
#define CONFIG_FEC_MXC_PHYADDR		7
#define CONFIG_PHYLIB
/* steven */
#define CONFIG_PHY_MICREL
#define CONFIG_PHY_MICREL_KSZ9031

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX              1
#define CONFIG_BAUDRATE                        115200

/* Command definition */
#include <config_cmd_default.h>

#define CONFIG_CMD_BMODE
#define CONFIG_CMD_BOOTZ
#undef CONFIG_CMD_IMLS
/* steven: 1 --> 3 */
#define CONFIG_BOOTDELAY               3

#define CONFIG_LOADADDR			0x12000000
#define CONFIG_SYS_TEXT_BASE		0x17800000
#define CONFIG_SYS_MMC_IMG_LOAD_PART	1
#define CONFIG_MMC_DEV_SET "mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV)

#define CONFIG_MFG_ENV_SETTINGS \
	"mfgtool_args=setenv bootargs console=" CONFIG_CONSOLE_DEV ",115200 " \
		"rdinit=/linuxrc " \
		"g_mass_storage.stall=0 g_mass_storage.removable=1 " \
		"g_mass_storage.idVendor=0x066F g_mass_storage.idProduct=0x37FF "\
		"g_mass_storage.iSerialNumber=\"\" "\
		"enable_wait_mode=off "\
		"\0" \
		"initrd_addr=0x12C00000\0" \
		"initrd_high=0xffffffff\0" \
		"bootcmd_mfg=run mfgtool_args;bootm ${loadaddr} ${initrd_addr} ${fdt_addr};\0" \


/* steven: modify uimage, loaduimage for vab820
   steven: CONFIG_DEFAULT_FDT_FILE --> imx6q-vab820.dtb
 * */
#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS \
	"script=boot.scr\0" \
	"uimage=uImage\0" \
	"fdt_file=" "imx6q-vab820.dtb" "\0" \
	"fdt_addr=0x18000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	CONFIG_MMC_DEV_SET \
	"\0" \
	"mmcpart=" __stringify(CONFIG_SYS_MMC_IMG_LOAD_PART) "\0" \
	"hdmi=video=mxcfb0:dev=hdmi,1920x1080M@60,bpp=32\0" \
	"lvds_auo_g070=video=mxcfb0:dev=ldb,480C60,if=RGB24 ldb=sep0\0" \
	"lvds_auo_g104=video=mxcfb0:dev=ldb,LDB-XGA,if=RGB24 ldb=sin0\0" \
	"lvds_auo_g220=video=mxcfb0:dev=ldb,LDB-WSXGA+,if=RGB24 ldb=spl0\0" \
	"lvds_qa=video=mxcfb0:dev=ldb,LDB-SXGA,if=RGB24 ldb=spl0\0" \
	"lvds=video=mxcfb0:dev=ldb,LDB-XGA,if=RGB24 ldb=sin0\0" \
	"hdmilvds=video=mxcfb0:dev=hdmi,1920x1080@60,bpp=32 video=mxcfb1:dev=ldb,LDB-SXGA,if=RGB24 ldb=spl0\0" \
	"output_display=video=mxcfb0:dev=hdmi,1920x1080M@60,bpp=32\0" \
	"o_hdmi=setenv output_display ${hdmi}\0" \
	"o_lvdsg070=setenv output_display ${lvds_auo_g070}\0" \
	"o_lvdsg104=setenv output_display ${lvds_auo_g104}\0" \
	"o_lvdsg220=setenv output_display ${lvds_auo_g220}\0" \
	"o_lvdsqa=setenv output_display ${lvds_qa}\0" \
	"o_hdmilvds=setenv output_display ${hdmilvds}\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} console=tty1,${baudrate} ${output_display} " \
		"root=${mmcroot}\0" \
	"loadbootscript=" \
		"fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from ${mmcname} ...; " \
		"source\0" \
	"loaduimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${uimage}\0" \
	"loadfdt=if cpudetect; then setenv fdt_file imx6q-vab820.dtb; else setenv fdt_file imx6dl-vab820.dtb; fi; " \
		"fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"mmcboot=echo Booting from ${mmcname} ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootm ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootm; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootm; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${uimage}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
				"bootm ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootm; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootm; " \
		"fi;\0" \
	"bootcmd_main=setenv mmcroot /dev/mmcblk${rootdev}p2 rootwait rw;" \
	"mmc dev ${mmcdev};" \
	"if mmc rescan; then " \
		"if run loadbootscript; then " \
		"run bootscript; " \
		"else " \
			"if run loaduimage; then " \
				"run mmcboot; " \
			"else run netboot; " \
			"fi; " \
		"fi; " \
	"else run netboot; fi\0" \
	"bootcmd_sd=setenv mmcdev 0; setenv rootdev 1; setenv mmcname sdcard; run bootcmd_main;\0" \
	"bootcmd_mmc=setenv mmcdev 1; setenv rootdev 0; setenv mmcname mmc; run bootcmd_main;\0" \
	"bootcmd_auto=if run loaduimage; then run bootcmd_sd; else run bootcmd_mmc; fi;\0"

#define CONFIG_BOOTCOMMAND \
	"run bootcmd_auto"

#define CONFIG_ARP_TIMEOUT     200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_SYS_PROMPT              "U-Boot > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE              1024

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS             16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END         0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR
#define CONFIG_SYS_HZ                  1000

#define CONFIG_CMDLINE_EDITING
#define CONFIG_STACKSIZE               (128 * 1024)

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(8 * 1024)

/* steven: use SPIROM, defined from boards.cfg */
/* #define CONFIG_SYS_USE_SPINOR */ 
/* steven: environment is in SPIROM */
#define CONFIG_ENV_IS_IN_SPI_FLASH
/* #define CONFIG_ENV_IS_IN_MMC */

#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif

#ifdef CONFIG_SYS_USE_SPINOR
#define CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
/* steven support SST, WINBOND, MXIC */
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_SPI_FLASH_SST
#define CONFIG_SPI_FLASH_WINBOND
#define CONFIG_SPI_FLASH_MACRONIX
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS  0
/* steven: 20000000 --> 25000000 */
#define CONFIG_SF_DEFAULT_SPEED 25000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
#endif

#if defined(CONFIG_ENV_IS_IN_MMC)
/* steven: use 768, not 8*64 */
#define CONFIG_ENV_OFFSET		(768 * 1024)
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		(768 * 1024)
#define CONFIG_ENV_SECT_SIZE		(64 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#elif defined(CONFIG_ENV_IS_IN_FLASH)
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_SIZE			CONFIG_SYS_FLASH_SECT_SIZE
#define CONFIG_ENV_SECT_SIZE		CONFIG_SYS_FLASH_SECT_SIZE
#define CONFIG_ENV_OFFSET		(4 * CONFIG_SYS_FLASH_SECT_SIZE)
#elif defined(CONFIG_ENV_IS_IN_NAND)
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_OFFSET		(8 << 20)
#define CONFIG_ENV_SECT_SIZE		(128 << 10)
#define CONFIG_ENV_SIZE			CONFIG_ENV_SECT_SIZE
#elif defined(CONFIG_ENV_IS_IN_SATA)
#define CONFIG_ENV_OFFSET		(768 * 1024)
#define CONFIG_SATA_ENV_DEV		0
#define CONFIG_SYS_DCACHE_OFF /* remove when sata driver support cache */
#endif

#define CONFIG_OF_LIBFDT

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

/*
 * I2C configs
 */
#define CONFIG_CMD_I2C
#define CONFIG_HARD_I2C         1
#define CONFIG_I2C_MXC          1
#define CONFIG_SYS_I2C_BASE             I2C2_BASE_ADDR
#define CONFIG_SYS_I2C_SPEED            100000
#define CONFIG_SYS_I2C_SLAVE            0x8

/* Framebuffer */
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
/* steven: need to do it -- CONFIG_IPUV3_CLK */
#ifdef CONFIG_MX6DL
#define CONFIG_IPUV3_CLK 198000000
#else
#define CONFIG_IPUV3_CLK 264000000
#endif
#define CONFIG_IMX_HDMI

#endif                         /* __MX6QSABRE_COMMON_CONFIG_H */
