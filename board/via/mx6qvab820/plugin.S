/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc.
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

#include <config.h>

//*********************************************************************
// steven: use old method
//*********************************************************************

#define ROM_API_TABLE_BASE_ADDR_LEGACY		0xC0
#define ROM_API_TABLE_BASE_ADDR_MX6DQ_TO15	0xC4
#define ROM_API_TABLE_BASE_ADDR_MX6DL_TO12	0xC4
#define ROM_API_HWCNFG_SETUP_OFFSET		0x08
#define ROM_VERSION_OFFSET			0x48
#define ROM_VERSION_TO12			0x12
#define ROM_VERSION_TO15			0x15
#define IRAM_FREE_START		0x00907000

#define MXC_DCD_ITEM(i, addr, val)   \
 ldr r1, =val; \
 ldr r0, =addr; \
 str r1, [r0];

.macro vab820_imx6_ddr_setting

// CCM_BASE_ADDR = 0x020C4000
//DDR clk
    ldr r0, =0x020C8260
    ldr r6, [r0]
    lsr r6, r6, #16
    and r6, r6, #0xff
    cmp r6, #0x63
    bne Label_mx6dl_speed
// 528MHz
    ldr r0, =CCM_BASE_ADDR
    ldr r1, =0x00020324
    str r1, [r0, #0x018]
    b Label_mx6_speed_end
Label_mx6dl_speed:
// 400MHz
    ldr r0, =CCM_BASE_ADDR
    ldr r1, =0x00060324
    str r1, [r0, #0x018]
Label_mx6_speed_end:

// vab820
    cmp r6, #0x63
    bne Label_mx6dl_gpio
    ldr r1, =0x05
    ldr r0, =0x20e02fc
    str r1, [r0]
    ldr r0, =0x20e0300
    str r1, [r0]
    b Label_mx6_gpio_end
Label_mx6dl_gpio:
    ldr r1, =0x05
    ldr r0, =0x20e0284
    str r1, [r0]
    ldr r0, =0x20e0288
    str r1, [r0]
Label_mx6_gpio_end:
    ldr r0, =0x20a0004
    ldr r1, =0x00
    str r1, [r0]
    ldr r0, =0x20a0008
    ldr r1, [r0]
    and r1, #3
    cmp r1, #1
    beq Label_vab820_4g_a
    cmp r1, #2
    beq Label_vab820_2g
    // default r1 = 3 = 1g
    b Label_vab820_1g
Label_vab820_4g_a:
    b Label_vab820_4g_b

    .ltorg
Label_vab820_2g:
    cmp r6, #0x63
    bne Label_mx6dl_2g_ram
// imx6q_2g_ram
#include "vab820_mx6q_2g.cfg"
    b Label_mx6_2g_ram_end
Label_mx6dl_2g_ram:
// imx6dl_2g_ram
#include "vab820_mx6dl_2g.cfg"
Label_mx6_2g_ram_end:
    b Label_vab820_ram_setting_end

    .ltorg
Label_vab820_1g:
    cmp r6, #0x63
    bne Label_mx6dl_1g_ram
// imx6q_1g_ram
#include "vab820_mx6q_1g.cfg"
    b Label_mx6_1g_ram_end
Label_mx6dl_1g_ram:
// imx6dl_1g_ram
#include "vab820_mx6dl_1g.cfg"
Label_mx6_1g_ram_end:
    b Label_vab820_ram_setting_end

    .ltorg
Label_vab820_4g_b:
  #include "vab820_mx6q_4g.cfg"


Label_vab820_ram_setting_end:

.endm

//*********************************************************************
// steven: ref. ./include/asm/arch/mx6_plugin.S
//*********************************************************************

.macro imx6_clock_gating
	ldr r0, =CCM_BASE_ADDR
	ldr r1, =0x00C03F3F
	str r1, [r0, #0x068]
	ldr r1, =0x0030FC03
	str r1, [r0, #0x06c]
	ldr r1, =0x0FFFC000
	str r1, [r0, #0x070]
	ldr r1, =0x3FF00000
	str r1, [r0, #0x074]
	ldr r1, =0x00FFF300
	str r1, [r0, #0x078]
	ldr r1, =0x0F0000C3
	str r1, [r0, #0x07c]
	ldr r1, =0x000003FF
	str r1, [r0, #0x080]
.endm

.macro imx6_qos_setting
	ldr r0, =IOMUXC_BASE_ADDR
	ldr r1, =0xF00000CF
	str r1, [r0, #0x10]

	ldr r1, =0x007F007F
	str r1, [r0, #0x18]
	str r1, [r0, #0x1c]
.endm

//*********************************************************************
// steven: ref. ./include/asm/arch/mx6_plugin.S
//*********************************************************************

plugin_start:

 push    {r0-r4, lr}

 vab820_imx6_ddr_setting
 imx6_clock_gating
 imx6_qos_setting

/*
 * The following is to fill in those arguments for this ROM function
 * pu_irom_hwcnfg_setup(void **start, size_t *bytes, const void *boot_data)
 * This function is used to copy data from the storage media into DDR.
 * start - Initial (possibly partial) image load address on entry.
 *         Final image load address on exit.
 * bytes - Initial (possibly partial) image size on entry.
 *         Final image size on exit.
 * boot_data - Initial @ref ivt Boot Data load address.
 */
	adr r0, boot_data2
	adr r1, image_len2
	adr r2, boot_data2

#ifdef CONFIG_SYS_BOOT_EIMNOR
	ldr r3, =0x00900800
	ldr r4, =0x08000000
	str r4, [r3, #0xc0]
#endif

/*
 * check the _pu_irom_api_table for the address
 */
before_calling_rom___pu_irom_hwcnfg_setup:
// steven: vab820 method.
// * r6 is set by vab820_imx6_ddr_setting
    ldr r3, =ROM_VERSION_OFFSET
    ldr r4, [r3]
    cmp r6, #0x63
    bne Label_mx6dl_version
    ldr r3, =ROM_VERSION_TO15
    cmp r4, r3
    ldrge r3, =ROM_API_TABLE_BASE_ADDR_MX6DQ_TO15
    b Label_mx6_version_end
Label_mx6dl_version:
    ldr r3, =ROM_VERSION_TO12
    cmp r4, r3
    ldrge r3, =ROM_API_TABLE_BASE_ADDR_MX6DL_TO12
Label_mx6_version_end:
    ldrlt r3, =ROM_API_TABLE_BASE_ADDR_LEGACY
    ldr r4, [r3, #ROM_API_HWCNFG_SETUP_OFFSET]
    blx r4
after_calling_rom___pu_irom_hwcnfg_setup:


/* To return to ROM from plugin, we need to fill in these argument.
 * Here is what need to do:
 * Need to construct the paramters for this function before return to ROM:
 * plugin_download(void **start, size_t *bytes, UINT32 *ivt_offset)
 */
	pop {r0-r4, lr}
	ldr r5, boot_data2
	str r5, [r0]
	ldr r5, image_len2
	str r5, [r1]
	ldr r5, second_ivt_offset
	str r5, [r2]
	mov r0, #1

	/* return back to ROM code */
	bx lr

/* make the following data right in the end of the output*/
.ltorg

#ifdef CONFIG_SYS_BOOT_EIMNOR
#define FLASH_OFFSET 0x1000
#else
#define FLASH_OFFSET 0x400
#endif

/*
 * second_ivt_offset is the offset from the "second_ivt_header" to
 * "image_copy_start", which involves FLASH_OFFSET, plus the first
 * ivt_header, the plugin code size itself recorded by "ivt2_header"
 */

second_ivt_offset:      .long (ivt2_header + 0x2C + FLASH_OFFSET)

/*
 * The following is the second IVT header plus the second boot data
 */
ivt2_header:            .long 0x0
app2_code_jump_v:       .long 0x0
reserv3:                .long 0x0
dcd2_ptr:               .long 0x0
boot_data2_ptr:         .long 0x0
self_ptr2:              .long 0x0
app_code_csf2:          .long 0x0
reserv4:                .long 0x0
boot_data2:             .long 0x0
image_len2:             .long 0x0
plugin2:                .long 0x0
