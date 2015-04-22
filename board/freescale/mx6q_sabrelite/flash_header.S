/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc.
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <config.h>
#include <asm/arch/mx6.h>

#ifdef	CONFIG_FLASH_HEADER
#ifndef CONFIG_FLASH_HEADER_OFFSET
# error "Must define the offset of flash header"
#endif

#ifndef CONFIG_FLASH_PLUG_IN

#define CPU_2_BE_32(l) \
       ((((l) & 0x000000FF) << 24) | \
	(((l) & 0x0000FF00) << 8)  | \
	(((l) & 0x00FF0000) >> 8)  | \
	(((l) & 0xFF000000) >> 24))

#define MXC_DCD_ITEM(i, addr, val)   \
dcd_node_##i:                        \
        .word CPU_2_BE_32(addr) ;     \
        .word CPU_2_BE_32(val)  ;     \

.section ".text.flasheader", "x"
	b	_start
	.org	CONFIG_FLASH_HEADER_OFFSET

ivt_header:       .word 0x402000D1 /* Tag=0xD1, Len=0x0020, Ver=0x40 */
app_code_jump_v:  .word _start
reserv1:          .word 0x0
dcd_ptr:          .word dcd_hdr
boot_data_ptr:	  .word boot_data
self_ptr:         .word ivt_header
app_code_csf:     .word 0x0
reserv2:          .word 0x0

boot_data:        .word TEXT_BASE
image_len:        .word _end_of_copy  - TEXT_BASE + CONFIG_FLASH_HEADER_OFFSET
plugin:           .word 0x0

// steven: only for mfg_tool, ref. mx6q_sabrelite_mfg.h
#ifdef VAB820_MX6Q
# steven: old value, Len=90
  dcd_hdr:          .word 0x40D802D2 /* Tag=0xD2, Len=90*8 + 4 + 4, Ver=0x40 */
  write_dcd_cmd:    .word 0x04D402CC /* Tag=0xCC, Len=90*8 + 4, Param=0x04 */

#  dcd_hdr:          .word 0x40A002D2 /* Tag=0xD2, Len=83*8 + 4 + 4, Ver=0x40 */
#  write_dcd_cmd:    .word 0x049C02CC /* Tag=0xCC, Len=83*8 + 4, Param=0x04 */

# steven: vab820_mx6q_1g, vab820_mx6q_2g must write Len=83
  #ifdef VAB820_4G
  #include "vab820_mx6q_4g.cfg"
  #elif defined(VAB820_2G)
  // imx6q_2g_ram
  #include "vab820_mx6q_2g.cfg"
  #else
  // imx6q_1g_ram
  #include "vab820_mx6q_1g.cfg"
  #endif

// steven: only for mfg_tool, ref. mx6q_sabrelite_mfg.h
#elif  defined(VAB820_MX6DL)

  dcd_hdr:          .word 0x40A002D2 /* Tag=0xD2, Len=83*8 + 4 + 4, Ver=0x40 */
  write_dcd_cmd:    .word 0x049C02CC /* Tag=0xCC, Len=83*8 + 4, Param=0x04 */

  #ifdef VAB820_2G
  // imx6dl_2g_ram
  #include "vab820_mx6dl_2g.cfg"
  #else
  // imx6dl_1g_ram
  #include "vab820_mx6dl_1g.cfg"
  #endif

#endif /* end of MX6 */


#else /* CONFIG_FLASH_PLUG_IN */


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



.section ".text.flasheader", "x"
origin:
	b	_start
	.org	CONFIG_FLASH_HEADER_OFFSET

/* First IVT to copy the plugin that initializes the system into OCRAM */
ivt_header:             .long 0x402000D1    /*Tag=0xD1, Len=0x0020, Ver=0x40 */
app_code_jump_v:        .long IRAM_FREE_START + (plugin_start - origin)    /* Plugin entry point, address after the second IVT table */
reserv1:                .long 0x0
dcd_ptr:                .long 0x0
boot_data_ptr:          .long IRAM_FREE_START + (boot_data - origin) /*0x00907420*/
self_ptr:               .long IRAM_FREE_START + (ivt_header - origin)
app_code_csf:           .long 0x0
reserv2:                .long 0x0

boot_data:              .long IRAM_FREE_START
image_len:              .long 16*1024  /* plugin can be upto 16KB in size */
plugin:                 .long 0x1  /* Enable plugin flag */

/* Second IVT to give entry point into the bootloader copied to DDR */
ivt2_header:            .long 0x402000D1    /*Tag=0xD1, Len=0x0020, Ver=0x40 */
app2_code_jump_v:       .long _start       /* Entry point for uboot */
reserv3:                .long 0x0
dcd2_ptr:               .long 0x0
boot_data2_ptr:         .long boot_data2
self_ptr2:              .long ivt2_header
app_code_csf2:          .long 0x0
reserv4:                .long 0x0

boot_data2:             .long TEXT_BASE
image_len2:             .long _end_of_copy  - TEXT_BASE + CONFIG_FLASH_HEADER_OFFSET
plugin2:                .long 0x0

/* Here starts the plugin code */
plugin_start:
/* Save the return address and the function arguments */
    push    {r0-r4, lr}


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
/*
    The following is to fill in those arguments for this ROM function
    pu_irom_hwcnfg_setup(void **start, size_t *bytes, const void *boot_data)

    This function is used to copy data from the storage media into DDR.

    start - Initial (possibly partial) image load address on entry.  Final
    image load address on exit.
    bytes - Initial (possibly partial) image size on entry.  Final image size
    on exit.
    boot_data - Initial @ref ivt Boot Data load address.
*/
    adr r0, DDR_DEST_ADDR
    adr r1, COPY_SIZE
    adr r2, BOOT_DATA

/*
 * check the _pu_irom_api_table for the address
 * pu_irom_hwcnfg_setup is in 0x1fb5  ERIC : < what is the address in Rigel >
 */
before_calling_rom___pu_irom_hwcnfg_setup:
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
    ldr r5, DDR_DEST_ADDR
    str r5, [r0]
    ldr r5, COPY_SIZE
    str r5, [r1]
    mov r5, #0x400  /* Point to the second IVT table at offset 0x42C */
    add r5, r5, #0x2C
    str r5, [r2]
    mov r0, #1

    bx lr          /* return back to ROM code */

DDR_DEST_ADDR:    .word   TEXT_BASE
COPY_SIZE:        .word   _end_of_copy  - TEXT_BASE + CONFIG_FLASH_HEADER_OFFSET
BOOT_DATA:        .word   TEXT_BASE
                  .word   _end_of_copy  - TEXT_BASE + CONFIG_FLASH_HEADER_OFFSET
                  .word   0
/*********************************************************************/




#endif

#endif
