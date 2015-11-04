/*
 * vGT virtual video BIOS data block parser
 *
 * Copyright(c) 2011-2014 Intel Corporation. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _VGT_VBIOS_H_
#define _VGT_VBIOS_H_

/*
 * video BIOS data block ID defination, each block has an ID and size.
 * used by get_block_by_id()
 */
#define VBIOS_OFFSET 0x400
#define VBIOS_GENERAL_FEATURES  1
#define VBIOS_GENERAL_DEFINITIONS  2

struct vbt_header {
	u8 product_string[20]; /* string of "$VBT HASWELL"*/
	u16 version;
	u16 header_size;
	u16 vbt_size;
	u8  checksum;
	u8  reserved;
	u32 bios_data_offset;	/* bios_data beginning offset in bytes */
	u32 aim_data_offset[4];
};

struct bios_data_header {
	u8 signature[16];	/* signature of 'BIOS_DATA_BLOCK' */
	u16 version;
	u16 header_size;	/* in bytes */
	u16 bdb_size;		/* in bytes */
};

struct block_header{
	u8  block_id;		/* data block ID */
	u16 block_size;		/* current data block size */
}__attribute__((packed));	/* packed struct, not to align to 4 bytes */


#define DEVTYPE_FLAG_DISPLAY_PORT 0x0004 /* BIT2 */
#define DEVTYPE_FLAG_DVI 0x0010 /* BIT4 */
#define DEVTYPE_FLAG_NOT_HDMI 0x0800 /* BIT11 */

enum efp_port_type{
	INVALID = 0,
	EFP_HDMI_B = 1,
	EFP_HDMI_C = 2,
	EFP_HDMI_D = 3,
	EFP_DPORT_B = 7,
	EFP_DPORT_C = 8,
	EFP_DPORT_D = 9,
	EFP_DPORT_A = 10,
	EFP_MIPI_A = 21,
	EFP_MIPI_C = 23
};

/* VBIOS version >= 165 integrated EFP (HDMI/DP) structure.
 * Valid for HSW/VLV
 */
struct child_devices
{
	u8 reserved[2];
	u16 dev_type;
	u8 reserved1[12];

	u8 efp_port;
	u8 reserved2[7];

	/* compatibility_flag */
	u8 is_hdmi_compatible:1;
	u8 is_dp_compatible:1;
	u8 is_dvi_compatible:1;
	u8 reservedbit:5;
	/* end of compatibility byte */
	u8 aux_channel;
	u8 reserved3[7];
}__attribute__((packed));


struct vbios_general_definitions
{
	struct block_header block_header;
	u8 crt_ddc_pin; 	/* CRT DDC GPIO pin*/
	u8 dpms;  		/* DPMS bits */
	u16 boot_dev;		/* boot device bits */
	u8 child_dev_size;	/* child_devices size */
	struct child_devices dev[0]; /* a block could be many child devices */
}__attribute__((packed));


#endif	/* _VGT_VBIOS_H_ */
