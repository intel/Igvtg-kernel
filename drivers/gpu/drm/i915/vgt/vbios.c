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

#include "vgt.h"
#include "vbios.h"

#define CHILD_DEV_FAKE_SIZE sizeof(struct child_devices)

static u8 child_dev_fake_dpb[CHILD_DEV_FAKE_SIZE] = {
	0x04, 0x00, 0xd6, 0x60, 0x00, 0x10, 0x10, 0x06,
	0x28, 0x14, 0x00, 0x20, 0x00, 0x00, 0x00, 0xd6,
	0x07, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00,
	0x07, 0x10, 0x01, 0x20, 0x01, 0x00, 0x00, 0x00,
	0x00
};

static u8 child_dev_fake_dpc[CHILD_DEV_FAKE_SIZE] = {
	0x40, 0x00, 0xd6, 0x60, 0x00, 0x10, 0x10, 0x06,
	0x3a, 0x14, 0x00, 0x20, 0x00, 0x00, 0x10, 0xd6,
	0x08, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
	0x07, 0x20, 0x01, 0x20, 0x02, 0x00, 0x00, 0x00,
	0x00
};

static u8 child_dev_fake_dpd[CHILD_DEV_FAKE_SIZE] = {
	0x20, 0x00, 0xd6, 0x60, 0x00, 0x10, 0x10, 0x06,
	0x4c, 0x14, 0x00, 0x20, 0x00, 0x00, 0x00, 0xd6,
	0x09, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00,
	0x07, 0x30, 0x01, 0x20, 0x03, 0x00, 0x00, 0x00,
	0x00,
};

static void child_dev_print(u8* bitstream, int size)
{
	int i;
	for(i=0; i < size; i+=8) {
	vgt_dbg(VGT_DBG_GENERIC,
	"VGT_VBIOS: %02x %02x %02x %02x %02x %02x %02x %02x\n",
		(i < size) ? bitstream[i+0] : 0xFF,
		(i+1 < size) ? bitstream[i+1] : 0xFF,
		(i+2 < size) ? bitstream[i+2] : 0xFF,
		(i+3 < size) ? bitstream[i+3] : 0xFF,
		(i+4 < size) ? bitstream[i+4] : 0xFF,
		(i+5 < size) ? bitstream[i+5] : 0xFF,
		(i+6 < size) ? bitstream[i+6] : 0xFF,
		(i+7 < size) ? bitstream[i+7] : 0xFF);
	}

	vgt_dbg(VGT_DBG_GENERIC, "VGT_VBIOS: %d bytes. End of print \n", size);
	vgt_dbg(VGT_DBG_GENERIC, "VGT_VBIOS: \n");
}

static void* get_block_by_id(struct bios_data_header *header, int id)
{
	void *curr = NULL;
	int offset;
	struct block_header *block = NULL;

	ASSERT(header != NULL);

	if (memcmp(header->signature, "BIOS_DATA_BLOCK", 15) != 0) {
		/* invalid bios_data_block */
		return NULL;
	}

	offset = header->header_size;

	while(offset < header->bdb_size) {
		block = (struct block_header*) ( ((u8*) header)+offset);

		/* find block by block ID */
		if (block->block_id == id) {
			curr = block;
			break;
		}
		else {
			/* search for next block */
			offset += block->block_size
				+ sizeof(struct block_header);
		}

	}

	return curr;
}

static struct child_devices*
child_dev_found_available_slot(struct child_devices* dev, int max_child_num)
{
#define EFP_PORT_B_SLOT 1
	int i;
	struct child_devices *child_dev = NULL;

	/* start from PORT_B, usually PORT_B/C/D takes slot: 1,2,3 */
	for (i=EFP_PORT_B_SLOT; i<max_child_num; i++) {

		child_dev = dev + i;
		if (child_dev->dev_type == 0) {
			/* Got available empty slot */
			break;
		}
	}

	return (i==max_child_num) ? NULL : child_dev;
}


static void
child_dev_set_capability(struct child_devices* child_dev)
{
	if (child_dev->efp_port <= EFP_HDMI_D) {
		/* update aux channel for HDMI B/C/D */
		child_dev->aux_channel = (child_dev->efp_port << 4);
		/* modify HDMI port to be DP port */
		child_dev->efp_port += (EFP_DPORT_B - EFP_HDMI_B);
	}

	/* set DP capable bit */
	child_dev->dev_type |= DEVTYPE_FLAG_DISPLAY_PORT;
	child_dev->is_dp_compatible = 1;

	/* set HDMI capable bit */
	child_dev->dev_type &= (~DEVTYPE_FLAG_NOT_HDMI);
	child_dev->is_hdmi_compatible = 1;

	/* set DVI capable bit */
	child_dev->dev_type |= DEVTYPE_FLAG_DVI;
	child_dev->is_dvi_compatible = 1;
	return;
}

static void
child_dev_insert_fake_port(struct child_devices* dev, int max_num, int efp_port)
{
	struct child_devices *child_dev;

	if((child_dev = child_dev_found_available_slot(dev, max_num)) == NULL) {
		return;
	}

	/* already got a available slot */
	switch(efp_port) {
	case EFP_HDMI_B:
	case EFP_DPORT_B:
		memcpy(child_dev, child_dev_fake_dpb,
			CHILD_DEV_FAKE_SIZE);
		break;
	case EFP_HDMI_C:
	case EFP_DPORT_C:
		memcpy(child_dev, child_dev_fake_dpc,
			CHILD_DEV_FAKE_SIZE);
		break;
	case EFP_HDMI_D:
	case EFP_DPORT_D:
		memcpy(child_dev, child_dev_fake_dpd,
			CHILD_DEV_FAKE_SIZE);
		break;
	default:
		break;
	}
}

/*
 * We modify opregion vbios data to indicate that we support full port
 * features: DP, HDMI, DVI
 */
bool vgt_prepare_vbios_general_definition(struct vgt_device *vgt)
{
	bool ret = true;
	struct vbt_header *header;
	struct bios_data_header *data_header;
	struct vbios_general_definitions *gendef;
	struct child_devices* child_dev;
	int child_dev_num = 0;
	int i;
	bool encoder_b_found = false;
	bool encoder_c_found = false;
	bool encoder_d_found = false;

	/* only valid for HSW */
	if (!IS_HSW(vgt->pdev)) {
		vgt_dbg(VGT_DBG_GENERIC, "Not HSW platform. Do nothing\n");
		return false;
	}

	header = (struct vbt_header*) (vgt->state.opregion_va + VBIOS_OFFSET);

	data_header = (struct bios_data_header*)
		(((u8*)header) + header->bios_data_offset);

	gendef = get_block_by_id(data_header, VBIOS_GENERAL_DEFINITIONS);
	if (gendef == NULL) {
		vgt_dbg(VGT_DBG_GENERIC,
			"VBIOS_GENERAL_DEFINITIONS block was not found. \n");
		return false;
	}

	child_dev_num = (gendef->block_header.block_size
		- sizeof(*gendef)
		+ sizeof(struct block_header))/ sizeof(struct child_devices);

	vgt_dbg(VGT_DBG_GENERIC,
		"VGT_VBIOS: block_size=%d child_dev_num=%d \n",
		gendef->block_header.block_size, child_dev_num);

	for (i=0; i<child_dev_num; i++) {
		child_dev = gendef->dev + i;

		/* print all VBT child dev structure */
		child_dev_print((u8*)child_dev,
			sizeof(struct child_devices));

		if (child_dev->dev_type == 0) {
			continue;
		}

		switch(child_dev->efp_port) {
		case EFP_HDMI_B:
			encoder_b_found = true;
			break;
		case EFP_HDMI_C:
			encoder_c_found = true;
			break;
		case EFP_HDMI_D:
			encoder_d_found = true;
			break;
		case EFP_DPORT_B:
			encoder_b_found = true;
			break;
		case EFP_DPORT_C:
			encoder_c_found = true;
			break;
		case EFP_DPORT_D:
			encoder_d_found = true;
			break;
		case EFP_DPORT_A:
			/* DPORT A is eDP, ignore */
			continue;
		default:
			/* not port description. Skip this child_dev */
			continue;
		}

		/* got valid PORT description */
		child_dev_set_capability(child_dev);

		vgt_dbg(VGT_DBG_GENERIC,
		"VGT_VBIOS: child_dev modified. child_dev[%d].dev_type=%04x \n",
			i, child_dev->dev_type);

		ret = true;
	}

	if (!encoder_b_found) {
		child_dev_insert_fake_port(gendef->dev,
			child_dev_num, EFP_DPORT_B);
	}

	if (!encoder_c_found) {
		child_dev_insert_fake_port(gendef->dev,
			child_dev_num, EFP_DPORT_C);
	}

	if (!encoder_d_found) {
		child_dev_insert_fake_port(gendef->dev,
			child_dev_num, EFP_DPORT_D);
	}

	return ret;
}

