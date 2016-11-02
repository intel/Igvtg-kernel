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

static u16
get_block_size(const void *p)
{
	u16 *block_ptr, block_size;

	block_ptr = (u16 *)((char *)p - 2);
	block_size = *block_ptr;
	return block_size;
}

static union child_device_config *
get_child_device_ptr(const struct bdb_general_definitions *p_defs, int i)
{
	return (void *) &p_defs->devices[i * p_defs->child_dev_size];
}

/* only DP is handled in full display virtualization.*/
static void
set_child_device_capability(union child_device_config *p_child, int dvo_port)
{
	/* update aux channel for DP B/C/D */
	p_child->raw[25] = DP_AUX_B + ((dvo_port - DVO_PORT_DPB) << 4);

	/* capability setting */
	p_child->raw[27] = 0x20;

	/* DP port capable bit */
	p_child->common.dvo_port = dvo_port;

	/* DP device type bit */
	p_child->common.device_type = DEVICE_TYPE_DISPLAYPORT_OUTPUT;

	/* iboost_level bit */
	p_child->common.flags_1 &= ~IBOOST_ENABLE;

	return;
}

static void *
find_bdb_section(void *_bdb, int section_id)
{
	struct bdb_header *bdb = _bdb;
	u8 *base = _bdb;
	int index = 0;
	u32 total, current_size;
	u8 current_id;

	/* skip to first section */
	index += bdb->header_size;
	total = bdb->bdb_size;

	/* walk the sections looking for section_id */
	while (index + 3 < total) {
		current_id = *(base + index);
		index++;

		current_size = *((u16 *)(base + index));
		index += 2;

		/* The MIPI Sequence Block v3+ has a separate size field. */
		if (current_id == BDB_MIPI_SEQUENCE && *(base + index) >= 3)
			current_size = *((u32 *)(base + index + 1));

		if (index + current_size > total)
			return NULL;

		if (current_id == section_id)
			return base + index;

		index += current_size;
	}

	return NULL;
}

bool vgt_prepare_vbios_general_definition(struct vgt_device *vgt)
{
	struct vbt_header *vbt_header;
	struct bdb_header *data_header;
	struct bdb_general_definitions *p_defs;
	union child_device_config *p_child;
	int i, child_device_num;
	u16 block_size;

	if (vgt->vm_id == 0)
		return true;

	vbt_header = (struct vbt_header *)
			(vgt->state.opregion_va + VBIOS_OFFSET);

	if (memcmp(vbt_header->signature, "$VBT", 4)) {
		vgt_dbg(VGT_DBG_GENERIC, "VBT invalid signature.\n");
		return false;
	}
	data_header = (struct bdb_header *)
			(((u8 *)vbt_header) + vbt_header->bdb_offset);

	p_defs = find_bdb_section(data_header, BDB_GENERAL_DEFINITIONS);
	if (!p_defs) {
		vgt_err("No general definition block is found.\n");
		return false;
	}

	if (p_defs->child_dev_size < sizeof(struct old_child_dev_config)) {
		vgt_err("Child device config size %u is too small.\n",
			p_defs->child_dev_size);
		return false;
	}

	/* get the block size of general definitions */
	block_size = get_block_size(p_defs);
	/* get the number of child device */
	child_device_num = (block_size - sizeof(*p_defs)) /
				p_defs->child_dev_size;

	vgt_dbg(VGT_DBG_GENERIC,
		"VGT_VBIOS: block_size=%d child_dev_num=%d \n",
		block_size, child_device_num);

	for (i = 0; i < child_device_num; i++) {

		p_child = get_child_device_ptr(p_defs, i);

		memset(&p_child->common, 0, p_defs->child_dev_size);

		if (i >= preallocated_monitor_to_guest)
			continue;

		set_child_device_capability(p_child, DVO_PORT_DPB+i);

		vgt_dbg(VGT_DBG_GENERIC,
		"VGT_VBIOS: child_dev modified. child_dev[%d].dev_type=%04x\n",
		i, p_child->common.device_type);
	}

	return true;
}
