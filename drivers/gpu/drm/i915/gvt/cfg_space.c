/*
 * Copyright(c) 2011-2016 Intel Corporation. All rights reserved.
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

#include "gvt.h"

static bool cfg_sci_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, int bytes)
{
	gvt_info("[vgt %d] Read SCI Trigger Register, bytes=%d value=0x%x",
			vgt->id, bytes, *(u16*)p_data);

	return true;
}

bool gvt_emulate_cfg_read(struct vgt_device *vgt, unsigned int offset, void *p_data, int bytes)
{
	ASSERT((offset + bytes) <= GVT_CFG_SPACE_SZ);
	memcpy(p_data, &vgt->state.cfg.space[offset], bytes);

	/* TODO: hooks */
	offset &= ~3;
	switch (offset) {
		case 0:
		case 4:
			break;
		case GVT_REG_CFG_SWSCI_TRIGGER:
			cfg_sci_read(vgt, offset, p_data, bytes);
			break;
		default:
			break;
	}
	return true;
}

bool gvt_emulate_cfg_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, int bytes)
{
	char *cfg_space = &vgt->state.cfg.space[0];
	u32 *cfg_reg, new;
	u64 size;
	u8 old_cmd, cmd_changed; /* we don't care the high 8 bits */
	bool rc = true;
	u32 low_mem_max_gpfn;

	ASSERT ((off + bytes) <= GVT_CFG_SPACE_SZ);
	cfg_reg = (u32*)(cfg_space + (off & ~3));
	switch (off & ~3) {
		case GVT_REG_CFG_VENDOR_ID:
			low_mem_max_gpfn = *(u32 *)p_data;
			gvt_info("low_mem_max_gpfn: 0x%x", low_mem_max_gpfn);
			if (bytes != 4 ||
				low_mem_max_gpfn >= (1UL << (32-PAGE_SHIFT))) {
				gvt_err("invalid low_mem_max_gpfn!");
				break;
			}
			if (vgt->low_mem_max_gpfn == 0)
				vgt->low_mem_max_gpfn = low_mem_max_gpfn;
			break;

		case GVT_REG_CFG_COMMAND:
			old_cmd = vgt->state.cfg.space[off];
			cmd_changed = old_cmd ^ (*(u8*)p_data);
			memcpy (&vgt->state.cfg.space[off], p_data, bytes);
			if (cmd_changed & _REGBIT_CFG_COMMAND_MEMORY) {
				if (old_cmd & _REGBIT_CFG_COMMAND_MEMORY) {
					 gvt_hvm_map_aperture(vgt, 0);
				} else {
					if(!vgt->state.cfg.bar_mapped[1]) {
						gvt_hvm_map_aperture(vgt, 1);
						gvt_hvm_set_trap_area(vgt, 1);
					}
				}
			} else {
				gvt_dbg(GVT_DBG_CORE, "need to trap the PIO BAR? "
					"old_cmd=0x%x, cmd_changed=%0x",
					old_cmd, cmd_changed);
			}
			break;
		case GVT_REG_CFG_SPACE_BAR0:	/* GTTMMIO */
		case GVT_REG_CFG_SPACE_BAR1:	/* GMADR */
		case GVT_REG_CFG_SPACE_BAR2:	/* IO */
			ASSERT((bytes == 4) && (off & 3) == 0);
			new = *(u32 *)p_data;
			gvt_info("Programming bar 0x%x with 0x%x", off, new);
			size = vgt->state.cfg.bar_size[(off - GVT_REG_CFG_SPACE_BAR0)/8];
			if (new == 0xFFFFFFFF) {
				/*
				 * Power-up software can determine how much address
				 * space the device requires by writing a value of
				 * all 1's to the register and then reading the value
				 * back. The device will return 0's in all don't-care
				 * address bits.
				 */
				new = new & ~(size-1);
				if ((off & ~3) == GVT_REG_CFG_SPACE_BAR1)
					gvt_hvm_map_aperture(vgt, 0);
				if ((off & ~3) == GVT_REG_CFG_SPACE_BAR0)
					gvt_hvm_set_trap_area(vgt, 0);
				gvt_pci_bar_write_32(vgt, off, new);
			} else {
				if ((off & ~3) == GVT_REG_CFG_SPACE_BAR1)
					gvt_hvm_map_aperture(vgt, 0);
				if ((off & ~3) == GVT_REG_CFG_SPACE_BAR0)
					gvt_hvm_set_trap_area(vgt, 0);
				gvt_pci_bar_write_32(vgt, off, new);
				if ((off & ~3) == GVT_REG_CFG_SPACE_BAR1)
					gvt_hvm_map_aperture(vgt, 1);
				if ((off & ~3) == GVT_REG_CFG_SPACE_BAR0)
					gvt_hvm_set_trap_area(vgt, 1);
			}
			break;

		case GVT_REG_CFG_SPACE_MSAC:
			gvt_info("Guest write MSAC %x, %d: Not supported yet",
					*(char *)p_data, bytes);
			break;

		case GVT_REG_CFG_SWSCI_TRIGGER:
			new = *(u32 *)p_data;
			gvt_emulate_opregion_request(vgt, new);
			break;

		case GVT_REG_CFG_OPREGION:
			new = *(u32 *)p_data;
			if (!gvt_init_instance_opregion(vgt, new))
				return false;

			memcpy(&vgt->state.cfg.space[off], p_data, bytes);
			break;

		case GVT_REG_CFG_SPACE_BAR1+4:
		case GVT_REG_CFG_SPACE_BAR0+4:
		case GVT_REG_CFG_SPACE_BAR2+4:
			ASSERT((bytes == 4) && (off & 3) == 0);
			new = *(u32 *)p_data;
			gvt_info("Programming bar 0x%x with 0x%x", off, new);
			size = vgt->state.cfg.bar_size[(off - (GVT_REG_CFG_SPACE_BAR0 + 4))/8];
			/* for 32bit mode bar it returns all-0 in upper 32 bit, for 64bit
			 * mode bar it will calculate the size with lower 32bit and return
			 * the corresponding value
			 */
			if (new == 0xFFFFFFFF) {
				if (GVT_GET_BITS(*(cfg_space + off - 4), 2, 1) == 2)
					new &= ~(size-1) >> 32;
				else
					new = 0;
				*cfg_reg = new;
			} else {
				if ((off & ~3) == GVT_REG_CFG_SPACE_BAR1 + 4)
					gvt_hvm_map_aperture(vgt, 0);
				if ((off & ~3) == GVT_REG_CFG_SPACE_BAR0 + 4)
					gvt_hvm_set_trap_area(vgt, 0);
				*cfg_reg = new;
				if ((off & ~3) == GVT_REG_CFG_SPACE_BAR1 + 4)
					gvt_hvm_map_aperture(vgt, 1);
				if ((off & ~3) == GVT_REG_CFG_SPACE_BAR0 + 4)
					gvt_hvm_set_trap_area(vgt, 1);
			}
			break;

		case 0x90:
		case 0x94:
		case 0x98:
			gvt_info("write to MSI capa(%x) with val (%x)", off, *(u32 *)p_data);
		default:
			memcpy(&vgt->state.cfg.space[off], p_data, bytes);
			break;
	}
	return rc;
}
