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

int gvt_hvm_map_aperture(struct vgt_device *vgt, int map)
{
	char *cfg_space = &vgt->state.cfg.space[0];
	uint64_t bar_s;
	int r, nr_mfns;
	unsigned long first_gfn, first_mfn;

	if (!gvt_pci_mmio_is_enabled(vgt))
		return 0;

	/* guarantee the sequence of map -> unmap -> map -> unmap */
	if (map == vgt->state.cfg.bar_mapped[1])
		return 0;

	cfg_space += GVT_REG_CFG_SPACE_BAR1;	/* APERTUR */
	if (GVT_GET_BITS(*cfg_space, 2, 1) == 2){
		/* 64 bits MMIO bar */
		bar_s = * (uint64_t *) cfg_space;
	} else {
		/* 32 bits MMIO bar */
		bar_s = * (uint32_t*) cfg_space;
	}

	first_gfn = (bar_s + gvt_aperture_offset(vgt)) >> PAGE_SHIFT;
	first_mfn = gvt_aperture_base(vgt) >> PAGE_SHIFT;
	nr_mfns = gvt_aperture_sz(vgt) >> PAGE_SHIFT;

	printk("%s: domid=%d gfn_s=0x%lx mfn_s=0x%lx nr_mfns=0x%x\n", map==0? "remove_map":"add_map",
		vgt->vm_id, first_gfn, first_mfn, nr_mfns);

	r = hypervisor_map_mfn_to_gpfn(vgt, first_gfn, first_mfn,
		nr_mfns, map, GVT_MAP_APERTURE);

	if (r != 0)
		printk(KERN_ERR "gvt_hvm_map_aperture fail with %d!\n", r);
	else
		vgt->state.cfg.bar_mapped[1] = map;

	return r;
}

/*
 * Zap the GTTMMIO bar area for vGT trap and emulation.
 */
int gvt_hvm_set_trap_area(struct vgt_device *vgt, int map)
{
	char *cfg_space = &vgt->state.cfg.space[0];
	uint64_t bar_s, bar_e;

	if (!gvt_pci_mmio_is_enabled(vgt))
		return 0;

	cfg_space += GVT_REG_CFG_SPACE_BAR0;
	if (GVT_GET_BITS(*cfg_space, 2, 1) == 2) {
		/* 64 bits MMIO bar */
		bar_s = * (uint64_t *) cfg_space;
	} else {
		/* 32 bits MMIO bar */
		bar_s = * (uint32_t*) cfg_space;
	}

	bar_s &= ~0xF; /* clear the LSB 4 bits */
	bar_e = bar_s + vgt->state.cfg.bar_size[0] - 1;

	return hypervisor_set_trap_area(vgt, bar_s, bar_e, map);
}
