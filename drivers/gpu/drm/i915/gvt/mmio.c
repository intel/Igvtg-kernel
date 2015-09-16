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

static inline unsigned int get_device_type(struct pgt_device *pdev)
{
	if (IS_BROADWELL(pdev->dev_priv))
		return D_BDW;
	return 0;
}

static inline bool match_device(struct pgt_device *pdev, struct gvt_reg_info *info)
{
	return info->device & get_device_type(pdev);
}

static void save_initial_mmio_state(struct pgt_device *pdev,
		struct gvt_reg_info *info, int num)
{
	u32 *mmio = pdev->initial_mmio_state;
	int i, j;

	for (i = 0; i < num; i++, info++) {
		if (!match_device(pdev, info))
			continue;

		for (j = 0; j < info->size; j += 4)
			mmio[REG_INDEX(info->reg + j)] =
				gvt_mmio_read(pdev, info->reg + j);
	}

	gvt_dbg_core("save %d registers as initial mmio values", num);
}

static void patch_display_mmio_state(u32 *mmio_array)
{
	gvt_dbg_core("patch display initial mmio state");

	/* TODO: vgt_dpy_init_modes. */
	mmio_array[REG_INDEX(_WRPLL_CTL1)] &= ~(1 << 31);
	mmio_array[REG_INDEX(_WRPLL_CTL2)] &= ~(1 << 31);
}

static void patch_initial_mmio_state(struct pgt_device *pdev)
{
	u32 *mmio_array = pdev->initial_mmio_state;

	gvt_dbg_core("patch initial mmio state");

	/* customize the initial MMIO
	 * 1, GMBUS status
	 * 2, Initial port status.
	 */

	/* GMBUS2 has an in-use bit as the hw semaphore, and we should recover
	 * it after the snapshot.
	 */
	mmio_array[REG_INDEX(_PCH_GMBUS2)] &= ~0x8000;

	gvt_mmio_write(pdev, _PCH_GMBUS2,
			gvt_mmio_read(pdev, _PCH_GMBUS2) | 0x8000);

	patch_display_mmio_state(mmio_array);
}

void gvt_clean_initial_mmio_state(struct pgt_device *pdev)
{
	if (pdev->initial_mmio_state) {
		vfree(pdev->initial_mmio_state);
		pdev->initial_mmio_state = NULL;
	}
}

bool gvt_setup_initial_mmio_state(struct pgt_device *pdev)
{
	gvt_dbg_core("setup initial mmio state");

	pdev->initial_mmio_state = vzalloc(pdev->mmio_size);
	if (!pdev->initial_mmio_state) {
		gvt_info("fail to allocate initial mmio state");
		return false;
	}

	gvt_dbg_core("save generic initial mmio state");

	save_initial_mmio_state(pdev, gvt_general_reg_info, gvt_get_reg_num(D_ALL));

	if(IS_BROADWELL(pdev->dev_priv)) {
		gvt_dbg_core("save broadwell initial mmio state");
		save_initial_mmio_state(pdev, gvt_broadwell_reg_info, gvt_get_reg_num(D_BDW));
	}

	patch_initial_mmio_state(pdev);

	return true;
}

static void add_mmio_entry(struct pgt_device *pdev, struct gvt_mmio_entry *e)
{
        hash_add(pdev->mmio_table, &e->hlist, e->base);
}

static struct gvt_mmio_entry *find_mmio_entry(struct pgt_device *pdev, unsigned int base)
{
        struct gvt_mmio_entry *e;

        hash_for_each_possible(pdev->mmio_table, e, hlist, base) {
                if (base == e->base)
                        return e;
        }
        return NULL;
}

void remove_mmio_entry(struct pgt_device *pdev, unsigned int base)
{
        struct gvt_mmio_entry *e;

        if ((e = find_mmio_entry(pdev, base))) {
                hash_del(&e->hlist);
                kfree(e);
        }
}

void free_mmio_table(struct pgt_device *pdev)
{
        struct hlist_node *tmp;
        struct gvt_mmio_entry *e;
        int i;

        hash_for_each_safe(pdev->mmio_table, i, tmp, e, hlist)
                kfree(e);

        hash_init(pdev->mmio_table);
}

bool register_mmio_handler(struct pgt_device *pdev, unsigned int start, int bytes,
	gvt_mmio_handler_t read, gvt_mmio_handler_t write)
{
	unsigned int i, end;
	struct gvt_mmio_entry *e;

	end = start + bytes -1;

	ASSERT((start & 3) == 0);
	ASSERT(((end+1) & 3) == 0);

	for (i = start; i < end; i += 4) {
		e = find_mmio_entry(pdev, i);
		if (e) {
			e->read = read;
			e->write = write;
			continue;
		}

		e = kzalloc(sizeof(*e), GFP_KERNEL);
		if (e == NULL) {
			gvt_err("fail to allocate memory for mmio entry");
			return false;
		}

		e->base = i;

		/*
		 * Win7 GFX driver uses memcpy to access the GVT PVINFO regs,
		 * hence align_bytes can be 1.
		 */
		if (start >= VGT_PVINFO_PAGE &&
			start < VGT_PVINFO_PAGE + VGT_PVINFO_SIZE)
			e->align_bytes = 1;
		else
			e->align_bytes = 4;

		e->read = read;
		e->write = write;
		INIT_HLIST_NODE(&e->hlist);
		add_mmio_entry(pdev, e);
	}
	return true;
}

static void set_reg_attribute(struct pgt_device *pdev,
		u32 reg, struct gvt_reg_info *info)
{
	/* ensure one entry per reg */
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);

	if (reg_is_tracked(pdev, reg))
		return;

	if (info->flags & GVT_REG_ADDR_FIX) {
		if (!info->addr_mask)
			gvt_info("ZERO addr fix mask for %x", reg);

		reg_set_addr_fix(pdev, reg, info->addr_mask);
		/* set the default range size to 4, might be updated later */
		reg_aux_addr_size(pdev, reg) = 4;
	}

	if (info->flags & GVT_REG_MODE_CTL)
		reg_set_mode_ctl(pdev, reg);
	if (info->flags & GVT_REG_VIRT)
		reg_set_virt(pdev, reg);
	if (info->flags & GVT_REG_HW_STATUS)
		reg_set_hw_status(pdev, reg);

	reg_set_tracked(pdev, reg);
}

static bool setup_reg_info(struct pgt_device *pdev, struct gvt_reg_info *info, int num)
{
	int i, count = 0, total = 0;
	u32 reg;

	for (i = 0; i < num; i++, info++) {
		if (!match_device(pdev, info))
			continue;

		count++;

		for (reg = info->reg;
				reg < info->reg + info->size;
				reg += 4) {
			set_reg_attribute(pdev, reg, info);
			total++;
		}

		if (info->read || info->write)
			if (!register_mmio_handler(pdev, info->reg, info->size,
					info->read, info->write))
				return false;
	}

	gvt_info("total register tracked %d, total mmio entry %d",
		count, total);

	return true;
}

void gvt_clean_mmio_emulation_state(struct pgt_device *pdev)
{
	free_mmio_table(pdev);
}

bool gvt_setup_mmio_emulation_state(struct pgt_device *pdev)
{
        struct gvt_mmio_entry *e;

	gvt_dbg_core("setup generic register info");

	if (!setup_reg_info(pdev, gvt_general_reg_info, gvt_get_reg_num(D_ALL)))
		goto err;

	if(IS_BROADWELL(pdev->dev_priv)) {
		gvt_dbg_core("setup broadwell register info");

		if (!setup_reg_info(pdev, gvt_broadwell_reg_info, gvt_get_reg_num(D_BDW)))
			goto err;
	}

	/* GDRST can be accessed by byte */
	e = find_mmio_entry(pdev, _GEN6_GDRST);
	if (e)
		e->align_bytes = 1;

	return true;
err:
	gvt_clean_mmio_emulation_state(pdev);
	return false;
}
