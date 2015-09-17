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
#include "i915_vgpu.h"

static void destroy_virtual_mmio_state(struct vgt_device *vgt)
{
	struct gvt_virtual_device_state *state = &vgt->state;

	if (state->mmio.vreg) {
		vfree(state->mmio.vreg);
		state->mmio.vreg = NULL;
	}
	if (state->mmio.sreg) {
		vfree(state->mmio.sreg);
		state->mmio.sreg = NULL;
	}
}

static bool create_virtual_mmio_state(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_virtual_device_state *state = &vgt->state;

	state->mmio.vreg = vzalloc(pdev->mmio_size);
	state->mmio.sreg = vzalloc(pdev->mmio_size);

	if (state->mmio.vreg == NULL || state->mmio.sreg == NULL ) {
		gvt_err("fail to allocate memory for virtual states.");
		goto err;
	}

	gvt_init_shadow_mmio_register(vgt);
	gvt_init_virtual_mmio_register(vgt);

	return true;
err:
	destroy_virtual_mmio_state(vgt);
	return false;
}

static void init_virtual_cfg_space_state(struct vgt_device *vgt,
	struct gvt_instance_info *info)
{
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_virtual_device_state *state = &vgt->state;
	int i;

	char *cfg_space;
	u16 *gmch_ctl;

	cfg_space = state->cfg.space;

	memcpy(cfg_space, pdev->initial_cfg_space, GVT_CFG_SPACE_SZ);
	cfg_space[GVT_REG_CFG_SPACE_MSAC] = state->cfg.bar_size[1];

	if (info->primary == 0 || ((info->primary == -1) && !gvt.primary)) {
		cfg_space[GVT_REG_CFG_CLASS_CODE] = GVT_PCI_CLASS_VGA;
		cfg_space[GVT_REG_CFG_SUB_CLASS_CODE] = GVT_PCI_CLASS_VGA_OTHER;
		cfg_space[GVT_REG_CFG_CLASS_PROG_IF] = GVT_PCI_CLASS_VGA_OTHER;
	}

	/* Show guest that there isn't any stolen memory.*/
	gmch_ctl = (u16 *)(cfg_space + _REG_GMCH_CONTROL);
	*gmch_ctl &= ~(_REGBIT_BDW_GMCH_GMS_MASK << _REGBIT_BDW_GMCH_GMS_SHIFT);

	gvt_pci_bar_write_32(vgt, GVT_REG_CFG_SPACE_BAR1, phys_aperture_base(pdev));

	cfg_space[GVT_REG_CFG_COMMAND] &= ~(_REGBIT_CFG_COMMAND_IO |
			_REGBIT_CFG_COMMAND_MEMORY |
			_REGBIT_CFG_COMMAND_MASTER);

	/* Clear the bar upper 32bit and let hvmloader to assign the new value */
	memset(&cfg_space[GVT_REG_CFG_SPACE_BAR0 + 4], 0, 4);
	memset(&cfg_space[GVT_REG_CFG_SPACE_BAR1 + 4], 0, 4);

	state->cfg.bar_size[0] = pdev->bar_size[0];	/* MMIOGTT */
	state->cfg.bar_size[1] = pdev->bar_size[1];
	state->cfg.bar_size[2] = pdev->bar_size[2];	/* PIO */
	state->cfg.bar_size[3] = pdev->bar_size[3];	/* ROM */

	for (i = 0; i < GVT_BAR_NUM; i++)
		state->cfg.bar_mapped[i] = false;
}

static void destroy_virtual_gm_state(struct vgt_device *vgt)
{
	gvt_clean_vgtt(vgt);
	gvt_free_gm_and_fence_resource(vgt);
}

static void populate_pvinfo_page(struct vgt_device *vgt)
{
	/* setup the ballooning information */
	__vreg64(vgt, _vgtif_reg(magic)) = VGT_MAGIC;
	__vreg(vgt, _vgtif_reg(version_major)) = 1;
	__vreg(vgt, _vgtif_reg(version_minor)) = 0;
	__vreg(vgt, _vgtif_reg(display_ready)) = 0;
	__vreg(vgt, _vgtif_reg(vgt_id)) = vgt->id;
	__vreg(vgt, _vgtif_reg(avail_rs.mappable_gmadr.base)) = gvt_visible_gm_base(vgt);
	__vreg(vgt, _vgtif_reg(avail_rs.mappable_gmadr.size)) = gvt_aperture_sz(vgt);
	__vreg(vgt, _vgtif_reg(avail_rs.nonmappable_gmadr.base)) = gvt_hidden_gm_base(vgt);
	__vreg(vgt, _vgtif_reg(avail_rs.nonmappable_gmadr.size)) = gvt_hidden_gm_sz(vgt);

	__vreg(vgt, _vgtif_reg(avail_rs.fence_num)) = gvt_fence_sz(vgt);
	gvt_info("filling VGT_PVINFO_PAGE for dom%d:"
			"   visable_gm_base=0x%llx, size=0x%llx"
			"   hidden_gm_base=0x%llx, size=0x%llx"
			"   fence_base=%d, num=%d",
			vgt->id,
			gvt_visible_gm_base(vgt), gvt_aperture_sz(vgt),
			gvt_hidden_gm_base(vgt), gvt_hidden_gm_sz(vgt),
			gvt_fence_base(vgt), gvt_fence_sz(vgt));

	ASSERT(sizeof(struct vgt_if) == VGT_PVINFO_SIZE);
}

static bool create_virtual_gm_state(struct vgt_device *vgt,
		struct gvt_instance_info *info)
{
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_virtual_device_state *state = &vgt->state;

	if (gvt_alloc_gm_and_fence_resource(vgt, info) < 0) {
		gvt_err("fail to allocate graphics memory and fence");
		return false;
	}

	state->gm.aperture_offset = aperture_2_gm(pdev, state->gm.aperture_base);
	state->gm.aperture_base_va = phys_aperture_vbase(pdev) + state->gm.aperture_offset;

	populate_pvinfo_page(vgt);

	if (!gvt_init_vgtt(vgt)) {
		gvt_err("fail to init vgtt");
		return false;
	}

	return true;
}

static void destroy_virtual_device_state(struct vgt_device *vgt)
{
	gvt_clean_vgtt(vgt);
	destroy_virtual_mmio_state(vgt);
	destroy_virtual_gm_state(vgt);
}

static bool create_virtual_device_state(struct vgt_device *vgt,
		struct gvt_instance_info *info)
{
	if (!create_virtual_mmio_state(vgt))
		return false;

	if (!create_virtual_gm_state(vgt, info))
		return false;

	init_virtual_cfg_space_state(vgt, info);

	return true;
}

void gvt_destroy_instance(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;

	mutex_lock(&pdev->lock);
	gvt_set_instance_offline(vgt);
	if (vgt->id != -1)
		idr_remove(&pdev->instance_idr, vgt->id);
	mutex_unlock(&pdev->lock);

	hypervisor_hvm_exit(vgt);
	destroy_virtual_device_state(vgt);
	vfree(vgt);
}

struct vgt_device *gvt_create_instance(struct pgt_device *pdev,
		struct gvt_instance_info *info)
{
	struct vgt_device *vgt = NULL;
	int id;

	gvt_info("vm_id=%d, low_gm_sz=%dMB, high_gm_sz=%dMB, fence_sz=%d",
		info->domid, info->low_gm_sz, info->high_gm_sz, info->fence_sz);

	vgt = vzalloc(sizeof(*vgt));
	if (vgt == NULL) {
		gvt_err("fail to allocate memory for instance.");
		return NULL;
	}

	mutex_lock(&pdev->lock);

	gvt_set_instance_offline(vgt);
	id = idr_alloc(&pdev->instance_idr, vgt, 1, GVT_MAX_VGPU - 1, GFP_KERNEL);
	if (id < 0) {
		gvt_err("fail to allocate id for vgt instance.");
		goto err;
	}

	mutex_unlock(&pdev->lock);

	vgt->vm_id = info->domid;
	vgt->id = id;
	vgt->pdev = pdev;

	if (!create_virtual_device_state(vgt, info))
		goto err;

	if (hypervisor_hvm_init(vgt) < 0)
		goto err;

	gvt_set_instance_online(vgt);

	return vgt;
err:
	mutex_unlock(&pdev->lock);
	gvt_destroy_instance(vgt);
	return NULL;
}
