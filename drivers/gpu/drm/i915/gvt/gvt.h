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

#ifndef _GVT_H_
#define _GVT_H_

#include "i915_drv.h"
#include "i915_vgpu.h"

#include "debug.h"
#include "params.h"
#include "reg.h"
#include "hypercall.h"
#include "mpt.h"
#include "fb_decoder.h"

#define GVT_MAX_VGPU 8

#define GVT_MAX_GM_SIZE		(1UL << 32)
#define GVT_GM_BITMAP_BITS	(GVT_MAX_GM_SIZE >> 20)
#define GVT_MAX_NUM_FENCES	32
#define GVT_FENCE_BITMAP_BITS	GVT_MAX_NUM_FENCES

enum {
	GVT_HYPERVISOR_TYPE_XEN = 0,
	GVT_HYPERVISOR_TYPE_KVM,
};

struct gvt_host {
	bool initialized;
	int hypervisor_type;
	struct mutex device_idr_lock;
	struct idr device_idr;
	struct gvt_kernel_dm *kdm;
};

extern struct gvt_host gvt_host;

/* Describe the limitation of HW.*/
struct gvt_device_info {
	u64 max_gtt_gm_sz;
	u32 gtt_start_offset;
	u32 gtt_end_offset;
	u32 max_gtt_size;
	u32 gtt_entry_size;
	u32 gtt_entry_size_shift;
	u32 gmadr_bytes_in_cmd;
};

struct gvt_gm_node {
	struct drm_mm_node *low_gm_node;
	struct drm_mm_node *high_gm_node;
};

struct gvt_virtual_gm_state {
	u64 aperture_base;
	void *aperture_base_va;
	u64 aperture_sz;
	u64 gm_sz;
	u64 aperture_offset;        /* address fix for visible GM */
	u64 hidden_gm_offset;       /* address fix for invisible GM */
	int fence_base;
	int fence_sz;
	struct gvt_gm_node node;
};

struct gvt_virtual_device_state {
	struct gvt_virtual_gm_state gm;
};

struct vgt_device {
	int id;
	int vm_id;
	struct pgt_device *pdev;
	bool warn_untrack;
	struct gvt_virtual_device_state state;
};

struct gvt_gm_allocator {
	struct drm_mm low_gm;
	struct drm_mm high_gm;
};

struct pgt_device {
	struct mutex lock;
	int id;

	struct drm_i915_private *dev_priv;
	struct idr instance_idr;

	struct gvt_device_info device_info;

	u8 initial_cfg_space[GVT_CFG_SPACE_SZ];
	u64 bar_size[GVT_BAR_NUM];

	u64 gttmmio_base;
	void *gttmmio_va;

	u64 gmadr_base;
	void *gmadr_va;

	u32 mmio_size;
	u32 reg_num;

	wait_queue_head_t service_thread_wq;
	struct task_struct *service_thread;
	unsigned long service_request;

	/* 1 bit corresponds to 1MB in the GM space */
	DECLARE_BITMAP(gm_bitmap, GVT_GM_BITMAP_BITS);

	/* 1 bit corresponds to 1 fence register */
	DECLARE_BITMAP(fence_bitmap, GVT_FENCE_BITMAP_BITS);

	u64 total_gm_sz;
	struct gvt_gm_allocator gm_allocator;
};

/* definitions for physical aperture/GM space */
#define phys_aperture_sz(pdev)          (pdev->bar_size[1])
#define phys_aperture_pages(pdev)       (phys_aperture_sz(pdev) >> GTT_PAGE_SHIFT)
#define phys_aperture_base(pdev)        (pdev->gmadr_base)
#define phys_aperture_vbase(pdev)       (pdev->gmadr_va)

#define gm_sz(pdev)                     (pdev->total_gm_sz)
#define gm_base(pdev)                   (0ULL)
#define gm_pages(pdev)                  (gm_sz(pdev) >> GTT_PAGE_SHIFT)
#define hidden_gm_base(pdev)            (phys_aperture_sz(pdev))

#define aperture_2_gm(pdev, addr)       (addr - phys_aperture_base(pdev))
#define v_aperture(pdev, addr)          (phys_aperture_vbase(pdev) + (addr))

/* definitions for vgt's aperture/gm space */
#define gvt_aperture_base(vgt)		(vgt->state.gm.aperture_base)
#define gvt_aperture_vbase(vgt)		(vgt->state.gm.aperture_base_va)
#define gvt_aperture_offset(vgt)	(vgt->state.gm.aperture_offset)
#define gvt_hidden_gm_offset(vgt)	(vgt->state.gm.hidden_gm_offset)
#define gvt_aperture_sz(vgt)		(vgt->state.gm.aperture_sz)
#define gvt_gm_sz(vgt)			(vgt->state.gm.gm_sz)
#define gvt_hidden_gm_sz(vgt)		(gvt_gm_sz(vgt) - gvt_aperture_sz(vgt))
#define gvt_fence_base(vgt)		(vgt->state.gm.fence_base)
#define gvt_fence_sz(vgt)		(vgt->state.gm.fence_sz)

#define gvt_aperture_end(vgt)           \
	(gvt_aperture_base(vgt) + gvt_aperture_sz(vgt) - 1)
#define gvt_visible_gm_base(vgt)        \
	(gm_base(vgt->pdev) + gvt_aperture_offset(vgt))
#define gvt_visible_gm_end(vgt)         \
	(gvt_visible_gm_base(vgt) + gvt_aperture_sz(vgt) - 1)
#define gvt_hidden_gm_base(vgt) \
	(gm_base(vgt->pdev) + gvt_hidden_gm_offset(vgt))
#define gvt_hidden_gm_end(vgt)          \
	(gvt_hidden_gm_base(vgt) + gvt_hidden_gm_sz(vgt) - 1)

/*
 * the view of the aperture/gm space from the VM's p.o.v
 *
 * when the VM supports ballooning, this view is the same as the
 * view of vGT driver.
 *
 * when the VM does not support ballooning, this view starts from
 * GM space ZERO
 */
#define gvt_guest_aperture_base(vgt)    \
	((*((u32*)&vgt->state.cfg.space[GVT_REG_CFG_SPACE_BAR1]) & ~0xf) + gvt_aperture_offset(vgt))
#define gvt_guest_aperture_end(vgt)     \
        (gvt_guest_aperture_base(vgt) + gvt_aperture_sz(vgt) - 1)
#define gvt_guest_visible_gm_base(vgt)  \
        (gvt_visible_gm_base(vgt))
#define gvt_guest_visible_gm_end(vgt)   \
        (gvt_guest_visible_gm_base(vgt) + gvt_aperture_sz(vgt) - 1)
#define gvt_guest_hidden_gm_base(vgt)   \
	gvt_hidden_gm_base(vgt)
#define gvt_guest_hidden_gm_end(vgt)    \
        (gvt_guest_hidden_gm_base(vgt) + gvt_hidden_gm_sz(vgt) - 1)

extern void gvt_init_resource_allocator(struct pgt_device *pdev);
extern void gvt_clean_resource_allocator(struct pgt_device *pdev);
extern int gvt_alloc_gm_and_fence_resource(struct vgt_device *vgt,
		struct gvt_instance_info *info);
extern void gvt_free_gm_and_fence_resource(struct vgt_device *vgt);

static inline u32 gvt_mmio_read(struct pgt_device *pdev,
		u32 reg)
{
	struct drm_i915_private *dev_priv = pdev->dev_priv;
	i915_reg_t tmp = {.reg = reg};
	return I915_READ(tmp);
}

static inline void gvt_mmio_write(struct pgt_device *pdev,
		u32 reg, u32 val)
{
	struct drm_i915_private *dev_priv = pdev->dev_priv;
	i915_reg_t tmp = {.reg = reg};
	I915_WRITE(tmp, val);
}

static inline u64 gvt_mmio_read64(struct pgt_device *pdev,
		u32 reg)
{
	struct drm_i915_private *dev_priv = pdev->dev_priv;
	i915_reg_t tmp = {.reg = reg};
	return I915_READ64(tmp);
}

static inline void gvt_mmio_write64(struct pgt_device *pdev,
		u32 reg, u64 val)
{
	struct drm_i915_private *dev_priv = pdev->dev_priv;
	i915_reg_t tmp = {.reg = reg};
	I915_WRITE64(tmp, val);
}

#endif
