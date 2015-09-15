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

struct vgt_device {
	int id;
	int vm_id;
	struct pgt_device *pdev;
	bool warn_untrack;
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
};

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
