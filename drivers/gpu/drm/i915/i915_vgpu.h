/*
 * Copyright(c) 2011-2015 Intel Corporation. All rights reserved.
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

#ifndef _I915_VGPU_H_
#define _I915_VGPU_H_

#include "gvt/params.h"
/* The MMIO offset of the shared info between guest and host emulator */
#define VGT_PVINFO_PAGE	0x78000
#define VGT_PVINFO_SIZE	0x1000

/*
 * The following structure pages are defined in GEN MMIO space
 * for virtualization. (One page for now)
 */
#define VGT_MAGIC         0x4776544776544776ULL	/* 'vGTvGTvG' */
#define VGT_VERSION_MAJOR 1
#define VGT_VERSION_MINOR 0

#define INTEL_VGT_IF_VERSION_ENCODE(major, minor) ((major) << 16 | (minor))
#define INTEL_VGT_IF_VERSION \
	INTEL_VGT_IF_VERSION_ENCODE(VGT_VERSION_MAJOR, VGT_VERSION_MINOR)

/*
 * The information set by the guest gfx driver, through the display_ready field
 */
#define    VGT_DRV_DISPLAY_NOT_READY	(0 << 0)
#define    VGT_DRV_DISPLAY_READY	(1 << 0)	/* ready for display switch */
#define    VGT_DRV_LEGACY_VGA_MODE	(1 << 1)	/* in the legacy VGA mode */

/*
 * guest-to-vgt notifications
 */
enum vgt_g2v_type {
	VGT_G2V_DISPLAY_REFRESH,
	VGT_G2V_SET_POINTER_SHAPE,
	VGT_G2V_PPGTT_L3_PAGE_TABLE_CREATE,
	VGT_G2V_PPGTT_L3_PAGE_TABLE_DESTROY,
	VGT_G2V_PPGTT_L4_PAGE_TABLE_CREATE,
	VGT_G2V_PPGTT_L4_PAGE_TABLE_DESTROY,
	VGT_G2V_EXECLIST_CONTEXT_ELEMENT_CREATE,
	VGT_G2V_EXECLIST_CONTEXT_ELEMENT_DESTROY,
	VGT_G2V_MAX,
};

/*
 * vgt-to-guest notifications
 */
enum vgt_v2g_type {
	VGT_V2G_SET_HW_CURSOR,
	VGT_V2G_SET_SW_CURSOR,
	VGT_V2G_MAX,
};

enum vgt_caps_type {
	VGT_CAPS_PREEMPTION = (1 << 0),
};

struct vgt_if {
	uint64_t magic;		/* VGT_MAGIC */
	uint16_t version_major;
	uint16_t version_minor;
	uint32_t vgt_id;	/* ID of vGT instance */
	uint32_t vgt_caps;     /* VGT capabilties */
	uint32_t rsv1[11];	/* pad to offset 0x40 */
	/*
	 *  Data structure to describe the balooning info of resources.
	 *  Each VM can only have one portion of continuous area for now.
	 *  (May support scattered resource in future)
	 *  (starting from offset 0x40)
	 */
	struct {
		/* Aperture register balooning */
		struct {
			uint32_t base;
			uint32_t size;
		} mappable_gmadr;	/* aperture */
		/* GMADR register balooning */
		struct {
			uint32_t base;
			uint32_t size;
		} nonmappable_gmadr;	/* non aperture */
		/* allowed fence registers */
		uint32_t fence_num;
		uint32_t rsv2[3];
	} avail_rs;		/* available/assigned resource */
	uint32_t rsv3[0x200 - 24];	/* pad to half page */
	/*
	 * The bottom half page is for response from Gfx driver to hypervisor.
	 */
	uint16_t  drv_version_major;
	uint16_t  drv_version_minor;
	uint32_t  display_ready;/* ready for display owner switch */
	/*
	 * driver reported status/error code
	 *     0: if the avail_rs is sufficient to driver
	 *  Bit 2,1,0 set indicating
	 *       Insufficient low_gmadr, high_gmadr, fence resources.
	 *  Other bits are reserved.
	 */
	uint32_t  rs_insufficient;
	/*
	 * The driver is required to update the following field with minimal
	 * required resource size.
	 */
	uint32_t  min_low_gmadr;
	uint32_t  min_high_gmadr;
	uint32_t  min_fence_num;

	/*
	 * notifications between guest and vgt
	 */
	uint32_t  g2v_notify;
	uint32_t  v2g_notify;

	/*
	 * PPGTT PTE table info
	 */
	uint32_t  gmm_gtt_seg_base;
	uint32_t  rsv4;
	uint32_t  gmm_gtt_seg_size;
	uint32_t  rsv5;

	/*
	 * Cursor hotspot info
	 */
	uint32_t  xhot;
	uint32_t  yhot;

	struct {
		uint32_t lo;
		uint32_t hi;
	} pdp[4];

	uint32_t execlist_context_descriptor_lo;
	uint32_t execlist_context_descriptor_hi;

	/*
	 * scratch space for debugging
	 */
	uint32_t  scratch;;

	uint32_t  rsv6[0x200-25];    /* pad to one page */
} __packed;

#define vgtif_reg(x) \
	_MMIO((VGT_PVINFO_PAGE + (long)&((struct vgt_if *)NULL)->x))

extern void i915_check_vgpu(struct drm_device *dev);
extern int intel_vgt_balloon(struct drm_device *dev);
extern void intel_vgt_deballoon(void);
extern void *gvt_create_pgt_device(struct drm_i915_private *dev_priv);
extern bool gvt_post_init_pgt_device(void *private_data);
extern void gvt_destroy_pgt_device(void *private_data);

#endif /* _I915_VGPU_H_ */
