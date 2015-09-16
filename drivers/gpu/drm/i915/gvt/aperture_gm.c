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

void init_gm_allocator(struct pgt_device *pdev,
		u64 start, u64 size, bool mappable)
{
	struct drm_mm *mm = mappable ?
		&pdev->gm_allocator.low_gm : &pdev->gm_allocator.high_gm;

	drm_mm_init(mm, start, size);
}

void clean_gm_allocator(struct pgt_device *pdev)
{
	if (!drm_mm_initialized(&pdev->gm_allocator.low_gm)
			|| !drm_mm_initialized(&pdev->gm_allocator.high_gm))
		return;

	drm_mm_takedown(&pdev->gm_allocator.low_gm);
	drm_mm_takedown(&pdev->gm_allocator.high_gm);
}

struct drm_mm_node *alloc_gm_node(struct pgt_device *pdev, u32 size, bool mappable)
{
	struct drm_mm_node *node;
	struct drm_mm *mm = mappable ?
		&pdev->gm_allocator.low_gm : &pdev->gm_allocator.high_gm;
	int ret;

	if (!drm_mm_initialized(mm))
		return NULL;

	DRM_DEBUG_KMS("creating vgt %s object: size=%x\n",
			mappable ? "mappable" : "unmappable", size);
	if (size == 0)
		return NULL;

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return NULL;

	ret = drm_mm_insert_node(mm, node, size,
			PAGE_SIZE, DRM_MM_SEARCH_DEFAULT);
	if (ret) {
		kfree(node);
		return NULL;
	}

	return node;
}

void free_gm_node(struct drm_mm_node *node)
{
	drm_mm_remove_node(node);
	kfree(node);
}

static bool check_instance_info(struct vgt_device *vgt,
		struct gvt_instance_info *info)
{
	struct pgt_device *pdev = vgt->pdev;

	if (gvt_aperture_base(vgt)) {
		gvt_err("resources have already been allocated");
		return false;
	}

	if (!info->low_gm_sz || !info->high_gm_sz || !info->fence_sz ||
		info->low_gm_sz > phys_aperture_sz(pdev) ||
		info->high_gm_sz > gm_sz(pdev) - phys_aperture_sz(pdev)) {
		gvt_err("invalid resource configuration");
		gvt_err("demand low GM size %u max low GM size %llu",
			info->low_gm_sz, phys_aperture_sz(pdev));
		gvt_err("demand high GM size %u max high GM size %llu",
			info->high_gm_sz, gm_sz(pdev) - phys_aperture_sz(pdev));
		gvt_err("fence size %u", info->fence_sz);
		return false;
	}

	return true;
}

static void clear_fence(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	int i;

	for (i = 0; i < gvt_fence_sz(vgt); i++)
		gvt_mmio_write64(pdev,
			i915_mmio_reg_offset(FENCE_REG_GEN6_LO(i + gvt_fence_base(vgt))), 0);
}

void gvt_free_gm_and_fence_resource(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	unsigned long *fence_bitmap = pdev->fence_bitmap;

	if (vgt->state.gm.node.low_gm_node) {
		free_gm_node(vgt->state.gm.node.low_gm_node);
		vgt->state.gm.node.low_gm_node = NULL;
	}

	if (vgt->state.gm.node.high_gm_node) {
		free_gm_node(vgt->state.gm.node.high_gm_node);
		vgt->state.gm.node.high_gm_node = NULL;
	}

	if (gvt_fence_sz(vgt) && gvt_fence_base(vgt)) {
		bitmap_clear(fence_bitmap, gvt_fence_base(vgt), gvt_fence_sz(vgt));
		clear_fence(vgt);
		gvt_fence_sz(vgt) = gvt_fence_base(vgt) = 0;
	}
}

int gvt_alloc_gm_and_fence_resource(struct vgt_device *vgt,
	struct gvt_instance_info *info)
{
	struct pgt_device *pdev = vgt->pdev;
	struct drm_mm_node *node;
	unsigned long *fence_bitmap = pdev->fence_bitmap;
	unsigned long fence_base;

	if (!check_instance_info(vgt, info)) {
		gvt_err("invalid resoure configuration");
		return -EINVAL;
	}

	node = alloc_gm_node(pdev, info->low_gm_sz << 20, true);
	if (!node) {
		gvt_err("fail to allocate low GM space");
		goto err;
	}

	vgt->state.gm.node.low_gm_node = node;

	gvt_aperture_base(vgt) = phys_aperture_base(vgt->pdev) + node->start;
	gvt_aperture_sz(vgt) = info->low_gm_sz << 20;

	node = alloc_gm_node(pdev, info->high_gm_sz << 20, false);
	if (!node) {
		gvt_err("fail to allocate high GM space");
		goto err;
	}

	vgt->state.gm.node.high_gm_node = node;

	gvt_hidden_gm_offset(vgt) = node->start;
	gvt_gm_sz(vgt) = (info->low_gm_sz + info->high_gm_sz) << 20;

	fence_base = bitmap_find_next_zero_area(fence_bitmap,
				GVT_FENCE_BITMAP_BITS, 0, info->fence_sz, 0);
	if (fence_base >= GVT_MAX_NUM_FENCES) {
		gvt_err("fail to allocate fence");
		goto err;
	}

	gvt_fence_base(vgt) = fence_base;
	gvt_fence_sz(vgt) = info->fence_sz;

	bitmap_set(fence_bitmap, fence_base, info->fence_sz);

	clear_fence(vgt);

	return 0;
err:
	gvt_free_gm_and_fence_resource(vgt);
	return -ENOMEM;
}

void gvt_init_resource_allocator(struct pgt_device *pdev)
{
	struct gvt_device_info *info = &pdev->device_info;
	int i;
	unsigned long *fence_bitmap = pdev->fence_bitmap;

	gvt_info("total aperture: 0x%llx bytes, total GM space: 0x%llx bytes\n",
		phys_aperture_sz(pdev), gm_sz(pdev));

	ASSERT(phys_aperture_sz(pdev) % (1 << 20) == 0);
	ASSERT(gm_sz(pdev) % (1 << 20) == 0);
	ASSERT(phys_aperture_sz(pdev) <= gm_sz(pdev) && gm_sz(pdev) <= info->max_gtt_gm_sz);
	ASSERT(info->max_gtt_gm_sz <= GVT_MAX_GM_SIZE);

	/* Basic memrange allocator for vgt low memory */
	init_gm_allocator(pdev, gvt.dom0_low_gm_sz << 20,
			(phys_aperture_sz(pdev) - (gvt.dom0_low_gm_sz << 20)), true);

	/* Basic memrange allocate for vgt high memory */
	init_gm_allocator(pdev,
			(phys_aperture_sz(pdev) + (gvt.dom0_high_gm_sz << 20)),
			(gm_sz(pdev) - (gvt.dom0_high_gm_sz << 20)), false);

	/* Reserve fence region for dom0 */
	bitmap_set(fence_bitmap, 0, gvt.dom0_fence_sz);

	for (i = 0; i < gvt.dom0_fence_sz; i++)
		gvt_mmio_write64(pdev, i915_mmio_reg_offset(FENCE_REG_GEN6_LO(i)), 0);
}

void gvt_clean_resource_allocator(struct pgt_device *pdev)
{
	clean_gm_allocator(pdev);
}
