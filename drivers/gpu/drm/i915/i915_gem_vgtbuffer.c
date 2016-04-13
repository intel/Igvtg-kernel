/*
 * Copyright © 2012 - 2015 Intel Corporation
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include "i915_drv.h"
#include "i915_trace.h"
#include "intel_drv.h"
#include <linux/swap.h>

bool enable_vgtbuffer = false;
module_param_named(enable_vgtbuffer, enable_vgtbuffer, bool, 0600);
MODULE_PARM_DESC(enable_vgtbuffer, "Control the vgtbuffer ioctl available or not (default: false)");

struct vgt_device;
#include "vgt/fb_decoder.h"

static int i915_gem_vgtbuffer_get_pages(struct drm_i915_gem_object *obj)
{
	BUG();
	return -EINVAL;
}

static void i915_gem_vgtbuffer_put_pages(struct drm_i915_gem_object *obj)
{
	/* like stolen memory, this should only be called during free
	 * after clearing pin count.
	 */
	sg_free_table(obj->pages);
	kfree(obj->pages);
}

static const struct drm_i915_gem_object_ops i915_gem_vgtbuffer_ops = {
	.get_pages = i915_gem_vgtbuffer_get_pages,
	.put_pages = i915_gem_vgtbuffer_put_pages,
};

#define GEN8_DECODE_PTE(pte) \
	((dma_addr_t)(((((u64)pte) >> 12) & 0x7ffffffULL) << 12))

#define GEN7_DECODE_PTE(pte) \
	((dma_addr_t)(((((u64)pte) & 0x7f0) << 28) | (u64)(pte & 0xfffff000)))

static struct sg_table *
i915_create_sg_pages_for_vgtbuffer(struct drm_device *dev,
			     u32 start, u32 num_pages)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct sg_table *st;
	struct scatterlist *sg;
	int i;

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return NULL;

	if (sg_alloc_table(st, num_pages, GFP_KERNEL)) {
		kfree(st);
		return NULL;
	}

	if (INTEL_INFO(dev)->gen >= 8) {
		gen8_pte_t __iomem *gtt_entries =
			(gen8_pte_t __iomem *)dev_priv->gtt.gsm +
			(start >> PAGE_SHIFT);
		for_each_sg(st->sgl, sg, num_pages, i) {
			sg->offset = 0;
			sg->length = PAGE_SIZE;
			sg_dma_address(sg) =
				GEN8_DECODE_PTE(readq(&gtt_entries[i]));
			sg_dma_len(sg) = PAGE_SIZE;
		}
	} else {
		gen6_pte_t __iomem *gtt_entries =
			(gen6_pte_t __iomem *)dev_priv->gtt.gsm +
			(start >> PAGE_SHIFT);
		for_each_sg(st->sgl, sg, num_pages, i) {
			sg->offset = 0;
			sg->length = PAGE_SIZE;
			sg_dma_address(sg) =
				GEN7_DECODE_PTE(readq(&gtt_entries[i]));
			sg_dma_len(sg) = PAGE_SIZE;
		}
	}

	return st;
}

struct drm_i915_gem_object *
i915_gem_object_create_vgtbuffer(struct drm_device *dev,
				 u32 start, u32 num_pages)
{
	struct drm_i915_gem_object *obj;
	obj = i915_gem_object_alloc(dev);
	if (obj == NULL)
		return NULL;

	drm_gem_private_object_init(dev, &obj->base, num_pages << PAGE_SHIFT);
	i915_gem_object_init(obj, &i915_gem_vgtbuffer_ops);

	obj->pages = i915_create_sg_pages_for_vgtbuffer(dev, start, num_pages);
	if (obj->pages == NULL) {
		i915_gem_object_free(obj);
		return NULL;
	}

	i915_gem_object_pin_pages(obj);
	obj->cache_level = I915_CACHE_L3_LLC;

	DRM_DEBUG_DRIVER("VGT_GEM: backing store base = 0x%x pages = 0x%x\n",
			 start, num_pages);
	return obj;
}

static int vgt_decode_information(struct drm_device *dev,
				  struct drm_i915_gem_vgtbuffer *args)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 vmid = args->vmid;
	struct vgt_fb_format fb;
	struct vgt_primary_plane_format *p;
	struct vgt_cursor_plane_format *c;
	struct vgt_pipe_format *pipe;

	if (vgt_decode_fb_format(vmid, &fb))
		return -EINVAL;

	pipe = ((args->pipe_id >= I915_MAX_PIPES) ?
		NULL : &fb.pipes[args->pipe_id]);

	if (!pipe || !pipe->primary.enabled) {
		DRM_DEBUG_DRIVER("VGT_GEM: Invalid pipe_id: %d\n",
				 args->pipe_id);
		return -EINVAL;
	}

	if ((args->plane_id) == I915_VGT_PLANE_PRIMARY) {
		p = &pipe->primary;
		args->enabled = p->enabled;
		args->x_offset = p->x_offset;
		args->y_offset = p->y_offset;
		args->start = p->base;
		args->width = p->width;
		args->height = p->height;
		args->stride = p->stride;
		args->bpp = p->bpp;
		args->hw_format = p->hw_format;
		args->drm_format = p->drm_format;
		args->tiled = vgt_get_tiling_mode(dev, p->tiled);
	} else if ((args->plane_id) == I915_VGT_PLANE_CURSOR) {
		c = &pipe->cursor;
		args->enabled = c->enabled;
		args->x_offset = c->x_hot;
		args->y_offset = c->y_hot;
		args->x_pos = c->x_pos;
		args->y_pos = c->y_pos;
		args->start = c->base;
		args->width = c->width;
		args->height = c->height;
		args->stride = c->width * (c->bpp / 8);
		args->bpp = c->bpp;
		args->tiled = 0;
	} else {
		DRM_DEBUG_DRIVER("VGT_GEM: Invalid plaine_id: %d\n",
				 args->plane_id);
		return -EINVAL;
	}

	args->size = (((args->width * args->height * args->bpp) / 8) +
		      (PAGE_SIZE - 1)) >> PAGE_SHIFT;

	if (args->start & (PAGE_SIZE - 1)) {
		DRM_DEBUG_DRIVER("VGT_GEM: Not aligned fb start address: "
				 "0x%x\n", args->start);
		return -EINVAL;
	}

	if (((args->start >> PAGE_SHIFT) + args->size) >
	    gtt_total_entries(dev_priv->gtt)) {
		DRM_DEBUG_DRIVER("VGT: Invalid GTT offset or size\n");
		return -EINVAL;
	}
	return 0;
}

/**
 * Creates a new mm object that wraps some user memory.
 */
int
i915_gem_vgtbuffer_ioctl(struct drm_device *dev, void *data,
			 struct drm_file *file)
{
	struct drm_i915_gem_vgtbuffer *args = data;
	struct drm_i915_gem_object *obj;
	u32 handle;
	int ret;

	if (args->flags & I915_VGTBUFFER_CHECK_CAPABILITY) {
		if (enable_vgtbuffer)
			return 0;
		else
			return -EINVAL;
	}

	if (INTEL_INFO(dev)->gen < 7)
		return -EPERM;

	if (!vgt_check_host())
		return -EPERM;

	ret = vgt_decode_information(dev, args);
	if (ret)
		return ret;

	if (args->flags & I915_VGTBUFFER_QUERY_ONLY)
		return 0;

	obj = i915_gem_object_create_vgtbuffer(dev, args->start, args->size);
	if (!obj) {
		DRM_DEBUG_DRIVER("VGT_GEM: Failed to create gem object"
					" for VM FB!\n");
		return -EINVAL;
	}

	obj->tiling_mode = args->tiled;
	obj->stride = args->tiled ? args->stride : 0;

	ret = drm_gem_handle_create(file, &obj->base, &handle);
	if (ret) {
		/* TODO: Double confirm the error handling path */
		i915_gem_object_free(obj);
		return ret;
	}

	/* drop reference from allocate - handle holds it now */
	drm_gem_object_unreference_unlocked(&obj->base);

	args->handle = handle;
	return 0;
}
