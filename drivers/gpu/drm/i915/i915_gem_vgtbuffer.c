/*
 * Copyright © 2012 Intel Corporation
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
#include "fb_decoder.h"

static int i915_gem_vgtbuffer_get_pages(struct drm_i915_gem_object *obj)
{
	return 0;
}

static void i915_gem_vgtbuffer_put_pages(struct drm_i915_gem_object *obj)
{
}

static const struct drm_i915_gem_object_ops i915_gem_vgtbuffer_ops = {
	.get_pages = i915_gem_vgtbuffer_get_pages,
	.put_pages = i915_gem_vgtbuffer_put_pages,
};

/**
 * Creates a new mm object that wraps some user memory.
 */
int
i915_gem_vgtbuffer_ioctl(struct drm_device *dev, void *data,
			 struct drm_file *file)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_gem_vgtbuffer *args = data;
	struct drm_i915_gem_object *obj;
	struct vgt_primary_plane_format *p;
	struct vgt_cursor_plane_format *c;
	struct vgt_fb_format fb;
	struct vgt_pipe_format *pipe;

	int ret;

	int num_pages = 0;

	u32 vmid;
	u32 handle;

	uint32_t __iomem *gtt_base = dev_priv->gtt.gsm;	/* mappable_base; */
	uint32_t gtt_fbstart;
	uint32_t gtt_pte;
	uint32_t gtt_offset = 0;

	/* Allocate the new object */
	DRM_DEBUG_DRIVER("VGT: gem_vgtbuffer_ioctl\n");

	if (!vgt_check_host())
		return -EPERM;

	obj = i915_gem_object_alloc(dev);
	if (obj == NULL)
		return -ENOMEM;

	vmid = args->vmid;
	DRM_DEBUG_DRIVER("VGT: calling decode\n");
	if (vgt_decode_fb_format(vmid, &fb)) {
		kfree(obj);
		return -EINVAL;
	}

	pipe = ((args->pipe_id >= I915_MAX_PIPES) ?
		NULL : &fb.pipes[args->pipe_id]);

	/* If plane is not enabled, bail */
	if (!pipe || !pipe->primary.enabled) {
		kfree(obj);
		return -ENOENT;
	}

	DRM_DEBUG_DRIVER("VGT: pipe = %d\n", args->pipe_id);
	if ((args->plane_id) == I915_VGT_PLANE_PRIMARY) {
		DRM_DEBUG_DRIVER("VGT: &pipe=0x%x\n", (&pipe));
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
		args->tiled = p->tiled;
		args->size = (((p->width * p->height * p->bpp) / 8) +
			      (PAGE_SIZE - 1)) >> PAGE_SHIFT;

		uint64_t range = p->base >> PAGE_SHIFT;
		range += args->size;

		if (range > gtt_total_entries(dev_priv->gtt)) {
			DRM_DEBUG_DRIVER("VGT: Invalid GTT offset or size\n");
			kfree(obj);
			return -EINVAL;
		}

		if (args->flags & I915_VGTBUFFER_QUERY_ONLY) {
			DRM_DEBUG_DRIVER("VGT: query only: primary");
			kfree(obj);
			return 0;
		}

		gtt_offset = p->base;
		num_pages = args->size;

		DRM_DEBUG_DRIVER("VGT GEM: Surface GTT Offset = %x\n", p->base);
		obj->tiling_mode = p->tiled ? I915_TILING_X : 0;
		obj->stride = p->tiled ? args->stride : 0;
	}

	if ((args->plane_id) == I915_VGT_PLANE_CURSOR) {
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
		args->size = (((c->width * c->height * c->bpp) / 8) +
			      (PAGE_SIZE - 1)) >> PAGE_SHIFT;

		uint64_t range = c->base >> PAGE_SHIFT;
		range += args->size;

		if (range > gtt_total_entries(dev_priv->gtt)) {
			DRM_DEBUG_DRIVER("VGT: Invalid GTT offset or size\n");
			kfree(obj);
			return -EINVAL;
		}

		if (args->flags & I915_VGTBUFFER_QUERY_ONLY) {
			DRM_DEBUG_DRIVER("VGT: query only: cursor");
			kfree(obj);
			return 0;
		}

		gtt_offset = c->base;
		num_pages = args->size;

		DRM_DEBUG_DRIVER("VGT GEM: Surface GTT Offset = %x\n", c->base);
		obj->tiling_mode = I915_TILING_NONE;
	}

	DRM_DEBUG_DRIVER("VGT GEM: Surface size = %d\n",
			 (int)(num_pages * PAGE_SIZE));

	gtt_fbstart = gtt_offset >> PAGE_SHIFT;

	DRM_DEBUG_DRIVER("VGT GEM: gtt start addr %x\n",
			 (unsigned int)gtt_base);
	DRM_DEBUG_DRIVER("VGT GEM: fb start %x\n", (unsigned int)gtt_fbstart);

	gtt_base += gtt_fbstart;

	DRM_DEBUG_DRIVER("VGT GEM: gtt + fb start  %x\n", (uint32_t) gtt_base);

	DRM_DEBUG_DRIVER("VGT: gtt_base=0x%x\n", gtt_base);

	gtt_pte = readl(gtt_base);

	DRM_DEBUG_DRIVER("VGT GEM: pte  %x\n", (uint32_t) gtt_pte);
	DRM_DEBUG_DRIVER("VGT GEM: num_pages from fb decode=%d  \n",
			 (uint32_t) num_pages);

	drm_gem_private_object_init(dev, &obj->base, num_pages * PAGE_SIZE);

	i915_gem_object_init(obj, &i915_gem_vgtbuffer_ops);
	obj->cache_level = I915_CACHE_L3_LLC;
	obj->has_vmfb_mapping = true;
	obj->pages = NULL;

	struct i915_address_space *ggtt_vm = &dev_priv->gtt.base;
	struct i915_vma *vma = i915_gem_obj_lookup_or_create_vma(obj, ggtt_vm);
	vma->node.start = gtt_offset;

	ret = drm_gem_handle_create(file, &obj->base, &handle);
	/* drop reference from allocate - handle holds it now */
	drm_gem_object_unreference(&obj->base);
	if (ret) {
		kfree(obj);
		i915_gem_vma_destroy(vma);
		return ret;
	}

	args->handle = handle;
	return 0;
}
