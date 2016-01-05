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

#include <linux/kthread.h>

static bool populate_shadow_context(struct gvt_workload *workload)
{
	struct vgt_device *vgt = workload->vgt;
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_workload_scheduler *scheduler = &pdev->workload_scheduler;
	int ring_id = workload->ring_id;

	struct intel_context *shadow_ctx = scheduler->shadow_ctx;
	struct drm_i915_gem_object *ctx_obj = shadow_ctx->engine[ring_id].state;

	struct execlist_ring_context *guest_ring_context, *shadow_ring_context;

	struct page *page;
	void *src, *dst;
	unsigned long guest_context_pn, context_page_num;
	int i;

	gvt_dbg_sched("ring id %d workload lrca %x", ring_id, workload->ctx_desc.lrca);

	guest_context_pn = workload->ctx_desc.lrca;

	context_page_num = intel_lr_context_size(&pdev->dev_priv->ring[ring_id]);
	context_page_num = context_page_num >> PAGE_SHIFT;

	i = 2;

	while (i < context_page_num) {
		src = gvt_gma_to_va(vgt->gtt.ggtt_mm,
				(guest_context_pn + i) << GTT_PAGE_SHIFT);
		if (!src) {
			gvt_err("invalid guest context descriptor");
			return false;
		}

		page = i915_gem_object_get_page(ctx_obj, LRC_PPHWSP_PN + i);
		dst = kmap_atomic(page);
		hypervisor_read_va(vgt, src, dst, GTT_PAGE_SIZE, 1);
		kunmap_atomic(dst);
		i++;
	}

	guest_ring_context = gvt_gma_to_va(vgt->gtt.ggtt_mm,
			(guest_context_pn + 1) << GTT_PAGE_SHIFT);
	if (!guest_ring_context) {
		gvt_err("invalid guest context descriptor");
		return false;
	}

	page = i915_gem_object_get_page(ctx_obj, LRC_PPHWSP_PN + 1);
	shadow_ring_context = kmap_atomic(page);

#define COPY_REG(name) \
	hypervisor_read_va(vgt, &guest_ring_context->name.val, \
		&shadow_ring_context->name.val, 4, 1);

	COPY_REG(ctx_ctrl);
	COPY_REG(ctx_timestamp);

	if (ring_id == RCS) {
		COPY_REG(bb_per_ctx_ptr);
		COPY_REG(rcs_indirect_ctx);
		COPY_REG(rcs_indirect_ctx_offset);
	}
#undef COPY_REG

	gvt_set_context_pdp_root_pointer(vgt, shadow_ring_context,
			workload->shadow_mm->shadow_page_table);

	hypervisor_read_va(vgt,
			(void *)guest_ring_context + sizeof(*guest_ring_context),
			(void *)shadow_ring_context + sizeof(*shadow_ring_context),
			GTT_PAGE_SIZE - sizeof(*guest_ring_context), 1);

	kunmap_atomic(shadow_ring_context);
	return true;
}

static void shadow_context_schedule_in(void *data)
{
	struct gvt_workload *workload = (struct gvt_workload *)data;

	gvt_load_render_mmio(workload->vgt, workload->ring_id);
	atomic_set(&workload->shadow_ctx_active, 1);
	wake_up(&workload->shadow_ctx_status_wq);
}

static void shadow_context_schedule_out(void *data)
{
	struct gvt_workload *workload = (struct gvt_workload *)data;

	gvt_restore_render_mmio(workload->vgt, workload->ring_id);
	atomic_set(&workload->shadow_ctx_active, 0);
	wake_up(&workload->shadow_ctx_status_wq);
}

static bool dispatch_workload(struct gvt_workload *workload)
{
	struct vgt_device *vgt = workload->vgt;
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_workload_scheduler *scheduler = &pdev->workload_scheduler;
	int ring_id = workload->ring_id;
	struct intel_context *shadow_ctx = scheduler->shadow_ctx;
	struct drm_i915_private *dev_priv = workload->vgt->pdev->dev_priv;

	gvt_dbg_sched("ring id %d prepare to dispatch workload %p",
		ring_id, workload);

	workload->req = i915_gem_request_alloc(&dev_priv->ring[ring_id],
					       shadow_ctx);
	if (IS_ERR_OR_NULL(workload->req)) {
		gvt_err("fail to allocate gem request");
		workload->status = PTR_ERR(workload->req);
		return true;
	}

	gvt_dbg_sched("ring id %d get i915 gem request %p",
			ring_id, workload->req);

	mutex_lock(&pdev->lock);

	if (!populate_shadow_context(workload)) {
		workload->status = -EINVAL;
		goto err;
	}

	mutex_unlock(&pdev->lock);

	gvt_dbg_sched("ring id %d submit workload to i915 %p",
			ring_id, workload->req);

	shadow_ctx->gvt_context_private_data[ring_id] = workload;
	shadow_ctx->gvt_context_addressing_mode[ring_id] =
		workload->ctx_desc.addressing_mode << 3;
	shadow_ctx->gvt_context_schedule_in = shadow_context_schedule_in;
	shadow_ctx->gvt_context_schedule_out = shadow_context_schedule_out;

	i915_gem_request_reference(workload->req);
	i915_add_request_no_flush(workload->req);

	workload->dispatched = true;
	return true;
err:
	if (workload->req) {
		i915_gem_request_cancel(workload->req);
		workload->req = NULL;
	}
	mutex_unlock(&pdev->lock);
	return false;
}

static struct gvt_workload *pick_next_workload(
		struct pgt_device *pdev, int ring_id)
{
	struct gvt_workload_scheduler *scheduler = &pdev->workload_scheduler;
	struct gvt_workload *workload = NULL;

	mutex_lock(&pdev->lock);

	/*
	 * no current instance / will be scheduled out / no workload
	 * bail out
	 */
	if (!scheduler->current_instance) {
		gvt_dbg_sched("ring id %d stop - no current instance", ring_id);
		goto out;
	}

	if (scheduler->need_reschedule) {
		gvt_dbg_sched("ring id %d stop - will reschedule", ring_id);
		goto out;
	}

	if (list_empty(workload_q_head(scheduler->current_instance, ring_id))) {
		gvt_dbg_sched("ring id %d stop - no avaiable workload", ring_id);
		goto out;
	}

	/*
	 * still have current workload, maybe the workload disptacher
	 * fail to submit it for some reason, resubmit it.
	 */
	if (scheduler->current_workload[ring_id]) {
		workload = scheduler->current_workload[ring_id];
		gvt_dbg_sched("ring id %d still have current workload %p",
				ring_id, workload);
		goto out;
	}

	/*
	 * pick a workload as current workload
	 * once current workload is set, schedule policy routines
	 * will wait the current workload to NULL when trying to
	 * schedule out an instance.
	 */
	scheduler->current_workload[ring_id] = container_of(
			workload_q_head(scheduler->current_instance, ring_id)->next,
			struct gvt_workload, list);

	workload = scheduler->current_workload[ring_id];

	gvt_dbg_sched("ring id %d pick new workload %p", ring_id, workload);

	atomic_inc(&workload->vgt->running_workload_num);
out:
	mutex_unlock(&pdev->lock);

	return workload;
}

static void update_guest_context(struct gvt_workload *workload)
{
	struct vgt_device *vgt = workload->vgt;
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_workload_scheduler *scheduler = &pdev->workload_scheduler;
	int ring_id = workload->ring_id;

	struct intel_context *shadow_ctx = scheduler->shadow_ctx;
	struct drm_i915_gem_object *ctx_obj = shadow_ctx->engine[ring_id].state;

	struct execlist_ring_context *guest_ring_context, *shadow_ring_context;

	struct page *page;
	void *src, *dst;
	unsigned long guest_context_pn, context_page_num;
	int i;

	gvt_dbg_sched("ring id %d workload lrca %x", ring_id, workload->ctx_desc.lrca);

	guest_context_pn = workload->ctx_desc.lrca;

	context_page_num = intel_lr_context_size(&pdev->dev_priv->ring[ring_id]);
	context_page_num = context_page_num >> PAGE_SHIFT;

	i = 2;

	while (i < context_page_num) {
		dst = gvt_gma_to_va(vgt->gtt.ggtt_mm,
				(guest_context_pn + i) << GTT_PAGE_SHIFT);
		if (!dst) {
			gvt_err("invalid guest context descriptor");
			return;
		}

		page = i915_gem_object_get_page(ctx_obj, LRC_PPHWSP_PN + i);
		src = kmap_atomic(page);
		hypervisor_write_va(vgt, dst, src, GTT_PAGE_SIZE, 1);
		kunmap_atomic(dst);
		i++;
	}

	guest_ring_context = gvt_gma_to_va(vgt->gtt.ggtt_mm,
			(guest_context_pn + 1) << GTT_PAGE_SHIFT);
	if (!guest_ring_context) {
		gvt_err("invalid guest context descriptor");
		return;
	}

	hypervisor_write_va(vgt, &guest_ring_context->ring_header.val,
		&workload->rb_tail, 4, 1);

	page = i915_gem_object_get_page(ctx_obj, LRC_PPHWSP_PN + 1);
	shadow_ring_context = kmap_atomic(page);

#define COPY_REG(name) \
	hypervisor_write_va(vgt, &guest_ring_context->name.val, \
		&shadow_ring_context->name.val, 4, 1);

	COPY_REG(ctx_ctrl);
	COPY_REG(ctx_timestamp);

#undef COPY_REG

	hypervisor_write_va(vgt,
			(void *)guest_ring_context + sizeof(*guest_ring_context),
			(void *)shadow_ring_context + sizeof(*shadow_ring_context),
			GTT_PAGE_SIZE - sizeof(*guest_ring_context), 1);

	kunmap_atomic(shadow_ring_context);
}

static void complete_current_workload(struct pgt_device *pdev, int ring_id)
{
	struct gvt_workload_scheduler *scheduler = &pdev->workload_scheduler;
	struct gvt_workload *workload;

	mutex_lock(&pdev->lock);

	workload = scheduler->current_workload[ring_id];

	if (!workload->status) {
		wait_event(workload->shadow_ctx_status_wq,
				!atomic_read(&workload->shadow_ctx_active));
		update_guest_context(workload);
	}

	if (workload->req)
		i915_gem_request_unreference(workload->req);

	gvt_dbg_sched("ring id %d complete workload %p status %d",
			ring_id, workload, workload->status);

	scheduler->current_workload[ring_id] = NULL;

	atomic_dec(&workload->vgt->running_workload_num);

	list_del_init(&workload->list);
	workload->complete(workload);

	if (waitqueue_active(&scheduler->workload_complete_wq))
		wake_up(&scheduler->workload_complete_wq);

	mutex_unlock(&pdev->lock);
}

struct workload_thread_param {
	struct pgt_device *pdev;
	int ring_id;
};

static int workload_thread(void *priv)
{
	struct workload_thread_param *p = (struct workload_thread_param *)priv;
	struct pgt_device *pdev = p->pdev;
	int ring_id = p->ring_id;
	struct gvt_workload_scheduler *scheduler = &pdev->workload_scheduler;
	struct gvt_workload *workload = NULL;
	int r;

	kfree(p);

	gvt_dbg_core("workload thread for ring %d started", ring_id);

	while (!kthread_should_stop()) {
		r = wait_event_interruptible(scheduler->waitq[ring_id],
				kthread_should_stop() ||
				(workload = pick_next_workload(pdev, ring_id)));

		if (r)
			gvt_warn("workload thread waken up by unexpected signal!");

		if (kthread_should_stop())
			break;

		gvt_dbg_sched("ring id %d next workload %p vgt %d",
				workload->ring_id, workload, workload->vgt->id);

		/*
		 * Always take i915 big lock first
		 */
		r = i915_mutex_lock_interruptible(pdev->dev_priv->dev);
		if (r < 0) {
			gvt_warn("i915 submission channel is not available, retry");
			schedule_timeout(1);
			continue;
		}

		gvt_dbg_sched("ring id %d will dispatch workload %p",
				workload->ring_id, workload);

		if (!dispatch_workload(workload)) {
			gvt_warn("fail to dispatch workload, skip");
			goto complete;
		}

		gvt_dbg_sched("ring id %d wait workload %p",
				workload->ring_id, workload);

		workload->status = i915_wait_request(workload->req);
		if (workload->status != 0)
			gvt_warn("fail to wait workload, skip");

complete:
		gvt_dbg_sched("will complete workload %p, status: %d",
				workload, workload->status);

		complete_current_workload(pdev, ring_id);
		mutex_unlock(&pdev->dev_priv->dev->struct_mutex);
	}

	return 0;
}

void gvt_wait_instance_idle(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_workload_scheduler *scheduler = &pdev->workload_scheduler;

	if (atomic_read(&vgt->running_workload_num)) {
		gvt_dbg_sched("wait instance idle");

		wait_event(scheduler->workload_complete_wq,
				!atomic_read(&vgt->running_workload_num));
	}
}

void gvt_clean_workload_scheduler(struct pgt_device *pdev)
{
	struct gvt_workload_scheduler *scheduler = &pdev->workload_scheduler;
	int i;

	gvt_dbg_core("clean workload scheduler");

	for (i = 0; i < I915_NUM_RINGS; i++) {
		if (scheduler->thread[i]) {
			kthread_stop(scheduler->thread[i]);
			scheduler->thread[i] = NULL;
		}
	}

	i915_gem_context_unreference(scheduler->shadow_ctx);
	scheduler->shadow_ctx = NULL;

	gvt_clean_sched_policy(pdev);
}

bool gvt_init_workload_scheduler(struct pgt_device *pdev)
{
	struct gvt_workload_scheduler *scheduler = &pdev->workload_scheduler;
	struct workload_thread_param *param = NULL;
	int i;

	gvt_dbg_core("init workload scheduler");

	memset(scheduler, 0, sizeof(*scheduler));

	init_waitqueue_head(&scheduler->workload_complete_wq);

	scheduler->shadow_ctx = i915_gem_create_gvt_context(pdev->dev_priv->dev);
	if (!scheduler->shadow_ctx) {
		gvt_err("fail to create shadow context");
		goto err;
	}

	for (i = 0; i < I915_NUM_RINGS; i++) {
		init_waitqueue_head(&scheduler->waitq[i]);

		param = kzalloc(sizeof(*param), GFP_KERNEL);
		if (!param) {
			gvt_err("fail to allocate workload thread param");
			goto err;
		}

		param->pdev = pdev;
		param->ring_id = i;

		scheduler->thread[i] = kthread_run(workload_thread, param,
			"gvt workload %d", i);
		if (!scheduler->thread[i]) {
			gvt_err("fail to create workload thread");
			goto err;
		}
	}

	if (!gvt_init_sched_policy(pdev))
		goto err;

	return true;
err:
	if (param) {
		kfree(param);
		param = NULL;
	}
	gvt_clean_workload_scheduler(pdev);
	return false;
}
