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

#ifndef _GVT_SCHEDULER_H_
#define _GVT_SCHEDULER_H_

struct gvt_workload_scheduler {
	struct vgt_device *current_instance;
	struct vgt_device *next_instance;
	struct gvt_workload *current_workload[I915_NUM_RINGS];
	bool need_reschedule;

	struct intel_context *shadow_ctx;

	wait_queue_head_t workload_complete_wq;
	struct task_struct *thread[I915_NUM_RINGS];
	wait_queue_head_t waitq[I915_NUM_RINGS];

	void *sched_data;
	struct gvt_schedule_policy_ops *sched_ops;
};

struct gvt_workload {
	struct vgt_device *vgt;
	int ring_id;

	struct drm_i915_gem_request *req;

	/* if this workload has been dispatched to i915? */
	bool dispatched;
	int status;
	atomic_t shadow_ctx_active;
	wait_queue_head_t shadow_ctx_status_wq;

	/* execlist context information */
	struct execlist_ctx_descriptor_format ctx_desc;
	struct execlist_ring_context *ring_context;
	unsigned long rb_head, rb_tail, rb_ctl, rb_start;

	struct gvt_mm *shadow_mm;

	/* different submission model may need different complete handler */
	bool (*complete)(struct gvt_workload *);

	struct list_head list;
};

#define workload_q_head(vgt, ring_id) \
	(&(vgt->virtual_execlist_info[ring_id].workload_q_head))

#define queue_workload(workload) do { \
	list_add_tail(&workload->list, workload_q_head(workload->vgt, workload->ring_id)); \
	wake_up(&workload->vgt->pdev->workload_scheduler.waitq[workload->ring_id]); \
}while(0)

bool gvt_init_workload_scheduler(struct pgt_device *pdev);

void gvt_clean_workload_scheduler(struct pgt_device *pdev);

void gvt_wait_instance_idle(struct vgt_device *vgt);

#endif
