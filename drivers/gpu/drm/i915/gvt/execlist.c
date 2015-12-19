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

#define execlist_ring_mmio(ring_id, offset) \
	(gvt_ring_id_to_render_mmio_base(ring_id) + (offset))

#define valid_context(ctx) ((ctx)->valid)
#define same_context(a, b) (((a)->context_id == (b)->context_id) && \
		((a)->lrca == (b)->lrca))

static int context_switch_events[] = {
	[RCS] = RCS_AS_CONTEXT_SWITCH,
	[BCS] = BCS_AS_CONTEXT_SWITCH,
	[VCS] = VCS_AS_CONTEXT_SWITCH,
	[VCS2] = VCS2_AS_CONTEXT_SWITCH,
	[VECS] = VECS_AS_CONTEXT_SWITCH,
};

static int ring_id_to_context_switch_event(int ring_id)
{
	ASSERT(ring_id >= 0 && ring_id < ARRAY_SIZE(context_switch_events));
	return context_switch_events[ring_id];
}

static void switch_virtual_execlist_slot(struct gvt_virtual_execlist_info *info)
{
	gvt_dbg_el("[before] running slot %d/context %x pending slot %d",
		info->running_execlist ? info->running_execlist->index : -1,
		info->running_context ? info->running_context->context_id : 0,
		info->pending_execlist ? info->pending_execlist->index : -1);

	info->running_execlist = info->pending_execlist;
	info->pending_execlist = NULL;
	info->running_context = info->running_context ?
		&info->running_execlist->ctx[0] : NULL;

	gvt_dbg_el("[after] running slot %d/context %x pending slot %d",
		info->running_execlist ? info->running_execlist->index : -1,
		info->running_context ? info->running_context->context_id : 0,
		info->pending_execlist ? info->pending_execlist->index : -1);
}

static void emulate_execlist_status(struct gvt_virtual_execlist_info *info)
{
	struct gvt_execlist_state *running = info->running_execlist;
	struct gvt_execlist_state *pending = info->pending_execlist;
	struct execlist_ctx_descriptor_format *desc = info->running_context;
	struct vgt_device *vgt = info->vgt;

	struct execlist_status_format status;

	u32 status_reg = execlist_ring_mmio(info->ring_id, _EL_OFFSET_STATUS);

	status.ldw = __vreg(vgt, status_reg);
	status.udw = __vreg(vgt, status_reg + 4);

	if (running) {
		status.current_execlist_pointer = !!running->index;
		status.execlist_write_pointer = !!!running->index;
		status.execlist_0_active = status.execlist_0_valid =
			!!!(running->index);
		status.execlist_1_active = status.execlist_1_valid =
			!!(running->index);
	} else {
		status.context_id = 0;
		status.execlist_0_active = status.execlist_0_valid = 0;
		status.execlist_1_active = status.execlist_1_valid = 0;
	}

	status.context_id = desc ? desc->context_id : 0;
	status.execlist_queue_full = !!(pending);

	__vreg(vgt, status_reg) = status.ldw;
	__vreg(vgt, status_reg + 4) = status.udw;

	gvt_dbg_el("[vgt %d] status reg offset %x ldw %x udw %x",
		vgt->id, status_reg, status.ldw, status.udw);
}

static void emulate_csb_update(struct gvt_virtual_execlist_info *info,
		struct execlist_context_status_format *status)
{
	struct vgt_device *vgt = info->vgt;
	struct execlist_context_status_pointer_format ctx_status_ptr;
	u32 write_pointer;
	u32 ctx_status_ptr_reg, ctx_status_buf_reg, offset;

	ctx_status_ptr_reg = execlist_ring_mmio(info->ring_id,
			_EL_OFFSET_STATUS_PTR);
	ctx_status_buf_reg = execlist_ring_mmio(info->ring_id,
			_EL_OFFSET_STATUS_BUF);

	ctx_status_ptr.dw = __vreg(vgt, ctx_status_ptr_reg);

	write_pointer = ctx_status_ptr.write_ptr;

	if (write_pointer == 0x7)
		write_pointer = 0;
	else {
		++write_pointer;
		write_pointer %= 0x6;
	}

	offset = ctx_status_buf_reg + write_pointer * 8;

	__vreg(vgt, offset) = status->ldw;
	__vreg(vgt, offset + 4) = status->udw;

	ctx_status_ptr.write_ptr = write_pointer;
	__vreg(vgt, ctx_status_ptr_reg) = ctx_status_ptr.dw;

	gvt_dbg_el("[vgt %d] w pointer %u reg %x csb l %x csb h %x",
		vgt->id, write_pointer, offset, status->ldw, status->udw);

	gvt_trigger_virtual_event(vgt, ring_id_to_context_switch_event(info->ring_id));
}

static bool emulate_execlist_ctx_schedule_out(struct gvt_virtual_execlist_info *info,
		struct execlist_ctx_descriptor_format *ctx)
{
	struct gvt_execlist_state *running = info->running_execlist;
	struct gvt_execlist_state *pending = info->pending_execlist;
	struct execlist_ctx_descriptor_format *ctx0 = &running->ctx[0];
	struct execlist_ctx_descriptor_format *ctx1 = &running->ctx[1];
	struct execlist_context_status_format status;

	memset(&status, 0, sizeof(status));

	gvt_dbg_el("schedule out context id %x", ctx->context_id);

	if (!same_context(ctx, info->running_context)) {
		gvt_err("schedule out context is not running context,"
				"ctx id %x running ctx id %x",
				ctx->context_id,
				info->running_context->context_id);
		return false;
	}

	/* ctx1 is valid, ctx0/ctx is scheduled-out -> element switch */
	if (valid_context(ctx1) && same_context(ctx0, ctx)) {
		gvt_dbg_el("ctx 1 valid, ctx/ctx 0 is scheduled-out");

		info->running_context = ctx1;

		emulate_execlist_status(info);

		status.context_complete = status.element_switch = 1;
		status.context_id = ctx->context_id;

		emulate_csb_update(info, &status);
		/*
		 * ctx1 is not valid, ctx == ctx0
		 * ctx1 is valid, ctx1 == ctx
		 * 	--> last element is finished
		 * emulate:
		 * 	active-to-idle if there is *no* pending execlist
		 * 	context-complete if there *is* pending execlist
		 */
	} else if ((!valid_context(ctx1) && same_context(ctx0, ctx))
			|| (valid_context(ctx1) && same_context(ctx1, ctx))) {
		gvt_dbg_el("need to switch virtual execlist slot");

		switch_virtual_execlist_slot(info);

		emulate_execlist_status(info);

		if (!pending)
			status.context_complete = status.active_to_idle = 1;
		else
			status.context_complete = 1;

		status.context_id = ctx->context_id;

		emulate_csb_update(info, &status);
	} else {
		ASSERT(0);
		return false;
	}

	return true;
}

static struct gvt_execlist_state *get_next_execlist_state(
		struct gvt_virtual_execlist_info *info)
{
	u32 status_reg = execlist_ring_mmio(info->ring_id, _EL_OFFSET_STATUS);
	struct vgt_device *vgt = info->vgt;
	struct execlist_status_format status;

	status.ldw = __vreg(vgt, status_reg);
	status.udw = __vreg(vgt, status_reg + 4);

	if (status.execlist_queue_full) {
		gvt_err("virtual execlist slots are full");
		return NULL;
	}

	return &info->execlist[status.execlist_write_pointer];
}

static bool emulate_execlist_schedule_in(struct gvt_virtual_execlist_info *info,
		struct execlist_ctx_descriptor_format ctx[2])
{
	struct gvt_execlist_state *running = info->running_execlist;
	struct gvt_execlist_state *execlist = get_next_execlist_state(info);

	struct execlist_ctx_descriptor_format *ctx0, *ctx1;
	struct execlist_context_status_format status;

	gvt_dbg_el("emulate schedule-in");

	if (!execlist) {
		gvt_err("no avaiable execlist slot");
		return false;
	}

	memset(&status, 0, sizeof(status));
	memset(execlist->ctx, 0, sizeof(execlist->ctx));

	execlist->ctx[0] = ctx[0];
	execlist->ctx[1] = ctx[1];

	gvt_dbg_el("alloc slot index %d ctx 0 %x ctx 1 %x",
			execlist->index, ctx[0].context_id,
			ctx[1].context_id);

	/*
	 * no running execlist, make this write bundle as running execlist
	 * -> idle-to-active
	 */
	if (!running) {
		gvt_dbg_el("no current running execlist");

		info->running_execlist = execlist;
		info->pending_execlist = NULL;
		info->running_context = &execlist->ctx[0];

		gvt_dbg_el("running slot index %d running context %x",
				info->running_execlist->index,
				info->running_context->context_id);

		emulate_execlist_status(info);

		status.idle_to_active = 1;
		status.context_id = 0;

		emulate_csb_update(info, &status);
		return true;
	}

	ctx0 = &running->ctx[0];
	ctx1 = &running->ctx[1];

	gvt_dbg_el("current running execlist index %d ctx 0 %x ctx 1 %x",
		running->index, ctx0->context_id, ctx1->context_id);

	/*
	 * already has an running execlist
	 *	a. running ctx1 is valid,
	 *	   ctx0 is finished, and running ctx1 == new execlist ctx[0]
	 *	b. running ctx1 is not valid,
	 *	   ctx0 == new execlist ctx[0]
	 * ----> lite-restore + preempted
	 */
	if ((valid_context(ctx1) && same_context(ctx1, &execlist->ctx[0]) &&
		(!same_context(ctx0, info->running_context))) || /* condition a */
			(!valid_context(ctx1) &&
			same_context(ctx0, &execlist->ctx[0]))) { /* condition b */
		gvt_dbg_el("need to switch virtual execlist slot");

		info->pending_execlist = execlist;
		switch_virtual_execlist_slot(info);

		emulate_execlist_status(info);

		status.lite_restore = status.preempted = 1;
		status.context_id = ctx0->context_id;

		emulate_csb_update(info, &status);
	} else {
		gvt_dbg_el("emulate as pending slot");
		/*
		 * otherwise
		 * --> emulate pending execlist exist + but no preemption case
		 */
		info->pending_execlist = execlist;
		emulate_execlist_status(info);
	}

	return true;
}

static bool execlist_workload_complete(struct gvt_workload *workload)
{
	struct vgt_device *vgt = workload->vgt;
	struct gvt_virtual_execlist_info *info =
		&vgt->virtual_execlist_info[workload->ring_id];
	struct gvt_workload *next_workload;
	struct list_head *next = workload_q_head(vgt, workload->ring_id);
	bool lite_restore = false;

	gvt_dbg_el("complete workload %p status %d", workload, workload->status);

	if (workload->status)
		goto out;

	if (!list_empty(workload_q_head(vgt, workload->ring_id))) {
		struct execlist_ctx_descriptor_format *this_desc, *next_desc;

		next_workload = container_of(next, struct gvt_workload, list);
		this_desc = &workload->ctx_desc;
		next_desc = &next_workload->ctx_desc;

		lite_restore = same_context(this_desc, next_desc);
	}

	if (lite_restore) {
		gvt_dbg_el("next workload context is same as current - no schedule-out");
		goto out;
	}

	if (!emulate_execlist_ctx_schedule_out(info, &workload->ctx_desc)) {
		kfree(workload);
		return false;
	}

out:
	gvt_destroy_mm(workload->shadow_mm);
	kfree(workload);
	return true;
}

void gvt_get_context_pdp_root_pointer(struct vgt_device *vgt,
		struct execlist_ring_context *ring_context,
		u32 pdp[8])
{
	struct gvt_execlist_mmio_pair *pdp_pair = &ring_context->pdp3_UDW;
	u32 v;
	int i;

	for (i = 0; i < 8; i++) {
		hypervisor_read_va(vgt, &pdp_pair[i].val, &v, 4, 1);
		pdp[7 - i] = v;
	}
}

void gvt_set_context_pdp_root_pointer(struct vgt_device *vgt,
		struct execlist_ring_context *ring_context,
		u32 pdp[8])
{
	struct gvt_execlist_mmio_pair *pdp_pair = &ring_context->pdp3_UDW;
	int i;

	for (i = 0; i < 8; i++)
		pdp_pair[i].val = pdp[7 - i];
}

static struct execlist_ring_context *get_ring_context(struct vgt_device *vgt,
		u32 lrca)
{
	struct execlist_ring_context *context;
	u32 gma = (lrca + 1) << GTT_PAGE_SHIFT;

	context = (struct execlist_ring_context *)
		gvt_gma_to_va(vgt->gtt.ggtt_mm, gma);

	return context;
}

static bool prepare_workload(struct gvt_workload *workload)
{
	struct execlist_ctx_descriptor_format *desc = &workload->ctx_desc;
	struct gvt_mm *mm;
	gtt_type_t root_entry_type;
	int page_table_level;
	u32 pdp[8];

	if (desc->addressing_mode == 1) { /* legacy 32-bit */
		page_table_level = 3;
		root_entry_type = GTT_TYPE_PPGTT_ROOT_L3_ENTRY;
	} else if (desc->addressing_mode == 3) { /* legacy 64 bit */
		page_table_level = 4;
		root_entry_type = GTT_TYPE_PPGTT_ROOT_L4_ENTRY;
	} else {
		gvt_err("Advanced Context mode(SVM) is not supported!\n");
		return false;
	}

	gvt_get_context_pdp_root_pointer(workload->vgt, workload->ring_context, pdp);

	mm = gvt_create_mm(workload->vgt, GVT_MM_PPGTT, root_entry_type,
			pdp, page_table_level, 0);
	if (!mm) {
		gvt_err("fail to create mm object.\n");
		return false;
	}

	workload->shadow_mm = mm;

	return true;
}

bool submit_context(struct vgt_device *vgt, int ring_id,
		struct execlist_ctx_descriptor_format *desc)
{
	struct list_head *q = workload_q_head(vgt, ring_id);
	struct gvt_workload *last_workload = list_empty(q) ? NULL :
			container_of(q->prev, struct gvt_workload, list);
	struct gvt_workload *workload = NULL;

	struct execlist_ring_context *ring_context = get_ring_context(
			vgt, desc->lrca);

	u32 head, tail, start, ctl;

	if (!ring_context) {
		gvt_err("invalid guest context LRCA: %x", desc->lrca);
		return false;
	}

	hypervisor_read_va(vgt, &ring_context->ring_header.val,
			&head, 4, 1);

	hypervisor_read_va(vgt, &ring_context->ring_tail.val,
			&tail, 4, 1);

	head &= RB_HEAD_OFF_MASK;
	tail &= RB_TAIL_OFF_MASK;

	if (last_workload && same_context(&last_workload->ctx_desc, desc)) {
		gvt_dbg_el("ring id %d same workload as last workload", ring_id);
		if (last_workload->dispatched) {
			gvt_dbg_el("ring id %d last workload has been dispatched",
					ring_id);
			gvt_dbg_el("ctx head %x real head %lx",
					head, last_workload->rb_tail);
			/*
			 * cannot use guest context head pointer here,
			 * as it might not be updated at this time
			 */
			head = last_workload->rb_tail;
		} else {
			gvt_dbg_el("ring id %d merged into last workload", ring_id);
			/*
			 * if last workload hasn't been dispatched (scanned + shadowed),
			 * and the context for current submission is just the same as last
			 * workload context, then we can merge this submission into
			 * last workload.
			 */
			last_workload->rb_tail = tail;
			return true;
		}
	}

	gvt_dbg_el("ring id %d begin a new workload", ring_id);

	workload = kzalloc(sizeof(*workload), GFP_KERNEL);
	if (!workload) {
		gvt_err("fail to allocate memory for workload");
		return false;
	}

	/* record some ring buffer register values for scan and shadow */
	hypervisor_read_va(vgt, &ring_context->rb_start.val,
			&start, 4, 1);
	hypervisor_read_va(vgt, &ring_context->rb_ctrl.val,
			&ctl, 4, 1);

	INIT_LIST_HEAD(&workload->list);

	init_waitqueue_head(&workload->shadow_ctx_status_wq);
	atomic_set(&workload->shadow_ctx_active, 0);

	workload->vgt = vgt;
	workload->ring_id = ring_id;
	workload->ctx_desc = *desc;
	workload->ring_context = ring_context;
	workload->rb_head = head;
	workload->rb_tail = tail;
	workload->rb_start = start;
	workload->rb_ctl = ctl;
	workload->complete = execlist_workload_complete;
	workload->status = -EINPROGRESS;

	gvt_dbg_el("workload %p ring id %d head %x tail %x start %x ctl %x",
			workload, ring_id, head, tail, start, ctl);

	if (!prepare_workload(workload)) {
		kfree(workload);
		return false;
	}

	queue_workload(workload);

	return true;
}

bool gvt_execlist_elsp_submit(struct vgt_device *vgt, int ring_id)
{
	struct gvt_virtual_execlist_info *info =
		&vgt->virtual_execlist_info[ring_id];
	struct execlist_ctx_descriptor_format *desc[2], valid_desc[2];
	unsigned long valid_desc_bitmap = 0;
	int i;

	memset(valid_desc, 0, sizeof(valid_desc));

	desc[0] = (struct execlist_ctx_descriptor_format *)&info->bundle.data[2];
	desc[1] = (struct execlist_ctx_descriptor_format *)&info->bundle.data[0];

	for (i = 0; i < 2; i++) {
		if (!desc[i]->valid)
			continue;

		if (!desc[i]->privilege_access) {
			gvt_err("[vgt %d] unexpected GGTT elsp submission", vgt->id);
			return false;
		}

		/* TODO: add another guest context checks here. */
		set_bit(i, &valid_desc_bitmap);
		valid_desc[i] = *desc[i];
	}

	if (!valid_desc_bitmap) {
		gvt_err("[vgt %d] no valid desc in a elsp submission",
				vgt->id);
		return false;
	}

	if (!test_bit(0, (void *)&valid_desc_bitmap) &&
			test_bit(1, (void *)&valid_desc_bitmap)) {
		gvt_err("[vgt %d] weird elsp submission, desc 0 is not valid",
				vgt->id);
		return false;
	}

	if (!emulate_execlist_schedule_in(info, valid_desc)) {
		gvt_err("[vgt %d] fail to emulate execlist schedule-in", vgt->id);
		return false;
	}

	/* submit workload */
	for_each_set_bit(i, (void *)&valid_desc_bitmap, 2) {
		if (!submit_context(vgt, ring_id, &valid_desc[i])) {
			gvt_err("[vgt %d] fail to schedule workload", vgt->id);
			return false;
		}
	}
	return true;
}

static bool init_virtual_execlist_info(struct vgt_device *vgt,
		int ring_id, struct gvt_virtual_execlist_info *info)
{
	struct execlist_context_status_pointer_format ctx_status_ptr;
	u32 ctx_status_ptr_reg;

	memset(info, 0, sizeof(*info));

	info->vgt = vgt;
	info->ring_id = ring_id;
	info->execlist[0].index = 0;
	info->execlist[1].index = 1;

	INIT_LIST_HEAD(&info->workload_q_head);

	ctx_status_ptr_reg = execlist_ring_mmio(info->ring_id,
			_EL_OFFSET_STATUS_PTR);

	ctx_status_ptr.dw = __vreg(vgt, ctx_status_ptr_reg);
	ctx_status_ptr.read_ptr = ctx_status_ptr.write_ptr = 0x7;
	__vreg(vgt, ctx_status_ptr_reg) = ctx_status_ptr.dw;

	return true;
}

bool gvt_init_virtual_execlist_info(struct vgt_device *vgt)
{
	int i;

	atomic_set(&vgt->running_workload_num, 0);

	/* each ring has a virtual execlist engine */
	for (i = 0; i < I915_NUM_RINGS; i++)
		init_virtual_execlist_info(vgt,
				i, &vgt->virtual_execlist_info[i]);

	return true;
}
