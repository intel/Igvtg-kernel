/*
 * BDW EXECLIST supports
 *
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

#include "vgt.h"
#include "trace.h"

/* #define EL_SLOW_DEBUG */

#define EXECLIST_CTX_PAGES(ring_id)	((ring_id) == RING_BUFFER_RCS ? 20 : 2)

#define ROOT_POINTER_2_CTX_STATE(state, root, i)	\
do{							\
	state->pdp##i##_LDW.val = root[(i)<<1];		\
	state->pdp##i##_UDW.val = root[((i) << 1) + 1];	\
}while(0);

#define CTX_STATE_2_ROOT_POINTER(root, state, i)	\
do{							\
	root[(i) << 1] = state->pdp##i##_LDW.val;	\
	root[((i) << 1) + 1] = state->pdp##i##_UDW.val;	\
}while(0);

#define ROOTP_CTX_STATE_2_CTX_STATE(dst, src, i)	\
do{							\
	dst->pdp##i##_LDW.val = src->pdp##i##_LDW.val;	\
	dst->pdp##i##_UDW.val = src->pdp##i##_UDW.val;	\
}while(0);

#define CTX_IS_SCHEDULED_OUT(ctx_status)		\
((ctx_status)->preempted ||				\
 (ctx_status)->element_switch ||			\
 (ctx_status)->active_to_idle ||			\
 (ctx_status)->context_complete ||			\
 (ctx_status)->wait_on_sync_flip ||			\
 (ctx_status)->wait_on_vblank ||			\
 (ctx_status)->wait_on_semaphore ||			\
 (ctx_status)->wait_on_scanline)

#define WAIT_FOR_RING_DONE(ring_base, count)			\
do {								\
	int j = 0;						\
	do {							\
		vgt_reg_t val;					\
		j ++;						\
		val = VGT_MMIO_READ(vgt->pdev,			\
			(ring_base) + 0x6c);			\
		if ((val & 0xfffe) == 0xfffe)			\
			break;					\
		if (j == count) {				\
			vgt_err("Did not get INSTDONE(0x%x)"	\
			" done bit set! reg value: 0x%x.\n",	\
			(ring_base) + 0x6c, val);		\
			break;					\
		}						\
	} while(1);						\
} while(0);

static inline struct reg_state_ctx_header *
vgt_get_reg_state_from_lrca(struct vgt_device *vgt, uint32_t lrca);
/* trace the queue ops: 0 for enqueue, 1 for dequeue, 2 for delete */
static void inline trace_el_queue_ops(struct vgt_device *vgt, int ring_id, int el_idx, int ops)
{
	int i;
	char str[128];
	int head = vgt_el_queue_head(vgt, ring_id);
	int tail = vgt_el_queue_tail(vgt, ring_id);

	/* if it was enqueue, the queue should not be empty now */
	if (ops == 0)
		ASSERT(head != tail);

	for (i = 0; i < 2; ++ i) {
		struct execlist_context *ctx;
		struct reg_state_ctx_header *guest_state;
		uint32_t lrca;
		ctx = vgt_el_queue_ctx(vgt, ring_id, el_idx, i);
		if (!ctx)
			continue;

		if (vgt_require_shadow_context(vgt))
			guest_state = (struct reg_state_ctx_header *)
				ctx->ctx_pages[1].guest_page.vaddr;
		else
			guest_state = vgt_get_reg_state_from_lrca(vgt,
				ctx->guest_context.lrca);

		lrca = ctx->guest_context.lrca;
		snprintf(str, 128, "slot[%d] ctx[%d] %s "
				"(queue head: %d; tail: %d) "
				"(guest rb head: 0x%x; tail: 0x%x)",
			el_idx, i,
			(ops == 0 ? "enqueue" : (ops == 1 ? "dequeue" : "delete")),
			head, tail,
			guest_state->ring_header.val, guest_state->ring_tail.val);
		trace_ctx_lifecycle(vgt->vm_id, ring_id, lrca, str);
	}
}

/* util functions */

static inline enum vgt_ring_id el_mmio_to_ring_id(unsigned int reg)
{
	enum vgt_ring_id ring_id = MAX_ENGINES;
	switch (reg) {
	case _REG_RCS_CTX_SR_CTL:
	case _REG_RCS_HEAD:
	case _REG_RCS_TAIL:
	case _REG_RCS_START:
	case _REG_RCS_CTL:
	case 0x2168:
	case _REG_RCS_BB_ADDR:
	case 0x2110:
	case 0x211C:
	case 0x2114:
	case 0x2118:
	case 0x21C0:
	case 0x21C4:
	case 0x21C8:
		ring_id = RING_BUFFER_RCS;
		break;
	case _REG_VCS_CTX_SR_CTL:
		ring_id = RING_BUFFER_VCS;
		break;
	case _REG_VECS_CTX_SR_CTL:
		ring_id = RING_BUFFER_VECS;
		break;
	case _REG_VCS2_CTX_SR_CTL:
		ring_id = RING_BUFFER_VCS2;
		break;
	case _REG_BCS_CTX_SR_CTL:
		ring_id = RING_BUFFER_BCS;
		break;
	default:
		break;
	}

	return ring_id;
}

static inline struct reg_state_ctx_header *
vgt_get_reg_state_from_lrca(struct vgt_device *vgt, uint32_t lrca)
{
	struct reg_state_ctx_header *header;
	uint32_t state_gma = (lrca + 1) << GTT_PAGE_SHIFT;

	header = (struct reg_state_ctx_header *)
			vgt_gma_to_va(vgt->gtt.ggtt_mm, state_gma);
	return header;
}

static inline enum vgt_ring_id vgt_get_ringid_from_lrca(struct vgt_device *vgt,
				unsigned int lrca)
{
	enum vgt_ring_id ring_id = MAX_ENGINES;
	struct reg_state_ctx_header *reg_state;

	reg_state = vgt_get_reg_state_from_lrca(vgt, lrca);

	if (reg_state == NULL)
		return ring_id;

	ring_id = el_mmio_to_ring_id(reg_state->ctx_ctrl.addr);

	return ring_id;
}

static int vgt_create_shadow_rb(struct vgt_device *vgt, struct execlist_context *el_ctx);
static void vgt_destroy_shadow_rb(struct vgt_device *vgt, struct execlist_context *el_ctx);
static void vgt_release_shadow_cmdbuf(struct vgt_device *vgt, struct shadow_batch_buffer *p);
static int vgt_create_shadow_indirect_ctx(struct vgt_device *vgt, struct execlist_context *el_ctx);
static void vgt_destroy_shadow_indirect_ctx(struct vgt_device *vgt, struct execlist_context *el_ctx);

/* a queue implementation
 *
 * It is used to hold the submitted execlists through writing ELSP.
 * In maximum VM can submit two execlists. The queue size
 * is designed to be 3 to better recognize the queue full and empty. Queue
 * tail points to the next slot to be written, whereas header points to the
 * slot to be addressed. (header == tail) means the queue is full.
 *
 * The reason to use a queue is to keep the information of submission order.
 */

static bool vgt_el_slots_enqueue(struct vgt_device *vgt,
			enum vgt_ring_id ring_id,
			struct execlist_context *ctx0,
			struct execlist_context *ctx1)
{
	struct vgt_exec_list *el_slot;
	int tail = vgt_el_queue_tail(vgt, ring_id);
	int new_tail = tail + 1;
	if (new_tail == EL_QUEUE_SLOT_NUM)
		new_tail = 0;

	if (new_tail == vgt_el_queue_head(vgt, ring_id)) {
		return false;
	}
	el_slot = &vgt_el_queue_slot(vgt, ring_id, tail);
	el_slot->el_ctxs[0] = ctx0;
	el_slot->el_ctxs[1] = ctx1;
	el_slot->status = EL_PENDING;
	vgt_el_queue_tail(vgt, ring_id) = new_tail;
	trace_el_queue_ops(vgt, ring_id, tail, 0);
	return true;
}
#if 0
static int vgt_el_slots_dequeue(struct vgt_device *vgt, enum vgt_ring_id ring_id)
{
	int new_head;
	int head = vgt_el_queue_head(vgt, ring_id);

	if (head == vgt_el_queue_tail(vgt, ring_id)) {
		// queue empty
		return -1;
	}

	new_head = head + 1;
	if (new_head == EL_QUEUE_SLOT_NUM)
		new_head = 0;

	vgt_el_queue_head(vgt, ring_id) = new_head;

	trace_el_queue_ops(vgt, ring_id, head, 1);

	return head;
}
#endif
static void vgt_el_slots_delete(struct vgt_device *vgt,
			enum vgt_ring_id ring_id, int idx)
{
	struct vgt_exec_list *el_slot;
	int head = vgt_el_queue_head(vgt, ring_id);

	if (idx == head) {
		head ++;
		if (head == EL_QUEUE_SLOT_NUM)
			head = 0;
		vgt_el_queue_head(vgt, ring_id) = head;
	} else {
		int idx_next = idx + 1;
		if (idx_next == EL_QUEUE_SLOT_NUM)
			idx_next = 0;
		ASSERT(idx_next == vgt_el_queue_tail(vgt, ring_id));
		vgt_el_queue_tail(vgt, ring_id) = idx;
	}

	trace_el_queue_ops(vgt, ring_id, idx, 2);

	el_slot = &vgt_el_queue_slot(vgt, ring_id, idx);
	el_slot->status = EL_EMPTY;
	el_slot->el_ctxs[0] = NULL;
	el_slot->el_ctxs[1] = NULL;
}

/* search the queue with FIFO order */
static void vgt_el_slots_find_submitted_ctx(vgt_state_ring_t *ring_state,
			uint32_t ctx_id, int *el_slot_idx, int *el_slot_ctx_idx)
{
	int head = ring_state->el_slots_head;
	int tail = ring_state->el_slots_tail;

	*el_slot_idx = -1;
	*el_slot_ctx_idx = -1;

	while ((head != tail) && (*el_slot_idx == -1)) {
		int i;
		struct vgt_exec_list *el_slot;

		el_slot = &ring_state->execlist_slots[head];
		if (el_slot->status != EL_SUBMITTED)
			continue;

		for (i = 0; i < 2; ++ i) {
			struct execlist_context *p = el_slot->el_ctxs[i];
			if ((p && p->guest_context.context_id == ctx_id) ||
			    (p && ctx_id == 0)) {
				*el_slot_idx = head;
				*el_slot_ctx_idx = i;
				break;
			}
		}

		head ++;
		if (head == EL_QUEUE_SLOT_NUM)
			head = 0;
	}
}

static int vgt_el_slots_next_sched(vgt_state_ring_t *ring_state)
{
	int head = ring_state->el_slots_head;
	int tail = ring_state->el_slots_tail;
	if (head == tail) {
		// queue empty
		return -1;
	} else {
		while (ring_state->execlist_slots[head].status != EL_PENDING) {
			head++;
			if (head == EL_QUEUE_SLOT_NUM)
				head = 0;
			if (head == tail) {
				head = -1;
				break;
			}
		}
		return head;
	}
}

int vgt_el_slots_number(vgt_state_ring_t *ring_state)
{
	int num;
	int head = ring_state->el_slots_head;
	int tail = ring_state->el_slots_tail;

	if (tail >= head)
		num = tail - head;
	else
		num = tail + EL_QUEUE_SLOT_NUM - head;

	return num;
}

/* validation functions */

static inline bool el_lrca_is_valid(struct vgt_device *vgt, uint32_t lrca)
{
	bool rc;
	uint32_t gma;

	gma = lrca << GTT_PAGE_SHIFT;
	rc = g_gm_is_valid(vgt, gma);
	if (!rc) {
		/* it is a shadow context */
		rc = g_gm_is_reserved(vgt, gma);
	}

	return rc;
}

static inline bool vgt_validate_elsp_descs(struct vgt_device *vgt,
			struct ctx_desc_format *ctx0,
			struct ctx_desc_format *ctx1)
{
	if (!ctx0->valid) {
		vgt_err("Context[0] is invalid! Which is not expected\n");
		return false;
	}

	if (!el_lrca_is_valid(vgt, ctx0->lrca)) {
		vgt_err("The context[0] in ELSP does not have a valid lrca(0x%x)!",
				ctx0->lrca);
		return false;
	}

	if (ctx1->valid) {
		if (!el_lrca_is_valid(vgt, ctx1->lrca)) {
			vgt_err("The context[1] in ELSP does not have a "
				"valid lrca(0x%x)!", ctx1->lrca);
			return false;
		}
		if ((ctx0->elm_low == ctx1->elm_low) &&
		    (ctx0->elm_high == ctx1->elm_high)) {
			/* warning only */
			vgt_warn("duplicated context submission!\n");
		}
	}

	return true;
}

static bool vgt_validate_elsp_submission(struct vgt_device *vgt,
			struct vgt_elsp_store *elsp_store)
{
	struct ctx_desc_format *ctx0;
	struct ctx_desc_format *ctx1;
	ctx0 = (struct ctx_desc_format *)&elsp_store->element[2];
	ctx1 = (struct ctx_desc_format *)&elsp_store->element[0];

	return vgt_validate_elsp_descs(vgt, ctx0, ctx1);
}

static bool vgt_validate_status_entry(struct vgt_device *vgt,
			enum vgt_ring_id ring_id,
			struct context_status_format *status)
{
	int i;

	struct vgt_device *v_try = NULL;
	struct execlist_context *el_ctx;

	/* FIXME
	 * a hack here to treat context_id as lrca. That is the current usage
	 * in both linux and windows gfx drivers, and it is currently the only
	 * way to get lrca from status.
	 * The value is used to check whether a given context status belongs to
	 * vgt.
	 *
	 * When context_id does not equal to lrca some day, the check will not
	 * be valid.
	 */
	uint32_t lrca = status->context_id;

	if (lrca == 0) {
		/* Always return true for lrca as 0. Context status buffer
		 * contains valid entry with lrca/ctx_id as 0, especially
		 * for the idle_to_active events.
		 */
		return true;
	}
	if (el_lrca_is_valid(vgt, lrca))
		return true;

	el_ctx = execlist_context_find(vgt, lrca);
	if (el_ctx == NULL) {
		vgt_err("VM-%d sees unknown contextID/lrca (0x%x) "
			"in status buffer!\n", vgt->vm_id, lrca);
		return false;
	}

	/* only report error once for one context */
	if (el_ctx->error_reported == 0)
		el_ctx->error_reported = 1;
	else
		return false;

	/* try to figure out which VM the lrca belongs to.
	 * It is doable only when shadow context is not used.
	 */
	if (shadow_execlist_context == PATCH_WITHOUT_SHADOW) {
		for (i = 0; i < VGT_MAX_VMS; ++ i) {
			struct vgt_device *v = vgt->pdev->device[i];
			if (!v)
				continue;
			if (el_lrca_is_valid(v, lrca)) {
				v_try = v;
				break;
			}
		}

		if (v_try) {
			vgt_err("The context(lrca: 0x%x) is given to VM-%d "
			"But it belongs to VM-%d actually!\n",
			lrca, vgt->vm_id, v_try->vm_id);
		} else {
			vgt_err("The given context(lrca: 0x%x) in status "
			"buffer does not belong to any VM!\n", lrca);
		}
	}

	dump_ctx_status_buf(vgt, ring_id, true);
	dump_el_context_information(vgt, el_ctx);

	vgt_err("The context(lrca: 0x%x) in status buffer will be ignored.\n", lrca);
	return false;
}

/* context shadow: write protection handler */

static inline void vgt_set_wp_guest_ctx(struct vgt_device *vgt,
			struct execlist_context *el_ctx, int idx)
{
	enum vgt_ring_id ring_id = el_ctx->ring_id;

	hypervisor_set_wp_pages(vgt,
				&el_ctx->ctx_pages[idx].guest_page);
	trace_ctx_protection(vgt->vm_id, ring_id, el_ctx->guest_context.lrca,
				idx, el_ctx->ctx_pages[idx].guest_page.gfn,
				"set_writeprotection");
}

static inline void vgt_clear_wp_guest_ctx(struct vgt_device *vgt,
			struct execlist_context *el_ctx, int idx)
{
	enum vgt_ring_id ring_id = el_ctx->ring_id;

	hypervisor_unset_wp_pages(vgt,
				&el_ctx->ctx_pages[idx].guest_page);
	trace_ctx_protection(vgt->vm_id, ring_id, el_ctx->guest_context.lrca,
				idx, el_ctx->ctx_pages[idx].guest_page.gfn,
				"clear_writeprotection");
}

static bool sctx_mirror_state_wp_handler(void *gp, uint64_t pa, void *p_data, int bytes)
{
	guest_page_t *guest_page = (guest_page_t *)gp;
	struct execlist_context *el_ctx = (struct execlist_context *)guest_page->data;
	struct shadow_ctx_page *ctx_page = container_of(guest_page,
					struct shadow_ctx_page, guest_page);
	uint32_t offset = pa & (PAGE_SIZE - 1);

	if (!guest_page->writeprotection) {
		vgt_err("EXECLIST Ctx mirror wp handler is called without write protection! "
			"addr <0x%llx>, bytes %i\n", pa, bytes);
		return false;
	}

	trace_ctx_write_trap(el_ctx->guest_context.lrca,
				el_ctx->shadow_lrca, pa, bytes, *(uint32_t *)p_data);

	if ((offset & (bytes -1)) != 0)
		vgt_warn("Not aligned EXECLIST context update!");

	memcpy(((unsigned char *)ctx_page->guest_page.vaddr) + offset,
				p_data, bytes);
	memcpy(((unsigned char *)ctx_page->shadow_page.vaddr) + offset,
				p_data, bytes);

	return true;
}

#define check_ldw_offset(offset, i, bytes)				\
do{									\
	int pdp_ldw;							\
	pdp_ldw = offsetof(struct reg_state_ctx_header, pdp##i##_LDW);	\
	if (((offset == pdp_ldw) && (bytes == 8)) ||			\
		((offset == pdp_ldw + 4) && (bytes == 4)))		\
		return i;						\
}while(0);

static int ctx_offset_2_rootp_idx(uint32_t offset, int bytes)
{
	check_ldw_offset(offset, 0, bytes);
	check_ldw_offset(offset, 1, bytes);
	check_ldw_offset(offset, 2, bytes);
	check_ldw_offset(offset, 3, bytes);

	return -1;
}

static bool sctx_reg_state_wp_handler(void *gp, uint64_t pa, void *p_data, int bytes)
{
	guest_page_t *guest_page = (guest_page_t *)gp;
	struct execlist_context *el_ctx = (struct execlist_context *)guest_page->data;

	uint32_t offset = pa & (PAGE_SIZE - 1);
	int idx;
	bool rc;

	if (!guest_page->writeprotection) {
		vgt_err("EXECLIST Ctx regstate wp handler is called without write protection! "
			"addr <0x%llx>, bytes %i\n", pa, bytes);
		return false;
	}

	rc = sctx_mirror_state_wp_handler(gp, pa, p_data, bytes);
	if (!rc)
		return rc;

	if ((offset & (bytes -1)) != 0) {
		vgt_warn("Not aligned write found in EXECLIST ctx wp handler. "
			"addr <0x%llx>, bytes <%i>", pa, bytes);
	}

	if ((bytes != 4) && (bytes != 8)) {
		/* FIXME Do not expect it is the chagne to root pointers.
		 * So return directly here. Add more check in future.
		 */
		return true;
	}

	idx = ctx_offset_2_rootp_idx(offset, bytes);
	if (idx != -1) {
		vgt_dbg(VGT_DBG_EXECLIST, "wp handler: Emulate the rootp[%d] change\n", idx);
		rc = vgt_handle_guest_write_rootp_in_context(el_ctx, idx);
	}

	return rc;
}

#define CHECK_CTX_VAL(MMIO, GUEST, REF, SRC)				\
do {									\
	if(GUEST->MMIO.val != REF->MMIO.val)				\
		trace_printk("CTX_SYNC_CHECK: "				\
			"guest "#MMIO"<0x%x> overwritten "		\
			"by shadow update with 0x%x\n",			\
			GUEST->MMIO.val, SRC->MMIO.val);\
}while(0);

static void check_guest_ctx_changes(struct reg_state_ctx_header *guest,
				    struct reg_state_ctx_header *ref,
				    struct reg_state_ctx_header *src)
{
	if (!shadow_ctx_check)
		return;

	CHECK_CTX_VAL(ctx_ctrl, guest, ref, src);
	CHECK_CTX_VAL(ring_header, guest, ref, src);
	CHECK_CTX_VAL(bb_cur_head_UDW, guest, ref, src);
	CHECK_CTX_VAL(bb_state, guest, ref, src);
	CHECK_CTX_VAL(second_bb_addr_UDW, guest, ref, src);
	CHECK_CTX_VAL(second_bb_addr_LDW, guest, ref, src);
	CHECK_CTX_VAL(second_bb_state, guest, ref, src);
	CHECK_CTX_VAL(bb_per_ctx_ptr, guest, ref, src);
	CHECK_CTX_VAL(rcs_indirect_ctx, guest, ref, src);
	CHECK_CTX_VAL(rcs_indirect_ctx_offset, guest, ref, src);
}

/* not to copy PDP root pointers */
static void update_guest_regstate_from_shadow(void *dest_page, void *src_page,
					      struct reg_state_ctx_header *g_ref)
{
	struct reg_state_ctx_header *dest_ctx;
	struct reg_state_ctx_header *src_ctx;
	int regstate_size = sizeof(struct reg_state_ctx_header);
	int pdp_offset = offsetof(struct reg_state_ctx_header, pdp3_UDW);
	int rbctrl_offset = offsetof(struct reg_state_ctx_header, rb_ctrl);
	int bbperctx_offset = offsetof(struct reg_state_ctx_header, bb_per_ctx_ptr);
	int ictxoffset_offset = offsetof(struct reg_state_ctx_header, rcs_indirect_ctx_offset);

	dest_ctx = (struct reg_state_ctx_header *)(dest_page);
	src_ctx = (struct reg_state_ctx_header *)(src_page);

	check_guest_ctx_changes(dest_ctx, g_ref, src_ctx);

	dest_ctx->lri_cmd_1 = src_ctx->lri_cmd_1;
	dest_ctx->ctx_ctrl.addr = src_ctx->ctx_ctrl.addr;
	dest_ctx->ctx_ctrl.val = src_ctx->ctx_ctrl.val;
	dest_ctx->ring_header.addr = src_ctx->ring_header.addr;
	dest_ctx->ring_header.val = src_ctx->ring_header.val;
	dest_ctx->ring_tail.addr = src_ctx->ring_tail.addr;
	dest_ctx->rb_start.addr = src_ctx->rb_start.addr;

	memcpy(dest_page + rbctrl_offset, src_page + rbctrl_offset,
	       bbperctx_offset - rbctrl_offset);

	dest_ctx->bb_per_ctx_ptr.addr = src_ctx->bb_per_ctx_ptr.addr;
	dest_ctx->rcs_indirect_ctx.addr = src_ctx->rcs_indirect_ctx.addr;

	memcpy(dest_page + ictxoffset_offset, src_page + ictxoffset_offset,
	       pdp_offset - ictxoffset_offset);

	dest_ctx->pdp0_LDW.addr = src_ctx->pdp0_LDW.addr;
	dest_ctx->pdp0_UDW.addr = src_ctx->pdp0_UDW.addr;
	dest_ctx->pdp1_LDW.addr = src_ctx->pdp1_LDW.addr;
	dest_ctx->pdp1_UDW.addr = src_ctx->pdp1_UDW.addr;
	dest_ctx->pdp2_LDW.addr = src_ctx->pdp2_LDW.addr;
	dest_ctx->pdp2_UDW.addr = src_ctx->pdp2_UDW.addr;
	dest_ctx->pdp3_LDW.addr = src_ctx->pdp3_LDW.addr;
	dest_ctx->pdp3_UDW.addr = src_ctx->pdp3_UDW.addr;

	memcpy(dest_page + regstate_size, src_page + regstate_size,
	       SIZE_PAGE - regstate_size);
	memcpy(g_ref, src_ctx, sizeof(struct reg_state_ctx_header));
}

#define ASSIGN_CHANGED_CTX_VAL(MMIO, SHADOW, REF, SRC)	\
do {							\
	if(SRC->MMIO.val != REF->MMIO.val)		\
		SHADOW->MMIO.val = SRC->MMIO.val;	\
}while(0);

/* perform check between src_page and ref_page, and only update the changed
 * fields from src to dest.
 */
static void update_shadow_regstate_from_guest(struct vgt_device *vgt,
					      struct execlist_context *el_ctx)
{
	bool tail_only = el_ctx->ctx_running;
	void *dest, *src;

	struct reg_state_ctx_header *dest_ctx;
	struct reg_state_ctx_header *src_ctx;
	struct reg_state_ctx_header *ref_ctx;
	int regstate_size = sizeof(struct reg_state_ctx_header);
	int pdp_offset = offsetof(struct reg_state_ctx_header, pdp3_UDW);

	dest = el_ctx->ctx_pages[1].shadow_page.vaddr;
	src = el_ctx->ctx_pages[1].guest_page.vaddr;

	dest_ctx = (struct reg_state_ctx_header *)dest;
	src_ctx = (struct reg_state_ctx_header *)src;
	ref_ctx = el_ctx->g_ctx_buf;

	if (tail_only) {
		dest_ctx->ring_tail.val = src_ctx->ring_tail.val;
		return;
	}

	if (!el_ctx->initialized || (ref_ctx == NULL)) {
		/* in the first submission, populate shadow context */
		memcpy(dest, src, pdp_offset);

		dest_ctx->pdp0_LDW.addr = src_ctx->pdp0_LDW.addr;
		dest_ctx->pdp0_UDW.addr = src_ctx->pdp0_UDW.addr;
		dest_ctx->pdp1_LDW.addr = src_ctx->pdp1_LDW.addr;
		dest_ctx->pdp1_UDW.addr = src_ctx->pdp1_UDW.addr;
		dest_ctx->pdp2_LDW.addr = src_ctx->pdp2_LDW.addr;
		dest_ctx->pdp2_UDW.addr = src_ctx->pdp2_UDW.addr;
		dest_ctx->pdp3_LDW.addr = src_ctx->pdp3_LDW.addr;
		dest_ctx->pdp3_UDW.addr = src_ctx->pdp3_UDW.addr;

		memcpy(dest + regstate_size, src + regstate_size,
			SIZE_PAGE - regstate_size);
		el_ctx->initialized = true;
	} else {
		/* only update changed value from guest context */
		ASSIGN_CHANGED_CTX_VAL(ctx_ctrl, dest_ctx, ref_ctx, src_ctx);
		ASSIGN_CHANGED_CTX_VAL(ring_header, dest_ctx, ref_ctx, src_ctx);
		ASSIGN_CHANGED_CTX_VAL(ring_tail, dest_ctx, ref_ctx, src_ctx);
		ASSIGN_CHANGED_CTX_VAL(rb_ctrl, dest_ctx, ref_ctx, src_ctx);
		ASSIGN_CHANGED_CTX_VAL(bb_cur_head_UDW, dest_ctx, ref_ctx, src_ctx);
		ASSIGN_CHANGED_CTX_VAL(bb_cur_head_LDW, dest_ctx, ref_ctx, src_ctx);
		ASSIGN_CHANGED_CTX_VAL(bb_state, dest_ctx, ref_ctx, src_ctx);
		ASSIGN_CHANGED_CTX_VAL(second_bb_addr_UDW, dest_ctx, ref_ctx, src_ctx);
		ASSIGN_CHANGED_CTX_VAL(second_bb_addr_LDW, dest_ctx, ref_ctx, src_ctx);
		ASSIGN_CHANGED_CTX_VAL(second_bb_state, dest_ctx, ref_ctx, src_ctx);
		ASSIGN_CHANGED_CTX_VAL(bb_per_ctx_ptr, dest_ctx, ref_ctx, src_ctx);
		ASSIGN_CHANGED_CTX_VAL(rcs_indirect_ctx, dest_ctx, ref_ctx, src_ctx);
		ASSIGN_CHANGED_CTX_VAL(rcs_indirect_ctx_offset, dest_ctx, ref_ctx, src_ctx);
	}

	memcpy(ref_ctx, src_ctx, sizeof(struct reg_state_ctx_header));

	/* update the shadow fields */
	if (shadow_cmd_buffer)
		dest_ctx->rb_start.val = el_ctx->shadow_rb.shadow_rb_base;

	if (shadow_indirect_ctx_bb) {
		dest_ctx->rcs_indirect_ctx.val =
			(dest_ctx->rcs_indirect_ctx.val &
				(~INDIRECT_CTX_ADDR_MASK)) |
				el_ctx->shadow_indirect_ctx.shadow_ctx_base;
		dest_ctx->bb_per_ctx_ptr.val =
			(dest_ctx->bb_per_ctx_ptr.val &
				(~BB_PER_CTX_ADDR_MASK)) |
				el_ctx->shadow_bb_per_ctx.shadow_bb_base;
	}

	ppgtt_update_shadow_ppgtt_for_ctx(vgt, el_ctx);
}

static void vgt_update_shadow_ctx_from_guest(struct vgt_device *vgt,
			struct execlist_context *el_ctx)
{

	if (!vgt_require_shadow_context(vgt))
		return;

	/* normal shadow does not need the guest-to-shadow sync-up
	 * since the update has been done in guest write protection handler.
	 */
	if ((shadow_execlist_context != LAZY_CTX_SHADOW) &&
	    (shadow_execlist_context != OPT_LAZY_CTX_SHADOW))
		return;

	if (shadow_ctx_check && el_ctx->s_ctx_buf && el_ctx->initialized) {
		if (!el_ctx->ctx_running &&
			(memcmp(el_ctx->s_ctx_buf,
				el_ctx->ctx_pages[1].shadow_page.vaddr,
				PAGE_SIZE) != 0))
			trace_printk("CTX_SYNC_CHECK: "
			     "shadow ctx changed from the last ctx save!\n");
	}

	/* only update the ring status shadow page. Other pages are not
	 * expected to be updated by guest driver.
	 */
	update_shadow_regstate_from_guest(vgt, el_ctx);
}

static void update_guest_hws_from_shadow(void *dest, void *src)
{
	int data_size = 0x20 << 2;
	memcpy(dest, src, data_size);
}

static void vgt_update_guest_ctx_from_shadow(struct vgt_device *vgt,
			enum vgt_ring_id ring_id,
			struct execlist_context *el_ctx)
{
	if (!vgt_require_shadow_context(vgt))
		return;

	if (shadow_execlist_context == OPT_LAZY_CTX_SHADOW) {
		/* only copy ring status page */
		void *dst = el_ctx->ctx_pages[1].guest_page.vaddr;
		void *src = el_ctx->ctx_pages[1].shadow_page.vaddr;

		ASSERT(dst && src);
		if (shadow_ctx_check && el_ctx->s_ctx_buf)
			memcpy(el_ctx->s_ctx_buf, src, PAGE_SIZE);
		update_guest_regstate_from_shadow(dst, src, el_ctx->g_ctx_buf);
	} else if (shadow_execlist_context == PATCH_WITHOUT_SHADOW) {
		/* Leave patched guest driver as it is since it is just
		 * a hack solution. It is working because normally guest
		 * will not read back the patched value.
		 */
	} else {
		int npages = EXECLIST_CTX_PAGES(ring_id);
		int i;

		if (ring_id == RING_BUFFER_RCS)
			npages -= 8;

		for (i = 0; i < npages; ++ i) {
			void *dst = el_ctx->ctx_pages[i].guest_page.vaddr;
			void *src = el_ctx->ctx_pages[i].shadow_page.vaddr;

			ASSERT(dst && src);
			if (i == 0) {
				update_guest_hws_from_shadow(dst, src);
			} else if (i == 1) {
				if (shadow_ctx_check && el_ctx->s_ctx_buf)
					memcpy(el_ctx->s_ctx_buf, src, PAGE_SIZE);
				update_guest_regstate_from_shadow(dst, src,
					el_ctx->g_ctx_buf);
			} else {
				memcpy(dst, src, SIZE_PAGE);
			}
		}
	}

	trace_ctx_lifecycle(vgt->vm_id, ring_id, el_ctx->guest_context.lrca, "sync to guest");
}

static void vgt_patch_guest_context(struct execlist_context *el_ctx)
{
	struct reg_state_ctx_header *guest_state;
	struct reg_state_ctx_header *shadow_state;

	guest_state = (struct reg_state_ctx_header *)
			el_ctx->ctx_pages[1].guest_page.vaddr;
	shadow_state = (struct reg_state_ctx_header *)
			el_ctx->ctx_pages[1].shadow_page.vaddr;

	ROOTP_CTX_STATE_2_CTX_STATE(guest_state, shadow_state, 0);
	ROOTP_CTX_STATE_2_CTX_STATE(guest_state, shadow_state, 1);
	ROOTP_CTX_STATE_2_CTX_STATE(guest_state, shadow_state, 2);
	ROOTP_CTX_STATE_2_CTX_STATE(guest_state, shadow_state, 3);

	if (shadow_cmd_buffer)
		guest_state->rb_start.val = el_ctx->shadow_rb.shadow_rb_base;
}

/* context shadow: context creation/destroy in execlist */

static struct execlist_context *vgt_allocate_el_context(struct vgt_device *vgt,
				struct ctx_desc_format *ctx_desc)
{
	uint32_t guest_lrca;
	struct execlist_context *el_ctx;

	el_ctx = kzalloc(sizeof(struct execlist_context), GFP_ATOMIC);
	if (!el_ctx) {
		vgt_err("Failed to allocate data structure for shadow context!\n");
		return NULL;
	}

	memcpy(&el_ctx->guest_context, ctx_desc, sizeof(struct ctx_desc_format));

	guest_lrca = el_ctx->guest_context.lrca;
	hash_add(vgt->gtt.el_ctx_hash_table, &el_ctx->node, guest_lrca);

	return el_ctx;
}

static void vgt_free_el_context(struct execlist_context *el_ctx)
{
	if (el_ctx == NULL)
		return;

	hash_del(&el_ctx->node);
	kfree(el_ctx);
}

static int vgt_create_shadow_pages(struct vgt_device *vgt, struct execlist_context *el_ctx)
{
	uint32_t ring_id = el_ctx->ring_id;
	uint32_t ctx_pages = EXECLIST_CTX_PAGES(ring_id);
	struct vgt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	unsigned long hpa;
	uint32_t size;
	uint32_t rsvd_pages_idx;
	unsigned long g_gma;
	unsigned long s_gma;
	int i;

	size = (EXECLIST_CTX_PAGES(ring_id) << GTT_PAGE_SHIFT);
	hpa = rsvd_aperture_alloc(vgt->pdev, size);
	if (hpa == 0) {
		vgt_err("VM-%d: Failed to allocate gm for shadow context!\n",
			vgt->vm_id);
		return -1;
	}

	g_gma = ((unsigned long)el_ctx->guest_context.lrca) << GTT_PAGE_SHIFT;
	s_gma = aperture_2_gm(vgt->pdev, hpa);

	el_ctx->shadow_lrca = s_gma >> GTT_PAGE_SHIFT;

	rsvd_pages_idx = aperture_page_idx(vgt->pdev, s_gma);

	for (i = 0; i < ctx_pages; ++ i) {
		shadow_page_t *p_shadow;
		guest_page_t *p_guest;
		p_shadow = &el_ctx->ctx_pages[i].shadow_page;
		p_guest = &el_ctx->ctx_pages[i].guest_page;
		if ((shadow_execlist_context == OPT_LAZY_CTX_SHADOW) &&
			(i != 1)) {
			gtt_entry_t gtt_entry;
			/* backup reserved gtt entry and set guest ctx's adddress */
			el_ctx->shadow_entry_backup[i].pdev = vgt->pdev;
			el_ctx->shadow_entry_backup[i].type = GTT_TYPE_GGTT_PTE;
			ops->get_entry(NULL, &el_ctx->shadow_entry_backup[i],
				s_gma >> GTT_PAGE_SHIFT, false, NULL);
			gtt_entry.pdev = vgt->pdev;
			gtt_entry.type = GTT_TYPE_GGTT_PTE;
			ops->get_entry(NULL, &gtt_entry, g_gma >> GTT_PAGE_SHIFT, false, NULL);
			ops->set_entry(NULL, &gtt_entry, s_gma >> GTT_PAGE_SHIFT, false, NULL);
		} else {
			p_shadow->page = aperture_page(vgt->pdev, rsvd_pages_idx);
			p_shadow->vaddr = page_address(p_shadow->page);
			memcpy(p_shadow->vaddr, p_guest->vaddr, SIZE_PAGE);
		}

		g_gma += PAGE_SIZE;
		s_gma += PAGE_SIZE;
		rsvd_pages_idx ++;
	}

	return 0;
}

static void vgt_destroy_shadow_pages(struct vgt_device *vgt, struct execlist_context *el_ctx)
{
	unsigned long hpa;
	uint32_t ring_id = el_ctx->ring_id;
	uint32_t ctx_pages = EXECLIST_CTX_PAGES(ring_id);
	struct vgt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	int i;

	if (el_ctx->shadow_lrca == 0)
		return;

	for (i = 0; i < ctx_pages; ++ i) {
		if ((shadow_execlist_context == OPT_LAZY_CTX_SHADOW) &&
			(i != 1)) {
			ops->set_entry(NULL, &el_ctx->shadow_entry_backup[i],
						 el_ctx->shadow_lrca + i, false, NULL);
		}
	}

	hpa = phys_aperture_base(vgt->pdev) + (el_ctx->shadow_lrca << GTT_PAGE_SHIFT);
	rsvd_aperture_free(vgt->pdev, hpa, ctx_pages << GTT_PAGE_SHIFT);

	return;
}

static int vgt_el_create_shadow_context(struct vgt_device *vgt,
				enum vgt_ring_id ring_id,
				struct execlist_context *el_ctx)
{
	struct vgt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	uint32_t guest_lrca = el_ctx->guest_context.lrca;
	int ret = 0;
	int i;

	/* init guest context */
	for (i = 0; i < EXECLIST_CTX_PAGES(ring_id); ++ i) {
		gtt_entry_t e;
		guest_page_t *p_guest;
		unsigned long gfn;
		guest_page_handler_t *handler;

		p_guest = &el_ctx->ctx_pages[i].guest_page;

		e.pdev = vgt->pdev;
		e.type = GTT_TYPE_GGTT_PTE;
		ggtt_get_guest_entry(vgt->gtt.ggtt_mm, &e, guest_lrca + i);

		gfn = ops->get_pfn(&e);
		handler = ((i == 1) ? sctx_reg_state_wp_handler :
				    sctx_mirror_state_wp_handler);

		if (false == vgt_init_guest_page(vgt, p_guest, gfn,
						 handler, &el_ctx)) {
			vgt_err("VM-%d: Failed to init guest ctx page!\n", vgt->vm_id);
			ret = -1;
			break;
		}

		el_ctx->ctx_pages[i].vgt = vgt;

		if (shadow_execlist_context == NORMAL_CTX_SHADOW)
			vgt_set_wp_guest_ctx(vgt, el_ctx, i);
	}

	if (ret)
		goto cleanup_guest_pages;

	/* init shadow context */
	ret = vgt_create_shadow_pages(vgt, el_ctx);
	if (ret)
		goto cleanup_guest_pages;

	el_ctx->s_ctx_buf = kmalloc(PAGE_SIZE, GFP_ATOMIC);
	if (el_ctx->s_ctx_buf == NULL) {
		vgt_err("VM-%d: Failed to allocate memory for "
			"shadow context buffer!\n", vgt->vm_id);
		ret = -1;
		goto cleanup_guest_pages;
	}

	el_ctx->g_ctx_buf = kmalloc(sizeof(struct reg_state_ctx_header),
				    GFP_ATOMIC);
	if (el_ctx->g_ctx_buf == NULL) {
		vgt_err("VM-%d: Failed to allocate memory for "
			"guest context buffer!\n", vgt->vm_id);
		ret = -1;
		goto cleanup_guest_pages;
	}
	memcpy(el_ctx->g_ctx_buf,
		el_ctx->ctx_pages[1].guest_page.vaddr,
		sizeof(struct reg_state_ctx_header));
	return ret;

cleanup_guest_pages:
	for (i = 0; i < EXECLIST_CTX_PAGES(ring_id); ++ i) {
		guest_page_t *p_guest;
		p_guest = &el_ctx->ctx_pages[i].guest_page;
		if (p_guest->writeprotection)
			vgt_clear_wp_guest_ctx(vgt, el_ctx, i);
		vgt_clean_guest_page(vgt, p_guest);
	}
	if (el_ctx->s_ctx_buf) {
		kfree(el_ctx->s_ctx_buf);
		el_ctx->s_ctx_buf = NULL;
	}
	if (el_ctx->g_ctx_buf) {
		kfree(el_ctx->g_ctx_buf);
		el_ctx->g_ctx_buf = NULL;
	}
	return ret;
}

static int vgt_el_destroy_shadow_context(struct vgt_device *vgt,
					 enum vgt_ring_id ring_id,
					 struct execlist_context *el_ctx)
{
	int i;

	if (!vgt_require_shadow_context(vgt))
		return 0;

	for (i = 0; i < EXECLIST_CTX_PAGES(ring_id); ++ i) {
		guest_page_t *p_guest;
		p_guest = &el_ctx->ctx_pages[i].guest_page;

		if (p_guest->writeprotection)
			vgt_clear_wp_guest_ctx(vgt, el_ctx, i);

		vgt_clean_guest_page(vgt, p_guest);
	}

	vgt_destroy_shadow_pages(vgt, el_ctx);

	kfree(el_ctx->s_ctx_buf);
	el_ctx->s_ctx_buf = NULL;
	kfree(el_ctx->g_ctx_buf);
	el_ctx->g_ctx_buf = NULL;

	return 0;
}

int vgt_el_create_shadow_ppgtt(struct vgt_device *vgt,
				enum vgt_ring_id ring_id,
				struct execlist_context *el_ctx)
{
	struct vgt_mm *mm;
	u32 pdp[8] = {0};
	uint32_t *s_rootp;

	struct reg_state_ctx_header *reg_state;
	struct ctx_desc_format *guest_ctx = &el_ctx->guest_context;
	gtt_type_t root_entry_type;
	int page_table_level;

	if (guest_ctx->addressing_mode == 1) { /* legacy 32-bit */
		page_table_level = 3;
		root_entry_type = GTT_TYPE_PPGTT_ROOT_L3_ENTRY;
	} else if (guest_ctx->addressing_mode == 3) { /* legacy 64 bit */
		page_table_level = 4;
		root_entry_type = GTT_TYPE_PPGTT_ROOT_L4_ENTRY;
	} else {
		page_table_level = 4;
		root_entry_type = GTT_TYPE_PPGTT_ROOT_L4_ENTRY;
		vgt_err("Advanced Context mode(SVM) is not supported!\n");
	}

	if (vgt_require_shadow_context(vgt)) {
		reg_state = (struct reg_state_ctx_header *)
				el_ctx->ctx_pages[1].guest_page.vaddr;
	} else {
		ASSERT(vgt->vm_id == 0);
		reg_state = vgt_get_reg_state_from_lrca(vgt,
				el_ctx->guest_context.lrca);
	}

	CTX_STATE_2_ROOT_POINTER(pdp, reg_state, 0);
	if (page_table_level == 3) {
		CTX_STATE_2_ROOT_POINTER(pdp, reg_state, 1);
		CTX_STATE_2_ROOT_POINTER(pdp, reg_state, 2);
		CTX_STATE_2_ROOT_POINTER(pdp, reg_state, 3);
	}

	mm = gen8_find_ppgtt_mm(vgt, page_table_level, pdp);
	if (mm)
		goto ppgtt_creation_done;

	mm = vgt_create_mm(vgt, VGT_MM_PPGTT, root_entry_type,
			pdp, page_table_level, 0);
	if (!mm) {
		vgt_err("fail to create mm object.\n");
		return -1;
	}

	vgt_warn("Given PPGTT in EL context for creation is not yet constructed! "
		"It is not expected to happen! lrca = 0x%x\n",
		el_ctx->guest_context.lrca);
	dump_regstate_ctx_header(reg_state);

ppgtt_creation_done:
	el_ctx->ppgtt_mm = mm;

	if (!mm->has_shadow_page_table)
		goto finish;

	/* update root pointers in context with shadow ones */
	s_rootp = (uint32_t *)mm->shadow_page_table;
	reg_state = (struct reg_state_ctx_header *)
			el_ctx->ctx_pages[1].shadow_page.vaddr;

	ROOT_POINTER_2_CTX_STATE(reg_state, s_rootp, 0);
	if (page_table_level == 3) {
		ROOT_POINTER_2_CTX_STATE(reg_state, s_rootp, 1);
		ROOT_POINTER_2_CTX_STATE(reg_state, s_rootp, 2);
		ROOT_POINTER_2_CTX_STATE(reg_state, s_rootp, 3);
	}
finish:
	if (vgt_debug & VGT_DBG_EXECLIST) {
		vgt_dbg(VGT_DBG_EXECLIST,
			"VM-%d: The reg_state after shadow PPGTT creation:\n",
			vgt->vm_id);
		dump_el_context_information(vgt, el_ctx);
	}
	return 0;
}

static struct execlist_context *vgt_create_execlist_context(
					struct vgt_device *vgt,
					struct ctx_desc_format *ctx,
					enum vgt_ring_id ring_id)
{
	struct execlist_context *el_ctx;

	if (execlist_context_find(vgt, ctx->lrca) != NULL) {
		vgt_err("VM-%d: Trying to create a context which already exists!\n",
			vgt->vm_id);
		dump_ctx_desc(vgt, ctx);
		return NULL;
	}

	if (ring_id == MAX_ENGINES) {
		ring_id = vgt_get_ringid_from_lrca(vgt, ctx->lrca);
		if (ring_id == MAX_ENGINES) {
			vgt_warn("VM-%d: Invalid execlist context! "
			"Ring info is not available in ring context.\n",
					vgt->vm_id);
			dump_ctx_desc(vgt, ctx);
			return NULL;
		}
	}

	el_ctx = vgt_allocate_el_context(vgt, ctx);
	if (el_ctx == NULL)
		return NULL;

	el_ctx->ring_id = ring_id;
	INIT_LIST_HEAD(&el_ctx->shadow_priv_bb.pages);

	if (vgt_require_shadow_context(vgt)) {
		int ret;
		ret = vgt_el_create_shadow_context(vgt, ring_id, el_ctx);
		if(ret) {
			vgt_free_el_context(el_ctx);
			return NULL;
		}

		ret = vgt_create_shadow_rb(vgt, el_ctx);
		if (ret) {
			vgt_el_destroy_shadow_context(vgt, ring_id, el_ctx);
			vgt_free_el_context(el_ctx);
			return NULL;
		}

		ret = vgt_create_shadow_indirect_ctx(vgt, el_ctx);
		if (ret) {
			vgt_destroy_shadow_rb(vgt, el_ctx);
			vgt_el_destroy_shadow_context(vgt, ring_id, el_ctx);
			vgt_free_el_context(el_ctx);
			return NULL;
		}
	}

	trace_ctx_lifecycle(vgt->vm_id, ring_id,
			el_ctx->guest_context.lrca, "create");
	return el_ctx;
}

static void vgt_destroy_execlist_context(struct vgt_device *vgt,
				struct execlist_context *el_ctx)
{
	enum vgt_ring_id ring_id;

	if (el_ctx == NULL)
		return;

	ring_id = el_ctx->ring_id;
	if (ring_id == MAX_ENGINES) {
		vgt_err("VM-%d: Invalid execlist context!\n", vgt->vm_id);
		ASSERT_VM(0, vgt);
	}

	trace_ctx_lifecycle(vgt->vm_id, ring_id,
			el_ctx->guest_context.lrca, "destroy");

	/* free the shadow cmd buffers */
	vgt_destroy_shadow_rb(vgt, el_ctx);
	vgt_destroy_shadow_indirect_ctx(vgt, el_ctx);
	vgt_release_shadow_cmdbuf(vgt, &el_ctx->shadow_priv_bb);

	vgt_el_destroy_shadow_context(vgt, ring_id, el_ctx);
	vgt_free_el_context(el_ctx);
}

/* emulate the EXECLIST related MMIOs when vgt is not render owner,
 * so that guest drivers treat the submission as a successful one.
 * Currently we simply emulate the status register (234h) to reflect
 * the active execlist, which is a must for the future other execlist
 * submission. Others, like status buffer, keep unchanged. That gives
 * guest driver the impression that submission has finished, but the
 * contexts have not yet entered hardware.
 *
 * There is another option to emulate more here, for instance, to send
 * virtual context switch interrupt of idle-to-active for the first
 * execlist submission, and update virtual status buffer accordingly.
 * But such emulation will bring complexity when the real ELSP write
 * happens. We have to recognize the duplicated physical context switch
 * interrupt and delete that one.
 */

static void vgt_emulate_submit_execlist(struct vgt_device *vgt, int ring_id,
			struct execlist_context *ctx0,
			struct execlist_context *ctx1)
{
	struct execlist_status_format status;
	bool render_owner = is_current_render_owner(vgt);
	uint32_t status_reg = el_ring_mmio(ring_id, _EL_OFFSET_STATUS);
	uint32_t el_index;

	if (!vgt_el_slots_enqueue(vgt, ring_id, ctx0, ctx1)) {
		vgt_err("VM-%d: <ring-%d> EXECLIST slots are full while adding new contexts! "
			"Contexts will be ignored:\n"
			" -ctxid-0: 0x%x\n", vgt->vm_id, ring_id,
				ctx0->guest_context.context_id);
	}

	if (render_owner)
		return;

	/* emulate status register below */
	status.ldw = __vreg(vgt, status_reg);
	status.udw = __vreg(vgt, status_reg + 4);
	el_index = status.execlist_write_pointer;
	if (status.execlist_queue_full) {
		vgt_err("VM(%d): EXECLIST submission while the EL is full! "
			"The submission will be ignored!\n", vgt->vm_id);
		dump_execlist_info(vgt->pdev, ring_id);
		return;
	}

	status.execlist_write_pointer = (el_index == 0 ? 1 : 0);

	if (status.execlist_0_valid == 0 && status.execlist_1_valid == 0) {

		status.udw = ctx0->guest_context.context_id;

		/* TODO
		 * 1, Check whether we should set below two states. According to the observation
		 * from dom0, when there is ELSP write, both active bit and valid bit will be
		 * set.
		 * 2, Consider the emulation of preemption and lite restore.
		 * It is designed to be in context switch by adding corresponding status entries
		 * into status buffer.
		 */
		if (el_index == 0) {
			status.execlist_0_active = 1;
			status.execlist_0_valid = 1;
			status.execlist_1_active = 0;
			status.execlist_1_valid = 0;
		} else {
			status.execlist_0_active = 0;
			status.execlist_0_valid = 0;
			status.execlist_1_active = 1;
			status.execlist_1_valid = 1;
		}
		/*update cur pointer to next */
		status.current_execlist_pointer = el_index;
	}
	else {
		/* TODO emulate the status. Need double confirm
		 *
		 * Here non-render owner will still receive context switch interrupt
		 * injected because of HW GPU status change. Meanwhile, the status register
		 * is emulated to reflect the port submission operation.
		 */
		status.udw = ctx0->guest_context.context_id;
		status.execlist_queue_full = 1;
		vgt_dbg(VGT_DBG_EXECLIST,"VM-%d: ring(%d) EXECLISTS becomes "
			"full due to workload submission!\n",
				vgt->vm_id, ring_id);
		dump_execlist_status(&status, ring_id);
	}

	__vreg(vgt, status_reg) = status.ldw;
	__vreg(vgt, status_reg + 4) = status.udw;

	return;
}

struct execlist_context *execlist_shadow_context_find(struct vgt_device *vgt,
				uint32_t shadow_lrca)
{
	int i;
	struct execlist_context *el_ctx;

	hash_for_each(vgt->gtt.el_ctx_hash_table, i, el_ctx, node) {
		if (el_ctx->shadow_lrca == shadow_lrca)
			return el_ctx;
	}

	return NULL;
}

struct execlist_context * execlist_context_find(struct vgt_device *vgt,
				uint32_t guest_lrca)
{
	struct execlist_context *el_ctx;
	hash_for_each_possible(vgt->gtt.el_ctx_hash_table, el_ctx, node, guest_lrca) {
		if (el_ctx->guest_context.lrca == guest_lrca)
			return el_ctx;
	}

	return NULL;
}

/* guest context event emulation */

static inline void vgt_add_ctx_switch_status(struct vgt_device *vgt, enum vgt_ring_id ring_id,
			struct context_status_format *ctx_status)
{
	uint32_t ctx_status_reg;
	uint32_t write_idx;
	uint32_t offset;

	ctx_status_reg = el_ring_mmio(ring_id, _EL_OFFSET_STATUS_BUF);

	write_idx = vgt->rb[ring_id].csb_write_ptr;
	if (write_idx == DEFAULT_INV_SR_PTR) {
		write_idx = 0;
	} else {
		write_idx ++;
		if (write_idx >= CTX_STATUS_BUF_NUM)
			write_idx = 0;
	}

	offset = ctx_status_reg + write_idx * 8;
	__vreg(vgt, offset) = ctx_status->ldw;
	__vreg(vgt, offset + 4) = ctx_status->udw;

	vgt->rb[ring_id].csb_write_ptr = write_idx;
}

static void vgt_emulate_context_status_change(struct vgt_device *vgt,
				enum vgt_ring_id ring_id,
				struct context_status_format *ctx_status)
{
	vgt_state_ring_t *ring_state;
	uint32_t el_slot_ctx_idx = -1;
	uint32_t el_slot_idx = -1;
	struct vgt_exec_list *el_slot = NULL;
	struct execlist_context *el_ctx = NULL;
	uint32_t ctx_id = ctx_status->context_id;
	bool lite_restore;

	ring_state = &vgt->rb[ring_id];
	vgt_el_slots_find_submitted_ctx(ring_state, ctx_id,
				&el_slot_idx, &el_slot_ctx_idx);
	if (el_slot_idx == -1)
		goto err_ctx_not_found;

	el_slot = &vgt_el_queue_slot(vgt, ring_id, el_slot_idx);

	ASSERT((el_slot_ctx_idx == 0) || (el_slot_ctx_idx == 1));
	el_ctx = el_slot->el_ctxs[el_slot_ctx_idx];

	lite_restore = ctx_status->preempted && ctx_status->lite_restore;

	if (CTX_IS_SCHEDULED_OUT(ctx_status)) {
		char str[64];
		snprintf(str, 64, "finish_running. status[0x%x]", ctx_status->ldw);
		trace_ctx_lifecycle(vgt->vm_id, ring_id,
			el_ctx->guest_context.lrca,
			str);

		if (!lite_restore) {
			el_ctx->scan_head_valid = false;

			if (ctx_status->preempted && el_slot_ctx_idx == 0) {
				if (el_slot->el_ctxs[1])
					el_slot->el_ctxs[1]->scan_head_valid = false;
			}
		}

		if ((((el_slot_ctx_idx == 0) || (el_slot->el_ctxs[0] == NULL)) &&
			((el_slot_ctx_idx == 1) || (el_slot->el_ctxs[1] == NULL))) ||
			(ctx_status->preempted)) {
			vgt_el_slots_delete(vgt, ring_id, el_slot_idx);
		}
		el_slot->el_ctxs[el_slot_ctx_idx] = NULL;
		if (ctx_status->preempted && !ctx_status->lite_restore) {
			/* In preemption case need to find the next running ctx for status track,
			 * since the context sync-up between guest and shadow needs that.
			 * No need for lite_restore case.
			 */
			struct vgt_exec_list *next_slot = NULL;
			struct execlist_context *next_ctx = NULL;
			uint32_t next_slot_idx = -1;
			uint32_t next_ctx_idx = -1;
			vgt_el_slots_find_submitted_ctx(ring_state, 0,
					&next_slot_idx, &next_ctx_idx);
			if (next_ctx_idx != -1) {
				next_slot = &vgt_el_queue_slot(vgt, ring_id, next_slot_idx);
				next_ctx = next_slot->el_ctxs[next_ctx_idx];
				next_ctx->ctx_running = true;
			}
		} else if (ctx_status->element_switch) {
			struct execlist_context *next_ctx = NULL;
			if (el_slot_ctx_idx != 0) {
				vgt_warn("something wrong of element switch CSB status!\n");
			}
			next_ctx = el_slot->el_ctxs[1];
			next_ctx->ctx_running = true;
		}
	} else if (!ctx_status->idle_to_active) {
		goto emulation_done;
	}

	if (!vgt_require_shadow_context(vgt))
		goto emulation_done;

	if (ctx_status->idle_to_active) {
		el_ctx->ctx_running = true;
	} else {
		ASSERT (CTX_IS_SCHEDULED_OUT(ctx_status));
		el_ctx->ctx_running = ctx_status->lite_restore;
		el_ctx->sync_needed = !el_ctx->ctx_running;
	}

	if (vgt_debug & VGT_DBG_EXECLIST)
		dump_el_context_information(vgt, el_ctx);

emulation_done:
	return;
err_ctx_not_found:
	{
		static int warned_once = 0;
		if ((ctx_id != 0) && !warned_once) {
			warned_once = 1;
			vgt_err("VM(%d) Ring(%d): Trying to emulate context status change"
			" but did not find the shadow context in execlist!\n"
			"\t\tContext ID: 0x%x; status: 0x%x\n", vgt->vm_id, ring_id,
				ctx_id, ctx_status->ldw);
		}
	}
	return;
}

static void vgt_emulate_csb_updates(struct vgt_device *vgt, enum vgt_ring_id ring_id)
{
	struct ctx_st_ptr_format ctx_ptr_val;
	uint32_t ctx_ptr_reg;
	uint32_t ctx_status_reg;

	int read_idx;
	int write_idx;

	ctx_ptr_reg = el_ring_mmio(ring_id, _EL_OFFSET_STATUS_PTR);
	ctx_ptr_val.dw = VGT_MMIO_READ(vgt->pdev, ctx_ptr_reg);

	read_idx = el_read_ptr(vgt->pdev, ring_id);
	ASSERT(read_idx == ctx_ptr_val.status_buf_read_ptr);

	write_idx = el_write_ptr(vgt->pdev, ring_id);

#ifdef EL_SLOW_DEBUG
	if (vgt_debug & VGT_DBG_EXECLIST) {
		vgt_dbg(VGT_DBG_EXECLIST, "Physical CTX Status buffer is below:\n");
		dump_ctx_status_buf(vgt, ring_id, true);
		vgt_dbg(VGT_DBG_EXECLIST, "Virtual CTX Status buffer before buffer "
					"update is below:\n");
		dump_ctx_status_buf(vgt, ring_id, false);
	}
#endif
	if (write_idx == DEFAULT_INV_SR_PTR) {
		vgt_err("No valid context switch status buffer in switch interrupts!\n");
		return;
	}

	if (read_idx == write_idx) {
		vgt_dbg(VGT_DBG_EXECLIST, "No status buffer update.\n");
		return;
	}

	if (read_idx == DEFAULT_INV_SR_PTR) {
		/* The first read of the status buffer will be from buffer entry 0 */
		read_idx = -1;
	}

	if (read_idx > write_idx)
		write_idx += CTX_STATUS_BUF_NUM;

	ctx_status_reg = el_ring_mmio(ring_id, _EL_OFFSET_STATUS_BUF);
	while (read_idx < write_idx) {
		struct context_status_format ctx_status;
		uint32_t offset;
		read_idx ++;
		offset = ctx_status_reg + (read_idx % CTX_STATUS_BUF_NUM) * 8;
		READ_STATUS_MMIO(vgt->pdev, offset, ctx_status);

		if (!vgt_validate_status_entry(vgt, ring_id, &ctx_status))
			continue;

		trace_ctx_csb_emulate(vgt->vm_id,
				      ring_id,
				      read_idx % CTX_STATUS_BUF_NUM,
				      write_idx % CTX_STATUS_BUF_NUM,
				      ctx_status.udw, ctx_status.ldw);

		vgt_emulate_context_status_change(vgt, ring_id, &ctx_status);
		vgt_add_ctx_switch_status(vgt, ring_id, &ctx_status);
	}

	if (vgt_require_shadow_context(vgt)) {
		struct execlist_context *el_ctx;
		int i;
		hash_for_each(vgt->gtt.el_ctx_hash_table, i, el_ctx, node) {
			if (!el_ctx->sync_needed)
				continue;
			vgt_update_guest_ctx_from_shadow(vgt, ring_id, el_ctx);
			if (!el_ctx->ctx_running) {
				vgt_release_shadow_cmdbuf(vgt,
						&el_ctx->shadow_priv_bb);
				el_ctx->sync_needed = false;
			}
		}
	}

	read_idx = write_idx % CTX_STATUS_BUF_NUM;
	el_read_ptr(vgt->pdev, ring_id) = read_idx;
	ctx_ptr_val.status_buf_read_ptr = read_idx;
	ctx_ptr_val.mask = _CTXBUF_READ_PTR_MASK;
	VGT_MMIO_WRITE(vgt->pdev, ctx_ptr_reg, ctx_ptr_val.dw);

#ifdef EL_SLOW_DEBUG
	if (vgt_debug & VGT_DBG_EXECLIST) {
		vgt_dbg(VGT_DBG_EXECLIST, "Virtual CTX Status buffer after buffer "
					"update is below:\n");
		dump_ctx_status_buf(vgt, ring_id, false);
	}
#endif
}

void vgt_emulate_context_switch_event(struct pgt_device *pdev,
				enum vgt_ring_id ring_id)
{
	enum vgt_event_type event;
	int cache_wptr;
	struct vgt_irq_host_state *hstate = pdev->irq_hstate;
	struct vgt_device *vgt = current_render_owner(pdev);

	if (!vgt) {
		vgt_err("Receiving context switch interrupt while there is "
			"no render owner set in system!\n");
		BUG();
		return;
	}

	ASSERT(spin_is_locked(&pdev->lock));

	cache_wptr = el_write_ptr(pdev, ring_id);
	if ((cache_wptr == DEFAULT_INV_SR_PTR) ||
			(cache_wptr == el_read_ptr(pdev, ring_id)))
		return;

	/* we cannot rely on hstate->pending_events that is set in irq handler
	 * to tell us which ring is having context switch interrupts. The
	 * reason is that irq physical handler could happen in parallel with this
	 * function, and the bit can be cleared by the previous forwarding
	 * function. It's a race condition.
	 */
	vgt_emulate_csb_updates(vgt, ring_id);
	event = vgt_ring_id_to_ctx_event(ring_id);
	set_bit(event, hstate->pending_events);
	set_bit(VGT_REQUEST_IRQ, (void *)&pdev->request);
}

/* scheduling */
#if 0
static void vgt_emulate_el_preemption(struct vgt_device *vgt, enum vgt_ring_id ring_id)
{
	int el_slot_idx;
	int num;
	struct vgt_exec_list *el_slot;
	struct execlist_context *el_ctx;
	vgt_state_ring_t *ring_state;
	struct context_status_format ctx_status;
	enum vgt_event_type ctx_event;

	ring_state = &vgt->rb[ring_id];
	num = vgt_el_slots_number(ring_state);
	if (num <= 1)
		return;

	ASSERT(num == 2);
	el_slot_idx = vgt_el_slots_dequeue(vgt, ring_id);
	el_slot = &vgt_el_queue_slot(vgt, ring_id, el_slot_idx);
	ctx_event = vgt_ring_id_to_ctx_event(ring_id);

	/* we do not need to care the second context in the preempted
	 * execlist, even if it has.
	 */
	el_ctx = el_slot->el_ctxs[0];
	ASSERT(el_ctx);

	ctx_status.ldw = 0;
	ctx_status.context_id = el_ctx->guest_context.context_id;
	ctx_status.idle_to_active = 1;

	vgt_add_ctx_switch_status(vgt, ring_id, &ctx_status);
	vgt_trigger_virtual_event(vgt, ctx_event);

	/* TODO
	 * We could emulate lite restore here, but do not seem
	 * to be a must right now.
	 */
	ctx_status.ldw = 0;
	ctx_status.preempted = 1;

	vgt_add_ctx_switch_status(vgt, ring_id, &ctx_status);
	vgt_trigger_virtual_event(vgt, ctx_event);

	trace_ctx_lifecycle(vgt->vm_id, ring_id,
				el_ctx->guest_context.lrca,
				"emulated_preemption");

	el_slot->status = EL_EMPTY;
	el_slot->el_ctxs[0] = NULL;
	el_slot->el_ctxs[1] = NULL;
}
#endif
static inline void update_last_submit_el(struct vgt_device *vgt,
				int ring_id, struct ctx_desc_format *ctx0,
				struct ctx_desc_format *ctx1)
{
	vgt_state_ring_t *rb_state = &vgt->rb[ring_id];

	rb_state->el_last_submit[0] = *ctx0;
	rb_state->el_last_submit[1] = *ctx1;
}
static inline bool vgt_hw_ELSP_write(struct vgt_device *vgt,
				int ring_id,
				struct ctx_desc_format *ctx0,
				struct ctx_desc_format *ctx1)
{
	int rc = true;
	uint32_t reg = el_ring_mmio(ring_id, _EL_OFFSET_SUBMITPORT);

	ASSERT(ctx0 && ctx1);

	ppgtt_sync_oos_pages(vgt);

	vgt_dbg(VGT_DBG_EXECLIST, "EXECLIST is submitted into hardware! "
			"Writing 0x%x with: 0x%x; 0x%x; 0x%x; 0x%x\n",
			reg,
			ctx1->elm_high, ctx1->elm_low,
			ctx0->elm_high, ctx0->elm_low);

	vgt_force_wake_get();

	VGT_MMIO_WRITE(vgt->pdev, reg, ctx1->elm_high);
	VGT_MMIO_WRITE(vgt->pdev, reg, ctx1->elm_low);
	VGT_MMIO_WRITE(vgt->pdev, reg, ctx0->elm_high);
	VGT_MMIO_WRITE(vgt->pdev, reg, ctx0->elm_low);

	vgt_force_wake_put();
	update_last_submit_el(vgt, ring_id, ctx0, ctx1);

	return rc;
}

/* Below format is considered to be the buffer resubmission from preemption.
 * <head> - <tail> - <last_tail>
 */
#define IS_PREEMPTION_RESUBMISSION(head, tail, last_tail)	\
((((head) < (last_tail)) &&					\
	((tail) < (last_tail)) &&				\
	((tail) > (head))) ||					\
 (((head) > (last_tail)) &&					\
	!(((tail) >= (last_tail)) &&				\
	  ((tail) <= (head)))))

/* Shadow implementation of command buffers */
static int vgt_create_shadow_rb(struct vgt_device *vgt,
				 struct execlist_context *el_ctx)
{
	unsigned long shadow_hpa;
	unsigned long shadow_gma;
	uint32_t rb_size;
	unsigned long rb_gma;
	struct reg_state_ctx_header *reg_state;

	if (!shadow_cmd_buffer)
		return 0;

	ASSERT(el_ctx->shadow_rb.shadow_rb_base == 0);

	reg_state = vgt_get_reg_state_from_lrca(vgt,
				el_ctx->guest_context.lrca);

	rb_size = _RING_CTL_BUF_SIZE(reg_state->rb_ctrl.val);
	if ((rb_size >= 2 * SIZE_1MB) || (rb_size == 0)) {
		vgt_err("VM-%d: RB size <0x%x> is invalid. "
			"Shadow RB will not be created!\n",
			vgt->vm_id, rb_size);
		return -1;
	}

	rb_gma = reg_state->rb_start.val;
	shadow_hpa = rsvd_aperture_alloc(vgt->pdev, rb_size);
	if (shadow_hpa == 0) {
		vgt_err("VM-%d: Failed to allocate gm for shadow privilege bb!\n",
			vgt->vm_id);
		return -1;
	}

	shadow_gma = aperture_2_gm(vgt->pdev, shadow_hpa);
	el_ctx->shadow_rb.guest_rb_base = rb_gma;

	el_ctx->shadow_rb.shadow_rb_base = shadow_gma;
	el_ctx->shadow_rb.ring_size = rb_size;

	return 0;
}

static int vgt_create_shadow_indirect_ctx(struct vgt_device *vgt,
				 struct execlist_context *el_ctx)
{
	unsigned long shadow_hpa;
	unsigned long shadow_gma;
	uint32_t ctx_size;
	unsigned long ctx_gma;
	struct reg_state_ctx_header *reg_state;

	if (!shadow_indirect_ctx_bb)
		return 0;

	ASSERT(el_ctx->shadow_indirect_ctx.guest_ctx_base == 0);

	reg_state = vgt_get_reg_state_from_lrca(vgt,
				el_ctx->guest_context.lrca);
	ctx_size = reg_state->rcs_indirect_ctx.val & INDIRECT_CTX_SIZE_MASK;
	if (!ctx_size)
		return 0;

	if (!(reg_state->bb_per_ctx_ptr.val & 0x1)) {
		vgt_err("VM-%d: indirect ctx and per bb should work together\n", vgt->vm_id);
		return -1;
	}

	/*indirect ctx only valid for RCS*/
	if (el_ctx->ring_id) {
		vgt_err("VM-%d: indirect ctx disallowed enable on ring %d\n", vgt->vm_id,
			el_ctx->ring_id);
		return -1;
	}

	el_ctx->shadow_bb_per_ctx.guest_bb_base =
			reg_state->bb_per_ctx_ptr.val & BB_PER_CTX_ADDR_MASK;

	ctx_gma = reg_state->rcs_indirect_ctx.val & INDIRECT_CTX_ADDR_MASK;

	/* extra cache line size here for combining bb per ctx,
	 * take indirect ctx as ring and bb per ctx as it's privilege bb
	 */
	shadow_hpa = rsvd_aperture_alloc(vgt->pdev, (ctx_size + 1) * CACHELINE_BYTES);
	if (shadow_hpa == 0) {
		vgt_err("VM-%d: Failed to allocate gm for shadow indirect ctx!\n",
			vgt->vm_id);
		return -1;
	}

	shadow_gma = aperture_2_gm(vgt->pdev, shadow_hpa);
	el_ctx->shadow_indirect_ctx.guest_ctx_base = ctx_gma;

	el_ctx->shadow_indirect_ctx.shadow_ctx_base = shadow_gma;
	el_ctx->shadow_indirect_ctx.ctx_size = ctx_size * CACHELINE_BYTES;

	return 0;
}


static void vgt_destroy_shadow_rb(struct vgt_device *vgt,
				  struct execlist_context *el_ctx)
{
	unsigned long hpa;
	if (!shadow_cmd_buffer)
		return;

	if (el_ctx->shadow_rb.ring_size == 0)
		return;

	ASSERT(el_ctx->shadow_rb.shadow_rb_base);
	hpa = phys_aperture_base(vgt->pdev) +
			el_ctx->shadow_rb.shadow_rb_base;
	rsvd_aperture_free(vgt->pdev, hpa,
			   el_ctx->shadow_rb.ring_size);

	el_ctx->shadow_rb.guest_rb_base = 0;
	el_ctx->shadow_rb.shadow_rb_base = 0;
	el_ctx->shadow_rb.ring_size = 0;

	return;
}

static void vgt_destroy_shadow_indirect_ctx(struct vgt_device *vgt,
				  struct execlist_context *el_ctx)
{
	unsigned long hpa;

	if (!shadow_indirect_ctx_bb)
		return;

	if (el_ctx->shadow_indirect_ctx.ctx_size == 0)
		return;

	ASSERT(el_ctx->ring_id == 0);
	ASSERT(el_ctx->shadow_indirect_ctx.shadow_ctx_base);
	hpa = phys_aperture_base(vgt->pdev) +
			el_ctx->shadow_indirect_ctx.shadow_ctx_base;
	rsvd_aperture_free(vgt->pdev, hpa,
			   el_ctx->shadow_indirect_ctx.ctx_size + CACHELINE_BYTES);

	el_ctx->shadow_indirect_ctx.guest_ctx_base = 0;
	el_ctx->shadow_indirect_ctx.shadow_ctx_base = 0;
	el_ctx->shadow_indirect_ctx.ctx_size = 0;

	return;
}

static void vgt_release_shadow_cmdbuf(struct vgt_device *vgt,
				      struct shadow_batch_buffer *s_buf)
{
	/* unbind the shadow bb from GGTT */
	struct shadow_cmd_page *s_page, *next;

	if (!shadow_cmd_buffer)
		return;

	if (!s_buf || s_buf->n_pages == 0) {
		/* no privilege bb to release */
		return;
	}

	/* free the shadow pages */
	list_for_each_entry_safe(s_page, next, &s_buf->pages, list) {
		unsigned long shadow_hpa;
		list_del(&s_page->list);
		shadow_hpa = phys_aperture_base(vgt->pdev) + s_page->bound_gma;
		rsvd_aperture_free(vgt->pdev, shadow_hpa, PAGE_SIZE);
		s_page->bound_gma = 0;
		kfree(s_page);
	}

	s_buf->n_pages = 0;
	INIT_LIST_HEAD(&s_buf->pages);
}

/* perform command buffer scan and shadowing */
static int vgt_manipulate_cmd_buf(struct vgt_device *vgt,
			struct execlist_context *el_ctx)
{
	struct reg_state_ctx_header *guest_state;
	vgt_ringbuffer_t	*vring;
	enum vgt_ring_id ring_id = el_ctx->ring_id;

	if (vgt_require_shadow_context(vgt)) {
		guest_state = (struct reg_state_ctx_header *)
			el_ctx->ctx_pages[1].guest_page.vaddr;
	} else {
		ASSERT(vgt->vm_id == 0);
		guest_state = vgt_get_reg_state_from_lrca(vgt,
					el_ctx->guest_context.lrca);
	}

	vring = &vgt->rb[ring_id].vring;

	vring->tail = guest_state->ring_tail.val & RB_TAIL_OFF_MASK;
	vring->head = guest_state->ring_header.val & RB_HEAD_OFF_MASK;
	vring->start = guest_state->rb_start.val;
	vring->ctl = guest_state->rb_ctrl.val;
#if 0
	if (vgt->rb[ring_id].active_ppgtt_mm) {
		vgt_warn("vgt has ppgtt set for ring_id %d: 0x%llx\n",
				ring_id,
				(unsigned long long)vgt->rb[ring_id].active_ppgtt_mm);
	}
#endif
	/* TODO
	 * Will have better way to handle the per-rb value.
	 * Right now we just leverage the cmd_scan/schedule code for ring buffer mode
	 */
	vgt->rb[ring_id].active_ppgtt_mm = el_ctx->ppgtt_mm;
	vgt->rb[ring_id].has_ppgtt_mode_enabled = 1;
	vgt->rb[ring_id].has_ppgtt_base_set = 1;
	vgt->rb[ring_id].request_id = el_ctx->request_id;
	vgt->rb[ring_id].el_ctx = el_ctx;
#if 0
	/* keep this trace for debug purpose */
	trace_printk("VRING: HEAD %04x TAIL %04x START %08x last_scan %08x PREEMPTION %d DPY %d\n",
		vring->head, vring->tail, vring->start, el_ctx->last_scan_head,
		IS_PREEMPTION_RESUBMISSION(vring->head, vring->tail,
		el_ctx->last_scan_head), current_foreground_vm(vgt->pdev) == vgt);
#endif
	if (!el_ctx->scan_head_valid) {
		vgt->rb[ring_id].last_scan_head = vring->head;
		el_ctx->last_guest_head = vring->head;
		el_ctx->scan_head_valid = true;
	} else {
		vgt->rb[ring_id].last_scan_head = el_ctx->last_scan_head;
	}

	if ((vring->start != el_ctx->shadow_rb.shadow_rb_base) &&
	    (el_ctx->shadow_rb.guest_rb_base != vring->start)) {
		vgt_dbg(VGT_DBG_EXECLIST,
			"VM-%d: rb base is changed in workload submission "
			 "from 0x%lx to 0x%x\n",
			 vgt->vm_id,
			 el_ctx->shadow_rb.guest_rb_base,
			 vring->start);
		el_ctx->shadow_rb.guest_rb_base = vring->start;
	}

	if (el_ctx->shadow_rb.ring_size != _RING_CTL_BUF_SIZE(vring->ctl)) {
		vgt_destroy_shadow_rb(vgt, el_ctx);
		vgt_create_shadow_rb(vgt, el_ctx);
	}

	if (el_ctx->shadow_indirect_ctx.ctx_size !=
			(guest_state->rcs_indirect_ctx.val &
				INDIRECT_CTX_SIZE_MASK) *
					CACHELINE_BYTES) {
		vgt_destroy_shadow_indirect_ctx(vgt, el_ctx);
		vgt_create_shadow_indirect_ctx(vgt, el_ctx);
	}

	if (!vgt_scan_vring(vgt, ring_id)) {
		/* the function is used to update ring/buffer only. No real submission inside */
		vgt_submit_commands(vgt, ring_id);
		el_ctx->request_id = vgt->rb[ring_id].request_id;
		el_ctx->last_scan_head = vring->tail;
		vgt->rb[ring_id].active_ppgtt_mm = NULL;
		return 0;
	} else {
		return -1;
	}
}

void vgt_kick_off_execlists(struct vgt_device *vgt)
{
	int i, rc;
	struct pgt_device *pdev = vgt->pdev;

	for (i = 0; i < pdev->max_engines; i ++) {
		int j;
		int num = vgt_el_slots_number(&vgt->rb[i]);

		if (num > 1)
			vgt_dbg(VGT_DBG_EXECLIST,
				"VM(%d) Ring-%d: Preemption is met while "
				"kicking off execlists.\n", vgt->vm_id, i);
		for (j = 0; j < num; ++j) {
			rc = vgt_submit_execlist(vgt, i);
			if (rc < 0) {
				if (rc != -EBUSY)
					vgt_warn("Execlist submit fails, rc=%d. Total %d ELs"
						" in queue and already submitted %d.\n",
						-rc, num, j);
				break;
			}
		}
	}
}

bool vgt_idle_execlist(struct pgt_device *pdev, enum vgt_ring_id ring_id)
{
	uint32_t el_ring_base;
	uint32_t el_status_reg;
	struct execlist_status_format el_status;
	uint32_t ctx_ptr_reg;
	struct ctx_st_ptr_format ctx_st_ptr;
	struct ctx_st_ptr_format guest_ctx_st_ptr;
	struct context_status_format ctx_status;
	uint32_t ctx_status_reg = el_ring_mmio(ring_id, _EL_OFFSET_STATUS_BUF);
	unsigned long last_csb_reg_offset;
	struct vgt_device* vgt = current_render_owner(pdev);

	el_ring_base = vgt_ring_id_to_EL_base(ring_id);
	el_status_reg = el_ring_base + _EL_OFFSET_STATUS;
	el_status.ldw = VGT_MMIO_READ(pdev, el_status_reg);
	if (el_status.execlist_0_valid || el_status.execlist_1_valid) {
		return false;
	}

	ctx_ptr_reg = el_ring_mmio(ring_id, _EL_OFFSET_STATUS_PTR);
	ctx_st_ptr.dw = VGT_MMIO_READ(pdev, ctx_ptr_reg);

	if (ctx_st_ptr.status_buf_write_ptr == DEFAULT_INV_SR_PTR)
		return true;

	if (ctx_st_ptr.status_buf_read_ptr != ctx_st_ptr.status_buf_write_ptr)
		return false;

	last_csb_reg_offset = ctx_status_reg + ctx_st_ptr.status_buf_write_ptr * 8;
	READ_STATUS_MMIO(pdev, last_csb_reg_offset, ctx_status);

	if (!ctx_status.active_to_idle)
		return false;

	/* check Guest ctx status pointers, make sure guest already received last irq update */
	guest_ctx_st_ptr.dw = __vreg(vgt, ctx_ptr_reg);
	if (guest_ctx_st_ptr.status_buf_write_ptr != vgt->rb[ring_id].csb_write_ptr) {
		return false;
	}

	return true;
}

bool is_context_running(struct vgt_device *vgt, int ring_id,
	uint32_t context_id)
{
	uint32_t el_ring_base;
	uint32_t el_status_reg;
	struct execlist_status_format el_status;

	el_ring_base = vgt_ring_id_to_EL_base(ring_id);
	el_status_reg = el_ring_base + _EL_OFFSET_STATUS;
	READ_STATUS_MMIO(vgt->pdev, el_status_reg, el_status);

	if (context_id == el_status.context_id)
		return true;
	return false;
}
bool is_lite_restore_submission(struct vgt_device *vgt, int ring_id,
				uint32_t ctx0_id)
{
	vgt_state_ring_t *rb_state = &vgt->rb[ring_id];
	struct ctx_desc_format *last_submit = rb_state->el_last_submit;

	/* there are 2 types lite-restore:
	 * simple: A/0; A/X
	 *    A/0 is the last submmited EL and also the one HW engine is working on,
	 *    A/X is the EL which  will be submitted to HW.
	 * complex: X/A(A is the current working-on context); A/X
	 *    X/A is the last submmitted EL and the one HW engine is now working on,
	 *    A/X is the EL which will be submitted to HW.
	 */
	 if (last_submit[0].valid &&
		!last_submit[1].valid &&
		last_submit[0].context_id == ctx0_id){
		/* this is the simple type lite-restore */
		return true;
	}
	if (last_submit[1].valid &&
		last_submit[1].context_id == ctx0_id){
		if (is_context_running(vgt, ring_id, ctx0_id)) {
			/* this is the complex type lite-restore */
			return true;
		}
	}
	return false;
}
bool could_submit_el(struct vgt_device *vgt,
				int ring_id, struct vgt_exec_list *execlist)
{
	vgt_state_ring_t *rb_state = &vgt->rb[ring_id];
	struct ctx_desc_format *last_submit = rb_state->el_last_submit;

	if (!(preemption_policy & VGT_PREEMPTION_DISABLED))
		return true;

	if (!last_submit[0].valid &&
		!last_submit[1].valid)
		return true;

	if (!vgt_idle_execlist(vgt->pdev, ring_id)) {
		if ((preemption_policy & VGT_LITERESTORE_DISABLED) ||
			!is_lite_restore_submission(vgt, ring_id,
			execlist->el_ctxs[0]->guest_context.context_id))
			return false;
	}
	return true;
}

static inline bool handle_tlb_done(struct vgt_device *vgt, unsigned int offset)
{
	return (VGT_MMIO_READ(vgt->pdev, offset) == 0);
}
void handle_tlb_pending_event(struct vgt_device *vgt, enum vgt_ring_id ring_id)
{
	unsigned int offset;

	if (test_and_clear_bit(ring_id, (void *)vgt->tlb_handle_pending)) {
		switch (ring_id) {
			case RING_BUFFER_RCS:
				offset = 0x4260;
				break;
			case RING_BUFFER_BCS:
				offset = 0x426c;
				break;
			case RING_BUFFER_VCS:
				offset = 0x4264;
				break;
			case RING_BUFFER_VCS2:
				offset = 0x4268;
				break;
			case RING_BUFFER_VECS:
				offset = 0x4270;
				break;
			default:
				return;
		}
		vgt_force_wake_get();
		VGT_MMIO_WRITE(vgt->pdev, offset, 0x01);
		if (wait_for_atomic(handle_tlb_done(vgt, offset), 50) != 0)
			vgt_err("Timeout in handle ring (%d) tlb invalidate\n",
				ring_id);
		vgt_force_wake_put();
	}
}
int vgt_submit_execlist(struct vgt_device *vgt, enum vgt_ring_id ring_id)
{
	int i;
	struct ctx_desc_format context_descs[2];
	int el_slot_idx;
	vgt_state_ring_t *ring_state;
	struct vgt_exec_list *execlist = NULL;
	bool render_owner = is_current_render_owner(vgt);

	if (!render_owner)
		return 0;

	ring_state = &vgt->rb[ring_id];
	el_slot_idx = vgt_el_slots_next_sched(ring_state);
	if (el_slot_idx == -1)
		return 0;
	execlist = &vgt_el_queue_slot(vgt, ring_id, el_slot_idx);
	if (execlist == NULL)
		return -EINVAL;
	if (!could_submit_el(vgt, ring_id, execlist)) {
		/* no pending EL to submit */
		return -EBUSY;
	}

	for (i = 0; i < 2; ++ i) {
		struct execlist_context *ctx = execlist->el_ctxs[i];
		struct reg_state_ctx_header *shadow_state;
		char str[128];

		if (ctx == NULL) {
			if (i == 0) {
				vgt_err ("Wrong workload with ctx_0 NULL!\n");
				return -EINVAL;
			}
			memset(&context_descs[i], 0,
			       sizeof(struct ctx_desc_format));
			continue;
		}

		memcpy(&context_descs[i], &ctx->guest_context,
				sizeof(struct ctx_desc_format));

		if (!ctx->ppgtt_mm)
			vgt_el_create_shadow_ppgtt(vgt, ctx->ring_id, ctx);

		if (vgt->vm_id) {
			if (vgt_manipulate_cmd_buf(vgt, ctx) < 0)
				return -EINVAL;
		}

		vgt_update_shadow_ctx_from_guest(vgt, ctx);

		if (vgt_require_shadow_context(vgt))
			shadow_state = (struct reg_state_ctx_header *)
				ctx->ctx_pages[1].shadow_page.vaddr;
		else
			shadow_state = vgt_get_reg_state_from_lrca(vgt,
				ctx->shadow_lrca);
		snprintf(str, 128, "schedule_to_run "
			 "(shadow rb head: 0x%x; tail: 0x%x)",
			 shadow_state->ring_header.val,
			 shadow_state->ring_tail.val);
		trace_ctx_lifecycle(vgt->vm_id, ring_id,
			ctx->guest_context.lrca, str);

		if (!vgt_require_shadow_context(vgt))
			continue;

		if (shadow_execlist_context == PATCH_WITHOUT_SHADOW)
			vgt_patch_guest_context(ctx);
		else
			context_descs[i].lrca = ctx->shadow_lrca;
	}

	handle_tlb_pending_event(vgt, ring_id);

	/* mark it submitted even if it failed the validation */
	execlist->status = EL_SUBMITTED;

	if (vgt_validate_elsp_descs(vgt, &context_descs[0], &context_descs[1])) {
		execlist->el_ctxs[0]->ctx_running = true;
		vgt_hw_ELSP_write(vgt, ring_id, &context_descs[0],
					&context_descs[1]);
	}
	return 0;
}

bool vgt_batch_ELSP_write(struct vgt_device *vgt, int ring_id)
{
	struct vgt_elsp_store *elsp_store = &vgt->rb[ring_id].elsp_store;

	struct execlist_context *el_ctxs[2];
	struct ctx_desc_format *ctx_descs[2];
	int i;

	ASSERT(elsp_store->count == ELSP_BUNDLE_NUM);
	if (!vgt_validate_elsp_submission(vgt, elsp_store)) {
		vgt_err("VM(%d): Failed to submit an execution list!\n",
						vgt->vm_id);
		return false;
	}

	ctx_descs[0] = (struct ctx_desc_format *)&elsp_store->element[2];
	ctx_descs[1] = (struct ctx_desc_format *)&elsp_store->element[0];

	elsp_store->count = 0;

	if (hvm_render_owner) {
		if (!is_current_render_owner(vgt)) {
			vgt_warn("VM-%d: ELSP submission but VM is not "
			"render owner! But it will still be submitted.\n",
				vgt->vm_id);
		}
		vgt_hw_ELSP_write(vgt, ring_id, ctx_descs[0], ctx_descs[1]);
		return true;
	}

	for (i = 0; i < 2; ++ i) {
		struct execlist_context *el_ctx;
		if (!ctx_descs[i]->valid) {
			vgt_dbg(VGT_DBG_EXECLIST, "ctx%d in SUBMISSION is invalid.\n", i);
			el_ctxs[i] = NULL;
			continue;
		}

		if (!ctx_descs[i]->privilege_access) {
			vgt_err("VM-%d: Unexpected GGTT base rendering!\n", vgt->vm_id);
			return false;
		}

		vgt_dbg(VGT_DBG_EXECLIST, "SUBMISSION: ctx%d guest lrca is: 0x%x\n",
						i, ctx_descs[i]->lrca);
		el_ctx = execlist_context_find(vgt, ctx_descs[i]->lrca);

		if (el_ctx == NULL) {
			vgt_warn("Given EXECLIST context is not yet constructed! "
			"It is not expected to happen! lrca = 0x%x\n", ctx_descs[i]->lrca);
			el_ctx = vgt_create_execlist_context(vgt,
						ctx_descs[i], ring_id);
			if (el_ctx == NULL) {
				vgt_err("VM-%d: Failed to create execlist "
				"context on ring %d in ELSP submission!\n",
				vgt->vm_id, ring_id);
				return false;
			}
		}

		el_ctxs[i] = el_ctx;

		vgt_dbg(VGT_DBG_EXECLIST, "SUBMISSION: ctx shadow lrca is: 0x%x\n",
						el_ctx->shadow_lrca);
	}

	vgt_emulate_submit_execlist(vgt, ring_id, el_ctxs[0], el_ctxs[1]);

	if (!ctx_switch_requested(vgt->pdev) && is_current_render_owner(vgt)) {
		int rc = vgt_submit_execlist(vgt, ring_id);

		if (rc < 0 && rc != -EBUSY)
			return false;
	}

	return true;
}

/* init interface */

void execlist_ctx_table_destroy(struct vgt_device *vgt)
{
	struct hlist_node *n;
	struct execlist_context *el_ctx;
	int i;

	hash_for_each_safe(vgt->gtt.el_ctx_hash_table, i, n, el_ctx, node)
		vgt_destroy_execlist_context(vgt, el_ctx);

	return;
}

void vgt_clear_submitted_el_record(struct pgt_device *pdev, enum vgt_ring_id ring_id)
{
	int i;
	for (i = 0; i < VGT_MAX_VMS; i++) {
		struct vgt_device *vgt = pdev->device[i];
		int idx;
		if (!vgt)
			continue;

		for (idx = 0; idx < EL_QUEUE_SLOT_NUM; ++ idx) {
			struct vgt_exec_list *execlist;
			execlist = &vgt_el_queue_slot(vgt, ring_id, idx);
			if (execlist->status == EL_SUBMITTED)
				vgt_el_slots_delete(vgt, ring_id, idx);
		}
	}
}

/* pv interface */

bool vgt_g2v_execlist_context_create(struct vgt_device *vgt)
{
	bool rc = true;
	struct execlist_context *el_ctx;
	struct ctx_desc_format ctx_desc;

	ctx_desc.elm_high = __vreg(vgt, vgt_info_off(
				execlist_context_descriptor_hi));
	ctx_desc.elm_low = __vreg(vgt, vgt_info_off(
				execlist_context_descriptor_lo));

	vgt_dbg(VGT_DBG_EXECLIST, "VM-%d: Receive the el context creation "
		"request for lrca 0x%x\n", vgt->vm_id, ctx_desc.lrca);

	el_ctx = execlist_context_find(vgt, ctx_desc.lrca);
	if (el_ctx) {
		vgt_warn("VM-%d: A context creation request is received "
			" but the context is already constructed!\n"
			"The request will be ignored.\n", vgt->vm_id);
		dump_ctx_desc(vgt, &ctx_desc);
		return rc;
	}

	el_ctx = vgt_create_execlist_context(vgt, &ctx_desc, MAX_ENGINES);
	if (el_ctx == NULL) {
		/* The guest does not have ring state ready while
		 * sending context creation notification to us. Such
		 * notification will be ignored. And the context is
		 * not expected to be used in ELSP submission.
		 */
		vgt_warn("VM-%d: Failed to create context with lrca 0x%x! "
			"The request will be ignored.\n",
			vgt->vm_id, ctx_desc.lrca);
	} else if (hvm_render_owner) {
		if (vgt_require_shadow_context(vgt)) {
			vgt_patch_guest_context(el_ctx);
		}
	}

	return rc;
}

bool vgt_g2v_execlist_context_destroy(struct vgt_device *vgt)
{
	bool rc = true;
	struct execlist_context *el_ctx;
	struct ctx_desc_format ctx_desc;

	ctx_desc.elm_high = __vreg(vgt, vgt_info_off(
				execlist_context_descriptor_hi));
	ctx_desc.elm_low = __vreg(vgt, vgt_info_off(
				execlist_context_descriptor_lo));

	vgt_dbg(VGT_DBG_EXECLIST, "VM-%d: Receive the el context destroy "
		"request for lrca 0x%x\n", vgt->vm_id, ctx_desc.lrca);

	el_ctx = execlist_context_find(vgt, ctx_desc.lrca);
	if (el_ctx == NULL) {
		vgt_warn("VM-%d: A context destroy request is received "
			" but the context is not found!\n"
			"The request will be ignored.\n", vgt->vm_id);
		dump_ctx_desc(vgt, &ctx_desc);
		return rc;
	}

	vgt_destroy_execlist_context(vgt, el_ctx);
	return rc;
}

void vgt_reset_execlist(struct vgt_device *vgt, unsigned long ring_bitmap)
{
	vgt_state_ring_t *rb;
	int bit, i;
	uint32_t ctx_ptr_reg;
	struct ctx_st_ptr_format ctx_ptr_val;

	for_each_set_bit(bit, &ring_bitmap, sizeof(ring_bitmap)) {
		if (bit >= vgt->pdev->max_engines)
			break;

		rb = &vgt->rb[bit];

		memset(&rb->vring, 0, sizeof(vgt_ringbuffer_t));
		memset(&rb->sring, 0, sizeof(vgt_ringbuffer_t));

		vgt_disable_ring(vgt, bit);

		memset(&rb->elsp_store, 0, sizeof(rb->elsp_store));

		rb->el_slots_head = rb->el_slots_tail = 0;
		for (i = 0; i < EL_QUEUE_SLOT_NUM; ++ i)
			memset(&rb->execlist_slots[i], 0,
					sizeof(struct vgt_exec_list));

		ctx_ptr_reg = el_ring_mmio(bit, _EL_OFFSET_STATUS_PTR);
		ctx_ptr_val.dw = __vreg(vgt, ctx_ptr_reg);
		ctx_ptr_val.status_buf_write_ptr = DEFAULT_INV_SR_PTR;

		rb->csb_write_ptr = DEFAULT_INV_SR_PTR;

		__vreg(vgt, ctx_ptr_reg) = ctx_ptr_val.dw;
	}
}
