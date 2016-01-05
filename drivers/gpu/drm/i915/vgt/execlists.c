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

//#define EL_SLOW_DEBUG

#define EXECLIST_CTX_PAGES(ring_id)	((ring_id) == RING_BUFFER_RCS ? 20 : 2)

#define ROOT_POINTER_2_CTX_STATE(state, root, i)	\
do{							\
	state->pdp##i##_LDW.val = root[(i)<<1];		\
	state->pdp##i##_UDW.val = root[((i) << 1) + 1];	\
	vgt_dbg(VGT_DBG_EXECLIST, "New root[%d] in state is: 0x%x(high)-0x%x(low)\n",	\
		i, root[((i) << 1) + 1], root[(i) << 1]);	\
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
		uint32_t lrca;
		ctx = vgt_el_queue_ctx(vgt, ring_id, el_idx, i);
		if (!ctx)
			continue;

		lrca = ctx->guest_context.lrca;
		snprintf(str, 128, "slot[%d] ctx[%d] %s "
				"(queue head: %d; tail: %d)",
			el_idx, i,
			(ops == 0 ? "enqueue" : (ops == 1 ? "dequeue" : "delete")),
			head, tail);
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

static void vgt_el_slots_find_submitted_ctx(bool forward_search, vgt_state_ring_t *ring_state,
			uint32_t ctx_id, int *el_slot_idx, int *el_slot_ctx_idx)
{
	int head = ring_state->el_slots_head;
	int tail = ring_state->el_slots_tail;

	*el_slot_idx = -1;
	*el_slot_ctx_idx = -1;

	while ((head != tail) && (*el_slot_idx == -1)) {
		int i;
		struct vgt_exec_list *el_slot;

		if (forward_search) {
			el_slot = &ring_state->execlist_slots[head];
		} else {
			if (tail == 0)
				tail = EL_QUEUE_SLOT_NUM;
			tail --;
			el_slot = &ring_state->execlist_slots[tail];
		}

		if (el_slot->status != EL_SUBMITTED)
			continue;

		for (i = 0; i < 2; ++ i) {
			struct execlist_context *p = el_slot->el_ctxs[i];
			if (p && p->guest_context.context_id == ctx_id) {
				*el_slot_idx = forward_search ? head : tail;
				*el_slot_ctx_idx = i;
				break;
			}
		}

		if (forward_search) {
			head ++;
			if (head == EL_QUEUE_SLOT_NUM)
				head = 0;
		}
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
			head ++;
			if (head == tail) {
				head = -1;
				break;
			} else if (head == EL_QUEUE_SLOT_NUM) {
				head = 0;
			}
		}
		return head;
	}
}

static int vgt_el_slots_number(vgt_state_ring_t *ring_state)
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
	enum vgt_ring_id ring_id;

	if (!wp_submitted_ctx &&
		(shadow_execlist_context != NORMAL_CTX_SHADOW)) {
		/* If option is set not to protect submitted_ctx, write
		 * protection will be disabled, except that the shadow policy
		 * is NORMAL_CTX_SHADOW. In normal shadowing case, the write
		 * protection is from context creation to the context destroy.
		 * It is needed for guest-shadow data sync-up, and cannot be
		 * disabled.
		 */
		return;
	}

	ring_id = el_ctx->ring_id;
	hypervisor_set_wp_pages(vgt,
				&el_ctx->ctx_pages[idx].guest_page);
	trace_ctx_protection(vgt->vm_id, ring_id, el_ctx->guest_context.lrca,
				idx, el_ctx->ctx_pages[idx].guest_page.gfn,
				"set_writeprotection");
}

static inline void vgt_clear_wp_guest_ctx(struct vgt_device *vgt,
			struct execlist_context *el_ctx, int idx)
{
	enum vgt_ring_id ring_id;

	if (!wp_submitted_ctx &&
		(shadow_execlist_context != NORMAL_CTX_SHADOW)) {
		return;
	}

	ring_id = el_ctx->ring_id;
	hypervisor_unset_wp_pages(vgt,
				&el_ctx->ctx_pages[idx].guest_page);
	trace_ctx_protection(vgt->vm_id, ring_id, el_ctx->guest_context.lrca,
				idx, el_ctx->ctx_pages[idx].guest_page.gfn,
				"clear_writeprotection");
}

static bool sctx_mirror_state_wp_handler(void *gp, uint64_t pa, void *p_data, int bytes)
{
	guest_page_t *guest_page = (guest_page_t *)gp;
	struct shadow_ctx_page *ctx_page = container_of(guest_page,
					struct shadow_ctx_page, guest_page);
	uint32_t offset = pa & (PAGE_SIZE - 1);

	trace_ctx_write_trap(pa, bytes);
	if (!guest_page->writeprotection) {
		vgt_err("EXECLIST Ctx mirror wp handler is called without write protection! "
			"addr <0x%llx>, bytes %i\n", pa, bytes);
		return false;
	}

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

	trace_ctx_write_trap(pa, bytes);
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

/* context shadow: context sync-up between guest/shadow */

static inline bool ppgtt_update_shadow_ppgtt_for_ctx(struct vgt_device *vgt,
				struct execlist_context *el_ctx)
{
	bool rc = true;
	int i;

	if (!vgt_require_shadow_context(vgt))
		return rc;

	for (i = 0; i < el_ctx->ppgtt_mm->page_table_entry_cnt; ++ i) {
		vgt_dbg(VGT_DBG_EXECLIST, "Emulate the rootp[%d] change\n", i);
		rc = vgt_handle_guest_write_rootp_in_context(el_ctx, i);
		if (!rc)
			break;
	}
	return rc;
}

/* not to copy PDP root pointers */
static void memcpy_reg_state_page(void *dest_page, void *src_page)
{
	uint32_t pdp_backup[8];
	struct reg_state_ctx_header *dest_ctx;
	struct reg_state_ctx_header *src_ctx;
	dest_ctx = (struct reg_state_ctx_header *)(dest_page);
	src_ctx = (struct reg_state_ctx_header *)(src_page);

	pdp_backup[0] = dest_ctx->pdp0_LDW.val;
	pdp_backup[1] = dest_ctx->pdp0_UDW.val;
	pdp_backup[2] = dest_ctx->pdp1_LDW.val;
	pdp_backup[3] = dest_ctx->pdp1_UDW.val;
	pdp_backup[4] = dest_ctx->pdp2_LDW.val;
	pdp_backup[5] = dest_ctx->pdp2_UDW.val;
	pdp_backup[6] = dest_ctx->pdp3_LDW.val;
	pdp_backup[7] = dest_ctx->pdp3_UDW.val;

	memcpy(dest_page, src_page, SIZE_PAGE);

	dest_ctx->pdp0_LDW.val = pdp_backup[0];
	dest_ctx->pdp0_UDW.val = pdp_backup[1];
	dest_ctx->pdp1_LDW.val = pdp_backup[2];
	dest_ctx->pdp1_UDW.val = pdp_backup[3];
	dest_ctx->pdp2_LDW.val = pdp_backup[4];
	dest_ctx->pdp2_UDW.val = pdp_backup[5];
	dest_ctx->pdp3_LDW.val = pdp_backup[6];
	dest_ctx->pdp3_UDW.val = pdp_backup[7];
}

static void vgt_update_shadow_ctx_from_guest(struct vgt_device *vgt,
			struct execlist_context *el_ctx)
{
	if (!vgt_require_shadow_context(vgt))
		return;
}

static void vgt_update_guest_ctx_from_shadow(struct vgt_device *vgt,
			enum vgt_ring_id ring_id,
			struct execlist_context *el_ctx)
{
	int ctx_pages = EXECLIST_CTX_PAGES(ring_id);

	if (shadow_execlist_context == PATCH_WITHOUT_SHADOW) {
#if 0
	/* For some unkonw reason, switch back to guest PDP will cause
	 * strange ring hangup after > ~20hours 3D testing.
	 * It is not necessary to swith back to guest PDP, since Guest
	 * will not touch it anymore after submission*/

		struct reg_state_ctx_header *reg_state;
		uint32_t *g_rootp;
		g_rootp = (uint32_t *)el_ctx->ppgtt_mm->virtual_page_table;
		reg_state = (struct reg_state_ctx_header *)
			el_ctx->ctx_pages[1].guest_page.vaddr;
		ROOT_POINTER_2_CTX_STATE(reg_state, g_rootp, 0);
		ROOT_POINTER_2_CTX_STATE(reg_state, g_rootp, 1);
		ROOT_POINTER_2_CTX_STATE(reg_state, g_rootp, 2);
		ROOT_POINTER_2_CTX_STATE(reg_state, g_rootp, 3);
#endif
	} else {
		int i;
		for (i = 0; i < ctx_pages; ++ i) {
			void *dst = el_ctx->ctx_pages[i].guest_page.vaddr;
			void *src = el_ctx->ctx_pages[i].shadow_page.vaddr;

			ASSERT(dst && src);
			if (i == 1)
				memcpy_reg_state_page(dst, src);
			else
				memcpy(dst, src, SIZE_PAGE);
		}
	}
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

static void vgt_el_create_shadow_context(struct vgt_device *vgt,
				enum vgt_ring_id ring_id,
				struct execlist_context *el_ctx)
{
	struct vgt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	uint32_t gfn;
	uint32_t shadow_context_gma;
	uint32_t guest_context_gma;
	uint32_t sl, gl;
	uint32_t rsvd_pages_idx;
	uint32_t rsvd_aperture_gm;
	int i;
	int ctx_pages = EXECLIST_CTX_PAGES(ring_id);

	guest_context_gma = el_ctx->guest_context.lrca << GTT_PAGE_SHIFT;

	shadow_context_gma = aperture_2_gm(vgt->pdev,
				rsvd_aperture_alloc(vgt->pdev,
					(EXECLIST_CTX_PAGES(ring_id) << GTT_PAGE_SHIFT)));

	ASSERT((shadow_context_gma & 0xfff) == 0);
	el_ctx->shadow_lrca = shadow_context_gma >> GTT_PAGE_SHIFT;

	rsvd_aperture_gm = aperture_2_gm(vgt->pdev, vgt->pdev->rsvd_aperture_base);
	rsvd_pages_idx = el_ctx->shadow_lrca - (rsvd_aperture_gm >> GTT_PAGE_SHIFT);

	vgt_dbg(VGT_DBG_EXECLIST, "Allocating aperture for shadow context "
			"with idx: 0x%x and addr: 0x%x\n",
			rsvd_pages_idx, shadow_context_gma);

	/* per page copy from guest context to shadow context since its virtual
	 * address may not be sequential.
	 */
	for (i = 0, sl = shadow_context_gma, gl = guest_context_gma; i < ctx_pages;
			++ i, ++ rsvd_pages_idx, sl += SIZE_PAGE, gl += SIZE_PAGE) {
		gtt_entry_t e;
		e.pdev = vgt->pdev;
		e.type = GTT_TYPE_GGTT_PTE;

		ggtt_get_guest_entry(vgt->gtt.ggtt_mm, &e, gl >> GTT_PAGE_SHIFT);

		gfn = ops->get_pfn(&e);
		vgt_dbg(VGT_DBG_EXECLIST,
			"pfn for context page %i (gma: 0x%x)is: 0x%x\n", i, gl, gfn);
		if (i == 1) {
			vgt_init_guest_page(vgt, &el_ctx->ctx_pages[i].guest_page,
				gfn, sctx_reg_state_wp_handler, &el_ctx);
		} else {
			vgt_init_guest_page(vgt, &el_ctx->ctx_pages[i].guest_page,
				gfn, sctx_mirror_state_wp_handler, &el_ctx);
		}

		/* backup the shadow context gtt entry */
		el_ctx->shadow_entry_backup[i].pdev = vgt->pdev;
		el_ctx->shadow_entry_backup[i].type = GTT_TYPE_GGTT_PTE;
		ops->get_entry(NULL, &el_ctx->shadow_entry_backup[i],
						sl >> GTT_PAGE_SHIFT, false, NULL);

		{
			el_ctx->ctx_pages[i].shadow_page.vaddr =
				phys_aperture_vbase(vgt->pdev) + sl;
			el_ctx->ctx_pages[i].shadow_page.page =
				(*vgt->pdev->rsvd_aperture_pages)[rsvd_pages_idx];
			ASSERT(el_ctx->ctx_pages[i].shadow_page.vaddr &&
				el_ctx->ctx_pages[i].guest_page.vaddr);

			vgt_dbg(VGT_DBG_EXECLIST, "memory copy for context page %d: dst addr: 0x%llx; "
					"src addr: 0x%llx\n",
				i, (u64)el_ctx->ctx_pages[i].shadow_page.vaddr,
				(u64)el_ctx->ctx_pages[i].guest_page.vaddr);

			if (shadow_execlist_context == NORMAL_CTX_SHADOW) {
				memcpy(el_ctx->ctx_pages[i].shadow_page.vaddr,
				el_ctx->ctx_pages[i].guest_page.vaddr, SIZE_PAGE);
				vgt_set_wp_guest_ctx(vgt, el_ctx, i);
			}
		}
		el_ctx->ctx_pages[i].vgt = vgt;
	}
}

static bool vgt_el_create_shadow_ppgtt(struct vgt_device *vgt,
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
		return false;
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
	return true;
}

static struct execlist_context *vgt_create_execlist_context(struct vgt_device *vgt,
				struct ctx_desc_format *ctx, enum vgt_ring_id ring_id)
{
	struct execlist_context *el_ctx;

	vgt_dbg(VGT_DBG_EXECLIST, "creating new execlist context with desc below:\n");
	if (vgt_debug & VGT_DBG_EXECLIST)
		dump_ctx_desc(vgt, ctx);

	ASSERT (execlist_context_find(vgt, ctx->lrca) == NULL);

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

	if (vgt_require_shadow_context(vgt))
		vgt_el_create_shadow_context(vgt, ring_id, el_ctx);

	vgt_el_create_shadow_ppgtt(vgt, ring_id, el_ctx);

	trace_ctx_lifecycle(vgt->vm_id, ring_id,
			el_ctx->guest_context.lrca, "create");
	return el_ctx;
}

static void vgt_destroy_execlist_context(struct vgt_device *vgt,
				struct execlist_context *el_ctx)
{
	int ctx_pages;
	enum vgt_ring_id ring_id;
	int i;

	if (el_ctx == NULL)
		return;

	ring_id = el_ctx->ring_id;
	if (ring_id == MAX_ENGINES) {
		vgt_err("Invalid execlist context!\n");
		ASSERT_VM(0, vgt);
	}

	trace_ctx_lifecycle(vgt->vm_id, ring_id,
			el_ctx->guest_context.lrca, "destroy");

	ctx_pages = EXECLIST_CTX_PAGES(ring_id);

	for (i = 0; i < ctx_pages; ++ i) {
		// remove the write protection;
		if (shadow_execlist_context == NORMAL_CTX_SHADOW) {
			hypervisor_unset_wp_pages(vgt,
				&el_ctx->ctx_pages[i].guest_page);
		}
		vgt_clean_guest_page(vgt, &el_ctx->ctx_pages[i].guest_page);
	}

	// free the shadow context;
	if (vgt_require_shadow_context(vgt)) {
		unsigned long start;
		unsigned int shadow_lrca = el_ctx->shadow_lrca;

		ASSERT(hvm_render_owner || shadow_lrca);
		if (!hvm_render_owner) {
			start = phys_aperture_base(vgt->pdev) +
					(shadow_lrca << GTT_PAGE_SHIFT);
			rsvd_aperture_free(vgt->pdev, start,
					ctx_pages << GTT_PAGE_SHIFT);
		}
	}

	hash_del(&el_ctx->node);
	kfree(el_ctx);
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
	bool forward_search = true;
	vgt_state_ring_t *ring_state;
	uint32_t el_slot_ctx_idx = -1;
	uint32_t el_slot_idx = -1;
	struct vgt_exec_list *el_slot = NULL;
	struct execlist_context *el_ctx = NULL;
	uint32_t ctx_id = ctx_status->context_id;
	bool lite_restore;

	ring_state = &vgt->rb[ring_id];
	if (vgt_el_slots_number(ring_state) > 1) {
		if (!ctx_status->preempted) {
			/* TODO we may give warning here.
			 * It is not expected but still work.
			 */
			forward_search = false;
		}
	}

	vgt_el_slots_find_submitted_ctx(forward_search, ring_state, ctx_id,
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
	} else {
		goto emulation_done;
	}

	if (!vgt_require_shadow_context(vgt))
		goto emulation_done;

	if (vgt_debug & VGT_DBG_EXECLIST)
		dump_el_context_information(vgt, el_ctx);

	if (ctx_status->context_complete)
		vgt_update_guest_ctx_from_shadow(vgt, ring_id, el_ctx);

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

		vgt_emulate_context_status_change(vgt, ring_id, &ctx_status);
		vgt_add_ctx_switch_status(vgt, ring_id, &ctx_status);
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
static inline bool vgt_hw_ELSP_write(struct vgt_device *vgt,
				unsigned int reg,
				struct ctx_desc_format *ctx0,
				struct ctx_desc_format *ctx1)
{
	int rc = true;

	ASSERT(ctx0 && ctx1);

	ppgtt_check_partial_access(vgt);
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

static void vgt_update_ring_info(struct vgt_device *vgt,
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

	vgt_scan_vring(vgt, ring_id);

	/* the function is used to update ring/buffer only. No real submission inside */
	vgt_submit_commands(vgt, ring_id);

	el_ctx->request_id = vgt->rb[ring_id].request_id;
	el_ctx->last_scan_head = vring->tail;
	vgt->rb[ring_id].active_ppgtt_mm = NULL;
}

void vgt_kick_off_execlists(struct vgt_device *vgt)
{
	int i;
	struct pgt_device *pdev = vgt->pdev;

	for (i = 0; i < pdev->max_engines; i ++) {
		int j;
		int num = vgt_el_slots_number(&vgt->rb[i]);
		if (num == 2)
			vgt_dbg(VGT_DBG_EXECLIST,
				"VM(%d) Ring-%d: Preemption is met while "
				"kicking off execlists.\n", vgt->vm_id, i);
		for (j = 0; j < num; ++ j)
			vgt_submit_execlist(vgt, i);
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
		//vgt_info("EXECLIST still have valid items in context switch!\n");
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

void vgt_submit_execlist(struct vgt_device *vgt, enum vgt_ring_id ring_id)
{
	int i, j = 0;
	struct ctx_desc_format context_descs[2];
	uint32_t elsp_reg;
	int el_slot_idx;
	vgt_state_ring_t *ring_state;
	struct vgt_exec_list *execlist = NULL;
	bool render_owner = is_current_render_owner(vgt);

	if (!render_owner)
		return;

	ring_state = &vgt->rb[ring_id];
	el_slot_idx = vgt_el_slots_next_sched(ring_state);
	if (el_slot_idx == -1) {
		return;
	}
	execlist = &vgt_el_queue_slot(vgt, ring_id, el_slot_idx);

	if (execlist == NULL) {
		/* no pending EL to submit */
		return;
	}

	ASSERT (execlist->el_ctxs[0] != NULL);

	memset(context_descs, 0, sizeof(context_descs));

	for (i = 0; i < 2; ++ i) {
		struct execlist_context *ctx = execlist->el_ctxs[i];

		if (ctx == NULL)
			continue;

		memcpy(&context_descs[j++], &ctx->guest_context,
				sizeof(struct ctx_desc_format));

		ASSERT_VM(ring_id == ctx->ring_id, vgt);
		vgt_update_shadow_ctx_from_guest(vgt, ctx);
		vgt_update_ring_info(vgt, ctx);

		trace_ctx_lifecycle(vgt->vm_id, ring_id,
			ctx->guest_context.lrca, "schedule_to_run");

		if (!vgt_require_shadow_context(vgt))
			continue;

		if (shadow_execlist_context == PATCH_WITHOUT_SHADOW)
			vgt_patch_guest_context(ctx);
		else
			context_descs[i].lrca = ctx->shadow_lrca;

#ifdef EL_SLOW_DEBUG
		dump_el_context_information(vgt, ctx);
#endif
	}

	if (context_descs[0].elm_low == context_descs[1].elm_low &&
		context_descs[0].elm_high == context_descs[1].elm_high)
		memset(&context_descs[1], 0, sizeof(context_descs[1]));

	elsp_reg = el_ring_mmio(ring_id, _EL_OFFSET_SUBMITPORT);
	/* mark it submitted even if it failed the validation */
	execlist->status = EL_SUBMITTED;

	if (vgt_validate_elsp_descs(vgt, &context_descs[0], &context_descs[1]) && j) {
#ifdef EL_SLOW_DEBUG
		struct execlist_status_format status;
		uint32_t status_reg = el_ring_mmio(ring_id, _EL_OFFSET_STATUS);
		READ_STATUS_MMIO(vgt->pdev, status_reg, status);
		vgt_dbg(VGT_DBG_EXECLIST, "The EL status before ELSP submission!\n");
		dump_execlist_status((struct execlist_status_format *)&status,
					ring_id);
#endif
		vgt_hw_ELSP_write(vgt, elsp_reg, &context_descs[0],
					&context_descs[1]);
#ifdef EL_SLOW_DEBUG
		READ_STATUS_MMIO(vgt->pdev, status_reg, status);
		vgt_dbg(VGT_DBG_EXECLIST, "The EL status after ELSP submission:\n");
		dump_execlist_status((struct execlist_status_format *)&status,
					ring_id);
#endif
	}
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
		uint32_t elsp_reg;
		elsp_reg = el_ring_mmio(ring_id, _EL_OFFSET_SUBMITPORT);
		if (!is_current_render_owner(vgt)) {
			vgt_warn("VM-%d: ELSP submission but VM is not "
			"render owner! But it will still be submitted.\n",
				vgt->vm_id);
		}
		vgt_hw_ELSP_write(vgt, elsp_reg, ctx_descs[0], ctx_descs[1]);
		return true;
	}

	for (i = 0; i < 2; ++ i) {
		struct execlist_context *el_ctx;
		if (!ctx_descs[i]->valid) {
			vgt_dbg(VGT_DBG_EXECLIST, "ctx%d in SUBMISSION is invalid.\n", i);
			el_ctxs[i] = NULL;
			continue;
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

	if (!ctx_switch_requested(vgt->pdev) && is_current_render_owner(vgt))
		vgt_submit_execlist(vgt, ring_id);

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
