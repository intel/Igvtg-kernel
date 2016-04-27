/*
 * vGT command parser
 *
 * Copyright(c) 2011-2013 Intel Corporation. All rights reserved.
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

#include <linux/slab.h>
#include "vgt.h"
#include "trace.h"
#include "fb_decoder.h"

/* vgt uses below bits in NOOP_ID:
 *	    bit 21 - 16 is command type.
 *	    bit 15 - 0  holds command specific information.
 *
 * Assumption: Linux/Windows guest will not use bits 21 - bits 16 with
 * non-zero value.
 */
#define VGT_NOOP_ID_CMD_SHIFT	16
#define VGT_NOOP_ID_CMD_MASK	(0x3f << VGT_NOOP_ID_CMD_SHIFT)
#define CMD_LENGTH_MASK		0xff
#define gmadr_dw_number(s)	\
	(s->vgt->pdev->device_info.gmadr_bytes_in_cmd >> 2)
/*
 * new cmd parser
 */

DEFINE_HASHTABLE(vgt_cmd_table, VGT_CMD_HASH_BITS);

static inline int cmd_address_audit(struct parser_exec_state *s, unsigned long g_addr,
	int op_size, bool index_mode);

static void vgt_add_cmd_entry(struct vgt_cmd_entry *e)
{
	hash_add(vgt_cmd_table, &e->hlist, e->info->opcode);
}

static inline struct cmd_info* vgt_find_cmd_entry(unsigned int opcode, int ring_id)
{
	struct vgt_cmd_entry *e;

	hash_for_each_possible(vgt_cmd_table, e, hlist, opcode) {
		if ((opcode == e->info->opcode) && (e->info->rings & (1<<ring_id)))
			return e->info;
	}
	return NULL;
}

static struct cmd_info* vgt_find_cmd_entry_any_ring(unsigned int opcode, int rings)
{
	struct cmd_info* info = NULL;
	unsigned int ring;
	for_each_set_bit(ring, (unsigned long*)&rings, MAX_ENGINES) {
		info = vgt_find_cmd_entry(opcode, ring);
		if (info)
			break;
	}
	return info;
}

void vgt_clear_cmd_table(void)
{
	int i;
	struct hlist_node *tmp;
	struct vgt_cmd_entry *e;

	hash_for_each_safe(vgt_cmd_table, i, tmp, e, hlist)
		kfree(e);

	hash_init(vgt_cmd_table);
}

void vgt_init_cmd_info(vgt_state_ring_t *rs)
{
	memset(&rs->patch_list, 0, sizeof(struct cmd_general_info));
	rs->patch_list.head = 0;
	rs->patch_list.tail = 0;
	rs->patch_list.count = CMD_PATCH_NUM;
	memset(&rs->handler_list, 0, sizeof(struct cmd_general_info));
	rs->handler_list.head = 0;
	rs->handler_list.tail = 0;
	rs->handler_list.count = CMD_HANDLER_NUM;
	memset(&rs->tail_list, 0, sizeof(struct cmd_general_info));
	rs->tail_list.head = 0;
	rs->tail_list.tail = 0;
	rs->tail_list.count = CMD_TAIL_NUM;
}

static int get_next_entry(struct cmd_general_info *list)
{
	int next;

	next = list->tail + 1;
	if (next == list->count)
		next = 0;

	if (next == list->head)
		next = list->count;

	return next;
}

/* TODO: support incremental patching */
static inline int add_patch_entry(struct parser_exec_state *s,
	void *addr, uint32_t val)
{
	vgt_state_ring_t *rs = &s->vgt->rb[s->ring_id];
	struct cmd_general_info *list = &rs->patch_list;
	struct cmd_patch_info *patch;
	int next;

	ASSERT(s->shadow != INDIRECT_CTX_SHADOW);

	if (addr == NULL) {
		vgt_err("VM(%d) CMD_SCAN: NULL address to be patched\n",
				s->vgt->vgt_id);
		return -EINVAL;
	}

	next = get_next_entry(list);
	if (next == list->count) {
		vgt_err("CMD_SCAN: no free patch entry\n");
		return -ENOSPC;
	}

	vgt_dbg(VGT_DBG_CMD, "VM(%d): Add patch entry-%d (addr: %llx, val: %x, id: %lld\n",
		s->vgt->vm_id, next, (uint64_t)addr, val, s->request_id);
	patch = &list->patch[next];
	patch->addr = addr;
	patch->new_val = val;

	patch->request_id = s->request_id;

	list->tail = next;
	return 0;
}

static inline int add_post_handle_entry(struct parser_exec_state *s,
	parser_cmd_handler handler)
{
	vgt_state_ring_t* rs = &s->vgt->rb[s->ring_id];
	struct cmd_general_info *list = &rs->handler_list;
	struct cmd_handler_info *entry;
	int next;

	ASSERT(s->shadow != INDIRECT_CTX_SHADOW);

	next = get_next_entry(list);
	if (next == list->count) {
		vgt_err("CMD_SCAN: no free post-handle entry\n");
		return -ENOSPC;
	}

	entry = &list->handler[next];
	/* two pages mapping are always valid */
	memcpy(&entry->exec_state, s, sizeof(struct parser_exec_state));
	/*
	 * Do not use ip buf in post handle entry,
	 * as ip buf has been freed at that time.
	 * Switch back to guest memory write/read method
	 */
	entry->exec_state.ip_buf = entry->exec_state.ip_buf_va = NULL;
	entry->handler = handler;
	entry->request_id = s->request_id;

	list->tail = next;
	return 0;

}

static int add_tail_entry(struct parser_exec_state *s,
	uint32_t tail, uint32_t cmd_nr, uint32_t flags, uint32_t ip_offset)
{
	vgt_state_ring_t* rs = &s->vgt->rb[s->ring_id];
	struct cmd_general_info *list = &rs->tail_list;
	struct cmd_tail_info *entry;
	int next;

	ASSERT(s->shadow != INDIRECT_CTX_SHADOW);

	next = get_next_entry(list);
	if (next == list->count) {
		vgt_err("CMD_SCAN: no free tail entry\n");
		return -ENOSPC;
	}

	entry = &list->cmd[next];
	entry->request_id = s->request_id;
	entry->tail = tail;
	entry->cmd_nr = cmd_nr;
	entry->flags = flags;
	entry->ip_offset = ip_offset;

	list->tail = next;
	return 0;

}

static void apply_patch_entry(struct vgt_device *vgt, struct cmd_patch_info *patch)
{
	ASSERT(patch->addr);

	if (shadow_cmd_buffer)
		*((uint32_t *)patch->addr) = patch->new_val;
	else
		hypervisor_write_va(vgt, patch->addr, &patch->new_val,
				sizeof(patch->new_val), 1);
}

/*
 * Apply all patch entries with request ID before or
 * equal to the submission ID
 */
static void apply_patch_list(struct vgt_device *vgt, vgt_state_ring_t *rs,
		uint64_t submission_id)
{
	int next;
	struct cmd_general_info *list = &rs->patch_list;
	struct cmd_patch_info *patch;

	next = list->head;
	while (next != list->tail) {
		next++;
		if (next == list->count)
			next = 0;
		patch = &list->patch[next];
		/* TODO: handle id wrap */
		if (patch->request_id > submission_id)
			break;

		vgt_dbg(VGT_DBG_CMD, "submission-%lld: apply patch entry-%d (addr: %llx, val: %x->%x, id: %lld\n",
			submission_id, next, (uint64_t)patch->addr,
			patch->old_val, patch->new_val, patch->request_id);
		apply_patch_entry(vgt, patch);
		list->head = next;
	}
}

/*
 * Invoke all post-handle entries with request ID before or
 * equal to the submission ID
 */
static void apply_post_handle_list(vgt_state_ring_t *rs, uint64_t submission_id)
{
	int next;
	struct cmd_general_info *list = &rs->handler_list;
	struct cmd_handler_info *entry;

	next = list->head;
	while (next != list->tail) {
		next++;
		if (next == list->count)
			next = 0;
		entry = &list->handler[next];
		/* TODO: handle id wrap */
		if (entry->request_id > submission_id)
			break;

		entry->handler(&entry->exec_state);
		list->head = next;
	}
}

/* submit tails according to submission id */
void apply_tail_list(struct vgt_device *vgt, int ring_id,
	uint64_t submission_id)
{
	int next;
	struct pgt_device *pdev = vgt->pdev;
	vgt_state_ring_t *rs = &vgt->rb[ring_id];
	struct cmd_general_info *list = &rs->tail_list;
	struct cmd_tail_info *entry;

	next = list->head;
	while (next != list->tail) {
		next++;
		if (next == list->count)
			next = 0;
		entry = &list->cmd[next];
		/* TODO: handle id wrap */
		if (entry->request_id > submission_id)
			break;

		apply_post_handle_list(rs, entry->request_id);
		apply_patch_list(vgt, rs, entry->request_id);

		if (!pdev->enable_execlist) {
			if ((rs->uhptr & _REGBIT_UHPTR_VALID) &&
					(rs->uhptr_id < entry->request_id)) {
				rs->uhptr &= ~_REGBIT_UHPTR_VALID;
				VGT_MMIO_WRITE(pdev, VGT_UHPTR(ring_id), rs->uhptr);
			}
			ppgtt_sync_oos_pages(vgt);
			VGT_WRITE_TAIL(pdev, ring_id, entry->tail);
		}
		list->head = next;
	}
}

/* find allowable tail info based on cmd budget */
int get_submission_id(vgt_state_ring_t *rs, int budget,
	uint64_t *submission_id)
{
	int next, cmd_nr = 0;
	struct cmd_general_info *list = &rs->tail_list;
	struct cmd_tail_info *entry, *target = NULL;

	next = list->head;
	while (next != list->tail) {
		next++;
		if (next == list->count)
			next = 0;
		entry = &list->cmd[next];
		budget -= entry->cmd_nr;
		if (budget < 0)
			break;
		target = entry;
		cmd_nr += entry->cmd_nr;
	}

	if (target) {
		*submission_id = target->request_id;
		return cmd_nr;
	} else
		return MAX_CMD_BUDGET;
}

/* ring ALL, type = 0 */
static struct sub_op_bits sub_op_mi[] = {
	{31, 29},
	{28, 23},
};

static struct decode_info decode_info_mi = {
	"MI",
	OP_LEN_MI,
	ARRAY_SIZE(sub_op_mi),
	sub_op_mi,
};


/* ring RCS, command type 2 */
static struct sub_op_bits sub_op_2d[] = {
	{31, 29},
	{28, 22},
};

static struct decode_info decode_info_2d = {
	"2D",
	OP_LEN_2D,
	ARRAY_SIZE(sub_op_2d),
	sub_op_2d,
};

/* ring RCS, command type 3 */
static struct sub_op_bits sub_op_3d_media[] = {
	{31, 29},
	{28, 27},
	{26, 24},
	{23, 16},
};

static struct decode_info decode_info_3d_media = {
	"3D_Media",
	OP_LEN_3D_MEDIA,
	ARRAY_SIZE(sub_op_3d_media),
	sub_op_3d_media,
};

/* ring VCS, command type 3 */
static struct sub_op_bits sub_op_mfx_vc[] = {
	{31, 29},
	{28, 27},
	{26, 24},
	{23, 21},
	{20, 16},
};

static struct decode_info decode_info_mfx_vc = {
	"MFX_VC",
	OP_LEN_MFX_VC,
	ARRAY_SIZE(sub_op_mfx_vc),
	sub_op_mfx_vc,
};

/* ring VECS, command type 3 */
static struct sub_op_bits sub_op_vebox[] = {
	{31, 29},
	{28, 27},
	{26, 24},
	{23, 21},
	{20, 16},
};

static struct decode_info decode_info_vebox = {
	"VEBOX",
	OP_LEN_VEBOX,
	ARRAY_SIZE(sub_op_vebox),
	sub_op_vebox,
};

static struct decode_info* ring_decode_info[MAX_ENGINES][8] = {
	[RING_BUFFER_RCS] = {
		&decode_info_mi,
		NULL,
		NULL,
		&decode_info_3d_media,
		NULL,
		NULL,
		NULL,
		NULL,
	},

	[RING_BUFFER_VCS] = {
		&decode_info_mi,
		NULL,
		NULL,
		&decode_info_mfx_vc,
		NULL,
		NULL,
		NULL,
		NULL,
	},

	[RING_BUFFER_BCS] = {
		&decode_info_mi,
		NULL,
		&decode_info_2d,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
	},

	[RING_BUFFER_VECS] = {
		&decode_info_mi,
		NULL,
		NULL,
		&decode_info_vebox,
		NULL,
		NULL,
		NULL,
		NULL,
	},

	[RING_BUFFER_VCS2] = {
		&decode_info_mi,
		NULL,
		NULL,
		&decode_info_mfx_vc,
		NULL,
		NULL,
		NULL,
		NULL,
	},
};

uint32_t vgt_get_opcode(uint32_t cmd, int ring_id)
{
	struct decode_info * d_info;

	if (ring_id >= MAX_ENGINES)
		return INVALID_OP;

	d_info = ring_decode_info[ring_id][CMD_TYPE(cmd)];
	if (d_info == NULL)
		return INVALID_OP;

	return cmd >> (32 - d_info->op_len);
}

static inline uint32_t sub_op_val(uint32_t cmd, uint32_t hi, uint32_t low)
{
	return (cmd >> low) & ((1U << (hi-low+1)) - 1);
}

static void vgt_print_opcode(uint32_t cmd, int ring_id)
{
	struct decode_info * d_info;
	int i;

	if (ring_id >= MAX_ENGINES)
		return;

	d_info = ring_decode_info[ring_id][CMD_TYPE(cmd)];
	if (d_info == NULL)
		return;

	vgt_err("opcode=0x%x %s sub_ops:", cmd >> (32 - d_info->op_len), d_info->name);
	for (i = 0; i< d_info->nr_sub_op; i++) {
		vgt_err("0x%x ", sub_op_val(cmd, d_info->sub_op[i].hi,  d_info->sub_op[i].low));
	}
	vgt_err("\n");
}

static inline struct cmd_info* vgt_get_cmd_info(uint32_t cmd, int ring_id)
{
	uint32_t opcode;

	opcode = vgt_get_opcode(cmd, ring_id);
	if (opcode == INVALID_OP) {
		return NULL;
	}

	return vgt_find_cmd_entry(opcode, ring_id);
}

static inline uint32_t *cmd_ptr(struct parser_exec_state *s, int index)
{
	if (index < s->ip_buf_len)
		return s->ip_va + index;
	else
		return s->ip_va_next_page + (index - s->ip_buf_len);
}

static inline uint32_t *cmd_buf_ptr(struct parser_exec_state *s, int index)
{
	ASSERT(s->ip_buf_va);

	return s->ip_buf_va + index;
}

static inline uint32_t cmd_val(struct parser_exec_state *s, int index)
{
	uint32_t *addr;
	uint32_t ret = 0;

	if (s->ip_buf) {
		ret = *cmd_buf_ptr(s, index);
	} else {
		addr = cmd_ptr(s, index);
		if (s->shadow)
			ret = *addr;
		else
			hypervisor_read_va(s->vgt, addr, &ret, sizeof(ret), 1);
	}

	return ret;
}

static void parser_exec_state_dump(struct parser_exec_state *s)
{
	vgt_err("  vgt%d RING%d: ring_start(%08lx) ring_end(%08lx), ring_scan_head(%08lx) ring_scan_tail(%08lx)\n"
			, s->vgt->vgt_id,
			s->ring_id, s->ring_start, s->ring_start + s->ring_size, s->ring_head, s->ring_tail);

	vgt_err("  %s %s ip_gma(%08lx) ",
			s->buf_type == RING_BUFFER_INSTRUCTION ? "RING_BUFFER": "BATCH_BUFFER",
			s->buf_addr_type == GTT_BUFFER ? "GTT" : "PPGTT", s->ip_gma);

	if (s->ip_va == NULL) {
		vgt_err(" ip_va(NULL)\n");
	} else {
		int cnt = 0;
		/* print the whole page to trace */
		if (s->ip_buf) {
			kfree(s->ip_buf);
			s->ip_buf = s->ip_buf_va = NULL;
		}

		vgt_err("  ip_va=%p: %08x %08x %08x %08x \n",
				s->ip_va, cmd_val(s, 0), cmd_val(s, 1), cmd_val(s, 2), cmd_val(s, 3));

		vgt_print_opcode(cmd_val(s, 0), s->ring_id);

		s->ip_va = (uint32_t*)((((u64)s->ip_va) >> 12) << 12);
		while(cnt < 1024) {
		vgt_err("  DUMP ip_va=%p: %08x %08x %08x %08x %08x %08x %08x %08x \n",
				s->ip_va, cmd_val(s, 0), cmd_val(s, 1), cmd_val(s, 2), cmd_val(s, 3),
				          cmd_val(s, 4), cmd_val(s, 5), cmd_val(s, 6), cmd_val(s, 7));

			s->ip_va+=8;
			cnt+=8;
		}

	}
}
#define RING_BUF_WRAP(s, ip_gma)	(((s)->buf_type == RING_BUFFER_INSTRUCTION) && \
		((ip_gma) >= (s)->ring_start + (s)->ring_size))

static inline struct vgt_mm *parser_exec_state_to_mm(struct parser_exec_state *s)
{
	if (s->buf_addr_type != PPGTT_BUFFER)
		return s->vgt->gtt.ggtt_mm;
	else
		return s->vgt->rb[s->ring_id].active_ppgtt_mm;
}

/* get the system virtual address of reserved aperture */
static inline void *rsvd_gma_to_sys_va(struct pgt_device *pdev, unsigned long rsvd_gma)
{
	int rsvd_page_idx;
	void *page_va;
	void *rsvd_va;

	rsvd_page_idx = aperture_page_idx(pdev, rsvd_gma);
	page_va = page_address(aperture_page(pdev, rsvd_page_idx));
	rsvd_va = page_va + (rsvd_gma & (PAGE_SIZE - 1));

	return rsvd_va;
}

static int ip_gma_set(struct parser_exec_state *s, unsigned long ip_gma)
{
	unsigned long gma_next_page;

	ASSERT(VGT_REG_IS_ALIGNED(ip_gma, 4));

	/* set ip_gma */

	if (RING_BUF_WRAP(s, ip_gma)) {
		ip_gma = ip_gma - s->ring_size;
	}

	s->ip_gma = ip_gma;
	if (s->shadow)
		s->ip_va = rsvd_gma_to_sys_va(s->vgt->pdev, ip_gma);
	else
		s->ip_va = vgt_gma_to_va(parser_exec_state_to_mm(s), ip_gma);
	if (s->ip_va == NULL) {
		vgt_err("ERROR: gma %lx is invalid, fail to set\n", s->ip_gma);
		dump_stack();
		parser_exec_state_dump(s);
		return -EFAULT;
	}

	s->ip_buf_len = (PAGE_SIZE - (ip_gma & (PAGE_SIZE-1)))
		/ sizeof(uint32_t);

	/* set ip of next page */

	if (RING_BUF_WRAP(s, ip_gma + PAGE_SIZE))
		gma_next_page = s->ring_start;
	else
		gma_next_page = ((ip_gma >> PAGE_SHIFT) + 1) << PAGE_SHIFT;

	if (s->shadow)
		s->ip_va_next_page = rsvd_gma_to_sys_va(s->vgt->pdev, gma_next_page);
	else
		s->ip_va_next_page = vgt_gma_to_va(parser_exec_state_to_mm(s),
							gma_next_page);
	if (s->ip_va_next_page == NULL) {
		vgt_err("ERROR: next page gma %lx is invalid, fail to set\n",gma_next_page);
		dump_stack();
		parser_exec_state_dump(s);
		return -EFAULT;
	}

	if (!s->shadow) {
		hypervisor_read_va(s->vgt, s->ip_va, s->ip_buf,
				s->ip_buf_len * sizeof(uint32_t), 1);
		hypervisor_read_va(s->vgt, s->ip_va_next_page,
				s->ip_buf + s->ip_buf_len * sizeof(uint32_t),
				PAGE_SIZE, 1);
		s->ip_buf_va = s->ip_buf;
	}

	return 0;
}

static inline int ip_gma_advance(struct parser_exec_state *s, unsigned int dw_len)
{
	int rc = 0;
	if (s->ip_buf_len > dw_len) {
		/* not cross page, advance ip inside page */
		s->ip_gma += dw_len * sizeof(uint32_t);
		s->ip_va += dw_len;
		if (s->ip_buf)
			s->ip_buf_va += dw_len;
		s->ip_buf_len -= dw_len;
	} else {
		/* cross page, reset ip_va */
		rc = ip_gma_set(s, s->ip_gma + dw_len * sizeof(uint32_t));
	}
	return rc;
}

static inline int get_cmd_length(struct cmd_info *info, uint32_t cmd)
{
	if ((info->flag & F_LEN_MASK) == F_LEN_CONST)
		return info->len;
	else /* F_LEN_VAR */
		return (cmd & ((1U << info->len) - 1)) + 2;
}

static inline int cmd_length(struct parser_exec_state *s)
{
	return get_cmd_length(s->info, cmd_val(s, 0));
}

static int vgt_cmd_handler_mi_set_context(struct parser_exec_state* s)
{
	struct vgt_device *vgt = s->vgt;
	struct pgt_device *pdev = vgt->pdev;

	if (!IS_HSW(pdev)) {
		vgt_err("Unexpectted %s in VM%d command buffer!\n", s->info->name, vgt->vm_id);
		return -1;
	}

	if (!vgt->has_context) {
		printk("VM %d activate context\n", vgt->vm_id);
		vgt->has_context = 1;
	}

	return 0;
}

/*
 * Actually, we don't like to emulate register behavior in LRI handlers,
 * But DE_RRMR is an exception, even we can modify i915 to access
 * DE_RRMR via MMIO, the 2D driver will also access it via submitted
 * batch buffer.
 *
 * So we have no choice and have to handle it here, as windows is
 * using deferred filp from gen8+, MI_DISPLAY_FLIP and MI_WAIT_FOR_EVENT
 * will not be in the same submission. If a i915 submission modify
 * DE_RRMR after the filp submission, the wait submission of windows
 * will hang as the needed events are disabled by i915. Only modify i915
 * will not work, as 2D driver(xf86-video-intel) also modify it directly.
 * */

#define BIT_RANGE_MASK(a, b)	\
	((1UL << ((a) + 1)) - (1UL << (b)))
static int vgt_cmd_handler_lri_emulate(struct parser_exec_state *s)
{
	int i = 1;
	int cmd_len = cmd_length(s);
	unsigned int offset;
	uint32_t val;

	do {
		offset = cmd_val(s, i) & BIT_RANGE_MASK(22, 2);
		val = cmd_val(s, i + 1);

		if (offset == _REG_DE_RRMR || offset == FORCEWAKE_MT)
			break;
		i += 2;
	} while (i < cmd_len);

	if (i == cmd_len) {
		vgt_err("No DE_RRMR or MUL_FORCEWAKE in LRI?\n");
		return -EINVAL;
	}

	if (offset == _REG_DE_RRMR) {
		if (!vgt_rrmr_mmio_write(s->vgt, offset, &val, 4)) {
			vgt_err("fail to emulate register 0x%x!\n", offset);
			return -EINVAL;
		}
	} else if (offset == FORCEWAKE_MT) {
		if (!mul_force_wake_write(s->vgt, offset, &val, 4)) {
			vgt_err("fail to emulate register 0x%x!\n", offset);
			return -EINVAL;
		}
	}

	if (offset == _REG_DE_RRMR && add_patch_entry(s, cmd_ptr(s, i + 1),
				VGT_MMIO_READ(s->vgt->pdev, offset))) {
		vgt_err("fail to patch DE_RRMR LRI.\n");
		return -ENOSPC;
	}

	return 0;
}

static bool is_shadowed_mmio(unsigned int offset)
{
	bool ret = false;
	if ((offset == 0x2168) || /*BB current head register UDW */
	    (offset == 0x2140) || /*BB current header register */
	    (offset == 0x211c) || /*second BB header register UDW */
	    (offset == 0x2114)) { /*second BB header register UDW */
		ret = true;
	}

	return ret;
}

static int cmd_reg_handler(struct parser_exec_state *s,
	unsigned int offset, unsigned int index, char *cmd)
{
	struct vgt_device *vgt = s->vgt;
	struct pgt_device *pdev = vgt->pdev;
	int rc = -1;

	if (IS_HSW(s->vgt->pdev))
		return 0;

	if (!reg_is_mmio(pdev, offset + 3)) {
		rc = -1;
		goto reg_handle;
	}

	if ((reg_is_render(pdev, offset) &&
		!reg_addr_fix(pdev, offset) && offset != 0x24d0) ||
				reg_passthrough(pdev, offset) ||
				reg_pt_readonly(pdev, offset) ||
		(!vgt->vm_id && reg_is_config(pdev, offset))) {
		rc = 0;
	} else if (offset == _REG_DE_RRMR || offset == FORCEWAKE_MT) {
		if (!strcmp(cmd, "lri")) {
			rc = add_post_handle_entry(s, vgt_cmd_handler_lri_emulate);
			if (rc) {
				vgt_err("fail to allocate post handle\n");
			}
		}
	} else if (is_shadowed_mmio(offset)) {
		vgt_warn("VM-%d: !!! Found access of shadowed MMIO<0x%x>!\n",
			 s->vgt->vm_id, offset);
	}

reg_handle:
	if (!rc)
		reg_set_cmd_access(pdev, offset);
	else
		vgt_err("%s access to non-render register (%x)\n", cmd, offset);

	return rc;
}

static int vgt_cmd_handler_lri(struct parser_exec_state *s)
{
	int i, rc = 0;
	int cmd_len = cmd_length(s);
	struct pgt_device *pdev = s->vgt->pdev;

	for (i = 1; i < cmd_len; i += 2) {
		if (IS_BDW(pdev) && (s->ring_id != RING_BUFFER_RCS)) {
			if (s->ring_id == RING_BUFFER_BCS &&
				(cmd_val(s, i) & BIT_RANGE_MASK(22, 2)) == _REG_DE_RRMR)
				rc |= 0;
			else
				rc |= (cmd_val(s, i) & BIT_RANGE_MASK(22, 18)) ? -1 : 0;
		}

		if (rc)
			break;

		rc |= cmd_reg_handler(s,
			cmd_val(s, i) & BIT_RANGE_MASK(22, 2), i, "lri");
	}

	return rc;
}

static int vgt_cmd_handler_lrr(struct parser_exec_state *s)
{
	int i, rc = 0;
	int cmd_len = cmd_length(s);
	struct pgt_device *pdev = s->vgt->pdev;

	for (i = 1; i < cmd_len; i += 2) {
		if (IS_BDW(pdev))
			rc |= ((cmd_val(s, i) & BIT_RANGE_MASK(22, 18)) ||
			       (cmd_val(s, i + 1) & BIT_RANGE_MASK(22, 18))) ?
				-1 : 0;
		if (rc)
			break;

		rc |= cmd_reg_handler(s,
			cmd_val(s, i) & BIT_RANGE_MASK(22, 2), i, "lrr-src");
		rc |= cmd_reg_handler(s,
			cmd_val(s, i+1) & BIT_RANGE_MASK(22, 2), i, "lrr-dst");
	}

	return rc;
}

static int vgt_cmd_handler_lrm(struct parser_exec_state *s)
{
	int gmadr_bytes = s->vgt->pdev->device_info.gmadr_bytes_in_cmd;
	unsigned long gma;
	int i, rc = 0;
	int cmd_len = cmd_length(s);
	struct pgt_device *pdev = s->vgt->pdev;

	for (i = 1; i < cmd_len;) {
		if (IS_BDW(pdev))
			rc |= (cmd_val(s, i) & BIT_RANGE_MASK(22, 18)) ? -1 : 0;
		if (rc)
			break;

		rc |= cmd_reg_handler(s,
			cmd_val(s, i) & BIT_RANGE_MASK(22, 2), i, "lrm");

		if (cmd_val(s, 0) & (1 << 22)) {
			gma = cmd_val(s, i + 1) & BIT_RANGE_MASK(31, 2);
			if (gmadr_bytes == 8)
				gma |= (cmd_val(s, i + 2) & BIT_RANGE_MASK(15, 0)) << 32;
			rc |= cmd_address_audit(s, gma, sizeof(uint32_t), false);
		}

		i += gmadr_dw_number(s) + 1;
	}

	return rc;
}

static int vgt_cmd_handler_srm(struct parser_exec_state *s)
{
	int gmadr_bytes = s->vgt->pdev->device_info.gmadr_bytes_in_cmd;
	unsigned long gma;
	int i, rc = 0;
	int cmd_len = cmd_length(s);

	for (i = 1; i < cmd_len;) {
		rc |= cmd_reg_handler(s,
			cmd_val(s, i) & BIT_RANGE_MASK(22, 2), i, "srm");

		if (cmd_val(s, 0) & (1 << 22)) {
			gma = cmd_val(s, i + 1) & BIT_RANGE_MASK(31, 2);
			if (gmadr_bytes == 8)
				gma |= (cmd_val(s, i + 2) & BIT_RANGE_MASK(15, 0)) << 32;
			rc |= cmd_address_audit(s, gma, sizeof(uint32_t), false);
		}

		i += gmadr_dw_number(s) + 1;
	}

	return rc;
}

static int vgt_cmd_handler_pipe_control(struct parser_exec_state *s)
{
	int gmadr_bytes = s->vgt->pdev->device_info.gmadr_bytes_in_cmd;
	unsigned long gma;
	bool index_mode = false;
	int rc = 0;

	/* LRI post sync */
	if (cmd_val(s, 1) & PIPE_CONTROL_LRI_POST_SYNC)
		rc = cmd_reg_handler(s,
			cmd_val(s, 2) & BIT_RANGE_MASK(22, 2), 1, "pipe_ctrl");
	/* post sync */
	else if ((cmd_val(s, 1) & BIT_RANGE_MASK(15, 14))) {
		if ((cmd_val(s, 1) & (2 << 14)) == (2 << 14))
			rc = cmd_reg_handler(s, 0x2350, 1, "pipe_ctrl");
		else if ((cmd_val(s, 1) & (3 << 14)) == (3 << 14))
			rc = cmd_reg_handler(s, _REG_RCS_TIMESTAMP, 1, "pipe_ctrl");
		/* check ggtt*/
		if ((cmd_val(s, 2) & (1 << 2))) {
			gma = cmd_val(s, 2) & BIT_RANGE_MASK(31, 3);
			if (gmadr_bytes == 8)
				gma |= (cmd_val(s, 3) & BIT_RANGE_MASK(15, 0)) << 32;
			/* Store Data Index */
			if (cmd_val(s, 1) & (1 << 21))
				index_mode = true;

			rc |= cmd_address_audit(s, gma, sizeof(uint64_t), index_mode);
		}
	}

	if (!rc)
		s->cmd_issue_irq = (cmd_val(s, 1) & PIPE_CONTROL_NOTIFY) ? true : false;

	return rc;
}

static int vgt_cmd_handler_mi_user_interrupt(struct parser_exec_state *s)
{
	s->cmd_issue_irq = true;
	return 0;
}

static int vgt_cmd_advance_default(struct parser_exec_state *s)
{
	return ip_gma_advance(s, cmd_length(s));
}

static inline unsigned long vgt_get_gma_from_bb_start(
				struct vgt_device *vgt,
				int ring_id, unsigned long ip_gma)
{
	unsigned long bb_start_gma;
	uint32_t cmd;
	uint32_t opcode;
	void *va;

	if (g_gm_is_valid(vgt, ip_gma)) {
		bb_start_gma = 0;
		va = vgt_gma_to_va(vgt->gtt.ggtt_mm, ip_gma);
		hypervisor_read_va(vgt, va, &cmd, 4, 1);
		opcode = vgt_get_opcode(cmd, ring_id);
		ASSERT(opcode == OP_MI_BATCH_BUFFER_START);
		va = vgt_gma_to_va(vgt->gtt.ggtt_mm, ip_gma + 4);
		hypervisor_read_va(vgt, va, &bb_start_gma, 4, 1);
	} else if (g_gm_is_reserved(vgt, ip_gma)) {
		va = v_aperture(vgt->pdev, ip_gma);
		cmd = *(uint32_t *)va;
		opcode = vgt_get_opcode(cmd, ring_id);
		ASSERT(opcode == OP_MI_BATCH_BUFFER_START);
		bb_start_gma = *(unsigned long *)(va + 4);
	}
	return bb_start_gma;
}

static uint32_t vgt_find_bb_size(struct vgt_device *vgt,
				 struct vgt_mm *mm,
				 int ring_id,
				 unsigned long bb_start_cmd_gma)
{
	uint32_t bb_size = 0;
	unsigned long gma = 0;
	bool met_bb_end = false;
	uint32_t *va;
	struct cmd_info *info;
	uint32_t max_bb_size = 0;

	/* set gma as the start gm address of the batch buffer */
	gma = vgt_get_gma_from_bb_start(vgt, ring_id, bb_start_cmd_gma);
	if (!g_gm_is_valid(vgt, gma))
		return 0;
	max_bb_size = (g_gm_is_visible(vgt, gma) ? vgt_guest_visible_gm_end(vgt)
			: vgt_guest_hidden_gm_end(vgt)) - gma;
	do {
		uint32_t cmd;
		uint32_t cmd_length;
		va = vgt_gma_to_va(mm, gma);
		if (va == NULL) {
			vgt_err("VM-%d(ring %d>: Failed to get va of guest gma 0x%lx!\n",
				vgt->vm_id, ring_id, gma);
			return 0;
		}
		hypervisor_read_va(vgt, va, &cmd, sizeof(uint32_t), 1);
		info = vgt_get_cmd_info(cmd, ring_id);
		if (info == NULL) {
			vgt_err("ERROR: VM-%d: unknown cmd 0x%x! "
				"Failed to get batch buffer length.\n",
				vgt->vm_id, cmd);
			return 0;
		}

		if (info->opcode == OP_MI_BATCH_BUFFER_END) {
			met_bb_end = true;
		} else if (info->opcode == OP_MI_BATCH_BUFFER_START) {
			if (BATCH_BUFFER_2ND_LEVEL_BIT(cmd) == 0) {
				/* chained batch buffer */
				met_bb_end = true;
			}
		}

		cmd_length = get_cmd_length(info, cmd) << 2;
		bb_size += cmd_length;
		gma += cmd_length;
	} while (!met_bb_end && (bb_size < max_bb_size));

	if (bb_size >= max_bb_size) {
		vgt_err("ERROR: VM-%d: Failed to get batch buffer length! "
			"Not be able to find mi_batch_buffer_end command.\n",
			vgt->vm_id);
		return 0;
	}

	return bb_size;
}

static int vgt_cmd_handler_mi_batch_buffer_end(struct parser_exec_state *s)
{
	int rc;

	if (s->buf_type == BATCH_BUFFER_2ND_LEVEL) {
		s->buf_type = BATCH_BUFFER_INSTRUCTION;
		rc = ip_gma_set(s, s->ret_ip_gma_bb);
		s->buf_addr_type = s->saved_buf_addr_type;
	} else {
		s->buf_type = RING_BUFFER_INSTRUCTION;
		s->buf_addr_type = GTT_BUFFER;
		rc = ip_gma_set(s, s->ret_ip_gma_ring);
	}

	return rc;
}

/* TODO
 *
 * The mi_display_flip handler below is just a workaround. The completed
 * handling is under discussion. The current approach is to NOOP the
 * MI_DISPLAY_FLIP command and then do pre-emulation. The pre-emulation
 * cannot exactly emulate command's behavior since it happens before
 * the command is issued. Consider the following two cases: one is that
 * right after the command-scan, display switch happens. Another case
 * is that commands inside ring buffer has some dependences.
 *
 * The interrupt is another consideration. mi_display_flip can trigger
 * interrupt for completion. VM's gfx driver may rely on that. Whether
 * we should inject virtual interrupt and when is the right time.
 *
 * The user space could resubmit ring/batch buffer with partially updated
 * MI_DISPLAY_FLIP. So special handling is needed in command parser for
 * such scenario.
 *
 * And we did not update HW state for the display flip.
 *
 */
#define PLANE_SELECT_SHIFT	19
#define PLANE_SELECT_MASK	(0x7 << PLANE_SELECT_SHIFT)
#define SKL_PLANE_SELECT_SHIFT  8
#define SKL_PLANE_SELECT_MASK   (0x1F << SKL_PLANE_SELECT_SHIFT)
#define SURF_MASK		0xFFFFF000
#define PITCH_MASK		0x0000FFC0
#define TILE_PARA_SHIFT		0x0
#define TILE_PARA_MASK		0x1
#define SKL_TILE_PARA_MASK      0x7
/* Primary plane and sprite plane has the same tile shift in control reg */
#define PLANE_TILE_SHIFT	_PRI_PLANE_TILE_SHIFT
#define PLANE_TILE_MASK		(0x1 << PLANE_TILE_SHIFT)
#define FLIP_TYPE_MASK		0x3

#define DISPLAY_FLIP_PLANE_A  0x0
#define DISPLAY_FLIP_PLANE_B  0x1
#define DISPLAY_FLIP_SPRITE_A  0x2
#define DISPLAY_FLIP_SPRITE_B  0x3
#define DISPLAY_FLIP_PLANE_C  0x4
#define DISPLAY_FLIP_SPRITE_C  0x5


/* The NOOP for MI_DISPLAY_FLIP has below information stored in NOOP_ID:
 *
 *	bit 21 - bit 16 is 0x14, opcode of MI_DISPLAY_FLIP;
 *	bit 10 - bit 8  is plane select;
 *	bit 7  - bit 0  is the cmd length
 */
#define PLANE_INFO_SHIFT	8
#define PLANE_INFO_MASK		(0x7 << PLANE_INFO_SHIFT)

static bool display_flip_decode_plane_info(struct pgt_device *pdev,
		uint32_t  plane_code, enum pipe *pipe,
		enum vgt_plane_type *plane)
{
	if (IS_SKL(pdev)) {
		plane_code <<= SKL_PLANE_SELECT_SHIFT;
		switch (plane_code) {
		case MI_DISPLAY_FLIP_SKL_PLANE_1_A:
			*pipe = PIPE_A;
			break;
		case MI_DISPLAY_FLIP_SKL_PLANE_1_B:
			*pipe = PIPE_B;
			break;
		case MI_DISPLAY_FLIP_SKL_PLANE_1_C:
			*pipe = PIPE_C;
			break;
		default:
			vgt_warn("unknown plane_code 0x%x\n", plane_code);
			return false;
		}
		*plane = PRIMARY_PLANE;
		return true;
	}

	switch (plane_code) {
		case DISPLAY_FLIP_PLANE_A:
			*pipe = PIPE_A;
			*plane = PRIMARY_PLANE;
			break;
		case DISPLAY_FLIP_PLANE_B:
			*pipe = PIPE_B;
			*plane = PRIMARY_PLANE;
			break;
		case DISPLAY_FLIP_SPRITE_A:
			*pipe = PIPE_A;
			*plane = SPRITE_PLANE;
			break;
		case DISPLAY_FLIP_SPRITE_B:
			*pipe = PIPE_B;
			*plane = SPRITE_PLANE;
			break;
		case DISPLAY_FLIP_PLANE_C:
			*pipe = PIPE_C;
			*plane = PRIMARY_PLANE;
			break;
		case DISPLAY_FLIP_SPRITE_C:
			*pipe = PIPE_C;
			*plane = SPRITE_PLANE;
			break;
		default:
			return false;
	}

	return true;

}

static bool display_flip_encode_plane_info(struct pgt_device *pdev,
		enum pipe pipe, enum vgt_plane_type plane,
		uint32_t *plane_code)
{
	if (IS_SKL(pdev)) {
		if (plane != PRIMARY_PLANE) {
			vgt_err("only support primary_plane\n");
			return false;
		}

		switch (pipe) {
		case PIPE_A:
			*plane_code = MI_DISPLAY_FLIP_SKL_PLANE_1_A;
			break;
		case PIPE_B:
			*plane_code = MI_DISPLAY_FLIP_SKL_PLANE_1_B;
			break;
		case PIPE_C:
			*plane_code = MI_DISPLAY_FLIP_SKL_PLANE_1_C;
			break;
		default:
			vgt_warn("unknown pipe 0x%x\n", pipe);
			return false;
		}
		return true;
	}

	if (pipe == PIPE_A && plane == PRIMARY_PLANE)
		*plane_code = DISPLAY_FLIP_PLANE_A;
	else if (pipe == PIPE_B && plane == PRIMARY_PLANE)
		*plane_code = DISPLAY_FLIP_PLANE_B;
	else if (pipe == PIPE_A && plane == SPRITE_PLANE)
		*plane_code = DISPLAY_FLIP_SPRITE_A;
	else if (pipe == PIPE_B && plane == SPRITE_PLANE)
		*plane_code = DISPLAY_FLIP_SPRITE_B;
	else if (pipe == PIPE_C && plane == PRIMARY_PLANE)
		*plane_code = DISPLAY_FLIP_PLANE_C;
	else if (pipe == PIPE_C && plane == SPRITE_PLANE)
		*plane_code = DISPLAY_FLIP_SPRITE_C;
	else
		return false;

	return true;

}

#define GET_INFO_FOR_FLIP(pipe, plane, 					\
			ctrl_reg, surf_reg, stride_reg, stride_mask)	\
do{									\
	if (plane == PRIMARY_PLANE) {					\
		ctrl_reg = VGT_DSPCNTR(pipe);				\
		surf_reg = VGT_DSPSURF(pipe);				\
		stride_reg = VGT_DSPSTRIDE(pipe);			\
		stride_mask = _PRI_PLANE_STRIDE_MASK;			\
	} else {							\
		ASSERT (plane == SPRITE_PLANE);				\
		ctrl_reg = VGT_SPRCTL(pipe);				\
		surf_reg = VGT_SPRSURF(pipe);				\
		stride_reg = VGT_SPRSTRIDE(pipe);			\
		stride_mask = _SPRITE_STRIDE_MASK;			\
	}								\
}while(0);

static bool vgt_flip_parameter_check(struct parser_exec_state *s,
				uint32_t plane_code,
				uint32_t stride_val,
				uint32_t surf_val)
{
	struct pgt_device *pdev = s->vgt->pdev;
	enum pipe pipe = I915_MAX_PIPES;
	enum vgt_plane_type plane = MAX_PLANE;
	uint32_t surf_reg, ctrl_reg;
	uint32_t stride_reg, stride_mask;
	uint32_t tile_para, tile_in_ctrl;
	uint32_t plane_tile_mask, tile_mask, stride_shift;
	bool async_flip;

	if (!display_flip_decode_plane_info(pdev, plane_code, &pipe, &plane))
		return false;

	GET_INFO_FOR_FLIP(pipe, plane,
			ctrl_reg, surf_reg, stride_reg, stride_mask);

	stride_mask = IS_SKL(pdev) ? SKL_PLANE_STRIDE_MASK : stride_mask;
	tile_mask = IS_SKL(pdev) ? SKL_TILE_PARA_MASK : TILE_PARA_MASK;
	plane_tile_mask = IS_SKL(pdev) ? PLANE_CTL_TILED_MASK : PLANE_TILE_MASK;
	stride_shift = IS_SKL(pdev) ? _PRI_PLANE_STRIDE_SHIFT : 0;

	async_flip = ((surf_val & FLIP_TYPE_MASK) == 0x1);
	tile_para = ((stride_val & tile_mask) >> TILE_PARA_SHIFT);
	tile_in_ctrl = (__vreg(s->vgt, ctrl_reg) & plane_tile_mask)
				>> PLANE_TILE_SHIFT;

	if (((__vreg(s->vgt, stride_reg) & stride_mask) << stride_shift)
		!= (stride_val & PITCH_MASK)) {

		if (async_flip) {
			vgt_warn("Cannot change stride in async flip!\n");
			return false;
		}
	}

	if (tile_para != tile_in_ctrl) {

		if (async_flip) {
			vgt_warn("Cannot change tiling in async flip!\n");
			return false;
		}
	}

	return true;
}

static int vgt_handle_mi_display_flip(struct parser_exec_state *s)
{
	struct vgt_device *vgt = s->vgt;
	struct pgt_device *pdev = vgt->pdev;
	uint32_t surf_reg, surf_val, ctrl_reg;
	uint32_t stride_reg, stride_val, stride_mask;
	uint32_t tile_para;
	uint32_t opcode, plane_code, real_plane_code;
	enum pipe pipe;
	enum pipe real_pipe;
	enum vgt_plane_type plane;
	int i, length, rc = 0;
	struct fb_notify_msg msg;
	uint32_t value;
	int surf_size = 0;
	uint32_t plane_select_mask, plane_select_shift;
	uint32_t tile_mask, plane_tile_mask, stride_shift;

	opcode = cmd_val(s, 0);
	stride_val = cmd_val(s, 1);
	surf_val = cmd_val(s, 2);

	plane_select_mask = IS_SKL(pdev) ? SKL_PLANE_SELECT_MASK :
		PLANE_SELECT_MASK;
	plane_select_shift = IS_SKL(pdev) ? SKL_PLANE_SELECT_SHIFT :
		PLANE_SELECT_MASK;
	tile_mask = IS_SKL(pdev) ? SKL_TILE_PARA_MASK : TILE_PARA_MASK;
	plane_tile_mask = IS_SKL(pdev) ? PLANE_CTL_TILED_MASK : PLANE_TILE_MASK;
	stride_shift = IS_SKL(pdev) ? _PRI_PLANE_STRIDE_SHIFT : 0;

	plane_code = (opcode & plane_select_mask) >> plane_select_shift;
	length = cmd_length(s);

	if (!display_flip_decode_plane_info(pdev, plane_code, &pipe, &plane)) {
		vgt_warn("Invalid pipe/plane in MI_DISPLAY_FLIP!\n");
		goto wrong_command;
	}

	real_pipe = s->vgt->pipe_mapping[pipe];

	if (length == 4) {
		vgt_warn("Page flip of Stereo 3D is not supported!\n");
		goto wrong_command;
	} else if (length != 3) {
		vgt_warn("Flip length not equal to 3, ignore handling flipping");
		goto wrong_command;
	}

	{
		if (!vgt_flip_parameter_check(s, plane_code, stride_val, surf_val))
			goto wrong_command;

		GET_INFO_FOR_FLIP(pipe, plane,
			ctrl_reg, surf_reg, stride_reg, stride_mask);

		stride_mask = IS_SKL(pdev) ? SKL_PLANE_STRIDE_MASK :
			stride_mask;
		tile_para = ((stride_val & tile_mask) >> TILE_PARA_SHIFT);

		__vreg(s->vgt, stride_reg) = ((stride_val >> stride_shift) &
				stride_mask) |
				(__vreg(s->vgt, stride_reg) & (~stride_mask));
		__vreg(s->vgt, ctrl_reg) = (tile_para << PLANE_TILE_SHIFT) |
				(__vreg(s->vgt, ctrl_reg) & (~plane_tile_mask));
		__vreg(s->vgt, surf_reg) = (surf_val & SURF_MASK) |
				(__vreg(s->vgt, surf_reg) & (~SURF_MASK));
		__sreg(s->vgt, stride_reg) = __vreg(s->vgt, stride_reg);
		__sreg(s->vgt, ctrl_reg) = __vreg(s->vgt, ctrl_reg);
		__sreg(s->vgt, surf_reg) = __vreg(s->vgt, surf_reg);

		if (plane == PRIMARY_PLANE) {
			struct vgt_primary_plane_format pri_fmt;
			if (!vgt_decode_primary_plane_format(vgt, real_pipe, &pri_fmt)) {
				if (pri_fmt.enabled)
					surf_size = pri_fmt.height * pri_fmt.stride;
			} else {
				return -1;
			}
		} else {
			struct vgt_sprite_plane_format spr_fmt;
			if (!vgt_decode_sprite_plane_format(vgt, real_pipe, &spr_fmt)) {
				if (spr_fmt.enabled)
					surf_size = spr_fmt.height * spr_fmt.width * spr_fmt.bpp / 8 ;
			} else {
				return -1;
			}
		}
		rc = cmd_address_audit(s, surf_val & BIT_RANGE_MASK(31, 12), surf_size, false);
		if (rc < 0)
			return rc;
	}

	__vreg(s->vgt, VGT_PIPE_FLIPCOUNT(pipe))++;

	msg.vm_id = s->vgt->vm_id;
	msg.pipe_id = pipe;
	msg.plane_id = plane;
	vgt_fb_notifier_call_chain(FB_DISPLAY_FLIP, &msg);

	if ((s->vgt == current_foreground_vm(s->vgt->pdev))) {
		if (!display_flip_encode_plane_info(pdev, real_pipe,
					plane, &real_plane_code))
			goto wrong_command;

		value = cmd_val(s, 0);
		add_patch_entry(s,
			cmd_ptr(s, 0),
			((value & ~plane_select_mask) |
			 (real_plane_code << plane_select_shift)));

		vgt_inject_flip_done(s->vgt, pipe);

		return 0;
	}

	vgt_dbg(VGT_DBG_CMD, "VM %d: mi_display_flip to be ignored\n",
		s->vgt->vm_id);

	for (i = 1; i < length; i ++) {
		rc |= add_patch_entry(s, cmd_ptr(s, i), MI_NOOP);
	}

	rc |= add_patch_entry(s, cmd_ptr(s, 0), MI_NOOP);

	vgt_inject_flip_done(s->vgt, pipe);

	return rc;

wrong_command:
	for (i = 0; i < length; i ++)
		rc |= add_patch_entry(s, cmd_ptr(s, i), MI_NOOP);
	return rc;
}

static int vgt_cmd_handler_mi_display_flip(struct parser_exec_state *s)
{
	return vgt_handle_mi_display_flip(s);
}
static bool is_wait_for_flip_pending(uint32_t cmd)
{
	return cmd & (MI_WAIT_FOR_PLANE_A_FLIP_PENDING |
		MI_WAIT_FOR_PLANE_B_FLIP_PENDING |
		MI_WAIT_FOR_PLANE_C_FLIP_PENDING |
		MI_WAIT_FOR_SPRITE_A_FLIP_PENDING |
		MI_WAIT_FOR_SPRITE_B_FLIP_PENDING |
		MI_WAIT_FOR_SPRITE_C_FLIP_PENDING);
}

static int vgt_handle_mi_wait_for_event(struct parser_exec_state *s)
{
	int rc = 0;
	enum pipe virtual_pipe = I915_MAX_PIPES;
	enum pipe real_pipe = I915_MAX_PIPES;
	uint32_t cmd = cmd_val(s, 0);
	uint32_t new_cmd = cmd;
	enum vgt_plane_type plane_type = MAX_PLANE;

	if (!is_wait_for_flip_pending(cmd))
		return rc;

	if (s->vgt != current_foreground_vm(s->vgt->pdev)) {
		rc |= add_patch_entry(s, cmd_ptr(s, 0), MI_NOOP);
		vgt_dbg(VGT_DBG_CMD, "VM %d: mi_wait_for_event to be ignored\n", s->vgt->vm_id);
		return rc;
	}

	if (cmd & MI_WAIT_FOR_PLANE_A_FLIP_PENDING) {
		virtual_pipe = PIPE_A;
		plane_type = PRIMARY_PLANE;
		new_cmd &= ~MI_WAIT_FOR_PLANE_A_FLIP_PENDING;
	} else if (cmd & MI_WAIT_FOR_PLANE_B_FLIP_PENDING) {
		virtual_pipe = PIPE_B;
		plane_type = PRIMARY_PLANE;
		new_cmd &= ~MI_WAIT_FOR_PLANE_B_FLIP_PENDING;
	} else if (cmd & MI_WAIT_FOR_PLANE_C_FLIP_PENDING) {
		virtual_pipe = PIPE_C;
		plane_type = PRIMARY_PLANE;
		new_cmd &= ~MI_WAIT_FOR_PLANE_C_FLIP_PENDING;
	} else if (cmd & MI_WAIT_FOR_SPRITE_A_FLIP_PENDING) {
		virtual_pipe = PIPE_A;
		plane_type = SPRITE_PLANE;
		new_cmd &= ~MI_WAIT_FOR_SPRITE_A_FLIP_PENDING;
	} else if (cmd & MI_WAIT_FOR_SPRITE_B_FLIP_PENDING) {
		virtual_pipe = PIPE_B;
		plane_type = SPRITE_PLANE;
		new_cmd &= ~MI_WAIT_FOR_SPRITE_B_FLIP_PENDING;
	} else  if(cmd & MI_WAIT_FOR_SPRITE_C_FLIP_PENDING) {
		virtual_pipe = PIPE_C;
		plane_type = SPRITE_PLANE;
		new_cmd &= ~MI_WAIT_FOR_SPRITE_C_FLIP_PENDING;
	} else {
		ASSERT(0);
	}

	real_pipe = s->vgt->pipe_mapping[virtual_pipe];

	if (real_pipe == PIPE_A && plane_type == PRIMARY_PLANE) {
		new_cmd |= MI_WAIT_FOR_PLANE_A_FLIP_PENDING;
	} else if (real_pipe == PIPE_B && plane_type == PRIMARY_PLANE) {
		new_cmd |= MI_WAIT_FOR_PLANE_B_FLIP_PENDING;
	} else if (real_pipe == PIPE_C && plane_type == PRIMARY_PLANE) {
		new_cmd |= MI_WAIT_FOR_PLANE_C_FLIP_PENDING;
	} else if (real_pipe == PIPE_A && plane_type == SPRITE_PLANE) {
		new_cmd |= MI_WAIT_FOR_SPRITE_A_FLIP_PENDING;
	} else if (real_pipe == PIPE_B && plane_type == SPRITE_PLANE) {
		new_cmd |= MI_WAIT_FOR_SPRITE_B_FLIP_PENDING;
	} else if (real_pipe == PIPE_C && plane_type == SPRITE_PLANE) {
		new_cmd |= MI_WAIT_FOR_SPRITE_C_FLIP_PENDING;
	} else {
		rc = add_patch_entry(s, cmd_ptr(s, 0), MI_NOOP);
		return rc;
	}
	rc = add_patch_entry(s, cmd_ptr(s, 0), new_cmd);
	return rc;
}

static unsigned long get_gma_bb_from_cmd(struct parser_exec_state *s, int index)
{
	unsigned long addr;
	unsigned long gma_high, gma_low;
	int gmadr_bytes = s->vgt->pdev->device_info.gmadr_bytes_in_cmd;

	gma_low = cmd_val(s, index) & BATCH_BUFFER_ADDR_MASK;

	if (gmadr_bytes == 4) {
		addr = gma_low;
	} else {
		gma_high = cmd_val(s, index + 1) & BATCH_BUFFER_ADDR_HIGH_MASK;
		addr = (((unsigned long)gma_high) << 32) | gma_low;
	}

	return addr;
}

static inline int cmd_address_audit(struct parser_exec_state *s, unsigned long g_addr,
	int op_size, bool index_mode)
{
	struct vgt_device *vgt = s->vgt;
	int max_surface_size = vgt->pdev->device_info.max_surface_size;
	int i;
	int rc = 0;

	if (op_size > max_surface_size) {
		vgt_err("cmd_parser: misusage of the address audit or malicious %s detected!\n", s->info->name);
		return -1;
	}

	if (s->vgt->vgt_id == 0)
		return rc;

	if (index_mode)	{
		if (g_addr >= PAGE_SIZE/sizeof(uint64_t))
			rc = -1;
	} else if ((!g_gm_is_valid(vgt, g_addr))
		|| (!g_gm_is_valid(vgt, g_addr + op_size - 1))) {
		rc = -1;
	}

	if (rc < 0) {
		vgt_err("cmd_parser: Malicious %s detected, addr=0x%lx, len=%d!\n",
			s->info->name, g_addr, op_size);

		printk("cmd dump: ");
		for (i = 0; i < cmd_length(s); i++) {
			if (!(i % 4))
				printk("\n%08x ", cmd_val(s, i));
			else
				printk("%08x ", cmd_val(s, i));
		}
		printk("\ncurrent VM addr range: visible 0x%llx - 0x%llx, hidden 0x%llx - 0x%llx\n",
			vgt_guest_visible_gm_base(vgt), vgt_guest_visible_gm_end(vgt),
			vgt_guest_hidden_gm_base(vgt), vgt_guest_hidden_gm_end(vgt));
	}

	return rc;
}

static int vgt_cmd_handler_mi_store_data_imm(struct parser_exec_state *s)
{
	int gmadr_bytes = s->vgt->pdev->device_info.gmadr_bytes_in_cmd;
	int op_size = (cmd_length(s) - 3) * sizeof(uint32_t);
	int core_id = (cmd_val(s, 2) & (1 << 0)) ? 1 : 0;
	unsigned long gma, gma_low, gma_high;
	int rc = 0;

	/* check ppggt */
	if (!(cmd_val(s, 0) & (1 << 22)))
		return rc;

	gma = cmd_val(s, 2) & BIT_RANGE_MASK(31, 2);

	if (gmadr_bytes == 8) {
		gma_low = cmd_val(s, 1) & BIT_RANGE_MASK(31, 2);
		gma_high = cmd_val(s, 2) & BIT_RANGE_MASK(15, 0);
		gma = (gma_high << 32) | gma_low;
		core_id = (cmd_val(s, 1) & (1 << 0)) ? 1 : 0;
	}

	rc = cmd_address_audit(s, gma + op_size * core_id, op_size, false);

	return rc;
}

static int vgt_cmd_handler_mi_semaphore_wait(struct parser_exec_state *s)
{
	struct vgt_device *vgt = s->vgt;
	struct pgt_device *pdev = vgt->pdev;

	if (IS_HSW(pdev))
		return 0;

	vgt_err("Unexpected %s in VM%d command buffer!\n", s->info->name, vgt->vm_id);
	return -1;
}

static int vgt_cmd_handler_mi_report_perf_count(struct parser_exec_state *s)
{
	struct vgt_device *vgt = s->vgt;
	struct pgt_device *pdev = vgt->pdev;

	if (IS_HSW(pdev))
		return 0;

	vgt_err("Unexpected %s in VM%d command buffer!\n", s->info->name, vgt->vm_id);
	return -1;
}

static int vgt_cmd_handler_mi_op_2e(struct parser_exec_state *s)
{
	struct vgt_device *vgt = s->vgt;
	struct pgt_device *pdev = vgt->pdev;

	if (IS_HSW(pdev))
		return 0;

	vgt_err("Unexpected %s in VM%d command buffer!\n", s->info->name, vgt->vm_id);
	return -1;
}

static int vgt_cmd_handler_mi_op_2f(struct parser_exec_state *s)
{
	int gmadr_bytes = s->vgt->pdev->device_info.gmadr_bytes_in_cmd;
	int op_size = (1 << (cmd_val(s, 0) & BIT_RANGE_MASK(20, 19) >> 19)) *
				sizeof(uint32_t);
	unsigned long gma, gma_high;
	int rc = 0;

	if (!(cmd_val(s, 0) & (1 << 22)))
		return rc;

	gma = cmd_val(s, 1) & BIT_RANGE_MASK(31, 2);
	if (gmadr_bytes == 8) {
		gma_high = cmd_val(s, 2) & BIT_RANGE_MASK(15, 0);
		gma = (gma_high << 32) | gma;
	}
	rc = cmd_address_audit(s, gma, op_size, false);

	return rc;
}

static int vgt_cmd_handler_mi_store_data_index(struct parser_exec_state *s)
{
	struct vgt_device *vgt = s->vgt;
	struct pgt_device *pdev = vgt->pdev;

	if (IS_HSW(pdev))
		return 0;

	vgt_err("Unexpected %s in VM%d command buffer!\n", s->info->name, vgt->vm_id);
	return -1;
}

static int vgt_cmd_handler_mi_clflush(struct parser_exec_state *s)
{
	struct vgt_device *vgt = s->vgt;
	struct pgt_device *pdev = vgt->pdev;

	if (IS_HSW(pdev))
		return 0;

	vgt_err("Unexpected %s in VM%d command buffer!\n", s->info->name, vgt->vm_id);
	return -1;
}

static int vgt_cmd_handler_mi_conditional_batch_buffer_end(struct parser_exec_state *s)
{
	struct vgt_device *vgt = s->vgt;
	struct pgt_device *pdev = vgt->pdev;

	if (IS_HSW(pdev))
		return 0;

	vgt_err("Unexpected %s in VM%d command buffer!\n", s->info->name, vgt->vm_id);
	return -1;
}

static int vgt_cmd_handler_mi_update_gtt(struct parser_exec_state *s)
{
	vgt_err("Unexpectted mi_update_gtt in VM command buffer\n");
	return -1;
}

static int vgt_cmd_handler_mi_flush_dw(struct parser_exec_state* s)
{
	int gmadr_bytes = s->vgt->pdev->device_info.gmadr_bytes_in_cmd;
	unsigned long gma;
	bool index_mode = false;
	int rc = 0;

	/* Check post-sync and ppgtt bit */
	if (((cmd_val(s, 0) >> 14) & 0x3) && (cmd_val(s, 1) & (1 << 2))) {

		gma = cmd_val(s, 1) & BIT_RANGE_MASK(31, 3);
		if (gmadr_bytes == 8)
			gma |= (cmd_val(s, 2) & BIT_RANGE_MASK(15, 0)) << 32;
		/* Store Data Index */
		if (cmd_val(s, 0) & (1 << 21))
			index_mode = true;

		rc = cmd_address_audit(s, gma, sizeof(uint64_t), index_mode);
	}
	/* Check notify bit */
	if (!rc)
		s->cmd_issue_irq = (cmd_val(s, 0) & (1 << 8)) ? true : false;

	return rc;
}

static void addr_type_update_snb(struct parser_exec_state* s)
{
	if ((s->buf_type == RING_BUFFER_INSTRUCTION) &&
			(s->vgt->rb[s->ring_id].has_ppgtt_mode_enabled) &&
			(BATCH_BUFFER_ADR_SPACE_BIT(cmd_val(s, 0)) == 1)) {
		s->buf_addr_type = PPGTT_BUFFER;
	}
}

/*
 * Check whether a batch buffer needs to be scanned. Currently
 * the only criteria is based on privilege.
 */
static int batch_buffer_needs_scan(struct parser_exec_state *s)
{
	struct pgt_device *pdev = s->vgt->pdev;

	if (IS_BDW(pdev) || IS_SKL(pdev)) {
		/* BDW decides privilege based on address space */
		if (cmd_val(s, 0) & (1 << 8))
			return 0;
	} else if (IS_HSW(pdev)) {
		/* only RCS on HSW has dedicated privilege bit */
		if (s->ring_id == 0) {
			if (cmd_val(s, 0) & (1 << 13))
				return 0;
		} else {
			if (cmd_val(s, 0) & (1 << 8))
				return 0;
		}
	}

	return 1;
}

#define LITE_RESTORE_FLOOD_THRESHOLD 1000
static int vgt_perform_bb_shadow(struct parser_exec_state *s)
{
	struct vgt_device *vgt = s->vgt;
	unsigned long *reloc_va;
	unsigned long bb_start_gma = 0;
	unsigned long bb_start_aligned;
	uint32_t bb_start_offset;

	uint32_t bb_size;
	uint32_t bb_page_num;
	unsigned long shadow_bb_start_gma;
	unsigned long shadow_base_hpa, shadow_hpa;
	unsigned long bb_guest_gma;
	int i;

	reloc_va = rsvd_gma_to_sys_va(vgt->pdev, s->ip_gma + 4);
	bb_start_gma = *reloc_va;

	bb_start_offset = bb_start_gma & (PAGE_SIZE - 1);
	bb_start_aligned = bb_start_gma - bb_start_offset;

	bb_size = vgt_find_bb_size(vgt, vgt->gtt.ggtt_mm,
				   s->ring_id, s->ip_gma);
	if (bb_size == 0) {
		vgt_err("VM-%d<ring-%d>: Failed to get batch buffer size!\n",
			vgt->vm_id, s->ring_id);
		goto shadow_err;
	}
	bb_page_num = (bb_start_offset + bb_size + PAGE_SIZE - 1) >> GTT_PAGE_SHIFT;

	/* allocate gm space */
	shadow_base_hpa = rsvd_aperture_alloc(vgt->pdev, bb_page_num * PAGE_SIZE);
	if (shadow_base_hpa == 0) {
		vgt_err("VM-%d: Failed to allocate gm for shadow privilege bb!\n",
			vgt->vm_id);
		goto shadow_err;
	}

	shadow_bb_start_gma = aperture_2_gm(vgt->pdev, shadow_base_hpa);
	shadow_bb_start_gma += bb_start_offset;

	/* copy aligned pages from guest cmd buf into shadow */
	shadow_hpa = shadow_base_hpa;
	bb_guest_gma = bb_start_aligned;
	for (i = 0; i < bb_page_num; ++ i) {
		struct shadow_cmd_page *s_cmd_page;
		unsigned long shadow_gma;
		void *guest_bb_va, *shadow_bb_va;

		shadow_gma = aperture_2_gm(vgt->pdev, shadow_hpa);

		s_cmd_page = kzalloc(sizeof(struct shadow_cmd_page), GFP_ATOMIC);
		if (!s_cmd_page) {
			vgt_err ("VM-%d<ring %d>: Failed to allocate memory "
				 "for shadow batch buffer!\n",
				 vgt->vm_id, s->ring_id);
			rsvd_aperture_free(vgt->pdev, shadow_base_hpa,
					   bb_page_num * PAGE_SIZE);
			goto shadow_err;
		}

		s_cmd_page->guest_gma = bb_guest_gma;
		s_cmd_page->bound_gma = shadow_gma;

		if (s->el_ctx->shadow_priv_bb.n_pages++ > LITE_RESTORE_FLOOD_THRESHOLD)
			rsvd_aperture_runout_handler(vgt->pdev);

		list_add_tail(&s_cmd_page->list,
			      &s->el_ctx->shadow_priv_bb.pages);

		guest_bb_va = vgt_gma_to_va(vgt->gtt.ggtt_mm, bb_guest_gma);
		if (guest_bb_va == NULL) {
			vgt_err("VM-%d(ring %d>: Failed to get guest bb va for 0x%lx! "
				"MI command gma: 0x%lx, size 0x%x\n",
				vgt->vm_id, s->ring_id, bb_guest_gma,
				s->ip_gma, bb_size);
			goto shadow_err;
		}

		shadow_bb_va = rsvd_gma_to_sys_va(vgt->pdev, s_cmd_page->bound_gma);

		hypervisor_read_va(vgt, guest_bb_va, shadow_bb_va,
				   PAGE_SIZE, 1);

		shadow_hpa += PAGE_SIZE;
		bb_guest_gma += PAGE_SIZE;
	}

	/* perform relocation for mi_batch_buffer_start */
	*reloc_va = shadow_bb_start_gma;
	trace_shadow_bb_relocate(vgt->vm_id, s->ring_id,
			      s->el_ctx->guest_context.lrca,
			      s->ip_gma + 4, bb_start_gma, shadow_bb_start_gma, bb_size);

	return 0;
shadow_err:
		printk("MI_BATCH_BUFFER_START<gma addr:0x%lx>: "
		       "[0x%x][0x%x][0x%x]\n",
		       s->ip_gma, cmd_val(s, 0), cmd_val(s, 1), cmd_val(s, 2));
	return -1;
}

static int vgt_cmd_handler_mi_batch_buffer_start(struct parser_exec_state *s)
{
	int rc=0;
	bool second_level;

	if (s->buf_type == BATCH_BUFFER_2ND_LEVEL) {
		vgt_err("MI_BATCH_BUFFER_START not allowd in 2nd level batch buffer\n");
		return -EINVAL;
	}

	second_level = BATCH_BUFFER_2ND_LEVEL_BIT(cmd_val(s, 0)) == 1;
	if (second_level && (s->buf_type != BATCH_BUFFER_INSTRUCTION)) {
		vgt_err("Jumping to 2nd level batch buffer from ring buffer is not allowd\n");
		return -EINVAL;
	}

	s->saved_buf_addr_type = s->buf_addr_type;

	/* FIXME: add IVB/HSW code */
	addr_type_update_snb(s);

	if (s->buf_type == RING_BUFFER_INSTRUCTION) {
		s->ret_ip_gma_ring = s->ip_gma + cmd_length(s) * sizeof(uint32_t);
		s->buf_type = BATCH_BUFFER_INSTRUCTION;
	} else if (second_level) {
		s->buf_type = BATCH_BUFFER_2ND_LEVEL;
		s->ret_ip_gma_bb = s->ip_gma + cmd_length(s) * sizeof(uint32_t);
	}

	if (batch_buffer_needs_scan(s)) {
		if (shadow_cmd_buffer) {
			rc = vgt_perform_bb_shadow(s);
			if (rc)
				return rc;
		}
		rc = ip_gma_set(s, get_gma_bb_from_cmd(s, 1));
		if (rc < 0)
			vgt_warn("invalid batch buffer addr, so skip scanning it\n");
	} else {
		struct vgt_statistics *stat = &s->vgt->stat;

		stat->skip_bb_cnt++;
		/* emulate a batch buffer end to do return right */
		rc = vgt_cmd_handler_mi_batch_buffer_end(s);
		if (rc < 0)
			vgt_err("skip batch buffer error\n");
	}

	return rc;
}

static struct cmd_info cmd_info[] = {
	{"MI_NOOP", OP_MI_NOOP, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_SET_PREDICATE", OP_MI_SET_PREDICATE, F_LEN_CONST, R_ALL, D_HSW_PLUS,
		0, 1, NULL},

	{"MI_USER_INTERRUPT", OP_MI_USER_INTERRUPT, F_LEN_CONST, R_ALL, D_ALL, 0, 1, vgt_cmd_handler_mi_user_interrupt},

	{"MI_WAIT_FOR_EVENT", OP_MI_WAIT_FOR_EVENT, F_LEN_CONST | F_POST_HANDLE, R_RCS | R_BCS,
		D_ALL, 0, 1, vgt_handle_mi_wait_for_event},

	{"MI_FLUSH", OP_MI_FLUSH, F_LEN_CONST, R_RCS, D_PRE_BDW, 0, 1, NULL},

	{"MI_ARB_CHECK", OP_MI_ARB_CHECK, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_RS_CONTROL", OP_MI_RS_CONTROL, F_LEN_CONST, R_RCS, D_HSW_PLUS, 0, 1, NULL},

	{"MI_REPORT_HEAD", OP_MI_REPORT_HEAD, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_ARB_ON_OFF", OP_MI_ARB_ON_OFF, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_URB_ATOMIC_ALLOC", OP_MI_URB_ATOMIC_ALLOC, F_LEN_CONST, R_RCS,
		D_HSW_PLUS, 0, 1, NULL},

	{"MI_BATCH_BUFFER_END", OP_MI_BATCH_BUFFER_END, F_IP_ADVANCE_CUSTOM|F_LEN_CONST,
		R_ALL, D_ALL, 0, 1, vgt_cmd_handler_mi_batch_buffer_end},

	{"MI_SUSPEND_FLUSH", OP_MI_SUSPEND_FLUSH, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_PREDICATE", OP_MI_PREDICATE, F_LEN_CONST, R_RCS, D_IVB_PLUS, 0, 1, NULL},

	{"MI_TOPOLOGY_FILTER", OP_MI_TOPOLOGY_FILTER, F_LEN_CONST, R_ALL,
		D_IVB_PLUS, 0, 1, NULL},

	{"MI_SET_APPID", OP_MI_SET_APPID, F_LEN_CONST, R_ALL, D_IVB_PLUS, 0, 1, NULL},

	{"MI_RS_CONTEXT", OP_MI_RS_CONTEXT, F_LEN_CONST, R_RCS, D_HSW_PLUS, 0, 1, NULL},

	{"MI_DISPLAY_FLIP", OP_MI_DISPLAY_FLIP, F_LEN_VAR|F_POST_HANDLE, R_RCS | R_BCS,
		D_ALL, 0, 8, vgt_cmd_handler_mi_display_flip},

	{"MI_SEMAPHORE_MBOX", OP_MI_SEMAPHORE_MBOX, F_LEN_VAR, R_ALL, D_ALL, 0, 8, NULL },

	{"MI_SET_CONTEXT", OP_MI_SET_CONTEXT, F_LEN_VAR, R_ALL, D_ALL,
		0, 8, vgt_cmd_handler_mi_set_context},

	{"MI_MATH", OP_MI_MATH, F_LEN_VAR, R_ALL, D_ALL, 0, 8, NULL},

	{"MI_URB_CLEAR", OP_MI_URB_CLEAR, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"ME_SEMAPHORE_SIGNAL", OP_MI_SEMAPHORE_SIGNAL, F_LEN_VAR, R_ALL, D_BDW_PLUS, 0, 8, NULL},

	{"ME_SEMAPHORE_WAIT", OP_MI_SEMAPHORE_WAIT, F_LEN_VAR, R_ALL, D_BDW_PLUS,
		ADDR_FIX_1(2), 8, vgt_cmd_handler_mi_semaphore_wait},

	{"MI_STORE_DWORD_IMM", OP_MI_STORE_DWORD_IMM, F_LEN_VAR, R_ALL, D_HSW,
		ADDR_FIX_1(2), 10, NULL},

	{"MI_STORE_DWORD_IMM", OP_MI_STORE_DWORD_IMM, F_LEN_VAR, R_ALL, D_BDW_PLUS,
		ADDR_FIX_1(1), 10, vgt_cmd_handler_mi_store_data_imm},

	{"MI_STORE_DATA_INDEX", OP_MI_STORE_DATA_INDEX, F_LEN_VAR, R_ALL, D_ALL,
		0, 8, vgt_cmd_handler_mi_store_data_index},

	{"MI_LOAD_REGISTER_IMM", OP_MI_LRI_CMD, F_LEN_VAR, R_ALL, D_ALL, 0, 8, vgt_cmd_handler_lri},

	{"MI_UPDATE_GTT", OP_MI_UPDATE_GTT, F_LEN_VAR, R_RCS, D_PRE_BDW,
		0, 8, vgt_cmd_handler_mi_update_gtt},

	{"MI_UPDATE_GTT", OP_MI_UPDATE_GTT, F_LEN_VAR, (R_VCS | R_BCS | R_VECS), D_PRE_BDW,
		0, 6, vgt_cmd_handler_mi_update_gtt},

	{"MI_UPDATE_GTT", OP_MI_UPDATE_GTT, F_LEN_VAR, R_ALL, D_BDW_PLUS,
		0, 10, vgt_cmd_handler_mi_update_gtt},

	{"MI_STORE_REGISTER_MEM", OP_MI_STORE_REGISTER_MEM, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(2), 8, vgt_cmd_handler_srm},

	{"MI_FLUSH_DW", OP_MI_FLUSH_DW, F_LEN_VAR, R_ALL, D_ALL,
		0, 6, vgt_cmd_handler_mi_flush_dw},

	{"MI_CLFLUSH", OP_MI_CLFLUSH, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(1), 10, vgt_cmd_handler_mi_clflush},

	{"MI_REPORT_PERF_COUNT", OP_MI_REPORT_PERF_COUNT, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(1), 6, vgt_cmd_handler_mi_report_perf_count},

	{"MI_LOAD_REGISTER_MEM", OP_MI_LOAD_REGISTER_MEM, F_LEN_VAR, R_ALL, D_GEN7PLUS,
		ADDR_FIX_1(2), 8, vgt_cmd_handler_lrm},

	{"MI_LOAD_REGISTER_REG", OP_MI_LOAD_REGISTER_REG, F_LEN_VAR, R_ALL, D_HSW_PLUS,
		0, 8, vgt_cmd_handler_lrr},

	{"MI_RS_STORE_DATA_IMM", OP_MI_RS_STORE_DATA_IMM, F_LEN_VAR, R_RCS, D_HSW_PLUS,
		0, 8, NULL},

	{"MI_LOAD_URB_MEM", OP_MI_LOAD_URB_MEM, F_LEN_VAR, R_RCS, D_HSW_PLUS,
		ADDR_FIX_1(2), 8, NULL},

	{"MI_STORE_URM_MEM", OP_MI_STORE_URM_MEM, F_LEN_VAR, R_RCS, D_HSW_PLUS,
		ADDR_FIX_1(2), 8, NULL},

	{"MI_OP_2E", OP_MI_2E, F_LEN_VAR, R_ALL, D_BDW_PLUS, ADDR_FIX_2(1, 2), 8, vgt_cmd_handler_mi_op_2e},

	{"MI_OP_2F", OP_MI_2F, F_LEN_VAR, R_ALL, D_BDW_PLUS, ADDR_FIX_1(1), 8, vgt_cmd_handler_mi_op_2f},

	{"MI_BATCH_BUFFER_START", OP_MI_BATCH_BUFFER_START, F_IP_ADVANCE_CUSTOM,
		R_ALL, D_ALL, 0, 8, vgt_cmd_handler_mi_batch_buffer_start},

	{"MI_CONDITIONAL_BATCH_BUFFER_END", OP_MI_CONDITIONAL_BATCH_BUFFER_END,
		F_LEN_VAR, R_ALL, D_ALL, ADDR_FIX_1(2), 8, vgt_cmd_handler_mi_conditional_batch_buffer_end},

	{"MI_LOAD_SCAN_LINES_INCL", OP_MI_LOAD_SCAN_LINES_INCL, F_LEN_CONST, R_RCS | R_BCS, D_HSW_PLUS,
		0, 2, NULL},

	{"XY_SETUP_BLT", OP_XY_SETUP_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_2(4, 7), 8, NULL},

	{"XY_SETUP_CLIP_BLT", OP_XY_SETUP_CLIP_BLT, F_LEN_VAR, R_BCS, D_ALL,
		0, 8, NULL},

	{"XY_SETUP_MONO_PATTERN_SL_BLT", OP_XY_SETUP_MONO_PATTERN_SL_BLT, F_LEN_VAR,
		R_BCS, D_ALL, ADDR_FIX_1(4), 8, NULL},

	{"XY_PIXEL_BLT", OP_XY_PIXEL_BLT, F_LEN_VAR, R_BCS, D_ALL, 0, 8, NULL},

	{"XY_SCANLINES_BLT", OP_XY_SCANLINES_BLT, F_LEN_VAR, R_BCS, D_ALL,
		0, 8, NULL},

	{"XY_TEXT_BLT", OP_XY_TEXT_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_1(3), 8, NULL},

	{"XY_TEXT_IMMEDIATE_BLT", OP_XY_TEXT_IMMEDIATE_BLT, F_LEN_VAR, R_BCS,
		D_ALL, 0, 8, NULL},

	{"COLOR_BLT", OP_COLOR_BLT, F_LEN_VAR, R_BCS, D_PRE_BDW, ADDR_FIX_1(3), 6, NULL},

	{"SRC_COPY_BLT", OP_SRC_COPY_BLT, F_LEN_VAR, R_BCS, D_PRE_BDW,
		ADDR_FIX_1(3), 6, NULL},

	{"XY_COLOR_BLT", OP_XY_COLOR_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_1(4), 8, NULL},

	{"XY_PAT_BLT", OP_XY_PAT_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_2(4, 5), 8, NULL},

	{"XY_MONO_PAT_BLT", OP_XY_MONO_PAT_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_1(4), 8, NULL},

	{"XY_SRC_COPY_BLT", OP_XY_SRC_COPY_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_2(4, 7), 8, NULL},

	{"XY_MONO_SRC_COPY_BLT", OP_XY_MONO_SRC_COPY_BLT, F_LEN_VAR, R_BCS,
		D_ALL, ADDR_FIX_2(4, 5), 8, NULL},

	{"XY_FULL_BLT", OP_XY_FULL_BLT, F_LEN_VAR, R_BCS, D_ALL, 0, 8, NULL},

	{"XY_FULL_MONO_SRC_BLT", OP_XY_FULL_MONO_SRC_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_3(4, 5, 8), 8, NULL},

	{"XY_FULL_MONO_PATTERN_BLT", OP_XY_FULL_MONO_PATTERN_BLT, F_LEN_VAR,
		R_BCS, D_ALL, ADDR_FIX_2(4, 7), 8, NULL},

	{"XY_FULL_MONO_PATTERN_MONO_SRC_BLT", OP_XY_FULL_MONO_PATTERN_MONO_SRC_BLT,
		F_LEN_VAR, R_BCS, D_ALL, ADDR_FIX_2(4, 5), 8, NULL},

	{"XY_MONO_PAT_FIXED_BLT", OP_XY_MONO_PAT_FIXED_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_1(4), 8, NULL},

	{"XY_MONO_SRC_COPY_IMMEDIATE_BLT", OP_XY_MONO_SRC_COPY_IMMEDIATE_BLT,
		F_LEN_VAR, R_BCS, D_ALL, ADDR_FIX_1(4), 8, NULL},

	{"XY_PAT_BLT_IMMEDIATE", OP_XY_PAT_BLT_IMMEDIATE, F_LEN_VAR, R_BCS,
		D_ALL, ADDR_FIX_1(4), 8, NULL},

	{"XY_SRC_COPY_CHROMA_BLT", OP_XY_SRC_COPY_CHROMA_BLT, F_LEN_VAR, R_BCS,
		D_ALL, ADDR_FIX_2(4, 7), 8, NULL},

	{"XY_FULL_IMMEDIATE_PATTERN_BLT", OP_XY_FULL_IMMEDIATE_PATTERN_BLT,
		F_LEN_VAR, R_BCS, D_ALL, ADDR_FIX_2(4, 7), 8, NULL},

	{"XY_FULL_MONO_SRC_IMMEDIATE_PATTERN_BLT", OP_XY_FULL_MONO_SRC_IMMEDIATE_PATTERN_BLT,
		F_LEN_VAR, R_BCS, D_ALL, ADDR_FIX_2(4, 5), 8, NULL},

	{"XY_PAT_CHROMA_BLT", OP_XY_PAT_CHROMA_BLT, F_LEN_VAR, R_BCS, D_ALL,
		ADDR_FIX_2(4, 5), 8, NULL},

	{"XY_PAT_CHROMA_BLT_IMMEDIATE", OP_XY_PAT_CHROMA_BLT_IMMEDIATE, F_LEN_VAR,
		R_BCS, D_ALL, ADDR_FIX_1(4), 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS", OP_3DSTATE_BINDING_TABLE_POINTERS,
		F_LEN_VAR, R_RCS, D_SNB, 0, 8, NULL},

	{"3DSTATE_VIEWPORT_STATE_POINTERS_SF_CLIP", OP_3DSTATE_VIEWPORT_STATE_POINTERS_SF_CLIP,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_VIEWPORT_STATE_POINTERS_CC", OP_3DSTATE_VIEWPORT_STATE_POINTERS_CC,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_BLEND_STATE_POINTERS", OP_3DSTATE_BLEND_STATE_POINTERS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_DEPTH_STENCIL_STATE_POINTERS", OP_3DSTATE_DEPTH_STENCIL_STATE_POINTERS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS_VS", OP_3DSTATE_BINDING_TABLE_POINTERS_VS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS_HS", OP_3DSTATE_BINDING_TABLE_POINTERS_HS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS_DS", OP_3DSTATE_BINDING_TABLE_POINTERS_DS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS_GS", OP_3DSTATE_BINDING_TABLE_POINTERS_GS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS_PS", OP_3DSTATE_BINDING_TABLE_POINTERS_PS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS_VS", OP_3DSTATE_SAMPLER_STATE_POINTERS_VS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS_HS", OP_3DSTATE_SAMPLER_STATE_POINTERS_HS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS_DS", OP_3DSTATE_SAMPLER_STATE_POINTERS_DS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS_GS", OP_3DSTATE_SAMPLER_STATE_POINTERS_GS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS_PS", OP_3DSTATE_SAMPLER_STATE_POINTERS_PS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_URB_VS", OP_3DSTATE_URB_VS, F_LEN_VAR, R_RCS, D_GEN7PLUS,
		0, 8, NULL},

	{"3DSTATE_URB_HS", OP_3DSTATE_URB_HS, F_LEN_VAR, R_RCS, D_GEN7PLUS,
		0, 8, NULL},

	{"3DSTATE_URB_DS", OP_3DSTATE_URB_DS, F_LEN_VAR, R_RCS, D_GEN7PLUS,
		0, 8, NULL},

	{"3DSTATE_URB_GS", OP_3DSTATE_URB_GS, F_LEN_VAR, R_RCS, D_GEN7PLUS,
		0, 8, NULL},

	{"3DSTATE_GATHER_CONSTANT_VS", OP_3DSTATE_GATHER_CONSTANT_VS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_GATHER_CONSTANT_GS", OP_3DSTATE_GATHER_CONSTANT_GS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_GATHER_CONSTANT_HS", OP_3DSTATE_GATHER_CONSTANT_HS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_GATHER_CONSTANT_DS", OP_3DSTATE_GATHER_CONSTANT_DS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_GATHER_CONSTANT_PS", OP_3DSTATE_GATHER_CONSTANT_PS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_DX9_CONSTANTF_VS", OP_3DSTATE_DX9_CONSTANTF_VS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 11, NULL},

	{"3DSTATE_DX9_CONSTANTF_PS", OP_3DSTATE_DX9_CONSTANTF_PS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 11, NULL},

	{"3DSTATE_DX9_CONSTANTI_VS", OP_3DSTATE_DX9_CONSTANTI_VS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_DX9_CONSTANTI_PS", OP_3DSTATE_DX9_CONSTANTI_PS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_DX9_CONSTANTB_VS", OP_3DSTATE_DX9_CONSTANTB_VS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_DX9_CONSTANTB_PS", OP_3DSTATE_DX9_CONSTANTB_PS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_DX9_LOCAL_VALID_VS", OP_3DSTATE_DX9_LOCAL_VALID_VS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_DX9_LOCAL_VALID_PS", OP_3DSTATE_DX9_LOCAL_VALID_PS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_DX9_GENERATE_ACTIVE_VS", OP_3DSTATE_DX9_GENERATE_ACTIVE_VS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_DX9_GENERATE_ACTIVE_PS", OP_3DSTATE_DX9_GENERATE_ACTIVE_PS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_EDIT_VS", OP_3DSTATE_BINDING_TABLE_EDIT_VS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 9, NULL},

	{"3DSTATE_BINDING_TABLE_EDIT_GS", OP_3DSTATE_BINDING_TABLE_EDIT_GS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 9, NULL},

	{"3DSTATE_BINDING_TABLE_EDIT_HS", OP_3DSTATE_BINDING_TABLE_EDIT_HS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 9, NULL},

	{"3DSTATE_BINDING_TABLE_EDIT_DS", OP_3DSTATE_BINDING_TABLE_EDIT_DS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 9, NULL},

	{"3DSTATE_BINDING_TABLE_EDIT_PS", OP_3DSTATE_BINDING_TABLE_EDIT_PS,
		F_LEN_VAR, R_RCS, D_HSW_PLUS, 0, 9, NULL},

	{"3DSTATE_VF_INSTANCING", OP_3DSTATE_VF_INSTANCING, F_LEN_VAR, R_RCS, D_BDW_PLUS, 0, 8, NULL},

	{"3DSTATE_VF_SGVS", OP_3DSTATE_VF_SGVS, F_LEN_VAR, R_RCS, D_BDW_PLUS, 0, 8, NULL},

	{"3DSTATE_VF_TOPOLOGY", OP_3DSTATE_VF_TOPOLOGY, F_LEN_VAR, R_RCS, D_BDW_PLUS, 0, 8, NULL},

	{"3DSTATE_WM_CHROMAKEY", OP_3DSTATE_WM_CHROMAKEY, F_LEN_VAR, R_RCS, D_BDW_PLUS, 0, 8, NULL},

	{"3DSTATE_PS_BLEND", OP_3DSTATE_PS_BLEND, F_LEN_VAR, R_RCS, D_BDW_PLUS, 0, 8, NULL},

	{"3DSTATE_WM_DEPTH_STENCIL", OP_3DSTATE_WM_DEPTH_STENCIL, F_LEN_VAR, R_RCS, D_BDW_PLUS, 0, 8, NULL},

	{"3DSTATE_PS_EXTRA", OP_3DSTATE_PS_EXTRA, F_LEN_VAR, R_RCS, D_BDW_PLUS, 0, 8, NULL},

	{"3DSTATE_RASTER", OP_3DSTATE_RASTER, F_LEN_VAR, R_RCS, D_BDW_PLUS, 0, 8, NULL},

	{"3DSTATE_SBE_SWIZ", OP_3DSTATE_SBE_SWIZ, F_LEN_VAR, R_RCS, D_BDW_PLUS, 0, 8, NULL},

	{"3DSTATE_WM_HZ_OP", OP_3DSTATE_WM_HZ_OP, F_LEN_VAR, R_RCS, D_BDW_PLUS, 0, 8, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS", OP_3DSTATE_SAMPLER_STATE_POINTERS,
		F_LEN_VAR, R_RCS, D_SNB, 0, 8, NULL},

	{"3DSTATE_URB", OP_3DSTATE_URB, F_LEN_VAR, R_RCS, D_SNB, 0, 8, NULL},

	{"3DSTATE_VERTEX_BUFFERS", OP_3DSTATE_VERTEX_BUFFERS, F_LEN_VAR, R_RCS,
		D_PRE_BDW, 0, 8, NULL},

	{"3DSTATE_VERTEX_BUFFERS", OP_3DSTATE_VERTEX_BUFFERS, F_LEN_VAR, R_RCS,
		D_BDW_PLUS, 0, 8, NULL},

	{"3DSTATE_VERTEX_ELEMENTS", OP_3DSTATE_VERTEX_ELEMENTS, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_INDEX_BUFFER", OP_3DSTATE_INDEX_BUFFER, F_LEN_VAR, R_RCS,
		D_PRE_BDW, 0, 8, NULL},

	{"3DSTATE_INDEX_BUFFER", OP_3DSTATE_INDEX_BUFFER, F_LEN_VAR, R_RCS,
		D_BDW_PLUS, ADDR_FIX_1(2), 8, NULL},

	{"3DSTATE_VF_STATISTICS", OP_3DSTATE_VF_STATISTICS, F_LEN_CONST,
		R_RCS, D_ALL, 0, 1, NULL},

	{"3DSTATE_VF", OP_3DSTATE_VF, F_LEN_VAR, R_RCS, D_GEN75PLUS, 0, 8, NULL},

	{"3DSTATE_VIEWPORT_STATE_POINTERS", OP_3DSTATE_VIEWPORT_STATE_POINTERS,
		F_LEN_VAR, R_RCS, D_SNB, 0, 8, NULL},

	{"3DSTATE_CC_STATE_POINTERS", OP_3DSTATE_CC_STATE_POINTERS, F_LEN_VAR,
		R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_SCISSOR_STATE_POINTERS", OP_3DSTATE_SCISSOR_STATE_POINTERS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_GS", OP_3DSTATE_GS, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_CLIP", OP_3DSTATE_CLIP, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_WM", OP_3DSTATE_WM, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_CONSTANT_GS", OP_3DSTATE_CONSTANT_GS, F_LEN_VAR, R_RCS,
		D_PRE_BDW, 0, 8, NULL},

	{"3DSTATE_CONSTANT_GS", OP_3DSTATE_CONSTANT_GS, F_LEN_VAR, R_RCS,
		D_BDW_PLUS, 0, 8, NULL},

	{"3DSTATE_CONSTANT_PS", OP_3DSTATE_CONSTANT_PS, F_LEN_VAR, R_RCS,
		D_PRE_BDW, 0, 8, NULL},

	{"3DSTATE_CONSTANT_PS", OP_3DSTATE_CONSTANT_PS, F_LEN_VAR, R_RCS,
		D_BDW_PLUS, 0, 8, NULL},

	{"3DSTATE_SAMPLE_MASK", OP_3DSTATE_SAMPLE_MASK, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_CONSTANT_HS", OP_3DSTATE_CONSTANT_HS, F_LEN_VAR, R_RCS,
		D_IVB|D_HSW, 0, 8, NULL},

	{"3DSTATE_CONSTANT_HS", OP_3DSTATE_CONSTANT_HS, F_LEN_VAR, R_RCS,
		D_BDW_PLUS, 0, 8, NULL},

	{"3DSTATE_CONSTANT_DS", OP_3DSTATE_CONSTANT_DS, F_LEN_VAR, R_RCS,
		D_IVB|D_HSW, 0, 8, NULL},

	{"3DSTATE_CONSTANT_DS", OP_3DSTATE_CONSTANT_DS, F_LEN_VAR, R_RCS,
		D_BDW_PLUS, 0, 8, NULL},

	{"3DSTATE_HS", OP_3DSTATE_HS, F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_TE", OP_3DSTATE_TE, F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_DS", OP_3DSTATE_DS, F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_STREAMOUT", OP_3DSTATE_STREAMOUT, F_LEN_VAR, R_RCS,
		D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_SBE", OP_3DSTATE_SBE, F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_PS", OP_3DSTATE_PS, F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_DRAWING_RECTANGLE", OP_3DSTATE_DRAWING_RECTANGLE, F_LEN_VAR,
		R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_SAMPLER_PALETTE_LOAD0", OP_3DSTATE_SAMPLER_PALETTE_LOAD0,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_CHROMA_KEY", OP_3DSTATE_CHROMA_KEY, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, NULL},

	{"3DSTATE_DEPTH_BUFFER", OP_SNB_3DSTATE_DEPTH_BUFFER, F_LEN_VAR, R_RCS,
		D_SNB, ADDR_FIX_1(2), 8, NULL},

	{"3DSTATE_DEPTH_BUFFER", OP_3DSTATE_DEPTH_BUFFER, F_LEN_VAR, R_RCS,
		D_GEN7PLUS, ADDR_FIX_1(2), 8, NULL},

	{"3DSTATE_POLY_STIPPLE_OFFSET", OP_3DSTATE_POLY_STIPPLE_OFFSET,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_POLY_STIPPLE_PATTERN", OP_3DSTATE_POLY_STIPPLE_PATTERN,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_LINE_STIPPLE", OP_3DSTATE_LINE_STIPPLE, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_AA_LINE_PARAMS", OP_3DSTATE_AA_LINE_PARAMS, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_GS_SVB_INDEX", OP_3DSTATE_GS_SVB_INDEX, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, NULL},

	{"3DSTATE_SAMPLER_PALETTE_LOAD1", OP_3DSTATE_SAMPLER_PALETTE_LOAD1,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_MULTISAMPLE", OP_3DSTATE_MULTISAMPLE, F_LEN_VAR, R_RCS, D_PRE_BDW,
		0, 8, NULL},

	{"3DSTATE_MULTISAMPLE", OP_3DSTATE_MULTISAMPLE_BDW, F_LEN_VAR, R_RCS, D_BDW_PLUS,
		0, 8, NULL},

	{"3DSTATE_RAST_MULTISAMPLE", OP_3DSTATE_RAST_MULTISAMPLE, F_LEN_VAR, R_RCS,
		D_HSW, 0, 8, NULL},

	{"3DSTATE_STENCIL_BUFFER", OP_SNB_3DSTATE_STENCIL_BUFFER, F_LEN_VAR, R_RCS,
		D_SNB, ADDR_FIX_1(2), 8, NULL},

	{"3DSTATE_STENCIL_BUFFER", OP_3DSTATE_STENCIL_BUFFER, F_LEN_VAR, R_RCS,
		D_GEN7PLUS, ADDR_FIX_1(2), 8, NULL},

	{"3DSTATE_HIER_DEPTH_BUFFER", OP_SNB_3DSTATE_HIER_DEPTH_BUFFER, F_LEN_VAR,
		R_RCS, D_SNB, 0, 8, NULL},

	{"3DSTATE_HIER_DEPTH_BUFFER", OP_3DSTATE_HIER_DEPTH_BUFFER, F_LEN_VAR,
		R_RCS, D_GEN7PLUS, ADDR_FIX_1(2), 8, NULL},

	{"3DSTATE_CLEAR_PARAMS", OP_SNB_3DSTATE_CLEAR_PARAMS, F_LEN_VAR, R_RCS, D_SNB,
		0, 8, NULL},

	{"3DSTATE_CLEAR_PARAMS", OP_3DSTATE_CLEAR_PARAMS, F_LEN_VAR,
		R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_PUSH_CONSTANT_ALLOC_VS", OP_3DSTATE_PUSH_CONSTANT_ALLOC_VS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_PUSH_CONSTANT_ALLOC_HS", OP_3DSTATE_PUSH_CONSTANT_ALLOC_HS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_PUSH_CONSTANT_ALLOC_DS", OP_3DSTATE_PUSH_CONSTANT_ALLOC_DS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_PUSH_CONSTANT_ALLOC_GS", OP_3DSTATE_PUSH_CONSTANT_ALLOC_GS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_PUSH_CONSTANT_ALLOC_PS", OP_3DSTATE_PUSH_CONSTANT_ALLOC_PS,
		F_LEN_VAR, R_RCS, D_GEN7PLUS, 0, 8, NULL},

	{"3DSTATE_MONOFILTER_SIZE", OP_3DSTATE_MONOFILTER_SIZE, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_SO_DECL_LIST", OP_3DSTATE_SO_DECL_LIST, F_LEN_VAR, R_RCS, D_ALL,
		0, 9, NULL},

	{"3DSTATE_SO_BUFFER", OP_3DSTATE_SO_BUFFER, F_LEN_VAR, R_RCS, D_IVB|D_HSW,
		ADDR_FIX_2(2, 3), 8, NULL},

	{"3DSTATE_SO_BUFFER", OP_3DSTATE_SO_BUFFER, F_LEN_VAR, R_RCS, D_BDW_PLUS,
		ADDR_FIX_2(2, 4), 8, NULL},

	{"3DSTATE_BINDING_TABLE_POOL_ALLOC", OP_3DSTATE_BINDING_TABLE_POOL_ALLOC,
		F_LEN_VAR, R_RCS, D_HSW, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_POOL_ALLOC", OP_3DSTATE_BINDING_TABLE_POOL_ALLOC,
		F_LEN_VAR, R_RCS, D_BDW_PLUS, ADDR_FIX_1(1), 8, NULL},

	{"3DSTATE_GATHER_POOL_ALLOC", OP_3DSTATE_GATHER_POOL_ALLOC,
		F_LEN_VAR, R_RCS, D_HSW, 0, 8, NULL},

	{"3DSTATE_GATHER_POOL_ALLOC", OP_3DSTATE_GATHER_POOL_ALLOC,
		F_LEN_VAR, R_RCS, D_BDW_PLUS, ADDR_FIX_1(1), 8, NULL},

	{"3DSTATE_DX9_CONSTANT_BUFFER_POOL_ALLOC", OP_3DSTATE_DX9_CONSTANT_BUFFER_POOL_ALLOC,
		F_LEN_VAR, R_RCS, D_HSW, 0, 8, NULL},

	{"3DSTATE_DX9_CONSTANT_BUFFER_POOL_ALLOC", OP_3DSTATE_DX9_CONSTANT_BUFFER_POOL_ALLOC,
		F_LEN_VAR, R_RCS, D_BDW_PLUS, ADDR_FIX_1(1), 8, NULL},

	{"3DSTATE_SAMPLE_PATTERN", OP_3DSTATE_SAMPLE_PATTERN, F_LEN_VAR, R_RCS, D_BDW_PLUS, 0, 8, NULL},

	{"PIPE_CONTROL", OP_PIPE_CONTROL, F_LEN_VAR, R_RCS, D_ALL,
		ADDR_FIX_1(2), 8, vgt_cmd_handler_pipe_control},

	{"3DPRIMITIVE", OP_3DPRIMITIVE, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"PIPELINE_SELECT", OP_PIPELINE_SELECT, F_LEN_CONST, R_RCS, D_ALL, 0, 1, NULL},

	{"STATE_PREFETCH", OP_STATE_PREFETCH, F_LEN_VAR, R_RCS, D_ALL,
		ADDR_FIX_1(1), 8, NULL},

	{"STATE_SIP", OP_STATE_SIP, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"STATE_BASE_ADDRESS", OP_STATE_BASE_ADDRESS, F_LEN_VAR, R_RCS, D_PRE_BDW,
		0, 8, NULL},

	{"STATE_BASE_ADDRESS", OP_STATE_BASE_ADDRESS, F_LEN_VAR, R_RCS, D_BDW_PLUS,
		ADDR_FIX_5(1, 3, 4, 5, 6), 8, NULL},

	{"OP_3D_MEDIA_0_1_4", OP_3D_MEDIA_0_1_4, F_LEN_VAR, R_RCS, D_HSW_PLUS,
		ADDR_FIX_1(1), 8, NULL},

	{"3DSTATE_VS", OP_3DSTATE_VS, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_SF", OP_3DSTATE_SF, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_CONSTANT_VS", OP_3DSTATE_CONSTANT_VS, F_LEN_VAR, R_RCS, D_PRE_BDW,
		0, 8, NULL},

	{"3DSTATE_CONSTANT_VS", OP_3DSTATE_CONSTANT_VS, F_LEN_VAR, R_RCS, D_BDW_PLUS,
		0, 8, NULL},

	{"3DSTATE_COMPONENT_PACKING", OP_3DSTATE_COMPONENT_PACKING, F_LEN_VAR, R_RCS,
		D_SKL_PLUS, 0, 8, NULL},

	{"MEDIA_INTERFACE_DESCRIPTOR_LOAD", OP_MEDIA_INTERFACE_DESCRIPTOR_LOAD,
		F_LEN_VAR, R_RCS, D_ALL, 0, 16, NULL},

	{"MEDIA_GATEWAY_STATE", OP_MEDIA_GATEWAY_STATE, F_LEN_VAR, R_RCS, D_ALL,
		0, 16, NULL},

	{"MEDIA_STATE_FLUSH", OP_MEDIA_STATE_FLUSH, F_LEN_VAR, R_RCS, D_ALL,
		0, 16, NULL},

	{"MEDIA_OBJECT", OP_MEDIA_OBJECT, F_LEN_VAR, R_RCS, D_ALL, 0, 16, NULL},

	{"MEDIA_CURBE_LOAD", OP_MEDIA_CURBE_LOAD, F_LEN_VAR, R_RCS, D_ALL,
		0, 16, NULL},

	{"MEDIA_OBJECT_PRT", OP_MEDIA_OBJECT_PRT, F_LEN_VAR, R_RCS, D_ALL,
		0, 16, NULL},

	{"MEDIA_OBJECT_WALKER", OP_MEDIA_OBJECT_WALKER, F_LEN_VAR, R_RCS, D_ALL,
		0, 16, NULL},

	{"GPGPU_WALKER", OP_GPGPU_WALKER, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, NULL},

	{"MEDIA_VFE_STATE", OP_MEDIA_VFE_STATE, F_LEN_VAR, R_RCS, D_ALL, 0, 16, NULL},

	{"3DSTATE_VF_STATISTICS_GM45", OP_3DSTATE_VF_STATISTICS_GM45, F_LEN_CONST,
		R_ALL, D_ALL, 0, 1, NULL},

	{"MFX_PIPE_MODE_SELECT", OP_MFX_PIPE_MODE_SELECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_SURFACE_STATE", OP_MFX_SURFACE_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_PIPE_BUF_ADDR_STATE", OP_MFX_PIPE_BUF_ADDR_STATE, F_LEN_VAR,
		R_VCS, D_PRE_BDW, 0, 12, NULL},

	{"MFX_PIPE_BUF_ADDR_STATE", OP_MFX_PIPE_BUF_ADDR_STATE, F_LEN_VAR,
		R_VCS, D_BDW_PLUS, 0, 12, NULL},

	{"MFX_IND_OBJ_BASE_ADDR_STATE", OP_MFX_IND_OBJ_BASE_ADDR_STATE, F_LEN_VAR,
		R_VCS, D_PRE_BDW, 0, 12, NULL},

	{"MFX_IND_OBJ_BASE_ADDR_STATE", OP_MFX_IND_OBJ_BASE_ADDR_STATE, F_LEN_VAR,
		R_VCS, D_BDW_PLUS, 0, 12, NULL},

	{"MFX_BSP_BUF_BASE_ADDR_STATE", OP_MFX_BSP_BUF_BASE_ADDR_STATE, F_LEN_VAR,
		R_VCS, D_PRE_BDW, ADDR_FIX_3(1, 2, 3), 12, NULL},

	{"MFX_BSP_BUF_BASE_ADDR_STATE", OP_MFX_BSP_BUF_BASE_ADDR_STATE, F_LEN_VAR,
		R_VCS, D_BDW_PLUS, ADDR_FIX_3(1, 3, 5), 12, NULL},

	{"OP_2_0_0_5", OP_2_0_0_5, F_LEN_VAR,
		R_VCS, D_PRE_BDW, ADDR_FIX_1(6), 12, NULL},

	{"OP_2_0_0_5", OP_2_0_0_5, F_LEN_VAR, R_VCS, D_BDW_PLUS, 0, 12, NULL},

	{"MFX_STATE_POINTER", OP_MFX_STATE_POINTER, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_QM_STATE", OP_MFX_QM_STATE, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"MFX_FQM_STATE", OP_MFX_FQM_STATE, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"MFX_PAK_INSERT_OBJECT", OP_MFX_PAK_INSERT_OBJECT, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"MFX_STITCH_OBJECT", OP_MFX_STITCH_OBJECT, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"MFD_IT_OBJECT", OP_MFD_IT_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_WAIT", OP_MFX_WAIT, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 6, NULL},

	{"MFX_AVC_IMG_STATE", OP_MFX_AVC_IMG_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_AVC_QM_STATE", OP_MFX_AVC_QM_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	/* to check: is "Direct MV Buffer Base Address" GMA ? */
	{"MFX_AVC_DIRECTMODE_STATE", OP_MFX_AVC_DIRECTMODE_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_AVC_SLICE_STATE", OP_MFX_AVC_SLICE_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_AVC_REF_IDX_STATE", OP_MFX_AVC_REF_IDX_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_AVC_WEIGHTOFFSET_STATE", OP_MFX_AVC_WEIGHTOFFSET_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFD_AVC_PICID_STATE", OP_MFD_AVC_PICID_STATE, F_LEN_VAR,
		R_VCS, D_GEN75PLUS, 0, 12, NULL},
	{"MFD_AVC_DPB_STATE", OP_MFD_AVC_DPB_STATE, F_LEN_VAR,
		R_VCS, D_IVB_PLUS, 0, 12, NULL},

	{"MFD_AVC_BSD_OBJECT", OP_MFD_AVC_BSD_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFD_AVC_SLICEADDR", OP_MFD_AVC_SLICEADDR, F_LEN_VAR,
		R_VCS, D_IVB_PLUS, ADDR_FIX_1(2), 12, NULL},

	{"MFC_AVC_FQM_STATE", OP_MFC_AVC_FQM_STATE, F_LEN_VAR,
		R_VCS, D_SNB, 0, 12, NULL},

	{"MFC_AVC_PAK_INSERT_OBJECT", OP_MFC_AVC_PAK_INSERT_OBJECT, F_LEN_VAR,
		R_VCS, D_SNB, 0, 12, NULL},

	{"MFC_AVC_PAK_OBJECT", OP_MFC_AVC_PAK_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_VC1_PIC_STATE", OP_MFX_VC1_PIC_STATE, F_LEN_VAR,
		R_VCS, D_SNB, 0, 12, NULL},

	{"MFX_VC1_PRED_PIPE_STATE", OP_MFX_VC1_PRED_PIPE_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_VC1_DIRECTMODE_STATE", OP_MFX_VC1_DIRECTMODE_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFD_VC1_SHORT_PIC_STATE", OP_MFD_VC1_SHORT_PIC_STATE, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"MFD_VC1_LONG_PIC_STATE", OP_MFD_VC1_LONG_PIC_STATE, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"MFD_VC1_BSD_OBJECT", OP_MFD_VC1_BSD_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFC_MPEG2_SLICEGROUP_STATE", OP_MFC_MPEG2_SLICEGROUP_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFC_MPEG2_PAK_OBJECT", OP_MFC_MPEG2_PAK_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_MPEG2_PIC_STATE", OP_MFX_MPEG2_PIC_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_MPEG2_QM_STATE", OP_MFX_MPEG2_QM_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFD_MPEG2_BSD_OBJECT", OP_MFD_MPEG2_BSD_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_2_6_0_0", OP_MFX_2_6_0_0, F_LEN_VAR, R_VCS, D_ALL,
		0, 16, NULL},

	{"MFX_2_6_0_9", OP_MFX_2_6_0_9, F_LEN_VAR, R_VCS, D_ALL, 0, 16, NULL},

	{"MFX_2_6_0_8", OP_MFX_2_6_0_8, F_LEN_VAR, R_VCS, D_ALL, 0, 16, NULL},

	{"MFX_JPEG_PIC_STATE", OP_MFX_JPEG_PIC_STATE, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"MFX_JPEG_HUFF_TABLE_STATE", OP_MFX_JPEG_HUFF_TABLE_STATE, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"MFD_JPEG_BSD_OBJECT", OP_MFD_JPEG_BSD_OBJECT, F_LEN_VAR,
		R_VCS, D_GEN7PLUS, 0, 12, NULL},

	{"VEBOX_STATE", OP_VEB_STATE, F_LEN_VAR, R_VECS, D_GEN75PLUS, 0, 12, NULL},

	{"VEBOX_SURFACE_STATE", OP_VEB_SURFACE_STATE, F_LEN_VAR, R_VECS, D_HSW_PLUS, 0, 12, NULL},

	{"VEB_DI_IECP", OP_VEB_DNDI_IECP_STATE, F_LEN_VAR, R_VECS, D_HSW, 0, 12, NULL},

	{"VEB_DI_IECP", OP_VEB_DNDI_IECP_STATE, F_LEN_VAR, R_VECS, D_BDW_PLUS, 0, 20, NULL},
};

static int cmd_hash_init(struct pgt_device *pdev)
{
	int i;
	struct vgt_cmd_entry *e;
	struct cmd_info	*info;
	unsigned int gen_type;

	gen_type = vgt_gen_dev_type(pdev);

	for (i = 0; i < ARRAY_SIZE(cmd_info); i++) {
		if (!(cmd_info[i].devices & gen_type)) {
			vgt_dbg(VGT_DBG_CMD, "CMD[%-30s] op[%04x] flag[%x] devs[%02x] rings[%02x] not registered\n",
					cmd_info[i].name, cmd_info[i].opcode, cmd_info[i].flag,
					cmd_info[i].devices, cmd_info[i].rings);
			continue;
		}

		e = kmalloc(sizeof(*e), GFP_KERNEL);
		if (e == NULL) {
			vgt_err("Insufficient memory in %s\n", __FUNCTION__);
			return -ENOMEM;
		}
		e->info = &cmd_info[i];

		info = vgt_find_cmd_entry_any_ring(e->info->opcode, e->info->rings);
		if (info) {
			vgt_err("%s %s duplicated\n", e->info->name, info->name);
			kfree(e);
			return -EINVAL;
		}

		INIT_HLIST_NODE(&e->hlist);
		vgt_add_cmd_entry(e);
		vgt_dbg(VGT_DBG_CMD, "CMD[%-30s] op[%04x] flag[%x] devs[%02x] rings[%02x] registered\n",
				e->info->name,e->info->opcode, e->info->flag, e->info->devices,
				e->info->rings);
	}
	return 0;
}

static void trace_cs_command(struct parser_exec_state *s, cycles_t cost_pre_cmd_handler, cycles_t cost_cmd_handler)
{
	/* This buffer is used by ftrace to store all commands copied from guest gma
	* space. Sometimes commands can cross pages, this should not be handled in
	 * ftrace logic. So this is just used as a 'bounce buffer' */
	u32 cmd_trace_buf[VGT_MAX_CMD_LENGTH];
	int i;
	u32 cmd_len = cmd_length(s);
	/* The chosen value of VGT_MAX_CMD_LENGTH are just based on
	 * following two considerations:
	 * 1) From observation, most common ring commands is not that long.
	 *    But there are execeptions. So it indeed makes sence to observe
	 *    longer commands.
	 * 2) From the performance and debugging point of view, dumping all
	 *    contents of very commands is not necessary.
	 * We mgith shrink VGT_MAX_CMD_LENGTH or remove this trace event in
	 * future for performance considerations.
	 */
	if (unlikely(cmd_len > VGT_MAX_CMD_LENGTH)) {
		vgt_dbg(VGT_DBG_CMD, "cmd length exceed tracing limitation!\n");
		cmd_len = VGT_MAX_CMD_LENGTH;
	}

	for (i = 0; i < cmd_len; i++)
		cmd_trace_buf[i] = cmd_val(s, i);

	trace_vgt_command(s->vgt->vm_id, s->ring_id, s->ip_gma, cmd_trace_buf,
			cmd_len, s->buf_type == RING_BUFFER_INSTRUCTION, cost_pre_cmd_handler, cost_cmd_handler);

}

/* call the cmd handler, and advance ip */
static int vgt_cmd_parser_exec(struct parser_exec_state *s)
{
	struct cmd_info *info;
	uint32_t cmd;
	int rc = 0;
	cycles_t t0, t1, t2;

	t0 = get_cycles();

	cmd = cmd_val(s, 0);

	info = vgt_get_cmd_info(cmd, s->ring_id);
	if (info == NULL) {
		vgt_err("ERROR: unknown cmd 0x%x, ring%d[%lx, %lx] gma[%lx] va[%p] opcode=0x%x\n", 
				cmd, s->ring_id, s->ring_start,
				s->ring_start + s->ring_size, s->ip_gma, s->ip_va,
				vgt_get_opcode(cmd, s->ring_id));
		parser_exec_state_dump(s);

		return -EFAULT;
	}

	s->info = info;

	t1 = get_cycles();

	if (info->handler) {
		if (info->flag & F_POST_HANDLE)
			rc = add_post_handle_entry(s, info->handler);
		else
			rc = info->handler(s);

		if (rc < 0) {
			vgt_err("%s handler error", info->name);
			parser_exec_state_dump(s);
			return rc;
		}
	}

	t2 = get_cycles();

	trace_cs_command(s, t1 - t0, t2 -t1);

	if (!(info->flag & F_IP_ADVANCE_CUSTOM)) {
		rc = vgt_cmd_advance_default(s);
		if (rc < 0) {
			vgt_err("%s IP advance error", info->name);
			return rc;
		}
	}

	return rc;
}

static inline bool gma_out_of_range(unsigned long gma, unsigned long gma_head, unsigned gma_tail)
{
	if ( gma_tail >= gma_head)
		return (gma < gma_head) || (gma > gma_tail);
	else
		return (gma > gma_tail) && (gma < gma_head);

}

#define MAX_PARSER_ERROR_NUM	10

static int __vgt_scan_vring(struct vgt_device *vgt, int ring_id, vgt_reg_t head,
			vgt_reg_t tail, vgt_reg_t base, vgt_reg_t size, cmd_shadow_t shadow)
{
	unsigned long gma_head, gma_tail, gma_bottom;
	struct parser_exec_state s;
	int rc=0;
	uint64_t cmd_nr = 0;
	vgt_state_ring_t *rs = &vgt->rb[ring_id];
	u32 cmd_flags = 0;
	unsigned long ip_gma = 0;

	/* ring base is page aligned */
	ASSERT((base & (PAGE_SIZE-1)) == 0);

	gma_head = base + head;
	gma_tail = base + tail;
	gma_bottom = base + size;

	s.buf_type = RING_BUFFER_INSTRUCTION;
	s.buf_addr_type = GTT_BUFFER;
	s.vgt = vgt;
	s.ring_id = ring_id;
	s.ring_start = base;
	s.ring_size = size;
	s.ring_head = gma_head;
	s.ring_tail = gma_tail;

	s.request_id = rs->request_id;
	s.el_ctx = rs->el_ctx;
	s.shadow = shadow;

	if (bypass_scan_mask & (1 << ring_id)) {
		add_tail_entry(&s, tail, 100, 0, 0);
		return 0;
	}

	if (!shadow) {
		s.ip_buf = kmalloc(PAGE_SIZE * 2, GFP_ATOMIC);
		if (!s.ip_buf) {
			vgt_err("fail to allocate buffer page.\n");
			return -ENOMEM;
		}
	} else {
		s.ip_buf = NULL;
	}

	rc = ip_gma_set(&s, base + head);
	if (rc < 0)
		goto out;

	vgt_dbg(VGT_DBG_CMD, "ring buffer scan start on ring %d: start=%lx end=%lx\n", 
		ring_id, gma_head, gma_tail);
	while(s.ip_gma != gma_tail){
		s.cmd_issue_irq = false;
		if (s.buf_type == RING_BUFFER_INSTRUCTION){
			if (!(s.ip_gma >= base) || !(s.ip_gma < gma_bottom)) {
				vgt_err("VM(%d) vgt_scan_vring: GMA(%lx)'s out of range\n",
						vgt->vgt_id, s.ip_gma);
				rc = -EINVAL;
				goto out;
			};
			if (gma_out_of_range(s.ip_gma, gma_head, gma_tail)) {
				vgt_err("ERROR: ip_gma %lx out of range."
					"(base:0x%x, head: 0x%x, tail: 0x%x)\n",
					s.ip_gma, base, head, tail);
				break;
			}
		}

		cmd_nr++;

		rc = vgt_cmd_parser_exec(&s);
		if (rc < 0) {
			vgt_err("cmd parser error\n");
			break;
		}

		if (irq_based_ctx_switch &&
			(s.buf_type == RING_BUFFER_INSTRUCTION)) {
		/* record the status of irq instruction in ring buffer */
			if (s.cmd_issue_irq) {
				cmd_flags |= F_CMDS_ISSUE_IRQ;
				ip_gma = s.ip_gma;
			}
		}
	}

	if (!rc && shadow != INDIRECT_CTX_SHADOW) {
		/*
		 * Set flag to indicate the command buffer is end with user interrupt,
		 * and save the instruction's offset in ring buffer.
		 */
		if (cmd_flags & F_CMDS_ISSUE_IRQ)
			ip_gma = ip_gma - base;
		add_tail_entry(&s, tail, cmd_nr, cmd_flags, ip_gma);
		rs->cmd_nr++;
	}

	vgt_dbg(VGT_DBG_CMD, "ring buffer scan end on ring %d\n", ring_id);
out:
	if (s.ip_buf)
		kfree(s.ip_buf);
	return rc;
}

static int vgt_copy_rb_to_shadow(struct vgt_device *vgt,
				  struct execlist_context *el_ctx,
				  uint32_t head,
				  uint32_t tail)
{
	uint32_t rb_size;
	uint32_t left_len;
	uint32_t rb_offset;
	unsigned long vbase, sbase;

	rb_size = el_ctx->shadow_rb.ring_size;
	vbase = el_ctx->shadow_rb.guest_rb_base;
	sbase = el_ctx->shadow_rb.shadow_rb_base;

	trace_shadow_rb_copy(vgt->vm_id, el_ctx->ring_id,
			     el_ctx->guest_context.lrca,
			     el_ctx->shadow_lrca,
			     rb_size, vbase, sbase, head, tail);

	if (head <= tail)
		left_len = tail - head;
	else
		left_len = rb_size - head + tail;

	rb_offset = head;

	while (left_len > 0) {
		void *ip_va, *ip_sva;
		uint32_t ip_buf_len;
		uint32_t copy_len;

		ip_va = vgt_gma_to_va(vgt->gtt.ggtt_mm, vbase + rb_offset);
		if (ip_va == NULL) {
			vgt_err("VM-%d(ring-%d): gma %lx is invalid!\n",
				vgt->vm_id, el_ctx->ring_id, vbase + rb_offset);
			dump_stack();
			return EFAULT;
		}

		ip_buf_len = PAGE_SIZE - ((vbase + rb_offset) & (PAGE_SIZE - 1));
		if (left_len <= ip_buf_len)
			copy_len = left_len;
		else
			copy_len = ip_buf_len;

		ip_sva = rsvd_gma_to_sys_va(vgt->pdev, sbase + rb_offset);
		hypervisor_read_va(vgt, ip_va, ip_sva, copy_len, 1);

		left_len -= copy_len;
		rb_offset = (rb_offset + copy_len) & (rb_size - 1);
	}

	return 0;
}

static int vgt_copy_indirect_ctx_to_shadow(struct vgt_device *vgt,
				  struct execlist_context *el_ctx)

{
	uint32_t left_len = el_ctx->shadow_indirect_ctx.ctx_size;
	unsigned long  vbase = el_ctx->shadow_indirect_ctx.guest_ctx_base;
	unsigned long  sbase = el_ctx->shadow_indirect_ctx.shadow_ctx_base;
	uint32_t ctx_offset = 0;
	void *ip_sva = NULL;

	if (!left_len)
		return 0;

	ASSERT(el_ctx->ring_id == 0);

	while (left_len > 0) {
		void *ip_va;
		uint32_t ip_buf_len;
		uint32_t copy_len;

		ip_va = vgt_gma_to_va(vgt->gtt.ggtt_mm, vbase + ctx_offset);
		if (ip_va == NULL) {
			vgt_err("VM-%d: gma %lx is invalid in indirect ctx!\n",
				vgt->vm_id, vbase + ctx_offset);
			dump_stack();
			return -EFAULT;
		}

		ip_buf_len = PAGE_SIZE - ((vbase + ctx_offset) & (PAGE_SIZE - 1));
		if (left_len <= ip_buf_len)
			copy_len = left_len;
		else
			copy_len = ip_buf_len;

		ip_sva = rsvd_gma_to_sys_va(vgt->pdev, sbase + ctx_offset);
		hypervisor_read_va(vgt, ip_va, ip_sva, copy_len, 1);

		left_len -= copy_len;
		ctx_offset = ctx_offset + copy_len;
	}

	return 0;
}

static int vgt_combine_indirect_ctx_bb(struct vgt_device *vgt,
				  struct execlist_context *el_ctx)
{
	unsigned long sbase = el_ctx->shadow_indirect_ctx.shadow_ctx_base;
	uint32_t ctx_size = el_ctx->shadow_indirect_ctx.ctx_size;
	void *bb_start_sva;
	uint32_t bb_per_ctx_start[CACHELINE_DWORDS] = {0};

	if (!el_ctx->shadow_bb_per_ctx.guest_bb_base) {
		vgt_err("invalid bb per ctx address\n");
		return -1;
	}

	bb_per_ctx_start[0] = 0x18800001;
	bb_per_ctx_start[1] = el_ctx->shadow_bb_per_ctx.guest_bb_base;
	bb_start_sva = rsvd_gma_to_sys_va(vgt->pdev, sbase + ctx_size);
	memcpy(bb_start_sva, bb_per_ctx_start, CACHELINE_BYTES);

	return 0;
}

static void vgt_get_bb_per_ctx_shadow_base(struct vgt_device *vgt,
				  struct execlist_context *el_ctx)
{
	unsigned long ctx_sbase = el_ctx->shadow_indirect_ctx.shadow_ctx_base;
	uint32_t ctx_size = el_ctx->shadow_indirect_ctx.ctx_size;
	void *va_bb = rsvd_gma_to_sys_va(vgt->pdev, ctx_sbase + ctx_size);

	el_ctx->shadow_bb_per_ctx.shadow_bb_base = *((unsigned int *)va_bb + 1);
	memset(va_bb, 0, CACHELINE_BYTES);
}

/*
 * Scan the guest ring.
 *   Return 0: success
 *         <0: Address violation.
 */
int vgt_scan_vring(struct vgt_device *vgt, int ring_id)
{
	vgt_state_ring_t *rs = &vgt->rb[ring_id];
	vgt_ringbuffer_t *vring = &rs->vring;
	int ret = 0;
	cycles_t t0, t1;
	uint32_t rb_base, ctx_base;
	struct vgt_statistics *stat = &vgt->stat;

	t0 = get_cycles();

	if (!(vring->ctl & _RING_CTL_ENABLE)) {
		/* Ring is enabled */
		vgt_dbg(VGT_DBG_CMD, "VGT-Parser.c vring is disabled. head %x tail %x ctl %x\n",
			vring->head, vring->tail, vring->ctl);
		if (IS_HSW(vgt->pdev))
			return 0;
		vgt_err("Unexpected ring %d disabled in context\n", ring_id);
		ret = -1;
		goto err;
	}

	stat->vring_scan_cnt++;
	rs->request_id++;

	rb_base = vring->start;

	/* copy the guest rb into shadow rb */
	if (shadow_cmd_buffer) {
		ret = vgt_copy_rb_to_shadow(vgt, rs->el_ctx,
				    rs->last_scan_head,
				    vring->tail & RB_TAIL_OFF_MASK);
		rb_base = rs->el_ctx->shadow_rb.shadow_rb_base;
	}

	if (ret == 0) {
		ret = __vgt_scan_vring(vgt, ring_id, rs->last_scan_head,
			vring->tail & RB_TAIL_OFF_MASK, rb_base,
			_RING_CTL_BUF_SIZE(vring->ctl),
			shadow_cmd_buffer ? NORMAL_CMD_SHADOW : NO_CMD_SHADOW);

		rs->last_scan_head = vring->tail;
	}

	if (ret)
		goto err;


	if (shadow_indirect_ctx_bb) {
		ret = vgt_copy_indirect_ctx_to_shadow(vgt, rs->el_ctx);
		ctx_base = rs->el_ctx->shadow_indirect_ctx.shadow_ctx_base;
		if (ret == 0 && ctx_base) {
			uint32_t ctx_tail =
				rs->el_ctx->shadow_indirect_ctx.ctx_size +
							3 * sizeof(uint32_t);
			uint32_t dummy_ctx_size =
				((ctx_tail >> PAGE_SHIFT) + 1) << PAGE_SHIFT;
			ASSERT(ctx_tail != dummy_ctx_size);
			ret = vgt_combine_indirect_ctx_bb(vgt, rs->el_ctx);
			if (ret)
				goto err;
			if (!__vgt_scan_vring(vgt, ring_id, 0, ctx_tail,
				ctx_base, dummy_ctx_size, INDIRECT_CTX_SHADOW)) {
				vgt_get_bb_per_ctx_shadow_base(vgt, rs->el_ctx);
			} else {
				ret = -1;
				vgt_err("error happen in indirect ctx scan\n");
			}
		}

	}

	t1 = get_cycles();
	stat->vring_scan_cycles += t1 - t0;

err:
	if (ret && vgt_cmd_audit) {
		vgt->force_removal = 1;
		vgt_kill_vm(vgt);
	}

	return ret;
}

int vgt_cmd_parser_init(struct pgt_device *pdev)
{
	return cmd_hash_init(pdev);
}

void vgt_cmd_parser_exit(void)
{
	vgt_clear_cmd_table();
}
