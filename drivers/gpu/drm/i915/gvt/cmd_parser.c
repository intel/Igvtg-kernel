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

#include <linux/slab.h>
#include "gvt.h"
#include "trace.h"

/* gvt uses below bits in NOOP_ID:
 *	    bit 21 - 16 is command type.
 *	    bit 15 - 0  holds command specific information.
 *
 * Assumption: Linux/Windows guest will not use bits 21 - bits 16 with
 * non-zero value.
 */
#define GVT_NOOP_ID_CMD_SHIFT	16
#define GVT_NOOP_ID_CMD_MASK	(0x3f << GVT_NOOP_ID_CMD_SHIFT)
#define CMD_LENGTH_MASK		0xff
#define gmadr_dw_number(s)	\
	(s->vgt->pdev->device_info.gmadr_bytes_in_cmd >> 2)
#define BIT_RANGE_MASK(a, b)	\
	((1UL << ((a) + 1)) - (1UL << (b)))

unsigned long bypass_scan_mask = 0;

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

static struct decode_info* ring_decode_info[I915_NUM_RINGS][8] = {
	[RCS] = {
		&decode_info_mi,
		NULL,
		NULL,
		&decode_info_3d_media,
		NULL,
		NULL,
		NULL,
		NULL,
	},

	[VCS] = {
		&decode_info_mi,
		NULL,
		NULL,
		&decode_info_mfx_vc,
		NULL,
		NULL,
		NULL,
		NULL,
	},

	[BCS] = {
		&decode_info_mi,
		NULL,
		&decode_info_2d,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
	},

	[VECS] = {
		&decode_info_mi,
		NULL,
		NULL,
		&decode_info_vebox,
		NULL,
		NULL,
		NULL,
		NULL,
	},

	[VCS2] = {
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

static inline u32 get_opcode(u32 cmd, int ring_id)
{
	struct decode_info * d_info;

	if (ring_id >= I915_NUM_RINGS)
		return INVALID_OP;

	d_info = ring_decode_info[ring_id][CMD_TYPE(cmd)];
	if (d_info == NULL)
		return INVALID_OP;

	return cmd >> (32 - d_info->op_len);
}

static inline struct cmd_info* find_cmd_entry(struct pgt_device *pdev,
		unsigned int opcode, int ring_id)
{
	struct gvt_cmd_entry *e;

	hash_for_each_possible(pdev->cmd_table, e, hlist, opcode) {
		if ((opcode == e->info->opcode) && (e->info->rings & (1<<ring_id)))
			return e->info;
	}
	return NULL;
}

static inline struct cmd_info* gvt_get_cmd_info(struct pgt_device *pdev,
		u32 cmd, int ring_id)
{
	u32 opcode;

	opcode = get_opcode(cmd, ring_id);
	if (opcode == INVALID_OP) {
		return NULL;
	}

	return find_cmd_entry(pdev, opcode, ring_id);
}

static inline u32 sub_op_val(u32 cmd, u32 hi, u32 low)
{
	return (cmd >> low) & ((1U << (hi-low+1)) - 1);
}

static inline void gvt_print_opcode(u32 cmd, int ring_id)
{
	struct decode_info * d_info;
	int i;

	if (ring_id >= I915_NUM_RINGS)
		return;

	d_info = ring_decode_info[ring_id][CMD_TYPE(cmd)];
	if (d_info == NULL)
		return;

	gvt_err("opcode=0x%x %s sub_ops:", cmd >> (32 - d_info->op_len), d_info->name);

	for (i = 0; i< d_info->nr_sub_op; i++)
		printk("0x%x ", sub_op_val(cmd, d_info->sub_op[i].hi,  d_info->sub_op[i].low));

	printk("\n");
}

static inline u32 *cmd_ptr(struct parser_exec_state *s, int index)
{
	return s->ip_va + (index << 2);
}

static inline u32 cmd_val(struct parser_exec_state *s, int index)
{
	return *cmd_ptr(s, index);
}

static void parser_exec_state_dump(struct parser_exec_state *s)
{
	gvt_err("  vgt%d RING%d: ring_start(%08lx) ring_end(%08lx)"
			" ring_head(%08lx) ring_tail(%08lx)", s->vgt->id,
			s->ring_id, s->ring_start, s->ring_start + s->ring_size, s->ring_head, s->ring_tail);

	gvt_err("  %s %s ip_gma(%08lx) ",
			s->buf_type == RING_BUFFER_INSTRUCTION ? "RING_BUFFER": "BATCH_BUFFER",
			s->buf_addr_type == GTT_BUFFER ? "GTT" : "PPGTT", s->ip_gma);

	if (s->ip_va == NULL) {
		gvt_err(" ip_va(NULL)");
	} else {
		int cnt = 0;
		gvt_err("  ip_va=%p: %08x %08x %08x %08x ",
				s->ip_va, cmd_val(s, 0), cmd_val(s, 1), cmd_val(s, 2), cmd_val(s, 3));

		gvt_print_opcode(cmd_val(s, 0), s->ring_id);

		/* print the whole page to trace */
		trace_printk("ERROR ip_va=%p: %08x %08x %08x %08x ",
				s->ip_va, cmd_val(s, 0), cmd_val(s, 1), cmd_val(s, 2), cmd_val(s, 3));

		s->ip_va = (u32*)((((u64)s->ip_va) >> 12) << 12);
		while(cnt < 1024) {
		trace_printk("DUMP ip_va=%p: %08x %08x %08x %08x %08x %08x %08x %08x ",
				s->ip_va, cmd_val(s, 0), cmd_val(s, 1), cmd_val(s, 2), cmd_val(s, 3),
				          cmd_val(s, 4), cmd_val(s, 5), cmd_val(s, 6), cmd_val(s, 7));

			s->ip_va+=8;
			cnt+=8;
		}

	}
}

static inline void update_ip_va(struct parser_exec_state *s)
{
	unsigned long len = 0;

	ASSERT(s->ring_head != s->ring_tail);

	if (s->buf_type == RING_BUFFER_INSTRUCTION) {
		unsigned long ring_top = s->ring_start + s->ring_size;
		if (s->ring_head > s->ring_tail) {
			if (s->ip_gma >= s->ring_head && ring_top)
				len = (s->ip_gma - s->ring_head);
			else if (s->ip_gma >= s->ring_start &&
					s->ip_gma <= s->ring_tail)
				len = (ring_top - s->ring_head) +
					(s->ip_gma - s->ring_start);
		} else
			len = (s->ip_gma - s->ring_head);

		s->ip_va = s->rb_va + len;
	} else {
		/* shadow batch buffer */
		ASSERT(0);
	}
}

static inline int ip_gma_set(struct parser_exec_state *s, unsigned long ip_gma)
{
	ASSERT(!(ip_gma & (4 - 1)));

	s->ip_gma = ip_gma;

	update_ip_va(s);

	return 0;
}

static inline int ip_gma_advance(struct parser_exec_state *s, unsigned int dw_len)
{
	int rc = 0;

	s->ip_gma += (dw_len << 2);

	if (s->buf_type == RING_BUFFER_INSTRUCTION) {
		/* cross page, reset ip_va */
		if (s->ip_gma >= s->ring_start + s->ring_size) {
			s->ip_gma -= s->ring_size;
		}
	}

	update_ip_va(s);

	return rc;
}

static inline int get_cmd_length(struct cmd_info *info, u32 cmd)
{
	/*
	 * MI_NOOP is special as the replacement elements. It's fixed
	 * length in definition, but variable length when using for
	 * replacement purpose. Instead of having the same handler
	 * invoke twice (may be postponed), special case length
	 * handling for MI_NOOP.
	 */
	if (info->opcode == OP_MI_NOOP) {
		unsigned int subop, length = info->len;
		subop = (cmd & GVT_NOOP_ID_CMD_MASK) >>
			GVT_NOOP_ID_CMD_SHIFT;
		if (subop)
			length = cmd & CMD_LENGTH_MASK;

		return length;
	} else if ((info->flag & F_LEN_MASK) == F_LEN_CONST) {
		return info->len;
	} else /* F_LEN_VAR */{
		return (cmd & ((1U << info->len) - 1)) + 2;
	}
}

static inline int cmd_length(struct parser_exec_state *s)
{
	return get_cmd_length(s->info, cmd_val(s, 0));
}

/* do not remove this, some platform may need clflush here */
#define patch_value(s, addr, val) do {\
	*addr = val; \
}while(0);

static bool addr_audit_32(struct parser_exec_state *s, int index)
{
	/* TODO:
	 * Add the address audit implementation here. Right now do nothing
	 */
	return true;
}

static inline bool address_audit(struct parser_exec_state *s, int index)
{
	int gmadr_bytes = s->vgt->pdev->device_info.gmadr_bytes_in_cmd;

	/* TODO:
	 * Add the address audit implementation here. Right now do nothing
	 */
	ASSERT(gmadr_bytes == 4 || gmadr_bytes == 8);

	return true;
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

static int cmd_handler_lri_de_rrmr(struct parser_exec_state *s)
{

	int i;
	int cmd_len = cmd_length(s);
	unsigned long offset;
	unsigned long val;

	for (i = 1; i < cmd_len; i += 2) {
		offset = cmd_val(s, i) & BIT_RANGE_MASK(22, 2);
		val = cmd_val(s, i + 1);

		if (offset == _DERRMR)
			break;
	}

	if (i == cmd_len) {
		gvt_err("No DE_RRMR in LRI?");
		return -EINVAL;
	}

	patch_value(s, cmd_ptr(s, i), 0);
	patch_value(s, cmd_ptr(s, i + 1), 0);
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

	if (!reg_is_mmio(pdev, offset + 4)){
		rc = -1;
		goto reg_handle;
	}

	if (is_shadowed_mmio(offset)) {
		gvt_warn("VM-%d: !!! Found access of shadowed MMIO<0x%x>!",
			 s->vgt->vm_id, offset);
	}

reg_handle:
	if (!rc)
		reg_set_cmd_access(pdev, offset);
	else {
		gvt_err("%s access to non-render register (%x)", cmd, offset);
	}

	return 0;
}

static int gvt_cmd_handler_lri(struct parser_exec_state *s)
{
	unsigned long offset;
	int i, rc = 0;
	int cmd_len = cmd_length(s);

	for (i = 1; i < cmd_len; i += 2) {
		offset = cmd_val(s, i) & BIT_RANGE_MASK(22, 2);
		rc |= cmd_reg_handler(s, offset, i, "lri");

		if (IS_BROADWELL(s->vgt->pdev->dev_priv) && offset == _DERRMR) {
			rc = cmd_handler_lri_de_rrmr(s);
			if (rc) {
				gvt_err("fail to handle lri de rrmr case");
				break;
			}
		}
	}

	return rc;
}

static int gvt_cmd_handler_lrr(struct parser_exec_state *s)
{
	int i, rc = 0;
	int cmd_len = cmd_length(s);

	for (i = 1; i < cmd_len; i += 2) {
		rc = cmd_reg_handler(s,
			cmd_val(s, i) & BIT_RANGE_MASK(22, 2), i, "lrr-src");
		rc |= cmd_reg_handler(s,
			cmd_val(s, i+1) & BIT_RANGE_MASK(22, 2), i, "lrr-dst");
	}

	return rc;
}

static int gvt_cmd_handler_lrm(struct parser_exec_state *s)
{
	int i, rc = 0;
	int cmd_len = cmd_length(s);

	for (i = 1; i < cmd_len;) {
		rc |= cmd_reg_handler(s,
			cmd_val(s, i) & BIT_RANGE_MASK(22, 2), i, "lrm");
		i += gmadr_dw_number(s) + 1;
	}

	return rc;
}

static int gvt_cmd_handler_srm(struct parser_exec_state *s)
{
	int i, rc = 0;
	int cmd_len = cmd_length(s);

	for (i = 1; i < cmd_len;) {
		rc |= cmd_reg_handler(s,
			cmd_val(s, i) & BIT_RANGE_MASK(22, 2), i, "srm");
		i += gmadr_dw_number(s) + 1;
	}

	return rc;
}

static int gvt_cmd_handler_pipe_control(struct parser_exec_state *s)
{
	int i, rc = 0;
	int cmd_len = cmd_length(s);


	for (i = 1; i < cmd_len;) {
		if (cmd_val(s, i) & PIPE_CONTROL_MMIO_WRITE)
			rc |= cmd_reg_handler(s,
				cmd_val(s, i+1) & BIT_RANGE_MASK(22, 2), i, "pipe_ctrl");
		else if (cmd_val(s, i) & (2 << 14))
			rc |= cmd_reg_handler(s, 0x2350, i, "pipe_ctrl");
		else if (cmd_val(s, i) & (3 << 14))
			rc |= cmd_reg_handler(s, _REG_RCS_TIMESTAMP, i, "pipe_ctrl");

		if (!rc) {
			if (cmd_val(s, i) & PIPE_CONTROL_NOTIFY) {
				set_bit(gvt_ring_id_to_pipe_control_notify_event(s->ring_id),
					s->workload->pending_events);
			}
		}
		i += gmadr_dw_number(s) + 3;
	}

	return rc;
}

static int gvt_cmd_handler_mi_user_interrupt(struct parser_exec_state *s)
{
	set_bit(gvt_ring_id_to_mi_user_interrupt_event(s->ring_id), s->workload->pending_events);
	return 0;
}

static int gvt_cmd_handler_mi_batch_buffer_end(struct parser_exec_state *s)
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

#define PLANE_SELECT_SHIFT	19
#define PLANE_SELECT_MASK	(0x7 << PLANE_SELECT_SHIFT)
#define SURF_MASK		0xFFFFF000
#define PITCH_MASK		0x0000FFC0
#define TILE_PARA_SHIFT		0x0
#define TILE_PARA_MASK		0x1
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

static bool display_flip_decode_plane_info(u32  plane_code, enum pipe *pipe, enum gvt_plane_type *plane )
{
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

#define GET_INFO_FOR_FLIP(pipe, plane, 					\
			ctrl_reg, surf_reg, stride_reg, stride_mask)	\
do{									\
	if (plane == PRIMARY_PLANE) {					\
		ctrl_reg = GVT_DSPCNTR(pipe);				\
		surf_reg = GVT_DSPSURF(pipe);				\
		stride_reg = GVT_DSPSTRIDE(pipe);			\
		stride_mask = _PRI_PLANE_STRIDE_MASK;			\
	} else {							\
		ASSERT (plane == SPRITE_PLANE);				\
		ctrl_reg = GVT_SPRCTL(pipe);				\
		surf_reg = GVT_SPRSURF(pipe);				\
		stride_reg = GVT_SPRSTRIDE(pipe);			\
		stride_mask = _SPRITE_STRIDE_MASK;			\
	}								\
}while(0);

static bool gvt_flip_parameter_check(struct parser_exec_state *s,
				u32 plane_code,
				u32 stride_val,
				u32 surf_val)
{
	enum pipe pipe = I915_MAX_PIPES;
	enum gvt_plane_type plane = MAX_PLANE;
	u32 surf_reg, ctrl_reg;
	u32 stride_reg, stride_mask, phys_stride;
	u32 tile_para, tile_in_ctrl;
	bool async_flip;

	if (!display_flip_decode_plane_info(plane_code, &pipe, &plane))
		return false;

	GET_INFO_FOR_FLIP(pipe, plane, ctrl_reg, surf_reg, stride_reg, stride_mask);

	async_flip = ((surf_val & FLIP_TYPE_MASK) == 0x1);
	tile_para = ((stride_val & TILE_PARA_MASK) >> TILE_PARA_SHIFT);
	tile_in_ctrl = (__vreg(s->vgt, ctrl_reg) & PLANE_TILE_MASK)
				>> PLANE_TILE_SHIFT;

	phys_stride = __vreg(s->vgt, stride_reg);

	if ((__vreg(s->vgt, stride_reg) & stride_mask)
		!= (stride_val & PITCH_MASK)) {

		if (async_flip) {
			gvt_warn("Cannot change stride in async flip!");
			return false;
		}
	}

	if (tile_para != tile_in_ctrl) {

		if (async_flip) {
			gvt_warn("Cannot change tiling in async flip!");
			return false;
		}
	}

	return true;
}

static int gvt_handle_mi_display_flip(struct parser_exec_state *s)
{
	u32 surf_reg, surf_val, ctrl_reg;
	u32 stride_reg, stride_val, stride_mask;
	u32 tile_para;
	u32 opcode, plane_code;
	enum pipe pipe;
	enum gvt_plane_type plane;
	int length, rc = 0;
	struct gvt_fb_notify_msg msg;
	int i;

	opcode = cmd_val(s, 0);
	stride_val = cmd_val(s, 1);
	surf_val = cmd_val(s, 2);

	plane_code = (opcode & PLANE_SELECT_MASK) >> PLANE_SELECT_SHIFT;
	length = cmd_length(s);

	if (!display_flip_decode_plane_info(plane_code, &pipe, &plane)) {
		goto wrong_command;
	}

	if (length == 4) {
		gvt_warn("Page flip of Stereo 3D is not supported!");
		goto wrong_command;
	} else if (length != 3) {
		gvt_warn("Flip length not equal to 3, ignore handling flipping");
		goto wrong_command;
	}

	if ((pipe == I915_MAX_PIPES) || (plane == MAX_PLANE)) {
		gvt_warn("Invalid pipe/plane in MI_DISPLAY_FLIP!");
		goto wrong_command;
	}

	if (!gvt_flip_parameter_check(s, plane_code, stride_val, surf_val))
		goto wrong_command;

	GET_INFO_FOR_FLIP(pipe, plane,
			ctrl_reg, surf_reg, stride_reg, stride_mask);
	tile_para = ((stride_val & TILE_PARA_MASK) >> TILE_PARA_SHIFT);

	__vreg(s->vgt, stride_reg) = (stride_val & stride_mask) |
		(__vreg(s->vgt, stride_reg) & (~stride_mask));
	__vreg(s->vgt, ctrl_reg) = (tile_para << PLANE_TILE_SHIFT) |
		(__vreg(s->vgt, ctrl_reg) & (~PLANE_TILE_MASK));
	__vreg(s->vgt, surf_reg) = (surf_val & SURF_MASK) |
		(__vreg(s->vgt, surf_reg) & (~SURF_MASK));
	__sreg(s->vgt, stride_reg) = __vreg(s->vgt, stride_reg);
	__sreg(s->vgt, ctrl_reg) = __vreg(s->vgt, ctrl_reg);
	__sreg(s->vgt, surf_reg) = __vreg(s->vgt, surf_reg);

	__vreg(s->vgt, GVT_PIPE_FLIPCOUNT(pipe))++;

	msg.vm_id = s->vgt->vm_id;
	msg.pipe_id = pipe;
	msg.plane_id = plane;
	gvt_fb_notifier_call_chain(FB_DISPLAY_FLIP, &msg);

	gvt_dbg_cmd("VM %d: mi_display_flip to be ignored",
			s->vgt->vm_id);

	for (i = 0; i < length; i ++)
		patch_value(s, cmd_ptr(s, i), MI_NOOP);

	gvt_inject_flip_done(s->vgt, pipe);

	return rc;

wrong_command:
	for (i = 0; i < length; i ++)
		patch_value(s, cmd_ptr(s, i), MI_NOOP);
	return rc;
}

static int gvt_cmd_handler_mi_display_flip(struct parser_exec_state *s)
{
	addr_audit_32(s, 2);
	return gvt_handle_mi_display_flip(s);
}

static bool is_wait_for_flip_pending(u32 cmd)
{
	return cmd & (MI_WAIT_FOR_PLANE_A_FLIP_PENDING |
		MI_WAIT_FOR_PLANE_B_FLIP_PENDING |
		MI_WAIT_FOR_PLANE_C_FLIP_PENDING |
		MI_WAIT_FOR_SPRITE_A_FLIP_PENDING |
		MI_WAIT_FOR_SPRITE_B_FLIP_PENDING |
		MI_WAIT_FOR_SPRITE_C_FLIP_PENDING);
}

static int gvt_cmd_handler_mi_wait_for_event(struct parser_exec_state *s)
{
	int rc = 0;
	u32 cmd = cmd_val(s, 0);

	if (!is_wait_for_flip_pending(cmd))
		return rc;

	patch_value(s, cmd_ptr(s, 0), MI_NOOP);
	return rc;
}

static unsigned long get_gma_bb_from_cmd(struct parser_exec_state *s, int index)
{
	unsigned long addr;
	unsigned long gma_high, gma_low;
	int gmadr_bytes = s->vgt->pdev->device_info.gmadr_bytes_in_cmd;

	ASSERT(gmadr_bytes == 4 || gmadr_bytes == 8);

	gma_low = cmd_val(s, index) & BATCH_BUFFER_ADDR_MASK;

	if (gmadr_bytes == 4) {
		addr = gma_low;
	} else {
		gma_high = cmd_val(s, index + 1) & BATCH_BUFFER_ADDR_HIGH_MASK;
		addr = (((unsigned long)gma_high) << 32) | gma_low;
	}

	return addr;
}

static inline bool gvt_cmd_addr_audit_with_bitmap(struct parser_exec_state *s,
			unsigned long addr_bitmap)
{
	unsigned int bit;
	unsigned int delta = 0;
	int cmd_len = cmd_length(s);

	if (!addr_bitmap)
		return true;

	for_each_set_bit(bit, &addr_bitmap, sizeof(addr_bitmap)*8) {
		if (bit + delta >= cmd_len)
			return false;
		address_audit(s, bit + delta);
		delta = delta + gmadr_dw_number(s) - 1;
	}

	return true;
}

static int gvt_cmd_handler_mi_update_gtt(struct parser_exec_state *s)
{
	gvt_err("Unexpectted mi_update_gtt in VM command buffer");
	return -1;
}

static int gvt_cmd_handler_mi_flush_dw(struct parser_exec_state *s)
{
	int i, len;
	int offset = 1;

	/* Check post-sync bit */
	if ((cmd_val(s, 0) >> 14) & 0x3)
		address_audit(s, offset);
	offset += gmadr_dw_number(s);

	/* Check notify bit */
	if ((cmd_val(s,0) & (1 << 8)))
		set_bit(gvt_ring_id_to_mi_flush_dw_event(s->ring_id), s->workload->pending_events);

	len = cmd_length(s);
	for (i = 2; i < len; i++) {
		address_audit(s, offset);
		offset += gmadr_dw_number(s);
	}

	return 0;
}

static void addr_type_update_snb(struct parser_exec_state* s)
{
	if ((s->buf_type == RING_BUFFER_INSTRUCTION) &&
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

	return 0;

	if (IS_BROADWELL(pdev->dev_priv)) {
		if (s->ring_id == BCS)
			return 0;
		/* BDW decides privilege based on address space */
		if (cmd_val(s, 0) & (1 << 8))
			return 0;
	}

	return 1;
}

static int gvt_cmd_handler_mi_batch_buffer_start(struct parser_exec_state *s)
{
	int rc = 0;
	bool second_level;

	if (s->buf_type == BATCH_BUFFER_2ND_LEVEL) {
		gvt_err("MI_BATCH_BUFFER_START not allowd in 2nd level batch buffer");
		return -EINVAL;
	}

	second_level = BATCH_BUFFER_2ND_LEVEL_BIT(cmd_val(s, 0)) == 1;
	if (second_level && (s->buf_type != BATCH_BUFFER_INSTRUCTION)) {
		gvt_err("Jumping to 2nd level batch buffer from ring buffer is not allowd");
		return -EINVAL;
	}

	s->saved_buf_addr_type = s->buf_addr_type;

	addr_type_update_snb(s);

	if (s->buf_type == RING_BUFFER_INSTRUCTION) {
		s->ret_ip_gma_ring = s->ip_gma + cmd_length(s) * sizeof(u32);
		s->buf_type = BATCH_BUFFER_INSTRUCTION;
	} else if (second_level) {
		s->buf_type = BATCH_BUFFER_2ND_LEVEL;
		s->ret_ip_gma_bb = s->ip_gma + cmd_length(s) * sizeof(u32);
	}

	address_audit(s, 1);

	if (batch_buffer_needs_scan(s)) {
		rc = ip_gma_set(s, get_gma_bb_from_cmd(s, 1));
		if (rc < 0)
			gvt_warn("invalid batch buffer addr, so skip scanning it");
	} else {
		struct gvt_statistics *stat = &s->vgt->stat;

		stat->skip_bb_cnt++;
		/* emulate a batch buffer end to do return right */
		rc = gvt_cmd_handler_mi_batch_buffer_end(s);
		if (rc < 0)
			gvt_err("skip batch buffer error");
	}

	return rc;
}

static int gvt_cmd_handler_3dstate_vertex_buffers(struct parser_exec_state *s)
{
	int length, offset;

	length = cmd_length(s);

	for (offset = 1; offset < length; offset += 4) {
		address_audit(s, offset + 1);
		address_audit(s, offset + 2);
	}

	return 0;
}

static int gvt_cmd_handler_3dstate_vertex_buffers_bdw(struct parser_exec_state *s)
{
	int length, offset;

	length = cmd_length(s);

	for (offset = 1; offset < length; offset += 4) {
		address_audit(s, offset + 1);
	}

	return 0;
}

static int gvt_cmd_handler_3dstate_index_buffer(struct parser_exec_state *s)
{
	address_audit(s, 1);

	if (cmd_val(s, 2) != 0)
		address_audit(s, 2);

	return 0;
}

static int gvt_cmd_handler_3dstate_constant(struct parser_exec_state *s)
{
	int offset = 3;
	int cmd_len = cmd_length(s);

	while (offset < cmd_len) {
		address_audit(s, offset);
		offset += gmadr_dw_number(s);
	}

	return 0;
}

static int gvt_cmd_handler_mfx_pipe_buf_addr_state(struct parser_exec_state *s)
{
	/*  address pattern of the command is like below:
	 *  from bit0: "01010101 01010111 11111111 11111010 1010"
	 */
	gvt_cmd_addr_audit_with_bitmap(s, 0x055fffeaaaUL);

	return 0;
}

static int gvt_cmd_handler_mfx_ind_obj_base_addr_state(struct parser_exec_state *s)
{
	/*  address pattern of the command is like below:
	 *  from bit0: "10110110 11011010"
	 */
	gvt_cmd_addr_audit_with_bitmap(s, 0x5b6d);

	return 0;
}

static inline int base_and_upper_addr_fix(struct parser_exec_state *s)
{
	address_audit(s, 1);
	/* Zero Bound is ignore */
	if (cmd_val(s, 2) >> 12)
		address_audit(s, 2);
	return 0;
}

static int gvt_cmd_handler_mfx_2_6_0_0(struct parser_exec_state *s)
{
	base_and_upper_addr_fix(s);
	address_audit(s, 2);
	return 0;
}

static struct cmd_info cmd_info[] = {
	{"MI_NOOP", OP_MI_NOOP, F_LEN_CONST|F_POST_HANDLE, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_SET_PREDICATE", OP_MI_SET_PREDICATE, F_LEN_CONST, R_ALL, D_ALL,
		0, 1, NULL},

	{"MI_USER_INTERRUPT", OP_MI_USER_INTERRUPT, F_LEN_CONST, R_ALL, D_ALL, 0, 1, gvt_cmd_handler_mi_user_interrupt},

	{"MI_WAIT_FOR_EVENT", OP_MI_WAIT_FOR_EVENT, F_LEN_CONST | F_POST_HANDLE, R_RCS | R_BCS,
		D_ALL, 0, 1, gvt_cmd_handler_mi_wait_for_event},

	{"MI_FLUSH", OP_MI_FLUSH, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_ARB_CHECK", OP_MI_ARB_CHECK, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_RS_CONTROL", OP_MI_RS_CONTROL, F_LEN_CONST, R_RCS, D_ALL, 0, 1, NULL},

	{"MI_REPORT_HEAD", OP_MI_REPORT_HEAD, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_ARB_ON_OFF", OP_MI_ARB_ON_OFF, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_URB_ATOMIC_ALLOC", OP_MI_URB_ATOMIC_ALLOC, F_LEN_CONST, R_RCS,
		D_ALL, 0, 1, NULL},

	{"MI_BATCH_BUFFER_END", OP_MI_BATCH_BUFFER_END, F_IP_ADVANCE_CUSTOM|F_LEN_CONST,
		R_ALL, D_ALL, 0, 1, gvt_cmd_handler_mi_batch_buffer_end},

	{"MI_SUSPEND_FLUSH", OP_MI_SUSPEND_FLUSH, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_PREDICATE", OP_MI_PREDICATE, F_LEN_CONST, R_RCS, D_ALL, 0, 1, NULL},

	{"MI_TOPOLOGY_FILTER", OP_MI_TOPOLOGY_FILTER, F_LEN_CONST, R_ALL,
		D_ALL, 0, 1, NULL},

	{"MI_SET_APPID", OP_MI_SET_APPID, F_LEN_CONST, R_ALL, D_ALL, 0, 1, NULL},

	{"MI_RS_CONTEXT", OP_MI_RS_CONTEXT, F_LEN_CONST, R_RCS, D_ALL, 0, 1, NULL},

	{"MI_DISPLAY_FLIP", OP_MI_DISPLAY_FLIP, F_LEN_VAR|F_POST_HANDLE, R_RCS | R_BCS,
		D_ALL, 0, 8, gvt_cmd_handler_mi_display_flip},

	{"MI_SEMAPHORE_MBOX", OP_MI_SEMAPHORE_MBOX, F_LEN_VAR, R_ALL, D_ALL, 0, 8, NULL },

	{"MI_MATH", OP_MI_MATH, F_LEN_VAR, R_ALL, D_ALL, 0, 8, NULL},

	{"MI_URB_CLEAR", OP_MI_URB_CLEAR, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"ME_SEMAPHORE_SIGNAL", OP_MI_SEMAPHORE_SIGNAL, F_LEN_VAR, R_ALL, D_BDW, 0, 8, NULL},

	{"ME_SEMAPHORE_WAIT", OP_MI_SEMAPHORE_WAIT, F_LEN_VAR, R_ALL, D_BDW, ADDR_FIX_1(2), 8, NULL},

	{"MI_STORE_DATA_IMM", OP_MI_STORE_DATA_IMM, F_LEN_VAR, R_ALL, D_BDW,
		ADDR_FIX_1(1), 10, NULL},

	{"MI_STORE_DATA_INDEX", OP_MI_STORE_DATA_INDEX, F_LEN_VAR, R_ALL, D_ALL,
		0, 8, NULL},

	{"MI_LOAD_REGISTER_IMM", OP_MI_LOAD_REGISTER_IMM, F_LEN_VAR, R_ALL, D_ALL, 0, 8, gvt_cmd_handler_lri},

	{"MI_UPDATE_GTT", OP_MI_UPDATE_GTT, F_LEN_VAR, R_ALL, D_BDW,
		0, 10, gvt_cmd_handler_mi_update_gtt},

	{"MI_STORE_REGISTER_MEM", OP_MI_STORE_REGISTER_MEM, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(2), 8, gvt_cmd_handler_srm},

	{"MI_FLUSH_DW", OP_MI_FLUSH_DW, F_LEN_VAR, R_ALL, D_ALL,
		0, 6, gvt_cmd_handler_mi_flush_dw},

	{"MI_CLFLUSH", OP_MI_CLFLUSH, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(1), 10, NULL},

	{"MI_REPORT_PERF_COUNT", OP_MI_REPORT_PERF_COUNT, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(1), 6, NULL},

	{"MI_LOAD_REGISTER_MEM", OP_MI_LOAD_REGISTER_MEM, F_LEN_VAR, R_ALL, D_ALL,
		ADDR_FIX_1(2), 8, gvt_cmd_handler_lrm},

	{"MI_LOAD_REGISTER_REG", OP_MI_LOAD_REGISTER_REG, F_LEN_VAR, R_ALL, D_ALL,
		0, 8, gvt_cmd_handler_lrr},

	{"MI_RS_STORE_DATA_IMM", OP_MI_RS_STORE_DATA_IMM, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, NULL},

	{"MI_LOAD_URB_MEM", OP_MI_LOAD_URB_MEM, F_LEN_VAR, R_RCS, D_ALL,
		ADDR_FIX_1(2), 8, NULL},

	{"MI_STORE_URM_MEM", OP_MI_STORE_URM_MEM, F_LEN_VAR, R_RCS, D_ALL,
		ADDR_FIX_1(2), 8, NULL},

	{"MI_OP_2E", OP_MI_2E, F_LEN_VAR, R_ALL, D_BDW, ADDR_FIX_2(1, 2), 8, NULL},

	{"MI_OP_2F", OP_MI_2F, F_LEN_VAR, R_ALL, D_BDW, ADDR_FIX_1(1), 8, NULL},

	{"MI_BATCH_BUFFER_START", OP_MI_BATCH_BUFFER_START, F_IP_ADVANCE_CUSTOM,
		R_ALL, D_ALL, 0, 8, gvt_cmd_handler_mi_batch_buffer_start},

	{"MI_CONDITIONAL_BATCH_BUFFER_END", OP_MI_CONDITIONAL_BATCH_BUFFER_END,
		F_LEN_VAR, R_ALL, D_ALL, ADDR_FIX_1(2), 8, NULL},

	{"MI_LOAD_SCAN_LINES_INCL", OP_MI_LOAD_SCAN_LINES_INCL, F_LEN_CONST, R_RCS | R_BCS, D_ALL,
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

	{"3DSTATE_VIEWPORT_STATE_POINTERS_SF_CLIP", OP_3DSTATE_VIEWPORT_STATE_POINTERS_SF_CLIP,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_VIEWPORT_STATE_POINTERS_CC", OP_3DSTATE_VIEWPORT_STATE_POINTERS_CC,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_BLEND_STATE_POINTERS", OP_3DSTATE_BLEND_STATE_POINTERS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_DEPTH_STENCIL_STATE_POINTERS", OP_3DSTATE_DEPTH_STENCIL_STATE_POINTERS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS_VS", OP_3DSTATE_BINDING_TABLE_POINTERS_VS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS_HS", OP_3DSTATE_BINDING_TABLE_POINTERS_HS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS_DS", OP_3DSTATE_BINDING_TABLE_POINTERS_DS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS_GS", OP_3DSTATE_BINDING_TABLE_POINTERS_GS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_POINTERS_PS", OP_3DSTATE_BINDING_TABLE_POINTERS_PS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS_VS", OP_3DSTATE_SAMPLER_STATE_POINTERS_VS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS_HS", OP_3DSTATE_SAMPLER_STATE_POINTERS_HS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS_DS", OP_3DSTATE_SAMPLER_STATE_POINTERS_DS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS_GS", OP_3DSTATE_SAMPLER_STATE_POINTERS_GS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_SAMPLER_STATE_POINTERS_PS", OP_3DSTATE_SAMPLER_STATE_POINTERS_PS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_URB_VS", OP_3DSTATE_URB_VS, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, NULL},

	{"3DSTATE_URB_HS", OP_3DSTATE_URB_HS, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, NULL},

	{"3DSTATE_URB_DS", OP_3DSTATE_URB_DS, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, NULL},

	{"3DSTATE_URB_GS", OP_3DSTATE_URB_GS, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, NULL},

	{"3DSTATE_GATHER_CONSTANT_VS", OP_3DSTATE_GATHER_CONSTANT_VS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_GATHER_CONSTANT_GS", OP_3DSTATE_GATHER_CONSTANT_GS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_GATHER_CONSTANT_HS", OP_3DSTATE_GATHER_CONSTANT_HS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_GATHER_CONSTANT_DS", OP_3DSTATE_GATHER_CONSTANT_DS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_GATHER_CONSTANT_PS", OP_3DSTATE_GATHER_CONSTANT_PS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_DX9_CONSTANTF_VS", OP_3DSTATE_DX9_CONSTANTF_VS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 11, NULL},

	{"3DSTATE_DX9_CONSTANTF_PS", OP_3DSTATE_DX9_CONSTANTF_PS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 11, NULL},

	{"3DSTATE_DX9_CONSTANTI_VS", OP_3DSTATE_DX9_CONSTANTI_VS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_DX9_CONSTANTI_PS", OP_3DSTATE_DX9_CONSTANTI_PS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_DX9_CONSTANTB_VS", OP_3DSTATE_DX9_CONSTANTB_VS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_DX9_CONSTANTB_PS", OP_3DSTATE_DX9_CONSTANTB_PS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_DX9_LOCAL_VALID_VS", OP_3DSTATE_DX9_LOCAL_VALID_VS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_DX9_LOCAL_VALID_PS", OP_3DSTATE_DX9_LOCAL_VALID_PS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_DX9_GENERATE_ACTIVE_VS", OP_3DSTATE_DX9_GENERATE_ACTIVE_VS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_DX9_GENERATE_ACTIVE_PS", OP_3DSTATE_DX9_GENERATE_ACTIVE_PS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_BINDING_TABLE_EDIT_VS", OP_3DSTATE_BINDING_TABLE_EDIT_VS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 9, NULL},

	{"3DSTATE_BINDING_TABLE_EDIT_GS", OP_3DSTATE_BINDING_TABLE_EDIT_GS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 9, NULL},

	{"3DSTATE_BINDING_TABLE_EDIT_HS", OP_3DSTATE_BINDING_TABLE_EDIT_HS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 9, NULL},

	{"3DSTATE_BINDING_TABLE_EDIT_DS", OP_3DSTATE_BINDING_TABLE_EDIT_DS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 9, NULL},

	{"3DSTATE_BINDING_TABLE_EDIT_PS", OP_3DSTATE_BINDING_TABLE_EDIT_PS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 9, NULL},

	{"3DSTATE_VF_INSTANCING", OP_3DSTATE_VF_INSTANCING, F_LEN_VAR, R_RCS, D_BDW, 0, 8, NULL},

	{"3DSTATE_VF_SGVS", OP_3DSTATE_VF_SGVS, F_LEN_VAR, R_RCS, D_BDW, 0, 8, NULL},

	{"3DSTATE_VF_TOPOLOGY", OP_3DSTATE_VF_TOPOLOGY, F_LEN_VAR, R_RCS, D_BDW, 0, 8, NULL},

	{"3DSTATE_WM_CHROMAKEY", OP_3DSTATE_WM_CHROMAKEY, F_LEN_VAR, R_RCS, D_BDW, 0, 8, NULL},

	{"3DSTATE_PS_BLEND", OP_3DSTATE_PS_BLEND, F_LEN_VAR, R_RCS, D_BDW, 0, 8, NULL},

	{"3DSTATE_WM_DEPTH_STENCIL", OP_3DSTATE_WM_DEPTH_STENCIL, F_LEN_VAR, R_RCS, D_BDW, 0, 8, NULL},

	{"3DSTATE_PS_EXTRA", OP_3DSTATE_PS_EXTRA, F_LEN_VAR, R_RCS, D_BDW, 0, 8, NULL},

	{"3DSTATE_RASTER", OP_3DSTATE_RASTER, F_LEN_VAR, R_RCS, D_BDW, 0, 8, NULL},

	{"3DSTATE_SBE_SWIZ", OP_3DSTATE_SBE_SWIZ, F_LEN_VAR, R_RCS, D_BDW, 0, 8, NULL},

	{"3DSTATE_WM_HZ_OP", OP_3DSTATE_WM_HZ_OP, F_LEN_VAR, R_RCS, D_BDW, 0, 8, NULL},

	{"3DSTATE_VERTEX_BUFFERS", OP_3DSTATE_VERTEX_BUFFERS, F_LEN_VAR, R_RCS,
		D_BDW, 0, 8, gvt_cmd_handler_3dstate_vertex_buffers_bdw},

	{"3DSTATE_VERTEX_ELEMENTS", OP_3DSTATE_VERTEX_ELEMENTS, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_INDEX_BUFFER", OP_3DSTATE_INDEX_BUFFER, F_LEN_VAR, R_RCS,
		D_BDW, ADDR_FIX_1(2), 8, NULL},

	{"3DSTATE_VF_STATISTICS", OP_3DSTATE_VF_STATISTICS, F_LEN_CONST,
		R_RCS, D_ALL, 0, 1, NULL},

	{"3DSTATE_VF", OP_3DSTATE_VF, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_CC_STATE_POINTERS", OP_3DSTATE_CC_STATE_POINTERS, F_LEN_VAR,
		R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_SCISSOR_STATE_POINTERS", OP_3DSTATE_SCISSOR_STATE_POINTERS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_GS", OP_3DSTATE_GS, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_CLIP", OP_3DSTATE_CLIP, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_WM", OP_3DSTATE_WM, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_CONSTANT_GS", OP_3DSTATE_CONSTANT_GS, F_LEN_VAR, R_RCS,
		D_BDW, 0, 8, gvt_cmd_handler_3dstate_constant},

	{"3DSTATE_CONSTANT_PS", OP_3DSTATE_CONSTANT_PS, F_LEN_VAR, R_RCS,
		D_BDW, 0, 8, gvt_cmd_handler_3dstate_constant},

	{"3DSTATE_SAMPLE_MASK", OP_3DSTATE_SAMPLE_MASK, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_CONSTANT_HS", OP_3DSTATE_CONSTANT_HS, F_LEN_VAR, R_RCS,
		D_BDW, 0, 8, gvt_cmd_handler_3dstate_constant},

	{"3DSTATE_CONSTANT_DS", OP_3DSTATE_CONSTANT_DS, F_LEN_VAR, R_RCS,
		D_BDW, 0, 8, gvt_cmd_handler_3dstate_constant},

	{"3DSTATE_HS", OP_3DSTATE_HS, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_TE", OP_3DSTATE_TE, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_DS", OP_3DSTATE_DS, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_STREAMOUT", OP_3DSTATE_STREAMOUT, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_SBE", OP_3DSTATE_SBE, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_PS", OP_3DSTATE_PS, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_DRAWING_RECTANGLE", OP_3DSTATE_DRAWING_RECTANGLE, F_LEN_VAR,
		R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_SAMPLER_PALETTE_LOAD0", OP_3DSTATE_SAMPLER_PALETTE_LOAD0,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_CHROMA_KEY", OP_3DSTATE_CHROMA_KEY, F_LEN_VAR, R_RCS, D_ALL,
		0, 8, NULL},

	{"3DSTATE_DEPTH_BUFFER", OP_3DSTATE_DEPTH_BUFFER, F_LEN_VAR, R_RCS,
		D_ALL, ADDR_FIX_1(2), 8, NULL},

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

	{"3DSTATE_MULTISAMPLE", OP_3DSTATE_MULTISAMPLE_BDW, F_LEN_VAR, R_RCS, D_BDW,
		0, 8, NULL},

	{"3DSTATE_STENCIL_BUFFER", OP_3DSTATE_STENCIL_BUFFER, F_LEN_VAR, R_RCS,
		D_ALL, ADDR_FIX_1(2), 8, NULL},

	{"3DSTATE_HIER_DEPTH_BUFFER", OP_3DSTATE_HIER_DEPTH_BUFFER, F_LEN_VAR,
		R_RCS, D_ALL, ADDR_FIX_1(2), 8, NULL},

	{"3DSTATE_CLEAR_PARAMS", OP_3DSTATE_CLEAR_PARAMS, F_LEN_VAR,
		R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_PUSH_CONSTANT_ALLOC_VS", OP_3DSTATE_PUSH_CONSTANT_ALLOC_VS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_PUSH_CONSTANT_ALLOC_HS", OP_3DSTATE_PUSH_CONSTANT_ALLOC_HS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_PUSH_CONSTANT_ALLOC_DS", OP_3DSTATE_PUSH_CONSTANT_ALLOC_DS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_PUSH_CONSTANT_ALLOC_GS", OP_3DSTATE_PUSH_CONSTANT_ALLOC_GS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_PUSH_CONSTANT_ALLOC_PS", OP_3DSTATE_PUSH_CONSTANT_ALLOC_PS,
		F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_MONOFILTER_SIZE", OP_3DSTATE_MONOFILTER_SIZE, F_LEN_VAR, R_RCS,
		D_ALL, 0, 8, NULL},

	{"3DSTATE_SO_DECL_LIST", OP_3DSTATE_SO_DECL_LIST, F_LEN_VAR, R_RCS, D_ALL,
		0, 9, NULL},

	{"3DSTATE_SO_BUFFER", OP_3DSTATE_SO_BUFFER, F_LEN_VAR, R_RCS, D_BDW,
		ADDR_FIX_2(2, 4), 8, NULL},

	{"3DSTATE_BINDING_TABLE_POOL_ALLOC", OP_3DSTATE_BINDING_TABLE_POOL_ALLOC,
		F_LEN_VAR, R_RCS, D_BDW, ADDR_FIX_1(1), 8, NULL},

	{"3DSTATE_GATHER_POOL_ALLOC", OP_3DSTATE_GATHER_POOL_ALLOC,
		F_LEN_VAR, R_RCS, D_BDW, ADDR_FIX_1(1), 8, NULL},

	{"3DSTATE_DX9_CONSTANT_BUFFER_POOL_ALLOC", OP_3DSTATE_DX9_CONSTANT_BUFFER_POOL_ALLOC,
		F_LEN_VAR, R_RCS, D_BDW, ADDR_FIX_1(1), 8, NULL},

	{"3DSTATE_SAMPLE_PATTERN", OP_3DSTATE_SAMPLE_PATTERN, F_LEN_VAR, R_RCS, D_BDW, 0, 8, NULL},

	{"PIPE_CONTROL", OP_PIPE_CONTROL, F_LEN_VAR, R_RCS, D_ALL,
		ADDR_FIX_1(2), 8, gvt_cmd_handler_pipe_control},

	{"3DPRIMITIVE", OP_3DPRIMITIVE, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"PIPELINE_SELECT", OP_PIPELINE_SELECT, F_LEN_CONST, R_RCS, D_ALL, 0, 1, NULL},

	{"STATE_PREFETCH", OP_STATE_PREFETCH, F_LEN_VAR, R_RCS, D_ALL,
		ADDR_FIX_1(1), 8, NULL},

	{"STATE_SIP", OP_STATE_SIP, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"STATE_BASE_ADDRESS", OP_STATE_BASE_ADDRESS, F_LEN_VAR, R_RCS, D_BDW,
		ADDR_FIX_5(1, 3, 4, 5, 6), 8, NULL},

	{"OP_3D_MEDIA_0_1_4", OP_3D_MEDIA_0_1_4, F_LEN_VAR, R_RCS, D_ALL,
		ADDR_FIX_1(1), 8, NULL},

	{"3DSTATE_VS", OP_3DSTATE_VS, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_SF", OP_3DSTATE_SF, F_LEN_VAR, R_RCS, D_ALL, 0, 8, NULL},

	{"3DSTATE_CONSTANT_VS", OP_3DSTATE_CONSTANT_VS, F_LEN_VAR, R_RCS, D_BDW,
		0, 8, gvt_cmd_handler_3dstate_constant},

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
		R_VCS, D_BDW, 0, 12, gvt_cmd_handler_mfx_pipe_buf_addr_state},

	{"MFX_IND_OBJ_BASE_ADDR_STATE", OP_MFX_IND_OBJ_BASE_ADDR_STATE, F_LEN_VAR,
		R_VCS, D_BDW, 0, 12, gvt_cmd_handler_mfx_ind_obj_base_addr_state},

	{"MFX_BSP_BUF_BASE_ADDR_STATE", OP_MFX_BSP_BUF_BASE_ADDR_STATE, F_LEN_VAR,
		R_VCS, D_BDW, ADDR_FIX_3(1, 3, 5), 12, NULL},

	{"OP_2_0_0_5", OP_2_0_0_5, F_LEN_VAR, R_VCS, D_BDW, 0, 12, NULL},

	{"MFX_STATE_POINTER", OP_MFX_STATE_POINTER, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_QM_STATE", OP_MFX_QM_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_FQM_STATE", OP_MFX_FQM_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_PAK_INSERT_OBJECT", OP_MFX_PAK_INSERT_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_STITCH_OBJECT", OP_MFX_STITCH_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFD_IT_OBJECT", OP_MFD_IT_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_WAIT", OP_MFX_WAIT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 6, NULL},

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
		R_VCS, D_ALL, 0, 12, NULL},
	{"MFD_AVC_DPB_STATE", OP_MFD_AVC_DPB_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFD_AVC_BSD_OBJECT", OP_MFD_AVC_BSD_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFD_AVC_SLICEADDR", OP_MFD_AVC_SLICEADDR, F_LEN_VAR,
		R_VCS, D_ALL, ADDR_FIX_1(2), 12, NULL},

	{"MFC_AVC_PAK_OBJECT", OP_MFC_AVC_PAK_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_VC1_PRED_PIPE_STATE", OP_MFX_VC1_PRED_PIPE_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_VC1_DIRECTMODE_STATE", OP_MFX_VC1_DIRECTMODE_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFD_VC1_SHORT_PIC_STATE", OP_MFD_VC1_SHORT_PIC_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFD_VC1_LONG_PIC_STATE", OP_MFD_VC1_LONG_PIC_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

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
		0, 16, gvt_cmd_handler_mfx_2_6_0_0},

	{"MFX_2_6_0_9", OP_MFX_2_6_0_9, F_LEN_VAR, R_VCS, D_ALL, 0, 16, NULL},

	{"MFX_2_6_0_8", OP_MFX_2_6_0_8, F_LEN_VAR, R_VCS, D_ALL, 0, 16, NULL},

	{"MFX_JPEG_PIC_STATE", OP_MFX_JPEG_PIC_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFX_JPEG_HUFF_TABLE_STATE", OP_MFX_JPEG_HUFF_TABLE_STATE, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"MFD_JPEG_BSD_OBJECT", OP_MFD_JPEG_BSD_OBJECT, F_LEN_VAR,
		R_VCS, D_ALL, 0, 12, NULL},

	{"VEBOX_STATE", OP_VEB_STATE, F_LEN_VAR, R_VECS, D_ALL, 0, 12, NULL},

	{"VEBOX_SURFACE_STATE", OP_VEB_SURFACE_STATE, F_LEN_VAR, R_VECS, D_ALL, 0, 12, NULL},

	{"VEB_DI_IECP", OP_VEB_DNDI_IECP_STATE, F_LEN_VAR, R_VECS, D_BDW_PLUS, 0, 20, NULL},
};


static int gvt_cmd_advance_default(struct parser_exec_state *s)
{
	return ip_gma_advance(s, cmd_length(s));
}

static void gvt_add_cmd_entry(struct pgt_device *pdev, struct gvt_cmd_entry *e)
{
	hash_add(pdev->cmd_table, &e->hlist, e->info->opcode);
}

#define GVT_MAX_CMD_LENGTH     20  /* In Dword */

static void trace_cs_command(struct parser_exec_state *s, cycles_t cost_pre_cmd_handler, cycles_t cost_cmd_handler)
{
	/* This buffer is used by ftrace to store all commands copied from guest gma
	* space. Sometimes commands can cross pages, this should not be handled in
	 * ftrace logic. So this is just used as a 'bounce buffer' */
	u32 cmd_trace_buf[GVT_MAX_CMD_LENGTH];
	int i;
	u32 cmd_len = cmd_length(s);
	/* The chosen value of GVT_MAX_CMD_LENGTH are just based on
	 * following two considerations:
	 * 1) From observation, most common ring commands is not that long.
	 *    But there are execeptions. So it indeed makes sence to observe
	 *    longer commands.
	 * 2) From the performance and debugging point of view, dumping all
	 *    contents of very commands is not necessary.
	 * We mgith shrink GVT_MAX_CMD_LENGTH or remove this trace event in
	 * future for performance considerations.
	 */
	if (unlikely(cmd_len > GVT_MAX_CMD_LENGTH)) {
		gvt_dbg_cmd("cmd length exceed tracing limitation!");
		cmd_len = GVT_MAX_CMD_LENGTH;
	}

	for (i = 0; i < cmd_len; i++)
		cmd_trace_buf[i] = cmd_val(s, i);

	trace_gvt_command(s->vgt->vm_id, s->ring_id, s->ip_gma, cmd_trace_buf,
			cmd_len, s->buf_type == RING_BUFFER_INSTRUCTION, cost_pre_cmd_handler, cost_cmd_handler);

}

/* call the cmd handler, and advance ip */
static bool gvt_cmd_parser_exec(struct parser_exec_state *s)
{
	struct cmd_info *info;
	u32 cmd;
	int rc = 0;
	cycles_t t0, t1, t2;

	t0 = get_cycles();

	cmd = cmd_val(s, 0);

	info = gvt_get_cmd_info(s->vgt->pdev, cmd, s->ring_id);
	if (info == NULL) {
		gvt_err("ERROR: unknown cmd 0x%x, opcode=0x%x", cmd,
				get_opcode(cmd, s->ring_id));
		parser_exec_state_dump(s);
		return false;
	}

	gvt_dbg_cmd("%s", info->name);

	s->info = info;

	gvt_cmd_addr_audit_with_bitmap(s, info->addr_bitmap);

	t1 = get_cycles();

	if (info->handler) {
		rc = info->handler(s);
		if (rc < 0) {
			gvt_err("%s handler error", info->name);
			return false;
		}
	}

	t2 = get_cycles();

	trace_cs_command(s, t1 - t0, t2 -t1);

	if (!(info->flag & F_IP_ADVANCE_CUSTOM)) {
		rc = gvt_cmd_advance_default(s);
		if (rc < 0) {
			gvt_err("%s IP advance error", info->name);
			return false;
		}
	}
	return true;
}

static inline bool gma_out_of_range(unsigned long gma, unsigned long gma_head, unsigned gma_tail)
{
	if ( gma_tail >= gma_head)
		return (gma < gma_head) || (gma > gma_tail);
	else
		return (gma > gma_tail) && (gma < gma_head);

}

static bool scan_workload(struct gvt_workload *workload)
{
	unsigned long gma_head, gma_tail, gma_bottom;
	struct parser_exec_state s;
	bool rc = false;

	/* ring base is page aligned */
	ASSERT((workload->rb_start & (PAGE_SIZE-1)) == 0);

	gma_head = workload->rb_start + workload->rb_head;
	gma_tail = workload->rb_start + workload->rb_tail;
	gma_bottom = workload->rb_start +  _RING_CTL_BUF_SIZE(workload->rb_ctl);

	s.buf_type = RING_BUFFER_INSTRUCTION;
	s.buf_addr_type = GTT_BUFFER;
	s.vgt = workload->vgt;
	s.ring_id = workload->ring_id;
	s.ring_start = workload->rb_start;
	s.ring_size = _RING_CTL_BUF_SIZE(workload->rb_ctl);
	s.ring_head = gma_head;
	s.ring_tail = gma_tail;
	s.rb_va = workload->shadow_ring_buffer_va;
	s.workload = workload;

	if (bypass_scan_mask & (1 << workload->ring_id))
		return true;

	if (ip_gma_set(&s, gma_head) < 0)
		goto out;

	gvt_dbg_cmd("scan_start: start=%lx end=%lx", gma_head, gma_tail);

	while(s.ip_gma != gma_tail) {
		if (s.buf_type == RING_BUFFER_INSTRUCTION) {
			if (!(s.ip_gma >= workload->rb_start) || !(s.ip_gma < gma_bottom)) {
				gvt_err("ERROR: ip_gma %lx out of ring scope."
					"(base:0x%lx, bottom: 0x%lx)",
					s.ip_gma, workload->rb_start, gma_bottom);
				return false;
			}
			if (gma_out_of_range(s.ip_gma, gma_head, gma_tail)) {
				gvt_err("ERROR: ip_gma %lx out of range."
					"(base:0x%lx, head: 0x%lx, tail: 0x%lx)",
					s.ip_gma, workload->rb_start, workload->rb_head, workload->rb_tail);
				break;
			}
		}

		rc = gvt_cmd_parser_exec(&s);
		if (!rc) {
			gvt_err("cmd parser error");
			break;
		}
	}

	gvt_dbg_cmd("scan_end");
out:
	return rc;
}

static bool copy_gma_to_hva(struct vgt_device *vgt, struct gvt_mm *mm,
		unsigned long gma, unsigned long end_gma, void *va)
{
	unsigned long copy_len, offset;
	unsigned long len = 0;
	void *gma_va;

	while (gma != end_gma) {
		gma_va = gvt_gma_to_va(mm, gma);
		if (!gma_va) {
			gvt_err("invalid gma address: %lx", gma);
			return false;
		}

		offset = gma & (GTT_PAGE_SIZE - 1);
		if (offset) {
			copy_len = (end_gma - gma) >= (GTT_PAGE_SIZE - offset) ?
				GTT_PAGE_SIZE - offset : end_gma - gma;
		} else {
			copy_len = (end_gma - gma) >= GTT_PAGE_SIZE ?
				GTT_PAGE_SIZE : end_gma - gma;
		}

		gvt_dbg_cmd("end gma %lx gma %lx offset %lx copy len %lx",
			end_gma, gma, offset, copy_len);

		hypervisor_read_va(vgt, gma_va,
			va + len, copy_len, 1);

		len += copy_len;
		gma += copy_len;
	}

	return true;
}

static bool shadow_workload_ring_buffer(struct gvt_workload *workload)
{
	struct vgt_device *vgt = workload->vgt;
	struct pgt_device *pdev = vgt->pdev;

	int ring_id = workload->ring_id;
	struct gvt_workload_scheduler *scheduler = &pdev->workload_scheduler;
	struct intel_context *shadow_ctx = scheduler->shadow_ctx;
	struct intel_ringbuffer *ringbuf = shadow_ctx->engine[ring_id].ringbuf;

	unsigned long gma_head, gma_tail, gma_top, guest_rb_size;
	unsigned copy_len = 0;

	int ret;

	guest_rb_size = _RING_CTL_BUF_SIZE(workload->rb_ctl);

	/* calculate workload ring buffer size */
	workload->rb_len = (workload->rb_tail + guest_rb_size -
			workload->rb_head) % guest_rb_size;

	gma_head = workload->rb_start + workload->rb_head;
	gma_tail = workload->rb_start + workload->rb_tail;
	gma_top = workload->rb_start + guest_rb_size;

	/* allocate shadow ring buffer */
	ret = intel_logical_ring_begin(workload->req, workload->rb_len / 4);
	if(ret)
		return false;

	/* get shadow ring buffer va */
	workload->shadow_ring_buffer_va = ringbuf->virtual_start + ringbuf->tail;

	/* head > tail --> copy head <-> top */
	if (gma_head > gma_tail) {
		if (!copy_gma_to_hva(vgt, vgt->gtt.ggtt_mm,
					gma_head, gma_top, workload->shadow_ring_buffer_va)) {
			gvt_err("fail to copy guest ring buffer");
			return false;
		}

		copy_len = gma_top - gma_head;
		gma_head = workload->rb_start;
	}

	/* copy head or start <-> tail */
	if (!copy_gma_to_hva(vgt, vgt->gtt.ggtt_mm,
				gma_head, gma_tail, workload->shadow_ring_buffer_va + copy_len)) {
		gvt_err("fail to copy guest ring buffer");
		return false;
	}

	ringbuf->tail += workload->rb_len;
	intel_logical_ring_advance(ringbuf);

	return true;
}

bool gvt_scan_and_shadow_workload(struct gvt_workload *workload)
{
	struct gvt_statistics *stat = &workload->vgt->stat;
	cycles_t t0, t1;

	t0 = get_cycles();

	stat->vring_scan_cnt++;

	if (!shadow_workload_ring_buffer(workload)) {
		gvt_err("fail to shadow workload ring_buffer");
		return false;
	}

	if (!scan_workload(workload)) {
		gvt_err("scan workload error");
		return false;
	}

	t1 = get_cycles();

	stat->vring_scan_cycles += t1 - t0;
	return true;
}

static struct cmd_info *find_cmd_entry_any_ring(struct pgt_device *pdev, unsigned int opcode, int rings)
{
	struct cmd_info* info = NULL;
	unsigned int ring;
	for_each_set_bit(ring, (unsigned long*)&rings, I915_NUM_RINGS) {
		info = find_cmd_entry(pdev, opcode, ring);
		if (info)
			break;
	}
	return info;
}

static bool init_cmd_table(struct pgt_device *pdev)
{
	int i;
	struct gvt_cmd_entry *e;
	struct cmd_info	*info;
	unsigned int gen_type;

	gen_type = gvt_get_device_type(pdev);

	for (i = 0; i < ARRAY_SIZE(cmd_info); i++) {
		if (!(cmd_info[i].devices & gen_type)) {
			gvt_dbg_cmd("CMD[%-30s] op[%04x] flag[%x] devs[%02x] rings[%02x] not registered",
					cmd_info[i].name, cmd_info[i].opcode, cmd_info[i].flag,
					cmd_info[i].devices, cmd_info[i].rings);
			continue;
		}

		e = kzalloc(sizeof(*e), GFP_KERNEL);
		if (e == NULL) {
			gvt_err("Insufficient memory in %s", __FUNCTION__);
			return false;
		}

		e->info = &cmd_info[i];

		info = find_cmd_entry_any_ring(pdev, e->info->opcode, e->info->rings);
		if (info) {
			gvt_err("%s %s duplicated", e->info->name, info->name);
			return false;;
		}

		INIT_HLIST_NODE(&e->hlist);
		gvt_add_cmd_entry(pdev, e);
		gvt_dbg_cmd("CMD[%-30s] op[%04x] flag[%x] devs[%02x] rings[%02x] registered",
				e->info->name,e->info->opcode, e->info->flag, e->info->devices,
				e->info->rings);
	}
	return true;
}

void clean_cmd_table(struct pgt_device *pdev)
{
	struct hlist_node *tmp;
	struct gvt_cmd_entry *e;
	int i;

	hash_for_each_safe(pdev->cmd_table, i, tmp, e, hlist)
		kfree(e);

	hash_init(pdev->cmd_table);
}

void gvt_clean_cmd_parser(struct pgt_device *pdev)
{
	clean_cmd_table(pdev);
}

bool gvt_init_cmd_parser(struct pgt_device *pdev)
{
	if (!init_cmd_table(pdev)) {
		gvt_clean_cmd_parser(pdev);
		return false;
	}
	return true;
}
