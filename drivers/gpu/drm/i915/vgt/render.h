/*
 * vGT ringbuffer header
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

#ifndef _VGT_RENDER_H_
#define _VGT_RENDER_H_

/*
 * Define registers of a ring buffer per hardware register layout.
 */
typedef struct {
	vgt_reg_t tail;
	vgt_reg_t head;
	vgt_reg_t start;
	vgt_reg_t ctl;
} vgt_ringbuffer_t;

/*
 * Ring ID definition.
 */
enum vgt_ring_id {
	RING_ID_INVALID = -1,
	RING_BUFFER_RCS,
	RING_BUFFER_VCS,
	RING_BUFFER_BCS,
	RING_BUFFER_VECS,
	RING_BUFFER_VCS2,
	MAX_ENGINES
};

struct pgt_device;

struct vgt_rsvd_ring {
	struct pgt_device *pdev;
	void *virtual_start;
	int start;
	uint64_t null_context;
	uint64_t indirect_state;
	int id;

	u32 head;
	u32 tail;
	int size;
	/* whether the engine requires special context switch */
	bool	stateless;
	/* whether the engine requires context switch */
	bool	need_switch;
	/* whether the engine end with user interrupt instruction */
	bool	need_irq;
	/* memory offset of the user interrupt instruction */
	u32	ip_offset;
};

#define _tail_reg_(ring_reg_off)	\
		(ring_reg_off & ~(sizeof(vgt_ringbuffer_t)-1))

typedef struct {
	vgt_reg_t base;
	vgt_reg_t cache_ctl;
	vgt_reg_t mode;
} vgt_ring_ppgtt_t;

struct execlist_context;
struct vgt_mm;

enum EL_SLOT_STATUS {
	EL_EMPTY	= 0,
	EL_PENDING,
	EL_SUBMITTED
};

struct vgt_exec_list {
	enum EL_SLOT_STATUS status;
	struct execlist_context *el_ctxs[2];
};

struct vgt_elsp_store {
	uint32_t count;
	uint32_t element[4];
};

#define EL_QUEUE_SLOT_NUM 6
#include "execlists.h"

typedef struct {
	vgt_ringbuffer_t	vring;		/* guest view ring */
	vgt_ringbuffer_t	sring;		/* shadow ring */
	/* In aperture, partitioned & 4KB aligned. */
	/* 64KB alignment requirement for walkaround. */
	uint64_t	context_save_area;	/* VGT default context space */
	uint32_t	active_vm_context;
	/* ppgtt info */
	vgt_ring_ppgtt_t	vring_ppgtt_info; /* guest view */
	vgt_ring_ppgtt_t	sring_ppgtt_info; /* shadow info */
	u8 has_ppgtt_base_set : 1;	/* Is PP dir base set? */
	u8 has_ppgtt_mode_enabled : 1;	/* Is ring's mode reg PPGTT enable set? */
	u8 has_execlist_enabled : 1;
	struct vgt_mm *active_ppgtt_mm;
	int ppgtt_root_pointer_type;
	int ppgtt_page_table_level;

	struct cmd_general_info	patch_list;
	struct cmd_general_info	handler_list;
	struct cmd_general_info	tail_list;

	uint64_t cmd_nr;
	vgt_reg_t	last_scan_head;
	uint64_t request_id;

	vgt_reg_t uhptr;
	uint64_t uhptr_id;
	int el_slots_head;
	int el_slots_tail;
	struct ctx_desc_format el_last_submit[2]; /* 2 ctx of last submit el */
	struct vgt_exec_list execlist_slots[EL_QUEUE_SLOT_NUM];
	struct vgt_elsp_store elsp_store;
	int csb_write_ptr;

	struct execlist_context *el_ctx;
} vgt_state_ring_t;

struct vgt_render_context_ops {
	bool (*init_null_context)(struct pgt_device *pdev, int id);
	bool (*save_hw_context)(int id, struct vgt_device *vgt);
	bool (*restore_hw_context)(int id, struct vgt_device *vgt);
	bool (*ring_context_switch)(struct pgt_device *pdev,
				enum vgt_ring_id ring_id,
				struct vgt_device *prev,
				struct vgt_device *next);
};

struct reg_mask_t {
	u32		reg;
	u8		mask;
	vgt_reg_t	val;
};

bool gen7_ring_switch(struct pgt_device *pdev,
		enum vgt_ring_id ring_id,
		struct vgt_device *prev,
		struct vgt_device *next);

void vgt_restore_ringbuffer(struct vgt_device *vgt, int id);
vgt_reg_t *vgt_get_extra_ctx_regs_gen7(void);
int vgt_get_extra_ctx_regs_num_gen7(void);

#endif /* _VGT_RENDER_H_ */
