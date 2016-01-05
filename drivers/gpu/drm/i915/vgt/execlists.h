/*
 * EXECLIST data structures
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

#ifndef _VGT_EXECLISTS_H_
#define _VGT_EXECLISTS_H_

#define vgt_require_shadow_context(vgt)	(!((vgt) && (vgt->vgt_id == 0)))

#define MAX_EXECLIST_CTX_PAGES	22
#define ELSP_BUNDLE_NUM		4
#define EXECLIST_CTX_SIZE (SIZE_PAGE * MAX_EXECLIST_CTX_PAGES)

#define CTX_OFFSET_PDP3		0x24
#define CTX_OFFSET_PDP2		0x28
#define CTX_OFFSET_PDP1		0x2c
#define CTX_OFFSET_PDP0		0x30

#define CTX_STATUS_BUF_NUM	6
#define DEFAULT_INV_SR_PTR	7

/* NORMAL_CTX_SHADOW will have guest contexts write protected
 * after creating. Other policies only have write protection
 * after the context submission.
 */
enum ctx_shadow_policy {
	PATCH_WITHOUT_SHADOW	= 0,
	NORMAL_CTX_SHADOW	= 1,
};

struct mmio_pair {
	uint32_t addr;
	uint32_t val;
};

/* The first 64 dwords in register state context */
struct reg_state_ctx_header {
	uint32_t nop1;
	uint32_t lri_cmd_1;
	struct mmio_pair ctx_ctrl;
	struct mmio_pair ring_header;
	struct mmio_pair ring_tail;
	struct mmio_pair rb_start;
	struct mmio_pair rb_ctrl;
	struct mmio_pair bb_cur_head_UDW;
	struct mmio_pair bb_cur_head_LDW;
	struct mmio_pair bb_state;
	struct mmio_pair second_bb_addr_UDW;
	struct mmio_pair second_bb_addr_LDW;
	struct mmio_pair second_bb_state;
	struct mmio_pair bb_per_ctx_ptr;
	struct mmio_pair rcs_indirect_ctx;
	struct mmio_pair rcs_indirect_ctx_offset;
	uint32_t nop2;
	uint32_t nop3;
	uint32_t nop4;
	uint32_t lri_cmd_2;
	struct mmio_pair ctx_timestamp;
	struct mmio_pair pdp3_UDW;
	struct mmio_pair pdp3_LDW;
	struct mmio_pair pdp2_UDW;
	struct mmio_pair pdp2_LDW;
	struct mmio_pair pdp1_UDW;
	struct mmio_pair pdp1_LDW;
	struct mmio_pair pdp0_UDW;
	struct mmio_pair pdp0_LDW;
	uint32_t nops[12];
};

struct ctx_desc_format {
	union {
		uint32_t elm_high;
		uint32_t context_id;
	};
	union {
		uint32_t elm_low;
		struct {
			uint32_t valid			: 1;
			uint32_t force_pd_restore	: 1;
			uint32_t force_restore		: 1;
			uint32_t addressing_mode	: 2;
			uint32_t llc_coherency		: 1;
			uint32_t fault_handling		: 2;
			uint32_t privilege_access	: 1;
			uint32_t reserved		: 3;
			uint32_t lrca			: 20;
		};
	};
};

struct execlist_status_format {
	union {
		uint32_t ldw;
		struct {
			uint32_t current_execlist_pointer	:1;
			uint32_t execlist_write_pointer		:1;
			uint32_t execlist_queue_full		:1;
			uint32_t execlist_1_valid		:1;
			uint32_t execlist_0_valid		:1;
			uint32_t last_ctx_switch_reason		:9;
			uint32_t current_active_elm_status	:2;
			uint32_t arbitration_enable		:1;
			uint32_t execlist_1_active		:1;
			uint32_t execlist_0_active		:1;
			uint32_t reserved			:13;
		};
	};	
	union {
		uint32_t udw;
		uint32_t context_id;
	};
};

struct context_status_format {
	union {
		uint32_t ldw;
		struct {
			uint32_t idle_to_active		:1;
			uint32_t preempted		:1;
			uint32_t element_switch		:1;
			uint32_t active_to_idle		:1;
			uint32_t context_complete	:1;
			uint32_t wait_on_sync_flip	:1;
			uint32_t wait_on_vblank		:1;
			uint32_t wait_on_semaphore	:1;
			uint32_t wait_on_scanline	:1;
			uint32_t reserved		:2;
			uint32_t semaphore_wait_mode	:1;
			uint32_t display_plane		:3;
			uint32_t lite_restore		:1;
			uint32_t reserved_2		:16;
		};
	};
	union {
		uint32_t udw;
		uint32_t context_id;
	};
};

struct ctx_st_ptr_format {
	union {
		uint32_t dw;
		struct {
			uint32_t status_buf_write_ptr	:3;
			uint32_t reserved		:5;
			uint32_t status_buf_read_ptr	:3;
			uint32_t reserved2		:5;
			uint32_t mask			:16;
		};
	};
};

/* shadow context */

struct shadow_ctx_page {
	guest_page_t guest_page;
	shadow_page_t shadow_page;
	struct vgt_device *vgt;
};

struct execlist_context {
	struct ctx_desc_format guest_context;
	uint32_t shadow_lrca;
	uint32_t error_reported;
	enum vgt_ring_id ring_id;
	/* below are some per-ringbuffer data. Since with execlist,
	 * each context has its own ring buffer, here we store the
	 * data and store them into vgt->rb[ring_id] before a
	 * context is submitted. We will have better handling later.
	 */
	vgt_reg_t last_guest_head;
	vgt_reg_t last_scan_head;
	bool scan_head_valid;
	uint64_t request_id;
	//uint64_t cmd_nr;
	//vgt_reg_t uhptr;
	//uint64_t uhptr_id;

	struct vgt_mm *ppgtt_mm;
	struct shadow_ctx_page ctx_pages[MAX_EXECLIST_CTX_PAGES];
	/* used for lazy context shadowing optimization */
	gtt_entry_t shadow_entry_backup[MAX_EXECLIST_CTX_PAGES];

	struct hlist_node node;
};

/* read execlist status or ctx status which are 64-bit MMIO
 * status can be different types but all with ldw/udw defined.
 */
#define READ_STATUS_MMIO(pdev, offset, status)		\
do {							\
	status.ldw = VGT_MMIO_READ(pdev, offset);	\
	status.udw = VGT_MMIO_READ(pdev, offset + 4);	\
} while(0);

#endif /* _VGT_EXECLISTS_H_ */
