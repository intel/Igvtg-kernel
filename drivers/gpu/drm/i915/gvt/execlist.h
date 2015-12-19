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

#ifndef _GVT_EXECLIST_H_
#define _GVT_EXECLIST_H_

struct execlist_ctx_descriptor_format {
	union {
		u32 udw;
		u32 context_id;
	};
	union {
		u32 ldw;
		struct {
			u32 valid                  : 1;
			u32 force_pd_restore       : 1;
			u32 force_restore          : 1;
			u32 addressing_mode        : 2;
			u32 llc_coherency          : 1;
			u32 fault_handling         : 2;
			u32 privilege_access       : 1;
			u32 reserved               : 3;
			u32 lrca                   : 20;
		};
	};
};

struct execlist_status_format {
        union {
                u32 ldw;
                struct {
                        u32 current_execlist_pointer       :1;
                        u32 execlist_write_pointer         :1;
                        u32 execlist_queue_full            :1;
                        u32 execlist_1_valid               :1;
                        u32 execlist_0_valid               :1;
                        u32 last_ctx_switch_reason         :9;
                        u32 current_active_elm_status      :2;
                        u32 arbitration_enable             :1;
                        u32 execlist_1_active              :1;
                        u32 execlist_0_active              :1;
                        u32 reserved                       :13;
                };
        };
        union {
                u32 udw;
                u32 context_id;
        };
};

struct execlist_context_status_pointer_format {
	union {
		u32 dw;
		struct {
			u32 write_ptr              :3;
			u32 reserved               :5;
			u32 read_ptr               :3;
			u32 reserved2              :5;
			u32 mask                   :16;
		};
	};
};

struct execlist_context_status_format {
	union {
		u32 ldw;
		struct {
			u32 idle_to_active         :1;
			u32 preempted              :1;
			u32 element_switch         :1;
			u32 active_to_idle         :1;
			u32 context_complete       :1;
			u32 wait_on_sync_flip      :1;
			u32 wait_on_vblank         :1;
			u32 wait_on_semaphore      :1;
			u32 wait_on_scanline       :1;
			u32 reserved               :2;
			u32 semaphore_wait_mode    :1;
			u32 display_plane          :3;
			u32 lite_restore           :1;
			u32 reserved_2             :16;
		};
	};
	union {
		u32 udw;
		u32 context_id;
	};
};

struct gvt_execlist_mmio_pair {
	u32 addr;
	u32 val;
};

/* The first 52 dwords in register state context */
struct execlist_ring_context {
	u32 nop1;
	u32 lri_cmd_1;
	struct gvt_execlist_mmio_pair ctx_ctrl;
	struct gvt_execlist_mmio_pair ring_header;
	struct gvt_execlist_mmio_pair ring_tail;
	struct gvt_execlist_mmio_pair rb_start;
	struct gvt_execlist_mmio_pair rb_ctrl;
	struct gvt_execlist_mmio_pair bb_cur_head_UDW;
	struct gvt_execlist_mmio_pair bb_cur_head_LDW;
	struct gvt_execlist_mmio_pair bb_state;
	struct gvt_execlist_mmio_pair second_bb_addr_UDW;
	struct gvt_execlist_mmio_pair second_bb_addr_LDW;
	struct gvt_execlist_mmio_pair second_bb_state;
	struct gvt_execlist_mmio_pair bb_per_ctx_ptr;
	struct gvt_execlist_mmio_pair rcs_indirect_ctx;
	struct gvt_execlist_mmio_pair rcs_indirect_ctx_offset;
	u32 nop2;
	u32 nop3;
	u32 nop4;
	u32 lri_cmd_2;
	struct gvt_execlist_mmio_pair ctx_timestamp;
	struct gvt_execlist_mmio_pair pdp3_UDW;
	struct gvt_execlist_mmio_pair pdp3_LDW;
	struct gvt_execlist_mmio_pair pdp2_UDW;
	struct gvt_execlist_mmio_pair pdp2_LDW;
	struct gvt_execlist_mmio_pair pdp1_UDW;
	struct gvt_execlist_mmio_pair pdp1_LDW;
	struct gvt_execlist_mmio_pair pdp0_UDW;
	struct gvt_execlist_mmio_pair pdp0_LDW;
};

struct gvt_execlist_state {
	struct execlist_ctx_descriptor_format ctx[2];
	u32 index;
};

struct gvt_execlist_write_bundle {
	u32 data[4];
	u32 index;
};

struct gvt_virtual_execlist_info {
	struct gvt_execlist_state execlist[2];
	struct gvt_execlist_state *running_execlist;
	struct gvt_execlist_state *pending_execlist;
	struct execlist_ctx_descriptor_format *running_context;
	struct gvt_execlist_write_bundle bundle;
	int ring_id;
	struct vgt_device *vgt;
	struct list_head workload_q_head;
};

bool gvt_init_virtual_execlist_info(struct vgt_device *vgt);

void gvt_get_context_pdp_root_pointer(struct vgt_device *vgt,
		struct execlist_ring_context *ring_context,
		u32 pdp[8]);

void gvt_set_context_pdp_root_pointer(struct vgt_device *vgt,
		struct execlist_ring_context *ring_context,
		u32 pdp[8]);

#endif /*_GVT_EXECLIST_H_*/
