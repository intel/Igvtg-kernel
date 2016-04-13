/*
 * vGT perf header
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

#ifndef _VGT_PERF_H_
#define _VGT_PERF_H_

struct vgt_mmio_accounting_reg_stat {
	u64 r_count;
	u64 r_cycles;
	u64 w_count;
	u64 w_cycles;
};

struct vgt_statistics {
	u64	schedule_in_time;	/* TSC time when it is last scheduled in */
	u64	allocated_cycles;
	u64	used_cycles;
	u64	irq_num;
	u64	events[EVENT_MAX];
	u64	irq_inject_fail;

	/* actually this is the number of pending
	* interrutps, check this in vgt_check_pending_events,
	* one injection can deliver more than one events
	*/
	u64	pending_events;
	u64	last_propagation;
	u64	last_blocked_propagation;
	u64	last_injection;

	/* mmio statistics */
	u64	gtt_mmio_rcnt;
	u64	gtt_mmio_wcnt;
	u64	gtt_mmio_wcycles;
	u64	gtt_mmio_rcycles;
	u64	mmio_rcnt;
	u64	mmio_wcnt;
	u64	mmio_wcycles;
	u64	mmio_rcycles;
	u64	ring_mmio_rcnt;
	u64	ring_mmio_wcnt;
	u64	ring_tail_mmio_wcnt;
	u64	ring_tail_mmio_wcycles;
	u64	vring_scan_cnt;
	u64	vring_scan_cycles;
	u64	wp_cnt;
	u64	wp_cycles;
	u64	ppgtt_wp_cnt;
	u64	ppgtt_wp_cycles;
	u64	spt_find_hit_cnt;
	u64	spt_find_hit_cycles;
	u64	spt_find_miss_cnt;
	u64	spt_find_miss_cycles;
	u64	gpt_find_hit_cnt;
	u64	gpt_find_hit_cycles;
	u64	gpt_find_miss_cnt;
	u64	gpt_find_miss_cycles;
	u64	skip_bb_cnt;

	struct vgt_mmio_accounting_reg_stat *mmio_accounting_reg_stats;
	bool mmio_accounting;
	struct mutex mmio_accounting_lock;
};

struct pgt_statistics {
	u64	irq_num;
	u64	last_pirq;
	u64	last_virq;
	u64	pirq_cycles;
	u64	virq_cycles;
	u64	irq_delay_cycles;
	u64	events[EVENT_MAX];
	u64	oos_page_cur_avail_cnt;
	u64	oos_page_min_avail_cnt;
	u64	oos_page_steal_cnt;
	u64	oos_page_attach_cnt;
	u64	oos_page_detach_cnt;
	u64	context_switch_cost;
	u64	context_switch_num;
	u64	ring_idle_wait;
	u64	ring_0_idle;
	u64	ring_0_busy;
};

typedef struct {
	char *node_name;
	u64 *stat;
} debug_statistics_t;

#endif /* _VGT_PERF_H_ */
