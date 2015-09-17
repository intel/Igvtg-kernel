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

#ifndef _GVT_PERF_H_
#define _GVT_PERF_H_

struct gvt_statistics {
	u64	irq_num;
	u64	events[GVT_EVENT_MAX];
	u64	last_injection;
	u64	mmio_rcnt;
	u64	mmio_wcnt;
	u64	mmio_wcycles;
	u64	mmio_rcycles;
	u64	gtt_mmio_rcnt;
	u64	gtt_mmio_wcnt;
	u64	gtt_mmio_wcycles;
	u64	gtt_mmio_rcycles;
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

};

struct pgt_statistics {
	u64	irq_num;
	u64	irq_delay_cycles;
	u64	events[GVT_EVENT_MAX];
	u64	oos_page_cur_avail_cnt;
	u64	oos_page_min_avail_cnt;
	u64	oos_page_steal_cnt;
	u64	oos_page_attach_cnt;
	u64	oos_page_detach_cnt;

};

#endif
