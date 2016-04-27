/*
 * vGT scheduler header
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

#ifndef _VGT_SCHED_H_
#define _VGT_SCHED_H_

typedef cycles_t vgt_tslice_t;
struct vgt_sched_info {
	vgt_tslice_t start_time;
	vgt_tslice_t end_time;
	vgt_tslice_t actual_end_time;
	vgt_tslice_t rb_empty_delay;	/* cost for "wait rendering engines empty */

	int32_t priority;
	int32_t weight;
	int64_t time_slice;
	/* more properties and policies should be added in*/
	u64 tbs_period;  /* default: VGT_TBS_DEFAULT_PERIOD(1ms) */
};

struct vgt_hrtimer {
	struct hrtimer timer;
};

#define VGT_TBS_PERIOD_MAX 15
#define VGT_TBS_PERIOD_MIN 1
#define VGT_TBS_DEFAULT_PERIOD(x) ((x) * 1000000) /* 15 ms */

#endif /*_VGT_SCHED_H_*/
