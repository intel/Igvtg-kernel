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

#ifndef __GVT_SCHED_POLICY__
#define __GVT_SCHED_POLICY__

struct gvt_schedule_policy_ops {
	bool (*init)(struct pgt_device *pdev);
	void (*clean)(struct pgt_device *pdev);
	bool (*instance_init)(struct vgt_device *vgt);
	void (*instance_clean)(struct vgt_device *vgt);
	void (*start_schedule)(struct vgt_device *vgt);
	void (*stop_schedule)(struct vgt_device *vgt);
};

bool gvt_init_sched_policy(struct pgt_device *pdev);

void gvt_clean_sched_policy(struct pgt_device *pdev);

bool gvt_init_instance_sched_policy(struct vgt_device *vgt);

void gvt_clean_instance_sched_policy(struct vgt_device *vgt);

void gvt_start_schedule(struct vgt_device *vgt);

void gvt_stop_schedule(struct vgt_device *vgt);

#endif
