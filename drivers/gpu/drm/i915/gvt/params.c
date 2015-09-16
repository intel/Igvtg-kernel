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

#include "gvt.h"

struct gvt_kernel_params gvt = {
	.enable = true,
	.debug = 0,
	.dom0_low_gm_sz = 96,
	.dom0_high_gm_sz = 384,
	.dom0_fence_sz = 4,
};

module_param_named(dom0_low_gm_sz, gvt.dom0_low_gm_sz, int, 0600);
MODULE_PARM_DESC(dom0_low_gm_sz, "Amount of aperture size of DOM0");

module_param_named(dom0_high_gm_sz, gvt.dom0_high_gm_sz, int, 0600);
MODULE_PARM_DESC(dom0_high_gm_sz, "Amount of high memory size of DOM0");

module_param_named(dom0_fence_sz, gvt.dom0_fence_sz, int, 0600);
MODULE_PARM_DESC(dom0_fence_sz, "Amount of fence size of DOM0");
