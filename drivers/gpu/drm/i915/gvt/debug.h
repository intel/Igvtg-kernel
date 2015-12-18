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

#ifndef __GVT_DEBUG_H__
#define __GVT_DEBUG_H__

#define ASSERT(x)                                                       \
        do {                                                            \
                if (!(x)) {                                             \
                        printk("Assert at %s line %d\n",                \
                                __FILE__, __LINE__);                    \
                }                                                       \
        } while (0);

#define ASSERT_NUM(x, y)                                                \
        do {                                                            \
                if (!(x)) {                                             \
                        printk("Assert at %s line %d para 0x%llx\n",    \
                                __FILE__, __LINE__, (u64)y);            \
                }                                                       \
        } while (0);

#define ASSERT_VM(x, vgt)                                              \
        do {                                                            \
                if (!(x)) {                                             \
                        printk("Assert at %s line %d\n",                \
                                        __FILE__, __LINE__);                    \
                        if (atomic_cmpxchg(&(vgt)->crashing, 0, 1))     \
                        break;                                  \
                        gvt_warn("Killing VM%d", (vgt)->vm_id);       \
                        if (!hypervisor_pause_domain((vgt)))            \
                        hypervisor_shutdown_domain((vgt));      \
                }                                                       \
        } while (0)

#define gvt_info(fmt, args...) \
	printk(KERN_INFO"[GVT-g] "fmt"\n", ##args)

#define gvt_err(fmt, args...) \
	printk(KERN_ERR"%s() - %d: "fmt"\n", __func__, __LINE__, ##args)

#define gvt_warn(fmt, args...) \
	printk(KERN_WARNING"%s() - %d: "fmt"\n", __func__, __LINE__, ##args)

#define gvt_dbg(level, fmt, args...) do { \
		if (gvt.debug & level) \
			printk(KERN_DEBUG"%s() - %d: "fmt"\n", __func__, __LINE__, ##args); \
	}while(0)

enum {
	GVT_DBG_CORE = (1 << 0),
	GVT_DBG_MM = (1 << 1),
	GVT_DBG_IRQ = (1 << 2),
	GVT_DBG_DPY = (1 << 3),
	GVT_DBG_RENDER = (1 << 4),
	GVT_DBG_EDID = (1 << 5),
	GVT_DBG_EL = (1 << 6),
};

#define gvt_dbg_core(fmt, args...) \
	gvt_dbg(GVT_DBG_CORE, fmt, ##args)

#define gvt_dbg_mm(fmt, args...) \
	gvt_dbg(GVT_DBG_MM, fmt, ##args)

#define gvt_dbg_irq(fmt, args...) \
	gvt_dbg(GVT_DBG_IRQ, fmt, ##args)

#define gvt_dbg_el(fmt, args...) \
	gvt_dbg(GVT_DBG_EL, fmt, ##args)

#endif
