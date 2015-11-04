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

#ifndef _GVT_MMIO_H_
#define _GVT_MMIO_H_

#include "i915_drv.h"

/* reg contains address, requiring fix */
#define GVT_REG_ADDR_FIX	(1 << 5)
/* Status bit updated from HW */
#define GVT_REG_HW_STATUS	(1 << 6)
/* Virtualized */
#define GVT_REG_VIRT		(1 << 7)
/* Mode ctl registers with high 16 bits as the mask bits */
#define GVT_REG_MODE_CTL	(1 << 8)
/* This reg has been tracked in vgt_base_reg_info */
#define GVT_REG_TRACKED		(1 << 10)
/* This reg has been accessed by a VM */
#define GVT_REG_ACCESSED	(1 << 11)
/* This reg is saved/restored at context switch time */
#define GVT_REG_SAVED		(1 << 12)
/* Accessed through GPU commands */
#define GVT_REG_CMD_ACCESS	(1 << 14)
/* index into another auxillary table. Maximum 256 entries now */
#define GVT_REG_INDEX_SHIFT	16
#define GVT_REG_INDEX_MASK	(0xFFFF << GVT_REG_INDEX_SHIFT)

#define GVT_AUX_TABLE_NUM	256

#define F_VIRT			GVT_REG_VIRT

#define F_DOM0			F_VIRT
#define F_RDR			F_VIRT
#define F_RDR_ADRFIX		F_VIRT
#define F_RDR_HWSTS		F_VIRT
#define F_RDR_MODE		F_VIRT
#define F_DPY			F_VIRT
#define F_DPY_ADRFIX		F_VIRT
#define F_DPY_HWSTS_ADRFIX	F_VIRT
#define F_PT			F_VIRT

/* suppose a reg won't set both bits */
typedef union {
	struct {
		u32 mask;
	} mode_ctl;
	struct {
		u32 mask;
		uint32_t  size;
	} addr_fix;
} gvt_aux_entry_t;

struct vgt_device;

typedef bool (*gvt_mmio_handler_t)(struct vgt_device *, unsigned int, void *, unsigned int);

struct gvt_mmio_entry {
	struct hlist_node hlist;
	unsigned int base;
	unsigned int align_bytes;
	gvt_mmio_handler_t read;
	gvt_mmio_handler_t write;
};

struct gvt_reg_info {
	u32 reg;
	u32 size;
	u32 flags;
	u32 addr_mask;
	u32 device;
	gvt_mmio_handler_t read;
	gvt_mmio_handler_t write;
};

struct pgt_device;

extern struct gvt_reg_info gvt_general_reg_info[];
extern struct gvt_reg_info gvt_broadwell_reg_info[];
extern int gvt_get_reg_num(int type);
extern unsigned int gvt_get_device_type(struct pgt_device *pdev);

bool gvt_emulate_mmio_read(struct vgt_device *vgt, uint64_t pa, void *p_data,int bytes);
bool gvt_emulate_mmio_write(struct vgt_device *vgt, uint64_t pa, void *p_data,int bytes);
#endif
