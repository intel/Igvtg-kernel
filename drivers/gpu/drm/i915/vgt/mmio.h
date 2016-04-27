/*
 * vGT mmio header
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

#ifndef _VGT_MMIO_H_
#define _VGT_MMIO_H_

/*
 * Below are some wrappers for commonly used policy flags.
 * Add on demand to feed your requirement
 */
/* virtualized */
#define F_VIRT			VGT_OT_NONE | VGT_REG_VIRT

/*
 * config context (global setting, pm, workaround, etc.)
 * 	- config owner access pReg
 *      - non-config owner access vReg
 * (dom0 is the unique config owner)
 */
#define F_DOM0			VGT_OT_CONFIG

/*
 * render context
 *	- render owner access pReg
 *	- non-render owner access vReg
 */
#define F_RDR			VGT_OT_RENDER
/* render context, require address fix */
#define F_RDR_ADRFIX		F_RDR | VGT_REG_ADDR_FIX
/* render context, status updated by hw */
#define F_RDR_HWSTS		F_RDR | VGT_REG_HW_STATUS
/* render context, mode register (high 16 bits as write mask) */
#define F_RDR_MODE		F_RDR | VGT_REG_MODE_CTL
/*
 * display context
 *	- display owner access pReg
 *	- non-display owner access vReg
 */
#define F_DPY			VGT_OT_DISPLAY
/* display context, require address fix */
#define F_DPY_ADRFIX		F_DPY | VGT_REG_ADDR_FIX
/* display context, require address fix, status updated by hw */
#define F_DPY_HWSTS_ADRFIX	F_DPY_ADRFIX | VGT_REG_HW_STATUS

/*
 * passthrough reg (DANGEROUS!)
 *	- any VM directly access pReg
 *	- no save/restore
 *	- dangerous as a workaround only
 */
#define F_PT			VGT_OT_NONE | VGT_REG_PASSTHROUGH

/*
 * read only pass through registers, not allowed write but allow guest read HW
 */
#define F_PT_RO			(VGT_OT_NONE | VGT_REG_PT_READONLY)

struct vgt_device;

typedef bool (*vgt_mmio_read)(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes);
typedef bool (*vgt_mmio_write)(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes);

struct vgt_mmio_entry {
	struct hlist_node hlist;
	unsigned int base;
	unsigned int align_bytes;
	vgt_mmio_read	read;
	vgt_mmio_write	write;
};

enum vgt_owner_type {
	VGT_OT_NONE = 0,		// No owner type
	VGT_OT_RENDER,			// the owner directly operating all render buffers (render/blit/video)
	VGT_OT_DISPLAY,			// the owner having its content directly shown on one or several displays
	VGT_OT_CONFIG,			// the owner is always dom0 (PM, workarounds, etc.)
	VGT_OT_MAX,
};

/* owner type of the reg, up to 16 owner type */
#define VGT_REG_OWNER		(0xF)
/*
 * TODO:
 * Allows pReg access from any VM but w/o save/restore,
 * since we don't know the actual bit detail or virtualization
 * policy yet. the examples include many workaround registers.
 * regs marked with this flag should be cleared before final
 * release, since this way is unsafe.
 */
#define VGT_REG_PASSTHROUGH	(1 << 4)
/* reg contains address, requiring fix */
#define VGT_REG_ADDR_FIX	(1 << 5)
/* Status bit updated from HW */
#define VGT_REG_HW_STATUS	(1 << 6)
/* Virtualized */
#define VGT_REG_VIRT		(1 << 7)
/* Mode ctl registers with high 16 bits as the mask bits */
#define VGT_REG_MODE_CTL	(1 << 8)
/* VMs have different settings on this reg */
#define VGT_REG_NEED_SWITCH	(1 << 9)
/* This reg has been tracked in vgt_base_reg_info */
#define VGT_REG_TRACKED		(1 << 10)
/* This reg has been accessed by a VM */
#define VGT_REG_ACCESSED	(1 << 11)
/* This reg is saved/restored at context switch time */
#define VGT_REG_SAVED		(1 << 12)
/* Policies not impacted by the superowner mode */
#define VGT_REG_STICKY		(1 << 13)
/* Accessed through GPU commands */
#define VGT_REG_CMD_ACCESS	(1 << 14)
/* read only pass through register */
#define VGT_REG_PT_READONLY    (1 << 15)
/* index into another auxillary table. Maximum 256 entries now */
#define VGT_REG_INDEX_SHIFT	16
#define VGT_REG_INDEX_MASK	(0xFFFF << VGT_REG_INDEX_SHIFT)

#define VGT_AUX_TABLE_NUM	256
/* suppose a reg won't set both bits */
typedef union {
	struct {
		vgt_reg_t mask;
	} mode_ctl;
	struct {
		vgt_reg_t mask;
		uint32_t  size;
	} addr_fix;
} vgt_aux_entry_t;

typedef struct {
	u32			reg;
	int			size;
	u32			flags;
	vgt_reg_t		addr_mask;
	int			device;
	vgt_mmio_read		read;
	vgt_mmio_write		write;
} reg_attr_t;

typedef struct {
	u32			reg;
	int			size;
} reg_list_t;

#endif /* _VGT_MMIO_H_ */
