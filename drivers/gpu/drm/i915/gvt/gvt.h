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

#ifndef _GVT_H_
#define _GVT_H_

#include "i915_drv.h"
#include "i915_vgpu.h"

#include "debug.h"
#include "params.h"
#include "reg.h"
#include "hypercall.h"
#include "fb_decoder.h"
#include "mmio.h"
#include "interrupt.h"
#include "perf.h"
#include "gtt.h"

#define GVT_MAX_VGPU 8

#define GVT_MAX_GM_SIZE		(1UL << 32)
#define GVT_GM_BITMAP_BITS	(GVT_MAX_GM_SIZE >> 20)
#define GVT_MAX_NUM_FENCES	32
#define GVT_FENCE_BITMAP_BITS	GVT_MAX_NUM_FENCES

#define GVT_HASH_BITS 9

enum {
	GVT_HYPERVISOR_TYPE_XEN = 0,
	GVT_HYPERVISOR_TYPE_KVM,
};

struct gvt_io_emulation_ops {
	bool (*emulate_mmio_read)(struct vgt_device *, uint64_t, void *, int);
	bool (*emulate_mmio_write)(struct vgt_device *, uint64_t, void *, int);
	bool (*emulate_cfg_read)(struct vgt_device *, unsigned int, void *, int);
	bool (*emulate_cfg_write)(struct vgt_device *, unsigned int, void *, int);
};

struct gvt_host {
	bool initialized;
	int hypervisor_type;
	struct mutex device_idr_lock;
	struct idr device_idr;
	struct gvt_kernel_dm *kdm;
	struct gvt_io_emulation_ops *emulate_ops;
};

extern struct gvt_host gvt_host;

/* Describe the limitation of HW.*/
struct gvt_device_info {
	u64 max_gtt_gm_sz;
	u32 gtt_start_offset;
	u32 gtt_end_offset;
	u32 max_gtt_size;
	u32 gtt_entry_size;
	u32 gtt_entry_size_shift;
	u32 gmadr_bytes_in_cmd;
};

struct gvt_gm_node {
	struct drm_mm_node *low_gm_node;
	struct drm_mm_node *high_gm_node;
};

struct gvt_virtual_mmio_state {
	void *vreg;
	void *sreg;
};

struct gvt_virtual_cfg_state {
	unsigned char space[GVT_CFG_SPACE_SZ];
	bool bar_mapped[GVT_BAR_NUM];
	u64 bar_size[GVT_BAR_NUM];
};

struct gvt_virtual_gm_state {
	u64 aperture_base;
	void *aperture_base_va;
	u64 aperture_sz;
	u64 gm_sz;
	u64 aperture_offset;        /* address fix for visible GM */
	u64 hidden_gm_offset;       /* address fix for invisible GM */
	int fence_base;
	int fence_sz;
	struct gvt_gm_node node;
};

struct gvt_virtual_device_state {
	struct gvt_virtual_gm_state gm;
	struct gvt_virtual_mmio_state mmio;
	struct gvt_virtual_cfg_state cfg;
};

struct vgt_device {
	int id;
	int vm_id;
	struct pgt_device *pdev;
	bool warn_untrack;
	atomic_t active;
	struct gvt_virtual_device_state state;
	struct gvt_statistics stat;
	struct gvt_vgtt_info gtt;
};

struct gvt_gm_allocator {
	struct drm_mm low_gm;
	struct drm_mm high_gm;
};

struct pgt_device {
	struct mutex lock;
	int id;

	struct drm_i915_private *dev_priv;
	struct idr instance_idr;

	struct gvt_device_info device_info;

	u8 initial_cfg_space[GVT_CFG_SPACE_SZ];
	u64 bar_size[GVT_BAR_NUM];

	u64 gttmmio_base;
	void *gttmmio_va;

	u64 gmadr_base;
	void *gmadr_va;

	u32 mmio_size;
	u32 reg_num;

	wait_queue_head_t service_thread_wq;
	struct task_struct *service_thread;
	unsigned long service_request;

	/* 1 bit corresponds to 1MB in the GM space */
	DECLARE_BITMAP(gm_bitmap, GVT_GM_BITMAP_BITS);

	/* 1 bit corresponds to 1 fence register */
	DECLARE_BITMAP(fence_bitmap, GVT_FENCE_BITMAP_BITS);

	u64 total_gm_sz;

	struct gvt_gm_allocator gm_allocator;

	u32 *initial_mmio_state;
	u32 *reg_info;

	gvt_aux_entry_t aux_table[GVT_AUX_TABLE_NUM];
	u32 aux_table_index;

	DECLARE_HASHTABLE(mmio_table, GVT_HASH_BITS);

	struct gvt_irq_state irq_state;
	struct pgt_statistics stat;

	struct gvt_gtt_info gtt;
};

/* definitions for physical aperture/GM space */
#define phys_aperture_sz(pdev)          (pdev->bar_size[1])
#define phys_aperture_pages(pdev)       (phys_aperture_sz(pdev) >> GTT_PAGE_SHIFT)
#define phys_aperture_base(pdev)        (pdev->gmadr_base)
#define phys_aperture_vbase(pdev)       (pdev->gmadr_va)

#define gm_sz(pdev)                     (pdev->total_gm_sz)
#define gm_base(pdev)                   (0ULL)
#define gm_pages(pdev)                  (gm_sz(pdev) >> GTT_PAGE_SHIFT)
#define hidden_gm_base(pdev)            (phys_aperture_sz(pdev))

#define aperture_2_gm(pdev, addr)       (addr - phys_aperture_base(pdev))
#define v_aperture(pdev, addr)          (phys_aperture_vbase(pdev) + (addr))

/* definitions for vgt's aperture/gm space */
#define gvt_aperture_base(vgt)		(vgt->state.gm.aperture_base)
#define gvt_aperture_vbase(vgt)		(vgt->state.gm.aperture_base_va)
#define gvt_aperture_offset(vgt)	(vgt->state.gm.aperture_offset)
#define gvt_hidden_gm_offset(vgt)	(vgt->state.gm.hidden_gm_offset)
#define gvt_aperture_sz(vgt)		(vgt->state.gm.aperture_sz)
#define gvt_gm_sz(vgt)			(vgt->state.gm.gm_sz)
#define gvt_hidden_gm_sz(vgt)		(gvt_gm_sz(vgt) - gvt_aperture_sz(vgt))
#define gvt_fence_base(vgt)		(vgt->state.gm.fence_base)
#define gvt_fence_sz(vgt)		(vgt->state.gm.fence_sz)

#define gvt_aperture_end(vgt)           \
	(gvt_aperture_base(vgt) + gvt_aperture_sz(vgt) - 1)
#define gvt_visible_gm_base(vgt)        \
	(gm_base(vgt->pdev) + gvt_aperture_offset(vgt))
#define gvt_visible_gm_end(vgt)         \
	(gvt_visible_gm_base(vgt) + gvt_aperture_sz(vgt) - 1)
#define gvt_hidden_gm_base(vgt) \
	(gm_base(vgt->pdev) + gvt_hidden_gm_offset(vgt))
#define gvt_hidden_gm_end(vgt)          \
	(gvt_hidden_gm_base(vgt) + gvt_hidden_gm_sz(vgt) - 1)

/*
 * the view of the aperture/gm space from the VM's p.o.v
 *
 * when the VM supports ballooning, this view is the same as the
 * view of vGT driver.
 *
 * when the VM does not support ballooning, this view starts from
 * GM space ZERO
 */
#define gvt_guest_aperture_base(vgt)    \
	((*((u32*)&vgt->state.cfg.space[GVT_REG_CFG_SPACE_BAR1]) & ~0xf) + gvt_aperture_offset(vgt))
#define gvt_guest_aperture_end(vgt)     \
        (gvt_guest_aperture_base(vgt) + gvt_aperture_sz(vgt) - 1)
#define gvt_guest_visible_gm_base(vgt)  \
        (gvt_visible_gm_base(vgt))
#define gvt_guest_visible_gm_end(vgt)   \
        (gvt_guest_visible_gm_base(vgt) + gvt_aperture_sz(vgt) - 1)
#define gvt_guest_hidden_gm_base(vgt)   \
	gvt_hidden_gm_base(vgt)
#define gvt_guest_hidden_gm_end(vgt)    \
        (gvt_guest_hidden_gm_base(vgt) + gvt_hidden_gm_sz(vgt) - 1)

extern void gvt_init_resource_allocator(struct pgt_device *pdev);
extern void gvt_clean_resource_allocator(struct pgt_device *pdev);
extern int gvt_alloc_gm_and_fence_resource(struct vgt_device *vgt,
		struct gvt_instance_info *info);
extern void gvt_free_gm_and_fence_resource(struct vgt_device *vgt);

#define REG_INDEX(reg) ((reg) >> 2)

#define D_SNB   (1 << 0)
#define D_IVB   (1 << 1)
#define D_HSW   (1 << 2)
#define D_BDW   (1 << 3)

#define D_GEN8PLUS      (D_BDW)
#define D_GEN75PLUS     (D_HSW | D_BDW)
#define D_GEN7PLUS      (D_IVB | D_HSW | D_BDW)

#define D_BDW_PLUS      (D_BDW)
#define D_HSW_PLUS      (D_HSW | D_BDW)
#define D_IVB_PLUS      (D_IVB | D_HSW | D_BDW)

#define D_PRE_BDW       (D_SNB | D_IVB | D_HSW)

#define D_ALL           (D_SNB | D_IVB | D_HSW | D_BDW)

#define reg_addr_fix(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & GVT_REG_ADDR_FIX)
#define reg_hw_status(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & GVT_REG_HW_STATUS)
#define reg_virt(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & GVT_REG_VIRT)
#define reg_mode_ctl(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & GVT_REG_MODE_CTL)
#define reg_is_tracked(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & GVT_REG_TRACKED)
#define reg_is_accessed(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & GVT_REG_ACCESSED)
#define reg_is_saved(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & GVT_REG_SAVED)

#define reg_aux_index(pdev, reg)	\
	((pdev->reg_info[REG_INDEX(reg)] & GVT_REG_INDEX_MASK) >> GVT_REG_INDEX_SHIFT)
#define reg_has_aux_info(pdev, reg)	(reg_mode_ctl(pdev, reg) | reg_addr_fix(pdev, reg))
#define reg_aux_mode_mask(pdev, reg)	\
	(pdev->aux_table[reg_aux_index(pdev, reg)].mode_ctl.mask)
#define reg_aux_addr_mask(pdev, reg)	\
	(pdev->aux_table[reg_aux_index(pdev, reg)].addr_fix.mask)
#define reg_aux_addr_size(pdev, reg)	\
	(pdev->aux_table[reg_aux_index(pdev, reg)].addr_fix.size)

static inline void reg_set_hw_status(struct pgt_device *pdev, u32 reg)
{
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] |= GVT_REG_HW_STATUS;
}

static inline void reg_set_virt(struct pgt_device *pdev, u32 reg)
{
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] |= GVT_REG_VIRT;
}

/* mask bits for addr fix */
static inline void reg_set_addr_fix(struct pgt_device *pdev,
		u32 reg, u32 mask)
{
	ASSERT(!reg_has_aux_info(pdev, reg));
	ASSERT(pdev->aux_table_index <= GVT_AUX_TABLE_NUM - 1);
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);

	pdev->aux_table[pdev->aux_table_index].addr_fix.mask = mask;
	pdev->reg_info[REG_INDEX(reg)] |= GVT_REG_ADDR_FIX |
		(pdev->aux_table_index << GVT_REG_INDEX_SHIFT);
	pdev->aux_table_index++;
}

/* mask bits for mode mask */
static inline void reg_set_mode_ctl(struct pgt_device *pdev,
		u32 reg)
{
	ASSERT(!reg_has_aux_info(pdev, reg));
	ASSERT(pdev->aux_table_index <= GVT_AUX_TABLE_NUM - 1);
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);

	pdev->reg_info[REG_INDEX(reg)] |= GVT_REG_MODE_CTL |
		(pdev->aux_table_index << GVT_REG_INDEX_SHIFT);
	pdev->aux_table_index++;
}

static inline void reg_set_tracked(struct pgt_device *pdev,
		u32 reg)
{
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] |= GVT_REG_TRACKED;
}

static inline void reg_set_accessed(struct pgt_device *pdev,
		u32 reg)
{
	pdev->reg_info[REG_INDEX(reg)] |= GVT_REG_ACCESSED;
}

static inline void reg_set_saved(struct pgt_device *pdev,
		u32 reg)
{
	pdev->reg_info[REG_INDEX(reg)] |= GVT_REG_SAVED;
}

static inline void reg_set_cmd_access(struct pgt_device *pdev,
		u32 reg)
{
	pdev->reg_info[REG_INDEX(reg)] |= GVT_REG_CMD_ACCESS;
	reg_set_accessed(pdev, reg);
}

static inline u32 gvt_mmio_read(struct pgt_device *pdev,
		u32 reg)
{
	struct drm_i915_private *dev_priv = pdev->dev_priv;
	i915_reg_t tmp = {.reg = reg};
	return I915_READ(tmp);
}

static inline void gvt_mmio_write(struct pgt_device *pdev,
		u32 reg, u32 val)
{
	struct drm_i915_private *dev_priv = pdev->dev_priv;
	i915_reg_t tmp = {.reg = reg};
	I915_WRITE(tmp, val);
}

static inline u64 gvt_mmio_read64(struct pgt_device *pdev,
		u32 reg)
{
	struct drm_i915_private *dev_priv = pdev->dev_priv;
	i915_reg_t tmp = {.reg = reg};
	return I915_READ64(tmp);
}

static inline void gvt_mmio_write64(struct pgt_device *pdev,
		u32 reg, u64 val)
{
	struct drm_i915_private *dev_priv = pdev->dev_priv;
	i915_reg_t tmp = {.reg = reg};
	I915_WRITE64(tmp, val);
}

static inline u64 gvt_read_gtt64(struct pgt_device *pdev, u32 index)
{
	struct gvt_device_info *info = &pdev->device_info;
	unsigned int off = index << info->gtt_entry_size_shift;

	return readq(pdev->gttmmio_va + info->gtt_start_offset + off);
}

static inline void gvt_write_gtt64(struct pgt_device *pdev, u32 index, u64 val)
{
	struct gvt_device_info *info = &pdev->device_info;
	unsigned int off = index << info->gtt_entry_size_shift;

	writeq(val, pdev->gttmmio_va + info->gtt_start_offset + off);
}

static inline void gvt_mmio_posting_read(struct pgt_device *pdev, u32 reg)
{
	struct drm_i915_private *dev_priv = pdev->dev_priv;
	i915_reg_t tmp = {.reg = reg};
	POSTING_READ(tmp);
}

extern void gvt_clean_initial_mmio_state(struct pgt_device *pdev);
extern bool gvt_setup_initial_mmio_state(struct pgt_device *pdev);

extern void gvt_clean_mmio_emulation_state(struct pgt_device *pdev);
extern bool gvt_setup_mmio_emulation_state(struct pgt_device *pdev);

static inline void gvt_pci_bar_write_32(struct vgt_device *vgt, uint32_t bar_offset, uint32_t val)
{
	uint32_t* cfg_reg;

	/* BAR offset should be 32 bits algiend */
	cfg_reg = (u32 *)&vgt->state.cfg.space[bar_offset & ~3];

	/* only write the bits 31-4, leave the 3-0 bits unchanged, as they are read-only */
	*cfg_reg = (val & 0xFFFFFFF0) | (*cfg_reg & 0xF);
}

static inline int gvt_pci_mmio_is_enabled(struct vgt_device *vgt)
{
	return vgt->state.cfg.space[GVT_REG_CFG_COMMAND] &
		_REGBIT_CFG_COMMAND_MEMORY;
}

#define __vreg(vgt, off) (*(u32*)(vgt->state.mmio.vreg + off))
#define __vreg8(vgt, off) (*(u8*)(vgt->state.mmio.vreg + off))
#define __vreg16(vgt, off) (*(u16*)(vgt->state.mmio.vreg + off))
#define __vreg64(vgt, off) (*(u64*)(vgt->state.mmio.vreg + off))

#define __sreg(vgt, off) (*(u32*)(vgt->state.mmio.sreg + off))
#define __sreg8(vgt, off) (*(u8*)(vgt->state.mmio.sreg + off))
#define __sreg16(vgt, off) (*(u16*)(vgt->state.mmio.sreg + off))
#define __sreg64(vgt, off) (*(u64*)(vgt->state.mmio.sreg + off))

static inline void gvt_set_instance_online(struct vgt_device *vgt)
{
	atomic_set(&vgt->active, 1);
}

static inline void gvt_set_instance_offline(struct vgt_device *vgt)
{
	atomic_set(&vgt->active, 0);
}

static inline bool gvt_instance_is_online(struct vgt_device *vgt)
{
	return atomic_read(&vgt->active);
}

#define for_each_online_instance(pdev, vgt, id) \
       idr_for_each_entry(&pdev->instance_idr, vgt, id) \
               if (gvt_instance_is_online(vgt))

extern void gvt_init_shadow_mmio_register(struct vgt_device *pdev);
extern void gvt_init_virtual_mmio_register(struct vgt_device *pdev);
extern struct vgt_device *gvt_create_instance(struct pgt_device *pdev,
		struct gvt_instance_info *info);
extern void gvt_destroy_instance(struct vgt_device *vgt);

/* check whether a guest GM address is within the CPU visible range */
static inline bool g_gm_is_visible(struct vgt_device *vgt, u64 g_addr)
{
	return (g_addr >= gvt_guest_visible_gm_base(vgt)) &&
		(g_addr <= gvt_guest_visible_gm_end(vgt));
}

/* check whether a guest GM address is out of the CPU visible range */
static inline bool g_gm_is_hidden(struct vgt_device *vgt, u64 g_addr)
{
	return (g_addr >= gvt_guest_hidden_gm_base(vgt)) &&
		(g_addr <= gvt_guest_hidden_gm_end(vgt));
}

static inline bool g_gm_is_valid(struct vgt_device *vgt, u64 g_addr)
{
	return g_gm_is_visible(vgt, g_addr) || g_gm_is_hidden(vgt, g_addr);
}

/* check whether a host GM address is within the CPU visible range */
static inline bool h_gm_is_visible(struct vgt_device *vgt, u64 h_addr)
{
	return (h_addr >= gvt_visible_gm_base(vgt)) &&
		(h_addr <= gvt_visible_gm_end(vgt));
}

/* check whether a host GM address is out of the CPU visible range */
static inline bool h_gm_is_hidden(struct vgt_device *vgt, u64 h_addr)
{
	return (h_addr >= gvt_hidden_gm_base(vgt)) &&
		(h_addr <= gvt_hidden_gm_end(vgt));
}

static inline bool h_gm_is_valid(struct vgt_device *vgt, u64 h_addr)
{
	return h_gm_is_visible(vgt, h_addr) || h_gm_is_hidden(vgt, h_addr);
}

/* for a guest GM address, return the offset within the CPU visible range */
static inline u64 g_gm_visible_offset(struct vgt_device *vgt, uint64_t g_addr)
{
	return g_addr - gvt_guest_visible_gm_base(vgt);
}

/* for a guest GM address, return the offset within the hidden range */
static inline u64 g_gm_hidden_offset(struct vgt_device *vgt, uint64_t g_addr)
{
	return g_addr - gvt_guest_hidden_gm_base(vgt);
}

/* for a host GM address, return the offset within the CPU visible range */
static inline u64 h_gm_visible_offset(struct vgt_device *vgt, uint64_t h_addr)
{
	return h_addr - gvt_visible_gm_base(vgt);
}

/* for a host GM address, return the offset within the hidden range */
static inline u64 h_gm_hidden_offset(struct vgt_device *vgt, uint64_t h_addr)
{
	return h_addr - gvt_hidden_gm_base(vgt);
}

/* validate a gm address and related range size, translate it to host gm address */
static inline int g2h_gm_range(struct vgt_device *vgt, u64 *addr, u32 size)
{
	ASSERT(addr);

	if ((!g_gm_is_valid(vgt, *addr)) || (size && !g_gm_is_valid(vgt, *addr + size - 1))) {
		gvt_err("VM(%d): invalid address range: g_addr(0x%llx), size(0x%x)\n",
			vgt->vm_id, *addr, size);
		return -EACCES;
	}

	if (g_gm_is_visible(vgt, *addr))	/* aperture */
		*addr = gvt_visible_gm_base(vgt) +
			g_gm_visible_offset(vgt, *addr);
	else	/* hidden GM space */
		*addr = gvt_hidden_gm_base(vgt) +
			g_gm_hidden_offset(vgt, *addr);
	return 0;
}

/* translate a guest gm address to host gm address */
static inline int g2h_gm(struct vgt_device *vgt, u64 *addr)
{
	return g2h_gm_range(vgt, addr, 4);
}

/* translate a host gm address to guest gm address */
static inline u64 h2g_gm(struct vgt_device *vgt, uint64_t h_addr)
{
	u64 g_addr;

	ASSERT_NUM(h_gm_is_valid(vgt, h_addr), h_addr);

	if (h_gm_is_visible(vgt, h_addr))
		g_addr = gvt_guest_visible_gm_base(vgt) +
			h_gm_visible_offset(vgt, h_addr);
	else
		g_addr = gvt_guest_hidden_gm_base(vgt) +
			h_gm_hidden_offset(vgt, h_addr);

	return g_addr;
}

#define reg_is_mmio(pdev, reg)	\
	(reg >= 0 && reg < pdev->mmio_size)

#define reg_is_gtt(pdev, reg)	\
	(reg >= pdev->device_info.gtt_start_offset \
	&& reg < pdev->device_info.gtt_end_offset)

static inline u32 g2h_gtt_index(struct vgt_device *vgt, uint32_t g_index)
{
	u64 addr = g_index << GTT_PAGE_SHIFT;

	g2h_gm(vgt, &addr);

	return (u32)(addr >> GTT_PAGE_SHIFT);
}

static inline u32 h2g_gtt_index(struct vgt_device *vgt, uint32_t h_index)
{
	u64 h_addr = h_index << GTT_PAGE_SHIFT;

	return (u32)(h2g_gm(vgt, h_addr) >> GTT_PAGE_SHIFT);
}

#include "mpt.h"

#endif
