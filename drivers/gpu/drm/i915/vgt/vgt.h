/*
 * vGT core headers
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

#ifndef _VGT_DRV_H_
#define _VGT_DRV_H_

#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/hashtable.h>
#include <linux/pci.h>
#include <linux/mempool.h>
#include <drm/drmP.h>

#include "vgt-if.h"
#include "host.h"

typedef uint32_t vgt_reg_t;

#include "reg.h"
#include "devtable.h"
#include "edid.h"
#include "cmd_parser.h"
#include "hypercall.h"
#include "execlists.h"

struct pgt_device;
struct vgt_device;
extern struct vgt_device *vgt_dom0;
extern struct pgt_device *perf_pgt;
extern struct list_head pgt_devices;
extern struct pgt_device default_device;
extern void show_ring_buffer(struct pgt_device *pdev, int ring_id, int bytes);
extern void show_mode_settings(struct pgt_device *pdev);
extern void show_ring_debug(struct pgt_device *pdev, int ring_id);
extern void show_debug(struct pgt_device *pdev);
void show_virtual_interrupt_regs(struct vgt_device *vgt, struct seq_file *seq);
extern void show_interrupt_regs(struct pgt_device *pdev, struct seq_file *seq);
void vgt_panic(void);

extern bool ignore_hvm_forcewake_req;
extern bool hvm_render_owner;
extern bool hvm_display_owner;
extern bool hvm_super_owner;
extern bool hvm_boot_foreground;
extern bool vgt_primary;
extern bool vgt_track_nest;
extern bool vgt_delay_nest;
extern int vgt_debug;
extern bool vgt_enabled;
extern bool fastpath_dpy_switch;
extern bool shadow_tail_based_qos;
extern bool event_based_qos;
extern int enable_video_switch;
extern int dom0_low_gm_sz;
extern int dom0_high_gm_sz;
extern int dom0_fence_sz;
extern int bypass_scan_mask;
extern bool bypass_dom0_addr_check;
extern bool render_engine_reset;
extern bool enable_panel_fitting;
extern bool enable_reset;
extern bool vgt_lock_irq;
extern int shadow_execlist_context;
extern bool wp_submitted_ctx;
extern bool propagate_monitor_to_guest;
extern bool irq_based_ctx_switch;
extern int preallocated_shadow_pages;
extern int preallocated_oos_pages;
extern bool spt_out_of_sync;
extern bool cmd_parser_ip_buf;
extern bool timer_based_qos;
extern int tbs_period_ms;
extern bool opregion_present;

enum vgt_event_type {
	// GT
	RCS_MI_USER_INTERRUPT = 0,
	RCS_DEBUG,
	RCS_MMIO_SYNC_FLUSH,
	RCS_CMD_STREAMER_ERR,
	RCS_PIPE_CONTROL,
	RCS_L3_PARITY_ERR,		/* IVB */
	RCS_WATCHDOG_EXCEEDED,
	RCS_PAGE_DIRECTORY_FAULT,
	RCS_AS_CONTEXT_SWITCH,
	RCS_MONITOR_BUFF_HALF_FULL,	/* IVB */

	VCS_MI_USER_INTERRUPT,
	VCS_MMIO_SYNC_FLUSH,
	VCS_CMD_STREAMER_ERR,
	VCS_MI_FLUSH_DW,
	VCS_WATCHDOG_EXCEEDED,
	VCS_PAGE_DIRECTORY_FAULT,
	VCS_AS_CONTEXT_SWITCH,

	VCS2_MI_USER_INTERRUPT,
	VCS2_MI_FLUSH_DW,
	VCS2_AS_CONTEXT_SWITCH,

	BCS_MI_USER_INTERRUPT,
	BCS_MMIO_SYNC_FLUSH,
	BCS_CMD_STREAMER_ERR,
	BCS_MI_FLUSH_DW,
	BCS_PAGE_DIRECTORY_FAULT,
	BCS_AS_CONTEXT_SWITCH,

	VECS_MI_USER_INTERRUPT,
	VECS_MI_FLUSH_DW,
	VECS_AS_CONTEXT_SWITCH,

	// DISPLAY
	PIPE_A_FIFO_UNDERRUN,	/* This is an active high level for the duration of the Pipe A FIFO underrun */
	PIPE_B_FIFO_UNDERRUN,	/* This is an active high level for the duration of the Pipe B FIFO underrun */
	PIPE_A_CRC_ERR,	/* This is an active high pulse on the Pipe A CRC error */
	PIPE_B_CRC_ERR,	/* This is an active high pulse on the Pipe B CRC error */
	PIPE_A_CRC_DONE,	/* This is an active high pulse on the Pipe A CRC done */
	PIPE_B_CRC_DONE,	/* This is an active high pulse on the Pipe B CRC done */
	PIPE_A_ODD_FIELD,	/* This is an active high level for the duration of the Pipe A interlaced odd field */
	PIPE_B_ODD_FIELD,	/* This is an active high level for the duration of the Pipe B interlaced odd field */
	PIPE_A_EVEN_FIELD,	/* This is an active high level for the duration of the Pipe A interlaced even field */
	PIPE_B_EVEN_FIELD,	/* This is an active high level for the duration of the Pipe B interlaced even field */
	PIPE_A_LINE_COMPARE,	/* This is an active high level for the duration of the selected Pipe A scan lines */
	PIPE_B_LINE_COMPARE,	/* This is an active high level for the duration of the selected Pipe B scan lines */
	PIPE_C_LINE_COMPARE,	/* This is an active high level for the duration of the selected Pipe C scan lines */
	PIPE_A_VBLANK,	/* This is an active high level for the duration of the Pipe A vertical blank */
	PIPE_B_VBLANK,	/* This is an active high level for the duration of the Pipe B vertical blank */
	PIPE_C_VBLANK,	/* This is an active high level for the duration of the Pipe C vertical blank */
	PIPE_A_VSYNC,	/* This is an active high level for the duration of the Pipe A vertical sync */
	PIPE_B_VSYNC,	/* This is an active high level for the duration of the Pipe B vertical sync */
	PIPE_C_VSYNC,	/* This is an active high level for the duration of the Pipe C vertical sync */
	PRIMARY_A_FLIP_DONE,	/* This is an active high pulse when a primary plane A flip is done */
	PRIMARY_B_FLIP_DONE,	/* This is an active high pulse when a primary plane B flip is done */
	PRIMARY_C_FLIP_DONE,	/* This is an active high pulse when a primary plane C flip is done */
	SPRITE_A_FLIP_DONE,	/* This is an active high pulse when a sprite plane A flip is done */
	SPRITE_B_FLIP_DONE,	/* This is an active high pulse when a sprite plane B flip is done */
	SPRITE_C_FLIP_DONE,	/* This is an active high pulse when a sprite plane C flip is done */

	DPST_PHASE_IN,	// This is an active high pulse on the DPST phase in event
	DPST_HISTOGRAM,	// This is an active high pulse on the AUX A done event.
	GSE,
	DP_A_HOTPLUG,
	AUX_CHANNEL_A,	// This is an active high pulse on the AUX A done event.
	PERF_COUNTER,	// This is an active high pulse when the performance counter reaches the threshold value programmed in the Performance Counter Source register
	POISON,		// This is an active high pulse on receiving the poison message
	GTT_FAULT,	// This is an active high level while either of the GTT Fault Status register bits are set
	ERROR_INTERRUPT_COMBINED,

	// PM
	GV_DOWN_INTERVAL,
	GV_UP_INTERVAL,
	RP_DOWN_THRESHOLD,
	RP_UP_THRESHOLD,
	FREQ_DOWNWARD_TIMEOUT_RC6,
	PCU_THERMAL,
	PCU_PCODE2DRIVER_MAILBOX,

	// PCH
	FDI_RX_INTERRUPTS_TRANSCODER_A,	// This is an active high level while any of the FDI_RX_ISR bits are set for transcoder A
	AUDIO_CP_CHANGE_TRANSCODER_A,	// This is an active high level while any of the FDI_RX_ISR bits are set for transcoder A
	AUDIO_CP_REQUEST_TRANSCODER_A,	// This is an active high level indicating content protection is requested by audio azalia verb programming for transcoder A
	FDI_RX_INTERRUPTS_TRANSCODER_B,
	AUDIO_CP_CHANGE_TRANSCODER_B,
	AUDIO_CP_REQUEST_TRANSCODER_B,
	FDI_RX_INTERRUPTS_TRANSCODER_C,
	AUDIO_CP_CHANGE_TRANSCODER_C,
	AUDIO_CP_REQUEST_TRANSCODER_C,
	ERR_AND_DBG,
	GMBUS,
	SDVO_B_HOTPLUG,
	CRT_HOTPLUG,
	DP_B_HOTPLUG,
	DP_C_HOTPLUG,
	DP_D_HOTPLUG,
	AUX_CHENNEL_B,
	AUX_CHENNEL_C,
	AUX_CHENNEL_D,
	AUDIO_POWER_STATE_CHANGE_B,
	AUDIO_POWER_STATE_CHANGE_C,
	AUDIO_POWER_STATE_CHANGE_D,

	RCS_IRQ,
	BCS_IRQ,
	VCS_IRQ,
	VCS2_IRQ,
	PM_IRQ,
	VECS_IRQ,
	DE_PIPE_A_IRQ,
	DE_PIPE_B_IRQ,
	DE_PIPE_C_IRQ,
	DE_PORT_IRQ,
	DE_MISC_IRQ,
	PCH_IRQ,
	PCU_IRQ,

	EVENT_RESERVED,
	EVENT_MAX,
};


enum transcoder {
	TRANSCODER_A = 0,
	TRANSCODER_B,
	TRANSCODER_C,
	TRANSCODER_EDP = 0xF,
};

enum map_type {
	VGT_MAP_APERTURE,
	VGT_MAP_OPREGION,
};

#define vgt_dbg(component, fmt, s...)	\
	do { if (vgt_debug & component) printk(KERN_DEBUG "vGT debug:(%s:%d) " fmt, __FUNCTION__, __LINE__, ##s); } while (0)

#define VGT_DBG_GENERIC		(1<<0)
#define VGT_DBG_DPY		(1<<1)
#define VGT_DBG_MEM		(1<<2)
#define VGT_DBG_RENDER		(1<<3)
#define VGT_DBG_CMD		(1<<4)
#define VGT_DBG_IRQ		(1<<5)
#define VGT_DBG_EDID		(1<<6)
#define VGT_DBG_EXECLIST	(1<<7)
#define VGT_DBG_ALL		(0xffff)

/*
 * Define registers of a ring buffer per hardware register layout.
 */
typedef struct {
	vgt_reg_t tail;
	vgt_reg_t head;
	vgt_reg_t start;
	vgt_reg_t ctl;
} vgt_ringbuffer_t;

#define SIZE_1KB		(1024UL)
#define SIZE_1MB		(1024UL*1024UL)
#define SIZE_PAGE		(4 * SIZE_1KB)

#define VGT_RSVD_RING_SIZE	(16 * SIZE_1KB)
struct vgt_rsvd_ring {
	struct pgt_device *pdev;
	void *virtual_start;
	int start;
	uint64_t null_context;
	uint64_t indirect_state;
	int id;

	u32 head;
	u32 tail;
	int size;
	/* whether the engine requires special context switch */
	bool	stateless;
	/* whether the engine requires context switch */
	bool	need_switch;
	/* whether the engine end with user interrupt instruction */
	bool	need_irq;
	/* memory offset of the user interrupt instruction */
	u32	ip_offset;
};

#define _tail_reg_(ring_reg_off)	\
		(ring_reg_off & ~(sizeof(vgt_ringbuffer_t)-1))

#define _vgt_mmio_va(pdev, x)		((uint64_t)((char*)pdev->gttmmio_base_va+x))	/* PA to VA */
#define _vgt_mmio_pa(pdev, x)		(pdev->gttmmio_base+x)			/* PA to VA */

#define VGT_RING_TIMEOUT	500	/* in ms */
#define VGT_VBLANK_TIMEOUT	50	/* in ms */

/* Maximum VMs supported by vGT. Actual number is device specific */
#define VGT_MAX_VMS_HSW 		4
#define VGT_MAX_VMS			8
#define VGT_RSVD_APERTURE_SZ		(32*SIZE_1MB)	/* reserve 8MB for vGT itself */

#define GTT_PAGE_SHIFT		12
#define GTT_PAGE_SIZE		(1UL << GTT_PAGE_SHIFT)
#define GTT_PAGE_MASK		(~(GTT_PAGE_SIZE-1))
#define GTT_PAE_MASK		((1UL <<12) - (1UL << 4)) /* bit 11:4 */

/*
 * The maximum GM size supported by VGT GM resource allocator.
 */
#define VGT_MAX_GM_SIZE			(1UL << 32)
#define VGT_GM_BITMAP_BITS		(VGT_MAX_GM_SIZE/SIZE_1MB)
#define VGT_MAX_NUM_FENCES		32
#define VGT_FENCE_BITMAP_BITS	VGT_MAX_NUM_FENCES
#define VGT_FENCE_REGION_SIZE	(VGT_MAX_NUM_FENCES*8)
#define VGT_RSVD_APERTURE_BITMAP_BITS (VGT_RSVD_APERTURE_SZ / GTT_PAGE_SIZE)
#define VGT_APERTURE_PAGES	(VGT_RSVD_APERTURE_SZ >> GTT_PAGE_SHIFT)

//#define SZ_CONTEXT_AREA_PER_RING	4096
#define SZ_CONTEXT_AREA_PER_RING	(4096*64)	/* use 256 KB for now */
#define SZ_INDIRECT_STATE		(4096)		/* use 4KB for now */
#define VGT_APERTURE_PER_INSTANCE_SZ		(4*SIZE_1KB)	/* 4KB per instance (?) */
#define VGT_ID_ALLOC_BITMAP		((1UL << VGT_MAX_VMS) - 1)

#define REG_SIZE			sizeof(vgt_reg_t)		/* size of gReg/sReg[0] */
#define REG_INDEX(reg)		((reg) / REG_SIZE)
#define VGT_MMIO_SPACE_SZ	(2*SIZE_1MB)
#define VGT_CFG_SPACE_SZ	256
#define VGT_BAR_NUM		4
typedef struct {
	uint64_t	mmio_base_gpa;	/* base guest physical address of the MMIO registers */
	vgt_reg_t	*vReg;		/* guest view of the register state */
	vgt_reg_t	*sReg;		/* Shadow (used by hardware) state of the register */
	uint8_t	cfg_space[VGT_CFG_SPACE_SZ];
	bool	bar_mapped[VGT_BAR_NUM];
	uint64_t	gt_mmio_base;	/* bar0/GTTMMIO */
	uint64_t	aperture_base;	/* bar1: guest aperture base */
//	uint64_t	gt_gmadr_base;	/* bar1/GMADR */

	uint64_t	bar_size[VGT_BAR_NUM];	/* 0: GTTMMIO, 1: GMADR, 2: PIO bar size */

	/* OpRegion state */
	void		*opregion_va;
	uint64_t	opregion_gfn[VGT_OPREGION_PAGES];
	struct page *opregion_pages[VGT_OPREGION_PAGES];
} vgt_state_t;

typedef struct {
	vgt_reg_t base;
	vgt_reg_t cache_ctl;
	vgt_reg_t mode;
} vgt_ring_ppgtt_t;

#define __vreg(vgt, off) (*(vgt_reg_t *)((char *)vgt->state.vReg + off))
#define __vreg8(vgt, off) (*(char *)((char *)vgt->state.vReg + off))
#define __vreg16(vgt, off) (*(uint16_t *)((char *)vgt->state.vReg + off))
#define __sreg(vgt, off) (*(vgt_reg_t *)((char *)vgt->state.sReg + off))
#define __sreg8(vgt, off) (*(char *)((char *)vgt->state.sReg + off))
#define __vreg64(vgt, off) (*(unsigned long *)((char *)vgt->state.vReg + off))
#define __sreg64(vgt, off) (*(unsigned long *)((char *)vgt->state.sReg + off))
#define vgt_vreg(vgt, off)	((vgt_reg_t *)((char *)vgt->state.vReg + off))
#define vgt_sreg(vgt, off)	((vgt_reg_t *)((char *)vgt->state.sReg + off))

#define RB_DWORDS_TO_SAVE	32
typedef	uint32_t	rb_dword;

struct execlist_context;
enum EL_SLOT_STATUS {
	EL_EMPTY	= 0,
	EL_PENDING,
	EL_SUBMITTED
};

struct vgt_exec_list {
	enum EL_SLOT_STATUS status;
	struct execlist_context *el_ctxs[2];
};

struct vgt_elsp_store {
	uint32_t count;
	uint32_t element[4];
};

#define EL_QUEUE_SLOT_NUM 3

struct vgt_mm;

typedef struct {
	vgt_ringbuffer_t	vring;		/* guest view ring */
	vgt_ringbuffer_t	sring;		/* shadow ring */
	/* In aperture, partitioned & 4KB aligned. */
	/* 64KB alignment requirement for walkaround. */
	uint64_t	context_save_area;	/* VGT default context space */
	uint32_t	active_vm_context;
	/* ppgtt info */
	vgt_ring_ppgtt_t	vring_ppgtt_info; /* guest view */
	vgt_ring_ppgtt_t	sring_ppgtt_info; /* shadow info */
	u8 has_ppgtt_base_set : 1;	/* Is PP dir base set? */
	u8 has_ppgtt_mode_enabled : 1;	/* Is ring's mode reg PPGTT enable set? */
	u8 has_execlist_enabled : 1;
	struct vgt_mm *active_ppgtt_mm;
	int ppgtt_root_pointer_type;
	int ppgtt_page_table_level;

	struct cmd_general_info	patch_list;
	struct cmd_general_info	handler_list;
	struct cmd_general_info	tail_list;

	uint64_t cmd_nr;
	vgt_reg_t	last_scan_head;
	uint64_t request_id;

	vgt_reg_t uhptr;
	uint64_t uhptr_id;
	int el_slots_head;
	int el_slots_tail;
	struct vgt_exec_list execlist_slots[EL_QUEUE_SLOT_NUM];
	struct vgt_elsp_store elsp_store;
	int csb_write_ptr;
} vgt_state_ring_t;

#define vgt_el_queue_head(vgt, ring_id) \
	((vgt)->rb[ring_id].el_slots_head)
#define vgt_el_queue_tail(vgt, ring_id) \
	((vgt)->rb[ring_id].el_slots_tail)
#define vgt_el_queue_slot(vgt, ring_id, slot_idx) \
	((vgt)->rb[ring_id].execlist_slots[slot_idx])
#define vgt_el_queue_ctx(vgt, ring_id, slot_idx, ctx_idx) \
	((vgt)->rb[ring_id].execlist_slots[slot_idx].el_ctxs[ctx_idx])

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

#define	VGT_HASH_BITS	8

/*
 * Ring ID definition.
 */
enum vgt_ring_id {
	RING_BUFFER_RCS = 0,
	RING_BUFFER_VCS,
	RING_BUFFER_BCS,
	RING_BUFFER_VECS,
	RING_BUFFER_VCS2,
	MAX_ENGINES
};

typedef enum {
	GTT_TYPE_INVALID = -1,

	GTT_TYPE_GGTT_PTE,

	GTT_TYPE_PPGTT_PTE_4K_ENTRY,
	GTT_TYPE_PPGTT_PTE_2M_ENTRY,
	GTT_TYPE_PPGTT_PTE_1G_ENTRY,

	GTT_TYPE_PPGTT_PTE_ENTRY,

	GTT_TYPE_PPGTT_PDE_ENTRY,
	GTT_TYPE_PPGTT_PDP_ENTRY,
	GTT_TYPE_PPGTT_PML4_ENTRY,

	GTT_TYPE_PPGTT_ROOT_ENTRY,

	GTT_TYPE_PPGTT_ROOT_L3_ENTRY,
	GTT_TYPE_PPGTT_ROOT_L4_ENTRY,

	GTT_TYPE_PPGTT_ENTRY,

	GTT_TYPE_PPGTT_PTE_PT,
	GTT_TYPE_PPGTT_PDE_PT,
	GTT_TYPE_PPGTT_PDP_PT,
	GTT_TYPE_PPGTT_PML4_PT,

	GTT_TYPE_MAX,
}gtt_type_t;

#define gtt_type_is_entry(type) \
	(type > GTT_TYPE_INVALID && type < GTT_TYPE_PPGTT_ENTRY \
	 && type != GTT_TYPE_PPGTT_PTE_ENTRY \
	 && type != GTT_TYPE_PPGTT_ROOT_ENTRY)

#define gtt_type_is_pt(type) \
	(type >= GTT_TYPE_PPGTT_PTE_PT && type < GTT_TYPE_MAX)

#define gtt_type_is_pte_pt(type) \
	(type == GTT_TYPE_PPGTT_PTE_PT)

#define gtt_type_is_root_pointer(type) \
	(gtt_type_is_entry(type) && type > GTT_TYPE_PPGTT_ROOT_ENTRY)

typedef struct {
	union {
		u32 val32[2];
		u64 val64;
	};
	gtt_type_t type;
	struct pgt_device *pdev;
}gtt_entry_t;

#define gtt_init_entry(e, t, p, v) do { \
	(e)->type = t; \
	(e)->pdev = p; \
	memcpy(&(e)->val64, &v, sizeof(v)); \
}while(0)

struct vgt_gtt_pte_ops {
	gtt_entry_t *(*get_entry)(void *pt, gtt_entry_t *e, unsigned long index,
			bool hypervisor_access, struct vgt_device *vgt);
	gtt_entry_t *(*set_entry)(void *pt, gtt_entry_t *e, unsigned long index,
			bool hypervisor_access, struct vgt_device *vgt);
	bool (*test_present)(gtt_entry_t *e);
	void (*clear_present)(gtt_entry_t *e);
	bool (*test_pse)(gtt_entry_t *e);
	void (*set_pfn)(gtt_entry_t *e, unsigned long pfn);
	unsigned long (*get_pfn)(gtt_entry_t *e);
};

struct vgt_gtt_gma_ops {
	unsigned long (*gma_to_ggtt_pte_index)(unsigned long gma);
	unsigned long (*gma_to_pte_index)(unsigned long gma);
	unsigned long (*gma_to_pde_index)(unsigned long gma);
	unsigned long (*gma_to_l3_pdp_index)(unsigned long gma);
	unsigned long (*gma_to_l4_pdp_index)(unsigned long gma);
	unsigned long (*gma_to_pml4_index)(unsigned long gma);
};

extern struct vgt_gtt_pte_ops gen7_gtt_pte_ops;
extern struct vgt_gtt_pte_ops gen8_gtt_pte_ops;
extern struct vgt_gtt_gma_ops gen7_gtt_gma_ops;
extern struct vgt_gtt_gma_ops gen8_gtt_gma_ops;

typedef struct {
	void *vaddr;
	struct page *page;
	gtt_type_t type;
	struct hlist_node node;
	unsigned long mfn;
}shadow_page_t;

typedef enum {
	VGT_MM_GGTT = 0,
	VGT_MM_PPGTT,
} vgt_mm_type_t;

struct vgt_mm {
	vgt_mm_type_t type;
	bool initialized;
	bool shadowed;

	gtt_type_t page_table_entry_type;
	u32 page_table_entry_size;
	u32 page_table_entry_cnt;
	void *virtual_page_table;
	void *shadow_page_table;

	int page_table_level;
	bool has_shadow_page_table;
	u32 pde_base_index;

	struct list_head list;
	atomic_t refcount;
	struct vgt_device *vgt;
};

extern gtt_entry_t *vgt_mm_get_entry(struct vgt_mm *mm,
                void *page_table, gtt_entry_t *e,
                unsigned long index);

extern gtt_entry_t *vgt_mm_set_entry(struct vgt_mm *mm,
                void *page_table, gtt_entry_t *e,
                unsigned long index);

#define ggtt_get_guest_entry(mm, e, index) \
	(mm->vgt->vm_id == 0) ? \
	vgt_mm_get_entry(mm, NULL, e, index) : \
	vgt_mm_get_entry(mm, mm->virtual_page_table, e, index)

#define ggtt_set_guest_entry(mm, e, index) \
	vgt_mm_set_entry(mm, mm->virtual_page_table, e, index)

#define ggtt_get_shadow_entry(mm, e, index) \
	vgt_mm_get_entry(mm, mm->shadow_page_table, e, index)

#define ggtt_set_shadow_entry(mm, e, index) \
	vgt_mm_set_entry(mm, mm->shadow_page_table, e, index)

#define ppgtt_get_guest_root_entry(mm, e, index) \
	vgt_mm_get_entry(mm, mm->virtual_page_table, e, index)

#define ppgtt_set_guest_root_entry(mm, e, index) \
	vgt_mm_set_entry(mm, mm->virtual_page_table, e, index)

#define ppgtt_get_shadow_root_entry(mm, e, index) \
	vgt_mm_get_entry(mm, mm->shadow_page_table, e, index)

#define ppgtt_set_shadow_root_entry(mm, e, index) \
	vgt_mm_set_entry(mm, mm->shadow_page_table, e, index)

extern struct vgt_mm *vgt_create_mm(struct vgt_device *vgt,
		vgt_mm_type_t mm_type, gtt_type_t page_table_entry_type,
		void *virtual_page_table, int page_table_level,
		u32 pde_base_index);
extern void vgt_destroy_mm(struct vgt_mm *mm);

extern bool gen7_mm_alloc_page_table(struct vgt_mm *mm);
extern void gen7_mm_free_page_table(struct vgt_mm *mm);
extern bool gen8_mm_alloc_page_table(struct vgt_mm *mm);
extern void gen8_mm_free_page_table(struct vgt_mm *mm);

struct guest_page;

struct vgt_vgtt_info {
	struct vgt_mm *ggtt_mm;
	unsigned long active_ppgtt_mm_bitmap;
	struct list_head mm_list_head;
	DECLARE_HASHTABLE(shadow_page_hash_table, VGT_HASH_BITS);
	DECLARE_HASHTABLE(guest_page_hash_table, VGT_HASH_BITS);
	DECLARE_HASHTABLE(el_ctx_hash_table, VGT_HASH_BITS);
	atomic_t n_write_protected_guest_page;
	struct list_head oos_page_list_head;
	int last_partial_ppgtt_access_index;
	gtt_entry_t last_partial_ppgtt_access_entry;
	struct guest_page *last_partial_ppgtt_access_gpt;
	bool warn_partial_ppgtt_access_once;
};

extern bool vgt_init_vgtt(struct vgt_device *vgt);
extern void vgt_clean_vgtt(struct vgt_device *vgt);

extern bool vgt_gtt_init(struct pgt_device *pdev);
extern void vgt_gtt_clean(struct pgt_device *pdev);

extern bool vgt_expand_shadow_page_mempool(struct pgt_device *pdev);

extern bool vgt_g2v_create_ppgtt_mm(struct vgt_device *vgt, int page_table_level);
extern bool vgt_g2v_destroy_ppgtt_mm(struct vgt_device *vgt, int page_table_level);

extern struct vgt_mm *gen8_find_ppgtt_mm(struct vgt_device *vgt,
                int page_table_level, void *root_entry);

extern bool ppgtt_check_partial_access(struct vgt_device *vgt);

typedef bool guest_page_handler_t(void *gp, uint64_t pa, void *p_data, int bytes);

struct oos_page;

struct guest_page {
	struct hlist_node node;
	int writeprotection;
	unsigned long gfn;
	void *vaddr;
	guest_page_handler_t *handler;
	void *data;
	unsigned long write_cnt;
	struct oos_page *oos_page;
};

typedef struct guest_page guest_page_t;

struct oos_page {
	guest_page_t *guest_page;
	struct list_head list;
	struct list_head vm_list;
	int id;
	unsigned char mem[GTT_PAGE_SIZE];
};
typedef struct oos_page oos_page_t;

typedef struct {
	shadow_page_t shadow_page;
	guest_page_t guest_page;
	gtt_type_t guest_page_type;
	atomic_t refcount;
	struct vgt_device *vgt;
} ppgtt_spt_t;

extern bool vgt_init_guest_page(struct vgt_device *vgt, guest_page_t *guest_page,
		unsigned long gfn, guest_page_handler_t handler, void *data);
extern void vgt_clean_guest_page(struct vgt_device *vgt, guest_page_t *guest_page);
extern bool vgt_set_guest_page_writeprotection(struct vgt_device *vgt,
		guest_page_t *guest_page);
extern bool vgt_clear_guest_page_writeprotection(struct vgt_device *vgt,
		guest_page_t *guest_page);
extern guest_page_t *vgt_find_guest_page(struct vgt_device *vgt, unsigned long gfn);

extern bool gen7_ppgtt_mm_setup(struct vgt_device *vgt, int ring_id);
bool ppgtt_sync_oos_pages(struct vgt_device *vgt);

/* shadow context */

struct shadow_ctx_page {
	guest_page_t guest_page;
	shadow_page_t shadow_page;
	struct vgt_device *vgt;
};

struct execlist_context {
	struct ctx_desc_format guest_context;
	uint32_t shadow_lrca;
	uint32_t error_reported;
	enum vgt_ring_id ring_id;
	/* below are some per-ringbuffer data. Since with execlist,
	 * each context has its own ring buffer, here we store the
	 * data and store them into vgt->rb[ring_id] before a
	 * context is submitted. We will have better handling later.
	 */
	vgt_reg_t last_guest_head;
	vgt_reg_t last_scan_head;
	uint64_t request_id;
	//uint64_t cmd_nr;
	//vgt_reg_t uhptr;
	//uint64_t uhptr_id;

	struct vgt_mm *ppgtt_mm;
	struct shadow_ctx_page ctx_pages[MAX_EXECLIST_CTX_PAGES];
	/* used for lazy context shadowing optimization */
	gtt_entry_t shadow_entry_backup[MAX_EXECLIST_CTX_PAGES];

	struct hlist_node node;
};

extern enum vgt_pipe surf_used_pipe;

struct pgt_device;

struct vgt_render_context_ops {
	bool (*init_null_context)(struct pgt_device *pdev, int id);
	bool (*save_hw_context)(int id, struct vgt_device *vgt);
	bool (*restore_hw_context)(int id, struct vgt_device *vgt);
	bool (*ring_context_switch)(struct pgt_device *pdev,
				enum vgt_ring_id ring_id,
				struct vgt_device *prev,
				struct vgt_device *next);
};

extern bool vgt_render_init(struct pgt_device *pdev);
extern bool idle_rendering_engines(struct pgt_device *pdev, int *id);
extern bool idle_render_engine(struct pgt_device *pdev, int id);
extern bool vgt_do_render_context_switch(struct pgt_device *pdev);
extern bool vgt_do_render_sched(struct pgt_device *pdev);
extern void vgt_destroy_debugfs(struct vgt_device *vgt);
extern void vgt_release_debugfs(void);
extern bool vgt_register_mmio_handler(unsigned int start, int bytes,
	vgt_mmio_read read, vgt_mmio_write write);
extern void vgt_clear_mmio_table(void);

extern bool need_scan_attached_ports;
extern bool vgt_reinitialize_mode(struct vgt_device *cur_vgt,
		struct vgt_device *next_vgt);
extern int vgt_hvm_info_init(struct vgt_device *vgt);
extern int vgt_hvm_opregion_init(struct vgt_device *vgt, uint32_t gpa);
extern void vgt_hvm_info_deinit(struct vgt_device *vgt);
extern bool vgt_prepare_vbios_general_definition(struct vgt_device *vgt);
extern void vgt_check_pending_context_switch(struct vgt_device *vgt);

struct vgt_irq_virt_state;

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

/* per-VM structure */
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

#define VGT_TBS_PERIOD_MAX 15
#define VGT_TBS_PERIOD_MIN 1
#define VGT_TBS_DEFAULT_PERIOD(x) ((x) * 1000000) /* 15 ms */

struct vgt_hrtimer {
	struct hrtimer timer;
};

#define VGT_TAILQ_RB_POLLING_PERIOD (2 * 1000000)
#define VGT_TAILQ_SIZE (SIZE_1MB)
#define VGT_TAILQ_MAX_ENTRIES ((VGT_TAILQ_SIZE)/sizeof(u32))
#define VGT_TAILQ_IDX_MASK (VGT_TAILQ_MAX_ENTRIES - 1)
/* Maximum number of tail can be cached is (VGT_TAILQ_MAX_ENTRIES - 1) */
struct vgt_tailq {
	u32 __head;
	u32 __tail;
	u32 *__buf_tail;  /* buffer to save tail value caught by tail-write */
	u32 *__buf_cmdnr; /* buffer to save cmd nr for each tail-write */
};
#define vgt_tailq_idx(idx) ((idx) & VGT_TAILQ_IDX_MASK)

/* DPCD start */
#define DPCD_SIZE	0x700

struct vgt_dpcd_data {
	bool data_valid;
	u8 data[DPCD_SIZE];
};

enum dpcd_index {
	DPCD_DPA = 0,
	DPCD_DPB,
	DPCD_DPC,
	DPCD_DPD,
	DPCD_MAX
};

/* DPCD addresses */
#define DPCD_REV			0x000
#define DPCD_MAX_LINK_RATE			0x001
#define DPCD_MAX_LANE_COUNT			0x002

#define DPCD_TRAINING_PATTERN_SET	0x102
#define	DPCD_SINK_COUNT			0x200
#define DPCD_LANE0_1_STATUS		0x202
#define DPCD_LANE2_3_STATUS		0x203
#define DPCD_LANE_ALIGN_STATUS_UPDATED	0x204
#define DPCD_SINK_STATUS		0x205

/* link training */
#define DPCD_TRAINING_PATTERN_SET_MASK	0x03
#define DPCD_LINK_TRAINING_DISABLED	0x00
#define DPCD_TRAINING_PATTERN_1		0x01
#define DPCD_TRAINING_PATTERN_2		0x02

#define DPCD_CP_READY_MASK		(1 << 6)

/* lane status */
#define DPCD_LANES_CR_DONE		0x11
#define DPCD_LANES_EQ_DONE		0x22
#define DPCD_SYMBOL_LOCKED		0x44

#define DPCD_INTERLANE_ALIGN_DONE	0x01

#define DPCD_SINK_IN_SYNC		0x03

/* DPCD end */

#define SBI_REG_MAX	20

struct sbi_register {
	unsigned int offset;
	vgt_reg_t value;
};

struct sbi_registers {
	int number;
	struct sbi_register registers[SBI_REG_MAX];
};

struct port_cache {
	bool valid;
	struct vgt_edid_data_t	*edid;	/* per display EDID information */
	enum vgt_port		port_override;
	enum vgt_port_type	type;
};

struct gt_port {
	struct kobject  	kobj;

	struct vgt_edid_data_t	*edid;	/* per display EDID information */
	struct vgt_dpcd_data	*dpcd;	/* per display DPCD information */
	enum vgt_port_type	type;
	enum vgt_port		port_override;
	struct port_cache	cache; /* the temporary updated information */
	enum vgt_port physcal_port;
};

struct vgt_device {
	enum vgt_pipe pipe_mapping[I915_MAX_PIPES];
	int vgt_id;		/* 0 is always for dom0 */
	int vm_id;		/* domain ID per hypervisor */
	struct pgt_device *pdev;	/* the pgt device where the GT device registered. */
	struct list_head	list;	/* FIXME: used for context switch ?? */
	vgt_state_t	state;		/* MMIO state except ring buffers */
	vgt_state_ring_t	rb[MAX_ENGINES];	/* ring buffer state */

	struct gt_port		ports[I915_MAX_PORTS]; /* one port per PIPE */
	struct vgt_i2c_edid_t	vgt_i2c_edid;	/* i2c bus state emulaton for reading EDID */

	uint64_t	aperture_base;
	void		*aperture_base_va;
	uint64_t	aperture_sz;
	uint64_t	gm_sz;
	uint64_t	aperture_offset;	/* address fix for visible GM */
	uint64_t	hidden_gm_offset;	/* address fix for invisible GM */
	int			fence_base;
	int			fence_sz;


	/* TODO: move to hvm_info  */
	unsigned long low_mem_max_gpfn;	/* the max gpfn of the <4G memory */
	void *hvm_info;

	vgt_reg_t	saved_wakeup;		/* disable PM before switching */

	struct kobject kobj;
	struct vgt_statistics	stat;		/* statistics info */

	DECLARE_BITMAP(enabled_rings, MAX_ENGINES);
	DECLARE_BITMAP(started_rings, MAX_ENGINES);
	struct vgt_vgtt_info gtt;

	/* embedded context scheduler information */
	struct vgt_sched_info sched_info;

	/* Tail Queue (used to cache tail-writingt) */
	struct vgt_tailq rb_tailq[MAX_ENGINES];

	uint8_t	ballooning:1; /* VM supports ballooning */
	uint8_t	force_removal:1; /* force removal from the render run queue */
	/* Temporary flag for VEBOX guest driver support.
	 * Linux VM will have official VEBOX support until kernel 3.9.
	 * Windows driver already enables VEBOX support now.
	 * So in order to determine whether VM has turned on VEBOX on HSW, this
	 * flag is used. Will remove in future when VM drivers all have VEBOX
	 * support. */
	uint8_t vebox_support:1;
	uint8_t has_context:1;
	/*
	 * Have HVM been visible from boot time?
	 * Used when hvm_boot_foreground mode is enabled.
	 */
	uint8_t hvm_boot_foreground_visible:1;
	uint8_t warn_untrack:1;
	uint8_t bypass_addr_check:1;

	atomic_t crashing;

	uint64_t total_cmds;		/* total CMDs since VM is started */
	uint64_t submitted_cmds;	/* CMDs submitted in current slice */
	uint64_t allocated_cmds;	/* CMDs allocated in current slice */

	uint32_t frmcount_delta[I915_MAX_PIPES]; /* used for vblank virtualization*/

	struct sbi_registers sbi_regs;

	unsigned long reset_flags;
	unsigned long enabled_rings_before_reset;
	unsigned long last_reset_time;
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
/* index into another auxillary table. Maximum 256 entries now */
#define VGT_REG_INDEX_SHIFT	16
#define VGT_REG_INDEX_MASK	(0xFFFF << VGT_REG_INDEX_SHIFT)
typedef u32 reg_info_t;

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

struct vgt_irq_host_state;
#define VGT_VBIOS_PAGES 16

/* PLUG_OUT must equal to PLUG_IN + 1
 * hot plug handler code has such assumption. Actually it might
 * be OK to send HOTPLUG only, not necessarily differ IN aond
 * OUT.
 */
enum vgt_uevent_type {
	CRT_HOTPLUG_IN = 0,
	CRT_HOTPLUG_OUT,
	PORT_A_HOTPLUG_IN,
	PORT_A_HOTPLUG_OUT,
	PORT_B_HOTPLUG_IN,
	PORT_B_HOTPLUG_OUT,
	PORT_C_HOTPLUG_IN,
	PORT_C_HOTPLUG_OUT,
	PORT_D_HOTPLUG_IN,
	PORT_D_HOTPLUG_OUT,
	VGT_ENABLE_VGA,
	VGT_DISABLE_VGA,
	VGT_DISPLAY_READY,
	VGT_DISPLAY_UNREADY,
	VGT_DETECT_PORT_A,
	VGT_DETECT_PORT_B,
	VGT_DETECT_PORT_C,
	VGT_DETECT_PORT_D,
	VGT_DETECT_PORT_E,
	UEVENT_MAX
};

#define HOTPLUG_VMID_FOR_ALL_VMS	0xff

#define VGT_MAX_UEVENT_VARS 20
struct vgt_uevent_info {
	char *uevent_name;
	int vm_id;
	enum kobject_action action;
	char *env_var_table[VGT_MAX_UEVENT_VARS];
	bool (*vgt_uevent_handler)(enum vgt_uevent_type event,
				struct vgt_uevent_info *uevent_entry,
				struct pgt_device *dev);
};

void vgt_set_uevent(struct vgt_device *vgt, enum vgt_uevent_type uevent);

enum vgt_trace_type {
	VGT_TRACE_READ,
	VGT_TRACE_WRITE
};

typedef union {
	uint32_t cmd;
	struct {
		uint32_t action : 1;
		uint32_t port_sel: 3;
		uint32_t rsvd_4_7 : 4;
		uint32_t vmid : 8;
		uint32_t rsvd_16_31 : 16;
	};
} vgt_hotplug_cmd_t;

typedef union {
	uint32_t dw;
	struct {
		uint32_t virtual_event: 16;
		uint32_t vmid : 8;
		uint32_t rsvd_24_31 : 8;
	};
} vgt_virtual_event_t;

struct hotplug_work {
	struct work_struct work;
	DECLARE_BITMAP(hotplug_uevent, UEVENT_MAX);
	struct mutex hpd_mutex;
};

enum vgt_output_type {
	VGT_OUTPUT_ANALOG = 0,
	VGT_OUTPUT_DISPLAYPORT,
	VGT_OUTPUT_EDP,
	VGT_OUTPUT_LVDS,
	VGT_OUTPUT_HDMI,
	VGT_OUTPUT_MAX
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
};

#define PCI_BDF2(b,df)  ((((b) & 0xff) << 8) | ((df) & 0xff))

struct vgt_mmio_dev;

enum {
	RESET_INPROGRESS = 0,
	WAIT_RESET,
};

#define device_is_reseting(pdev) \
	test_bit(RESET_INPROGRESS, &pdev->device_reset_flags)

#define MKGEN(major, minor, rev) \
	((major << 16) | (minor << 8) | (rev))

#define GEN_MAJOR(gen) ((gen >> 16) & 0xff)
#define GEN_MINOR(gen) ((gen >> 8) & 0xff)
#define GEN_REV(gen) ((gen) & 0xff)

/* Describe the limitation of HW.*/
struct vgt_device_info {
	u32 gen;
	u64 max_gtt_gm_sz;
	u32 gtt_start_offset;
	u32 max_gtt_size;
	u32 gtt_entry_size;
	u32 gtt_entry_size_shift;
	u32 gmadr_bytes_in_cmd;
};

struct vgt_gtt_info {
	struct vgt_gtt_pte_ops *pte_ops;
	struct vgt_gtt_gma_ops *gma_ops;
	bool (*mm_alloc_page_table)(struct vgt_mm *mm);
	void (*mm_free_page_table)(struct vgt_mm *mm);
	mempool_t *mempool;
	struct mutex mempool_lock;
	struct list_head oos_page_use_list_head;
	struct list_head oos_page_free_list_head;
};

/* per-device structure */
struct pgt_device {
	struct list_head	list; /* list node for 'pgt_devices' */

	struct vgt_device_info device_info;

	struct pci_bus *pbus;	/* parent bus of the device */
	struct pci_dev *pdev;	/* the gfx device bound to */
	int bus;		/* parent bus number */
	int devfn;		/* device function number */

	struct task_struct *p_thread;
	wait_queue_head_t event_wq;
	wait_queue_head_t destroy_wq;

	unsigned long device_reset_flags;

	uint32_t request;

	uint64_t ctx_check;	/* the number of checked count in vgt thread */
	uint64_t ctx_switch;	/* the number of context switch count in vgt thread */
	uint32_t magic;		/* the magic number for checking the completion of context switch */

	vgt_reg_t *initial_mmio_state;	/* copy from physical at start */
	uint8_t initial_cfg_space[VGT_CFG_SPACE_SZ];	/* copy from physical at start */
	uint64_t bar_size[VGT_BAR_NUM];
	uint64_t total_gm_sz;	/* size of available GM space, e.g 2M GTT is 2GB */

	uint64_t gttmmio_base;	/* base of GTT and MMIO */
	void *gttmmio_base_va;	/* virtual base of GTT and MMIO */
	uint64_t gmadr_base;	/* base of GMADR */
	void *gmadr_va;		/* virtual base of GMADR */
	u32 mmio_size;
	u64 gtt_size;
	int reg_num;
	uint32_t *saved_gtt;
	uint64_t saved_fences[VGT_MAX_NUM_FENCES];

	uint32_t saved_rrmr;
	uint32_t saved_shotplug_ctl;

	int max_engines;	/* supported max engines */
	u32 ring_mmio_base[MAX_ENGINES];
	u32 ring_mi_mode[MAX_ENGINES];
	u32 ring_xxx[MAX_ENGINES];
	u8 ring_xxx_bit[MAX_ENGINES];
	u8 ring_xxx_valid;

	struct vgt_gtt_info gtt;

	struct gt_port ports[I915_MAX_PORTS];

	 /* 1 bit corresponds to 1MB in the GM space */
	DECLARE_BITMAP(gm_bitmap, VGT_GM_BITMAP_BITS);

	/* 1 bit corresponds to 1 fence register */
	DECLARE_BITMAP(fence_bitmap, VGT_FENCE_BITMAP_BITS);

	/* 1 bit corresponds to 1 PAGE(4K) in aperture */
	DECLARE_BITMAP(rsvd_aperture_bitmap, VGT_RSVD_APERTURE_BITMAP_BITS);

	struct page *dummy_page;
	struct page *(*rsvd_aperture_pages)[VGT_APERTURE_PAGES];
	gtt_entry_t dummy_gtt_entry;

	uint64_t rsvd_aperture_sz;
	uint64_t rsvd_aperture_base;
	uint64_t scratch_page;		/* page used for data written from GPU */

	struct vgt_device *device[VGT_MAX_VMS];	/* a list of running VMs */
	struct vgt_device *owner[VGT_OT_MAX];	/* owner list of different engines */
	struct vgt_device *foreground_vm;		/* current visible domain on display. */
	struct vgt_device *next_sched_vgt;
	struct vgt_device *next_foreground_vm;
	struct list_head rendering_runq_head; /* reuse this for context scheduler */
	struct list_head rendering_idleq_head; /* reuse this for context scheduler */
	spinlock_t lock;
	spinlock_t irq_lock;

	reg_info_t *reg_info;	/* virtualization policy for a given reg */
	struct vgt_irq_host_state *irq_hstate;

	DECLARE_BITMAP(dpy_emul_request, VGT_MAX_VMS);

	u8 gen_dev_type;

	u8 enable_ppgtt : 1;
	u8 in_ctx_switch : 1;
	u8 enable_execlist : 1;

	vgt_aux_entry_t vgt_aux_table[VGT_AUX_TABLE_NUM];
	int at_index;

	struct pgt_statistics stat;

	struct vgt_mmio_dev *mmio_dev;

	struct vgt_rsvd_ring ring_buffer[MAX_ENGINES]; /* vGT ring buffer */

	uint32_t opregion_pa;
	void *opregion_va;

	bool dom0_irq_pending;
	unsigned long dom0_ipi_irq_injecting;
	int dom0_irq_cpu;

	struct hotplug_work hpd_work;

	bool ctx_switch_pending;

	uint32_t el_read_ptr[MAX_ENGINES];
};

/*
 * MI_STORE_DATA is used widely for synchronization between GPU and driver,
 * which suppports the destination in either a specific hardware status
 * page, or any other aperture pages mapped to main memory. We don't want
 * to switch the hardware status page from the VM, so adopt the latter form
 * with a scratch page created as the destination with layout defined as
 * below:
 */
#define VGT_DATA_CTX_MAGIC	0x0	/* the magic number used in the context switch */
#define vgt_data_ctx_magic(d)		(d->scratch_page + VGT_DATA_CTX_MAGIC)

#define vgt_get_owner(d, t)		(d->owner[t])
#define current_render_owner(d)		(vgt_get_owner(d, VGT_OT_RENDER))
#define current_display_owner(d)	(vgt_get_owner(d, VGT_OT_DISPLAY))
#define current_foreground_vm(d)	(d->foreground_vm)
#define current_config_owner(d)		(vgt_get_owner(d, VGT_OT_CONFIG))
#define is_current_render_owner(vgt)	(vgt && vgt == current_render_owner(vgt->pdev))
#define is_current_display_owner(vgt)	(vgt && vgt == current_display_owner(vgt->pdev))
#define is_current_config_owner(vgt)	(vgt && vgt == current_config_owner(vgt->pdev))
#define ctx_switch_requested(d)		\
	(d->next_sched_vgt &&		\
	 (d->next_sched_vgt != current_render_owner(d)))
#define vgt_ctx_check(d)		(d->ctx_check)
#define vgt_ctx_switch(d)		(d->ctx_switch)
#define vgt_has_edp_enabled(vgt, pipe)							\
		(vgt && ((pipe) >= PIPE_A) && ((pipe) < I915_MAX_PIPES) &&		\
		(__vreg((vgt), _REG_PIPE_EDP_CONF) & _REGBIT_PIPE_ENABLE) &&		\
		(pipe == get_edp_input(__vreg(vgt, _REG_TRANS_DDI_FUNC_CTL_EDP))))
#define vgt_has_pipe_enabled(vgt, pipe)				\
		(vgt && ((pipe) >= PIPE_A) && ((pipe) < I915_MAX_PIPES) &&	\
		((__vreg((vgt), VGT_PIPECONF(pipe)) & _REGBIT_PIPE_ENABLE) ||	\
			vgt_has_edp_enabled(vgt, pipe)))
#define pdev_has_pipe_enabled(pdev, pipe)					\
		(pdev && ((pipe) >= PIPE_A) && ((pipe) < I915_MAX_PIPES) &&	\
		((__vreg(current_display_owner(pdev),				\
			VGT_PIPECONF(pipe)) & _REGBIT_PIPE_ENABLE) ||		\
			vgt_has_edp_enabled(current_display_owner(pdev), pipe)))
#define dpy_is_valid_port(port)							\
		(((port) >= PORT_A) && ((port) < I915_MAX_PORTS))

#define dpy_has_monitor_on_port(vgt, port)					\
		(vgt && dpy_is_valid_port(port) &&				\
		vgt->ports[port].edid && vgt->ports[port].edid->data_valid)

#define dpy_port_is_dp(vgt, port)						\
		((vgt) && dpy_is_valid_port(port)				\
		&& ((vgt->ports[port].type == VGT_DP_A) ||			\
		    (vgt->ports[port].type == VGT_DP_B) ||			\
		    (vgt->ports[port].type == VGT_DP_C) ||			\
		    (vgt->ports[port].type == VGT_DP_D)))

extern int prepare_for_display_switch(struct pgt_device *pdev);
extern void do_vgt_fast_display_switch(struct pgt_device *pdev);

#define reg_addr_fix(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_ADDR_FIX)
#define reg_hw_status(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_HW_STATUS)
#define reg_virt(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_VIRT)
#define reg_mode_ctl(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_MODE_CTL)
#define reg_passthrough(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_PASSTHROUGH)
#define reg_need_switch(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_NEED_SWITCH)
#define reg_is_tracked(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_TRACKED)
#define reg_is_accessed(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_ACCESSED)
#define reg_is_saved(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_SAVED)
#define reg_is_sticky(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_STICKY)
#define reg_get_owner(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_OWNER)
#define reg_is_render(pdev, reg)	(reg_get_owner(pdev, reg) == VGT_OT_RENDER)
#define reg_is_config(pdev, reg)	(reg_get_owner(pdev, reg) == VGT_OT_CONFIG)
#define reg_invalid(pdev, reg)		(!pdev->reg_info[REG_INDEX(reg)])
#define reg_aux_index(pdev, reg)	\
	((pdev->reg_info[REG_INDEX(reg)] & VGT_REG_INDEX_MASK) >> VGT_REG_INDEX_SHIFT)
#define reg_has_aux_info(pdev, reg)	(reg_mode_ctl(pdev, reg) | reg_addr_fix(pdev, reg))
#define reg_aux_mode_mask(pdev, reg)	\
	(pdev->vgt_aux_table[reg_aux_index(pdev, reg)].mode_ctl.mask)
#define reg_aux_addr_mask(pdev, reg)	\
	(pdev->vgt_aux_table[reg_aux_index(pdev, reg)].addr_fix.mask)
#define reg_aux_addr_size(pdev, reg)	\
	(pdev->vgt_aux_table[reg_aux_index(pdev, reg)].addr_fix.size)

#define el_read_ptr(pdev, ring_id) ((pdev)->el_read_ptr[ring_id])
#define el_write_ptr(pdev, ring_id) ((VGT_MMIO_READ((pdev), el_ring_mmio((ring_id), _EL_OFFSET_STATUS_PTR))) & 0x7 )

#define ASSERT(x)							\
	do {								\
		if (!(x)) {						\
			printk("Assert at %s line %d\n",		\
				__FILE__, __LINE__);			\
			vgt_ops->panic();				\
		}							\
	} while (0);
#define ASSERT_NUM(x, y)						\
	do {								\
		if (!(x)) {						\
			printk("Assert at %s line %d para 0x%llx\n",	\
				__FILE__, __LINE__, (u64)y);		\
			vgt_ops->panic();				\
		}							\
	} while (0);

static inline void reg_set_hw_status(struct pgt_device *pdev, vgt_reg_t reg)
{
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_HW_STATUS;
}

static inline void reg_set_virt(struct pgt_device *pdev, vgt_reg_t reg)
{
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_VIRT;
}

/* mask bits for addr fix */
static inline void reg_set_addr_fix(struct pgt_device *pdev,
	vgt_reg_t reg, vgt_reg_t mask)
{
	ASSERT(!reg_has_aux_info(pdev, reg));
	ASSERT(pdev->at_index <= VGT_AUX_TABLE_NUM - 1);
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);

	pdev->vgt_aux_table[pdev->at_index].addr_fix.mask = mask;
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_ADDR_FIX |
		(pdev->at_index << VGT_REG_INDEX_SHIFT);
	pdev->at_index++;
}

/* mask bits for mode mask */
static inline void reg_set_mode_ctl(struct pgt_device *pdev,
	vgt_reg_t reg)
{
	ASSERT(!reg_has_aux_info(pdev, reg));
	ASSERT(pdev->at_index <= VGT_AUX_TABLE_NUM - 1);
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);

	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_MODE_CTL |
		(pdev->at_index << VGT_REG_INDEX_SHIFT);
	pdev->at_index++;
}

/* if the type is invalid, we assume dom0 always has the permission */
static inline bool reg_is_owner(struct vgt_device *vgt, vgt_reg_t reg)
{
	enum vgt_owner_type type;

	type = vgt->pdev->reg_info[REG_INDEX(reg)] & VGT_REG_OWNER;
	return vgt == vgt_get_owner(vgt->pdev, type);
}

static inline void reg_set_owner(struct pgt_device *pdev,
	vgt_reg_t reg, enum vgt_owner_type type)
{
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] |= type & VGT_REG_OWNER;
}

static inline void reg_change_owner(struct pgt_device *pdev,
	vgt_reg_t reg, enum vgt_owner_type type)
{
	ASSERT_NUM(reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] &= ~VGT_REG_OWNER;
	pdev->reg_info[REG_INDEX(reg)] |= type & VGT_REG_OWNER;
	if ((type != VGT_OT_NONE) && (type != VGT_OT_MAX))
		pdev->reg_info[REG_INDEX(reg)] &= ~VGT_REG_VIRT;
}

static inline void reg_set_passthrough(struct pgt_device *pdev,
	vgt_reg_t reg)
{
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_PASSTHROUGH;
}

static inline void reg_set_tracked(struct pgt_device *pdev,
	vgt_reg_t reg)
{
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_TRACKED;
}

static inline void reg_set_accessed(struct pgt_device *pdev,
	vgt_reg_t reg)
{
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_ACCESSED;
}

static inline void reg_set_saved(struct pgt_device *pdev,
	vgt_reg_t reg)
{
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_SAVED;
}

static inline void reg_set_sticky(struct pgt_device *pdev,
	vgt_reg_t reg)
{
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_STICKY;
}

static inline void reg_set_cmd_access(struct pgt_device *pdev,
	vgt_reg_t reg)
{
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_CMD_ACCESS;
	reg_set_accessed(pdev, reg);
}

static inline void reg_update_handlers(struct pgt_device *pdev,
	vgt_reg_t reg, int size, vgt_mmio_read read, vgt_mmio_write write)
{
	ASSERT_NUM(reg_is_tracked(pdev, reg), reg);
	/* TODO search attr table to update fields there */
	vgt_register_mmio_handler(reg, size, read, write);
}

/* request types to wake up main thread */
#define VGT_REQUEST_IRQ		0	/* a new irq pending from device */
#define VGT_REQUEST_UEVENT	1
#define VGT_REQUEST_CTX_SWITCH	2	/* immediate reschedule(context switch) requested */
#define VGT_REQUEST_EMUL_DPY_EVENTS	3
#define VGT_REQUEST_DPY_SWITCH	4	/* immediate reschedule(display switch) requested */
#define VGT_REQUEST_DEVICE_RESET 5
#define VGT_REQUEST_SCHED	6
#define VGT_REQUEST_CTX_EMULATION_RCS	7 /* Emulate context switch irq of Gen8 */
#define VGT_REQUEST_CTX_EMULATION_VCS	8 /* Emulate context switch irq of Gen8 */
#define VGT_REQUEST_CTX_EMULATION_BCS	9 /* Emulate context switch irq of Gen8 */
#define VGT_REQUEST_CTX_EMULATION_VECS	10 /* Emulate context switch irq of Gen8 */
#define VGT_REQUEST_CTX_EMULATION_VCS2	11 /* Emulate context switch irq of Gen8 */

static inline void vgt_raise_request(struct pgt_device *pdev, uint32_t flag)
{
	set_bit(flag, (void *)&pdev->request);
	if (waitqueue_active(&pdev->event_wq))
		wake_up(&pdev->event_wq);
}

static inline bool vgt_chk_raised_request(struct pgt_device *pdev, uint32_t flag)
{
	return !!(test_bit(flag, (void *)&pdev->request));
}

/* check whether a reg access should happen on real hw */
static inline bool reg_hw_access(struct vgt_device *vgt, unsigned int reg)
{
	struct pgt_device *pdev = vgt->pdev;

	/*
	 * In superowner mode, all registers, except those explicitly marked
	 * as sticky, are virtualized to Dom0 while passthrough to the 1st
	 * HVM.
	 */
	if (hvm_super_owner && !reg_is_sticky(pdev, reg)) {
		if (vgt->vgt_id)
			return true;
		else
			return false;
	}

	/* allows access from any VM. dangerous!!! */
	if (reg_passthrough(pdev, reg))
		return true;

	/* normal phase of passthrough registers if vgt is the owner */
	if (reg_is_owner(vgt, reg))
		return true;

	//ASSERT(reg_virt(pdev, reg));
	return false;
}

#define IGD_INVALID	0
#define IGD_SNB		1
#define IGD_IVB		2
#define IGD_HSW		3
#define IGD_BDW		4
#define IGD_MAX		IGD_BDW

#define IS_SNB(pdev)	((pdev)->gen_dev_type == IGD_SNB)
#define IS_IVB(pdev)	((pdev)->gen_dev_type == IGD_IVB)
#define IS_HSW(pdev)	((pdev)->gen_dev_type == IGD_HSW)
#define IS_BDW(pdev)	((pdev)->gen_dev_type == IGD_BDW)

#define IS_PREBDW(pdev) (IS_SNB(pdev) || IS_IVB(pdev) || IS_HSW(pdev))
#define IS_BDWPLUS(pdev) (IS_BDW(pdev))
#define IS_BDWGT3(pdev) (IS_BDW(pdev) && (GEN_REV(pdev->device_info.gen) == 3))

#define D_SNB	(1 << 0)
#define D_IVB	(1 << 1)
#define D_HSW	(1 << 2)
#define D_BDW	(1 << 3)

#define D_GEN8PLUS	(D_BDW)
#define D_GEN75PLUS	(D_HSW | D_BDW)
#define D_GEN7PLUS	(D_IVB | D_HSW | D_BDW)

#define D_BDW_PLUS	(D_BDW)
#define D_HSW_PLUS	(D_HSW | D_BDW)
#define D_IVB_PLUS	(D_IVB | D_HSW | D_BDW)

#define D_PRE_BDW	(D_SNB | D_IVB | D_HSW)

#define D_ALL		(D_SNB | D_IVB | D_HSW | D_BDW)

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

/*
 * Comments copied from i915 driver - i915_reg.h :
 * Haswell does have the CXT_SIZE register however it does not appear to be
 * valid. Now, docs explain in dwords what is in the context object. The full
 * size is 70720 bytes, however, the power context and execlist context will
 * never be saved (power context is stored elsewhere, and execlists don't work
 * on HSW) - so the final size is 66944 bytes, which rounds to 17 pages.
 */
#define HSW_CXT_TOTAL_SIZE		(17 * PAGE_SIZE)

typedef struct {
	vgt_reg_t   reg;
	u32			size;
	int			device;
} reg_addr_sz_t;

static inline unsigned int vgt_gen_dev_type(struct pgt_device *pdev)
{
	if (IS_SNB(pdev))
		return D_SNB;
	if (IS_IVB(pdev))
		return D_IVB;
	if (IS_HSW(pdev))
		return D_HSW;
	if (IS_BDW(pdev))
		return D_BDW;
	WARN_ONCE(1, KERN_ERR "vGT: unknown GEN type!\n");
	return 0;
}

static inline bool vgt_match_device_attr(struct pgt_device *pdev, reg_attr_t *attr)
{
	return attr->device & vgt_gen_dev_type(pdev);
}

static inline enum vgt_port vgt_get_port(struct vgt_device *vgt, struct gt_port *port_ptr)
{
	enum vgt_port port_type;

	if (!vgt || !port_ptr)
		return I915_MAX_PORTS;

	for (port_type = PORT_A; port_type < I915_MAX_PORTS; ++ port_type)
		if (port_ptr == &vgt->ports[port_type])
			break;

	return port_type;
}

static inline enum vgt_pipe vgt_get_pipe_from_port(struct vgt_device *vgt,
						enum vgt_port port)
{
	enum vgt_pipe pipe;

	if (port == I915_MAX_PORTS)
		return I915_MAX_PIPES;

	ASSERT (port != PORT_A);

	for (pipe = PIPE_A; pipe < I915_MAX_PIPES; ++ pipe) {
		vgt_reg_t ddi_func_ctl;
		vgt_reg_t ddi_port_info;

		ddi_func_ctl  = __vreg(vgt, _VGT_TRANS_DDI_FUNC_CTL(pipe));

		if (!(ddi_func_ctl & _REGBIT_TRANS_DDI_FUNC_ENABLE))
			continue;

		ddi_port_info = (ddi_func_ctl & _REGBIT_TRANS_DDI_PORT_MASK) >>
					_TRANS_DDI_PORT_SHIFT;
		if (ddi_port_info == port) {
			// pipe has the port setting same as input
			break;
		}
	}

	return pipe;
}

static inline int tail_to_ring_id(struct pgt_device *pdev, unsigned int tail_off)
{
	int i;

	for (i = 0; i < pdev->max_engines; i++) {
		if ( pdev->ring_mmio_base[i] == tail_off )
			return i;
	}
	printk("Wrong tail register %s\n", __FUNCTION__);
	ASSERT(0);
	return 0;
}

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

extern int vgt_ctx_switch;
extern bool vgt_validate_ctx_switch;
extern bool fastpath_dpy_switch;
extern void vgt_toggle_ctx_switch(bool enable);
extern void vgt_kick_off_ringbuffers(struct vgt_device *vgt);
extern void vgt_kick_off_execution(struct vgt_device *vgt);
extern void vgt_setup_reg_info(struct pgt_device *pdev);
extern bool vgt_post_setup_mmio_hooks(struct pgt_device *pdev);
extern bool vgt_initial_mmio_setup (struct pgt_device *pdev);
extern void vgt_initial_opregion_setup(struct pgt_device *pdev);
extern void state_vreg_init(struct vgt_device *vgt);
extern void state_sreg_init(struct vgt_device *vgt);

/* definitions for physical aperture/GM space */
#define phys_aperture_sz(pdev)		(pdev->bar_size[1])
#define phys_aperture_pages(pdev)	(phys_aperture_sz(pdev) >> GTT_PAGE_SHIFT)
#define phys_aperture_base(pdev)	(pdev->gmadr_base)
#define phys_aperture_vbase(pdev)	(pdev->gmadr_va)

#define gm_sz(pdev)			(pdev->total_gm_sz)
#define gm_base(pdev)			(0ULL)
#define gm_pages(pdev)			(gm_sz(pdev) >> GTT_PAGE_SHIFT)
#define hidden_gm_base(pdev)		(phys_aperture_sz(pdev))

#define aperture_2_gm(pdev, addr)	(addr - phys_aperture_base(pdev))
#define v_aperture(pdev, addr)		(phys_aperture_vbase(pdev) + (addr))

#define vm_aperture_sz(pdev)		(pdev->vm_aperture_sz)
#define vm_gm_sz(pdev)			(pdev->vm_gm_sz)
#define vm_gm_hidden_sz(pdev)		(vm_gm_sz(pdev) - vm_aperture_sz(pdev))

static inline uint64_t vgt_mmio_bar_base(struct vgt_device *vgt)
{
	char *cfg_space = &vgt->state.cfg_space[0];
	return *(uint64_t *)(cfg_space + VGT_REG_CFG_SPACE_BAR0);
}


/*
 * Aperture/GM virtualization
 *
 * NOTE: the below description says dom0's aperture starts at a non-zero place,
 * this is only true if you enable the dom0's kernel parameter
 * dom0_aperture_starts_at_128MB: now by default dom0's aperture starts at 0 of
 * the GM space since dom0 is the first vm to request for GM space.
 *
 * GM is split into two parts: the 1st part visible to CPU through an aperture
 * window mapping, and the 2nd part only accessible from GPU. The virtualization
 * policy is like below:
 *
 *                | VM1 | VM2 | DOM0| RSVD|    VM1   |    VM2   |
 *                ------------------------------------------------
 * Aperture Space |/////|\\\\\|xxxxx|ooooo|                     v
 * (Dev2_BAR)     v                       v                     v
 *                v                       v                     v
 * GM space       v   (visibale part)     v   (invisible part)  v
 * (start from 0) |/////|\\\\\|xxxxx|ooooo|//////////|\\\\\\\\\\|
 *                ^     ^                 ^          ^
 *                |     |  _______________|          |
 *                |     | /          ________________|
 * VM1 GM space   |     |/          /
 * (start from 0) |/////|//////////|
 */

/* definitions for vgt's aperture/gm space */
#define vgt_aperture_base(vgt)		(vgt->aperture_base)
#define vgt_aperture_vbase(vgt)		(vgt->aperture_base_va)
#define vgt_aperture_offset(vgt)	(vgt->aperture_offset)
#define vgt_hidden_gm_offset(vgt)	(vgt->hidden_gm_offset)
#define vgt_aperture_sz(vgt)		(vgt->aperture_sz)
#define vgt_gm_sz(vgt)			(vgt->gm_sz)
#define vgt_hidden_gm_sz(vgt)		(vgt_gm_sz(vgt) - vgt_aperture_sz(vgt))

#define vgt_aperture_end(vgt)		\
	(vgt_aperture_base(vgt) + vgt_aperture_sz(vgt) - 1)
#define vgt_visible_gm_base(vgt)	\
	(gm_base(vgt->pdev) + vgt_aperture_offset(vgt))
#define vgt_visible_gm_end(vgt)		\
	(vgt_visible_gm_base(vgt) + vgt_aperture_sz(vgt) - 1)
#define vgt_hidden_gm_base(vgt)	\
	(gm_base(vgt->pdev) + vgt_hidden_gm_offset(vgt))
#define vgt_hidden_gm_end(vgt)		\
	(vgt_hidden_gm_base(vgt) + vgt_hidden_gm_sz(vgt) - 1)

/*
 * the view of the aperture/gm space from the VM's p.o.v
 *
 * when the VM supports ballooning, this view is the same as the
 * view of vGT driver.
 *
 * when the VM does not support ballooning, this view starts from
 * GM space ZERO
 */
#define vgt_guest_aperture_base(vgt)	\
	(vgt->ballooning ?		\
		(*((u32*)&vgt->state.cfg_space[VGT_REG_CFG_SPACE_BAR1]) & ~0xf) + vgt_aperture_offset(vgt) :	\
		(*((u32*)&vgt->state.cfg_space[VGT_REG_CFG_SPACE_BAR1]) & ~0xf))
#define vgt_guest_aperture_end(vgt)	\
	(vgt_guest_aperture_base(vgt) + vgt_aperture_sz(vgt) - 1)
#define vgt_guest_visible_gm_base(vgt)	\
	(vgt->ballooning ? vgt_visible_gm_base(vgt) : gm_base(vgt->pdev))
#define vgt_guest_visible_gm_end(vgt)	\
	(vgt_guest_visible_gm_base(vgt) + vgt_aperture_sz(vgt) - 1)
#define vgt_guest_hidden_gm_base(vgt)	\
	(vgt->ballooning ?		\
		vgt_hidden_gm_base(vgt) :	\
		vgt_guest_visible_gm_end(vgt) + 1)
#define vgt_guest_hidden_gm_end(vgt)	\
	(vgt_guest_hidden_gm_base(vgt) + vgt_hidden_gm_sz(vgt) - 1)

#if 0
/* These unused functions are for non-ballooning case. */
/* translate a guest aperture address to host aperture address */
static inline uint64_t g2h_aperture(struct vgt_device *vgt, uint64_t g_addr)
{
	uint64_t offset;

	ASSERT_NUM((g_addr >= vgt_guest_aperture_base(vgt)) &&
		(g_addr <= vgt_guest_aperture_end(vgt)), g_addr);

	offset = g_addr - vgt_guest_aperture_base(vgt);
	return vgt_aperture_base(vgt) + offset;
}

/* translate a host aperture address to guest aperture address */
static inline uint64_t h2g_aperture(struct vgt_device *vgt, uint64_t h_addr)
{
	uint64_t offset;

	ASSERT_NUM((h_addr >= vgt_aperture_base(vgt)) &&
		(h_addr <= vgt_aperture_end(vgt)), h_addr);

	offset = h_addr - vgt_aperture_base(vgt);
	return vgt_guest_aperture_base(vgt) + offset;
}
#endif

/* check whether a guest GM address is within the CPU visible range */
static inline bool g_gm_is_visible(struct vgt_device *vgt, uint64_t g_addr)
{
	return (g_addr >= vgt_guest_visible_gm_base(vgt)) &&
		(g_addr <= vgt_guest_visible_gm_end(vgt));
}

/* check whether a guest GM address is out of the CPU visible range */
static inline bool g_gm_is_hidden(struct vgt_device *vgt, uint64_t g_addr)
{
	return (g_addr >= vgt_guest_hidden_gm_base(vgt)) &&
		(g_addr <= vgt_guest_hidden_gm_end(vgt));
}

static inline bool g_gm_is_reserved(struct vgt_device *vgt, uint64_t g_addr)
{
	uint64_t rsvd_gm_base = aperture_2_gm(vgt->pdev,
					vgt->pdev->rsvd_aperture_base);

	return ((g_addr >= rsvd_gm_base) &&
		(g_addr < (rsvd_gm_base + vgt->pdev->rsvd_aperture_sz)));
}

static inline bool g_gm_is_valid(struct vgt_device *vgt, uint64_t g_addr)
{
	if (vgt->bypass_addr_check)
		return false;

	return g_gm_is_visible(vgt, g_addr) || g_gm_is_hidden(vgt, g_addr);
}

/* check whether a host GM address is within the CPU visible range */
static inline bool h_gm_is_visible(struct vgt_device *vgt, uint64_t h_addr)
{
	if (vgt->bypass_addr_check)
		return true;

	return (h_addr >= vgt_visible_gm_base(vgt)) &&
		(h_addr <= vgt_visible_gm_end(vgt));
}

/* check whether a host GM address is out of the CPU visible range */
static inline bool h_gm_is_hidden(struct vgt_device *vgt, uint64_t h_addr)
{
	if (vgt->bypass_addr_check)
		return true;

	return (h_addr >= vgt_hidden_gm_base(vgt)) &&
		(h_addr <= vgt_hidden_gm_end(vgt));
}

static inline bool h_gm_is_valid(struct vgt_device *vgt, uint64_t h_addr)
{
	return h_gm_is_visible(vgt, h_addr) || h_gm_is_hidden(vgt, h_addr);
}

/* for a guest GM address, return the offset within the CPU visible range */
static inline uint64_t g_gm_visible_offset(struct vgt_device *vgt, uint64_t g_addr)
{
	return g_addr - vgt_guest_visible_gm_base(vgt);
}

/* for a guest GM address, return the offset within the hidden range */
static inline uint64_t g_gm_hidden_offset(struct vgt_device *vgt, uint64_t g_addr)
{
	return g_addr - vgt_guest_hidden_gm_base(vgt);
}

/* for a host GM address, return the offset within the CPU visible range */
static inline uint64_t h_gm_visible_offset(struct vgt_device *vgt, uint64_t h_addr)
{
	return h_addr - vgt_visible_gm_base(vgt);
}

/* for a host GM address, return the offset within the hidden range */
static inline uint64_t h_gm_hidden_offset(struct vgt_device *vgt, uint64_t h_addr)
{
	return h_addr - vgt_hidden_gm_base(vgt);
}

/* validate a gm address and related range size, translate it to host gm address */
static inline int g2h_gm_range(struct vgt_device *vgt, uint64_t *addr, uint32_t size)
{
	ASSERT(addr);

	if (vgt->bypass_addr_check)
		return 0;

	if ((!g_gm_is_valid(vgt, *addr)) || (size && !g_gm_is_valid(vgt, *addr + size - 1))) {
		vgt_err("VM(%d): invalid address range: g_addr(0x%llx), size(0x%x)\n",
			vgt->vm_id, *addr, size);
		return -EACCES;
	}

	if (g_gm_is_visible(vgt, *addr))	/* aperture */
		*addr = vgt_visible_gm_base(vgt) +
			g_gm_visible_offset(vgt, *addr);
	else	/* hidden GM space */
		*addr = vgt_hidden_gm_base(vgt) +
			g_gm_hidden_offset(vgt, *addr);
	return 0;
}

/* translate a guest gm address to host gm address */
static inline int g2h_gm(struct vgt_device *vgt, uint64_t *addr)
{
	return g2h_gm_range(vgt, addr, 4);
}

/* translate a host gm address to guest gm address */
static inline uint64_t h2g_gm(struct vgt_device *vgt, uint64_t h_addr)
{
	uint64_t g_addr;

	if (vgt->bypass_addr_check)
		return h_addr;

	ASSERT_NUM(h_gm_is_valid(vgt, h_addr), h_addr);

	if (h_gm_is_visible(vgt, h_addr))
		g_addr = vgt_guest_visible_gm_base(vgt) +
			h_gm_visible_offset(vgt, h_addr);
	else
		g_addr = vgt_guest_hidden_gm_base(vgt) +
			h_gm_hidden_offset(vgt, h_addr);

	return g_addr;
}

extern unsigned long rsvd_aperture_alloc(struct pgt_device *pdev,
		unsigned long size);
extern void rsvd_aperture_free(struct pgt_device *pdev, unsigned long start,
		unsigned long size);

#if 0
/* This unused function is for non-ballooning case. */
/*
 * check whether a structure pointed by MMIO, or an instruction filled in
 * the command buffer, may cross the visible and invisible boundary. That
 * should be avoid since physically two parts are not contiguous
 */
static inline bool check_g_gm_cross_boundary(struct vgt_device *vgt,
	uint64_t g_start, uint64_t size)
{
	if (vgt->bypass_addr_check)
		return false;

	if (!vgt_hidden_gm_offset(vgt))
		return false;

	return g_gm_is_visible(vgt, g_start) &&
		g_gm_is_hidden(vgt, g_start + size - 1);
}
#endif

#define reg_is_mmio(pdev, reg)	\
	(reg >= 0 && reg < pdev->mmio_size)
#define reg_is_gtt(pdev, reg)	\
	(reg >= pdev->device_info.gtt_start_offset \
	&& reg < pdev->device_info.gtt_start_offset + pdev->gtt_size)

#define GTT_INDEX(pdev, addr)		\
	((u32)((addr - gm_base(pdev)) >> GTT_PAGE_SHIFT))

static inline uint32_t g2h_gtt_index(struct vgt_device *vgt, uint32_t g_index)
{
	uint64_t addr = g_index << GTT_PAGE_SHIFT;

	g2h_gm(vgt, &addr);

	return (uint32_t)(addr >> GTT_PAGE_SHIFT);
}

static inline uint32_t h2g_gtt_index(struct vgt_device *vgt, uint32_t h_index)
{
	uint64_t h_addr = h_index << GTT_PAGE_SHIFT;

	return (uint32_t)(h2g_gm(vgt, h_addr) >> GTT_PAGE_SHIFT);
}

static inline void __REG_WRITE(struct pgt_device *pdev,
	unsigned long reg, unsigned long val, int bytes)
{
	int ret;

	/*
	 * TODO: a simple mechanism to capture registers being
	 * saved/restored at render/display context switch time.
	 * It's not accurate, since vGT's normal mmio access
	 * within that window also falls here. But suppose that
	 * set is small for now.
	 *
	 * In the future let's wrap interface like vgt_restore_vreg
	 * for accurate tracking purpose.
	 */
	if (pdev->in_ctx_switch)
		reg_set_saved(pdev, reg);
	ret = vgt_native_mmio_write(reg, &val, bytes, false);
}

static inline unsigned long __REG_READ(struct pgt_device *pdev,
	unsigned long reg, int bytes)
{
	unsigned long data = 0;
	int ret;

	if (pdev->in_ctx_switch)
		reg_set_saved(pdev, reg);
	ret = vgt_native_mmio_read(reg, &data, bytes, false);
	return data;
}

#define VGT_MMIO_READ_BYTES(pdev, mmio_offset, bytes)	\
		__REG_READ(pdev, mmio_offset, bytes)

#define VGT_MMIO_WRITE_BYTES(pdev, mmio_offset, val, bytes)	\
		__REG_WRITE(pdev, mmio_offset, val, bytes)

#define VGT_MMIO_WRITE(pdev, mmio_offset, val)	\
		VGT_MMIO_WRITE_BYTES(pdev, mmio_offset, (unsigned long)val, REG_SIZE)

#define VGT_MMIO_READ(pdev, mmio_offset)		\
		((vgt_reg_t)VGT_MMIO_READ_BYTES(pdev, mmio_offset, REG_SIZE))

#define VGT_MMIO_WRITE64(pdev, mmio_offset, val)	\
		__REG_WRITE(pdev, mmio_offset, val, 8)

#define VGT_MMIO_READ64(pdev, mmio_offset)		\
		__REG_READ(pdev, mmio_offset, 8)

#define VGT_REG_IS_ALIGNED(reg, bytes) (!((reg)&((bytes)-1)))
#define VGT_REG_ALIGN(reg, bytes) ((reg) & ~((bytes)-1))

#define vgt_restore_vreg(vgt, off)		\
	VGT_MMIO_WRITE(vgt->pdev, off, __vreg(vgt, off))

#define ARRAY_NUM(x)		(sizeof(x) / sizeof(x[0]))

/* context scheduler */
#define CYCLES_PER_USEC	0x10c7ull
#define VGT_DEFAULT_TSLICE (4 * 1000 * CYCLES_PER_USEC)
#define ctx_start_time(vgt) ((vgt)->sched_info.start_time)
#define ctx_end_time(vgt) ((vgt)->sched_info.end_time)
#define ctx_remain_time(vgt) ((vgt)->sched_info.time_slice)
#define ctx_actual_end_time(vgt) ((vgt)->sched_info.actual_end_time)
#define ctx_rb_empty_delay(vgt) ((vgt)->sched_info.rb_empty_delay)
#define ctx_tbs_period(vgt) ((vgt)->sched_info.tbs_period)

#define vgt_get_cycles() ({		\
	cycles_t __ret;				\
	rdtsc_barrier();			\
	__ret = get_cycles();		\
	rdtsc_barrier();			\
	__ret;						\
	})

#define RB_HEAD_TAIL_EQUAL(head, tail) \
	(((head) & RB_HEAD_OFF_MASK) == ((tail) & RB_TAIL_OFF_MASK))

extern bool event_based_qos;
extern bool vgt_vrings_empty(struct vgt_device *vgt);

/* context scheduler facilities functions */
static inline bool vgt_runq_is_empty(struct pgt_device *pdev)
{
	return (list_empty(&pdev->rendering_runq_head));
}

static inline void vgt_runq_insert(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	list_add(&vgt->list, &pdev->rendering_runq_head);
}

static inline void vgt_runq_remove(struct vgt_device *vgt)
{
	list_del(&vgt->list);
}

static inline void vgt_idleq_insert(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	list_add(&vgt->list, &pdev->rendering_idleq_head);
}

static inline void vgt_idleq_remove(struct vgt_device *vgt)
{
	list_del(&vgt->list);
}

static inline int vgt_nr_in_runq(struct pgt_device *pdev)
{
	int count = 0;
	struct list_head *pos;
	list_for_each(pos, &pdev->rendering_runq_head)
		count++;
	return count;
}

static inline void vgt_init_sched_info(struct vgt_device *vgt)
{
	if (event_based_qos) {
		ctx_remain_time(vgt) = VGT_DEFAULT_TSLICE;
		ctx_start_time(vgt) = 0;
		ctx_end_time(vgt) = 0;
		ctx_actual_end_time(vgt) = 0;
		ctx_rb_empty_delay(vgt) = 0;
	}

	if (timer_based_qos) {

		if (tbs_period_ms == -1) {
			tbs_period_ms = IS_BDW(vgt->pdev) ?
				VGT_TBS_PERIOD_MIN : VGT_TBS_PERIOD_MAX;
		}

		if (tbs_period_ms > VGT_TBS_PERIOD_MAX
			|| tbs_period_ms < VGT_TBS_PERIOD_MIN) {
			vgt_err("Invalid tbs_period=%d parameters. "
				"Best value between <%d..%d>\n",
				VGT_TBS_PERIOD_MIN, VGT_TBS_PERIOD_MAX,
				tbs_period_ms);
			tbs_period_ms = IS_BDW(vgt->pdev) ?
				VGT_TBS_PERIOD_MIN : VGT_TBS_PERIOD_MAX;
		}

		ctx_tbs_period(vgt) = VGT_TBS_DEFAULT_PERIOD(tbs_period_ms);
		vgt_info("VM-%d setup timebased schedule period %d ms\n",
			vgt->vm_id, tbs_period_ms);
	}
}

/* main context scheduling process */
extern void vgt_sched_ctx(struct pgt_device *pdev);
extern void vgt_setup_countdown(struct vgt_device *vgt);
extern void vgt_initialize_ctx_scheduler(struct pgt_device *pdev);
extern void vgt_cleanup_ctx_scheduler(struct pgt_device *pdev);

extern void __raise_ctx_sched(struct vgt_device *vgt);
#define raise_ctx_sched(vgt) \
	if (event_based_qos)	\
		__raise_ctx_sched((vgt))

extern bool shadow_tail_based_qos;
int vgt_init_rb_tailq(struct vgt_device *vgt);
void vgt_destroy_rb_tailq(struct vgt_device *vgt);
int vgt_tailq_pushback(struct vgt_tailq *tailq, u32 tail, u32 cmdnr);
u32 vgt_tailq_last_stail(struct vgt_tailq *tailq);
/*
 *
 * Activate a VGT instance to render runqueue.
 */
static inline void vgt_enable_render(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	ASSERT(spin_is_locked(&pdev->lock));
	if (bitmap_empty(vgt->enabled_rings, MAX_ENGINES))
		printk("vGT-%d: Enable render but no ring is enabled yet\n",
			vgt->vgt_id);
	/* remove from idle queue */
	list_del(&vgt->list);
	/* add to run queue */
	list_add(&vgt->list, &pdev->rendering_runq_head);
	printk("vGT-%d: add to render run queue!\n", vgt->vgt_id);
}

/* now we scheduler all render rings together */
/* whenever there is a ring enabled, the render(context switch ?) are enabled */
static inline void vgt_enable_ring(struct vgt_device *vgt, int ring_id)
{
	int enable = bitmap_empty(vgt->enabled_rings, MAX_ENGINES);

	set_bit(ring_id, (void *)vgt->enabled_rings);
	if (enable)
		vgt_enable_render(vgt);
}

/*
 * Remove a VGT instance from render runqueue.
 */
static inline void vgt_disable_render(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	ASSERT(spin_is_locked(&pdev->lock));
	if (!bitmap_empty(vgt->enabled_rings, MAX_ENGINES))
		printk("vGT-%d: disable render with enabled rings\n",
			vgt->vgt_id);
	/* remove from run queue */
	list_del(&vgt->list);
	/* add to idle queue */
	list_add(&vgt->list, &pdev->rendering_idleq_head);
	printk("vGT-%d: remove from render run queue!\n", vgt->vgt_id);
}

static inline void vgt_disable_ring(struct vgt_device *vgt, int ring_id)
{
	struct pgt_device *pdev = vgt->pdev;

	clear_bit(ring_id, (void *)vgt->started_rings);

	/* multiple disables */
	if (!test_and_clear_bit(ring_id, (void *)vgt->enabled_rings)) {
		printk("vGT-%d: disable a disabled ring (%d)\n",
			vgt->vgt_id, ring_id);
		return;
	}

	/* request to remove from runqueue if all rings are disabled */
	if (bitmap_empty(vgt->enabled_rings, MAX_ENGINES)) {
		ASSERT(spin_is_locked(&pdev->lock));
		if (current_render_owner(pdev) == vgt) {
			pdev->next_sched_vgt = vgt_dom0;
			vgt_raise_request(pdev, VGT_REQUEST_SCHED);
		} else
			vgt_disable_render(vgt);
	}
}

static inline uint32_t vgt_ring_id_to_EL_base(enum vgt_ring_id ring_id)
{
	uint32_t base = 0;

	switch (ring_id) {
	case RING_BUFFER_RCS:
		base = _EL_BASE_RCS;
		break;
	case RING_BUFFER_VCS:
		base = _EL_BASE_VCS;
		break;
	case RING_BUFFER_VECS:
		base = _EL_BASE_VECS;
		break;
	case RING_BUFFER_VCS2:
		base = _EL_BASE_VCS2;
		break;
	case RING_BUFFER_BCS:
		base = _EL_BASE_BCS;
		break;
	default:
		BUG();
	}
	return base;
}

#define el_ring_mmio(ring_id, offset_to_base) \
(vgt_ring_id_to_EL_base((ring_id)) + (offset_to_base))

static inline enum vgt_event_type vgt_ring_id_to_ctx_event(enum vgt_ring_id ring_id)
{
	enum vgt_event_type event;

	switch (ring_id) {
	case RING_BUFFER_RCS:
		event = RCS_AS_CONTEXT_SWITCH;
		break;
	case RING_BUFFER_VCS:
		event = VCS_AS_CONTEXT_SWITCH;
		break;
	case RING_BUFFER_VECS:
		event = VECS_AS_CONTEXT_SWITCH;
		break;
	case RING_BUFFER_VCS2:
		event = VCS2_AS_CONTEXT_SWITCH;
		break;
	case RING_BUFFER_BCS:
		event = BCS_AS_CONTEXT_SWITCH;
		break;
	default:
		BUG();
	}
	return event;
}

static inline bool is_ring_empty(struct pgt_device *pdev, int ring_id)
{
	if (pdev->enable_execlist) {
		struct execlist_status_format status;
		uint32_t status_reg = vgt_ring_id_to_EL_base(ring_id)
						+ _EL_OFFSET_STATUS;
		status.ldw = VGT_MMIO_READ(pdev, status_reg);
		status.udw = VGT_MMIO_READ(pdev, status_reg + 4);
		return ((status.execlist_0_active == 0) &&
				(status.execlist_1_active == 0));
	} else {
		vgt_reg_t head = VGT_MMIO_READ(pdev, RB_HEAD(pdev, ring_id));
		vgt_reg_t tail = VGT_MMIO_READ(pdev, RB_TAIL(pdev, ring_id));

		head &= RB_HEAD_OFF_MASK;
		/*
		 * PRM said bit2-20 for head count, but bit3-20 for tail count:
		 * this means: HW increases HEAD by 4, and SW must increase TAIL
		 * by 8(SW must add padding of MI_NOOP if necessary).
		 */
		tail &= RB_TAIL_OFF_MASK;
		return (head == tail);
	}
}

#define VGT_POST_READ(pdev, reg)		\
	do {					\
		vgt_reg_t val;			\
		val = VGT_MMIO_READ(pdev, reg);	\
	} while (0)

#define VGT_READ_CTL(pdev, id)	VGT_MMIO_READ(pdev, RB_CTL(pdev, id))
#define VGT_WRITE_CTL(pdev, id, val) VGT_MMIO_WRITE(pdev, RB_CTL(pdev, id), val)
#define VGT_POST_READ_CTL(pdev, id)	VGT_POST_READ(pdev, RB_CTL(pdev,id))

#define VGT_READ_HEAD(pdev, id)	VGT_MMIO_READ(pdev, RB_HEAD(pdev, id))
#define VGT_WRITE_HEAD(pdev, id, val) VGT_MMIO_WRITE(pdev, RB_HEAD(pdev, id), val)
#define VGT_POST_READ_HEAD(pdev, id)	VGT_POST_READ(pdev, RB_HEAD(pdev,id))

#define VGT_READ_TAIL(pdev, id)	VGT_MMIO_READ(pdev, RB_TAIL(pdev, id))
#define VGT_WRITE_TAIL(pdev, id, val) VGT_MMIO_WRITE(pdev, RB_TAIL(pdev, id), val)
#define VGT_POST_READ_TAIL(pdev, id)	VGT_POST_READ(pdev, RB_TAIL(pdev,id))

#define VGT_READ_START(pdev, id) VGT_MMIO_READ(pdev, RB_START(pdev, id))
#define VGT_WRITE_START(pdev, id, val) VGT_MMIO_WRITE(pdev, RB_START(pdev, id), val)
#define VGT_POST_READ_START(pdev, id)	VGT_POST_READ(pdev, RB_START(pdev,id))

static inline bool is_ring_enabled (struct pgt_device *pdev, int ring_id)
{
	return (VGT_MMIO_READ(pdev, RB_CTL(pdev, ring_id)) & 1);	/* bit 0: enable/disable RB */
}
extern void vgt_ring_init(struct pgt_device *pdev, int id);

static inline u32 vgt_read_gtt(struct pgt_device *pdev, u32 index)
{
	struct vgt_device_info *info = &pdev->device_info;
	unsigned int off = index << info->gtt_entry_size_shift;
	u32 ret = 0;

	vgt_native_gtt_read(off, &ret, sizeof(ret));
	return ret;
}

static inline void vgt_write_gtt(struct pgt_device *pdev, u32 index, u32 val)
{
	struct vgt_device_info *info = &pdev->device_info;
	unsigned int off = index << info->gtt_entry_size_shift;

	vgt_native_gtt_write(off, &val, sizeof(val));
}

static inline u64 vgt_read_gtt64(struct pgt_device *pdev, u32 index)
{
	struct vgt_device_info *info = &pdev->device_info;
	unsigned int off = index << info->gtt_entry_size_shift;
	u64 ret = 0;

	vgt_native_gtt_read(off, &ret, sizeof(ret));
	return ret;
}

static inline void vgt_write_gtt64(struct pgt_device *pdev, u32 index, u64 val)
{
	struct vgt_device_info *info = &pdev->device_info;
	unsigned int off = index << info->gtt_entry_size_shift;

	vgt_native_gtt_write(off, &val, sizeof(val));
}

static inline void vgt_pci_bar_write_32(struct vgt_device *vgt, uint32_t bar_offset, uint32_t val)
{
	uint32_t* cfg_reg;

	/* BAR offset should be 32 bits algiend */
	cfg_reg = (uint32_t*)&vgt->state.cfg_space[bar_offset & ~3];

	/* only write the bits 31-4, leave the 3-0 bits unchanged, as they are read-only */
	*cfg_reg = (val & 0xFFFFFFF0) | (*cfg_reg & 0xF);
}

static inline int vgt_pci_mmio_is_enabled(struct vgt_device *vgt)
{
	return vgt->state.cfg_space[VGT_REG_CFG_COMMAND] &
		_REGBIT_CFG_COMMAND_MEMORY;
}

#define VGT_DPY_EMUL_PERIOD	16000000	// 16 ms for now

struct vgt_irq_host_state;
typedef void (*vgt_event_phys_handler_t)(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event);
typedef void (*vgt_event_virt_handler_t)(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event, struct vgt_device *vgt);

struct vgt_irq_ops {
	void (*init_irq) (struct vgt_irq_host_state *hstate);
	irqreturn_t (*irq_handler) (struct vgt_irq_host_state *hstate);
	void (*check_pending_irq) (struct vgt_device *vgt);
	void (*disable_irq) (struct vgt_irq_host_state *hstate);
	void (*enable_irq) (struct vgt_irq_host_state *hstate);
};

/* the list of physical interrupt control register groups */
enum vgt_irq_type {
	IRQ_INFO_GT,
	IRQ_INFO_DPY,
	IRQ_INFO_PCH,
	IRQ_INFO_PM,

	IRQ_INFO_MASTER,
	IRQ_INFO_GT0,
	IRQ_INFO_GT1,
	IRQ_INFO_GT2,
	IRQ_INFO_GT3,
	IRQ_INFO_DE_PIPE_A,
	IRQ_INFO_DE_PIPE_B,
	IRQ_INFO_DE_PIPE_C,
	IRQ_INFO_DE_PORT,
	IRQ_INFO_DE_MISC,
	IRQ_INFO_AUD,
	IRQ_INFO_PCU,

	IRQ_INFO_MAX,
};

#define VGT_IRQ_BITWIDTH	32
/* device specific interrupt bit definitions */
struct vgt_irq_info {
	char *name;
	int reg_base;
	enum vgt_event_type bit_to_event[VGT_IRQ_BITWIDTH];
	unsigned long warned;
	unsigned long default_enabled_events;
	int group;
	DECLARE_BITMAP(downstream_irq_bitmap, VGT_IRQ_BITWIDTH);
	bool has_upstream_irq;
};

#define	EVENT_FW_ALL 0	/* event forwarded to all instances */
#define	EVENT_FW_DOM0 1	/* event forwarded to dom0 only */
#define	EVENT_FW_NONE 2	/* no forward */

/* the handoff state from p-event to v-event */
union vgt_event_state {
	/* common state for bit based status */
	vgt_reg_t val;

	/* command stream error */
	struct {
		int eir_reg;
		vgt_reg_t eir_val;
	} cmd_err;
};

/* per-event information */
struct vgt_event_info {
	/* device specific info */
	int			bit;	/* map to register bit */
	union vgt_event_state	state;	/* handoff state*/
	struct vgt_irq_info	*info;	/* register info */

	/* device neutral info */
	int			policy;	/* forwarding policy */
	vgt_event_phys_handler_t	p_handler;	/* for p_event */
	vgt_event_virt_handler_t	v_handler;	/* for v_event */
};

struct vgt_emul_timer {
	struct hrtimer timer;
	u64 period;
};

#define REGBIT_INTERRUPT_PIPE_MASK    0x1f

struct vgt_irq_map {
	int up_irq_group;
	int up_irq_bit;
	int down_irq_group;
	u32 down_irq_bitmask;
};

/* structure containing device specific IRQ state */
struct vgt_irq_host_state {
	struct pgt_device *pdev;
	struct vgt_irq_ops *ops;
	int i915_irq;
	int pirq;
	struct vgt_irq_info	*info[IRQ_INFO_MAX];
	DECLARE_BITMAP(irq_info_bitmap, IRQ_INFO_MAX);
	struct vgt_event_info	events[EVENT_MAX];
	DECLARE_BITMAP(pending_events, EVENT_MAX);
	struct vgt_emul_timer dpy_timer;
	u32  pipe_mask;
	bool installed;
	struct vgt_irq_map *irq_map;
};

#define vgt_get_event_phys_handler(h, e)	(h->events[e].p_handler)
#define vgt_get_event_virt_handler(h, e)	(h->events[e].v_handler)
#define vgt_set_event_val(h, e, v)	(h->events[e].state.val = v)
#define vgt_get_event_val(h, e)		(h->events[e].state.val)
#define vgt_get_event_policy(h, e)	(h->events[e].policy)
#define vgt_get_irq_info(h, e)		(h->events[e].info)
#define vgt_get_irq_ops(p)		(p->irq_hstate->ops)

/* common offset among interrupt control registers */
#define regbase_to_isr(base)	(base)
#define regbase_to_imr(base)	(base + 0x4)
#define regbase_to_iir(base)	(base + 0x8)
#define regbase_to_ier(base)	(base + 0xC)

#define iir_to_regbase(iir)    (iir - 0x8)
#define ier_to_regbase(ier)    (ier - 0xC)

static inline void vgt_clear_all_vreg_bit(struct pgt_device *pdev, unsigned int value, unsigned int offset)
{
	struct vgt_device *vgt;
	vgt_reg_t vreg_data;
	unsigned int i;

	offset &= ~0x3;
	for (i = 0; i < VGT_MAX_VMS; i++) {
		vgt = pdev->device[i];
		if (vgt) {
			vreg_data = __vreg(vgt, offset) & (~value);
			__vreg(vgt, offset) = vreg_data;
		}
	}
}

static inline void vgt_set_all_vreg_bit(struct pgt_device *pdev, unsigned int value, unsigned int offset)
 {
	struct vgt_device *vgt;
	vgt_reg_t vreg_data;
	unsigned int i;

	offset &= ~0x3;
	for (i = 0; i < VGT_MAX_VMS; i++) {
		vgt = pdev->device[i];
		if (vgt) {
			vreg_data = __vreg(vgt, offset) | value;
			__vreg(vgt, offset) = vreg_data;
		}
	}
}

/* wrappers for criticl section in vgt */
#define vgt_lock_dev(pdev, cpu) {		\
	if (likely(vgt_track_nest))		\
		cpu = vgt_enter();		\
	else					\
		cpu = 0;			\
	if (vgt_lock_irq)			\
		spin_lock_irq(&pdev->lock);	\
	else					\
		spin_lock(&pdev->lock);		\
}

#define vgt_unlock_dev(pdev, cpu) {		\
	if (vgt_lock_irq)			\
		spin_unlock_irq(&pdev->lock);	\
	else					\
		spin_unlock(&pdev->lock);	\
	if (likely(vgt_track_nest))		\
		vgt_exit(cpu);			\
	else					\
		cpu = 0;			\
}

#define vgt_lock_dev_flags(pdev, cpu, flags) {	\
	flags = 0;				\
	if (likely(vgt_track_nest))		\
		cpu = vgt_enter();		\
	else					\
		cpu = 0;			\
	if (vgt_lock_irq)			\
		spin_lock_irqsave(&pdev->lock, flags);	\
	else					\
		spin_lock(&pdev->lock);		\
}

#define vgt_unlock_dev_flags(pdev, cpu, flags) {	\
	if (vgt_lock_irq)				\
		spin_unlock_irqrestore(&pdev->lock, flags); \
	else						\
		spin_unlock(&pdev->lock);		\
	if (likely(vgt_track_nest))			\
		vgt_exit(cpu);				\
	else						\
		cpu = 0;				\
}

#define vgt_get_irq_lock(pdev, flags) {		\
	spin_lock_irqsave(&pdev->irq_lock, flags);	\
}

#define vgt_put_irq_lock(pdev, flags) {		\
	spin_unlock_irqrestore(&pdev->irq_lock, flags);	\
}

void vgt_reset_virtual_states(struct vgt_device *vgt, unsigned long ring_bitmap);
void vgt_reset_ppgtt(struct vgt_device *vgt, unsigned long ring_bitmap);
void vgt_reset_execlist(struct vgt_device *vgt, unsigned long ring_bitmap);

enum vgt_pipe get_edp_input(uint32_t wr_data);
void vgt_forward_events(struct pgt_device *pdev);
void vgt_emulate_dpy_events(struct pgt_device *pdev);
bool vgt_manage_emul_dpy_events(struct pgt_device *pdev);
void vgt_update_frmcount(struct vgt_device *vgt, enum vgt_pipe pipe);
void vgt_calculate_frmcount_delta(struct vgt_device *vgt, enum vgt_pipe pipe);
void *vgt_install_irq(struct pci_dev *pdev, struct drm_device *dev);
int vgt_irq_init(struct pgt_device *pgt);
void vgt_irq_exit(struct pgt_device *pgt);

void vgt_inject_flip_done(struct vgt_device *vgt, enum vgt_pipe pipe);

bool vgt_rrmr_mmio_write(struct vgt_device *vgt, unsigned int offset,
        void *p_data, unsigned int bytes);

void vgt_trigger_virtual_event(struct vgt_device *vgt,
	enum vgt_event_type event);

void vgt_trigger_display_hot_plug(struct pgt_device *dev, vgt_hotplug_cmd_t hotplug_cmd);

void vgt_signal_uevent(struct pgt_device *dev);
void vgt_hotplug_udev_notify_func(struct work_struct *work);

u32 vgt_recalculate_ier(struct pgt_device *pdev, unsigned int reg);
u32 vgt_recalculate_mask_bits(struct pgt_device *pdev, unsigned int reg);

void recalculate_and_update_imr(struct pgt_device *pdev, vgt_reg_t reg);
void recalculate_and_update_ier(struct pgt_device *pdev, vgt_reg_t reg);

bool vgt_reg_master_irq_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes);
bool vgt_reg_imr_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes);
bool vgt_reg_ier_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes);
bool vgt_reg_iir_handler(struct vgt_device *vgt, unsigned int reg,
	void *p_data, unsigned int bytes);
bool vgt_reg_isr_write(struct vgt_device *vgt, unsigned int reg,
	void *p_data, unsigned int bytes);
bool vgt_reg_isr_read(struct vgt_device *vgt, unsigned int reg,
	void *p_data, unsigned int bytes);
void vgt_reg_watchdog_handler(struct vgt_device *state,
	uint32_t reg, uint32_t val, bool write, ...);
extern char *vgt_irq_name[EVENT_MAX];
ssize_t get_avl_vm_aperture_gm_and_fence(struct pgt_device *pdev, char *buf,
		ssize_t buf_sz);
vgt_reg_t mmio_g2h_gmadr(struct vgt_device *vgt, unsigned long reg, vgt_reg_t g_value);
vgt_reg_t mmio_h2g_gmadr(struct vgt_device *vgt, unsigned long reg, vgt_reg_t h_value);
unsigned long rsvd_aperture_alloc(struct pgt_device *pdev, unsigned long size);
void rsvd_aperture_free(struct pgt_device *pdev, unsigned long start, unsigned long size);
int allocate_vm_aperture_gm_and_fence(struct vgt_device *vgt, vgt_params_t vp);
void free_vm_aperture_gm_and_fence(struct vgt_device *vgt);
int alloc_vm_rsvd_aperture(struct vgt_device *vgt);
void free_vm_rsvd_aperture(struct vgt_device *vgt);
void initialize_gm_fence_allocation_bitmaps(struct pgt_device *pdev);
void vgt_init_reserved_aperture(struct pgt_device *pdev);
bool vgt_map_plane_reg(struct vgt_device *vgt, unsigned int reg, unsigned int *p_real_offset);

unsigned int vgt_pa_to_mmio_offset(struct vgt_device *vgt, uint64_t pa);

static inline void vgt_set_pipe_mapping(struct vgt_device *vgt,
	unsigned int v_pipe, unsigned int p_pipe)
{
	/* p_pipe == I915_MAX_PIPES means an invalid p_pipe */
	if (v_pipe < I915_MAX_PIPES && p_pipe <= I915_MAX_PIPES) {
		vgt->pipe_mapping[v_pipe] = p_pipe;
	}
	else {
		vgt_err("v_pipe=%d, p_pipe=%d!\n", v_pipe, p_pipe);
		WARN_ON(1);
	}
}

bool rebuild_pipe_mapping(struct vgt_device *vgt, unsigned int reg, uint32_t new_data, uint32_t old_data);
bool update_pipe_mapping(struct vgt_device *vgt, unsigned int physical_reg, uint32_t physical_wr_data);

#include <drm/drmP.h>

extern void *i915_drm_to_pgt(struct drm_device *dev);
extern void vgt_schedule_host_isr(struct drm_device *dev);

extern void i915_handle_error(struct drm_device *dev, bool wedged,
		       const char *fmt, ...);

extern int i915_wait_error_work_complete(struct drm_device *dev);

int vgt_reset_device(struct pgt_device *pgt);
int vgt_del_state_sysfs(vgt_params_t vp);
void reset_cached_interrupt_registers(struct pgt_device *pdev);

int create_vgt_instance(struct pgt_device *pdev, struct vgt_device **ptr_vgt, vgt_params_t vp);
void vgt_release_instance(struct vgt_device *vgt);
int vgt_init_sysfs(struct pgt_device *pdev);
void vgt_destroy_sysfs(void);
extern void vgt_clear_port(struct vgt_device *vgt, int index);
void vgt_update_monitor_status(struct vgt_device *vgt);
void vgt_detect_display(struct vgt_device *vgt, int index);
void vgt_dpy_init_modes(vgt_reg_t *mmio_array);

bool default_mmio_read(struct vgt_device *vgt, unsigned int offset,	void *p_data, unsigned int bytes);
bool default_mmio_write(struct vgt_device *vgt, unsigned int offset, void *p_data, unsigned int bytes);
bool default_passthrough_mmio_read(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes);

bool ring_mmio_write_in_rb_mode(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes);

bool ring_mmio_read_in_rb_mode(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes);

bool ring_uhptr_write_in_rb_mode(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes);

bool set_panel_fitting(struct vgt_device *vgt, enum vgt_pipe pipe);
void vgt_set_power_well(struct vgt_device *vgt, bool enable);
void vgt_flush_port_info(struct vgt_device *vgt, struct gt_port *port);

extern bool gtt_emulate_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes);

extern bool gtt_emulate_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes);

#define INVALID_ADDR (~0UL)

extern void* vgt_gma_to_va(struct vgt_mm *mm, unsigned long gma);


#define INVALID_MFN	(~0UL)

extern void vgt_ppgtt_switch(struct vgt_device *vgt);
extern int ring_ppgtt_mode(struct vgt_device *vgt, int ring_id, u32 off, u32 mode);

extern struct dentry *vgt_init_debugfs(struct pgt_device *pdev);
extern int vgt_create_debugfs(struct vgt_device *vgt);

/* command parser interface */
#define MAX_CMD_BUDGET  0x7fffffff
extern int vgt_cmd_parser_init(struct pgt_device *pdev);
extern void vgt_cmd_parser_exit(void);
extern int vgt_scan_vring(struct vgt_device *vgt, int ring_id);
extern void vgt_init_cmd_info(vgt_state_ring_t *rs);
extern void apply_tail_list(struct vgt_device *vgt, int ring_id,
	uint64_t submission_id);
extern int get_submission_id(vgt_state_ring_t *rs, int budget, uint64_t *submission_id);

extern void vgt_submit_commands(struct vgt_device *vgt, int ring_id);
extern void vgt_sched_update_prev(struct vgt_device *vgt, cycles_t time);
extern void vgt_sched_update_next(struct vgt_device *vgt);
extern void vgt_schedule(struct pgt_device *pdev);
extern void vgt_request_force_removal(struct vgt_device *vgt);

/* klog facility for buck printk */
extern int vgt_klog_init(void);
extern void vgt_klog_cleanup(void);
extern void klog_printk(const char *fmt, ...);

typedef struct {
	char *node_name;
	u64 *stat;
} debug_statistics_t;

extern u64 context_switch_cost;
extern u64 context_switch_num;
extern u64 ring_idle_wait;
extern u64 ring_0_idle;
extern u64 ring_0_busy;
extern u64 vm_pending_irq[VGT_MAX_VMS];

struct vgt_port_output_struct {
	unsigned int ctrl_reg;
	vgt_reg_t enable_bitmask;
	vgt_reg_t select_bitmask;
	enum vgt_output_type output_type;
};

struct vgt_mmio_dev {
	int devid_major;
	char *dev_name;
	struct class *class;
	struct cdev cdev;
	struct device *devnode[VGT_MAX_VMS];
};
#define VGT_MMIO_DEV_NAME "vgt_mmio"
int vgt_init_mmio_device(struct pgt_device *pdev);
void vgt_cleanup_mmio_dev(struct pgt_device *pdev);
int vgt_create_mmio_dev(struct vgt_device *vgt);
void vgt_destroy_mmio_dev(struct vgt_device *vgt);

/* invoked likely in irq disabled condition */
#define wait_for_atomic(COND, MS) ({					\
	unsigned long cnt = MS*100;					\
	int ret__ = 0;							\
	while (!(COND)) {						\
		if (!(--cnt)) {						\
			ret__ = -ETIMEDOUT;				\
			break;						\
		}							\
		udelay(10);						\
	}								\
	ret__;								\
})

extern reg_attr_t vgt_base_reg_info[];
extern reg_addr_sz_t vgt_reg_addr_sz[];
extern int vgt_get_base_reg_num(void);
extern int vgt_get_reg_addr_sz_num(void);
reg_list_t *vgt_get_sticky_regs(struct pgt_device *pdev);
extern int vgt_get_sticky_reg_num(struct pgt_device *pdev);

int vgt_hvm_opregion_map(struct vgt_device *vgt, int map);
int vgt_hvm_set_trap_area(struct vgt_device *vgt, int map);
int vgt_hvm_map_aperture (struct vgt_device *vgt, int map);
int setup_gtt(struct pgt_device *pdev);
void check_gtt(struct pgt_device *pdev);
void free_gtt(struct pgt_device *pdev);
void vgt_clear_gtt(struct vgt_device *vgt);
void vgt_save_gtt_and_fence(struct pgt_device *pdev);
void vgt_restore_gtt_and_fence(struct pgt_device *pdev);
uint64_t vgt_get_gtt_size(struct pgt_device *pdev);
uint64_t pci_bar_size(struct pgt_device *pdev, unsigned int bar_off);
struct vgt_device *vmid_2_vgt_device(int vmid);
extern void vgt_print_edid(struct vgt_edid_data_t *edid);
extern void vgt_print_dpcd(struct vgt_dpcd_data *dpcd);
int vgt_fb_notifier_call_chain(unsigned long val, void *data);
void vgt_init_fb_notify(void);
void vgt_dom0_ready(struct vgt_device *vgt);



struct dump_buffer {
	char *buffer;
	int buf_len;
	int buf_size;
};

int create_dump_buffer(struct dump_buffer *buf, int buf_size);
void destroy_dump_buffer(struct dump_buffer *buf);
void dump_string(struct dump_buffer *buf, const char *fmt, ...);

bool vgt_ppgtt_update_shadow_ppgtt_for_ctx(struct vgt_device *vgt, struct execlist_context *el_ctx);
bool vgt_handle_guest_write_rootp_in_context(struct execlist_context *el_ctx, int idx);
gtt_entry_t *vgt_get_entry(void *pt, gtt_entry_t *e, unsigned long index);
void execlist_ctx_table_destroy(struct vgt_device *vgt);

void dump_ctx_desc(struct vgt_device *vgt, struct ctx_desc_format *desc);
void dump_execlist_status(struct execlist_status_format *status, enum vgt_ring_id ring_id);
void dump_execlist_info(struct pgt_device *pdev, enum vgt_ring_id ring_id);
void dump_ctx_status_buf(struct vgt_device *vgt, enum vgt_ring_id ring_id, bool hw_status);
void dump_regstate_ctx_header (struct reg_state_ctx_header *regstate);
void dump_el_context_information(struct vgt_device *vgt, struct execlist_context *el_ctx);
void dump_all_el_contexts(struct pgt_device *pdev);
void dump_el_status(struct pgt_device *pdev);

void vgt_clear_submitted_el_record(struct pgt_device *pdev, enum vgt_ring_id ring_id);
void vgt_emulate_context_switch_event(struct pgt_device *pdev, enum vgt_ring_id ring_id);
void vgt_submit_execlist(struct vgt_device *vgt, enum vgt_ring_id ring_id);
void vgt_kick_off_execlists(struct vgt_device *vgt);
bool vgt_idle_execlist(struct pgt_device *pdev, enum vgt_ring_id ring_id);
struct execlist_context * execlist_context_find(struct vgt_device *vgt, uint32_t guest_lrca);

bool vgt_g2v_execlist_context_create(struct vgt_device *vgt);
bool vgt_g2v_execlist_context_destroy(struct vgt_device *vgt);

bool vgt_batch_ELSP_write(struct vgt_device *vgt, int ring_id);

static inline void reset_el_structure(struct pgt_device *pdev,
				enum vgt_ring_id ring_id)
{
	el_read_ptr(pdev, ring_id) = DEFAULT_INV_SR_PTR;
	vgt_clear_submitted_el_record(pdev, ring_id);
	/* reset read ptr in MMIO as well */
	VGT_MMIO_WRITE(pdev, el_ring_mmio(ring_id, _EL_OFFSET_STATUS_PTR),
			((_CTXBUF_READ_PTR_MASK << 16) |
			(DEFAULT_INV_SR_PTR << _CTXBUF_READ_PTR_SHIFT)));

}

extern struct kernel_dm *vgt_pkdm;

static inline unsigned long hypervisor_g2m_pfn(struct vgt_device *vgt,
	unsigned long g_pfn)
{
	return vgt_pkdm->g2m_pfn(vgt->vm_id, g_pfn);
}

static inline int hypervisor_pause_domain(struct vgt_device *vgt)
{
	return vgt_pkdm->pause_domain(vgt->vm_id);
}

static inline int hypervisor_shutdown_domain(struct vgt_device *vgt)
{
	return vgt_pkdm->shutdown_domain(vgt->vm_id);
}

static inline int hypervisor_map_mfn_to_gpfn(struct vgt_device *vgt,
	unsigned long gpfn, unsigned long mfn, int nr, int map, enum map_type type)
{
	if (vgt_pkdm && vgt_pkdm->map_mfn_to_gpfn)
		return vgt_pkdm->map_mfn_to_gpfn(vgt->vm_id, gpfn, mfn, nr, map, type);

	return 0;
}

static inline int hypervisor_set_trap_area(struct vgt_device *vgt,
	uint64_t start, uint64_t end, bool map)
{
	return vgt_pkdm->set_trap_area(vgt, start, end, map);
}

static inline int hypervisor_set_wp_pages(struct vgt_device *vgt, guest_page_t *p)
{
	return vgt_pkdm->set_wp_pages(vgt, p);
}

static inline int hypervisor_unset_wp_pages(struct vgt_device *vgt, guest_page_t *p)
{
	return vgt_pkdm->unset_wp_pages(vgt, p);
}

static inline int hypervisor_check_host(void)
{
	return vgt_pkdm->check_host();
}

static inline int hypervisor_virt_to_mfn(void *addr)
{
	return vgt_pkdm->from_virt_to_mfn(addr);
}

static inline void *hypervisor_mfn_to_virt(int mfn)
{
	return vgt_pkdm->from_mfn_to_virt(mfn);
}

static inline void hypervisor_inject_msi(struct vgt_device *vgt)
{
#define MSI_CAP_OFFSET 0x90	/* FIXME. need to get from cfg emulation */
#define MSI_CAP_CONTROL (MSI_CAP_OFFSET + 2)
#define MSI_CAP_ADDRESS (MSI_CAP_OFFSET + 4)
#define MSI_CAP_DATA	(MSI_CAP_OFFSET + 8)
#define MSI_CAP_EN 0x1

	char *cfg_space = &vgt->state.cfg_space[0];
	u16 control = *(u16 *)(cfg_space + MSI_CAP_CONTROL);
	u32 addr = *(u32 *)(cfg_space + MSI_CAP_ADDRESS);
	u16 data = *(u16 *)(cfg_space + MSI_CAP_DATA);
	int r;

	/* Do not generate MSI if MSIEN is disable */
	if (!(control & MSI_CAP_EN))
		return;

	/* FIXME: currently only handle one MSI format */
	ASSERT_NUM(!(control & 0xfffe), control);

	vgt_dbg(VGT_DBG_IRQ, "vGT: VM(%d): hvm injections. address (%x) data(%x)!\n",
			vgt->vm_id, addr, data);
	r = vgt_pkdm->inject_msi(vgt->vm_id, addr, data);
	if (r < 0)
		vgt_err("vGT(%d): failed to inject vmsi\n", vgt->vgt_id);
}

static inline int hypervisor_hvm_init(struct vgt_device *vgt)
{
	if (vgt_pkdm && vgt_pkdm->hvm_init)
		return vgt_pkdm->hvm_init(vgt);

	return 0;
}

static inline void hypervisor_hvm_exit(struct vgt_device *vgt)
{
	if (vgt_pkdm && vgt_pkdm->hvm_exit)
		vgt_pkdm->hvm_exit(vgt);
}

static inline void *hypervisor_gpa_to_va(struct vgt_device *vgt, unsigned long gpa)
{
	if (!vgt->vm_id)
		return (char *)hypervisor_mfn_to_virt(gpa >> PAGE_SHIFT) + offset_in_page(gpa);

	return vgt_pkdm->gpa_to_va(vgt, gpa);
}

static inline bool hypervisor_read_va(struct vgt_device *vgt, void *va,
		void *val, int len, int atomic)
{
	bool ret;

	if (!vgt->vm_id) {
		memcpy(val, va, len);
		return true;
	}

	ret = vgt_pkdm->read_va(vgt, va, val, len, atomic);
	if (unlikely(!ret))
		vgt_err("VM(%d): read va failed, va: 0x%p, atomic : %s\n", vgt->vm_id,
				va, atomic ? "yes" : "no");

	return ret;
}

static inline bool hypervisor_write_va(struct vgt_device *vgt, void *va,
		void *val, int len, int atomic)
{
	bool ret;

	if (!vgt->vm_id) {
		memcpy(va, val, len);
		return true;
	}

	ret = vgt_pkdm->write_va(vgt, va, val, len, atomic);
	if (unlikely(!ret))
		vgt_err("VM(%d): write va failed, va: 0x%p, atomic : %s\n", vgt->vm_id,
				va, atomic ? "yes" : "no");

	return ret;
}


#define ASSERT_VM(x, vgt)						\
	do {								\
		if (!(x)) {						\
			printk("Assert at %s line %d\n",		\
				__FILE__, __LINE__);			\
			if (atomic_cmpxchg(&(vgt)->crashing, 0, 1))	\
				break;					\
			vgt_warn("Killing VM%d\n", (vgt)->vm_id);	\
			if (!hypervisor_pause_domain((vgt)))		\
				hypervisor_shutdown_domain((vgt));	\
		}							\
	} while (0)


#endif	/* _VGT_DRV_H_ */
