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

typedef uint32_t vgt_reg_t;
struct pgt_device;
struct vgt_device;

#include "../i915_vgpu.h"
#include "../i915_drv.h"
#include "host.h"
#include "reg.h"
#include "devtable.h"
#include "display.h"
#include "edid.h"
#include "cmd_parser.h"
#include "hypercall.h"
#include "gtt.h"
#include "interrupt.h"
#include "mmio.h"
#include "perf.h"
#include "render.h"
#include "sched.h"
#include "execlists.h"
#include "fb_decoder.h"

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
extern int reset_count_threshold;
extern int reset_dur_threshold;
extern int reset_max_threshold;
extern bool vgt_lock_irq;
extern int shadow_execlist_context;
extern int shadow_cmd_buffer;
extern int shadow_ctx_check;
extern int shadow_indirect_ctx_bb;
extern int vgt_cmd_audit;
extern bool propagate_monitor_to_guest;
extern bool irq_based_ctx_switch;
extern int preallocated_shadow_pages;
extern int preallocated_oos_pages;
extern bool spt_out_of_sync;
extern bool timer_based_qos;
extern int tbs_period_ms;
extern bool opregion_present;
extern int preemption_policy;

#define VGT_PREEMPTION_DISABLED   (1<<0)
#define VGT_LITERESTORE_DISABLED  (1<<1)

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
#define VGT_DBG_RESET		(1<<8)
#define VGT_DBG_ALL		(0xffff)

#define SIZE_1KB		(1024UL)
#define SIZE_1MB		(1024UL*1024UL)
#define SIZE_PAGE		(4 * SIZE_1KB)

#define VGT_RSVD_RING_SIZE	(16 * SIZE_1KB)

#define _vgt_mmio_va(pdev, x)		((uint64_t)((char*)pdev->gttmmio_base_va+x))	/* PA to VA */
#define _vgt_mmio_pa(pdev, x)		(pdev->gttmmio_base+x)			/* PA to VA */

#define VGT_RING_TIMEOUT	500	/* in ms */

/* Maximum VMs supported by vGT. Actual number is device specific */
#define VGT_MAX_VMS			8
#define VGT_RSVD_APERTURE_SZ		(32*SIZE_1MB)	/* reserve 8MB for vGT itself */

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

#define __vreg(vgt, off) (*(vgt_reg_t *)((char *)vgt->state.vReg + off))
#define __vreg8(vgt, off) (*(char *)((char *)vgt->state.vReg + off))
#define __vreg16(vgt, off) (*(uint16_t *)((char *)vgt->state.vReg + off))
#define __sreg(vgt, off) (*(vgt_reg_t *)((char *)vgt->state.sReg + off))
#define __sreg8(vgt, off) (*(char *)((char *)vgt->state.sReg + off))
#define __vreg64(vgt, off) (*(unsigned long *)((char *)vgt->state.vReg + off))
#define __sreg64(vgt, off) (*(unsigned long *)((char *)vgt->state.sReg + off))
#define vgt_vreg(vgt, off)	((vgt_reg_t *)((char *)vgt->state.vReg + off))
#define vgt_sreg(vgt, off)	((vgt_reg_t *)((char *)vgt->state.sReg + off))

struct vgt_mm;
struct execlist_context;

#define vgt_el_queue_head(vgt, ring_id) \
	((vgt)->rb[ring_id].el_slots_head)
#define vgt_el_queue_tail(vgt, ring_id) \
	((vgt)->rb[ring_id].el_slots_tail)
#define vgt_el_queue_slot(vgt, ring_id, slot_idx) \
	((vgt)->rb[ring_id].execlist_slots[slot_idx])
#define vgt_el_queue_ctx(vgt, ring_id, slot_idx, ctx_idx) \
	((vgt)->rb[ring_id].execlist_slots[slot_idx].el_ctxs[ctx_idx])

struct vgt_device;

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

/* per-VM structure */

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

#define VGT_MAX_PPAT_TABLE_SIZE 8

struct vgt_ppat_table {
	u8 mapping_table[VGT_MAX_PPAT_TABLE_SIZE];
	bool is_vaild; /* indicate if ppat exist, not translate if zero. */
};

bool gen8_ppat_update_mapping_table(struct vgt_device *vgt);
void gen8_dump_ppat_registers(struct vgt_device *vgt);

struct vgt_device {
	enum pipe pipe_mapping[I915_MAX_PIPES];
	int vgt_id;		/* 0 is always for dom0 */
	int vm_id;		/* domain ID per hypervisor */
	struct pgt_device *pdev;	/* the pgt device where the GT device registered. */
	struct list_head	list;	/* FIXME: used for context switch ?? */
	vgt_state_t	state;		/* MMIO state except ring buffers */
	vgt_state_ring_t	rb[MAX_ENGINES];	/* ring buffer state */

	struct gt_port		ports[I915_MAX_PORTS]; /* one port per PIPE */
	struct vgt_i2c_edid_t	vgt_i2c_edid;	/* i2c bus state emulaton for reading EDID */

	struct vgt_ppat_table	ppat_table;

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
	DECLARE_BITMAP(tlb_handle_pending, MAX_ENGINES);
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

	struct sbi_registers sbi_regs;

	unsigned long reset_flags;
	unsigned long enabled_rings_before_reset;
	unsigned long last_reset_time;

	unsigned long *reset_count_start_time; /*record each reset time*/
	int reset_count;
	int reset_count_head; /*sliding window methods*/
};

typedef u32 reg_info_t;

struct vgt_irq_host_state;

enum vgt_trace_type {
	VGT_TRACE_READ,
	VGT_TRACE_WRITE
};

typedef union {
	uint32_t dw;
	struct {
		uint32_t virtual_event: 16;
		uint32_t vmid : 8;
		uint32_t rsvd_24_31 : 8;
	};
} vgt_virtual_event_t;

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
	u32 max_surface_size;
	u32 max_support_vms;
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
	struct vgt_device *cur_reset_vm;	/* the VM who trigger reset */
	struct list_head rendering_runq_head; /* reuse this for context scheduler */
	struct list_head rendering_idleq_head; /* reuse this for context scheduler */
	spinlock_t lock;
	spinlock_t irq_lock;

	reg_info_t *reg_info;	/* virtualization policy for a given reg */
	struct vgt_irq_host_state *irq_hstate;

	DECLARE_BITMAP(dpy_emul_request, VGT_MAX_VMS);

	u8 gen_dev_type;

#define GEN_CACHE_UC 0U
#define GEN_CACHE_WC 1U
	u8 gen_cache_type;

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

	u32 memory_latency[2];

	int (*vgt_get_pixel_format)(u32 plane_ctl,
		struct vgt_common_plane_format *common_plane, enum vgt_plane_type plane);

	bool dummy_vm_switch;
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

#define reg_addr_fix(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_ADDR_FIX)
#define reg_hw_status(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_HW_STATUS)
#define reg_virt(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_VIRT)
#define reg_mode_ctl(pdev, reg)		(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_MODE_CTL)
#define reg_passthrough(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_PASSTHROUGH)
#define reg_pt_readonly(pdev, reg)	(pdev->reg_info[REG_INDEX(reg)] & VGT_REG_PT_READONLY)
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

static inline void reg_set_pt_readonly(struct pgt_device *pdev,
	vgt_reg_t reg)
{
	ASSERT_NUM(!reg_is_tracked(pdev, reg), reg);
	pdev->reg_info[REG_INDEX(reg)] |= VGT_REG_PT_READONLY;
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
#define IGD_SKL		5
#define IGD_MAX		IGD_SKL

#define IS_SNB(pdev)	((pdev)->gen_dev_type == IGD_SNB)
#define IS_IVB(pdev)	((pdev)->gen_dev_type == IGD_IVB)
#define IS_HSW(pdev)	((pdev)->gen_dev_type == IGD_HSW)
#define IS_BDW(pdev)	((pdev)->gen_dev_type == IGD_BDW)
#define IS_SKL(pdev)	((pdev)->gen_dev_type == IGD_SKL)

#define IS_PREBDW(pdev) (IS_SNB(pdev) || IS_IVB(pdev) || IS_HSW(pdev))
#define IS_BDWPLUS(pdev) (IS_BDW(pdev) || IS_SKL(pdev))
#define IS_PRESKL(pdev) (IS_BDW(pdev) || IS_HSW(pdev) || IS_IVB(pdev) || IS_SNB(pdev))
#define IS_SKLPLUS(pdev) (IS_SKL(pdev))
#define IS_BDWGT3(pdev) (IS_BDW(pdev) && (GEN_REV(pdev->device_info.gen) == 3))
#define IS_SKLGT3(pdev) (IS_SKL(pdev) && (GEN_REV(pdev->device_info.gen) == 3))
#define IS_SKLGT4(pdev) (IS_SKL(pdev) && (GEN_REV(pdev->device_info.gen) == 4))

#define D_SNB	(1 << 0)
#define D_IVB	(1 << 1)
#define D_HSW	(1 << 2)
#define D_BDW	(1 << 3)
#define D_SKL	(1 << 4)

#define D_GEN9PLUS	(D_SKL)
#define D_GEN8PLUS	(D_BDW | D_SKL)
#define D_GEN75PLUS	(D_HSW | D_BDW | D_SKL)
#define D_GEN7PLUS	(D_IVB | D_HSW | D_BDW | D_SKL)

#define D_SKL_PLUS	(D_SKL)
#define D_BDW_PLUS	(D_BDW | D_SKL)
#define D_HSW_PLUS	(D_HSW | D_BDW | D_SKL)
#define D_IVB_PLUS	(D_IVB | D_HSW | D_BDW | D_SKL)

#define D_PRE_BDW	(D_SNB | D_IVB | D_HSW)
#define D_PRE_SKL	(D_SNB | D_IVB | D_HSW | D_BDW)

#define D_ALL		(D_SNB | D_IVB | D_HSW | D_BDW | D_SKL)

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
	if (IS_SKL(pdev))
		return D_SKL;
	WARN_ONCE(1, KERN_ERR "vGT: unknown GEN type!\n");
	return 0;
}

static inline bool vgt_match_device_attr(struct pgt_device *pdev, reg_attr_t *attr)
{
	return attr->device & vgt_gen_dev_type(pdev);
}

static inline enum port vgt_get_port(struct vgt_device *vgt, struct gt_port *port_ptr)
{
	enum port port_type;

	if (!vgt || !port_ptr)
		return I915_MAX_PORTS;

	for (port_type = PORT_A; port_type < I915_MAX_PORTS; ++ port_type)
		if (port_ptr == &vgt->ports[port_type])
			break;

	return port_type;
}

static inline enum pipe vgt_get_pipe_from_port(struct vgt_device *vgt,
						enum port port)
{
	enum pipe pipe;

	if (port == I915_MAX_PORTS)
		return I915_MAX_PIPES;

	ASSERT (port != PORT_A);

	for (pipe = PIPE_A; pipe < I915_MAX_PIPES; ++ pipe) {
		vgt_reg_t ddi_func_ctl;
		vgt_reg_t ddi_port_info;

		ddi_func_ctl  = __vreg(vgt, _VGT_TRANS_DDI_FUNC_CTL(pipe));

		if (!(ddi_func_ctl & TRANS_DDI_FUNC_ENABLE))
			continue;

		ddi_port_info = (ddi_func_ctl & TRANS_DDI_PORT_MASK) >>
					TRANS_DDI_PORT_SHIFT;
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

#define aperture_page_idx(pdev, gma)	(((gma) - aperture_2_gm(pdev, pdev->rsvd_aperture_base)) >> GTT_PAGE_SHIFT)
#define aperture_page(pdev, idx)	 ((*pdev->rsvd_aperture_pages)[idx])


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
extern void rsvd_aperture_runout_handler(struct pgt_device *pdev);
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
	rdtsc_ordered();			\
	__ret = get_cycles();		\
	rdtsc_ordered();			\
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

static inline bool is_ring_enabled (struct pgt_device *pdev, int ring_id)
{
	return (VGT_MMIO_READ(pdev, RB_CTL(pdev, ring_id)) & 1);	/* bit 0: enable/disable RB */
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

static inline bool ring_is_empty(struct pgt_device *pdev,
	int id)
{
	if ( is_ring_enabled(pdev, id) && !is_ring_empty(pdev, id) )
		return false;

	return true;
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

enum pipe get_edp_input(uint32_t wr_data);
void vgt_forward_events(struct pgt_device *pdev);
void vgt_emulate_dpy_events(struct pgt_device *pdev);
void *vgt_install_irq(struct pci_dev *pdev, struct drm_device *dev);
int vgt_irq_init(struct pgt_device *pgt);
void vgt_irq_exit(struct pgt_device *pgt);
void vgt_reset_virtual_interrupt_registers(struct vgt_device *vgt);
void vgt_inject_flip_done(struct vgt_device *vgt, enum pipe pipe);

bool vgt_rrmr_mmio_write(struct vgt_device *vgt, unsigned int offset,
        void *p_data, unsigned int bytes);
bool mul_force_wake_write(struct vgt_device *vgt, unsigned int offset,
		    void *p_data, unsigned int bytes);

void vgt_trigger_virtual_event(struct vgt_device *vgt,
	enum vgt_event_type event);

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
int mmio_g2h_gmadr(struct vgt_device *vgt, unsigned long reg,
		vgt_reg_t g_value, vgt_reg_t *h_gmadr);
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


#include <drm/drmP.h>

extern void *i915_drm_to_pgt(struct drm_device *dev);
extern void vgt_schedule_host_isr(struct drm_device *dev);

extern void i915_handle_error(struct drm_device *dev, bool wedged,
		       const char *fmt, ...);

extern int i915_wait_error_work_complete(struct drm_device *dev);

int vgt_reset_device(struct pgt_device *pgt);
bool vgt_reset_stat(struct vgt_device *vgt);
int vgt_del_state_sysfs(vgt_params_t vp);
void reset_cached_interrupt_registers(struct pgt_device *pdev);

int create_vgt_instance(struct pgt_device *pdev, struct vgt_device **ptr_vgt, vgt_params_t vp);
void vgt_release_instance(struct vgt_device *vgt);
int vgt_init_sysfs(struct pgt_device *pdev);
void vgt_destroy_sysfs(void);
extern void vgt_clear_port(struct vgt_device *vgt, int index);

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
extern int vgt_el_slots_number(vgt_state_ring_t *ring_state);

/* klog facility for buck printk */
extern int vgt_klog_init(void);
extern void vgt_klog_cleanup(void);
extern void klog_printk(const char *fmt, ...);

extern u64 context_switch_cost;
extern u64 context_switch_num;
extern u64 ring_idle_wait;
extern u64 ring_0_idle;
extern u64 ring_0_busy;
extern u64 vm_pending_irq[VGT_MAX_VMS];

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

extern reg_attr_t vgt_reg_info_general[];
extern reg_attr_t vgt_reg_info_hsw[];
extern reg_attr_t vgt_reg_info_bdw[];
extern reg_attr_t vgt_reg_info_skl[];
extern reg_addr_sz_t vgt_reg_addr_sz[];
extern int vgt_get_reg_num(int type);
extern int vgt_get_hsw_reg_num(void);
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
int  vgt_submit_execlist(struct vgt_device *vgt, enum vgt_ring_id ring_id);
void vgt_kick_off_execlists(struct vgt_device *vgt);
bool vgt_idle_execlist(struct pgt_device *pdev, enum vgt_ring_id ring_id);
struct execlist_context *execlist_context_find(struct vgt_device *vgt, uint32_t guest_lrca);
struct execlist_context *execlist_shadow_context_find(struct vgt_device *vgt,
				uint32_t guest_lrca);

bool vgt_g2v_execlist_context_create(struct vgt_device *vgt);
bool vgt_g2v_execlist_context_destroy(struct vgt_device *vgt);

bool vgt_batch_ELSP_write(struct vgt_device *vgt, int ring_id);
bool ppgtt_update_shadow_ppgtt_for_ctx(struct vgt_device *vgt,struct execlist_context *el_ctx);
int vgt_el_create_shadow_ppgtt(struct vgt_device *vgt,
				enum vgt_ring_id ring_id,
				struct execlist_context *el_ctx);
static inline void reset_phys_el_structure(struct pgt_device *pdev,
				enum vgt_ring_id ring_id)
{
	el_read_ptr(pdev, ring_id) = DEFAULT_INV_SR_PTR;
	/* reset read ptr in MMIO as well */
	VGT_MMIO_WRITE(pdev, el_ring_mmio(ring_id, _EL_OFFSET_STATUS_PTR),
			((_CTXBUF_READ_PTR_MASK << 16) |
			(DEFAULT_INV_SR_PTR << _CTXBUF_READ_PTR_SHIFT)));

}

static inline void reset_el_structure(struct pgt_device *pdev,
				enum vgt_ring_id ring_id)
{
	vgt_clear_submitted_el_record(pdev, ring_id);
	reset_phys_el_structure(pdev, ring_id);
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

#include "mpt.h"

static inline bool vgt_kill_vm(struct vgt_device *vgt)
{
	if (vgt->vm_id == 0) {
		vgt_err("Try to kill the Dom0!\n");
		return false;
	}

	if (atomic_cmpxchg(&vgt->crashing, 0, 1)) {
		vgt_err("VM-%d is under crashing!\n", vgt->vm_id);
		return false;
	}

	vgt_warn("Killing VM-%d\n", vgt->vm_id);
	if (!hypervisor_pause_domain(vgt) &&
			!hypervisor_shutdown_domain(vgt)) {
		return true;
	} else {
		vgt_err("Failed to kill VM-%d!\n", vgt->vm_id);
		return false;
	}
}

#endif	/* _VGT_DRV_H_ */
