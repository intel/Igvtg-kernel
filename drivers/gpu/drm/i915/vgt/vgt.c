/*
 * vGT module interface
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

#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#include "vgt.h"


MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Virtual GPU device model for Intel Processor Graphics");
MODULE_LICENSE("GPL and additional rights");
MODULE_VERSION("0.1");

extern struct kernel_dm xengt_kdm;
extern struct kernel_dm kvmgt_kdm;
struct kernel_dm *vgt_pkdm = NULL;

/*
 * opregion pages could be logically present for a VM like KVMGT guest.
 * In this case, don't allocate pages from host kernel in vgt.
 */
bool opregion_present __read_mostly = false;

bool hvm_render_owner = false;
module_param_named(hvm_render_owner, hvm_render_owner, bool, 0600);
MODULE_PARM_DESC(hvm_render_owner, "Make HVM to be render owner after create (default: false)");

bool hvm_dpy_owner = false;
module_param_named(hvm_dpy_owner, hvm_dpy_owner, bool, 0600);
MODULE_PARM_DESC(hvm_dpy_owner, "Deprecated option! Please use hvm_boot_foreground or hvm_display_owner!");

bool hvm_display_owner = false;
module_param_named(hvm_display_owner, hvm_display_owner, bool, 0600);
MODULE_PARM_DESC(hvm_display_owner, "Make HVM to be display owner after create (default: false)");

bool hvm_super_owner = false;
module_param_named(hvm_super_owner, hvm_super_owner, bool, 0600);
MODULE_PARM_DESC(hvm_super_owner, "Make HVM to be GPU owner after create (default: false)");

bool hvm_boot_foreground = false;
module_param_named(hvm_boot_foreground, hvm_boot_foreground, bool, 0600);
MODULE_PARM_DESC(hvm_boot_foreground, "Make HVM to be foreground after create and visible on screen from booting (default: false)");

bool vgt_primary = false;
module_param_named(vgt_primary, vgt_primary, bool, 0600);

bool vgt_track_nest = true;
module_param_named(track_nest, vgt_track_nest, bool, 0600);

bool vgt_delay_nest = true;
module_param_named(delay_nest, vgt_delay_nest, bool, 0600);

int vgt_debug = 0;
module_param_named(debug, vgt_debug, int, 0600);

bool vgt_enabled = true;
module_param_named(vgt, vgt_enabled, bool, 0400);

bool fastpath_dpy_switch = true;
module_param_named(fastpath_dpy_switch, fastpath_dpy_switch, bool, 0600);

bool event_based_qos = false;
module_param_named(event_based_qos, event_based_qos, bool, 0600);
MODULE_PARM_DESC(event_based_qos, "Use event based QoS scheduler (default: false)");

int tbs_period_ms = -1;
module_param_named(tbs_period_ms, tbs_period_ms, int, 0600);
MODULE_PARM_DESC(event_based_qos, "Set the time based QoS scheduler timer in unit of ms (default: BDW 1ms, HSW 15ms)");

bool shadow_tail_based_qos = false;
module_param_named(shadow_tail_based_qos, shadow_tail_based_qos, bool, 0600);
MODULE_PARM_DESC(shadow_tail_based_qos, "Use Shadow tail based QoS scheduler (default: false)");

bool render_engine_reset = true;
module_param_named(render_engine_reset, render_engine_reset, bool, 0600);
MODULE_PARM_DESC(render_engine_reset, "Reset rendering engines before loading another VM's context");

bool propagate_monitor_to_guest = true;
module_param_named(propagate_monitor_to_guest, propagate_monitor_to_guest, bool, 0600);
MODULE_PARM_DESC(propagate_monitor_to_guest, "Propagate monitor information to guest by XenGT, other than dom0 services to do so");

bool irq_based_ctx_switch = true;
module_param_named(irq_based_ctx_switch, irq_based_ctx_switch, bool, 0600);
MODULE_PARM_DESC(irq_based_ctx_switch, "Use user interrupt based context switch (default: true)");

int preallocated_shadow_pages = -1;
module_param_named(preallocated_shadow_pages, preallocated_shadow_pages, int, 0600);
MODULE_PARM_DESC(preallocated_shadow_pages, "Amount of pre-allocated shadow pages");

int preallocated_oos_pages = -1;
module_param_named(preallocated_oos_pages, preallocated_oos_pages, int, 0600);
MODULE_PARM_DESC(preallocated_oos_pages, "Amount of pre-allocated oos pages");

bool spt_out_of_sync = true;
module_param_named(spt_out_of_sync, spt_out_of_sync, bool, 0600);
MODULE_PARM_DESC(spt_out_of_sync, "Enable SPT out of sync");

/*
 * FIXME: now video ring switch has weird issue. The cmd
 * parser may enter endless loop even when head/tail is
 * zero. earlier posting read doesn't solve the issue.
 * so disable it for now.
 *
 * Dexuan: let's enable VCS switch, because on HSW, win7 gfx drver's PAVP
 * initialization uses VCS. Without enabling this option, win7 guest's gfx
 * driver's initializtion will hang when we create the guest for the 2nd
 * time(VCS.TAIL is 0x70, but VCS.HEAD is always 0x30).
 */
int enable_video_switch = 1;
module_param_named(enable_video_switch, enable_video_switch, int, 0600);

/*
 * On HSW, the max low/high gm sizes are 512MB/1536MB.
 * If each VM takes 512MB GM, we can support 4VMs.
 * By default Dom0 has 512MB GM, including 120MB low gm used by i915 and
 * 8MB low gm used by vGT driver itself(see VGT_RSVD_APERTURE_SZ), and
 * (512-120-8)MB high GM space used by i915.
 * We can reduce the GM space used by Dom0 i915, but remember: Dom0
 * render/display may not work properly without enough GM space.
 */
int dom0_low_gm_sz = 96;	//in MB.
module_param_named(dom0_low_gm_sz, dom0_low_gm_sz, int, 0600);

int dom0_high_gm_sz = 384;	//in MB.
module_param_named(dom0_high_gm_sz, dom0_high_gm_sz, int, 0600);

int dom0_fence_sz = 4;
module_param_named(dom0_fence_sz, dom0_fence_sz, int, 0600);

int bypass_scan_mask = 0;
module_param_named(bypass_scan, bypass_scan_mask, int, 0600);

bool bypass_dom0_addr_check = false;
module_param_named(bypass_dom0_addr_check, bypass_dom0_addr_check, bool, 0600);

bool cmd_parser_ip_buf = true;
module_param_named(cmd_parser_ip_buf, cmd_parser_ip_buf, bool, 0600);

bool enable_panel_fitting = true;
module_param_named(enable_panel_fitting, enable_panel_fitting, bool, 0600);

bool enable_reset = true;
module_param_named(enable_reset, enable_reset, bool, 0600);

bool vgt_lock_irq = false;
module_param_named(vgt_lock_irq, vgt_lock_irq, bool, 0400);

bool vgt_preliminary_hw_support = true;
module_param_named(vgt_preliminary_hw_support, vgt_preliminary_hw_support, bool, 0400);

int shadow_execlist_context = 0;
module_param_named(shadow_execlist_context, shadow_execlist_context, int, 0400);

/* Very frequent set/clear write protection can see wrong write trap even if
+ * write protection has been cleared. Below option is to disable the context
+ * protection between ctx submission and ctx completion. Normal context shadow
+ * will not be impacted by this option, which will have ctx write protection
+ * between ctx creation and ctx destroy.
+ */
bool wp_submitted_ctx = false;
module_param_named(wp_submitted_ctx, wp_submitted_ctx, bool, 0400);

static struct vgt_ops __vgt_ops = {
	.emulate_read = vgt_emulate_read,
	.emulate_write = vgt_emulate_write,
	.emulate_cfg_read = vgt_emulate_cfg_read,
	.emulate_cfg_write = vgt_emulate_cfg_write,
	.panic = vgt_panic,
	.pa_to_mmio_offset = vgt_pa_to_mmio_offset,
	.expand_shadow_page_mempool = vgt_expand_shadow_page_mempool,
	.del_state_sysfs = vgt_del_state_sysfs,
};

LIST_HEAD(pgt_devices);
struct pgt_device default_device = {
	.bus = 0,
	.devfn = 0x10,		/* BDF: 0:2:0 */
};

struct vgt_device *vgt_dom0;
struct pgt_device *pdev_default = &default_device;
DEFINE_PER_CPU(u8, in_vgt);

uint64_t vgt_gttmmio_va(struct pgt_device *pdev, off_t reg)
{
	return (uint64_t)((char *)pdev->gttmmio_base_va + reg);
}

uint64_t vgt_gttmmio_pa(struct pgt_device *pdev, off_t reg)
{
	return (uint64_t)(pdev->gttmmio_base + reg);
}

struct pci_dev *pgt_to_pci(struct pgt_device *pdev)
{
	return pdev->pdev;
}

/*
 * The thread to perform the VGT ownership switch.
 *
 * We need to handle race conditions from different paths around
 * vreg/sreg/hwreg. So far there're 4 paths at least:
 *   a) the vgt thread to conduct context switch
 *   b) the GP handler to emulate MMIO for dom0
 *   c) the event handler to emulate MMIO for other VMs
 *   d) the interrupt handler to do interrupt virtualization
 *   e) /sysfs interaction from userland program
 *
 * Now d) is removed from the race path, because we adopt a delayed
 * injection mechanism. Physical interrupt handler only saves pending
 * IIR bits, and then wake up the vgt thread. Later the vgt thread
 * checks the pending bits to do the actual virq injection. This approach
 * allows vgt thread to handle ownership switch cleanly.
 *
 * So it's possible for other 3 paths to touch vreg/sreg/hwreg:
 *   a) the vgt thread may need to update HW updated regs into
 *	  vreg/sreg of the prev owner
 *   b) the GP handler and event handler always updates vreg/sreg,
 *	  and may touch hwreg if vgt is the current owner
 *	  and then update vreg for interrupt virtualization
 *
 * To simplify the lock design, we make below assumptions:
 *   a) the vgt thread doesn't trigger GP fault itself, i.e. always
 *	  issues hypercall to do hwreg access
 *   b) the event handler simply notifies another kernel thread, leaving
 *	  to that thread for actual MMIO emulation
 *
 * Given above assumption, no nest would happen among 4 paths, and a
 * simple global spinlock now should be enough to protect the whole
 * vreg/sreg/ hwreg. In the future we can futher tune this part on
 * a necessary base.
 */
static void vgt_processe_lo_priority_request(struct pgt_device *pdev)
{
	int cpu;

	/* Send uevent to userspace */
	if (test_and_clear_bit(VGT_REQUEST_UEVENT,
				(void *)&pdev->request)) {
		vgt_signal_uevent(pdev);
	}

	if (test_and_clear_bit(VGT_REQUEST_DPY_SWITCH,
				(void *)&pdev->request)) {
		vgt_lock_dev(pdev, cpu);
		if (prepare_for_display_switch(pdev) == 0)
			do_vgt_fast_display_switch(pdev);
		vgt_unlock_dev(pdev, cpu);
	}

	/* Handle render engine scheduling */
	if (vgt_ctx_switch &&
	    test_and_clear_bit(VGT_REQUEST_SCHED,
			(void *)&pdev->request)) {
		if (!vgt_do_render_sched(pdev)) {
			if (enable_reset) {
				vgt_err("Hang in render sched, try to reset device.\n");

				vgt_reset_device(pdev);
			} else {
				vgt_err("Hang in render sched, panic the system.\n");
				ASSERT(0);
			}
		}
	}

	/* Handle render context switch */
	if (vgt_ctx_switch &&
	    test_and_clear_bit(VGT_REQUEST_CTX_SWITCH,
			(void *)&pdev->request)) {
		if (!vgt_do_render_context_switch(pdev)) {
			if (enable_reset) {
				vgt_err("Hang in context switch, try to reset device.\n");

				vgt_reset_device(pdev);
			} else {
				vgt_err("Hang in context switch, panic the system.\n");
				ASSERT(0);
			}
		}
	}

	if (test_and_clear_bit(VGT_REQUEST_EMUL_DPY_EVENTS,
			(void *)&pdev->request)) {
		vgt_lock_dev(pdev, cpu);
		vgt_emulate_dpy_events(pdev);
		vgt_unlock_dev(pdev, cpu);
	}

	return;
}

static void vgt_processe_hi_priority_request(struct pgt_device *pdev)
{
	int cpu;
	enum vgt_ring_id ring_id;
	bool ctx_irq_received = false;

	if (test_and_clear_bit(VGT_REQUEST_DEVICE_RESET,
				(void *)&pdev->request)) {
		vgt_reset_device(pdev);
	}

	for (ring_id = 0; ring_id < MAX_ENGINES; ++ ring_id) {
		if (test_and_clear_bit(
			VGT_REQUEST_CTX_EMULATION_RCS + ring_id,
			(void *)&pdev->request)) {
			vgt_lock_dev(pdev, cpu);
			vgt_emulate_context_switch_event(pdev, ring_id);
			vgt_unlock_dev(pdev, cpu);
			ctx_irq_received = true;
		}
	}

	if (ctx_irq_received && ctx_switch_requested(pdev)) {
		bool all_rings_empty = true;
		for (ring_id = 0; ring_id < MAX_ENGINES; ++ ring_id) {
			if(!vgt_idle_execlist(pdev, ring_id)) {
				all_rings_empty = false;
				break;
			}
		}
		if (all_rings_empty)
			vgt_raise_request(pdev, VGT_REQUEST_CTX_SWITCH);
	}

	/* forward physical GPU events to VMs */
	if (test_and_clear_bit(VGT_REQUEST_IRQ,
				(void *)&pdev->request)) {
		unsigned long flags;
		vgt_lock_dev(pdev, cpu);
		vgt_get_irq_lock(pdev, flags);
		vgt_forward_events(pdev);
		vgt_put_irq_lock(pdev, flags);
		vgt_unlock_dev(pdev, cpu);
	}

	return;
}

#define REQUEST_LOOP(pdev)	((pdev)->request & 	\
	((1<<VGT_REQUEST_IRQ) | 			\
	(1<<VGT_REQUEST_CTX_EMULATION_RCS) |		\
	(1<<VGT_REQUEST_CTX_EMULATION_VCS) |		\
	(1<<VGT_REQUEST_CTX_EMULATION_BCS) |		\
	(1<<VGT_REQUEST_CTX_EMULATION_VECS) |		\
	(1<<VGT_REQUEST_CTX_EMULATION_VCS2)))

static int vgt_thread(void *priv)
{
	struct pgt_device *pdev = (struct pgt_device *)priv;
	int ret;
	int cpu;

	//ASSERT(current_render_owner(pdev));
	printk("vGT: start kthread for dev (%x, %x)\n", pdev->bus, pdev->devfn);

	set_freezable();
	while (!kthread_should_stop()) {
		ret = wait_event_interruptible(pdev->event_wq, kthread_should_stop() ||
					pdev->request || freezing(current));

		if (ret)
			vgt_warn("Main thread waken up by unexpected signal!\n");

		if (kthread_should_stop())
			break;

		if (!pdev->request && !freezing(current)) {
			vgt_warn("Main thread waken up by unknown reasons!\n");
			continue;
		}

		if (freezing(current)) {
			if (current_render_owner(pdev) == vgt_dom0) {
				try_to_freeze();
			}
			else {
				vgt_lock_dev(pdev, cpu);
				pdev->next_sched_vgt = vgt_dom0;
				vgt_raise_request(pdev, VGT_REQUEST_SCHED);
				vgt_unlock_dev(pdev, cpu);
			}
		}

		do {
			/* give another chance for high priority request */
			vgt_processe_hi_priority_request(pdev);
		}
		while(REQUEST_LOOP(pdev));

		vgt_processe_lo_priority_request(pdev);
	}
	return 0;
}


bool initial_phys_states(struct pgt_device *pdev)
{
	struct vgt_device_info *info = &pdev->device_info;
	int i;
	uint64_t	bar0, bar1;
	struct pci_dev *dev = pdev->pdev;

	vgt_dbg(VGT_DBG_GENERIC, "VGT: Initial_phys_states\n");

	pdev->gtt_size = vgt_get_gtt_size(pdev);
	gm_sz(pdev) = pdev->gtt_size >> info->gtt_entry_size_shift << GTT_PAGE_SHIFT;

	ASSERT(gm_sz(pdev) <= info->max_gtt_gm_sz);

	pdev->saved_gtt = vzalloc(pdev->gtt_size);
	if (!pdev->saved_gtt)
		return false;

	for (i=0; i<VGT_CFG_SPACE_SZ; i+=4)
		pci_read_config_dword(dev, i,
				(uint32_t *)&pdev->initial_cfg_space[i]);

	for (i=0; i<VGT_CFG_SPACE_SZ; i+=4) {
		if (!(i % 16))
			vgt_dbg(VGT_DBG_GENERIC, "\n[%2x]: ", i);

		vgt_dbg(VGT_DBG_GENERIC, "%02x %02x %02x %02x ",
			*((uint32_t *)&pdev->initial_cfg_space[i]) & 0xff,
			(*((uint32_t *)&pdev->initial_cfg_space[i]) & 0xff00) >> 8,
			(*((uint32_t *)&pdev->initial_cfg_space[i]) & 0xff0000) >> 16,
			(*((uint32_t *)&pdev->initial_cfg_space[i]) & 0xff000000) >> 24);
	}
	for (i=0; i < 3; i++) {
		pdev->bar_size[i] = pci_bar_size(pdev, VGT_REG_CFG_SPACE_BAR0 + 8*i);
		printk("bar-%d size: %llx\n", i, pdev->bar_size[i]);
	}

	bar0 = *(uint64_t *)&pdev->initial_cfg_space[VGT_REG_CFG_SPACE_BAR0];
	bar1 = *(uint64_t *)&pdev->initial_cfg_space[VGT_REG_CFG_SPACE_BAR1];
	printk("bar0: 0x%llx, bar1: 0x%llx\n", bar0, bar1);

	ASSERT ((bar0 & 7) == 4);
	/* memory, 64 bits bar0 */
	pdev->gttmmio_base = bar0 & ~0xf;
	pdev->mmio_size = VGT_MMIO_SPACE_SZ;
	pdev->reg_num = pdev->mmio_size/REG_SIZE;
	printk("mmio size: %x, gtt size: %llx\n", pdev->mmio_size,
		pdev->gtt_size);
	ASSERT(pdev->mmio_size + pdev->gtt_size <= pdev->bar_size[0]);

	ASSERT ((bar1 & 7) == 4);
	/* memory, 64 bits bar */
	pdev->gmadr_base = bar1 & ~0xf;
	printk("gttmmio: 0x%llx, gmadr: 0x%llx\n", pdev->gttmmio_base, pdev->gmadr_base);

	pdev->gttmmio_base_va = ioremap(pdev->gttmmio_base, pdev->bar_size[0]);
	if (pdev->gttmmio_base_va == NULL) {
		WARN_ONCE(1, "insufficient memory for ioremap!\n");
		return false;
	}
	printk("gttmmio_base_va: 0x%llx\n", (uint64_t)pdev->gttmmio_base_va);

	/*
	 * From now on, the vgt driver can invoke the
	 * VGT_MMIO_READ()/VGT_MMIO_WRITE()hypercalls, and any access to the
	 * 4MB MMIO of the GEN device is trapped into the vgt driver.
	 */

	// TODO: runtime sanity check warning...
	pdev->gmadr_va = ioremap (pdev->gmadr_base, pdev->bar_size[1]);
	if ( pdev->gmadr_va == NULL ) {
		iounmap(pdev->gttmmio_base_va);
		printk("Insufficient memory for ioremap2\n");
		return false;
	}
	printk("gmadr_va: 0x%llx\n", (uint64_t)pdev->gmadr_va);

	vgt_initial_mmio_setup(pdev);
	vgt_initial_opregion_setup(pdev);

	/* FIXME: GMBUS2 has an in-use bit as the hw semaphore, and we should recover
	 * it after the snapshot. Remove this workaround after GMBUS virtualization
	 */
	{
		u32 val = VGT_MMIO_READ(pdev, 0xc5108);
		pdev->initial_mmio_state[REG_INDEX(0xc5108)] &= ~0x8000;
		printk("vGT: GMBUS2 init value: %x, %x\n", pdev->initial_mmio_state[REG_INDEX(0xc5108)], val);
		VGT_MMIO_WRITE(pdev, 0xc5108, val | 0x8000);
	}

	for (i = 0; i < MAX_ENGINES; ++ i) {
		pdev->el_read_ptr[i] = DEFAULT_INV_SR_PTR;
	}

	return true;
}

static bool vgt_set_device_type(struct pgt_device *pdev)
{
	if (_is_sandybridge(pdev->pdev->device)) {
		pdev->gen_dev_type = IGD_SNB;
		vgt_info("Detected Sandybridge\n");
		return true;
	}

	if (_is_ivybridge(pdev->pdev->device)) {
		pdev->gen_dev_type = IGD_IVB;
		vgt_info("Detected Ivybridge\n");
		return true;
	}

	if (_is_haswell(pdev->pdev->device)) {
		pdev->gen_dev_type = IGD_HSW;
		vgt_info("Detected Haswell\n");
		return true;
	}

	if (_is_broadwell(pdev->pdev->device)) {
		pdev->gen_dev_type = IGD_BDW;
		vgt_info("Detected Broadwell\n");
		return true;
	}

	if (_is_skylake(pdev->pdev->device)) {
		pdev->gen_dev_type = IGD_SKL;
		vgt_info("Detected Skylake\n");
		return true;
	}

	vgt_err("Unknown chip 0x%x\n", pdev->pdev->device);
	return false;
}

static bool vgt_initialize_device_info(struct pgt_device *pdev)
{
	struct vgt_device_info *info = &pdev->device_info;

	if (!vgt_set_device_type(pdev))
		return false;

	if (!IS_HSW(pdev) && !IS_BDW(pdev) && !IS_SKL(pdev)) {
		vgt_err("Unsupported gen_dev_type(%s)!\n",
			IS_IVB(pdev) ?
			"IVB" : "SNB(or unknown GEN types)");
		return false;
	}

	if (IS_SKL(pdev) && !vgt_preliminary_hw_support) {
		vgt_err("VGT haven't fully supported preliminary platform: skylake.\n");
		return false;
	}

	if (IS_HSW(pdev)) {
		info->gen = MKGEN(7, 5, 0);
		info->max_gtt_gm_sz = (1UL << 31);	/* 2G */
		/*
		 * The layout of BAR0 in PreBDW:
		 * |< - MMIO 2MB ->|<- MAX GTT 2MB ->|
		 *
		 * GTT offset in BAR0 starts from 2MB to 4MB
		 */
		info->gtt_start_offset = (1UL << 21);
		info->max_gtt_size = (1UL << 21);
		info->gtt_entry_size = 4;
		info->gtt_entry_size_shift = 2;
		info->gmadr_bytes_in_cmd = 4;
	} else if (IS_BDW(pdev) || IS_SKL(pdev)) {
		int gen = IS_BDW(pdev) ? 8 : 9;

		info->gen = MKGEN(gen, 0, ((pdev->pdev->device >> 4) & 0xf) + 1);
		info->max_gtt_gm_sz = (1UL << 32);
		/*
		 * The layout of BAR0 in BDW:
		 * |< - MMIO 2MB ->|<- Reserved 6MB ->|<- MAX GTT 8MB->|
		 *
		 * GTT offset in BAR0 starts from 8MB to 16MB, and
		 * Whatever GTT size is configured in BIOS,
		 * the size of BAR0 is always 16MB. The actual configured
		 * GTT size can be found in GMCH_CTRL.
		 */
		info->gtt_start_offset = (1UL << 23);
		info->max_gtt_size = (1UL << 23);
		info->gtt_entry_size = 8;
		info->gtt_entry_size_shift = 3;
		info->gmadr_bytes_in_cmd = 8;
	}

	printk("GEN device info:\n");
	printk("	major: %u minor: %u rev: %u\n", GEN_MAJOR(info->gen),
			GEN_MINOR(info->gen), GEN_REV(info->gen));
	printk("	max_gtt_gm_sz: %llx\n", info->max_gtt_gm_sz);
	printk("	gtt_start_offset: %x\n", info->gtt_start_offset);
	printk("	max_gtt_size: %x\n", info->max_gtt_size);
	printk("	gtt_size_entry: %x\n", info->gtt_entry_size);
	printk("	gtt_entry_size_shift: %x.\n", info->gtt_entry_size_shift);

	return true;
}

static bool vgt_get_memory_latency(struct pgt_device *pdev)
{
	if (VGT_MMIO_READ(pdev, 0x138124) & (1 << 31))
		goto timeout;

	/* first set */
	VGT_MMIO_WRITE(pdev, 0x138128, 0);
	VGT_MMIO_WRITE(pdev, 0x13812c, 0);

	VGT_MMIO_WRITE(pdev, 0x138124, (1 << 31) | 0x6);

	if (wait_for_atomic(!(VGT_MMIO_READ(pdev, 0x138124) & (1 << 31)), 500))
		goto timeout;

	pdev->memory_latency[0] = VGT_MMIO_READ(pdev, 0x138128);

	/* second set */
	VGT_MMIO_WRITE(pdev, 0x138128, 1);
	VGT_MMIO_WRITE(pdev, 0x13812c, 0);

	VGT_MMIO_WRITE(pdev, 0x138124, (1 << 31) | 0x6);

	if (wait_for_atomic(!(VGT_MMIO_READ(pdev, 0x138124) & (1 << 31)), 500))
		goto timeout;

	pdev->memory_latency[1] = VGT_MMIO_READ(pdev, 0x138128);

	printk("vgt: memory latency: [0] %x [1] %x\n", pdev->memory_latency[0],
		pdev->memory_latency[1]);

	return true;

timeout:
	vgt_err("wait mailbox idle timeout!\n");
	return false;
}

static bool vgt_initialize_platform(struct pgt_device *pdev)
{
	/* check PPGTT enabling. */
	if (IS_IVB(pdev) || IS_HSW(pdev) || IS_BDWPLUS(pdev))
		pdev->enable_ppgtt = 1;

	/* execlist depends on ppgtt */
	if (pdev->enable_ppgtt) {
		if (IS_BDWPLUS(pdev))
			pdev->enable_execlist = 1;
	}

	pdev->max_engines = 3;
	pdev->ring_mmio_base[RING_BUFFER_RCS] = _REG_RCS_TAIL;
	pdev->ring_mmio_base[RING_BUFFER_VCS] = _REG_VCS_TAIL;
	pdev->ring_mmio_base[RING_BUFFER_BCS] = _REG_BCS_TAIL;

	pdev->ring_mi_mode[RING_BUFFER_RCS] = MI_MODE;
	pdev->ring_mi_mode[RING_BUFFER_VCS] = _REG_VCS_MI_MODE;
	pdev->ring_mi_mode[RING_BUFFER_BCS] = _REG_BCS_MI_MODE;

	pdev->ring_xxx[RING_BUFFER_RCS] = 0x2050;
	pdev->ring_xxx[RING_BUFFER_VCS] = 0x12050;
	pdev->ring_xxx[RING_BUFFER_BCS] = 0x22050;
	pdev->ring_xxx_bit[RING_BUFFER_RCS] = 3;
	pdev->ring_xxx_bit[RING_BUFFER_VCS] = 3;
	pdev->ring_xxx_bit[RING_BUFFER_BCS] = 3;
	/* this check is broken on SNB */
	pdev->ring_xxx_valid = 0;

	if (IS_HSW(pdev)) {
		pdev->max_engines = 4;
		pdev->ring_mmio_base[RING_BUFFER_VECS] = _REG_VECS_TAIL;
		pdev->ring_mi_mode[RING_BUFFER_VECS] = _REG_VECS_MI_MODE;
		pdev->ring_xxx[RING_BUFFER_RCS] = 0x8000;
		pdev->ring_xxx[RING_BUFFER_VCS] = 0x8000;
		pdev->ring_xxx[RING_BUFFER_BCS] = 0x8000;
		pdev->ring_xxx[RING_BUFFER_VECS] = 0x8008;
		pdev->ring_xxx_bit[RING_BUFFER_RCS] = 0;
		pdev->ring_xxx_bit[RING_BUFFER_VCS] = 1;
		pdev->ring_xxx_bit[RING_BUFFER_BCS] = 2;
		pdev->ring_xxx_bit[RING_BUFFER_VECS] = 10;
		pdev->ring_xxx_valid = 1;
	} else if (IS_BDWPLUS(pdev)) {
		pdev->max_engines = 4;
		pdev->ring_mmio_base[RING_BUFFER_VECS] = _REG_VECS_TAIL;
		pdev->ring_mi_mode[RING_BUFFER_VECS] = _REG_VECS_MI_MODE;
		pdev->ring_xxx_valid = 0;

		/*
		 * Add GT3 VCS2 ring for BDW and SKL GT3/4
		 */
		if (IS_BDWGT3(pdev) || IS_SKLGT3(pdev) || IS_SKLGT4(pdev)) {
			pdev->max_engines = 5;
			pdev->ring_mmio_base[RING_BUFFER_VCS2] = _REG_VCS2_TAIL;
			pdev->ring_mi_mode[RING_BUFFER_VCS2] = _REG_VCS2_MI_MODE;
			pdev->ring_xxx[RING_BUFFER_VCS2] = 0x8008;
			pdev->ring_xxx_bit[RING_BUFFER_VCS2] = 0;
		}

		if (IS_SKL(pdev)) {
			enable_panel_fitting = false;
			vgt_get_memory_latency(pdev);
			VGT_MMIO_WRITE(pdev, 0x4dfc, 0x1);
		}
	} else {
		vgt_err("Unsupported platform.\n");
		return false;
	}

	return true;
}

static bool vgt_initialize_pgt_device(struct pci_dev *dev, struct pgt_device *pdev)
{
	int i;

	pdev->pdev = dev;
	pdev->pbus = dev->bus;

	if (!vgt_initialize_device_info(pdev)) {
		vgt_err("failed to initalize device info.\n");
		return false;
	}

	if (!vgt_initialize_platform(pdev)) {
		vgt_err("failed to initialize platform\n");
		return false;
	}

	INIT_LIST_HEAD(&pdev->rendering_runq_head);
	INIT_LIST_HEAD(&pdev->rendering_idleq_head);

	bitmap_zero(pdev->dpy_emul_request, VGT_MAX_VMS);

	/* initialize ports */
	memset(pdev->ports, 0, sizeof(struct gt_port) * I915_MAX_PORTS);
	for (i = 0; i < I915_MAX_PORTS; i ++) {
		pdev->ports[i].type = VGT_PORT_MAX;
		pdev->ports[i].cache.type = VGT_PORT_MAX;
		pdev->ports[i].port_override = i;
		pdev->ports[i].physcal_port = i;
	}

	if (!initial_phys_states(pdev)) {
		printk("vGT: failed to initialize physical state\n");
		return false;
	}

	pdev->reg_info = vzalloc (pdev->reg_num * sizeof(reg_info_t));
	if (!pdev->reg_info) {
		printk("vGT: failed to allocate reg_info\n");
		return false;
	}

	initialize_gm_fence_allocation_bitmaps(pdev);

	vgt_setup_reg_info(pdev);
	vgt_post_setup_mmio_hooks(pdev);
	if (vgt_irq_init(pdev) != 0) {
		printk("vGT: failed to initialize irq\n");
		return false;
	}

	if (!vgt_gtt_init(pdev)) {
		vgt_err("failed to initialize gtt\n");
		return false;
	}

	vgt_init_reserved_aperture(pdev);

	for (i = 0; i < pdev->max_engines; i++)
		vgt_ring_init(pdev, i);

	perf_pgt = pdev;
	return true;
}

void vgt_destroy(void)
{
	struct vgt_device *vgt, *tmp;
	struct pgt_device *pdev = &default_device;
	int i;
	unsigned long flags;

	vgt_cleanup_mmio_dev(pdev);

	perf_pgt = NULL;
	list_del(&pdev->list);

	vgt_cleanup_ctx_scheduler(pdev);

	cancel_work_sync(&pdev->hpd_work.work);

	/* do we need the thread actually stopped? */
	kthread_stop(pdev->p_thread);

	vgt_irq_exit(pdev);

	/* Deactive all VGTs */
	spin_lock_irqsave(&pdev->lock, flags);

	list_for_each_entry_safe(vgt, tmp, &pdev->rendering_runq_head, list)
		vgt_disable_render(vgt);

	if (pdev->saved_gtt)
		vfree(pdev->saved_gtt);
	free_gtt(pdev);
	vgt_gtt_clean(pdev);

	if (pdev->gmadr_va)
		iounmap(pdev->gmadr_va);
	if (pdev->opregion_va)
		iounmap(pdev->opregion_va);

	spin_unlock_irqrestore(&pdev->lock, flags);

	/* destruct all vgt-related debugfs/sysfs */
	vgt_release_debugfs();
	vgt_destroy_sysfs();

	list_for_each_entry_safe(vgt, tmp, &pdev->rendering_idleq_head, list)
		vgt_release_instance(vgt);

	vgt_clear_mmio_table();
	vfree(pdev->reg_info);
	vfree(pdev->initial_mmio_state);

	for (i = 0; i < I915_MAX_PORTS; ++ i) {
		if (pdev->ports[i].edid) {
			kfree(pdev->ports[i].edid);
			pdev->ports[i].edid = NULL;
		}

		if (pdev->ports[i].dpcd) {
			kfree(pdev->ports[i].dpcd);
			pdev->ports[i].dpcd = NULL;
		}

		if (pdev->ports[i].cache.edid) {
			kfree(pdev->ports[i].cache.edid);
			pdev->ports[i].cache.edid = NULL;
		}
	}

	vgt_cmd_parser_exit();
}

static int vgt_initialize(struct pci_dev *dev)
{
	struct pgt_device *pdev = &default_device;
	struct task_struct *p_thread;
	vgt_params_t vp;

	spin_lock_init(&pdev->lock);

	if (!vgt_initialize_pgt_device(dev, pdev))
		return -EINVAL;

	if (vgt_cmd_parser_init(pdev) < 0)
		goto err;

	mutex_init(&pdev->hpd_work.hpd_mutex);
	INIT_WORK(&pdev->hpd_work.work, vgt_hotplug_udev_notify_func);
	
	/* create debugfs interface */
	if (!vgt_init_debugfs(pdev)) {
		printk("vGT:failed to create debugfs\n");
		goto err;
	}

	/* init all mmio_device */
	vgt_init_mmio_device(pdev);

	/* create domain 0 instance */
	vp.vm_id = 0;
	vp.aperture_sz = dom0_low_gm_sz;
	vp.gm_sz = dom0_low_gm_sz + dom0_high_gm_sz;
	vp.fence_sz = dom0_fence_sz;
	vp.vgt_primary = 1; /* this isn't actually used for dom0 */
	if (create_vgt_instance(pdev, &vgt_dom0, vp) < 0)
		goto err;

	reset_cached_interrupt_registers(pdev);

	vgt_dbg(VGT_DBG_GENERIC, "create dom0 instance succeeds\n");

	//show_mode_settings(pdev);

	if (setup_gtt(pdev))
		goto err;

	if (!hvm_render_owner)
		current_render_owner(pdev) = vgt_dom0;
	else
		vgt_ctx_switch = 0;

	current_foreground_vm(pdev) = vgt_dom0;
	if (!hvm_display_owner) {
		current_display_owner(pdev) = vgt_dom0;
	}

	if (hvm_super_owner) {
		ASSERT(hvm_render_owner);
		ASSERT(hvm_display_owner);
		ASSERT(hvm_boot_foreground);
	} else {
		current_config_owner(pdev) = vgt_dom0;
	}

	pdev->ctx_check = 0;
	pdev->ctx_switch = 0;
	pdev->magic = 0;

	init_waitqueue_head(&pdev->event_wq);
	init_waitqueue_head(&pdev->destroy_wq);

	pdev->device_reset_flags = 0;

	p_thread = kthread_run(vgt_thread, pdev, "vgt_main");
	if (!p_thread) {
		goto err;
	}
	pdev->p_thread = p_thread;
	//show_debug(pdev, 0);

	vgt_initialize_ctx_scheduler(pdev);

	list_add(&pdev->list, &pgt_devices);

	vgt_init_sysfs(pdev);

	vgt_init_fb_notify();

	printk("vgt_initialize succeeds.\n");

	return 0;
err:
	printk("vgt_initialize failed.\n");
	vgt_destroy();
	return -1;
}

int vgt_suspend(struct pci_dev *pdev)
{
	struct pgt_device *node, *pgt = NULL;

	if (!hypervisor_check_host() || !vgt_enabled)
		return 0;

	if (list_empty(&pgt_devices)) {
		printk("vGT: no valid pgt_device registered at suspend\n");
		return 0;
	}

	list_for_each_entry(node, &pgt_devices, list) {
		if (node->pdev == pdev) {
			pgt = node;
			break;
		}
	}

	if (!pgt) {
		printk("vGT: no matching pgt_device at suspend\n");
		return 0;
	}

	vgt_host_irq_sync();
	vgt_info("Suspending vGT driver...\n");

	/* TODO: check vGT instance state */
	/* ... */

	pgt->saved_rrmr = VGT_MMIO_READ(pgt, _REG_DE_RRMR);
	pgt->saved_shotplug_ctl = VGT_MMIO_READ(pgt, PCH_PORT_HOTPLUG);

	/* save GTT and FENCE information */
	vgt_save_gtt_and_fence(pgt);

	vgt_reset_ppgtt(vgt_dom0, 0xff);

	return 0;
}

int vgt_resume(struct pci_dev *pdev)
{
	struct pgt_device *node, *pgt = NULL;

	if (!hypervisor_check_host() || !vgt_enabled)
		return 0;


	if (list_empty(&pgt_devices)) {
		printk("vGT: no valid pgt_device registered at resume\n");
		return 0;
	}

	list_for_each_entry(node, &pgt_devices, list) {
		if (node->pdev == pdev) {
			pgt = node;
			break;
		}
	}

	if (!pgt) {
		printk("vGT: no matching pgt_device at resume\n");
		return 0;
	}

	vgt_info("Resuming vGT driver...\n");

	/* restore GTT table and FENCE regs */
	vgt_restore_gtt_and_fence(pgt);

	VGT_MMIO_WRITE(pgt, _REG_DE_RRMR, pgt->saved_rrmr);
	VGT_MMIO_WRITE(pgt, PCH_PORT_HOTPLUG, pgt->saved_shotplug_ctl);

	/* redo the MMIO snapshot */
	vgt_initial_mmio_setup(pgt);

	/* XXX: need redo the PCI config space snapshot too? */

	/*
	 * TODO: need a better place to sync vmmio state
	 * for now, force override dom0's vmmio only. other
	 * VMs are supposed to be paused.
	 */
	state_sreg_init(vgt_dom0);
	state_vreg_init(vgt_dom0);

	/* TODO, GMBUS inuse bit? */

	spin_lock(&pgt->lock);

	recalculate_and_update_imr(pgt, DEIMR);
	recalculate_and_update_imr(pgt, GTIMR);
	recalculate_and_update_imr(pgt, GEN6_PMIMR);
	recalculate_and_update_imr(pgt, SDEIMR);

	recalculate_and_update_imr(pgt, IMR);
	recalculate_and_update_imr(pgt, _REG_BCS_IMR);
	recalculate_and_update_imr(pgt, _REG_VCS_IMR);

	if (IS_HSW(pgt))
		recalculate_and_update_imr(pgt, _REG_VECS_IMR);

	recalculate_and_update_ier(pgt, GTIER);
	recalculate_and_update_ier(pgt, GEN6_PMIER);
	recalculate_and_update_ier(pgt, SDEIER);

	if (pgt->enable_execlist) {
		enum vgt_ring_id ring_id;
		for (ring_id = 0; ring_id < MAX_ENGINES; ++ ring_id)
			reset_el_structure(pgt, ring_id);
	}

	spin_unlock(&pgt->lock);

	return 0;
}

/*
 * Kernel BUG() doesn't work, because bust_spinlocks try to unblank screen
 * which may call into i915 and thus cause undesired more errors on the
 * screen
 */
void vgt_panic(void)
{
        struct pgt_device *pdev = &default_device;
 
        show_debug(pdev);
 
        dump_stack();
        printk("________end of stack dump_________\n");
        panic("FATAL VGT ERROR\n");
}

static void do_device_reset(struct pgt_device *pdev)
{
	struct drm_device *drm_dev = pci_get_drvdata(pdev->pdev);
	vgt_reg_t head, tail, start, ctl;
	vgt_reg_t ier, imr, iir, isr;
	int i;

	vgt_info("Request DOM0 to reset device.\n");

	ASSERT(drm_dev);

	set_bit(WAIT_RESET, &vgt_dom0->reset_flags);

	i915_handle_error(drm_dev, true, "VGT device reset");

	i915_wait_error_work_complete(drm_dev);

	/*
	 * User may set i915.reset=0 in kernel command line, which will
	 * disable the reset logic of i915, without that logics we can
	 * do nothing, so we panic here and let user remove that parameters.
	 */
	if (test_bit(WAIT_RESET, &vgt_dom0->reset_flags)) {
		vgt_err("DOM0 GPU reset didn't happen?.\n");
		vgt_err("Maybe you set i915.reset=0 in kernel command line? Panic the system.\n");
		ASSERT(0);
	}

	if (IS_PREBDW(pdev)) {
		vgt_info("GPU ring status:\n");

		for (i = 0; i < pdev->max_engines; i++) {
			head = VGT_READ_HEAD(pdev, i);
			tail = VGT_READ_TAIL(pdev, i);
			start = VGT_READ_START(pdev, i);
			ctl = VGT_READ_CTL(pdev, i);

			vgt_info("RING %d: H: %x T: %x S: %x C: %x.\n",
					i, head, tail, start, ctl);
		}

		ier = VGT_MMIO_READ(pdev, DEIER);
		iir = VGT_MMIO_READ(pdev, DEIIR);
		imr = VGT_MMIO_READ(pdev, DEIMR);
		isr = VGT_MMIO_READ(pdev, DEISR);

		vgt_info("DE: ier: %x iir: %x imr: %x isr: %x.\n",
				ier, iir, imr, isr);
	} else {
		for (i = 0; i < pdev->max_engines; i++) {
			if (pdev->enable_execlist)
				reset_el_structure(pdev, i);
		}
	}

	vgt_info("Finish.\n");

	return;
}

bool vgt_handle_dom0_device_reset(void)
{
	struct pgt_device *pdev = &default_device;
	struct drm_device *drm_dev;

	unsigned long flags;
	int cpu;

	int id;
	bool rc;

	if (!hypervisor_check_host() || !vgt_enabled)
		return false;

	vgt_info("DOM0 hangcheck timer request reset device.\n");

	drm_dev = pci_get_drvdata(pdev->pdev);
	ASSERT(drm_dev);

	vgt_lock_dev_flags(pdev, cpu, flags);
	rc = idle_rendering_engines(pdev, &id);
	vgt_unlock_dev_flags(pdev, cpu, flags);

	if (!rc) {
		vgt_info("Really hung, request to reset device.\n");
		vgt_raise_request(pdev, VGT_REQUEST_DEVICE_RESET);
	} else {
		vgt_info("Not really hung, continue DOM0 reset sequence.\n");
		i915_handle_error(drm_dev, true, "VGT DOM0 device reset");
	}

	return true;
}

int vgt_reset_device(struct pgt_device *pdev)
{
	struct vgt_irq_host_state *hstate = pdev->irq_hstate;
	struct vgt_device *vgt;
	struct list_head *pos, *n;
	unsigned long ier_reg = IS_PREBDW(pdev) ? DEIER : GEN8_MASTER_IRQ;
	unsigned long ier_value;
	unsigned long flags;
	int i;

	if (get_seconds() - vgt_dom0->last_reset_time < 6) {
		vgt_err("Try to reset device too fast.\n");
		return -EAGAIN;
	}

	if (test_and_set_bit(RESET_INPROGRESS,
				&pdev->device_reset_flags)) {
		vgt_err("Another device reset has been already running.\n");
		return -EBUSY;
	}

	vgt_info("Stop VGT context switch.\n");

	vgt_cleanup_ctx_scheduler(pdev);

	current_render_owner(pdev) = vgt_dom0;

	current_foreground_vm(pdev) = vgt_dom0;

	spin_lock_irqsave(&pdev->lock, flags);

	list_for_each_safe(pos, n, &pdev->rendering_runq_head) {
		vgt = list_entry(pos, struct vgt_device, list);

		if (vgt->vm_id) {
			for (i = 0; i < pdev->max_engines; i++) {
				if (test_bit(i, (void *)vgt->enabled_rings)) {
					vgt_info("VM %d: disable ring %d\n", vgt->vm_id, i);

					vgt_disable_ring(vgt, i);

					set_bit(i, &vgt->enabled_rings_before_reset);
				}
			}

			set_bit(WAIT_RESET, &vgt->reset_flags);
		}
	}

	spin_unlock_irqrestore(&pdev->lock, flags);

	vgt_info("Disable master interrupt.\n");

	vgt_get_irq_lock(pdev, flags);

	hstate->ops->disable_irq(hstate);

	vgt_put_irq_lock(pdev, flags);

	do_device_reset(pdev);

	vgt_info("Restart VGT context switch.\n");

	vgt_initialize_ctx_scheduler(pdev);

	clear_bit(RESET_INPROGRESS, &pdev->device_reset_flags);

	spin_lock_irqsave(&pdev->lock, flags);
	vgt_get_irq_lock(pdev, flags);

	reset_cached_interrupt_registers(pdev);

	ier_value = vgt_recalculate_ier(pdev, ier_reg);
	VGT_MMIO_WRITE(pdev, ier_reg, ier_value);

	vgt_put_irq_lock(pdev, flags);

	spin_unlock_irqrestore(&pdev->lock, flags);

	vgt_info("Enable master interrupt, master ier register %lx value %lx\n",
			ier_reg, ier_value);

	return 0;
}

static void vgt_param_check(void)
{
	/* TODO: hvm_display/render_owner are broken */
	if (hvm_super_owner) {
		hvm_display_owner = true;
		hvm_render_owner = true;
		hvm_boot_foreground = true;
	}

	if (hvm_display_owner) {
		hvm_boot_foreground = true;
	}

	if (hvm_dpy_owner) {
		vgt_warn("hvm_dpy_owner is deprecated option! "
			 "Please use hvm_boot_foreground or hvm_display_owner instead!\n");
	}

	/* see the comment where dom0_low_gm_sz is defined */
	if (dom0_low_gm_sz > 512 - 64)
		dom0_low_gm_sz = 512 - 64;

	if (dom0_low_gm_sz + dom0_high_gm_sz > 2048)
		dom0_high_gm_sz = 2048 - dom0_low_gm_sz;

	if (dom0_fence_sz > VGT_MAX_NUM_FENCES)
		dom0_fence_sz = VGT_MAX_NUM_FENCES;
}

bool vgt_check_host(void)
{
	if (!vgt_enabled)
		return false;

	if (!vgt_pkdm)
		return false;

	if (!hypervisor_check_host())
		return false;

	return true;
}

void i915_stop_vgt(void)
{
	vgt_destroy();
	__symbol_put(vgt_pkdm->name);
	vgt_pkdm = NULL;
	vgt_ops = NULL;
}

bool i915_start_vgt(struct pci_dev *pdev)
{
	vgt_ops = &__vgt_ops;

	if (xen_initial_domain()) {
		/* Xen Dom0 */
		vgt_pkdm = try_then_request_module(symbol_get(xengt_kdm), "xengt");
		if (!vgt_pkdm)
			return false;
	} else if (xen_domain()) {
		/* Xen DomU */
		return false;
	} else {
		/* not in Xen. Try KVMGT */
		vgt_pkdm = try_then_request_module(symbol_get(kvmgt_kdm), "kvm");
		if (!vgt_pkdm)
			return false;

		opregion_present = true;
	}


	if (!vgt_check_host())
		return false;

	vgt_param_check();

	return vgt_initialize(pdev) == 0;
}
