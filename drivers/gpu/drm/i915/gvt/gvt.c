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

#include <linux/types.h>
#include <xen/xen.h>
#include <linux/kthread.h>

#include "gvt.h"

struct gvt_host gvt_host;
EXPORT_SYMBOL(gvt_host);

extern struct gvt_kernel_dm xengt_kdm;
extern struct gvt_kernel_dm kvmgt_kdm;

static struct gvt_io_emulation_ops default_io_emulation_ops = {
	.emulate_mmio_read = gvt_emulate_mmio_read,
	.emulate_mmio_write = gvt_emulate_mmio_write,
	.emulate_cfg_read = gvt_emulate_cfg_read,
	.emulate_cfg_write = gvt_emulate_cfg_write,
};

unsigned int pa_to_mmio_offset(struct vgt_device *vgt,
               uint64_t pa);

static struct gvt_mpt_ops default_export_mpt_ops = {
	.pa_to_mmio_offset = pa_to_mmio_offset,
};

static const char *supported_hypervisors[] = {
	[GVT_HYPERVISOR_TYPE_XEN] = "Xen Hypervisor",
	[GVT_HYPERVISOR_TYPE_KVM] = "KVM",
};

static bool gvt_init_host(void)
{
	struct gvt_host *host = &gvt_host;

	if (!gvt.enable) {
		gvt_dbg_core("GVT-g has been disabled by kernel parameter");
		return false;
	}

	if (host->initialized) {
		gvt_err("GVT-g has already been initialized!");
		return false;
	}

	if (xen_initial_domain()) {
		/* Xen Dom0 */
		host->kdm = try_then_request_module(symbol_get(xengt_kdm), "xengt");
		host->hypervisor_type = GVT_HYPERVISOR_TYPE_XEN;
	} else if(xen_domain()) {
		/* Xen DomU */
		return false;
	} else {
		/* not in Xen. Try KVMGT */
		host->kdm = try_then_request_module(symbol_get(kvmgt_kdm), "kvm");
		host->hypervisor_type = GVT_HYPERVISOR_TYPE_KVM;
	}

	if (!host->kdm)
		return false;

	if (!hypervisor_detect_host())
		return false;

	gvt_info("Running with hypervisor %s in host mode",
			supported_hypervisors[host->hypervisor_type]);

	host->emulate_ops = &default_io_emulation_ops;
	host->mpt_ops = &default_export_mpt_ops;
	idr_init(&host->device_idr);
	mutex_init(&host->device_idr_lock);

	host->initialized = true;
	return true;
}

static bool init_device_info(struct pgt_device *pdev)
{
	struct gvt_device_info *info = &pdev->device_info;

	if (!IS_BROADWELL(pdev->dev_priv)) {
		gvt_err("Unsupported GEN device");
		return false;
	}

	if (IS_BROADWELL(pdev->dev_priv)) {
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

	gvt_info("Device info:");
	printk("        max_gtt_gm_sz: %llx\n", info->max_gtt_gm_sz);
	printk("        max_gtt_size: %x\n", info->max_gtt_size);
	printk("        gtt_size_entry: %x\n", info->gtt_entry_size);
	printk("        gtt_entry_size_shift: %x\n", info->gtt_entry_size_shift);
	printk("        gtt_start_offset: %x\n", info->gtt_start_offset);
	printk("        gtt_end_offset: %x\n", info->gtt_end_offset);

	if (!gvt_init_avail_gtt_size(pdev)) {
		gvt_err("fail to get avail gtt size");
		return false;
	}
	return true;
}

static void init_initial_cfg_space_state(struct pgt_device *pdev)
{
	struct pci_dev *pci_dev = pdev->dev_priv->dev->pdev;
	int i;

	gvt_dbg_core("init initial cfg space, id %d", pdev->id);

	for (i = 0; i < GVT_CFG_SPACE_SZ; i += 4)
		pci_read_config_dword(pci_dev, i,
				(u32 *)&pdev->initial_cfg_space[i]);

	for (i = 0; i < 3; i++) {
		pdev->bar_size[i] = pci_resource_len(pci_dev, i * 2);
		gvt_info("bar %d size: %llx", i, pdev->bar_size[i]);
	}
}

static void clean_initial_mmio_state(struct pgt_device *pdev)
{
	gvt_clean_initial_mmio_state(pdev);

	if (pdev->gttmmio_va) {
		iounmap(pdev->gttmmio_va);
		pdev->gttmmio_va = NULL;
	}

	if (pdev->gmadr_va) {
		iounmap(pdev->gmadr_va);
		pdev->gmadr_va = NULL;
	}

	if(pdev->reg_info) {
		vfree(pdev->reg_info);
		pdev->reg_info = NULL;
	}
}

static bool init_initial_mmio_state(struct pgt_device *pdev)
{
	u64 bar0, bar1;

	gvt_dbg_core("init initial mmio state, id %d", pdev->id);

	bar0 = *(u64 *)&pdev->initial_cfg_space[GVT_REG_CFG_SPACE_BAR0];
	bar1 = *(u64 *)&pdev->initial_cfg_space[GVT_REG_CFG_SPACE_BAR1];

	pdev->gttmmio_base = bar0 & ~0xf;
	pdev->mmio_size = 2 * 1024 * 1024;
	pdev->reg_num = pdev->mmio_size / 4;
	pdev->gmadr_base = bar1 & ~0xf;

	pdev->gttmmio_va = ioremap(pdev->gttmmio_base, pdev->bar_size[0]);
	if (!pdev->gttmmio_va) {
		gvt_err("fail to map GTTMMIO BAR.");
		return false;
	}

	pdev->gmadr_va = ioremap(pdev->gmadr_base, pdev->bar_size[2]);
	if (!pdev->gmadr_va) {
		gvt_err("fail to map GMADR BAR.");
		goto err;
	}

	pdev->reg_info = vzalloc(pdev->reg_num * sizeof(u32));
	if (!pdev->reg_info) {
		printk("vGT: failed to allocate reg_info\n");
		goto err;
	}

	gvt_info("bar0: 0x%llx, bar1: 0x%llx", bar0, bar1);
	gvt_info("mmio size: %x", pdev->mmio_size);
	gvt_info("gttmmio: 0x%llx, gmadr: 0x%llx", pdev->gttmmio_base, pdev->gmadr_base);
	gvt_info("gttmmio_va: %p", pdev->gttmmio_va);
	gvt_info("gmadr_va: %p", pdev->gmadr_va);

	if (!gvt_setup_initial_mmio_state(pdev))
		goto err;

	return true;
err:
	clean_initial_mmio_state(pdev);
	return false;
}

static int gvt_service_thread(void *data)
{
	struct pgt_device *pdev = (struct pgt_device *)data;
	int r;

	gvt_dbg_core("service thread start, pgt %d", pdev->id);

	while(!kthread_should_stop()) {
		r = wait_event_interruptible(pdev->service_thread_wq,
				kthread_should_stop() || pdev->service_request);

		if (kthread_should_stop())
			break;

		if (test_and_clear_bit(GVT_REQUEST_UEVENT,
					(void *)&pdev->service_request)) {
			gvt_dpy_ready_uevent_handler(pdev);
		}

		if (test_and_clear_bit(GVT_REQUEST_EMUL_DPY_EVENTS,
					(void *)&pdev->service_request)) {
			mutex_lock(&pdev->lock);
			gvt_emulate_display_events(pdev);
			mutex_unlock(&pdev->lock);
		}

		if (r) {
			gvt_warn("service thread is waken up by unexpected signal.");
			continue;
		}
	}
	return 0;
}

static void clean_service_thread(struct pgt_device *pdev)
{
	if (pdev->service_thread) {
		kthread_stop(pdev->service_thread);
		pdev->service_thread = NULL;
	}
}

static bool init_service_thread(struct pgt_device *pdev)
{
	init_waitqueue_head(&pdev->service_thread_wq);

	pdev->service_thread = kthread_run(gvt_service_thread,
			pdev, "gvt_service_thread%d", pdev->id);

	if (!pdev->service_thread) {
		gvt_err("fail to start service thread.");
		return false;
	}

	return true;
}

static void clean_pgt_device(struct pgt_device *pdev)
{
	clean_service_thread(pdev);
	gvt_clean_cmd_parser(pdev);
	gvt_clean_workload_scheduler(pdev);
	gvt_clean_control_interface(pdev);
	gvt_clean_gtt(pdev);
	gvt_irq_exit(pdev);
	gvt_clean_mmio_emulation_state(pdev);
	gvt_clean_opregion(pdev);
	clean_initial_mmio_state(pdev);
	gvt_clean_resource_allocator(pdev);
}

static bool init_pgt_device(struct pgt_device *pdev, struct drm_i915_private *dev_priv)
{
	if (!init_device_info(pdev))
		return false;

	init_initial_cfg_space_state(pdev);

	if (!init_initial_mmio_state(pdev))
		goto err;

	if (!gvt_init_opregion(pdev))
		goto err;

	gvt_init_resource_allocator(pdev);

	if (!gvt_setup_mmio_emulation_state(pdev))
		goto err;

	if (!gvt_irq_init(pdev))
		goto err;

	if (!gvt_init_gtt(pdev))
		goto err;

	if (!gvt_setup_control_interface(pdev))
		goto err;

	if (!gvt_init_cmd_parser(pdev))
		goto err;

	if (!init_service_thread(pdev))
		goto err;

	return true;
err:
	clean_pgt_device(pdev);
	return false;
}

static bool post_init_pgt_device(struct pgt_device *pdev)
{
	if (!gvt_init_workload_scheduler(pdev))
		return false;

	return true;
}

static void free_pgt_device(struct pgt_device *pdev)
{
	struct gvt_host *host = &gvt_host;

	mutex_lock(&host->device_idr_lock);
	idr_remove(&host->device_idr, pdev->id);
	mutex_unlock(&host->device_idr_lock);

	vfree(pdev);
}

static struct pgt_device *alloc_pgt_device(struct drm_i915_private *dev_priv)
{
	struct gvt_host *host = &gvt_host;
	struct pgt_device *pdev = NULL;

	pdev = vzalloc(sizeof(*pdev));
	if (!pdev) {
		gvt_err("fail to allocate memory for pgt device.");
		return NULL;
	}

	mutex_lock(&host->device_idr_lock);
	pdev->id = idr_alloc(&host->device_idr, pdev, 0, 0, GFP_KERNEL);
	mutex_unlock(&host->device_idr_lock);

	if (pdev->id < 0) {
		gvt_err("fail to allocate pgt device id.");
		goto err;
	}

	mutex_init(&pdev->lock);
	pdev->dev_priv = dev_priv;
	idr_init(&pdev->instance_idr);
	return pdev;
err:
	free_pgt_device(pdev);
	return NULL;
}

void gvt_destroy_pgt_device(void *private_data)
{
	struct pgt_device *pdev = (struct pgt_device *)private_data;

	clean_pgt_device(pdev);
	free_pgt_device(pdev);
}

void *gvt_create_pgt_device(struct drm_i915_private *dev_priv)
{
	struct pgt_device *pdev = NULL;
	struct gvt_host *host = &gvt_host;

	if (!host->initialized && !gvt_init_host()) {
		gvt_err("gvt_init_host fail");
		return NULL;
	}

	gvt_dbg_core("create new pgt device, i915 dev_priv: %p", dev_priv);

	pdev = alloc_pgt_device(dev_priv);
	if (!pdev) {
		gvt_err("fail to allocate memory for pgt device.");
		goto err;
	}

	gvt_dbg_core("init pgt device, id %d", pdev->id);

	if (!init_pgt_device(pdev, dev_priv)) {
		gvt_err("fail to init physical device state.");
		goto err;
	}

	gvt_dbg_core("pgt device creation done, id %d", pdev->id);

	return pdev;
err:
	if (pdev) {
		gvt_destroy_pgt_device(pdev);
		pdev = NULL;
	}
	return NULL;
}

bool gvt_post_init_pgt_device(void *private_data)
{
	struct pgt_device *pdev = (struct pgt_device *)private_data;
	struct gvt_host *host = &gvt_host;

	if (!host->initialized) {
		gvt_err("gvt_host haven't been initialized.");
		return false;
	}

	gvt_dbg_core("post init pgt device %d", pdev->id);

	if (!post_init_pgt_device(pdev)) {
		gvt_err("fail to post init physical device state.");
		return false;
	}

	return true;
}
