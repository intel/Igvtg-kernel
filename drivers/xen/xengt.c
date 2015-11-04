/*
 * Interfaces coupled to Xen
 *
 * Copyright(c) 2011-2013 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of Version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*
 * NOTE:
 * This file contains hypervisor specific interactions to
 * implement the concept of mediated pass-through framework.
 * What this file provides is actually a general abstraction
 * of in-kernel device model, which is not vgt specific.
 *
 * Now temporarily in vgt code. long-term this should be
 * in hypervisor (xen/kvm) specific directory
 */
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/kthread.h>
#include <linux/time.h>
#include <linux/freezer.h>
#include <linux/wait.h>
#include <linux/sched.h>

#include <asm/xen/hypercall.h>
#include <asm/xen/page.h>
#include <xen/xen-ops.h>
#include <xen/events.h>
#include <xen/interface/hvm/params.h>
#include <xen/interface/hvm/ioreq.h>
#include <xen/interface/hvm/hvm_op.h>
#include <xen/interface/memory.h>
#include <xen/interface/platform.h>
#include <xen/interface/vcpu.h>

#include "vgt.h"

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("XenGT mediated passthrough driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

#define MAX_HVM_VCPUS_SUPPORTED 128
struct vgt_hvm_info {
	/* iopage_vma->addr is just iopage. We need iopage_vma on VM destroy */
	shared_iopage_t *iopage;
	struct vm_struct *iopage_vma;
	int *evtchn_irq; /* the event channle irqs to handle HVM io request
				index is vcpu id */

	DECLARE_BITMAP(ioreq_pending, MAX_HVM_VCPUS_SUPPORTED);
	wait_queue_head_t io_event_wq;
	struct task_struct *emulation_thread;

	int nr_vcpu;

	ioservid_t iosrv_id;    /* io-request server id */

#define VMEM_1MB		(1ULL << 20)	/* the size of the first 1MB */
#define VMEM_BUCK_SHIFT		20
#define VMEM_BUCK_SIZE		(1ULL << VMEM_BUCK_SHIFT)
#define VMEM_BUCK_MASK		(~(VMEM_BUCK_SIZE - 1))
	uint64_t vmem_sz;
	/* for the 1st 1MB memory of HVM: each vm_struct means one 4K-page */
	struct vm_struct **vmem_vma_low_1mb;
	/* for >1MB memory of HVM: each vm_struct means 1MB */
	struct vm_struct **vmem_vma;
	/* for >1MB memory of HVM: each vm_struct means 4KB */
	struct vm_struct **vmem_vma_4k;
};

static int xen_pause_domain(int vm_id);
static int xen_shutdown_domain(int vm_id);
static void *xen_gpa_to_va(struct vgt_device *vgt, unsigned long gpa);

#define XEN_ASSERT_VM(x, vgt)						\
	do {								\
		if (!(x)) {						\
			printk("Assert at %s line %d\n",		\
				__FILE__, __LINE__);			\
			if (atomic_cmpxchg(&(vgt)->crashing, 0, 1))	\
				break;					\
			vgt_warn("Killing VM%d\n", (vgt)->vm_id);	\
			if (!xen_pause_domain((vgt->vm_id)))		\
				xen_shutdown_domain((vgt->vm_id));	\
		}							\
	} while (0)

/* Translate from VM's guest pfn to machine pfn */
static unsigned long xen_g2m_pfn(int vm_id, unsigned long g_pfn)
{
	struct xen_get_mfn_from_pfn pfn_arg;
	int rc;
	unsigned long pfn_list[1];

	pfn_list[0] = g_pfn;

	set_xen_guest_handle(pfn_arg.pfn_list, pfn_list);
	pfn_arg.nr_pfns = 1;
	pfn_arg.domid = vm_id;

	rc = HYPERVISOR_memory_op(XENMEM_get_mfn_from_pfn, &pfn_arg);
	if(rc < 0){
		printk("failed to get mfn for gpfn(0x%lx)\n, errno=%d\n", g_pfn, rc);
		return INVALID_MFN;
	}

	return pfn_list[0];
}

static int xen_get_max_gpfn(int vm_id)
{
	domid_t dom_id = vm_id;
	int max_gpfn = HYPERVISOR_memory_op(XENMEM_maximum_gpfn, &dom_id);
	BUG_ON(max_gpfn < 0);
	return max_gpfn;
}

static int xen_pause_domain(int vm_id)
{
	int rc;
	struct xen_domctl domctl;

	domctl.domain = vm_id;
	domctl.cmd = XEN_DOMCTL_pausedomain;
	domctl.interface_version = XEN_DOMCTL_INTERFACE_VERSION;

	rc = HYPERVISOR_domctl(&domctl);
	if (rc != 0)
		printk("HYPERVISOR_domctl pausedomain fail with %d!\n", rc);

	return rc;
}

static int xen_shutdown_domain(int vm_id)
{
	int rc;
	struct sched_remote_shutdown r;

	r.reason = SHUTDOWN_crash;
	r.domain_id = vm_id;
	rc = HYPERVISOR_sched_op(SCHEDOP_remote_shutdown, &r);
	if (rc != 0)
		printk("HYPERVISOR_sched_op failed: %d\n", rc);
	return rc;
}

static int xen_domain_iomem_perm(uint32_t domain_id, uint64_t first_mfn,
                               uint64_t nr_mfns, uint8_t allow_access)
{
	struct xen_domctl arg;
	int rc;

	arg.domain = domain_id;
	arg.cmd = XEN_DOMCTL_iomem_permission;
	arg.interface_version = XEN_DOMCTL_INTERFACE_VERSION;
	arg.u.iomem_perm.first_mfn = first_mfn;
	arg.u.iomem_perm.nr_mfns = nr_mfns;
	arg.u.iomem_perm.allow_access = allow_access;
	rc = HYPERVISOR_domctl(&arg);

	return rc;
}

static int xen_hvm_memory_mapping(int vm_id, uint64_t first_gfn, uint64_t first_mfn,
				  uint32_t nr_mfns, uint32_t add_mapping)
{
	struct xen_domctl arg;
	int rc;

	if (add_mapping) {
		rc = xen_domain_iomem_perm(vm_id, first_mfn, nr_mfns, 1);
	        if (rc < 0) {
			printk(KERN_ERR "xen_domain_iomem_perm failed: %d\n", rc);
	        	return rc;
		}
	}

	arg.domain = vm_id;
	arg.cmd = XEN_DOMCTL_memory_mapping;
	arg.interface_version = XEN_DOMCTL_INTERFACE_VERSION;
	arg.u.memory_mapping.first_gfn = first_gfn;
	arg.u.memory_mapping.first_mfn = first_mfn;
	arg.u.memory_mapping.nr_mfns = nr_mfns;
	arg.u.memory_mapping.add_mapping = add_mapping;

	rc = HYPERVISOR_domctl(&arg);
	if (rc < 0) {
		printk(KERN_ERR "HYPERVISOR_domctl failed: %d\n", rc);
		return rc;
	}

	if (!add_mapping) {
		rc = xen_domain_iomem_perm(vm_id, first_mfn, nr_mfns, 0);
	        if (rc < 0) {
			printk(KERN_ERR "xen_domain_iomem_perm failed: %d\n", rc);
			return rc;
		}
	}

	return rc;
}

static int xen_map_mfn_to_gpfn(int vm_id, unsigned long gpfn,
	unsigned long mfn, int nr, int map, enum map_type type)
{
	int rc;
	rc = xen_hvm_memory_mapping(vm_id, gpfn, mfn, nr,
			map ? DPCI_ADD_MAPPING : DPCI_REMOVE_MAPPING);
	if (rc != 0)
		printk("xen_hvm_memory_mapping failed: %d\n", rc);
	return rc;
}

static int xen_get_nr_vcpu(int vm_id)
{
	struct xen_domctl arg;
	int rc;

	arg.domain = vm_id;
	arg.cmd = XEN_DOMCTL_getdomaininfo;
	arg.interface_version = XEN_DOMCTL_INTERFACE_VERSION;

	rc = HYPERVISOR_domctl(&arg);
	if (rc<0){
		printk(KERN_ERR "HYPERVISOR_domctl fail ret=%d\n",rc);
		/* assume it is UP */
		return 1;
	}

	return arg.u.getdomaininfo.max_vcpu_id + 1;
}

static int hvm_create_iorequest_server(struct vgt_device *vgt)
{
	struct vgt_hvm_info *info = vgt->hvm_info;
	struct xen_hvm_create_ioreq_server arg;
	int r;

	arg.domid = vgt->vm_id;
	arg.handle_bufioreq = 0;
	r = HYPERVISOR_hvm_op(HVMOP_create_ioreq_server, &arg);
	if (r < 0) {
		printk(KERN_ERR "Cannot create io-requset server: %d!\n", r);
		return r;
	}
	info->iosrv_id = arg.id;

	return r;
}

static int hvm_toggle_iorequest_server(struct vgt_device *vgt, bool enable)
{
	struct vgt_hvm_info *info = vgt->hvm_info;
	struct xen_hvm_set_ioreq_server_state arg;
	int r;

	arg.domid = vgt->vm_id;
	arg.id = info->iosrv_id;
	arg.enabled = enable;
	r = HYPERVISOR_hvm_op(HVMOP_set_ioreq_server_state, &arg);
	if (r < 0) {
		printk(KERN_ERR "Cannot %s io-request server: %d!\n",
			enable ? "enable" : "disbale",  r);
		return r;
	}

       return r;
}

static int hvm_get_ioreq_pfn(struct vgt_device *vgt, uint64_t *value)
{
	struct vgt_hvm_info *info = vgt->hvm_info;
	struct xen_hvm_get_ioreq_server_info arg;
	int r;

	arg.domid = vgt->vm_id;
	arg.id = info->iosrv_id;
	r = HYPERVISOR_hvm_op(HVMOP_get_ioreq_server_info, &arg);
	if (r < 0) {
		printk(KERN_ERR "Cannot get ioreq pfn: %d!\n", r);
		return r;
	}
	*value = arg.ioreq_pfn;
	return r;
}

static int hvm_destroy_iorequest_server(struct vgt_device *vgt)
{
	struct vgt_hvm_info *info = vgt->hvm_info;
	struct xen_hvm_destroy_ioreq_server arg;
	int r;

	arg.domid = vgt->vm_id;
	arg.id = info->iosrv_id;
	r = HYPERVISOR_hvm_op(HVMOP_destroy_ioreq_server, &arg);
	if (r < 0) {
		printk(KERN_ERR "Cannot destroy io-request server(%d): %d!\n",
			info->iosrv_id, r);
		return r;
	}
	info->iosrv_id = 0;

	return r;
}

static int hvm_map_io_range_to_ioreq_server(struct vgt_device *vgt,
	int is_mmio, uint64_t start, uint64_t end, int map)
{
	struct vgt_hvm_info *info = vgt->hvm_info;
	xen_hvm_io_range_t arg;
	int rc;

	arg.domid = vgt->vm_id;
	arg.id = info->iosrv_id;
	arg.type = is_mmio ? HVMOP_IO_RANGE_MEMORY : HVMOP_IO_RANGE_PORT;
	arg.start = start;
	arg.end = end;

	if (map)
		rc = HYPERVISOR_hvm_op(HVMOP_map_io_range_to_ioreq_server, &arg);
	else
		rc = HYPERVISOR_hvm_op(HVMOP_unmap_io_range_from_ioreq_server, &arg);

	return rc;
}

static int hvm_map_pcidev_to_ioreq_server(struct vgt_device *vgt, uint64_t sbdf)
{
	struct vgt_hvm_info *info = vgt->hvm_info;
	xen_hvm_io_range_t arg;
	int rc;

	arg.domid = vgt->vm_id;
	arg.id = info->iosrv_id;
	arg.type = HVMOP_IO_RANGE_PCI;
	arg.start = arg.end = sbdf;
	rc = HYPERVISOR_hvm_op(HVMOP_map_io_range_to_ioreq_server, &arg);
	if (rc < 0) {
		printk(KERN_ERR "Cannot map pci_dev to ioreq_server: %d!\n", rc);
		return rc;
	}

	return rc;
}

static int hvm_set_mem_type(struct vgt_device *vgt,
	uint16_t mem_type, uint64_t first_pfn, uint64_t nr)
{
	xen_hvm_set_mem_type_t args;
	int rc;

	args.domid = vgt->vm_id;
	args.hvmmem_type = mem_type;
	args.first_pfn = first_pfn;
	args.nr = 1;
	rc = HYPERVISOR_hvm_op(HVMOP_set_mem_type, &args);

	return rc;
}

static int hvm_wp_page_to_ioreq_server(struct vgt_device *vgt, unsigned long page, int set)
{
	int rc = 0;
	uint64_t start, end;
	uint16_t mem_type;

	start = page << PAGE_SHIFT;
	end = ((page + 1) << PAGE_SHIFT) - 1;

	rc = hvm_map_io_range_to_ioreq_server(vgt, 1, start, end, set);
	if (rc < 0) {
		printk(KERN_ERR "Failed to %s page 0x%lx to ioreq_server: %d!\n",
			set ? "map":"unmap", page , rc);
		return rc;
	}

	mem_type = set ? HVMMEM_mmio_write_dm : HVMMEM_ram_rw;
	rc = hvm_set_mem_type(vgt, mem_type, page, 1);
	if (rc < 0) {
		printk(KERN_ERR "Failed to set mem type of page 0x%lx to %s!\n", page,
			set ? "HVMMEM_mmio_write_dm":"HVMMEM_ram_rw");
		return rc;
	}
	return rc;
}

static int xen_set_trap_area(struct vgt_device *vgt, uint64_t start, uint64_t end, bool map)
{
	if (!vgt_pci_mmio_is_enabled(vgt))
		return 0;

	return hvm_map_io_range_to_ioreq_server(vgt, 1, start, end, map);
}

static struct vm_struct *xen_map_iopage(struct vgt_device *vgt)
{
	uint64_t ioreq_pfn;
	int rc;

	rc = hvm_create_iorequest_server(vgt);
	if (rc < 0)
		return NULL;
	rc = hvm_get_ioreq_pfn(vgt, &ioreq_pfn);
	if (rc < 0) {
		hvm_destroy_iorequest_server(vgt);
		return NULL;
	}

	return xen_remap_domain_mfn_range_in_kernel(ioreq_pfn, 1, vgt->vm_id);
}

static bool xen_set_guest_page_writeprotection(struct vgt_device *vgt,
		guest_page_t *guest_page)
{
	int r;

	if (guest_page->writeprotection)
		return true;

	r = hvm_wp_page_to_ioreq_server(vgt, guest_page->gfn, 1);
	if (r) {
		vgt_err("fail to set write protection.\n");
		return false;
	}

	guest_page->writeprotection = true;

	atomic_inc(&vgt->gtt.n_write_protected_guest_page);

	return true;
}

static bool xen_clear_guest_page_writeprotection(struct vgt_device *vgt,
		guest_page_t *guest_page)
{
	int r;

	if (!guest_page->writeprotection)
		return true;

	r = hvm_wp_page_to_ioreq_server(vgt, guest_page->gfn, 0);
	if (r) {
		vgt_err("fail to clear write protection.\n");
		return false;
	}

	guest_page->writeprotection = false;

	atomic_dec(&vgt->gtt.n_write_protected_guest_page);

	return true;
}

static int xen_check_host(void)
{
	return xen_initial_domain();
}

static int xen_virt_to_mfn(void *addr)
{
	return virt_to_mfn(addr);
}

static void *xen_mfn_to_virt(int mfn)
{
	return mfn_to_virt(mfn);
}

static int xen_inject_msi(int vm_id, u32 addr_lo, u16 data)
{
	struct xen_hvm_inject_msi info = {
		.domid	= vm_id,
		.addr	= addr_lo, /* only low addr used */
		.data	= data,
	};

	return HYPERVISOR_hvm_op(HVMOP_inject_msi, &info);
}

static int vgt_hvm_vmem_init(struct vgt_device *vgt)
{
	unsigned long i, j, gpfn, count;
	unsigned long nr_low_1mb_bkt, nr_high_bkt, nr_high_4k_bkt;
	struct vgt_hvm_info *info = vgt->hvm_info;

	if (!vgt->vm_id)
		return 0;

	ASSERT(info->vmem_vma == NULL && info->vmem_vma_low_1mb == NULL);

	info->vmem_sz = xen_get_max_gpfn(vgt->vm_id) + 1;
	info->vmem_sz <<= PAGE_SHIFT;

	/* warn on non-1MB-aligned memory layout of HVM */
	if (info->vmem_sz & ~VMEM_BUCK_MASK)
		vgt_warn("VM%d: vmem_sz=0x%llx!\n", vgt->vm_id, info->vmem_sz);

	nr_low_1mb_bkt = VMEM_1MB >> PAGE_SHIFT;
	nr_high_bkt = (info->vmem_sz >> VMEM_BUCK_SHIFT);
	nr_high_4k_bkt = (info->vmem_sz >> PAGE_SHIFT);

	info->vmem_vma_low_1mb =
		vzalloc(sizeof(*info->vmem_vma) * nr_low_1mb_bkt);
	info->vmem_vma =
		vzalloc(sizeof(*info->vmem_vma) * nr_high_bkt);
	info->vmem_vma_4k =
		vzalloc(sizeof(*info->vmem_vma) * nr_high_4k_bkt);

	if (info->vmem_vma_low_1mb == NULL || info->vmem_vma == NULL ||
		info->vmem_vma_4k == NULL) {
		vgt_err("Insufficient memory for vmem_vma, vmem_sz=0x%llx\n",
				info->vmem_sz );
		goto err;
	}

	/* map the low 1MB memory */
	for (i = 0; i < nr_low_1mb_bkt; i++) {
		info->vmem_vma_low_1mb[i] =
			xen_remap_domain_mfn_range_in_kernel(i, 1, vgt->vm_id);

		if (info->vmem_vma_low_1mb[i] != NULL)
			continue;

		/* Don't warn on [0xa0000, 0x100000): a known non-RAM hole */
		if (i < (0xa0000 >> PAGE_SHIFT))
			printk(KERN_ERR "vGT: VM%d: can't map GPFN %ld!\n",
				vgt->vm_id, i);
	}

	printk("start vmem_map\n");
	count = 0;
	/* map the >1MB memory */
	for (i = 1; i < nr_high_bkt; i++) {
		gpfn = i << (VMEM_BUCK_SHIFT - PAGE_SHIFT);
		info->vmem_vma[i] = xen_remap_domain_mfn_range_in_kernel(
				gpfn, VMEM_BUCK_SIZE >> PAGE_SHIFT, vgt->vm_id);

		if (info->vmem_vma[i] != NULL)
			continue;


		/* for <4G GPFNs: skip the hole after low_mem_max_gpfn */
		if (gpfn < (1 << (32 - PAGE_SHIFT)) &&
			vgt->low_mem_max_gpfn != 0 &&
			gpfn > vgt->low_mem_max_gpfn)
			continue;

		for (j = gpfn;
		     j < ((i + 1) << (VMEM_BUCK_SHIFT - PAGE_SHIFT));
		     j++) {
			info->vmem_vma_4k[j] = xen_remap_domain_mfn_range_in_kernel(j, 1, vgt->vm_id);

			if (info->vmem_vma_4k[j]) {
				count++;
				printk(KERN_ERR "map 4k gpa (%lx)\n", j << PAGE_SHIFT);
			}
		}

		/* To reduce the number of err messages(some of them, due to
		 * the MMIO hole, are spurious and harmless) we only print a
		 * message if it's at every 64MB boundary or >4GB memory.
		 */
		if ((i % 64 == 0) || (i >= (1ULL << (32 - VMEM_BUCK_SHIFT))))
			printk(KERN_ERR "vGT: VM%d: can't map %ldKB\n",
				vgt->vm_id, i);
	}
	printk("end vmem_map (%ld 4k mappings)\n", count);

	return 0;
err:
	vfree(info->vmem_vma);
	vfree(info->vmem_vma_low_1mb);
	vfree(info->vmem_vma_4k);
	info->vmem_vma = info->vmem_vma_low_1mb = info->vmem_vma_4k = NULL;
	return -ENOMEM;
}

static void vgt_vmem_destroy(struct vgt_device *vgt)
{
	int i, j;
	unsigned long nr_low_1mb_bkt, nr_high_bkt, nr_high_bkt_4k;
	struct vgt_hvm_info *info = vgt->hvm_info;

	if (vgt->vm_id == 0)
		return;

	/*
	 * Maybe the VM hasn't accessed GEN MMIO(e.g., still in the legacy VGA
	 * mode), so no mapping is created yet.
	 */
	if (info->vmem_vma == NULL && info->vmem_vma_low_1mb == NULL)
		return;

	ASSERT(info->vmem_vma != NULL && info->vmem_vma_low_1mb != NULL);

	nr_low_1mb_bkt = VMEM_1MB >> PAGE_SHIFT;
	nr_high_bkt = (info->vmem_sz >> VMEM_BUCK_SHIFT);
	nr_high_bkt_4k = (info->vmem_sz >> PAGE_SHIFT);

	for (i = 0; i < nr_low_1mb_bkt; i++) {
		if (info->vmem_vma_low_1mb[i] == NULL)
			continue;
		xen_unmap_domain_mfn_range_in_kernel(info->vmem_vma_low_1mb[i],
				1, vgt->vm_id);
	}

	for (i = 1; i < nr_high_bkt; i++) {
		if (info->vmem_vma[i] == NULL) {
			for (j = (i << (VMEM_BUCK_SHIFT - PAGE_SHIFT));
			     j < ((i + 1) << (VMEM_BUCK_SHIFT - PAGE_SHIFT));
			     j++) {
				if (info->vmem_vma_4k[j] == NULL)
					continue;
				xen_unmap_domain_mfn_range_in_kernel(
					info->vmem_vma_4k[j], 1, vgt->vm_id);
			}
			continue;
		}
		xen_unmap_domain_mfn_range_in_kernel(
			info->vmem_vma[i], VMEM_BUCK_SIZE >> PAGE_SHIFT,
			vgt->vm_id);
	}

	vfree(info->vmem_vma);
	vfree(info->vmem_vma_low_1mb);
	vfree(info->vmem_vma_4k);
}

static int _hvm_mmio_emulation(struct vgt_device *vgt, struct ioreq *req)
{
	int i, sign;
	void *gva;
	unsigned long gpa;
	uint64_t base = vgt_mmio_bar_base(vgt);
	uint64_t tmp;
	int pvinfo_page;
	struct vgt_hvm_info *info = vgt->hvm_info;

	if (info->vmem_vma == NULL) {
		tmp = vgt_ops->pa_to_mmio_offset(vgt, req->addr);
		pvinfo_page = (tmp >= VGT_PVINFO_PAGE
				&& tmp < (VGT_PVINFO_PAGE + VGT_PVINFO_SIZE));
		/*
		 * hvmloader will read PVINFO to identify if HVM is in VGT
		 * or VTD. So we don't trigger HVM mapping logic here.
		 */
		if (!pvinfo_page && vgt_hvm_vmem_init(vgt) < 0) {
			vgt_err("can not map the memory of VM%d!!!\n", vgt->vm_id);
			XEN_ASSERT_VM(info->vmem_vma != NULL, vgt);
			return -EINVAL;
		}
	}

	sign = req->df ? -1 : 1;

	if (req->dir == IOREQ_READ) {
		/* MMIO READ */
		if (!req->data_is_ptr) {
			if (req->count != 1)
				goto err_ioreq_count;

			//vgt_dbg(VGT_DBG_GENERIC,"HVM_MMIO_read: target register (%lx).\n",
			//	(unsigned long)req->addr);
			if (!vgt_ops->emulate_read(vgt, req->addr, &req->data, req->size))
				return -EINVAL;
		}
		else {
			if ((req->addr + sign * req->count * req->size < base)
			   || (req->addr + sign * req->count * req->size >=
				base + vgt->state.bar_size[0]))
				goto err_ioreq_range;
			//vgt_dbg(VGT_DBG_GENERIC,"HVM_MMIO_read: rep %d target memory %lx, slow!\n",
			//	req->count, (unsigned long)req->addr);

			for (i = 0; i < req->count; i++) {
				if (!vgt_ops->emulate_read(vgt, req->addr + sign * i * req->size,
					&tmp, req->size))
					return -EINVAL;
				gpa = req->data + sign * i * req->size;
				if(!vgt->vm_id)
					gva = (char *)xen_mfn_to_virt(gpa >> PAGE_SHIFT) + offset_in_page(gpa);
				else
					gva = xen_gpa_to_va(vgt, gpa);
				if (gva) {
					if (!IS_SNB(vgt->pdev))
						memcpy(gva, &tmp, req->size);
					else {
						// On the SNB laptop, writing tmp to gva can
						//cause bug 119. So let's do the writing only on HSW for now.
						vgt_err("vGT: disable support of string copy instruction on SNB, gpa: 0x%lx\n", gpa);
					}
				} else
					vgt_err("VM %d is trying to store mmio data block to invalid gpa: 0x%lx.\n", vgt->vm_id, gpa);
			}
		}
	}
	else { /* MMIO Write */
		if (!req->data_is_ptr) {
			if (req->count != 1)
				goto err_ioreq_count;
			//vgt_dbg(VGT_DBG_GENERIC,"HVM_MMIO_write: target register (%lx).\n", (unsigned long)req->addr);
			if (!vgt_ops->emulate_write(vgt, req->addr, &req->data, req->size))
				return -EINVAL;
		}
		else {
			if ((req->addr + sign * req->count * req->size < base)
			    || (req->addr + sign * req->count * req->size >=
				base + vgt->state.bar_size[0]))
				goto err_ioreq_range;
			//vgt_dbg(VGT_DBG_GENERIC,"HVM_MMIO_write: rep %d target memory %lx, slow!\n",
			//	req->count, (unsigned long)req->addr);

			for (i = 0; i < req->count; i++) {
				gpa = req->data + sign * i * req->size;
				if(!vgt->vm_id)
					gva = (char *)xen_mfn_to_virt(gpa >> PAGE_SHIFT) + offset_in_page(gpa);
				else
					gva = xen_gpa_to_va(vgt, gpa);

				if (gva != NULL)
					memcpy(&tmp, gva, req->size);
				else {
					tmp = 0;
					printk(KERN_ERR "vGT: can not read gpa = 0x%lx!!!\n", gpa);
				}
				if (!vgt_ops->emulate_write(vgt, req->addr + sign * i * req->size, &tmp, req->size))
					return -EINVAL;
			}
		}
	}

	return 0;

err_ioreq_count:
	vgt_err("VM(%d): Unexpected %s request count(%d)\n",
		vgt->vm_id, req->dir == IOREQ_READ ? "read" : "write",
		req->count);
	return -EINVAL;

err_ioreq_range:
	vgt_err("VM(%d): Invalid %s request addr end(%016llx)\n",
		vgt->vm_id, req->dir == IOREQ_READ ? "read" : "write",
		req->addr + sign * req->count * req->size);
	return -ERANGE;
}

static bool vgt_hvm_write_cfg_space(struct vgt_device *vgt,
	uint64_t addr, unsigned int bytes, unsigned long val)
{
	/* Low 32 bit of addr is real address, high 32 bit is bdf */
	unsigned int port = addr & 0xffffffff;

	ASSERT(((bytes == 4) && ((port & 3) == 0)) ||
		((bytes == 2) && ((port & 1) == 0)) || (bytes == 1));
	vgt_ops->emulate_cfg_write(vgt, port, &val, bytes);
	return true;
}

static bool vgt_hvm_read_cfg_space(struct vgt_device *vgt,
	uint64_t addr, unsigned int bytes, unsigned long *val)
{
	unsigned long data;
	/* Low 32 bit of addr is real address, high 32 bit is bdf */
	unsigned int port = addr & 0xffffffff;

	ASSERT (((bytes == 4) && ((port & 3) == 0)) ||
		((bytes == 2) && ((port & 1) == 0)) || (bytes == 1));
	vgt_ops->emulate_cfg_read(vgt, port, &data, bytes);
	memcpy(val, &data, bytes);
	return true;
}

static int _hvm_pio_emulation(struct vgt_device *vgt, struct ioreq *ioreq)
{
	int sign;

	sign = ioreq->df ? -1 : 1;

	if (ioreq->dir == IOREQ_READ) {
		/* PIO READ */
		if (!ioreq->data_is_ptr) {
			if(!vgt_hvm_read_cfg_space(vgt,
				ioreq->addr,
				ioreq->size,
				(unsigned long*)&ioreq->data))
				return -EINVAL;
		} else {
			printk(KERN_ERR "VGT: _hvm_pio_emulation read data_ptr %lx\n",
			(long)ioreq->data);
			goto err_data_ptr;
		}
	} else {
		/* PIO WRITE */
		if (!ioreq->data_is_ptr) {
			if (!vgt_hvm_write_cfg_space(vgt,
				ioreq->addr,
				ioreq->size,
				(unsigned long)ioreq->data))
				return -EINVAL;
		} else {
			printk(KERN_ERR "VGT: _hvm_pio_emulation write data_ptr %lx\n",
			(long)ioreq->data);
			goto err_data_ptr;
		}
	}
	return 0;
err_data_ptr:
	/* The data pointer of emulation is guest physical address
	 * so far, which goes to Qemu emulation, but hard for
	 * vGT driver which doesn't know gpn_2_mfn translation.
	 * We may ask hypervisor to use mfn for vGT driver.
	 * We mark it as unsupported in case guest really it.
	 */
	vgt_err("VM(%d): Unsupported %s data_ptr(%lx)\n",
		vgt->vm_id, ioreq->dir == IOREQ_READ ? "read" : "write",
		(long)ioreq->data);
	return -EINVAL;
}

static int vgt_hvm_do_ioreq(struct vgt_device *vgt, struct ioreq *ioreq)
{
	struct pgt_device *pdev = vgt->pdev;
	uint64_t bdf = PCI_BDF2(pdev->pbus->number, pdev->devfn);

	int rc = 0;

	BUG_ON(ioreq->state != STATE_IOREQ_INPROCESS);

	switch (ioreq->type) {
	case IOREQ_TYPE_PCI_CONFIG:
		/* High 32 bit of ioreq->addr is bdf */
		if ((ioreq->addr >> 32) != bdf) {
			printk(KERN_ERR "vGT: Unexpected PCI Dev %lx emulation\n",
			(unsigned long) (ioreq->addr>>32));
			rc = -EINVAL;
		} else
			rc = _hvm_pio_emulation(vgt, ioreq);
		break;
	case IOREQ_TYPE_COPY:   /* MMIO */
		rc = _hvm_mmio_emulation(vgt, ioreq);
		break;
	case IOREQ_TYPE_INVALIDATE:
	case IOREQ_TYPE_TIMEOFFSET:
		break;
	default:
		printk(KERN_ERR "vGT: Unknown ioreq type %x addr %llx size %u state %u\n", 
			ioreq->type, ioreq->addr, ioreq->size, ioreq->state);
		rc = -EINVAL;
		break;
	}

	wmb();

	return rc;
}

static struct ioreq *vgt_get_hvm_ioreq(struct vgt_device *vgt, int vcpu)
{
	struct vgt_hvm_info *info = vgt->hvm_info;
	ioreq_t *req = &(info->iopage->vcpu_ioreq[vcpu]);

	if (req->state != STATE_IOREQ_READY)
	  return NULL;

	rmb();

	req->state = STATE_IOREQ_INPROCESS;
	return req;
}

static int vgt_emulation_thread(void *priv)
{
	struct vgt_device *vgt = (struct vgt_device *)priv;
	struct vgt_hvm_info *info = vgt->hvm_info;

	int vcpu;
	int nr_vcpus = info->nr_vcpu;

	struct ioreq *ioreq;
	int irq, ret;

	vgt_info("start kthread for VM%d\n", vgt->vm_id);

	ASSERT(info->nr_vcpu <= MAX_HVM_VCPUS_SUPPORTED);

	set_freezable();
	while (1) {
		ret = wait_event_freezable(info->io_event_wq,
			kthread_should_stop() ||
			bitmap_weight(info->ioreq_pending, nr_vcpus));

		if (kthread_should_stop())
			return 0;

		if (ret)
			vgt_warn("Emulation thread(%d) waken up"
				 "by unexpected signal!\n", vgt->vm_id);

		for (vcpu = 0; vcpu < nr_vcpus; vcpu++) {
			if (!test_and_clear_bit(vcpu, info->ioreq_pending))
				continue;

			ioreq = vgt_get_hvm_ioreq(vgt, vcpu);
			if (ioreq == NULL)
				continue;

			if (vgt_hvm_do_ioreq(vgt, ioreq) ||
					!vgt_ops->expand_shadow_page_mempool(vgt->pdev)) {
				xen_pause_domain(vgt->vm_id);
				xen_shutdown_domain(vgt->vm_id);
			}

			if (vgt->force_removal) {
				wait_event(vgt->pdev->destroy_wq,
						kthread_should_stop() ||
						!vgt->force_removal);
				if (kthread_should_stop())
					return 0;
			}


			ioreq->state = STATE_IORESP_READY;

			irq = info->evtchn_irq[vcpu];
			notify_remote_via_irq(irq);
		}
	}

	BUG(); /* It's actually impossible to reach here */
	return 0;
}

static inline void vgt_raise_emulation_request(struct vgt_device *vgt,
	int vcpu)
{
	struct vgt_hvm_info *info = vgt->hvm_info;
	set_bit(vcpu, info->ioreq_pending);
	if (waitqueue_active(&info->io_event_wq))
		wake_up(&info->io_event_wq);
}

static irqreturn_t vgt_hvm_io_req_handler(int irq, void* dev)
{
	struct vgt_device *vgt;
	struct vgt_hvm_info *info;
	int vcpu;

	vgt = (struct vgt_device *)dev;
	info = vgt->hvm_info;

	for(vcpu=0; vcpu < info->nr_vcpu; vcpu++){
		if(info->evtchn_irq[vcpu] == irq)
			break;
	}
	if (vcpu == info->nr_vcpu){
		/*opps, irq is not the registered one*/
		vgt_info("Received a IOREQ w/o vcpu target\n");
		vgt_info("Possible a false request from event binding\n");
		return IRQ_NONE;
	}

	vgt_raise_emulation_request(vgt, vcpu);

	return IRQ_HANDLED;
}

static void xen_hvm_exit(struct vgt_device *vgt)
{
	struct vgt_hvm_info *info;
	int vcpu;

	info = vgt->hvm_info;

	if (info == NULL)
		return;

	if (info->emulation_thread != NULL)
		kthread_stop(info->emulation_thread);

	if (!info->nr_vcpu || info->evtchn_irq == NULL)
		goto out1;

	if (info->iosrv_id != 0)
		hvm_destroy_iorequest_server(vgt);

	for (vcpu = 0; vcpu < info->nr_vcpu; vcpu++){
		if(info->evtchn_irq[vcpu] >= 0)
			unbind_from_irqhandler(info->evtchn_irq[vcpu], vgt);
	}

	if (info->iopage_vma != NULL)
		xen_unmap_domain_mfn_range_in_kernel(info->iopage_vma, 1, vgt->vm_id);

	kfree(info->evtchn_irq);

out1:
	vgt_vmem_destroy(vgt);
	kfree(info);
}

static int xen_hvm_init(struct vgt_device *vgt)
{
	struct vgt_hvm_info *info;
	int vcpu, irq, rc = 0;
	struct task_struct *thread;
	struct pgt_device *pdev = vgt->pdev;

	info = kzalloc(sizeof(struct vgt_hvm_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;

	vgt->hvm_info = info;

	info->iopage_vma = xen_map_iopage(vgt);
	if (info->iopage_vma == NULL) {
		printk(KERN_ERR "Failed to map HVM I/O page for VM%d\n", vgt->vm_id);
		rc = -EFAULT;
		goto err;
	}
	info->iopage = info->iopage_vma->addr;

	init_waitqueue_head(&info->io_event_wq);

	info->nr_vcpu = xen_get_nr_vcpu(vgt->vm_id);
	ASSERT(info->nr_vcpu > 0);
	ASSERT(info->nr_vcpu <= MAX_HVM_VCPUS_SUPPORTED);

	info->evtchn_irq = kmalloc(info->nr_vcpu * sizeof(int), GFP_KERNEL);
	if (info->evtchn_irq == NULL){
		rc = -ENOMEM;
		goto err;
	}
	for( vcpu = 0; vcpu < info->nr_vcpu; vcpu++ )
		info->evtchn_irq[vcpu] = -1;

	rc = hvm_map_pcidev_to_ioreq_server(vgt, PCI_BDF2(pdev->pbus->number, pdev->devfn));
	if (rc < 0)
		goto err;
	rc = hvm_toggle_iorequest_server(vgt, 1);
	if (rc < 0)
		goto err;

	for (vcpu = 0; vcpu < info->nr_vcpu; vcpu++){
		irq = bind_interdomain_evtchn_to_irqhandler( vgt->vm_id,
				info->iopage->vcpu_ioreq[vcpu].vp_eport,
				vgt_hvm_io_req_handler, 0,
				"vgt", vgt );
		if ( irq < 0 ){
			rc = irq;
			printk(KERN_ERR "Failed to bind event channle for vgt HVM IO handler, rc=%d\n", rc);
			goto err;
		}
		info->evtchn_irq[vcpu] = irq;
	}

	thread = kthread_run(vgt_emulation_thread, vgt,
			"vgt_emulation:%d", vgt->vm_id);
	if(IS_ERR(thread))
		goto err;
	info->emulation_thread = thread;

	return 0;

err:
	xen_hvm_exit(vgt);
	return rc;
}

static void *xen_gpa_to_va(struct vgt_device *vgt, unsigned long gpa)
{
	unsigned long buck_index, buck_4k_index;
	struct vgt_hvm_info *info = vgt->hvm_info;

	if (!vgt->vm_id)
		return (char*)xen_mfn_to_virt(gpa>>PAGE_SHIFT) + (gpa & (PAGE_SIZE-1));
	/*
	 * At the beginning of _hvm_mmio_emulation(), we already initialize
	 * info->vmem_vma and info->vmem_vma_low_1mb.
	 */
	ASSERT(info->vmem_vma != NULL && info->vmem_vma_low_1mb != NULL);

	/* handle the low 1MB memory */
	if (gpa < VMEM_1MB) {
		buck_index = gpa >> PAGE_SHIFT;
		if (!info->vmem_vma_low_1mb[buck_index])
			return NULL;

		return (char*)(info->vmem_vma_low_1mb[buck_index]->addr) +
			(gpa & ~PAGE_MASK);

	}

	/* handle the >1MB memory */
	buck_index = gpa >> VMEM_BUCK_SHIFT;

	if (!info->vmem_vma[buck_index]) {
		buck_4k_index = gpa >> PAGE_SHIFT;
		if (!info->vmem_vma_4k[buck_4k_index]) {
			if (buck_4k_index > vgt->low_mem_max_gpfn)
				vgt_err("vGT failed to map gpa=0x%lx?\n", gpa);
			return NULL;
		}

		return (char*)(info->vmem_vma_4k[buck_4k_index]->addr) +
			(gpa & ~PAGE_MASK);
	}

	return (char*)(info->vmem_vma[buck_index]->addr) +
		(gpa & (VMEM_BUCK_SIZE -1));
}

static bool xen_read_va(struct vgt_device *vgt, void *va, void *val,
		int len, int atomic)
{
	memcpy(val, va, len);

	return true;
}

static bool xen_write_va(struct vgt_device *vgt, void *va, void *val,
		int len, int atomic)
{
	memcpy(va, val, len);
	return true;
}

static struct kernel_dm xengt_kdm = {
	.name = "xengt_kdm",
	.g2m_pfn = xen_g2m_pfn,
	.pause_domain = xen_pause_domain,
	.shutdown_domain = xen_shutdown_domain,
	.map_mfn_to_gpfn = xen_map_mfn_to_gpfn,
	.set_trap_area = xen_set_trap_area,
	.set_wp_pages = xen_set_guest_page_writeprotection,
	.unset_wp_pages = xen_clear_guest_page_writeprotection,
	.check_host = xen_check_host,
	.from_virt_to_mfn = xen_virt_to_mfn,
	.from_mfn_to_virt = xen_mfn_to_virt,
	.inject_msi = xen_inject_msi,
	.hvm_init = xen_hvm_init,
	.hvm_exit = xen_hvm_exit,
	.gpa_to_va = xen_gpa_to_va,
	.read_va = xen_read_va,
	.write_va = xen_write_va,
};
EXPORT_SYMBOL(xengt_kdm);

static int __init xengt_init(void)
{
       if (!xen_initial_domain())
               return -EINVAL;
       printk(KERN_INFO "xengt: loaded\n");
       return 0;
}

static void __exit xengt_exit(void)
{
	printk(KERN_INFO "xengt: unloaded\n");
}

module_init(xengt_init);
module_exit(xengt_exit);
