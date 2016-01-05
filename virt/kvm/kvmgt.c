/*
 * KVM implementation of mediated pass-through framework of VGT.
 *
 * Copyright(c) 2014-2015 Intel Corporation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/kvm.h>
#include <linux/kvm_host.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <asm/page.h>
#include <asm/vmx.h>
#include <kvm/iodev.h>

#include "kvmgt.h"
#include "vgt.h"


struct kvmgt_trap_info {
	u64 base_addr;
	int len;
	struct kvm_io_device iodev;
	bool set;
};

struct kvmgt_hvm_info {
	struct kvm *kvm;
	struct vgt_device *vgt;
	struct kvmgt_trap_info trap_mmio;
};

static struct kvm *kvmgt_find_by_domid(int domid)
{
	struct kvm *kvm = NULL;

	if (unlikely(domid <= 0)) {
		vgt_err("FIXME! domid=%d\n", domid);
		return NULL;
	}

	spin_lock(&kvm_lock);
	list_for_each_entry(kvm,  &vm_list, vm_list) {
		if (kvm->domid == domid) {
			spin_unlock(&kvm_lock);
			goto found;
		}
	}
	spin_unlock(&kvm_lock);
	return NULL;

found:
	return kvm;
}

static int kvmgt_vm_getdomid(void)
{
	/* 0 is reserved for host */
	static int domid = 1;

	return domid++;
}

void kvmgt_init(struct kvm *kvm)
{
	kvm->domid = kvmgt_vm_getdomid();
	kvm->vgt_enabled = false;
	kvm->vgt = NULL;
}

void kvmgt_exit(struct kvm *kvm)
{
	vgt_params_t vp;

	if (!kvm->vgt_enabled || !kvm->vgt)
		return;

	vgt_info("release vgt resrouce for KVM\n");

	vp.vm_id = -kvm->domid;
	vgt_ops->del_state_sysfs(vp);

	kvmgt_protect_table_destroy(kvm);
	kvmgt_unpin_guest(kvm);

	kvm->vgt_enabled = false;
	kvm->vgt = NULL;
}

void kvmgt_record_cf8(struct kvm_vcpu *vcpu, unsigned port, unsigned long rax)
{
	if (port == 0xcf8)
		vcpu->arch.last_cfg_addr = (u32)rax;
}

bool kvmgt_pio_is_igd_cfg(struct kvm_vcpu *vcpu)
{
	unsigned int b, d, f;
	u32 addr = vcpu->arch.last_cfg_addr;

	if (!vcpu->kvm->vgt_enabled)
		return false;

	switch (vcpu->arch.pio.port) {
	case 0xcfc ... 0xcff:
		break;
	default:
		return false;
	}

	b = (addr >> 16) & 0xff;
	d = (addr >> 11) & 0x1f;
	f = (addr >> 8) & 0x7;

	return (b == 0 && d == 2 && f == 0);
}

bool kvmgt_pio_igd_cfg(struct kvm_vcpu *vcpu)
{
	bool ret = false;

	if (vcpu->arch.pio.in) {
		ret = vgt_ops->emulate_cfg_read(vcpu->kvm->vgt,
					(vcpu->arch.last_cfg_addr & 0xfc) + (vcpu->arch.pio.port & 3),
					vcpu->arch.pio_data,
					vcpu->arch.pio.size);
	} else {
		ret = vgt_ops->emulate_cfg_write(vcpu->kvm->vgt,
					(vcpu->arch.last_cfg_addr & 0xfc) + (vcpu->arch.pio.port & 3),
					vcpu->arch.pio_data,
					vcpu->arch.pio.size);
	}

	return ret;
}

static unsigned long __kvmgt_gfn_to_pfn(struct kvm *kvm, gfn_t gfn)
{
	pfn_t pfn;
	struct kvm_memory_slot *slot;
	bool unlock = false;

	pfn = kvmgt_gfn_to_pfn_by_rmap(kvm, gfn);
	if (!is_error_pfn(pfn))
		return pfn;

	/*
	 * gfn_to_xxx() requires kvm->srcu lock, as per Documentation/virtual/kvm/locking.txt.
	 * However, according to __kvm_memslots(), any of following conditions does the trick:
	 * 	- kvm->slots_lock is held
	 * 	- kvm->srcu is read-held
	 * 	- general rcu is read-held
	 * "lockdep_is_held" is difficult to use in non-RCU code, so ignore it here.
	 * We pick the sleepless one in case that none of them is held.
	 */
	if (!srcu_read_lock_held(&kvm->srcu) && !rcu_read_lock_held()) {
		rcu_read_lock();
		unlock = true;
	}

	slot = gfn_to_memslot(kvm, gfn);
	if (!slot) {
		vgt_err("VM%d, gfn:0x%llx: slot is NULL!\n", kvm->domid, gfn);
		goto err;
	}

	BUG_ON(slot->flags & KVM_MEMSLOT_INVALID);

	if (!slot->pfn_list) {
		vgt_err("VM%d, slot:%hd: pfn_list is NULL!\n", kvm->domid, slot->id);
		goto err;
	}

	pfn = slot->pfn_list[gfn - slot->base_gfn];

err:
	if (unlock)
		rcu_read_unlock();
	return pfn;
}

static unsigned long kvmgt_gfn_to_pfn(int vm_id, unsigned long gfn)
{
	struct kvm *kvm;

	if (!vm_id)
		return gfn;

	kvm = kvmgt_find_by_domid(vm_id);
	if (!kvm) {
		vgt_err("VM%d: cannot find kvm\n", vm_id);
		return INVALID_MFN;
	}

	return __kvmgt_gfn_to_pfn(kvm, gfn);
}

static int kvmgt_pause_domain(int vm_id)
{
	/*TODO*/
	return 0;
}

static int kvmgt_shutdown_domain(int vm_id)
{
	/*TODO*/
	return 0;
}

static int kvmgt_guest_mmio_in_range(struct kvmgt_trap_info *info, gpa_t addr)
{
	return ((addr >= info->base_addr) &&
		(addr < info->base_addr + info->len));
}

static int kvmgt_guest_mmio_read(struct kvm_vcpu *vcpu, struct kvm_io_device *this,
			gpa_t addr, int len, void *val)
{
	struct kvmgt_trap_info *info = container_of(this, struct kvmgt_trap_info,
				iodev);
	struct kvmgt_hvm_info *hvm = container_of(info, struct kvmgt_hvm_info,
				trap_mmio);
	struct vgt_device *vgt = hvm->vgt;
	u64 result = 0;

	if (!kvmgt_guest_mmio_in_range(info, addr))
		return -EOPNOTSUPP;

	if (!vgt_ops->emulate_read(vgt, addr, &result, len)) {
		vgt_err("vgt_emulate_read failed!\n");
		return -EFAULT;
	}

	switch (len) {
	case 8:
		*(u64 *)val = result;
		break;
	case 1:
	case 2:
	case 4:
		memcpy(val, (char *)&result, len);
		break;
	default:
		vgt_err("FIXME! len is %d\n", len);
		return -EFAULT;
	}

	return 0;
}

static int kvmgt_guest_mmio_write(struct kvm_vcpu *vcpu, struct kvm_io_device *this,
			gpa_t addr, int len, const void *val)
{
	struct kvmgt_trap_info *info = container_of(this, struct kvmgt_trap_info,
				iodev);
	struct kvmgt_hvm_info *hvm = container_of(info, struct kvmgt_hvm_info,
				trap_mmio);
	struct vgt_device *vgt = hvm->vgt;

	if (!kvmgt_guest_mmio_in_range(info, addr))
		return -EOPNOTSUPP;

	if (!vgt_ops->emulate_write(vgt, addr, (void *)val, len)) {
		vgt_err("vgt_emulate_write failed\n");
		return 0;
	}

	return 0;
}

const struct kvm_io_device_ops trap_mmio_ops = {
	.read	= kvmgt_guest_mmio_read,
	.write	= kvmgt_guest_mmio_write,
};

static int __kvmgt_set_trap_area(struct vgt_device *vgt, uint64_t start,
			uint64_t end)
{
	int r;
	struct kvm *kvm;
	struct kvmgt_hvm_info *info = vgt->hvm_info;

	if (info->trap_mmio.set)
		return 0;

	kvm = kvmgt_find_by_domid(vgt->vm_id);
	if (kvm == NULL) {
		vgt_err("cannot find kvm for VM%d\n", vgt->vm_id);
		return 0;
	}

	info->trap_mmio.base_addr = start;
	info->trap_mmio.len = end - start + 1;

	kvm_iodevice_init(&info->trap_mmio.iodev, &trap_mmio_ops);
	mutex_lock(&kvm->slots_lock);
	r = kvm_io_bus_register_dev(kvm, KVM_MMIO_BUS,
				info->trap_mmio.base_addr,
				info->trap_mmio.len, &info->trap_mmio.iodev);
	mutex_unlock(&kvm->slots_lock);
	if (r < 0) {
		vgt_err("kvm_io_bus_register_dev failed: %d\n", r);
		return r;
	}

	vgt_info("VM%d: registered iodev: 0x%llx - 0x%llx\n", vgt->vm_id, start, end);
	info->trap_mmio.set = true;

	return r;
}

static int __kvmgt_unset_trap_area(struct vgt_device *vgt)
{
	int r;
	struct kvmgt_hvm_info *info = vgt->hvm_info;
	struct kvm *kvm = info->kvm;

	if (!info->trap_mmio.set)
		return 0;

	mutex_lock(&kvm->slots_lock);
	r = kvm_io_bus_unregister_dev(kvm, KVM_MMIO_BUS,
			&info->trap_mmio.iodev);
	mutex_unlock(&kvm->slots_lock);
	if (r < 0) {
		vgt_err("kvm_io_bus_unregister_dev failed: %d\n", r);
		return r;
	}

	info->trap_mmio.set = false;

	return r;
}

static int kvmgt_set_trap_area(struct vgt_device *vgt, uint64_t start,
			uint64_t end, bool map)
{
	if (!vgt_pci_mmio_is_enabled(vgt))
		return 0;

	return map ? __kvmgt_set_trap_area(vgt, start, end) :
		__kvmgt_unset_trap_area(vgt);
}

static bool kvmgt_set_guest_page_writeprotection(struct vgt_device *vgt,
			guest_page_t *guest_page)
{
	struct kvmgt_hvm_info *info = vgt->hvm_info;
	struct kvm *kvm = info->kvm;
	gfn_t gfn = guest_page->gfn;

	spin_lock(&kvm->mmu_lock);

	if (guest_page->writeprotection) {
		spin_unlock(&kvm->mmu_lock);
		return true;
	}

	if (kvmgt_write_protect(kvm, gfn, true))
		kvm_flush_remote_tlbs(kvm);
	kvmgt_protect_table_add(kvm, gfn);

	guest_page->writeprotection = true;
	atomic_inc(&vgt->gtt.n_write_protected_guest_page);

	spin_unlock(&kvm->mmu_lock);

	return true;
}

static bool kvmgt_clear_guest_page_writeprotection(struct vgt_device *vgt,
			guest_page_t *guest_page)
{
	struct kvmgt_hvm_info *info = vgt->hvm_info;
	struct kvm *kvm = info->kvm;
	gfn_t gfn = guest_page->gfn;

	spin_lock(&kvm->mmu_lock);

	if (!guest_page->writeprotection) {
		spin_unlock(&kvm->mmu_lock);
		return true;
	}

	if (kvmgt_write_protect(kvm, gfn, false))
		kvm_flush_remote_tlbs(kvm);
	kvmgt_protect_table_del(kvm, gfn);

	guest_page->writeprotection = false;
	atomic_dec(&vgt->gtt.n_write_protected_guest_page);

	spin_unlock(&kvm->mmu_lock);

	return true;
}

static int kvmgt_check_guest(void)
{
	unsigned int eax, ebx, ecx, edx;
	char s[12];
	unsigned int *i;

	/* KVM_CPUID_SIGNATURE */
	eax = 0x40000000;
	ebx = ecx = edx = 0;

	asm volatile ("cpuid"
		      : "+a"(eax), "=b"(ebx), "=c"(ecx), "=d"(edx)
		      :
		      : "cc", "memory");
	i = (unsigned int *)s;
	i[0] = ebx;
	i[1] = ecx;
	i[2] = edx;

	return !strncmp(s, "KVMKVMKVM", strlen("KVMKVMKVM"));
}

/* NOTE:
 * It's actually impossible to check if we are running in KVM host,
 * since the "KVM host" is simply native. So we only dectect guest here.
 */
static int kvmgt_check_host(void)
{
	return !kvmgt_check_guest();
}

static int kvmgt_virt_to_pfn(void *addr)
{
	return PFN_DOWN(__pa(addr));
}

static void *kvmgt_pfn_to_virt(int pfn)
{
	return pfn_to_kaddr((unsigned long)pfn);
}

static int kvmgt_hvm_init(struct vgt_device *vgt)
{
	struct kvm *kvm;
	struct kvmgt_hvm_info *info;

	kvm = kvmgt_find_by_domid(vgt->vm_id);
	if (kvm == NULL) {
		vgt_err("cannot find kvm for VM%d\n", vgt->vm_id);
		return -EFAULT;
	}

	kvm->vgt_enabled = true;
	kvm->vgt = vgt;

	info = kzalloc(sizeof(struct kvmgt_hvm_info), GFP_KERNEL);
	if (!info) {
		vgt_err("cannot alloc hvm info\n");
		return -ENOMEM;
	}

	vgt->hvm_info = info;
	info->vgt = vgt;
	info->kvm = kvm;

	kvmgt_protect_table_init(kvm);
	kvmgt_pin_guest(kvm);

	return 0;
}

static void *kvmgt_gpa_to_hva(struct vgt_device *vgt, unsigned long gpa)
{
	struct kvmgt_hvm_info *info = vgt->hvm_info;
	struct kvm *kvm = info->kvm;
	pfn_t pfn;

	BUG_ON(!vgt->vm_id);

	pfn = __kvmgt_gfn_to_pfn(kvm, gpa_to_gfn(gpa));
	if (is_error_pfn(pfn))
		return NULL;

	return (char *)pfn_to_kaddr(pfn) + offset_in_page(gpa);
}

static int kvmgt_inject_msi(int vm_id, u32 addr_lo, u16 data)
{
	struct kvm_msi info = {
		.address_lo = addr_lo,
		.address_hi = 0,
		.data = data,
		.flags = 0,
	};
	struct kvm *kvm = kvmgt_find_by_domid(vm_id);
	if (kvm == NULL)
		return -ENOENT;

	memset(info.pad, 0, sizeof(info.pad));
	kvm_send_userspace_msi(kvm, &info);

	return 0;
}

static void kvmgt_hvm_exit(struct vgt_device *vgt)
{
	kfree(vgt->hvm_info);
}

static bool kvmgt_read_hva(struct vgt_device *vgt, void *hva,
			void *data, int len, int atomic)
{
	memcpy(data, hva, len);
	return true;
}

static bool kvmgt_write_hva(struct vgt_device *vgt, void *hva, void *data,
			int len, int atomic)
{
	memcpy(hva, data, len);
	return true;
}

static bool kvmgt_add_apt_slot(struct vgt_device *vgt, pfn_t p1, gfn_t g1,
			int nr_mfns, u64 hva)
{
	struct kvm_userspace_memory_region kvm_userspace_mem;
	struct kvmgt_hvm_info *info = vgt->hvm_info;
	int r = 0;
	struct kvm *kvm = info->kvm;

	vgt_info("vgt-%d, p1: 0x%lx, g1: 0x%lx, nr_mfns: %d, hva: 0x%lx\n",
				vgt->vm_id, (unsigned long)p1, (unsigned long)g1,
				nr_mfns, (unsigned long)hva);

	kvm_userspace_mem.slot = VGT_APERTURE_PRIVATE_MEMSLOT;
	kvm_userspace_mem.flags = 0;
	kvm_userspace_mem.guest_phys_addr = g1 << PAGE_SHIFT;
	kvm_userspace_mem.memory_size = nr_mfns * PAGE_SIZE;

	kvm->aperture_hpa = p1 << PAGE_SHIFT;

	mutex_lock(&kvm->slots_lock);
	r = __kvm_set_memory_region(kvm, &kvm_userspace_mem);
	if (r) {
		vgt_err("__kvm_set_memory_region failed: %d\n", r);
		mutex_unlock(&kvm->slots_lock);
		return false;
	}
	mutex_unlock(&kvm->slots_lock);

	return true;
}

static bool kvmgt_opregion_init(struct vgt_device *vgt)
{
	int rc;
	int i;
	struct kvmgt_hvm_info *info = vgt->hvm_info;
	struct kvm *kvm = info->kvm;
	unsigned long addr;

	if (unlikely(vgt->state.opregion_va))
		return true;

	addr = gfn_to_hva(kvm, gpa_to_gfn(kvm->opregion_gpa));
	if (kvm_is_error_hva(addr)) {
		vgt_err("failed to find hva for gpa:0x%x\n", kvm->opregion_gpa);
		return false;
	}

	down_read(&(kvm->mm->mmap_sem));
	rc = get_user_pages(NULL, kvm->mm, addr, VGT_OPREGION_PAGES, 1, 1, vgt->state.opregion_pages, NULL);
	up_read(&kvm->mm->mmap_sem);
	if (rc != VGT_OPREGION_PAGES) {
		vgt_err("get_user_pages failed: %d\n", rc);
		return false;
	}

	vgt->state.opregion_va = vmap(vgt->state.opregion_pages, VGT_OPREGION_PAGES, 0, PAGE_KERNEL);
	if (vgt->state.opregion_va == NULL) {
		vgt_err("VM%d: failed to allocate kernel space for opregion\n", vgt->vm_id);
		for (i = 0; i < VGT_OPREGION_PAGES; i++)
			put_page(vgt->state.opregion_pages[i]);
		return false;
	}

	memcpy_fromio(vgt->state.opregion_va, vgt->pdev->opregion_va, VGT_OPREGION_SIZE);
	memcpy(&vgt->state.cfg_space[VGT_REG_CFG_OPREGION], &kvm->opregion_gpa, sizeof(kvm->opregion_gpa));

	vgt_info("opregion initialized\n");
	return true;
}

static int kvmgt_map_mfn_to_gpfn(int vm_id, unsigned long gpfn,
			unsigned long mfn, int nr, int map, enum map_type type)
{
	struct kvm *kvm = NULL;
	struct vgt_device *vgt = NULL;
	int r = 0;

	kvm = kvmgt_find_by_domid(vm_id);
	if (kvm == NULL) {
		vgt_err("cannot find kvm for VM%d\n", vm_id);
		return -EFAULT;
	}

	if (!map)
		return r;

	vgt = kvm->vgt;
	switch (type) {
	case VGT_MAP_APERTURE:
		if (kvm->aperture_hpa == 0) {
			if (kvmgt_add_apt_slot(vgt, mfn, gpfn, nr,
							(u64)vgt_aperture_vbase(vgt))) {
				r = 0;
				vgt->state.bar_mapped[1] = 1;
			} else
				r = -EFAULT;
		}
		break;
	case VGT_MAP_OPREGION:
		r = kvmgt_opregion_init(vgt);
		break;
	default:
		vgt_err("type:%d not supported!\n", type);
		r = -EOPNOTSUPP;
	}

	return r;
}

struct kernel_dm kvmgt_kdm = {
	.name = "kvmgt_kdm",
	.g2m_pfn = kvmgt_gfn_to_pfn,
	.pause_domain = kvmgt_pause_domain,
	.shutdown_domain = kvmgt_shutdown_domain,
	.map_mfn_to_gpfn = kvmgt_map_mfn_to_gpfn,
	.set_trap_area = kvmgt_set_trap_area,
	.set_wp_pages = kvmgt_set_guest_page_writeprotection,
	.unset_wp_pages = kvmgt_clear_guest_page_writeprotection,
	.check_host = kvmgt_check_host,
	.from_virt_to_mfn = kvmgt_virt_to_pfn,
	.from_mfn_to_virt = kvmgt_pfn_to_virt,
	.inject_msi = kvmgt_inject_msi,
	.hvm_init = kvmgt_hvm_init,
	.hvm_exit = kvmgt_hvm_exit,
	.gpa_to_va = kvmgt_gpa_to_hva,
	.read_va = kvmgt_read_hva,
	.write_va = kvmgt_write_hva,
};
EXPORT_SYMBOL(kvmgt_kdm);

void kvmgt_protect_table_init(struct kvm *kvm)
{
	if (kvm->vgt_enabled)
		hash_init(kvm->ptable);
}

void kvmgt_protect_table_destroy(struct kvm *kvm)
{
	kvmgt_pgfn_t *p;
	struct hlist_node *tmp;
	int i;

	if (!kvm->vgt_enabled)
		return;

	hash_for_each_safe(kvm->ptable, i, tmp, p, hnode) {
		hash_del(&p->hnode);
		kfree(p);
	}
}

void kvmgt_protect_table_add(struct kvm *kvm, gfn_t gfn)
{
	kvmgt_pgfn_t *p;

	if (!kvm->vgt_enabled)
		return;

	if (kvmgt_gfn_is_write_protected(kvm, gfn))
		return;

	p = kmalloc(sizeof(kvmgt_pgfn_t), GFP_ATOMIC);
	if (!p) {
		vgt_err("kmalloc failed for 0x%llx\n", gfn);
		return;
	}

	p->gfn = gfn;
	hash_add(kvm->ptable, &p->hnode, gfn);
}

static kvmgt_pgfn_t *__kvmgt_protect_table_find(struct kvm *kvm, gfn_t gfn)
{
	kvmgt_pgfn_t *p, *res = NULL;

	hash_for_each_possible(kvm->ptable, p, hnode, gfn) {
		if (gfn == p->gfn) {
			res = p;
			break;
		}
	}

	return res;
}

void kvmgt_protect_table_del(struct kvm *kvm, gfn_t gfn)
{
	kvmgt_pgfn_t *p;

	if (!kvm->vgt_enabled)
		return;

	p = __kvmgt_protect_table_find(kvm, gfn);
	if (p)
		hash_del(&p->hnode);
}

bool kvmgt_gfn_is_write_protected(struct kvm *kvm, gfn_t gfn)
{
	kvmgt_pgfn_t *p;

	if (!kvm->vgt_enabled)
		return false;

	p = __kvmgt_protect_table_find(kvm, gfn);
	return !!p;
}

bool kvmgt_emulate_write(struct kvm *kvm, gpa_t gpa, const void *val, int len)
{
	return vgt_ops->emulate_write(kvm->vgt, gpa, (void *)val, len);
}

int kvmgt_pin_slot(struct kvm *kvm, struct kvm_memory_slot *slot)
{
	int i;
	pfn_t pfn;
	struct page *page;

	if (!kvm->vgt_enabled || !slot->npages ||
				slot->id >= KVM_USER_MEM_SLOTS ||
				(slot->flags & KVM_MEM_READONLY))
		return -EFAULT;
	if (slot->pfn_list) {
		vgt_info("VM%d: slot %d: reuse pinned pages\n", kvm->domid, slot->id);
		return 0;
	}

	/* Try the 1st page, ignore this slot on errors. It's possible */
	page = gfn_to_page(kvm, slot->base_gfn);
	if (is_error_page(page)) {
		vgt_info("VM%d: ignore slot(%hd)\n", kvm->domid, slot->id);
		return -EFAULT;
	}
	kvm_release_page_clean(page);

	slot->pfn_list = kvm_kvzalloc(sizeof(pfn_t *) * slot->npages);
	if (!slot->pfn_list) {
		vgt_err("VM%d: slot %hd: failed to allocate pfn_list\n", kvm->domid, slot->id);
		return -ENOMEM;
	}

	/* Pin and record every pfn */
	for (i = 0; i < slot->npages; i++) {
		pfn = gfn_to_pfn(kvm, slot->base_gfn + i);

		WARN_ON(is_error_pfn(pfn));
		slot->pfn_list[i] = pfn;
	}

	vgt_info("VM%d: pinned slot id(%02hd) base_gfn(0x%llx) npages(%lu)\n",
			kvm->domid, slot->id, slot->base_gfn, slot->npages);
	return 0;
}

int kvmgt_unpin_slot(struct kvm *kvm, struct kvm_memory_slot *slot)
{
	int i;

	if (!kvm->vgt_enabled || !slot->npages ||
				slot->id >= KVM_USER_MEM_SLOTS ||
				(slot->flags & KVM_MEM_READONLY))
		return -EFAULT;

	if (!slot->pfn_list) {
		vgt_warn("VM%d: slot %hd: pfn_list is NULL!\n", kvm->domid, slot->id);
		return -ENOENT;
	}

	for (i = 0; i < slot->npages; i++)
		put_page(pfn_to_page(slot->pfn_list[i]));

	kvfree(slot->pfn_list);
	slot->pfn_list = NULL;

	vgt_info("VM%d: unpinned slot id(%02hd) base_gfn(0x%llx) npages(%lu)\n",
			kvm->domid, slot->id, slot->base_gfn, slot->npages);
	return 0;
}

void kvmgt_pin_guest(struct kvm *kvm)
{
	struct kvm_memslots *slots;
	struct kvm_memory_slot *memslot;
	int idx;

	mutex_lock(&kvm->slots_lock);
	idx = srcu_read_lock(&kvm->srcu);

	slots = kvm_memslots(kvm);
	kvm_for_each_memslot(memslot, slots)
		kvmgt_pin_slot(kvm, memslot);

	srcu_read_unlock(&kvm->srcu, idx);
	mutex_unlock(&kvm->slots_lock);
}

void kvmgt_unpin_guest(struct kvm *kvm)
{
	struct kvm_memslots *slots;
	struct kvm_memory_slot *memslot;
	int idx;

	mutex_lock(&kvm->slots_lock);
	idx = srcu_read_lock(&kvm->srcu);

	slots = kvm_memslots(kvm);
	kvm_for_each_memslot(memslot, slots)
		kvmgt_unpin_slot(kvm, memslot);

	srcu_read_unlock(&kvm->srcu, idx);
	mutex_unlock(&kvm->slots_lock);
}
