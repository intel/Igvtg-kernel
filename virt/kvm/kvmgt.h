/*
 * Interface abstraction for kvmgt
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
#ifndef _KVMGT_H_
#define _KVMGT_H_

#include <linux/kvm_host.h>

void kvmgt_init(struct kvm *kvm);
void kvmgt_exit(struct kvm *kvm);

void kvmgt_record_cf8(struct kvm_vcpu *vcpu, unsigned port,	unsigned long rax);
bool kvmgt_pio_is_igd_cfg(struct kvm_vcpu *vcpu);
bool kvmgt_pio_igd_cfg(struct kvm_vcpu *vcpu);

int kvmgt_pin_slot(struct kvm *kvm, struct kvm_memory_slot *slot);
int kvmgt_unpin_slot(struct kvm *kvm, struct kvm_memory_slot *slot);
void kvmgt_pin_guest(struct kvm *kvm);
void kvmgt_unpin_guest(struct kvm *kvm);

typedef struct kvmgt_pgfn {
	gfn_t gfn;
	struct hlist_node hnode;
} kvmgt_pgfn_t;

void kvmgt_protect_table_init(struct kvm *kvm);
void kvmgt_protect_table_destroy(struct kvm *kvm);
void kvmgt_protect_table_add(struct kvm *kvm, gfn_t gfn);
void kvmgt_protect_table_del(struct kvm *kvm, gfn_t gfn);
bool kvmgt_write_protect(struct kvm *kvm, gfn_t gfn, bool add);
bool kvmgt_gfn_is_write_protected(struct kvm *kvm, gfn_t gfn);
bool kvmgt_emulate_write(struct kvm *kvm, gpa_t gpa, const void *val, int len);

pfn_t kvmgt_gfn_to_pfn_by_rmap(struct kvm *kvm, gfn_t gfn);

static inline bool kvmgt_is_disabled_msr(u32 msr)
{
	switch (msr) {
	case 0x95: /* enable/disable cache cos */
	case 0xd00 ... 0xd03: /* eLLC cos ways mask */
		return true;
	default:
		break;
	}

	return false;
}

#endif /* _KVMGT_H_ */
