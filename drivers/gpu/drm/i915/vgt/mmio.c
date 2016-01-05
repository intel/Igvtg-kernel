/*
 * MMIO virtualization framework
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

#include <linux/acpi.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#include "vgt.h"

#define CREATE_TRACE_POINTS
#include "trace.h"

DEFINE_HASHTABLE(vgt_mmio_table, VGT_HASH_BITS);

void vgt_add_mmio_entry(struct vgt_mmio_entry *e)
{
	hash_add(vgt_mmio_table, &e->hlist, e->base);
}

struct vgt_mmio_entry * vgt_find_mmio_entry(unsigned int base)
{
	struct vgt_mmio_entry *e;

	hash_for_each_possible(vgt_mmio_table, e, hlist, base) {
		if (base == e->base)
			return e;
	}
	return NULL;
}

void vgt_del_mmio_entry(unsigned int base)
{
	struct vgt_mmio_entry *e;

	if ((e = vgt_find_mmio_entry(base))) {
		hash_del(&e->hlist);
		kfree(e);
	}
}

void vgt_clear_mmio_table(void)
{
	int i;
	struct hlist_node *tmp;
	struct vgt_mmio_entry *e;

	hash_for_each_safe(vgt_mmio_table, i, tmp, e, hlist)
		kfree(e);

	hash_init(vgt_mmio_table);
}

/* Default MMIO handler registration
 * These MMIO are registered as at least 4-byte aligned
 */
bool vgt_register_mmio_handler(unsigned int start, int bytes,
	vgt_mmio_read read, vgt_mmio_write write)
{
	int i, j, end;
	struct vgt_mmio_entry *mht;

	end = start + bytes -1;

	vgt_dbg(VGT_DBG_GENERIC, "start=0x%x end=0x%x\n", start, end);

	ASSERT((start & 3) == 0);
	ASSERT(((end+1) & 3) == 0);

	for ( i = start; i < end; i += 4 ) {
		mht = vgt_find_mmio_entry(i);
		if (mht) {
			mht->read = read;
			mht->write = write;
			continue;
		}

		mht = kmalloc(sizeof(*mht), GFP_KERNEL);
		if (mht == NULL) {
			printk("Insufficient memory in %s\n", __FUNCTION__);
			for (j = start; j < i; j += 4) {
				vgt_del_mmio_entry (j);
			}
			BUG();
		}
		mht->base = i;

		/*
		 * Win7 GFX driver uses memcpy to access the vgt PVINFO regs,
		 * hence align_bytes can be 1.
		 */
		if (start >= VGT_PVINFO_PAGE &&
			start < VGT_PVINFO_PAGE + VGT_PVINFO_SIZE)
			mht->align_bytes = 1;
		else
			mht->align_bytes = 4;

		mht->read = read;
		mht->write = write;
		INIT_HLIST_NODE(&mht->hlist);
		vgt_add_mmio_entry(mht);
	}
	return true;
}

static inline unsigned long vgt_get_passthrough_reg(struct vgt_device *vgt,
		unsigned int reg)
{
	__sreg(vgt, reg) = VGT_MMIO_READ(vgt->pdev, reg);
	__vreg(vgt, reg) = mmio_h2g_gmadr(vgt, reg, __sreg(vgt, reg));
	return __vreg(vgt, reg);
}

static unsigned long vgt_get_reg(struct vgt_device *vgt, unsigned int reg)
{
	/* check whether to update vreg from HW */
//	if (reg_hw_status(pdev, reg) &&
	if (reg_hw_access(vgt, reg))
		return vgt_get_passthrough_reg(vgt, reg);
	else
		return __vreg(vgt, reg);
}

static inline unsigned long vgt_get_passthrough_reg_64(struct vgt_device *vgt, unsigned int reg)
{
	__sreg64(vgt, reg) = VGT_MMIO_READ_BYTES(vgt->pdev, reg, 8);
	__vreg(vgt, reg) = mmio_h2g_gmadr(vgt, reg, __sreg(vgt, reg));
	__vreg(vgt, reg + 4) = mmio_h2g_gmadr(vgt, reg + 4, __sreg(vgt, reg + 4));
	return __vreg64(vgt, reg);
}
/*
 * for 64bit reg access, we split into two 32bit accesses since each part may
 * require address fix
 *
 * TODO: any side effect with the split? or instead install specific handler
 * for 64bit regs like fence?
 */
static unsigned long vgt_get_reg_64(struct vgt_device *vgt, unsigned int reg)
{
	/* check whether to update vreg from HW */
//	if (reg_hw_status(pdev, reg) &&
	if (reg_hw_access(vgt, reg))
		return vgt_get_passthrough_reg_64(vgt, reg);
	else
		return __vreg64(vgt, reg);
}

static void vgt_update_reg(struct vgt_device *vgt, unsigned int reg)
{
	struct pgt_device *pdev = vgt->pdev;
	/*
	 * update sreg if pass through;
	 * update preg if boot_time or vgt is reg's cur owner
	 */
	__sreg(vgt, reg) = mmio_g2h_gmadr(vgt, reg, __vreg(vgt, reg));
	if (reg_hw_access(vgt, reg))
		VGT_MMIO_WRITE(pdev, reg, __sreg(vgt, reg));
}

static void vgt_update_reg_64(struct vgt_device *vgt, unsigned int reg)
{
	struct pgt_device *pdev = vgt->pdev;
	/*
	 * update sreg if pass through;
	 * update preg if boot_time or vgt is reg's cur owner
	 */
	__sreg(vgt, reg) = mmio_g2h_gmadr(vgt, reg, __vreg(vgt, reg));
	__sreg(vgt, reg + 4) = mmio_g2h_gmadr(vgt, reg + 4, __vreg(vgt, reg + 4));
	if (reg_hw_access(vgt, reg))
			VGT_MMIO_WRITE_BYTES(pdev, reg, __sreg64(vgt, reg), 8);
}

bool default_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int reg;
	unsigned long wvalue;
	reg = offset & ~(bytes - 1);

	if (bytes <= 4) {
		wvalue = vgt_get_reg(vgt, reg);
	} else {
		wvalue = vgt_get_reg_64(vgt, reg);
	}

	memcpy(p_data, &wvalue + (offset & (bytes - 1)), bytes);

	return true;
}

bool default_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	memcpy((char *)vgt->state.vReg + offset,
			p_data, bytes);

	offset &= ~(bytes - 1);
	if (bytes <= 4)
		vgt_update_reg(vgt, offset);
	else
		vgt_update_reg_64(vgt, offset);

	return true;
}

bool default_passthrough_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int reg;
	unsigned long wvalue;
	reg = offset & ~(bytes - 1);

	if (bytes <= 4) {
		wvalue = vgt_get_passthrough_reg(vgt, reg);
	} else {
		wvalue = vgt_get_passthrough_reg_64(vgt, reg);
	}

	memcpy(p_data, &wvalue + (offset & (bytes - 1)), bytes);

	return true;
}

unsigned int vgt_pa_to_mmio_offset(struct vgt_device *vgt,
	uint64_t pa)
{
#define PCI_BAR_ADDR_MASK (~0xFUL)  /* 4 LSB bits are not address */
	return (vgt->vm_id == 0)?
		pa - vgt->pdev->gttmmio_base :
		pa - ( (*(uint64_t*)(vgt->state.cfg_space + VGT_REG_CFG_SPACE_BAR0))
				& PCI_BAR_ADDR_MASK );
}

static inline bool valid_mmio_alignment(struct vgt_mmio_entry *mht,
		unsigned int offset, int bytes)
{
	if ((bytes >= mht->align_bytes) && !(offset & (bytes - 1)))
		return true;
	vgt_err("Invalid MMIO offset(%08x), bytes(%d)\n",offset, bytes);
	return false;
}

static inline void mmio_accounting_read(struct vgt_device *vgt, unsigned long offset, cycles_t cycles)
{
	struct vgt_mmio_accounting_reg_stat *stat;

	if (!vgt->stat.mmio_accounting)
		return;

	stat = &vgt->stat.mmio_accounting_reg_stats[offset >> 2];
	stat->r_count++;
	stat->r_cycles += cycles;
}

static inline void mmio_accounting_write(struct vgt_device *vgt, unsigned long offset, cycles_t cycles)
{
	struct vgt_mmio_accounting_reg_stat *stat;

	if (!vgt->stat.mmio_accounting)
		return;

	stat = &vgt->stat.mmio_accounting_reg_stats[offset >> 2];
	stat->w_count++;
	stat->w_cycles += cycles;
}

/*
 * Emulate the VGT MMIO register read ops.
 * Return : true/false
 * */
bool vgt_emulate_read(struct vgt_device *vgt, uint64_t pa, void *p_data,int bytes)
{
	struct vgt_mmio_entry *mht;
	struct pgt_device *pdev = vgt->pdev;
	unsigned int offset;
	unsigned long flags;
	bool rc;
	cycles_t t0, t1;
	struct vgt_statistics *stat = &vgt->stat;
	int cpu;

	t0 = get_cycles();

	vgt_lock_dev_flags(pdev, cpu, flags);

	if (atomic_read(&vgt->gtt.n_write_protected_guest_page)) {
		guest_page_t *gp;
		gp = vgt_find_guest_page(vgt, pa >> PAGE_SHIFT);
		if (gp) {
			memcpy(p_data, gp->vaddr + (pa & ~PAGE_MASK), bytes);
			vgt_unlock_dev_flags(pdev, cpu, flags);
			return true;
		}
	}

	offset = vgt_pa_to_mmio_offset(vgt, pa);

	/* FENCE registers / GTT entries(sometimes) are accessed in 8 bytes. */
	if (bytes > 8 || (offset & (bytes - 1)))
		goto err_mmio;

	if (bytes > 4)
		vgt_dbg(VGT_DBG_GENERIC,"vGT: capture >4 bytes read to %x\n", offset);

	raise_ctx_sched(vgt);

	if (reg_is_gtt(pdev, offset)) {
		rc = gtt_emulate_read(vgt, offset, p_data, bytes);
		vgt_unlock_dev_flags(pdev, cpu, flags);
		return rc;
	}

	if (!reg_is_mmio(pdev, offset + bytes))
		goto err_mmio;

	mht = vgt_find_mmio_entry(offset);
	if ( mht && mht->read ) {
		if (!valid_mmio_alignment(mht, offset, bytes))
			goto err_mmio;
		if (!mht->read(vgt, offset, p_data, bytes))
			goto err_mmio;
	} else
		if (!default_mmio_read(vgt, offset, p_data, bytes))
			goto err_mmio;

	if (!reg_is_tracked(pdev, offset) && vgt->warn_untrack) {
		vgt_warn("vGT: untracked MMIO read: vm_id(%d), offset=0x%x,"
			"len=%d, val=0x%x!!!\n",
			vgt->vm_id, offset, bytes, *(u32 *)p_data);

		if (offset == 0x206c) {
			printk("------------------------------------------\n");
			printk("VM(%d) likely triggers a gfx reset\n", vgt->vm_id);
			printk("Disable untracked MMIO warning for VM(%d)\n", vgt->vm_id);
			printk("------------------------------------------\n");
			vgt->warn_untrack = 0;
			show_debug(pdev);
		}

		//WARN_ON(vgt->vm_id == 0); /* The call stack is meaningless for HVM */
	}

	reg_set_accessed(pdev, offset);

	vgt_unlock_dev_flags(pdev, cpu, flags);
	trace_vgt_mmio_rw(VGT_TRACE_READ, vgt->vm_id, offset, p_data, bytes);

	t1 = get_cycles();
	stat->mmio_rcnt++;
	stat->mmio_rcycles += t1 - t0;

	mmio_accounting_read(vgt, offset, t1 - t0);
	return true;
err_mmio:
	vgt_unlock_dev_flags(pdev, cpu, flags);
	vgt_err("VM(%d): invalid MMIO offset(%08x), bytes(%d)!\n",
		vgt->vm_id, offset, bytes);
	show_debug(pdev);
	return false;
}

/*
 * Emulate the VGT MMIO register write ops.
 * Return : true/false
 * */
bool vgt_emulate_write(struct vgt_device *vgt, uint64_t pa,
	void *p_data, int bytes)
{
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_mmio_entry *mht;
	unsigned int offset;
	unsigned long flags;
	int cpu;
	vgt_reg_t old_vreg=0, old_sreg=0;
	bool rc;
	cycles_t t0, t1;
	struct vgt_statistics *stat = &vgt->stat;

	vgt_lock_dev_flags(pdev, cpu, flags);

	t0 = get_cycles();

	if (atomic_read(&vgt->gtt.n_write_protected_guest_page)) {
		guest_page_t *guest_page;
		guest_page = vgt_find_guest_page(vgt, pa >> PAGE_SHIFT);
		if (guest_page) {
			rc = guest_page->handler(guest_page, pa, p_data, bytes);
			t1 = get_cycles();
			stat->wp_cycles += t1 - t0;
			stat->wp_cnt++;
			vgt_unlock_dev_flags(pdev, cpu, flags);
			return rc;
		}
	}

	offset = vgt_pa_to_mmio_offset(vgt, pa);

	/* FENCE registers / GTT entries(sometimes) are accessed in 8 bytes. */
	if (bytes > 8 || (offset & (bytes - 1)))
		goto err_mmio;

	if (bytes > 4)
		vgt_dbg(VGT_DBG_GENERIC,"vGT: capture >4 bytes write to %x with val (%lx)\n", offset, *(unsigned long*)p_data);
/*
	if (reg_rdonly(pdev, offset & (~(bytes - 1)))) {
		printk("vGT: captured write to read-only reg (%x)\n", offset);
		return true;
	}
*/

	raise_ctx_sched(vgt);

	if (reg_is_gtt(pdev, offset)) {
		rc = gtt_emulate_write(vgt, offset, p_data, bytes);
		vgt_unlock_dev_flags(pdev, cpu, flags);
		return rc;
	}

	if (!reg_is_mmio(pdev, offset + bytes))
		goto err_mmio;

	if (reg_mode_ctl(pdev, offset)) {
		old_vreg = __vreg(vgt, offset);
		old_sreg = __sreg(vgt, offset);
	}

	if (!reg_is_tracked(pdev, offset) && vgt->warn_untrack) {
		vgt_warn("vGT: untracked MMIO write: vm_id(%d), offset=0x%x,"
			"len=%d, val=0x%x!!!\n",
			vgt->vm_id, offset, bytes, *(u32 *)p_data);

		//WARN_ON(vgt->vm_id == 0); /* The call stack is meaningless for HVM */
	}

	mht = vgt_find_mmio_entry(offset);
	if ( mht && mht->write ) {
		if (!valid_mmio_alignment(mht, offset, bytes))
			goto err_mmio;
		if (!mht->write(vgt, offset, p_data, bytes))
			goto err_mmio;
	} else
		if (!default_mmio_write(vgt, offset, p_data, bytes))
			goto err_mmio;

	/* higher 16bits of mode ctl regs are mask bits for change */
	if (reg_mode_ctl(pdev, offset)) {
		u32 mask = __vreg(vgt, offset) >> 16;

		vgt_dbg(VGT_DBG_GENERIC,"old mode (%x): %x/%x, mask(%x)\n", offset,
			__vreg(vgt, offset), __sreg(vgt, offset),
			reg_aux_mode_mask(pdev, offset));
		/*
		 * share the global mask among VMs, since having one VM touch a bit
		 * not changed by another VM should be still saved/restored later
		 */
		reg_aux_mode_mask(pdev, offset) |= mask << 16;
		__vreg(vgt, offset) = (old_vreg & ~mask) | (__vreg(vgt, offset) & mask);
		__sreg(vgt, offset) = (old_sreg & ~mask) | (__sreg(vgt, offset) & mask);
		vgt_dbg(VGT_DBG_GENERIC,"new mode (%x): %x/%x, mask(%x)\n", offset,
			__vreg(vgt, offset), __sreg(vgt, offset),
			reg_aux_mode_mask(pdev, offset));
		//show_mode_settings(vgt->pdev);
	}

	if (offset == _REG_RCS_UHPTR)
		vgt_dbg(VGT_DBG_GENERIC,"vGT: write to UHPTR (%x,%x)\n", __vreg(vgt, offset), __sreg(vgt, offset));

	reg_set_accessed(pdev, offset);
	vgt_unlock_dev_flags(pdev, cpu, flags);
	trace_vgt_mmio_rw(VGT_TRACE_WRITE, vgt->vm_id, offset, p_data, bytes);

	t1 = get_cycles();
	stat->mmio_wcycles += t1 - t0;
	stat->mmio_wcnt++;
	mmio_accounting_write(vgt, offset, t1 - t0);
	return true;
err_mmio:
	vgt_unlock_dev_flags(pdev, cpu, flags);
	vgt_err("VM(%d): invalid MMIO offset(0x%08x, pa:0x%016llx),"
		"bytes(%d)!\n", vgt->vm_id, offset, pa, bytes);
	show_debug(pdev);
	return false;
}

static bool vgt_hvm_opregion_resinit(struct vgt_device *vgt, uint32_t gpa)
{
	void *orig_va = vgt->pdev->opregion_va;
	uint8_t	*buf;
	int i;

	if (vgt->state.opregion_va) {
		vgt_err("VM%d tried to init opregion multiple times!\n",
				vgt->vm_id);
		return false;
	}
	if (orig_va == NULL) {
		vgt_err("VM%d: No mapped OpRegion available\n", vgt->vm_id);
		return false;
	}

	vgt->state.opregion_va = (void *)__get_free_pages(GFP_ATOMIC |
			GFP_DMA32 | __GFP_ZERO,
			VGT_OPREGION_PORDER);
	if (vgt->state.opregion_va == NULL) {
		vgt_err("VM%d: failed to allocate memory for opregion\n",
				vgt->vm_id);
		return false;
	}

	memcpy_fromio(vgt->state.opregion_va, orig_va, VGT_OPREGION_SIZE);

	for (i = 0; i < VGT_OPREGION_PAGES; i++)
		vgt->state.opregion_gfn[i] = (gpa >> PAGE_SHIFT) + i;

	/* for unknown reason, the value in LID field is incorrect
	 * which block the windows guest, so workaround it by force
	 * setting it to "OPEN"
	 */
	buf = (uint8_t *)vgt->state.opregion_va;
	buf[VGT_OPREGION_REG_CLID] = 0x3;

	return true;
}

int vgt_hvm_opregion_map(struct vgt_device *vgt, int map)
{
	void *opregion;
	int rc;
	int i;

	opregion = vgt->state.opregion_va;

	for (i = 0; i < VGT_OPREGION_PAGES; i++) {
		rc = hypervisor_map_mfn_to_gpfn(vgt,
			vgt->state.opregion_gfn[i],
			hypervisor_virt_to_mfn(opregion + i*PAGE_SIZE),
			1,
			map,
			VGT_MAP_OPREGION);
		if (rc != 0)
			vgt_err("hypervisor_map_mfn_to_gpfn fail with %d!\n", rc);
	}

	return rc;
}

int vgt_hvm_opregion_init(struct vgt_device *vgt, uint32_t gpa)
{
	if (!opregion_present) {
		/* Need to allocate pages from host kernel */
		vgt_hvm_opregion_resinit(vgt, gpa);
		vgt_hvm_opregion_map(vgt, 1);
	} else {
		/* If opregion pages are not allocated from host kenrel, most of
		 * the params are meaningless */
		hypervisor_map_mfn_to_gpfn(vgt,
				0, //not used
				0, //not used
				2, //not used
				1,
				VGT_MAP_OPREGION);
	}

	/* modify the vbios parameters for PORTs,
	 * Let guest see full port capability.
	 */
	if (!propagate_monitor_to_guest && !is_current_display_owner(vgt))
		vgt_prepare_vbios_general_definition(vgt);

	return 0;
}

void vgt_initial_opregion_setup(struct pgt_device *pdev)
{
	pci_read_config_dword(pdev->pdev, VGT_REG_CFG_OPREGION,
			&pdev->opregion_pa);
	pdev->opregion_va = acpi_os_ioremap(pdev->opregion_pa,
			VGT_OPREGION_SIZE);
	if (pdev->opregion_va == NULL)
		vgt_err("Directly map OpRegion failed\n");
}

static void vgt_set_reg_attr(struct pgt_device *pdev,
	u32 reg, reg_attr_t *attr, bool track)
{
	/* ensure one entry per reg */
	ASSERT_NUM(!reg_is_tracked(pdev, reg) || !track, reg);

	if (reg_is_tracked(pdev, reg)) {
		if (track)
			printk("vGT: init a tracked reg (%x)!!!\n", reg);

		return;
	}

	reg_set_owner(pdev, reg, attr->flags & VGT_REG_OWNER);
	if (attr->flags & VGT_REG_PASSTHROUGH)
		reg_set_passthrough(pdev, reg);
	if (attr->flags & VGT_REG_ADDR_FIX ) {
		if (!attr->addr_mask)
			printk("vGT: ZERO addr fix mask for %x\n", reg);
		reg_set_addr_fix(pdev, reg, attr->addr_mask);

		/* set the default range size to 4, might be updated later */
		reg_aux_addr_size(pdev, reg) = 4;
	}
	if (attr->flags & VGT_REG_MODE_CTL)
		reg_set_mode_ctl(pdev, reg);
	if (attr->flags & VGT_REG_VIRT)
		reg_set_virt(pdev, reg);
	if (attr->flags & VGT_REG_HW_STATUS)
		reg_set_hw_status(pdev, reg);

	/* last mark the reg as tracked */
	if (track)
		reg_set_tracked(pdev, reg);
}

static void vgt_initialize_reg_attr(struct pgt_device *pdev,
	reg_attr_t *info, int num, bool track)
{
	int i, cnt = 0, tot = 0;
	u32 reg;
	reg_attr_t *attr;

	attr = info;
	for (i = 0; i < num; i++, attr++) {
		if (!vgt_match_device_attr(pdev, attr))
			continue;

		cnt++;
		if (track)
			vgt_dbg(VGT_DBG_GENERIC,"reg(%x): size(%x), device(%d), flags(%x), mask(%x), read(%llx), write(%llx)\n",
				attr->reg, attr->size, attr->device,
				attr->flags,
				attr->addr_mask,
				(u64)attr->read, (u64)attr->write);
		for (reg = attr->reg;
			reg < attr->reg + attr->size;
			reg += REG_SIZE) {
			vgt_set_reg_attr(pdev, reg, attr, track);
			tot++;
		}

		if (attr->read || attr->write)
			vgt_register_mmio_handler(attr->reg, attr->size,
				attr->read, attr->write);
	}
	printk("%d listed, %d used\n", num, cnt);
	printk("total %d registers tracked\n", tot);
}

void vgt_setup_reg_info(struct pgt_device *pdev)
{
	int i, reg;
	struct vgt_mmio_entry *mht;
	reg_addr_sz_t *reg_addr_sz;
	reg_list_t *reg_list = vgt_get_sticky_regs(pdev);

	printk("vGT: setup tracked reg info\n");
	vgt_initialize_reg_attr(pdev, vgt_reg_info_general,
		vgt_get_reg_num(D_ALL), true);

	if(IS_HSW(pdev))
		vgt_initialize_reg_attr(pdev, vgt_reg_info_hsw,
				vgt_get_hsw_reg_num(), true);
	
	if(IS_BDW(pdev))
		vgt_initialize_reg_attr(pdev, vgt_reg_info_bdw,
				vgt_get_reg_num(D_BDW), true);

	if(IS_SKL(pdev)) {
		vgt_initialize_reg_attr(pdev, vgt_reg_info_bdw,
				vgt_get_reg_num(D_BDW), true);
		vgt_initialize_reg_attr(pdev, vgt_reg_info_skl,
				vgt_get_reg_num(D_SKL), true);
	}

	/* GDRST can be accessed by byte */
	mht = vgt_find_mmio_entry(GEN6_GDRST);
	if (mht)
		mht->align_bytes = 1;

	for (i = 0; i < vgt_get_sticky_reg_num(pdev); i++) {
		for (reg = reg_list[i].reg;
		     reg < reg_list[i].reg + reg_list[i].size;
		     reg += REG_SIZE)
			reg_set_sticky(pdev, reg);
	}

	/* update the address range size in aux table */
	for (i =0; i < vgt_get_reg_addr_sz_num(); i++) {
		reg_addr_sz = &vgt_reg_addr_sz[i];
		if (reg_addr_sz->device & vgt_gen_dev_type(pdev))
			reg_aux_addr_size(pdev, reg_addr_sz->reg) = reg_addr_sz->size;
	}
}

static void __vgt_initial_mmio_space (struct pgt_device *pdev,
					reg_attr_t *info, int num)
{
	int i, j;
	reg_attr_t *attr;

	attr = info;

	for (i = 0; i < num; i++, attr++) {
		if (!vgt_match_device_attr(pdev, attr))
			continue;

		for (j = 0; j < attr->size; j += 4) {
			pdev->initial_mmio_state[REG_INDEX(attr->reg + j)] =
				VGT_MMIO_READ(pdev, attr->reg + j);
		}
	}

}

bool vgt_initial_mmio_setup (struct pgt_device *pdev)
{
	vgt_reg_t val;

	if (!pdev->initial_mmio_state) {
		pdev->initial_mmio_state = vzalloc(pdev->mmio_size);
		if (!pdev->initial_mmio_state) {
			printk("vGT: failed to allocate initial_mmio_state\n");
			return false;
		}
	}

	__vgt_initial_mmio_space(pdev, vgt_reg_info_general, vgt_get_reg_num(D_ALL));
	if(IS_HSW(pdev))
		__vgt_initial_mmio_space(pdev, vgt_reg_info_hsw, vgt_get_hsw_reg_num());
	if(IS_BDW(pdev))
		__vgt_initial_mmio_space(pdev, vgt_reg_info_bdw, vgt_get_reg_num(D_BDW));
	if(IS_SKL(pdev)) {
		__vgt_initial_mmio_space(pdev, vgt_reg_info_bdw, vgt_get_reg_num(D_BDW));
		__vgt_initial_mmio_space(pdev, vgt_reg_info_skl, vgt_get_reg_num(D_SKL));
	}

	/* customize the initial MMIO
	 * 1, GMBUS status
	 * 2, Initial port status. 
	 */

	/* GMBUS2 has an in-use bit as the hw semaphore, and we should recover
	 * it after the snapshot.
	 */
	pdev->initial_mmio_state[REG_INDEX(PCH_GMBUS2)] &= ~0x8000;

	val = (DEFAULT_INV_SR_PTR << _CTXBUF_READ_PTR_SHIFT) | DEFAULT_INV_SR_PTR;

	pdev->initial_mmio_state[REG_INDEX(_REG_RCS_CTX_STATUS_PTR)] = val;
	pdev->initial_mmio_state[REG_INDEX(_REG_VCS_CTX_STATUS_PTR)] = val;
	pdev->initial_mmio_state[REG_INDEX(_REG_VECS_CTX_STATUS_PTR)] = val;
	pdev->initial_mmio_state[REG_INDEX(_REG_VCS2_CTX_STATUS_PTR)] = val;
	pdev->initial_mmio_state[REG_INDEX(_REG_BCS_CTX_STATUS_PTR)] = val;

	val = ((_CTXBUF_READ_PTR_MASK << 16) |
			(DEFAULT_INV_SR_PTR << _CTXBUF_READ_PTR_SHIFT));

	VGT_MMIO_WRITE(pdev, _REG_RCS_CTX_STATUS_PTR, val);
	VGT_MMIO_WRITE(pdev, _REG_VCS_CTX_STATUS_PTR, val);
	VGT_MMIO_WRITE(pdev, _REG_VECS_CTX_STATUS_PTR, val);
	VGT_MMIO_WRITE(pdev, _REG_VCS2_CTX_STATUS_PTR, val);
	VGT_MMIO_WRITE(pdev, _REG_BCS_CTX_STATUS_PTR, val);

	VGT_MMIO_WRITE(pdev, PCH_GMBUS2,
			VGT_MMIO_READ(pdev, PCH_GMBUS2) | 0x8000);

	vgt_dpy_init_modes(pdev->initial_mmio_state);

	pdev->initial_mmio_state[REG_INDEX(WRPLL_CTL1)] &= ~(1 << 31);
	pdev->initial_mmio_state[REG_INDEX(WRPLL_CTL2)] &= ~(1 << 31);

	return true;
}

void state_vreg_init(struct vgt_device *vgt)
{
	int i;
	struct pgt_device *pdev = vgt->pdev;

	for (i = 0; i < pdev->mmio_size; i += sizeof(vgt_reg_t)) {
		/*
		 * skip the area of VGT PV INFO PAGE because we need keep
		 * its content across Dom0 S3.
		*/
		if (i >= VGT_PVINFO_PAGE &&
			i < VGT_PVINFO_PAGE + VGT_PVINFO_SIZE)
			continue;

		__vreg(vgt, i) = pdev->initial_mmio_state[i/sizeof(vgt_reg_t)];
	}

	/* set the bit 0:2 (Thread C-State) to C0
	 * TODO: consider other bit 3:31
	 */
	__vreg(vgt, GEN6_GT_THREAD_STATUS_REG) = 0;

	/* set the bit 0:2(Core C-State ) to C0 */
	__vreg(vgt, _REG_GT_CORE_STATUS) = 0;

	/*TODO: init other regs that need different value from pdev */

	if (IS_HSW(vgt->pdev)) {
		/*
		 * Clear FPGA_DBG_RM_NOCLAIM for not causing DOM0
		 * or Ubuntu HVM complains about unclaimed MMIO registers.
		 */
		__vreg(vgt, FPGA_DBG) &= ~FPGA_DBG_RM_NOCLAIM;
	}
}

/* TODO: figure out any security holes by giving the whole initial state */
void state_sreg_init(struct vgt_device *vgt)
{
	vgt_reg_t *sreg;

	sreg = vgt->state.sReg;
	memcpy (sreg, vgt->pdev->initial_mmio_state, vgt->pdev->mmio_size);

	/*
	 * Do we really need address fix for initial state? Any address information
	 * there is meaningless to a VM, unless that address is related to allocated
	 * GM space to the VM. Translate a host address '0' to a guest GM address
	 * is just a joke.
	 */
#if 0
	/* FIXME: add off in addr table to avoid checking all regs */
	for (i = 0; i < vgt->pdev->reg_num; i++) {
		if (reg_addr_fix(vgt->pdev, i * REG_SIZE)) {
			__sreg(vgt, i) = mmio_g2h_gmadr(vgt, i, __vreg(vgt, i));
			vgt_dbg(VGT_DBG_GENERIC,"vGT: address fix for reg (%x): (%x->%x)\n",
				i, __vreg(vgt, i), __sreg(vgt, i));
		}
	}
#endif
}
