/*
 * Instance life-cycle management
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

#include "vgt.h"

/*
 * bitmap of allocated vgt_ids.
 * bit = 0 means free ID, =1 means allocated ID.
 */
static unsigned long vgt_id_alloc_bitmap;

struct vgt_device *vmid_2_vgt_device(int vmid)
{
	unsigned int bit;
	struct vgt_device *vgt;

	ASSERT(vgt_id_alloc_bitmap != ~0UL)
	for_each_set_bit(bit, &vgt_id_alloc_bitmap, VGT_MAX_VMS) {
		vgt = default_device.device[bit];
		if (vgt && vgt->vm_id == vmid)
			return vgt;
	}
	return NULL;
}

static int allocate_vgt_id(struct pgt_device *pdev)
{
	struct vgt_device_info *info = &pdev->device_info;
	unsigned long bit_index;

	ASSERT(vgt_id_alloc_bitmap != ~0UL)
	do {
		bit_index = ffz (vgt_id_alloc_bitmap);
		if (bit_index >= info->max_support_vms) {
			vgt_err("vGT: allocate_vgt_id() failed\n");
			return -ENOSPC;
		}
	} while (test_and_set_bit(bit_index, &vgt_id_alloc_bitmap) != 0);

	return bit_index;
}

static void free_vgt_id(int vgt_id)
{
	ASSERT(vgt_id >= 0 && vgt_id < VGT_MAX_VMS);
	ASSERT(vgt_id_alloc_bitmap & (1UL << vgt_id));
	clear_bit(vgt_id, &vgt_id_alloc_bitmap);
}

/*
 * Initialize the vgt state instance.
 * Return:	0: failed
 *		1: success
 *
 */
static int create_state_instance(struct vgt_device *vgt)
{
	vgt_state_t	*state;
	int i;

	vgt_dbg(VGT_DBG_GENERIC, "create_state_instance\n");
	state = &vgt->state;
	state->vReg = vzalloc(vgt->pdev->mmio_size);
	state->sReg = vzalloc(vgt->pdev->mmio_size);
	if ( state->vReg == NULL || state->sReg == NULL )
	{
		printk("VGT: insufficient memory allocation at %s\n", __FUNCTION__);
		if ( state->vReg )
			vfree (state->vReg);
		if ( state->sReg )
			vfree (state->sReg);
		state->sReg = state->vReg = NULL;
		return -ENOMEM;
	}

	for (i = 0; i < I915_MAX_PIPES; i++) {
		vgt->pipe_mapping[i] = i;
	}

	for (i = 0; i < VGT_BAR_NUM; i++)
		state->bar_mapped[i] = 0;
	return 0;
}

/*
 * priv: VCPU ?
 */
int create_vgt_instance(struct pgt_device *pdev, struct vgt_device **ptr_vgt, vgt_params_t vp)
{
	int cpu;
	struct vgt_device *vgt;
	char *cfg_space;
	u16 *gmch_ctl;
	int rc = -ENOMEM;
	int i;

	vgt_info("vm_id=%d, low_gm_sz=%dMB, high_gm_sz=%dMB, fence_sz=%d, vgt_primary=%d\n",
		vp.vm_id, vp.aperture_sz, vp.gm_sz-vp.aperture_sz, vp.fence_sz, vp.vgt_primary);

	vgt = vzalloc(sizeof(*vgt));
	if (vgt == NULL) {
		printk("Insufficient memory for vgt_device in %s\n", __FUNCTION__);
		return rc;
	}
	if (reset_dur_threshold > 0 && reset_count_threshold > 0)
		vgt->reset_count_start_time =
			vzalloc(reset_count_threshold * sizeof(unsigned long));

	atomic_set(&vgt->crashing, 0);

	if ((rc = vgt->vgt_id = allocate_vgt_id(pdev)) < 0 )
		goto err2;

	vgt->vm_id = vp.vm_id;
	vgt->pdev = pdev;

	vgt->force_removal = 0;

	INIT_LIST_HEAD(&vgt->list);

	if ((rc = create_state_instance(vgt)) < 0)
		goto err2;

	for (i = 0; i < I915_MAX_PORTS; i++) {
		vgt->ports[i].type = VGT_PORT_MAX;
		vgt->ports[i].cache.type = VGT_PORT_MAX;
		vgt->ports[i].port_override = i;
		vgt->ports[i].cache.port_override = i;
		vgt->ports[i].physcal_port = i;
	}

	/* Hard code ballooning now. We can support non-ballooning too in the future */
	vgt->ballooning = 1;

	/* present aperture to the guest at the same host address */
	vgt->state.aperture_base = phys_aperture_base(pdev);

	/* init aperture/gm ranges allocated to this vgt */
	if ((rc = allocate_vm_aperture_gm_and_fence(vgt, vp)) < 0) {
		printk("vGT: %s: no enough available aperture/gm/fence!\n", __func__);
		goto err2;
	}

	vgt->aperture_offset = aperture_2_gm(pdev, vgt->aperture_base);
	vgt->aperture_base_va = phys_aperture_vbase(pdev) +
		vgt->aperture_offset;

	alloc_vm_rsvd_aperture(vgt);

	vgt->state.bar_size[0] = pdev->bar_size[0];	/* MMIOGTT */
	vgt->state.bar_size[1] =			/* Aperture */
		vgt->ballooning ? pdev->bar_size[1] : vgt_aperture_sz(vgt);
	vgt->state.bar_size[2] = pdev->bar_size[2];	/* PIO */
	vgt->state.bar_size[3] = pdev->bar_size[3];	/* ROM */

	/* Set initial configuration space and MMIO space registers. */
	cfg_space = &vgt->state.cfg_space[0];
	memcpy (cfg_space, pdev->initial_cfg_space, VGT_CFG_SPACE_SZ);
	cfg_space[VGT_REG_CFG_SPACE_MSAC] = vgt->state.bar_size[1];

	/* Show guest that there isn't any stolen memory.*/
	gmch_ctl = (u16 *)(cfg_space + _REG_GMCH_CONTRL);
	if (IS_PREBDW(pdev))
		*gmch_ctl &= ~(_REGBIT_SNB_GMCH_GMS_MASK << _REGBIT_SNB_GMCH_GMS_SHIFT);
	else
		*gmch_ctl &= ~(_REGBIT_BDW_GMCH_GMS_MASK << _REGBIT_BDW_GMCH_GMS_SHIFT);

	vgt_pci_bar_write_32(vgt, VGT_REG_CFG_SPACE_BAR1, phys_aperture_base(pdev) );

	/* mark HVM's GEN device's IO as Disabled. hvmloader will enable it */
	if (vgt->vm_id != 0) {
		cfg_space[VGT_REG_CFG_COMMAND] &= ~(_REGBIT_CFG_COMMAND_IO |
						_REGBIT_CFG_COMMAND_MEMORY |
						_REGBIT_CFG_COMMAND_MASTER);
		/* Clear the bar upper 32bit and let hvmloader to assign the new value */
		memset (&vgt->state.cfg_space[VGT_REG_CFG_SPACE_BAR0 + 4], 0, 4);
		memset (&vgt->state.cfg_space[VGT_REG_CFG_SPACE_BAR1 + 4], 0, 4);
	}

	vgt_info("aperture: [0x%llx, 0x%llx] guest [0x%llx, 0x%llx] "
		"va(0x%llx)\n",
		vgt_aperture_base(vgt),
		vgt_aperture_end(vgt),
		vgt_guest_aperture_base(vgt),
		vgt_guest_aperture_end(vgt),
		(uint64_t)vgt->aperture_base_va);

	vgt_info("GM: [0x%llx, 0x%llx], [0x%llx, 0x%llx], "
		"guest[0x%llx, 0x%llx], [0x%llx, 0x%llx]\n",
		vgt_visible_gm_base(vgt),
		vgt_visible_gm_end(vgt),
		vgt_hidden_gm_base(vgt),
		vgt_hidden_gm_end(vgt),
		vgt_guest_visible_gm_base(vgt),
		vgt_guest_visible_gm_end(vgt),
		vgt_guest_hidden_gm_base(vgt),
		vgt_guest_hidden_gm_end(vgt));

	/* If the user explicitly specified a value, use it; or, use the
	 * global vgt_primary.
	 */
	ASSERT(vgt->vm_id == 0 || (vp.vgt_primary >= -1 && vp.vgt_primary <= 1));
	if (vgt->vm_id != 0 &&
		(vp.vgt_primary == 0 || (vp.vgt_primary == -1 && !vgt_primary))) {
		/* Mark vgt device as non primary VGA */
		cfg_space[VGT_REG_CFG_CLASS_CODE] = VGT_PCI_CLASS_VGA;
		cfg_space[VGT_REG_CFG_SUB_CLASS_CODE] = VGT_PCI_CLASS_VGA_OTHER;
		cfg_space[VGT_REG_CFG_CLASS_PROG_IF] = VGT_PCI_CLASS_VGA_OTHER;
	}

	state_sreg_init (vgt);
	state_vreg_init(vgt);

	/* setup the ballooning information */
	if (vgt->ballooning) {
		__vreg64(vgt, vgt_info_off(magic)) = VGT_MAGIC;
		__vreg(vgt, vgt_info_off(version_major)) = 1;
		__vreg(vgt, vgt_info_off(version_minor)) = 0;
		__vreg(vgt, vgt_info_off(display_ready)) = 0;
		__vreg(vgt, vgt_info_off(vgt_id)) = vgt->vgt_id;
		__vreg(vgt, vgt_info_off(avail_rs.mappable_gmadr.base)) = vgt_visible_gm_base(vgt);
		__vreg(vgt, vgt_info_off(avail_rs.mappable_gmadr.size)) = vgt_aperture_sz(vgt);
		__vreg(vgt, vgt_info_off(avail_rs.nonmappable_gmadr.base)) = vgt_hidden_gm_base(vgt);
		__vreg(vgt, vgt_info_off(avail_rs.nonmappable_gmadr.size)) = vgt_hidden_gm_sz(vgt);

		__vreg(vgt, vgt_info_off(avail_rs.fence_num)) = vgt->fence_sz;
		vgt_info("filling VGT_PVINFO_PAGE for dom%d:\n"
			"   visable_gm_base=0x%llx, size=0x%llx\n"
			"   hidden_gm_base=0x%llx, size=0x%llx\n"
			"   fence_base=%d, num=%d\n",
			vgt->vm_id,
			vgt_visible_gm_base(vgt), vgt_aperture_sz(vgt),
			vgt_hidden_gm_base(vgt), vgt_hidden_gm_sz(vgt),
			vgt->fence_base, vgt->fence_sz);

		ASSERT(sizeof(struct vgt_if) == VGT_PVINFO_SIZE);
	}

	vgt->bypass_addr_check = bypass_dom0_addr_check && (vgt->vm_id == 0);

	for (i = 0; i < MAX_ENGINES; ++i)
		vgt->rb[i].csb_write_ptr = DEFAULT_INV_SR_PTR;

	vgt_reset_virtual_interrupt_registers(vgt);

	vgt_lock_dev(pdev, cpu);

	pdev->device[vgt->vgt_id] = vgt;
	list_add(&vgt->list, &pdev->rendering_idleq_head);

	vgt_unlock_dev(pdev, cpu);

	if (!vgt_init_vgtt(vgt)) {
		vgt_err("fail to initialize vgt vgtt.\n");
		goto err2;
	}

	if (vgt->vm_id != 0){
		/* HVM specific init */
		if ((rc = hypervisor_hvm_init(vgt)) < 0)
			goto err;
	}

	if (vgt->vm_id) {
		if (hvm_render_owner)
			current_render_owner(pdev) = vgt;

		if (hvm_display_owner)
			current_display_owner(pdev) = vgt;

		if (hvm_super_owner) {
			ASSERT(hvm_render_owner);
			ASSERT(hvm_display_owner);
			ASSERT(hvm_boot_foreground);
			current_config_owner(pdev) = vgt;
			current_foreground_vm(pdev) = vgt;
		}
		if (opregion_present) {
			vgt_info("writing virtual 0xfc opregion for VM%d\n", vgt->vm_id);
			vgt_hvm_opregion_init(vgt, 0);
		}
	}
	bitmap_zero(vgt->enabled_rings, MAX_ENGINES);
	bitmap_zero(vgt->started_rings, MAX_ENGINES);
	bitmap_zero(vgt->tlb_handle_pending, MAX_ENGINES);

	/* create debugfs per vgt */
	if ((rc = vgt_create_debugfs(vgt)) < 0) {
		vgt_err("failed to create debugfs for vgt-%d\n",
			vgt->vgt_id);
		goto err;
	}

	if ((rc = vgt_create_mmio_dev(vgt)) < 0) {
		vgt_err("failed to create mmio devnode for vgt-%d\n",
				vgt->vgt_id);
		goto err;
	}

	vgt_init_i2c_edid(vgt);

	*ptr_vgt = vgt;

	/* initialize context scheduler infor */
	vgt_init_sched_info(vgt);

	if (shadow_tail_based_qos)
		vgt_init_rb_tailq(vgt);

	mutex_init(&vgt->stat.mmio_accounting_lock);
	vgt->stat.mmio_accounting = false;

	vgt->warn_untrack = 1;
	return 0;
err:
	vgt_clean_vgtt(vgt);
err2:
	if (vgt->vm_id)
		hypervisor_hvm_exit(vgt);
	if (vgt->aperture_base > 0)
		free_vm_aperture_gm_and_fence(vgt);
	vfree(vgt->state.vReg);
	vfree(vgt->state.sReg);
	if (vgt->vgt_id >= 0)
		free_vgt_id(vgt->vgt_id);
	vfree(vgt->reset_count_start_time);
	vfree(vgt);
	return rc;
}

void vgt_release_instance(struct vgt_device *vgt)
{
	int i;
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_device *v = NULL;
	int cpu;

	printk("prepare to destroy vgt (%d)\n", vgt->vgt_id);

	vgt_hvm_set_trap_area(vgt, 0);
	/* destroy vgt_mmio_device */
	vgt_destroy_mmio_dev(vgt);

	vgt_destroy_debugfs(vgt);

	vgt_lock_dev(pdev, cpu);

	printk("check render ownership...\n");
	list_for_each_entry(v, &pdev->rendering_runq_head, list) {
		if (v == vgt)
			break;
	}

	if (v != vgt)
		printk("vgt instance has been removed from run queue\n");
	else if (hvm_render_owner || current_render_owner(pdev) != vgt) {
		printk("remove vgt(%d) from runqueue safely\n",
			vgt->vgt_id);
		vgt_disable_render(vgt);
	} else {
		printk("vgt(%d) is current owner, request reschedule\n",
			vgt->vgt_id);
		vgt_request_force_removal(vgt);
	}

	printk("check display ownership...\n");
	if (!hvm_super_owner && (current_display_owner(pdev) == vgt)) {
		vgt_dbg(VGT_DBG_DPY, "switch display ownership back to dom0\n");
		current_display_owner(pdev) = vgt_dom0;
	}

	if (!hvm_super_owner && (current_foreground_vm(pdev) == vgt)) {
		vgt_dbg(VGT_DBG_DPY, "switch foreground vm back to dom0\n");
		pdev->next_foreground_vm = vgt_dom0;
		do_vgt_fast_display_switch(pdev);
	}

	vgt_unlock_dev(pdev, cpu);
	if (vgt->force_removal)
		/* wait for removal completion */
		wait_event_killable(pdev->destroy_wq, !vgt->force_removal);

	printk("release display/render ownership... done\n");

	/* FIXME: any conflicts between destroy_wq ? */
	if (shadow_tail_based_qos)
		vgt_destroy_rb_tailq(vgt);

	vgt_clean_vgtt(vgt);

	if (vgt->state.opregion_va) {
		if (opregion_present) {
			vunmap(vgt->state.opregion_va);
			for (i = 0; i < VGT_OPREGION_PAGES; i++)
				put_page(vgt->state.opregion_pages[i]);
		} else {
			vgt_hvm_opregion_map(vgt, 0);
			free_pages((unsigned long)vgt->state.opregion_va,
						VGT_OPREGION_PORDER);
		}
	}

	if (vgt->vm_id)
		hypervisor_hvm_exit(vgt);

	vgt_lock_dev(pdev, cpu);

	vgt->pdev->device[vgt->vgt_id] = NULL;
	free_vgt_id(vgt->vgt_id);

	/* already idle */
	list_del(&vgt->list);

	vgt_unlock_dev(pdev, cpu);

	for (i = 0; i < I915_MAX_PORTS; i++) {
		if (vgt->ports[i].edid) {
			kfree(vgt->ports[i].edid);
			vgt->ports[i].edid = NULL;
		}

		if (vgt->ports[i].dpcd) {
			kfree(vgt->ports[i].dpcd);
			vgt->ports[i].dpcd = NULL;
		}

		if (vgt->ports[i].cache.edid) {
			kfree(vgt->ports[i].cache.edid);
			vgt->ports[i].cache.edid = NULL;
		}

		if (vgt->ports[i].kobj.state_initialized) {
			kobject_put(&vgt->ports[i].kobj);
		}
	}

	/* clear the gtt entries for GM of this vgt device */
	vgt_clear_gtt(vgt);

	free_vm_aperture_gm_and_fence(vgt);
	free_vm_rsvd_aperture(vgt);
	vfree(vgt->state.vReg);
	vfree(vgt->state.sReg);
	vfree(vgt->reset_count_start_time);
	vfree(vgt);
	printk("vGT: vgt_release_instance done\n");
}

void vgt_reset_ppgtt(struct vgt_device *vgt, unsigned long ring_bitmap)
{
	int bit;

	if (!vgt->pdev->enable_ppgtt || !vgt->gtt.active_ppgtt_mm_bitmap)
		return;

	if (ring_bitmap == 0xff)
		vgt_info("VM %d: Reset full virtual PPGTT state.\n", vgt->vm_id);

	for_each_set_bit(bit, &ring_bitmap, sizeof(ring_bitmap)) {
		if (bit >= vgt->pdev->max_engines)
			break;

		if (!test_bit(bit, &vgt->gtt.active_ppgtt_mm_bitmap))
			continue;

		vgt_info("VM %d: Reset ring %d PPGTT state.\n", vgt->vm_id, bit);

		vgt->rb[bit].has_ppgtt_mode_enabled = 0;
		vgt->rb[bit].has_ppgtt_base_set = 0;
		vgt->rb[bit].ppgtt_page_table_level = 0;
		vgt->rb[bit].ppgtt_root_pointer_type = GTT_TYPE_INVALID;

		clear_bit(bit, &vgt->gtt.active_ppgtt_mm_bitmap);
	}

	return;
}

static void vgt_reset_ringbuffer(struct vgt_device *vgt, unsigned long ring_bitmap)
{
	vgt_state_ring_t *rb;
	int bit;

	ASSERT((IS_PREBDW(vgt->pdev)));

	for_each_set_bit(bit, &ring_bitmap, sizeof(ring_bitmap)) {
		if (bit >= vgt->pdev->max_engines)
			break;

		rb = &vgt->rb[bit];

		/* Drop all submitted commands. */
		vgt_init_cmd_info(rb);

		rb->uhptr = 0;
		rb->request_id = rb->uhptr_id = 0;

		memset(&rb->vring, 0, sizeof(vgt_ringbuffer_t));
		memset(&rb->sring, 0, sizeof(vgt_ringbuffer_t));

		vgt_disable_ring(vgt, bit);

		if (bit == RING_BUFFER_RCS) {
			struct pgt_device *pdev = vgt->pdev;
			struct vgt_rsvd_ring *ring = &pdev->ring_buffer[bit];

			memcpy((char *)v_aperture(pdev, rb->context_save_area),
					(char *)v_aperture(pdev, ring->null_context),
					SZ_CONTEXT_AREA_PER_RING);

			vgt->has_context = rb->active_vm_context = 0;
		}
	}

	return;
}

void vgt_reset_virtual_states(struct vgt_device *vgt, unsigned long ring_bitmap)
{
	ASSERT(spin_is_locked(&vgt->pdev->lock));

	if (!vgt->pdev->enable_execlist)
		vgt_reset_ringbuffer(vgt, ring_bitmap);
	else
		vgt_reset_execlist(vgt, ring_bitmap);

	vgt_reset_ppgtt(vgt, ring_bitmap);

	return;
}
