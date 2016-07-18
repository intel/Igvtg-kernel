/*
 * MMIO virtualization handlers
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

#include <linux/delay.h>
#include <linux/acpi.h>

#include "vgt.h"
#include "fb_decoder.h"

/* working for both HSW/BDW and SKL+ */
#define OFFSET_TO_DP_AUX_PORT(offset) (((offset) & 0xF00) >> 8)


static bool vgt_error_handler(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	printk("vGT: reg (%x) needs special handler\n", offset);
	return false;
}

static bool vgt_not_allowed_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	vgt_err("VM(%d): MMIO reading of reg 0x%x is not allowed. "
			"0 will be returned!\n", vgt->vm_id, offset);
	*(vgt_reg_t *)p_data = 0;
	return true;
}

static bool vgt_not_allowed_mmio_write(struct vgt_device *vgt,
	unsigned int offset, void *p_data, unsigned int bytes)
{
	vgt_err("VM(%d): MMIO write of reg 0x%x with 0x%x (%d)bytes is not allowed. ",
			vgt->vm_id, offset, *(vgt_reg_t *)p_data, bytes);
	return true;
}

static bool gmbus_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	if (reg_hw_access(vgt, offset))
		return default_mmio_read(vgt, offset, p_data, bytes);
	else
		return vgt_i2c_handle_gmbus_read(vgt, offset, p_data, bytes);
}

static bool gmbus_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	if (reg_hw_access(vgt, offset))
		return default_mmio_write(vgt, offset, p_data, bytes);
	else
		return vgt_i2c_handle_gmbus_write(vgt, offset, p_data, bytes);
}

static bool fence_mmio_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int id;
	if (bytes > 8 || (off & (bytes - 1))) {
		vgt_err("vGT(%d) fence_mmio_read: invalid offset(%x) or bytes(%d)\n",
				vgt->vgt_id, off, bytes);
		return false;
	};

	id = (off - _REG_FENCE_0_LOW) >> 3;

	if (id >= vgt->fence_sz) {
		printk("vGT(%d) , read fence register %x,"
			" %x out of assignment %x.\n", vgt->vgt_id,
			off, id, vgt->fence_sz);
	}
	memcpy (p_data, (char *)vgt->state.vReg + off, bytes);
	return true;
}

static bool fence_mmio_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int id;
	if (bytes > 8 || (off & (bytes - 1))) {
		vgt_err("vGT(%d) fence_mmio_write: invalid offset(%x) or bytes(%d)\n",
				vgt->vgt_id, off, bytes);
		return false;
	};

	id = (off - _REG_FENCE_0_LOW) >> 3;

	if (id >= vgt->fence_sz) {
		printk("vGT (%d) , write fence register %x,"
			" %x out of assignment %x.\n", vgt->vgt_id,
			off, id, vgt->fence_sz);
	}
	else {
		memcpy ((char *)vgt->state.vReg + off, p_data, bytes);
		memcpy ((char *)vgt->state.sReg + off, p_data, bytes);
		/* TODO: Check address space */

		/* FENCE registers are physically assigned, update! */
		if (bytes < 8)
			VGT_MMIO_WRITE(vgt->pdev, off + vgt->fence_base * 8,
				__sreg(vgt, off));
		else
			VGT_MMIO_WRITE_BYTES(vgt->pdev, off + vgt->fence_base * 8,
				__sreg64(vgt, off), 8);
	}
	return true;
}

static inline void set_vRC(struct vgt_device *vgt, int c)
{
	__vreg(vgt, _REG_GT_CORE_STATUS) = c;
	__vreg(vgt, GEN6_GT_THREAD_STATUS_REG) = c;
}

static void set_vRC_to_C6(struct vgt_device *vgt)
{
	vgt_dbg(VGT_DBG_GENERIC, "Virtual Render C state set to C6\n");
	set_vRC(vgt, 3);
}

static void set_vRC_to_C0(struct vgt_device *vgt)
{
	vgt_dbg(VGT_DBG_GENERIC, "Virtual Render C state set to C0\n");
	set_vRC(vgt, 0);
}

static void v_force_wake_get(struct vgt_device *vgt)
{
	/* ignore hvm guest's forcewake req */
	if (vgt->vm_id != 0)
		return;

	WARN(1, "Host driver should take care forcewake itself!\n");
}

static void v_force_wake_put(struct vgt_device *vgt)
{
	/* ignore hvm guest's forcewake req */
	if (vgt->vm_id != 0)
		return;

	WARN(1, "Host driver should take care forcewake itself!\n");
}

static bool force_wake_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t data;

	data = (*(uint32_t*) p_data);

	vgt_dbg(VGT_DBG_GENERIC, "VM%d write register FORCE_WAKE with %x\n", vgt->vm_id, data);

	data &= FORCEWAKE_KERNEL;

	if (IS_HSW(vgt->pdev)) {
		__vreg(vgt, FORCEWAKE_ACK_HSW) = data;
	} else if (IS_BDW(vgt->pdev)) {
		__vreg(vgt, FORCEWAKE_ACK) = data;
	}

	__vreg(vgt, FORCEWAKE) = data;
	if (data == 1) {
		set_vRC_to_C0(vgt);
		v_force_wake_get(vgt);
	} else {
		set_vRC_to_C6(vgt);
		v_force_wake_put(vgt);
	}
	return true;
}

static bool mul_force_wake_ack_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	*(u32 *)p_data = __vreg(vgt, offset);
	return true;
}

bool mul_force_wake_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t data, mask, wake, old_wake, new_wake;
	uint32_t ack_reg_offset;

	data = *(uint32_t*) p_data;

	vgt_dbg(VGT_DBG_GENERIC, "VM%d write register FORCE_WAKE_MT with %x\n", vgt->vm_id, data);

	if (!IS_BDWPLUS(vgt->pdev) && !(__vreg(vgt, ECOBUS) & FORCEWAKE_MT_ENABLE)) {
		__vreg(vgt, FORCEWAKE_MT) = data;
		return true;
	}

	/* bit 16-31: mask
	   bit 0-15: force wake
	   forcewake bit apply only if its mask bit is 1
	 */
	if (IS_SKL(vgt->pdev)) {
		switch (offset) {
		case FORCEWAKE_RENDER_GEN9:
			ack_reg_offset = FORCEWAKE_ACK_RENDER_GEN9;
			break;
		case FORCEWAKE_BLITTER_GEN9:
			ack_reg_offset = FORCEWAKE_ACK_BLITTER_GEN9;
			break;
		case FORCEWAKE_MEDIA_GEN9:
			ack_reg_offset = FORCEWAKE_ACK_MEDIA_GEN9;
			break;
		default:
			/*should not hit here*/
			vgt_err("invalid forcewake offset 0x%x\n",
				offset);
			return false;
		}
	} else {
		ack_reg_offset = FORCEWAKE_ACK_HSW;
	}

	mask = data >> 16;
	wake = data & 0xFFFF;
	old_wake = __vreg(vgt, offset) & 0xFFFF;

	new_wake = (old_wake & ~mask) + (wake & mask);
	__vreg(vgt, offset) = (data & 0xFFFF0000) + new_wake;
	__vreg(vgt, ack_reg_offset) = new_wake;

	if (new_wake){
		v_force_wake_get(vgt);
		set_vRC_to_C0(vgt);
	}else{
		v_force_wake_put(vgt);
		set_vRC_to_C6(vgt);
	}

	return true;
}

static bool rc_state_ctrl_1_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t data;

	data = *(uint32_t*)p_data;
	printk("VM%d write register RC_STATE_CTRL_1 with 0x%x\n", vgt->vm_id, data);

	if ( (data & _REGBIT_RC_HW_CTRL_ENABLE) && (data & (_REGBIT_RC_RC6_ENABLE
					| _REGBIT_RC_DEEPEST_RC6_ENABLE	| _REGBIT_RC_DEEP_RC6_ENABLE) ) )
		set_vRC_to_C6(vgt);
	else
		set_vRC_to_C0(vgt);

	return default_mmio_write(vgt, offset, p_data, bytes);

}

static bool handle_device_reset(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes, unsigned long ring_bitmap)
{
	int bit;

	vgt_info("VM %d is trying to reset device: %s.\n", vgt->vm_id,
		ring_bitmap == 0xff ? "full reset" : "per-engine reset");

	vgt->pdev->cur_reset_vm = vgt;
	show_debug(vgt->pdev);
	if (vgt_debug & VGT_DBG_RESET)
		dump_all_el_contexts(vgt->pdev);
	dump_el_status(vgt->pdev);
	vgt->pdev->cur_reset_vm = NULL;

	/* after this point, driver should re-initialize the device */
	vgt->warn_untrack = 1;
	set_bit(RESET_INPROGRESS, &vgt->reset_flags);

	clear_bit(WAIT_RESET, &vgt->reset_flags);

	vgt_reset_virtual_states(vgt, ring_bitmap);

	if (ring_bitmap != 0xff && vgt->vm_id && vgt->enabled_rings_before_reset) {
		vgt->enabled_rings_before_reset &= ~ring_bitmap;

		for_each_set_bit(bit, &vgt->enabled_rings_before_reset,
				sizeof(vgt->enabled_rings_before_reset)) {
			vgt_info("VM %d: re-enable ring %d after per-engine reset.\n",
					vgt->vm_id, bit);
			vgt_enable_ring(vgt, bit);
		}

		vgt->enabled_rings_before_reset = 0;
	}

	vgt->last_reset_time = get_seconds();

	if (vgt->vm_id == 0) {
		if (device_is_reseting(vgt->pdev))
			return default_mmio_write(vgt, offset, p_data, bytes);
	} else {
		if (current_render_owner(vgt->pdev) == vgt) {
			vgt_request_force_removal(vgt);

			vgt_info("VM %d: unlock before wait for force removal event\n",
					vgt->vm_id);

			spin_unlock(&vgt->pdev->lock);
			if (vgt->force_removal)
				wait_event_killable(vgt->pdev->destroy_wq, !vgt->force_removal);

			vgt_info("VM %d: force removal event... wake up\n",
					vgt->vm_id);

			spin_lock(&vgt->pdev->lock);

			vgt_info("VM %d: lock again afterforce removal event\n",
					vgt->vm_id);


		}

		/*clean up during reset */
		if (test_and_clear_bit(RESET_INPROGRESS, &vgt->reset_flags)) {

			vgt_info("VM %d: vgt_clean_up begin.\n", vgt->vm_id);

			/*unlock first, may sleep @ vfree in vgt_clean_vgtt*/
			spin_unlock(&vgt->pdev->lock);
			vgt_clean_vgtt(vgt);
			vgt_clear_gtt(vgt);
			state_sreg_init(vgt);
			state_vreg_init(vgt);
			vgt_init_vgtt(vgt);

			vgt_info("VM %d: vgt_clean_up end.\n", vgt->vm_id);

			spin_lock(&vgt->pdev->lock);

			vgt_info("VM %d: lock.again\n", vgt->vm_id);
		}
	}

	return true;
}

/*be noted that big lock is called inside handle_device_reset*/
static bool gen6_gdrst_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	uint32_t data = 0;
	unsigned long ring_bitmap = 0;

	memcpy(&data, p_data, bytes);

	if (data & GEN6_GRDOM_FULL) {
		vgt_info("VM %d request Full GPU Reset\n", vgt->vm_id);
		ring_bitmap = 0xff;
	}

	if (data & GEN6_GRDOM_RENDER) {
		vgt_info("VM %d request GPU Render Reset\n", vgt->vm_id);
		ring_bitmap |= (1 << RING_BUFFER_RCS);
	}

	if (data & GEN6_GRDOM_MEDIA) {
		vgt_info("VM %d request GPU Media Reset\n", vgt->vm_id);
		ring_bitmap |= (1 << RING_BUFFER_VCS);
	}

	if (data & GEN6_GRDOM_BLT) {
		vgt_info("VM %d request GPU BLT Reset\n", vgt->vm_id);
		ring_bitmap |= (1 << RING_BUFFER_BCS);
	}

	if (IS_HSW(vgt->pdev) && (data & (1 << 4))) {
		vgt_info("VM %d request GPU VECS Reset\n", vgt->vm_id);
		ring_bitmap |= (1 << RING_BUFFER_VECS);
	}

	return handle_device_reset(vgt, offset, p_data, bytes, ring_bitmap);
}


static bool gen6_gdrst_mmio_read(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	vgt_reg_t v;

	*(u32 *)p_data = 0;

	if (device_is_reseting(vgt->pdev) && vgt->vm_id == 0) {
		v = VGT_MMIO_READ(vgt->pdev, offset);

		memcpy(p_data, &v, bytes);

		if (v) {
			vgt_info("device is still reseting...\n");
		} else {
			vgt_info("device is idle.\n");

			show_interrupt_regs(vgt->pdev, NULL);
		}
	}

	return true;
}

bool vgt_rrmr_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t old_rrmr, new_rrmr, new_physical_rrmr;
	struct pgt_device *pdev = vgt->pdev;

	old_rrmr = __vreg(vgt, offset);
	new_physical_rrmr = new_rrmr = *(u32 *)p_data;

	__vreg(vgt, offset) = new_rrmr;

	if (old_rrmr != new_rrmr) {
		new_physical_rrmr = vgt_recalculate_mask_bits(pdev, offset);
		VGT_MMIO_WRITE(pdev, offset, new_physical_rrmr);
	}

	vgt_dbg(VGT_DBG_DPY, "RRMR: VM%d: old (%x), new (%x), new_physical (%x)\n",
		vgt->vm_id, old_rrmr, new_rrmr, new_physical_rrmr);
	return true;
}

static bool pch_pp_control_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t data;
	uint32_t reg;
	union _PCH_PP_CONTROL pp_control;
	union _PCH_PP_STAUTS pp_status;

	reg = offset & ~(bytes - 1);
	if (reg_hw_access(vgt, reg)){
		return default_mmio_write(vgt, offset, p_data, bytes);
	}

	data = *(uint32_t*)p_data;

	__vreg(vgt, PCH_PP_CONTROL) = data;

	pp_control.data = data;
	pp_status.data = __vreg(vgt, PCH_PP_STATUS);
	if (pp_control.power_state_target == 1){
		/* power on panel */
		pp_status.panel_powere_on_statue = 1;
		pp_status.power_sequence_progress = 0;
		pp_status.power_cycle_delay_active = 0;
	} else {
		/* power down panel */
		pp_status.panel_powere_on_statue = 0;
		pp_status.power_sequence_progress = 0;
		pp_status.power_cycle_delay_active = 0;
	}
	__vreg(vgt, PCH_PP_STATUS) = pp_status.data;

	return true;
}

static bool transaconf_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t reg;
	union _TRANS_CONFIG config;

	reg = offset & ~(bytes - 1);
	if (reg_hw_access(vgt, reg)){
		return default_mmio_write(vgt, offset, p_data, bytes);
	}

	config.data = *(uint32_t*)p_data;
	/* transcoder state should synced with enable */
	config.transcoder_state = config.transcoder_enable;

	__vreg(vgt, reg) = config.data;

	return true;
}

static bool shotplug_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	vgt_reg_t val = *(vgt_reg_t *)p_data;
	vgt_reg_t sticky_mask = _REGBIT_DP_B_STATUS |
				_REGBIT_DP_C_STATUS |
				_REGBIT_DP_D_STATUS;

	__vreg(vgt, offset) = (val & ~sticky_mask) |
				(__vreg(vgt, offset) & sticky_mask);
	__vreg(vgt, offset) &= ~(val & sticky_mask);

	__sreg(vgt, offset) = val;

	if (reg_hw_access(vgt, offset)) {
		vgt_reg_t enable_mask = _REGBIT_DP_B_ENABLE |
					_REGBIT_DP_C_ENABLE |
					_REGBIT_DP_D_ENABLE;

		if (~(val & enable_mask) & enable_mask) {
			vgt_warn("vGT(%d): Is trying to disable HOTPLUG"
			" with writing 0x%x to SHOTPLUG_CTL!\n",
			vgt->vgt_id, val);
		}
		/* do not let display owner clear the status bits.
		 * vgt driver will do so in interrupt handling.
		 */
		val &= ~sticky_mask;
		VGT_MMIO_WRITE(vgt->pdev, offset, val);
	}

	return true;
}
static bool lcpll_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int reg;
	vgt_reg_t vreg_data;
	bool rc;

	reg = offset & ~(bytes - 1);
	vreg_data = *(vgt_reg_t *)p_data;

	rc = default_mmio_write(vgt, offset, &vreg_data, bytes);

	if (!reg_hw_access(vgt, reg)) {
		vreg_data = __vreg(vgt, offset);

		if (vreg_data & LCPLL_PLL_DISABLE)
			vreg_data &= ~LCPLL_PLL_LOCK;
		else
			vreg_data |= LCPLL_PLL_LOCK;

		if (vreg_data & LCPLL_CD_SOURCE_FCLK)
			vreg_data |= LCPLL_CD_SOURCE_FCLK_DONE;
		else
			vreg_data &= ~LCPLL_CD_SOURCE_FCLK_DONE;

		__vreg(vgt, offset) = vreg_data;
	}

	return rc;

}

/* Pipe Frame Count */
static bool pipe_frmcount_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	enum pipe pipe;

	/* TODO
	 *
	 * If we can switch display owner, the frmcount should be handled specially
	 * also so that hvm(including dom0) could have monotinic view of frmcount
	 * during the owner ship switching. But Right now we do not allow the
	 * display owner switch, so it is OK.
	 */
	if (is_current_display_owner(vgt))
		return default_passthrough_mmio_read(vgt, offset,
				p_data, bytes);

	pipe = VGT_FRMCOUNTPIPE(offset);
	ASSERT(pipe >= PIPE_A && pipe < I915_MAX_PIPES);

	*(vgt_reg_t *)p_data = __vreg(vgt, offset);

	return true;
}

/* Pipe Display Scan Line*/
static bool pipe_dsl_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	return default_passthrough_mmio_read(vgt, offset, p_data, bytes);
}

static bool dpy_reg_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	*(uint32_t*)p_data = (1<<17);

	return true;
}

static bool dpy_reg_mmio_read_2(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	*(uint32_t*)p_data = 3;

	return true;
}

static bool dpy_reg_mmio_read_3(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	*(uint32_t*)p_data = (0x2F << 16);

	return true;
}

static int mmio_to_ring_id(unsigned int reg)
{
	int ring_id;

	switch (reg) {
	case _REG_RCS_PP_DIR_BASE_IVB:
	case GFX_MODE_GEN7:
	case _REG_RCS_EXECLIST_SUBMITPORT:
	case _REG_RCS_EXECLIST_STATUS:
	case _REG_RCS_CTX_STATUS_PTR:
		ring_id = RING_BUFFER_RCS;
		break;
	case _REG_BCS_PP_DIR_BASE:
	case _REG_BCS_BLT_MODE_IVB:
	case _REG_BCS_EXECLIST_SUBMITPORT:
	case _REG_BCS_EXECLIST_STATUS:
	case _REG_BCS_CTX_STATUS_PTR:
		ring_id = RING_BUFFER_BCS;
		break;
	case _REG_VCS_PP_DIR_BASE:
	case _REG_VCS_MFX_MODE_IVB:
	case _REG_VCS_EXECLIST_SUBMITPORT:
	case _REG_VCS_EXECLIST_STATUS:
	case _REG_VCS_CTX_STATUS_PTR:
		ring_id = RING_BUFFER_VCS;
		break;
	case _REG_VECS_PP_DIR_BASE:
	case _REG_VEBOX_MODE:
	case _REG_VECS_EXECLIST_SUBMITPORT:
	case _REG_VECS_EXECLIST_STATUS:
	case _REG_VECS_CTX_STATUS_PTR:
		ring_id = RING_BUFFER_VECS;
		break;
	case _REG_VCS2_MFX_MODE_BDW:
	case _REG_VCS2_EXECLIST_SUBMITPORT:
	case _REG_VCS2_EXECLIST_STATUS:
	case _REG_VCS2_CTX_STATUS_PTR:
		ring_id = RING_BUFFER_VCS2;
		break;
	default:
		ring_id = RING_ID_INVALID;
		break;
	}

	return ring_id;
}

static bool pp_dir_base_read(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	int ring_id = mmio_to_ring_id(off);
	vgt_ring_ppgtt_t *v_info = &vgt->rb[ring_id].vring_ppgtt_info;

	if (ring_id == RING_ID_INVALID) {
		vgt_err("vGT(%d) pp_dir_base_read: invalid ring_id(-1), offset(%x)\n",
				vgt->vgt_id, off);
		return false;
	};

	*(u32 *)p_data = v_info->base;

	vgt_dbg(VGT_DBG_RENDER, "<ring-%d>PP_DIR_BASE read: 0x%x\n", ring_id, v_info->base);
	return true;
}

static bool pp_dir_base_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 base = *(u32 *)p_data;
	int ring_id = mmio_to_ring_id(off);
	vgt_ring_ppgtt_t *v_info = &vgt->rb[ring_id].vring_ppgtt_info;
	vgt_ring_ppgtt_t *s_info = &vgt->rb[ring_id].sring_ppgtt_info;

	if (ring_id == RING_ID_INVALID) {
		vgt_err("vGT(%d) pp_dir_base_write: invalid ring_id(-1), offset(%x)\n",
				vgt->vgt_id, off);
		return false;
	};

	vgt_dbg(VGT_DBG_RENDER, "<ring-%d> PP_DIR_BASE write: 0x%x\n", ring_id, base);

	/* convert base which is in form of bit 31-16 in 64bytes cachelines,
	 * it turns out to be ((((base >> 16) * 64) >> 2) << PAGE_SHIFT), which
	 * is just base. */
	v_info->base = base;
	if (mmio_g2h_gmadr(vgt, off, v_info->base, &(s_info->base)) < 0) {
		vgt_err("vGT(%d): Fail to conver graphics memory(0x%x), with value(%x)\n",
				vgt->vgt_id, off, v_info->base);
		return false;
	}

	__vreg(vgt, off) = base;
	__sreg(vgt, off) = s_info->base;

	vgt->rb[ring_id].has_ppgtt_base_set = 1;

	return gen7_ppgtt_mm_setup(vgt, ring_id);
}

static bool pp_dclv_read(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	*(u32 *)p_data = 0xFFFFFFFF;
	return true;
}

static bool pp_dclv_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 dclv = *(u32 *)p_data;
	__vreg(vgt, off) = dclv;
	__sreg(vgt, off) = dclv;

	/* TODO: forward to pReg? */
	vgt_dbg(VGT_DBG_RENDER, "PP_DCLV write: 0x%x\n", dclv);
	return true;
}

/* TODO: there are other mode control bits in the registers */
static bool ring_pp_mode_read(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	int ring_id = mmio_to_ring_id(off);
	vgt_ring_ppgtt_t *v_info = &vgt->rb[ring_id].vring_ppgtt_info;

	if (ring_id == RING_ID_INVALID) {
		vgt_err("vGT(%d) ring_pp_mode_read: invalid ring_id(-1), offset(%x)\n",
				vgt->vgt_id, off);
		return false;
	};

	*(u32 *)p_data = v_info->mode;
	vgt_dbg(VGT_DBG_RENDER, "<ring-%d>GFX_MODE read: 0x%x\n", ring_id, v_info->mode);
	return true;
}

static bool ring_pp_mode_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 mode = *(u32 *)p_data;
	int ring_id = mmio_to_ring_id(off);
	if (ring_id == RING_ID_INVALID) {
		vgt_err("vGT(%d) ring_pp_mode_write: invalid ring_id(-1), offset(%x)\n",
				vgt->vgt_id, off);
		return false;
	};

	vgt_dbg(VGT_DBG_RENDER, "<ring-%d>GFX_MODE write: 0x%x\n", ring_id, mode);

	if (ring_id == RING_BUFFER_VECS)
		vgt->vebox_support = 1;

	/* check if guest is trying to enable GuC */
	if (GFX_MODE_BIT_SET_IN_MASK(mode, GFX_INTERRUPT_STEERING)) {
		WARN_ONCE(1, "VM(%d): should send interrupt message to display engine instead of on-chip micro controller.\n",
				vgt->vm_id);
		return true;
	}

	/* check for execlist */
	if (GFX_MODE_BIT_SET_IN_MASK(mode, _REGBIT_EXECLIST_ENABLE)) {
		bool ring_execlist = !!(mode & _REGBIT_EXECLIST_ENABLE);

		/* execlist mode is enabled if anyone wants execlist mode*/
		if (ring_execlist)
			vgt->pdev->enable_execlist = true;

		vgt->rb[ring_id].has_execlist_enabled = ring_execlist;
		vgt_info("EXECLIST %s on ring %d.\n",
			(ring_execlist ? "enabling" : "disabling"), ring_id);

		if (ring_execlist)
			vgt_enable_ring(vgt, ring_id);
	}

	ring_ppgtt_mode(vgt, ring_id, off, mode);
	return true;
}
static bool dma_ctrl_write(struct vgt_device *vgt, unsigned int off,
			void *p_data, unsigned int bytes)
{
	u32 mode = *(u32 *)p_data;

	if (GFX_MODE_BIT_SET_IN_MASK(mode, START_DMA)) {
		WARN_ONCE(1, "VM(%d): Guest is trying to enable GuC which is not supported by iGVT-g\n", vgt->vm_id);
		return true;
	}

	return true;
}
static bool dpy_trans_ddi_ctl_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	uint32_t new_data;
	uint32_t old_data;
	int i;

	/* force to use panel fitting path for eDP for HSW,
	 * it's no need for BDW as the panel fitter for pipe A
	 * is now also in the always-on power well.
	 */
	if (IS_HSW(vgt->pdev) &&
		enable_panel_fitting &&
		is_current_display_owner(vgt) &&
		offset == TRANS_DDI_FUNC_CTL_EDP &&
		PIPE_A  == get_edp_input(*((uint32_t *)p_data))) {
		*((uint32_t *)p_data) |= TRANS_DDI_EDP_INPUT_A_ONOFF;
		vgt_set_power_well(vgt, true);
	}

	old_data = __vreg(vgt, offset);
	default_mmio_write(vgt, offset, p_data, bytes);

	new_data = *((uint32_t *)p_data);

	/* if it is to enable this pipe, then rebuild the mapping for this pipe*/
	if (is_current_display_owner(vgt)) {
		/*when dom0 change the physical pipe/port connection,
		we need to rebuild pipe mapping for the vgt device.*/
		for (i = 0; i < VGT_MAX_VMS; ++ i) {
			struct vgt_device *vgt_virtual = vgt->pdev->device[i];
			if (!vgt_virtual || vgt_virtual->vm_id == 0)
				continue;
			update_pipe_mapping(vgt_virtual, offset, new_data);
		}

	} else {
		rebuild_pipe_mapping(vgt,  offset, new_data, old_data);
	}

	return true;
}

int vgt_surf_base_range_check(struct vgt_device *vgt,
	enum pipe pipe, enum vgt_plane_type plane, vgt_reg_t *surf_base)
{
	uint32_t  reg = _REG_INVALID;
	uint32_t  range;
	struct vgt_primary_plane_format primary_plane;
	struct vgt_sprite_plane_format  sprite_plane;
	struct vgt_cursor_plane_format  cursor_plane;

	if (!vgt_has_pipe_enabled(vgt, pipe)) {
		*surf_base = 0;
		return 0;
	}

	switch (plane)
	{
	case PRIMARY_PLANE:
		vgt_decode_primary_plane_format(vgt, pipe, &primary_plane);
		if (primary_plane.enabled){
			reg = VGT_DSPSURF(pipe);
			range = primary_plane.stride * primary_plane.height;
		}
		break;

	case SPRITE_PLANE:
		vgt_decode_sprite_plane_format(vgt, pipe, &sprite_plane);
		if (sprite_plane.enabled){
			reg = VGT_SPRSURF(pipe);
			range = sprite_plane.width* sprite_plane.height*
					(sprite_plane.bpp / 8);
		}
		break;

	case CURSOR_PLANE:
		vgt_decode_cursor_plane_format(vgt, pipe, &cursor_plane);
		if (cursor_plane.enabled) {
			reg = VGT_CURBASE(pipe);
			range = cursor_plane.width * cursor_plane.height *
					(cursor_plane.bpp / 8);
		}
		break;

	default:
		break;
	}

	if (reg != _REG_INVALID){
		reg_aux_addr_size(vgt->pdev, reg) = range;
		return mmio_g2h_gmadr(vgt, reg, __vreg(vgt, reg) , surf_base);
	}

	return 0;
}

static bool pipe_conf_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	bool rc, orig_pipe_enabled, curr_pipe_enabled;
	unsigned int reg;
	enum pipe pipe;
	enum vgt_plane_type plane;
	uint32_t wr_data;
	vgt_reg_t ret_val;

	reg = offset & ~(bytes - 1);

	wr_data = *((uint32_t *)p_data);
	/* vreg status will be updated when when read hardware status */
	if (!reg_hw_access(vgt, reg)) {
		if (wr_data & _REGBIT_PIPE_ENABLE)
			wr_data |= _REGBIT_PIPE_STAT_ENABLED;
		else if (!(wr_data & _REGBIT_PIPE_ENABLE))
			wr_data &= ~_REGBIT_PIPE_STAT_ENABLED;
	}

	if (offset == _REG_PIPE_EDP_CONF) {
		vgt_reg_t ctl_edp;
		ctl_edp = __vreg(vgt, TRANS_DDI_FUNC_CTL_EDP);
		pipe = get_edp_input(ctl_edp);
	} else {
		pipe = VGT_PIPECONFPIPE(offset);
	}
	orig_pipe_enabled = vgt_has_pipe_enabled(vgt, pipe);
	rc = default_mmio_write(vgt, offset, &wr_data, bytes);
	curr_pipe_enabled = vgt_has_pipe_enabled(vgt, pipe);

	if (offset == _REG_PIPE_EDP_CONF) {
		if (!curr_pipe_enabled)
			pipe = I915_MAX_PIPES;
	}

	if (!orig_pipe_enabled && curr_pipe_enabled) {
		if (pipe == I915_MAX_PIPES) {
			vgt_err("VM(%d): eDP pipe does not have corresponding"
				"mapped pipe while it is enabled!\n", vgt->vm_id);
			return false;
		}

		for (plane = PRIMARY_PLANE; plane < MAX_PLANE; plane++) {
			rc &= !vgt_surf_base_range_check(vgt, pipe, plane, &ret_val);
		}
	}

	if (rc)
		rc = vgt_manage_emul_dpy_events(vgt->pdev);

	return rc;
}

static bool ddi_buf_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	bool rc;
	vgt_reg_t reg_val;

	reg_val = *(vgt_reg_t *)p_data;

	// set the fully virtualized RO bit with its original value
	reg_val = (reg_val & ~_DDI_BUFCTL_DETECT_MASK)
		| (__vreg(vgt, offset) & _DDI_BUFCTL_DETECT_MASK);

	rc = default_mmio_write(vgt, offset, &reg_val, bytes);

	//update idle status when enable/disable DDI buf
	if (!reg_hw_access(vgt, offset)) {
		reg_val = __vreg(vgt, offset);

		if (reg_val & _REGBIT_DDI_BUF_ENABLE)
			reg_val &= ~_REGBIT_DDI_BUF_IS_IDLE;
		else
			reg_val |= _REGBIT_DDI_BUF_IS_IDLE;

		__vreg(vgt, offset) = reg_val;
	}

	// clear the auto_training done bit
	if ((offset == _REG_DDI_BUF_CTL_E) &&
		(!(reg_val & _REGBIT_DDI_BUF_ENABLE))) {
		if (!reg_hw_access(vgt, offset)) {
			__vreg(vgt, _REG_DP_TP_STATUS_E) &=
				~DP_TP_STATUS_AUTOTRAIN_DONE;
		}
	}

	return rc;
}

static bool fdi_rx_iir_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int reg;
	vgt_reg_t wr_data, old_iir;
	bool rc;

	reg = offset & ~(bytes -1);

	wr_data = *(vgt_reg_t *)p_data;
	old_iir = __vreg(vgt, reg);

	rc = default_mmio_write(vgt, offset, p_data, bytes);

	/* FIXME: sreg will be updated only when reading hardware status happened,
	 * so when dumping sreg space, the "hardware status" related bits may not
	 * be trusted */
	if (!reg_hw_access(vgt, reg))
		__vreg(vgt, reg) = old_iir ^ wr_data;

	return rc;
}

#define FDI_LINK_TRAIN_PATTERN1		0
#define FDI_LINK_TRAIN_PATTERN2		1

static bool fdi_auto_training_started(struct vgt_device *vgt)
{
	bool rc = false;
	vgt_reg_t ddi_buf_ctl = __vreg(vgt, _REG_DDI_BUF_CTL_E);
	vgt_reg_t rx_ctl = __vreg(vgt, _FDI_RXA_CTL);
	vgt_reg_t tx_ctl = __vreg(vgt, _REG_DP_TP_CTL_E);

	if ((ddi_buf_ctl & _REGBIT_DDI_BUF_ENABLE) &&
		(rx_ctl & FDI_RX_ENABLE) &&
		(rx_ctl & _REGBIT_FDI_RX_FDI_AUTO_TRAIN_ENABLE) &&
		(tx_ctl & DP_TP_CTL_ENABLE) &&
		(tx_ctl & _REGBIT_DP_TP_FDI_AUTO_TRAIN_ENABLE)) {
			rc = true;
	}

	return rc;
}

/* FIXME: this function is highly platform-dependent (SNB + CPT) */
static bool check_fdi_rx_train_status(struct vgt_device *vgt,
		enum pipe pipe, unsigned int train_pattern)
{
	unsigned int fdi_rx_imr, fdi_tx_ctl, fdi_rx_ctl;
	unsigned int fdi_rx_check_bits, fdi_tx_check_bits;
	unsigned int fdi_rx_train_bits, fdi_tx_train_bits;
	unsigned int fdi_iir_check_bits;

	fdi_rx_imr = VGT_FDI_RX_IMR(pipe);
	fdi_tx_ctl = VGT_FDI_TX_CTL(pipe);
	fdi_rx_ctl = VGT_FDI_RX_CTL(pipe);

	if (train_pattern == FDI_LINK_TRAIN_PATTERN1) {
		fdi_rx_train_bits =FDI_LINK_TRAIN_PATTERN_1_CPT;
		fdi_tx_train_bits = FDI_LINK_TRAIN_PATTERN_1;
		fdi_iir_check_bits = _REGBIT_FDI_RX_BIT_LOCK;
	} else if (train_pattern == FDI_LINK_TRAIN_PATTERN2) {
		fdi_rx_train_bits = FDI_LINK_TRAIN_PATTERN_2_CPT;
		fdi_tx_train_bits = FDI_LINK_TRAIN_PATTERN_2;
		fdi_iir_check_bits = _REGBIT_FDI_RX_SYMBOL_LOCK;
	} else {
		BUG();
	}

	fdi_rx_check_bits = FDI_RX_ENABLE | fdi_rx_train_bits;
	fdi_tx_check_bits = _REGBIT_FDI_TX_ENABLE | fdi_tx_train_bits;

	/* If imr bit not been masked */
	if (((__vreg(vgt, fdi_rx_imr) & fdi_iir_check_bits) == 0)
		&& ((__vreg(vgt, fdi_tx_ctl)
			& fdi_tx_check_bits) == fdi_tx_check_bits)
		&& ((__vreg(vgt, fdi_rx_ctl)
			& fdi_rx_check_bits) == fdi_rx_check_bits))
		return true;
	else
		return false;
}

static bool update_fdi_rx_iir_status(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	enum pipe pipe;
	unsigned int reg, fdi_rx_iir;
	bool rc;

	reg = offset & ~(bytes - 1);

	switch (offset) {
		case _FDI_RXA_CTL:
		case _FDI_TXA_CTL:
		case _FDI_RXA_IMR:
			pipe = PIPE_A;
			break;

		case _FDI_RXB_CTL:
		case _FDI_TXB_CTL:
		case _FDI_RXB_IMR:
			pipe = PIPE_B;
			break;

		case _REG_FDI_RXC_CTL:
		case _REG_FDI_TXC_CTL:
		case _REG_FDI_RXC_IMR:
			pipe = PIPE_C;
			break;

		default:
			BUG();
	}

	fdi_rx_iir = VGT_FDI_RX_IIR(pipe);

	rc = default_mmio_write(vgt, offset, p_data, bytes);
	if (!reg_hw_access(vgt, reg)) {
		if (check_fdi_rx_train_status(vgt, pipe, FDI_LINK_TRAIN_PATTERN1))
			__vreg(vgt, fdi_rx_iir) |= _REGBIT_FDI_RX_BIT_LOCK;
		if (check_fdi_rx_train_status(vgt, pipe, FDI_LINK_TRAIN_PATTERN2))
			__vreg(vgt, fdi_rx_iir) |= _REGBIT_FDI_RX_SYMBOL_LOCK;
		if (offset == _FDI_RXA_CTL) {
			if (fdi_auto_training_started(vgt))
				__vreg(vgt, _REG_DP_TP_STATUS_E) |=
					DP_TP_STATUS_AUTOTRAIN_DONE;
		}
	}
	return rc;
}

#define DP_TP_CTL_10_8_MASK	0x00000700
#define DP_TP_CTL_8_SHIFT	0x8
#define DP_TP_STATUS_25_SHIFT	25

static bool dp_tp_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	enum port port;
	unsigned int dp_tp_status_reg, val;
	vgt_reg_t ctl_val;
	bool rc;

	rc = default_mmio_write(vgt, offset, p_data, bytes);

	if (!reg_hw_access(vgt, offset)) {
		port = DP_TP_PORT(offset);
		ctl_val = __vreg(vgt, offset);
		val = (ctl_val & DP_TP_CTL_10_8_MASK) >> DP_TP_CTL_8_SHIFT;

		if (val == 0x2) {
			dp_tp_status_reg = DP_TP_STATUS(port);
			__vreg(vgt, dp_tp_status_reg) |= (1 << DP_TP_STATUS_25_SHIFT);
			__sreg(vgt, dp_tp_status_reg) = __vreg(vgt, dp_tp_status_reg);
		}
	}

	return rc;
}

#define BIT_27		27
#define BIT_26		26
#define BIT_24		24

static bool dp_tp_status_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	bool rc = true;
	vgt_reg_t reg_val;
	vgt_reg_t sticky_mask;

	reg_val = *((vgt_reg_t *)p_data);
	sticky_mask = (1 << BIT_27) | (1 << BIT_26) | (1 << BIT_24);

	__vreg(vgt, offset) = (reg_val & ~sticky_mask) |
				(__vreg(vgt, offset) & sticky_mask);
	__vreg(vgt, offset) &= ~(reg_val & sticky_mask);

	__sreg(vgt, offset) = reg_val;

	if (reg_hw_access(vgt, offset)) {
		VGT_MMIO_WRITE(vgt->pdev, offset, reg_val);
	}

	return rc;
}

static bool pch_adpa_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	vgt_reg_t old, new;

	new = *(vgt_reg_t *)p_data;
	old = __vreg(vgt, offset);

	/* Clear the bits of 'force hotplug trigger' and status because they
	 * will be fully virtualized. Other bits will be written to hardware.
	 */
	default_mmio_write(vgt, offset, p_data, bytes);

	if (reg_hw_access(vgt, offset))
		return true;

	if (new & ADPA_CRT_HOTPLUG_FORCE_TRIGGER) {

		if ((new & _REGBIT_ADPA_DAC_ENABLE)) {
			vgt_warn("HOTPLUG_FORCE_TRIGGER is set while VGA is enabled!\n");
		}

		/* emulate the status based on monitor connection information */
		new &= ~ADPA_CRT_HOTPLUG_FORCE_TRIGGER;

		if (dpy_has_monitor_on_port(vgt, PORT_E))
			new |= ADPA_CRT_HOTPLUG_MONITOR_MASK;
		else
			new &= ~ADPA_CRT_HOTPLUG_MONITOR_MASK;
	} else {
		/* ignore the status bits in new value
		 * since they are read only actually
		 */
		new = (new & ~ADPA_CRT_HOTPLUG_MONITOR_MASK) |
			(old & ADPA_CRT_HOTPLUG_MONITOR_MASK);
	}

	__vreg(vgt, offset) = __sreg(vgt, offset) = new;

	return true;
}

bool inline vgt_legacy_map_plane_reg(struct vgt_device *vgt, unsigned int reg, unsigned int *p_real_reg)
{
	enum pipe virtual_pipe;
	enum pipe real_pipe;

	switch (reg)
	{
	case _CURABASE:
	case _CURACNTR:
	case _CURAPOS:
	case _DSPACNTR:
	case _DSPASURF:
	case _DSPASURFLIVE:
	case _DSPAADDR:
	case _DSPASTRIDE:
	case _DSPAPOS:
	case _DSPASIZE:
	case _DSPATILEOFF:
	case _SPRA_SURF:
	case _SPRA_CTL:
	case _PIPEASRC:
		real_pipe = vgt->pipe_mapping[0];
		virtual_pipe = PIPE_A;
		break;

	case _CURBBASE:
	case _CURBCNTR:
	case _CURBPOS:
	case _CURBBASE_IVB:
	case _CURBCNTR_IVB:
	case _CURBPOS_IVB:
	case _REG_DSPBCNTR:
	case _REG_DSPBSURF:
	case _REG_DSPBSURFLIVE:
	case _REG_DSPBLINOFF:
	case _REG_DSPBSTRIDE:
	case _REG_DSPBPOS:
	case _REG_DSPBSIZE:
	case _REG_DSPBTILEOFF:
	case _PLANE_SURF_2_B:
	case _PLANE_CTL_2_B:
	case _PIPEBSRC:
		real_pipe = vgt->pipe_mapping[1];
		virtual_pipe = PIPE_B;
		break;

	case _REG_CURCBASE:
	case _REG_CURCNTR:
	case _REG_CURCPOS:
	case _DVSACNTR:
	case _DVSASURF:
	case _DVSASURFLIVE:
	case _DVSALINOFF:
	case _DVSASTRIDE:
	case _DVSAPOS:
	case _DVSASIZE:
	case _DVSATILEOFF:
	case _REG_SPRCSURF:
	case _REG_SPRC_CTL:
	case _REG_PIPECSRC:
		real_pipe = vgt->pipe_mapping[2];
		virtual_pipe = PIPE_C;
		break;

	default:
		vgt_warn("try to map mmio that is not plane related! reg = %x\n", reg);
		return false;
	}

	if(real_pipe == I915_MAX_PIPES)
	{
		vgt_dbg(VGT_DBG_DPY, "the mapping for pipe %d is not ready or created!\n", virtual_pipe);
		return false;
	}

	*p_real_reg = reg + 0x1000 * real_pipe - 0x1000 * virtual_pipe;

	return true;

}

bool inline vgt_skl_map_plane_reg(struct vgt_device *vgt,
	unsigned int reg, unsigned int *p_real_reg)
{
	enum pipe virtual_pipe;
	enum pipe real_pipe;

	if (reg >= PIPE_WM_LINETIME(PIPE_A) && reg <= PIPE_WM_LINETIME(PIPE_C)) {
		virtual_pipe = (reg - PIPE_WM_LINETIME(PIPE_A)) / 4;
	} else {
		virtual_pipe = (reg >> 12) & 0xf;
		if (virtual_pipe > I915_MAX_PIPES) {
			vgt_warn("try to map invalid plane mmio, reg: %x\n", reg);
			ASSERT(0);
		}
	}

	real_pipe = vgt->pipe_mapping[virtual_pipe];
	if(real_pipe == I915_MAX_PIPES) {
		vgt_err("the mapping for pipe %d is not ready or created!\n", virtual_pipe);
		return false;
	}

	if (reg >= PIPE_WM_LINETIME(PIPE_A) && reg <= PIPE_WM_LINETIME(PIPE_C)) {
		*p_real_reg = reg + 4 * real_pipe - 4 * virtual_pipe;
	} else {
		*p_real_reg = reg + 0x1000 * real_pipe - 0x1000 * virtual_pipe;
	}
	return true;
}

bool vgt_map_plane_reg(struct vgt_device *vgt, unsigned int reg, unsigned int *p_real_reg)
{
	if (IS_SKLPLUS(vgt->pdev))
		return vgt_skl_map_plane_reg(vgt, reg, p_real_reg);
	else
		return vgt_legacy_map_plane_reg(vgt, reg, p_real_reg);

	return false;
}

static bool dpy_plane_mmio_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{

	*(vgt_reg_t *)p_data = __vreg(vgt, offset);

	return true;
}

static bool dpy_plane_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int real_offset;

	memcpy ((char *)vgt->state.vReg + offset, p_data, bytes);
	memcpy ((char *)vgt->state.sReg + offset, p_data, bytes);
	if (current_foreground_vm(vgt->pdev) == vgt &&
		vgt_map_plane_reg(vgt, offset, &real_offset)) {
		VGT_MMIO_WRITE(vgt->pdev, real_offset, __sreg(vgt, offset));
	}

	return true;
}

static bool dpy_plane_ctl_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	enum pipe pipe = PIPE_A;
	enum pipe p_pipe = I915_MAX_PIPES;
	enum pipe v_pipe = I915_MAX_PIPES;
	vgt_reg_t new_plane_ctl;
	bool enable_plane = false;
	struct vgt_device *foreground_vgt;
	vgt_reg_t ret_val;
	int i;
	bool rc = true;

	new_plane_ctl = *(vgt_reg_t *)p_data;
	pipe = VGT_DSPCNTRPIPE(offset);
	if ( (DISPLAY_PLANE_ENABLE & new_plane_ctl) &&  (DISPLAY_PLANE_ENABLE & __vreg(vgt, offset)) == 0) {
		enable_plane = true;
	}

	dpy_plane_mmio_write(vgt, offset, p_data, bytes);
	if (enable_plane) {
		if (current_foreground_vm(vgt->pdev) == vgt) {
			set_panel_fitting(vgt, pipe);
		} else if (is_current_display_owner(vgt)) {
			p_pipe = vgt->pipe_mapping[pipe];
			foreground_vgt = current_foreground_vm(vgt->pdev);
			for (i = 0; i < I915_MAX_PIPES; i++) {
				if (foreground_vgt->pipe_mapping[i] == p_pipe) {
					v_pipe = i;
					break;
				}
			}
			if (p_pipe != I915_MAX_PIPES && v_pipe != I915_MAX_PIPES) {
				set_panel_fitting(foreground_vgt, v_pipe);
			}
		}
		rc &= !vgt_surf_base_range_check(vgt, pipe, PRIMARY_PLANE, &ret_val);
	}

	return rc;
}


static bool pri_surf_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	struct fb_notify_msg msg;
	enum pipe pipe = VGT_DSPSURFPIPE(offset);
	unsigned int real_offset;
	vgt_reg_t ret_val = 0;
	bool rc = true;

	__vreg(vgt, offset) = *(vgt_reg_t*)p_data;
	rc &= !vgt_surf_base_range_check(vgt, pipe, PRIMARY_PLANE, &ret_val);
	__sreg(vgt, offset) = ret_val ? ret_val : __vreg(vgt, offset);

	__vreg(vgt, VGT_PIPE_FLIPCOUNT(pipe))++;

	if (current_foreground_vm(vgt->pdev) == vgt &&
		vgt_map_plane_reg(vgt, offset, &real_offset)) {
		VGT_MMIO_WRITE(vgt->pdev, real_offset, __sreg(vgt, offset));
	}

	msg.vm_id = vgt->vm_id;
	msg.plane_id = PRIMARY_PLANE;
	msg.pipe_id = VGT_DSPSURFPIPE(offset);
	vgt_fb_notifier_call_chain(FB_DISPLAY_FLIP, &msg);

	vgt_inject_flip_done(vgt, VGT_DSPSURFPIPE(offset), PRIMARY_PLANE);

	return rc;
}

static bool sprite_plane_ctl_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	enum pipe pipe = VGT_SPRCNTRPIPE(offset);
	vgt_reg_t ret_val;
	bool rc = true;

	dpy_plane_mmio_write(vgt, offset, p_data, bytes);
	rc &= !vgt_surf_base_range_check(vgt, pipe, SPRITE_PLANE, &ret_val);

	return rc;
}

static bool spr_surf_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	struct fb_notify_msg msg;
	enum pipe pipe = VGT_SPRSURFPIPE(offset);
	unsigned int real_offset;
	vgt_reg_t ret_val = 0;
	bool rc = true;

	__vreg(vgt, offset) = *(vgt_reg_t*)p_data;
	rc &= !vgt_surf_base_range_check(vgt, pipe, SPRITE_PLANE, &ret_val);
	__sreg(vgt, offset) = ret_val ? ret_val : __vreg(vgt, offset);

	if (current_foreground_vm(vgt->pdev) == vgt &&
		vgt_map_plane_reg(vgt, offset, &real_offset)) {
		VGT_MMIO_WRITE(vgt->pdev, real_offset, __sreg(vgt, offset));
	}

	msg.vm_id = vgt->vm_id;
	msg.plane_id = SPRITE_PLANE;
	msg.pipe_id = VGT_SPRSURFPIPE(offset);
	vgt_fb_notifier_call_chain(FB_DISPLAY_FLIP, &msg);

	vgt_inject_flip_done(vgt, VGT_DSPSURFPIPE(offset), SPRITE_PLANE);

	return rc;
}

static bool cur_plane_ctl_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	enum pipe pipe = VGT_CURCNTRPIPE(offset);
	vgt_reg_t ret_val;
	bool rc = true;

	dpy_plane_mmio_write(vgt,offset, p_data, bytes);
	rc &= !vgt_surf_base_range_check(vgt, pipe, CURSOR_PLANE, &ret_val);

	return rc;
}

static bool cur_surf_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	enum pipe pipe = VGT_CURSURFPIPE(offset);
	unsigned int real_offset;
	vgt_reg_t ret_val = 0;
	bool rc = true;

	__vreg(vgt, offset) = *(vgt_reg_t*)p_data;
	rc &= !vgt_surf_base_range_check(vgt, pipe, CURSOR_PLANE, &ret_val);
	__sreg(vgt, offset) = ret_val ? ret_val : __vreg(vgt, offset);

	if (current_foreground_vm(vgt->pdev) == vgt &&
		vgt_map_plane_reg(vgt, offset, &real_offset)) {
		VGT_MMIO_WRITE(vgt->pdev, real_offset, __sreg(vgt, offset));
	}

	return rc;
}

static bool dpy_modeset_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc;

	rc = default_mmio_write(vgt, offset, p_data, bytes);

	if (!reg_hw_access(vgt, offset) &&
		(*(vgt_reg_t *)p_data != __vreg(vgt, offset))) {

		vgt_warn("modeset mmio[0x%x] change value from 0x%x to 0x%x\n"
			 "\twhich is not supported. MMIO write is ignored!\n",
						offset,
						__vreg(vgt, offset),
						*(vgt_reg_t *)p_data);
	}

	return true;
}

static bool south_chicken2_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	if (!default_mmio_write(vgt, offset, p_data, bytes))
		return false;

	if (!reg_hw_access(vgt, offset)) {
		if (__vreg(vgt, offset) & FDI_MPHY_IOSFSB_RESET_CTL)
			__vreg(vgt, offset) |= FDI_MPHY_IOSFSB_RESET_STATUS;
		else
			__vreg(vgt, offset) &= ~FDI_MPHY_IOSFSB_RESET_STATUS;

		__sreg(vgt, offset) = __vreg(vgt, offset);
	}

	return true;
}

static bool surflive_mmio_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes, enum vgt_plane_type plane)
{
	vgt_reg_t surflive_val;
	unsigned int surf_reg = 0;
	enum pipe pipe;

	if (plane == PRIMARY_PLANE) {
		pipe = VGT_DSPSURFLIVEPIPE(offset);
		surf_reg = VGT_DSPSURF(pipe);
	} else if (plane == CURSOR_PLANE) {
		if (offset == _REG_CURBSURFLIVE_SNB) {
			surf_reg = _CURBBASE;
		} else {
			pipe = VGT_CURSURFPIPE(offset);
			surf_reg = VGT_CURSURF(pipe);
		}
	} else if (plane == SPRITE_PLANE) {
		pipe = VGT_SPRSURFPIPE(offset);
		surf_reg = VGT_SPRSURF(pipe);
	} else {
		BUG();
	}

	surflive_val = __vreg(vgt, surf_reg);
	__vreg(vgt, offset) = __sreg(vgt, offset) = surflive_val;
	*(vgt_reg_t *)p_data = surflive_val;

	return true;
}

static bool pri_surflive_mmio_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	return surflive_mmio_read(vgt, offset, p_data, bytes, PRIMARY_PLANE);
}

static bool cur_surflive_mmio_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	return surflive_mmio_read(vgt, offset, p_data, bytes, CURSOR_PLANE);
}

static bool spr_surflive_mmio_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	return surflive_mmio_read(vgt, offset, p_data, bytes, SPRITE_PLANE);
}

static bool surflive_mmio_write (struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	/* surflive is readonly registers. ignore the write from driver*/
	return true;
}

static void dp_aux_ch_trigger_interrupt_on_done(struct vgt_device *vgt, vgt_reg_t value,
	 unsigned int reg)
{
	enum vgt_event_type event = EVENT_MAX;

	if (reg == DPA_AUX_CH_CTL) {
		event = AUX_CHANNEL_A;
	} else if (reg == PCH_DPB_AUX_CH_CTL
		|| reg == DPB_AUX_CH_CTL) {
		event = AUX_CHANNEL_B;
	} else if (reg == PCH_DPC_AUX_CH_CTL
		|| reg == DPC_AUX_CH_CTL) {
		event = AUX_CHANNEL_C;
	} else if (reg == PCH_DPD_AUX_CH_CTL
		|| reg == DPD_AUX_CH_CTL) {
		event = AUX_CHANNEL_D;
	}

	if (event != EVENT_MAX && (DP_AUX_CH_CTL_INTERRUPT & value)) {
		vgt_trigger_virtual_event(vgt, event);
	}
}

static void dp_aux_ch_ctl_trans_done(struct vgt_device *vgt, vgt_reg_t value,
	 unsigned int reg, int len, bool data_valid)
{
	/* mark transaction done */
	value |= _REGBIT_DP_AUX_CH_CTL_DONE;
	value &= ~_REGBIT_DP_AUX_CH_CTL_SEND_BUSY;
	value &= ~DP_AUX_CH_CTL_RECEIVE_ERROR;

	if (data_valid) {
		value &= ~DP_AUX_CH_CTL_TIME_OUT_ERROR;
	} else {
		value |= DP_AUX_CH_CTL_TIME_OUT_ERROR;
	}

	/* message size */
	value &= ~(0xf << 20);
	value |= (len << 20);
	__vreg(vgt, reg) = value;

	dp_aux_ch_trigger_interrupt_on_done(vgt, value, reg);
}

static void dp_aux_ch_ctl_link_training(struct vgt_dpcd_data *dpcd, uint8_t t)
{
	if ((t & DPCD_TRAINING_PATTERN_SET_MASK) == DPCD_TRAINING_PATTERN_1) {

		/* training pattern 1 for CR */
		/* set LANE0_CR_DONE, LANE1_CR_DONE */
		dpcd->data[DPCD_LANE0_1_STATUS] |= DPCD_LANES_CR_DONE;
		/* set LANE2_CR_DONE, LANE3_CR_DONE */
		dpcd->data[DPCD_LANE2_3_STATUS] |= DPCD_LANES_CR_DONE;

	} else if ((t & DPCD_TRAINING_PATTERN_SET_MASK) ==
		DPCD_TRAINING_PATTERN_2) {

		/* training pattern 2 for EQ */

		/* Set CHANNEL_EQ_DONE and  SYMBOL_LOCKED for Lane0_1 */
		dpcd->data[DPCD_LANE0_1_STATUS] |= DPCD_LANES_EQ_DONE;
		dpcd->data[DPCD_LANE0_1_STATUS] |= DPCD_SYMBOL_LOCKED;

		/* Set CHANNEL_EQ_DONE and  SYMBOL_LOCKED for Lane2_3 */
		dpcd->data[DPCD_LANE2_3_STATUS] |= DPCD_LANES_EQ_DONE;
		dpcd->data[DPCD_LANE2_3_STATUS] |= DPCD_SYMBOL_LOCKED;
		/* set INTERLANE_ALIGN_DONE */
		dpcd->data[DPCD_LANE_ALIGN_STATUS_UPDATED] |=
			DPCD_INTERLANE_ALIGN_DONE;

	} else if ((t & DPCD_TRAINING_PATTERN_SET_MASK) ==
		DPCD_LINK_TRAINING_DISABLED) {

		/* finish link training */
		/* set sink status as synchronized */
		dpcd->data[DPCD_SINK_STATUS] = DPCD_SINK_IN_SYNC;
	}

}

static bool dp_aux_ch_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int reg = 0;
	vgt_reg_t value = *(vgt_reg_t *)p_data;
	int msg, addr, ctrl, op, len;
	struct vgt_dpcd_data *dpcd = NULL;
	enum port port_idx = OFFSET_TO_DP_AUX_PORT(offset);
	struct gt_port *port = NULL;

	if ((bytes != 4) || ((offset & (bytes - 1)) != 0)) {
		vgt_err("vGT(%d) dp_aux_ch_ctl_mmio_write: invalid offset(%x) or bytes(%d)\n",
			vgt->vgt_id, offset, bytes);
		return false;
	};

	reg = offset & ~(bytes - 1);

	default_mmio_write(vgt, offset, p_data, bytes);

	/* HW access had been handled by default_mmio_write() */
	if (reg_hw_access(vgt, reg))
		return true;

	if (!dpy_is_valid_port(port_idx)) {
		vgt_warn("vGT(%d): Unsupported DP port access!\n",
				vgt->vgt_id);
		return true;
	}

	if (IS_SKL(vgt->pdev) && reg != _REG_SKL_DP_AUX_CH_CTL(port_idx)) {
		/* SKL DPB/C/D aux ctl register changed */
		return true;
	} else if (IS_PRESKL(vgt->pdev) && reg != _REG_HSW_DP_AUX_CH_CTL(port_idx)) {
		/* write to the data registers */
		return true;
	}

	if (!(value & _REGBIT_DP_AUX_CH_CTL_SEND_BUSY)) {
		/* just want to clear the sticky bits */
		__vreg(vgt, reg) = 0;
		return true;
	}

	port = &vgt->ports[port_idx];

	if (port) {
		dpcd = port->dpcd;
	}

	/* read out message from DATA1 register */
	msg = __vreg(vgt, reg + 4);
	addr = (msg >> 8) & 0xffff;
	ctrl = (msg >> 24) & 0xff;
	len = msg & 0xff;
	op = ctrl >> 4;

	if (op == VGT_AUX_NATIVE_WRITE) {
		int t;
		uint8_t buf[16];

		if ((addr + len + 1) >= DPCD_SIZE) {
			/*
			 * Write request exceeds what we supported,
			 * DCPD spec: When a Source Device is writing a DPCD
			 * address not supported by the Sink Device, the Sink
			 * Device shall reply with AUX NACK and “M” equal to zero.
			 */

			/* NAK the write */
			__vreg(vgt, reg + 4) = AUX_NATIVE_REPLY_NAK;

			dp_aux_ch_ctl_trans_done(vgt, value, reg, 2, true);

			return true;
		}

		/*
		 * Write request format: (command + address) occupies
		 * 3 bytes, followed by (len + 1) bytes of data.
		 */
		ASSERT((len + 4) <= AUX_BURST_SIZE);

		/* unpack data from vreg to buf */
		for (t = 0; t < 4; t ++) {
			vgt_reg_t r = __vreg(vgt, reg + 8 + t*4);

			buf[t*4] = (r >> 24) & 0xff;
			buf[t*4 + 1] = (r >> 16) & 0xff;
			buf[t*4 + 2] = (r >> 8) & 0xff;
			buf[t*4 + 3] = r & 0xff;
		}

		/* write to virtual DPCD */
		if (dpcd && dpcd->data_valid) {
			for (t = 0; t <= len; t ++) {
				int p = addr + t;

				dpcd->data[p] = buf[t];

				/* check for link training */
				if (p == DPCD_TRAINING_PATTERN_SET)
					dp_aux_ch_ctl_link_training(dpcd, buf[t]);
			}
		}

		/* ACK the write */
		__vreg(vgt, reg + 4) = 0;

		dp_aux_ch_ctl_trans_done(vgt, value, reg, 1, dpcd && dpcd->data_valid);

		return true;
	}

	if (op == VGT_AUX_NATIVE_READ) {
		int idx, i, ret = 0;

		if ((addr + len + 1) >= DPCD_SIZE) {
			/*
			 * read request exceeds what we supported
			 * DPCD spec: A Sink Device receiving a Native AUX CH
			 * read request for an unsupported DPCD address must
			 * reply with an AUX ACK and read data set equal to
			 * zero instead of replying with AUX NACK.
			 */

			/* ACK the READ*/
			__vreg(vgt, reg + 4) = 0;
			__vreg(vgt, reg + 8) = 0;
			__vreg(vgt, reg + 12) = 0;
			__vreg(vgt, reg + 16) = 0;
			__vreg(vgt, reg + 20) = 0;

			dp_aux_ch_ctl_trans_done(vgt ,value, reg, len + 2, true);

			return true;
		}

		for (idx = 1; idx <= 5; idx ++) {
			/* clear the data registers */
			__vreg(vgt, reg + 4 * idx) = 0;
		}

		/*
		 * Read reply format: ACK (1 byte) plus (len + 1) bytes of data.
		 */
		ASSERT((len + 2) <= AUX_BURST_SIZE);

		/* read from virtual DPCD to vreg */
		/* first 4 bytes: [ACK][addr][addr+1][addr+2] */
		if (dpcd && dpcd->data_valid) {
			for (i = 1; i <= (len + 1); i ++) {
				int t;

				t = dpcd->data[addr + i - 1];
				t <<= (24 - 8*(i%4));
				ret |= t;

				if ((i%4 == 3) || (i == (len + 1))) {
					__vreg(vgt, reg + (i/4 + 1)*4) = ret;
					ret = 0;
				}
			}
		}

		dp_aux_ch_ctl_trans_done(vgt, value, reg, len + 2, dpcd && dpcd->data_valid);

		return true;
	}

	/* i2c transaction starts */
	vgt_i2c_handle_aux_ch_write(vgt, port_idx, offset, p_data);

	dp_aux_ch_trigger_interrupt_on_done(vgt, value, reg);
	return true;
}

static bool vgt_dpy_stat_notify(struct vgt_device *vgt,
	enum vgt_uevent_type event)
{
	struct pgt_device *pdev = vgt->pdev;

	if (event < VGT_ENABLE_VGA || event > VGT_DISPLAY_UNREADY) {
		vgt_err("vGT(%d) vgt_dpy_stat_notify: invalid event(%d)\n",
				vgt->vgt_id, event);
		return false;
	}

	vgt_set_uevent(vgt, event);
	vgt_raise_request(pdev, VGT_REQUEST_UEVENT);
	return true;
}

static bool vga_control_r(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	return default_mmio_read(vgt, offset, p_data, bytes);
}

static bool vga_control_w (struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	enum vgt_uevent_type event;
	bool vga_disable;

	default_mmio_write(vgt, offset, p_data, bytes);

	vga_disable = __vreg(vgt, offset) & _REGBIT_VGA_DISPLAY_DISABLE;

	vgt_info("VM(%d): %s VGA mode %x\n", vgt->vgt_id,
		vga_disable ? "Disable" : "Enable",
		(unsigned int)__vreg(vgt, offset));

	event = vga_disable ? VGT_DISABLE_VGA : VGT_ENABLE_VGA;

	if (!vgt_dpy_stat_notify(vgt, event))
		return false;

	return true;
}

static bool err_int_r(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc = default_mmio_read(vgt, offset, p_data, bytes);
	return rc;
}

static bool err_int_w(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc = default_mmio_write(vgt, offset, p_data, bytes);
	return rc;
}

static void cache_sbi_reg_value(struct vgt_device *vgt, unsigned int sbi_offset,
	vgt_reg_t value)
{
	int i;
	int num = vgt->sbi_regs.number;

	for (i = 0; i < num; ++ i) {
		if (vgt->sbi_regs.registers[i].offset == sbi_offset)
			break;
	}

	if (i == num) {
		if (num < SBI_REG_MAX) {
			vgt->sbi_regs.number++;
		} else {
			vgt_warn("vGT(%d): SBI caching meets maximum limits!\n",
				vgt->vgt_id);
			return;
		}
	}

	vgt->sbi_regs.registers[i].offset = sbi_offset;
	vgt->sbi_regs.registers[i].value = value;
}

static vgt_reg_t get_sbi_reg_cached_value(struct vgt_device *vgt,
	unsigned int sbi_offset)
{
	int i;
	int num = vgt->sbi_regs.number;
	vgt_reg_t value = 0;

	for (i = 0; i < num; ++i) {
		if (vgt->sbi_regs.registers[i].offset == sbi_offset)
			break;
	}

	if (i < num) {
		value = vgt->sbi_regs.registers[i].value;
	} else {
		cache_sbi_reg_value(vgt, sbi_offset, 0);
		vgt_dbg(VGT_DBG_DPY,
			"vGT(%d): SBI reading did not find the cached value"
			" for offset 0x%x. 0 will be returned!\n",
			vgt->vgt_id, sbi_offset);
	}

	return value;
}

static bool sbi_mmio_data_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc;

	rc = default_mmio_read(vgt, offset, p_data, bytes);

	if (!reg_hw_access(vgt, offset)) {
		if (((__vreg(vgt, SBI_CTL_STAT) & SBI_OPCODE_MASK) >>
			SBI_OPCODE_SHIFT) == SBI_CMD_CRRD) {
			unsigned int sbi_offset = (__vreg(vgt, SBI_ADDR) &
				SBI_ADDR_OFFSET_MASK) >> SBI_ADDR_OFFSET_SHIFT;
			vgt_reg_t val = get_sbi_reg_cached_value(vgt, sbi_offset);
			*(vgt_reg_t *)p_data = val;
		}
	}

	return rc;
}

static bool sbi_mmio_ctl_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc;

	rc = default_mmio_write(vgt, offset, p_data, bytes);

	if (!reg_hw_access(vgt, offset)) {
		vgt_reg_t data = __vreg(vgt, offset);

		data &= ~(SBI_STAT_MASK << SBI_STAT_SHIFT);
		data |= SBI_READY;

		data &= ~(SBI_RESPONSE_MASK << SBI_RESPONSE_SHIFT);
		data |= SBI_RESPONSE_SUCCESS;

		__vreg(vgt, offset) = data;

		if (((__vreg(vgt, SBI_CTL_STAT) & SBI_OPCODE_MASK) >>
			SBI_OPCODE_SHIFT) == SBI_CMD_CRWR) {
			unsigned int sbi_offset = (__vreg(vgt, SBI_ADDR) &
				SBI_ADDR_OFFSET_MASK) >> SBI_ADDR_OFFSET_SHIFT;
			vgt_reg_t val = __vreg(vgt, SBI_DATA);
			cache_sbi_reg_value(vgt, sbi_offset, val);
		}
	}

	return rc;
}

static bool pvinfo_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc = default_mmio_read(vgt, offset, p_data, bytes);
	bool invalid_read = false;

	switch (offset) {
		case vgt_info_off(magic) ... vgt_info_off(vgt_id):
			if (offset + bytes > vgt_info_off(vgt_id) + 4)
				invalid_read = true;
			break;

		case vgt_info_off(avail_rs.mappable_gmadr.base) ...
			vgt_info_off(avail_rs.fence_num):
			if (offset + bytes >
				vgt_info_off(avail_rs.fence_num) + 4)
				invalid_read = true;
			break;

		case vgt_info_off(drv_version_major) ...
			vgt_info_off(min_fence_num):
			if (offset + bytes > vgt_info_off(min_fence_num) + 4)
				invalid_read = true;
			break;
		case vgt_info_off(v2g_notify):
			/* set cursor setting here.  For example:
			 *   *((unsigned int *)p_data)) = VGT_V2G_SET_SW_CURSOR;
			 */
			break;
		case vgt_info_off(vgt_caps):
			break;
		default:
			invalid_read = true;
			break;
	}

	if (invalid_read)
		vgt_warn("invalid pvinfo read: [%x:%x] = %x!!!\n",
			offset, bytes, *(vgt_reg_t *)p_data);

	return rc;
}

static void fb_notify_all_mapped_pipes(struct vgt_device *vgt,
	enum vgt_plane_type planeid)
{
	unsigned i;

	for (i = 0; i < I915_MAX_PIPES; i++) {
		if (vgt->pipe_mapping[i] != I915_MAX_PIPES) {
			struct fb_notify_msg msg;

			msg.vm_id = vgt->vm_id;
			msg.plane_id = planeid;
			msg.pipe_id = i;

			vgt_fb_notifier_call_chain(FB_DISPLAY_FLIP, &msg);
		}
	}
}

static bool pvinfo_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	vgt_reg_t val = *(vgt_reg_t *)p_data;
	vgt_reg_t min;
	bool rc = true;
	enum vgt_uevent_type event;

	switch (offset) {
		case vgt_info_off(min_low_gmadr):
			min = val;
			if (vgt->aperture_sz < min) {
				vgt_err("VM(%d): aperture size(%llx) is less than"
					"its driver's minimum requirement(%x)!\n",
					vgt->vm_id, vgt->aperture_sz, min);
				rc = false;
			}
			break;
		case vgt_info_off(min_high_gmadr):
			min = val;
			if (vgt->gm_sz - vgt->aperture_sz < min) {
				vgt_err("VM(%d): hiden gm size(%llx) is less than"
					"its driver's minimum requirement(%x)!\n",
					vgt->vm_id, vgt->gm_sz - vgt->aperture_sz,
				        min);
				rc = false;
			}
			break;
		case vgt_info_off(min_fence_num):
			min = val;
			if (vgt->fence_sz < min) {
				vgt_err("VM(%d): fence size(%x) is less than"
					"its drivers minimum requirement(%x)!\n",
					vgt->vm_id, vgt->fence_sz, min);
				rc = false;
			}
			break;
		case vgt_info_off(display_ready):
			switch (val) {
			case 0:
				event = VGT_DISPLAY_UNREADY;
				break;
			case 1:
				event = VGT_DISPLAY_READY;
				break;
			case 2:
				event = VGT_ENABLE_VGA;
				break;
			default:
				event = UEVENT_MAX;
				vgt_warn("invalid display event: %d\n", val);
				break;
			}

			if (event != UEVENT_MAX){
				rc = vgt_dpy_stat_notify(vgt, event);
				if (!rc)
					break;
			}

			if (vgt->vm_id && event == VGT_DISPLAY_READY
				&& hvm_boot_foreground == true
				&& !vgt->hvm_boot_foreground_visible) {
				/*
				 * Guest had a vaild surface to show.
				 */
				vgt->hvm_boot_foreground_visible = 1;
				vgt->pdev->next_foreground_vm = vgt;
				vgt_raise_request(vgt->pdev, VGT_REQUEST_DPY_SWITCH);
			}
			break;
		case vgt_info_off(g2v_notify):
			if (val == VGT_G2V_DISPLAY_REFRESH) {
				fb_notify_all_mapped_pipes(vgt, PRIMARY_PLANE);
			} else if (val == VGT_G2V_SET_POINTER_SHAPE) {
				struct fb_notify_msg msg;
				msg.vm_id = vgt->vm_id;
				msg.plane_id = CURSOR_PLANE;
				msg.pipe_id = 0;
				vgt_fb_notifier_call_chain(FB_DISPLAY_FLIP, &msg);
			} else if (val == VGT_G2V_PPGTT_L3_PAGE_TABLE_CREATE) {
				rc = vgt_g2v_create_ppgtt_mm(vgt, 3);
			} else if (val == VGT_G2V_PPGTT_L3_PAGE_TABLE_DESTROY) {
				rc = vgt_g2v_destroy_ppgtt_mm(vgt, 3);
			} else if (val == VGT_G2V_PPGTT_L4_PAGE_TABLE_CREATE) {
				rc = vgt_g2v_create_ppgtt_mm(vgt, 4);
			} else if (val == VGT_G2V_PPGTT_L4_PAGE_TABLE_DESTROY) {
				rc = vgt_g2v_destroy_ppgtt_mm(vgt, 4);
			} else if (val == VGT_G2V_EXECLIST_CONTEXT_ELEMENT_CREATE) {
				rc = vgt_g2v_execlist_context_create(vgt);
			} else if (val == VGT_G2V_EXECLIST_CONTEXT_ELEMENT_DESTROY) {
				rc = vgt_g2v_execlist_context_destroy(vgt);
			} else {
				vgt_warn("Invalid PV notification. %x\n", val);
			}
			break;
		case vgt_info_off(xhot):
		case vgt_info_off(yhot):
			{
				struct fb_notify_msg msg;
				msg.vm_id = vgt->vm_id;
				msg.plane_id = CURSOR_PLANE;
				msg.pipe_id = 0;
				vgt_fb_notifier_call_chain(FB_DISPLAY_FLIP, &msg);
			}
			break;

                case vgt_info_off(pdp0_lo):
                case vgt_info_off(pdp0_hi):
                case vgt_info_off(pdp1_lo):
                case vgt_info_off(pdp1_hi):
                case vgt_info_off(pdp2_lo):
                case vgt_info_off(pdp2_hi):
                case vgt_info_off(pdp3_lo):
                case vgt_info_off(pdp3_hi):
                case vgt_info_off(execlist_context_descriptor_lo):
                case vgt_info_off(execlist_context_descriptor_hi):
                        break;

		default:
			/* keep rc's default value: true.
			 * NOTE: returning false will crash the VM.
			 */
			vgt_warn("invalid pvinfo write: [%x:%x] = %x!!!\n",
				offset, bytes, val);
			break;
	}

	if (rc == true)
		 rc = default_mmio_write(vgt, offset, p_data, bytes);

	return rc;
}

static bool pf_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	if (enable_panel_fitting) {
		*(vgt_reg_t *)p_data = __vreg(vgt, offset);
	} else {
		default_mmio_read(vgt, offset, p_data, bytes);
	}

	return true;
}

static bool pf_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{

	u32 val = *(u32 *)p_data;

	if(offset == _PS_1A_CTRL ||
		offset == _PS_2A_CTRL ||
		offset == _PS_1B_CTRL ||
		offset == _PS_2B_CTRL ||
		offset == _PS_1C_CTRL) {

		if((val & PS_PLANE_SEL_MASK) != 0)
			WARN_ONCE(1, "VM(%d): guest is trying to scaling a plane\n",
					vgt->vm_id);
			return true;
	}

	if (enable_panel_fitting) {
		memcpy ((char *)vgt->state.vReg + offset, p_data, bytes);
	} else {
		default_mmio_write(vgt, offset, p_data, bytes);
	}

	return true;
}

static bool power_well_ctl_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	bool rc = true;
	vgt_reg_t data;
	if (is_current_display_owner(vgt)) {
		data = VGT_MMIO_READ(vgt->pdev, offset);
	} else {
		data = __vreg(vgt, offset);
	}

	if (IS_HSW(vgt->pdev) && enable_panel_fitting && offset == HSW_PWR_WELL_DRIVER) {
		data = __vreg(vgt, offset);
	}

	*(vgt_reg_t *)p_data = data;
	return rc;
}

static bool power_well_ctl_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc = true;
	vgt_reg_t value = *(vgt_reg_t *)p_data;

	memcpy ((char *)vgt->state.vReg + offset, p_data, bytes);

	if (value & HSW_PWR_WELL_ENABLE_REQUEST) {
		__vreg(vgt, offset) |= HSW_PWR_WELL_STATE_ENABLED;
	} else {
		__vreg(vgt, offset) &= ~HSW_PWR_WELL_STATE_ENABLED;
	}

	if (is_current_display_owner(vgt)) {
		/* force to enable power well physically */
		if (IS_HSW(vgt->pdev) && enable_panel_fitting && offset == HSW_PWR_WELL_DRIVER) {
			value |= HSW_PWR_WELL_ENABLE_REQUEST;
		}
		VGT_MMIO_WRITE(vgt->pdev, offset, value);
	}

	return rc;
}

static bool skl_power_well_ctl_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	u32 v = *(u32 *)p_data;

	ASSERT(bytes <= 4);

	if (is_current_display_owner(vgt))
		return default_mmio_write(vgt, offset, p_data, bytes);

	v &= (1 << 31) | (1 << 29) | (1 << 9) |
		(1 << 7) | (1 << 5) | (1 << 3) | (1 << 1);

	v |= (v >> 1);

	return default_mmio_write(vgt, offset, &v, bytes);
}

static bool ring_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	/* TODO
	 * We do not support the mix usage of RB mode and EXECLIST from
	 * different VMs. If that happens, VM with RB mode cannot have
	 * workload being submitted/executed correctly.
	 */
	if (vgt->pdev->enable_execlist)
		return default_mmio_read(vgt, offset, p_data, bytes);
	else
		return ring_mmio_read_in_rb_mode(vgt, offset, p_data, bytes);
}

static bool ring_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	if (vgt->pdev->enable_execlist) {
		int ring_id = tail_to_ring_id(vgt->pdev, _tail_reg_(offset));
		if (!vgt->rb[ring_id].has_execlist_enabled) {
			vgt_err("VM(%d): Workload submission with ringbuffer "
			"mode is not allowed since system is in execlist mode. "
			"VM will be killed!\n", vgt->vm_id);

			return false;
		}
		return default_mmio_write(vgt, offset, p_data, bytes);
	} else {
		return ring_mmio_write_in_rb_mode(vgt, offset, p_data, bytes);
	}
}

static bool ring_uhptr_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	/* TODO
	 * Same as ring_mmio_read/write
	 */
	if (vgt->pdev->enable_execlist)
		return default_mmio_write(vgt, offset, p_data, bytes);
	else
		return ring_uhptr_write_in_rb_mode(vgt, offset, p_data, bytes);
}

static bool instpm_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_reg_t val = *(vgt_reg_t *)p_data;
	uint16_t val_low = val & 0xFFFF;
	uint16_t val_high = val >> 16;
	uint16_t old_val_low = __vreg16(vgt, offset);
	unsigned int bit;
	bool hw_access = reg_hw_access(vgt, offset);
	bool sync_flush = false;
	bool tlb_invd = false;
	bool warn_msg = false;

	__vreg16(vgt, offset) =  (old_val_low & ~val_high) |
		(val_low & val_high);

	for_each_set_bit(bit, (unsigned  long *)&val_high, 16) {
		bool enable = !!test_bit(bit, (void *)&val_low);

		switch (1 << bit)  {
		case  INSTPM_SYNC_FLUSH:
			sync_flush = enable;
			break;

		case INSTPM_FORCE_ORDERING:
			if (enable && offset != INSTPM)
				warn_msg = true;
			break;

		case  INSTPM_TLB_INVALIDATE:
			if (!enable)
				break;
			if (!sync_flush) {
				warn_msg = true;
				break;
			}
			tlb_invd = true;
			break;
		default:
			if (enable && !hw_access)
				warn_msg = true;
			break;
		}
	}

	if (warn_msg)
		vgt_warn("unknown INSTPM write: VM%d: off=0x%x, val=0x%x\n",
			vgt->vm_id, offset, val);

	if (hw_access || tlb_invd) {
		if (!hw_access && tlb_invd)
			__vreg(vgt, offset) = _MASKED_BIT_ENABLE(
				INSTPM_TLB_INVALIDATE |
				INSTPM_SYNC_FLUSH);

		VGT_MMIO_WRITE(pdev, offset, __vreg(vgt, offset));

		if (tlb_invd) {
			/*
			 * The time is usually 0.2ms for 3.11.6 Ubuntu guest.
			 * 3.8 Linux and Win don't use this to flush GPU tlb.
			 */
			if (wait_for_atomic((VGT_MMIO_READ(pdev, offset) &
				INSTPM_SYNC_FLUSH) == 0, 1))
				vgt_warn("INSTPM_TLB_INVALIDATE timed out!\n");
			__vreg16(vgt, offset) &= ~INSTPM_SYNC_FLUSH;
		}

	}

	__sreg(vgt, offset) = __vreg(vgt, offset);

	return true;
}

bool fpga_dbg_mmio_read(struct vgt_device *vgt, unsigned int reg,
        void *p_data, unsigned int bytes)
{
	bool rc;

	rc = default_mmio_read(vgt, reg, p_data, bytes);
	if (!rc)
		return false;

	if (vgt->vm_id == 0 && (__vreg(vgt, reg) & FPGA_DBG_RM_NOCLAIM)) {
		VGT_MMIO_WRITE(vgt->pdev, reg, FPGA_DBG_RM_NOCLAIM);

		__vreg(vgt, reg) &= ~FPGA_DBG_RM_NOCLAIM;
		__sreg(vgt, reg) = __vreg(vgt, reg);

		*(vgt_reg_t *)p_data &= ~FPGA_DBG_RM_NOCLAIM;
	}

	return true;
}

bool fpga_dbg_mmio_write(struct vgt_device *vgt, unsigned int reg,
	void *p_data, unsigned int bytes)
{
	vgt_reg_t v = *(vgt_reg_t *)p_data;

	vgt_warn("VM %d writes FPGA_DBG register: %x.\n", vgt->vm_id, v);

	if (vgt->vm_id == 0)
		return default_mmio_write(vgt, reg, p_data, bytes);
	else {
		__vreg(vgt, reg) &= ~v;
		__sreg(vgt, reg) = __vreg(vgt, reg);
	}

	return true;
}

static bool sfuse_strap_mmio_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc = default_mmio_read(vgt, offset, p_data, bytes);
	/*
	 * VM guest driver using SFUSE_STRAP to detect PORT_B/C/D,
	 * for indirect mode, we provide full PORT B,C,D capability to VM
	 */
	if (!propagate_monitor_to_guest && !is_current_display_owner(vgt)) {
		*(vgt_reg_t*)p_data |=  (SFUSE_STRAP_DDIB_DETECTED
			| SFUSE_STRAP_DDIC_DETECTED | SFUSE_STRAP_DDID_DETECTED);
	}
	return rc;
}

static bool vgt_write_submitport(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	bool rc = true;
	int ring_id = mmio_to_ring_id(offset);
	struct vgt_elsp_store *elsp_store = &vgt->rb[ring_id].elsp_store;
	if (ring_id == RING_ID_INVALID) {
		vgt_err("vGT(%d) vgt_write_submitport: invalid ring_id(-1), offset(%x)\n",
				vgt->vgt_id, offset);
		return false;
	};

	ASSERT((bytes == 4) && ((offset & 3) == 0));
	ASSERT(elsp_store->count < ELSP_BUNDLE_NUM);

	elsp_store->element[elsp_store->count] = *(vgt_reg_t *)p_data;
	elsp_store->count ++;
	vgt_dbg(VGT_DBG_EXECLIST,
		"VM(%d): MMIO write to virtual submitPort 0x%x with 0x%x\n",
			vgt->vm_id, offset, *(vgt_reg_t *)p_data);
	if (elsp_store->count == ELSP_BUNDLE_NUM) {
		rc = vgt_batch_ELSP_write(vgt, ring_id);
	}

	return rc;
}


static bool vgt_read_ctx_status_ptr(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	int ring_id = mmio_to_ring_id(offset);
	if (ring_id == RING_ID_INVALID) {
		vgt_err("vGT(%d) vgt_read_ctx_status_ptr: invalid ring_id(-1), offset(%x)\n",
				vgt->vgt_id, offset);
		return false;
	};

	if (ring_id >= vgt->pdev->max_engines)
		WARN_ONCE(1, "vGT(%d) accessing ring%d offset 0x%x not supported\n",
				vgt->vgt_id, ring_id, offset);
	else if (vgt == current_render_owner(vgt->pdev)) {
		/* update HW CSB status to guest if we are render owner
		 * this is to make sure that guest always can get latest HW status,
		 * even if we delay/did not send ctx switch events to guest.
		 */
		vgt_emulate_context_switch_event(vgt->pdev, ring_id);
	}

	return default_mmio_read(vgt, offset, p_data, bytes);
}

static bool vgt_write_ctx_status_ptr(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
#if 0
	int ring_id = mmio_to_ring_id(offset);
	uint32_t ctx_ptr_reg;
	struct ctx_st_ptr_format ctx_ptr_val;
	struct ctx_st_ptr_format* guest_ctx_st = (struct ctx_st_ptr_format*)p_data;

	ASSERT(bytes == 4);

	ctx_ptr_reg = el_ring_mmio(ring_id, _EL_OFFSET_STATUS_PTR);
	ctx_ptr_val.dw = __vreg(vgt, ctx_ptr_reg);

	/* Guest modify write_ptr as long as mask bits not zero */
	if ((guest_ctx_st->mask & _CTXBUF_WRITE_PTR_MASK) == _CTXBUF_WRITE_PTR_MASK) {
		ctx_ptr_val.status_buf_write_ptr = guest_ctx_st->status_buf_write_ptr;
	}

	/* Guest modify read_ptr as long as not zero */
	if ((guest_ctx_st->mask & _CTXBUF_READ_PTR_MASK) == _CTXBUF_READ_PTR_MASK) {
		ctx_ptr_val.status_buf_read_ptr = guest_ctx_st->status_buf_read_ptr;
	}

	/* update into vreg */
	guest_ctx_st->dw = ctx_ptr_val.dw;
#endif
	return default_mmio_write(vgt, offset, p_data, bytes);
}
static bool vgt_write_force_nonpriv(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	if ((bytes != 4) || ((offset & (bytes - 1)) != 0)) {
		vgt_err("VM(%d) vgt_write_force_nonpriv: invalid offset(%x) or bytes(%d)\n",
				vgt->vgt_id, offset, bytes);
		return false;
	}

	if (reg_is_render(vgt->pdev, *(vgt_reg_t *)p_data))
		return default_mmio_write(vgt, offset, p_data, bytes);
	else {
		vgt_err("Unexpected force_to_nonpriv 0x%x mmio write, value=0x%x\n",
				offset, *(vgt_reg_t *)p_data);
		return false;
	}

}

static bool skl_lcpll_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	u32 v = *(u32 *)p_data;

	if (is_current_display_owner(vgt))
		return default_mmio_write(vgt, offset, p_data, bytes);

	/* other bits are MBZ. */
	v &= (1 << 31) | (1 << 30);
	v & (1 << 31) ? (v |= (1 << 30)) : (v &= ~(1 << 30));

	__vreg(vgt, offset) = __sreg(vgt, offset) = v;

	return true;
}

static bool dpll_status_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	u32 v = 0;

	if (is_current_display_owner(vgt))
		goto out;

	if (__vreg(vgt, 0x46010) & (1 << 31))
		v |= (1 << 0);

	if (__vreg(vgt, 0x46014) & (1 << 31))
		v |= (1 << 8);

	if (__vreg(vgt, 0x46040) & (1 << 31))
		v |= (1 << 16);

	if (__vreg(vgt, 0x46060) & (1 << 31))
		v |= (1 << 24);

	__vreg(vgt, offset) = __sreg(vgt, offset) = v;
out:
	return default_mmio_read(vgt, offset, p_data, bytes);
}

static bool mailbox_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	u32 v = *(u32 *)p_data;
	u32 cmd = v & 0xff;
	u32 *data0 = &__vreg(vgt, 0x138128);

	if (!vgt->vm_id)
		goto out;

	switch (cmd) {
		case 0x6:
			/* "Read memory latency" command on gen9. */
			if (!*data0)
				*data0 = vgt->pdev->memory_latency[0];
			else
				*data0 = vgt->pdev->memory_latency[1];
			break;
		case 0x5:
			*data0 |= 0x1;
			break;
	}

	vgt_info("VM %d write %x to mailbox, return data0 %x\n", vgt->vm_id,
		v, *data0);

	v &= ~(1 << 31);
out:
	return default_mmio_write(vgt, offset, &v, bytes);
}


/* BDW PAT index definion.
PAT_INDEX_H(0x40E4)
Bit[63:56]: PAT Index#7 definition for page tables.
Bit[55:48]: PAT Index#6 definition for page tables.
Bit[47:40]: PAT Index#5 definition for page tables.
Bit[39:32]: PAT Index#4 definition for page tables.
PAT_INDEX_L(0x40E0)
Bit[31:24]: PAT Index#3 definition for page tables.
Bit[23:16]: PAT Index#2 definition for page tables.
Bit[15:8]: PAT Index#1 definition for page tables.
Bit[7:0]: PAT Index#0 definition for page tables.
*/

static inline bool gen8_translate_ppat(struct vgt_device *vgt,
		u8 p_index, u8 *m_index)
{
	u8 pat_index, pat_reg_index;
	unsigned long v_pat_value, s_pat_value;
	unsigned long v_cache_attr, s_cache_attr;
	unsigned long v_tc, s_tc;
	unsigned long val;
	bool found = false;

	/*
	Translate rules:
	1.If Target Cache and Cache Attribute exactly match, translate it.
	2.If they are not exactly match, find which has same Cache Attribute.
	3.Target Cache is accepted to mismatch, and LRU Age is igron.
	*/

	*m_index = -1;

	pat_index = p_index;
	if (pat_index > VGT_MAX_PPAT_TABLE_SIZE) {
		vgt_err("Invalid guest PAT index: %x\n", pat_index);
		return false;
	}

	/* Find the Hi/Lo register for the index. */
	pat_reg_index = pat_index / 4;
	/* Find the offset within the register. */
	pat_index -= pat_reg_index * 4;

	val = __vreg(vgt, _REG_GEN8_PRIVATE_PAT + pat_reg_index * 4);
	v_pat_value = (val >> (pat_index * 8)) & 0xf;

	v_cache_attr = v_pat_value & 0x3;
	v_tc = v_pat_value >> 2 & 0x3;

	for (pat_reg_index = 0; pat_reg_index < 2; pat_reg_index++) {
		val = __vreg(vgt_dom0, _REG_GEN8_PRIVATE_PAT + pat_reg_index * 4);
		for (pat_index = 0; pat_index < 4; pat_index++) {
			s_pat_value = (val >> (pat_index * 8)) & 0xf;
			s_cache_attr = s_pat_value & 0x3;
			s_tc = s_pat_value >> 2 & 0x3;

			if (s_cache_attr != v_cache_attr)
				continue;

			/* Target Cache & Cache Attribute exactly match. */
			if (s_tc == v_tc) {
				*m_index = pat_reg_index * 4 + pat_index;
				return true;
			}

			if (found)
				continue;

			/* Cache Attribute match, mark in case for not exactly
			match. */
			found = true;
			*m_index = pat_reg_index * 4 + pat_index;
		}
	}

	if (!found) {
		return false;
	}

	return true;
}


bool gen8_ppat_update_mapping_table(struct vgt_device *vgt)
{
	struct vgt_ppat_table *pt = &vgt->ppat_table;
	int i = 0;
	bool ret = false;
	int cnt_failed = 0;

	pt->is_vaild = true;

	for (; i < VGT_MAX_PPAT_TABLE_SIZE; i++) {
		ret = gen8_translate_ppat(vgt, i, &(pt->mapping_table[i]));
		if (ret == false) {
			cnt_failed++;
		}
	}

	gen8_dump_ppat_registers(vgt);

	if (cnt_failed)
		return false;

	return true;
}

static bool gen8_ppat_write(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	unsigned int virtual = 0;
	unsigned int shadow = 0;
	bool ret = false;

	ret = default_mmio_write(vgt, offset, p_data, bytes);

	if (vgt->vgt_id == 0)
		return ret;

	virtual = __vreg(vgt, _REG_GEN8_PRIVATE_PAT);
	shadow = __vreg(vgt_dom0, _REG_GEN8_PRIVATE_PAT);

	gen8_ppat_update_mapping_table(vgt);

        if ((virtual & 0x3) != (shadow & 0x3))
                vgt_err("VM(%d) gen8_ppat_write, error value at ppat index 0.\n"
                        , vgt->vgt_id);

	return ret;
}

static bool vgt_reg_write_flash_tlb_handler(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	struct pgt_device *pdev = vgt->pdev;
	bool rc = false;

	if ((bytes != 4) || ((offset & (bytes - 1)) != 0)) {
		vgt_err("VM(%d) vgt_reg_write_flash_tlb_handler: invalid offset(%x) or bytes(%d)\n",
				vgt->vm_id, offset, bytes);
		return false;
	}

	rc = default_mmio_write(vgt, offset, p_data, bytes);

	if (vgt->pdev->gen_cache_type == GEN_CACHE_WC)
		VGT_MMIO_WRITE(pdev, offset, __vreg(vgt, offset));

	return rc;
}

static bool vgt_reg_write_misc_ctl_handler(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	struct pgt_device *pdev = vgt->pdev;

	if ((bytes != 4) || ((offset & (bytes - 1)) != 0)) {
		vgt_err("VM(%d) vgt_reg_write_misc_ctl_handler: invalid offset(%x) or bytes(%d)\n",
				vgt->vm_id, offset, bytes);
		return false;
	}

	if (offset == 0x4ddc)
		__vreg(vgt, offset) = 0x8000003c;
	else if (offset == 0x42080)
		__vreg(vgt, offset) = 0x8000;
	else
		ASSERT(0);

	/* TODO: need detect stepping info after pdev contain such information
	 *  0x4ddc enabled after C0, 0x42080 enabled after E0
	 */
	VGT_MMIO_WRITE(pdev, offset, __vreg(vgt, offset));

	return true;
}

static bool vgt_reg_tlb_control_handler(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{
	bool rc = true;
	unsigned int id = 0;

	switch (offset) {
		case 0x4260:
			id = RING_BUFFER_RCS;
			break;
		case 0x4264:
			id = RING_BUFFER_VCS;
			break;
		case 0x4268:
			id = RING_BUFFER_VCS2;
			break;
		case 0x426c:
			id = RING_BUFFER_BCS;
			break;
		case 0x4270:
			id = RING_BUFFER_VECS;
			break;
		default:
			rc = false;
			break;
	}
	set_bit(id, (void *)vgt->tlb_handle_pending);

	return rc;
}

/*
 * Track policies of all captured registers
 *
 * The registers are organized in blocks according to their locations
 * on the spec:
 *	- render
 *	- display
 *	- others (pm, workaround, etc.)
 *      - un-categorized
 *
 * The poclies within the same block can vary:
 *      - [F_VIRT]: default virtualization policy
 *          * all VMs access vReg
 *      - [F_RDR]/[F_DPY]: ownership based virtualization
 *          * owner accesses pReg
 *          * non-owner accesses vReg
 *          * vReg<->pReg at ownership switch time
 *      - [F_DOM0]: uniquely owned by Dom0
 *          * dom0 accesses pReg
 *          * other VMs accesses vReg
 *      - [F_PT]: passthrough policy with HIGH RISK
 *          * all VMs access pReg!!!
 *          * temp solution. must be removed in the end
 *
 * There are some ancillary attributes, which can be linked together
 *      - [ADRFIX]: require address check
 *      - [HWSTS]: need sync with pReg for status bit change
 *      - [MODE]: higher 16bits are mask bits
 *
 * When there are handlers registered, handlers can supersede all
 * above policies.
 */
reg_attr_t vgt_reg_info_general[] = {

/* Interrupt registers - GT */
{IMR, 4, F_RDR, 0, D_ALL, NULL, vgt_reg_imr_handler},
{_REG_BCS_IMR, 4, F_RDR, 0, D_ALL, NULL, vgt_reg_imr_handler},
{_REG_VCS_IMR, 4, F_RDR, 0, D_ALL, NULL, vgt_reg_imr_handler},
{_REG_VECS_IMR, 4, F_RDR, 0, D_ALL, NULL, vgt_reg_imr_handler},

/* Interrupt registers - PCH */
{SDEIMR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_imr_handler},
{SDEIER, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_ier_handler},
{SDEIIR, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_iir_handler},
{SDEISR, 4, F_VIRT, 0, D_ALL, vgt_reg_isr_read, vgt_reg_isr_write},

/* -------render regs---------- */
{_REG_RCS_HWSTAM, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VCS_HWSTAM, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BCS_HWSTAM, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VECS_HWSTAM, 4, F_RDR, 0, D_ALL, NULL, NULL},
{RENDER_HWS_PGA_GEN7, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{BSD_HWS_PGA_GEN7, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{BLT_HWS_PGA_GEN7, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{VEBOX_HWS_PGA_GEN7, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},

/* maybe an error in Linux driver. meant for VCS_HWS_PGA */
{_REG_RCS_EXCC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VCS_EXCC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BCS_EXCC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_RCS_UHPTR, 4, F_RDR_HWSTS, 0, D_ALL, NULL, ring_uhptr_write},
{_REG_VCS_UHPTR, 4, F_RDR_HWSTS, 0, D_ALL, NULL, ring_uhptr_write},
{_REG_BCS_UHPTR, 4, F_RDR_HWSTS, 0, D_ALL, NULL, ring_uhptr_write},
{_REG_VECS_UHPTR, 4, F_RDR_HWSTS, 0, D_ALL, NULL, ring_uhptr_write},
{_REG_RCS_BB_PREEMPT_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{CCID, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{0x12198, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},

{GEN7_CXT_SIZE, 4, F_VIRT, 0, D_ALL, NULL, NULL},

{_REG_RCS_TAIL, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_RCS_HEAD, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_RCS_START, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL,
	ring_mmio_read, ring_mmio_write},
{_REG_RCS_CTL, 4, F_VIRT, 0, D_BDW_PLUS, ring_mmio_read, ring_mmio_write},
{_REG_RCS_CTL, 4, F_RDR, 0, D_HSW, ring_mmio_read, ring_mmio_write},
{_REG_VCS_TAIL, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_VCS_HEAD, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_VCS_START, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL,
	ring_mmio_read, ring_mmio_write},
{_REG_VCS_CTL, 4, F_VIRT, 0, D_BDW_PLUS, ring_mmio_read, ring_mmio_write},
{_REG_VCS_CTL, 4, F_RDR, 0, D_HSW, ring_mmio_read, ring_mmio_write},
{_REG_BCS_TAIL, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_BCS_HEAD, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_BCS_START, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL,
	ring_mmio_read, ring_mmio_write},
{_REG_BCS_CTL, 4, F_VIRT, 0, D_BDW_PLUS, ring_mmio_read, ring_mmio_write},
{_REG_BCS_CTL, 4, F_RDR, 0, D_HSW, ring_mmio_read, ring_mmio_write},

{_REG_VECS_TAIL, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_VECS_HEAD, 4, F_RDR, 0, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_VECS_START, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, ring_mmio_read, ring_mmio_write},
{_REG_VECS_CTL, 4, F_VIRT, 0, D_BDW_PLUS, ring_mmio_read, ring_mmio_write},
{_REG_VECS_CTL, 4, F_RDR, 0, D_HSW, ring_mmio_read, ring_mmio_write},

{ACTHD_I965, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BCS_ACTHD, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VCS_ACTHD, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VECS_ACTHD, 4, F_RDR, 0, D_ALL, NULL, NULL},

{GFX_MODE_GEN7, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_VCS_MFX_MODE_IVB, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_BCS_BLT_MODE_IVB, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_VEBOX_MODE, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{ARB_MODE, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},

{MI_MODE, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_VCS_MI_MODE, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_BCS_MI_MODE, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_VECS_MI_MODE, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},

{INSTPM, 4, F_RDR_MODE, 0, D_ALL, NULL, instpm_write},
{_REG_VCS_INSTPM, 4, F_RDR_MODE, 0, D_ALL, NULL, instpm_write},
{_REG_BCS_INSTPM, 4, F_RDR_MODE, 0, D_ALL, NULL, instpm_write},
{_REG_VECS_INSTPM, 4, F_RDR_MODE, 0, D_ALL, NULL, instpm_write},

{GEN7_GT_MODE, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_CACHE_MODE_0, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_CACHE_MODE_1, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{CACHE_MODE_0_GEN7, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{CACHE_MODE_1, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_RCS_BB_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_VCS_BB_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_BCS_BB_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
{_REG_VECS_BB_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
/* TODO: need a handler */
{_REG_RCS_PP_DCLV, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VCS_PP_DCLV, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BCS_PP_DCLV, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_RBSYNC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_RVSYNC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BRSYNC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_BVSYNC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VBSYNC, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_VRSYNC, 4, F_RDR, 0, D_ALL, NULL, NULL},

{0x2050, 4, F_PT, 0, D_PRE_BDW, NULL, NULL},
{0x12050, 4, F_PT, 0, D_PRE_BDW, NULL, NULL},
{0x22050, 4, F_PT, 0, D_PRE_BDW, NULL, NULL},
{0x1A050, 4, F_PT, 0, D_PRE_BDW, NULL, NULL},

{0x20dc, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_3D_CHICKEN3, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{0x2088, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{0x20e4, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{_REG_VFSKPD, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{GAM_ECOCHK, 4, F_RDR, 0, D_ALL, NULL, NULL},
{GEN7_COMMON_SLICE_CHICKEN1, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{COMMON_SLICE_CHICKEN2, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
{0x9030, 4, F_RDR, 0, D_ALL, NULL, NULL},
{0x20a0, 4, F_RDR, 0, D_ALL, NULL, NULL},
{_REG_RCS_TIMESTAMP, 8, F_PT_RO, 0, D_ALL, NULL, NULL},
{_REG_VCS_TIMESTAMP, 8, F_PT_RO, 0, D_ALL, NULL, NULL},
{0x1a358, 8, F_PT_RO, 0, D_ALL, NULL, NULL},
{_REG_BCS_TIMESTAMP, 8, F_PT_RO, 0, D_ALL, NULL, NULL},
{0x2420, 4, F_RDR, 0, D_ALL, NULL, NULL},
{0x2430, 4, F_RDR, 0, D_ALL, NULL, NULL},
{0x2434, 4, F_RDR, 0, D_ALL, NULL, NULL},
{0x2438, 4, F_RDR, 0, D_ALL, NULL, NULL},
{0x243c, 4, F_RDR, 0, D_ALL, NULL, NULL},
{0x7018, 4, F_RDR, 0, D_ALL, NULL, NULL},
{0xe184, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
	/* -------display regs---------- */

{0x60220, 0x20, F_DPY, 0, D_ALL, NULL, NULL},
{0x602a0, 4, F_DPY, 0, D_ALL, NULL, NULL},

{0x65050, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x650b4, 4, F_DPY, 0, D_ALL, NULL, NULL},

{VGA_CR_INDEX_MDA, 1, F_DPY, 0, D_ALL, NULL, NULL},
{VGA_ST01_MDA, 1, F_DPY, 0, D_ALL, NULL, NULL},
{VGA_AR_INDEX, 1, F_DPY, 0, D_ALL, NULL, NULL},
{VGA_DACMASK, 1, F_DPY, 0, D_ALL, NULL, NULL},
{VGA_MSR_READ, 1, F_DPY, 0, D_ALL, NULL, NULL},
{VGA0, 4, F_DPY, 0, D_ALL, NULL, NULL},
{VGA1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{VGA_PD, 4, F_DPY, 0, D_ALL, NULL, NULL},

{0x42080, 4, F_DOM0, 0, D_PRE_SKL, NULL, NULL},
{0xc4040, 4, F_VIRT, 0, D_ALL, NULL, NULL},

{_REG_DE_RRMR, 4, F_VIRT, 0, D_ALL, NULL, vgt_rrmr_mmio_write},

{_PIPEADSL, 4, F_DPY, 0, D_ALL, pipe_dsl_mmio_read, NULL},
{_PIPEACONF, 4, F_DPY, 0, D_ALL, NULL, pipe_conf_mmio_write},
{_PIPEASTAT, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PIPEA_FRMCOUNT_G4X, 4, F_DPY, 0, D_ALL, pipe_frmcount_mmio_read, NULL},
{_PIPEA_FLIPCOUNT_G4X, 4, F_VIRT, 0, D_ALL, NULL, NULL},

{PIPE_B_OFFSET, 4, F_DPY, 0, D_ALL, pipe_dsl_mmio_read, NULL},
{_REG_PIPEBCONF, 4, F_DPY, 0, D_ALL, NULL, pipe_conf_mmio_write},
{_REG_PIPEBSTAT, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEB_FRMCOUNT, 4, F_DPY, 0, D_ALL, pipe_frmcount_mmio_read, NULL},
{_REG_PIPEB_FLIPCOUNT, 4, F_VIRT, 0, D_ALL, NULL, NULL},

{PIPE_C_OFFSET, 4, F_DPY, 0, D_ALL, pipe_dsl_mmio_read, NULL},
{_REG_PIPECCONF, 4, F_DPY, 0, D_ALL, NULL, pipe_conf_mmio_write},
{_REG_PIPEC_FRMCOUNT, 4, F_DPY, 0, D_ALL, pipe_frmcount_mmio_read, NULL},
{_REG_PIPEC_FLIPCOUNT, 4, F_VIRT, 0, D_ALL, NULL, NULL},

{_REG_PIPE_EDP_CONF, 4, F_DPY, 0, D_ALL, NULL, pipe_conf_mmio_write},

{_CURABASE, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, dpy_plane_mmio_read,
						cur_surf_mmio_write},
{_CURACNTR, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read, cur_plane_ctl_write},
{_CURAPOS, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read, dpy_plane_mmio_write},
{_REG_CURASURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, cur_surflive_mmio_read,
					surflive_mmio_write},
{_REG_CURAPALET_0, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CURAPALET_1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CURAPALET_2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CURAPALET_3, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_CURBBASE_IVB, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, dpy_plane_mmio_read,
						cur_surf_mmio_write},
{_CURBCNTR_IVB, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
						cur_plane_ctl_write},
{_CURBPOS_IVB, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
						dpy_plane_mmio_write},
{_REG_CURBSURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, cur_surflive_mmio_read,
					surflive_mmio_write},
{_REG_CURCBASE, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, dpy_plane_mmio_read,
						cur_surf_mmio_write},

{IVB_CURSOR_C_OFFSET, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
						cur_plane_ctl_write},
{_REG_CURCPOS, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
						dpy_plane_mmio_write},
{_REG_CURCSURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, cur_surflive_mmio_read,
					surflive_mmio_write},

{0x7008C, 4, F_DPY, 0, D_ALL, NULL, vgt_error_handler},
{0x701b0, 4, F_VIRT, 0, D_ALL, NULL, NULL},

{_DSPACNTR, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_ctl_write},
{_DSPASURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, dpy_plane_mmio_read,
							pri_surf_mmio_write},
{_DSPASURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, pri_surflive_mmio_read,
							surflive_mmio_write},
{_DSPAADDR, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_DSPASTRIDE, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_DSPAPOS, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_DSPASIZE, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_DSPATILEOFF, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},

{_REG_DSPBCNTR, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_ctl_write},
{_REG_DSPBSURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, dpy_plane_mmio_read,
							pri_surf_mmio_write},
{_REG_DSPBSURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, pri_surflive_mmio_read,
							surflive_mmio_write},
{_REG_DSPBLINOFF, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_REG_DSPBSTRIDE, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_REG_DSPBPOS, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_REG_DSPBSIZE, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_REG_DSPBTILEOFF, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},

{_DVSACNTR, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_ctl_write},
{_DVSASURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, dpy_plane_mmio_read,
							pri_surf_mmio_write},
{_DVSASURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, pri_surflive_mmio_read,
							surflive_mmio_write},
{_DVSASTRIDE, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_DVSATILEOFF, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read,
							dpy_plane_mmio_write},

{_DVSBSURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},

{_SPRA_SURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL,
			dpy_plane_mmio_read, spr_surf_mmio_write},
{_SPRA_SURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL,
			spr_surflive_mmio_read, surflive_mmio_write},

{_PLANE_SURF_2_B, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL,
			dpy_plane_mmio_read, spr_surf_mmio_write},
{_SPRB_SURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL,
			spr_surflive_mmio_read, surflive_mmio_write},

{_REG_SPRCSURF, 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL,
			dpy_plane_mmio_read, spr_surf_mmio_write},
{_REG_SPRCSURFLIVE, 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL,
			spr_surflive_mmio_read, surflive_mmio_write},

{_SPRA_CTL, 4, F_DPY, 0, D_ALL, NULL, sprite_plane_ctl_write},

{_PLANE_CTL_2_B, 4, F_DPY, 0, D_ALL, NULL, sprite_plane_ctl_write},

{_REG_SPRC_CTL, 4, F_DPY, 0, D_ALL, NULL, sprite_plane_ctl_write},


{_LGC_PALETTE_A, 4*256, F_DPY, 0, D_ALL, NULL, NULL},
{_LGC_PALETTE_B, 4*256, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_LGC_PALETTE_C, 4*256, F_DPY, 0, D_ALL, NULL, NULL},

{_HTOTAL_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_HBLANK_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_HSYNC_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_VTOTAL_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_VBLANK_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_VSYNC_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_PIPEASRC, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read, dpy_plane_mmio_write},
{_BCLRPAT_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_VSYNCSHIFT_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},

{_HTOTAL_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_HBLANK_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_HSYNC_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_VTOTAL_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_VBLANK_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_VSYNC_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_PIPEBSRC, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read, dpy_plane_mmio_write},
{_BCLRPAT_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_VSYNCSHIFT_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},

{_REG_HTOTAL_C, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_HBLANK_C, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_HSYNC_C, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_VTOTAL_C, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_VBLANK_C, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_VSYNC_C, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_REG_PIPECSRC, 4, F_DPY, 0, D_ALL, dpy_plane_mmio_read, dpy_plane_mmio_write},
{_REG_BCLRPAT_C, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_VSYNCSHIFT_C, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},

{0x6F000, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{0x6F004, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{0x6F008, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{0x6F00C, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{0x6F010, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{0x6F014, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{0x6F028, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{0x6F030, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x6F034, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x6F040, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x6F044, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_PIPEA_DATA_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PIPEA_DATA_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PIPEA_LINK_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PIPEA_LINK_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_PIPEB_DATA_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PIPEB_DATA_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PIPEB_LINK_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PIPEB_LINK_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_PIPEC_DATA_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEC_DATA_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEC_LINK_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PIPEC_LINK_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_PFA_CTL_1, 4, F_DPY, 0, D_ALL, pf_read, pf_write},
{_PFA_WIN_SZ, 4, F_DPY, 0, D_ALL, pf_read, pf_write},
{_PFA_WIN_POS, 4, F_DPY, 0, D_ALL, pf_read, pf_write},
{_PFB_CTL_1, 4, F_DPY, 0, D_ALL, pf_read, pf_write},
{_PFB_WIN_SZ, 4, F_DPY, 0, D_ALL, pf_read, pf_write},
{_PFB_WIN_POS, 4, F_DPY, 0, D_ALL, pf_read, pf_write},
{_REG_PF_CTL_2, 4, F_DPY, 0, D_ALL, pf_read, pf_write},
{_REG_PF_WIN_SZ_2, 4, F_DPY, 0, D_ALL, pf_read, pf_write},
{_REG_PF_WIN_POS_2, 4, F_DPY, 0, D_ALL, pf_read, pf_write},

{WM0_PIPEA_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
{WM0_PIPEB_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
{WM0_PIPEC_IVB, 4, F_DPY, 0, D_ALL, NULL, NULL},
{WM1_LP_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
{WM2_LP_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
{WM3_LP_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
{WM1S_LP_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
{WM2S_LP_IVB, 4, F_DPY, 0, D_ALL, NULL, NULL},
{WM3S_LP_IVB, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_HISTOGRAM_THRSH, 4, F_DPY, 0, D_ALL, NULL, NULL},

{BLC_PWM_CPU_CTL2, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{BLC_PWM_CPU_CTL, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{BLC_PWM_PCH_CTL1, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{BLC_PWM_PCH_CTL2, 4, F_DOM0, 0, D_ALL, NULL, NULL},

{PCH_GMBUS0, 4*4, F_DPY, 0, D_ALL, gmbus_mmio_read, gmbus_mmio_write},
{PCH_GPIOA, 6*4, F_VIRT, 0, D_ALL, NULL, NULL},

{_REG_DP_BUFTRANS, 0x28, F_DPY, 0, D_ALL, NULL, NULL},

{PCH_DPB_AUX_CH_CTL, 6*4, F_DPY, 0, D_PRE_SKL, NULL, dp_aux_ch_ctl_mmio_write},
{PCH_DPC_AUX_CH_CTL, 6*4, F_DPY, 0, D_PRE_SKL, NULL, dp_aux_ch_ctl_mmio_write},
{PCH_DPD_AUX_CH_CTL, 6*4, F_DPY, 0, D_PRE_SKL, NULL, dp_aux_ch_ctl_mmio_write},

{PCH_ADPA, 4, F_DPY, 0, D_ALL, NULL, pch_adpa_mmio_write},
{_PCH_TRANSACONF, 4, F_DPY, 0, D_ALL, NULL, transaconf_mmio_write},
{_PCH_TRANSBCONF, 4, F_DPY, 0, D_ALL, NULL, transaconf_mmio_write},
{_FDI_RXA_IIR, 4, F_DPY, 0, D_ALL, NULL, fdi_rx_iir_mmio_write},
{_FDI_RXB_IIR, 4, F_DPY, 0, D_ALL, NULL, fdi_rx_iir_mmio_write},
{_REG_FDI_RXC_IIR, 4, F_DPY, 0, D_ALL, NULL, fdi_rx_iir_mmio_write},
{_FDI_RXA_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_FDI_RXB_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_REG_FDI_RXC_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_FDI_TXA_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_FDI_TXB_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_REG_FDI_TXC_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_FDI_RXA_IMR, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_FDI_RXB_IMR, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
{_REG_FDI_RXC_IMR, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},

{_PCH_TRANS_HTOTAL_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_PCH_TRANS_HBLANK_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_PCH_TRANS_HSYNC_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_PCH_TRANS_VTOTAL_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_PCH_TRANS_VBLANK_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_PCH_TRANS_VSYNC_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_PCH_TRANS_VSYNCSHIFT_A, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},

{_PCH_TRANS_HTOTAL_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_PCH_TRANS_HBLANK_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_PCH_TRANS_HSYNC_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_PCH_TRANS_VTOTAL_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_PCH_TRANS_VBLANK_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_PCH_TRANS_VSYNC_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},
{_PCH_TRANS_VSYNCSHIFT_B, 4, F_DPY, 0, D_ALL, NULL, dpy_modeset_mmio_write},

{_PCH_TRANSA_DATA_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PCH_TRANSA_DATA_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PCH_TRANSA_DATA_M2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PCH_TRANSA_DATA_N2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PCH_TRANSA_LINK_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PCH_TRANSA_LINK_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PCH_TRANSA_LINK_M2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PCH_TRANSA_LINK_N2, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_VIDEO_DIP_CTL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_VIDEO_DIP_DATA_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_VIDEO_DIP_GCP_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{TRANS_DP_CTL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_VIDEO_DIP_CTL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_VIDEO_DIP_DATA_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_VIDEO_DIP_GCP_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{TRANS_DP_CTL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSC_VIDEO_DIP_CTL, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSC_VIDEO_DIP_DATA, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANSC_VIDEO_DIP_GCP, 4, F_DPY, 0, D_ALL, NULL, NULL},
{TRANS_DP_CTL_C, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_FDI_RXA_MISC, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_FDI_RXB_MISC, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_FDI_RXA_TUSIZE1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_FDI_RXA_TUSIZE2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_FDI_RXB_TUSIZE1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_FDI_RXB_TUSIZE2, 4, F_DPY, 0, D_ALL, NULL, NULL},

{PCH_PP_CONTROL, 4, F_DPY, 0, D_ALL, NULL, pch_pp_control_mmio_write},
{PCH_PP_DIVISOR, 4, F_DPY, 0, D_ALL, NULL, NULL},
{PCH_PP_STATUS, 4, F_DPY, 0, D_ALL, NULL, NULL},
{PCH_LVDS, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PCH_DPLL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PCH_DPLL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PCH_FPA0, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PCH_FPA1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PCH_FPB0, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_PCH_FPB1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{PCH_DREF_CONTROL, 4, F_DPY, 0, D_ALL, NULL, NULL},
{PCH_RAWCLK_FREQ, 4, F_DPY, 0, D_ALL, NULL, NULL},
{PCH_DPLL_SEL, 4, F_DPY, 0, D_ALL, NULL, NULL},
	/* Linux defines as PP_ON_DEPLAY/PP_OFF_DELAY. Not in spec */
{0x61208, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x6120c, 4, F_DPY, 0, D_ALL, NULL, NULL},
{PCH_PP_ON_DELAYS, 4, F_DPY, 0, D_ALL, NULL, NULL},
{PCH_PP_OFF_DELAYS, 4, F_DPY, 0, D_ALL, NULL, NULL},

{0xE651C, 4, F_DPY, 0, D_ALL, dpy_reg_mmio_read, NULL},
{0xE661C, 4, F_DPY, 0, D_ALL, dpy_reg_mmio_read, NULL},
{0xE671C, 4, F_DPY, 0, D_ALL, dpy_reg_mmio_read, NULL},
{0xE681C, 4, F_DPY, 0, D_ALL, dpy_reg_mmio_read, NULL},
{0xE6C04, 4, F_DPY, 0, D_ALL,
	dpy_reg_mmio_read_2, NULL},
{0xE6E1C, 4, F_DPY, 0, D_ALL,
	dpy_reg_mmio_read_3, NULL},
{PCH_PORT_HOTPLUG, 4, F_VIRT, 0, D_ALL, NULL, shotplug_ctl_mmio_write},
{LCPLL_CTL, 4, F_DPY, 0, D_ALL, NULL, lcpll_ctl_mmio_write},
{FUSE_STRAP, 4, F_DPY, 0, D_ALL, NULL, NULL},
{DIGITAL_PORT_HOTPLUG_CNTRL, 4, F_DPY, 0, D_ALL, NULL, NULL},

{DISP_ARB_CTL, 4, F_DPY, 0, D_ALL, NULL, NULL},
{DISP_ARB_CTL2, 4, F_DPY, 0, D_ALL, NULL, NULL},

{ILK_DISPLAY_CHICKEN1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{ILK_DISPLAY_CHICKEN2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{ILK_DSPCLK_GATE_D, 4, F_DPY, 0, D_ALL, NULL, NULL},

{SOUTH_CHICKEN1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{SOUTH_CHICKEN2, 4, F_DPY, 0, D_ALL, NULL, south_chicken2_write},
{_TRANSA_CHICKEN1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_TRANSB_CHICKEN1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{SOUTH_DSPCLK_GATE_D, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_TRANSA_CHICKEN2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_TRANSB_CHICKEN2, 4, F_DPY, 0, D_ALL, NULL, NULL},
/*
 * framebuffer compression is disabled for now
 * until it's handled at display context switch
 * and we figure out how stolen memory should be virtualized (FBC needs use
 * stolen memory).
 */
{_REG_DPFC_CB_BASE, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_DPFC_CONTROL, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_DPFC_RECOMP_CTL, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_DPFC_CPU_FENCE_OFFSET, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_DPFC_CONTROL_SA, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_DPFC_CPU_FENCE_OFFSET_SA, 4, F_VIRT, 0, D_ALL, NULL, NULL},

{IPS_CTL, 4, F_DOM0, 0, D_ALL, NULL, NULL},

{_REG_CSC_A_COEFFICIENTS, 4*6, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CSC_A_MODE, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_A_HIGH_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_A_MEDIUM_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_A_LOW_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_CSC_B_COEFFICIENTS, 4*6, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CSC_B_MODE, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_B_HIGH_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_B_MEDIUM_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_B_LOW_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_CSC_C_COEFFICIENTS, 4*6, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_CSC_C_MODE, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_C_HIGH_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_C_MEDIUM_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PRECSC_C_LOW_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},

{0x60110, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x61110, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x70400, 0x40, F_DPY, 0, D_ALL, NULL, NULL},
{0x71400, 0x40, F_DPY, 0, D_ALL, NULL, NULL},
{0x72400, 0x40, F_DPY, 0, D_ALL, NULL, NULL},

{0x70440, 0xc, F_DPY, 0, D_PRE_SKL, NULL, NULL},
{0x71440, 0xc, F_DPY, 0, D_PRE_SKL, NULL, NULL},
{0x72440, 0xc, F_DPY, 0, D_PRE_SKL, NULL, NULL},

{0x7044c, 0xc, F_DPY, 0, D_PRE_SKL, NULL, NULL},
{0x7144c, 0xc, F_DPY, 0, D_PRE_SKL, NULL, NULL},
{0x7244c, 0xc, F_DPY, 0, D_PRE_SKL, NULL, NULL},

{PIPE_WM_LINETIME_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{PIPE_WM_LINETIME_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x45278, 4, F_DPY, 0, D_ALL, NULL, NULL},
{SPLL_CTL, 4, F_DPY, 0, D_ALL, NULL, NULL},
{WRPLL_CTL1, 4, F_DPY, 0, D_ALL, NULL, NULL},
{WRPLL_CTL2, 4, F_DPY, 0, D_ALL, NULL, NULL},
{PORT_CLK_SEL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{PORT_CLK_SEL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PORT_CLK_SEL_DDIC, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PORT_CLK_SEL_DDID, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_PORT_CLK_SEL_DDIE, 4, F_DPY, 0, D_ALL, NULL, NULL},
{TRANS_CLK_SEL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{TRANS_CLK_SEL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_TRANS_CLK_SEL_C, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x46408, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x46508, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x49040, 0xc, F_DPY, 0, D_ALL, NULL, NULL},
{0x49140, 0xc, F_DPY, 0, D_ALL, NULL, NULL},
{0x49240, 0xc, F_DPY, 0, D_ALL, NULL, NULL},
{0x49080, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x49090, 0x14, F_DPY, 0, D_ALL, NULL, NULL},
{0x49180, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x49190, 0x14, F_DPY, 0, D_ALL, NULL, NULL},
{0x49280, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x49290, 0x14, F_DPY, 0, D_ALL, NULL, NULL},
{0x4A400, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x4A480, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x4AC00, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x4AC80, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x4B400, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x4B480, 4, F_DPY, 0, D_ALL, NULL, NULL},

{0x6002C, 4, F_DPY, 0, D_ALL, NULL, NULL},

{HSW_VIDEO_DIP_CTL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{HSW_VIDEO_DIP_CTL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_HSW_VIDEO_DIP_CTL_C, 4, F_DPY, 0, D_ALL, NULL, NULL},
{_REG_HSW_VIDEO_DIP_CTL_EDP, 4, F_DPY, 0, D_ALL, NULL, NULL},

{SFUSE_STRAP, 4, F_DPY, 0, D_ALL, sfuse_strap_mmio_read, NULL},
{SBI_ADDR, 4, F_DPY, 0, D_ALL, NULL, NULL},
{SBI_DATA, 4, F_DPY, 0, D_ALL, sbi_mmio_data_read, NULL},
{SBI_CTL_STAT, 4, F_DPY, 0, D_ALL, NULL, sbi_mmio_ctl_write},
{PIXCLK_GATE, 4, F_DPY, 0, D_ALL, NULL, NULL},

{DPA_AUX_CH_CTL, 6*4, F_DPY, 0, D_ALL, NULL, dp_aux_ch_ctl_mmio_write},

{DDI_BUF_CTL_A, 4, F_DPY, 0, D_ALL, NULL, ddi_buf_ctl_mmio_write},
{DDI_BUF_CTL_B, 4, F_DPY, 0, D_ALL, NULL, ddi_buf_ctl_mmio_write},
{_REG_DDI_BUF_CTL_C, 4, F_DPY, 0, D_ALL, NULL, ddi_buf_ctl_mmio_write},
{_REG_DDI_BUF_CTL_D, 4, F_DPY, 0, D_ALL, NULL, ddi_buf_ctl_mmio_write},
{_REG_DDI_BUF_CTL_E, 4, F_DPY, 0, D_ALL, NULL, ddi_buf_ctl_mmio_write},

{DP_TP_CTL_A, 4, F_DPY, 0, D_ALL, NULL, dp_tp_ctl_mmio_write},
{DP_TP_CTL_B, 4, F_DPY, 0, D_ALL, NULL, dp_tp_ctl_mmio_write},
{_REG_DP_TP_CTL_C, 4, F_DPY, 0, D_ALL, NULL, dp_tp_ctl_mmio_write},
{_REG_DP_TP_CTL_D, 4, F_DPY, 0, D_ALL, NULL, dp_tp_ctl_mmio_write},
{_REG_DP_TP_CTL_E, 4, F_DPY, 0, D_ALL, NULL, NULL},

{DP_TP_STATUS_A, 4, F_DPY, 0, D_ALL, NULL, dp_tp_status_mmio_write},
{DP_TP_STATUS_B, 4, F_DPY, 0, D_ALL, NULL, dp_tp_status_mmio_write},
{_REG_DP_TP_STATUS_C, 4, F_DPY, 0, D_ALL, NULL, dp_tp_status_mmio_write},
{_REG_DP_TP_STATUS_D, 4, F_DPY, 0, D_ALL, NULL, dp_tp_status_mmio_write},
{_REG_DP_TP_STATUS_E, 4, F_DPY, 0, D_ALL, NULL, NULL},
{DDI_BUF_TRANS_A, 0x50, F_DPY, 0, D_ALL, NULL, NULL},
{0x64E60, 0x50, F_DPY, 0, D_ALL, NULL, NULL},
{0x64Ec0, 0x50, F_DPY, 0, D_ALL, NULL, NULL},
{0x64F20, 0x50, F_DPY, 0, D_ALL, NULL, NULL},
{0x64F80, 0x50, F_DPY, 0, D_ALL, NULL, NULL},
{_HSW_AUD_CONFIG_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x650C0, 4, F_DPY, 0, D_ALL, NULL, NULL},

{TRANS_DDI_FUNC_CTL_A, 4, F_DPY, 0, D_ALL, NULL, dpy_trans_ddi_ctl_write},
{TRANS_DDI_FUNC_CTL_B, 4, F_DPY, 0, D_ALL, NULL, dpy_trans_ddi_ctl_write},
{TRANS_DDI_FUNC_CTL_C, 4, F_DPY, 0, D_ALL, NULL, dpy_trans_ddi_ctl_write},
{TRANS_DDI_FUNC_CTL_EDP, 4, F_DPY, 0, D_ALL, NULL, dpy_trans_ddi_ctl_write},

{TRANSA_MSA_MISC, 4, F_DPY, 0, D_ALL, NULL, NULL},
{TRANSB_MSA_MISC, 4, F_DPY, 0, D_ALL, NULL, NULL},
{TRANSC_MSA_MISC, 4, F_DPY, 0, D_ALL, NULL, NULL},
{0x6F410, 4, F_DPY, 0, D_ALL, NULL, NULL},

	/* -------others---------- */
{FORCEWAKE, 4, F_VIRT, 0, D_ALL, NULL, force_wake_write},
{FORCEWAKE_ACK, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_GT_CORE_STATUS, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{GEN6_GT_THREAD_STATUS_REG, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{GTFIFODBG, 4, F_RDR, 0, D_ALL, NULL, NULL},
{GTFIFOCTL, 4, F_RDR, 0, D_ALL, NULL, NULL},
{FORCEWAKE_MT, 4, F_VIRT, 0, D_PRE_SKL, NULL, mul_force_wake_write},
{FORCEWAKE_ACK_HSW, 4, F_VIRT, 0, D_HSW | D_BDW, mul_force_wake_ack_read, NULL},
{ECOBUS, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RC_CONTROL, 4, F_DOM0, 0, D_ALL, NULL, rc_state_ctrl_1_mmio_write},
{GEN6_RC_STATE, 4, F_DOM0, 0, D_ALL, NULL, rc_state_ctrl_1_mmio_write},
{GEN6_RPNSWREQ, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RC_VIDEO_FREQ, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RP_DOWN_TIMEOUT, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RP_INTERRUPT_LIMITS, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RPSTAT1, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RP_CONTROL, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RP_UP_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RP_DOWN_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RP_CUR_UP_EI, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RP_CUR_UP, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RP_PREV_UP, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RP_CUR_DOWN_EI, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RP_CUR_DOWN, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RP_PREV_DOWN, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RP_UP_EI, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RP_DOWN_EI, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RP_IDLE_HYSTERSIS, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RC1_WAKE_RATE_LIMIT, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RC6_WAKE_RATE_LIMIT, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RC6pp_WAKE_RATE_LIMIT, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RC_EVALUATION_INTERVAL, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RC_IDLE_HYSTERSIS, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RC_SLEEP, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RC1e_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RC6_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RC6p_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_RC6pp_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_PMINTRMSK, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{HSW_PWR_WELL_BIOS, 4, F_DOM0, 0, D_HSW | D_BDW, power_well_ctl_read, power_well_ctl_write},
{HSW_PWR_WELL_DRIVER, 4, F_DOM0, 0, D_HSW | D_BDW, power_well_ctl_read, power_well_ctl_write},
{HSW_PWR_WELL_KVMR, 4, F_DOM0, 0, D_HSW | D_BDW, power_well_ctl_read, power_well_ctl_write},
{HSW_PWR_WELL_DEBUG, 4, F_DOM0, 0, D_HSW | D_BDW, power_well_ctl_read, power_well_ctl_write},
{HSW_PWR_WELL_CTL5, 4, F_DOM0, 0, D_HSW | D_BDW, power_well_ctl_read, power_well_ctl_write},
{HSW_PWR_WELL_CTL6, 4, F_DOM0, 0, D_HSW | D_BDW, power_well_ctl_read, power_well_ctl_write},

{RSTDBYCTL, 4, F_DOM0, 0, D_ALL, NULL, NULL},

{GEN6_GDRST, 4, F_DOM0, 0, D_ALL, gen6_gdrst_mmio_read, gen6_gdrst_mmio_write},
{_REG_FENCE_0_LOW, 0x80, F_VIRT, 0, D_ALL, fence_mmio_read, fence_mmio_write},
{VGT_PVINFO_PAGE, VGT_PVINFO_SIZE, F_VIRT, 0, D_ALL, pvinfo_read, pvinfo_write},
{CPU_VGACNTRL, 4, F_DOM0, 0, D_ALL, vga_control_r, vga_control_w},

/* TODO: MCHBAR, suppose read-only */
{MCHBAR_MIRROR_BASE_SNB, 0x40000, F_VIRT, 0, D_ALL, NULL, NULL},

{TILECTL, 4, F_DOM0, 0, D_ALL, NULL, NULL},

{GEN6_UCGCTL1, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN6_UCGCTL2, 4, F_DOM0, 0, D_ALL, NULL, NULL},

{_REG_SWF, 0x90, F_VIRT, 0, D_ALL, NULL, NULL},

{GEN6_PCODE_MAILBOX, 4, F_DOM0, 0, D_PRE_SKL, NULL, NULL},
{GEN6_PCODE_DATA, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{0x13812c, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GEN7_ERR_INT, 4, F_VIRT, 0, D_ALL, err_int_r, err_int_w},
{0x120010, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x9008, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{GFX_FLSH_CNTL_GEN6, 4, F_VIRT, 0, D_ALL, NULL, vgt_reg_write_flash_tlb_handler},

	/* -------un-categorized regs--------- */
{0x3c, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{0x860, 4, F_VIRT, 0, D_ALL, NULL, NULL},
/* no definition on this. from Linux */
{ECOSKPD, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x121d0, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{GEN6_BLITTER_ECOSKPD, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x41d0, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{GAC_ECO_BITS, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_2D_CG_DIS, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_3D_CG_DIS, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{_REG_3D_CG_DIS2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x7118, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x7180, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x7408, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x7c00, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{GEN6_MBCTL, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x911c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x9120, 4, F_VIRT, 0, D_ALL, NULL, NULL},

{GAB_CTL, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0x48800, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xce044, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6500, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6504, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6600, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6604, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6700, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6704, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6800, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xe6804, 4, F_VIRT, 0, D_ALL, NULL, NULL},
/* FIXME: now looks gmbus handler can't cover 4/5 ports */
{PCH_GMBUS4, 4, F_DPY, 0, D_ALL, NULL, NULL},
{PCH_GMBUS5, 4, F_DPY, 0, D_ALL, NULL, NULL},

{_REG_SUPER_QUEUE_CONFIG, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec008, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec00c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec008+0x18, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec00c+0x18, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec008+0x18*2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec00c+0x18*2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec008+0x18*3, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec00c+0x18*3, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec408, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec40c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec408+0x18, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec40c+0x18, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec408+0x18*2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec40c+0x18*2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec408+0x18*3, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xec40c+0x18*3, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfc810, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfc81c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfc828, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfc834, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfcc00, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfcc0c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfcc18, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfcc24, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfd000, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfd00c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfd018, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfd024, 4, F_VIRT, 0, D_ALL, NULL, NULL},
{0xfd034, 4, F_VIRT, 0, D_ALL, NULL, NULL},

/* HSW */
{FPGA_DBG, 4, F_DOM0, 0, D_ALL, fpga_dbg_mmio_read, fpga_dbg_mmio_write},
/* MAXCNT means max idle count */

{_REG_RC_PWRCTX_MAXCNT, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{0x12054, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{0x22054, 4, F_DOM0, 0, D_ALL, NULL, NULL},
{0x1A054, 4, F_DOM0, 0, D_ALL, NULL, NULL},

{0x44070, 4, F_DOM0, 0, D_ALL, NULL, NULL},

/*command accessed registers, supplement for reg audit in cmd parser*/
{0x215c, 4, F_RDR, 0, D_HSW_PLUS, NULL, NULL},
{0x2178, 4, F_RDR, 0, D_ALL, NULL, NULL},
{0x217c, 4, F_RDR, 0, D_ALL, NULL, NULL},
{0x12178, 4, F_RDR, 0, D_ALL, NULL, NULL},
{0x1217c, 4, F_RDR, 0, D_ALL, NULL, NULL},
{0x2290, 8, F_RDR, 0, D_HSW_PLUS, NULL, NULL},
{OACONTROL, 4, F_DOM0, 0, D_HSW, NULL, NULL},
{_REG_OACTXCONTROL, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{_REG_OACONTROL_GEN8, 4, F_DOM0, 0, D_BDW_PLUS, NULL, NULL},
{0x5200, 32, F_RDR, 0, D_ALL, NULL, NULL},
{0x5240, 32, F_RDR, 0, D_ALL, NULL, NULL},
{0x5280, 16, F_RDR, 0, D_ALL, NULL, NULL},
{0x1c178, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{0x1c17c, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{BCS_SWCTRL, 4, F_RDR, 0, D_ALL, NULL, NULL},
{HS_INVOCATION_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
{DS_INVOCATION_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
{IA_VERTICES_COUNT  , 8, F_RDR, 0, D_ALL, NULL, NULL},
{IA_PRIMITIVES_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
{VS_INVOCATION_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
{GS_INVOCATION_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
{GS_PRIMITIVES_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
{CL_INVOCATION_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
{CL_PRIMITIVES_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
{PS_INVOCATION_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
{PS_DEPTH_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},

/* BDW */
{0xe100, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},

{0x4260, 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_tlb_control_handler},
{0x4264, 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_tlb_control_handler},
{0x4268, 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_tlb_control_handler},
{0x426c, 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_tlb_control_handler},
{0x4270, 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_tlb_control_handler},

{_RING_FAULT_REG(RING_BUFFER_RCS), 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{0x6651c, 4, F_DPY, 0, D_BDW_PLUS, NULL, NULL},
{0x6671c, 4, F_DPY, 0, D_BDW_PLUS, NULL, NULL},
{0x44484, 4, F_DOM0, 0, D_BDW_PLUS, NULL, NULL},
{0x4448c, 4, F_DOM0, 0, D_BDW_PLUS, NULL, NULL},
{0x4a404, 4, F_DPY, 0, D_BDW_PLUS, NULL, NULL},
};

reg_attr_t vgt_reg_info_bdw[] = {
/* WA in indirect ctx */
{0xe220, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{0xe230, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{0xe240, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{0xe260, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{0xe270, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{0xe280, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{0xe2a0, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{0xe2b0, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{0xe2c0, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},

/* Interrupt registers - GT */
{_REG_VCS2_IMR, 4, F_RDR, 0, D_BDW_PLUS, NULL, vgt_reg_imr_handler},

/* Interrupt registers - BDW */
{_REG_GT_IMR(0), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_imr_handler},
{_REG_GT_IER(0), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_ier_handler},
{_REG_GT_IIR(0), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_iir_handler},
{_REG_GT_ISR(0), 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_GT_IMR(1), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_imr_handler},
{_REG_GT_IER(1), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_ier_handler},
{_REG_GT_IIR(1), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_iir_handler},
{_REG_GT_ISR(1), 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_GT_IMR(2), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_imr_handler},
{_REG_GT_IER(2), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_ier_handler},
{_REG_GT_IIR(2), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_iir_handler},
{_REG_GT_ISR(2), 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_GT_IMR(3), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_imr_handler},
{_REG_GT_IER(3), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_ier_handler},
{_REG_GT_IIR(3), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_iir_handler},
{_REG_GT_ISR(3), 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_DE_PIPE_IMR(PIPE_A), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_imr_handler},
{_REG_DE_PIPE_IER(PIPE_A), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_ier_handler},
{_REG_DE_PIPE_IIR(PIPE_A), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_iir_handler},
{_REG_DE_PIPE_ISR(PIPE_A), 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_DE_PIPE_IMR(PIPE_B), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_imr_handler},
{_REG_DE_PIPE_IER(PIPE_B), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_ier_handler},
{_REG_DE_PIPE_IIR(PIPE_B), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_iir_handler},
{_REG_DE_PIPE_ISR(PIPE_B), 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_DE_PIPE_IMR(PIPE_C), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_imr_handler},
{_REG_DE_PIPE_IER(PIPE_C), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_ier_handler},
{_REG_DE_PIPE_IIR(PIPE_C), 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_iir_handler},
{_REG_DE_PIPE_ISR(PIPE_C), 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{GEN8_DE_PORT_IMR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_imr_handler},
{GEN8_DE_PORT_IER, 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_ier_handler},
{GEN8_DE_PORT_IIR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_iir_handler},
{GEN8_DE_PORT_ISR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{GEN8_DE_MISC_IMR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_imr_handler},
{GEN8_DE_MISC_IER, 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_ier_handler},
{GEN8_DE_MISC_IIR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_iir_handler},
{GEN8_DE_MISC_ISR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{GEN8_PCU_IMR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_imr_handler},
{GEN8_PCU_IER, 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_ier_handler},
{GEN8_PCU_IIR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_iir_handler},
{GEN8_PCU_ISR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{GEN8_MASTER_IRQ, 4, F_VIRT, 0, D_BDW_PLUS, NULL, vgt_reg_master_irq_handler},

/* -------render regs---------- */
{_REG_VCS2_HWSTAM, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},

/* maybe an error in Linux driver. meant for VCS_HWS_PGA */
{_REG_VCS2_UHPTR, 4, F_RDR_HWSTS, 0, D_BDW_PLUS, NULL, ring_uhptr_write},

{_REG_VCS2_TAIL, 4, F_RDR, 0, D_BDW_PLUS, ring_mmio_read, ring_mmio_write},
{_REG_VCS2_HEAD, 4, F_RDR, 0, D_BDW_PLUS, ring_mmio_read, ring_mmio_write},
{_REG_VCS2_START, 4, F_RDR_ADRFIX, 0xFFFFF000, D_BDW_PLUS,
	ring_mmio_read, ring_mmio_write},
{_REG_VCS2_CTL, 4, F_VIRT, 0, D_BDW_PLUS, ring_mmio_read, ring_mmio_write},

{_REG_VCS2_ACTHD, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},

{_REG_RCS_ACTHD_UDW, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{_REG_BCS_ACTHD_UDW, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VCS_ACTHD_UDW, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VCS2_ACTHD_UDW, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VECS_ACTHD_UDW, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},

{_REG_VCS2_MFX_MODE_BDW, 4, F_RDR_MODE, 0, D_BDW_PLUS, NULL, NULL},

{_REG_VCS2_MI_MODE, 4, F_RDR_MODE, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VCS2_INSTPM, 4, F_RDR_MODE, 0, D_BDW_PLUS, NULL, NULL},

{_REG_VCS2_TIMESTAMP, 8, F_PT_RO, 0, D_BDW_PLUS, NULL, NULL},

{_REG_RCS_EXECLIST_SUBMITPORT, 4, F_VIRT, 0, D_BDW_PLUS,
			vgt_not_allowed_mmio_read, vgt_write_submitport},
{_REG_VCS_EXECLIST_SUBMITPORT, 4, F_VIRT, 0, D_BDW_PLUS,
			vgt_not_allowed_mmio_read, vgt_write_submitport},
{_REG_VECS_EXECLIST_SUBMITPORT, 4, F_VIRT, 0, D_BDW_PLUS,
			vgt_not_allowed_mmio_read, vgt_write_submitport},
{_REG_VCS2_EXECLIST_SUBMITPORT, 4, F_VIRT, 0, D_BDW_PLUS,
			vgt_not_allowed_mmio_read, vgt_write_submitport},
{_REG_BCS_EXECLIST_SUBMITPORT, 4, F_VIRT, 0, D_BDW_PLUS,
			vgt_not_allowed_mmio_read, vgt_write_submitport},

{_REG_RCS_EXECLIST_STATUS, 8, F_RDR, 0, D_BDW_PLUS, NULL,
					vgt_not_allowed_mmio_write},
{_REG_VCS_EXECLIST_STATUS, 8, F_RDR, 0, D_BDW_PLUS, NULL,
					vgt_not_allowed_mmio_write},
{_REG_VECS_EXECLIST_STATUS, 8, F_RDR, 0, D_BDW_PLUS, NULL,
					vgt_not_allowed_mmio_write},
{_REG_VCS2_EXECLIST_STATUS, 8, F_RDR, 0, D_BDW_PLUS, NULL,
					vgt_not_allowed_mmio_write},
{_REG_BCS_EXECLIST_STATUS, 8, F_RDR, 0, D_BDW_PLUS, NULL,
					vgt_not_allowed_mmio_write},

{_REG_RCS_CTX_SR_CTL, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VCS_CTX_SR_CTL, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VECS_CTX_SR_CTL, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VCS2_CTX_SR_CTL, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{_REG_BCS_CTX_SR_CTL, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},

{_REG_RCS_CTX_STATUS_BUF, 48, F_VIRT, 0, D_BDW_PLUS, NULL,
					vgt_not_allowed_mmio_write},
{_REG_VCS_CTX_STATUS_BUF, 48, F_VIRT, 0, D_BDW_PLUS, NULL,
					vgt_not_allowed_mmio_write},
{_REG_VECS_CTX_STATUS_BUF, 48, F_VIRT, 0, D_BDW_PLUS, NULL,
					vgt_not_allowed_mmio_write},
{_REG_VCS2_CTX_STATUS_BUF, 48, F_VIRT, 0, D_BDW_PLUS, NULL,
					vgt_not_allowed_mmio_write},
{_REG_BCS_CTX_STATUS_BUF, 48, F_VIRT, 0, D_BDW_PLUS, NULL,
					vgt_not_allowed_mmio_write},

{_REG_RCS_CTX_STATUS_PTR, 4, F_VIRT | VGT_REG_MODE_CTL, 0, D_BDW_PLUS, vgt_read_ctx_status_ptr,
	vgt_write_ctx_status_ptr},
{_REG_VCS_CTX_STATUS_PTR, 4, F_VIRT | VGT_REG_MODE_CTL, 0, D_BDW_PLUS, vgt_read_ctx_status_ptr,
	vgt_write_ctx_status_ptr},
{_REG_VECS_CTX_STATUS_PTR, 4, F_VIRT | VGT_REG_MODE_CTL, 0, D_BDW_PLUS, vgt_read_ctx_status_ptr,
	vgt_write_ctx_status_ptr},
{_REG_VCS2_CTX_STATUS_PTR, 4, F_VIRT | VGT_REG_MODE_CTL, 0, D_BDW_PLUS, vgt_read_ctx_status_ptr,
	vgt_write_ctx_status_ptr},
{_REG_BCS_CTX_STATUS_PTR, 4, F_VIRT | VGT_REG_MODE_CTL, 0, D_BDW_PLUS, vgt_read_ctx_status_ptr,
	vgt_write_ctx_status_ptr},
	/* -------display regs---------- */

{_PIPE_MISC_A, 4, F_DPY, 0, D_BDW_PLUS, NULL, NULL},

{_PIPE_MISC_B, 4, F_DPY, 0, D_BDW_PLUS, NULL, NULL},

{_REG_PIPE_MISC_C, 4, F_DPY, 0, D_BDW_PLUS, NULL, NULL},

	/* -------others---------- */

	/* -------un-categorized regs--------- */
/* no definition on this. from Linux */
{0x1c1d0, 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{GEN6_MBCUNIT_SNPCR, 4, F_DOM0, 0, D_BDW_PLUS, NULL, NULL},
{GEN7_MISCCPCTL, 4, F_DOM0, 0, D_BDW_PLUS, NULL, NULL},

{0x1C054, 4, F_DOM0, 0, D_BDW_PLUS, NULL, NULL},
/* BDW */
{GEN8_PRIVATE_PAT_LO, 4, F_DOM0, 0, D_BDW_PLUS, NULL, gen8_ppat_write},
{GEN8_PRIVATE_PAT_HI, 4, F_DOM0, 0, D_BDW_PLUS, NULL, gen8_ppat_write},

{GAMTARBMODE, 4, F_DOM0, 0, D_BDW_PLUS, NULL, NULL},

{_REG_RCS_PDP_UDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_RCS_PDP_LDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_RCS_PDP_UDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_RCS_PDP_LDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_RCS_PDP_UDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_RCS_PDP_LDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_RCS_PDP_UDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_RCS_PDP_LDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_VCS_PDP_UDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VCS_PDP_LDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_VCS_PDP_UDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VCS_PDP_LDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_VCS_PDP_UDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VCS_PDP_LDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_VCS_PDP_UDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VCS_PDP_LDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_VECS_PDP_UDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VECS_PDP_LDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_VECS_PDP_UDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VECS_PDP_LDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_VECS_PDP_UDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VECS_PDP_LDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_VECS_PDP_UDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VECS_PDP_LDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_VCS2_PDP_UDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VCS2_PDP_LDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_VCS2_PDP_UDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VCS2_PDP_LDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_VCS2_PDP_UDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VCS2_PDP_LDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_VCS2_PDP_UDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_VCS2_PDP_LDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_BCS_PDP_UDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_BCS_PDP_LDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_BCS_PDP_UDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_BCS_PDP_LDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_BCS_PDP_UDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_BCS_PDP_LDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{_REG_BCS_PDP_UDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{_REG_BCS_PDP_LDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{0x2080, 4, F_RDR_ADRFIX, 0xFFFFF000, D_BDW_PLUS, NULL, NULL},
{0x12080, 4, F_RDR_ADRFIX, 0xFFFFF000, D_BDW_PLUS, NULL, NULL},
{0x1c080, 4, F_RDR_ADRFIX, 0xFFFFF000, D_BDW_PLUS, NULL, NULL},
{0x1a080, 4, F_RDR_ADRFIX, 0xFFFFF000, D_BDW_PLUS, NULL, NULL},
{0x22080, 4, F_RDR_ADRFIX, 0xFFFFF000, D_BDW_PLUS, NULL, NULL},

{0x7300, 4, F_RDR_MODE, 0, D_BDW_PLUS, NULL, NULL},

{0x420b0, 4, F_DPY, 0, D_BDW_PLUS, NULL, NULL},
{0x420b4, 4, F_DPY, 0, D_BDW_PLUS, NULL, NULL},
{0x420b8, 4, F_DPY, 0, D_BDW_PLUS, NULL, NULL},

{0x45260, 4, F_DPY, 0, D_BDW, NULL, NULL},
{0x6f800, 4, F_DPY, 0, D_BDW, NULL, NULL},

{0x66c00, 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{0x66c04, 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{0x4024, 4, F_DOM0, 0, D_BDW_PLUS, NULL, NULL},

{0x9134, 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{0x9138, 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{0x913c, 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

/* WA */
{0xfdc, 4, F_DOM0, 0, D_BDW_PLUS, NULL, NULL},
{0xe4f0, 4, F_RDR_MODE, 0, D_BDW_PLUS, NULL, NULL},
{0xe4f4, 4, F_RDR_MODE, 0, D_BDW_PLUS, NULL, NULL},
{0x9430, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},

/* L3 */
{0xb1f0, 4, F_RDR, 0, D_BDW, NULL, NULL},
{0xb1c0, 4, F_RDR, 0, D_BDW, NULL, NULL},
{0xb118, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
{0xb100, 4, F_RDR, 0, D_BDW, NULL, NULL},
{0xb10c, 4, F_RDR, 0, D_BDW, NULL, NULL},
{0xb110, 4, F_DOM0, 0, D_BDW, NULL, NULL},

/* NON-PRIV */
{0x24d0, 4, F_RDR, 0, D_BDW_PLUS, NULL, vgt_write_force_nonpriv},
{0x24d4, 4, F_RDR, 0, D_SKL_PLUS, NULL, vgt_write_force_nonpriv},
{0x24d8, 4, F_RDR, 0, D_SKL_PLUS, NULL, vgt_write_force_nonpriv},


{0x83a4, 4, F_RDR, 0, D_BDW, NULL, NULL},
{0x4dd4, 4, F_DOM0, 0, D_BDW_PLUS, NULL, NULL},

/* UCG */
{0x8430, 4, F_RDR, 0, D_BDW, NULL, NULL},

{0x110000, 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

{0x48400, 4, F_DPY, 0, D_BDW_PLUS, NULL, NULL},

{0x6e570, 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
{0x65f10, 4, F_DOM0, 0, D_BDW_PLUS, NULL, NULL},

{0xe194, 4, F_RDR_MODE, 0, D_BDW_PLUS, NULL, NULL},
{0xe188, 4, F_RDR_MODE, 0, D_BDW_PLUS, NULL, NULL},
{0xe180, 4, F_RDR_MODE, 0, D_BDW_PLUS, NULL, NULL},
{0x2580, 4, F_RDR_MODE, 0, D_BDW_PLUS, NULL, NULL},

{0x2248, 4, F_RDR, 0, D_BDW, NULL, NULL},
};

reg_attr_t vgt_reg_info_skl[] = {
{FORCEWAKE_RENDER_GEN9, 4, F_VIRT, 0, D_SKL_PLUS, NULL, mul_force_wake_write},
{FORCEWAKE_ACK_RENDER_GEN9, 4, F_VIRT, 0, D_SKL_PLUS, mul_force_wake_ack_read, NULL},
{FORCEWAKE_BLITTER_GEN9, 4, F_VIRT, 0, D_SKL_PLUS, NULL, mul_force_wake_write},
{FORCEWAKE_ACK_BLITTER_GEN9, 4, F_VIRT, 0, D_SKL_PLUS, mul_force_wake_ack_read, NULL},
{FORCEWAKE_MEDIA_GEN9, 4, F_VIRT, 0, D_SKL_PLUS, NULL, mul_force_wake_write},
{FORCEWAKE_ACK_MEDIA_GEN9, 4, F_VIRT, 0, D_SKL_PLUS, mul_force_wake_ack_read, NULL},
{DPB_AUX_CH_CTL, 6*4, F_DPY, 0, D_SKL, NULL, dp_aux_ch_ctl_mmio_write},
{DPC_AUX_CH_CTL, 6*4, F_DPY, 0, D_SKL, NULL, dp_aux_ch_ctl_mmio_write},
{DPD_AUX_CH_CTL, 6*4, F_DPY, 0, D_SKL, NULL, dp_aux_ch_ctl_mmio_write},
{HSW_PWR_WELL_BIOS, 4, F_DOM0, 0, D_SKL, NULL, NULL},
{HSW_PWR_WELL_DRIVER, 4, F_DOM0, 0, D_SKL, NULL, skl_power_well_ctl_write},
{GEN6_PCODE_MAILBOX, 4, F_DOM0, 0, D_SKL, NULL, mailbox_write},
{0xa210, 4, F_DOM0, 0, D_SKL_PLUS, NULL, NULL},
{GEN9_MEDIA_PG_IDLE_HYSTERESIS, 4, F_DOM0, 0, D_SKL_PLUS, NULL, NULL},
{GEN9_RENDER_PG_IDLE_HYSTERESIS, 4, F_DOM0, 0, D_SKL_PLUS, NULL, NULL},
{0x4ddc, 4, F_VIRT, 0, D_SKL, NULL, vgt_reg_write_misc_ctl_handler},
{0x42080, 4, F_VIRT, 0, D_SKL_PLUS, NULL, vgt_reg_write_misc_ctl_handler},
{0x45504, 4, F_DOM0, 0, D_SKL, NULL, NULL},
{0x45520, 4, F_DOM0, 0, D_SKL, NULL, NULL},
{0x46000, 4, F_DPY, 0, D_SKL, NULL, NULL},
{0x46010, 4, F_DPY, 0, D_SKL, NULL, skl_lcpll_write},
{0x46014, 4, F_DPY, 0, D_SKL, NULL, skl_lcpll_write},
{0x6C040, 4, F_DPY, 0, D_SKL, NULL, NULL},
{0x6C048, 4, F_DPY, 0, D_SKL, NULL, NULL},
{0x6C050, 4, F_DPY, 0, D_SKL, NULL, NULL},
{0x6C044, 4, F_DPY, 0, D_SKL, NULL, NULL},
{0x6C04C, 4, F_DPY, 0, D_SKL, NULL, NULL},
{0x6C054, 4, F_DPY, 0, D_SKL, NULL, NULL},
{0x6c058, 4, F_DPY, 0, D_SKL, NULL, NULL},
{0x6c05c, 4, F_DPY, 0, D_SKL, NULL, NULL},
{0X6c060, 4, F_DPY, 0, D_SKL, dpll_status_read, NULL},

{SKL_PS_WIN_POS(PIPE_A, 0), 4, F_DPY, 0, D_SKL, pf_read, pf_write},
{SKL_PS_WIN_POS(PIPE_A, 1), 4, F_DPY, 0, D_SKL, pf_read, pf_write},
{SKL_PS_WIN_POS(PIPE_B, 0), 4, F_DPY, 0, D_SKL, pf_read, pf_write},
{SKL_PS_WIN_POS(PIPE_B, 1), 4, F_DPY, 0, D_SKL, pf_read, pf_write},
{SKL_PS_WIN_POS(PIPE_C, 0), 4, F_DPY, 0, D_SKL, pf_read, pf_write},
{SKL_PS_WIN_POS(PIPE_C, 1), 4, F_DPY, 0, D_SKL, pf_read, pf_write},

{SKL_PS_WIN_SZ(PIPE_A, 0), 4, F_DPY, 0, D_SKL, pf_read, pf_write},
{SKL_PS_WIN_SZ(PIPE_A, 1), 4, F_DPY, 0, D_SKL, pf_read, pf_write},
{SKL_PS_WIN_SZ(PIPE_B, 0), 4, F_DPY, 0, D_SKL, pf_read, pf_write},
{SKL_PS_WIN_SZ(PIPE_B, 1), 4, F_DPY, 0, D_SKL, pf_read, pf_write},
{SKL_PS_WIN_SZ(PIPE_C, 0), 4, F_DPY, 0, D_SKL, pf_read, pf_write},
{SKL_PS_WIN_SZ(PIPE_C, 1), 4, F_DPY, 0, D_SKL, pf_read, pf_write},

{SKL_PS_CTRL(PIPE_A, 0), 4, F_DPY, 0, D_SKL, pf_read, pf_write},
{SKL_PS_CTRL(PIPE_A, 1), 4, F_DPY, 0, D_SKL, pf_read, pf_write},
{SKL_PS_CTRL(PIPE_B, 0), 4, F_DPY, 0, D_SKL, pf_read, pf_write},
{SKL_PS_CTRL(PIPE_B, 1), 4, F_DPY, 0, D_SKL, pf_read, pf_write},
{SKL_PS_CTRL(PIPE_C, 0), 4, F_DPY, 0, D_SKL, pf_read, pf_write},
{SKL_PS_CTRL(PIPE_C, 1), 4, F_DPY, 0, D_SKL, pf_read, pf_write},

{PLANE_BUF_CFG(PIPE_A, 0), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_BUF_CFG(PIPE_A, 1), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_BUF_CFG(PIPE_A, 2), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_BUF_CFG(PIPE_A, 3), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{PLANE_BUF_CFG(PIPE_B, 0), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_BUF_CFG(PIPE_B, 1), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_BUF_CFG(PIPE_B, 2), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_BUF_CFG(PIPE_B, 3), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{PLANE_BUF_CFG(PIPE_C, 0), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_BUF_CFG(PIPE_C, 1), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_BUF_CFG(PIPE_C, 2), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_BUF_CFG(PIPE_C, 3), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{CUR_BUF_CFG(PIPE_A), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{CUR_BUF_CFG(PIPE_B), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{CUR_BUF_CFG(PIPE_C), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{PLANE_WM(PIPE_A, 0, 0), 4 * 8, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_WM(PIPE_A, 1, 0), 4 * 8, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_WM(PIPE_A, 2, 0), 4 * 8, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{PLANE_WM(PIPE_B, 0, 0), 4 * 8, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_WM(PIPE_B, 1, 0), 4 * 8, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_WM(PIPE_B, 2, 0), 4 * 8, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{PLANE_WM(PIPE_C, 0, 0), 4 * 8, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_WM(PIPE_C, 1, 0), 4 * 8, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_WM(PIPE_C, 2, 0), 4 * 8, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{CUR_WM(PIPE_A, 0), 4 * 8, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{CUR_WM(PIPE_B, 0), 4 * 8, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{CUR_WM(PIPE_C, 0), 4 * 8, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{PLANE_WM_TRANS(PIPE_A, 0), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_WM_TRANS(PIPE_A, 1), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_WM_TRANS(PIPE_A, 2), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{PLANE_WM_TRANS(PIPE_B, 0), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_WM_TRANS(PIPE_B, 1), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_WM_TRANS(PIPE_B, 2), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{PLANE_WM_TRANS(PIPE_C, 0), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_WM_TRANS(PIPE_C, 1), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_WM_TRANS(PIPE_C, 2), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{CUR_WM_TRANS(PIPE_A), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{CUR_WM_TRANS(PIPE_B), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{CUR_WM_TRANS(PIPE_C), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{PLANE_NV12_BUF_CFG(PIPE_A, 0), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_NV12_BUF_CFG(PIPE_A, 1), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_NV12_BUF_CFG(PIPE_A, 2), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_NV12_BUF_CFG(PIPE_A, 3), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{PLANE_NV12_BUF_CFG(PIPE_B, 0), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_NV12_BUF_CFG(PIPE_B, 1), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_NV12_BUF_CFG(PIPE_B, 2), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_NV12_BUF_CFG(PIPE_B, 3), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{PLANE_NV12_BUF_CFG(PIPE_C, 0), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_NV12_BUF_CFG(PIPE_C, 1), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_NV12_BUF_CFG(PIPE_C, 2), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{PLANE_NV12_BUF_CFG(PIPE_C, 3), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{_REG_701C0(PIPE_A, 1), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C0(PIPE_A, 2), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C0(PIPE_A, 3), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C0(PIPE_A, 4), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{_REG_701C0(PIPE_B, 1), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C0(PIPE_B, 2), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C0(PIPE_B, 3), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C0(PIPE_B, 4), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{_REG_701C0(PIPE_C, 1), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C0(PIPE_C, 2), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C0(PIPE_C, 3), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C0(PIPE_C, 4), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{_REG_701C4(PIPE_A, 1), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C4(PIPE_A, 2), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C4(PIPE_A, 3), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C4(PIPE_A, 4), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{_REG_701C4(PIPE_B, 1), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C4(PIPE_B, 2), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C4(PIPE_B, 3), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C4(PIPE_B, 4), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{_REG_701C4(PIPE_C, 1), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C4(PIPE_C, 2), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C4(PIPE_C, 3), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},
{_REG_701C4(PIPE_C, 4), 4, F_DPY, 0, D_SKL, NULL, dpy_plane_mmio_write},

{0x70380, 4, F_DPY, 0, D_SKL, NULL, NULL},
{0x71380, 4, F_DPY, 0, D_SKL, NULL, NULL},
{0x72380, 4, F_DPY, 0, D_SKL, NULL, NULL},
{0x7039c, 4, F_DPY, 0, D_SKL, NULL, NULL},

{0x80000, 0x3000, F_DPY, 0, D_SKL, NULL, NULL},
{0x8f074, 4, F_DPY, 0, D_SKL, NULL, NULL},
{0x8f004, 4, F_DPY, 0, D_SKL, NULL, NULL},
{0x8f034, 4, F_DPY, 0, D_SKL, NULL, NULL},

{0xb11c, 4, F_DOM0, 0, D_SKL, NULL, NULL},

{0x51000, 4, F_VIRT, 0, D_SKL, NULL, NULL},
{0x6c00c, 4, F_DPY, 0, D_SKL, NULL, NULL},

{0xc800, 0x7f8, F_RDR, 0, D_SKL, NULL, NULL},
{0xb020, 0x80, F_RDR, 0, D_SKL, NULL, NULL},

{0xd08, 4, F_VIRT, 0, D_SKL, NULL, NULL},
{0x20e0, 4, F_DOM0, 0, D_SKL, NULL, NULL},
{0x20ec, 4, F_RDR_MODE, 0, D_SKL, NULL, NULL},

{0x4de0, 4, F_RDR, 0, D_SKL, NULL, NULL},
{0x4de4, 4, F_RDR, 0, D_SKL, NULL, NULL},
{0x4de8, 4, F_RDR, 0, D_SKL, NULL, NULL},
{0x4dec, 4, F_RDR, 0, D_SKL, NULL, NULL},
{0x4df0, 4, F_RDR, 0, D_SKL, NULL, NULL},
{0x4df4, 4, F_RDR, 0, D_SKL, NULL, NULL},
{0x4dfc, 4, F_VIRT, 0, D_SKL, NULL, NULL},

{0x45008, 4, F_VIRT, 0, D_SKL, NULL, NULL},

{0x46430, 4, F_VIRT, 0, D_SKL, NULL, NULL},

{0x46520, 4, F_VIRT, 0, D_SKL, NULL, NULL},

{0xc403c, 4, F_VIRT, 0, D_SKL, NULL, NULL},
{0xb004, 4, F_DOM0, 0, D_SKL, NULL, NULL},
{DMA_CTRL, 4, F_DOM0, 0, D_SKL_PLUS, NULL, dma_ctrl_write},

{0x65900, 4, F_DOM0, 0, D_SKL, NULL, NULL},
{0x1082c0, 4, F_DOM0, 0, D_SKL, NULL, NULL},
{0x4068, 4, F_DOM0, 0, D_SKL, NULL, NULL},
{0x67054, 4, F_DOM0, 0, D_SKL, NULL, NULL},
{0x6e560, 4, F_DOM0, 0, D_SKL, NULL, NULL},
{0x6e544, 4, F_DOM0, 0, D_SKL, NULL, NULL},
{0x2b20, 4, F_DOM0, 0, D_SKL, NULL, NULL},
{0x65f00, 4, F_DOM0, 0, D_SKL, NULL, NULL},
{0x65f08, 4, F_DOM0, 0, D_SKL, NULL, NULL},
{0x320f0, 4, F_DOM0, 0, D_SKL, NULL, NULL},

{_REG_VCS2_EXCC, 4, F_RDR, 0, D_SKL, NULL, NULL},
{_REG_VECS_EXCC, 4, F_RDR, 0, D_SKL, NULL, NULL},
{0x70034, 4, F_DPY, 0, D_SKL, NULL, NULL},
{0x71034, 4, F_DPY, 0, D_SKL, NULL, NULL},
{0x72034, 4, F_DPY, 0, D_SKL, NULL, NULL},

{_PLANE_KEYVAL_1(PIPE_A), 4, F_DPY, 0, D_SKL, NULL, NULL},
{_PLANE_KEYVAL_1(PIPE_B), 4, F_DPY, 0, D_SKL, NULL, NULL},
{_PLANE_KEYVAL_1(PIPE_C), 4, F_DPY, 0, D_SKL, NULL, NULL},
{_PLANE_KEYMSK_1(PIPE_A), 4, F_DPY, 0, D_SKL, NULL, NULL},
{_PLANE_KEYMSK_1(PIPE_B), 4, F_DPY, 0, D_SKL, NULL, NULL},
{_PLANE_KEYMSK_1(PIPE_C), 4, F_DPY, 0, D_SKL, NULL, NULL},

{SPRKEYMAX(PIPE_A), 4, F_DPY, 0, D_SKL, NULL, NULL},
{SPRKEYMAX(PIPE_B), 4, F_DPY, 0, D_SKL, NULL, NULL},
{SPRKEYMAX(PIPE_C), 4, F_DPY, 0, D_SKL, NULL, NULL},
{SPRPOS(PIPE_A), 4, F_DPY, 0, D_SKL, NULL, NULL},
{SPRPOS(PIPE_B), 4, F_DPY, 0, D_SKL, NULL, NULL},
{SPRPOS(PIPE_C), 4, F_DPY, 0, D_SKL, NULL, NULL},
{SPRKEYVAL(PIPE_A), 4, F_DPY, 0, D_SKL, NULL, NULL},
{SPRKEYVAL(PIPE_B), 4, F_DPY, 0, D_SKL, NULL, NULL},
{SPRKEYVAL(PIPE_C), 4, F_DPY, 0, D_SKL, NULL, NULL},
{SPRKEYMSK(PIPE_A), 4, F_DPY, 0, D_SKL, NULL, NULL},
{SPRKEYMSK(PIPE_B), 4, F_DPY, 0, D_SKL, NULL, NULL},
{SPRKEYMSK(PIPE_C), 4, F_DPY, 0, D_SKL, NULL, NULL},
{SPROFFSET(PIPE_A), 4, F_DPY, 0, D_SKL, NULL, NULL},
{SPROFFSET(PIPE_B), 4, F_DPY, 0, D_SKL, NULL, NULL},
{SPROFFSET(PIPE_C), 4, F_DPY, 0, D_SKL, NULL, NULL},

{VGT_SPRSTRIDE(PIPE_A), 4, F_DPY, 0, D_SKL, NULL, NULL},
{VGT_SPRSTRIDE(PIPE_B), 4, F_DPY, 0, D_SKL, NULL, NULL},
{VGT_SPRSTRIDE(PIPE_C), 4, F_DPY, 0, D_SKL, NULL, NULL},

{SPRSIZE(PIPE_A), 4, F_DPY, 0, D_SKL, NULL, NULL},
{SPRSIZE(PIPE_B), 4, F_DPY, 0, D_SKL, NULL, NULL},
{SPRSIZE(PIPE_C), 4, F_DPY, 0, D_SKL, NULL, NULL},
{0x44500, 4, F_DPY, 0, D_SKL, NULL, NULL},
};

static void vgt_passthrough_execlist(struct pgt_device *pdev)
{
       int i;
       reg_update_handlers(pdev, _REG_RCS_CTX_STATUS_BUF, 48, NULL, NULL);
       reg_update_handlers(pdev, _REG_VCS_CTX_STATUS_BUF, 48, NULL, NULL);
       reg_update_handlers(pdev, _REG_VECS_CTX_STATUS_BUF, 48, NULL, NULL);
       reg_update_handlers(pdev, _REG_VCS2_CTX_STATUS_BUF, 48, NULL, NULL);
       reg_update_handlers(pdev, _REG_BCS_CTX_STATUS_BUF, 48, NULL, NULL);

       for (i = 0; i < 48; i += 4) {
               reg_change_owner(pdev, _REG_RCS_CTX_STATUS_BUF + i, VGT_OT_RENDER);
               reg_change_owner(pdev, _REG_VCS_CTX_STATUS_BUF + i, VGT_OT_RENDER);
               reg_change_owner(pdev, _REG_VECS_CTX_STATUS_BUF + i, VGT_OT_RENDER);
               reg_change_owner(pdev, _REG_VCS2_CTX_STATUS_BUF + i, VGT_OT_RENDER);
               reg_change_owner(pdev, _REG_BCS_CTX_STATUS_BUF + i, VGT_OT_RENDER);
       }

       reg_update_handlers(pdev, _REG_RCS_CTX_STATUS_PTR, 4, NULL, NULL);
       reg_update_handlers(pdev, _REG_VCS_CTX_STATUS_PTR, 4, NULL, NULL);
       reg_update_handlers(pdev, _REG_VECS_CTX_STATUS_PTR, 4, NULL, NULL);
       reg_update_handlers(pdev, _REG_VCS2_CTX_STATUS_PTR, 4, NULL, NULL);
       reg_update_handlers(pdev, _REG_BCS_CTX_STATUS_PTR, 4, NULL, NULL);

       reg_change_owner(pdev, _REG_RCS_CTX_STATUS_PTR, F_RDR);
       reg_change_owner(pdev, _REG_VCS_CTX_STATUS_PTR, F_RDR);
       reg_change_owner(pdev, _REG_VECS_CTX_STATUS_PTR, F_RDR);
       reg_change_owner(pdev, _REG_VCS2_CTX_STATUS_PTR, F_RDR);
       reg_change_owner(pdev, _REG_BCS_CTX_STATUS_PTR, F_RDR);
}

bool vgt_post_setup_mmio_hooks(struct pgt_device *pdev)
{
	printk("post mmio hooks initialized\n");

	if (hvm_render_owner)
		vgt_passthrough_execlist(pdev);

	if (!pdev->enable_ppgtt)
		return true;

	vgt_dbg(VGT_DBG_MEM,"Hook up PPGTT register handlers\n");

	if (IS_PREBDW(pdev)) {
		/* trap PPGTT base register */
		reg_update_handlers(pdev, _REG_RCS_PP_DIR_BASE_IVB, 4,
				pp_dir_base_read, pp_dir_base_write);
		reg_update_handlers(pdev, _REG_BCS_PP_DIR_BASE, 4,
				pp_dir_base_read, pp_dir_base_write);
		reg_update_handlers(pdev, _REG_VCS_PP_DIR_BASE, 4,
				pp_dir_base_read, pp_dir_base_write);

		reg_update_handlers(pdev, _REG_RCS_PP_DCLV, 4,
				pp_dclv_read, pp_dclv_write);
		reg_update_handlers(pdev, _REG_BCS_PP_DCLV, 4,
				pp_dclv_read, pp_dclv_write);
		reg_update_handlers(pdev, _REG_VCS_PP_DCLV, 4,
				pp_dclv_read, pp_dclv_write);

		if (IS_HSW(pdev)) {
			reg_update_handlers(pdev, _REG_VECS_PP_DIR_BASE, 4,
					pp_dir_base_read,
					pp_dir_base_write);
			reg_update_handlers(pdev, _REG_VECS_PP_DCLV, 4,
					pp_dclv_read, pp_dclv_write);
		}
	}

	/* XXX cache register? */
	/* PPGTT enable register */
	reg_update_handlers(pdev, GFX_MODE_GEN7, 4,
			ring_pp_mode_read, ring_pp_mode_write);
	reg_update_handlers(pdev, _REG_BCS_BLT_MODE_IVB, 4,
			ring_pp_mode_read, ring_pp_mode_write);
	reg_update_handlers(pdev, _REG_VCS_MFX_MODE_IVB, 4,
			ring_pp_mode_read, ring_pp_mode_write);

	if (IS_HSW(pdev) || IS_BDWPLUS(pdev))
		reg_update_handlers(pdev, _REG_VEBOX_MODE, 4,
				ring_pp_mode_read,
				ring_pp_mode_write);

	if (IS_BDWGT3(pdev) || IS_SKLGT3(pdev) || IS_SKLGT4(pdev)) {
		reg_update_handlers(pdev, _REG_VCS2_MFX_MODE_BDW, 4,
				ring_pp_mode_read,
				ring_pp_mode_write);
	}

	return true;
}

int vgt_get_reg_num(int type)
{
	switch(type){
		case D_ALL:
			return ARRAY_NUM(vgt_reg_info_general);
		case D_BDW:
			return ARRAY_NUM(vgt_reg_info_bdw);
		case D_SKL:
			return ARRAY_NUM(vgt_reg_info_skl);
		default:
			return 0;
	}

	return 0;
}

/*
 * This array lists registers which stick to original policy, as
 * specified in vgt_base_reg_info, and not impacted by the super
 * owner mode (which has most registers owned by HVM instead of
 * dom0).
 *
 * Currently the registers in this list are those, which must be
 * virtualized, with XenGT driver itself as the exclusive owner.
 * Some features like monitor hotplug may be broken, due to the
 * whole handling flow already fixed (first to dom0). But that
 * should be fine, since super owner mode is used for analyze
 * basic stability issues.
 */
reg_list_t vgt_gen7_sticky_regs[] = {
	/* interrupt control registers */
	{GTIMR, 4},
	{GTIER, 4},
	{GTIIR, 4},
	{GTISR, 4},
	{IMR, 4},
	{_REG_BCS_IMR, 4},
	{_REG_VCS_IMR, 4},
	{_REG_VECS_IMR, 4},
	{DEIMR, 4},
	{DEIER, 4},
	{DEIIR, 4},
	{DEISR, 4},
	{SDEIMR, 4},
	{SDEIER, 4},
	{SDEIIR, 4},
	{SDEISR, 4},
	{GEN6_PMIMR, 4},
	{GEN6_PMIER, 4},
	{GEN6_PMIIR, 4},
	{GEN6_PMISR, 4},

	/* PPGTT related registers */
	{GFX_MODE_GEN7, 4},
	{_REG_VCS_MFX_MODE_IVB, 4},
	{_REG_BCS_BLT_MODE_IVB, 4},
	{_REG_VEBOX_MODE, 4},
	{_REG_RCS_PP_DIR_BASE_IVB, 4},
	{_REG_VCS_PP_DIR_BASE, 4},
	{_REG_BCS_PP_DIR_BASE, 4},
	{_REG_VECS_PP_DIR_BASE, 4},
	{_REG_RCS_PP_DCLV, 4},
	{_REG_VCS_PP_DCLV, 4},
	{_REG_BCS_PP_DCLV, 4},
	{_REG_VECS_PP_DCLV, 4},

	/* forcewake */
	{FORCEWAKE, 4},
	{FORCEWAKE_ACK, 4},
	{_REG_GT_CORE_STATUS, 4},
	{GEN6_GT_THREAD_STATUS_REG, 4},
	{GTFIFODBG, 4},
	{GTFIFOCTL, 4},
	{FORCEWAKE_MT, 4},
	{LCPLL_CTL, 4},
	{FORCEWAKE_ACK_HSW, 4},

	/* misc */
	{GEN6_GDRST, 4},
	{_REG_FENCE_0_LOW, 0x80},
	{VGT_PVINFO_PAGE, VGT_PVINFO_SIZE},
	{CPU_VGACNTRL, 4},
};

reg_list_t vgt_gen8_sticky_regs[] = {
	/* interrupt control registers */
	{IMR, 4},
	{_REG_BCS_IMR, 4},
	{_REG_VCS_IMR, 4},
	{_REG_VCS2_IMR, 4},
	{_REG_VECS_IMR, 4},

	{SDEIMR, 4},
	{SDEIER, 4},
	{SDEIIR, 4},
	{SDEISR, 4},

	/* Interrupt registers - BDW */
	{_REG_GT_IMR(0), 4},
	{_REG_GT_IER(0), 4},
	{_REG_GT_IIR(0), 4},
	{_REG_GT_ISR(0), 4},

	{_REG_GT_IMR(1), 4},
	{_REG_GT_IER(1), 4},
	{_REG_GT_IIR(1), 4},
	{_REG_GT_ISR(1), 4},

	{_REG_GT_IMR(2), 4},
	{_REG_GT_IER(2), 4},
	{_REG_GT_IIR(2), 4},
	{_REG_GT_ISR(2), 4},

	{_REG_GT_IMR(3), 4},
	{_REG_GT_IER(3), 4},
	{_REG_GT_IIR(3), 4},
	{_REG_GT_ISR(3), 4},

	{_REG_DE_PIPE_IMR(PIPE_A), 4},
	{_REG_DE_PIPE_IER(PIPE_A), 4},
	{_REG_DE_PIPE_IIR(PIPE_A), 4},
	{_REG_DE_PIPE_ISR(PIPE_A), 4},

	{_REG_DE_PIPE_IMR(PIPE_B), 4},
	{_REG_DE_PIPE_IER(PIPE_B), 4},
	{_REG_DE_PIPE_IIR(PIPE_B), 4},
	{_REG_DE_PIPE_ISR(PIPE_B), 4},

	{_REG_DE_PIPE_IMR(PIPE_C), 4},
	{_REG_DE_PIPE_IER(PIPE_C), 4},
	{_REG_DE_PIPE_IIR(PIPE_C), 4},
	{_REG_DE_PIPE_ISR(PIPE_C), 4},

	{GEN8_DE_PORT_IMR, 4},
	{GEN8_DE_PORT_IER, 4},
	{GEN8_DE_PORT_IIR, 4},
	{GEN8_DE_PORT_ISR, 4},

	{GEN8_DE_MISC_IMR, 4},
	{GEN8_DE_MISC_IER, 4},
	{GEN8_DE_MISC_IIR, 4},
	{GEN8_DE_MISC_ISR, 4},

	{GEN8_PCU_IMR, 4},
	{GEN8_PCU_IER, 4},
	{GEN8_PCU_IIR, 4},
	{GEN8_PCU_ISR, 4},

	{GEN8_MASTER_IRQ, 4},

	/* PPGTT related registers */
	{GFX_MODE_GEN7, 4},
	{_REG_VCS_MFX_MODE_IVB, 4},
	{_REG_VCS2_MFX_MODE_BDW, 4},
	{_REG_BCS_BLT_MODE_IVB, 4},
	{_REG_VEBOX_MODE, 4},

	/* forcewake */
	{FORCEWAKE, 4},
	{FORCEWAKE_ACK, 4},
	{_REG_GT_CORE_STATUS, 4},
	{GEN6_GT_THREAD_STATUS_REG, 4},
	{GTFIFODBG, 4},
	{GTFIFOCTL, 4},
	{FORCEWAKE_MT, 4},
	{LCPLL_CTL, 4},
	{FORCEWAKE_ACK_HSW, 4},

	/* misc */
	{GEN6_GDRST, 4},
	{_REG_FENCE_0_LOW, 0x80},
	{VGT_PVINFO_PAGE, VGT_PVINFO_SIZE},
	{CPU_VGACNTRL, 4},
};

reg_list_t *vgt_get_sticky_regs(struct pgt_device *pdev)
{
	if (IS_PREBDW(pdev))
		return vgt_gen7_sticky_regs;
	else
		return vgt_gen8_sticky_regs;
}

int vgt_get_sticky_reg_num(struct pgt_device *pdev)
{
	if (IS_PREBDW(pdev))
		return ARRAY_NUM(vgt_gen7_sticky_regs);
	else
		return ARRAY_NUM(vgt_gen8_sticky_regs);
}

reg_addr_sz_t vgt_reg_addr_sz[] = {
	{RENDER_HWS_PGA_GEN7, 4096, D_ALL},
	{BSD_HWS_PGA_GEN7, 4096, D_ALL},
	{BLT_HWS_PGA_GEN7, 4096, D_GEN7PLUS},
	{VEBOX_HWS_PGA_GEN7, 4096, D_GEN7PLUS},
	{_REG_VECS_HWS_PGA, 4096, D_HSW},
	{CCID, HSW_CXT_TOTAL_SIZE, D_HSW},
};

int vgt_get_reg_addr_sz_num()
{
	return ARRAY_NUM(vgt_reg_addr_sz);
}
