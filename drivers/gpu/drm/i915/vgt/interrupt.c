/*
 * vGT interrupt handler
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

#include <linux/linkage.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/list.h>

#include "vgt.h"

/*
 * TODO:
 *   - IIR could store two pending interrupts. need emulate the behavior
 *   - GT has 2nd level IMR registers (render/blitter/video)
 *   - Handle more events (like hdmi/dp hotplug, pipe-c, watchdog, etc.)
 */

/*
 * Below are necessary steps to add a new event handling:
 *   a) (device specific) add bit<->event mapping information in
 *      vgt_base_init_irq
 *
 *   b) (event specific) add event forwarding policy in vgt_init_events
 *
 *      Normally those are the only steps required, if the event is only
 *      associated to the 1st leve interrupt control registers (iir/ier
 *      imr/isr). The default handler will take care automatically
 *
 *      In the case where the event is associated with status/control
 *      bits in other registers (e.g. monitor hotplug), you'll provide
 *      specific handler for both physical event and virtual event
 *
 *   c) create a vgt_handle_XXX_phys handler, which deals with any required
 *      housekeeping, and may optionally cache some state to be forwarded
 *      to a VM
 *
 *   d) create a vgt_handle_XXX_virt handler, which emulates a virtual
 *      event generation with any required state emulated accordingly, may
 *      optionally use cached state from p_handler
 *
 *   e) setup virt/phys handler in vgt_init_events
 */
static void vgt_handle_events(struct vgt_irq_host_state *hstate, void *iir,
		struct vgt_irq_info *info);

static void update_upstream_irq(struct vgt_device *vgt,
		struct vgt_irq_info *info);

static int vgt_irq_warn_once[VGT_MAX_VMS+1][EVENT_MAX];

char *vgt_irq_name[EVENT_MAX] = {
	// GT
	[RCS_MI_USER_INTERRUPT] = "Render Command Streamer MI USER INTERRUPT",
	[RCS_DEBUG] = "Render EU debug from SVG",
	[RCS_MMIO_SYNC_FLUSH] = "Render MMIO sync flush status",
	[RCS_CMD_STREAMER_ERR] = "Render Command Streamer error interrupt",
	[RCS_PIPE_CONTROL] = "Render PIPE CONTROL notify",
	[RCS_WATCHDOG_EXCEEDED] = "Render Command Streamer Watchdog counter exceeded",
	[RCS_PAGE_DIRECTORY_FAULT] = "Render page directory faults",
	[RCS_AS_CONTEXT_SWITCH] = "Render AS Context Switch Interrupt",

	[VCS_MI_USER_INTERRUPT] = "Video Command Streamer MI USER INTERRUPT",
	[VCS_MMIO_SYNC_FLUSH] = "Video MMIO sync flush status",
	[VCS_CMD_STREAMER_ERR] = "Video Command Streamer error interrupt",
	[VCS_MI_FLUSH_DW] = "Video MI FLUSH DW notify",
	[VCS_WATCHDOG_EXCEEDED] = "Video Command Streamer Watchdog counter exceeded",
	[VCS_PAGE_DIRECTORY_FAULT] = "Video page directory faults",
	[VCS_AS_CONTEXT_SWITCH] = "Video AS Context Switch Interrupt",
	[VCS2_MI_USER_INTERRUPT] = "VCS2 Video Command Streamer MI USER INTERRUPT",
	[VCS2_MI_FLUSH_DW] = "VCS2 Video MI FLUSH DW notify",
	[VCS2_AS_CONTEXT_SWITCH] = "VCS2 Context Switch Interrupt",

	[BCS_MI_USER_INTERRUPT] = "Blitter Command Streamer MI USER INTERRUPT",
	[BCS_MMIO_SYNC_FLUSH] = "Billter MMIO sync flush status",
	[BCS_CMD_STREAMER_ERR] = "Blitter Command Streamer error interrupt",
	[BCS_MI_FLUSH_DW] = "Blitter MI FLUSH DW notify",
	[BCS_PAGE_DIRECTORY_FAULT] = "Blitter page directory faults",
	[BCS_AS_CONTEXT_SWITCH] = "Blitter AS Context Switch Interrupt",

	[VECS_MI_FLUSH_DW] = "Video Enhanced Streamer MI FLUSH DW notify",
	[VECS_AS_CONTEXT_SWITCH] = "VECS Context Switch Interrupt",

	// DISPLAY
	[PIPE_A_FIFO_UNDERRUN] = "Pipe A FIFO underrun",
	[PIPE_A_CRC_ERR] = "Pipe A CRC error",
	[PIPE_A_CRC_DONE] = "Pipe A CRC done",
	[PIPE_A_VSYNC] = "Pipe A vsync",
	[PIPE_A_LINE_COMPARE] = "Pipe A line compare",
	[PIPE_A_ODD_FIELD] = "Pipe A odd field",
	[PIPE_A_EVEN_FIELD] = "Pipe A even field",
	[PIPE_A_VBLANK] = "Pipe A vblank",
	[PIPE_B_FIFO_UNDERRUN] = "Pipe B FIFO underrun",
	[PIPE_B_CRC_ERR] = "Pipe B CRC error",
	[PIPE_B_CRC_DONE] = "Pipe B CRC done",
	[PIPE_B_VSYNC] = "Pipe B vsync",
	[PIPE_B_LINE_COMPARE] = "Pipe B line compare",
	[PIPE_B_ODD_FIELD] = "Pipe B odd field",
	[PIPE_B_EVEN_FIELD] = "Pipe B even field",
	[PIPE_B_VBLANK] = "Pipe B vblank",
	[PIPE_C_VBLANK] = "Pipe C vblank",
	[DPST_PHASE_IN] = "DPST phase in event",
	[DPST_HISTOGRAM] = "DPST histogram event",
	[GSE] = "GSE",
	[DP_A_HOTPLUG] = "DP A Hotplug",
	[AUX_CHANNEL_A] = "AUX Channel A",
	[PERF_COUNTER] = "Performance counter",
	[POISON] = "Poison",
	[GTT_FAULT] = "GTT fault",
	[PRIMARY_A_FLIP_DONE] = "Primary Plane A flip done",
	[PRIMARY_B_FLIP_DONE] = "Primary Plane B flip done",
	[PRIMARY_C_FLIP_DONE] = "Primary Plane C flip done",
	[SPRITE_A_FLIP_DONE] = "Sprite Plane A flip done",
	[SPRITE_B_FLIP_DONE] = "Sprite Plane B flip done",
	[SPRITE_C_FLIP_DONE] = "Sprite Plane C flip done",

	// PM
	[GV_DOWN_INTERVAL] = "Render geyserville Down evaluation interval interrupt",
	[GV_UP_INTERVAL] = "Render geyserville UP evaluation interval interrupt",
	[RP_DOWN_THRESHOLD] = "RP DOWN threshold interrupt",
	[RP_UP_THRESHOLD] = "RP UP threshold interrupt",
	[FREQ_DOWNWARD_TIMEOUT_RC6] = "Render Frequency Downward Timeout During RC6 interrupt",
	[PCU_THERMAL] = "PCU Thermal Event",
	[PCU_PCODE2DRIVER_MAILBOX] = "PCU pcode2driver mailbox event",

	// PCH
	[FDI_RX_INTERRUPTS_TRANSCODER_A] = "FDI RX Interrupts Combined A",
	[AUDIO_CP_CHANGE_TRANSCODER_A] = "Audio CP Change Transcoder A",
	[AUDIO_CP_REQUEST_TRANSCODER_A] = "Audio CP Request Transcoder A",
	[FDI_RX_INTERRUPTS_TRANSCODER_B] = "FDI RX Interrupts Combined B",
	[AUDIO_CP_CHANGE_TRANSCODER_B] = "Audio CP Change Transcoder B",
	[AUDIO_CP_REQUEST_TRANSCODER_B] = "Audio CP Request Transcoder B",
	[FDI_RX_INTERRUPTS_TRANSCODER_C] = "FDI RX Interrupts Combined C",
	[AUDIO_CP_CHANGE_TRANSCODER_C] = "Audio CP Change Transcoder C",
	[AUDIO_CP_REQUEST_TRANSCODER_C] = "Audio CP Request Transcoder C",
	[ERR_AND_DBG] = "South Error and Debug Interupts Combined",
	[GMBUS] = "Gmbus",
	[SDVO_B_HOTPLUG] = "SDVO B hotplug",
	[CRT_HOTPLUG] = "CRT Hotplug",
	[DP_B_HOTPLUG] = "DisplayPort/HDMI/DVI B Hotplug",
	[DP_C_HOTPLUG] = "DisplayPort/HDMI/DVI C Hotplug",
	[DP_D_HOTPLUG] = "DisplayPort/HDMI/DVI D Hotplug",
	[AUX_CHANNEL_B] = "AUX Channel B",
	[AUX_CHANNEL_C] = "AUX Channel C",
	[AUX_CHANNEL_D] = "AUX Channel D",
	[AUDIO_POWER_STATE_CHANGE_B] = "Audio Power State change Port B",
	[AUDIO_POWER_STATE_CHANGE_C] = "Audio Power State change Port C",
	[AUDIO_POWER_STATE_CHANGE_D] = "Audio Power State change Port D",

	[EVENT_RESERVED] = "RESERVED EVENTS!!!",
};

void reset_cached_interrupt_registers(struct pgt_device *pdev)
{
	struct vgt_irq_host_state *hstate = pdev->irq_hstate;
	struct vgt_irq_info *info;
	u32 reg_base, ier, imr;
	int i;

	for (i = 0; i < IRQ_INFO_MAX; i++) {
		info = hstate->info[i];
		if (!info || info->reg_base == GEN8_MASTER_IRQ)
			continue;

		reg_base = info->reg_base;

		imr = regbase_to_imr(reg_base);
		ier = regbase_to_ier(reg_base);

		__sreg(vgt_dom0, imr) = VGT_MMIO_READ(pdev, imr);
		__sreg(vgt_dom0, ier) = VGT_MMIO_READ(pdev, ier);
	}

	for (i = 0; i < pdev->max_engines; i++) {
		imr = pdev->ring_mmio_base[i] - 0x30 + 0xa8;
		__sreg(vgt_dom0, imr) = VGT_MMIO_READ(pdev, imr);
	}
}

void vgt_reset_virtual_interrupt_registers(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_irq_host_state *hstate = pdev->irq_hstate;
	struct vgt_irq_info *info;
	u32 reg_base, ier, imr, iir, isr;
	int i;

	for_each_set_bit(i, hstate->irq_info_bitmap, IRQ_INFO_MAX) {
		info = hstate->info[i];
		if (!info)
			continue;

		reg_base = hstate->info[i]->reg_base;

		imr = regbase_to_imr(reg_base);
		ier = regbase_to_ier(reg_base);
		iir = regbase_to_iir(reg_base);
		isr = regbase_to_isr(reg_base);

		__vreg(vgt, imr) = 0xffffffff;
		__vreg(vgt, ier) = __vreg(vgt, iir) = __vreg(vgt, isr) = 0x0;
	}

	for (i = 0; i < pdev->max_engines; i++) {
		imr = pdev->ring_mmio_base[i] - 0x30 + 0xa8;
		__vreg(vgt, imr) = 0xffffffff;
	}
}

static inline u32 vgt_read_cached_interrupt_register(struct pgt_device *pdev,
		vgt_reg_t reg)
{
	return __sreg(vgt_dom0, reg);
}

static inline void vgt_write_cached_interrupt_register(struct pgt_device *pdev,
		vgt_reg_t reg, u32 v)
{
	unsigned long flags;

	if (__sreg(vgt_dom0, reg) == v)
		return;

	__sreg(vgt_dom0, reg) = v;

	vgt_get_irq_lock(pdev, flags);

	VGT_MMIO_WRITE(pdev, reg, v);
	VGT_POST_READ(pdev, reg);

	vgt_put_irq_lock(pdev, flags);
}

/* we need to translate interrupts that is pipe related.
* for DE IMR or DE IER, bit 0~4 is interrupts for Pipe A, bit 5~9 is interrupts for Pipe B, bit 10~14 is interrupts
* for pipe C. we can move the interrupts to the right bits when translating interrupts
*/
static u32 gen6_translate_pipe_interrupt(struct vgt_device *vgt, unsigned int reg)
{
	struct vgt_irq_host_state *irq_hstate = vgt->pdev->irq_hstate;
	u32 interrupt = __vreg(vgt, reg);
	int i = 0;
	u32 mapped_interrupt = interrupt;
	u32 temp;

	if (DEIMR == reg) {
		mapped_interrupt |= irq_hstate->pipe_mask;
		mapped_interrupt |= (irq_hstate->pipe_mask << 5);
		mapped_interrupt |= (irq_hstate->pipe_mask << 10);
		// clear the initial mask bit in DEIMR for VBLANKS, so that when pipe mapping
		// is not valid, physically there are still vblanks generated.
		mapped_interrupt &= ~((1 << 0) | (1 << 5) | (1 << 10));
		for (i = 0; i < I915_MAX_PIPES; i++) {
			if (vgt->pipe_mapping[i] == I915_MAX_PIPES)
				continue;

			mapped_interrupt &= ~(irq_hstate->pipe_mask <<
				(vgt->pipe_mapping[i] * 5));

			temp = interrupt >> (i * 5);
			temp &= irq_hstate->pipe_mask;
			mapped_interrupt |= temp << (vgt->pipe_mapping[i] * 5);
		}
	} else if (DEIER == reg) {
		mapped_interrupt &= ~irq_hstate->pipe_mask;
		mapped_interrupt &= ~(irq_hstate->pipe_mask<<5);
		mapped_interrupt &= ~(irq_hstate->pipe_mask<<10);
		for (i = 0; i < I915_MAX_PIPES; i++) {
			temp = interrupt >> (i * 5);
			temp &= irq_hstate->pipe_mask;
			if (vgt->pipe_mapping[i] != I915_MAX_PIPES) {
				mapped_interrupt |= temp << (vgt->pipe_mapping[i] * 5);
			}
		}
	}
	return mapped_interrupt;
}

static u32 gen8_translate_pipe_interrupt(struct vgt_device *vgt, vgt_reg_t reg)
{
	struct pgt_device *pdev = vgt->pdev;
	u32 v, v_val, p_val, p_reg;
	int v_pipe, p_pipe;
	bool is_imr;

	v_val = __vreg(vgt, reg);

	if (reg < 0x44400 || reg > 0x4442c)
		return v_val;

	is_imr = ((reg & 0xf) == 0x4);
	v_pipe = reg >> 4 & 0xf;
	p_pipe = vgt->pipe_mapping[v_pipe];

	if (p_pipe == I915_MAX_PIPES) {
		// clear the initial mask bit VBLANKS, so that when pipe mapping
		// is not valid, physically there are still vblanks generated.
		return is_imr ? 0xfffffffe : 0;
	}

	if (v_pipe == p_pipe)
		return v_val;

	p_reg = (reg & ~(0xf << 4)) | p_pipe << 4;
	p_val = __vreg(vgt, p_reg);

	v = vgt_read_cached_interrupt_register(pdev, p_reg);
	if (is_imr)
		v &= v_val;
	else
		v |= v_val;
	vgt_write_cached_interrupt_register(pdev, p_reg, v);

	return p_val;
}

u32 pipe_mapping_interrupt_virt_to_phys(struct vgt_device *vgt, vgt_reg_t reg)
{
	if (IS_PREBDW(vgt->pdev))
		return gen6_translate_pipe_interrupt(vgt, reg);
	else
		return gen8_translate_pipe_interrupt(vgt, reg);
}


/* =======================IRR/IMR/IER handlers===================== */

/* Now we have physical mask bits generated by ANDing virtual
 * mask bits from all VMs. That means, the event is physically unmasked
 * as long as a VM wants it. This is safe because we still use a single
 * big lock for all critical paths, but not efficient.
 */
u32 vgt_recalculate_mask_bits(struct pgt_device *pdev, unsigned int reg)
{
	int i;
	u32 imr = 0xffffffff;
	u32 mapped_interrupt;

	ASSERT(spin_is_locked(&pdev->lock));
	for (i = 0; i < VGT_MAX_VMS; i++) {
		if (pdev->device[i]) {
			mapped_interrupt = pipe_mapping_interrupt_virt_to_phys(
					pdev->device[i], reg);
			imr &= mapped_interrupt;
		}
	}

	return imr;
}

/*
 * Now we have physical enabling bits generated by ORing virtual
 * enabling bits from all VMs. That means, the event is physically enabled
 * as long as a VM wants it. This is safe because we still use a single
 * big lock for all critical paths, but not efficient.
 */
u32 vgt_recalculate_ier(struct pgt_device *pdev, unsigned int reg)
{
	int i;
	u32 ier = 0;
	u32 mapped_interrupt;

	ASSERT(spin_is_locked(&pdev->lock));
	for (i = 0; i < VGT_MAX_VMS; i++) {
		if (pdev->device[i]) {
			mapped_interrupt = pipe_mapping_interrupt_virt_to_phys(
					pdev->device[i], reg);
			ier |= mapped_interrupt;
		}
	}

	return ier;
}

static enum vgt_irq_type irq_reg_to_info(struct pgt_device *pdev, vgt_reg_t reg)
{
	enum vgt_irq_type irq_type;

	switch (reg) {
	case GTIMR:
	case GTIIR:
	case GTIER:
	case GTISR:
	case IMR:
	case _REG_BCS_IMR:
	case _REG_VCS_IMR:
	case _REG_VECS_IMR:
		irq_type = IRQ_INFO_GT;
		break;
	case DEIMR:
	case DEIIR:
	case DEIER:
	case DEISR:
		irq_type = IRQ_INFO_DPY;
		break;
	case SDEIMR:
	case SDEIIR:
	case SDEIER:
	case SDEISR:
		irq_type = IRQ_INFO_PCH;
		break;
	case GEN6_PMIMR:
	case GEN6_PMIIR:
	case GEN6_PMIER:
	case GEN6_PMISR:
		irq_type = IRQ_INFO_PM;
		break;
	default:
		irq_type = IRQ_INFO_MAX;
		break;
	}
	return irq_type;
}

void recalculate_and_update_imr(struct pgt_device *pdev, vgt_reg_t reg)
{
	struct vgt_irq_host_state *hstate = pdev->irq_hstate;
	enum vgt_irq_type irq_type;
	u32 new_imr;

	new_imr = vgt_recalculate_mask_bits(pdev, reg);
	/*
	 * unmask the default bits and update imr
	 */
	if (irq_based_ctx_switch) {
		irq_type = irq_reg_to_info(pdev, reg);
		ASSERT(irq_type != IRQ_INFO_MAX);
		new_imr &= ~(hstate->info[irq_type]->default_enabled_events);
	}

	vgt_write_cached_interrupt_register(pdev, reg, new_imr);
}

static inline struct vgt_irq_info *regbase_to_irq_info(struct pgt_device *pdev,
		unsigned int reg)
{
	struct vgt_irq_host_state *hstate = pdev->irq_hstate;
	int i;

	for_each_set_bit(i, hstate->irq_info_bitmap, IRQ_INFO_MAX) {
		if (hstate->info[i]->reg_base == reg)
			return hstate->info[i];
	}

	return NULL;
}

/* general write handler for all imr registers */
bool vgt_reg_imr_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes)
{
	uint32_t changed, masked, unmasked;
	uint32_t imr = *(u32 *)p_data;
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);

	vgt_dbg(VGT_DBG_IRQ, "IRQ: capture IMR write on reg (%x) with val (%x)\n",
		reg, imr);

	vgt_dbg(VGT_DBG_IRQ, "IRQ: old vIMR(%x), pIMR(%x)\n",
		 __vreg(vgt, reg), VGT_MMIO_READ(pdev, reg));

	/* figure out newly masked/unmasked bits */
	changed = __vreg(vgt, reg) ^ imr;
	if (reg == DEIMR)
		changed &= ~MASTER_INTERRUPT_ENABLE;
	masked = (__vreg(vgt, reg) & changed) ^ changed;
	unmasked = masked ^ changed;

	vgt_dbg(VGT_DBG_IRQ, "IRQ: changed (%x), masked(%x), unmasked (%x)\n",
		changed, masked, unmasked);

	__vreg(vgt, reg) = imr;

	if (changed || device_is_reseting(pdev))
		recalculate_and_update_imr(pdev, reg);

	ops->check_pending_irq(vgt);
	vgt_dbg(VGT_DBG_IRQ, "IRQ: new vIMR(%x), pIMR(%x)\n",
		 __vreg(vgt, reg), VGT_MMIO_READ(pdev, reg));
	return true;
}

void recalculate_and_update_ier(struct pgt_device *pdev, vgt_reg_t reg)
{
	struct vgt_irq_host_state *hstate = pdev->irq_hstate;
	enum vgt_irq_type irq_type;
	u32 new_ier;

	new_ier = vgt_recalculate_ier(pdev, reg);

	/*
	 * enable the default bits and update ier
	 */
	if (irq_based_ctx_switch) {
		irq_type = irq_reg_to_info(pdev, reg);
		ASSERT(irq_type != IRQ_INFO_MAX);
		new_ier |= hstate->info[irq_type]->default_enabled_events;
	}

	if (device_is_reseting(pdev)) {
		if (IS_BDWPLUS(pdev)) {
			if (reg == GEN8_MASTER_IRQ)
				new_ier &= ~GEN8_MASTER_IRQ_CONTROL;
		} else {
			if (reg == DEIER)
				new_ier &= ~MASTER_INTERRUPT_ENABLE;
		}
	}

	vgt_write_cached_interrupt_register(pdev, reg, new_ier);
}

bool vgt_reg_master_irq_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes)
{
	uint32_t changed, enabled, disabled;
	uint32_t ier = *(u32 *)p_data;
	uint32_t virtual_ier = __vreg(vgt, reg);
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);

	vgt_dbg(VGT_DBG_IRQ, "IRQ: capture master irq write on reg (%x) with val (%x)\n",
		reg, ier);

	vgt_dbg(VGT_DBG_IRQ, "IRQ: old vreg(%x), preg(%x)\n",
		 __vreg(vgt, reg), VGT_MMIO_READ(pdev, reg));

	if (likely(vgt_track_nest) && !vgt->vgt_id &&
		__this_cpu_read(in_vgt) != 1) {
		vgt_err("i915 virq happens in nested vgt context(%d)!!!\n",
			__this_cpu_read(in_vgt));
		ASSERT(0);
	}

	/*
	 * GEN8_MASTER_IRQ is a special irq register,
	 * only bit 31 is allowed to be modified
	 * and treated as an IER bit.
	 */
	ier &= GEN8_MASTER_IRQ_CONTROL;
	virtual_ier &= GEN8_MASTER_IRQ_CONTROL;
	__vreg(vgt, reg) &= ~GEN8_MASTER_IRQ_CONTROL;
	__vreg(vgt, reg) |= ier;

	/* figure out newly enabled/disable bits */
	changed = virtual_ier ^ ier;
	enabled = (virtual_ier & changed) ^ changed;
	disabled = enabled ^ changed;

	vgt_dbg(VGT_DBG_IRQ, "vGT_IRQ: changed (%x), enabled(%x), disabled(%x)\n",
			changed, enabled, disabled);

	if (changed || device_is_reseting(pdev))
		recalculate_and_update_ier(pdev, reg);

	ops->check_pending_irq(vgt);
	vgt_dbg(VGT_DBG_IRQ, "IRQ: new vreg(%x), preg(%x)\n",
		 __vreg(vgt, reg), VGT_MMIO_READ(pdev, reg));
	return true;
}

/* general write handler for all level-1 ier registers */
bool vgt_reg_ier_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes)
{
	uint32_t changed, enabled, disabled;
	uint32_t ier = *(u32 *)p_data;
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);
	struct vgt_irq_info *info;

	vgt_dbg(VGT_DBG_IRQ, "IRQ: capture IER write on reg (%x) with val (%x)\n",
		reg, ier);

	vgt_dbg(VGT_DBG_IRQ, "IRQ: old vIER(%x), pIER(%x)\n",
		 __vreg(vgt, reg), VGT_MMIO_READ(pdev, reg));

	if (likely(vgt_track_nest) && !vgt->vgt_id &&
		__this_cpu_read(in_vgt) != 1) {
		vgt_err("i915 virq happens in nested vgt context(%d)!!!\n",
			__this_cpu_read(in_vgt));
		ASSERT(0);
	}

	/* figure out newly enabled/disable bits */
	changed = __vreg(vgt, reg) ^ ier;
	enabled = (__vreg(vgt, reg) & changed) ^ changed;
	disabled = enabled ^ changed;

	vgt_dbg(VGT_DBG_IRQ, "vGT_IRQ: changed (%x), enabled(%x), disabled(%x)\n",
			changed, enabled, disabled);
	__vreg(vgt, reg) = ier;

	info = regbase_to_irq_info(pdev, ier_to_regbase(reg));
	if (!info)
		return false;

	if (info->has_upstream_irq)
		update_upstream_irq(vgt, info);

	if (changed || device_is_reseting(pdev))
		recalculate_and_update_ier(pdev, reg);

	ops->check_pending_irq(vgt);
	vgt_dbg(VGT_DBG_IRQ, "IRQ: new vIER(%x), pIER(%x)\n",
		 __vreg(vgt, reg), VGT_MMIO_READ(pdev, reg));
	return true;
}

bool vgt_reg_iir_handler(struct vgt_device *vgt, unsigned int reg,
	void *p_data, unsigned int bytes)
{
	struct vgt_irq_info *info = regbase_to_irq_info(vgt->pdev, iir_to_regbase(reg));
	vgt_reg_t iir = *(vgt_reg_t *)p_data;

	vgt_dbg(VGT_DBG_IRQ, "IRQ: capture IIR write on reg (%x) with val (%x)\n",
		reg, iir);

	if (!info)
		return false;

	/* TODO: need use an atomic operation. Now it's safe due to big lock */
	__vreg(vgt, reg) &= ~iir;

	if (info->has_upstream_irq)
		update_upstream_irq(vgt, info);

	return true;
}

bool vgt_reg_isr_read(struct vgt_device *vgt, unsigned int reg,
	void *p_data, unsigned int bytes)
{
	vgt_reg_t isr_value;
	if (is_current_display_owner(vgt) && reg == SDEISR) {
		isr_value = VGT_MMIO_READ(vgt->pdev, SDEISR);
		memcpy(p_data, (char *)&isr_value, bytes);
		return true;
	} else {
		return default_mmio_read(vgt, reg, p_data, bytes);
	}
}

bool vgt_reg_isr_write(struct vgt_device *vgt, unsigned int reg,
	void *p_data, unsigned int bytes)
{
	vgt_dbg(VGT_DBG_IRQ, "IRQ: capture ISR write on reg (%x) with val (%x)." \
		" Will be ignored!\n", reg, *(vgt_reg_t *)p_data);

	return true;
}

#define IIR_WRITE_MAX	5

static bool process_irq(struct vgt_irq_host_state *hstate,
		struct vgt_irq_info *info)
{
	struct pgt_device *pdev = hstate->pdev;
	u32 val;
	u32 reg;
	int count = 0;

	if (info->group == IRQ_INFO_MASTER)
		reg = info->reg_base;
	else
		reg = regbase_to_iir(info->reg_base);

	val = VGT_MMIO_READ(pdev, reg);
	if (!val)
		return false;

	vgt_handle_events(hstate, &val, info);

	if (reg != SDEIIR) {
		if (info->group != IRQ_INFO_MASTER)
			VGT_MMIO_WRITE(pdev, reg, val);
	} else {
		while((count < IIR_WRITE_MAX) && (val != 0)) {
			VGT_MMIO_WRITE(pdev, SDEIIR, val);
			val = VGT_MMIO_READ(pdev, SDEIIR);
			count ++;
		}
	}

	return true;
}

struct vgt_irq_map snb_irq_map[] = {
	{ IRQ_INFO_DPY, 21, IRQ_INFO_PCH, ~0 },
	{ -1, -1, -1, ~0},
};

struct vgt_irq_map base_irq_map[] = {
	{ IRQ_INFO_DPY, 28, IRQ_INFO_PCH, ~0 },
	{ -1, -1, -1, ~0},
};

struct vgt_irq_map gen8_irq_map[] = {
	{ IRQ_INFO_MASTER, 0, IRQ_INFO_GT0, 0xffff },
	{ IRQ_INFO_MASTER, 1, IRQ_INFO_GT0, 0xffff0000 },
	{ IRQ_INFO_MASTER, 2, IRQ_INFO_GT1, 0xffff },
	{ IRQ_INFO_MASTER, 3, IRQ_INFO_GT1, 0xffff0000 },
	{ IRQ_INFO_MASTER, 4, IRQ_INFO_GT2, 0xffff },
	{ IRQ_INFO_MASTER, 6, IRQ_INFO_GT3, 0xffff },
	{ IRQ_INFO_MASTER, 16, IRQ_INFO_DE_PIPE_A, ~0 },
	{ IRQ_INFO_MASTER, 17, IRQ_INFO_DE_PIPE_B, ~0 },
	{ IRQ_INFO_MASTER, 18, IRQ_INFO_DE_PIPE_C, ~0 },
	{ IRQ_INFO_MASTER, 20, IRQ_INFO_DE_PORT, ~0 },
	{ IRQ_INFO_MASTER, 22, IRQ_INFO_DE_MISC, ~0 },
	{ IRQ_INFO_MASTER, 23, IRQ_INFO_PCH, ~0 },
	{ IRQ_INFO_MASTER, 30, IRQ_INFO_PCU, ~0 },
	{ -1, -1, ~0 },
};

static void process_downstream_irq(struct vgt_irq_host_state *hstate,
		struct vgt_irq_info *info, int bit)
{
	struct vgt_irq_map *map;

	for (map = hstate->irq_map; map->up_irq_bit != -1; map++) {
		if (map->up_irq_group != info->group || map->up_irq_bit != bit)
			continue;

		process_irq(hstate, hstate->info[map->down_irq_group]);
	}
}

static void update_upstream_irq(struct vgt_device *vgt,
		struct vgt_irq_info *info)
{
	struct vgt_irq_host_state *hstate = vgt->pdev->irq_hstate;
	struct vgt_irq_map *map = hstate->irq_map;
	struct vgt_irq_info *up_irq_info = NULL;
	u32 set_bits = 0;
	u32 clear_bits = 0;
	int bit;
	u32 val = __vreg(vgt, regbase_to_iir(info->reg_base))
			& __vreg(vgt, regbase_to_ier(info->reg_base));

	if (!info->has_upstream_irq)
		return;

	for (map = hstate->irq_map; map->up_irq_bit != -1; map++) {
		if (info->group != map->down_irq_group)
			continue;

		if (!up_irq_info)
			up_irq_info = hstate->info[map->up_irq_group];
		else
			ASSERT(up_irq_info == hstate->info[map->up_irq_group]);

		bit = map->up_irq_bit;

		if (val & map->down_irq_bitmask)
			set_bits |= (1 << bit);
		else
			clear_bits |= (1 << bit);
	}

	ASSERT(up_irq_info);

	if (up_irq_info->group == IRQ_INFO_MASTER) {
		u32 isr = up_irq_info->reg_base;
		__vreg(vgt, isr) &= ~clear_bits;
		__vreg(vgt, isr) |= set_bits;
	} else {
		u32 iir = regbase_to_iir(up_irq_info->reg_base);
		u32 imr = regbase_to_imr(up_irq_info->reg_base);
		__vreg(vgt, iir) |= (set_bits & ~__vreg(vgt, imr));
	}

	if (up_irq_info->has_upstream_irq)
		update_upstream_irq(vgt, up_irq_info);
}

static void vgt_irq_map_init(struct vgt_irq_host_state *hstate)
{
	struct vgt_irq_map *map;
	struct vgt_irq_info *up_info, *down_info;
	int up_bit;

	for (map = hstate->irq_map; map->up_irq_bit != -1; map++) {
		up_info = hstate->info[map->up_irq_group];
		up_bit = map->up_irq_bit;
		down_info = hstate->info[map->down_irq_group];

		set_bit(up_bit, up_info->downstream_irq_bitmap);
		down_info->has_upstream_irq = true;

		printk("vGT: irq map [upstream] group: %d, bit: %d -> [downstream] group: %d, bitmask: 0x%x\n",
			up_info->group, up_bit, down_info->group, map->down_irq_bitmask);
	}
}

/* =======================vEvent injection===================== */

DEFINE_PER_CPU(unsigned long, delay_event_bitmap);
static unsigned long next_avail_delay_event = 1;

static void *delay_event_timers[BITS_PER_LONG];

static bool vgt_check_delay_event(void *timer)
{
	int bit;
	unsigned long *bitmap;

	if (!vgt_delay_nest || !hypervisor_check_host()
			|| !vgt_enabled)
		return true;

	if (!get_cpu_var(in_vgt)) {
		put_cpu_var(in_vgt);
		return true;
	}
	put_cpu_var(in_vgt);

	if (timer == NULL) {
		bit = 0;
	} else {
		for (bit = 1; bit < next_avail_delay_event; bit++)
			if (delay_event_timers[bit] == timer)
				break;

		if (bit == next_avail_delay_event) {
			vgt_warn("Unknown delay timer event: %p\n", timer);
			return true;
		}
	}

	bitmap = get_cpu_ptr(&delay_event_bitmap);
	set_bit(bit, bitmap);
	put_cpu_ptr(&delay_event_bitmap);
	return false;
}

bool vgt_can_process_irq(void)
{
	return vgt_check_delay_event(NULL);
}

bool vgt_can_process_timer(void *timer)
{
	return vgt_check_delay_event(timer);
}

void vgt_new_delay_event_timer(void *timer)
{
	if (next_avail_delay_event == ARRAY_SIZE(delay_event_timers)) {
		vgt_warn("cannot allocate new delay event timer.\n");
		return;
	}

	delay_event_timers[next_avail_delay_event] = timer;
	next_avail_delay_event++;

	return;
}

static void vgt_flush_delay_events(void)
{
	int bit;

	for_each_set_bit(bit, this_cpu_ptr(&delay_event_bitmap), BITS_PER_LONG) {
		if (bit == next_avail_delay_event)
			break;

		clear_bit(bit, this_cpu_ptr(&delay_event_bitmap));

		if (bit == 0) {
			struct pgt_device *pdev = &default_device;
			int i915_irq = pdev->irq_hstate->i915_irq;
			vgt_host_irq(i915_irq);
		} else {
			struct timer_list *t = delay_event_timers[bit];
			if (t)
				mod_timer(t, jiffies);
		}
	}

	return;
}

/*
 * dom0 virtual interrupt can only be pended here. Immediate
 * injection at this point may cause race condition on nested
 * lock, regardless of whether the target vcpu is the current
 * or not.
 */
static void pend_dom0_virtual_interrupt(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	int i915_irq = pdev->irq_hstate->i915_irq;

	ASSERT(spin_is_locked(&pdev->lock));

	/*
	 * Some wired devices leave dirty IIR bits before system
	 * booting. It will trigger unexpected interrupt injection
	 * before VGT irq framework works.
	 */
	if (!pdev->irq_hstate->installed)
		return;

	if (unlikely(!vgt_track_nest)) {
		vgt_host_irq(i915_irq);
		return;
	}

	if (pdev->dom0_irq_pending)
		return;

	/*
	 * set current cpu to do delayed check, wchih may
	 * trigger ipi call function but at this piont irq
	 * may be disabled already.
	 */
	pdev->dom0_irq_cpu = smp_processor_id();
	wmb();
	pdev->dom0_irq_pending = true;

	/* TODO: may do a kick here */
}

/*
 * actual virq injection happens here. called in vgt_exit()
 * or IPI handler
 */
static void do_inject_dom0_virtual_interrupt(void *info, int ipi)
{
	unsigned long flags;
	struct pgt_device *pdev = &default_device;
	int i915_irq;
	int this_cpu;

	if (ipi)
		clear_bit(0, &pdev->dom0_ipi_irq_injecting);

	/* still in vgt. the injection will happen later */
	if (__this_cpu_read(in_vgt))
		return;

	spin_lock_irqsave(&pdev->lock, flags);
	if (!pdev->dom0_irq_pending) {
		spin_unlock_irqrestore(&pdev->lock, flags);
		return;
	}

	ASSERT(pdev->dom0_irq_cpu != -1);
	this_cpu = smp_processor_id();
	if (this_cpu != pdev->dom0_irq_cpu) {
		spin_unlock_irqrestore(&pdev->lock, flags);
		return;
	}

	i915_irq = pdev->irq_hstate->i915_irq;

	//TODO: remove the logic used for injecting irq to dom0!
	pdev->dom0_irq_pending = false;
	wmb();
	pdev->dom0_irq_cpu = -1;

	spin_unlock_irqrestore(&pdev->lock, flags);
	vgt_host_irq(i915_irq);
}

void inject_dom0_virtual_interrupt(void *info)
{
	if (vgt_delay_nest)
		vgt_flush_delay_events();

	do_inject_dom0_virtual_interrupt(info, 0);

	return;
}

static int vgt_inject_virtual_interrupt(struct vgt_device *vgt)
{
	if (vgt->vm_id) {
		if (hypervisor_inject_msi(vgt) < 0) {
			if (!vgt->stat.irq_inject_fail)
				vgt_err("vGT(%d): failed to inject vmsi\n",
					vgt->vgt_id);
			vgt->stat.irq_inject_fail++;
		}
	}
	else
		pend_dom0_virtual_interrupt(vgt);

	vgt->stat.irq_num++;
	vgt->stat.last_injection = get_cycles();
	return 0;
}

static void vgt_propagate_event(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event, struct vgt_device *vgt)
{
	int bit;
	struct vgt_irq_info *info;
	unsigned int reg_base;

	info = vgt_get_irq_info(hstate, event);
	if (!info) {
		vgt_err("IRQ(%d): virt-inject: no irq reg info!!!\n",
			vgt->vm_id);
		return;
	}

	reg_base = info->reg_base;
	bit = hstate->events[event].bit;

	/*
         * this function call is equivalent to a rising edge ISR
         * TODO: need check 2nd level IMR for render events
         */
	if (!test_bit(bit, (void*)vgt_vreg(vgt, regbase_to_imr(reg_base)))) {
		vgt_dbg(VGT_DBG_IRQ, "IRQ: set bit (%d) for (%s) for VM (%d)\n",
			bit, vgt_irq_name[event], vgt->vm_id);
		set_bit(bit, (void*)vgt_vreg(vgt, regbase_to_iir(reg_base)));
	}
}

/* =======================vEvent Handlers===================== */

static void vgt_handle_default_event_virt(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event, struct vgt_device *vgt)
{
	if (!vgt_irq_warn_once[vgt->vgt_id][event]) {
		vgt_info("IRQ: VM(%d) receive event %d (%s)\n",
			vgt->vm_id, event, vgt_irq_name[event]);
		vgt_irq_warn_once[vgt->vgt_id][event] = 1;
	}
	vgt_propagate_event(hstate, event, vgt);
	vgt->stat.events[event]++;
}

static void vgt_handle_ring_empty_notify_virt(struct vgt_irq_host_state *hstate,
       enum vgt_event_type event, struct vgt_device *vgt)
{
	vgt_check_pending_context_switch(vgt);
	vgt_handle_default_event_virt(hstate, event, vgt);
}

static void vgt_handle_phase_in_virt(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event, struct vgt_device *vgt)
{
	__vreg(vgt, BLC_PWM_CPU_CTL2) |= BLM_PHASE_IN_INTERUPT_STATUS;
	vgt_handle_default_event_virt(hstate, event, vgt);
}

static void vgt_handle_histogram_virt(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event, struct vgt_device *vgt)
{
	__vreg(vgt, _REG_HISTOGRAM_THRSH) |= _REGBIT_HISTOGRAM_IRQ_STATUS;
	vgt_handle_default_event_virt(hstate, event, vgt);
}

static void vgt_handle_crt_hotplug_virt(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event, struct vgt_device *vgt)
{
	/* update channel status */
	if (__vreg(vgt, PCH_ADPA) & ADPA_CRT_HOTPLUG_ENABLE) {

		if (!is_current_display_owner(vgt)) {
			__vreg(vgt, PCH_ADPA) &=
				~ADPA_CRT_HOTPLUG_MONITOR_MASK;
			if (dpy_has_monitor_on_port(vgt, PORT_E))
				__vreg(vgt, PCH_ADPA) |=
					ADPA_CRT_HOTPLUG_MONITOR_MASK;
		}

		vgt_handle_default_event_virt(hstate, event, vgt);
	}
}

static void vgt_handle_port_hotplug_virt(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event, struct vgt_device *vgt)
{
	vgt_reg_t enable_mask, status_mask;

	if (event == DP_B_HOTPLUG) {
		enable_mask = _REGBIT_DP_B_ENABLE;
		status_mask = _REGBIT_DP_B_STATUS;
	} else if (event == DP_C_HOTPLUG) {
		enable_mask = _REGBIT_DP_C_ENABLE;
		status_mask = _REGBIT_DP_C_STATUS;
	} else {
		ASSERT(event == DP_D_HOTPLUG);
		enable_mask = _REGBIT_DP_D_ENABLE;
		status_mask = _REGBIT_DP_D_STATUS;
	}

	if (__vreg(vgt, PCH_PORT_HOTPLUG) & enable_mask) {

		__vreg(vgt, PCH_PORT_HOTPLUG) &= ~status_mask;
		if (is_current_display_owner(vgt)) {
			__vreg(vgt, PCH_PORT_HOTPLUG) |=
				vgt_get_event_val(hstate, event) & status_mask;
		} else {
			__vreg(vgt, PCH_PORT_HOTPLUG) |= status_mask;
		}

		vgt_handle_default_event_virt(hstate, event, vgt);
	}
}

static inline enum vgt_ring_id event_to_ring_id(enum vgt_event_type event)
{
	enum vgt_ring_id ring_id;

	switch(event) {
	case RCS_AS_CONTEXT_SWITCH:
		ring_id = RING_BUFFER_RCS;
		break;
	case VCS_AS_CONTEXT_SWITCH:
		ring_id = RING_BUFFER_VCS;
		break;
	case VCS2_AS_CONTEXT_SWITCH:
		ring_id = RING_BUFFER_VCS2;
		break;
	case BCS_AS_CONTEXT_SWITCH:
		ring_id = RING_BUFFER_BCS;
		break;
	case VECS_AS_CONTEXT_SWITCH:
		ring_id = RING_BUFFER_VECS;
		break;
	default:
		ring_id = MAX_ENGINES;
		BUG();
	}
	return ring_id;
}

static void vgt_handle_ctx_switch_virt(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event, struct vgt_device *vgt)
{
	enum vgt_ring_id ring_id;
	uint32_t ctx_ptr_reg;
	struct ctx_st_ptr_format ctx_ptr_val;
	int v_write_ptr;
	int s_write_ptr;
	bool csb_has_new_updates = false;

	ring_id = event_to_ring_id(event);
	ctx_ptr_reg = el_ring_mmio(ring_id, _EL_OFFSET_STATUS_PTR);
	ctx_ptr_val.dw = __vreg(vgt, ctx_ptr_reg);
	v_write_ptr = ctx_ptr_val.status_buf_write_ptr;
	s_write_ptr = vgt->rb[ring_id].csb_write_ptr;

	if (v_write_ptr != s_write_ptr)
		csb_has_new_updates = true;

	if (hvm_render_owner || csb_has_new_updates) {

		if (current_render_owner(vgt->pdev) != vgt) {
			/* In any case, we should not go here! */
			vgt_err("ERROR VM inject irq without ownership"
			" VM%d owner=%d, csb=%04x, s=%x\n",
			vgt->vm_id, current_render_owner(vgt->pdev)->vm_id,
			ctx_ptr_val.dw, s_write_ptr);
		}

		ctx_ptr_val.status_buf_write_ptr = s_write_ptr;
		__vreg(vgt, ctx_ptr_reg) = ctx_ptr_val.dw;
		vgt_handle_default_event_virt(hstate, event, vgt);
	}
}

static enum vgt_event_type translate_physical_event(struct vgt_device *vgt,
	enum vgt_event_type event)
{
	enum pipe virtual_pipe = I915_MAX_PIPES;
	enum pipe physical_pipe = I915_MAX_PIPES;
	enum vgt_event_type virtual_event = event;
	int i;

	switch (event) {
	case PIPE_A_VSYNC:
	case PIPE_A_LINE_COMPARE:
	case PIPE_A_VBLANK:
	case PRIMARY_A_FLIP_DONE:
	case SPRITE_A_FLIP_DONE:
		physical_pipe = PIPE_A;
		break;

	case PIPE_B_VSYNC:
	case PIPE_B_LINE_COMPARE:
	case PIPE_B_VBLANK:
	case PRIMARY_B_FLIP_DONE:
	case SPRITE_B_FLIP_DONE:
		physical_pipe = PIPE_B;
		break;

	case PIPE_C_VSYNC:
	case PIPE_C_LINE_COMPARE:
	case PIPE_C_VBLANK:
	case PRIMARY_C_FLIP_DONE:
	case SPRITE_C_FLIP_DONE:
		physical_pipe = PIPE_C;
		break;
	default:
		physical_pipe = I915_MAX_PIPES;
	}

	for (i = 0; i < I915_MAX_PIPES; i++) {
		if (vgt->pipe_mapping[i] == physical_pipe) {
			virtual_pipe = i;
			break;
		}
	}

	if (virtual_pipe != I915_MAX_PIPES && physical_pipe  != I915_MAX_PIPES) {
		virtual_event = event + ((int)virtual_pipe - (int)physical_pipe);
	}

	return virtual_event;
}


/* =======================pEvent Handlers===================== */

static void vgt_handle_default_event_phys(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event)
{
	if (!vgt_irq_warn_once[VGT_MAX_VMS][event]) {
		vgt_info("IRQ: receive event (%s)\n",
				vgt_irq_name[event]);
		vgt_irq_warn_once[VGT_MAX_VMS][event] = 1;
	}
}

static void vgt_handle_phase_in_phys(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event)
{
	uint32_t val;
	struct pgt_device *pdev = hstate->pdev;

	val = VGT_MMIO_READ(pdev, BLC_PWM_CPU_CTL2);
	val &= ~BLM_PHASE_IN_INTERUPT_STATUS;
	VGT_MMIO_WRITE(pdev, BLC_PWM_CPU_CTL2, val);

	vgt_handle_default_event_phys(hstate, event);
}

static void vgt_handle_histogram_phys(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event)
{
	uint32_t val;
	struct pgt_device *pdev = hstate->pdev;

	val = VGT_MMIO_READ(pdev, _REG_HISTOGRAM_THRSH);
	val &= ~_REGBIT_HISTOGRAM_IRQ_STATUS;
	VGT_MMIO_WRITE(pdev, _REG_HISTOGRAM_THRSH, val);

	vgt_handle_default_event_phys(hstate, event);
}

/*
 * It's said that CRT hotplug detection through below method does not
 * always work. For example in Linux i915 not hotplug handler is installed
 * for CRT (likely through some other polling method). But let's use this
 * as the example for how hotplug event is generally handled here.
 */
static void vgt_handle_crt_hotplug_phys(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event)
{
	vgt_reg_t adpa_ctrl;
	struct pgt_device *pdev = hstate->pdev;

	adpa_ctrl = VGT_MMIO_READ(pdev, PCH_ADPA);
	if (!(adpa_ctrl & _REGBIT_ADPA_DAC_ENABLE)) {
		vgt_warn("IRQ: captured CRT hotplug event when CRT is disabled\n");
	}

	/* check blue/green channel status for attachment status */
	if (adpa_ctrl & ADPA_CRT_HOTPLUG_MONITOR_MASK) {
		vgt_info("IRQ: detect crt insert event!\n");
		vgt_set_uevent(vgt_dom0, CRT_HOTPLUG_IN);
	} else {
		vgt_info("IRQ: detect crt removal event!\n");
		vgt_set_uevent(vgt_dom0, CRT_HOTPLUG_OUT);
	}

	if (propagate_monitor_to_guest)
		vgt_set_uevent(vgt_dom0, VGT_DETECT_PORT_E);

	/* send out udev events when handling physical interruts */
	vgt_raise_request(pdev, VGT_REQUEST_UEVENT);

	vgt_handle_default_event_phys(hstate, event);
}

static void vgt_handle_port_hotplug_phys(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event)
{
	vgt_reg_t hotplug_ctrl;
	vgt_reg_t enable_mask, status_mask, tmp;
	enum vgt_uevent_type hotplug_event;
	enum vgt_uevent_type detect_event;
	struct pgt_device *pdev = hstate->pdev;

	if (event == DP_B_HOTPLUG) {
		enable_mask = _REGBIT_DP_B_ENABLE;
		status_mask = _REGBIT_DP_B_STATUS;
		hotplug_event = PORT_B_HOTPLUG_IN;
		detect_event = VGT_DETECT_PORT_B;
	} else if (event == DP_C_HOTPLUG) {
		enable_mask = _REGBIT_DP_C_ENABLE;
		status_mask = _REGBIT_DP_C_STATUS;
		hotplug_event = PORT_C_HOTPLUG_IN;
		detect_event = VGT_DETECT_PORT_C;
	} else {
		ASSERT(event == DP_D_HOTPLUG);
		enable_mask = _REGBIT_DP_D_ENABLE;
		status_mask = _REGBIT_DP_D_STATUS;
		hotplug_event = PORT_D_HOTPLUG_IN;
		detect_event = VGT_DETECT_PORT_D;
	}

	hotplug_ctrl = VGT_MMIO_READ(pdev, PCH_PORT_HOTPLUG);

	if (!(hotplug_ctrl & enable_mask)) {
		vgt_warn("IRQ: captured port hotplug event when HPD is disabled\n");
	}

	tmp = hotplug_ctrl & ~(_REGBIT_DP_B_STATUS |
				_REGBIT_DP_C_STATUS |
				_REGBIT_DP_D_STATUS);
	tmp |= hotplug_ctrl & status_mask;
	/* write back value to clear specific port status */
	VGT_MMIO_WRITE(pdev, PCH_PORT_HOTPLUG, tmp);

	if (hotplug_ctrl & status_mask) {
		vgt_info("IRQ: detect monitor insert event on port!\n");
		vgt_set_uevent(vgt_dom0, hotplug_event);
	} else {
		vgt_info("IRQ: detect monitor removal eventon port!\n");
		vgt_set_uevent(vgt_dom0, hotplug_event + 1);
	}

	if (propagate_monitor_to_guest)
		vgt_set_uevent(vgt_dom0, detect_event);

	vgt_set_event_val(hstate, event, hotplug_ctrl);
	/* send out udev events when handling physical interruts */
	vgt_raise_request(pdev, VGT_REQUEST_UEVENT);

	vgt_handle_default_event_phys(hstate, event);
}

static void vgt_handle_ctx_switch_phys(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event)
{
	struct pgt_device *pdev = hstate->pdev;
	enum vgt_ring_id ring_id = event_to_ring_id(event);

	vgt_raise_request(pdev, VGT_REQUEST_CTX_EMULATION_RCS + ring_id);

	vgt_handle_default_event_phys(hstate, event);
}

/* =====================GEN specific logic======================= */

/*
 * Here we only check IIR/IER. IMR/ISR is not checked
 * because only rising-edge of ISR is captured as an event,
 * so that current value of vISR doesn't matter.
 */
static void vgt_base_check_pending_irq(struct vgt_device *vgt)
{
	struct vgt_irq_host_state *hstate = vgt->pdev->irq_hstate;
	struct vgt_irq_info *info = hstate->info[IRQ_INFO_PCH];

	if (!(__vreg(vgt, DEIER) & MASTER_INTERRUPT_ENABLE))
		return;

	if ((__vreg(vgt, regbase_to_iir(info->reg_base))
				& __vreg(vgt, regbase_to_ier(info->reg_base))))
		update_upstream_irq(vgt, info);

	/* then check 1st level pending events */
	if ((__vreg(vgt, DEIIR) & __vreg(vgt, DEIER)) ||
	    (__vreg(vgt, GTIIR) & __vreg(vgt, GTIER)) ||
	    (__vreg(vgt, GEN6_PMIIR) & __vreg(vgt, GEN6_PMIER))) {
		vgt_inject_virtual_interrupt(vgt);
	}
}

/* base interrupt handler, for snb/ivb/hsw */
static irqreturn_t vgt_base_irq_handler(struct vgt_irq_host_state *hstate)
{
	struct pgt_device *pdev = hstate->pdev;
	bool rc = false;

	vgt_dbg(VGT_DBG_IRQ, "IRQ: receive interrupt (de-%x, gt-%x, pch-%x, pm-%x)\n",
			VGT_MMIO_READ(pdev, DEIIR),
			VGT_MMIO_READ(pdev, GTIIR),
			VGT_MMIO_READ(pdev, SDEIIR),
			VGT_MMIO_READ(pdev, GEN6_PMIIR));

	rc |= process_irq(hstate, hstate->info[IRQ_INFO_GT]);
	rc |= process_irq(hstate, hstate->info[IRQ_INFO_DPY]);
	rc |= process_irq(hstate, hstate->info[IRQ_INFO_PM]);

	return rc ? IRQ_HANDLED : IRQ_NONE;
}

/* SNB/IVB/HSW share the similar interrupt register scheme */
static struct vgt_irq_info vgt_base_gt_info = {
	.name = "GT-IRQ",
	.reg_base = GTISR,
	.bit_to_event = {[0 ... VGT_IRQ_BITWIDTH-1] = EVENT_RESERVED},
};

static struct vgt_irq_info vgt_base_dpy_info = {
	.name = "DPY-IRQ",
	.reg_base = DEISR,
	.bit_to_event = {[0 ... VGT_IRQ_BITWIDTH-1] = EVENT_RESERVED},
};

static struct vgt_irq_info vgt_base_pch_info = {
	.name = "PCH-IRQ",
	.reg_base = SDEISR,
	.bit_to_event = {[0 ... VGT_IRQ_BITWIDTH-1] = EVENT_RESERVED},
};

static struct vgt_irq_info vgt_base_pm_info = {
	.name = "PM-IRQ",
	.reg_base = GEN6_PMISR,
	.bit_to_event = {[0 ... VGT_IRQ_BITWIDTH-1] = EVENT_RESERVED},
};

/* associate gen specific register bits to general events */
/* TODO: add all hardware bit definitions */
static void vgt_base_init_irq(
	struct vgt_irq_host_state *hstate)
{
	struct pgt_device *pdev = hstate->pdev;

#define SET_BIT_INFO(s, b, e, i)		\
	do {					\
		s->events[e].bit = b;		\
		s->events[e].info = s->info[i];	\
		s->info[i]->bit_to_event[b] = e;\
	} while (0);

#define SET_DEFAULT_ENABLED_EVENTS(s, e, i)			      \
	set_bit(s->events[e].bit, &(s->info[i]->default_enabled_events));\

#define SET_IRQ_GROUP(s, g, i) \
	do { \
		s->info[g] = i; \
		(i)->group = g; \
		set_bit(g, s->irq_info_bitmap); \
	} while (0);

	hstate->pipe_mask = REGBIT_INTERRUPT_PIPE_MASK;

	SET_IRQ_GROUP(hstate, IRQ_INFO_GT, &vgt_base_gt_info);
	SET_IRQ_GROUP(hstate, IRQ_INFO_DPY, &vgt_base_dpy_info);
	SET_IRQ_GROUP(hstate, IRQ_INFO_PM, &vgt_base_pm_info);
	SET_IRQ_GROUP(hstate, IRQ_INFO_PCH, &vgt_base_pch_info);

	/* Render events */
	SET_BIT_INFO(hstate, 0, RCS_MI_USER_INTERRUPT, IRQ_INFO_GT);
	SET_BIT_INFO(hstate, 4, RCS_PIPE_CONTROL, IRQ_INFO_GT);
	SET_BIT_INFO(hstate, 12, VCS_MI_USER_INTERRUPT, IRQ_INFO_GT);
	SET_BIT_INFO(hstate, 16, VCS_MI_FLUSH_DW, IRQ_INFO_GT);
	SET_BIT_INFO(hstate, 22, BCS_MI_USER_INTERRUPT, IRQ_INFO_GT);
	SET_BIT_INFO(hstate, 26, BCS_MI_FLUSH_DW, IRQ_INFO_GT);
	/* No space in GT, so put it in PM */
	SET_BIT_INFO(hstate, 13, VECS_MI_FLUSH_DW, IRQ_INFO_PM);

	/* Display events */
	if (IS_IVB(pdev) || IS_HSW(pdev)) {
		SET_BIT_INFO(hstate, 0, PIPE_A_VBLANK, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 3, PRIMARY_A_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 4, SPRITE_A_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 5, PIPE_B_VBLANK, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 8, PRIMARY_B_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 9, SPRITE_B_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 10, PIPE_C_VBLANK, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 13, PRIMARY_C_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 14, SPRITE_C_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 24, DPST_PHASE_IN, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 25, DPST_HISTOGRAM, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 26, AUX_CHANNEL_A, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 27, DP_A_HOTPLUG, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 29, GSE, IRQ_INFO_DPY);
	} else if (IS_SNB(pdev)) {
		SET_BIT_INFO(hstate, 7, PIPE_A_VBLANK, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 15, PIPE_B_VBLANK, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 16, DPST_PHASE_IN, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 17, DPST_HISTOGRAM, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 18, GSE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 19, DP_A_HOTPLUG, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 20, AUX_CHANNEL_A, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 26, PRIMARY_A_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 27, PRIMARY_B_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 28, SPRITE_A_FLIP_DONE, IRQ_INFO_DPY);
		SET_BIT_INFO(hstate, 29, SPRITE_B_FLIP_DONE, IRQ_INFO_DPY);
	}

	/* PM events */
	SET_BIT_INFO(hstate, 1, GV_DOWN_INTERVAL, IRQ_INFO_PM);
	SET_BIT_INFO(hstate, 2, GV_UP_INTERVAL, IRQ_INFO_PM);
	SET_BIT_INFO(hstate, 4, RP_DOWN_THRESHOLD, IRQ_INFO_PM);
	SET_BIT_INFO(hstate, 5, RP_UP_THRESHOLD, IRQ_INFO_PM);
	SET_BIT_INFO(hstate, 6, FREQ_DOWNWARD_TIMEOUT_RC6, IRQ_INFO_PM);
	SET_BIT_INFO(hstate, 24, PCU_THERMAL, IRQ_INFO_PM);
	SET_BIT_INFO(hstate, 25, PCU_PCODE2DRIVER_MAILBOX, IRQ_INFO_PM);

	/* PCH events */
	SET_BIT_INFO(hstate, 17, GMBUS, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 19, CRT_HOTPLUG, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 21, DP_B_HOTPLUG, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 22, DP_C_HOTPLUG, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 23, DP_D_HOTPLUG, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 25, AUX_CHANNEL_B, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 26, AUX_CHANNEL_C, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 27, AUX_CHANNEL_D, IRQ_INFO_PCH);

	SET_DEFAULT_ENABLED_EVENTS(hstate, RCS_MI_USER_INTERRUPT, IRQ_INFO_GT);
	SET_DEFAULT_ENABLED_EVENTS(hstate, RCS_PIPE_CONTROL, IRQ_INFO_GT);
	SET_DEFAULT_ENABLED_EVENTS(hstate, VCS_MI_USER_INTERRUPT, IRQ_INFO_GT);
	SET_DEFAULT_ENABLED_EVENTS(hstate, VCS_MI_FLUSH_DW, IRQ_INFO_GT);
	SET_DEFAULT_ENABLED_EVENTS(hstate, BCS_MI_USER_INTERRUPT, IRQ_INFO_GT);

	if (IS_HSW(pdev))
		SET_DEFAULT_ENABLED_EVENTS(hstate, VECS_MI_FLUSH_DW, IRQ_INFO_PM);

}

static void vgt_base_disable_irq(struct vgt_irq_host_state *hstate)
{
	struct pgt_device *pdev = hstate->pdev;

	VGT_MMIO_WRITE(pdev, DEIER,
			VGT_MMIO_READ(pdev, DEIER) & ~MASTER_INTERRUPT_ENABLE);
}

static void vgt_base_enable_irq(struct vgt_irq_host_state *hstate)
{
	struct pgt_device *pdev = hstate->pdev;

	VGT_MMIO_WRITE(pdev, DEIER,
			VGT_MMIO_READ(pdev, DEIER) | MASTER_INTERRUPT_ENABLE);
}

struct vgt_irq_ops vgt_base_irq_ops = {
	.irq_handler = vgt_base_irq_handler,
	.init_irq = vgt_base_init_irq,
	.check_pending_irq = vgt_base_check_pending_irq,
	.disable_irq = vgt_base_disable_irq,
	.enable_irq = vgt_base_enable_irq,
};

/* GEN8 interrupt routines. */

#define DEFINE_VGT_GEN8_IRQ_INFO(regname, regbase) \
       static struct vgt_irq_info vgt_gen8_##regname##_info = { \
               .name = #regname"-IRQ", \
               .reg_base = regbase, \
               .bit_to_event = {[0 ... VGT_IRQ_BITWIDTH-1] = EVENT_RESERVED}, \
       };

DEFINE_VGT_GEN8_IRQ_INFO(gt0, GEN8_GT_ISR(0));
DEFINE_VGT_GEN8_IRQ_INFO(gt1, GEN8_GT_ISR(1));
DEFINE_VGT_GEN8_IRQ_INFO(gt2, GEN8_GT_ISR(2));
DEFINE_VGT_GEN8_IRQ_INFO(gt3, GEN8_GT_ISR(3));
DEFINE_VGT_GEN8_IRQ_INFO(de_pipe_a, GEN8_DE_PIPE_ISR(PIPE_A));
DEFINE_VGT_GEN8_IRQ_INFO(de_pipe_b, GEN8_DE_PIPE_ISR(PIPE_B));
DEFINE_VGT_GEN8_IRQ_INFO(de_pipe_c, GEN8_DE_PIPE_ISR(PIPE_C));
DEFINE_VGT_GEN8_IRQ_INFO(de_port, GEN8_DE_PORT_ISR);
DEFINE_VGT_GEN8_IRQ_INFO(de_misc, GEN8_DE_MISC_ISR);
DEFINE_VGT_GEN8_IRQ_INFO(pcu, GEN8_PCU_ISR);
DEFINE_VGT_GEN8_IRQ_INFO(master, GEN8_MASTER_IRQ);

static void vgt_gen8_check_pending_irq(struct vgt_device *vgt)
{
	struct vgt_irq_host_state *hstate = vgt->pdev->irq_hstate;
	int i;

	if (!(__vreg(vgt, GEN8_MASTER_IRQ) &
				GEN8_MASTER_IRQ_CONTROL))
		return;

	for_each_set_bit(i, hstate->irq_info_bitmap, IRQ_INFO_MAX) {
		struct vgt_irq_info *info = hstate->info[i];

		if (!info->has_upstream_irq)
			continue;

		if ((__vreg(vgt, regbase_to_iir(info->reg_base))
					& __vreg(vgt, regbase_to_ier(info->reg_base))))
			update_upstream_irq(vgt, info);
	}

	if (__vreg(vgt, GEN8_MASTER_IRQ) & ~GEN8_MASTER_IRQ_CONTROL)
		vgt_inject_virtual_interrupt(vgt);
}

/* GEN8 interrupt handler */
static irqreturn_t vgt_gen8_irq_handler(struct vgt_irq_host_state *hstate)
{
	struct pgt_device *pdev = hstate->pdev;
	u32 master_ctl;
	bool rc;

	master_ctl = VGT_MMIO_READ(pdev, GEN8_MASTER_IRQ);
	master_ctl &= ~GEN8_MASTER_IRQ_CONTROL;

	if (!master_ctl)
		return IRQ_NONE;

	vgt_dbg(VGT_DBG_IRQ, "IRQ: receive interrupt master_ctl %x\n", master_ctl);

	rc = process_irq(hstate, hstate->info[IRQ_INFO_MASTER]);

	return rc ? IRQ_HANDLED : IRQ_NONE;
}

static void vgt_gen8_init_irq(
		struct vgt_irq_host_state *hstate)
{
	struct pgt_device *pdev = hstate->pdev;

	hstate->pipe_mask = REGBIT_INTERRUPT_PIPE_MASK;

	hstate->info[IRQ_INFO_MASTER] = &vgt_gen8_master_info;
	hstate->info[IRQ_INFO_MASTER]->group = IRQ_INFO_MASTER;

	SET_IRQ_GROUP(hstate, IRQ_INFO_GT0, &vgt_gen8_gt0_info);
	SET_IRQ_GROUP(hstate, IRQ_INFO_GT1, &vgt_gen8_gt1_info);
	SET_IRQ_GROUP(hstate, IRQ_INFO_GT2, &vgt_gen8_gt2_info);
	SET_IRQ_GROUP(hstate, IRQ_INFO_GT3, &vgt_gen8_gt3_info);
	SET_IRQ_GROUP(hstate, IRQ_INFO_DE_PIPE_A, &vgt_gen8_de_pipe_a_info);
	SET_IRQ_GROUP(hstate, IRQ_INFO_DE_PIPE_B, &vgt_gen8_de_pipe_b_info);
	SET_IRQ_GROUP(hstate, IRQ_INFO_DE_PIPE_C, &vgt_gen8_de_pipe_c_info);
	SET_IRQ_GROUP(hstate, IRQ_INFO_DE_PORT, &vgt_gen8_de_port_info);
	SET_IRQ_GROUP(hstate, IRQ_INFO_DE_MISC, &vgt_gen8_de_misc_info);
	SET_IRQ_GROUP(hstate, IRQ_INFO_PCU, &vgt_gen8_pcu_info);
	SET_IRQ_GROUP(hstate, IRQ_INFO_PCH, &vgt_base_pch_info);

	/* GEN8 level 2 interrupts. */

	/* GEN8 interrupt GT0 events */
	SET_BIT_INFO(hstate, 0, RCS_MI_USER_INTERRUPT, IRQ_INFO_GT0);
	SET_BIT_INFO(hstate, 4, RCS_PIPE_CONTROL, IRQ_INFO_GT0);
	SET_BIT_INFO(hstate, 8, RCS_AS_CONTEXT_SWITCH, IRQ_INFO_GT0);

	SET_BIT_INFO(hstate, 16, BCS_MI_USER_INTERRUPT, IRQ_INFO_GT0);
	SET_BIT_INFO(hstate, 20, BCS_MI_FLUSH_DW, IRQ_INFO_GT0);
	SET_BIT_INFO(hstate, 24, BCS_AS_CONTEXT_SWITCH, IRQ_INFO_GT0);

	/* GEN8 interrupt GT1 events */
	SET_BIT_INFO(hstate, 0, VCS_MI_USER_INTERRUPT, IRQ_INFO_GT1);
	SET_BIT_INFO(hstate, 4, VCS_MI_FLUSH_DW, IRQ_INFO_GT1);
	SET_BIT_INFO(hstate, 8, VCS_AS_CONTEXT_SWITCH, IRQ_INFO_GT1);

	if (IS_BDWGT3(pdev) || IS_SKLGT3(pdev) || IS_SKLGT4(pdev)) {
		SET_BIT_INFO(hstate, 16, VCS2_MI_USER_INTERRUPT, IRQ_INFO_GT1);
		SET_BIT_INFO(hstate, 20, VCS2_MI_FLUSH_DW, IRQ_INFO_GT1);
		SET_BIT_INFO(hstate, 24, VCS2_AS_CONTEXT_SWITCH, IRQ_INFO_GT1);
	}

	/* GEN8 interrupt GT2 events */
	SET_BIT_INFO(hstate, 1, GV_DOWN_INTERVAL, IRQ_INFO_GT2);
	SET_BIT_INFO(hstate, 2, GV_UP_INTERVAL, IRQ_INFO_GT2);
	SET_BIT_INFO(hstate, 4, RP_DOWN_THRESHOLD, IRQ_INFO_GT2);
	SET_BIT_INFO(hstate, 5, RP_UP_THRESHOLD, IRQ_INFO_GT2);
	SET_BIT_INFO(hstate, 6, FREQ_DOWNWARD_TIMEOUT_RC6, IRQ_INFO_GT2);

	/* GEN8 interrupt GT3 events */
	SET_BIT_INFO(hstate, 0, VECS_MI_USER_INTERRUPT, IRQ_INFO_GT3);
	SET_BIT_INFO(hstate, 4, VECS_MI_FLUSH_DW, IRQ_INFO_GT3);
	SET_BIT_INFO(hstate, 8, VECS_AS_CONTEXT_SWITCH, IRQ_INFO_GT3);

	SET_BIT_INFO(hstate, 0, PIPE_A_VBLANK, IRQ_INFO_DE_PIPE_A);
	SET_BIT_INFO(hstate, 0, PIPE_B_VBLANK, IRQ_INFO_DE_PIPE_B);
	SET_BIT_INFO(hstate, 0, PIPE_C_VBLANK, IRQ_INFO_DE_PIPE_C);

	/* GEN8 interrupt DE PORT events */
	SET_BIT_INFO(hstate, 0, AUX_CHANNEL_A, IRQ_INFO_DE_PORT);
	SET_BIT_INFO(hstate, 3, DP_A_HOTPLUG, IRQ_INFO_DE_PORT);

	/* GEN8 interrupt DE MISC events */
	SET_BIT_INFO(hstate, 0, GSE, IRQ_INFO_DE_MISC);

	/* PCH events */
	SET_BIT_INFO(hstate, 17, GMBUS, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 19, CRT_HOTPLUG, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 21, DP_B_HOTPLUG, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 22, DP_C_HOTPLUG, IRQ_INFO_PCH);
	SET_BIT_INFO(hstate, 23, DP_D_HOTPLUG, IRQ_INFO_PCH);

	/* GEN8 interrupt PCU events */
	SET_BIT_INFO(hstate, 24, PCU_THERMAL, IRQ_INFO_PCU);
	SET_BIT_INFO(hstate, 25, PCU_PCODE2DRIVER_MAILBOX, IRQ_INFO_PCU);

	if (IS_BDW(pdev)) {
		SET_BIT_INFO(hstate, 25, AUX_CHANNEL_B, IRQ_INFO_PCH);
		SET_BIT_INFO(hstate, 26, AUX_CHANNEL_C, IRQ_INFO_PCH);
		SET_BIT_INFO(hstate, 27, AUX_CHANNEL_D, IRQ_INFO_PCH);

		SET_BIT_INFO(hstate, 4, PRIMARY_A_FLIP_DONE, IRQ_INFO_DE_PIPE_A);
		SET_BIT_INFO(hstate, 5, SPRITE_A_FLIP_DONE, IRQ_INFO_DE_PIPE_A);

		SET_BIT_INFO(hstate, 4, PRIMARY_B_FLIP_DONE, IRQ_INFO_DE_PIPE_B);
		SET_BIT_INFO(hstate, 5, SPRITE_B_FLIP_DONE, IRQ_INFO_DE_PIPE_B);

		SET_BIT_INFO(hstate, 4, PRIMARY_C_FLIP_DONE, IRQ_INFO_DE_PIPE_C);
		SET_BIT_INFO(hstate, 5, SPRITE_C_FLIP_DONE, IRQ_INFO_DE_PIPE_C);
	}

	if (IS_SKL(pdev)) {
		SET_BIT_INFO(hstate, 25, AUX_CHANNEL_B, IRQ_INFO_DE_PORT);
		SET_BIT_INFO(hstate, 26, AUX_CHANNEL_C, IRQ_INFO_DE_PORT);
		SET_BIT_INFO(hstate, 27, AUX_CHANNEL_D, IRQ_INFO_DE_PORT);
		/*
		 * Only support page flip interrupt on primary plane.
		 */
		SET_BIT_INFO(hstate, 3, PRIMARY_A_FLIP_DONE, IRQ_INFO_DE_PIPE_A);
		SET_BIT_INFO(hstate, 3, PRIMARY_B_FLIP_DONE, IRQ_INFO_DE_PIPE_B);
		SET_BIT_INFO(hstate, 3, PRIMARY_C_FLIP_DONE, IRQ_INFO_DE_PIPE_C);
	}

	irq_based_ctx_switch = false;
}

static void vgt_gen8_disable_irq(struct vgt_irq_host_state *hstate)
{
	struct pgt_device *pdev = hstate->pdev;

	VGT_MMIO_WRITE(pdev, GEN8_MASTER_IRQ,
			(VGT_MMIO_READ(pdev, GEN8_MASTER_IRQ)
			 & ~GEN8_MASTER_IRQ_CONTROL));
	VGT_POST_READ(pdev, GEN8_MASTER_IRQ);
}

static void vgt_gen8_enable_irq(struct vgt_irq_host_state *hstate)
{
	struct pgt_device *pdev = hstate->pdev;

	VGT_MMIO_WRITE(pdev, GEN8_MASTER_IRQ,
			(VGT_MMIO_READ(pdev, GEN8_MASTER_IRQ)
			 | GEN8_MASTER_IRQ_CONTROL));
	VGT_POST_READ(pdev, GEN8_MASTER_IRQ);
}

struct vgt_irq_ops vgt_gen8_irq_ops = {
	.irq_handler = vgt_gen8_irq_handler,
	.init_irq = vgt_gen8_init_irq,
	.check_pending_irq = vgt_gen8_check_pending_irq,
	.disable_irq = vgt_gen8_disable_irq,
	.enable_irq = vgt_gen8_enable_irq,
};

/* ======================common event logic====================== */

/*
 * Trigger a virtual event which comes from other requests like hotplug agent
 * instead of from pirq.
 */
void vgt_trigger_virtual_event(struct vgt_device *vgt,
	enum vgt_event_type event)
{
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_irq_host_state *hstate = pdev->irq_hstate;
	vgt_event_virt_handler_t handler;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);

	ASSERT(spin_is_locked(&pdev->lock));

	handler = vgt_get_event_virt_handler(hstate, event);
	ASSERT(handler);

	handler(hstate, event, vgt);

	ops->check_pending_irq(vgt);
}

/*
 * Forward cached physical events to VMs, invoked from kernel thread
 */
void vgt_forward_events(struct pgt_device *pdev)
{
	int i, event;
	cycles_t delay;
	struct vgt_irq_host_state *hstate = pdev->irq_hstate;
	vgt_event_virt_handler_t handler;
	struct vgt_irq_ops *ops = vgt_get_irq_ops(pdev);
	enum vgt_event_type virtual_event;

	/* WARING: this should be under lock protection */
	//raise_ctx_sched(vgt_dom0);

	pdev->stat.last_virq = get_cycles();
	delay = pdev->stat.last_virq - pdev->stat.last_pirq;

	/*
	 * it's possible a new pirq coming before last request is handled.
	 * or the irq may come before kthread is ready. So skip the 1st 5.
	 */
	if (delay > 0 && pdev->stat.irq_num > 5)
		pdev->stat.irq_delay_cycles += delay;

	ASSERT(spin_is_locked(&pdev->lock));
	for_each_set_bit(event, hstate->pending_events, EVENT_MAX) {
		clear_bit(event, hstate->pending_events);

		handler = vgt_get_event_virt_handler(hstate, event);
		ASSERT(handler);

		switch (vgt_get_event_policy(hstate, event)) {
		case EVENT_FW_ALL:
			for (i = 0; i < VGT_MAX_VMS; i++) {
				if (pdev->device[i]) {
					virtual_event = translate_physical_event(pdev->device[i], event);
					handler(hstate, virtual_event, pdev->device[i]);
				}
			}
			break;
		case EVENT_FW_DOM0:
			virtual_event = translate_physical_event(vgt_dom0, event);
			handler(hstate, virtual_event, vgt_dom0);
			break;
		case EVENT_FW_RDR:
			handler(hstate, event, current_render_owner(pdev));
			break;
		case EVENT_FW_NONE:
		default:
			break;
		}
	}

	for (i = 0; i < VGT_MAX_VMS; i++) {
		if (pdev->device[i])
			ops->check_pending_irq(pdev->device[i]);
	}

	pdev->stat.virq_cycles += get_cycles() - pdev->stat.last_virq;
}

inline bool vgt_need_emulated_irq(struct vgt_device *vgt, enum pipe pipe)
{
	bool rc = false;
	if (vgt_has_pipe_enabled(vgt, pipe)) {
		enum pipe phys_pipe = vgt->pipe_mapping[pipe];
		if ((phys_pipe == I915_MAX_PIPES) ||
			!pdev_has_pipe_enabled(vgt->pdev, phys_pipe))
			rc = true;
	}
	return rc;
}

static inline void vgt_emulate_vblank(struct vgt_device *vgt,
			enum pipe pipe)
{
	enum vgt_event_type vblank;
	switch (pipe) {
	case PIPE_A:
		vblank = PIPE_A_VBLANK; break;
	case PIPE_B:
		vblank = PIPE_B_VBLANK; break;
	case PIPE_C:
		vblank = PIPE_C_VBLANK; break;
	default:
		ASSERT(0);
	}

	if ((__vreg(vgt, VGT_PIPECONF(pipe)) & _REGBIT_PIPE_ENABLE) ||
			(__vreg(vgt, _REG_PIPE_EDP_CONF) & _REGBIT_PIPE_ENABLE)) {
		__vreg(vgt, VGT_PIPE_FRMCOUNT(pipe))++;
		vgt_trigger_virtual_event(vgt, vblank);
	}
}

/*TODO
 * In vgt_emulate_dpy_events(), so far only one virtual virtual
 * event is injected into VM. If more than one events are injected, we
 * should use a new function other than vgt_trigger_virtual_event(),
 * that new one can combine multiple virtual events into a single
 * virtual interrupt.
 */
void vgt_emulate_dpy_events(struct pgt_device *pdev)
{
	int i;

	ASSERT(spin_is_locked(&pdev->lock));
	for (i = 0; i < VGT_MAX_VMS; i ++) {
		struct vgt_device *vgt = pdev->device[i];

		if (!vgt || is_current_display_owner(vgt))
			continue;

		vgt_emulate_vblank(vgt, PIPE_A);
		vgt_emulate_vblank(vgt, PIPE_B);
		vgt_emulate_vblank(vgt, PIPE_C);

		vgt->stat.last_vblank_time = vgt_get_cycles();
	}
}

/*
 * Scan all pending events in the specified category, and then invoke
 * registered handler accordingly
 */
static void vgt_handle_events(struct vgt_irq_host_state *hstate, void *iir,
		struct vgt_irq_info *info)
{
	int bit;
	enum vgt_event_type event;
	vgt_event_phys_handler_t handler;
	struct pgt_device *pdev = hstate->pdev;

	ASSERT(spin_is_locked(&pdev->irq_lock));

	for_each_set_bit(bit, iir, VGT_IRQ_BITWIDTH) {
		if (test_bit(bit, info->downstream_irq_bitmap)) {
			process_downstream_irq(hstate, info, bit);
			continue;
		}

		event = info->bit_to_event[bit];
		pdev->stat.events[event]++;

		if (unlikely(event == EVENT_RESERVED)) {
			if (!test_and_set_bit(bit, &info->warned))
				vgt_err("IRQ: abandon non-registered [%s, bit-%d] event (%s)\n",
						info->name, bit, vgt_irq_name[event]);
			continue;
		}

		handler = vgt_get_event_phys_handler(hstate, event);
		ASSERT(handler);

		handler(hstate, event);
		set_bit(event, hstate->pending_events);
	}
}

/*
 * Physical interrupt handler for Intel HD serious graphics
 *   - handle various interrupt reasons
 *   - may trigger virtual interrupt instances to dom0 or other VMs
 */
irqreturn_t vgt_interrupt(int irq, void *data)
{
	struct pgt_device *pdev = i915_drm_to_pgt(data);
	struct vgt_irq_host_state *hstate = pdev->irq_hstate;
	irqreturn_t ret;

	/******************  PLEASE NOTE!!! **********************
	 * we should not try to hold any pdev->lock in irq env   *
	 *********************************************************/
	pdev->stat.irq_num++;
	pdev->stat.last_pirq = get_cycles();


	/* avoid nested handling by disabling master interrupt */
	hstate->ops->disable_irq(hstate);

	spin_lock(&pdev->irq_lock);

	ret = hstate->ops->irq_handler(hstate);
	if (ret == IRQ_NONE) {
		vgt_dbg(VGT_DBG_IRQ, "Spurious interrupt received (or shared vector)\n");
		goto out;
	}

	vgt_raise_request(pdev, VGT_REQUEST_IRQ);

out:
	spin_unlock(&pdev->irq_lock);

	/* re-enable master interrupt */
	hstate->ops->enable_irq(hstate);

	pdev->stat.pirq_cycles += get_cycles() - pdev->stat.last_pirq;

	return IRQ_HANDLED;
}

/* default handler will be invoked, if not explicitly specified here */
static void vgt_init_events(
	struct vgt_irq_host_state *hstate)
{
	int i;

#define SET_POLICY_ALL(h, e)	\
	((h)->events[e].policy = EVENT_FW_ALL)
#define SET_POLICY_DOM0(h, e)	\
	((h)->events[e].policy = EVENT_FW_DOM0)
#define SET_POLICY_NONE(h, e)	\
	((h)->events[e].policy = EVENT_FW_NONE)
#define SET_POLICY_RDR(h, e)	\
	((h)->events[e].policy = EVENT_FW_RDR)
#define SET_P_HANDLER(s, e, h)	\
	((s)->events[e].p_handler = h)
#define SET_V_HANDLER(s, e, h)	\
	((s)->events[e].v_handler = h)

	for (i = 0; i < EVENT_MAX; i++) {
		hstate->events[i].info = NULL;
		/* Default forwarding to all VMs (render and most display events) */
		SET_POLICY_DOM0(hstate, i);
		hstate->events[i].p_handler = vgt_handle_default_event_phys;
		hstate->events[i].v_handler = vgt_handle_default_event_virt;;
	}

	SET_P_HANDLER(hstate, DPST_PHASE_IN, vgt_handle_phase_in_phys);
	SET_P_HANDLER(hstate, DPST_HISTOGRAM, vgt_handle_histogram_phys);
	SET_P_HANDLER(hstate, CRT_HOTPLUG, vgt_handle_crt_hotplug_phys);
	SET_P_HANDLER(hstate, DP_B_HOTPLUG, vgt_handle_port_hotplug_phys);
	SET_P_HANDLER(hstate, DP_C_HOTPLUG, vgt_handle_port_hotplug_phys);
	SET_P_HANDLER(hstate, DP_D_HOTPLUG, vgt_handle_port_hotplug_phys);

	SET_P_HANDLER(hstate, RCS_AS_CONTEXT_SWITCH, vgt_handle_ctx_switch_phys);
	SET_P_HANDLER(hstate, VCS_AS_CONTEXT_SWITCH, vgt_handle_ctx_switch_phys);
	SET_P_HANDLER(hstate, VCS2_AS_CONTEXT_SWITCH, vgt_handle_ctx_switch_phys);
	SET_P_HANDLER(hstate, BCS_AS_CONTEXT_SWITCH, vgt_handle_ctx_switch_phys);
	SET_P_HANDLER(hstate, VECS_AS_CONTEXT_SWITCH, vgt_handle_ctx_switch_phys);

	SET_V_HANDLER(hstate, DPST_PHASE_IN, vgt_handle_phase_in_virt);
	SET_V_HANDLER(hstate, DPST_HISTOGRAM, vgt_handle_histogram_virt);
	SET_V_HANDLER(hstate, CRT_HOTPLUG, vgt_handle_crt_hotplug_virt);
	SET_V_HANDLER(hstate, DP_B_HOTPLUG, vgt_handle_port_hotplug_virt);
	SET_V_HANDLER(hstate, DP_C_HOTPLUG, vgt_handle_port_hotplug_virt);
	SET_V_HANDLER(hstate, DP_D_HOTPLUG, vgt_handle_port_hotplug_virt);

	SET_V_HANDLER(hstate, RCS_AS_CONTEXT_SWITCH, vgt_handle_ctx_switch_virt);
	SET_V_HANDLER(hstate, VCS_AS_CONTEXT_SWITCH, vgt_handle_ctx_switch_virt);
	SET_V_HANDLER(hstate, VCS2_AS_CONTEXT_SWITCH, vgt_handle_ctx_switch_virt);
	SET_V_HANDLER(hstate, BCS_AS_CONTEXT_SWITCH, vgt_handle_ctx_switch_virt);
	SET_V_HANDLER(hstate, VECS_AS_CONTEXT_SWITCH, vgt_handle_ctx_switch_virt);

	SET_V_HANDLER(hstate, RCS_MI_USER_INTERRUPT, vgt_handle_ring_empty_notify_virt);
	SET_V_HANDLER(hstate, VCS_MI_USER_INTERRUPT, vgt_handle_ring_empty_notify_virt);
	SET_V_HANDLER(hstate, BCS_MI_USER_INTERRUPT, vgt_handle_ring_empty_notify_virt);
	SET_V_HANDLER(hstate, RCS_PIPE_CONTROL, vgt_handle_ring_empty_notify_virt);
	SET_V_HANDLER(hstate, VCS_MI_FLUSH_DW, vgt_handle_ring_empty_notify_virt);
	SET_V_HANDLER(hstate, VECS_MI_FLUSH_DW, vgt_handle_ring_empty_notify_virt);

	/*for render related*/
	SET_POLICY_ALL(hstate,RCS_MI_USER_INTERRUPT);
	SET_POLICY_ALL(hstate,RCS_PIPE_CONTROL);
	SET_POLICY_ALL(hstate,RCS_AS_CONTEXT_SWITCH);

	SET_POLICY_ALL(hstate,BCS_MI_USER_INTERRUPT);
	SET_POLICY_ALL(hstate,BCS_MI_FLUSH_DW);
	SET_POLICY_ALL(hstate,BCS_AS_CONTEXT_SWITCH);
	
	SET_POLICY_ALL(hstate,VCS_MI_USER_INTERRUPT);
	SET_POLICY_ALL(hstate,VCS_MI_FLUSH_DW);
	SET_POLICY_ALL(hstate,VCS_AS_CONTEXT_SWITCH);
	
	SET_POLICY_ALL(hstate,VCS2_MI_USER_INTERRUPT);
	SET_POLICY_ALL(hstate,VCS2_MI_FLUSH_DW);
	SET_POLICY_ALL(hstate,VCS2_AS_CONTEXT_SWITCH);

	SET_POLICY_ALL(hstate,VECS_MI_USER_INTERRUPT);
	SET_POLICY_ALL(hstate,VECS_MI_FLUSH_DW);
	SET_POLICY_ALL(hstate,VECS_AS_CONTEXT_SWITCH);

	if (IS_BDW(hstate->pdev) || IS_SKL(hstate->pdev)) {
		SET_POLICY_RDR(hstate, RCS_MI_USER_INTERRUPT);
		SET_POLICY_RDR(hstate, VCS_MI_USER_INTERRUPT);
		SET_POLICY_RDR(hstate, BCS_MI_USER_INTERRUPT);
		SET_POLICY_RDR(hstate, VECS_MI_USER_INTERRUPT);
		SET_POLICY_RDR(hstate, RCS_PIPE_CONTROL);
		SET_POLICY_RDR(hstate, BCS_MI_FLUSH_DW);
		SET_POLICY_RDR(hstate, VCS_MI_FLUSH_DW);
		SET_POLICY_RDR(hstate, VECS_MI_FLUSH_DW);
	}
}

static enum hrtimer_restart vgt_dpy_timer_fn(struct hrtimer *data)
{
	struct vgt_emul_timer *dpy_timer;
	struct vgt_irq_host_state *hstate;
	struct pgt_device *pdev;

	dpy_timer = container_of(data, struct vgt_emul_timer, timer);
	hstate = container_of(dpy_timer, struct vgt_irq_host_state, dpy_timer);
	pdev = hstate->pdev;

	vgt_raise_request(pdev, VGT_REQUEST_EMUL_DPY_EVENTS);

	hrtimer_add_expires_ns(&dpy_timer->timer, dpy_timer->period);
	return HRTIMER_RESTART;
}

/*
 * Do interrupt initialization for vGT driver
 */
int vgt_irq_init(struct pgt_device *pdev)
{
	struct vgt_irq_host_state *hstate;
	struct vgt_emul_timer *dpy_timer;

	hstate = kzalloc(sizeof(struct vgt_irq_host_state), GFP_KERNEL);
	if (hstate == NULL)
		return -ENOMEM;

	if (IS_SNB(pdev)) {
		hstate->ops = &vgt_base_irq_ops;
		hstate->irq_map = snb_irq_map;
	} else if (IS_IVB(pdev) || IS_HSW(pdev)) {
		hstate->ops = &vgt_base_irq_ops;
		hstate->irq_map = base_irq_map;
	} else if (IS_BDW(pdev) || IS_SKL(pdev)) {
		hstate->ops = &vgt_gen8_irq_ops;
		hstate->irq_map = gen8_irq_map;
	} else {
		vgt_err("Unsupported device\n");
		kfree(hstate);
		return -EINVAL;
	}

	spin_lock_init(&pdev->irq_lock);

	hstate->pdev = pdev;
	hstate->i915_irq = -1;
	//hstate.pirq = IRQ_INVALID;

	/* common event initialization */
	vgt_init_events(hstate);

	/* gen specific initialization */
	hstate->ops->init_irq(hstate);

	vgt_irq_map_init(hstate);

	pdev->irq_hstate = hstate;
	pdev->dom0_irq_cpu = -1;
	pdev->dom0_irq_pending = false;
	pdev->dom0_ipi_irq_injecting = 0;

	dpy_timer = &hstate->dpy_timer;
	hrtimer_init(&dpy_timer->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	dpy_timer->timer.function = vgt_dpy_timer_fn;
	dpy_timer->period = VGT_DPY_EMUL_PERIOD;

	return 0;
}

void vgt_irq_exit(struct pgt_device *pdev)
{
	hrtimer_cancel(&pdev->irq_hstate->dpy_timer.timer);
	kfree(pdev->irq_hstate);
}

void *vgt_init_irq(struct pci_dev *pdev, struct drm_device *dev)
{
	struct pgt_device *node, *pgt = NULL;
	int irq;
	struct vgt_irq_host_state *hstate;

	if (!hypervisor_check_host() || !vgt_enabled)
		return NULL;

	if (list_empty(&pgt_devices)) {
		printk("vGT: no valid pgt_device registered when installing irq\n");
		return NULL;
	}

	list_for_each_entry(node, &pgt_devices, list) {
		if (node->pdev == pdev) {
			pgt = node;
			break;
		}
	}

	if (!pgt) {
		printk("vGT: no matching pgt_device when registering irq\n");
		return NULL;
	}

	printk("vGT: found matching pgt_device when registering irq for dev (0x%x)\n", pdev->devfn);

	hstate = pgt->irq_hstate;
	if (hstate->installed) {
		printk("vGT: IRQ has been installed already.\n");
		return NULL;
	}

	irq = -1;
	hstate->pirq = pdev->irq;
	hstate->i915_irq = irq;

	hstate->installed = true;

	printk("vGT: track_nest: %s\n", vgt_track_nest ? "enabled" : "disabled");

	return pgt;
}

void vgt_fini_irq(struct pci_dev *pdev)
{
	struct pgt_device *node, *pgt = NULL;
	struct vgt_irq_host_state *hstate;

	if (!hypervisor_check_host() || !vgt_enabled)
		return;

	if (list_empty(&pgt_devices)) {
		printk("vGT: no valid pgt_device registered when installing irq\n");
		return;
	}

	list_for_each_entry(node, &pgt_devices, list) {
		if (node->pdev == pdev) {
			pgt = node;
			break;
		}
	}

	if (!pgt) {
		printk("vGT: no matching pgt_device when registering irq\n");
		return;
	}

	hstate = pgt->irq_hstate;
	if (!hstate->installed) {
		printk("vGT: IRQ hasn't been installed yet.\n");
		return;
	}

	/* Mask all GEN interrupts */
	VGT_MMIO_WRITE(pgt, DEIER,
		VGT_MMIO_READ(pgt, DEIER) & ~MASTER_INTERRUPT_ENABLE);

	hstate->installed = false;
}

void vgt_inject_flip_done(struct vgt_device *vgt, enum pipe pipe)
{
	enum vgt_event_type event = EVENT_MAX;
	if (current_display_owner(vgt->pdev) != vgt) {
		if (pipe == PIPE_A) {
			event = PRIMARY_A_FLIP_DONE;
		} else if (pipe == PIPE_B) {
			event = PRIMARY_B_FLIP_DONE;
		} else if (pipe == PIPE_C) {
			event = PRIMARY_C_FLIP_DONE;
		}

		if (event != EVENT_MAX) {
			vgt_trigger_virtual_event(vgt, event);
		}
	}
}
