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

#include "gvt.h"
#include "interrupt.h"

static void update_upstream_irq(struct vgt_device *vgt,
		struct gvt_irq_info *info);

static int gvt_irq_warn_once[GVT_MAX_VGPU+1][GVT_EVENT_MAX];

char *gvt_irq_name[GVT_EVENT_MAX] = {
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
	[AUX_CHENNEL_B] = "AUX Channel B",
	[AUX_CHENNEL_C] = "AUX Channel C",
	[AUX_CHENNEL_D] = "AUX Channel D",
	[AUDIO_POWER_STATE_CHANGE_B] = "Audio Power State change Port B",
	[AUDIO_POWER_STATE_CHANGE_C] = "Audio Power State change Port C",
	[AUDIO_POWER_STATE_CHANGE_D] = "Audio Power State change Port D",

	[GVT_EVENT_RESERVED] = "RESERVED EVENTS!!!",
};

static inline struct gvt_irq_info *regbase_to_irq_info(struct pgt_device *pdev,
		unsigned int reg)
{
	struct gvt_irq_state *state = &pdev->irq_state;
	int i;

	for_each_set_bit(i, state->irq_info_bitmap, GVT_IRQ_INFO_MAX) {
		if (state->info[i]->reg_base == reg)
			return state->info[i];
	}

	return NULL;
}

bool gvt_reg_imr_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes)
{
	uint32_t changed, masked, unmasked;
	uint32_t imr = *(u32 *)p_data;
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_irq_ops *ops = gvt_get_irq_ops(pdev);

	gvt_dbg_irq("IRQ: capture IMR write on reg (%x) with val (%x)",
		reg, imr);

	gvt_dbg_irq("IRQ: old vIMR(%x), pIMR(%x)",
		 __vreg(vgt, reg), gvt_mmio_read(pdev, reg));

	/* figure out newly masked/unmasked bits */
	changed = __vreg(vgt, reg) ^ imr;
	masked = (__vreg(vgt, reg) & changed) ^ changed;
	unmasked = masked ^ changed;

	gvt_dbg_irq("IRQ: changed (%x), masked(%x), unmasked (%x)",
		changed, masked, unmasked);

	__vreg(vgt, reg) = imr;

	ops->check_pending_irq(vgt);
	gvt_dbg_irq("IRQ: new vIMR(%x), pIMR(%x)",
		 __vreg(vgt, reg), gvt_mmio_read(pdev, reg));
	return true;
}

bool gvt_reg_master_irq_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes)
{
	uint32_t changed, enabled, disabled;
	uint32_t ier = *(u32 *)p_data;
	uint32_t virtual_ier = __vreg(vgt, reg);
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_irq_ops *ops = gvt_get_irq_ops(pdev);

	gvt_dbg_irq("IRQ: capture master irq write on reg (%x) with val (%x)",
		reg, ier);

	gvt_dbg_irq("IRQ: old vreg(%x), preg(%x)",
		 __vreg(vgt, reg), gvt_mmio_read(pdev, reg));

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

	gvt_dbg_irq("vGT_IRQ: changed (%x), enabled(%x), disabled(%x)",
			changed, enabled, disabled);

	ops->check_pending_irq(vgt);
	gvt_dbg_irq("IRQ: new vreg(%x), preg(%x)",
		 __vreg(vgt, reg), gvt_mmio_read(pdev, reg));
	return true;
}

bool gvt_reg_ier_handler(struct vgt_device *vgt,
	unsigned int reg, void *p_data, unsigned int bytes)
{
	uint32_t changed, enabled, disabled;
	uint32_t ier = *(u32 *)p_data;
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_irq_ops *ops = gvt_get_irq_ops(pdev);
	struct gvt_irq_info *info;

	gvt_dbg_irq("IRQ: capture IER write on reg (%x) with val (%x)",
		reg, ier);

	gvt_dbg_irq("IRQ: old vIER(%x), pIER(%x)",
		 __vreg(vgt, reg), gvt_mmio_read(pdev, reg));

	/* figure out newly enabled/disable bits */
	changed = __vreg(vgt, reg) ^ ier;
	enabled = (__vreg(vgt, reg) & changed) ^ changed;
	disabled = enabled ^ changed;

	gvt_dbg_irq("vGT_IRQ: changed (%x), enabled(%x), disabled(%x)",
			changed, enabled, disabled);
	__vreg(vgt, reg) = ier;

	info = regbase_to_irq_info(pdev, ier_to_regbase(reg));
	if (!info)
		return false;

	if (info->has_upstream_irq)
		update_upstream_irq(vgt, info);

	ops->check_pending_irq(vgt);
	gvt_dbg_irq("IRQ: new vIER(%x), pIER(%x)",
		 __vreg(vgt, reg), gvt_mmio_read(pdev, reg));
	return true;
}

bool gvt_reg_iir_handler(struct vgt_device *vgt, unsigned int reg,
	void *p_data, unsigned int bytes)
{
	struct gvt_irq_info *info = regbase_to_irq_info(vgt->pdev, iir_to_regbase(reg));
	u32 iir = *(u32 *)p_data;

	gvt_dbg_irq("IRQ: capture IIR write on reg (%x) with val (%x)",
		reg, iir);

	if (!info)
		return false;

	/* TODO: need use an atomic operation. Now it's safe due to big lock */
	__vreg(vgt, reg) &= ~iir;

	if (info->has_upstream_irq)
		update_upstream_irq(vgt, info);

	return true;
}

bool gvt_reg_isr_write(struct vgt_device *vgt, unsigned int reg,
	void *p_data, unsigned int bytes)
{
	gvt_dbg_irq("IRQ: capture ISR write on reg (%x) with val (%x)." \
		" Will be ignored!", reg, *(u32 *)p_data);

	return true;
}

struct gvt_irq_map gen8_irq_map[] = {
	{ GVT_IRQ_INFO_MASTER, 0, GVT_IRQ_INFO_GT0, 0xffff },
	{ GVT_IRQ_INFO_MASTER, 1, GVT_IRQ_INFO_GT0, 0xffff0000 },
	{ GVT_IRQ_INFO_MASTER, 2, GVT_IRQ_INFO_GT1, 0xffff },
	{ GVT_IRQ_INFO_MASTER, 3, GVT_IRQ_INFO_GT1, 0xffff0000 },
	{ GVT_IRQ_INFO_MASTER, 4, GVT_IRQ_INFO_GT2, 0xffff },
	{ GVT_IRQ_INFO_MASTER, 6, GVT_IRQ_INFO_GT3, 0xffff },
	{ GVT_IRQ_INFO_MASTER, 16, GVT_IRQ_INFO_DE_PIPE_A, ~0 },
	{ GVT_IRQ_INFO_MASTER, 17, GVT_IRQ_INFO_DE_PIPE_B, ~0 },
	{ GVT_IRQ_INFO_MASTER, 18, GVT_IRQ_INFO_DE_PIPE_C, ~0 },
	{ GVT_IRQ_INFO_MASTER, 20, GVT_IRQ_INFO_DE_PORT, ~0 },
	{ GVT_IRQ_INFO_MASTER, 22, GVT_IRQ_INFO_DE_MISC, ~0 },
	{ GVT_IRQ_INFO_MASTER, 23, GVT_IRQ_INFO_PCH, ~0 },
	{ GVT_IRQ_INFO_MASTER, 30, GVT_IRQ_INFO_PCU, ~0 },
	{ -1, -1, ~0 },
};

static void update_upstream_irq(struct vgt_device *vgt,
		struct gvt_irq_info *info)
{
	struct gvt_irq_state *state = &vgt->pdev->irq_state;
	struct gvt_irq_map *map = state->irq_map;
	struct gvt_irq_info *up_irq_info = NULL;
	u32 set_bits = 0;
	u32 clear_bits = 0;
	int bit;
	u32 val = __vreg(vgt, regbase_to_iir(info->reg_base))
			& __vreg(vgt, regbase_to_ier(info->reg_base));

	if (!info->has_upstream_irq)
		return;

	for (map = state->irq_map; map->up_irq_bit != -1; map++) {
		if (info->group != map->down_irq_group)
			continue;

		if (!up_irq_info)
			up_irq_info = state->info[map->up_irq_group];
		else
			ASSERT(up_irq_info == state->info[map->up_irq_group]);

		bit = map->up_irq_bit;

		if (val & map->down_irq_bitmask)
			set_bits |= (1 << bit);
		else
			clear_bits |= (1 << bit);
	}

	ASSERT(up_irq_info);

	if (up_irq_info->group == GVT_IRQ_INFO_MASTER) {
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

static void gvt_irq_map_init(struct gvt_irq_state *state)
{
	struct gvt_irq_map *map;
	struct gvt_irq_info *up_info, *down_info;
	int up_bit;

	for (map = state->irq_map; map->up_irq_bit != -1; map++) {
		up_info = state->info[map->up_irq_group];
		up_bit = map->up_irq_bit;
		down_info = state->info[map->down_irq_group];

		set_bit(up_bit, up_info->downstream_irq_bitmap);
		down_info->has_upstream_irq = true;

		gvt_dbg_irq("irq map [upstream] group: %d, bit: %d -> [downstream] group: %d, bitmask: 0x%x",
			up_info->group, up_bit, down_info->group, map->down_irq_bitmask);
	}
}

/* =======================vEvent injection===================== */
static int gvt_inject_virtual_interrupt(struct vgt_device *vgt)
{
	hypervisor_inject_msi(vgt);

	vgt->stat.irq_num++;
	vgt->stat.last_injection = get_cycles();
	return 0;
}

static void propagate_event(struct gvt_irq_state *state,
	enum gvt_event_type event, struct vgt_device *vgt)
{
	int bit;
	struct gvt_irq_info *info;
	unsigned int reg_base;

	info = gvt_get_irq_info(state, event);
	if (!info) {
		gvt_err("IRQ(%d): virt-inject: no irq reg info!!!",
			vgt->vm_id);
		return;
	}

	reg_base = info->reg_base;
	bit = state->events[event].bit;

	if (!test_bit(bit, (void*)&__vreg(vgt, regbase_to_imr(reg_base)))) {
		gvt_dbg_irq("IRQ: set bit (%d) for (%s) for VM (%d)",
			bit, gvt_irq_name[event], vgt->vm_id);
		set_bit(bit, (void*)&__vreg(vgt, regbase_to_iir(reg_base)));
	}
}

/* =======================vEvent Handlers===================== */
static void handle_default_event_virt(struct gvt_irq_state *state,
	enum gvt_event_type event, struct vgt_device *vgt)
{
	if (!gvt_irq_warn_once[vgt->id][event]) {
		gvt_info("IRQ: VM(%d) receive event %d (%s)",
			vgt->vm_id, event, gvt_irq_name[event]);
		gvt_irq_warn_once[vgt->id][event] = 1;
	}
	propagate_event(state, event, vgt);
	vgt->stat.events[event]++;
}

/* =====================GEN specific logic======================= */
/* GEN8 interrupt routines. */

#define DEFINE_GVT_GEN8_GVT_IRQ_INFO(regname, regbase) \
       static struct gvt_irq_info gen8_##regname##_info = { \
               .name = #regname"-IRQ", \
               .reg_base = regbase, \
               .bit_to_event = {[0 ... GVT_IRQ_BITWIDTH-1] = GVT_EVENT_RESERVED}, \
       };

DEFINE_GVT_GEN8_GVT_IRQ_INFO(gt0, _GEN8_GT_ISR(0));
DEFINE_GVT_GEN8_GVT_IRQ_INFO(gt1, _GEN8_GT_ISR(1));
DEFINE_GVT_GEN8_GVT_IRQ_INFO(gt2, _GEN8_GT_ISR(2));
DEFINE_GVT_GEN8_GVT_IRQ_INFO(gt3, _GEN8_GT_ISR(3));
DEFINE_GVT_GEN8_GVT_IRQ_INFO(de_pipe_a, _GEN8_DE_PIPE_ISR(PIPE_A));
DEFINE_GVT_GEN8_GVT_IRQ_INFO(de_pipe_b, _GEN8_DE_PIPE_ISR(PIPE_B));
DEFINE_GVT_GEN8_GVT_IRQ_INFO(de_pipe_c, _GEN8_DE_PIPE_ISR(PIPE_C));
DEFINE_GVT_GEN8_GVT_IRQ_INFO(de_port, _GEN8_DE_PORT_ISR);
DEFINE_GVT_GEN8_GVT_IRQ_INFO(de_misc, _GEN8_DE_MISC_ISR);
DEFINE_GVT_GEN8_GVT_IRQ_INFO(pcu, _GEN8_PCU_ISR);
DEFINE_GVT_GEN8_GVT_IRQ_INFO(master, _GEN8_MASTER_IRQ);

static struct gvt_irq_info gvt_base_pch_info = {
        .name = "PCH-IRQ",
        .reg_base = _SDEISR,
        .bit_to_event = {[0 ... GVT_IRQ_BITWIDTH-1] = GVT_EVENT_RESERVED},
};

static void gen8_check_pending_irq(struct vgt_device *vgt)
{
	struct gvt_irq_state *state = &vgt->pdev->irq_state;
	int i;

	if (!(__vreg(vgt, _GEN8_MASTER_IRQ) &
				GEN8_MASTER_IRQ_CONTROL))
		return;

	for_each_set_bit(i, state->irq_info_bitmap, GVT_IRQ_INFO_MAX) {
		struct gvt_irq_info *info = state->info[i];

		if (!info->has_upstream_irq)
			continue;

		if ((__vreg(vgt, regbase_to_iir(info->reg_base))
					& __vreg(vgt, regbase_to_ier(info->reg_base))))
			update_upstream_irq(vgt, info);
	}

	if (__vreg(vgt, _GEN8_MASTER_IRQ) & ~GEN8_MASTER_IRQ_CONTROL)
		gvt_inject_virtual_interrupt(vgt);
}

static void gen8_init_irq(
		struct gvt_irq_state *state)
{
	struct pgt_device *pdev = gvt_irq_state_to_pdev(state);

#define SET_BIT_INFO(s, b, e, i)		\
	do {					\
		s->events[e].bit = b;		\
		s->events[e].info = s->info[i];	\
		s->info[i]->bit_to_event[b] = e;\
	} while (0);

#define SET_IRQ_GROUP(s, g, i) \
	do { \
		s->info[g] = i; \
		(i)->group = g; \
		set_bit(g, s->irq_info_bitmap); \
	} while (0);

	SET_IRQ_GROUP(state, GVT_IRQ_INFO_MASTER, &gen8_master_info);
	SET_IRQ_GROUP(state, GVT_IRQ_INFO_GT0, &gen8_gt0_info);
	SET_IRQ_GROUP(state, GVT_IRQ_INFO_GT1, &gen8_gt1_info);
	SET_IRQ_GROUP(state, GVT_IRQ_INFO_GT2, &gen8_gt2_info);
	SET_IRQ_GROUP(state, GVT_IRQ_INFO_GT3, &gen8_gt3_info);
	SET_IRQ_GROUP(state, GVT_IRQ_INFO_DE_PIPE_A, &gen8_de_pipe_a_info);
	SET_IRQ_GROUP(state, GVT_IRQ_INFO_DE_PIPE_B, &gen8_de_pipe_b_info);
	SET_IRQ_GROUP(state, GVT_IRQ_INFO_DE_PIPE_C, &gen8_de_pipe_c_info);
	SET_IRQ_GROUP(state, GVT_IRQ_INFO_DE_PORT, &gen8_de_port_info);
	SET_IRQ_GROUP(state, GVT_IRQ_INFO_DE_MISC, &gen8_de_misc_info);
	SET_IRQ_GROUP(state, GVT_IRQ_INFO_PCU, &gen8_pcu_info);
	SET_IRQ_GROUP(state, GVT_IRQ_INFO_PCH, &gvt_base_pch_info);

	/* GEN8 level 2 interrupts. */

	/* GEN8 interrupt GT0 events */
	SET_BIT_INFO(state, 0, RCS_MI_USER_INTERRUPT, GVT_IRQ_INFO_GT0);
	SET_BIT_INFO(state, 4, RCS_PIPE_CONTROL, GVT_IRQ_INFO_GT0);
	SET_BIT_INFO(state, 8, RCS_AS_CONTEXT_SWITCH, GVT_IRQ_INFO_GT0);

	SET_BIT_INFO(state, 16, BCS_MI_USER_INTERRUPT, GVT_IRQ_INFO_GT0);
	SET_BIT_INFO(state, 20, BCS_MI_FLUSH_DW, GVT_IRQ_INFO_GT0);
	SET_BIT_INFO(state, 24, BCS_AS_CONTEXT_SWITCH, GVT_IRQ_INFO_GT0);

	/* GEN8 interrupt GT1 events */
	SET_BIT_INFO(state, 0, VCS_MI_USER_INTERRUPT, GVT_IRQ_INFO_GT1);
	SET_BIT_INFO(state, 4, VCS_MI_FLUSH_DW, GVT_IRQ_INFO_GT1);
	SET_BIT_INFO(state, 8, VCS_AS_CONTEXT_SWITCH, GVT_IRQ_INFO_GT1);

	if (IS_BDW_GT3(pdev->dev_priv)) {
		SET_BIT_INFO(state, 16, VCS2_MI_USER_INTERRUPT, GVT_IRQ_INFO_GT1);
		SET_BIT_INFO(state, 20, VCS2_MI_FLUSH_DW, GVT_IRQ_INFO_GT1);
		SET_BIT_INFO(state, 24, VCS2_AS_CONTEXT_SWITCH, GVT_IRQ_INFO_GT1);
	}

	/* GEN8 interrupt GT3 events */
	SET_BIT_INFO(state, 0, VECS_MI_USER_INTERRUPT, GVT_IRQ_INFO_GT3);
	SET_BIT_INFO(state, 4, VECS_MI_FLUSH_DW, GVT_IRQ_INFO_GT3);
	SET_BIT_INFO(state, 8, VECS_AS_CONTEXT_SWITCH, GVT_IRQ_INFO_GT3);

	SET_BIT_INFO(state, 0, PIPE_A_VBLANK, GVT_IRQ_INFO_DE_PIPE_A);
	SET_BIT_INFO(state, 4, PRIMARY_A_FLIP_DONE, GVT_IRQ_INFO_DE_PIPE_A);
	SET_BIT_INFO(state, 5, SPRITE_A_FLIP_DONE, GVT_IRQ_INFO_DE_PIPE_A);

	SET_BIT_INFO(state, 0, PIPE_B_VBLANK, GVT_IRQ_INFO_DE_PIPE_B);
	SET_BIT_INFO(state, 4, PRIMARY_B_FLIP_DONE, GVT_IRQ_INFO_DE_PIPE_B);
	SET_BIT_INFO(state, 5, SPRITE_B_FLIP_DONE, GVT_IRQ_INFO_DE_PIPE_B);

	SET_BIT_INFO(state, 0, PIPE_C_VBLANK, GVT_IRQ_INFO_DE_PIPE_C);
	SET_BIT_INFO(state, 4, PRIMARY_C_FLIP_DONE, GVT_IRQ_INFO_DE_PIPE_C);
	SET_BIT_INFO(state, 5, SPRITE_C_FLIP_DONE, GVT_IRQ_INFO_DE_PIPE_C);

	/* GEN8 interrupt DE PORT events */
	SET_BIT_INFO(state, 0, AUX_CHANNEL_A, GVT_IRQ_INFO_DE_PORT);
	SET_BIT_INFO(state, 3, DP_A_HOTPLUG, GVT_IRQ_INFO_DE_PORT);

	/* GEN8 interrupt DE MISC events */
	SET_BIT_INFO(state, 0, GSE, GVT_IRQ_INFO_DE_MISC);

	/* PCH events */
	SET_BIT_INFO(state, 17, GMBUS, GVT_IRQ_INFO_PCH);
	SET_BIT_INFO(state, 19, CRT_HOTPLUG, GVT_IRQ_INFO_PCH);
	SET_BIT_INFO(state, 21, DP_B_HOTPLUG, GVT_IRQ_INFO_PCH);
	SET_BIT_INFO(state, 22, DP_C_HOTPLUG, GVT_IRQ_INFO_PCH);
	SET_BIT_INFO(state, 23, DP_D_HOTPLUG, GVT_IRQ_INFO_PCH);
	SET_BIT_INFO(state, 25, AUX_CHENNEL_B, GVT_IRQ_INFO_PCH);
	SET_BIT_INFO(state, 26, AUX_CHENNEL_C, GVT_IRQ_INFO_PCH);
	SET_BIT_INFO(state, 27, AUX_CHENNEL_D, GVT_IRQ_INFO_PCH);

	/* GEN8 interrupt PCU events */
	SET_BIT_INFO(state, 24, PCU_THERMAL, GVT_IRQ_INFO_PCU);
	SET_BIT_INFO(state, 25, PCU_PCODE2DRIVER_MAILBOX, GVT_IRQ_INFO_PCU);
}

struct gvt_irq_ops gen8_irq_ops = {
	.init_irq = gen8_init_irq,
	.check_pending_irq = gen8_check_pending_irq,
};

/* ======================common event logic====================== */

/*
 * Trigger a virtual event which comes from other requests like hotplug agent
 * instead of from pirq.
 */
void gvt_trigger_virtual_event(struct vgt_device *vgt,
	enum gvt_event_type event)
{
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_irq_state *state = &pdev->irq_state;
	gvt_event_virt_handler_t handler;
	struct gvt_irq_ops *ops = gvt_get_irq_ops(pdev);

	handler = gvt_get_event_virt_handler(state, event);
	ASSERT(handler);

	handler(state, event, vgt);

	ops->check_pending_irq(vgt);
}

/* default handler will be invoked, if not explicitly specified here */
static void gvt_init_events(
	struct gvt_irq_state *state)
{
	int i;

#define SET_POLICY_ALL(h, e)	\
	((h)->events[e].policy = GVT_EVENT_FW_ALL)
#define SET_POLICY_NONE(h, e)	\
	((h)->events[e].policy = GVT_EVENT_FW_NONE)
#define SET_V_HANDLER(s, e, h)	\
	((s)->events[e].v_handler = h)

	for (i = 0; i < GVT_EVENT_MAX; i++) {
		state->events[i].info = NULL;
		/* Default forwarding to all VMs (render and most display events) */
		SET_POLICY_ALL(state, i);
		state->events[i].v_handler = handle_default_event_virt;;
	}
}

static enum hrtimer_restart gvt_dpy_timer_fn(struct hrtimer *data)
{
	struct gvt_emul_timer *dpy_timer;
	struct gvt_irq_state *state;
	struct pgt_device *pdev;

	dpy_timer = container_of(data, struct gvt_emul_timer, timer);
	state = container_of(dpy_timer, struct gvt_irq_state, dpy_timer);
	pdev = gvt_irq_state_to_pdev(state);

	gvt_raise_request(pdev, GVT_REQUEST_EMUL_DPY_EVENTS);

	hrtimer_add_expires_ns(&dpy_timer->timer, dpy_timer->period);
	return HRTIMER_RESTART;
}

/*
 * Do interrupt initialization for vGT driver
 */
bool gvt_irq_init(struct pgt_device *pdev)
{
	struct gvt_irq_state *state = &pdev->irq_state;
	struct gvt_emul_timer *dpy_timer;

	gvt_dbg_core("init irq framework");

	if (IS_BROADWELL(pdev->dev_priv)) {
		state->ops = &gen8_irq_ops;
		state->irq_map = gen8_irq_map;
	} else {
		gvt_err("Unsupported device");
		return false;
	}

	/* common event initialization */
	gvt_init_events(state);

	/* gen specific initialization */
	state->ops->init_irq(state);

	gvt_irq_map_init(state);

	dpy_timer = &state->dpy_timer;
	hrtimer_init(&dpy_timer->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	dpy_timer->timer.function = gvt_dpy_timer_fn;
	dpy_timer->period = GVT_DPY_EMUL_PERIOD;

	return true;
}

void gvt_irq_exit(struct pgt_device *pdev)
{
	struct gvt_irq_state *state = &pdev->irq_state;

	hrtimer_cancel(&state->dpy_timer.timer);
}

void gvt_inject_flip_done(struct vgt_device *vgt, int pipe)
{
	enum gvt_event_type event = GVT_EVENT_MAX;

	if (pipe == PIPE_A) {
		event = PRIMARY_A_FLIP_DONE;
	} else if (pipe == PIPE_B) {
		event = PRIMARY_B_FLIP_DONE;
	} else if (pipe == PIPE_C) {
		event = PRIMARY_C_FLIP_DONE;
	}

	if (event != GVT_EVENT_MAX)
		gvt_trigger_virtual_event(vgt, event);
}
