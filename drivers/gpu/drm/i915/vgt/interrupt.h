/*
 * vGT interrupt header
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

#ifndef _VGT_INTERRUPT_H_
#define _VGT_INTERRUPT_H_

enum vgt_event_type {
	// GT
	RCS_MI_USER_INTERRUPT = 0,
	RCS_DEBUG,
	RCS_MMIO_SYNC_FLUSH,
	RCS_CMD_STREAMER_ERR,
	RCS_PIPE_CONTROL,
	RCS_L3_PARITY_ERR,		/* IVB */
	RCS_WATCHDOG_EXCEEDED,
	RCS_PAGE_DIRECTORY_FAULT,
	RCS_AS_CONTEXT_SWITCH,
	RCS_MONITOR_BUFF_HALF_FULL,	/* IVB */

	VCS_MI_USER_INTERRUPT,
	VCS_MMIO_SYNC_FLUSH,
	VCS_CMD_STREAMER_ERR,
	VCS_MI_FLUSH_DW,
	VCS_WATCHDOG_EXCEEDED,
	VCS_PAGE_DIRECTORY_FAULT,
	VCS_AS_CONTEXT_SWITCH,

	VCS2_MI_USER_INTERRUPT,
	VCS2_MI_FLUSH_DW,
	VCS2_AS_CONTEXT_SWITCH,

	BCS_MI_USER_INTERRUPT,
	BCS_MMIO_SYNC_FLUSH,
	BCS_CMD_STREAMER_ERR,
	BCS_MI_FLUSH_DW,
	BCS_PAGE_DIRECTORY_FAULT,
	BCS_AS_CONTEXT_SWITCH,

	VECS_MI_USER_INTERRUPT,
	VECS_MI_FLUSH_DW,
	VECS_AS_CONTEXT_SWITCH,

	// DISPLAY
	PIPE_A_FIFO_UNDERRUN,	/* This is an active high level for the duration of the Pipe A FIFO underrun */
	PIPE_B_FIFO_UNDERRUN,	/* This is an active high level for the duration of the Pipe B FIFO underrun */
	PIPE_A_CRC_ERR,	/* This is an active high pulse on the Pipe A CRC error */
	PIPE_B_CRC_ERR,	/* This is an active high pulse on the Pipe B CRC error */
	PIPE_A_CRC_DONE,	/* This is an active high pulse on the Pipe A CRC done */
	PIPE_B_CRC_DONE,	/* This is an active high pulse on the Pipe B CRC done */
	PIPE_A_ODD_FIELD,	/* This is an active high level for the duration of the Pipe A interlaced odd field */
	PIPE_B_ODD_FIELD,	/* This is an active high level for the duration of the Pipe B interlaced odd field */
	PIPE_A_EVEN_FIELD,	/* This is an active high level for the duration of the Pipe A interlaced even field */
	PIPE_B_EVEN_FIELD,	/* This is an active high level for the duration of the Pipe B interlaced even field */
	PIPE_A_LINE_COMPARE,	/* This is an active high level for the duration of the selected Pipe A scan lines */
	PIPE_B_LINE_COMPARE,	/* This is an active high level for the duration of the selected Pipe B scan lines */
	PIPE_C_LINE_COMPARE,	/* This is an active high level for the duration of the selected Pipe C scan lines */
	PIPE_A_VBLANK,	/* This is an active high level for the duration of the Pipe A vertical blank */
	PIPE_B_VBLANK,	/* This is an active high level for the duration of the Pipe B vertical blank */
	PIPE_C_VBLANK,	/* This is an active high level for the duration of the Pipe C vertical blank */
	PIPE_A_VSYNC,	/* This is an active high level for the duration of the Pipe A vertical sync */
	PIPE_B_VSYNC,	/* This is an active high level for the duration of the Pipe B vertical sync */
	PIPE_C_VSYNC,	/* This is an active high level for the duration of the Pipe C vertical sync */
	PRIMARY_A_FLIP_DONE,	/* This is an active high pulse when a primary plane A flip is done */
	PRIMARY_B_FLIP_DONE,	/* This is an active high pulse when a primary plane B flip is done */
	PRIMARY_C_FLIP_DONE,	/* This is an active high pulse when a primary plane C flip is done */
	SPRITE_A_FLIP_DONE,	/* This is an active high pulse when a sprite plane A flip is done */
	SPRITE_B_FLIP_DONE,	/* This is an active high pulse when a sprite plane B flip is done */
	SPRITE_C_FLIP_DONE,	/* This is an active high pulse when a sprite plane C flip is done */

	DPST_PHASE_IN,	// This is an active high pulse on the DPST phase in event
	DPST_HISTOGRAM,	// This is an active high pulse on the AUX A done event.
	GSE,
	DP_A_HOTPLUG,
	AUX_CHANNEL_A,	// This is an active high pulse on the AUX A done event.
	PERF_COUNTER,	// This is an active high pulse when the performance counter reaches the threshold value programmed in the Performance Counter Source register
	POISON,		// This is an active high pulse on receiving the poison message
	GTT_FAULT,	// This is an active high level while either of the GTT Fault Status register bits are set
	ERROR_INTERRUPT_COMBINED,

	// PM
	GV_DOWN_INTERVAL,
	GV_UP_INTERVAL,
	RP_DOWN_THRESHOLD,
	RP_UP_THRESHOLD,
	FREQ_DOWNWARD_TIMEOUT_RC6,
	PCU_THERMAL,
	PCU_PCODE2DRIVER_MAILBOX,

	// PCH
	FDI_RX_INTERRUPTS_TRANSCODER_A,	// This is an active high level while any of the FDI_RX_ISR bits are set for transcoder A
	AUDIO_CP_CHANGE_TRANSCODER_A,	// This is an active high level while any of the FDI_RX_ISR bits are set for transcoder A
	AUDIO_CP_REQUEST_TRANSCODER_A,	// This is an active high level indicating content protection is requested by audio azalia verb programming for transcoder A
	FDI_RX_INTERRUPTS_TRANSCODER_B,
	AUDIO_CP_CHANGE_TRANSCODER_B,
	AUDIO_CP_REQUEST_TRANSCODER_B,
	FDI_RX_INTERRUPTS_TRANSCODER_C,
	AUDIO_CP_CHANGE_TRANSCODER_C,
	AUDIO_CP_REQUEST_TRANSCODER_C,
	ERR_AND_DBG,
	GMBUS,
	SDVO_B_HOTPLUG,
	CRT_HOTPLUG,
	DP_B_HOTPLUG,
	DP_C_HOTPLUG,
	DP_D_HOTPLUG,
	AUX_CHANNEL_B,
	AUX_CHANNEL_C,
	AUX_CHANNEL_D,
	AUDIO_POWER_STATE_CHANGE_B,
	AUDIO_POWER_STATE_CHANGE_C,
	AUDIO_POWER_STATE_CHANGE_D,

	RCS_IRQ,
	BCS_IRQ,
	VCS_IRQ,
	VCS2_IRQ,
	PM_IRQ,
	VECS_IRQ,
	DE_PIPE_A_IRQ,
	DE_PIPE_B_IRQ,
	DE_PIPE_C_IRQ,
	DE_PORT_IRQ,
	DE_MISC_IRQ,
	PCH_IRQ,
	PCU_IRQ,

	EVENT_RESERVED,
	EVENT_MAX,
};

struct vgt_irq_host_state;
typedef void (*vgt_event_phys_handler_t)(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event);
typedef void (*vgt_event_virt_handler_t)(struct vgt_irq_host_state *hstate,
	enum vgt_event_type event, struct vgt_device *vgt);

struct vgt_irq_ops {
	void (*init_irq) (struct vgt_irq_host_state *hstate);
	irqreturn_t (*irq_handler) (struct vgt_irq_host_state *hstate);
	void (*check_pending_irq) (struct vgt_device *vgt);
	void (*disable_irq) (struct vgt_irq_host_state *hstate);
	void (*enable_irq) (struct vgt_irq_host_state *hstate);
};

/* the list of physical interrupt control register groups */
enum vgt_irq_type {
	IRQ_INFO_GT,
	IRQ_INFO_DPY,
	IRQ_INFO_PCH,
	IRQ_INFO_PM,

	IRQ_INFO_MASTER,
	IRQ_INFO_GT0,
	IRQ_INFO_GT1,
	IRQ_INFO_GT2,
	IRQ_INFO_GT3,
	IRQ_INFO_DE_PIPE_A,
	IRQ_INFO_DE_PIPE_B,
	IRQ_INFO_DE_PIPE_C,
	IRQ_INFO_DE_PORT,
	IRQ_INFO_DE_MISC,
	IRQ_INFO_AUD,
	IRQ_INFO_PCU,

	IRQ_INFO_MAX,
};

#define VGT_IRQ_BITWIDTH	32
/* device specific interrupt bit definitions */
struct vgt_irq_info {
	char *name;
	int reg_base;
	enum vgt_event_type bit_to_event[VGT_IRQ_BITWIDTH];
	unsigned long warned;
	unsigned long default_enabled_events;
	int group;
	DECLARE_BITMAP(downstream_irq_bitmap, VGT_IRQ_BITWIDTH);
	bool has_upstream_irq;
};

#define	EVENT_FW_ALL 0	/* event forwarded to all instances */
#define	EVENT_FW_DOM0 1	/* event forwarded to dom0 only */
#define	EVENT_FW_NONE 2	/* no forward */

/* the handoff state from p-event to v-event */
union vgt_event_state {
	/* common state for bit based status */
	vgt_reg_t val;

	/* command stream error */
	struct {
		int eir_reg;
		vgt_reg_t eir_val;
	} cmd_err;
};

/* per-event information */
struct vgt_event_info {
	/* device specific info */
	int			bit;	/* map to register bit */
	union vgt_event_state	state;	/* handoff state*/
	struct vgt_irq_info	*info;	/* register info */

	/* device neutral info */
	int			policy;	/* forwarding policy */
	vgt_event_phys_handler_t	p_handler;	/* for p_event */
	vgt_event_virt_handler_t	v_handler;	/* for v_event */
};

struct vgt_emul_timer {
	struct hrtimer timer;
	u64 period;
};

#define REGBIT_INTERRUPT_PIPE_MASK    0x1f

struct vgt_irq_map {
	int up_irq_group;
	int up_irq_bit;
	int down_irq_group;
	u32 down_irq_bitmask;
};

/* structure containing device specific IRQ state */
struct vgt_irq_host_state {
	struct pgt_device *pdev;
	struct vgt_irq_ops *ops;
	int i915_irq;
	int pirq;
	struct vgt_irq_info	*info[IRQ_INFO_MAX];
	DECLARE_BITMAP(irq_info_bitmap, IRQ_INFO_MAX);
	struct vgt_event_info	events[EVENT_MAX];
	DECLARE_BITMAP(pending_events, EVENT_MAX);
	struct vgt_emul_timer dpy_timer;
	u32  pipe_mask;
	bool installed;
	struct vgt_irq_map *irq_map;
};

#define vgt_get_event_phys_handler(h, e)	(h->events[e].p_handler)
#define vgt_get_event_virt_handler(h, e)	(h->events[e].v_handler)
#define vgt_set_event_val(h, e, v)	(h->events[e].state.val = v)
#define vgt_get_event_val(h, e)		(h->events[e].state.val)
#define vgt_get_event_policy(h, e)	(h->events[e].policy)
#define vgt_get_irq_info(h, e)		(h->events[e].info)
#define vgt_get_irq_ops(p)		(p->irq_hstate->ops)

/* common offset among interrupt control registers */
#define regbase_to_isr(base)	(base)
#define regbase_to_imr(base)	(base + 0x4)
#define regbase_to_iir(base)	(base + 0x8)
#define regbase_to_ier(base)	(base + 0xC)

#define iir_to_regbase(iir)    (iir - 0x8)
#define ier_to_regbase(ier)    (ier - 0xC)

#endif /* _VGT_INTERRUPT_H_ */
