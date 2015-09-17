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

#ifndef _GVT_INTERRUPT_H_
#define _GVT_INTERRUPT_H_

enum gvt_event_type {
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

	PCU_THERMAL,
	PCU_PCODE2DRIVER_MAILBOX,

	DPST_PHASE_IN,	// This is an active high pulse on the DPST phase in event
	DPST_HISTOGRAM,	// This is an active high pulse on the AUX A done event.
	GSE,
	DP_A_HOTPLUG,
	AUX_CHANNEL_A,	// This is an active high pulse on the AUX A done event.
	PERF_COUNTER,	// This is an active high pulse when the performance counter reaches the threshold value programmed in the Performance Counter Source register
	POISON,		// This is an active high pulse on receiving the poison message
	GTT_FAULT,	// This is an active high level while either of the GTT Fault Status register bits are set
	ERROR_INTERRUPT_COMBINED,

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
	AUX_CHENNEL_B,
	AUX_CHENNEL_C,
	AUX_CHENNEL_D,
	AUDIO_POWER_STATE_CHANGE_B,
	AUDIO_POWER_STATE_CHANGE_C,
	AUDIO_POWER_STATE_CHANGE_D,

	GVT_EVENT_RESERVED,
	GVT_EVENT_MAX,
};

struct gvt_irq_state;
struct pgt_device;

typedef void (*gvt_event_phys_handler_t)(struct gvt_irq_state *hstate,
	enum gvt_event_type event);
typedef void (*gvt_event_virt_handler_t)(struct gvt_irq_state *hstate,
	enum gvt_event_type event, struct vgt_device *vgt);

struct gvt_irq_ops {
	void (*init_irq) (struct gvt_irq_state *hstate);
	void (*check_pending_irq) (struct vgt_device *vgt);
};

/* the list of physical interrupt control register groups */
enum gvt_irq_type {
	GVT_IRQ_INFO_GT,
	GVT_IRQ_INFO_DPY,
	GVT_IRQ_INFO_PCH,
	GVT_IRQ_INFO_PM,

	GVT_IRQ_INFO_MASTER,
	GVT_IRQ_INFO_GT0,
	GVT_IRQ_INFO_GT1,
	GVT_IRQ_INFO_GT2,
	GVT_IRQ_INFO_GT3,
	GVT_IRQ_INFO_DE_PIPE_A,
	GVT_IRQ_INFO_DE_PIPE_B,
	GVT_IRQ_INFO_DE_PIPE_C,
	GVT_IRQ_INFO_DE_PORT,
	GVT_IRQ_INFO_DE_MISC,
	GVT_IRQ_INFO_AUD,
	GVT_IRQ_INFO_PCU,

	GVT_IRQ_INFO_MAX,
};

#define GVT_IRQ_BITWIDTH	32

/* device specific interrupt bit definitions */
struct gvt_irq_info {
	char *name;
	int reg_base;
	enum gvt_event_type bit_to_event[GVT_IRQ_BITWIDTH];
	unsigned long warned;
	int group;
	DECLARE_BITMAP(downstream_irq_bitmap, GVT_IRQ_BITWIDTH);
	bool has_upstream_irq;
};

#define	GVT_EVENT_FW_ALL 0	/* event forwarded to all instances */
#define	GVT_EVENT_FW_NONE 1	/* no forward */

/* per-event information */
struct gvt_event_info {
	/* device specific info */
	int			bit;	/* map to register bit */
	struct gvt_irq_info	*info;	/* register info */

	/* device neutral info */
	int			policy;	/* forwarding policy */
	gvt_event_phys_handler_t	p_handler;	/* for p_event */
	gvt_event_virt_handler_t	v_handler;	/* for v_event */
};

struct gvt_irq_map {
	int up_irq_group;
	int up_irq_bit;
	int down_irq_group;
	u32 down_irq_bitmask;
};

/* structure containing device specific IRQ state */
struct gvt_irq_state {
	struct gvt_irq_ops *ops;
	struct gvt_irq_info	*info[GVT_IRQ_INFO_MAX];
	DECLARE_BITMAP(irq_info_bitmap, GVT_IRQ_INFO_MAX);
	struct gvt_event_info	events[GVT_EVENT_MAX];
	DECLARE_BITMAP(pending_events, GVT_EVENT_MAX);
	struct gvt_irq_map *irq_map;
};

#define gvt_get_event_virt_handler(h, e)	(h->events[e].v_handler)
#define gvt_get_event_policy(h, e)		(h->events[e].policy)
#define gvt_get_irq_info(h, e)			(h->events[e].info)
#define gvt_get_irq_ops(p)			(p->irq_state.ops)

/* common offset among interrupt control registers */
#define regbase_to_isr(base)	(base)
#define regbase_to_imr(base)	(base + 0x4)
#define regbase_to_iir(base)	(base + 0x8)
#define regbase_to_ier(base)	(base + 0xC)

#define iir_to_regbase(iir)    (iir - 0x8)
#define ier_to_regbase(ier)    (ier - 0xC)

#define gvt_irq_state_to_pdev(state) \
	container_of(state, struct pgt_device, irq_state)

bool gvt_irq_init(struct pgt_device *pdev);
void gvt_irq_exit(struct pgt_device *pdev);

#endif /* _GVT_INTERRUPT_H_ */
