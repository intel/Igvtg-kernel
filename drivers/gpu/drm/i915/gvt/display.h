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

#ifndef _GVT_DISPLAY_H_
#define _GVT_DISPLAY_H_

#define SBI_REG_MAX	20
#define DPCD_SIZE	0x700

#define dpy_is_valid_port(port)							\
		(((port) >= PORT_A) && ((port) < I915_MAX_PORTS))

#define gvt_vport(vgt, port) \
	(&(vgt)->state.display.ports[port])

#define dpy_has_monitor_on_port(vgt, port)					\
		(vgt && dpy_is_valid_port(port) &&				\
		gvt_vport(vgt, port)->edid && gvt_vport(vgt, port)->edid->data_valid)

#define dpy_port_is_dp(vgt, port)						\
		((vgt) && dpy_is_valid_port(port)				\
		&& ((gvt_vport(vgt, port)->type == GVT_DP_A) ||			\
		    (gvt_vport(vgt, port)->type == GVT_DP_B) ||			\
		    (gvt_vport(vgt, port)->type == GVT_DP_C) ||			\
		    (gvt_vport(vgt, port)->type == GVT_DP_D)))

#define GVT_MAX_UEVENT_VARS	3
/* DPCD start */
#define DPCD_SIZE	0x700

/* DPCD addresses */
#define DPCD_REV			0x000
#define DPCD_MAX_LINK_RATE		0x001
#define DPCD_MAX_LANE_COUNT		0x002

#define DPCD_TRAINING_PATTERN_SET	0x102
#define	DPCD_SINK_COUNT			0x200
#define DPCD_LANE0_1_STATUS		0x202
#define DPCD_LANE2_3_STATUS		0x203
#define DPCD_LANE_ALIGN_STATUS_UPDATED	0x204
#define DPCD_SINK_STATUS		0x205

/* link training */
#define DPCD_TRAINING_PATTERN_SET_MASK	0x03
#define DPCD_LINK_TRAINING_DISABLED	0x00
#define DPCD_TRAINING_PATTERN_1		0x01
#define DPCD_TRAINING_PATTERN_2		0x02

#define DPCD_CP_READY_MASK		(1 << 6)

/* lane status */
#define DPCD_LANES_CR_DONE		0x11
#define DPCD_LANES_EQ_DONE		0x22
#define DPCD_SYMBOL_LOCKED		0x44

#define DPCD_INTERLANE_ALIGN_DONE	0x01

#define DPCD_SINK_IN_SYNC		0x03

/* DPCD end */

struct sbi_register {
	unsigned int offset;
	u32 value;
};

struct sbi_registers {
	int number;
	struct sbi_register registers[SBI_REG_MAX];
};

enum gvt_plane_type {
	PRIMARY_PLANE = 0,
	CURSOR_PLANE,
	SPRITE_PLANE,
	MAX_PLANE
};

struct gvt_dpcd_data {
	bool data_valid;
	u8 data[DPCD_SIZE];
};

enum gvt_port_type {
	GVT_CRT = 0,
	GVT_DP_A,
	GVT_DP_B,
	GVT_DP_C,
	GVT_DP_D,
	GVT_HDMI_B,
	GVT_HDMI_C,
	GVT_HDMI_D,
	GVT_PORT_MAX
};

struct gt_port {
	struct gvt_edid_data_t	*edid;	/* per display EDID information */
	struct gvt_dpcd_data	*dpcd;	/* per display DPCD information */
	enum gvt_port_type	type;
};

extern int gvt_get_edp_pipe(struct vgt_device *vgt);
extern bool gvt_edp_pipe_is_enabled(struct vgt_device *vgt);
extern bool gvt_pipe_is_enabled(struct vgt_device *vgt, int pipe);

bool gvt_init_virtual_display_state(struct vgt_device *vgt);
void gvt_clean_virtual_display_state(struct vgt_device *vgt);

#endif
