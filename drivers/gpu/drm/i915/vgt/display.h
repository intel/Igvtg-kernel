/*
 * vGT display header
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

#ifndef _VGT_DISPLAY_H_
#define _VGT_DISPLAY_H_

enum vgt_uevent_type;

#define VGT_MAX_UEVENT_VARS 20

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

#define SBI_REG_MAX	20

#define VGT_PORT_NAME(p)	\
	((p) == PORT_A ? "PORT_A" : \
	((p) == PORT_B ? "PORT_B" : \
	((p) == PORT_C ? "PORT_C" : \
	((p) == PORT_D ? "PORT_D" : \
	((p) == PORT_E ? "PORT_E" : "PORT_X")))))

#define VGT_PIPE_NAME(p)	\
	((p) == PIPE_A ? "Pipe A" : \
		((p) == PIPE_B ? "Pipe B" : \
			((p) == PIPE_C ? "Pipe C" : "PIPE X")))
#define VGT_PIPE_CHAR(p)	\
	((p) == PIPE_A ? 'A' : \
		((p) == PIPE_B ? 'B' : \
			((p) == PIPE_C ? 'C' : 'X')))

#define VGT_PORT_TYPE_NAME(p)	\
        ((p) == VGT_CRT ? "VGT_CRT" : \
        ((p) == VGT_DP_A ? "VGT_DP_A" : \
        ((p) == VGT_DP_B ? "VGT_DP_B" : \
        ((p) == VGT_DP_C ? "VGT_DP_C" : \
	((p) == VGT_DP_D ? "VGT_DP_D" : \
	((p) == VGT_HDMI_B ? "VGT_HDMI_B" : \
	((p) == VGT_HDMI_C ? "VGT_HDMI_C" : \
	((p) == VGT_HDMI_D ? "VGT_HDMI_D" : "UNKNOWN"))))))))

#define HOTPLUG_VMID_FOR_ALL_VMS	0xff

#define VGT_VBLANK_TIMEOUT	50	/* in ms */

#define vgt_has_edp_enabled(vgt, pipe)							\
		(vgt && ((pipe) >= PIPE_A) && ((pipe) < I915_MAX_PIPES) &&		\
		(__vreg((vgt), _REG_PIPE_EDP_CONF) & _REGBIT_PIPE_ENABLE) &&		\
		(pipe == get_edp_input(__vreg(vgt, TRANS_DDI_FUNC_CTL_EDP))))
#define vgt_has_pipe_enabled(vgt, pipe)				\
		(vgt && ((pipe) >= PIPE_A) && ((pipe) < I915_MAX_PIPES) &&	\
		((__vreg((vgt), VGT_PIPECONF(pipe)) & _REGBIT_PIPE_ENABLE) ||	\
			vgt_has_edp_enabled(vgt, pipe)))
#define pdev_has_pipe_enabled(pdev, pipe)					\
		(pdev && ((pipe) >= PIPE_A) && ((pipe) < I915_MAX_PIPES) &&	\
		((__vreg(current_display_owner(pdev),				\
			VGT_PIPECONF(pipe)) & _REGBIT_PIPE_ENABLE) ||		\
			vgt_has_edp_enabled(current_display_owner(pdev), pipe)))
#define dpy_is_valid_port(port)							\
		(((port) >= PORT_A) && ((port) < I915_MAX_PORTS))

#define dpy_has_monitor_on_port(vgt, port)					\
		(vgt && dpy_is_valid_port(port) &&				\
		vgt->ports[port].edid && vgt->ports[port].edid->data_valid)

#define dpy_port_is_dp(vgt, port)						\
		((vgt) && dpy_is_valid_port(port)				\
		&& ((vgt->ports[port].type == VGT_DP_A) ||			\
		    (vgt->ports[port].type == VGT_DP_B) ||			\
		    (vgt->ports[port].type == VGT_DP_C) ||			\
		    (vgt->ports[port].type == VGT_DP_D)))

enum vgt_plane_type {
	PRIMARY_PLANE = 0,
	CURSOR_PLANE,
	SPRITE_PLANE,
	MAX_PLANE
};

enum vgt_port_type {
	VGT_CRT = 0,
	VGT_DP_A,
	VGT_DP_B,
	VGT_DP_C,
	VGT_DP_D,
	VGT_HDMI_B,
	VGT_HDMI_C,
	VGT_HDMI_D,
	VGT_PORT_MAX
};

enum vgt_output_type {
	VGT_OUTPUT_ANALOG = 0,
	VGT_OUTPUT_DISPLAYPORT,
	VGT_OUTPUT_EDP,
	VGT_OUTPUT_LVDS,
	VGT_OUTPUT_HDMI,
	VGT_OUTPUT_MAX
};

/* PLUG_OUT must equal to PLUG_IN + 1
 * hot plug handler code has such assumption. Actually it might
 * be OK to send HOTPLUG only, not necessarily differ IN aond
 * OUT.
 */
enum vgt_uevent_type {
	CRT_HOTPLUG_IN = 0,
	CRT_HOTPLUG_OUT,
	PORT_A_HOTPLUG_IN,
	PORT_A_HOTPLUG_OUT,
	PORT_B_HOTPLUG_IN,
	PORT_B_HOTPLUG_OUT,
	PORT_C_HOTPLUG_IN,
	PORT_C_HOTPLUG_OUT,
	PORT_D_HOTPLUG_IN,
	PORT_D_HOTPLUG_OUT,
	VGT_ENABLE_VGA,
	VGT_DISABLE_VGA,
	VGT_DISPLAY_READY,
	VGT_DISPLAY_UNREADY,
	VGT_DETECT_PORT_A,
	VGT_DETECT_PORT_B,
	VGT_DETECT_PORT_C,
	VGT_DETECT_PORT_D,
	VGT_DETECT_PORT_E,
	UEVENT_MAX
};

typedef union {
	uint32_t cmd;
	struct {
		uint32_t action : 1;
		uint32_t port_sel: 3;
		uint32_t rsvd_4_7 : 4;
		uint32_t vmid : 8;
		uint32_t rsvd_16_31 : 16;
	};
} vgt_hotplug_cmd_t;

struct vgt_uevent_info {
	char *uevent_name;
	int vm_id;
	enum kobject_action action;
	char *env_var_table[VGT_MAX_UEVENT_VARS];
	bool (*vgt_uevent_handler)(enum vgt_uevent_type event,
				struct vgt_uevent_info *uevent_entry,
				struct pgt_device *dev);
};

struct hotplug_work {
	struct work_struct work;
	DECLARE_BITMAP(hotplug_uevent, UEVENT_MAX);
	struct mutex hpd_mutex;
};

struct vgt_dpcd_data {
	bool data_valid;
	u8 data[DPCD_SIZE];
};

struct sbi_register {
	unsigned int offset;
	vgt_reg_t value;
};

struct sbi_registers {
	int number;
	struct sbi_register registers[SBI_REG_MAX];
};

struct port_cache {
	bool valid;
	struct vgt_edid_data_t	*edid;	/* per display EDID information */
	enum port		port_override;
	enum vgt_port_type	type;
};

struct gt_port {
	struct kobject  	kobj;

	struct vgt_edid_data_t	*edid;	/* per display EDID information */
	struct vgt_dpcd_data	*dpcd;	/* per display DPCD information */
	enum vgt_port_type	type;
	enum port		port_override;
	struct port_cache	cache; /* the temporary updated information */
	enum port physcal_port;
};

struct vgt_port_output_struct {
	unsigned int ctrl_reg;
	vgt_reg_t enable_bitmask;
	vgt_reg_t select_bitmask;
	enum vgt_output_type output_type;
};

static inline int port_to_port_type(int port_sel)
{
        switch(port_sel) {
        case PORT_A:
                return VGT_DP_A;
        case PORT_B:
                return VGT_DP_B;
        case PORT_C:
                return VGT_DP_C;
        case PORT_D:
                return VGT_DP_D;
        case PORT_E:
                return VGT_CRT;
	}
        return VGT_PORT_MAX;
}

static inline int port_type_to_port(int port_sel)
{
	switch(port_sel) {
	case VGT_DP_A:
		return PORT_A;
	case VGT_DP_B:
	case VGT_HDMI_B:
		return PORT_B;
	case VGT_DP_C:
	case VGT_HDMI_C:
		return PORT_C;
	case VGT_DP_D:
	case VGT_HDMI_D:
		return PORT_D;
	case VGT_CRT:
		return PORT_E;
	}

	return I915_MAX_PORTS;
}

int prepare_for_display_switch(struct pgt_device *pdev);

void do_vgt_fast_display_switch(struct pgt_device *pdev);

void vgt_trigger_display_hot_plug(struct pgt_device *dev, vgt_hotplug_cmd_t hotplug_cmd);

void vgt_set_uevent(struct vgt_device *vgt, enum vgt_uevent_type uevent);

void vgt_signal_uevent(struct pgt_device *dev);

void vgt_hotplug_udev_notify_func(struct work_struct *work);

void vgt_update_monitor_status(struct vgt_device *vgt);

bool rebuild_pipe_mapping(struct vgt_device *vgt, unsigned int reg, uint32_t new_data, uint32_t old_data);

bool update_pipe_mapping(struct vgt_device *vgt, unsigned int physical_reg, uint32_t physical_wr_data);

bool set_panel_fitting(struct vgt_device *vgt, enum pipe pipe);

bool vgt_manage_emul_dpy_events(struct pgt_device *pdev);

void vgt_update_frmcount(struct vgt_device *vgt, enum pipe pipe);

void vgt_calculate_frmcount_delta(struct vgt_device *vgt, enum pipe pipe);

void vgt_set_power_well(struct vgt_device *vgt, bool enable);

void vgt_flush_port_info(struct vgt_device *vgt, struct gt_port *port);

void vgt_detect_display(struct vgt_device *vgt, int index);

void vgt_dpy_init_modes(vgt_reg_t *mmio_array);
#endif /*_VGT_DISPLAY_H_*/
