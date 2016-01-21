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

int gvt_get_edp_pipe(struct vgt_device *vgt)
{
	u32 data = __vreg(vgt, _TRANS_DDI_FUNC_CTL_EDP);
	int pipe = I915_MAX_PIPES;

	switch (data & TRANS_DDI_EDP_INPUT_MASK) {
		case TRANS_DDI_EDP_INPUT_A_ON:
		case TRANS_DDI_EDP_INPUT_A_ONOFF:
			pipe = PIPE_A;
			break;
		case TRANS_DDI_EDP_INPUT_B_ONOFF:
			pipe = PIPE_B;
			break;
		case TRANS_DDI_EDP_INPUT_C_ONOFF:
			pipe = PIPE_C;
			break;
	}
	return pipe;
}

bool gvt_edp_pipe_is_enabled(struct vgt_device *vgt)
{
	if (!(__vreg(vgt, _REG_PIPE_EDP_CONF) & PIPECONF_ENABLE))
		return false;

	if (!(__vreg(vgt, _TRANS_DDI_FUNC_CTL_EDP) & TRANS_DDI_FUNC_ENABLE))
		return false;

	return true;
}

bool gvt_pipe_is_enabled(struct vgt_device *vgt, int pipe)
{
	ASSERT(pipe >= PIPE_A && pipe < I915_MAX_PIPES);

	if (__vreg(vgt, GVT_PIPECONF(pipe)) & PIPECONF_ENABLE)
		return true;

	if (gvt_edp_pipe_is_enabled(vgt) &&
			gvt_get_edp_pipe(vgt) == pipe)
		return true;

	return false;
}

static const unsigned char virtual_dp_monitor_edid[] = {
	0x00,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x22,0xf0,0x54,0x29,
	0x00,0x00,0x00,0x00,0x04,0x17,0x01,0x04,0xa5,0x34,0x20,0x78,
	0x23,0xfc,0x81,0xa4,0x55,0x4d,0x9d,0x25,0x12,0x50,0x54,0x21,
	0x08,0x00,0xd1,0xc0,0x81,0xc0,0x81,0x40,0x81,0x80,0x95,0x00,
	0xa9,0x40,0xb3,0x00,0x01,0x01,0x28,0x3c,0x80,0xa0,0x70,0xb0,
	0x23,0x40,0x30,0x20,0x36,0x00,0x06,0x44,0x21,0x00,0x00,0x1a,
	0x00,0x00,0x00,0xfd,0x00,0x18,0x3c,0x18,0x50,0x11,0x00,0x0a,
	0x20,0x20,0x20,0x20,0x20,0x20,0x00,0x00,0x00,0xfc,0x00,0x48,
	0x50,0x20,0x5a,0x52,0x32,0x34,0x34,0x30,0x77,0x0a,0x20,0x20,
	0x00,0x00,0x00,0xff,0x00,0x43,0x4e,0x34,0x33,0x30,0x34,0x30,
	0x44,0x58,0x51,0x0a,0x20,0x20,0x01,0x44,0x02,0x03,0x19,0xc1,
	0x4c,0x90,0x1f,0x05,0x14,0x04,0x13,0x03,0x02,0x07,0x06,0x12,
	0x01,0x23,0x09,0x07,0x07,0x83,0x01,0x00,0x00,0x02,0x3a,0x80,
	0x18,0x71,0x38,0x2d,0x40,0x58,0x2c,0x45,0x00,0x06,0x44,0x21,
	0x00,0x00,0x1e,0x02,0x3a,0x80,0xd0,0x72,0x38,0x2d,0x40,0x10,
	0x2c,0x45,0x80,0x06,0x44,0x21,0x00,0x00,0x1e,0x01,0x1d,0x00,
	0x72,0x51,0xd0,0x1e,0x20,0x6e,0x28,0x55,0x00,0x06,0x44,0x21,
	0x00,0x00,0x1e,0x01,0x1d,0x00,0xbc,0x52,0xd0,0x1e,0x20,0xb8,
	0x28,0x55,0x40,0x06,0x44,0x21,0x00,0x00,0x1e,0x8c,0x0a,0xd0,
	0x8a,0x20,0xe0,0x2d,0x10,0x10,0x3e,0x96,0x00,0x06,0x44,0x21,
	0x00,0x00,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x7b,
};

#define DPCD_HEADER_SIZE        0xb

u8 dpcd_fix_data[DPCD_HEADER_SIZE] = {
        0x11, 0x0a, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static void emulate_monitor_status_change(struct vgt_device *vgt)
{
	__vreg(vgt, _SDEISR) &= ~(_REGBIT_DP_B_HOTPLUG |
			_REGBIT_DP_C_HOTPLUG |
			_REGBIT_DP_D_HOTPLUG);

	if (dpy_has_monitor_on_port(vgt, PORT_B))
		__vreg(vgt, _SDEISR) |= _REGBIT_DP_B_HOTPLUG;

	if (dpy_has_monitor_on_port(vgt, PORT_C))
		__vreg(vgt, _SDEISR) |= _REGBIT_DP_C_HOTPLUG;

	if (dpy_has_monitor_on_port(vgt, PORT_D))
		__vreg(vgt, _SDEISR) |= _REGBIT_DP_D_HOTPLUG;

	if (dpy_has_monitor_on_port(vgt, PORT_A))
		__vreg(vgt, _GEN8_DE_PORT_ISR) |= GEN8_PORT_DP_A_HOTPLUG;
}

static void clean_virtual_dp_monitor(struct vgt_device *vgt)
{
	struct gt_port *port = gvt_vport(vgt, PORT_A);

	if (port->edid) {
		kfree(port->edid);
		port->edid = NULL;
	}

	if (port->dpcd) {
		kfree(port->dpcd);
		port->dpcd = NULL;
	}
}

static bool setup_virtual_dp_monitor(struct vgt_device *vgt)
{
	struct gt_port *port = gvt_vport(vgt, PORT_A);

	port->edid = kzalloc(sizeof(*(port->edid)), GFP_KERNEL);
	if (!port->edid)
		goto err;

	port->dpcd = kzalloc(sizeof(*(port->dpcd)), GFP_KERNEL);
	if (!port->dpcd)
		goto err;

	memcpy(port->edid->edid_block, virtual_dp_monitor_edid,
			EDID_SIZE);
	port->edid->data_valid = true;

	memcpy(port->dpcd->data, dpcd_fix_data, DPCD_HEADER_SIZE);
	port->dpcd->data_valid = true;

	port->type = GVT_DP_A;

	emulate_monitor_status_change(vgt);
	return true;
err:
	clean_virtual_dp_monitor(vgt);
	return false;
}

bool gvt_update_display_events_emulation(struct pgt_device *pdev)
{
	struct gvt_irq_state *irq = &pdev->irq_state;
	struct vgt_device *vgt;
	bool have_enabled_pipe = false;
	int pipe, id;

	ASSERT(mutex_is_locked(&pdev->lock));

	hrtimer_cancel(&irq->dpy_timer.timer);

	for_each_online_instance(pdev, vgt, id) {
		for (pipe = 0; pipe < I915_MAX_PIPES; pipe++) {
			have_enabled_pipe =
				gvt_pipe_is_enabled(vgt, pipe);
			if (have_enabled_pipe)
				break;
		}
	}

	if (have_enabled_pipe)
		hrtimer_start(&irq->dpy_timer.timer,
				ktime_add_ns(ktime_get(), irq->dpy_timer.period),
				HRTIMER_MODE_ABS);
	return true;
}

static void emulate_vblank_on_pipe(struct vgt_device *vgt, int pipe)
{
	int vblank_event[] = {
		[PIPE_A] = PIPE_A_VBLANK,
		[PIPE_B] = PIPE_B_VBLANK,
		[PIPE_C] = PIPE_C_VBLANK,
	};

	ASSERT(pipe >= PIPE_A && pipe <= PIPE_C);

	if (gvt_pipe_is_enabled(vgt, pipe))
		gvt_trigger_virtual_event(vgt, vblank_event[pipe]);
}

static void emulate_vblank_for_instance(struct vgt_device *vgt)
{
	int pipe;

	for (pipe = 0; pipe < I915_MAX_PIPES; pipe++)
		emulate_vblank_on_pipe(vgt, pipe);
}

void gvt_emulate_display_events(struct pgt_device *pdev)
{
	struct vgt_device *vgt;
	int id;

	ASSERT(mutex_is_locked(&pdev->lock));

	for_each_online_instance(pdev, vgt, id)
		emulate_vblank_for_instance(vgt);
}

void gvt_clean_virtual_display_state(struct vgt_device *vgt)
{
	clean_virtual_dp_monitor(vgt);
}

bool gvt_init_virtual_display_state(struct vgt_device *vgt)
{
	gvt_init_i2c_edid(vgt);
	return setup_virtual_dp_monitor(vgt);
}
