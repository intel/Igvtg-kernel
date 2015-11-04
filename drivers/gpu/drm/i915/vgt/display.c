/*
 * Display context switch
 *
 * Copyright 2008 (c) Intel Corporation
 *   Jesse Barnes <jbarnes@virtuousgeek.org>
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

#include <linux/slab.h>
#include <linux/delay.h>
#include "vgt.h"

static void vgt_restore_sreg(struct vgt_device *vgt,unsigned int reg)
{
	unsigned int real_reg;
	if(vgt_map_plane_reg(vgt, reg, &real_reg))
	{
		VGT_MMIO_WRITE(vgt->pdev, real_reg, __sreg(vgt, (reg)));
	}

}

static int vgt_restore_state(struct vgt_device *vgt, enum vgt_pipe pipe)
{
#if 0
	unsigned int pipe_ctrl = VGT_MMIO_READ(vgt->pdev, VGT_PIPECONF(pipe));
	if (pipe_ctrl & _REGBIT_PIPE_ENABLE) {
#endif
		vgt_dbg (VGT_DBG_DPY, "start to restore pipe %d.\n", pipe + 1);
		vgt_restore_sreg(vgt, VGT_DSPCNTR(pipe));
		vgt_restore_sreg(vgt, VGT_DSPSTRIDE(pipe));
		vgt_restore_sreg(vgt, VGT_DSPSURF(pipe));
		vgt_restore_sreg(vgt, VGT_DSPTILEOFF(pipe));
		vgt_restore_sreg(vgt, VGT_DSPLINOFF(pipe));

		vgt_restore_sreg(vgt, VGT_CURPOS(pipe));
		vgt_restore_sreg(vgt, VGT_CURCNTR(pipe));
		vgt_restore_sreg(vgt, VGT_CURBASE(pipe));
		vgt_dbg (VGT_DBG_DPY, "finished pipe %d restore.\n", pipe + 1);
#if 0
	} else {
		vgt_dbg (VGT_DBG_DPY, "pipe %d is not enabled.\n", pipe + 1);
	}
#endif
	return 0;
}

static int wait_for_vblank_atomic(struct pgt_device *pdev, enum vgt_pipe pipe)
{
	int ret;
	unsigned int frmcnt_mmio = VGT_PIPE_FRMCOUNT(pipe);
	vgt_reg_t frmcnt = VGT_MMIO_READ(pdev, frmcnt_mmio);

	ret = wait_for_atomic((VGT_MMIO_READ(pdev, frmcnt_mmio) != frmcnt),
				VGT_VBLANK_TIMEOUT);
	if (ret == -ETIMEDOUT) {
		vgt_warn("pipe-%d: Timeout for waiting vblank!\n", pipe);
	}
	return ret;
}

static int wait_for_vblanks_atomic(struct pgt_device *pdev)
{
	int ret = 0;
	enum vgt_pipe pipe;

	for (pipe = PIPE_A; (pipe < I915_MAX_PIPES) && !ret; ++ pipe) {
		vgt_reg_t pipeconf = VGT_MMIO_READ(pdev, VGT_PIPECONF(pipe));
		if (pipeconf & _REGBIT_PIPE_ENABLE) {
			ret = wait_for_vblank_atomic(pdev, pipe);
		}
	}
	return ret;
}

int prepare_for_display_switch(struct pgt_device *pdev)
{
	int ret = 0;

	if (!(idle_render_engine(pdev, RING_BUFFER_RCS) &&
		idle_render_engine(pdev, RING_BUFFER_BCS))) {
		vgt_warn("vGT: Ring RCS or Ring BCS is busy "
			"in display switch!\n");
		ret = -1;
	}

	if (!ret) {
		ret = wait_for_vblanks_atomic(pdev);
		if (ret)
			vgt_warn("Failed to get vblank in display switch!\n");
	}

	return ret;
}

/*
 * Do foreground vm switch.
 */
void do_vgt_fast_display_switch(struct pgt_device *pdev)
{
	struct vgt_device *to_vgt = pdev->next_foreground_vm;
	enum vgt_pipe pipe;

	vgt_dbg(VGT_DBG_DPY, "vGT: doing display switch: from %p to %p\n",
			current_foreground_vm(pdev), to_vgt);

	ASSERT(fastpath_dpy_switch);
	ASSERT(spin_is_locked(&pdev->lock));

	for (pipe = PIPE_A; pipe < I915_MAX_PIPES; ++ pipe) {
		vgt_restore_state(to_vgt, pipe);
		if (_PRI_PLANE_ENABLE & __vreg(to_vgt, VGT_DSPCNTR(pipe))) {
			set_panel_fitting(to_vgt, pipe);
		}
	}

	current_foreground_vm(pdev) = to_vgt;
}

static inline int get_event_and_edid_info(vgt_hotplug_cmd_t cmd,
				enum vgt_event_type *pevent,
				enum vgt_port_type *pedid_idx)
{
	int ret = 0;

	switch(cmd.port_sel) {
	case 0:
		*pedid_idx = PORT_E;
		*pevent = CRT_HOTPLUG;
		break;
	case 1:
		*pedid_idx = I915_MAX_PORTS;
		*pevent = EVENT_MAX;
		printk("vGT: No support for hot plug type: DP_A!\n");
		ret = -EINVAL;
		break;
	case 2:
		*pedid_idx = PORT_B;
		*pevent = DP_B_HOTPLUG;
		break;
	case 3:
		*pedid_idx = PORT_C;
		*pevent = DP_C_HOTPLUG;
		break;
	case 4:
		*pedid_idx = PORT_D;
		*pevent = DP_D_HOTPLUG;
		break;
	default:
		*pedid_idx = I915_MAX_PORTS;
		*pevent = EVENT_MAX;
		printk("vGT: Not supported hot plug type: 0x%x!\n",
			cmd.port_sel);
		ret = -EINVAL;
		break;
	}
	return ret;
}

void vgt_trigger_display_hot_plug(struct pgt_device *dev,
		vgt_hotplug_cmd_t  hotplug_cmd)
{
	int i;
	enum vgt_event_type event = EVENT_MAX;
	enum vgt_port_type port_idx = VGT_PORT_MAX;
	int cpu;

	if (get_event_and_edid_info(hotplug_cmd, &event, &port_idx) < 0)
		return;

	vgt_lock_dev(dev, cpu);
	for (i = 0; i < VGT_MAX_VMS; ++ i) {
		struct vgt_device *vgt = dev->device[i];

		if (!vgt)
			continue;

		if (hotplug_cmd.vmid != HOTPLUG_VMID_FOR_ALL_VMS) {
			if (vgt != vmid_2_vgt_device(hotplug_cmd.vmid))
				continue;
		}

		if (is_current_display_owner(vgt)) {
			continue;
		}

		if (hotplug_cmd.action != 0x1) {
			/* pull out */
			vgt_clear_port(vgt, port_idx);
		}

		vgt_update_monitor_status(vgt);
		vgt_trigger_virtual_event(vgt, event);
	}

	vgt_unlock_dev(dev, cpu);
	return;
}

DECLARE_BITMAP(vgt_uevents_bitmap, UEVENT_MAX);
extern struct kobject *vgt_ctrl_kobj;

bool vgt_default_uevent_handler(struct vgt_uevent_info *uevent_entry, struct pgt_device *pdev)
{
	int retval;
	retval = kobject_uevent_env(vgt_ctrl_kobj, uevent_entry->action, uevent_entry->env_var_table);
	if (retval == 0)
		return true;
	else
		return false;
}

bool vgt_hotplug_uevent_handler(enum vgt_uevent_type event,
			struct vgt_uevent_info *uevent_entry,
			struct pgt_device *pdev)
{
	return vgt_default_uevent_handler(uevent_entry, pdev);
}

static bool vgt_dpy_stat_uevent_handler(enum vgt_uevent_type event,
			struct vgt_uevent_info *uevent_entry,
			struct pgt_device *pdev)
{
	/* Add vmid */
	int retval;
	char vmid_str[20];
	retval = snprintf(vmid_str, 20, "VMID=%d", uevent_entry->vm_id);
	uevent_entry->env_var_table[1] = vmid_str;
	return vgt_default_uevent_handler(uevent_entry, pdev);
}

static bool vgt_dpy_detect_uevent_handler(enum vgt_uevent_type event,
			struct vgt_uevent_info *uevent_entry,
			struct pgt_device *pdev)
{
	return vgt_default_uevent_handler(uevent_entry, pdev);
}

/*
 When you add new uevents or add new environmental variable,
 you should following rules:
 Now you can at most define VGT_MAX_UEVENT_VARS environmental
 variables with the form like "VAR=VALUE", all the
 pointer of string are stored in env_var_table (below).
struct vgt_uevent_info {
	...
	char *env_var_table[VGT_MAX_UEVENT_VARS];
	...
};
 You should place a NULL as the termination of variable
 definition, or function add_uevent_var() in line 219
 of lib/kobject_uevent.c will fail.
*/

static struct vgt_uevent_info vgt_default_uevent_info_table[UEVENT_MAX] = {
	{"CRT insert", -1, KOBJ_ADD, {"CRT_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"CRT remove", -1, KOBJ_REMOVE, {"CRT_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"PORT A insert", -1, KOBJ_ADD, {"PORT_A_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"PORT A remove", -1,KOBJ_REMOVE, {"PORT_A_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"PORT B insert", -1, KOBJ_ADD, {"PORT_B_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"PORT B remove", -1, KOBJ_REMOVE, {"PORT_B_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"PORT C insert", -1, KOBJ_ADD, {"PORT_C_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"PORT C remove", -1, KOBJ_REMOVE, {"PORT_C_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"PORT D insert", -1, KOBJ_ADD, {"PORT_D_INSERT=1", NULL}, vgt_hotplug_uevent_handler},
	{"PORT D remove", -1, KOBJ_REMOVE, {"PORT_D_REMOVE=1", NULL}, vgt_hotplug_uevent_handler},
	{"VGT enable VGA mode", -1, KOBJ_ADD, {"VGT_ENABLE_VGA=1", NULL, NULL}, vgt_dpy_stat_uevent_handler},
	{"VGT disable VGA mode", -1, KOBJ_ADD, {"VGT_ENABLE_VGA=0", NULL, NULL}, vgt_dpy_stat_uevent_handler},
	{"VGT display ready", -1, KOBJ_ADD, {"VGT_DISPLAY_READY=1", NULL, NULL}, vgt_dpy_stat_uevent_handler},
	{"VGT display unready", -1, KOBJ_ADD, {"VGT_DISPLAY_READY=0", NULL, NULL}, vgt_dpy_stat_uevent_handler},
	{"VGT detect PORT A", -1, KOBJ_ADD, {"VGT_DETECT_PORT_A=1", NULL, NULL}, vgt_dpy_detect_uevent_handler},
	{"VGT detect PORT B", -1, KOBJ_ADD, {"VGT_DETECT_PORT_B=1", NULL, NULL}, vgt_dpy_detect_uevent_handler},
	{"VGT detect PORT C", -1, KOBJ_ADD, {"VGT_DETECT_PORT_C=1", NULL, NULL}, vgt_dpy_detect_uevent_handler},
	{"VGT detect PORT D", -1, KOBJ_ADD, {"VGT_DETECT_PORT_D=1", NULL, NULL}, vgt_dpy_detect_uevent_handler},
	{"VGT detect PORT E", -1, KOBJ_ADD, {"VGT_DETECT_PORT_E=1", NULL, NULL}, vgt_dpy_detect_uevent_handler},
};

void vgt_set_uevent(struct vgt_device *vgt, enum vgt_uevent_type uevent)
{
	struct vgt_uevent_info *entry;

	ASSERT(uevent < UEVENT_MAX);

	entry = &vgt_default_uevent_info_table[uevent];
	entry->vm_id = vgt->vm_id;

	set_bit(uevent, vgt_uevents_bitmap);
}

void vgt_signal_uevent(struct pgt_device *pdev)
{
	struct vgt_uevent_info *info_entry;
	bool rc;
	int bit;

	for_each_set_bit(bit, vgt_uevents_bitmap, UEVENT_MAX) {
		clear_bit(bit, vgt_uevents_bitmap);

		info_entry = &vgt_default_uevent_info_table[bit];

		ASSERT(info_entry);
		ASSERT(info_entry->vgt_uevent_handler);

		rc = info_entry->vgt_uevent_handler(bit, info_entry, pdev);
		if (rc == false)
			printk("%s: %d: vGT: failed to send uevent [%s]!\n",
					__func__, __LINE__, info_entry->uevent_name);
	}
}

void vgt_hotplug_udev_notify_func(struct work_struct *work)
{
	struct hotplug_work *hpd_work = (struct hotplug_work *)work;
	struct pgt_device *pdev = container_of(hpd_work, struct pgt_device, hpd_work);
	int bit;

	mutex_lock(&hpd_work->hpd_mutex);
	for_each_set_bit(bit, hpd_work->hotplug_uevent, UEVENT_MAX) {
		struct vgt_uevent_info *info_entry;
		clear_bit(bit, hpd_work->hotplug_uevent);
		info_entry = &vgt_default_uevent_info_table[bit];
		vgt_default_uevent_handler(info_entry, pdev);
	}
	mutex_unlock(&hpd_work->hpd_mutex);
}

void vgt_update_monitor_status(struct vgt_device *vgt)
{
	if (is_current_display_owner(vgt))
		return;

	__vreg(vgt, _REG_SDEISR) &= ~(_REGBIT_DP_B_HOTPLUG |
					_REGBIT_DP_C_HOTPLUG |
					_REGBIT_DP_D_HOTPLUG);

	if (dpy_has_monitor_on_port(vgt, PORT_B)) {
		vgt_dbg(VGT_DBG_DPY, "enable B port monitor\n");
		__vreg(vgt, _REG_SDEISR) |= _REGBIT_DP_B_HOTPLUG;
	}
	if (dpy_has_monitor_on_port(vgt, PORT_C)) {
		vgt_dbg(VGT_DBG_DPY, "enable C port monitor\n");
		__vreg(vgt, _REG_SDEISR) |= _REGBIT_DP_C_HOTPLUG;
	}
	if (dpy_has_monitor_on_port(vgt, PORT_D)) {
		vgt_dbg(VGT_DBG_DPY, "enable D port monitor\n");
		__vreg(vgt, _REG_SDEISR) |= _REGBIT_DP_D_HOTPLUG;
	}
	if (dpy_has_monitor_on_port(vgt, PORT_A)) {
		__vreg(vgt, _REG_DDI_BUF_CTL_A) |= _DDI_BUFCTL_DETECT_MASK;
		if (IS_PREBDW(vgt->pdev))
			__vreg(vgt, _REG_DEISR) |= _REGBIT_DP_A_HOTPLUG_IVB;
		else
			__vreg(vgt, _REG_DE_PORT_ISR) |= _REGBIT_PORT_DP_A_HOTPLUG; 
	}
}

enum vgt_pipe get_edp_input(uint32_t wr_data)
{
	enum vgt_pipe pipe = I915_MAX_PIPES;

	if ((_REGBIT_TRANS_DDI_FUNC_ENABLE & wr_data) == 0) {
		return I915_MAX_PIPES;
	}

	switch (wr_data & _REGBIT_TRANS_DDI_EDP_INPUT_MASK) {
		case _REGBIT_TRANS_DDI_EDP_INPUT_A_ON:
		case _REGBIT_TRANS_DDI_EDP_INPUT_A_ONOFF:
			pipe = PIPE_A;
			break;
		case _REGBIT_TRANS_DDI_EDP_INPUT_B_ONOFF:
			pipe = PIPE_B;
			break;
		case _REGBIT_TRANS_DDI_EDP_INPUT_C_ONOFF:
			pipe = PIPE_C;
			break;
		default:
			pipe = I915_MAX_PIPES;
	}
	return pipe;
}

enum vgt_pipe get_pipe(unsigned int reg, uint32_t wr_data)
{
	enum vgt_pipe pipe = I915_MAX_PIPES;

	if (reg == _REG_TRANS_DDI_FUNC_CTL_A) {
		pipe = PIPE_A;
	}
	else if (reg == _REG_TRANS_DDI_FUNC_CTL_B) {
		pipe = PIPE_B;
	}
	else if (reg == _REG_TRANS_DDI_FUNC_CTL_C) {
		pipe = PIPE_C;
	}else if (reg == _REG_TRANS_DDI_FUNC_CTL_EDP) {
		pipe = get_edp_input (wr_data);
	}

	return pipe;
}

static void vgt_update_irq_reg(struct vgt_device *vgt)
{
	if (IS_PREBDW(vgt->pdev)) {
		recalculate_and_update_ier(vgt->pdev, _REG_DEIER);
		recalculate_and_update_imr(vgt->pdev, _REG_DEIMR);
	} else {
		recalculate_and_update_ier(vgt->pdev, _REG_DE_PIPE_IER(PIPE_A));
		recalculate_and_update_ier(vgt->pdev, _REG_DE_PIPE_IER(PIPE_B));
		recalculate_and_update_ier(vgt->pdev, _REG_DE_PIPE_IER(PIPE_C));

		recalculate_and_update_imr(vgt->pdev, _REG_DE_PIPE_IMR(PIPE_A));
		recalculate_and_update_imr(vgt->pdev, _REG_DE_PIPE_IMR(PIPE_B));
		recalculate_and_update_imr(vgt->pdev, _REG_DE_PIPE_IMR(PIPE_C));
	}

	return;
}

bool rebuild_pipe_mapping(struct vgt_device *vgt, unsigned int reg, uint32_t new_data, uint32_t old_data)
{
	vgt_reg_t hw_value;
	int i = 0;

	enum vgt_pipe virtual_pipe = I915_MAX_PIPES;
	enum vgt_pipe physical_pipe = I915_MAX_PIPES;

	if (vgt->vm_id == 0) {
		return true;
	}

	virtual_pipe = get_pipe(reg, new_data);

	/*disable pipe case*/
	if ((_REGBIT_TRANS_DDI_FUNC_ENABLE & new_data) == 0) {
		if (reg == _REG_TRANS_DDI_FUNC_CTL_EDP) {
			/*for disable case, we need to get edp input from old value
			since the new data does not contain the edp input*/
			virtual_pipe = get_edp_input(old_data);
		}
		if (virtual_pipe != I915_MAX_PIPES) {
			vgt_set_pipe_mapping(vgt, virtual_pipe, I915_MAX_PIPES);
			vgt_update_irq_reg(vgt);
			vgt_dbg(VGT_DBG_DPY, "vGT: delete pipe mapping %x\n", virtual_pipe);
			if (vgt_has_pipe_enabled(vgt, virtual_pipe))
				vgt_update_frmcount(vgt, virtual_pipe);
			vgt_calculate_frmcount_delta(vgt, virtual_pipe);
		}
		return true;
	}

	/*enable pipe case*/
	ASSERT((reg == _REG_TRANS_DDI_FUNC_CTL_EDP) ||
			(new_data & _REGBIT_TRANS_DDI_PORT_MASK));

	if (reg == _REG_TRANS_DDI_FUNC_CTL_EDP) {
		// In such case, it is virtual PORT_A mapping to physical PORT_A
		hw_value = VGT_MMIO_READ(vgt->pdev, _REG_TRANS_DDI_FUNC_CTL_EDP);
		if (_REGBIT_TRANS_DDI_FUNC_ENABLE & hw_value)
			physical_pipe = get_edp_input(hw_value);
	} else {
		enum vgt_port vport, vport_override;
		vport = (new_data & _REGBIT_TRANS_DDI_PORT_MASK) >> _TRANS_DDI_PORT_SHIFT;
		vport_override = vgt->ports[vport].port_override;
		if (vport_override == I915_MAX_PORTS) {
			vgt_warn("Unexpected driver behavior to enable TRANS_DDI"
					" for not ready port!!\n");
			physical_pipe = I915_MAX_PIPES;
		} else if (vport_override == PORT_A) {
			hw_value = VGT_MMIO_READ(vgt->pdev, _REG_TRANS_DDI_FUNC_CTL_EDP);
			if (_REGBIT_TRANS_DDI_FUNC_ENABLE & hw_value)
				physical_pipe = get_edp_input(hw_value);
								
		} else {
			for (i = 0; i <= TRANSCODER_C; i++) {
				enum vgt_port pport;
				hw_value = VGT_MMIO_READ(vgt->pdev, _VGT_TRANS_DDI_FUNC_CTL(i));
				pport = (hw_value & _REGBIT_TRANS_DDI_PORT_MASK) >>
						_TRANS_DDI_PORT_SHIFT;

				printk("%s: Enable. pport = %d, vport = %d, "
					"hw_value = 0x%08x, new_data = 0x%08x\n",
			       		__FUNCTION__, pport, vport, hw_value, new_data);

				if (!(_REGBIT_TRANS_DDI_FUNC_ENABLE & hw_value)) {
					continue;
				}

				if (vport_override == pport) {
					physical_pipe = i;
					break;
				}
			}
		}
	}

	ASSERT(virtual_pipe != I915_MAX_PIPES);
	vgt_set_pipe_mapping(vgt, virtual_pipe, physical_pipe);
	vgt_dbg(VGT_DBG_DPY, "vGT: add pipe mapping  %x - > %x \n", virtual_pipe, physical_pipe);
	vgt_update_irq_reg(vgt);
	if (vgt_has_pipe_enabled(vgt, virtual_pipe))
		vgt_update_frmcount(vgt, virtual_pipe);
	vgt_calculate_frmcount_delta(vgt, virtual_pipe);

	if (current_foreground_vm(vgt->pdev) == vgt) {
		vgt_restore_state(vgt, virtual_pipe);
	}

	return true;
}

bool update_pipe_mapping(struct vgt_device *vgt, unsigned int physical_reg, uint32_t physical_wr_data)
{
	int i = 0;
	uint32_t virtual_wr_data;
	enum vgt_pipe virtual_pipe = I915_MAX_PIPES;
	enum vgt_pipe physical_pipe = I915_MAX_PIPES;
	enum vgt_port pport;

	physical_pipe = get_pipe(physical_reg, physical_wr_data);

	/*disable pipe case*/
	if ((_REGBIT_TRANS_DDI_FUNC_ENABLE & physical_wr_data) == 0) {
		for (i = 0; i < I915_MAX_PIPES; i ++) {
			if(vgt->pipe_mapping[i] == physical_pipe) {
				vgt_set_pipe_mapping(vgt, i, I915_MAX_PIPES);
				vgt_dbg(VGT_DBG_DPY, "vGT: Update mapping: delete pipe %x  \n", i);
				if (vgt_has_pipe_enabled(vgt, i))
					vgt_update_frmcount(vgt, i);
				vgt_calculate_frmcount_delta(vgt, i);
			}
		}
		vgt_update_irq_reg(vgt);
		return true;
	}

	/*enable case*/
	if (physical_reg == _REG_TRANS_DDI_FUNC_CTL_EDP) {
		pport = PORT_A;
		if (vgt->ports[PORT_A].port_override == PORT_A) {
			virtual_pipe = get_edp_input(__vreg(vgt, _REG_TRANS_DDI_FUNC_CTL_EDP));
		}
	} else {
		pport = (physical_wr_data & _REGBIT_TRANS_DDI_PORT_MASK) >> _TRANS_DDI_PORT_SHIFT;
	}

	for (i = 0; i <= TRANSCODER_C; i++) {
		enum vgt_port vport, vport_override;
		virtual_wr_data = __vreg(vgt, _VGT_TRANS_DDI_FUNC_CTL(i));
		vport = (virtual_wr_data & _REGBIT_TRANS_DDI_PORT_MASK) >>
				_TRANS_DDI_PORT_SHIFT;
		vport_override = vgt->ports[vport].port_override;

		printk("%s: Enable. pport = %d, vport = %d\n", __FUNCTION__, pport, vport);

		if (!(_REGBIT_TRANS_DDI_FUNC_ENABLE & virtual_wr_data) ||
			(vport_override == I915_MAX_PORTS)) {
			continue;
		}

		if (vport_override == pport) {
			virtual_pipe = i;
			break;
		}
	}

	if (virtual_pipe != I915_MAX_PIPES) {
		vgt_set_pipe_mapping(vgt, virtual_pipe, physical_pipe);
		vgt_dbg(VGT_DBG_DPY, "vGT: Update pipe mapping  %x - > %x \n", virtual_pipe, physical_pipe);
		vgt_update_irq_reg(vgt);
		if (vgt_has_pipe_enabled(vgt, virtual_pipe))
			vgt_update_frmcount(vgt, virtual_pipe);
		vgt_calculate_frmcount_delta(vgt, virtual_pipe);
	}

	if (current_foreground_vm(vgt->pdev) == vgt &&
		virtual_pipe != I915_MAX_PIPES &&
		(_PRI_PLANE_ENABLE & VGT_MMIO_READ(vgt->pdev, VGT_DSPCNTR(physical_pipe)))) {
		vgt_restore_state(vgt, virtual_pipe);
	}

	return true;
}

/*
TODO: 1, program watermark in vgt. 2, make sure dom0 set the max timing for
each monitor in i915 driver
*/

bool set_panel_fitting(struct vgt_device *vgt, enum vgt_pipe pipe)
{
	unsigned int src_width, src_height;
	unsigned int target_width, target_height;
	unsigned int pf_ctl;
	enum vgt_pipe real_pipe;
	unsigned int h_total_reg;
	unsigned int v_total_reg;
	uint32_t edp_trans_code;
	uint64_t  plane_wm;
	uint64_t  sprite_wm;
	uint64_t  cursor_wm;
	unsigned int wm_reg;
	unsigned int wm_value;

	real_pipe = vgt->pipe_mapping[pipe];

	if (!enable_panel_fitting) {
		vgt_warn("panel fitting function is not enabled!\n");
		return false;
	}

	if (real_pipe == I915_MAX_PIPES) {
		vgt_dbg(VGT_DBG_DPY, "try to set panel fitting before pipe is mapped!\n");
		return false;
	}
	if (((_PRI_PLANE_ENABLE & __vreg(vgt, VGT_DSPCNTR(pipe))) == 0) ||
		(_PRI_PLANE_ENABLE & VGT_MMIO_READ(vgt->pdev, VGT_DSPCNTR(real_pipe))) == 0) {
		return false;
	}
	src_width = (__vreg(vgt, VGT_PIPESRC(pipe)) & 0xffff0000) >> 16;
	src_height = __vreg(vgt, VGT_PIPESRC(pipe)) & 0xffff;
	ASSERT_VM(src_width != 0, vgt);
	ASSERT_VM(src_height != 0, vgt);
	src_width += 1;
	src_height += 1;

	h_total_reg = VGT_HTOTAL(real_pipe);
	v_total_reg = VGT_VTOTAL(real_pipe);

	edp_trans_code = VGT_MMIO_READ(vgt->pdev, _REG_TRANS_DDI_FUNC_CTL_EDP);
	if ((_REGBIT_TRANS_DDI_FUNC_ENABLE & edp_trans_code)) {
		if (real_pipe == get_edp_input(edp_trans_code)) {
			h_total_reg = _REG_HTOTAL_EDP;
			v_total_reg = _REG_VTOTAL_EDP;
		}
	}

	target_width = VGT_MMIO_READ(vgt->pdev, h_total_reg) & 0xffff;
	target_height = VGT_MMIO_READ(vgt->pdev, v_total_reg) & 0xffff;

	ASSERT_VM(target_width != 0, vgt);
	ASSERT_VM(target_height != 0, vgt);
	target_width += 1;
	target_height += 1;

	/*fixed panel fitting mode to 3x3 mode, Restriction : A 3x3 capable filter must not be enabled
		when the pipe horizontal source size is greater than 2048 pixels*/
	if (IS_HSW(vgt->pdev))
		pf_ctl =  _REGBIT_PF_FILTER_MED_3x3 | _REGBIT_PF_PIPE_SEL(real_pipe);
	else /*after BDW the panel fitter is on the pipe, no need to assign.*/
		pf_ctl =  _REGBIT_PF_FILTER_MED_3x3;

	/*enable panel fitting only when the source mode does not eqaul to the target mode*/
	if (src_width != target_width || src_height != target_height ) {
		vgt_dbg(VGT_DBG_DPY, "enable panel fitting for VM %d, pipe %d, src_width:%d, src_height: %d, tgt_width:%d, tgt_height:%d!\n",
			vgt->vm_id, real_pipe, src_width, src_height, target_width, target_height);
		pf_ctl = pf_ctl | _REGBIT_PF_ENABLE;
	} else {
		vgt_dbg(VGT_DBG_DPY, "disable panel fitting for VM %d, for pipe %d!\n", vgt->vm_id, real_pipe);
	}

	/* we need to increase Water Mark in down scaling case */
	if (src_width > target_width || src_height > target_height) {
		wm_reg = real_pipe == PIPE_A ? _REG_WM0_PIPEA_ILK :
			(real_pipe == PIPE_B ? _REG_WM0_PIPEB_ILK : _REG_WM0_PIPEC_IVB);
		plane_wm = (__vreg(vgt_dom0, wm_reg) & _REGBIT_WM0_PIPE_PLANE_MASK)
			>> _REGBIT_WM0_PIPE_PLANE_SHIFT;
		sprite_wm = (__vreg(vgt_dom0, wm_reg) & _REGBIT_WM0_PIPE_SPRITE_MASK)
			>> _REGBIT_WM0_PIPE_SPRITE_SHIFT;
		cursor_wm = __vreg(vgt_dom0, wm_reg) & _REGBIT_WM0_PIPE_CURSOR_MASK;
		plane_wm = plane_wm * src_width * src_height / (target_width * target_height);
		sprite_wm = sprite_wm * src_width * src_height / (target_width * target_height);
		cursor_wm = cursor_wm * src_width * src_height / (target_width * target_height);
		plane_wm = plane_wm > DISPLAY_MAXWM ? DISPLAY_MAXWM : plane_wm;
		sprite_wm = sprite_wm > DISPLAY_MAXWM ? DISPLAY_MAXWM : sprite_wm;
		cursor_wm = cursor_wm > CURSOR_MAXWM ? CURSOR_MAXWM : cursor_wm;
		wm_value = cursor_wm & _REGBIT_WM0_PIPE_CURSOR_MASK;
		wm_value = wm_value | (sprite_wm  << _REGBIT_WM0_PIPE_SPRITE_SHIFT);
		wm_value = wm_value | ((plane_wm << _REGBIT_WM0_PIPE_PLANE_SHIFT) &
			_REGBIT_WM0_PIPE_PLANE_MASK);
		VGT_MMIO_WRITE(vgt->pdev, wm_reg, wm_value);
	}

	VGT_MMIO_WRITE(vgt->pdev, VGT_PIPESRC(real_pipe),  ((src_width -1) << 16) | (src_height - 1));
	VGT_MMIO_WRITE(vgt->pdev, VGT_PF_WIN_POS(real_pipe), 0);
	VGT_MMIO_WRITE(vgt->pdev, VGT_PF_CTL(real_pipe), pf_ctl);
	/* PF ctrl is a double buffered registers and gets updated when window
	 size registered is updated*/
	VGT_MMIO_WRITE(vgt->pdev, VGT_PF_WIN_SZ(real_pipe),  (target_width << 16) | target_height);
	return true;
}

bool vgt_manage_emul_dpy_events(struct pgt_device *pdev)
{
	int i;
	enum vgt_pipe pipe;
	unsigned hw_enabled_pipes, hvm_required_pipes;
	struct vgt_irq_host_state *hstate = pdev->irq_hstate;
	bool hvm_no_pipe_mapping = false;


	ASSERT(spin_is_locked(&pdev->lock));
	hw_enabled_pipes = hvm_required_pipes = 0;

	for (i = 0; i < VGT_MAX_VMS; i++) {
		struct vgt_device *vgt = pdev->device[i];
		vgt_reg_t pipeconf;

		if (vgt == NULL)
			continue;

		for (pipe = PIPE_A; pipe < I915_MAX_PIPES; pipe ++) {
			pipeconf = __vreg(vgt, VGT_PIPECONF(pipe));
			if (pipeconf & _REGBIT_PIPE_ENABLE) {
				if (is_current_display_owner(vgt))
					hw_enabled_pipes |= (1 << pipe);
				else {
					enum vgt_pipe p_pipe;
					p_pipe  = vgt->pipe_mapping[pipe];
					if (p_pipe != I915_MAX_PIPES) {
						hvm_required_pipes |=
								(1 << pipe);
					} else {
						hvm_no_pipe_mapping = true;
						break;
					}
				}
			}
		}

		pipeconf = __vreg(vgt, _REG_PIPE_EDP_CONF);
		if (pipeconf & _REGBIT_PIPE_ENABLE) {
			pipe = get_edp_input(
				__vreg(vgt, _REG_TRANS_DDI_FUNC_CTL_EDP));
			if (pipe == I915_MAX_PIPES) {
				vgt_err("vGT(%d): "
					"Invalid input selection for eDP\n",
					vgt->vgt_id);
				return false;
			}
			if (is_current_display_owner(vgt))
				hw_enabled_pipes |= (1 << pipe);
			else {
				enum vgt_pipe p_pipe = vgt->pipe_mapping[pipe];
				if (p_pipe != I915_MAX_PIPES) {
					hvm_required_pipes |= (1 << pipe);
				} else {
					hvm_no_pipe_mapping = true;
					break;
				}
			}
		}
	}

	hrtimer_cancel(&hstate->dpy_timer.timer);
	if (hvm_no_pipe_mapping || (hvm_required_pipes & ~hw_enabled_pipes)) {
		/*there is hvm enabled pipe which is not enabled on hardware */
		hrtimer_start(&hstate->dpy_timer.timer,
			ktime_add_ns(ktime_get(), hstate->dpy_timer.period),
			HRTIMER_MODE_ABS);
	}

	return true;
}

void vgt_update_frmcount(struct vgt_device *vgt,
	enum vgt_pipe pipe)
{
	uint32_t v_counter_addr, count, delta;
	enum vgt_pipe phys_pipe;
	v_counter_addr = VGT_PIPE_FRMCOUNT(pipe);
	phys_pipe = vgt->pipe_mapping[pipe];
	delta = vgt->frmcount_delta[pipe];
	if (phys_pipe == I915_MAX_PIPES)
		__vreg(vgt, v_counter_addr) = delta;
	else {
		uint32_t p_counter_addr = VGT_PIPE_FRMCOUNT(phys_pipe);
		count = VGT_MMIO_READ(vgt->pdev, p_counter_addr);
		if (count <= 0xffffffff - delta) {
			__vreg(vgt, v_counter_addr) = count + delta;
		} else { /* wrap it */
			count = 0xffffffff - count;
			__vreg(vgt, v_counter_addr) = delta - count - 1;
		}
	}
}

/* the calculation of delta may eliminate un-read frmcount in vreg.
 * so if pipe is enabled, need to update frmcount first before
 * calculating the delta
 */
void vgt_calculate_frmcount_delta(struct vgt_device *vgt,
	enum vgt_pipe pipe)
{
	uint32_t delta;
	uint32_t virt_counter = __vreg(vgt, VGT_PIPE_FRMCOUNT(pipe));
	enum vgt_pipe phys_pipe = vgt->pipe_mapping[pipe];
	uint32_t hw_counter;

	/* if physical pipe is not enabled yet, Delta will be used
	 * as the frmcount. When physical pipe is enabled, new delta
	 * will be calculated based on the hw count value.
	 */
	if (phys_pipe == I915_MAX_PIPES) {
		vgt->frmcount_delta[pipe] = virt_counter;
	} else {
		hw_counter = VGT_MMIO_READ(vgt->pdev,
					VGT_PIPE_FRMCOUNT(pipe));
		if (virt_counter >= hw_counter)
			delta = virt_counter - hw_counter;
		else {
			delta = 0xffffffff - hw_counter;
			delta += virt_counter + 1;
		}
		vgt->frmcount_delta[pipe] = delta;
	}
}

void vgt_set_power_well(struct vgt_device *vgt, bool to_enable)
{
	bool is_enabled, enable_requested;
	uint32_t tmp;

	tmp = VGT_MMIO_READ(vgt->pdev, _REG_HSW_PWR_WELL_CTL2);
	is_enabled = tmp & _REGBIT_HSW_PWR_WELL_STATE;
	enable_requested = tmp & _REGBIT_HSW_PWR_WELL_ENABLE;

	if (to_enable) {
		if (!enable_requested)
			VGT_MMIO_WRITE(vgt->pdev, _REG_HSW_PWR_WELL_CTL2, _REGBIT_HSW_PWR_WELL_ENABLE);

		if (!is_enabled) {
			if (wait_for_atomic((VGT_MMIO_READ(vgt->pdev, _REG_HSW_PWR_WELL_CTL2) &
				      _REGBIT_HSW_PWR_WELL_STATE), 20))
				vgt_err("Timeout enabling power well\n");
		}
	} else {
		if (enable_requested) {
			VGT_MMIO_WRITE(vgt->pdev, _REG_HSW_PWR_WELL_CTL2, 0);
			tmp = VGT_MMIO_READ(vgt->pdev, _REG_HSW_PWR_WELL_CTL2);
		}
	}
}

#define DPCD_HEADER_SIZE	0xb

u8 dpcd_fix_data[DPCD_HEADER_SIZE] = {
	0x11, 0x0a, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static bool is_dp_port_type(enum vgt_port_type port_type)
{
	if (port_type == VGT_DP_A ||
		port_type == VGT_DP_B ||
		port_type == VGT_DP_C ||
		port_type == VGT_DP_D) {
		return true;
	}
	return false;
}


/* copy the cached value into corresponding port field. Meanwhile,
 * Update system monitor state for EDID changes
 */
void vgt_flush_port_info(struct vgt_device *vgt, struct gt_port *port)
{
	int port_idx;
	enum vgt_port_type legacy_porttype;
	int i;
	unsigned int reg_ddi[4] ={
		_REG_TRANS_DDI_FUNC_CTL_A,
		_REG_TRANS_DDI_FUNC_CTL_B,
		_REG_TRANS_DDI_FUNC_CTL_C,
		_REG_TRANS_DDI_FUNC_CTL_EDP,
	};


	if (!vgt || !port)
		return;
	if (!port->cache.valid) {
		vgt_warn("port cache flush with invalid data. "
				"Will be ignored!\n");
		return;
	}

	port_idx = vgt_get_port(vgt, port);

	if (port_idx == I915_MAX_PORTS) {
		vgt_err ("VM-%d: port is not a valid pointer", vgt->vm_id);
		goto finish_flush;
	} 

	legacy_porttype = port->cache.type;

	if (legacy_porttype == VGT_PORT_MAX) {
		if (port_idx == PORT_E)
			legacy_porttype = VGT_CRT;
		else if (port->dpcd && port->dpcd->data_valid)
			legacy_porttype = VGT_DP_B + port_idx - 1;
		else
			legacy_porttype = VGT_HDMI_B + port_idx - 1;
	}

	if (port->edid == NULL) {
		port->edid = kmalloc(sizeof(struct vgt_edid_data_t), GFP_ATOMIC);
	}
	if (port->edid == NULL) {
		vgt_err("Memory allocation fail for EDID block!\n");
		return;
	}

	if (!(port->cache.edid && port->cache.edid->data_valid)) {
		port->edid->data_valid = false;
		if (port->dpcd)
			port->dpcd->data_valid = false;
		port->type = VGT_PORT_MAX;
		port->port_override = I915_MAX_PORTS;
	} else {
		memcpy(port->edid->edid_block,
			port->cache.edid->edid_block, EDID_SIZE);
		port->edid->data_valid = true;
		port->type = legacy_porttype;
		port->port_override = port->cache.port_override;
		if (vgt_debug & VGT_DBG_DPY) {
			vgt_info("Monitor detection:new monitor detected on %s\n", VGT_PORT_NAME(port->physcal_port));
			vgt_print_edid(port->edid);
		}

		if (is_dp_port_type(port->type)) {
			if (port->dpcd == NULL) {
				port->dpcd = kmalloc(sizeof(struct vgt_dpcd_data),
				GFP_ATOMIC);
			}

			if (port->dpcd == NULL) {
				return;
			}
			memset(port->dpcd->data, 0, DPCD_SIZE);
			memcpy(port->dpcd->data, dpcd_fix_data, DPCD_HEADER_SIZE);
			port->dpcd->data_valid = true;
			if (vgt_debug & VGT_DBG_DPY) {
				vgt_info("Monitor detection:assign fixed dpcd to port %s\n", VGT_PORT_NAME(port->physcal_port));
			}
		}
		
		for (i = 0; i <= 3; i++) {
			unsigned int ddi_value;
			ddi_value = VGT_MMIO_READ(vgt->pdev, reg_ddi[i]);
			if (_REGBIT_TRANS_DDI_FUNC_ENABLE & ddi_value) {
				update_pipe_mapping(vgt, reg_ddi[i], ddi_value);
			}
		}
	}
	vgt_update_monitor_status(vgt);

finish_flush:
	port->cache.valid = false;
	port->cache.type = VGT_PORT_MAX;
}

/*send uevent to user space to do display detection*/
void vgt_detect_display(struct vgt_device *vgt, int index)
{
	struct pgt_device *pdev = vgt->pdev;
	int i;
	enum vgt_uevent_type uevent;

	if (index == -1) { /* -1 index means "ALL" */
		for (i = 0; i < I915_MAX_PORTS; i++) {
			vgt_detect_display(vgt, i);
		}
		return;
	}

	uevent = VGT_DETECT_PORT_A + index;

	vgt_set_uevent(vgt, uevent);
	vgt_raise_request(pdev, VGT_REQUEST_UEVENT);
}

/* Set the initial plane/pipe/port state to be disabled,
 * letting gfx driver's mode setting to configure them late.
 * Notice that display owner could access physical MMIO states. Here
 * the setting only works for VMs who are not display owner.
 */
void vgt_dpy_init_modes(vgt_reg_t *mmio_array)
{
	enum vgt_port port;
	enum vgt_pipe pipe;
	unsigned int offset;

	mmio_array[REG_INDEX(_REG_DDI_BUF_CTL_A)] &=
				~_DDI_BUFCTL_DETECT_MASK;

	for (port = PORT_A; port <= PORT_E; ++ port) {
		offset = VGT_DDI_BUF_CTL(port);
		mmio_array[REG_INDEX(offset)] &= ~_REGBIT_DDI_BUF_ENABLE;
		offset = VGT_DP_TP_CTL(port);
		mmio_array[REG_INDEX(offset)] &= ~_REGBIT_DP_TP_ENABLE;
	}

	for (pipe = PIPE_A; pipe <= PIPE_C; ++ pipe) {
		offset = _VGT_TRANS_DDI_FUNC_CTL(pipe);
		mmio_array[REG_INDEX(offset)] &= ~_REGBIT_TRANS_DDI_FUNC_ENABLE;
		offset = VGT_PIPECONF(pipe);
		mmio_array[REG_INDEX(offset)] &= ~_REGBIT_PIPE_ENABLE;
		offset = VGT_TRANSCONF(pipe);
		mmio_array[REG_INDEX(offset)] &= ~_REGBIT_TRANS_ENABLE;
		offset = VGT_PF_CTL(pipe);
		mmio_array[REG_INDEX(offset)] &= ~_REGBIT_PF_ENABLE;
	}

	mmio_array[REG_INDEX(_REG_TRANS_DDI_FUNC_CTL_EDP)] &=
				~_REGBIT_TRANS_DDI_FUNC_ENABLE;
	mmio_array[REG_INDEX(_REG_PIPE_EDP_CONF)] &=
				~_REGBIT_PIPE_ENABLE;

	mmio_array[REG_INDEX(_REG_SPLL_CTL)] &= ~_REGBIT_SPLL_CTL_ENABLE;
	mmio_array[REG_INDEX(_REG_WRPLL_CTL1)] &= ~_REGBIT_WRPLL_ENABLE;
	mmio_array[REG_INDEX(_REG_WRPLL_CTL2)] &= ~_REGBIT_WRPLL_ENABLE;
}
