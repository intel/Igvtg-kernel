/*
 * vGT sysfs interface (the original code comes from samples/kobject-example.c)
 *
 * Copyright(c) 2011-2013 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of Version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/slab.h>
#include "vgt.h"

struct kobject *vgt_ctrl_kobj;
static struct kset *vgt_kset;
static DEFINE_MUTEX(vgt_sysfs_lock);

static void vgt_kobj_release(struct kobject *kobj)
{
	pr_debug("kobject: (%p): %s\n", kobj, __func__);
	/* NOTE: we do not deallocate our kobject */
	/* see the comment before vgt_init_sysfs() */
	//kfree(kobj);
}

static int vgt_add_state_sysfs(vgt_params_t vp);
int vgt_del_state_sysfs(vgt_params_t vp);
static ssize_t vgt_create_instance_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	vgt_params_t vp;
	int param_cnt;
	char param_str[64];
	int rc;
	int high_gm_sz;
	int low_gm_sz;

	/* We expect the param_str should be vmid,a,b,c (where the guest
	* wants a MB aperture and b MB gm, and c fence registers) or -vmid
	* (where we want to release the vgt instance).
	*/
	(void)sscanf(buf, "%63s", param_str);
	param_cnt = sscanf(param_str, "%d,%d,%d,%d,%d", &vp.vm_id, &low_gm_sz,
		&high_gm_sz, &vp.fence_sz, &vp.vgt_primary);
	vp.aperture_sz = low_gm_sz;
	vp.gm_sz = high_gm_sz + low_gm_sz;

	if (param_cnt == 1) {
		if (vp.vm_id >= 0)
			return -EINVAL;
	} else if (param_cnt == 4 || param_cnt == 5) {
		if (!(vp.vm_id > 0 && vp.aperture_sz > 0 &&
			vp.aperture_sz <= vp.gm_sz && vp.fence_sz > 0))
			return -EINVAL;

		if (param_cnt == 5) {
			/* -1/0/1 means: not-specified, non-primary, primary */
			if (vp.vgt_primary < -1 && vp.vgt_primary > 1)
				return -EINVAL;
		} else {
			vp.vgt_primary = -1; /* no valid value specified. */
		}
	} else
		return -EINVAL;

	mutex_lock(&vgt_sysfs_lock);
	rc = (vp.vm_id > 0) ? vgt_add_state_sysfs(vp) : vgt_del_state_sysfs(vp);
	mutex_unlock(&vgt_sysfs_lock);

	return rc < 0 ? rc : count;
}

static ssize_t vgt_display_owner_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	struct pgt_device *pdev = &default_device;
	return sprintf(buf,"%d\n", current_display_owner(pdev)->vm_id);
}

static ssize_t vgt_display_owner_store(struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t count)
{
	int vmid;
	if (sscanf(buf, "%d", &vmid) != 1)
		return -EINVAL;

	if (vmid != 0) {
		vgt_warn("Cannot change display_owner to vms other than domain0!\n");
	}

	return count;
}

static ssize_t vgt_foreground_vm_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf,"%d\n", current_foreground_vm((&default_device))->vm_id);
}

static ssize_t vgt_foreground_vm_store(struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t count)
{
	unsigned long flags;
	int ret = count;
	int vmid;
	struct vgt_device *next_vgt;
	struct pgt_device *pdev = &default_device;
	int cpu;

	if (sscanf(buf, "%d", &vmid) != 1)
		return -EINVAL;

	mutex_lock(&vgt_sysfs_lock);

	vgt_lock_dev_flags(pdev, cpu, flags);

	next_vgt = vmid_2_vgt_device(vmid);
	if (next_vgt == NULL) {
		printk("vGT: can not find the vgt instance of dom%d!\n", vmid);
		ret = -ENODEV;
		goto out;
	}

	if (current_foreground_vm(pdev) == next_vgt) {
		goto out;
	}

	if (!__vreg(next_vgt, vgt_info_off(display_ready))) {
		printk("VGT %d: Display is not ready.\n", vmid);
		ret = -EAGAIN;
		goto out;
	}

	pdev->next_foreground_vm = next_vgt;
	vgt_raise_request(pdev, VGT_REQUEST_DPY_SWITCH);
out:
	vgt_unlock_dev_flags(pdev, cpu, flags);

	mutex_unlock(&vgt_sysfs_lock);

	return ret;
}

static ssize_t vgt_ctx_switch_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	int val;
	bool enabled;

	if (sscanf(buf, "%du", &val) != 1)
		return -EINVAL;
	enabled = !!val;
	vgt_toggle_ctx_switch(enabled);
	return count;
}

static ssize_t vgt_validate_ctx_switch_store(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%du", &val) != 1)
		return -EINVAL;
	vgt_validate_ctx_switch = !!val;
	return count;
}

static ssize_t vgt_ctx_switch_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "VGT context switch: %s\n",
			vgt_ctx_switch ? "enabled" : "disabled");
}

static ssize_t vgt_validate_ctx_switch_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "VGT check mmio restore: %s\n",
			vgt_validate_ctx_switch ? "enabled" : "disabled");
}

static ssize_t vgt_dpy_switch_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	int val;
	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

//	fastpath_dpy_switch = !!val;
	fastpath_dpy_switch = true;
	return count;
}

static ssize_t vgt_dpy_switch_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "VGT display_owner switch: using the %s\n",
				fastpath_dpy_switch ?
				"fast-path method. (write 0 to use the slow-path method)"
				: "slow-path method. (write 1 to use the fast-path method)");
}

 static ssize_t vgt_available_res_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	struct pgt_device *pdev = &default_device;
	ssize_t buf_len;
	int cpu;

	mutex_lock(&vgt_sysfs_lock);
	vgt_lock_dev(pdev, cpu);
	buf_len = get_avl_vm_aperture_gm_and_fence(pdev, buf,
			PAGE_SIZE);
	vgt_unlock_dev(pdev, cpu);
	mutex_unlock(&vgt_sysfs_lock);

	return buf_len;
}


static ssize_t vgt_virtual_event_reader(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	ssize_t nbytes = sprintf(buf, "== README ==\n"
			"This interface is only used for debugging purpose.\n"
			"Cat an integer to trigger a virtual event.\n"
			"\t bit 0 - bit 15: for virtual event number;\n"
			"\t bit 16 - bit 23: for vm id;\n"
			"Supported event list:\n"
			"[0x%x]: %s\n"
			"[0x%x]: %s\n"
			"[0x%x]: %s\n",
			RCS_AS_CONTEXT_SWITCH, vgt_irq_name[RCS_AS_CONTEXT_SWITCH],
			VCS_AS_CONTEXT_SWITCH, vgt_irq_name[VCS_AS_CONTEXT_SWITCH],
			BCS_AS_CONTEXT_SWITCH, vgt_irq_name[BCS_AS_CONTEXT_SWITCH]);
	return nbytes;
}

static ssize_t vgt_virtual_event_trigger(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int cpu;
	enum vgt_event_type event;
	struct vgt_device *vgt;
	vgt_virtual_event_t v_event;
	struct pgt_device *pdev = &default_device;

	if (sscanf(buf, "%i", &v_event.dw) != 1)
		return -EINVAL;

	event = v_event.virtual_event;
	if ((event != RCS_AS_CONTEXT_SWITCH) &&
			(event != VCS_AS_CONTEXT_SWITCH) &&
			(event != BCS_AS_CONTEXT_SWITCH)) {
		return -EINVAL;
	}

	vgt = vmid_2_vgt_device(v_event.vmid);

	if (!vgt)
		return -EINVAL;

	vgt_lock_dev(pdev, cpu);
	vgt_trigger_virtual_event(vgt, event);
	vgt_unlock_dev(pdev, cpu);

	return count;
}

static struct kobj_attribute create_vgt_instance_attrs =
	__ATTR(create_vgt_instance, 0220, NULL, vgt_create_instance_store);
static struct kobj_attribute display_owner_ctrl_attrs =
	__ATTR(display_owner, 0660, vgt_display_owner_show, vgt_display_owner_store);
static struct kobj_attribute foreground_vm_ctrl_attrs =
	__ATTR(foreground_vm, 0660, vgt_foreground_vm_show, vgt_foreground_vm_store);

static struct kobj_attribute virtual_event_attrs =
	__ATTR(virtual_event, 0660, vgt_virtual_event_reader, vgt_virtual_event_trigger);

static struct kobj_attribute ctx_switch_attrs =
	__ATTR(ctx_switch, 0660, vgt_ctx_switch_show, vgt_ctx_switch_store);

static struct kobj_attribute validate_ctx_switch_attrs =
	__ATTR(validate_ctx_switch, 0660, vgt_validate_ctx_switch_show, vgt_validate_ctx_switch_store);

static struct kobj_attribute dpy_switch_attrs =
	__ATTR(display_switch_method, 0660, vgt_dpy_switch_show, vgt_dpy_switch_store);

static struct kobj_attribute available_res_attrs =
	__ATTR(available_resource, 0440, vgt_available_res_show, NULL);

static struct attribute *vgt_ctrl_attrs[] = {
	&create_vgt_instance_attrs.attr,
	&display_owner_ctrl_attrs.attr,
	&foreground_vm_ctrl_attrs.attr,
	&virtual_event_attrs.attr,
	&ctx_switch_attrs.attr,
	&validate_ctx_switch_attrs.attr,
	&dpy_switch_attrs.attr,
	&available_res_attrs.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

#define kobj_to_port(kobj) container_of((kobj), struct gt_port, kobj)
#define kobj_to_vgt(xkobj) container_of((xkobj), struct vgt_device, kobj)

static ssize_t vgt_port_edid_show(struct file *filp, struct kobject *kobj,
				  struct bin_attribute *attr, char *buf, loff_t off,
				  size_t count)
{
	struct gt_port *port = kobj_to_port(kobj);
	struct vgt_edid_data_t *edid;

	if (off >= EDID_SIZE) {
		return 0;
	}

	if (off + count > EDID_SIZE) {
		count = EDID_SIZE - off;
	}

	mutex_lock(&vgt_sysfs_lock);

	edid = port->edid;

	if (edid && edid->data_valid) {
		memcpy(buf, edid->edid_block + off, count);
	} else {
		count = 0;
	}

	mutex_unlock(&vgt_sysfs_lock);

	return count;
}

static ssize_t
vgt_port_edid_store(struct file* filp, struct kobject *kobj,
		    struct bin_attribute *bin_attr,
		    char *buf, loff_t off, size_t count)
{
	struct gt_port *port = kobj_to_port(kobj);
	int write_count = count;
	char *dest;

	if (!count || off < 0 || (off & 3))
		return count;

	if (off >= bin_attr->size)
		return -EINVAL;

	if (off + count > bin_attr->size)
		write_count = bin_attr->size - off;

	mutex_lock(&vgt_sysfs_lock);

	if (port->cache.edid == NULL) {
		port->cache.edid = kmalloc(sizeof(struct vgt_edid_data_t),
			      GFP_ATOMIC);
	}

	if (port->cache.edid == NULL) {
		mutex_unlock(&vgt_sysfs_lock);
		return -ENOMEM;
	}

	dest = port->cache.edid->edid_block + off;
	memcpy(dest, buf, write_count);
	if (off + write_count == bin_attr->size &&
		vgt_is_edid_valid(port->cache.edid->edid_block)) {
		
		// customize the EDID to remove extended EDID block.
		u8 *block = port->cache.edid->edid_block;
		if (block[0x7e]) {
			block[0x7f] += block[0x7e];
			block[0x7e] = 0;
		}
		port->cache.edid->data_valid = true;
		port->cache.valid = true;
	}

	mutex_unlock(&vgt_sysfs_lock);

	return count;
}

static inline int ctoi(char chr)
{
    return (chr >= '0' && chr <= '9' ? chr - '0' :
		(chr >= 'a' && chr <= 'f' ? chr - 'a' + 10 :
		(chr >= 'A' && chr <= 'F' ? chr - 'A' + 10 :
			-1)));
}

static ssize_t
vgt_port_edid_text_store(struct kobject *kobj, struct kobj_attribute *attr,
                        const char *buf, size_t count)
{
	struct gt_port *port = kobj_to_port(kobj);
	int i;
	char *dest;

	if (count != (EDID_SIZE << 1))
		return -EINVAL;

	mutex_lock(&vgt_sysfs_lock);

	if (port->cache.edid == NULL) {
		port->cache.edid = kmalloc(sizeof(struct vgt_edid_data_t),
			      GFP_ATOMIC);
	}

	if (port->cache.edid == NULL) {
		return -ENOMEM;
	}

	port->cache.edid->data_valid = false;

	dest = port->cache.edid->edid_block;

	for (i = 0; i < count; i += 2) {
		int hi = ctoi(buf[i]);
		int lo  = ctoi(buf[i + 1]);
		if (hi < 0 || lo < 0) {
			vgt_warn("invalid injected EDID!\n");
			break;
		}

		*dest= (hi << 4) + lo;
		dest++;
	}

	if ((i == count) && vgt_is_edid_valid(port->cache.edid->edid_block)) {
		// customize the EDID to remove extended EDID block.
		u8 *block = port->cache.edid->edid_block;
		if (block[0x7e]) {
			block[0x7f] += block[0x7e];
			block[0x7e] = 0;
		}
		port->cache.edid->data_valid = true;
		port->cache.valid = true;
	}

	mutex_unlock(&vgt_sysfs_lock);

	return count;
}


static bool is_port_connected(struct gt_port *port)
{
	if (port && port->edid && port->edid->data_valid) {
		return true;
	}
	return false;
}

static ssize_t vgt_pport_connection_show(struct kobject *kobj, struct kobj_attribute *attr,
				   char *buf)
{
	struct pgt_device *pgt = &default_device;
	ssize_t buf_len;
	int i;

        for (i = 0; i < I915_MAX_PORTS; i++) {
                if (strcmp(VGT_PORT_NAME(i), kobj->name) == 0) {
                        break;
                }
        }

	if (i >= I915_MAX_PORTS) {
		return 0;
	}

	mutex_lock(&vgt_sysfs_lock);

	buf_len = sprintf(buf, "%s\n", is_port_connected(&(pgt->ports[i])) ?
			"connected" : "disconnected");

	mutex_unlock(&vgt_sysfs_lock);

	return buf_len;
}

static bool is_pport_present(struct pgt_device *pgt, struct gt_port *port)
{

	bool found = false;

	switch (port->physcal_port) {
	case PORT_A:
		found = VGT_MMIO_READ(pgt, _REG_DDI_BUF_CTL_A) & _DDI_BUFCTL_DETECT_MASK;
		break;
	case PORT_B:
		found = VGT_MMIO_READ(pgt,_REG_SFUSE_STRAP) & _REGBIT_SFUSE_STRAP_B_PRESENTED;
		break;
	case PORT_C:
		found = VGT_MMIO_READ(pgt,_REG_SFUSE_STRAP) & _REGBIT_SFUSE_STRAP_C_PRESENTED;
		break;
	case PORT_D:
		found = VGT_MMIO_READ(pgt,_REG_SFUSE_STRAP) & _REGBIT_SFUSE_STRAP_D_PRESENTED;
		break;
	case PORT_E:
		found = true;
		break;
	default:
		found = false;
		break;
	}

	return found;

}

static ssize_t vgt_pport_presnece_show(struct kobject *kobj, struct kobj_attribute *attr,
				   char *buf)
{
	struct pgt_device *pgt = &default_device;

	ssize_t buf_len;
	int i;

        for (i = 0; i < I915_MAX_PORTS; i++) {
                if (strcmp(VGT_PORT_NAME(i), kobj->name) == 0) {
                        break;
                }
        }

	if (i >= I915_MAX_PORTS) {
		return 0;
	}

	mutex_lock(&vgt_sysfs_lock);

	buf_len = sprintf(buf, "%s\n", is_pport_present(pgt, &(pgt->ports[i])) ?
			"present" : "unpresent");
	mutex_unlock(&vgt_sysfs_lock);

	return buf_len;
}

static ssize_t vgt_pport_connection_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	bool is_connected = false;

	mutex_lock(&vgt_sysfs_lock);
	if (strncmp("connect", buf, 7) == 0) {
			is_connected = true;
	} else if (strncmp("disconnect", buf, 10) == 0) {
			is_connected = false;
	}
	mutex_unlock(&vgt_sysfs_lock);

	return count;
}

static ssize_t vgt_vport_connection_show(struct kobject *kobj, struct kobj_attribute *attr,
				   char *buf)
{
	struct gt_port *port = kobj_to_port(kobj);
	ssize_t buf_len;

	mutex_lock(&vgt_sysfs_lock);
	
	buf_len = sprintf(buf, "%s\n", is_port_connected(port) ? "connected" : "disconnected");

	mutex_unlock(&vgt_sysfs_lock);

	return buf_len;
}

static ssize_t vgt_vport_connection_store(struct kobject *kobj, struct kobj_attribute *attr,
                        const char *buf, size_t count)
{
	struct gt_port *port = kobj_to_port(kobj);
	struct vgt_device *vgt = kobj_to_vgt(kobj->parent);
	enum vgt_event_type event;
	bool flush_request = false;
	bool hotplug_request = false;
	int cpu;
	bool is_current_connected = is_port_connected(port);

	mutex_lock(&vgt_sysfs_lock);
	vgt_lock_dev(vgt->pdev, cpu);

	if (strncmp("connect", buf, 7) == 0) {
		vgt_info("Monitor detection: %s  is connected\n", VGT_PORT_NAME(port->physcal_port));
		if (!(port->cache.valid && port->cache.edid &&
				port->cache.edid->data_valid))
			vgt_warn("Request to connect a monitor but new monitor "
				"setting is not ready. Will be ignored\n");
		else if (!is_current_connected) {
			flush_request = hotplug_request = true;
		} else if (is_current_connected &&
			memcmp(port->edid->edid_block, port->cache.edid->edid_block, EDID_SIZE)) {
			flush_request = hotplug_request = true;
		}
	} else if (strncmp("disconnect", buf, 10) == 0) {
		vgt_info("Monitor detection: %s  is disconnected\n", VGT_PORT_NAME(port->physcal_port));
		if (is_current_connected) {
			if (port->cache.edid)
				port->cache.edid->data_valid = false;
			port->cache.valid = true;
			flush_request = hotplug_request = true;
		}
	} else if (strncmp("flush", buf, 5) == 0) {
		flush_request = true;
	} else {
		vgt_warn("Input string not recognized: %s\n", buf);
	}

	if (flush_request)
		vgt_flush_port_info(vgt, port);

	if (hotplug_request) {
		enum vgt_port port_type = vgt_get_port(vgt, port);
		switch (port_type) {
		case PORT_A:
			event = EVENT_MAX; break;
		case PORT_B:
			event = DP_B_HOTPLUG; break;
		case PORT_C:
			event = DP_C_HOTPLUG; break;
		case PORT_D:
			event = DP_D_HOTPLUG; break;
		case PORT_E:
			event = CRT_HOTPLUG; break;
		default:
			event = EVENT_MAX;
			vgt_err("Invalid port(%s) for hotplug!\n",
				VGT_PORT_NAME(port_type));
		}
		if (event != EVENT_MAX)
			vgt_trigger_virtual_event(vgt, event);
	}

	vgt_unlock_dev(vgt->pdev, cpu);
	mutex_unlock(&vgt_sysfs_lock);

	return count;
}

static ssize_t vgt_port_type_show(struct kobject *kobj, struct kobj_attribute *attr,
				   char *buf)
{
	struct gt_port *port = kobj_to_port(kobj);
	ssize_t buf_len;

	mutex_lock(&vgt_sysfs_lock);
	buf_len = sprintf(buf, "%s\n", VGT_PORT_TYPE_NAME(port->type));
	mutex_unlock(&vgt_sysfs_lock);

	return buf_len;
}

static ssize_t vgt_port_type_store(struct kobject *kobj, struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	struct gt_port *port = kobj_to_port(kobj);
	int portIndex;


	mutex_lock(&vgt_sysfs_lock);
	if (sscanf(buf, "%d", &portIndex) != 1) {
		mutex_unlock(&vgt_sysfs_lock);
		return -EINVAL;
	}

	port->cache.type = VGT_CRT + portIndex;
	mutex_unlock(&vgt_sysfs_lock);

	return count;
}

static ssize_t vgt_vport_port_override_show(struct kobject *kobj, struct kobj_attribute *attr,
				   char *buf)
{
	struct gt_port *port = kobj_to_port(kobj);
	ssize_t buf_len;

	mutex_lock(&vgt_sysfs_lock);
	buf_len = sprintf(buf, "%s\n", VGT_PORT_NAME(port->port_override));
	mutex_unlock(&vgt_sysfs_lock);

	return buf_len;
}

static ssize_t vgt_vport_port_override_store(struct kobject *kobj, struct kobj_attribute *attr,
                        const char *buf, size_t count)
{
	struct gt_port *port = kobj_to_port(kobj);
	enum vgt_port override;

	if (strncmp("PORT_A", buf, 6) == 0) {
		override = PORT_A;
	} else if (strncmp("PORT_B", buf, 6) == 0) {
		override = PORT_B;
	} else if (strncmp("PORT_C", buf, 6) == 0) {
		override  = PORT_C;
	} else if (strncmp("PORT_D", buf, 6) == 0) {
		override  = PORT_D;
	} else if (strncmp("PORT_E", buf, 6) == 0) {
		override = PORT_E;
	} else {
		return -EINVAL;
	}

	mutex_lock(&vgt_sysfs_lock);

	port->cache.port_override = override;
	port->cache.valid = true;

	mutex_unlock(&vgt_sysfs_lock);

	return count;
}

static ssize_t vgt_vport_pipe_show(struct kobject *kobj, struct kobj_attribute *attr,
				   char *buf)
{
	struct gt_port *port_ptr = kobj_to_port(kobj);
	struct vgt_device *vgt = kobj_to_vgt(kobj->parent);
	enum vgt_port port;
	ssize_t buf_len;
	int cpu;

	mutex_lock(&vgt_sysfs_lock);
	vgt_lock_dev(vgt->pdev, cpu);

	port = vgt_get_port(vgt, port_ptr);
	if (port == PORT_A)
		buf_len = sprintf(buf, "PIPE_EDP\n");
	else {
		enum vgt_pipe pipe = vgt_get_pipe_from_port(vgt, port);
		buf_len = sprintf(buf, "%s\n", VGT_PIPE_NAME(pipe));
	}

	vgt_unlock_dev(vgt->pdev, cpu);
	mutex_unlock(&vgt_sysfs_lock);

	return buf_len;
}

static struct kobj_attribute vport_connection_attrs =
	__ATTR(connection, 0660, vgt_vport_connection_show, vgt_vport_connection_store);

static struct kobj_attribute vport_type_attrs =
	__ATTR(type, 0660, vgt_port_type_show, vgt_port_type_store);

static struct kobj_attribute vport_port_override_attrs =
	__ATTR(port_override, 0660, vgt_vport_port_override_show, vgt_vport_port_override_store);

static struct kobj_attribute vport_pipe_attrs =
	__ATTR(pipe, 0440, vgt_vport_pipe_show, NULL);

// EDID text mode input interface for the convenience of testing
static struct kobj_attribute vport_edid_text_attrs =
	__ATTR(edid_text, 0660, NULL, vgt_port_edid_text_store);

static struct attribute *vgt_vport_attrs[] = {
	&vport_connection_attrs.attr,
	&vport_type_attrs.attr,
	&vport_port_override_attrs.attr,
	&vport_pipe_attrs.attr,
	&vport_edid_text_attrs.attr,
	NULL,
};

static struct bin_attribute port_edid_attr = {
        .attr = {
                .name = "edid",
                .mode = 0660
        },
        .size = EDID_SIZE,
        .read = vgt_port_edid_show,
        .write = vgt_port_edid_store,
};

static struct kobj_attribute pport_type_attrs =
	__ATTR(type, 0660, vgt_port_type_show, vgt_port_type_store);

static struct kobj_attribute pport_connection_attrs =
	__ATTR(connection, 0660, vgt_pport_connection_show, vgt_pport_connection_store);

static struct kobj_attribute pport_presence_attrs =
	__ATTR(presence, 0440, vgt_pport_presnece_show, NULL);

static struct attribute *vgt_pport_attrs[] = {
	&pport_connection_attrs.attr,
	&pport_type_attrs.attr,
	&pport_presence_attrs.attr,
	NULL,
};

/* copied code from here */
static ssize_t kobj_attr_show(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	struct kobj_attribute *kattr;
	ssize_t ret = -EIO;

	kattr = container_of(attr, struct kobj_attribute, attr);
	if (kattr->show)
		ret = kattr->show(kobj, kattr, buf);
	return ret;
}

static ssize_t kobj_attr_store(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	struct kobj_attribute *kattr;
	ssize_t ret = -EIO;

	kattr = container_of(attr, struct kobj_attribute, attr);
	if (kattr->store)
		ret = kattr->store(kobj, kattr, buf, count);
	return ret;
}

const struct sysfs_ops vgt_kobj_sysfs_ops = {
	.show	= kobj_attr_show,
	.store	= kobj_attr_store,
};


/* copied code end */

static ssize_t vgt_id_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct vgt_device *vgt = kobj_to_vgt(kobj);
	return sprintf(buf, "%x\n", vgt->vgt_id);
}

static ssize_t gm_sz_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
	struct vgt_device *vgt = kobj_to_vgt(kobj);
	return sprintf(buf, "%016llx\n", vgt->gm_sz);
}

static ssize_t aperture_sz_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
	struct vgt_device *vgt = kobj_to_vgt(kobj);
	return sprintf(buf, "%016llx\n", vgt->aperture_sz);
}

static ssize_t aperture_base_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
	struct vgt_device *vgt = kobj_to_vgt(kobj);
	return sprintf(buf, "%016llx\n", vgt->aperture_base);
}

static ssize_t aperture_base_va_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
	struct vgt_device *vgt = kobj_to_vgt(kobj);
	return sprintf(buf, "%p\n", vgt->aperture_base_va);
}

static struct kobj_attribute vgt_id_attribute =
	__ATTR_RO(vgt_id);

static struct kobj_attribute gm_sz_attribute =
	__ATTR_RO(gm_sz);

static struct kobj_attribute aperture_sz_attribute =
	__ATTR_RO(aperture_sz);

static struct kobj_attribute aperture_base_attribute =
	__ATTR_RO(aperture_base);

static struct kobj_attribute aperture_base_va_attribute =
	__ATTR_RO(aperture_base_va);

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *vgt_instance_attrs[] = {
	&vgt_id_attribute.attr,
	&gm_sz_attribute.attr,
	&aperture_sz_attribute.attr,
	&aperture_base_attribute.attr,
	&aperture_base_va_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

/*
 * An unnamed attribute group will put all of the attributes directly in
 * the kobject directory.  If we specify a name, a subdirectory will be
 * created for the attributes with the directory being the name of the
 * attribute group.
 */
#if 0
static struct attribute_group attr_group = {
	.attrs = attrs,
};
#endif

static struct kobj_type vgt_instance_ktype = {
	.release	= vgt_kobj_release,
	.sysfs_ops	= &vgt_kobj_sysfs_ops,
	.default_attrs = vgt_instance_attrs,
};

static struct kobj_type vgt_ctrl_ktype = {
	.release	= vgt_kobj_release,
	.sysfs_ops  = &vgt_kobj_sysfs_ops,
	.default_attrs = vgt_ctrl_attrs,
};

static struct kobj_type vgt_vport_ktype = {
	.release	= vgt_kobj_release,
	.sysfs_ops	= &vgt_kobj_sysfs_ops,
	.default_attrs	= vgt_vport_attrs,
};

static struct kobj_type vgt_pport_ktype = {
	.release	= vgt_kobj_release,
	.sysfs_ops	= &vgt_kobj_sysfs_ops,
	.default_attrs	= vgt_pport_attrs,
};

static ssize_t
igd_mmio_read(struct file *filp, struct kobject *kobj,
		struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct pgt_device *pdev = &default_device;
	size_t init_count = count, len;
	unsigned long data;
	int cpu;

	if (!count || off < 0 || off + count > bin_attr->size || (off & 0x3))
		return -EINVAL;

	vgt_lock_dev(pdev, cpu);

	while (count > 0) {
		len = (count > sizeof(unsigned long)) ? sizeof(unsigned long) :
				count;

		if (vgt_native_mmio_read(off, &data, len, false) != 0) {
			vgt_unlock_dev(pdev, cpu);
			return -EIO;
		}

		memcpy(buf, &data, len);
		buf += len;
		count -= len;
	}

	vgt_unlock_dev(pdev, cpu);

	return init_count;
}

static ssize_t
igd_mmio_write(struct file* filp, struct kobject *kobj,
		struct bin_attribute *bin_attr,
		char *buf, loff_t off, size_t count)
{
	struct pgt_device *pdev = &default_device;
	size_t init_count = count, len;
	unsigned long data;
	int cpu;

	if (!count || off < 0 || off + count > bin_attr->size || (off & 0x3))
		return -EINVAL;

	vgt_lock_dev(pdev, cpu);

	while (count > 0) {
		len = (count > sizeof(unsigned long)) ? sizeof(unsigned long) :
				count;

		memcpy(&data, buf, len);
		if (vgt_native_mmio_read(off, &data, len, false) != 0) {
			vgt_unlock_dev(pdev, cpu);
			return -EIO;
		}

		buf += len;
		count -= len;
	}

	vgt_unlock_dev(pdev, cpu);
	return init_count;
}

static struct bin_attribute igd_mmio_attr = {
	.attr =	{
		.name = "igd_mmio",
		.mode = 0660
	},
	.size = VGT_MMIO_SPACE_SZ,
	.read = igd_mmio_read,
	.write = igd_mmio_write,
};


static int vgt_add_state_sysfs(vgt_params_t vp)
{
	int retval, i;
	struct vgt_device *vgt;
	/*
	* Create a simple kobject located under /sys/kernel/
	* As this is a simple directory, no uevent will be sent to
	* userspace.  That is why this function should not be used for
	* any type of dynamic kobjects, where the name and number are
	* not known ahead of time.
	*/

	ASSERT(vgt_ctrl_kobj);

	/* check if such vmid has been used */
	if (vmid_2_vgt_device(vp.vm_id))
		return -EINVAL;

	retval = create_vgt_instance(&default_device, &vgt, vp);

	if (retval < 0)
		return retval;

	/* init kobject */
	kobject_init(&vgt->kobj, &vgt_instance_ktype);

	/* set it before calling the kobject core */
	vgt->kobj.kset = vgt_kset;

	/* add kobject, NULL parent indicates using kset as parent */
	retval = kobject_add(&vgt->kobj, NULL, "vm%u", vgt->vm_id);
	if (retval) {
		printk(KERN_WARNING "%s: vgt kobject add error: %d\n",
					__func__, retval);
		kobject_put(&vgt->kobj);
	}

	for (i = 0; i < I915_MAX_PORTS; i++) {
		retval = kobject_init_and_add(&vgt->ports[i].kobj,
					      &vgt_vport_ktype,
					      &vgt->kobj,
					      "%s",
					      VGT_PORT_NAME(i));

		if (retval) {
			printk(KERN_WARNING
			       "%s: vgt vport kobject add error: %d\n",
			       __func__, retval);
			retval = -EINVAL;
			goto kobj_fail;
		}

		retval = sysfs_create_bin_file(&vgt->ports[i].kobj,
					    &port_edid_attr);
		if (retval < 0) {
			retval = -EINVAL;
			goto kobj_fail;
		}
	}

	if ((propagate_monitor_to_guest) && (vgt->vm_id != 0)) {
		vgt_detect_display(vgt, -1);
	}

	return retval;

kobj_fail:
	for (; i >= 0; i++) {
		kobject_put(&vgt->ports[i].kobj);
	}
	kobject_put(&vgt->kobj);
	return retval;
}

int vgt_del_state_sysfs(vgt_params_t vp)
{
	struct vgt_device *vgt;
	int i;

	vp.vm_id = -vp.vm_id;
	vgt = vmid_2_vgt_device(vp.vm_id);
	if (!vgt)
		return -ENODEV;

	for (i = 0; i < I915_MAX_PORTS; i++) {
		kobject_put(&vgt->ports[i].kobj);
	}

	kobject_put(&vgt->kobj);

	vgt_release_instance(vgt);

	return 0;
}

int vgt_init_sysfs(struct pgt_device *pdev)
{
	struct pgt_device *pgt = &default_device;
	int ret, i = 0;

	vgt_kset = kset_create_and_add("vgt", NULL, kernel_kobj);
	if (!vgt_kset) {
		ret = -ENOMEM;
		goto kset_fail;
	}

	vgt_ctrl_kobj = kzalloc(sizeof(struct kobject), GFP_KERNEL);
	if (!vgt_ctrl_kobj) {
		ret = -ENOMEM;
		goto ctrl_fail;
	}

	vgt_ctrl_kobj->kset = vgt_kset;

	ret = kobject_init_and_add(vgt_ctrl_kobj, &vgt_ctrl_ktype, NULL, "control");
	if (ret) {
		ret = -EINVAL;
		goto kobj_fail;
	}

	ret = sysfs_create_bin_file(vgt_ctrl_kobj, &igd_mmio_attr);
	if (ret < 0) {
		ret = -EINVAL;
		goto kobj_fail;
	}

	for (i = 0; i < I915_MAX_PORTS; i++) {
		ret = kobject_init_and_add(&pgt->ports[i].kobj,
					      &vgt_pport_ktype,
					      vgt_ctrl_kobj,
					      "%s",
					      VGT_PORT_NAME(i));

		if (ret) {
			printk(KERN_WARNING
			       "%s: vgt pport kobject add error: %d\n",
			       __func__, ret);
			ret = -EINVAL;
			kobject_put(&pgt->ports[i].kobj);
			goto kobj_fail;
		}

		ret = sysfs_create_bin_file(&pgt->ports[i].kobj,
					    &port_edid_attr);
		if (ret < 0) {
			ret = -EINVAL;
			goto kobj_fail;
		}
	}

	return 0;

kobj_fail:
	for (; i > 0; i--) {
		kobject_put(&pgt->ports[i - 1].kobj);
	}
	kobject_put(vgt_ctrl_kobj);
ctrl_fail:
	kset_unregister(vgt_kset);
kset_fail:
	return ret;
}

void vgt_destroy_sysfs(void)
{
	sysfs_remove_bin_file(vgt_ctrl_kobj, &igd_mmio_attr);
	kobject_put(vgt_ctrl_kobj);
	kset_unregister(vgt_kset);
}
