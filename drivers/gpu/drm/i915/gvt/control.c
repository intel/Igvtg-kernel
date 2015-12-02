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

#define GVT_TYPE (';')
#define GVT_BASE 0xA0

#define GVT_CREATE_INSTANCE _IOW(GVT_TYPE, GVT_BASE + 0, struct gvt_instance_info)
#define GVT_DESTROY_INSTANCE _IOW(GVT_TYPE, GVT_BASE + 1, s32)

struct misc_device_client_info {
	struct pgt_device *pdev;
	struct vgt_device *vgt;
	struct mutex lock;
};

static int misc_device_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *dev = filp->private_data;
	struct pgt_device *pdev = container_of(dev, struct pgt_device, control.misc_device);
	struct misc_device_client_info *info = NULL;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		gvt_err("fail to allocate memory");
		return -ENOMEM;
	}

	info->pdev = pdev;
	mutex_init(&info->lock);

	filp->private_data = info;

	return 0;
}

static int misc_device_close(struct inode *inode, struct file *filp)
{
	struct misc_device_client_info *info = filp->private_data;

	if (info->vgt)
		gvt_destroy_instance(info->vgt);

	kfree(info);
	filp->private_data = NULL;
	return 0;
}

static long misc_device_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct misc_device_client_info *info = filp->private_data;
	struct gvt_instance_info param;
	int r = 0;

	if (copy_from_user(&param, (void *)arg, sizeof(param)))
		return -EFAULT;

	mutex_lock(&info->lock);
	switch(cmd) {
		case GVT_CREATE_INSTANCE:
			if (info->vgt) {
				gvt_err("instance has already been created");
				r = -EEXIST;
				goto out;
			}

			info->vgt = gvt_create_instance(info->pdev, &param);

			if (!info->vgt) {
				gvt_err("fail to create instance");
				r = -ENOMEM;
				goto out;
			}
			break;
		case GVT_DESTROY_INSTANCE:
			if (!info->vgt) {
				r = -ENODEV;
				goto out;
			}
			gvt_destroy_instance(info->vgt);
			info->vgt = NULL;
			break;
		default:
			r = -EINVAL;
			goto out;
	}
out:
	mutex_unlock(&info->lock);
	return r;
}

static struct file_operations misc_device_fops =
{
	.open = misc_device_open,
	.release = misc_device_close,
	.unlocked_ioctl = misc_device_ioctl,
	.compat_ioctl = misc_device_ioctl,
};

void clean_misc_device(struct pgt_device *pdev)
{
	struct miscdevice *dev = &pdev->control.misc_device;

	if (!list_empty(&dev->list))
		misc_deregister(dev);

	if (dev->name) {
		kfree(dev->name);
		dev->name = NULL;
	}
}

bool setup_misc_device(struct pgt_device *pdev)
{
	struct miscdevice *dev = &pdev->control.misc_device;
	unsigned long len;
	char name[32];

	len = snprintf(name, 32, "dri/gvt%d", pdev->id);
	dev->name = kzalloc(len + 1, GFP_KERNEL);
	if (!dev->name) {
		gvt_err("fail to allocate memory");
		return false;
	}

	dev->name = name;
	dev->minor = MISC_DYNAMIC_MINOR;
	dev->fops = &misc_device_fops;

	INIT_LIST_HEAD(&dev->list);

	if (misc_register(dev) < 0) {
		gvt_err("fail to register miscdevice");
		goto err;
	}

	return true;
err:
	clean_misc_device(pdev);
	return false;
}

bool gvt_setup_control_interface(struct pgt_device *pdev)
{
	bool r;

	r = setup_misc_device(pdev);
	if (!r) {
		gvt_err("fail to setup misc device node\n");
		return false;
	}

	return true;
}

void gvt_clean_control_interface(struct pgt_device *pdev)
{
	clean_misc_device(pdev);
}
