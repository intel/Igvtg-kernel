/*
 * Per-instance device node
 *
 * This is used for userland program to access MMIO of each vgt
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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include "vgt.h"

static int vgt_mmio_dev_pgfault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct file *vm_file = vma->vm_file;
	struct vgt_device *vgt = vm_file->private_data;
	struct page *page = NULL;
	unsigned long offset = 0;
	void *req_addr;

	offset = (vmf->pgoff + vma->vm_pgoff) << PAGE_SHIFT;

	if(offset >= VGT_MMIO_SPACE_SZ)
		return -EACCES;

	req_addr = vgt_vreg(vgt, offset);
	page = vmalloc_to_page(req_addr);

	ASSERT(page);

	get_page(page);
	vmf->page = page;

	return 0;
}

static const struct vm_operations_struct vgt_mmio_dev_vm_ops = {
	.fault = vgt_mmio_dev_pgfault,
};

static int vgt_mmio_dev_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int vgt_mmio_dev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	/* Since
	 * 1) remap_pfn_range() can only be used
	 *	  for memory allocated by kmalloc
	 * 2) Most of the time, user may only access part
	 *    of vreg space, so page fault handling for
	 *    page requested is more efficient
	 */
	vma->vm_ops = &vgt_mmio_dev_vm_ops;
	/* mark the page as read-only */
	pgprot_val(vma->vm_page_prot) &= ~_PAGE_RW;
	return 0;
}

static int vgt_mmio_dev_open(struct inode *inode, struct file *filp)
{
	unsigned int vgt_id = iminor(inode);
	ASSERT(vgt_id < VGT_MAX_VMS);

	if (!(default_device.device[vgt_id]))
		return -ENODEV;

	/* point to device(data) */
	filp->private_data = default_device.device[vgt_id];
	//ASSERT(filp->private_data == vgt_dom0);

	return 0;
}

static struct file_operations vgt_mmio_dev_fops = {
	.owner = THIS_MODULE,
	.open = vgt_mmio_dev_open,
	.mmap = vgt_mmio_dev_mmap,
	.llseek = no_llseek,
	.release = vgt_mmio_dev_release,
};


int vgt_init_mmio_device(struct pgt_device *pdev)
{
	int retval, devid;
	struct vgt_mmio_dev *mmio_dev = NULL;

	if ((retval = alloc_chrdev_region(&devid, 0,
			VGT_MAX_VMS, VGT_MMIO_DEV_NAME)) < 0) {
		vgt_err("failed to alloc chrdev region!\n");
		return retval;
	}

	if ((mmio_dev = vmalloc(sizeof(struct vgt_mmio_dev))) == NULL) {
		vgt_err("failed to alloc struct vgt_mmio_dev!\n");
		return -ENOMEM;
	}

	mmio_dev->devid_major = MAJOR(devid);
	mmio_dev->dev_name = VGT_MMIO_DEV_NAME;
	pdev->mmio_dev = mmio_dev;

	cdev_init(&mmio_dev->cdev, &vgt_mmio_dev_fops);

	if ((retval = cdev_add(&mmio_dev->cdev, devid, VGT_MAX_VMS)) < 0) {
		vgt_err("failed to add char device vgt_mmio_dev!\n");
		goto free_chrdev_region;
	}

	mmio_dev->class = class_create(THIS_MODULE,
			mmio_dev->dev_name);
	if (IS_ERR_OR_NULL(mmio_dev->class)) {
		vgt_err("mmio device class creation failed!\n");
		retval = -EINVAL;
		goto delete_cdev;
	}

	return 0;

delete_cdev:
	cdev_del(&mmio_dev->cdev);
free_chrdev_region:
	unregister_chrdev_region(
			MKDEV(mmio_dev->devid_major, 0),
			VGT_MAX_VMS);
	vfree(mmio_dev);

	return retval;
}

int vgt_create_mmio_dev(struct vgt_device *vgt)
{
	int vgt_id = vgt->vgt_id;
	struct vgt_mmio_dev *mmio_dev = vgt->pdev->mmio_dev;
	struct device *devnode;

	ASSERT(mmio_dev->class);
	devnode = device_create(mmio_dev->class,
			NULL,
			MKDEV(mmio_dev->devid_major, vgt_id),
			NULL,
			"%s%d",
			mmio_dev->dev_name,
			vgt_id);
	if (IS_ERR_OR_NULL(devnode))
		return -EINVAL;

	mmio_dev->devnode[vgt_id] = devnode;

	return 0;
}

void vgt_destroy_mmio_dev(struct vgt_device *vgt)
{
	struct vgt_mmio_dev *mmio_dev = vgt->pdev->mmio_dev;
	int vgt_id = vgt->vgt_id;

	if (!mmio_dev)
		return;

	if (mmio_dev->devnode[vgt_id] && mmio_dev->class) {
		device_destroy(mmio_dev->class,
				MKDEV(mmio_dev->devid_major, vgt_id));
		mmio_dev->devnode[vgt_id] = NULL;
	}
}

void vgt_cleanup_mmio_dev(struct pgt_device *pdev)
{
	struct vgt_mmio_dev *mmio_dev = pdev->mmio_dev;
	int id;

	if (!pdev->mmio_dev)
		return;

	for (id = 0; id < VGT_MAX_VMS; id++)
		if (pdev->device[id])
			vgt_destroy_mmio_dev(pdev->device[id]);

	if (mmio_dev->class) {
		class_destroy(mmio_dev->class);
		mmio_dev->class = NULL;
	}

	cdev_del(&mmio_dev->cdev);

	if (mmio_dev->devid_major != -EINVAL) {
		unregister_chrdev_region(
				MKDEV(mmio_dev->devid_major, 0),
				VGT_MAX_VMS);
		mmio_dev->devid_major = -EINVAL;
	}

	vfree(mmio_dev);
	pdev->mmio_dev = NULL;
}
