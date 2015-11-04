/*
 * Copyright(c) 2011-2015 Intel Corporation. All rights reserved.
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

#ifndef _VGT_HOST_MEDIATE_H_
#define _VGT_HOST_MEDIATE_H_

#define vgt_info(fmt, s...)	\
	do { printk(KERN_INFO "vGT info:(%s:%d) " fmt, __FUNCTION__, __LINE__, ##s); } while (0)

#define vgt_warn(fmt, s...)	\
	do { printk(KERN_WARNING "vGT warning:(%s:%d) " fmt, __FUNCTION__, __LINE__, ##s); } while (0)

#define vgt_err(fmt, s...)	\
	do { printk(KERN_ERR "vGT error:(%s:%d) " fmt, __FUNCTION__, __LINE__, ##s); } while (0)

struct pgt_device;
struct vgt_device;
extern struct pgt_device *pdev_default;
extern struct vgt_device *vgt_dom0;

bool vgt_native_mmio_read(u32 reg, void *val, int len, bool trace);
bool vgt_native_mmio_write(u32 reg, void *val, int len, bool trace);
bool vgt_native_gtt_read(u32 reg, void *val, int len);
bool vgt_native_gtt_write(u32 reg, void *val, int len);
void vgt_host_irq(int);
void vgt_host_irq_sync(void);
void tmp_vgt_clear_gtt(unsigned int);

void vgt_force_wake_get(void);
void vgt_force_wake_put(void);

uint64_t vgt_gttmmio_va(struct pgt_device *pdev, off_t reg);
uint64_t vgt_gttmmio_pa(struct pgt_device *pdev, off_t reg);
struct pci_dev *pgt_to_pci(struct pgt_device *pdev);

#endif /* _VGT_HOST_MEDIATE_H_ */
