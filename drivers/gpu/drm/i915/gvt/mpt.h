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

#ifndef _GVT_MPT_H_
#define _GVT_MPT_H_

static inline unsigned long hypervisor_g2m_pfn(struct vgt_device *vgt,
	unsigned long g_pfn)
{
	return gvt_host.kdm->g2m_pfn(vgt->vm_id, g_pfn);
}

static inline int hypervisor_pause_domain(struct vgt_device *vgt)
{
	return gvt_host.kdm->pause_domain(vgt->vm_id);
}

static inline int hypervisor_shutdown_domain(struct vgt_device *vgt)
{
	return gvt_host.kdm->shutdown_domain(vgt->vm_id);
}

static inline int hypervisor_map_mfn_to_gpfn(struct vgt_device *vgt,
	unsigned long gpfn, unsigned long mfn, int nr, int map, enum map_type type)
{
	if (gvt_host.kdm && gvt_host.kdm->map_mfn_to_gpfn)
		return gvt_host.kdm->map_mfn_to_gpfn(vgt->vm_id, gpfn, mfn, nr, map, type);

	return 0;
}

static inline int hypervisor_set_trap_area(struct vgt_device *vgt,
	u64 start, u64 end, bool map)
{
	return gvt_host.kdm->set_trap_area(vgt, start, end, map);
}

static inline int hypervisor_set_wp_pages(struct vgt_device *vgt, guest_page_t *p)
{
	return gvt_host.kdm->set_wp_pages(vgt, p);
}

static inline int hypervisor_unset_wp_pages(struct vgt_device *vgt, guest_page_t *p)
{
	return gvt_host.kdm->unset_wp_pages(vgt, p);
}

static inline int hypervisor_detect_host(void)
{
	return gvt_host.kdm->detect_host();
}

static inline int hypervisor_virt_to_mfn(void *addr)
{
	return gvt_host.kdm->from_virt_to_mfn(addr);
}

static inline void *hypervisor_mfn_to_virt(int mfn)
{
	return gvt_host.kdm->from_mfn_to_virt(mfn);
}

static inline void hypervisor_inject_msi(struct vgt_device *vgt)
{
#define MSI_CAP_OFFSET 0x90	/* FIXME. need to get from cfg emulation */
#define MSI_CAP_CONTROL (MSI_CAP_OFFSET + 2)
#define MSI_CAP_ADDRESS (MSI_CAP_OFFSET + 4)
#define MSI_CAP_DATA	(MSI_CAP_OFFSET + 8)
#define MSI_CAP_EN 0x1

	char *cfg_space = &vgt->state.cfg.space[0];
	u16 control = *(u16 *)(cfg_space + MSI_CAP_CONTROL);
	u32 addr = *(u32 *)(cfg_space + MSI_CAP_ADDRESS);
	u16 data = *(u16 *)(cfg_space + MSI_CAP_DATA);
	int r;

	/* Do not generate MSI if MSIEN is disable */
	if (!(control & MSI_CAP_EN))
		return;

	/* FIXME: currently only handle one MSI format */
	ASSERT_NUM(!(control & 0xfffe), control);

	gvt_dbg(GVT_DBG_IRQ, "VM %d hvm injections. address (%x) data(%x)!",
			vgt->vm_id, addr, data);
	r = gvt_host.kdm->inject_msi(vgt->vm_id, addr, data);
	if (r < 0)
		gvt_err("VGT %d failed to inject vmsi", vgt->id);
}

static inline int hypervisor_hvm_init(struct vgt_device *vgt)
{
	if (gvt_host.kdm && gvt_host.kdm->hvm_init)
		return gvt_host.kdm->hvm_init(vgt);

	return 0;
}

static inline void hypervisor_hvm_exit(struct vgt_device *vgt)
{
	if (gvt_host.kdm && gvt_host.kdm->hvm_exit)
		gvt_host.kdm->hvm_exit(vgt);
}

static inline void *hypervisor_gpa_to_va(struct vgt_device *vgt, unsigned long gpa)
{
	if (!vgt->vm_id)
		return (char *)hypervisor_mfn_to_virt(gpa >> PAGE_SHIFT) + offset_in_page(gpa);

	return gvt_host.kdm->gpa_to_va(vgt, gpa);
}

static inline bool hypervisor_read_va(struct vgt_device *vgt, void *va,
		void *val, int len, int atomic)
{
	bool ret;

	if (!vgt->vm_id) {
		memcpy(val, va, len);
		return true;
	}

	ret = gvt_host.kdm->read_va(vgt, va, val, len, atomic);
	if (unlikely(!ret))
		gvt_err("VM(%d): read va failed, va: 0x%p, atomic : %s\n", vgt->vm_id,
				va, atomic ? "yes" : "no");

	return ret;
}

static inline bool hypervisor_write_va(struct vgt_device *vgt, void *va,
		void *val, int len, int atomic)
{
	bool ret;

	if (!vgt->vm_id) {
		memcpy(va, val, len);
		return true;
	}

	ret = gvt_host.kdm->write_va(vgt, va, val, len, atomic);
	if (unlikely(!ret))
		gvt_err("VM(%d): write va failed, va: 0x%p, atomic : %s\n", vgt->vm_id,
				va, atomic ? "yes" : "no");

	return ret;
}

#endif /* _GVT_MPT_H_ */
