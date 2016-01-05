/*
 * vGT header for mediate pass-through
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

#ifndef _VGT_MPT_H_
#define _VGT_MPT_H_

extern struct kernel_dm *vgt_pkdm;

static inline unsigned long hypervisor_g2m_pfn(struct vgt_device *vgt,
	unsigned long g_pfn)
{
	return vgt_pkdm->g2m_pfn(vgt->vm_id, g_pfn);
}

static inline int hypervisor_pause_domain(struct vgt_device *vgt)
{
	return vgt_pkdm->pause_domain(vgt->vm_id);
}

static inline int hypervisor_shutdown_domain(struct vgt_device *vgt)
{
	return vgt_pkdm->shutdown_domain(vgt->vm_id);
}

static inline int hypervisor_map_mfn_to_gpfn(struct vgt_device *vgt,
	unsigned long gpfn, unsigned long mfn, int nr, int map, enum map_type type)
{
	if (vgt_pkdm && vgt_pkdm->map_mfn_to_gpfn)
		return vgt_pkdm->map_mfn_to_gpfn(vgt->vm_id, gpfn, mfn, nr, map, type);

	return 0;
}

static inline int hypervisor_set_trap_area(struct vgt_device *vgt,
	uint64_t start, uint64_t end, bool map)
{
	return vgt_pkdm->set_trap_area(vgt, start, end, map);
}

static inline int hypervisor_set_wp_pages(struct vgt_device *vgt, guest_page_t *p)
{
	return vgt_pkdm->set_wp_pages(vgt, p);
}

static inline int hypervisor_unset_wp_pages(struct vgt_device *vgt, guest_page_t *p)
{
	return vgt_pkdm->unset_wp_pages(vgt, p);
}

static inline int hypervisor_check_host(void)
{
	return vgt_pkdm->check_host();
}

static inline int hypervisor_virt_to_mfn(void *addr)
{
	return vgt_pkdm->from_virt_to_mfn(addr);
}

static inline void *hypervisor_mfn_to_virt(int mfn)
{
	return vgt_pkdm->from_mfn_to_virt(mfn);
}

static inline void hypervisor_inject_msi(struct vgt_device *vgt)
{
#define MSI_CAP_CONTROL (msi_cap_offset + 2)
#define MSI_CAP_ADDRESS (msi_cap_offset + 4)
#define MSI_CAP_DATA	(msi_cap_offset + 8)
#define MSI_CAP_EN 0x1

	char *cfg_space = &vgt->state.cfg_space[0];
	u32 msi_cap_offset = IS_SKLPLUS(vgt->pdev) ? 0xAC : 0x90;

	u16 control = *(u16 *)(cfg_space + MSI_CAP_CONTROL);
	u32 addr = *(u32 *)(cfg_space + MSI_CAP_ADDRESS);
	u16 data = *(u16 *)(cfg_space + MSI_CAP_DATA);
	int r;

	/* Do not generate MSI if MSIEN is disable */
	if (!(control & MSI_CAP_EN))
		return;

	/* FIXME: currently only handle one MSI format */
	ASSERT_NUM(!(control & 0xfffe), control);

	vgt_dbg(VGT_DBG_IRQ, "vGT: VM(%d): hvm injections. address (%x) data(%x)!\n",
			vgt->vm_id, addr, data);
	r = vgt_pkdm->inject_msi(vgt->vm_id, addr, data);
	if (r < 0)
		vgt_err("vGT(%d): failed to inject vmsi\n", vgt->vgt_id);
}

static inline int hypervisor_hvm_init(struct vgt_device *vgt)
{
	if (vgt_pkdm && vgt_pkdm->hvm_init)
		return vgt_pkdm->hvm_init(vgt);

	return 0;
}

static inline void hypervisor_hvm_exit(struct vgt_device *vgt)
{
	if (vgt_pkdm && vgt_pkdm->hvm_exit)
		vgt_pkdm->hvm_exit(vgt);
}

static inline void *hypervisor_gpa_to_va(struct vgt_device *vgt, unsigned long gpa)
{
	if (!vgt->vm_id)
		return (char *)hypervisor_mfn_to_virt(gpa >> PAGE_SHIFT) + offset_in_page(gpa);

	return vgt_pkdm->gpa_to_va(vgt, gpa);
}

static inline bool hypervisor_read_va(struct vgt_device *vgt, void *va,
		void *val, int len, int atomic)
{
	bool ret;

	if (!vgt->vm_id) {
		memcpy(val, va, len);
		return true;
	}

	ret = vgt_pkdm->read_va(vgt, va, val, len, atomic);
	if (unlikely(!ret))
		vgt_err("VM(%d): read va failed, va: 0x%p, atomic : %s\n", vgt->vm_id,
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

	ret = vgt_pkdm->write_va(vgt, va, val, len, atomic);
	if (unlikely(!ret))
		vgt_err("VM(%d): write va failed, va: 0x%p, atomic : %s\n", vgt->vm_id,
				va, atomic ? "yes" : "no");

	return ret;
}

#endif /* _VGT_MPT_H_ */
