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

static bool mmio_not_allow_read(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	gvt_err("[vgt %d]: reading MMIO reg 0x%x is not allowed.", vgt->id, offset);
	memset(p_data, 0, bytes);
	return true;
}

static bool mmio_not_allow_write(struct vgt_device *vgt,
		unsigned int offset, void *p_data, unsigned int bytes)
{
	gvt_err("[vgt %d]: writing MMIO reg 0x%x is not allowed.",
			vgt->id, offset);
	return true;
}

/* Fence MMIO handlers. */
static bool check_fence_mmio_access(struct vgt_device *vgt,
		unsigned int off, void *p_data, unsigned int bytes)
{
	unsigned long fence_num;

	ASSERT(off >= i915_mmio_reg_offset(FENCE_REG_GEN6_LO(0)) &&
			off <= i915_mmio_reg_offset(FENCE_REG_GEN6_HI(GVT_MAX_NUM_FENCES)));

	if (bytes > 8 && (off & (bytes - 1))) {
		gvt_err("[vgt %d] unsupported access pattern, off %x bytes %x",
				vgt->id, off, bytes);
		return false;
	}

	fence_num = (off - i915_mmio_reg_offset(FENCE_REG_GEN6_LO(0))) >> 3;

	if (fence_num >= vgt->state.gm.fence_sz)
		gvt_warn("[vgt %d] access unassigned fence reg %x, total: %x",
				vgt->id, off, vgt->state.gm.fence_sz);
	return true;
}

static bool fence_mmio_read(struct vgt_device *vgt, unsigned int off,
		void *p_data, unsigned int bytes)
{
	if (!check_fence_mmio_access(vgt, off, p_data, bytes))
		return false;

	return gvt_default_mmio_read(vgt, off, p_data, bytes);
}

static bool fence_mmio_write(struct vgt_device *vgt, unsigned int off,
		void *p_data, unsigned int bytes)
{
	if (!check_fence_mmio_access(vgt, off, p_data, bytes))
		return false;

	if (!gvt_default_mmio_write(vgt, off, p_data, bytes))
		return false;

	/* TODO: Check address space */

	/* FENCE registers are physically assigned, update! */
	if (bytes < 8)
		gvt_mmio_write(vgt->pdev, off + vgt->state.gm.fence_base * 8,
				__sreg(vgt, off));
	else
		gvt_mmio_write64(vgt->pdev, off + vgt->state.gm.fence_base * 8,
				__sreg64(vgt, off));
	return true;
}

static bool mt_force_wake_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	u32 data, mask, wake, old_wake, new_wake;

	data = *(u32*) p_data;

	/* bit 16-31: mask
	   bit 0-15: force wake
	   forcewake bit apply only if its mask bit is 1
	 */
	mask = data >> 16;
	wake = data & 0xFFFF;
	old_wake = __vreg(vgt, _FORCEWAKE_MT) & 0xFFFF;

	new_wake = (old_wake & ~mask) + (wake & mask);
	__vreg(vgt, _FORCEWAKE_MT) = (data & 0xFFFF0000) + new_wake;
	__vreg(vgt, _FORCEWAKE_ACK_HSW) = new_wake;

	return true;
}

static bool gdrst_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	u32 data = *(u32 *)p_data;

	if (data & GEN6_GRDOM_FULL) {
		gvt_info("VM %d request Full GPU Reset\n", vgt->vm_id);
	}

	if (data & GEN6_GRDOM_RENDER) {
		gvt_info("VM %d request GPU Render Reset\n", vgt->vm_id);
	}

	if (data & GEN6_GRDOM_MEDIA) {
		gvt_info("VM %d request GPU Media Reset\n", vgt->vm_id);
	}

	if (data & GEN6_GRDOM_BLT) {
		gvt_info("VM %d request GPU BLT Reset\n", vgt->vm_id);
	}

	return true;
}

static bool ring_mode_write(struct vgt_device *vgt, unsigned int off,
		void *p_data, unsigned int bytes)
{
	return true;
}

static bool pvinfo_read(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	bool rc = gvt_default_mmio_read(vgt, offset, p_data, bytes);
	bool invalid_read = false;

	switch (offset) {
		case _vgtif_reg(magic) ... _vgtif_reg(vgt_id):
			if (offset + bytes > _vgtif_reg(vgt_id) + 4)
			invalid_read = true;
			break;

			case _vgtif_reg(avail_rs.mappable_gmadr.base) ...
				_vgtif_reg(avail_rs.fence_num):
					if (offset + bytes >
							_vgtif_reg(avail_rs.fence_num) + 4)
					invalid_read = true;
			break;

			case _vgtif_reg(drv_version_major) ...
				_vgtif_reg(min_fence_num):
					if (offset + bytes > _vgtif_reg(min_fence_num) + 4)
					invalid_read = true;
			break;
		case _vgtif_reg(v2g_notify):
			/* set cursor setting here.  For example:
			 *   *((unsigned int *)p_data)) = VGT_V2G_SET_SW_CURSOR;
			 */
			break;
		case _vgtif_reg(vgt_caps):
			break;
		default:
			invalid_read = true;
			break;
	}

	if (invalid_read)
		gvt_warn("invalid pvinfo read: [%x:%x] = %x!!!\n",
				offset, bytes, *(u32 *)p_data);

	return rc;
}

static bool pvinfo_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	u32 val = *(u32 *)p_data;
	u32 min;
	bool rc = true;

	switch (offset) {
		case _vgtif_reg(min_low_gmadr):
			min = val;
			if (vgt->state.gm.aperture_sz < min) {
				gvt_err("VM(%d): aperture size(%llx) is less than"
						"its driver's minimum requirement(%x)!\n",
						vgt->vm_id, vgt->state.gm.aperture_sz, min);
				rc = false;
			}
			break;
		case _vgtif_reg(min_high_gmadr):
			min = val;
			if (vgt->state.gm.gm_sz - vgt->state.gm.aperture_sz < min) {
				gvt_err("VM(%d): hiden gm size(%llx) is less than"
						"its driver's minimum requirement(%x)!\n",
						vgt->vm_id, vgt->state.gm.gm_sz - vgt->state.gm.aperture_sz,
						min);
				rc = false;
			}
			break;
		case _vgtif_reg(min_fence_num):
			min = val;
			if (vgt->state.gm.fence_sz < min) {
				gvt_err("VM(%d): fence size(%x) is less than"
						"its drivers minimum requirement(%x)!\n",
						vgt->vm_id, vgt->state.gm.fence_sz, min);
				rc = false;
			}
			break;
		case _vgtif_reg(g2v_notify):
				if (val == VGT_G2V_PPGTT_L3_PAGE_TABLE_CREATE) {
					rc = gvt_g2v_create_ppgtt_mm(vgt, 3);
				} else if (val == VGT_G2V_PPGTT_L3_PAGE_TABLE_DESTROY) {
					rc = gvt_g2v_destroy_ppgtt_mm(vgt, 3);
				} else if (val == VGT_G2V_PPGTT_L4_PAGE_TABLE_CREATE) {
					rc = gvt_g2v_create_ppgtt_mm(vgt, 4);
				} else if (val == VGT_G2V_PPGTT_L4_PAGE_TABLE_DESTROY) {
					rc = gvt_g2v_destroy_ppgtt_mm(vgt, 4);
				} else {
					gvt_warn("Invalid PV notification. %x\n", val);
				}
			break;
		case _vgtif_reg(pdp[0].lo):
		case _vgtif_reg(pdp[0].hi):
		case _vgtif_reg(pdp[1].lo):
		case _vgtif_reg(pdp[1].hi):
		case _vgtif_reg(pdp[2].lo):
		case _vgtif_reg(pdp[2].hi):
		case _vgtif_reg(pdp[3].lo):
		case _vgtif_reg(pdp[3].hi):
		case _vgtif_reg(execlist_context_descriptor_lo):
		case _vgtif_reg(execlist_context_descriptor_hi):
			break;

		default:
			/* keep rc's default value: true.
			 * NOTE: returning false will crash the VM.
			 */
			gvt_warn("invalid pvinfo write: [%x:%x] = %x!!!\n",
					offset, bytes, val);
			break;
	}

	if (rc == true)
		rc = gvt_default_mmio_write(vgt, offset, p_data, bytes);
	return rc;
}

bool fpga_dbg_write(struct vgt_device *vgt, unsigned int reg,
        void *p_data, unsigned int bytes)
{
        u32 v = *(u32 *)p_data;

	if (v & FPGA_DBG_RM_NOCLAIM)
		v &= ~ FPGA_DBG_RM_NOCLAIM;

        return gvt_default_mmio_write(vgt, reg, &v, bytes);
}

struct gvt_reg_info gvt_general_reg_info[] = {
	/* Interrupt registers - GT */
	{_RING_IMR(RENDER_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, gvt_reg_imr_handler},
	{_RING_IMR(BLT_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, gvt_reg_imr_handler},
	{_RING_IMR(GEN6_BSD_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, gvt_reg_imr_handler},
	{_RING_IMR(VEBOX_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, gvt_reg_imr_handler},

	/* Interrupt registers - PCH */
	{_SDEIMR, 4, F_VIRT, 0, D_ALL, NULL, gvt_reg_imr_handler},
	{_SDEIER, 4, F_VIRT, 0, D_ALL, NULL, gvt_reg_ier_handler},
	{_SDEIIR, 4, F_VIRT, 0, D_ALL, NULL, gvt_reg_iir_handler},
	{_SDEISR, 4, F_VIRT, 0, D_ALL, NULL, gvt_reg_isr_write},

	/* -------render regs---------- */
	{_RING_HWSTAM(RENDER_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_RING_HWSTAM(BLT_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_RING_HWSTAM(GEN6_BSD_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_RING_HWSTAM(VEBOX_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},

	{_RENDER_HWS_PGA_GEN7, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
	{_BSD_HWS_PGA_GEN7, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
	{_BLT_HWS_PGA_GEN7, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
	{_VEBOX_HWS_PGA_GEN7, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},

	/* maybe an error in Linux driver. meant for VCS_HWS_PGA */
	{_REG_RCS_EXCC, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_REG_VCS_EXCC, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_REG_BCS_EXCC, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_REG_RCS_UHPTR, 4, F_RDR_HWSTS, 0, D_ALL, NULL, NULL},
	{_REG_VCS_UHPTR, 4, F_RDR_HWSTS, 0, D_ALL, NULL, NULL},
	{_REG_BCS_UHPTR, 4, F_RDR_HWSTS, 0, D_ALL, NULL, NULL},
	{_REG_VECS_UHPTR, 4, F_RDR_HWSTS, 0, D_ALL, NULL, NULL},
	{_REG_RCS_BB_PREEMPT_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},

	{0x12198, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},

	{_RING_TAIL(RENDER_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_RING_HEAD(RENDER_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_RING_START(RENDER_RING_BASE), 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL,
		NULL, NULL},
	{_RING_CTL(RENDER_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_RING_ACTHD(RENDER_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},

	{_RING_TAIL(GEN6_BSD_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_RING_HEAD(GEN6_BSD_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_RING_START(GEN6_BSD_RING_BASE), 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL,
		NULL, NULL},
	{_RING_CTL(GEN6_BSD_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_RING_ACTHD(GEN6_BSD_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},

	{_RING_TAIL(BLT_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_RING_HEAD(BLT_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_RING_START(BLT_RING_BASE), 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL,
		NULL, NULL},
	{_RING_CTL(BLT_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_RING_ACTHD(BLT_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},

	{_RING_TAIL(VEBOX_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_RING_HEAD(VEBOX_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_RING_START(VEBOX_RING_BASE), 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL,
		NULL, NULL},
	{_RING_CTL(VEBOX_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_RING_ACTHD(VEBOX_RING_BASE), 4, F_RDR, 0, D_ALL, NULL, NULL},

	{GVT_RING_MODE(RENDER_RING_BASE), 4, F_RDR_MODE, 0, D_ALL, NULL, ring_mode_write},
	{GVT_RING_MODE(BLT_RING_BASE), 4, F_RDR_MODE, 0, D_ALL, NULL, ring_mode_write},
	{GVT_RING_MODE(GEN6_BSD_RING_BASE), 4, F_RDR_MODE, 0, D_ALL, NULL, ring_mode_write},
	{GVT_RING_MODE(VEBOX_RING_BASE), 4, F_RDR_MODE, 0, D_ALL, NULL, ring_mode_write},

	{_RING_MI_MODE(RENDER_RING_BASE), 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
	{_RING_MI_MODE(BLT_RING_BASE), 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
	{_RING_MI_MODE(GEN6_BSD_RING_BASE), 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
	{_RING_MI_MODE(VEBOX_RING_BASE), 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},

	{_RING_INSTPM(RENDER_RING_BASE), 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
	{_RING_INSTPM(BLT_RING_BASE), 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
	{_RING_INSTPM(GEN6_BSD_RING_BASE), 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
	{_RING_INSTPM(VEBOX_RING_BASE), 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},

	{_GEN7_GT_MODE, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
	{_CACHE_MODE_0_GEN7, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
	{_CACHE_MODE_1, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
	{_REG_RCS_BB_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
	{_REG_VCS_BB_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
	{_REG_BCS_BB_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
	{_REG_VECS_BB_ADDR, 4, F_RDR_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},

	{0x2050, 4, F_PT, 0, D_ALL, NULL, NULL},
	{0x12050, 4, F_PT, 0, D_ALL, NULL, NULL},
	{0x22050, 4, F_PT, 0, D_ALL, NULL, NULL},
	{0x1A050, 4, F_PT, 0, D_ALL, NULL, NULL},

	{0x20dc, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
	{__3D_CHICKEN3, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
	{0x2088, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
	{0x20e4, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
	{_REG_VFSKPD, 4, F_RDR_MODE, 0, D_ALL, NULL, NULL},
	{_GAM_ECOCHK, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_GEN7_COMMON_SLICE_CHICKEN1, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_COMMON_SLICE_CHICKEN2, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{0x9030, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{0x20a0, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_REG_RCS_TIMESTAMP, 8, F_PT, 0, D_ALL, NULL, NULL},
	{_REG_VCS_TIMESTAMP, 8, F_PT, 0, D_ALL, NULL, NULL},
	{0x1a358, 8, F_PT, 0, D_ALL, NULL, NULL},
	{_REG_BCS_TIMESTAMP, 8, F_PT, 0, D_ALL, NULL, NULL},
	{0x2420, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{0x2430, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{0x2434, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{0x2438, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{0x243c, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{0x7018, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{0xe184, 4, F_RDR, 0, D_ALL, NULL, NULL},

	/* -------display regs---------- */
	{0x60220, 0x20, F_DPY, 0, D_ALL, NULL, NULL},
	{0x602a0, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{0x65050, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x650b4, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{VGA_CR_INDEX_MDA, 1, F_DPY, 0, D_ALL, NULL, NULL},
	{VGA_ST01_MDA, 1, F_DPY, 0, D_ALL, NULL, NULL},
	{VGA_AR_INDEX, 1, F_DPY, 0, D_ALL, NULL, NULL},
	{VGA_DACMASK, 1, F_DPY, 0, D_ALL, NULL, NULL},
	{VGA_MSR_READ, 1, F_DPY, 0, D_ALL, NULL, NULL},
	{_VGA0, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_VGA1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_VGA_PD, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{0x42080, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{0xc4040, 4, F_VIRT, 0, D_ALL, NULL, NULL},

	{_DERRMR, 4, F_VIRT, 0, D_ALL, NULL, NULL},

	{GVT_CURSURF(PIPE_A), 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
	{GVT_CURCNTR(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_CURPOS(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_CURSURFLIVE(PIPE_A), 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, NULL,
		mmio_not_allow_write},

	{GVT_CURSURF(PIPE_B), 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
	{GVT_CURCNTR(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_CURPOS(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_CURSURFLIVE(PIPE_B), 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, NULL,
		mmio_not_allow_write},

	{GVT_CURSURF(PIPE_C), 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, NULL, NULL},
	{GVT_CURCNTR(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_CURPOS(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_CURSURFLIVE(PIPE_C), 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, NULL,
		mmio_not_allow_write},

	{_REG_CURAPALET_0, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_CURAPALET_1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_CURAPALET_2, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_CURAPALET_3, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_LGC_PALETTE_A, 4 * 256, F_DPY, 0, D_ALL, NULL, NULL},
	{_LGC_PALETTE_B, 4 * 256, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_LGC_PALETTE_C, 4 * 256, F_DPY, 0, D_ALL, NULL, NULL},

	{0x701b0, 4, F_VIRT, 0, D_ALL, NULL, NULL},

	{GVT_HTOTAL(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_HBLANK(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_HSYNC(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_VTOTAL(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_VBLANK(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_VSYNC(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_PIPESRC(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_BCLRPAT(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_VSYNCSHIFT(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},

	{GVT_HTOTAL(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_HBLANK(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_HSYNC(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_VTOTAL(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_VBLANK(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_VSYNC(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_PIPESRC(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_BCLRPAT(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_VSYNCSHIFT(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},

	{GVT_HTOTAL(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_HBLANK(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_HSYNC(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_VTOTAL(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_VBLANK(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_VSYNC(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_PIPESRC(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_BCLRPAT(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_VSYNCSHIFT(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},

	{0x6F000, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x6F004, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x6F008, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x6F00C, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x6F010, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x6F014, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x6F028, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x6F030, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x6F034, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x6F040, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x6F044, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_PIPEA_DATA_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PIPEA_DATA_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PIPEA_LINK_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PIPEA_LINK_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_PIPEB_DATA_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PIPEB_DATA_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PIPEB_LINK_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PIPEB_LINK_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_REG_PIPEC_DATA_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PIPEC_DATA_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PIPEC_LINK_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PIPEC_LINK_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_PFA_CTL_1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PFA_WIN_SZ, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PFA_WIN_POS, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PFB_CTL_1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PFB_WIN_SZ, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PFB_WIN_POS, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PF_CTL_2, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PF_WIN_SZ_2, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PF_WIN_POS_2, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_WM0_PIPEA_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_WM0_PIPEB_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_WM0_PIPEC_IVB, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_WM1_LP_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_WM2_LP_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_WM3_LP_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_WM1S_LP_ILK, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_WM2S_LP_IVB, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_WM3S_LP_IVB, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_REG_HISTOGRAM_THRSH, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_BLC_PWM_CPU_CTL2, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_BLC_PWM_CPU_CTL, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_BLC_PWM_PCH_CTL1, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_BLC_PWM_PCH_CTL2, 4, F_DOM0, 0, D_ALL, NULL, NULL},

	{_PCH_TRANS_HTOTAL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANS_HBLANK_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANS_HSYNC_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANS_VTOTAL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANS_VBLANK_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANS_VSYNC_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANS_VSYNCSHIFT_A, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_PCH_TRANS_HTOTAL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANS_HBLANK_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANS_HSYNC_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANS_VTOTAL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANS_VBLANK_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANS_VSYNC_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANS_VSYNCSHIFT_B, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_PCH_TRANSA_DATA_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANSA_DATA_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANSA_DATA_M2, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANSA_DATA_N2, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANSA_LINK_M1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANSA_LINK_N1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANSA_LINK_M2, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_TRANSA_LINK_N2, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_VIDEO_DIP_CTL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_VIDEO_DIP_DATA_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_VIDEO_DIP_GCP_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_TRANS_DP_CTL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_VIDEO_DIP_CTL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_VIDEO_DIP_DATA_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_VIDEO_DIP_GCP_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_TRANS_DP_CTL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_TRANSC_VIDEO_DIP_CTL, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_TRANSC_VIDEO_DIP_DATA, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_TRANSC_VIDEO_DIP_GCP, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_TRANS_DP_CTL_C, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_FDI_RXA_MISC, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_FDI_RXB_MISC, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_FDI_RXA_TUSIZE1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_FDI_RXA_TUSIZE2, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_FDI_RXB_TUSIZE1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_FDI_RXB_TUSIZE2, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_PCH_LVDS, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_DPLL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_DPLL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_FPA0, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_FPA1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_FPB0, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_FPB1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_DREF_CONTROL, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_RAWCLK_FREQ, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_DPLL_SEL, 4, F_DPY, 0, D_ALL, NULL, NULL},
	/* Linux defines as PP_ON_DEPLAY/PP_OFF_DELAY. Not in spec */
	{0x61208, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x6120c, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_PP_ON_DELAYS, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_PP_OFF_DELAYS, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_FUSE_STRAP, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_DIGITAL_PORT_HOTPLUG_CNTRL, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_DISP_ARB_CTL, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_DISP_ARB_CTL2, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_ILK_DISPLAY_CHICKEN1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_ILK_DISPLAY_CHICKEN2, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_ILK_DSPCLK_GATE_D, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_TRANSA_CHICKEN1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_TRANSB_CHICKEN1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_SOUTH_DSPCLK_GATE_D, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_TRANSA_CHICKEN2, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_TRANSB_CHICKEN2, 4, F_DPY, 0, D_ALL, NULL, NULL},
	/*
	 * framebuffer compression is disabled for now
	 * until it's handled at display context switch
	 * and we figure out how stolen memory should be virtualized (FBC needs use
	 * stolen memory).
	 */
	{_REG_DPFC_CB_BASE, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{_REG_DPFC_CONTROL, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{_REG_DPFC_RECOMP_CTL, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{_REG_DPFC_CPU_FENCE_OFFSET, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{_REG_DPFC_CONTROL_SA, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{_REG_DPFC_CPU_FENCE_OFFSET_SA, 4, F_VIRT, 0, D_ALL, NULL, NULL},

	{_IPS_CTL, 4, F_DOM0, 0, D_ALL, NULL, NULL},

	{_REG_CSC_A_COEFFICIENTS, 4*6, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_CSC_A_MODE, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PRECSC_A_HIGH_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PRECSC_A_MEDIUM_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PRECSC_A_LOW_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_REG_CSC_B_COEFFICIENTS, 4*6, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_CSC_B_MODE, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PRECSC_B_HIGH_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PRECSC_B_MEDIUM_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PRECSC_B_LOW_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_REG_CSC_C_COEFFICIENTS, 4*6, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_CSC_C_MODE, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PRECSC_C_HIGH_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PRECSC_C_MEDIUM_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PRECSC_C_LOW_COLOR_CHANNEL_OFFSET, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{0x60110, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x61110, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x70400, 0x40, F_DPY, 0, D_ALL, NULL, NULL},
	{0x71400, 0x40, F_DPY, 0, D_ALL, NULL, NULL},
	{0x72400, 0x40, F_DPY, 0, D_ALL, NULL, NULL},

	{0x70440, 0xc, F_DPY, 0, D_ALL, NULL, NULL},
	{0x71440, 0xc, F_DPY, 0, D_ALL, NULL, NULL},
	{0x72440, 0xc, F_DPY, 0, D_ALL, NULL, NULL},

	{0x7044c, 0xc, F_DPY, 0, D_ALL, NULL, NULL},
	{0x7144c, 0xc, F_DPY, 0, D_ALL, NULL, NULL},
	{0x7244c, 0xc, F_DPY, 0, D_ALL, NULL, NULL},

	{_PIPE_WM_LINETIME_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PIPE_WM_LINETIME_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x45278, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_SPLL_CTL, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_WRPLL_CTL1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_WRPLL_CTL2, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PORT_CLK_SEL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PORT_CLK_SEL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PORT_CLK_SEL_DDIC, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PORT_CLK_SEL_DDID, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_PORT_CLK_SEL_DDIE, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_TRANS_CLK_SEL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_TRANS_CLK_SEL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_TRANS_CLK_SEL_C, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x46408, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x46508, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x49040, 0xc, F_DPY, 0, D_ALL, NULL, NULL},
	{0x49140, 0xc, F_DPY, 0, D_ALL, NULL, NULL},
	{0x49240, 0xc, F_DPY, 0, D_ALL, NULL, NULL},
	{0x49080, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x49090, 0x14, F_DPY, 0, D_ALL, NULL, NULL},
	{0x49180, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x49190, 0x14, F_DPY, 0, D_ALL, NULL, NULL},
	{0x49280, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x49290, 0x14, F_DPY, 0, D_ALL, NULL, NULL},
	{0x4A400, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x4A480, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x4AC00, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x4AC80, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x4B400, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x4B480, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{0x6002C, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_HSW_VIDEO_DIP_CTL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_HSW_VIDEO_DIP_CTL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_HSW_VIDEO_DIP_CTL_C, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_REG_HSW_VIDEO_DIP_CTL_EDP, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_SFUSE_STRAP, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_SBI_ADDR, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PIXCLK_GATE, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{0x64E60, 0x50, F_DPY, 0, D_ALL, NULL, NULL},
	{0x64Ec0, 0x50, F_DPY, 0, D_ALL, NULL, NULL},
	{0x64F20, 0x50, F_DPY, 0, D_ALL, NULL, NULL},
	{0x64F80, 0x50, F_DPY, 0, D_ALL, NULL, NULL},
	{_HSW_AUD_CONFIG_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x650C0, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_TRANS_DDI_FUNC_CTL_A, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_TRANS_DDI_FUNC_CTL_B, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_TRANS_DDI_FUNC_CTL_C, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_TRANS_DDI_FUNC_CTL_EDP, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_TRANSA_MSA_MISC, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_TRANSB_MSA_MISC, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_TRANSC_MSA_MISC, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{0x6F410, 4, F_DPY, 0, D_ALL, NULL, NULL},

	/* -------others---------- */
	{_FORCEWAKE_MT, 4, F_VIRT, 0, D_ALL, NULL, mt_force_wake_write},
	{_FORCEWAKE_ACK_HSW, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{_ECOBUS, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RC_CONTROL, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RC_STATE, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RPNSWREQ, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RC_VIDEO_FREQ, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RP_DOWN_TIMEOUT, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RP_INTERRUPT_LIMITS, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RPSTAT1, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RP_CONTROL, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RP_UP_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RP_DOWN_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RP_CUR_UP_EI, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RP_CUR_UP, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RP_PREV_UP, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RP_CUR_DOWN_EI, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RP_CUR_DOWN, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RP_PREV_DOWN, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RP_UP_EI, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RP_DOWN_EI, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RP_IDLE_HYSTERSIS, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RC1_WAKE_RATE_LIMIT, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RC6_WAKE_RATE_LIMIT, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RC6pp_WAKE_RATE_LIMIT, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RC_EVALUATION_INTERVAL, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RC_IDLE_HYSTERSIS, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RC_SLEEP, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RC1e_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RC6_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RC6p_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_RC6pp_THRESHOLD, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_PMINTRMSK, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_GDRST, 4, F_DOM0, 0, D_ALL, NULL, gdrst_mmio_write},

	{0x100000, 0x80, F_VIRT, 0, D_ALL, fence_mmio_read, fence_mmio_write},
	{VGT_PVINFO_PAGE, VGT_PVINFO_SIZE, F_VIRT, 0, D_ALL, pvinfo_read, pvinfo_write},

	/* TODO: MCHBAR, suppose read-only */
	{MCHBAR_MIRROR_BASE_SNB, 0x40000, F_VIRT, 0, D_ALL, NULL, NULL},

	{_TILECTL, 4, F_DOM0, 0, D_ALL, NULL, NULL},

	{_GEN6_UCGCTL1, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_UCGCTL2, 4, F_DOM0, 0, D_ALL, NULL, NULL},

	{_REG_SWF, 0x90, F_VIRT, 0, D_ALL, NULL, NULL},

	{_GEN6_PCODE_MAILBOX, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN6_PCODE_DATA, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{0x13812c, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GEN7_ERR_INT, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0x120010, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0x9008, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{_GFX_FLSH_CNTL_GEN6, 4, F_PT, 0, D_ALL, NULL, NULL},

	/* -------un-categorized regs--------- */
	{0x3c, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{0x860, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	/* no definition on this. from Linux */
	{_ECOSKPD, 4, F_PT, 0, D_ALL, NULL, NULL},
	{0x121d0, 4, F_PT, 0, D_ALL, NULL, NULL},
	{_GEN6_BLITTER_ECOSKPD, 4, F_PT, 0, D_ALL, NULL, NULL},
	{0x41d0, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{_GAC_ECO_BITS, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{_REG_2D_CG_DIS, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{_REG_3D_CG_DIS, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{_REG_3D_CG_DIS2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0x7118, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0x7180, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0x7408, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0x7c00, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{_GEN6_MBCTL, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0x911c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0x9120, 4, F_VIRT, 0, D_ALL, NULL, NULL},

	{_GAB_CTL, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0x48800, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xce044, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xe6500, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xe6504, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xe6600, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xe6604, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xe6700, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xe6704, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xe6800, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xe6804, 4, F_VIRT, 0, D_ALL, NULL, NULL},

	{_REG_SUPER_QUEUE_CONFIG, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xec008, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xec00c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xec008+0x18, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xec00c+0x18, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xec008+0x18*2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xec00c+0x18*2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xec008+0x18*3, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xec00c+0x18*3, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xec408, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xec40c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xec408+0x18, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xec40c+0x18, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xec408+0x18*2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xec40c+0x18*2, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xec408+0x18*3, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xec40c+0x18*3, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xfc810, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xfc81c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xfc828, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xfc834, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xfcc00, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xfcc0c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xfcc18, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xfcc24, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xfd000, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xfd00c, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xfd018, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xfd024, 4, F_VIRT, 0, D_ALL, NULL, NULL},
	{0xfd034, 4, F_VIRT, 0, D_ALL, NULL, NULL},

	/* MAXCNT means max idle count */
	{_REG_RC_PWRCTX_MAXCNT, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{0x12054, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{0x22054, 4, F_DOM0, 0, D_ALL, NULL, NULL},
	{0x1A054, 4, F_DOM0, 0, D_ALL, NULL, NULL},

	{0x44070, 4, F_DOM0, 0, D_ALL, NULL, NULL},

	{_FPGA_DBG, 4, F_VIRT, 0, D_ALL, NULL, fpga_dbg_write},
	{_GEN6_GT_THREAD_STATUS_REG, 4, F_VIRT, 0, D_ALL, NULL, NULL},

	/*command accessed registers, supplement for reg audit in cmd parser*/
	{0x2178, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{0x217c, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{0x12178, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{0x1217c, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_BCS_SWCTRL, 4, F_RDR, 0, D_ALL, NULL, NULL},
	{_HS_INVOCATION_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
	{_DS_INVOCATION_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
	{_IA_VERTICES_COUNT  , 8, F_RDR, 0, D_ALL, NULL, NULL},
	{_IA_PRIMITIVES_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
	{_VS_INVOCATION_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
	{_GS_INVOCATION_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
	{_GS_PRIMITIVES_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
	{_CL_INVOCATION_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
	{_CL_PRIMITIVES_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
	{_PS_INVOCATION_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},
	{_PS_DEPTH_COUNT, 8, F_RDR, 0, D_ALL, NULL, NULL},

	/* BDW */
	{0xe100, 4, F_RDR, 0, D_ALL, NULL, NULL},
};

struct gvt_reg_info gvt_broadwell_reg_info[] = {
	/* Interrupt registers - GT */
	{_RING_IMR(GEN8_BSD2_RING_BASE), 4, F_RDR, 0, D_BDW_PLUS, NULL, gvt_reg_imr_handler},

	/* Interrupt registers - BDW */
	{_REG_GT_IMR(0), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_imr_handler},
	{_REG_GT_IER(0), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_ier_handler},
	{_REG_GT_IIR(0), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_iir_handler},
	{_REG_GT_ISR(0), 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_GT_IMR(1), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_imr_handler},
	{_REG_GT_IER(1), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_ier_handler},
	{_REG_GT_IIR(1), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_iir_handler},
	{_REG_GT_ISR(1), 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_GT_IMR(2), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_imr_handler},
	{_REG_GT_IER(2), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_ier_handler},
	{_REG_GT_IIR(2), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_iir_handler},
	{_REG_GT_ISR(2), 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_GT_IMR(3), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_imr_handler},
	{_REG_GT_IER(3), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_ier_handler},
	{_REG_GT_IIR(3), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_iir_handler},
	{_REG_GT_ISR(3), 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_DE_PIPE_IMR(PIPE_A), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_imr_handler},
	{_REG_DE_PIPE_IER(PIPE_A), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_ier_handler},
	{_REG_DE_PIPE_IIR(PIPE_A), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_iir_handler},
	{_REG_DE_PIPE_ISR(PIPE_A), 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_DE_PIPE_IMR(PIPE_B), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_imr_handler},
	{_REG_DE_PIPE_IER(PIPE_B), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_ier_handler},
	{_REG_DE_PIPE_IIR(PIPE_B), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_iir_handler},
	{_REG_DE_PIPE_ISR(PIPE_B), 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_DE_PIPE_IMR(PIPE_C), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_imr_handler},
	{_REG_DE_PIPE_IER(PIPE_C), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_ier_handler},
	{_REG_DE_PIPE_IIR(PIPE_C), 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_iir_handler},
	{_REG_DE_PIPE_ISR(PIPE_C), 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_GEN8_DE_PORT_IMR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_imr_handler},
	{_GEN8_DE_PORT_IER, 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_ier_handler},
	{_GEN8_DE_PORT_IIR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_iir_handler},
	{_GEN8_DE_PORT_ISR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_GEN8_DE_MISC_IMR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_imr_handler},
	{_GEN8_DE_MISC_IER, 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_ier_handler},
	{_GEN8_DE_MISC_IIR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_iir_handler},
	{_GEN8_DE_MISC_ISR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_GEN8_PCU_IMR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_imr_handler},
	{_GEN8_PCU_IER, 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_ier_handler},
	{_GEN8_PCU_IIR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_iir_handler},
	{_GEN8_PCU_ISR, 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_GEN8_MASTER_IRQ, 4, F_VIRT, 0, D_BDW_PLUS, NULL, gvt_reg_master_irq_handler},

	/* -------render regs---------- */
	{_RING_HWSTAM(GEN8_BSD2_RING_BASE), 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},

	/* maybe an error in Linux driver. meant for VCS_HWS_PGA */
	{_REG_VCS2_UHPTR, 4, F_RDR_HWSTS, 0, D_BDW_PLUS, NULL, NULL},

	{_RING_TAIL(GEN8_BSD2_RING_BASE), 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
	{_RING_HEAD(GEN8_BSD2_RING_BASE), 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
	{_RING_START(GEN8_BSD2_RING_BASE), 4, F_RDR_ADRFIX, 0xFFFFF000, D_BDW_PLUS,
		NULL, NULL},
	{_RING_CTL(GEN8_BSD2_RING_BASE), 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
	{_RING_ACTHD(GEN8_BSD2_RING_BASE), 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
	{GVT_RING_MODE(GEN8_BSD2_RING_BASE), 4, F_RDR_MODE, 0, D_BDW_PLUS, NULL, ring_mode_write},
	{_RING_MI_MODE(GEN8_BSD2_RING_BASE), 4, F_RDR_MODE, 0, D_BDW_PLUS, NULL, NULL},
	{_RING_INSTPM(GEN8_BSD2_RING_BASE), 4, F_RDR_MODE, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_RCS_ACTHD_UDW, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_BCS_ACTHD_UDW, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VCS_ACTHD_UDW, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VCS2_ACTHD_UDW, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VECS_ACTHD_UDW, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},

	/* TODO: need a handler */
	{0x1c050, 4, F_PT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_VCS2_TIMESTAMP, 8, F_PT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_RCS_EXECLIST_SUBMITPORT, 4, F_VIRT, 0, D_BDW_PLUS,
		mmio_not_allow_read, NULL},
	{_REG_VCS_EXECLIST_SUBMITPORT, 4, F_VIRT, 0, D_BDW_PLUS,
		mmio_not_allow_read, NULL},
	{_REG_VECS_EXECLIST_SUBMITPORT, 4, F_VIRT, 0, D_BDW_PLUS,
		mmio_not_allow_read, NULL},
	{_REG_VCS2_EXECLIST_SUBMITPORT, 4, F_VIRT, 0, D_BDW_PLUS,
		mmio_not_allow_read, NULL},
	{_REG_BCS_EXECLIST_SUBMITPORT, 4, F_VIRT, 0, D_BDW_PLUS,
		mmio_not_allow_read, NULL},

	{_REG_RCS_EXECLIST_STATUS, 8, F_RDR, 0, D_BDW_PLUS, NULL,
		mmio_not_allow_write},
	{_REG_VCS_EXECLIST_STATUS, 8, F_RDR, 0, D_BDW_PLUS, NULL,
		mmio_not_allow_write},
	{_REG_VECS_EXECLIST_STATUS, 8, F_RDR, 0, D_BDW_PLUS, NULL,
		mmio_not_allow_write},
	{_REG_VCS2_EXECLIST_STATUS, 8, F_RDR, 0, D_BDW_PLUS, NULL,
		mmio_not_allow_write},
	{_REG_BCS_EXECLIST_STATUS, 8, F_RDR, 0, D_BDW_PLUS, NULL,
		mmio_not_allow_write},

	{_REG_RCS_CTX_SR_CTL, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VCS_CTX_SR_CTL, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VECS_CTX_SR_CTL, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VCS2_CTX_SR_CTL, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_BCS_CTX_SR_CTL, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_RCS_CTX_STATUS_BUF, 48, F_VIRT, 0, D_BDW_PLUS, NULL,
		mmio_not_allow_write},
	{_REG_VCS_CTX_STATUS_BUF, 48, F_VIRT, 0, D_BDW_PLUS, NULL,
		mmio_not_allow_write},
	{_REG_VECS_CTX_STATUS_BUF, 48, F_VIRT, 0, D_BDW_PLUS, NULL,
		mmio_not_allow_write},
	{_REG_VCS2_CTX_STATUS_BUF, 48, F_VIRT, 0, D_BDW_PLUS, NULL,
		mmio_not_allow_write},
	{_REG_BCS_CTX_STATUS_BUF, 48, F_VIRT, 0, D_BDW_PLUS, NULL,
		mmio_not_allow_write},

	{_REG_RCS_CTX_STATUS_PTR, 4, F_VIRT | GVT_REG_MODE_CTL, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VCS_CTX_STATUS_PTR, 4, F_VIRT | GVT_REG_MODE_CTL, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VECS_CTX_STATUS_PTR, 4, F_VIRT | GVT_REG_MODE_CTL, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VCS2_CTX_STATUS_PTR, 4, F_VIRT | GVT_REG_MODE_CTL, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_BCS_CTX_STATUS_PTR, 4, F_VIRT | GVT_REG_MODE_CTL, 0, D_BDW_PLUS, NULL, NULL},
	/* -------display regs---------- */

	{_PIPE_MISC_A, 4, F_DPY, 0, D_BDW_PLUS, NULL, NULL},

	{_PIPE_MISC_B, 4, F_DPY, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_PIPE_MISC_C, 4, F_DPY, 0, D_BDW_PLUS, NULL, NULL},

	/* -------others---------- */

	/* -------un-categorized regs--------- */
	/* no definition on this. from Linux */
	{0x1c1d0, 4, F_PT, 0, D_BDW_PLUS, NULL, NULL},
	{_GEN6_MBCUNIT_SNPCR, 4, F_PT, 0, D_BDW_PLUS, NULL, NULL},
	{_GEN7_MISCCPCTL, 4, F_PT, 0, D_BDW_PLUS, NULL, NULL},

	{0x1C054, 4, F_DOM0, 0, D_BDW_PLUS, NULL, NULL},
	/* BDW */
	{_GEN8_PRIVATE_PAT_LO, 4, F_PT, 0, D_BDW_PLUS, NULL, NULL},
	{_GEN8_PRIVATE_PAT_HI, 4, F_PT, 0, D_BDW_PLUS, NULL, NULL},

	{_GAMTARBMODE, 4, F_DOM0, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_RCS_PDP_UDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_RCS_PDP_LDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_RCS_PDP_UDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_RCS_PDP_LDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_RCS_PDP_UDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_RCS_PDP_LDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_RCS_PDP_UDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_RCS_PDP_LDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_VCS_PDP_UDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VCS_PDP_LDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_VCS_PDP_UDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VCS_PDP_LDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_VCS_PDP_UDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VCS_PDP_LDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_VCS_PDP_UDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VCS_PDP_LDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_VECS_PDP_UDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VECS_PDP_LDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_VECS_PDP_UDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VECS_PDP_LDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_VECS_PDP_UDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VECS_PDP_LDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_VECS_PDP_UDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VECS_PDP_LDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_VCS2_PDP_UDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VCS2_PDP_LDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_VCS2_PDP_UDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VCS2_PDP_LDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_VCS2_PDP_UDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VCS2_PDP_LDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_VCS2_PDP_UDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_VCS2_PDP_LDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_BCS_PDP_UDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_BCS_PDP_LDW(0) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_BCS_PDP_UDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_BCS_PDP_LDW(1) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_BCS_PDP_UDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_BCS_PDP_LDW(2) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{_REG_BCS_PDP_UDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{_REG_BCS_PDP_LDW(3) , 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},

	{0x2080, 4, F_RDR_ADRFIX, 0xFFFFF000, D_BDW_PLUS, NULL, NULL},
	{0x12080, 4, F_RDR_ADRFIX, 0xFFFFF000, D_BDW_PLUS, NULL, NULL},
	{0x1c080, 4, F_RDR_ADRFIX, 0xFFFFF000, D_BDW_PLUS, NULL, NULL},
	{0x1a080, 4, F_RDR_ADRFIX, 0xFFFFF000, D_BDW_PLUS, NULL, NULL},
	{0x22080, 4, F_RDR_ADRFIX, 0xFFFFF000, D_BDW_PLUS, NULL, NULL},

	{0x7300, 4, F_RDR, 0, D_BDW_PLUS, NULL, NULL},

	{0x420b0, 4, F_DPY, 0, D_BDW, NULL, NULL},
	{0x420b4, 4, F_DPY, 0, D_BDW, NULL, NULL},
	{0x420b8, 4, F_DPY, 0, D_BDW, NULL, NULL},

	{0x45260, 4, F_DPY, 0, D_BDW, NULL, NULL},
	{0x6f800, 4, F_DPY, 0, D_BDW, NULL, NULL},

	{0x66c00, 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
	{0x66c04, 4, F_VIRT, 0, D_BDW, NULL, NULL},

	{0x4024, 4, F_DOM0, 0, D_BDW, NULL, NULL},

	{0x9134, 4, F_VIRT, 0, D_BDW, NULL, NULL},
	{0x9138, 4, F_VIRT, 0, D_BDW, NULL, NULL},
	{0x913c, 4, F_VIRT, 0, D_BDW, NULL, NULL},

	/* WA */
	{0xfdc, 4, F_DOM0, 0, D_BDW, NULL, NULL},
	{0xe4f0, 4, F_RDR, 0, D_BDW, NULL, NULL},
	{0xe4f4, 4, F_RDR, 0, D_BDW, NULL, NULL},
	{0x9430, 4, F_RDR, 0, D_BDW, NULL, NULL},

	/* L3 */
	{0xb1f0, 4, F_RDR, 0, D_BDW, NULL, NULL},
	{0xb1c0, 4, F_RDR, 0, D_BDW, NULL, NULL},
	{0xb118, 4, F_RDR, 0, D_BDW, NULL, NULL},
	{0xb100, 4, F_RDR, 0, D_BDW, NULL, NULL},
	{0xb10c, 4, F_RDR, 0, D_BDW, NULL, NULL},
	{0xb110, 4, F_PT, 0, D_BDW, NULL, NULL},

	/* NON-PRIV */
	{0x24d0, 4, F_RDR, 0, D_BDW, NULL, NULL},
	{0x24d4, 4, F_RDR, 0, D_BDW, NULL, NULL},
	{0x24d8, 4, F_RDR, 0, D_BDW, NULL, NULL},
	{0x24dc, 4, F_RDR, 0, D_BDW, NULL, NULL},

	{0x83a4, 4, F_RDR, 0, D_BDW, NULL, NULL},
	{0x4dd4, 4, F_PT, 0, D_BDW, NULL, NULL},

	/* UCG */
	{0x8430, 4, F_PT, 0, D_BDW, NULL, NULL},

	{0x110000, 4, F_VIRT, 0, D_BDW_PLUS, NULL, NULL},
};

int gvt_get_reg_num(int type)
{
        switch (type) {
                case D_ALL:
                        return ARRAY_SIZE(gvt_general_reg_info);
                case D_BDW:
                        return ARRAY_SIZE(gvt_broadwell_reg_info);
                default:
			return 0;
        }
        return 0;
}
