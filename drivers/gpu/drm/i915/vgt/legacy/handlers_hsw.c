/* MMIO virtualization handlers HSW specific
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

#include <linux/delay.h>
#include <linux/acpi.h>

#include "vgt.h"
#include "fb_decoder.h"

static bool dpy_plane_mmio_read(struct vgt_device *vgt, unsigned int offset,
			void *p_data, unsigned int bytes)
{

	*(vgt_reg_t *)p_data = __vreg(vgt, offset);

	return true;
}

static bool dpy_plane_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	unsigned int real_offset;

	memcpy ((char *)vgt->state.vReg + offset, p_data, bytes);
	memcpy ((char *)vgt->state.sReg + offset, p_data, bytes);
	if (current_foreground_vm(vgt->pdev) == vgt &&
		vgt_map_plane_reg(vgt, offset, &real_offset)) {
		VGT_MMIO_WRITE(vgt->pdev, real_offset, __sreg(vgt, offset));
	}
	return true;
}

reg_attr_t vgt_reg_info_hsw[] = {

/* Interrupt registers - GT */
{GTIMR, 4, F_VIRT, 0, D_HSW, NULL, vgt_reg_imr_handler},
{GTIER, 4, F_VIRT, 0, D_HSW, NULL, vgt_reg_ier_handler},
{GTIIR, 4, F_VIRT, 0, D_HSW, NULL, vgt_reg_iir_handler},
{GTISR, 4, F_VIRT, 0, D_HSW, NULL, NULL},

/* Interrupt registers - Display */
{DEIMR, 4, F_VIRT, 0, D_HSW, NULL, vgt_reg_imr_handler},
{DEIER, 4, F_VIRT, 0, D_HSW, NULL, vgt_reg_ier_handler},
{DEIIR, 4, F_VIRT, 0, D_HSW, NULL, vgt_reg_iir_handler},
{DEISR, 4, F_VIRT, 0, D_HSW, NULL, NULL},

/* Interrupt registers - PM */
{GEN6_PMIMR, 4, F_VIRT, 0, D_HSW, NULL, vgt_reg_imr_handler},
{GEN6_PMIER, 4, F_VIRT, 0, D_HSW, NULL, vgt_reg_ier_handler},
{GEN6_PMIIR, 4, F_VIRT, 0, D_HSW, NULL, vgt_reg_iir_handler},
{GEN6_PMISR, 4, F_VIRT, 0, D_HSW, NULL, NULL},

/* -------render regs---------- */
{_REG_VECS_HWS_PGA, 4, F_RDR_ADRFIX, 0xFFFFF000, D_HSW, NULL, NULL},

/* maybe an error in Linux driver. meant for VCS_HWS_PGA */
{_REG_VECS_EXCC, 4, F_RDR, 0, D_HSW, NULL, NULL},
/* TODO: need a handler */
{_REG_RCS_PP_DIR_BASE_IVB, 4, F_RDR_ADRFIX, 0xFFFFF000, D_HSW, NULL, NULL},
{_REG_VCS_PP_DIR_BASE, 4, F_RDR_ADRFIX, 0xFFFFF000, D_HSW, NULL, NULL},
{_REG_BCS_PP_DIR_BASE, 4, F_RDR_ADRFIX, 0xFFFFF000, D_HSW, NULL, NULL},
{_REG_VECS_PP_DIR_BASE, 4, F_RDR_ADRFIX, 0xFFFFF000, D_HSW, NULL, NULL},
{_REG_VECS_PP_DCLV, 4, F_RDR, 0, D_HSW, NULL, NULL},
{_REG_RVESYNC, 4, F_RDR, 0, D_HSW, NULL, NULL},
{_REG_BVESYNC, 4, F_RDR, 0, D_HSW, NULL, NULL},
{_REG_VVESYNC, 4, F_RDR, 0, D_HSW, NULL, NULL},
{_REG_VEBSYNC, 4, F_RDR, 0, D_HSW, NULL, NULL},
{_REG_VERSYNC, 4, F_RDR, 0, D_HSW, NULL, NULL},
{_REG_VEVSYNC, 4, F_RDR, 0, D_HSW, NULL, NULL},

{0x2450, 8, F_RDR, 0, D_HSW, NULL, NULL},
{0x91b8, 4, F_VIRT, 0, D_HSW, NULL, NULL},
{0x91c0, 4, F_VIRT, 0, D_HSW, NULL, NULL},
{0x9150, 4, F_VIRT, 0, D_HSW, NULL, NULL},
{0x9154, 4, F_VIRT, 0, D_HSW, NULL, NULL},
{0x9160, 4, F_VIRT, 0, D_HSW, NULL, NULL},
{0x9164, 4, F_VIRT, 0, D_HSW, NULL, NULL},

{0x4040, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0xb010, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0xb020, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0xb024, 4, F_RDR, 0, D_HSW, NULL, NULL},

{GEN7_L3CNTLREG1, 4, F_RDR, 0, D_HSW, NULL, NULL},
{GEN7_L3_CHICKEN_MODE_REGISTER, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x20e8, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0xb008, 4, F_VIRT, 0, D_HSW, NULL, NULL},
{0xb208, 4, F_VIRT, 0, D_HSW, NULL, NULL},


	/* -------display regs---------- */

{_REG_PIPECSTAT, 4, F_DPY, 0, D_HSW, NULL, NULL},
{_DVSALINOFF, 4, F_DPY, 0, D_HSW, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_DVSAPOS, 4, F_DPY, 0, D_HSW, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_DVSASIZE, 4, F_DPY, 0, D_HSW, dpy_plane_mmio_read,
							dpy_plane_mmio_write},
{_SPRA_SCALE, 4, F_DPY, 0, D_HSW, NULL, NULL},

{_SPRB_SCALE, 4, F_DPY, 0, D_HSW, NULL, NULL},


{_REG_SPRC_SCALE, 4, F_DPY, 0, D_HSW, NULL, NULL},

/*
 * framebuffer compression is disabled for now
 * until it's handled at display context switch
 * and we figure out how stolen memory should be virtualized (FBC needs use
 * stolen memory).
 */

{WM_DBG, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x6661c, 4, F_DPY, 0, D_HSW, NULL, NULL},
{0x66C00, 8, F_DPY, 0, D_HSW, NULL, NULL},

	/* -------others---------- */
/* TODO: MCHBAR, suppose read-only */
{0x320f0, 8, F_DOM0, 0, D_HSW, NULL, NULL},
{0x320fc, 4, F_DOM0, 0, D_HSW, NULL, NULL},
{0x32230, 4, F_DOM0, 0, D_HSW, NULL, NULL},
{0x44084, 4, F_DOM0, 0, D_HSW, NULL, NULL},
{0x4408c, 4, F_DOM0, 0, D_HSW, NULL, NULL},
{0x1082c0, 4, F_DOM0, 0, D_HSW, NULL, NULL},

	/* -------un-categorized regs--------- */

{GEN6_MBCUNIT_SNPCR, 4, F_VIRT, 0, D_HSW, NULL, NULL},
/* FIXME: now looks gmbus handler can't cover 4/5 ports */
{GEN7_MISCCPCTL, 4, F_VIRT, 0, D_HSW, NULL, NULL},

/* HSW */
{0x2214, 4, F_PT, 0, D_HSW, NULL, NULL},

{0x8000, 4, F_PT, 0, D_HSW, NULL, NULL},
{0x8008, 4, F_PT, 0, D_HSW, NULL, NULL},

{0x45260, 4, F_PT, 0, D_HSW, NULL, NULL},
{0x13005c, 4, F_PT, 0, D_HSW, NULL, NULL},

/* DOM0 PM owns these registers. */
{HSW_SCRATCH1, 4, F_RDR, 0, D_HSW, NULL, NULL},
{HSW_ROW_CHICKEN3, 4, F_DOM0, 0, D_HSW, NULL, NULL},
/*command accessed registers, supplement for reg audit in cmd parser*/
{GEN7_L3SQCREG4, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x12400, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x12468, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x124a0, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x124a4, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x124b4, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x124b8, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x124bc, 4, F_RDR, 0, D_HSW, NULL, NULL},
{0x124d0, 4, F_RDR, 0, D_HSW, NULL, NULL},
};

int vgt_get_hsw_reg_num(void)
{
	return ARRAY_NUM(vgt_reg_info_hsw);
}
