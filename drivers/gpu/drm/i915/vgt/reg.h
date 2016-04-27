/*
 * vGT core headers
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

#ifndef _VGT_REG_H_
#define _VGT_REG_H_
#include "../i915_reg.h"
/*
 * Definition of MMIO registers.
 */
#define _VGT_MMIO_THROUGH_OFFSET(index, a, b)	((a) + (index)*((b)-(a)))
#define _VGT_MMIO_GET_INDEX(reg, a, b)		(((reg)-(a))/((b)-(a)))

#define _VGT_GET_PIPE(reg, a, b)	_VGT_MMIO_GET_INDEX(reg, a, b)
#define _VGT_GET_PORT(reg, a, b)	_VGT_MMIO_GET_INDEX(reg, a, b)

static inline uint32_t __RING_REG(int32_t ring_id, uint32_t rcs_reg)
{
	if (ring_id == 0 /* RING_BUFFER_RCS */) return rcs_reg;
	if (ring_id == 1 /* RING_BUFFER_VCS */) return rcs_reg+0x10000;
	if (ring_id == 2 /* RING_BUFFER_BCS */) return rcs_reg+0x20000;
	if (ring_id == 3 /* RING_BUFFER_VECS*/) return rcs_reg+0x18000;
	if (ring_id == 4 /* RING_BUFFER_VCS2*/) return rcs_reg+0x1a000;
	return 0; /* invalid ring_id, trigger crash */
}

#define _REG_INVALID	0xFFFFFFFF

/*
 * Registers used only by the command parser
 */
#define _REG_OACONTROL_GEN8      0x2B00
#define _REG_OACTXCONTROL        0x2360

/* PRB0, RCS */
#define _REG_RCS_TAIL	0x02030
#define _REG_RCS_HEAD	0x02034
#define _REG_RCS_START	0x02038
#define _REG_RCS_CTL	0x0203c

/* VECS: HSW+ */
#define _REG_VECS_TAIL	0x1A030
#define _REG_VECS_HEAD	0x1A034
#define _REG_VECS_START	0x1A038
#define _REG_VECS_CTL	0x1A03c

/* VCS */
#define _REG_VCS_TAIL	0x12030
#define _REG_VCS_HEAD	0x12034
#define _REG_VCS_START	0x12038
#define _REG_VCS_CTL	0x1203c

/* VCS2 for BDW GT3 */
#define _REG_VCS2_TAIL	0x1C030
#define _REG_VCS2_HEAD	0x1C034
#define _REG_VCS2_START	0x1C038
#define _REG_VCS2_CTL	0x1C03C

/* BCS */
#define _REG_BCS_TAIL	0x22030
#define _REG_BCS_HEAD	0x22034
#define _REG_BCS_START	0x22038
#define _REG_BCS_CTL	0x2203c

#define RB_OFFSET_TAIL		0
#define RB_OFFSET_HEAD		4
#define RB_OFFSET_START		8
#define RB_OFFSET_CTL		0xC
#define RB_REGS_SIZE		0x10

#define RB_TAIL(pdev, id)	(pdev->ring_mmio_base[id] + RB_OFFSET_TAIL)
#define RB_HEAD(pdev, id)	(pdev->ring_mmio_base[id] + RB_OFFSET_HEAD)
#define RB_START(pdev, id)	(pdev->ring_mmio_base[id] + RB_OFFSET_START)
#define RB_CTL(pdev, id)	(pdev->ring_mmio_base[id] + RB_OFFSET_CTL)

#define RB_HEAD_OFF_MASK	((1U << 21) - (1U << 2))	/* bit 2 to 20 */
#define RB_HEAD_OFF_SHIFT	2
#define RB_TAIL_OFF_MASK	((1U << 21) - (1U << 3))	/* bit 3 to 20 */
#define RB_TAIL_OFF_SHIFT	3

#define RB_TAIL_SIZE_MASK	((1U << 21) - (1U << 12))	/* bit 12 to 20 */
#define _RING_CTL_BUF_SIZE(ctl)	(((ctl) & RB_TAIL_SIZE_MASK) + GTT_PAGE_SIZE)
#define _RING_CTL_ENABLE	0x1	/* bit 0 */

#define CCID_MBO_BITS		(1 << 8)	/* bit 8 must be one */
#define CCID_EXTENDED_STATE_SAVE_ENABLE		(1 << 3)
#define CCID_EXTENDED_STATE_RESTORE_ENABLE	(1 << 2)
#define _REG_VECS_CXT_SIZE	0x1A1A8

#define	_REGBIT_MI_ASYNC_FLIP_PERFORMANCE_MODE	(1 << 14)
#define	MI_FLUSH_ENABLE_PERFORMANCE_MODE	(1 << 13)
#define	_REGBIT_MI_INVALIDATE_UHPTR	(1 << 11)

#define _REG_VCS_MI_MODE	0x1209C
#define _REG_VCS2_MI_MODE	0x1c09C
#define _REG_BCS_MI_MODE	0x2209C
#define _REG_VECS_MI_MODE	0x1A09c
#define        _REGBIT_PPGTT64_ENABLE                   (1 << 7)
#define        _REGBIT_ADDRESS_SWIZZLING		(3 << 4)

#define _REG_GAC_MODE		0x120A0
#define _REG_GAB_MODE		0x220A0

#define _REG_VCS_INSTPM		0x120C0
#define _REG_VCS2_INSTPM	0x1c0C0
#define _REG_BCS_INSTPM		0x220C0
#define _REG_VECS_INSTPM	0x1A0C0

#define INSTPM_CONS_BUF_ADDR_OFFSET_DIS (1<<6)

/* IVB+ */
#define _REG_BCS_BLT_MODE_IVB	0x2229C
#define _REG_VCS_MFX_MODE_IVB	0x1229C
#define _REG_VCS2_MFX_MODE_BDW	0x1c29C
#define _REGBIT_EXECLIST_ENABLE       (1 << 15)
#define _REG_VEBOX_MODE		0x1A29C

#define GFX_MODE_BIT_SET_IN_MASK(val, bit) \
		((((bit) & 0xffff0000) == 0) && !!((val) & (((bit) << 16))))

/* PPGTT entry */
#define _REGBIT_PDE_VALID	(1<<0)
#define _REGBIT_PDE_PAGE_32K	(1<<1)
#define _REGBIT_PTE_VALID	(1<<0)
/* control bits except address and valid bit */
#define _REGBIT_PTE_CTL_MASK_GEN7	0xe	/* SNB/IVB */
#define _REGBIT_PTE_CTL_MASK_GEN7_5	0x80e	/* HSW */
	
#define _REG_VCS_IMR		0x120A8
#define _REG_BCS_IMR		0x220A8
#define _REG_VECS_IMR		0x1A0A8
#define _REG_VCS2_IMR		0x1c0A8

#define _REG_RCS_BB_ADDR	0x2140
#define _REG_VCS_BB_ADDR	0x12140
#define _REG_BCS_BB_ADDR	0x22140
#define _REG_VECS_BB_ADDR	0x1A140
#define _REG_VCS2_BB_ADDR	0x1c140

#define _REG_VECS_CTX_WA_BB_ADDR 0x1A144
#define	_REG_BCS_HWS_PGA	0x24080
#define _REG_VECS_HWS_PGA	0x1A080

#define _REG_RCS_EXCC		0x2028
#define _REG_VCS_EXCC		0x12028
#define _REG_BCS_EXCC		0x22028
#define _REG_VECS_EXCC		0x1A028
#define _REG_VCS2_EXCC		0x1c028

#define _REG_RCS_UHPTR		0x2134
#define _REG_VCS_UHPTR		0x12134
#define _REG_BCS_UHPTR		0x22134
#define _REG_VECS_UHPTR		0x1A134
#define _REG_VCS2_UHPTR		0x1c134
#define 	_REGBIT_UHPTR_VALID	(1 << 0)
#define VGT_UHPTR(ring_id) __RING_REG(ring_id, _REG_RCS_UHPTR)

#define _REG_VCS_ACTHD		0x12074
#define _REG_BCS_ACTHD		0x22074
#define _REG_VECS_ACTHD		0x1A074
#define _REG_VCS2_ACTHD		0x1c074

#define _REG_RCS_ACTHD_UDW	0x205c
#define _REG_VCS_ACTHD_UDW	0x1205c
#define _REG_BCS_ACTHD_UDW	0x2205c
#define _REG_VECS_ACTHD_UDW	0x1A05c
#define _REG_VCS2_ACTHD_UDW	0x1c05c

#define VGT_ACTHD(ring_id) __RING_REG(ring_id, ACTHD_I965)
#define _REG_RCS_HWSTAM		0x2098
#define _REG_VCS_HWSTAM		0x12098
#define _REG_BCS_HWSTAM		0x22098
#define _REG_VECS_HWSTAM	0x1A098
#define _REG_VCS2_HWSTAM	0x1c098

#define _REG_RCS_BB_PREEMPT_ADDR	0x2148

#define _REG_RCS_BB_ADDR_DIFF		0x2154

#define _REG_RCS_PP_DIR_BASE_READ	0x2518
#define _REG_RCS_PP_DIR_BASE_IVB	0x2228
#define _REG_RCS_PP_DCLV		0x2220
#define _REG_BCS_PP_DIR_BASE		0x22228
#define _REG_BCS_PP_DCLV		0x22220
#define _REG_VCS_PP_DIR_BASE		0x12228
#define _REG_VCS_PP_DCLV		0x12220
#define _REG_VECS_PP_DIR_BASE		0x1A228
#define _REG_VECS_PP_DCLV		0x1A220

#define _REG_RVSYNC		0x2040
#define _REG_RBSYNC		0x2044
#define _REG_RVESYNC		0x2048

#define _REG_BRSYNC		0x22040
#define _REG_BVSYNC		0x22044
#define _REG_BVESYNC		0x22048

#define _REG_VBSYNC		0x12040
#define _REG_VRSYNC		0x12044
#define _REG_VVESYNC		0x12048

#define _REG_VEBSYNC		0x1A040
#define _REG_VERSYNC		0x1A044
#define _REG_VEVSYNC		0x1A048

#define _REG_RCS_TIMESTAMP	0x2358
#define _REG_VCS_TIMESTAMP	0x12358
#define _REG_VCS2_TIMESTAMP	0x1c358
#define _REG_BCS_TIMESTAMP	0x22358

#define _EL_BASE_RCS		0x02000
#define _EL_BASE_VCS		0x12000
#define _EL_BASE_VECS		0x1A000
#define _EL_BASE_VCS2		0x1C000
#define _EL_BASE_BCS		0x22000

#define _EL_OFFSET_SUBMITPORT	0x230
#define _EL_OFFSET_STATUS	0x234
#define _EL_OFFSET_SR_CTL	0x244
#define _EL_OFFSET_STATUS_BUF	0x370
#define _EL_OFFSET_STATUS_PTR	0x3A0

#define _REG_RCS_EXECLIST_SUBMITPORT	0x02230
#define _REG_VCS_EXECLIST_SUBMITPORT	0x12230
#define _REG_VECS_EXECLIST_SUBMITPORT	0x1A230
#define _REG_VCS2_EXECLIST_SUBMITPORT	0x1C230
#define _REG_BCS_EXECLIST_SUBMITPORT	0x22230

#define _EXECLIST_LRCA_MASK		0xfffff000

#define _REG_RCS_EXECLIST_STATUS	0x02234
#define _REG_VCS_EXECLIST_STATUS	0x12234
#define _REG_VECS_EXECLIST_STATUS	0x1A234
#define _REG_VCS2_EXECLIST_STATUS	0x1C234
#define _REG_BCS_EXECLIST_STATUS	0x22234

#define _REG_RCS_CTX_SR_CTL	0x02244
#define _REG_VCS_CTX_SR_CTL	0x12244
#define _REG_VECS_CTX_SR_CTL	0x1A244
#define _REG_VCS2_CTX_SR_CTL	0x1C244
#define _REG_BCS_CTX_SR_CTL	0x22244

#define _REG_RCS_CTX_STATUS_BUF		0x02370
#define _REG_VCS_CTX_STATUS_BUF		0x12370
#define _REG_VECS_CTX_STATUS_BUF	0x1A370
#define _REG_VCS2_CTX_STATUS_BUF	0x1C370
#define _REG_BCS_CTX_STATUS_BUF		0x22370

#define _REG_RCS_CTX_STATUS_PTR		0x023A0
#define _REG_VCS_CTX_STATUS_PTR		0x123A0
#define _REG_VECS_CTX_STATUS_PTR	0x1A3A0
#define _REG_VCS2_CTX_STATUS_PTR	0x1C3A0
#define _REG_BCS_CTX_STATUS_PTR		0x223A0

#define  _CTXBUF_READ_PTR_SHIFT		8
#define  _CTXBUF_READ_PTR_MASK		(0x7 << _CTXBUF_READ_PTR_SHIFT)
#define  _CTXBUF_WRITE_PTR_MASK		(0x7)

#define _REG_FENCE_0_LOW	0x100000

#define _CURSOR_MODE	0x3f
#define _CURSOR_MODE_DISABLE	0x00
#define _CURSOR_ALPHA_FORCE_SHIFT	8
#define _CURSOR_ALPHA_FORCE_MASK	(0x3 << _CURSOR_ALPHA_FORCE_SHIFT)
#define _CURSOR_ALPHA_PLANE_SHIFT	10
#define _CURSOR_ALPHA_PLANE_MASK	(0x3 << _CURSOR_ALPHA_PLANE_SHIFT)
#define _CURSOR_POS_X_SHIFT	0
#define _CURSOR_POS_X_MASK	(0x1fff << _CURSOR_POS_X_SHIFT)
#define _CURSOR_SIGN_X_SHIFT	15
#define _CURSOR_SIGN_X_MASK	(1 << _CURSOR_SIGN_X_SHIFT)
#define _CURSOR_POS_Y_SHIFT		16
#define _CURSOR_POS_Y_MASK	(0xfff << _CURSOR_POS_Y_SHIFT)
#define _CURSOR_SIGN_Y_SHIFT	31
#define _CURSOR_SIGN_Y_MASK	(1 << _CURSOR_SIGN_Y_SHIFT)
#define _REG_CURASURFLIVE	0x700AC

#define _REG_CURAPALET_0	0x70090
#define _REG_CURAPALET_1	0x70094
#define _REG_CURAPALET_2	0x70098
#define _REG_CURAPALET_3	0x7009C

#define _REG_CURBSURFLIVE_SNB	0x700EC
#define _REG_CURBSURFLIVE	0x710AC
#define _REG_CURCNTR		0x72080
#define _REG_CURCBASE	0x72084
#define _REG_CURCPOS	0x72088
#define _REG_CURCSURFLIVE	0x720AC

#define VGT_CURCNTR_SNB(pipe)	_PIPE(pipe, _CURACNTR, _CURBCNTR)
#define VGT_CURBASE_SNB(pipe)	_PIPE(pipe, _CURABASE, _CURBBASE)
#define VGT_CURPOS_SNB(pipe)	_PIPE(pipe, _CURAPOS, _CURBPOS)

#define VGT_CURCNTR(pipe)	_PIPE(pipe, _CURACNTR, _CURBCNTR_IVB)
#define VGT_CURBASE(pipe)	_PIPE(pipe, _CURABASE, _CURBBASE_IVB)
#define VGT_CURPOS(pipe)	_PIPE(pipe, _CURAPOS, _CURBPOS_IVB)

#define	_PRI_PLANE_FMT_SHIFT	26
#define	_PRI_PLANE_TILE_SHIFT	10

#define	_PRI_PLANE_STRIDE_SHIFT	6
#define	_PRI_PLANE_STRIDE_MASK	(0x3ff << _PRI_PLANE_STRIDE_SHIFT)
#define	SKL_PLANE_STRIDE_MASK	0x3ff

#define	_PRI_PLANE_X_OFF_SHIFT	0
#define	_PRI_PLANE_X_OFF_MASK	(0x1fff << _PRI_PLANE_X_OFF_SHIFT)
#define	_PRI_PLANE_Y_OFF_SHIFT	16
#define	_PRI_PLANE_Y_OFF_MASK	(0xfff << _PRI_PLANE_Y_OFF_SHIFT)

#define	_REG_DSPBCNTR	0x71180
#define	_REG_DSPBLINOFF	0x71184
#define	_REG_DSPBSTRIDE	0x71188
#define	_REG_DSPBPOS	0x7118C
#define _REG_DSPBSIZE	0x71190
#define _REG_DSPBSURF	0x7119C
#define _REG_DSPBTILEOFF	0x711A4
#define _REG_DSPBSURFLIVE	0x711AC


#define VGT_DSPSURF(pipe)	_PIPE(pipe, _DSPASURF, _REG_DSPBSURF)
#define VGT_DSPCNTR(pipe)	_PIPE(pipe, _DSPACNTR, _REG_DSPBCNTR)
#define VGT_DSPCNTRPIPE(dspcntr)	_VGT_GET_PIPE(dspcntr, _DSPACNTR,_REG_DSPBCNTR)

#define VGT_DSPLINOFF(plane) _PIPE(plane, _DSPAADDR, _REG_DSPBLINOFF)
#define VGT_DSPSTRIDE(plane) _PIPE(plane, _DSPASTRIDE, _REG_DSPBSTRIDE)
#define VGT_DSPTILEOFF(plane) _PIPE(plane, _DSPATILEOFF, _REG_DSPBTILEOFF)

#define VGT_DSPSURFPIPE(dspsurf) _VGT_GET_PIPE(dspsurf, _DSPASURF,_REG_DSPBSURF)
#define VGT_DSPSURFLIVEPIPE(dspsurf) _VGT_GET_PIPE(dspsurf, _DSPASURFLIVE, \
							_REG_DSPBSURFLIVE)
#define VGT_DSPSURFLIVE(pipe)	_PIPE(pipe, _DSPASURFLIVE, _REG_DSPBSURFLIVE)

#define VGT_CURSURFPIPE(cursurf)	_VGT_GET_PIPE(cursurf, _CURABASE, _CURBBASE_IVB)
#define VGT_CURSURF(pipe)	_PIPE(pipe, _CURABASE, _CURBBASE_IVB)

/* sprite */
#define	_SPRITE_FMT_SHIFT	25
#define	_SPRITE_COLOR_ORDER_SHIFT	20
#define	_SPRITE_YUV_ORDER_SHIFT	16

#define	_SPRITE_STRIDE_SHIFT	6
#define	_SPRITE_STRIDE_MASK	(0x1ff << _SPRITE_STRIDE_SHIFT)

#define	_SPRITE_POS_X_SHIFT	0
#define	_SPRITE_POS_Y_SHIFT	16
#define	_SPRITE_POS_X_MASK	(0x1fff << _SPRITE_POS_X_SHIFT)
#define	_SPRITE_POS_Y_MASK	(0xfff << _SPRITE_POS_Y_SHIFT)

#define	_SPRITE_SIZE_WIDTH_SHIFT	0
#define	_SPRITE_SIZE_HEIGHT_SHIFT	16
#define	_SPRITE_SIZE_WIDTH_MASK	(0x1fff << _SPRITE_SIZE_WIDTH_SHIFT)
#define	_SPRITE_SIZE_HEIGHT_MASK	(0xfff << _SPRITE_SIZE_HEIGHT_SHIFT)

#define	_SPRITE_OFFSET_START_X_SHIFT	0
#define	_SPRITE_OFFSET_START_Y_SHIFT	16
#define	_SPRITE_OFFSET_START_X_MASK	(0x1fff << _SPRITE_OFFSET_START_X_SHIFT)
#define	_SPRITE_OFFSET_START_Y_MASK	(0xfff << _SPRITE_OFFSET_START_Y_SHIFT)
#define _REG_SPRC_CTL				0x72280
#define _REG_SPRC_STRIDE			0x72288
#define _REG_SPRCSURF				0x7229C
#define _REG_SPRCSURFLIVE			0x722AC
#define _REG_SPRC_SCALE				0x72304

#define VGT_SPRCTL(pipe)	_PIPE(pipe, _SPRA_CTL, _PLANE_CTL_2_B)
#define VGT_SPRSTRIDE(pipe)	_PIPE(pipe, _SPRA_STRIDE, _PLANE_STRIDE_2_B)
#define VGT_SPRPOS(pipe)	_PIPE(pipe, _PLANE_POS_2_A, _PLANE_POS_2_B)
#define VGT_SPRSIZE(pipe)	_PIPE(pipe, _PLANE_SIZE_2_A, _PLANE_SIZE_2_B)
#define VGT_SPRSURF(pipe)	_PIPE(pipe, _SPRA_SURF, _PLANE_SURF_2_B)
#define VGT_SPRSURFPIPE(sprsurf) _VGT_GET_PIPE(sprsurf, _SPRA_SURF, _PLANE_SURF_2_B)
#define VGT_SPRSURFLIVE(pipe)	_PIPE(pipe, _REG_SPRASURFLIVE, _SPRBSURFLIVE)
#define VGT_SPROFFSET(pipe)	_PIPE(pipe, _PLANE_OFFSET_2_A, _PLANE_OFFSET_2_B)

#define VGT_SPRCNTRPIPE(sprcntr) _VGT_GET_PIPE(sprcntr, _SPRA_CTL,_PLANE_CTL_2_B)
#define VGT_CURCNTRPIPE(curcntr) _VGT_GET_PIPE(curcntr, _CURACNTR,_CURBCNTR_IVB)

#define _REG_GT_CORE_STATUS	0x138060

#define _REGBIT_RC_HW_CTRL_ENABLE	(1<<31)
#define _REGBIT_RC_RC1_ENABLE		(1<<20)
#define _REGBIT_RC_RC6_ENABLE		(1<<18)
#define _REGBIT_RC_DEEP_RC6_ENABLE	(1<<17)
#define _REGBIT_RC_DEEPEST_RC6_ENABLE	(1<<16)

/*
 * We use _IMM instead of _INDEX, to avoid switching hardware
 * status page
 */
#define MI_STORE_DATA_IMM		((0x20<<23) | 2)
#define MI_STORE_DATA_IMM_QWORD		((0x20<<23) | 3)
#define   MI_SDI_USE_GTT		(1<<22)
#define MI_LRI_CMD			(0x22<<23 | 1)
#define   MI_LRI_BYTE0_DISABLE		(1<<8)
#define   MI_LRI_BYTE1_DISABLE		(1<<9)
#define   MI_LRI_BYTE2_DISABLE		(1<<10)
#define   MI_LRI_BYTE3_DISABLE		(1<<11)

#define   MI_WAIT_FOR_PLANE_C_FLIP_PENDING      (1<<15)
#define   MI_WAIT_FOR_PLANE_B_FLIP_PENDING      (1<<9)
#define   MI_WAIT_FOR_PLANE_A_FLIP_PENDING      (1<<1)

#define   MI_WAIT_FOR_SPRITE_C_FLIP_PENDING      (1<<20)
#define   MI_WAIT_FOR_SPRITE_B_FLIP_PENDING      (1<<10)
#define   MI_WAIT_FOR_SPRITE_A_FLIP_PENDING      (1<<2)

#define	PIPE_CONTROL_DC_FLUSH_ENABLE			(1<<5)
#define PIPE_CONTROL_LRI_POST_SYNC			(1<<23)
#define DUMMY_3D		(0x6d800005)
#define PRIM_TRILIST		(0x4)
/*
 * Display engine regs
 */

#define _ACTIVE_WIDTH_MASK (0xFFF)

/* Pipe A timing regs */

#define     _PIPE_V_SRCSZ_SHIFT	0
#define     _PIPE_V_SRCSZ_MASK	(0xfff << _PIPE_V_SRCSZ_SHIFT)
#define     _PIPE_H_SRCSZ_SHIFT	16
#define     _PIPE_H_SRCSZ_MASK	(0x1fff << _PIPE_H_SRCSZ_SHIFT)

/* Pipe C timing regs */
#define _REG_HTOTAL_C		0x62000
#define _REG_HBLANK_C		0x62004
#define _REG_HSYNC_C		0x62008
#define _REG_VTOTAL_C		0x6200c
#define _REG_VBLANK_C		0x62010
#define _REG_VSYNC_C		0x62014
#define _REG_PIPECSRC		0x6201c
#define _REG_BCLRPAT_C		0x62020	/*not needed*/
#define _REG_VSYNCSHIFT_C	0x62028

/* Pipe EDP timing regs */
#define _REG_HTOTAL_EDP		0x6F000
#define _REG_HBLANK_EDP		0x6F004
#define _REG_HSYNC_EDP			0x6F008
#define _REG_VTOTAL_EDP		0x6F00c
#define _REG_VBLANK_EDP		0x6F010
#define _REG_VSYNC_EDP			0x6F014
#define _REG_VSYNCSHIFT_EDP	0x6F028


#define VGT_HTOTAL(pipe)	_PIPE(pipe, _HTOTAL_A, _HTOTAL_B)
#define VGT_HBLANK(pipe)	_PIPE(pipe, _HBLANK_A, _HBLANK_B)
#define VGT_HSYNC(pipe)		_PIPE(pipe, _HSYNC_A, _HSYNC_B)
#define VGT_VTOTAL(pipe)	_PIPE(pipe, _VTOTAL_A, _VTOTAL_B)
#define VGT_VBLANK(pipe)	_PIPE(pipe, _VBLANK_A, _VBLANK_B)
#define VGT_VSYNC(pipe)		_PIPE(pipe, _VSYNC_A, _VSYNC_B)

#define VGT_BCLRPAT(pipe)	_PIPE(pipe, _BCLRPAT_A, _BCLRPAT_B)
#define VGT_VSYNCSHIFT(pipe)	_PIPE(pipe, _VSYNCSHIFT_A, _VSYNCSHIFT_B)
#define VGT_PIPESRC(pipe)	_PIPE(pipe, _PIPEASRC, _PIPEBSRC)

#define VGT_PCH_DPLL(pipe)	_PIPE(pipe, _REG_PCH_DPLL_A, _REG_PCH_DPLL_B)

#define VGT_PCH_FP0(pipe)	_PIPE(pipe, _REG_PCH_FPA0, _REG_PCH_FPB0)
#define VGT_PCH_FP1(pipe)	_PIPE(pipe, _REG_PCH_FPA1, _REG_PCH_FPB1)

/* PIPE C timing regs are same start from 0x61000 */
#define _REG_PIPEC_DATA_M1		0x62030
#define _REG_PIPEC_DATA_N1		0x62034
#define _REG_PIPEC_LINK_M1		0x62040
#define _REG_PIPEC_LINK_N1		0x62044

#define _REG_PIPEC_DATA_M2		0x62038
#define _REG_PIPEC_DATA_N2		0x6203c
#define _REG_PIPEC_LINK_M2		0x62048
#define _REG_PIPEC_LINK_N2		0x6204c

#define VGT_PIPE_DATA_M1(pipe) _PIPE(pipe, _REG_PIPEA_DATA_M1, _REG_PIPEB_DATA_M1)
#define VGT_PIPE_DATA_N1(pipe) _PIPE(pipe, _REG_PIPEA_DATA_N1, _REG_PIPEB_DATA_N1)
#define VGT_PIPE_DATA_M2(pipe) _PIPE(pipe, _REG_PIPEA_DATA_M2, _REG_PIPEB_DATA_M2)
#define VGT_PIPE_DATA_N2(pipe) _PIPE(pipe, _REG_PIPEA_DATA_N2, _REG_PIPEB_DATA_N2)
#define VGT_PIPE_LINK_M1(pipe) _PIPE(pipe, _REG_PIPEA_LINK_M1, _REG_PIPEB_LINK_M1)
#define VGT_PIPE_LINK_N1(pipe) _PIPE(pipe, _REG_PIPEA_LINK_N1, _REG_PIPEB_LINK_N1)
#define VGT_PIPE_LINK_M2(pipe) _PIPE(pipe, _REG_PIPEA_LINK_M2, _REG_PIPEB_LINK_M2)
#define VGT_PIPE_LINK_N2(pipe) _PIPE(pipe, _REG_PIPEA_LINK_N2, _REG_PIPEB_LINK_N2)

/* FDI_RX, FDI_X is hard-wired to Transcoder_X */

#define _REG_FDI_RXC_CTL			0xf200c
#define _REGBIT_FDI_RX_PORT_WIDTH_MASK		(0x7 << 19)
#define _REGBIT_FDI_RX_FDI_AUTO_TRAIN_ENABLE	(0x1 << 10)

#define _REG_FDI_RXC_IIR			0xf2014
#define _REG_FDI_RXC_IMR			0xf2018

#define VGT_FDI_RX_IIR(pipe) _PIPE(pipe, _FDI_RXA_IIR, _FDI_RXB_IIR)
#define VGT_FDI_RX_IMR(pipe) _PIPE(pipe, _FDI_RXA_IMR, _FDI_RXB_IMR)

#define _REGBIT_FDI_RX_INTER_LANE_ALIGN		(1<<10)
#define _REGBIT_FDI_RX_SYMBOL_LOCK		(1 << 9) /* train 2*/
#define _REGBIT_FDI_RX_BIT_LOCK			(1 << 8) /* train 1*/
#define _REGBIT_FDI_RX_TRAIN_PATTERN_2_FAIL	(1<<7)
#define _REGBIT_FDI_RX_FS_CODE_ERR		(1<<6)
#define _REGBIT_FDI_RX_FE_CODE_ERR		(1<<5)
#define _REGBIT_FDI_RX_SYMBOL_ERR_RATE_ABOVE	(1<<4)
#define _REGBIT_FDI_RX_HDCP_LINK_FAIL		(1<<3)
#define _REGBIT_FDI_RX_PIXEL_FIFO_OVERFLOW	(1<<2)
#define _REGBIT_FDI_RX_CROSS_CLOCK_OVERFLOW	(1<<1)
#define _REGBIT_FDI_RX_SYMBOL_QUEUE_OVERFLOW	(1<<0)


#define VGT_FDI_RX_CTL_BPC_MASK		(0x7 << 16)
#define VGT_FDI_RX_CTL(pipe) _PIPE(pipe, _FDI_RXA_CTL, _FDI_RXB_CTL)

#define VGT_FDI_RX_TUSIZE1(pipe) _PIPE(pipe, _REG_FDI_RXA_TUSIZE1,_REG_FDI_RXB_TUSIZE1)

/* CPU: FDI_TX */
#define _REG_FDI_TXC_CTL		0x62100

#define _REGBIT_FDI_TX_ENABLE				(1 << 31)
#define _REGBIT_FDI_TX_PLL_ENABLE			(1 << 14)
#define _REGBIT_FDI_TX_ENHANCE_FRAME_ENABLE		(1<<18)

#define VGT_FDI_TX_CTL(pipe) _PIPE(pipe, _FDI_TXA_CTL, _FDI_TXB_CTL)

/* CRT */
#define _REGBIT_ADPA_DAC_ENABLE			(1 << 31)
#define PORT_TRANS_SEL_SHIFT			29
#define VGT_PORT_TRANS_SEL_CPT(pipe)		((pipe) << PORT_TRANS_SEL_SHIFT)
#define _REGBIT_ADPA_VSYNC_ACTIVE_HIGH		(1 << 4)
#define _REGBIT_ADPA_HSYNC_ACTIVE_HIGH		(1 << 3)

/* HDMI/DVI/SDVO port */
#define HDMI_TRANS_SEL_MASK		(3 << 29)
#define _REGBIT_HDMI_PORT_ENABLE	(1 << 31)
#define _REGBIT_HDMI_PORT_DETECTED	(1 << 2)

/* PCH SDVOB multiplex with HDMIB */
#define VGT_BACKLIGHT_DUTY_CYCLE_MASK		(0xffff)

#define _REGBIT_POWER_TARGET_ON		(1 << 0)


/* Watermark register (Ironlake) */

#define  _REGBIT_WM0_PIPE_PLANE_MASK	(0x7f<<16)
#define  _REGBIT_WM0_PIPE_PLANE_SHIFT	16
#define  _REGBIT_WM0_PIPE_SPRITE_MASK	(0x3f<<8)
#define  _REGBIT_WM0_PIPE_SPRITE_SHIFT	8
#define  _REGBIT_WM0_PIPE_CURSOR_MASK	(0x1f)

#define DISPLAY_MAXWM	0x7f	/* bit 16:22 */
#define CURSOR_MAXWM	0x1f	/* bit 4:0 */

/*Intermediate Pixel Storage*/
union _PCH_PP_CONTROL
{
	uint32_t data;
	struct
	{
		uint32_t power_state_target	: 1; // bit 0
		uint32_t power_down_on_reset	: 1; // bit 1
		uint32_t backlight_enable	: 1; // bit 2
		uint32_t edp_vdd_override_for_aux : 1; // bit 3
		uint32_t reserve : 12;			// bits 15:4
		uint32_t write_protect_key :16; // bits 31:16 0xABCD to disable protected)
	};
};

union _PCH_PP_STAUTS
{
	uint32_t data;
	struct
	{
		uint32_t reserv1	: 4;	// bit 3:0
		uint32_t reserv2	: 23;	// bit 26:4
		uint32_t power_cycle_delay_active	:1;	// bit 27
		uint32_t power_sequence_progress	:2;	// bits 29:28
		uint32_t require_asset_status		:1;	// bit 30
		uint32_t panel_powere_on_statue		:1;	// bit 31 (0 - Disable, 1 - Enable)
	};
};

/* Clocking configuration register */
/* CPU panel fitter */

#define _REG_PF_CTL_2			0x69080
#define _REG_PF_WIN_SZ_2		0x69074
#define _REG_PF_WIN_POS_2		0x69070

/* Per-transcoder DIP controls */

#define VGT_TRANSCONF(plane)	_PIPE(plane, _PCH_TRANSACONF, _PCH_TRANSBCONF)

union _TRANS_CONFIG
{
	uint32_t data;
	struct
	{
		uint32_t reserve1 : 10;			// bit 9:0
		uint32_t xvycc_color_range_limit : 1;	// bit 10
		uint32_t reserve2 : 10;			// bit 20:11
		uint32_t interlaced_mode: 3;		// bit 23:21
		uint32_t reserve3 : 6;			// bit 29:24
		uint32_t transcoder_state : 1;		// bit 30
		uint32_t transcoder_enable : 1;		// bit 31
	};
};

#define _REG_TRANSC_VIDEO_DIP_CTL	0xE2200
#define _REG_TRANSC_VIDEO_DIP_DATA	0xE2208
#define _REG_TRANSC_VIDEO_DIP_GCP	0xE2210

#define _REG_PIPEBCONF		0x71008
#define _REG_PIPEBSTAT		0x71024
#define _REG_PIPEB_FRMCOUNT 	0x71040
#define _REG_PIPEB_FLIPCOUNT	0x71044

/* Pipe C */
#define _REG_PIPECCONF		0x72008
#define _REG_PIPECSTAT		0x72024
#define _REG_PIPEC_FRMCOUNT 	0x72040
#define _REG_PIPEC_FLIPCOUNT	0x72044
#define _REG_PIPE_MISC_C	0x72030

/* eDP */
#define _REG_PIPE_EDP_CONF	0x7f008

/* bit fields of pipeconf */
#define _REGBIT_PIPE_ENABLE		(1 << 31)
#define _REGBIT_PIPE_STAT_ENABLED	(1 << 30)
#define _REGBIT_PIPE_BPC_MASK		(7 << 5) /* ironlake */
#define _REGBIT_PIPE_8BPC		(0 << 5)

/* bit fields of pipestat */

#define VGT_PIPEDSL(pipe)	_PIPE(pipe, _PIPEADSL, _REG_PIPEBDSL)
#define VGT_PIPECONF(pipe)	_PIPE(pipe, _PIPEACONF, _REG_PIPEBCONF)
#define VGT_PIPESTAT(pipe)	_PIPE(pipe, _PIPEASTAT, _REG_PIPEBSTAT)
#define VGT_PIPE_FRMCOUNT(pipe)	_PIPE(pipe, _PIPEA_FRMCOUNT_G4X, _REG_PIPEB_FRMCOUNT)
#define VGT_PIPE_FLIPCOUNT(pipe) _PIPE(pipe, _PIPEA_FLIPCOUNT_G4X, _REG_PIPEB_FLIPCOUNT)

#define VGT_PIPECONFPIPE(pipeconf) _VGT_GET_PIPE(pipeconf, _PIPEACONF, _REG_PIPEBCONF)
#define VGT_FRMCOUNTPIPE(frmcount) _VGT_GET_PIPE(frmcount, _PIPEA_FRMCOUNT_G4X, _REG_PIPEB_FRMCOUNT)

#define VGT_PALETTE(pipe) _PIPE(pipe, _PALETTE_A_OFFSET, _PALETTE_B_OFFSET)

/* legacy palette */

#define _REG_LGC_PALETTE_C		0x4b000

/* Display Port */

#define _REG_DP_TP_CTL_C		0x64240
#define _REG_DP_TP_CTL_D		0x64340
#define _REG_DP_TP_CTL_E		0x64440
#define  _REGBIT_DP_TP_FDI_AUTO_TRAIN_ENABLE	(1 << 15)
#define  _DDI_BUFCTL_DETECT_MASK	0x1
#define  _REGBIT_DDI_BUF_ENABLE		(1 << 31)
#define  _REGBIT_DDI_BUF_IS_IDLE	(1<<7)
#define _REG_DDI_BUF_CTL_C		0x64200
#define _REG_DDI_BUF_CTL_D		0x64300
#define _REG_DDI_BUF_CTL_E		0x64400
#define _REG_DP_TP_STATUS_C			0x64244
#define _REG_DP_TP_STATUS_D			0x64344
#define _REG_DP_TP_STATUS_E			0x64444
#define VGT_DP_TP_CTL(port)		_PORT(port, DP_TP_CTL_A, \
						DP_TP_CTL_B)

#define DP_TP_PORT(reg)		_VGT_GET_PORT(reg, DP_TP_CTL_A, \
						DP_TP_CTL_B)
#define DP_TP_STATUS(port)		_PORT(port, DP_TP_STATUS_A, \
						DP_TP_STATUS_B)

#define DRM_MODE_DPMS_ON		0

/* DPCD */
#define DP_SET_POWER		0x600
#define DP_SET_POWER_D0		0x1
#define AUX_NATIVE_WRITE	0x8
#define AUX_NATIVE_READ		0x9

#define AUX_NATIVE_REPLY_MASK	(0x3 << 4)
#define AUX_NATIVE_REPLY_ACK	(0x0 << 4)
#define AUX_NATIVE_REPLY_NAK	(0x1 << 4)
#define AUX_NATIVE_REPLY_DEFER	(0x2 << 4)

#define AUX_BURST_SIZE		16

/* DPCD 0x106 */

#define DP_TRAINING_PATTERN_SET			0x102
#define DP_TRAINING_PATTERN_DISABLE		0
#define DP_TRAINING_PATTERN_1			1
#define DP_TRAINING_PATTERN_2			2
#define DP_LINK_SCRAMBLING_DISABLE		(1 << 5)

#define DP_LINK_CONFIGURATION_SIZE		9
#define    DP_LINK_BW_SET			0x100
# define DP_SET_ANSI_8B10B			(1 << 0)

#define DP_LINK_STATUS_SIZE			6
#define DP_TRAIN_MAX_SWING_REACHED		(1 << 2)

#define DP_TRAINING_LANE0_SET			0x103

#define DP_TRAIN_VOLTAGE_SWING_MASK		0x3
#define DP_TRAIN_VOLTAGE_SWING_SHIFT		0
#define DP_TRAIN_VOLTAGE_SWING_400		(0 << 0)
#define DP_TRAIN_VOLTAGE_SWING_600		(1 << 0)
#define DP_TRAIN_VOLTAGE_SWING_800		(2 << 0)
#define DP_TRAIN_VOLTAGE_SWING_1200		(3 << 0)

#define DP_TRAIN_PRE_EMPHASIS_MASK		(3 << 3)
#define DP_TRAIN_PRE_EMPHASIS_0			(0 << 3)
#define DP_TRAIN_PRE_EMPHASIS_3_5		(1 << 3)
#define DP_TRAIN_PRE_EMPHASIS_6			(2 << 3)
#define DP_TRAIN_PRE_EMPHASIS_9_5		(3 << 3)

#define DP_TRAIN_PRE_EMPHASIS_SHIFT		3
#define DP_TRAIN_MAX_PRE_EMPHASIS_REACHED	(1 << 5)

#define DP_LANE0_1_STATUS			0x202
#define DP_LANE_CR_DONE				(1 << 0)

#define DP_LANE_ALIGN_STATUS_UPDATED		0x204
#define DP_INTERLANE_ALIGN_DONE			(1 << 0)
#define DP_LANE_CHANNEL_EQ_DONE			(1 << 1)
#define DP_LANE_SYMBOL_LOCKED			(1 << 2)

#define DP_ADJUST_REQUEST_LANE0_1		0x206

#define DP_ADJUST_VOLTAGE_SWING_LANE0_SHIFT 0
#define DP_ADJUST_VOLTAGE_SWING_LANE1_SHIFT 4
#define DP_ADJUST_PRE_EMPHASIS_LANE0_SHIFT  2
#define DP_ADJUST_PRE_EMPHASIS_LANE1_SHIFT  6
/* Ironlake */
#define	_REG_CPU_VGACNTRL	CPU_VGACNTRL
#define _REGBIT_VGA_DISPLAY_DISABLE	(1UL << 31)
#define _REG_DPFC_CB_BASE		0x43200
#define _REG_DPFC_CONTROL		0x43208
#define _REG_DPFC_RECOMP_CTL		0x4320c
#define _REG_DPFC_CPU_FENCE_OFFSET	0x43218
#define _REG_DPFC_CONTROL_SA		0x100100
#define _REG_DPFC_CPU_FENCE_OFFSET_SA	0x100104

#define _REG_CSC_A_COEFFICIENTS		0x49010
#define _REG_CSC_A_MODE			0x49028
#define _REG_PRECSC_A_HIGH_COLOR_CHANNEL_OFFSET		0x49030
#define _REG_PRECSC_A_MEDIUM_COLOR_CHANNEL_OFFSET	0x49034
#define _REG_PRECSC_A_LOW_COLOR_CHANNEL_OFFSET		0x49038

#define _REG_CSC_B_COEFFICIENTS		0x49110
#define _REG_CSC_B_MODE			0x49128
#define _REG_PRECSC_B_HIGH_COLOR_CHANNEL_OFFSET		0x49130
#define _REG_PRECSC_B_MEDIUM_COLOR_CHANNEL_OFFSET	0x49134
#define _REG_PRECSC_B_LOW_COLOR_CHANNEL_OFFSET		0x49138

#define _REG_CSC_C_COEFFICIENTS		0x49210
#define _REG_CSC_C_MODE			0x49228
#define _REG_PRECSC_C_HIGH_COLOR_CHANNEL_OFFSET		0x49230
#define _REG_PRECSC_C_MEDIUM_COLOR_CHANNEL_OFFSET	0x49234
#define _REG_PRECSC_C_LOW_COLOR_CHANNEL_OFFSET		0x49238

/*
 * Instruction and interrupt control regs
 */
#define _REG_HWS_PGA		0x02080
#define _REG_IER		0x020a0
#define _REG_IMR		0x020a8
#define _REG_DE_RRMR		0x44050

#define _REG_CACHE_MODE_0	0x02120 /* 915+ only */
#define _REG_CACHE_MODE_1	0x02124
#define	_REG_GEN3_MI_ARB_STATE	MI_ARB_STATE
#define _REG_SWF		0x4f000

#define _REG_DP_BUFTRANS	0xe4f00

/* digital port hotplug */
/* GMBUS1 bits definitions */

#define GMBUS1_TOTAL_BYTES_SHIFT 16
#define GMBUS1_TOTAL_BYTES_MASK 0x1ff
#define gmbus1_total_byte_count(v) (((v) >> GMBUS1_TOTAL_BYTES_SHIFT) & GMBUS1_TOTAL_BYTES_MASK)
#define gmbus1_slave_addr(v) (((v) & 0xff) >> 1)
#define gmbus1_slave_index(v) (((v) >> 8) & 0xff)
#define gmbus1_bus_cycle(v) (((v) >> 25) & 0x7)

/* GMBUS0 bits definitions */
#define _GMBUS_PIN_SEL_MASK	(0x7)

#define _REG_RC_PWRCTX_MAXCNT		0x2054
#define _REG_VFSKPD			0x2470
#define _REG_2D_CG_DIS			0x6200
#define _REG_3D_CG_DIS			0x6204			
#define _REG_3D_CG_DIS2			0x6208
#define _REG_SUPER_QUEUE_CONFIG		0x902c

/* interrupt related definitions */
#define	_REGSHIFT_MASTER_INTERRUPT	31
#define	_REGSHIFT_PCH	21
#define	_REGBIT_PCH	(1 << 21)
/* GEN7 */
#define	_REGSHIFT_PCH_GEN7	28
#define	_REGBIT_PCH_GEN7	(1 << 28)

#define	_REGBIT_DP_A_PULSE_DURATION	(3 << 2)

#define	_REGBIT_CRT_HOTPLUG	(1 << 19)
#define	_REGBIT_DP_B_HOTPLUG	(1 << 21)
#define	_REGBIT_DP_C_HOTPLUG	(1 << 22)
#define	_REGBIT_DP_D_HOTPLUG	(1 << 23)

#define        _REGBIT_DP_B_STATUS			(3 << 0)
#define        _REGBIT_DP_B_PULSE_DURATION		(3 << 2)
#define        _REGBIT_DP_B_ENABLE			(1 << 4)
#define        _REGBIT_DP_C_STATUS			(3 << 8)
#define        _REGBIT_DP_C_PULSE_DURATION		(3 << 10)
#define        _REGBIT_DP_C_ENABLE			(1 << 12)
#define        _REGBIT_DP_D_STATUS			(3 << 16)
#define        _REGBIT_DP_D_PULSE_DURATION		(3 << 18)
#define        _REGBIT_DP_D_ENABLE			(1 << 20)

#define _REG_RCS_WATCHDOG_CTL	0x2178
#define _REG_RCS_WATCHDOG_THRSH	0x217C
#define _REG_RCS_WATCHDOG_CTR	0x2190
#define _REG_VCS_WATCHDOG_CTR	0x12178
#define _REG_VCS_WATCHDOG_THRSH	0x1217C
#define _REG_BCS_EIR	0x220B0
#define _REG_BCS_EMR	0x220B4
#define _REG_BCS_ESR	0x220B8
#define _REG_VCS_EIR	0x120B0
#define _REG_VCS_EMR	0x120B4
#define _REG_VCS_ESR	0x120B8
#define _REG_VECS_EIR	0x1A0B0
#define _REG_VECS_EMR	0x1A0B4
#define _REG_VECS_ESR	0x1A0B8
#define RING_EIR(ring) \
	__RING_REG((ring), EIR)
#define RING_EMR(ring) \
	__RING_REG((ring), EMR)
#define RING_ESR(ring) \
	__RING_REG((ring), ESR)

#define RING_REG_2064(ring) \
	({ASSERT((ring) > 0); \
	 __RING_REG((ring), 0x2064);})

#define RING_REG_2068(ring) \
	__RING_REG((ring), 0x2068)

#define RING_REG_2078(ring) \
	__RING_REG((ring), 0x2078)

#define RING_REG_206C(ring) \
	__RING_REG((ring), 0x206C)

/* blacklight PWM control */

#define _REG_HISTOGRAM_THRSH	0x48268
#define        _REGBIT_HISTOGRAM_IRQ_ENABLE	(1 << 31)
#define        _REGBIT_HISTOGRAM_IRQ_STATUS	(1 << 30)

/*
 * Next MACROs for GT configuration space.
 */
#define VGT_PCI_CLASS_VGA			0x03
#define VGT_PCI_CLASS_VGA_OTHER			0x80

#define VGT_REG_CFG_VENDOR_ID			0x00
#define VGT_REG_CFG_COMMAND			0x04
#define _REGBIT_CFG_COMMAND_IO			(1 << 0)
#define _REGBIT_CFG_COMMAND_MEMORY		(1 << 1)
#define _REGBIT_CFG_COMMAND_MASTER		(1 << 2)
#define VGT_REG_CFG_CLASS_PROG_IF		0x09
#define VGT_REG_CFG_SUB_CLASS_CODE		0x0A
#define VGT_REG_CFG_CLASS_CODE			0x0B
#define VGT_REG_CFG_SPACE_BAR0			0x10
#define VGT_REG_CFG_SPACE_BAR1			0x18
#define VGT_REG_CFG_SPACE_BAR2			0x20
#define VGT_REG_CFG_SPACE_BAR_ROM		0x30
#define VGT_REG_CFG_SPACE_MSAC			0x62
#define VGT_REG_CFG_SWSCI_TRIGGER		0xE8
#define	_REGBIT_CFG_SWSCI_SCI_SELECT		(1 << 15)
#define	_REGBIT_CFG_SWSCI_SCI_TRIGGER		1
#define VGT_REG_CFG_OPREGION			0xFC

#define VGT_OPREGION_PAGES			2
#define VGT_OPREGION_PORDER			1
#define VGT_OPREGION_SIZE			(8 * 1024)
#define VGT_OPREGION_REG_CLID			0x1AC
#define VGT_OPREGION_REG_SCIC			0x200
#define _REGBIT_OPREGION_SCIC_FUNC_MASK		0x1E
#define _REGBIT_OPREGION_SCIC_FUNC_SHIFT	1
#define _REGBIT_OPREGION_SCIC_SUBFUNC_MASK	0xFF00
#define _REGBIT_OPREGION_SCIC_SUBFUNC_SHIFT	8
#define _REGBIT_OPREGION_SCIC_EXIT_MASK		0xE0
#define VGT_OPREGION_SCIC_F_GETBIOSDATA		4
#define VGT_OPREGION_SCIC_F_GETBIOSCALLBACKS	6
#define VGT_OPREGION_SCIC_SF_SUPPRTEDCALLS	0
#define VGT_OPREGION_SCIC_SF_REQEUSTEDCALLBACKS	1
#define VGT_OPREGION_REG_PARM			0x204

#define MSAC_APERTURE_SIZE_128M			(0 << 1)
#define MSAC_APERTURE_SIZE_256M			(1 << 1)
#define MSAC_APERTURE_SIZE_512M			(3 << 1)

/*
 * Configuration register definition for BDF: 0:0:0.
 */
#define _REG_GMCH_CONTRL		0x50
#define    _REGBIT_SNB_GMCH_GMS_SHIFT   3 /* Graphics Mode Select */
#define    _REGBIT_SNB_GMCH_GMS_MASK    0x1f
#define    _REGBIT_BDW_GMCH_GMS_SHIFT   8
#define    _REGBIT_BDW_GMCH_GMS_MASK    0xff

/* HSW */

#define  _REGBIT_SPLL_CTL_ENABLE	(1 << 31)

#define _REG_PORT_CLK_SEL_DDIC	0x46108
#define _REG_PORT_CLK_SEL_DDID	0x4610C
#define _REG_PORT_CLK_SEL_DDIE	0x46110

#define _REG_TRANS_CLK_SEL_C	0x46148
#define SBI_RESPONSE_MASK		0x3
#define SBI_RESPONSE_SHIFT		0x1
#define SBI_STAT_MASK			0x1
#define SBI_STAT_SHIFT			0x0
#define SBI_OPCODE_SHIFT		8
#define SBI_OPCODE_MASK		(0xff << SBI_OPCODE_SHIFT)
#define SBI_CMD_IORD			2
#define SBI_CMD_IOWR			3
#define SBI_CMD_CRRD			6
#define SBI_CMD_CRWR			7
#define SBI_ADDR_OFFSET_SHIFT		16
#define SBI_ADDR_OFFSET_MASK		(0xffff << SBI_ADDR_OFFSET_SHIFT)

#define _VGT_TRANS_DDI_FUNC_CTL(tran)   _TRANSCODER(tran, TRANS_DDI_FUNC_CTL_A, \
						   TRANS_DDI_FUNC_CTL_B)

/* Those bits are ignored by pipe EDP since it can only connect to DDI A */
#define  _TRANS_DDI_MODE_SELECT_HIFT		24
#define  _TRANS_DDI_EDP_INPUT_SHIFT		12

#define _REG_HSW_DP_AUX_CH_CTL(dp)	\
	((dp)? (PCH_DPB_AUX_CH_CTL + ((dp)-1)*0x100) : DPA_AUX_CH_CTL)

#define _REG_SKL_DP_AUX_CH_CTL(dp) (0x64010 + (dp) * 0x100)

#define _REG_GEN7_SQ_CHICKEN_MBCUNIT_CONFIG		0x9030
#define GEN7_L3SQCREG1				0xB010
#define  VLV_B0_WA_L3SQCREG1_VALUE		0x00D30000

#define GEN8_L3SQCREG1				0xB100
#define  BDW_WA_L3SQCREG1_DEFAULT		0x784000

#define GEN7_L3CNTLREG3				0xB024
#define _REG_PIPE_WM_LINETIME_C			0x45278

#define _REG_HSW_VIDEO_DIP_CTL_C		0x62200
#define _REG_HSW_VIDEO_DIP_CTL_EDP		0x6F200
/* GEN8 interrupt registers definations */

#define _REG_GT_ISR(which) (0x44300 + (0x10 * (which)))
#define _REG_GT_IMR(which) (0x44304 + (0x10 * (which)))
#define _REG_GT_IIR(which) (0x44308 + (0x10 * (which)))
#define _REG_GT_IER(which) (0x4430c + (0x10 * (which)))

#define _REG_DE_PIPE_ISR(pipe) (0x44400 + (0x10 * (pipe)))
#define _REG_DE_PIPE_IMR(pipe) (0x44404 + (0x10 * (pipe)))
#define _REG_DE_PIPE_IIR(pipe) (0x44408 + (0x10 * (pipe)))
#define _REG_DE_PIPE_IER(pipe) (0x4440c + (0x10 * (pipe)))

#define _REG_GEN8_PRIVATE_PAT  0x40e0

#define _REG_RING_PDP_UDW(base, n)      (base + 0x270 + ((n) * 8 + 4))
#define _REG_RING_PDP_LDW(base, n)      (base + 0x270 + (n) * 8)

#define _REG_RCS_PDP_UDW(n)	_REG_RING_PDP_UDW(0x2000, n)
#define _REG_RCS_PDP_LDW(n)	_REG_RING_PDP_LDW(0x2000, n)

#define _REG_VCS_PDP_UDW(n)	_REG_RING_PDP_UDW(0x12000, n)
#define _REG_VCS_PDP_LDW(n)	_REG_RING_PDP_LDW(0x12000, n)

#define _REG_VCS2_PDP_UDW(n)	_REG_RING_PDP_UDW(0x1c000, n)
#define _REG_VCS2_PDP_LDW(n)	_REG_RING_PDP_LDW(0x1c000, n)

#define _REG_VECS_PDP_UDW(n)	_REG_RING_PDP_UDW(0x1a000, n)
#define _REG_VECS_PDP_LDW(n)	_REG_RING_PDP_LDW(0x1a000, n)

#define _REG_BCS_PDP_UDW(n)	_REG_RING_PDP_UDW(0x22000, n)
#define _REG_BCS_PDP_LDW(n)	_REG_RING_PDP_LDW(0x22000, n)

#define _REG_CUR_DESC(n)	(0x4400 + (n)*0x40)

#define _REG_RCS_BB_STATE	0x2110
#define _REG_RCS_BB_HEAD	0x2140
#define _REG_RCS_BB_HEAD_U	0x2168
#define _REG_RCS_BB_START	0x2150
#define _REG_RCS_BB_START_U	0x2170

#define _REG_BB_STATE(ring)	__RING_REG((ring), _REG_RCS_BB_STATE)
#define _REG_BB_HEAD(ring)	__RING_REG((ring), _REG_RCS_BB_HEAD)
#define _REG_BB_HEAD_U(ring)	__RING_REG((ring), _REG_RCS_BB_HEAD_U)
#define _REG_BB_START(ring)	__RING_REG((ring), _REG_RCS_BB_START)
#define _REG_BB_START_U(ring)	__RING_REG((ring), _REG_RCS_BB_START_U)

#define _REG_RCS_SBB_STATE	0x2118
#define _REG_RCS_SBB_HEAD	0x2114
#define _REG_RCS_SBB_HEAD_U	0x211C

#define _REG_SBB_STATE(ring)	__RING_REG((ring), _REG_RCS_SBB_STATE)
#define _REG_SBB_HEAD(ring)	__RING_REG((ring), _REG_RCS_SBB_HEAD)
#define _REG_SBB_HEAD_U(ring)	__RING_REG((ring), _REG_RCS_SBB_HEAD_U)

#define _REG_701C0(pipe, plane) (0x701c0 + pipe * 0x1000 + (plane - 1) * 0x100)
#define _REG_701C4(pipe, plane) (0x701c4 + pipe * 0x1000 + (plane - 1) * 0x100)

#define _RING_FAULT_REG(ring)	(0x4094 + 0x100*(ring))

#endif	/* _VGT_REG_H_ */
