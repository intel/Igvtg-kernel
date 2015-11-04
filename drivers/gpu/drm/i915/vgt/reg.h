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

/*
 * Definition of MMIO registers.
 */
#define _VGT_MMIO_THROUGH_OFFSET(index, a, b)	((a) + (index)*((b)-(a)))
#define _VGT_MMIO_GET_INDEX(reg, a, b)		(((reg)-(a))/((b)-(a)))

#define _VGT_PIPE(pipe, a, b)		_VGT_MMIO_THROUGH_OFFSET(pipe, a, b)
#define _VGT_PORT(port, a, b)		_VGT_MMIO_THROUGH_OFFSET(port, a, b)
#define _VGT_TRANSCODER(tran, a, b)   _VGT_MMIO_THROUGH_OFFSET(tran, a, b)

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

#define _MASKED_BIT_ENABLE(a) (((a) << 16) | (a))
#define _MASKED_BIT_DISABLE(a) ((a) << 16)

/*
 * Registers used only by the command parser
 */
#define _REG_BCS_SWCTRL 0x22200

#define _REG_HS_INVOCATION_COUNT 0x2300
#define _REG_DS_INVOCATION_COUNT 0x2308
#define _REG_IA_VERTICES_COUNT   0x2310
#define _REG_IA_PRIMITIVES_COUNT 0x2318
#define _REG_VS_INVOCATION_COUNT 0x2320
#define _REG_GS_INVOCATION_COUNT 0x2328
#define _REG_GS_PRIMITIVES_COUNT 0x2330
#define _REG_CL_INVOCATION_COUNT 0x2338
#define _REG_CL_PRIMITIVES_COUNT 0x2340
#define _REG_PS_INVOCATION_COUNT 0x2348
#define _REG_PS_DEPTH_COUNT      0x2350


/* PRB0, RCS */
#define _REG_RCS_TAIL		0x02030
#define _REG_RCS_HEAD		0x02034
#define _REG_RCS_START		0x02038
#define _REG_RCS_CTL		0x0203c

/* VECS: HSW+ */
#define _REG_VECS_TAIL		0x1A030
#define _REG_VECS_HEAD		0x1A034
#define _REG_VECS_START		0x1A038
#define _REG_VECS_CTL		0x1A03c

/* VCS */
#define _REG_VCS_TAIL		0x12030
#define _REG_VCS_HEAD		0x12034
#define _REG_VCS_START		0x12038
#define _REG_VCS_CTL		0x1203c

/* VCS2 for BDW GT3 */
#define _REG_VCS2_TAIL		0x1C030
#define _REG_VCS2_HEAD		0x1C034
#define _REG_VCS2_START		0x1C038
#define _REG_VCS2_CTL		0x1C03C

/* BCS */
#define _REG_BCS_TAIL		0x22030
#define _REG_BCS_HEAD		0x22034
#define _REG_BCS_START		0x22038
#define _REG_BCS_CTL		0x2203c

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
#define _RING_CTL_RB_WAIT	(1 << 11)

#define _REG_CCID		0x02180
#define CCID_MBO_BITS		(1 << 8)	/* bit 8 must be one */
#define CCID_EXTENDED_STATE_SAVE_ENABLE		(1 << 3)
#define CCID_EXTENDED_STATE_RESTORE_ENABLE	(1 << 2)
#define CCID_VALID		(1 << 0)
#define _REG_CXT_SIZE		0x021a0
#define _REG_GEN7_CXT_SIZE	0x021a8
#define _REG_VECS_CXT_SIZE	0x1A1A8

#define _REG_RCS_MI_MODE	0x209C
#define        _REGBIT_MI_ASYNC_FLIP_PERFORMANCE_MODE	(1 << 14)
#define        _REGBIT_MI_FLUSH_PERFORMANCE_MODE	(1 << 13)
//#define        _REGBIT_MI_FLUSH			(3 << 11)
#define        _REGBIT_MI_FLUSH				(1 << 12)
#define        _REGBIT_MI_INVALIDATE_UHPTR		(1 << 11)
#define        _REGBIT_MI_RINGS_IDLE			(1 << 9)
#define        _REGBIT_MI_STOP_RINGS			(1 << 8)
#define _REG_VCS_MI_MODE	0x1209C
#define _REG_VCS2_MI_MODE	0x1c09C
#define _REG_BCS_MI_MODE	0x2209C
#define _REG_VECS_MI_MODE	0x1A09c
#define _REG_GFX_MODE	0x2520
#define        _REGBIT_FLUSH_TLB_INVALIDATION_MODE	(1 << 13)
#define        _REGBIT_REPLAY_MODE			(1 << 11)
#define        _REGBIT_PPGTT_ENABLE			(1 << 9)
#define        _REGBIT_PPGTT64_ENABLE                   (1 << 7)
#define _REG_ARB_MODE	0x4030
#define        _REGBIT_ADDRESS_SWIZZLING		(3 << 4)
#define _REG_GT_MODE	0x20D0

#define _REG_GAC_MODE		0x120A0
#define _REG_GAB_MODE		0x220A0

#define _REG_RCS_INSTPM		0x20C0
#define _REG_VCS_INSTPM		0x120C0
#define _REG_VCS2_INSTPM	0x1c0C0
#define _REG_BCS_INSTPM		0x220C0
#define _REG_VECS_INSTPM	0x1A0C0
#define     _REGBIT_INSTPM_SYNC_FLUSH		(1<<5)
#define     _REGBIT_INSTPM_FORCE_ORDERING	(1<<7) /* GEN6+ */
#define     _REGBIT_INSTPM_TLB_INVALIDATE	(1<<9)

#define INSTPM_CONS_BUF_ADDR_OFFSET_DIS (1<<6)

/* IVB+ */
#define _REG_BCS_BLT_MODE_IVB	0x2229C
#define _REG_RCS_GFX_MODE_IVB	0x0229C
#define _REG_VCS_MFX_MODE_IVB	0x1229C
#define _REG_VCS2_MFX_MODE_BDW	0x1c29C
#define  _REGBIT_EXECLIST_ENABLE       (1 << 15)
#define _REG_CACHE_MODE_0_IVB	0x7000
#define _REG_CACHE_MODE_1_IVB	0x7004
#define _REG_GT_MODE_IVB	0x7008
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

#define _REG_RCS_IMR		0x20A8
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

#define _REG_RCS_HWS_PGA	0x4080
#define _REG_VCS_HWS_PGA	0x4180
#define _REG_BCS_HWS_PGA	0x24080
#define _REG_BCS_HWS_PGA_GEN7	0x4280
#define _REG_VECS_HWS_PGA	0x1A080
#define _REG_VEBOX_HWS_PGA_GEN7	0x4380

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

#define _REG_RCS_ACTHD		0x2074
#define _REG_VCS_ACTHD		0x12074
#define _REG_BCS_ACTHD		0x22074
#define _REG_VECS_ACTHD		0x1A074
#define _REG_VCS2_ACTHD		0x1c074

#define _REG_RCS_ACTHD_UDW	0x205c
#define _REG_VCS_ACTHD_UDW	0x1205c
#define _REG_BCS_ACTHD_UDW	0x2205c
#define _REG_VECS_ACTHD_UDW	0x1A05c
#define _REG_VCS2_ACTHD_UDW	0x1c05c

#define VGT_ACTHD(ring_id) __RING_REG(ring_id, _REG_RCS_ACTHD)

#define _REG_RCS_HWSTAM		0x2098
#define _REG_VCS_HWSTAM		0x12098
#define _REG_BCS_HWSTAM		0x22098
#define _REG_VECS_HWSTAM	0x1A098
#define _REG_VCS2_HWSTAM	0x1c098

#define _REG_RCS_BB_PREEMPT_ADDR	0x2148

#define _REG_RCS_BB_ADDR_DIFF		0x2154
#define _REG_RCS_FBC_RT_BASE_ADDR	0x2128
#define _REG_IVB_RCS_FBC_RT_BASE_ADDR	0X7020

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
#define 	_REGBIT_FENCE_VALID	(1 << 0)

#define _REG_CURACNTR		0x70080
#define    _CURSOR_MODE			0x3f
#define    _CURSOR_MODE_DISABLE		0x00
#define    _CURSOR_ALPHA_FORCE_SHIFT	8
#define    _CURSOR_ALPHA_FORCE_MASK	(0x3 << _CURSOR_ALPHA_FORCE_SHIFT)
#define    _CURSOR_ALPHA_PLANE_SHIFT	10
#define    _CURSOR_ALPHA_PLANE_MASK	(0x3 << _CURSOR_ALPHA_PLANE_SHIFT)
#define _REG_CURABASE		0x70084
#define _REG_CURAPOS		0x70088
#define    _CURSOR_POS_X_SHIFT		0
#define    _CURSOR_POS_X_MASK		(0x1fff << _CURSOR_POS_X_SHIFT)
#define    _CURSOR_SIGN_X_SHIFT	15
#define    _CURSOR_SIGN_X_MASK		(1 << _CURSOR_SIGN_X_SHIFT)
#define    _CURSOR_POS_Y_SHIFT		16
#define    _CURSOR_POS_Y_MASK		(0xfff << _CURSOR_POS_Y_SHIFT)
#define    _CURSOR_SIGN_Y_SHIFT	31
#define    _CURSOR_SIGN_Y_MASK		(1 << _CURSOR_SIGN_Y_SHIFT)
#define _REG_CURASURFLIVE	0x700AC

#define _REG_CURAPALET_0	0x70090
#define _REG_CURAPALET_1	0x70094
#define _REG_CURAPALET_2	0x70098
#define _REG_CURAPALET_3	0x7009C

#define _REG_CURBCNTR_SNB	0x700C0
#define _REG_CURBBASE_SNB	0x700C4
#define _REG_CURBPOS_SNB	0x700C8
#define _REG_CURBSURFLIVE_SNB	0x700EC

#define _REG_CURBCNTR		0x71080
#define _REG_CURBBASE		0x71084
#define _REG_CURBPOS		0x71088
#define _REG_CURBSURFLIVE	0x710AC

#define _REG_CURCCNTR		0x72080
#define _REG_CURCBASE		0x72084
#define _REG_CURCPOS		0x72088
#define _REG_CURCSURFLIVE	0x720AC

#define VGT_CURCNTR_SNB(pipe)	_VGT_PIPE(pipe, _REG_CURACNTR, _REG_CURBCNTR_SNB)
#define VGT_CURBASE_SNB(pipe)	_VGT_PIPE(pipe, _REG_CURABASE, _REG_CURBBASE_SNB)
#define VGT_CURPOS_SNB(pipe)	_VGT_PIPE(pipe, _REG_CURAPOS, _REG_CURBPOS_SNB)

#define VGT_CURCNTR(pipe)	_VGT_PIPE(pipe, _REG_CURACNTR, _REG_CURBCNTR)
#define VGT_CURBASE(pipe)	_VGT_PIPE(pipe, _REG_CURABASE, _REG_CURBBASE)
#define VGT_CURPOS(pipe)	_VGT_PIPE(pipe, _REG_CURAPOS, _REG_CURBPOS)

#define _REG_DSPACNTR		0x70180
#define    _PRI_PLANE_ENABLE		(1 << 31)
#define    _PRI_PLANE_GAMMA_ENABLE	(1 << 30)
#define    _PRI_PLANE_FMT_SHIFT		26
#define    _PRI_PLANE_FMT_MASK		(0xf << _PRI_PLANE_FMT_SHIFT)
#define    _PRI_PLANE_TRICKLE_FEED_DISABLE	(1 << 14)
#define    _PRI_PLANE_TILE_SHIFT		10
#define    _PRI_PLANE_TILE_MASK		(1 << _PRI_PLANE_TILE_SHIFT)

#define _REG_DSPALINOFF		0x70184
#define _REG_DSPASTRIDE		0x70188
#define    _PRI_PLANE_STRIDE_SHIFT	6
#define    _PRI_PLANE_STRIDE_MASK	(0x3ff << _PRI_PLANE_STRIDE_SHIFT)
#define _REG_DSPAPOS		0x7018C /* reserved */
#define _REG_DSPASIZE		0x70190
#define _REG_DSPASURF		0x7019C
#define _REG_DSPATILEOFF	0x701A4
#define     _PRI_PLANE_X_OFF_SHIFT	0
#define     _PRI_PLANE_X_OFF_MASK	(0x1fff << _PRI_PLANE_X_OFF_SHIFT)
#define     _PRI_PLANE_Y_OFF_SHIFT	16
#define     _PRI_PLANE_Y_OFF_MASK	(0xfff << _PRI_PLANE_Y_OFF_SHIFT)
#define _REG_DSPASURFLIVE	0x701AC

#define _REG_DSPBCNTR		0x71180
#define _REG_DSPBLINOFF		0x71184
#define _REG_DSPBSTRIDE		0x71188
#define _REG_DSPBPOS		0x7118C
#define _REG_DSPBSIZE		0x71190
#define _REG_DSPBSURF		0x7119C
#define _REG_DSPBTILEOFF	0x711A4
#define _REG_DSPBSURFLIVE	0x711AC

#define _REG_DSPCCNTR		0x72180
#define _REG_DSPCLINOFF		0x72184
#define _REG_DSPCSTRIDE		0x72188
#define _REG_DSPCPOS		0x7218C
#define _REG_DSPCSIZE		0x72190
#define _REG_DSPCSURF		0x7219C
#define _REG_DSPCTILEOFF	0x721A4
#define _REG_DSPCSURFLIVE	0x721AC

#define VGT_DSPSURF(pipe)	_VGT_PIPE(pipe, _REG_DSPASURF, _REG_DSPBSURF)
#define VGT_DSPCNTR(pipe)	_VGT_PIPE(pipe, _REG_DSPACNTR, _REG_DSPBCNTR)
#define VGT_DSPCNTRPIPE(dspcntr) _VGT_GET_PIPE(dspcntr, _REG_DSPACNTR,_REG_DSPBCNTR)

#define VGT_DSPLINOFF(plane) _VGT_PIPE(plane, _REG_DSPALINOFF, _REG_DSPBLINOFF)
#define VGT_DSPSTRIDE(plane) _VGT_PIPE(plane, _REG_DSPASTRIDE, _REG_DSPBSTRIDE)
#define VGT_DSPTILEOFF(plane) _VGT_PIPE(plane, _REG_DSPATILEOFF, _REG_DSPBTILEOFF)

#define VGT_DSPSURFPIPE(dspsurf) _VGT_GET_PIPE(dspsurf, _REG_DSPASURF,_REG_DSPBSURF)
#define VGT_DSPSURFLIVEPIPE(dspsurf) _VGT_GET_PIPE(dspsurf, _REG_DSPASURFLIVE, \
							_REG_DSPBSURFLIVE)
#define VGT_DSPSURFLIVE(pipe)	_VGT_PIPE(pipe, _REG_DSPASURFLIVE, _REG_DSPBSURFLIVE)

#define VGT_CURSURFPIPE(cursurf) _VGT_GET_PIPE(cursurf, _REG_CURABASE, _REG_CURBBASE)
#define VGT_CURSURF(pipe)	_VGT_PIPE(pipe, _REG_CURABASE, _REG_CURBBASE)

/* sprite */

#define _REG_SPRA_CTL				0x70280
#define    _SPRITE_ENABLE		(1 << 31)
#define    _SPRITE_FMT_SHIFT		25
#define    _SPRITE_FMT_MASK		(0x7 << _SPRITE_FMT_SHIFT)
#define    _SPRITE_COLOR_ORDER_SHIFT	20
#define    _SPRITE_COLOR_ORDER_MASK	(0x1 << _SPRITE_COLOR_ORDER_SHIFT)
#define    _SPRITE_YUV_ORDER_SHIFT	16
#define    _SPRITE_YUV_ORDER_MASK	(0x3 << _SPRITE_YUV_ORDER_SHIFT)
#define    _SPRITE_TILED		(1 << 10)

#define _REG_SPRA_STRIDE			0x70288
#define    _SPRITE_STRIDE_SHIFT		6
#define    _SPRITE_STRIDE_MASK		(0x1ff << _SPRITE_STRIDE_SHIFT)

#define _REG_SPRASURF				0x7029C

#define _REG_SPRASURFLIVE			0x702AC

#define _REG_SPRA_SCALE				0x70304

#define _REG_SPRA_POS				0x7028c
#define    _SPRITE_POS_X_SHIFT		0
#define    _SPRITE_POS_Y_SHIFT		16
#define    _SPRITE_POS_X_MASK		(0x1fff << _SPRITE_POS_X_SHIFT)
#define    _SPRITE_POS_Y_MASK		(0xfff << _SPRITE_POS_Y_SHIFT)

#define _REG_SPRA_SIZE				0x70290
#define    _SPRITE_SIZE_WIDTH_SHIFT		0
#define    _SPRITE_SIZE_HEIGHT_SHIFT		16
#define    _SPRITE_SIZE_WIDTH_MASK		(0x1fff << _SPRITE_SIZE_WIDTH_SHIFT)
#define    _SPRITE_SIZE_HEIGHT_MASK		(0xfff << _SPRITE_SIZE_HEIGHT_SHIFT)

#define _REG_SPRA_OFFSET			0x702a4
#define    _SPRITE_OFFSET_START_X_SHIFT	0
#define    _SPRITE_OFFSET_START_Y_SHIFT	16
#define    _SPRITE_OFFSET_START_X_MASK	(0x1fff << _SPRITE_OFFSET_START_X_SHIFT)
#define    _SPRITE_OFFSET_START_Y_MASK	(0xfff << _SPRITE_OFFSET_START_Y_SHIFT)

#define _REG_SPRB_CTL				0x71280
#define _REG_SPRB_STRIDE			0x71288
#define _REG_SPRB_POS				0x7128c
#define _REG_SPRB_SIZE				0x71290
#define _REG_SPRB_OFFSET			0x712a4
#define _REG_SPRB_SCALE				0x71304

#define _REG_SPRB_CTL				0x71280
#define _REG_SPRB_STRIDE			0x71288
#define _REG_SPRBSURF				0x7129C
#define _REG_SPRBSURFLIVE			0x712AC
#define _REG_SPRB_SCALE				0x71304

#define _REG_SPRC_CTL				0x72280
#define _REG_SPRC_STRIDE			0x72288
#define _REG_SPRCSURF				0x7229C
#define _REG_SPRCSURFLIVE			0x722AC
#define _REG_SPRC_SCALE				0x72304

#define VGT_SPRCTL(pipe)	_VGT_PIPE(pipe, _REG_SPRA_CTL, _REG_SPRB_CTL)
#define VGT_SPRSTRIDE(pipe)	_VGT_PIPE(pipe, _REG_SPRA_STRIDE, _REG_SPRB_STRIDE)
#define VGT_SPRPOS(pipe)	_VGT_PIPE(pipe, _REG_SPRA_POS, _REG_SPRB_POS)
#define VGT_SPRSIZE(pipe)	_VGT_PIPE(pipe, _REG_SPRA_SIZE, _REG_SPRB_SIZE)
#define VGT_SPRSURF(pipe)	_VGT_PIPE(pipe, _REG_SPRASURF, _REG_SPRBSURF)
#define VGT_SPRSURFPIPE(sprsurf) _VGT_GET_PIPE(sprsurf, _REG_SPRASURF, _REG_SPRBSURF)
#define VGT_SPRSURFLIVE(pipe)	_VGT_PIPE(pipe, _REG_SPRASURFLIVE, _REG_SPRBSURFLIVE)
#define VGT_SPROFFSET(pipe)	_VGT_PIPE(pipe, _REG_SPRA_OFFSET, _REG_SPRB_OFFSET)

#define VGT_SPRCNTRPIPE(sprcntr) _VGT_GET_PIPE(sprcntr, _REG_SPRA_CTL,_REG_SPRB_CTL)
#define VGT_CURCNTRPIPE(curcntr) _VGT_GET_PIPE(curcntr, _REG_CURACNTR,_REG_CURBCNTR)

#define _REG_DVSACNTR		0x72180
#define _REG_DVSALINOFF		0x72184
#define _REG_DVSASTRIDE		0x72188
#define _REG_DVSAPOS		0x7218C
#define _REG_DVSASIZE		0x72190
#define _REG_DVSAKEYVAL		0x72194
#define _REG_DVSAKEYMSK		0x72198
#define _REG_DVSASURF		0x7219C
#define _REG_DVSAKEYMAXVAL	0x721A0
#define _REG_DVSATILEOFF	0x721A4
#define _REG_DVSASURFLIVE	0x721AC
#define _REG_DVSASCALE		0x72204
/* DVSAGAMC: 0x72300 - 0x7234B */

#define _REG_DVSBCNTR		0x73180
#define _REG_DVSBLINOFF		0x73184
#define _REG_DVSBSTRIDE		0x73188
#define _REG_DVSBPOS		0x7318C
#define _REG_DVSBSIZE		0x73190
#define _REG_DVSBKEYVAL		0x73194
#define _REG_DVSBKEYMSK		0x73198
#define _REG_DVSBSURF		0x7319C
#define _REG_DVSBKEYMAXVAL	0x731A0
#define _REG_DVSBTILEOFF	0x731A4
#define _REG_DVSBSURFLIVE	0x731AC
#define _REG_DVSBSCALE		0x73204
/* DVSBGAMC: 0x73300 - 0x7334B */

#define _REG_PCH_DPB_AUX_CH_CTL		0xe4110
#define _REG_PCH_DPB_AUX_CH_DATA1	0xe4114
#define _REG_PCH_DPB_AUX_CH_DATA2	0xe4118
#define _REG_PCH_DPB_AUX_CH_DATA3	0xe411c
#define _REG_PCH_DPB_AUX_CH_DATA4	0xe4120
#define _REG_PCH_DPB_AUX_CH_DATA5	0xe4124

#define _REG_PCH_DPC_AUX_CH_CTL		0xe4210
#define _REG_PCH_DPC_AUX_CH_DATA1	0xe4214
#define _REG_PCH_DPC_AUX_CH_DATA2	0xe4218
#define _REG_PCH_DPC_AUX_CH_DATA3	0xe421c
#define _REG_PCH_DPC_AUX_CH_DATA4	0xe4220
#define _REG_PCH_DPC_AUX_CH_DATA5	0xe4224

#define _REG_PCH_DPD_AUX_CH_CTL		0xe4310
#define _REG_PCH_DPD_AUX_CH_DATA1	0xe4314
#define _REG_PCH_DPD_AUX_CH_DATA2	0xe4318
#define _REG_PCH_DPD_AUX_CH_DATA3	0xe431c
#define _REG_PCH_DPD_AUX_CH_DATA4	0xe4320
#define _REG_PCH_DPD_AUX_CH_DATA5	0xe4324

#define _REGBIT_DP_AUX_CH_CTL_SEND_BUSY		(1 << 31)
#define _REGBIT_DP_AUX_CH_CTL_DONE		(1 << 30)
#define _REGBIT_DP_AUX_CH_CTL_INTERRUPT		(1 << 29)
#define _REGBIT_DP_AUX_CH_CTL_TIME_OUT_ERR	(1 << 28)
#define _REGBIT_DP_AUX_CH_CTL_RECV_ERR		(1 << 25)
#define _REGBIT_DP_AUX_CH_CTL_MESSAGE_SIZE_MASK	(0x1f << 20)
#define _REGBIT_DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT	20
#define _REGBIT_DP_AUX_CH_CTL_TIME_OUT_400us	(0 << 26)
#define _DP_DETECTED				(1 << 2)
#define _DP_AUX_CH_CTL_BIT_CLOCK_2X_SHIFT	0
#define _DP_AUX_CH_CTL_PRECHARGE_2US_SHIFT	16
#define _REG_FORCEWAKE		0xA18C
#define _REG_FORCEWAKE_ACK	0x130090
#define _REG_MUL_FORCEWAKE	0xA188
#define _REG_MUL_FORCEWAKE_ACK	 0x130040
#define _REG_FORCEWAKE_ACK_HSW	0x130044
#define _REG_ECOBUS		0xA180
#define        ECOBUS_FORCEWAKE_MT_ENABLE	(1<<5)
#define _REGBIT_MUL_FORCEWAKE_ENABLE		(1<<5)

#define _REG_GEN6_GDRST	0x941c
#define    _REGBIT_GEN6_GRDOM_FULL		(1 << 0)
#define    _REGBIT_GEN6_GRDOM_RENDER		(1 << 1)
#define    _REGBIT_GEN6_GRDOM_MEDIA		(1 << 2)
#define    _REGBIT_GEN6_GRDOM_BLT		(1 << 3)

#define _REG_GT_THREAD_STATUS	0x13805C
#define _REG_GT_CORE_STATUS	0x138060

#define _REG_RC_CONTROL				0xA090
#define _REGBIT_RC_HW_CTRL_ENABLE	(1<<31)
#define _REGBIT_RC_RC1_ENABLE		(1<<20)
#define _REGBIT_RC_RC6_ENABLE		(1<<18)
#define _REGBIT_RC_DEEP_RC6_ENABLE	(1<<17)
#define _REGBIT_RC_DEEPEST_RC6_ENABLE	(1<<16)

#define _REG_RPNSWREQ				0xA008
#define _REG_RC_VIDEO_FREQ			0xA00C
#define _REG_RP_DOWN_TIMEOUT			0xA010
#define _REG_RP_INTERRUPT_LIMITS		0xA014
#define _REG_RPSTAT1				0xA01C
#define _REG_RP_CONTROL				0xA024
#define _REG_RP_UP_THRESHOLD			0xA02C
#define _REG_RP_DOWN_THRESHOLD			0xA030
#define _REG_RP_CUR_UP_EI			0xA050
#define _REG_RP_CUR_UP				0xA054
#define _REG_RP_PREV_UP				0xA058
#define _REG_RP_CUR_DOWN_EI			0xA05C
#define _REG_RP_CUR_DOWN			0xA060
#define _REG_RP_PREV_DOWN			0xA064
#define _REG_RP_UP_EI				0xA068
#define _REG_RP_DOWN_EI				0xA06C
#define _REG_RP_IDLE_HYSTERSIS			0xA070
#define _REG_RC_STATE				0xA094
#define _REG_RC1_WAKE_RATE_LIMIT		0xA098
#define _REG_RC6_WAKE_RATE_LIMIT		0xA09C
#define _REG_RC6pp_WAKE_RATE_LIMIT		0xA0A0
#define _REG_RC_EVALUATION_INTERVAL		0xA0A8
#define _REG_RC_IDLE_HYSTERSIS			0xA0AC
#define _REG_RC_SLEEP				0xA0B0
#define _REG_RC1e_THRESHOLD			0xA0B4
#define _REG_RC6_THRESHOLD			0xA0B8
#define _REG_RC6p_THRESHOLD			0xA0BC
#define _REG_RC6pp_THRESHOLD			0xA0C0
#define _REG_PMINTRMSK				0xA168

#define MI_NOOP				0
#define MI_FLUSH			(0x4 << 23)
#define MI_SUSPEND_FLUSH		(0xb << 23)
#define    MI_SUSPEND_FLUSH_EN		(1<<0)
#define MI_SET_CONTEXT			(0x18 << 23)
#define    MI_MM_SPACE_GTT		(1<<8)
#define    MI_MM_SPACE_PHYSICAL		(0<<8)	/* deprecated */
#define    MI_SAVE_EXT_STATE_EN		(1<<3)
#define    MI_RESTORE_EXT_STATE_EN	(1<<2)
#define    MI_FORCE_RESTORE		(1<<1)
#define    MI_RESTORE_INHIBIT		(1<<0)
#define MI_ARB_ON_OFF			(0x08 << 23)
#define    MI_ARB_ENABLE		(1<<0)
#define	   MI_ARB_DISABLE		(0<<0)
/*
 * We use _IMM instead of _INDEX, to avoid switching hardware
 * status page
 */
#define MI_STORE_DATA_IMM		((0x20<<23) | 2)
#define MI_STORE_DATA_IMM_QWORD		((0x20<<23) | 3)
#define   MI_SDI_USE_GTT		(1<<22)
#define MI_LOAD_REGISTER_IMM		(0x22<<23 | 1)
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

#define PIPE_CONTROL(len)		((0x3<<29)|(0x3<<27)|(0x2<<24)|(len-2))
#define   PIPE_CONTROL_POST_SYNC_GLOBAL_GTT		(1<<24)
#define   PIPE_CONTROL_POST_SYNC			(1<<23)
#define   PIPE_CONTROL_CS_STALL				(1<<20)
#define   PIPE_CONTROL_TLB_INVALIDATE			(1<<18)
#define   PIPE_CONTROL_MEDIA_STATE_CLEAR		(1<<16)
#define   PIPE_CONTROL_POST_SYNC_IMM			(1<<14)
#define   PIPE_CONTROL_DEPTH_STALL			(1<<13)
#define   PIPE_CONTROL_RENDER_TARGET_CACHE_FLUSH	(1<<12)
#define   PIPE_CONTROL_INSTRUCTION_CACHE_INVALIDATE	(1<<11) /* MBZ on Ironlake */
#define   PIPE_CONTROL_TEXTURE_CACHE_INVALIDATE		(1<<10) /* GM45+ only */
#define   PIPE_CONTROL_INDIRECT_STATE_DISABLE		(1<<9)
#define   PIPE_CONTROL_NOTIFY				(1<<8)
#define   PIPE_CONTROL_FLUSH_ENABLE			(1<<7)
#define   PIPE_CONTROL_DC_FLUSH_ENABLE			(1<<5)
#define   PIPE_CONTROL_VF_CACHE_INVALIDATE		(1<<4)
#define   PIPE_CONTROL_CONST_CACHE_INVALIDATE		(1<<3)
#define   PIPE_CONTROL_STATE_CACHE_INVALIDATE		(1<<2)
#define   PIPE_CONTROL_STALL_AT_SCOREBOARD		(1<<1)
#define   PIPE_CONTROL_DEPTH_CACHE_FLUSH		(1<<0)

#define DUMMY_3D		(0x6d800005)
#define PRIM_TRILIST		(0x4)
/* PCI config space */
#define _REG_LBB	0xf4

/* VGA stuff */
#define _REG_VGA_MSR_WRITE	0x3c2
#define _REG_VGA_MSR_READ	0x3cc
#define    VGA_MSR_CGA_MODE	(1<<0)

#define _REG_VGA_CR_INDEX_MDA	0x3b4
#define _REG_VGA_CR_DATA_MDA	0x3b5
#define _REG_VGA_ST01_MDA	0x3ba

#define _REG_VGA_CR_INDEX_CGA	0x3d4
#define _REG_VGA_CR_DATA_CGA	0x3d5
#define _REG_VGA_ST01_CGA	0x3da

#define _REG_VGA_SR_INDEX	0x3c4
#define _REG_VGA_SR_DATA	0x3c5

#define _REG_VGA_GR_INDEX	0x3ce
#define _REG_VGA_GR_DATA	0x3cf

#define _REG_VGA_AR_INDEX	0x3c0
#define _REG_VGA_AR_DATA_WRITE	0x3c0
#define _REG_VGA_AR_DATA_READ	0x3c1

#define _REG_VGA_DACMASK	0x3c6
/*
 * Display engine regs
 */

#define _ACTIVE_WIDTH_MASK (0xFFF)

/* Pipe A timing regs */
#define _REG_HTOTAL_A		0x60000
#define _REG_HBLANK_A		0x60004
#define _REG_HSYNC_A		0x60008
#define _REG_VTOTAL_A		0x6000c
#define _REG_VBLANK_A		0x60010
#define _REG_VSYNC_A		0x60014
#define _REG_PIPEASRC		0x6001c
#define     _PIPE_V_SRCSZ_SHIFT	0
#define     _PIPE_V_SRCSZ_MASK	(0xfff << _PIPE_V_SRCSZ_SHIFT)
#define     _PIPE_H_SRCSZ_SHIFT	16
#define     _PIPE_H_SRCSZ_MASK	(0x1fff << _PIPE_H_SRCSZ_SHIFT)
#define _REG_BCLRPAT_A		0x60020
#define _REG_VSYNCSHIFT_A	0x60028

/* Pipe B timing regs */
#define _REG_HTOTAL_B		0x61000
#define _REG_HBLANK_B		0x61004
#define _REG_HSYNC_B		0x61008
#define _REG_VTOTAL_B		0x6100c
#define _REG_VBLANK_B		0x61010
#define _REG_VSYNC_B		0x61014
#define _REG_PIPEBSRC		0x6101c
#define _REG_BCLRPAT_B		0x61020
#define _REG_VSYNCSHIFT_B	0x61028

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


#define VGT_HTOTAL(pipe)	_VGT_PIPE(pipe, _REG_HTOTAL_A, _REG_HTOTAL_B)
#define VGT_HBLANK(pipe)	_VGT_PIPE(pipe, _REG_HBLANK_A, _REG_HBLANK_B)
#define VGT_HSYNC(pipe)		_VGT_PIPE(pipe, _REG_HSYNC_A, _REG_HSYNC_B)
#define VGT_VTOTAL(pipe)	_VGT_PIPE(pipe, _REG_VTOTAL_A, _REG_VTOTAL_B)
#define VGT_VBLANK(pipe)	_VGT_PIPE(pipe, _REG_VBLANK_A, _REG_VBLANK_B)
#define VGT_VSYNC(pipe)		_VGT_PIPE(pipe, _REG_VSYNC_A, _REG_VSYNC_B)

#define VGT_BCLRPAT(pipe)	_VGT_PIPE(pipe, _REG_BCLRPAT_A, _REG_BCLRPAT_B)
#define VGT_VSYNCSHIFT(pipe)	_VGT_PIPE(pipe, _REG_VSYNCSHIFT_A, _REG_VSYNCSHIFT_B)
#define VGT_PIPESRC(pipe)	_VGT_PIPE(pipe, _REG_PIPEASRC, _REG_PIPEBSRC)

#define _REG_DISP_ARB_CTL	0x45000
#define _REG_DISP_ARB_CTL2	0x45004
#define _REG_TILECTL		0x101000

/* PCH */
#define _REG_PCH_DREF_CONTROL			0xc6200
#define    _REGBIT_DREF_CONTROL_MASK			0x7fc3
#define    _REGBIT_DREF_CPU_SOURCE_OUTPUT_DISABLE	(0<<13)
#define    _REGBIT_DREF_CPU_SOURCE_OUTPUT_DOWNSPREAD	(2<<13)
#define    _REGBIT_DREF_CPU_SOURCE_OUTPUT_NONSPREAD	(3<<13)
#define    _REGBIT_DREF_CPU_SOURCE_OUTPUT_MASK		(3<<13)
#define    _REGBIT_DREF_SSC_SOURCE_DISABLE		(0<<11)
#define    _REGBIT_DREF_SSC_SOURCE_ENABLE		(2<<11)
#define    _REGBIT_DREF_SSC_SOURCE_MASK			(3<<11)
#define    _REGBIT_DREF_NONSPREAD_SOURCE_DISABLE	(0<<9)
#define    _REGBIT_DREF_NONSPREAD_CK505_ENABLE		(1<<9)
#define    _REGBIT_DREF_NONSPREAD_SOURCE_ENABLE		(2<<9)
#define    _REGBIT_DREF_NONSPREAD_SOURCE_MASK		(3<<9)
#define    _REGBIT_DREF_SUPERSPREAD_SOURCE_DISABLE	(0<<7)
#define    _REGBIT_DREF_SUPERSPREAD_SOURCE_ENABLE	(2<<7)
#define    _REGBIT_DREF_SUPERSPREAD_SOURCE_MASK		(3<<7)
#define    _REGBIT_DREF_SSC4_DOWNSPREAD			(0<<6)
#define    _REGBIT_DREF_SSC4_CENTERSPREAD		(1<<6)
#define    _REGBIT_DREF_SSC1_DISABLE			(0<<1)
#define    _REGBIT_DREF_SSC1_ENABLE			(1<<1)
#define    _REGBIT_DREF_SSC4_DISABLE			(0)
#define    _REGBIT_DREF_SSC4_ENABLE			(1)

#define _REG_PCH_RAWCLK_FREQ		0xc6204
#define  _REGBIT_RAWCLK_FREQ_MASK       0x3ff

/*
 * digital port hotplug
 */
#define _REG_PCH_DPLL_A			0xc6014
#define _REG_PCH_DPLL_B			0xc6018

#define _REGBIT_DPLL_VCO_ENABLE		(1 << 31)
#define VGT_PCH_DPLL(pipe)	_VGT_PIPE(pipe, _REG_PCH_DPLL_A, _REG_PCH_DPLL_B)

#define _REG_PCH_FPA0				0xc6040
#define    FP_CB_TUNE				(0x3<<22)
#define _REG_PCH_FPA1				0xc6044
#define _REG_PCH_FPB0				0xc6048
#define _REG_PCH_FPB1				0xc604c
#define VGT_PCH_FP0(pipe)	_VGT_PIPE(pipe, _REG_PCH_FPA0, _REG_PCH_FPB0)
#define VGT_PCH_FP1(pipe)	_VGT_PIPE(pipe, _REG_PCH_FPA1, _REG_PCH_FPB1)

#define _REG_PCH_DPLL_SEL			0xc7000
#define _REGBIT_TRANSA_DPLL_ENABLE		(1 << 3)
#define    _REGBIT_TRANSA_DPLLB_SEL		(1 << 0)
#define    _REGBIT_TRANSA_DPLLA_SEL		0
#define _REGBIT_TRANSB_DPLL_ENABLE		(1 << 7)
#define    _REGBIT_TRANSB_DPLLB_SEL		(1 << 4)
#define    _REGBIT_TRANSB_DPLLA_SEL		0
#define _REGBIT_TRANSC_DPLL_ENABLE		(1 << 11)
#define    _REGBIT_TRANSC_DPLLB_SEL		(1 << 8)
#define    _REGBIT_TRANSC_DPLLA_SEL		0

/*
 * Clock control & power management
 */
#define _REG_VGA0	0x6000
#define _REG_VGA1	0x6004
#define _REG_VGA_PD	0x6010

/* refresh rate hardware control */
#define _REG_PIPEA_DATA_M1		0x60030
#define _REG_PIPEA_DATA_N1		0x60034
#define _REG_PIPEA_LINK_M1		0x60040
#define _REG_PIPEA_LINK_N1		0x60044

#define _REG_PIPEA_DATA_M2		0x60038
#define _REG_PIPEA_DATA_N2		0x6003c
#define _REG_PIPEA_LINK_M2		0x60048
#define _REG_PIPEA_LINK_N2		0x6004c

/* PIPE B timing regs are same start from 0x61000 */
#define _REG_PIPEB_DATA_M1		0x61030
#define _REG_PIPEB_DATA_N1		0x61034
#define _REG_PIPEB_LINK_M1		0x61040
#define _REG_PIPEB_LINK_N1		0x61044

#define _REG_PIPEB_DATA_M2		0x61038
#define _REG_PIPEB_DATA_N2		0x6103c
#define _REG_PIPEB_LINK_M2		0x61048
#define _REG_PIPEB_LINK_N2		0x6104c

/* PIPE C timing regs are same start from 0x61000 */
#define _REG_PIPEC_DATA_M1		0x62030
#define _REG_PIPEC_DATA_N1		0x62034
#define _REG_PIPEC_LINK_M1		0x62040
#define _REG_PIPEC_LINK_N1		0x62044

#define _REG_PIPEC_DATA_M2		0x62038
#define _REG_PIPEC_DATA_N2		0x6203c
#define _REG_PIPEC_LINK_M2		0x62048
#define _REG_PIPEC_LINK_N2		0x6204c

#define VGT_PIPE_DATA_M1(pipe) _VGT_PIPE(pipe, _REG_PIPEA_DATA_M1, _REG_PIPEB_DATA_M1)
#define VGT_PIPE_DATA_N1(pipe) _VGT_PIPE(pipe, _REG_PIPEA_DATA_N1, _REG_PIPEB_DATA_N1)
#define VGT_PIPE_DATA_M2(pipe) _VGT_PIPE(pipe, _REG_PIPEA_DATA_M2, _REG_PIPEB_DATA_M2)
#define VGT_PIPE_DATA_N2(pipe) _VGT_PIPE(pipe, _REG_PIPEA_DATA_N2, _REG_PIPEB_DATA_N2)
#define VGT_PIPE_LINK_M1(pipe) _VGT_PIPE(pipe, _REG_PIPEA_LINK_M1, _REG_PIPEB_LINK_M1)
#define VGT_PIPE_LINK_N1(pipe) _VGT_PIPE(pipe, _REG_PIPEA_LINK_N1, _REG_PIPEB_LINK_N1)
#define VGT_PIPE_LINK_M2(pipe) _VGT_PIPE(pipe, _REG_PIPEA_LINK_M2, _REG_PIPEB_LINK_M2)
#define VGT_PIPE_LINK_N2(pipe) _VGT_PIPE(pipe, _REG_PIPEA_LINK_N2, _REG_PIPEB_LINK_N2)

/* VGA port control */
#define _REG_ADPA			0x61100

/* FDI_RX, FDI_X is hard-wired to Transcoder_X */
#define _REG_FDI_RXA_CTL			0xf000c
#define _REG_FDI_RXB_CTL			0xf100c
#define _REG_FDI_RXC_CTL			0xf200c

#define _REGBIT_FDI_RX_ENABLE			(1 << 31)
#define _REGBIT_FDI_RX_PLL_ENABLE		(1 << 13)
#define _REGBIT_FDI_RX_PORT_WIDTH_MASK		(0x7 << 19)
#define _REGBIT_FDI_RX_FDI_AUTO_TRAIN_ENABLE	(0x1 << 10)
#define _REGBIT_FDI_LINK_TRAIN_PATTERN_1_CPT	(0 << 8)
#define _REGBIT_FDI_LINK_TRAIN_PATTERN_2_CPT	(1 << 8)
#define _REGBIT_FDI_LINK_TRAIN_NORMAL_CPT	(3 << 8)
#define _REGBIT_FDI_LINK_TRAIN_PATTERN_MASK_CPT	(3 << 8)
#define _REGBIT_FDI_RX_ENHANCE_FRAME_ENABLE	(1 << 6)
#define _REGBIT_FDI_PCDCLK			(1 << 4)

#define _REG_FDI_RXA_IIR			0xf0014
#define _REG_FDI_RXB_IIR			0xf1014
#define _REG_FDI_RXC_IIR			0xf2014
#define _REG_FDI_RXA_IMR			0xf0018
#define _REG_FDI_RXB_IMR			0xf1018
#define _REG_FDI_RXC_IMR			0xf2018
#define VGT_FDI_RX_IIR(pipe) _VGT_PIPE(pipe, _REG_FDI_RXA_IIR, _REG_FDI_RXB_IIR)
#define VGT_FDI_RX_IMR(pipe) _VGT_PIPE(pipe, _REG_FDI_RXA_IMR, _REG_FDI_RXB_IMR)

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
#define VGT_FDI_RX_CTL(pipe) _VGT_PIPE(pipe, _REG_FDI_RXA_CTL, _REG_FDI_RXB_CTL)

#define _REG_FDI_RXA_MISC			0xf0010
#define _REG_FDI_RXB_MISC			0xf1010
#define _REG_FDI_RXA_TUSIZE1		0xf0030
#define _REG_FDI_RXA_TUSIZE2		0xf0038
#define _REG_FDI_RXB_TUSIZE1		0xf1030
#define _REG_FDI_RXB_TUSIZE2		0xf1038

#define VGT_FDI_RX_TUSIZE1(pipe) _VGT_PIPE(pipe, _REG_FDI_RXA_TUSIZE1,_REG_FDI_RXB_TUSIZE1)

/* CPU: FDI_TX */
#define _REG_FDI_TXA_CTL		0x60100
#define _REG_FDI_TXB_CTL		0x61100
#define _REG_FDI_TXC_CTL		0x62100

#define _REGBIT_FDI_TX_ENABLE				(1 << 31)
#define _REGBIT_FDI_LINK_TRAIN_PATTERN_1		(0 << 28)
#define _REGBIT_FDI_LINK_TRAIN_PATTERN_2		(1 << 28)
#define _REGBIT_FDI_LINK_TRAIN_NONE			(3 << 28)
#define _REGBIT_FDI_TX_PLL_ENABLE			(1 << 14)
#define _REGBIT_FDI_LINK_TRAIN_400MV_0DB_SNB_B		(0x0<<22)
#define _REGBIT_FDI_LINK_TRAIN_400MV_6DB_SNB_B		(0x3a<<22)
#define _REGBIT_FDI_LINK_TRAIN_600MV_3_5DB_SNB_B	(0x39<<22)
#define _REGBIT_FDI_LINK_TRAIN_800MV_0DB_SNB_B		(0x38<<22)
#define _REGBIT_FDI_LINK_TRAIN_VOL_EMP_MASK		(0x3f<<22)
#define _REGBIT_FDI_TX_ENHANCE_FRAME_ENABLE		(1<<18)

#define VGT_FDI_TX_CTL(pipe) _VGT_PIPE(pipe, _REG_FDI_TXA_CTL, _REG_FDI_TXB_CTL)

/* CRT */
#define _REG_PCH_ADPA				0xe1100
#define _REGBIT_ADPA_DAC_ENABLE			(1 << 31)
#define PORT_TRANS_SEL_SHIFT			29
#define PORT_TRANS_SEL_MASK			(3 << PORT_TRANS_SEL_SHIFT)
#define VGT_PORT_TRANS_SEL_CPT(pipe)		((pipe) << PORT_TRANS_SEL_SHIFT)
#define _REGBIT_ADPA_CRT_HOTPLUG_MONITOR_MASK	(3 << 24)
#define _REGBIT_ADPA_CRT_HOTPLUG_ENABLE		(1 << 23)
#define _REGBIT_ADPA_CRT_HOTPLUG_PERIOD_128	(1 << 22)
#define _REGBIT_ADPA_CRT_HOTPLUG_WARMUP_10MS	(1 << 21)
#define _REGBIT_ADPA_CRT_HOTPLUG_SAMPLE_4S	(1 << 20)
#define _REGBIT_ADPA_CRT_HOTPLUG_VOLTAGE_50	(1 << 18)
#define _REGBIT_ADPA_CRT_HOTPLUG_VOLREF_325MV	(0 << 17)
#define _REGBIT_ADPA_CRT_HOTPLUG_FORCE_TRIGGER	(1 << 16)
#define _REGBIT_ADPA_VSYNC_ACTIVE_HIGH		(1 << 4)
#define _REGBIT_ADPA_HSYNC_ACTIVE_HIGH		(1 << 3)

/* Display port */
#define _REG_DP_B_CTL	0xe4100
#define _REG_DP_C_CTL	0xe4200
#define _REG_DP_D_CTL	0xe4300
#define _REGBIT_DP_PORT_ENABLE		(1 << 31)

#define  _REGBIT_DP_VOLTAGE_0_4		(0 << 25)
#define  _REGBIT_DP_VOLTAGE_0_6		(1 << 25)
#define  _REGBIT_DP_VOLTAGE_0_8		(2 << 25)
#define  _REGBIT_DP_VOLTAGE_1_2		(3 << 25)
#define  _REGBIT_DP_VOLTAGE_MASK	(7 << 25)
#define  DP_VOLTAGE_SHIFT		25

#define _REGBIT_DP_PRE_EMPHASIS_0		(0 << 22)
#define _REGBIT_DP_PRE_EMPHASIS_3_5		(1 << 22)
#define _REGBIT_DP_PRE_EMPHASIS_6		(2 << 22)
#define _REGBIT_DP_PRE_EMPHASIS_9_5		(3 << 22)
#define _REGBIT_DP_PRE_EMPHASIS_MASK		(7 << 22)

#define _REGBIT_DP_LINK_TRAIN_PAT_1_CPT		(0 << 8)
#define _REGBIT_DP_LINK_TRAIN_PAT_2_CPT		(1 << 8)
#define _REGBIT_DP_LINK_TRAIN_PAT_IDLE_CPT	(2 << 8)
#define _REGBIT_DP_LINK_TRAIN_OFF_CPT		(3 << 8)
#define _REGBIT_DP_LINK_TRAIN_MASK_CPT		(7 << 8)
#define _REGBIT_DP_AUDIO_OUTPUT_ENABLE		(1 << 6)
#define _REGBIT_DP_PORT_DETECTED		(1 << 2)

/* legacy or PCH_IBX ? */
#define _REGBIT_DP_LINK_TRAIN_MASK		(3 << 28)


#define _REG_TRANS_DP_A_CTL	0xe0300
#define _REG_TRANS_DP_B_CTL 0xe1300
#define _REG_TRANS_DP_C_CTL 0xe2300
#define _REGBIT_TRANS_DP_PORT_SEL_MASK	(3 << 29)
#define _REGBIT_TRANS_DP_PORT_SEL_NONE	(3 << 29)
#define _REGBIT_TRANS_DP_OUTPUT_ENABLE	(1 << 31)
#define VGT_TRANS_DP_CTL(pipe)	(_REG_TRANS_DP_A_CTL + (pipe) * 0x01000)
#define _REGBIT_TRANS_DP_PORT_SEL_B	(0 << 29)
#define _REGBIT_TRANS_DP_PORT_SEL_C	(1 << 29)
#define _REGBIT_TRANS_DP_PORT_SEL_D	(2 << 29)


/* Digital display A (DP_A, embedded) */
#define _REGBIT_DP_PORT_A_DETECTED	(1 << 2)

/* HDMI/DVI/SDVO port */
#define _REG_HDMI_B_CTL	0xe1140
#define _REG_HDMI_C_CTL	0xe1150
#define _REG_HDMI_D_CTL	0xe1160
#define HDMI_TRANS_SEL_MASK		(3 << 29)
#define _REGBIT_HDMI_PORT_ENABLE	(1 << 31)
#define _REGBIT_HDMI_PORT_DETECTED	(1 << 2)

/* PCH SDVOB multiplex with HDMIB */
#define _REG_PCH_LVDS	0xe1180

#define _REG_BLC_PWM_CPU_CTL2	0x48250

#define _REG_BLC_PWM_CPU_CTL	0x48254
#define VGT_BACKLIGHT_DUTY_CYCLE_MASK		(0xffff)

#define _REG_BLC_PWM_PCH_CTL1	0xc8250
#define _REG_BLC_PWM_PCH_CTL2	0xc8254
#define _REG_PCH_PP_ON_DELAYS	0xc7208
#define _REG_PCH_PP_OFF_DELAYS	0xc720c
#define _REGBIT_PANEL_POWER_DOWN_DELAY_MASK	(0x1fff0000)
#define _REGBIT_PANEL_POWER_DOWN_DELAY_SHIFT	16
#define _REGBIT_PANEL_LIGHT_OFF_DELAY_MASK	(0x1fff)
#define _REGBIT_PANEL_LIGHT_OFF_DELAY_SHIFT	0

#define _REG_PCH_PP_DIVISOR		0xc7210

#define _REG_PCH_PP_STATUS		0xc7200
#define _REGBIT_PANEL_POWER_ON		(1 << 31)
#define _REG_PCH_PP_CONTROL		0xc7204
#define _REGBIT_POWER_TARGET_ON		(1 << 0)
#define _REGBIT_PANEL_UNLOCK_REGS	(0xabcd << 16) /* Write Protect Key is 0xABCD */


/* Watermark register (Ironlake) */
#define _REG_WM0_PIPEA_ILK	0x45100
#define _REG_WM0_PIPEB_ILK	0x45104
#define _REG_WM0_PIPEC_IVB	0x45200
#define _REG_WM1_LP_ILK		0x45108
#define _REG_WM2_LP_ILK		0x4510c
#define _REG_WM3_LP_ILK		0x45110
#define _REG_WM1S_LP_ILK	0x45120
#define _REG_WM2S_LP_IVB	0x45124
#define _REG_WM3S_LP_IVB	0x45128

#define  _REGBIT_WM0_PIPE_PLANE_MASK	(0x7f<<16)
#define  _REGBIT_WM0_PIPE_PLANE_SHIFT	16
#define  _REGBIT_WM0_PIPE_SPRITE_MASK	(0x3f<<8)
#define  _REGBIT_WM0_PIPE_SPRITE_SHIFT	8
#define  _REGBIT_WM0_PIPE_CURSOR_MASK	(0x1f)

#define DISPLAY_MAXWM	0x7f	/* bit 16:22 */
#define CURSOR_MAXWM	0x1f	/* bit 4:0 */

/*Intermediate Pixel Storage*/
#define _REG_IPS_CTL		0x43408

union PCH_PP_CONTROL
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

union PCH_PP_STAUTS
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
#define _REG_RSTDBYCTL		0x111b8

/* CPU panel fitter */
#define _REG_PF_CTL_0			0x68080
#define _REG_PF_CTL_1			0x68880
#define _REG_PF_CTL_2			0x69080
#define _REGBIT_PF_ENABLE		(1 << 31)
#define  _REGBIT_PF_PIPE_SEL_MASK	(3<<29)
#define  _REGBIT_PF_PIPE_SEL(pipe)	((pipe)<<29)
#define _REGBIT_PF_FILTER_MASK		(3 << 23)
#define _REGBIT_PF_FILTER_PROGRAMMED	(0 << 23)
#define _REGBIT_PF_FILTER_MED_3x3	(1 << 23)
#define _REGBIT_PF_FILTER_EDGE_ENHANCE	(2 << 23)
#define _REGBIT_PF_FILTER_EDGE_SOFTEN	(3 << 23)

#define _REG_PF_WIN_SZ_0		0x68074
#define _REG_PF_WIN_SZ_1		0x68874
#define _REG_PF_WIN_SZ_2		0x69074

#define _REG_PF_WIN_POS_0		0x68070
#define _REG_PF_WIN_POS_1		0x68870
#define _REG_PF_WIN_POS_2		0x69070

#define VGT_PF_CTL(pipe)	_VGT_PIPE(pipe, _REG_PF_CTL_0, _REG_PF_CTL_1)
#define VGT_PF_WIN_SZ(pipe)	_VGT_PIPE(pipe, _REG_PF_WIN_SZ_0, _REG_PF_WIN_SZ_1)
#define    VGT_PF_WIN_POS(pipe) _VGT_PIPE(pipe, _REG_PF_WIN_POS_0, _REG_PF_WIN_POS_1)

/* Per-transcoder DIP controls */
#define _REG_TRANSACONF			0xf0008
#define _REG_TRANSBCONF			0xf1008
#define _REGBIT_TRANS_ENABLE		(1 << 31)
#define _REGBIT_TRANS_STATE_ENABLED	(1 << 30)
#define _REGBIT_TRANS_INTERLACE_MASK	(7 << 21)
#define VGT_TRANSCONF(plane)	_VGT_PIPE(plane, _REG_TRANSACONF, _REG_TRANSBCONF)

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

#define _REG_TRANSA_CHICKEN1	0xf0060
#define _REG_TRANSA_CHICKEN2	0xf0064
#define _REG_TRANSB_CHICKEN1	0xf1060
#define _REG_TRANSB_CHICKEN2	0xf1064
#define VGT_TRANS_CHICKEN2(pipe) _VGT_PIPE(pipe, _REG_TRANSA_CHICKEN2, _REG_TRANSB_CHICKEN2)
#define _REGBIT_TRANS_AUTOTRAIN_GEN_STALL_DISABLE	(1<<31)

/* transcoder */
#define _REG_TRANS_HTOTAL_A		0xe0000
#define _REG_TRANS_HBLANK_A		0xe0004
#define _REG_TRANS_HSYNC_A		0xe0008
#define _REG_TRANS_VTOTAL_A		0xe000c
#define _REG_TRANS_VBLANK_A		0xe0010
#define _REG_TRANS_VSYNC_A		0xe0014
#define _REG_TRANS_VSYNCSHIFT_A		0xe0028
#define _REG_TRANS_HTOTAL_B		0xe1000
#define _REG_TRANS_HBLANK_B		0xe1004
#define _REG_TRANS_HSYNC_B		0xe1008
#define _REG_TRANS_VTOTAL_B		0xe100c
#define _REG_TRANS_VBLANK_B		0xe1010
#define _REG_TRANS_VSYNC_B		0xe1014
#define _REG_TRANS_VSYNCSHIFT_B		0xe1028

#define VGT_TRANS_HTOTAL(pipe)	_VGT_PIPE(pipe, _REG_TRANS_HTOTAL_A, _REG_TRANS_HTOTAL_B)
#define VGT_TRANS_HBLANK(pipe)	_VGT_PIPE(pipe, _REG_TRANS_HBLANK_A, _REG_TRANS_HBLANK_B)
#define VGT_TRANS_HSYNC(pipe)	 _VGT_PIPE(pipe, _REG_TRANS_HSYNC_A, _REG_TRANS_HSYNC_B)
#define VGT_TRANS_VTOTAL(pipe)	_VGT_PIPE(pipe, _REG_TRANS_VTOTAL_A, _REG_TRANS_VTOTAL_B)
#define VGT_TRANS_VBLANK(pipe)	_VGT_PIPE(pipe, _REG_TRANS_VBLANK_A, _REG_TRANS_VBLANK_B)
#define VGT_TRANS_VSYNC(pipe)	 _VGT_PIPE(pipe, _REG_TRANS_VSYNC_A, _REG_TRANS_VSYNC_B)
#define VGT_TRANS_VSYNCSHIFT(pipe)	_VGT_PIPE(pipe, _REG_TRANS_VSYNCSHIFT_A, \
					_REG_TRANS_VSYNCSHIFT_B)



#define _REG_SOUTH_CHICKEN1			0xc2000
#define    VGT_FDIA_PHASE_SYNC_SHIFT_EN	18
#define VGT_FDIA_PHASE_SYNC_SHIFT_OVR	19
#define    VGT_FDI_PHASE_SYNC_EN(pipe)	(1 << (VGT_FDIA_PHASE_SYNC_SHIFT_EN - ((pipe) * 2)))
#define VGT_FDI_PHASE_SYNC_OVR(pipe)(1 << (VGT_FDIA_PHASE_SYNC_SHIFT_OVR - ((pipe) *2)))
#define _REG_SOUTH_CHICKEN2			0xc2004
#define    _REGBIT_FDI_MPHY_IOSFSB_RESET_STATUS	(1<<13)
#define    _REGBIT_MPHY_IOSFSB_RESET_CTL	(1<<12)
#define _REG_SOUTH_DSPCLK_GATE_D		0xc2020

#define _REG_TRANSA_DATA_M1		0xe0030
#define _REG_TRANSA_DATA_N1		0xe0034
#define _REG_TRANSA_DATA_M2		0xe0038
#define _REG_TRANSA_DATA_N2		0xe003c
#define _REG_TRANSA_DP_LINK_M1		0xe0040
#define _REG_TRANSA_DP_LINK_N1		0xe0044
#define _REG_TRANSA_DP_LINK_M2		0xe0048
#define _REG_TRANSA_DP_LINK_N2		0xe004c

#define _REG_TRANSB_DATA_M1		0xe1030
#define _REG_TRANSB_DATA_N1		0xe1034
#define _REG_TRANSB_DATA_M2		0xe1038
#define _REG_TRANSB_DATA_N2		0xe103c
#define _REG_TRANSB_DP_LINK_M1		0xe1040
#define _REG_TRANSB_DP_LINK_N1		0xe1044
#define _REG_TRANSB_DP_LINK_M2		0xe1048
#define _REG_TRANSB_DP_LINK_N2		0xe104c

#define VGT_TRANSDATA_M1(pipe)	_VGT_PIPE(pipe, _REG_TRANSA_DATA_M1, _REG_TRANSB_DATA_M1)
#define VGT_TRANSDATA_N1(pipe)	_VGT_PIPE(pipe, _REG_TRANSA_DATA_N1, _REG_TRANSB_DATA_N1)
#define VGT_TRANSDATA_M2(pipe)	_VGT_PIPE(pipe, _REG_TRANSA_DATA_M2, _REG_TRANSB_DATA_M2)
#define VGT_TRANSDATA_N2(pipe)	_VGT_PIPE(pipe, _REG_TRANSA_DATA_N2, _REG_TRANSB_DATA_N2)

#define _REG_TRANSA_VIDEO_DIP_CTL	0xE0200
#define _REG_TRANSA_VIDEO_DIP_DATA	0xE0208
#define _REG_TRANSA_VIDEO_DIP_GCP	0xE0210
#define _REG_TRANSA_DP_CTL		0xE0300
#define _REG_TRANSB_VIDEO_DIP_CTL	0xE1200
#define _REG_TRANSB_VIDEO_DIP_DATA	0xE1208
#define _REG_TRANSB_VIDEO_DIP_GCP	0xE1210
#define _REG_TRANSB_DP_CTL		0xE1300
#define _REG_TRANSC_VIDEO_DIP_CTL	0xE2200
#define _REG_TRANSC_VIDEO_DIP_DATA	0xE2208
#define _REG_TRANSC_VIDEO_DIP_GCP	0xE2210
#define _REG_TRANSC_DP_CTL		0xE2300

/* Display & cursor control */

/* Pipe A */
#define _REG_PIPEADSL		0x70000
#define _REG_PIPEACONF		0x70008
#define _REG_PIPEASTAT		0x70024
#define _REG_PIPEA_FRMCOUNT 	0x70040
#define _REG_PIPEA_FLIPCOUNT	0x70044
#define _REG_PIPE_MISC_A	0x70030

/* Pipe B */
#define _REG_PIPEBDSL		0x71000
#define _REG_PIPEBCONF		0x71008
#define _REG_PIPEBSTAT		0x71024
#define _REG_PIPEB_FRMCOUNT 	0x71040
#define _REG_PIPEB_FLIPCOUNT	0x71044
#define _REG_PIPE_MISC_B	0x71030

/* Pipe C */
#define _REG_PIPECDSL		0x72000
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
#define _REGBIT_PIPE_VBLANK_INTERRUPT_STATUS	(1 << 1)

#define VGT_PIPEDSL(pipe)	_VGT_PIPE(pipe, _REG_PIPEADSL, _REG_PIPEBDSL)
#define VGT_PIPECONF(pipe)	_VGT_PIPE(pipe, _REG_PIPEACONF, _REG_PIPEBCONF)
#define VGT_PIPESTAT(pipe)	_VGT_PIPE(pipe, _REG_PIPEASTAT, _REG_PIPEBSTAT)
#define VGT_PIPE_FRMCOUNT(pipe)	_VGT_PIPE(pipe, _REG_PIPEA_FRMCOUNT, _REG_PIPEB_FRMCOUNT)
#define VGT_PIPE_FLIPCOUNT(pipe) _VGT_PIPE(pipe, _REG_PIPEA_FLIPCOUNT, _REG_PIPEB_FLIPCOUNT)

#define VGT_PIPECONFPIPE(pipeconf) _VGT_GET_PIPE(pipeconf, _REG_PIPEACONF, _REG_PIPEBCONF)
#define VGT_FRMCOUNTPIPE(frmcount) _VGT_GET_PIPE(frmcount, _REG_PIPEA_FRMCOUNT, _REG_PIPEB_FRMCOUNT)

/* For Gen 2 */
#define _REG_CURSIZE		0x700a0
/*
 * Palette regs
 */
#define _REG_PALETTE_A		0x0a000
#define _REG_PALETTE_B		0x0a800
#define VGT_PALETTE(pipe) _VGT_PIPE(pipe, _REG_PALETTE_A, _REG_PALETTE_B)

/* legacy palette */
#define _REG_LGC_PALETTE_A		0x4a000
#define _REG_LGC_PALETTE_B		0x4a800
#define _REG_LGC_PALETTE_C		0x4b000
#define VGT_LGC_PALETTE(pipe) _VGT_PIPE(pipe, _REG_LGC_PALETTE_A, _REG_LGC_PALETTE_B)

/* Display Port */
#define _REG_DP_TP_CTL_A		0x64040
#define _REG_DP_TP_CTL_B		0x64140
#define _REG_DP_TP_CTL_C		0x64240
#define _REG_DP_TP_CTL_D		0x64340
#define _REG_DP_TP_CTL_E		0x64440
#define  _REGBIT_DP_TP_ENABLE		(1 << 31)
#define  _REGBIT_DP_TP_FDI_AUTO_TRAIN_ENABLE	(1 << 15)
#define _REG_DDI_BUF_CTL_A		0x64000
#define  _DDI_BUFCTL_DETECT_MASK	0x1
#define  _REGBIT_DDI_BUF_ENABLE		(1 << 31)
#define  _REGBIT_DDI_BUF_IS_IDLE	(1<<7)
#define _REG_DDI_BUF_CTL_B		0x64100
#define _REG_DDI_BUF_CTL_C		0x64200
#define _REG_DDI_BUF_CTL_D		0x64300
#define _REG_DDI_BUF_CTL_E		0x64400

#define _REG_DP_TP_STATUS_A			0x64044
#define _REG_DP_TP_STATUS_B			0x64144
#define _REG_DP_TP_STATUS_C			0x64244
#define _REG_DP_TP_STATUS_D			0x64344
#define _REG_DP_TP_STATUS_E			0x64444
#define  _REGBIT_DP_TP_STATUS_AUTOTRAIN_DONE	(1 << 12)

#define VGT_DP_TP_CTL(port)		_VGT_PORT(port, _REG_DP_TP_CTL_A, \
						_REG_DP_TP_CTL_B)
#define VGT_DP_TP_CTL_PORT(reg)		_VGT_GET_PORT(reg, _REG_DP_TP_CTL_A, \
						_REG_DP_TP_CTL_B)
#define VGT_DP_TP_STATUS(port)		_VGT_PORT(port, _REG_DP_TP_STATUS_A, \
						_REG_DP_TP_STATUS_B)
#define VGT_DP_TP_STATUS_PORT(reg)	_VGT_GET_PORT(reg, _REG_DP_TP_STATUS_A, \
						_REG_DP_TP_STATUS_B)
#define VGT_DDI_BUF_CTL(port)		_VGT_PORT(port, _REG_DDI_BUF_CTL_A, \
						_REG_DDI_BUF_CTL_B)

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
#define _REG_CPU_VGACNTRL	0x41000
#define _REGBIT_VGA_DISPLAY_DISABLE	(1UL << 31)

#define _REG_DISPLAY_CHICKEN_BITS_1	0x42000
#define _REG_DISPLAY_CHICKEN_BITS_2	0x42004
#define _REG_DSPCLK_GATE_D		0x42020

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
#define _REG_GEN3_MI_ARB_STATE	0x020e4 /* 915+ only */

#define _REG_SWF		0x4f000

#define _REG_DP_BUFTRANS	0xe4f00

/* digital port hotplug */

#define _REG_PCH_GPIOA		0xc5010
#define _REG_PCH_GPIOB		0xc5014
#define _REG_PCH_GPIOC		0xc5018
#define _REG_PCH_GPIOD		0xc501c
#define _REG_PCH_GPIOE		0xc5020
#define _REG_PCH_GPIOF		0xc5024

#define _REG_PCH_GMBUS0		0xc5100
#define _REG_PCH_GMBUS1		0xc5104
#define _REG_PCH_GMBUS2		0xc5108
#define _REG_PCH_GMBUS3		0xc510c
#define _REG_PCH_GMBUS4		0xc5110
#define _REG_PCH_GMBUS5		0xc5120

/* GMBUS1 bits definitions */
#define _GMBUS_SW_CLR_INT	(1 << 31)
#define _GMBUS_SW_RDY		(1 << 30)
#define _GMBUS_CYCLE_WAIT	(1 << 25)
#define _GMBUS_CYCLE_INDEX	(1 << 26)
#define _GMBUS_CYCLE_STOP	(1 << 27)
#define _GMBUS_SLAVE_READ	(1 << 0)
#define GMBUS1_TOTAL_BYTES_SHIFT 16
#define GMBUS1_TOTAL_BYTES_MASK 0x1ff
#define gmbus1_total_byte_count(v) (((v) >> GMBUS1_TOTAL_BYTES_SHIFT) & GMBUS1_TOTAL_BYTES_MASK)
#define gmbus1_slave_addr(v) (((v) & 0xff) >> 1)
#define gmbus1_slave_index(v) (((v) >> 8) & 0xff)
#define gmbus1_bus_cycle(v) (((v) >> 25) & 0x7)

/* GMBUS0 bits definitions */
#define _GMBUS_PIN_SEL_MASK	(0x7)

/* GMBUS2 bits definitions */
#define _GMBUS_IN_USE		(1 << 15)
#define _GMBUS_HW_WAIT		(1 << 14)
#define _GMBUS_HW_RDY		(1 << 11)
#define _GMBUS_INT_STAT		(1 << 12)
#define _GMBUS_NAK		(1 << 10)
#define _GMBUS_ACTIVE		(1 << 9)

#define _GMBUS_SLAVE_READ	(1 << 0)
#define _GMBUS_SLAVE_WRITE	(0 << 0)
#define _GMBUS_BYTE_COUNT_SHIFT	16
#define _GMBUS_SLAVE_ADDR_SHIFT	1
#define _GMBUS_TRANS_MAX_BYTES	((1 << 9) - 1)

#define _REG_GTFIFODBG			0x120000
#define _REG_GTFIFO_FREE_ENTRIES	0x120008
#define _REG_MCHBAR_MIRROR		0x140000
#define _REG_UCG_CTL1			0x9400
#define _REG_UCG_CTL2			0x9404
#define _REG_RC_PWRCTX_MAXCNT		0x2054
#define _REG_3D_CHICKEN1		0x2084
#define _REG_3D_CHICKEN2		0x208C
#define _REG_3D_CHICKEN3		0x2090
#define _REG_RCS_ECOSKPD		0x21d0
#define _REG_BCS_ECOSKPD		0x221d0
#define _REG_VFSKPD			0x2470
#define _REG_ECOCHK			0x4090
#define _REG_GAC_ECOCHK			0x14090
#define _REG_2D_CG_DIS			0x6200
#define _REG_3D_CG_DIS			0x6204
#define _REG_3D_CG_DIS2			0x6208
#define _REG_SNPCR			0x900c
#define _REG_MBCTL			0x907c
#define _REG_GAB_CTL			0x24000
#define _REG_SUPER_QUEUE_CONFIG		0x902c
#define _REG_MISC_CLOCK_GATING		0x9424
#define _REG_GTDRIVER_MAILBOX_INTERFACE	0x138124
#define _REG_GTDRIVER_MAILBOX_DATA0	0x138128

/*
 * GPIO regs
 */
#define _REG_GMBUS0			0x5100 /* clock/port select */

enum vgt_pipe {
	PIPE_A = 0,
	PIPE_B,
	PIPE_C,
	I915_MAX_PIPES
};

enum vgt_port {
	PORT_A = 0,
	PORT_B,
	PORT_C,
	PORT_D,
	PORT_E,
	I915_MAX_PORTS
};

#define VGT_PORT_NAME(p)	\
	((p) == PORT_A ? "PORT_A" : \
	((p) == PORT_B ? "PORT_B" : \
	((p) == PORT_C ? "PORT_C" : \
	((p) == PORT_D ? "PORT_D" : \
	((p) == PORT_E ? "PORT_E" : "PORT_X")))))

#define VGT_PIPE_NAME(p)	\
	((p) == PIPE_A ? "Pipe A" : \
		((p) == PIPE_B ? "Pipe B" : \
			((p) == PIPE_C ? "Pipe C" : "PIPE X")))
#define VGT_PIPE_CHAR(p)	\
	((p) == PIPE_A ? 'A' : \
		((p) == PIPE_B ? 'B' : \
			((p) == PIPE_C ? 'C' : 'X')))

enum vgt_plane_type {
	PRIMARY_PLANE = 0,
	CURSOR_PLANE,
	SPRITE_PLANE,
	MAX_PLANE
};

enum vgt_port_type {
	VGT_CRT = 0,
	VGT_DP_A,
	VGT_DP_B,
	VGT_DP_C,
	VGT_DP_D,
	VGT_HDMI_B,
	VGT_HDMI_C,
	VGT_HDMI_D,
	VGT_PORT_MAX
};

#define VGT_PORT_TYPE_NAME(p)	\
        ((p) == VGT_CRT ? "VGT_CRT" : \
        ((p) == VGT_DP_A ? "VGT_DP_A" : \
        ((p) == VGT_DP_B ? "VGT_DP_B" : \
        ((p) == VGT_DP_C ? "VGT_DP_C" : \
	((p) == VGT_DP_D ? "VGT_DP_D" : \
	((p) == VGT_HDMI_B ? "VGT_HDMI_B" : \
	((p) == VGT_HDMI_C ? "VGT_HDMI_C" : \
	((p) == VGT_HDMI_D ? "VGT_HDMI_D" : "UNKNOWN"))))))))

static inline int port_to_port_type(int port_sel)
{
        switch(port_sel) {
        case PORT_A:
                return VGT_DP_A;
        case PORT_B:
                return VGT_DP_B;
        case PORT_C:
                return VGT_DP_C;
        case PORT_D:
                return VGT_DP_D;
        case PORT_E:
                return VGT_CRT;
	}
        return VGT_PORT_MAX;
}

static inline int port_type_to_port(int port_sel)
{
	switch(port_sel) {
	case VGT_DP_A:
		return PORT_A;
	case VGT_DP_B:
	case VGT_HDMI_B:
		return PORT_B;
	case VGT_DP_C:
	case VGT_HDMI_C:
		return PORT_C;
	case VGT_DP_D:
	case VGT_HDMI_D:
		return PORT_D;
	case VGT_CRT:
		return PORT_E;
	}

	return I915_MAX_PORTS;
}

/* interrupt related definitions */
#define _REG_DEISR	0x44000
#define _REG_DEIMR	0x44004
#define _REG_DEIIR	0x44008
#define _REG_DEIER	0x4400C
#define        _REGSHIFT_MASTER_INTERRUPT	31
#define        _REGBIT_MASTER_INTERRUPT	(1 << 31)
#define        _REGBIT_DP_A_HOTPLUG		(1 << 19)
#define        _REGBIT_DP_A_HOTPLUG_IVB		(1 << 27)
#define        _REGBIT_PIPE_A_VBLANK		(1 << 7)
#define        _REGSHIFT_PCH			21
#define        _REGBIT_PCH			(1 << 21)
/* GEN7 */
#define        _REGSHIFT_PCH_GEN7		28
#define        _REGBIT_PCH_GEN7			(1 << 28)
#define _REG_GTISR	0x44010
#define _REG_GTIMR	0x44014
#define _REG_GTIIR	0x44018
#define _REG_GTIER	0x4401C
#define _REG_PMISR	0x44020
#define _REG_PMIMR	0x44024
#define _REG_PMIIR	0x44028
#define _REG_PMIER	0x4402C
#define _REG_DP_A_HOTPLUG_CNTL	0x44030
#define        _REGBIT_DP_A_HOTPLUG_STATUS		(3 << 0)
#define        _REGBIT_DP_A_PULSE_DURATION		(3 << 2)
#define        _REGBIT_DP_A_HOTPLUG_ENABLE		(1 << 4)
#define _REG_GTT_FAULT_STATUS	0x44040

#define    _REG_SDEISR	0xC4000
#define    _REG_SDEIMR	0xC4004
#define    _REG_SDEIIR	0xC4008
#define        _REGBIT_CRT_HOTPLUG		(1 << 19)
#define        _REGBIT_DP_B_HOTPLUG		(1 << 21)
#define        _REGBIT_DP_C_HOTPLUG		(1 << 22)
#define        _REGBIT_DP_D_HOTPLUG		(1 << 23)
#define    _REG_SDEIER	0xC400C
#define _REG_SHOTPLUG_CTL	0xC4030
#define        _REGBIT_DP_B_STATUS			(3 << 0)
#define        _REGBIT_DP_B_PULSE_DURATION		(3 << 2)
#define        _REGBIT_DP_B_ENABLE			(1 << 4)
#define        _REGBIT_DP_C_STATUS			(3 << 8)
#define        _REGBIT_DP_C_PULSE_DURATION		(3 << 10)
#define        _REGBIT_DP_C_ENABLE			(1 << 12)
#define        _REGBIT_DP_D_STATUS			(3 << 16)
#define        _REGBIT_DP_D_PULSE_DURATION		(3 << 18)
#define        _REGBIT_DP_D_ENABLE			(1 << 20)

#define RING_IMR(ring) \
	__RING_REG((ring), _REG_RCS_IMR)

#define _REG_RCS_WATCHDOG_CTL	0x2178
#define _REG_RCS_WATCHDOG_THRSH	0x217C
#define _REG_RCS_WATCHDOG_CTR	0x2190
#define _REG_VCS_WATCHDOG_CTR	0x12178
#define _REG_VCS_WATCHDOG_THRSH	0x1217C

#define _REG_RCS_EIR	0x20B0
#define _REG_RCS_EMR	0x20B4
#define _REG_RCS_ESR	0x20B8
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
	__RING_REG((ring), _REG_RCS_EIR)
#define RING_EMR(ring) \
	__RING_REG((ring), _REG_RCS_EMR)
#define RING_ESR(ring) \
	__RING_REG((ring), _REG_RCS_ESR)

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
#define _REG_BLC_PWM_CTL2	0x48250
#define        _REGBIT_PHASE_IN_IRQ_ENABLE	(1 << 24)
#define        _REGBIT_PHASE_IN_IRQ_STATUS	(1 << 26)
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

//#define MSAC_APERTURE_SIZE_MASK		0x3
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
#define _REG_LCPLL_CTL		0x130040
#define  _REGBIT_LCPLL_PLL_DISABLE		(1<<31)
#define  _REGBIT_LCPLL_PLL_LOCK			(1<<30)
#define  _REGBIT_LCPLL_CLK_FREQ_MASK		(3<<26)
#define  _REGBIT_LCPLL_CD_SOURCE_FCLK		(1<<21)
#define  _REGBIT_LCPLL_CD_SOURCE_FCLK_DONE	(1<<19)
#define  _LCPLL_CLK_FREQ_450		(0<<26)
#define _REG_HSW_FUSE_STRAP	0x42014
#define  _REGBIT_HSW_CDCLK_LIMIT	(1 << 24)
#define _REG_GFX_FLSH_CNT	0x101008

#define _REG_HSW_PWR_WELL_CTL1	0x45400
#define _REG_HSW_PWR_WELL_CTL2	0x45404
#define _REG_HSW_PWR_WELL_CTL3	0x45408
#define _REG_HSW_PWR_WELL_CTL4	0x4540C
#define _REG_HSW_PWR_WELL_CTL5	0x45410
#define _REG_HSW_PWR_WELL_CTL6	0x45414

#define   _REGBIT_HSW_PWR_WELL_ENABLE			(1<<31)
#define   _REGBIT_HSW_PWR_WELL_STATE				(1<<30)
#define   _REGBIT_HSW_PWR_WELL_ENABLE_SINGLE_STEP	(1<<31)
#define   _REGBIT_HSW_PWR_WELL_PWR_GATE_OVERRIDE	(1<<20)
#define   _REGBIT_HSW_PWR_WELL_FORCE_ON			(1<<19)

#define _REG_SPLL_CTL		0x46020
#define  _REGBIT_SPLL_CTL_ENABLE	(1 << 31)

#define _REG_WRPLL_CTL1		0x46040
#define _REG_WRPLL_CTL2		0x46060
#define  _REGBIT_WRPLL_ENABLE	(1 << 31)

#define _REG_PORT_CLK_SEL_DDIA	0x46100
#define _REG_PORT_CLK_SEL_DDIB	0x46104
#define _REG_PORT_CLK_SEL_DDIC	0x46108
#define _REG_PORT_CLK_SEL_DDID	0x4610C
#define _REG_PORT_CLK_SEL_DDIE	0x46110

#define _REG_TRANS_CLK_SEL_A	0x46140
#define _REG_TRANS_CLK_SEL_B	0x46144
#define _REG_TRANS_CLK_SEL_C	0x46148

#define _REG_SBI_ADDR			0xc6000
#define _REG_SBI_DATA			0xc6004
#define _REG_SBI_CTL_STAT		0xc6008
#define _SBI_RESPONSE_MASK		0x3
#define _SBI_RESPONSE_SHIFT		0x1
#define _SBI_STAT_MASK			0x1
#define _SBI_STAT_SHIFT			0x0
#define _SBI_RESPONSE_FAIL		(0x1<<_SBI_RESPONSE_SHIFT)
#define _SBI_RESPONSE_SUCCESS		(0x0<<_SBI_RESPONSE_SHIFT)
#define _SBI_BUSY			(0x1<<_SBI_STAT_SHIFT)
#define _SBI_READY			(0x0<<_SBI_STAT_SHIFT)
#define _SBI_OPCODE_SHIFT		8
#define _SBI_OPCODE_MASK		(0xff << _SBI_OPCODE_SHIFT)
#define _SBI_CMD_IORD			2
#define _SBI_CMD_IOWR			3
#define _SBI_CMD_CRRD			6
#define _SBI_CMD_CRWR			7
#define _SBI_ADDR_OFFSET_SHIFT		16
#define _SBI_ADDR_OFFSET_MASK		(0xffff << _SBI_ADDR_OFFSET_SHIFT)

#define _REG_TRANS_DDI_FUNC_CTL_A	0x60400
#define _REG_TRANS_DDI_FUNC_CTL_B	0x61400
#define _REG_TRANS_DDI_FUNC_CTL_C	0x62400
#define _REG_TRANS_DDI_FUNC_CTL_EDP	0x6F400

#define _VGT_TRANS_DDI_FUNC_CTL(tran)   _VGT_TRANSCODER(tran, _REG_TRANS_DDI_FUNC_CTL_A, \
						   _REG_TRANS_DDI_FUNC_CTL_B)


#define  _REGBIT_TRANS_DDI_FUNC_ENABLE		(1<<31)
/* Those bits are ignored by pipe EDP since it can only connect to DDI A */
#define  _TRANS_DDI_PORT_SHIFT			28
#define  _REGBIT_TRANS_DDI_PORT_MASK		(7<<_TRANS_DDI_PORT_SHIFT)
#define  _REGBIT_TRANS_DDI_SELECT_PORT(x)	((x)<<_TRANS_DDI_PORT_SHIFT)
#define  _REGBIT_TRANS_DDI_PORT_NONE		(0<<_TRANS_DDI_PORT_SHIFT)
#define  _TRANS_DDI_MODE_SELECT_HIFT		24
#define  _REGBIT_TRANS_DDI_MODE_SELECT_MASK	(7<<_TRANS_DDI_MODE_SELECT_HIFT)
#define  _REGBIT_TRANS_DDI_MODE_SELECT_HDMI	(0<<_TRANS_DDI_MODE_SELECT_HIFT)
#define  _REGBIT_TRANS_DDI_MODE_SELECT_DVI	(1<<_TRANS_DDI_MODE_SELECT_HIFT)
#define  _REGBIT_TRANS_DDI_MODE_SELECT_DP_SST	(2<<_TRANS_DDI_MODE_SELECT_HIFT)
#define  _REGBIT_TRANS_DDI_MODE_SELECT_DP_MST	(3<<_TRANS_DDI_MODE_SELECT_HIFT)
#define  _REGBIT_TRANS_DDI_MODE_SELECT_FDI	(4<<_TRANS_DDI_MODE_SELECT_HIFT)
#define  _REGBIT_TRANS_DDI_BPC_MASK		(7<<20)
#define  _REGBIT_TRANS_DDI_BPC_8		(0<<20)
#define  _REGBIT_TRANS_DDI_BPC_10		(1<<20)
#define  _REGBIT_TRANS_DDI_BPC_6		(2<<20)
#define  _REGBIT_TRANS_DDI_BPC_12		(3<<20)
#define  _REGBIT_TRANS_DDI_PVSYNC		(1<<17)
#define  _REGBIT_TRANS_DDI_PHSYNC		(1<<16)
#define  _TRANS_DDI_EDP_INPUT_SHIFT		12
#define  _REGBIT_TRANS_DDI_EDP_INPUT_MASK	(7<<_TRANS_DDI_EDP_INPUT_SHIFT)
#define  _REGBIT_TRANS_DDI_EDP_INPUT_A_ON	(0<<_TRANS_DDI_EDP_INPUT_SHIFT)
#define  _REGBIT_TRANS_DDI_EDP_INPUT_A_ONOFF	(4<<_TRANS_DDI_EDP_INPUT_SHIFT)
#define  _REGBIT_TRANS_DDI_EDP_INPUT_B_ONOFF	(5<<_TRANS_DDI_EDP_INPUT_SHIFT)
#define  _REGBIT_TRANS_DDI_EDP_INPUT_C_ONOFF	(6<<_TRANS_DDI_EDP_INPUT_SHIFT)
#define  _REGBIT_TRANS_DDI_BFI_ENABLE		(1<<4)
#define  _REGBIT_TRANS_DDI_PORT_WIDTH_X1	(0<<1)
#define  _REGBIT_TRANS_DDI_PORT_WIDTH_X2	(1<<1)
#define  _REGBIT_TRANS_DDI_PORT_WIDTH_X4	(3<<1)


#define _REG_TRANS_MSA_MISC_A	0x60410
#define _REG_TRANS_MSA_MISC_B	0x61410
#define _REG_TRANS_MSA_MISC_C	0x62410

#define _REG_GEN7_COMMON_SLICE_CHICKEN1		0x7010
#define _REG_GEN7_COMMON_SLICE_CHICKEN2		0x7014
#define _REG_GEN7_L3CNTLREG1			0xB01C
#define _REG_GEN7_L3_CHICKEN_MODE_REGISTER	0xB030
#define _REG_GEN7_SQ_CHICKEN_MBCUNIT_CONFIG	0x9030
#define _REG_WM_DBG				0x45280

#define _REG_PIPE_WM_LINETIME_A			0x45270
#define _REG_PIPE_WM_LINETIME_B			0x45274
#define _REG_PIPE_WM_LINETIME_C			0x45278

#define _REG_HSW_VIDEO_DIP_CTL_A		0x60200
#define _REG_HSW_VIDEO_DIP_CTL_B		0x61200
#define _REG_HSW_VIDEO_DIP_CTL_C		0x62200
#define _REG_HSW_VIDEO_DIP_CTL_EDP		0x6F200

#define _REG_DPA_AUX_CH_CTL			0x64010
#define _REG_DPA_AUX_CH_DATA1			0x64014

#define _REG_DDI_BUF_TRANS_A			0x64E00
#define _REG_HSW_AUD_CONFIG_A			0x65000

#define _REG_SFUSE_STRAP			0xC2014
#define  _REGBIT_SFUSE_STRAP_B_PRESENTED	(1 << 2)
#define  _REGBIT_SFUSE_STRAP_C_PRESENTED	(1 << 1)
#define  _REGBIT_SFUSE_STRAP_D_PRESENTED	(1 << 0)

#define _REG_PIXCLK_GATE			0xC6020

#define _REG_SCRATCH1				0xB038
#define _REG_GEN7_L3SQCREG4			0xB034
#define _REG_ROW_CHICKEN3			0xE49C

#define _REG_FPGA_DBG				0x42300
#define _REGBIT_FPGA_DBG_RM_NOCLAIM		(1 << 31)

/* GEN8 interrupt registers definations */
#define _REG_MASTER_IRQ			0x44200
#define  _REGBIT_MASTER_IRQ_CONTROL	(1<<31)
#define  _REGBIT_PCU_IRQ			(1<<30)
#define  _REGBIT_DE_PCH_IRQ		(1<<23)
#define  _REGBIT_DE_MISC_IRQ		(1<<22)
#define  _REGBIT_DE_PORT_IRQ		(1<<20)
#define  _REGBIT_DE_PIPE_C_IRQ		(1<<18)
#define  _REGBIT_DE_PIPE_B_IRQ		(1<<17)
#define  _REGBIT_DE_PIPE_A_IRQ		(1<<16)
#define  _REGBIT_GT_VECS_IRQ		(1<<6)
#define  _REGBIT_GT_VCS2_IRQ		(1<<3)
#define  _REGBIT_GT_VCS1_IRQ		(1<<2)
#define  _REGBIT_GT_BCS_IRQ		(1<<1)
#define  _REGBIT_GT_RCS_IRQ		(1<<0)

#define _REG_GT_ISR(which) (0x44300 + (0x10 * (which)))
#define _REG_GT_IMR(which) (0x44304 + (0x10 * (which)))
#define _REG_GT_IIR(which) (0x44308 + (0x10 * (which)))
#define _REG_GT_IER(which) (0x4430c + (0x10 * (which)))

#define _REG_DE_PIPE_ISR(pipe) (0x44400 + (0x10 * (pipe)))
#define _REG_DE_PIPE_IMR(pipe) (0x44404 + (0x10 * (pipe)))
#define _REG_DE_PIPE_IIR(pipe) (0x44408 + (0x10 * (pipe)))
#define _REG_DE_PIPE_IER(pipe) (0x4440c + (0x10 * (pipe)))

#define _REG_DE_PORT_ISR 0x44440
#define _REG_DE_PORT_IMR 0x44444
#define _REG_DE_PORT_IIR 0x44448
#define _REG_DE_PORT_IER 0x4444c
#define  _REGBIT_PORT_DP_A_HOTPLUG	(1 << 3)
#define  _REGBIT_AUX_CHANNEL_A	(1 << 0)

#define _REG_DE_MISC_ISR 0x44460
#define _REG_DE_MISC_IMR 0x44464
#define _REG_DE_MISC_IIR 0x44468
#define _REG_DE_MISC_IER 0x4446c

#define _REG_PCU_ISR 0x444e0
#define _REG_PCU_IMR 0x444e4
#define _REG_PCU_IIR 0x444e8
#define _REG_PCU_IER 0x444ec

#define _REG_GEN8_PRIVATE_PAT  0x40e0

#define _REG_GAMTARBMODE		0x04a08

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

#endif	/* _VGT_REG_H_ */
