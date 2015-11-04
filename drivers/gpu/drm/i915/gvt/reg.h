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

#ifndef _GVT_REG_H
#define _GVT_REG_H

#define GVT_CFG_SPACE_SZ	256
#define GVT_BAR_NUM		4

#define GVT_PCI_CLASS_VGA			0x03
#define GVT_PCI_CLASS_VGA_OTHER			0x80

#define GVT_REG_CFG_VENDOR_ID                   0x00
#define GVT_REG_CFG_COMMAND                     0x04
#define _REGBIT_CFG_COMMAND_IO                  (1 << 0)
#define _REGBIT_CFG_COMMAND_MEMORY              (1 << 1)
#define _REGBIT_CFG_COMMAND_MASTER              (1 << 2)
#define GVT_REG_CFG_CLASS_PROG_IF               0x09
#define GVT_REG_CFG_SUB_CLASS_CODE              0x0A
#define GVT_REG_CFG_CLASS_CODE                  0x0B
#define GVT_REG_CFG_SPACE_BAR0                  0x10
#define GVT_REG_CFG_SPACE_BAR1                  0x18
#define GVT_REG_CFG_SPACE_BAR2                  0x20
#define GVT_REG_CFG_SPACE_BAR_ROM               0x30
#define GVT_REG_CFG_SPACE_MSAC                  0x62
#define GVT_REG_CFG_SWSCI_TRIGGER               0xE8
#define _REGBIT_CFG_SWSCI_SCI_SELECT            (1 << 15)
#define _REGBIT_CFG_SWSCI_SCI_TRIGGER           1
#define GVT_REG_CFG_OPREGION                    0xFC

#define GVT_OPREGION_PAGES                      2
#define GVT_OPREGION_PORDER                     1
#define GVT_OPREGION_SIZE                       (8 * 1024)
#define GVT_OPREGION_REG_CLID                   0x1AC
#define GVT_OPREGION_REG_SCIC                   0x200
#define _REGBIT_OPREGION_SCIC_FUNC_MASK         0x1E
#define _REGBIT_OPREGION_SCIC_FUNC_SHIFT        1
#define _REGBIT_OPREGION_SCIC_SUBFUNC_MASK      0xFF00
#define _REGBIT_OPREGION_SCIC_SUBFUNC_SHIFT     8
#define _REGBIT_OPREGION_SCIC_EXIT_MASK         0xE0
#define GVT_OPREGION_SCIC_F_GETBIOSDATA         4
#define GVT_OPREGION_SCIC_F_GETBIOSCALLBACKS    6
#define GVT_OPREGION_SCIC_SF_SUPPRTEDCALLS      0
#define GVT_OPREGION_SCIC_SF_REQEUSTEDCALLBACKS 1
#define GVT_OPREGION_REG_PARM			0x204

#define _REG_GMCH_CONTROL               0x50
#define    _REGBIT_BDW_GMCH_GMS_SHIFT   8
#define    _REGBIT_BDW_GMCH_GMS_MASK    0xff

#define _GVT_MMIO_THROUGH_OFFSET(index, a, b)	((a) + (index)*((b)-(a)))
#define _GVT_MMIO_GET_INDEX(reg, a, b)		(((reg)-(a))/((b)-(a)))

#define _GVT_GET_PIPE(reg, a, b)	_GVT_MMIO_GET_INDEX(reg, a, b)
#define _GVT_GET_PORT(reg, a, b)	_GVT_MMIO_GET_INDEX(reg, a, b)

#define _REG_GAC_MODE		0x120A0
#define _REG_GAB_MODE		0x220A0

#define _REG_RCS_BB_ADDR	0x2140
#define _REG_VCS_BB_ADDR	0x12140
#define _REG_BCS_BB_ADDR	0x22140
#define _REG_VECS_BB_ADDR	0x1A140
#define _REG_VCS2_BB_ADDR	0x1c140

#define _REG_VECS_CTX_WA_BB_ADDR 0x1A144

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

#define _REG_RCS_ACTHD_UDW	0x205c
#define _REG_VCS_ACTHD_UDW	0x1205c
#define _REG_BCS_ACTHD_UDW	0x2205c
#define _REG_VECS_ACTHD_UDW	0x1A05c
#define _REG_VCS2_ACTHD_UDW	0x1c05c

#define _REG_RCS_BB_PREEMPT_ADDR	0x2148
#define _REG_RCS_BB_ADDR_DIFF		0x2154

#define _REG_RCS_TIMESTAMP	0x2358
#define _REG_VCS_TIMESTAMP	0x12358
#define _REG_VCS2_TIMESTAMP	0x1c358
#define _REG_BCS_TIMESTAMP	0x22358

#define GVT_RING_MODE(base) (base + 0x29c)

#define RB_HEAD_OFF_MASK        ((1U << 21) - (1U << 2))
#define RB_TAIL_OFF_MASK       ((1U << 21) - (1U << 3))        /* bit 3 to 20 */
#define RB_TAIL_SIZE_MASK      ((1U << 21) - (1U << 12))       /* bit 12 to 20 */
#define _RING_CTL_BUF_SIZE(ctl)        (((ctl) & RB_TAIL_SIZE_MASK) + GTT_PAGE_SIZE)

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

#define _REG_CURASURFLIVE	0x700AC

#define _REG_CURAPALET_0	0x70090
#define _REG_CURAPALET_1	0x70094
#define _REG_CURAPALET_2	0x70098
#define _REG_CURAPALET_3	0x7009C

#define	_PRI_PLANE_FMT_SHIFT	26
#define	_PRI_PLANE_TILE_SHIFT	10

#define	_PRI_PLANE_STRIDE_SHIFT	6
#define	_PRI_PLANE_STRIDE_MASK	(0x3ff << _PRI_PLANE_STRIDE_SHIFT)

#define	_PRI_PLANE_X_OFF_SHIFT	0
#define	_PRI_PLANE_X_OFF_MASK	(0x1fff << _PRI_PLANE_X_OFF_SHIFT)
#define	_PRI_PLANE_Y_OFF_SHIFT	16
#define	_PRI_PLANE_Y_OFF_MASK	(0xfff << _PRI_PLANE_Y_OFF_SHIFT)

#define     _PIPE_V_SRCSZ_SHIFT	0
#define     _PIPE_V_SRCSZ_MASK	(0xfff << _PIPE_V_SRCSZ_SHIFT)
#define     _PIPE_H_SRCSZ_SHIFT	16
#define     _PIPE_H_SRCSZ_MASK	(0x1fff << _PIPE_H_SRCSZ_SHIFT)

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

#define GVT_CURCNTR(pipe)	_PIPE(pipe, _CURACNTR, _CURBCNTR_IVB)
#define GVT_CURBASE(pipe)	_PIPE(pipe, _CURABASE, _CURBBASE_IVB)
#define GVT_CURPOS(pipe)	_PIPE(pipe, _CURAPOS, _CURBPOS_IVB)

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

#define GVT_SPRCTL(pipe)	_PIPE(pipe, _SPRA_CTL, _PLANE_CTL_2_B)
#define GVT_SPRSTRIDE(pipe)	_PIPE(pipe, _SPRA_STRIDE, _PLANE_STRIDE_2_B)
#define GVT_SPRPOS(pipe)	_PIPE(pipe, _PLANE_POS_2_A, _PLANE_POS_2_B)
#define GVT_SPRSIZE(pipe)	_PIPE(pipe, _PLANE_SIZE_2_A, _PLANE_SIZE_2_B)
#define GVT_SPROFFSET(pipe)	_PIPE(pipe, _PLANE_OFFSET_2_A, _PLANE_OFFSET_2_B)

#define GVT_DSPCNTR(pipe) _PIPE(pipe, _DSPACNTR, 0x71180)
#define GVT_DSPCNTRPIPE(dspcntr) _GVT_GET_PIPE(dspcntr, _DSPACNTR, 0x71180)

#define GVT_DSPLINOFF(pipe) _PIPE(pipe, _DSPAADDR, 0x71184)
#define GVT_DSPSTRIDE(pipe) _PIPE(pipe, _DSPASTRIDE, 0x71188)
#define GVT_DSPTILEOFF(pipe) _PIPE(pipe, _DSPATILEOFF, 0x711A4)

#define GVT_DSPSURF(pipe) _PIPE(pipe, _DSPASURF, 0x7119C)
#define GVT_DSPSURFPIPE(dspsurf) _GVT_GET_PIPE(dspsurf, _DSPASURF, 0x7119C)

#define GVT_DSPSURFLIVE(pipe) _PIPE(pipe, _DSPASURFLIVE, 0x711AC)
#define GVT_DSPSURFLIVEPIPE(dspsurf) _GVT_GET_PIPE(dspsurf, _DSPASURFLIVE, 0x711AC)

#define GVT_DSPPOS(pipe) _PIPE(pipe, _DSPAPOS, 0x7118C)
#define GVT_DSPSIZE(pipe) _PIPE(pipe, _DSPASIZE, 0x71190)

#define GVT_CURSURF(pipe) _PIPE(pipe, _CURABASE, _CURBBASE_IVB)
#define GVT_CURCNTR(pipe) _PIPE(pipe, _CURACNTR, _CURBCNTR_IVB)
#define GVT_CURPOS(pipe)	_PIPE(pipe, _CURAPOS, _CURBPOS_IVB)
#define GVT_CURSURFLIVE(pipe) _PIPE(pipe, 0x700AC, 0x710AC)

#define GVT_CURSURFPIPE(cursurf) _GVT_GET_PIPE(cursurf, _CURABASE, _CURBBASE_IVB)
#define GVT_CURCNTRPIPE(curcntr) _GVT_GET_PIPE(curcntr, _CURACNTR,_CURBCNTR_IVB)

#define GVT_SPRSURF(pipe) _PIPE(pipe, _SPRA_SURF, _SPRB_SURF)
#define GVT_SPRSURFLIVE(pipe) _PIPE(pipe, _SPRA_SURFLIVE, _SPRB_SURFLIVE)
#define GVT_SPRSURFPIPE(sprsurf) _GVT_GET_PIPE(sprsurf, _SPRA_SURF, _SPRB_SURF)

#define GVT_SPRCNTR(pipe) _PIPE(pipe, _SPRA_CTL, _SPRB_CTL)
#define GVT_SPRCNTRPIPE(sprcntr) _GVT_GET_PIPE(sprcntr, _SPRA_CTL, _SPRB_CTL)

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
#define DUMMY_3D		(0x6d800005)
#define PRIM_TRILIST		(0x4)

/*
 * Display engine regs
 */

#define _REG_SWF		0x4f000
#define _REG_PIPE_MISC_C	0x72030

/* CPU panel fitter */
#define _REG_PF_CTL_2			0x69080
#define _REG_PF_WIN_SZ_2		0x69074
#define _REG_PF_WIN_POS_2		0x69070

#define GVT_HTOTAL(pipe)	_PIPE(pipe, _HTOTAL_A, _HTOTAL_B)
#define GVT_HBLANK(pipe)	_PIPE(pipe, _HBLANK_A, _HBLANK_B)
#define GVT_HSYNC(pipe)		_PIPE(pipe, _HSYNC_A, _HSYNC_B)
#define GVT_VTOTAL(pipe)	_PIPE(pipe, _VTOTAL_A, _VTOTAL_B)
#define GVT_VBLANK(pipe)	_PIPE(pipe, _VBLANK_A, _VBLANK_B)
#define GVT_VSYNC(pipe)		_PIPE(pipe, _VSYNC_A, _VSYNC_B)
#define GVT_BCLRPAT(pipe)	_PIPE(pipe, _BCLRPAT_A, _BCLRPAT_B)
#define GVT_VSYNCSHIFT(pipe)	_PIPE(pipe, _VSYNCSHIFT_A, _VSYNCSHIFT_B)
#define GVT_PIPESRC(pipe)	_PIPE(pipe, _PIPEASRC, _PIPEBSRC)

#define GVT_PCH_DPLL(pipe)	_PIPE(pipe, _REG_PCH_DPLL_A, _REG_PCH_DPLL_B)

#define GVT_PCH_FP0(pipe)	_PIPE(pipe, _REG_PCH_FPA0, _REG_PCH_FPB0)
#define GVT_PCH_FP1(pipe)	_PIPE(pipe, _REG_PCH_FPA1, _REG_PCH_FPB1)

/* PIPE C timing regs are same start from 0x61000 */
#define _REG_PIPEC_DATA_M1		0x62030
#define _REG_PIPEC_DATA_N1		0x62034
#define _REG_PIPEC_LINK_M1		0x62040
#define _REG_PIPEC_LINK_N1		0x62044

#define _REG_PIPEC_DATA_M2		0x62038
#define _REG_PIPEC_DATA_N2		0x6203c
#define _REG_PIPEC_LINK_M2		0x62048
#define _REG_PIPEC_LINK_N2		0x6204c

#define GVT_PIPE_DATA_M1(pipe) _PIPE(pipe, _REG_PIPEA_DATA_M1, _REG_PIPEB_DATA_M1)
#define GVT_PIPE_DATA_N1(pipe) _PIPE(pipe, _REG_PIPEA_DATA_N1, _REG_PIPEB_DATA_N1)
#define GVT_PIPE_DATA_M2(pipe) _PIPE(pipe, _REG_PIPEA_DATA_M2, _REG_PIPEB_DATA_M2)
#define GVT_PIPE_DATA_N2(pipe) _PIPE(pipe, _REG_PIPEA_DATA_N2, _REG_PIPEB_DATA_N2)
#define GVT_PIPE_LINK_M1(pipe) _PIPE(pipe, _REG_PIPEA_LINK_M1, _REG_PIPEB_LINK_M1)
#define GVT_PIPE_LINK_N1(pipe) _PIPE(pipe, _REG_PIPEA_LINK_N1, _REG_PIPEB_LINK_N1)
#define GVT_PIPE_LINK_M2(pipe) _PIPE(pipe, _REG_PIPEA_LINK_M2, _REG_PIPEB_LINK_M2)
#define GVT_PIPE_LINK_N2(pipe) _PIPE(pipe, _REG_PIPEA_LINK_N2, _REG_PIPEB_LINK_N2)

/* FDI_RX, FDI_X is hard-wired to Transcoder_X */

#define _REG_FDI_RXC_CTL			0xf200c
#define _REGBIT_FDI_RX_PORT_WIDTH_MASK		(0x7 << 19)
#define _REGBIT_FDI_RX_FDI_AUTO_TRAIN_ENABLE	(0x1 << 10)

#define _REG_FDI_RXC_IIR			0xf2014
#define _REG_FDI_RXC_IMR			0xf2018

#define GVT_FDI_RX_IIR(pipe) _PIPE(pipe, _FDI_RXA_IIR, _FDI_RXB_IIR)
#define GVT_FDI_RX_IMR(pipe) _PIPE(pipe, _FDI_RXA_IMR, _FDI_RXB_IMR)

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


#define GVT_FDI_RX_CTL_BPC_MASK		(0x7 << 16)
#define GVT_FDI_RX_CTL(pipe) _PIPE(pipe, _FDI_RXA_CTL, _FDI_RXB_CTL)

#define GVT_FDI_RX_TUSIZE1(pipe) _PIPE(pipe, _REG_FDI_RXA_TUSIZE1,_REG_FDI_RXB_TUSIZE1)

/* CPU: FDI_TX */
#define _REG_FDI_TXC_CTL		0x62100

#define _REGBIT_FDI_TX_ENABLE				(1 << 31)
#define _REGBIT_FDI_TX_PLL_ENABLE			(1 << 14)
#define _REGBIT_FDI_TX_ENHANCE_FRAME_ENABLE		(1<<18)

#define GVT_FDI_TX_CTL(pipe) _PIPE(pipe, _FDI_TXA_CTL, _FDI_TXB_CTL)

/* CRT */
#define _REGBIT_ADPA_DAC_ENABLE			(1 << 31)
#define PORT_TRANS_SEL_SHIFT			29
#define GVT_PORT_TRANS_SEL_CPT(pipe)		((pipe) << PORT_TRANS_SEL_SHIFT)
#define _REGBIT_ADPA_VSYNC_ACTIVE_HIGH		(1 << 4)
#define _REGBIT_ADPA_HSYNC_ACTIVE_HIGH		(1 << 3)

/*Intermediate Pixel Storage*/
union _PCH_PP_CTL
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

/* Per-transcoder DIP controls */

#define GVT_TRANSCONF(plane)	_PIPE(plane, _PCH_TRANSACONF, _PCH_TRANSBCONF)

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

/* eDP */
#define _REG_PIPE_EDP_CONF	0x7f008

/* bit fields of pipestat */
#define GVT_PIPEDSL(pipe)	_PIPE(pipe, _PIPEADSL, 0x71000)
#define GVT_PIPECONF(pipe)	_PIPE(pipe, _PIPEACONF, 0x71008)
#define GVT_PIPESTAT(pipe)	_PIPE(pipe, _PIPEASTAT, 0x71024)
#define GVT_PIPE_FRMCOUNT(pipe)	_PIPE(pipe, _PIPEA_FRMCOUNT_G4X, 0x71040)
#define GVT_PIPE_FLIPCOUNT(pipe) _PIPE(pipe, _PIPEA_FLIPCOUNT_G4X, 0x71044)

#define GVT_PIPECONFPIPE(pipeconf) _GVT_GET_PIPE(pipeconf, _PIPEACONF, 0x71008)
#define GVT_FRMCOUNTPIPE(frmcount) _GVT_GET_PIPE(frmcount, _PIPEA_FRMCOUNT_G4X, 0x71040)

#define GVT_PALETTE(pipe) _PIPE(pipe, _PALETTE_A_OFFSET, _PALETTE_B_OFFSET)

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
#define GVT_DP_TP_CTL(port)		_PORT(port, _DP_TP_CTL_A, \
		_DP_TP_CTL_B)

#define DP_TP_PORT(reg)		_GVT_GET_PORT(reg, _DP_TP_CTL_A, \
		_DP_TP_CTL_B)

#define DRM_MODE_DPMS_ON		0

#define _PCH_DPA_AUX_CH_CTL	0x64010
#define _PCH_DPB_AUX_CH_CTL	0xe4110
#define _PCH_DPC_AUX_CH_CTL	0xe4210
#define _PCH_DPD_AUX_CH_CTL	0xe4310

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

#define _REG_DP_BUFTRANS		0xe4f00

#define _PCH_GMBUS0			0xc5100
#define _PCH_GMBUS1			0xc5104
#define _PCH_GMBUS2			0xc5108

#define _GEN6_GDRST			0x941c
#define _GEN6_GT_THREAD_STATUS_REG	0x13805c
#define _GEN6_GT_CORE_STATUS		0x138060

#define _FORCEWAKE_MT			0xa188
#define _FORCEWAKE_ACK_HSW		0x130044
#define _SBI_ADDR			0xC6000
#define _SBI_DATA			0xC6004
#define _SBI_CTL_STAT			0xC6008

#define _RING_IMR(base)			((base) + 0xa8)
#define _SPLL_CTL			0x46020
#define _SFUSE_STRAP			0xc2014
#define _PIXCLK_GATE			0xC6020
#define _ECOBUS				0xa180
#define _GEN6_RC_CONTROL		0xA090
#define _GEN6_RC_STATE			0xA094
#define _GEN6_RPNSWREQ				(0xA008)
#define _GEN6_RC_VIDEO_FREQ			(0xA00C)
#define _GEN6_RP_DOWN_TIMEOUT			(0xA010)
#define _GEN6_RP_INTERRUPT_LIMITS		(0xA014)
#define _GEN6_RPSTAT1				(0xA01C)
#define _GEN6_RP_CONTROL				(0xA024)
#define _GEN6_RP_UP_THRESHOLD			(0xA02C)
#define _GEN6_RP_DOWN_THRESHOLD			(0xA030)
#define _GEN6_RP_CUR_UP_EI			(0xA050)
#define _GEN6_RP_CUR_UP				(0xA054)
#define _GEN6_RP_PREV_UP				(0xA058)
#define _GEN6_RP_CUR_DOWN_EI			(0xA05C)
#define _GEN6_RP_CUR_DOWN			(0xA060)
#define _GEN6_RP_PREV_DOWN			(0xA064)
#define _GEN6_RP_UP_EI				(0xA068)
#define _GEN6_RP_DOWN_EI				(0xA06C)
#define _GEN6_RP_IDLE_HYSTERSIS			(0xA070)
#define _GEN6_RC1_WAKE_RATE_LIMIT		(0xA098)
#define _GEN6_RC6_WAKE_RATE_LIMIT		(0xA09C)
#define _GEN6_RC6pp_WAKE_RATE_LIMIT		(0xA0A0)
#define _GEN6_RC_EVALUATION_INTERVAL		(0xA0A8)
#define _GEN6_RC_IDLE_HYSTERSIS			(0xA0AC)
#define _GEN6_RC_SLEEP				(0xA0B0)
#define _GEN6_RC1e_THRESHOLD			(0xA0B4)
#define _GEN6_RC6_THRESHOLD			(0xA0B8)
#define _GEN6_RC6p_THRESHOLD			(0xA0BC)
#define _GEN6_RC6pp_THRESHOLD			(0xA0C0)
#define _GEN6_PMINTRMSK				(0xA168)
#define _HSW_PWR_WELL_BIOS			(0x45400)
#define _HSW_PWR_WELL_DRIVER			(0x45404)
#define _HSW_PWR_WELL_KVMR			(0x45408)
#define _HSW_PWR_WELL_DEBUG			(0x4540C)
#define _HSW_PWR_WELL_CTL5			(0x45410)
#define _HSW_PWR_WELL_CTL6			(0x45414)
#define _CPU_VGACNTRL	(0x41000)
#define _TILECTL				(0x101000)
#define _GEN6_UCGCTL1				(0x9400)
#define _GEN6_UCGCTL2				(0x9404)
#define _GEN6_PCODE_MAILBOX			(0x138124)
#define _GEN6_PCODE_DATA				(0x138128)
#define _GEN7_ERR_INT	(0x44040)
#define _GFX_FLSH_CNTL_GEN6	(0x101008)
#define _ECOSKPD		(0x21d0)
#define _GEN6_BLITTER_ECOSKPD	(0x221d0)
#define _GAC_ECO_BITS			(0x14090)
#define _GEN6_MBCTL		(0x0907c)
#define _GAB_CTL				(0x24000)
#define _FPGA_DBG		(0x42300)
#define _BCS_SWCTRL (0x22200)
#define _HS_INVOCATION_COUNT             (0x2300)
#define _DS_INVOCATION_COUNT             (0x2308)
#define _IA_VERTICES_COUNT               (0x2310)
#define _IA_PRIMITIVES_COUNT             (0x2318)
#define _VS_INVOCATION_COUNT             (0x2320)
#define _GS_INVOCATION_COUNT             (0x2328)
#define _GS_PRIMITIVES_COUNT             (0x2330)
#define _CL_INVOCATION_COUNT             (0x2338)
#define _CL_PRIMITIVES_COUNT             (0x2340)
#define _PS_INVOCATION_COUNT             (0x2348)
#define _PS_DEPTH_COUNT                  (0x2350)

#define _GEN8_DE_PORT_IMR (0x44444)
#define _GEN8_DE_PORT_IER (0x4444c)
#define _GEN8_DE_PORT_IIR (0x44448)
#define _GEN8_DE_PORT_ISR (0x44440)

#define _GEN8_DE_MISC_IMR (0x44464)
#define _GEN8_DE_MISC_IER (0x4446c)
#define _GEN8_DE_MISC_IIR (0x44468)
#define _GEN8_DE_MISC_ISR (0x44460)

#define _GEN8_PCU_IMR (0x444e4)
#define _GEN8_PCU_IER (0x444ec)
#define _GEN8_PCU_IIR (0x444e8)
#define _GEN8_PCU_ISR (0x444e0)
#define _GEN8_MASTER_IRQ			(0x44200)

#define _RING_HWSTAM(base)	((base)+0x98)
#define _RING_TAIL(base)		((base)+0x30)
#define _RING_HEAD(base)		((base)+0x34)
#define _RING_START(base)	((base)+0x38)
#define _RING_CTL(base)		((base)+0x3c)
#define _RING_ACTHD(base)	((base)+0x74)
#define _RING_MI_MODE(base)	((base)+0x9c)
#define _RING_INSTPM(base)	((base)+0xc0)

#define _GEN6_MBCUNIT_SNPCR	(0x900c)
#define _GEN7_MISCCPCTL				(0x9424)
#define _GEN8_PRIVATE_PAT_LO	(0x40e0)
#define _GEN8_PRIVATE_PAT_HI	(0x40e0 + 4)
#define _GAMTARBMODE		(0x04a08)

#define _SDEIMR  (0xc4004)
#define _SDEIER  (0xc400c)
#define _SDEIIR  (0xc4008)
#define _SDEISR  (0xc4000)

#define _RENDER_HWS_PGA_GEN7	(0x04080)
#define _BSD_HWS_PGA_GEN7	(0x04180)
#define _BLT_HWS_PGA_GEN7	(0x04280)
#define _VEBOX_HWS_PGA_GEN7	(0x04380)
#define _RING_MI_MODE(base)	((base)+0x9c)
#define _GEN7_GT_MODE	(0x7008)
#define _CACHE_MODE_0_GEN7	(0x7000) /* IVB+ */
#define _CACHE_MODE_1		(0x7004) /* IVB+ */
#define _GAM_ECOCHK			(0x4090)
#define _GEN7_COMMON_SLICE_CHICKEN1		(0x7010)
#define _COMMON_SLICE_CHICKEN2			(0x7014)
#define _VGA0	(0x6000)
#define _VGA1	(0x6004)
#define _VGA_PD	(0x6010)
#define _DERRMR		(0x44050)
#define _WM0_PIPEA_ILK		(0x45100)
#define _WM0_PIPEB_ILK		(0x45104)
#define _WM0_PIPEC_IVB		(0x45200)
#define _WM1_LP_ILK		(0x45108)
#define _WM2_LP_ILK		(0x4510c)
#define _WM3_LP_ILK		(0x45110)
#define _WM1S_LP_ILK		(0x45120)
#define _WM2S_LP_IVB		(0x45124)
#define _WM3S_LP_IVB		(0x45128)
#define _BLC_PWM_CPU_CTL2	(0x48250)
#define _BLC_PWM_CPU_CTL		(0x48254)
#define _BLC_PWM_PCH_CTL1	(0xc8250)
#define _BLC_PWM_PCH_CTL2	(0xc8254)
#define _PCH_GPIOA               (0xc5010)
#define _PCH_ADPA                (0xe1100)
#define __3D_CHICKEN3		(0x2090)
#define _PCH_LVDS	(0xe1180)
#define _PCH_DREF_CONTROL        (0xC6200)
#define _PCH_RAWCLK_FREQ         (0xc6204)
#define _PCH_DPLL_SEL		(0xc7000)
#define _PCH_PORT_HOTPLUG		(0xc4030)	/* SHOTPLUG_CTL */
#define _LCPLL_CTL			(0x130040)
#define _FUSE_STRAP			(0x42014)
#define _DIGITAL_PORT_HOTPLUG_CNTRL	(0x44030)
#define _DISP_ARB_CTL	(0x45000)
#define _DISP_ARB_CTL2	(0x45004)
#define _ILK_DISPLAY_CHICKEN1	(0x42000)
#define _ILK_DISPLAY_CHICKEN2	(0x42004)
#define _ILK_DSPCLK_GATE_D			(0x42020)
#define _SOUTH_CHICKEN1		(0xc2000)
#define _SOUTH_CHICKEN2		(0xc2004)
#define _SOUTH_DSPCLK_GATE_D	(0xc2020)
#define _IPS_CTL		(0x43408)

#define _GEN8_GT_ISR(which) (0x44300 + (0x10 * (which)))
#define _GEN8_GT_IMR(which) (0x44304 + (0x10 * (which)))
#define _GEN8_GT_IIR(which) (0x44308 + (0x10 * (which)))
#define _GEN8_GT_IER(which) (0x4430c + (0x10 * (which)))

#define _GEN8_DE_PIPE_ISR(pipe) (0x44400 + (0x10 * (pipe)))
#define _GEN8_DE_PIPE_IMR(pipe) (0x44404 + (0x10 * (pipe)))
#define _GEN8_DE_PIPE_IIR(pipe) (0x44408 + (0x10 * (pipe)))
#define _GEN8_DE_PIPE_IER(pipe) (0x4440c + (0x10 * (pipe)))

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

/* blacklight PWM control */
#define _REG_HISTOGRAM_THRSH	0x48268
#define        _REGBIT_HISTOGRAM_IRQ_ENABLE	(1 << 31)
#define        _REGBIT_HISTOGRAM_IRQ_STATUS	(1 << 30)

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

#define _GVT_TRANS_DDI_FUNC_CTL(tran)   _TRANS(tran, _TRANS_DDI_FUNC_CTL_A, \
		_TRANS_DDI_FUNC_CTL_B)

/* Those bits are ignored by pipe EDP since it can only connect to DDI A */
#define  _TRANS_DDI_MODE_SELECT_HIFT		24
#define  _TRANS_DDI_EDP_INPUT_SHIFT		12

#define _REG_GEN7_SQ_CHICKEN_MBCUNIT_CONFIG		0x9030

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

#endif
