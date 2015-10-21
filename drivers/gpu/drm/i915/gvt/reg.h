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

#define _SDEIMR  (0xc4004)
#define _SDEIER  (0xc400c)
#define _SDEIIR  (0xc4008)
#define _SDEISR  (0xc4000)

#define _GEN8_GT_ISR(which) (0x44300 + (0x10 * (which)))
#define _GEN8_GT_IMR(which) (0x44304 + (0x10 * (which)))
#define _GEN8_GT_IIR(which) (0x44308 + (0x10 * (which)))
#define _GEN8_GT_IER(which) (0x4430c + (0x10 * (which)))

#define _GEN8_DE_PIPE_ISR(pipe) (0x44400 + (0x10 * (pipe)))
#define _GEN8_DE_PIPE_IMR(pipe) (0x44404 + (0x10 * (pipe)))
#define _GEN8_DE_PIPE_IIR(pipe) (0x44408 + (0x10 * (pipe)))
#define _GEN8_DE_PIPE_IER(pipe) (0x4440c + (0x10 * (pipe)))

#define _GVT_TRANS_DDI_FUNC_CTL(tran)   _TRANS(tran, _TRANS_DDI_FUNC_CTL_A, \
		_TRANS_DDI_FUNC_CTL_B)

#endif
