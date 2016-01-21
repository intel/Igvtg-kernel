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

static bool gmbus_mmio_read(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	return gvt_i2c_handle_gmbus_read(vgt, offset, p_data, bytes);
}

static bool gmbus_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	return gvt_i2c_handle_gmbus_write(vgt, offset, p_data, bytes);
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

static bool pch_pp_control_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	u32 data;
	u32 reg;
	union _PCH_PP_CTL pp_control;
	union _PCH_PP_STAUTS pp_status;

	reg = offset & ~(bytes - 1);

	data = *(u32*)p_data;

	__vreg(vgt, _PCH_PP_CONTROL) = data;

	pp_control.data = data;
	pp_status.data = __vreg(vgt, _PCH_PP_STATUS);
	if (pp_control.power_state_target == 1){
		/* power on panel */
		pp_status.panel_powere_on_statue = 1;
		pp_status.power_sequence_progress = 0;
		pp_status.power_cycle_delay_active = 0;
	} else {
		/* power down panel */
		pp_status.panel_powere_on_statue = 0;
		pp_status.power_sequence_progress = 0;
		pp_status.power_cycle_delay_active = 0;
	}
	__vreg(vgt, _PCH_PP_STATUS) = pp_status.data;

	return true;
}

static bool transaconf_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	union _TRANS_CONFIG config;

	config.data = *(u32*)p_data;
	config.transcoder_state = config.transcoder_enable;

	__vreg(vgt, offset) = config.data;
	return true;
}

/*
 * TODO: Check the hotplug bit definitions on BDW+
 */
static bool shotplug_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	u32 val = *(u32 *)p_data;
	u32 sticky_mask = _REGBIT_DP_B_STATUS |
		_REGBIT_DP_C_STATUS |
		_REGBIT_DP_D_STATUS;

	__vreg(vgt, offset) = (val & ~sticky_mask) |
		(__vreg(vgt, offset) & sticky_mask);
	__vreg(vgt, offset) &= ~(val & sticky_mask);

	__sreg(vgt, offset) = val;

	return true;
}

static bool lcpll_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	u32 data = *(u32 *)p_data;

	if (data & LCPLL_PLL_DISABLE)
		data &= ~LCPLL_PLL_LOCK;
	else
		data |= LCPLL_PLL_LOCK;

	if (data & LCPLL_CD_SOURCE_FCLK)
		data |= LCPLL_CD_SOURCE_FCLK_DONE;
	else
		data &= ~LCPLL_CD_SOURCE_FCLK_DONE;

	return gvt_default_mmio_write(vgt, offset, &data, bytes);
}

static bool dpy_reg_mmio_read(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	*(u32*)p_data = (1<<17);

	return true;
}

static bool dpy_reg_mmio_read_2(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	*(u32*)p_data = 3;

	return true;
}

static bool dpy_reg_mmio_read_3(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	*(u32*)p_data = (0x2F << 16);

	return true;
}

static bool ring_mode_write(struct vgt_device *vgt, unsigned int off,
		void *p_data, unsigned int bytes)
{
	return true;
}

static bool pipe_conf_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	u32 wr_data = *((u32 *)p_data);

	/* vreg status will be updated when when read hardware status */
	if (wr_data & PIPECONF_ENABLE)
		wr_data |= I965_PIPECONF_ACTIVE;
	else if (!(wr_data & PIPECONF_ENABLE))
		wr_data &= I965_PIPECONF_ACTIVE;

	if (!gvt_default_mmio_write(vgt, offset, &wr_data, bytes))
		return false;

	return gvt_update_display_events_emulation(vgt->pdev);
}

static bool ddi_buf_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	bool rc;
	u32 reg_val;
	reg_val = *(u32 *)p_data;

	// set the fully virtualized RO bit with its original value
	reg_val = (reg_val & ~_DDI_BUFCTL_DETECT_MASK)
		| (__vreg(vgt, offset) & _DDI_BUFCTL_DETECT_MASK);

	rc = gvt_default_mmio_write(vgt, offset, &reg_val, bytes);

	//update idle status when enable/disable DDI buf
	reg_val = __vreg(vgt, offset);

	if (reg_val & _REGBIT_DDI_BUF_ENABLE)
		reg_val &= ~_REGBIT_DDI_BUF_IS_IDLE;
	else
		reg_val |= _REGBIT_DDI_BUF_IS_IDLE;

	__vreg(vgt, offset) = reg_val;

	// clear the auto_training done bit
	if ((offset == _REG_DDI_BUF_CTL_E) &&
			(!(reg_val & _REGBIT_DDI_BUF_ENABLE))) {
		__vreg(vgt, _REG_DP_TP_STATUS_E) &=
			~DP_TP_STATUS_AUTOTRAIN_DONE;
	}

	return rc;
}

static bool fdi_rx_iir_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	unsigned int reg;
	u32 wr_data, old_iir;
	bool rc;
	reg = offset & ~(bytes -1);

	wr_data = *(u32 *)p_data;
	old_iir = __vreg(vgt, reg);

	rc = gvt_default_mmio_write(vgt, offset, p_data, bytes);

	/* FIXME: sreg will be updated only when reading hardware status happened,
	 * so when dumping sreg space, the "hardware status" related bits may not
	 * be trusted */
	__vreg(vgt, reg) = old_iir ^ wr_data;
	return rc;
}

#define FDI_LINK_TRAIN_PATTERN1		0
#define FDI_LINK_TRAIN_PATTERN2		1

static bool fdi_auto_training_started(struct vgt_device *vgt)
{
	bool rc = false;

	u32 ddi_buf_ctl = __vreg(vgt, _REG_DDI_BUF_CTL_E);
	u32 rx_ctl = __vreg(vgt, _FDI_RXA_CTL);
	u32 tx_ctl = __vreg(vgt, _REG_DP_TP_CTL_E);

	if ((ddi_buf_ctl & _REGBIT_DDI_BUF_ENABLE) &&
			(rx_ctl & FDI_RX_ENABLE) &&
			(rx_ctl & _REGBIT_FDI_RX_FDI_AUTO_TRAIN_ENABLE) &&
			(tx_ctl & DP_TP_CTL_ENABLE) &&
			(tx_ctl & _REGBIT_DP_TP_FDI_AUTO_TRAIN_ENABLE)) {
		rc = true;
	}

	return rc;
}

/* FIXME: this function is highly platform-dependent (SNB + CPT) */
static bool check_fdi_rx_train_status(struct vgt_device *vgt,
		enum pipe pipe, unsigned int train_pattern)
{
	unsigned int fdi_rx_imr, fdi_tx_ctl, fdi_rx_ctl;
	unsigned int fdi_rx_check_bits, fdi_tx_check_bits;
	unsigned int fdi_rx_train_bits, fdi_tx_train_bits;
	unsigned int fdi_iir_check_bits;
	fdi_rx_imr = GVT_FDI_RX_IMR(pipe);
	fdi_tx_ctl = GVT_FDI_TX_CTL(pipe);
	fdi_rx_ctl = GVT_FDI_RX_CTL(pipe);

	if (train_pattern == FDI_LINK_TRAIN_PATTERN1) {
		fdi_rx_train_bits =FDI_LINK_TRAIN_PATTERN_1_CPT;
		fdi_tx_train_bits = FDI_LINK_TRAIN_PATTERN_1;
		fdi_iir_check_bits = _REGBIT_FDI_RX_BIT_LOCK;
	} else if (train_pattern == FDI_LINK_TRAIN_PATTERN2) {
		fdi_rx_train_bits = FDI_LINK_TRAIN_PATTERN_2_CPT;
		fdi_tx_train_bits = FDI_LINK_TRAIN_PATTERN_2;
		fdi_iir_check_bits = _REGBIT_FDI_RX_SYMBOL_LOCK;
	} else {
		BUG();
	}

	fdi_rx_check_bits = FDI_RX_ENABLE | fdi_rx_train_bits;
	fdi_tx_check_bits = _REGBIT_FDI_TX_ENABLE | fdi_tx_train_bits;

	/* If imr bit not been masked */
	if (((__vreg(vgt, fdi_rx_imr) & fdi_iir_check_bits) == 0)
			&& ((__vreg(vgt, fdi_tx_ctl)
					& fdi_tx_check_bits) == fdi_tx_check_bits)
			&& ((__vreg(vgt, fdi_rx_ctl)
					& fdi_rx_check_bits) == fdi_rx_check_bits))
		return true;
	else
		return false;
}

static bool update_fdi_rx_iir_status(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	enum pipe pipe;
	unsigned int reg, fdi_rx_iir;
	bool rc;
	reg = offset & ~(bytes - 1);

	switch (offset) {
		case _FDI_RXA_CTL:
		case _FDI_TXA_CTL:
		case _FDI_RXA_IMR:
			pipe = PIPE_A;
			break;

		case _FDI_RXB_CTL:
		case _FDI_TXB_CTL:
		case _FDI_RXB_IMR:
			pipe = PIPE_B;
			break;

		case _REG_FDI_RXC_CTL:
		case _REG_FDI_TXC_CTL:
		case _REG_FDI_RXC_IMR:
			pipe = PIPE_C;
			break;

		default:
			BUG();
	}

	fdi_rx_iir = GVT_FDI_RX_IIR(pipe);

	rc = gvt_default_mmio_write(vgt, offset, p_data, bytes);
	if (check_fdi_rx_train_status(vgt, pipe, FDI_LINK_TRAIN_PATTERN1))
		__vreg(vgt, fdi_rx_iir) |= _REGBIT_FDI_RX_BIT_LOCK;
	if (check_fdi_rx_train_status(vgt, pipe, FDI_LINK_TRAIN_PATTERN2))
		__vreg(vgt, fdi_rx_iir) |= _REGBIT_FDI_RX_SYMBOL_LOCK;
	if (offset == _FDI_RXA_CTL) {
		if (fdi_auto_training_started(vgt))
			__vreg(vgt, _REG_DP_TP_STATUS_E) |=
				DP_TP_STATUS_AUTOTRAIN_DONE;
	}
	return rc;
}

#define DP_TP_CTL_10_8_MASK	0x00000700
#define DP_TP_CTL_8_SHIFT	0x8
#define DP_TP_STATUS_25_SHIFT	25

static bool dp_tp_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	enum port port;
	unsigned int dp_tp_status_reg, val;
	u32 ctl_val;
	bool rc;
	rc = gvt_default_mmio_write(vgt, offset, p_data, bytes);

	port = DP_TP_PORT(offset);
	ctl_val = __vreg(vgt, offset);
	val = (ctl_val & DP_TP_CTL_10_8_MASK) >> DP_TP_CTL_8_SHIFT;

	if (val == 0x2) {
		dp_tp_status_reg = i915_mmio_reg_offset(DP_TP_STATUS(port));
		__vreg(vgt, dp_tp_status_reg) |= (1 << DP_TP_STATUS_25_SHIFT);
		__sreg(vgt, dp_tp_status_reg) = __vreg(vgt, dp_tp_status_reg);
	}
	return rc;
}

#define BIT_27		27
#define BIT_26		26
#define BIT_24		24

static bool dp_tp_status_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	bool rc = true;
	u32 reg_val;
	u32 sticky_mask;
	reg_val = *((u32 *)p_data);
	sticky_mask = (1 << BIT_27) | (1 << BIT_26) | (1 << BIT_24);

	__vreg(vgt, offset) = (reg_val & ~sticky_mask) |
		(__vreg(vgt, offset) & sticky_mask);
	__vreg(vgt, offset) &= ~(reg_val & sticky_mask);

	__sreg(vgt, offset) = reg_val;

	return rc;
}

static bool pch_adpa_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	u32 old = __vreg(vgt, offset);
	u32 new = *(u32 *)p_data;

	if (new & ADPA_CRT_HOTPLUG_FORCE_TRIGGER) {
		new &= ~(ADPA_CRT_HOTPLUG_FORCE_TRIGGER |
				ADPA_CRT_HOTPLUG_MONITOR_MASK);
	} else {
		/* ignore the status bits in new value
		 * since they are read only actually
		 */
		new = (new & ~ADPA_CRT_HOTPLUG_MONITOR_MASK) |
			(old & ADPA_CRT_HOTPLUG_MONITOR_MASK);
	}

	return gvt_default_mmio_write(vgt, offset, &new, bytes);
}

static bool pri_surf_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	struct gvt_fb_notify_msg msg;
	enum pipe pipe = GVT_DSPSURFPIPE(offset);
	u32 surflive_reg = GVT_DSPSURFLIVE(pipe);

	if (!gvt_default_mmio_write(vgt, offset, p_data, bytes))
		return false;

	/* Update virtual surflive register */
	if (!gvt_default_mmio_write(vgt, surflive_reg, p_data, bytes))
		return false;

	__vreg(vgt, GVT_PIPE_FLIPCOUNT(pipe))++;
	gvt_inject_flip_done(vgt, GVT_DSPSURFPIPE(offset));

	msg.vm_id = vgt->vm_id;
	msg.plane_id = PRIMARY_PLANE;
	msg.pipe_id = GVT_DSPSURFPIPE(offset);
	gvt_fb_notifier_call_chain(FB_DISPLAY_FLIP, &msg);

	return true;
}

static bool spr_surf_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	struct gvt_fb_notify_msg msg;
	enum pipe pipe = GVT_SPRSURFPIPE(offset);
	u32 surflive_reg = GVT_SPRSURFLIVE(pipe);

	if (!gvt_default_mmio_write(vgt, offset, p_data, bytes))
		return false;

	/* Update virtual surflive register */
	if (!gvt_default_mmio_write(vgt, surflive_reg, p_data, bytes))
		return false;

	msg.vm_id = vgt->vm_id;
	msg.plane_id = SPRITE_PLANE;
	msg.pipe_id = GVT_SPRSURFPIPE(offset);
	gvt_fb_notifier_call_chain(FB_DISPLAY_FLIP, &msg);

	return true;
}

static bool south_chicken2_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	u32 data = *(u32 *)p_data;

	if (data & FDI_MPHY_IOSFSB_RESET_CTL)
		data |= FDI_MPHY_IOSFSB_RESET_STATUS;
	else
		data &= ~FDI_MPHY_IOSFSB_RESET_STATUS;

	return gvt_default_mmio_write(vgt, offset, &data, bytes);
}

static void dp_aux_ch_trigger_interrupt_on_done(struct vgt_device *vgt, u32 value,
		unsigned int reg)
{
	enum gvt_event_type event = GVT_EVENT_MAX;
	if (reg == _PCH_DPA_AUX_CH_CTL) {
		event = AUX_CHANNEL_A;
	} else if (reg == _PCH_DPB_AUX_CH_CTL) {
		event = AUX_CHENNEL_B;
	} else if (reg == _PCH_DPC_AUX_CH_CTL) {
		event = AUX_CHENNEL_C;
	} else if (reg == _PCH_DPD_AUX_CH_CTL) {
		event = AUX_CHENNEL_D;
	}

	if (event != GVT_EVENT_MAX && (DP_AUX_CH_CTL_INTERRUPT & value)) {
		gvt_trigger_virtual_event(vgt, event);
	}
}

static void dp_aux_ch_ctl_trans_done(struct vgt_device *vgt, u32 value,
		unsigned int reg, int len, bool data_valid)
{
	/* mark transaction done */
	value |= _REGBIT_DP_AUX_CH_CTL_DONE;
	value &= ~_REGBIT_DP_AUX_CH_CTL_SEND_BUSY;
	value &= ~DP_AUX_CH_CTL_RECEIVE_ERROR;

	if (data_valid) {
		value &= ~DP_AUX_CH_CTL_TIME_OUT_ERROR;
	} else {
		value |= DP_AUX_CH_CTL_TIME_OUT_ERROR;
	}

	/* message size */
	value &= ~(0xf << 20);
	value |= (len << 20);
	__vreg(vgt, reg) = value;

	dp_aux_ch_trigger_interrupt_on_done(vgt, value, reg);
}

static void dp_aux_ch_ctl_link_training(struct gvt_dpcd_data *dpcd, uint8_t t)
{
	if ((t & DPCD_TRAINING_PATTERN_SET_MASK) == DPCD_TRAINING_PATTERN_1) {

		/* training pattern 1 for CR */
		/* set LANE0_CR_DONE, LANE1_CR_DONE */
		dpcd->data[DPCD_LANE0_1_STATUS] |= DPCD_LANES_CR_DONE;
		/* set LANE2_CR_DONE, LANE3_CR_DONE */
		dpcd->data[DPCD_LANE2_3_STATUS] |= DPCD_LANES_CR_DONE;

	} else if ((t & DPCD_TRAINING_PATTERN_SET_MASK) ==
			DPCD_TRAINING_PATTERN_2) {

		/* training pattern 2 for EQ */

		/* Set CHANNEL_EQ_DONE and  SYMBOL_LOCKED for Lane0_1 */
		dpcd->data[DPCD_LANE0_1_STATUS] |= DPCD_LANES_EQ_DONE;
		dpcd->data[DPCD_LANE0_1_STATUS] |= DPCD_SYMBOL_LOCKED;

		/* Set CHANNEL_EQ_DONE and  SYMBOL_LOCKED for Lane2_3 */
		dpcd->data[DPCD_LANE2_3_STATUS] |= DPCD_LANES_EQ_DONE;
		dpcd->data[DPCD_LANE2_3_STATUS] |= DPCD_SYMBOL_LOCKED;
		/* set INTERLANE_ALIGN_DONE */
		dpcd->data[DPCD_LANE_ALIGN_STATUS_UPDATED] |=
			DPCD_INTERLANE_ALIGN_DONE;

	} else if ((t & DPCD_TRAINING_PATTERN_SET_MASK) ==
			DPCD_LINK_TRAINING_DISABLED) {

		/* finish link training */
		/* set sink status as synchronized */
		dpcd->data[DPCD_SINK_STATUS] = DPCD_SINK_IN_SYNC;
	}
}

static bool dp_aux_ch_ctl_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	unsigned int reg = 0;
	u32 value = *(u32 *)p_data;
	int msg, addr, ctrl, op, len;
	struct gvt_dpcd_data *dpcd = NULL;
	enum port port_idx = gvt_get_dp_port_idx(offset);
	struct gt_port *port = NULL;
	ASSERT(bytes == 4);
	ASSERT((offset & (bytes - 1)) == 0);

	reg = offset & ~(bytes - 1);

	gvt_default_mmio_write(vgt, offset, p_data, bytes);

	if (reg != _PCH_DPA_AUX_CH_CTL &&
			reg != _PCH_DPB_AUX_CH_CTL &&
			reg != _PCH_DPC_AUX_CH_CTL &&
			reg != _PCH_DPD_AUX_CH_CTL) {
		/* write to the data registers */
		return true;
	}

	if (!(value & _REGBIT_DP_AUX_CH_CTL_SEND_BUSY)) {
		/* just want to clear the sticky bits */
		__vreg(vgt, reg) = 0;
		return true;
	}

	if (!dpy_is_valid_port(port_idx)) {
		gvt_warn("vGT(%d): Unsupported DP port access!\n",
				vgt->id);
		return true;
	}

	port = gvt_vport(vgt, port_idx);

	if (port) {
		dpcd = port->dpcd;
	}

	/* read out message from DATA1 register */
	msg = __vreg(vgt, reg + 4);
	addr = (msg >> 8) & 0xffff;
	ctrl = (msg >> 24) & 0xff;
	len = msg & 0xff;
	op = ctrl >> 4;

	if (op == GVT_AUX_NATIVE_WRITE) {
		int t;
		uint8_t buf[16];

		if ((addr + len + 1) >= DPCD_SIZE) {
			/*
			 * Write request exceeds what we supported,
			 * DCPD spec: When a Source Device is writing a DPCD
			 * address not supported by the Sink Device, the Sink
			 * Device shall reply with AUX NACK and “M” equal to zero.
			 */

			/* NAK the write */
			__vreg(vgt, reg + 4) = AUX_NATIVE_REPLY_NAK;

			dp_aux_ch_ctl_trans_done(vgt, value, reg, 2, true);

			return true;
		}

		/*
		 * Write request format: (command + address) occupies
		 * 3 bytes, followed by (len + 1) bytes of data.
		 */
		ASSERT((len + 4) <= AUX_BURST_SIZE);

		/* unpack data from vreg to buf */
		for (t = 0; t < 4; t ++) {
			u32 r = __vreg(vgt, reg + 8 + t*4);

			buf[t*4] = (r >> 24) & 0xff;
			buf[t*4 + 1] = (r >> 16) & 0xff;
			buf[t*4 + 2] = (r >> 8) & 0xff;
			buf[t*4 + 3] = r & 0xff;
		}

		/* write to virtual DPCD */
		if (dpcd && dpcd->data_valid) {
			for (t = 0; t <= len; t ++) {
				int p = addr + t;

				dpcd->data[p] = buf[t];

				/* check for link training */
				if (p == DPCD_TRAINING_PATTERN_SET)
					dp_aux_ch_ctl_link_training(dpcd, buf[t]);
			}
		}

		/* ACK the write */
		__vreg(vgt, reg + 4) = 0;

		dp_aux_ch_ctl_trans_done(vgt, value, reg, 1, dpcd && dpcd->data_valid);

		return true;
	}

	if (op == GVT_AUX_NATIVE_READ) {
		int idx, i, ret = 0;

		if ((addr + len + 1) >= DPCD_SIZE) {
			/*
			 * read request exceeds what we supported
			 * DPCD spec: A Sink Device receiving a Native AUX CH
			 * read request for an unsupported DPCD address must
			 * reply with an AUX ACK and read data set equal to
			 * zero instead of replying with AUX NACK.
			 */

			/* ACK the READ*/
			__vreg(vgt, reg + 4) = 0;
			__vreg(vgt, reg + 8) = 0;
			__vreg(vgt, reg + 12) = 0;
			__vreg(vgt, reg + 16) = 0;
			__vreg(vgt, reg + 20) = 0;

			dp_aux_ch_ctl_trans_done(vgt ,value, reg, len + 2, true);

			return true;
		}

		for (idx = 1; idx <= 5; idx ++) {
			/* clear the data registers */
			__vreg(vgt, reg + 4 * idx) = 0;
		}

		/*
		 * Read reply format: ACK (1 byte) plus (len + 1) bytes of data.
		 */
		ASSERT((len + 2) <= AUX_BURST_SIZE);

		/* read from virtual DPCD to vreg */
		/* first 4 bytes: [ACK][addr][addr+1][addr+2] */
		if (dpcd && dpcd->data_valid) {
			for (i = 1; i <= (len + 1); i ++) {
				int t;

				t = dpcd->data[addr + i - 1];
				t <<= (24 - 8*(i%4));
				ret |= t;

				if ((i%4 == 3) || (i == (len + 1))) {
					__vreg(vgt, reg + (i/4 + 1)*4) = ret;
					ret = 0;
				}
			}
		}

		dp_aux_ch_ctl_trans_done(vgt, value, reg, len + 2, dpcd && dpcd->data_valid);

		return true;
	}

	/* i2c transaction starts */
	gvt_i2c_handle_aux_ch_write(vgt, port_idx, offset, p_data);

	dp_aux_ch_trigger_interrupt_on_done(vgt, value, reg);
	return true;
}

static void gvt_dpy_stat_notify(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;

	gvt_set_dpy_uevent(vgt);
	gvt_raise_request(pdev, GVT_REQUEST_UEVENT);
}

static bool vga_control_w(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	bool vga_disable;
	gvt_default_mmio_write(vgt, offset, p_data, bytes);

	vga_disable = __vreg(vgt, offset) & _REGBIT_VGA_DISPLAY_DISABLE;

	gvt_info("VM(%d): %s VGA mode %x\n", vgt->vm_id,
			vga_disable ? "Disable" : "Enable",
			(unsigned int)__vreg(vgt, offset));
	return true;
}

static u32 get_sbi_reg_cached_value(struct vgt_device *vgt,
		unsigned int sbi_offset)
{
	int i;
	int num = vgt->state.display.sbi_regs.number;
	u32 value = 0;
	for (i = 0; i < num; ++ i) {
		if (vgt->state.display.sbi_regs.registers[i].offset == sbi_offset)
			break;
	}

	if (i < num) {
		value = vgt->state.display.sbi_regs.registers[i].value;
	} else {
		gvt_warn("vGT(%d): SBI reading did not find the cached value"
				" for offset 0x%x. 0 will be returned!\n",
				vgt->id, sbi_offset);
	}
	return value;
}

static void cache_sbi_reg_value(struct vgt_device *vgt, unsigned int sbi_offset,
		u32 value)
{
	int i;
	int num = vgt->state.display.sbi_regs.number;
	for (i = 0; i < num; ++ i) {
		if (vgt->state.display.sbi_regs.registers[i].offset == sbi_offset)
			break;
	}

	if (i == num) {
		if (num < SBI_REG_MAX) {
			vgt->state.display.sbi_regs.number ++;
		} else {
			gvt_warn("vGT(%d): SBI caching meets maximum limits!\n",
					vgt->id);
			return;
		}
	}

	vgt->state.display.sbi_regs.registers[i].offset = sbi_offset;
	vgt->state.display.sbi_regs.registers[i].value = value;
}

static bool sbi_mmio_data_read(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	bool rc = 0;
	rc = gvt_default_mmio_read(vgt, offset, p_data, bytes);

	if (((__vreg(vgt, _SBI_CTL_STAT) & SBI_OPCODE_MASK) >>
				SBI_OPCODE_SHIFT) == SBI_CMD_CRRD) {
		unsigned int sbi_offset = (__vreg(vgt, _SBI_ADDR) &
				SBI_ADDR_OFFSET_MASK) >> SBI_ADDR_OFFSET_SHIFT;
		u32 val = get_sbi_reg_cached_value(vgt, sbi_offset);
		*(u32 *)p_data = val;
	}
	return rc;
}

static bool sbi_mmio_ctl_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	bool rc = 0;
	u32 data;
	rc = gvt_default_mmio_write(vgt, offset, p_data, bytes);

	data = __vreg(vgt, offset);

	data &= ~(SBI_STAT_MASK << SBI_STAT_SHIFT);
	data |= SBI_READY;

	data &= ~(SBI_RESPONSE_MASK << SBI_RESPONSE_SHIFT);
	data |= SBI_RESPONSE_SUCCESS;

	__vreg(vgt, offset) = data;

	if (((__vreg(vgt, _SBI_CTL_STAT) & SBI_OPCODE_MASK) >>
				SBI_OPCODE_SHIFT) == SBI_CMD_CRWR) {
		unsigned int sbi_offset = (__vreg(vgt, _SBI_ADDR) &
				SBI_ADDR_OFFSET_MASK) >> SBI_ADDR_OFFSET_SHIFT;
		u32 val = __vreg(vgt, _SBI_DATA);
		cache_sbi_reg_value(vgt, sbi_offset, val);
	}

	return rc;
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
	bool invalid_event = false;

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
		case _vgtif_reg(display_ready):
			switch (val) {
				case 0:
				case 1:
				case 2:
					break;
				default:
					invalid_event = true;
					gvt_warn("invalid display event: %d\n", val);
					break;
			}

			if (!invalid_event)
				gvt_dpy_stat_notify(vgt);

				break;
		case _vgtif_reg(g2v_notify):
				if (val == VGT_G2V_SET_POINTER_SHAPE) {
					struct gvt_fb_notify_msg msg;
					msg.vm_id = vgt->vm_id;
					msg.plane_id = CURSOR_PLANE;
					msg.pipe_id = 0;
					gvt_fb_notifier_call_chain(FB_DISPLAY_FLIP, &msg);
				} else if (val == VGT_G2V_PPGTT_L3_PAGE_TABLE_CREATE) {
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
		case _vgtif_reg(xhot):
		case _vgtif_reg(yhot):
			{
				struct gvt_fb_notify_msg msg;
				msg.vm_id = vgt->vm_id;
				msg.plane_id = CURSOR_PLANE;
				msg.pipe_id = 0;
				gvt_fb_notifier_call_chain(FB_DISPLAY_FLIP, &msg);
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

static bool power_well_ctl_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	bool rc = true;
	u32 value = *(u32 *)p_data;
	memcpy ((char *)vgt->state.mmio.vreg + offset, p_data, bytes);

	if (value & HSW_PWR_WELL_ENABLE_REQUEST) {
		__vreg(vgt, offset) |= HSW_PWR_WELL_STATE_ENABLED;
	} else {
		__vreg(vgt, offset) &= ~HSW_PWR_WELL_STATE_ENABLED;
	}

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

	{GVT_PIPEDSL(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_PIPECONF(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, pipe_conf_mmio_write},
	{GVT_PIPESTAT(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_PIPE_FRMCOUNT(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_PIPE_FLIPCOUNT(PIPE_A), 4, F_VIRT, 0, D_ALL, NULL, NULL},

	{GVT_PIPEDSL(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_PIPECONF(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, pipe_conf_mmio_write},
	{GVT_PIPESTAT(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_PIPE_FRMCOUNT(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_PIPE_FLIPCOUNT(PIPE_B), 4, F_VIRT, 0, D_ALL, NULL, NULL},

	{GVT_PIPEDSL(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_PIPECONF(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, pipe_conf_mmio_write},
	{GVT_PIPESTAT(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_PIPE_FRMCOUNT(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_PIPE_FLIPCOUNT(PIPE_C), 4, F_VIRT, 0, D_ALL, NULL, NULL},

	{_REG_PIPE_EDP_CONF, 4, F_DPY, 0, D_ALL, NULL, pipe_conf_mmio_write},

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

	{GVT_DSPCNTR(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_DSPSURF(PIPE_A), 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, NULL,
		pri_surf_mmio_write},
	{GVT_DSPSURFLIVE(PIPE_A), 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, NULL,
		mmio_not_allow_write},
	{GVT_DSPPOS(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_DSPLINOFF(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_DSPSTRIDE(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_DSPSIZE(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_DSPTILEOFF(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},

	{GVT_DSPCNTR(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_DSPSURF(PIPE_B), 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, NULL,
	    pri_surf_mmio_write},
	{GVT_DSPSURFLIVE(PIPE_B), 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, NULL,
	    mmio_not_allow_write},
	{GVT_DSPPOS(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_DSPLINOFF(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_DSPSTRIDE(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_DSPSIZE(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_DSPTILEOFF(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},

	{GVT_DSPCNTR(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_DSPSURF(PIPE_C), 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL, NULL,
	    pri_surf_mmio_write},
	{GVT_DSPSURFLIVE(PIPE_C), 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL, NULL,
	    mmio_not_allow_write},
	{GVT_DSPPOS(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_DSPLINOFF(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_DSPSTRIDE(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_DSPSIZE(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},
	{GVT_DSPTILEOFF(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},

	{GVT_SPRSURF(PIPE_A), 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL,
		NULL, spr_surf_mmio_write},
	{GVT_SPRSURFLIVE(PIPE_A), 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL,
		NULL, mmio_not_allow_write},
	{GVT_SPRCNTR(PIPE_A), 4, F_DPY, 0, D_ALL, NULL, NULL},

	{GVT_SPRSURF(PIPE_B), 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL,
		NULL, spr_surf_mmio_write},
	{GVT_SPRSURFLIVE(PIPE_B), 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL,
		NULL, mmio_not_allow_write},
	{GVT_SPRCNTR(PIPE_B), 4, F_DPY, 0, D_ALL, NULL, NULL},

	{GVT_SPRSURF(PIPE_C), 4, F_DPY_ADRFIX, 0xFFFFF000, D_ALL,
		NULL, spr_surf_mmio_write},
	{GVT_SPRSURFLIVE(PIPE_C), 4, F_DPY_HWSTS_ADRFIX, 0xFFFFF000, D_ALL,
		NULL, mmio_not_allow_write},
	{GVT_SPRCNTR(PIPE_C), 4, F_DPY, 0, D_ALL, NULL, NULL},

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

	{_PCH_GMBUS0, 4*4, F_DPY, 0, D_ALL, gmbus_mmio_read, gmbus_mmio_write},
	{_PCH_GPIOA, 6*4, F_VIRT, 0, D_ALL, NULL, NULL},

	{_REG_DP_BUFTRANS, 0x28, F_DPY, 0, D_ALL, NULL, NULL},

	{_PCH_DPB_AUX_CH_CTL, 6*4, F_DPY, 0, D_ALL, NULL, dp_aux_ch_ctl_mmio_write},
	{_PCH_DPC_AUX_CH_CTL, 6*4, F_DPY, 0, D_ALL, NULL, dp_aux_ch_ctl_mmio_write},
	{_PCH_DPD_AUX_CH_CTL, 6*4, F_DPY, 0, D_ALL, NULL, dp_aux_ch_ctl_mmio_write},

	{_PCH_ADPA, 4, F_DPY, 0, D_ALL, NULL, pch_adpa_mmio_write},
	{_PCH_TRANSACONF, 4, F_DPY, 0, D_ALL, NULL, transaconf_mmio_write},
	{_PCH_TRANSBCONF, 4, F_DPY, 0, D_ALL, NULL, transaconf_mmio_write},
	{_FDI_RXA_IIR, 4, F_DPY, 0, D_ALL, NULL, fdi_rx_iir_mmio_write},
	{_FDI_RXB_IIR, 4, F_DPY, 0, D_ALL, NULL, fdi_rx_iir_mmio_write},
	{_REG_FDI_RXC_IIR, 4, F_DPY, 0, D_ALL, NULL, fdi_rx_iir_mmio_write},
	{_FDI_RXA_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
	{_FDI_RXB_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
	{_REG_FDI_RXC_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
	{_FDI_TXA_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
	{_FDI_TXB_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
	{_REG_FDI_TXC_CTL, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
	{_FDI_RXA_IMR, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
	{_FDI_RXB_IMR, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},
	{_REG_FDI_RXC_IMR, 4, F_DPY, 0, D_ALL, NULL, update_fdi_rx_iir_status},

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

	{_PCH_PP_CONTROL, 4, F_DPY, 0, D_ALL, NULL, pch_pp_control_mmio_write},
	{_PCH_PP_DIVISOR, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_PP_STATUS, 4, F_DPY, 0, D_ALL, NULL, NULL},
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

	{0xE651C, 4, F_DPY, 0, D_ALL, dpy_reg_mmio_read, NULL},
	{0xE661C, 4, F_DPY, 0, D_ALL, dpy_reg_mmio_read, NULL},
	{0xE671C, 4, F_DPY, 0, D_ALL, dpy_reg_mmio_read, NULL},
	{0xE681C, 4, F_DPY, 0, D_ALL, dpy_reg_mmio_read, NULL},
	{0xE6C04, 4, F_DPY, 0, D_ALL,
		dpy_reg_mmio_read_2, NULL},
	{0xE6E1C, 4, F_DPY, 0, D_ALL,
		dpy_reg_mmio_read_3, NULL},
	{_PCH_PORT_HOTPLUG, 4, F_VIRT, 0, D_ALL, NULL, shotplug_ctl_mmio_write},
	{_LCPLL_CTL, 4, F_DPY, 0, D_ALL, NULL, lcpll_ctl_mmio_write},
	{_FUSE_STRAP, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_DIGITAL_PORT_HOTPLUG_CNTRL, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_DISP_ARB_CTL, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_DISP_ARB_CTL2, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_ILK_DISPLAY_CHICKEN1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_ILK_DISPLAY_CHICKEN2, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_ILK_DSPCLK_GATE_D, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_SOUTH_CHICKEN1, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_SOUTH_CHICKEN2, 4, F_DPY, 0, D_ALL, NULL, south_chicken2_write},
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
	{_SBI_DATA, 4, F_DPY, 0, D_ALL, sbi_mmio_data_read, NULL},
	{_SBI_CTL_STAT, 4, F_DPY, 0, D_ALL, NULL, sbi_mmio_ctl_write},
	{_PIXCLK_GATE, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_PCH_DPA_AUX_CH_CTL, 6*4, F_DPY, 0, D_ALL, NULL, dp_aux_ch_ctl_mmio_write},

	{_DDI_BUF_CTL_A, 4, F_DPY, 0, D_ALL, NULL, ddi_buf_ctl_mmio_write},
	{_DDI_BUF_CTL_B, 4, F_DPY, 0, D_ALL, NULL, ddi_buf_ctl_mmio_write},
	{_REG_DDI_BUF_CTL_C, 4, F_DPY, 0, D_ALL, NULL, ddi_buf_ctl_mmio_write},
	{_REG_DDI_BUF_CTL_D, 4, F_DPY, 0, D_ALL, NULL, ddi_buf_ctl_mmio_write},
	{_REG_DDI_BUF_CTL_E, 4, F_DPY, 0, D_ALL, NULL, ddi_buf_ctl_mmio_write},

	{_DP_TP_CTL_A, 4, F_DPY, 0, D_ALL, NULL, dp_tp_ctl_mmio_write},
	{_DP_TP_CTL_B, 4, F_DPY, 0, D_ALL, NULL, dp_tp_ctl_mmio_write},
	{_REG_DP_TP_CTL_C, 4, F_DPY, 0, D_ALL, NULL, dp_tp_ctl_mmio_write},
	{_REG_DP_TP_CTL_D, 4, F_DPY, 0, D_ALL, NULL, dp_tp_ctl_mmio_write},
	{_REG_DP_TP_CTL_E, 4, F_DPY, 0, D_ALL, NULL, NULL},

	{_DP_TP_STATUS_A, 4, F_DPY, 0, D_ALL, NULL, dp_tp_status_mmio_write},
	{_DP_TP_STATUS_B, 4, F_DPY, 0, D_ALL, NULL, dp_tp_status_mmio_write},
	{_REG_DP_TP_STATUS_C, 4, F_DPY, 0, D_ALL, NULL, dp_tp_status_mmio_write},
	{_REG_DP_TP_STATUS_D, 4, F_DPY, 0, D_ALL, NULL, dp_tp_status_mmio_write},
	{_REG_DP_TP_STATUS_E, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_DDI_BUF_TRANS_A, 0x50, F_DPY, 0, D_ALL, NULL, NULL},
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
	{_HSW_PWR_WELL_BIOS, 4, F_DOM0, 0, D_ALL, NULL, power_well_ctl_write},
	{_HSW_PWR_WELL_DRIVER, 4, F_DOM0, 0, D_ALL, NULL, power_well_ctl_write},
	{_HSW_PWR_WELL_KVMR, 4, F_DOM0, 0, D_ALL, NULL, power_well_ctl_write},
	{_HSW_PWR_WELL_DEBUG, 4, F_DOM0, 0, D_ALL, NULL, power_well_ctl_write},
	{_HSW_PWR_WELL_CTL5, 4, F_DOM0, 0, D_ALL, NULL, power_well_ctl_write},
	{_HSW_PWR_WELL_CTL6, 4, F_DOM0, 0, D_ALL, NULL, power_well_ctl_write},

	{_GEN6_GDRST, 4, F_DOM0, 0, D_ALL, NULL, gdrst_mmio_write},
	{0x100000, 0x80, F_VIRT, 0, D_ALL, fence_mmio_read, fence_mmio_write},
	{VGT_PVINFO_PAGE, VGT_PVINFO_SIZE, F_VIRT, 0, D_ALL, pvinfo_read, pvinfo_write},
	{_CPU_VGACNTRL, 4, F_DOM0, 0, D_ALL, NULL, vga_control_w},

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
	/* FIXME: now looks gmbus handler can't cover 4/5 ports */
	{_PCH_GMBUS4, 4, F_DPY, 0, D_ALL, NULL, NULL},
	{_PCH_GMBUS5, 4, F_DPY, 0, D_ALL, NULL, NULL},

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
