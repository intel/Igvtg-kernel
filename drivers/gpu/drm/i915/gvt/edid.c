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

static unsigned char edid_get_byte(struct vgt_device *vgt)
{
	unsigned char chr = 0;
	struct gvt_i2c_edid_t *edid = &vgt->state.display.gvt_i2c_edid;

	if (edid->state == I2C_NOT_SPECIFIED || !edid->slave_selected) {
		gvt_warn("Driver tries to read EDID without proper sequence!\n");
		return 0;
	}
	if (edid->current_edid_read >= EDID_SIZE) {
		gvt_warn("edid_get_byte() exceeds the size of EDID!\n");
		return 0;
	}

	if (!edid->edid_available) {
		gvt_warn("Reading EDID but EDID is not available!"
			" Will return 0.\n");
		return 0;
	}

	if (dpy_has_monitor_on_port(vgt, edid->port)) {
		struct gvt_edid_data_t *edid_data = gvt_vport(vgt, edid->port)->edid;
		chr = edid_data->edid_block[edid->current_edid_read];
		gvt_dbg(GVT_DBG_EDID,
			"edid_get_byte with offset %d and value %d\n",
			edid->current_edid_read, chr);
		edid->current_edid_read ++;
	} else {
		gvt_warn("No EDID available during the reading?\n");
	}

	return chr;
}

static inline enum port gvt_get_port_from_gmbus0(u32 gmbus0){
	enum port port = I915_MAX_PORTS;
	int port_select = gmbus0 & _GMBUS_PIN_SEL_MASK;

	if (port_select == 2)
		port = PORT_E;
	else if (port_select == 4)
		port = PORT_C;
	else if (port_select == 5)
		port = PORT_B;
	else if (port_select == 6)
		port = PORT_D;

	return port;
}

void gvt_reset_gmbus_controller(struct vgt_device *vgt)
{
	__vreg(vgt, _PCH_GMBUS2) = GMBUS_HW_RDY;
	if (!vgt->state.display.gvt_i2c_edid.edid_available) {
		__vreg(vgt, _PCH_GMBUS2) |= GMBUS_SATOER;
	}
	vgt->state.display.gvt_i2c_edid.gmbus.phase = GMBUS_IDLE_PHASE;
}


/* GMBUS0 */
static bool gvt_gmbus0_mmio_write(struct vgt_device *vgt,
			unsigned int offset, void *p_data, unsigned int bytes)
{
	u32 wvalue = *(u32 *)p_data;
	enum port port = I915_MAX_PORTS;
	int pin_select = wvalue & _GMBUS_PIN_SEL_MASK;

	gvt_init_i2c_edid(vgt);

	if (pin_select == 0)
		return true;

	vgt->state.display.gvt_i2c_edid.state = I2C_GMBUS;
	port = gvt_get_port_from_gmbus0(pin_select);
	if (!dpy_is_valid_port(port)) {
		gvt_dbg(GVT_DBG_EDID,
			"VM(%d): Driver tries GMBUS write not on valid port!\n"
			"gmbus write value is: 0x%x\n", vgt->id, wvalue);
		return true;
	}

	vgt->state.display.gvt_i2c_edid.gmbus.phase = GMBUS_IDLE_PHASE;

	/* FIXME: never clear GMBUS_HW_WAIT_PHASE */
	__vreg(vgt, _PCH_GMBUS2) &= ~ GMBUS_ACTIVE;
	__vreg(vgt, _PCH_GMBUS2) |= GMBUS_HW_RDY | GMBUS_HW_WAIT_PHASE;

	if (dpy_has_monitor_on_port(vgt, port) && !dpy_port_is_dp(vgt, port)) {
		vgt->state.display.gvt_i2c_edid.port = port;
		vgt->state.display.gvt_i2c_edid.edid_available = true;
		__vreg(vgt, _PCH_GMBUS2) &= ~GMBUS_SATOER;
	} else {
		__vreg(vgt, _PCH_GMBUS2) |= GMBUS_SATOER;
	}

	memcpy(p_data, (char *)vgt->state.mmio.vreg + offset, bytes);
	return true;
}

static bool gvt_gmbus1_mmio_write(struct vgt_device *vgt, unsigned int offset,
void *p_data, unsigned int bytes)
{
	u32 slave_addr;
	struct gvt_i2c_edid_t *i2c_edid = &vgt->state.display.gvt_i2c_edid;

	u32 wvalue = *(u32 *)p_data;
	if (__vreg(vgt, offset) & GMBUS_SW_CLR_INT) {
		if (!(wvalue & GMBUS_SW_CLR_INT)) {
			__vreg(vgt, offset) &= ~GMBUS_SW_CLR_INT;
			gvt_reset_gmbus_controller(vgt);
		}
		/* TODO: "This bit is cleared to zero when an event
		 * causes the HW_RDY bit transition to occur "*/
	} else {
		/* per bspec setting this bit can cause:
		 1) INT status bit cleared
		 2) HW_RDY bit asserted
		 */
		if (wvalue & GMBUS_SW_CLR_INT) {
			__vreg(vgt, _PCH_GMBUS2) &= ~GMBUS_INT;
			__vreg(vgt, _PCH_GMBUS2) |= GMBUS_HW_RDY;
		}

		/* For virtualization, we suppose that HW is always ready,
		 * so GMBUS_SW_RDY should always be cleared
		 */
		if (wvalue & GMBUS_SW_RDY)
			wvalue &= ~GMBUS_SW_RDY;

		i2c_edid->gmbus.total_byte_count =
			gmbus1_total_byte_count(wvalue);
		slave_addr = gmbus1_slave_addr(wvalue);

		/* vgt gmbus only support EDID */
		if (slave_addr == EDID_ADDR) {
			i2c_edid->slave_selected = true;
		} else if (slave_addr != 0) {
			gvt_dbg(GVT_DBG_DPY,
				"vGT(%d): unsupported gmbus slave addr(0x%x)\n"
				"	gmbus operations will be ignored.\n",
					vgt->id, slave_addr);
		}

		if (wvalue & GMBUS_CYCLE_INDEX) {
			i2c_edid->current_edid_read = gmbus1_slave_index(wvalue);
		}

		i2c_edid->gmbus.cycle_type = gmbus1_bus_cycle(wvalue);
		switch (gmbus1_bus_cycle(wvalue)) {
			case GMBUS_NOCYCLE:
				break;
			case GMBUS_STOP:
				/* From spec:
				This can only cause a STOP to be generated
				if a GMBUS cycle is generated, the GMBUS is
				currently in a data/wait/idle phase, or it is in a
				WAIT phase
				 */
				if (gmbus1_bus_cycle(__vreg(vgt, offset)) != GMBUS_NOCYCLE) {
					gvt_init_i2c_edid(vgt);
					/* After the 'stop' cycle, hw state would become
					 * 'stop phase' and then 'idle phase' after a few
					 * milliseconds. In emulation, we just set it as
					 * 'idle phase' ('stop phase' is not
					 * visible in gmbus interface)
					 */
					i2c_edid->gmbus.phase = GMBUS_IDLE_PHASE;
					/*
					FIXME: never clear GMBUS_WAIT
					__vreg(vgt, _PCH_GMBUS2) &=
						~(GMBUS_ACTIVE | GMBUS_HW_WAIT_PHASE);
					*/
					__vreg(vgt, _PCH_GMBUS2) &= ~GMBUS_ACTIVE;
				}
				break;
			case NIDX_NS_W:
			case IDX_NS_W:
			case NIDX_STOP:
			case IDX_STOP:
				/* From hw spec the GMBUS phase
				 * transition like this:
				 * START (-->INDEX) -->DATA
				 */
				i2c_edid->gmbus.phase = GMBUS_DATA_PHASE;
				__vreg(vgt, _PCH_GMBUS2) |= GMBUS_ACTIVE;
				/* FIXME: never clear GMBUS_WAIT */
				//__vreg(vgt, _PCH_GMBUS2) &= ~GMBUS_HW_WAIT_PHASE;
				break;
			default:
				gvt_err("Unknown/reserved GMBUS cycle detected!");
				break;
		}
		/* From hw spec the WAIT state will be
		 * cleared:
		 * (1) in a new GMBUS cycle
		 * (2) by generating a stop
		 */
		/* FIXME: never clear GMBUS_WAIT
		if (gmbus1_bus_cycle(wvalue) != GMBUS_NOCYCLE)
			__vreg(vgt, _PCH_GMBUS2) &= ~GMBUS_HW_WAIT_PHASE;
		*/

		__vreg(vgt, offset) = wvalue;
	}
	return true;
}

bool gvt_gmbus3_mmio_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	ASSERT_VM(0, vgt);
	return true;
}

bool gvt_gmbus3_mmio_read(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	int i;
	unsigned char byte_data;
	struct gvt_i2c_edid_t *i2c_edid = &vgt->state.display.gvt_i2c_edid;
	int byte_left = i2c_edid->gmbus.total_byte_count -
				i2c_edid->current_edid_read;
	int byte_count = byte_left;
	u32 reg_data = 0;

	/* Data can only be recevied if previous settings correct */
	if (__vreg(vgt, _PCH_GMBUS1) & GMBUS_SLAVE_READ) {
		if (byte_left <= 0) {
			memcpy((char *)p_data, (char *)vgt->state.mmio.vreg + offset, bytes);
			return true;
		}

		if (byte_count > 4)
			byte_count = 4;
		for (i = 0; i< byte_count; i++) {
			byte_data = edid_get_byte(vgt);
			reg_data |= (byte_data << (i << 3));
		}

		memcpy((char *)p_data, (char *)&reg_data, byte_count);
		memcpy((char *)vgt->state.mmio.vreg + offset, (char *)&reg_data, byte_count);

		if (byte_left <= 4) {
			switch (i2c_edid->gmbus.cycle_type) {
				case NIDX_STOP:
				case IDX_STOP:
					i2c_edid->gmbus.phase = GMBUS_IDLE_PHASE;
					break;
				case NIDX_NS_W:
				case IDX_NS_W:
				default:
					i2c_edid->gmbus.phase = GMBUS_WAIT_PHASE;
					break;
			}
			gvt_init_i2c_edid(vgt);
		}

		/* Read GMBUS3 during send operation, return the latest written value */
	} else {
		memcpy((char *)p_data, (char *)vgt->state.mmio.vreg + offset, bytes);
		printk("vGT(%d): warning: gmbus3 read with nothing retuned\n",
				vgt->id);
	}

	return true;
}

static bool gvt_gmbus2_mmio_read(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	u32 value = __vreg(vgt, offset);
	if (!(__vreg(vgt, offset) & GMBUS_INUSE)) {
		__vreg(vgt, offset) |= GMBUS_INUSE;
	}

	memcpy(p_data, (void *)&value, bytes);
	return true;
}

bool gvt_gmbus2_mmio_write(struct vgt_device *vgt, unsigned int offset,
		void *p_data, unsigned int bytes)
{
	u32 wvalue = *(u32 *)p_data;
	if (wvalue & GMBUS_INUSE)
		__vreg(vgt, offset) &= ~GMBUS_INUSE;
	/* All other bits are read-only */
	return true;
}

bool gvt_i2c_handle_gmbus_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	ASSERT(bytes <= 8 && !(offset & (bytes - 1)));
	switch (offset) {
		case _PCH_GMBUS2:
			return gvt_gmbus2_mmio_read(vgt, offset, p_data, bytes);
		case _PCH_GMBUS3:
			return gvt_gmbus3_mmio_read(vgt, offset, p_data, bytes);
		default:
			memcpy(p_data, (char *)vgt->state.mmio.vreg + offset, bytes);
	}
	return true;
}

bool gvt_i2c_handle_gmbus_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, unsigned int bytes)
{
	ASSERT(bytes <= 8 && !(offset & (bytes - 1)));
	switch (offset) {
		case _PCH_GMBUS0:
			return gvt_gmbus0_mmio_write(vgt, offset, p_data, bytes);
		case _PCH_GMBUS1:
			return gvt_gmbus1_mmio_write(vgt, offset, p_data, bytes);
		case _PCH_GMBUS2:
			return gvt_gmbus2_mmio_write(vgt, offset, p_data, bytes);
		/* TODO: */
		case _PCH_GMBUS3:
			BUG();
			return false;
		default:
			memcpy((char *)vgt->state.mmio.vreg + offset, p_data, bytes);
	}
	return true;
}

static inline AUX_CH_REGISTERS gvt_get_aux_ch_reg(unsigned int offset)
{
	AUX_CH_REGISTERS reg;
	switch (offset & 0xff) {
	case 0x10:
		reg = AUX_CH_CTL;
		break;
	case 0x14:
		reg = AUX_CH_DATA1;
		break;
	case 0x18:
		reg = AUX_CH_DATA2;
		break;
	case 0x1c:
		reg = AUX_CH_DATA3;
		break;
	case 0x20:
		reg = AUX_CH_DATA4;
		break;
	case 0x24:
		reg = AUX_CH_DATA5;
		break;
	default:
		reg = AUX_CH_INV;
		break;
	}
	return reg;
}

#define AUX_CTL_MSG_LENGTH(reg) \
	((reg & _DP_AUX_CH_CTL_MESSAGE_SIZE_MASK) >> \
		_DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT)

void gvt_i2c_handle_aux_ch_write(struct vgt_device *vgt,
				enum port port_idx,
				unsigned int offset,
				void *p_data)
{
	struct gvt_i2c_edid_t *i2c_edid = &vgt->state.display.gvt_i2c_edid;
	int msg_length, ret_msg_size;
	int msg, addr, ctrl, op;
	int value = *(int *)p_data;
	int aux_data_for_write = 0;
	AUX_CH_REGISTERS reg = gvt_get_aux_ch_reg(offset);

	if (reg != AUX_CH_CTL) {
		__vreg(vgt, offset) = value;
		return;
	}

	msg_length = AUX_CTL_MSG_LENGTH(value);
	// check the msg in DATA register.
	msg = __vreg(vgt, offset + 4);
	addr = (msg >> 8) & 0xffff;
	ctrl = (msg >> 24)& 0xff;
	op = ctrl >> 4;
	if (!(value & _REGBIT_DP_AUX_CH_CTL_SEND_BUSY)) {
		/* The ctl write to clear some states */
		return;
	}

	/* Always set the wanted value for vms. */
	ret_msg_size = (((op & 0x1) == GVT_AUX_I2C_READ) ? 2 : 1);
	__vreg(vgt, offset) =
		_REGBIT_DP_AUX_CH_CTL_DONE |
		((ret_msg_size << _DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT) &
		_DP_AUX_CH_CTL_MESSAGE_SIZE_MASK);

	if (msg_length == 3) {
		if (!(op & GVT_AUX_I2C_MOT)) {
			/* stop */
			gvt_dbg(GVT_DBG_EDID,
				"AUX_CH: stop. reset I2C!\n");
			gvt_init_i2c_edid(vgt);
		} else {
			/* start or restart */
			gvt_dbg(GVT_DBG_EDID,
				"AUX_CH: start or restart I2C!\n");
			i2c_edid->aux_ch.i2c_over_aux_ch = true;
			i2c_edid->aux_ch.aux_ch_mot = true;
			if (addr == 0) {
				/* reset the address */
				gvt_dbg(GVT_DBG_EDID,
					"AUX_CH: reset I2C!\n");
				gvt_init_i2c_edid(vgt);
			} else if (addr == EDID_ADDR) {
				gvt_dbg(GVT_DBG_EDID,
					"AUX_CH: setting EDID_ADDR!\n");
				i2c_edid->state = I2C_AUX_CH;
				i2c_edid->port = port_idx;
				i2c_edid->slave_selected = true;
				if (dpy_has_monitor_on_port(vgt, port_idx) &&
					dpy_port_is_dp(vgt, port_idx))
					i2c_edid->edid_available = true;
			} else {
				gvt_dbg(GVT_DBG_EDID,
		"Not supported address access [0x%x]with I2C over AUX_CH!\n",
				addr);
			}
		}
	} else if ((op & 0x1) == GVT_AUX_I2C_WRITE) {
		/* TODO
		 * We only support EDID reading from I2C_over_AUX. And
		 * we do not expect the index mode to be used. Right now
		 * the WRITE operation is ignored. It is good enough to
		 * support the gfx driver to do EDID access.
		 */
	} else {
		ASSERT((op & 0x1) == GVT_AUX_I2C_READ);
		ASSERT(msg_length == 4);
		if (i2c_edid->edid_available && i2c_edid->slave_selected) {
			unsigned char val = edid_get_byte(vgt);
			aux_data_for_write = (val << 16);
		}
	}

	/* write the return value in AUX_CH_DATA reg which includes:
	 * ACK of I2C_WRITE
	 * returned byte if it is READ
	 */
	aux_data_for_write |= (GVT_AUX_I2C_REPLY_ACK & 0xff) << 24;
	__vreg(vgt, offset + 4) = aux_data_for_write;

	return;
}

void gvt_init_i2c_edid(struct vgt_device *vgt)
{
	struct gvt_i2c_edid_t *edid = &vgt->state.display.gvt_i2c_edid;

	edid->state = I2C_NOT_SPECIFIED;

	edid->port = I915_MAX_PORTS;
	edid->slave_selected = false;
	edid->edid_available = false;
	edid->current_edid_read = 0;

	memset(&edid->gmbus, 0, sizeof(struct gvt_i2c_gmbus_t));

	edid->aux_ch.i2c_over_aux_ch = false;
	edid->aux_ch.aux_ch_mot = false;
}
