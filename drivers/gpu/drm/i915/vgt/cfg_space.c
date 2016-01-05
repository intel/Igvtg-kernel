/*
 * PCI Configuration Space virtualization
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


#include "vgt.h"

typedef union _SCI_REG_DATA{
	uint16_t data;
	struct {
		uint16_t trigger:1; /* bit 0: trigger SCI */
		uint16_t reserve:14;
		uint16_t method:1; /* bit 15: 1 - SCI, 0 - SMI */
	};
} SCI_REG_DATA;

static bool vgt_cfg_sci_read(struct vgt_device *vgt, unsigned int offset,
	void *p_data, int bytes)
{
	printk("VM%d Read SCI Trigger Register, bytes=%d value=0x%x\n", vgt->vm_id, bytes, *(uint16_t*)p_data);

	return true;
}

static bool vgt_cfg_sci_write(struct vgt_device *vgt, unsigned int offset,
	void *p_data, int bytes)
{
	SCI_REG_DATA sci_reg;

	printk("VM%d Write SCI Trigger Register, bytes=%d value=0x%x\n", vgt->vm_id, bytes, *(uint32_t*)p_data);

	if( (bytes == 2) || (bytes == 4)){
		memcpy (&vgt->state.cfg_space[offset], p_data, bytes);
	} else {
		printk("Warning: VM%d vgt_cfg_sci_write invalid bytes=%d, ignore it\n", vgt->vm_id, bytes);
		return false;
	}

	sci_reg.data = *(uint16_t*)(vgt->state.cfg_space + offset);
	sci_reg.method = 1; /* set method to SCI */
	if (sci_reg.trigger == 1){
		printk("SW SCI Triggered by VM%d\n", vgt->vm_id);
		/* TODO: add SCI emulation */
		sci_reg.trigger = 0; /* SCI completion indicator */
	}

	memcpy (&vgt->state.cfg_space[offset], &sci_reg.data , 2);

	return true;
}

#define VGT_OPREGION_FUNC(scic)						\
	({								\
		uint32_t __ret;						\
		__ret = (scic & _REGBIT_OPREGION_SCIC_FUNC_MASK) >>	\
		_REGBIT_OPREGION_SCIC_FUNC_SHIFT;			\
		__ret;							\
	})

#define VGT_OPREGION_SUBFUNC(scic)					\
	({								\
		uint32_t __ret;						\
		__ret = (scic & _REGBIT_OPREGION_SCIC_SUBFUNC_MASK) >>	\
			_REGBIT_OPREGION_SCIC_SUBFUNC_SHIFT;		\
		__ret;							\
	})

static const char *vgt_opregion_func_name(uint32_t func)
{
	const char *name = NULL;

	switch (func) {
	case 0 ... 3:
	case 5:
	case 7 ... 15:
		name = "Reserved";
		break;

	case 4:
		name = "Get BIOS Data";
		break;

	case 6:
		name = "System BIOS Callbacks";
		break;

	default:
		name = "Unknown";
		break;
	}
	return name;
}

static const char *vgt_opregion_subfunc_name(uint32_t subfunc)
{
	const char *name = NULL;
	switch (subfunc) {
	case 0:
		name = "Supported Calls";
		break;

	case 1:
		name = "Requested Callbacks";
		break;

	case 2 ... 3:
	case 8 ... 9:
		name = "Reserved";
		break;

	case 5:
		name = "Boot Display";
		break;

	case 6:
		name = "TV-Standard/Video-Connector";
		break;

	case 7:
		name = "Internal Graphics";
		break;

	case 10:
		name = "Spread Spectrum Clocks";
		break;

	case 11:
		name = "Get AKSV";
		break;

	default:
		name = "Unknown";
		break;
	}
	return name;
};

/* Only allowing capability queries */
static bool vgt_opregion_is_capability_get(uint32_t scic)
{
	uint32_t func, subfunc;

	func = VGT_OPREGION_FUNC(scic);
	subfunc = VGT_OPREGION_SUBFUNC(scic);

	if ((func == VGT_OPREGION_SCIC_F_GETBIOSDATA &&
			subfunc == VGT_OPREGION_SCIC_SF_SUPPRTEDCALLS) ||
			(func == VGT_OPREGION_SCIC_F_GETBIOSDATA &&
			 subfunc == VGT_OPREGION_SCIC_SF_REQEUSTEDCALLBACKS) ||
			(func == VGT_OPREGION_SCIC_F_GETBIOSCALLBACKS &&
			 subfunc == VGT_OPREGION_SCIC_SF_SUPPRTEDCALLS)) {
		return true;
	}

	return false;
}
/*
 * emulate multiple capability query requests
 */
static void vgt_hvm_opregion_handle_request(struct vgt_device *vgt, uint32_t swsci)
{
	uint32_t *scic, *parm;
	uint32_t func, subfunc;
	scic = vgt->state.opregion_va + VGT_OPREGION_REG_SCIC;
	parm = vgt->state.opregion_va + VGT_OPREGION_REG_PARM;

	if (!(swsci & _REGBIT_CFG_SWSCI_SCI_SELECT)) {
		vgt_warn("VM%d requesting SMI service\n", vgt->vm_id);
		return;
	}
	/* ignore non 0->1 trasitions */
	if ((vgt->state.cfg_space[VGT_REG_CFG_SWSCI_TRIGGER] &
				_REGBIT_CFG_SWSCI_SCI_TRIGGER) ||
			!(swsci & _REGBIT_CFG_SWSCI_SCI_TRIGGER)) {
		return;
	}

	func = VGT_OPREGION_FUNC(*scic);
	subfunc = VGT_OPREGION_SUBFUNC(*scic);
	if (!vgt_opregion_is_capability_get(*scic)) {
		vgt_warn("VM%d requesting runtime service: func \"%s\", subfunc \"%s\"\n",
				vgt->vm_id,
				vgt_opregion_func_name(func),
				vgt_opregion_subfunc_name(subfunc));

		/*
		 * emulate exit status of function call, '0' means
		 * "failure, generic, unsupported or unkown cause"
		 */
		*scic &= ~_REGBIT_OPREGION_SCIC_EXIT_MASK;
		return;
	}

	*scic = 0;
	*parm = 0;
}

bool vgt_emulate_cfg_read(struct vgt_device *vgt, unsigned int offset, void *p_data, int bytes)
{

	ASSERT ((offset + bytes) <= VGT_CFG_SPACE_SZ);
	memcpy(p_data, &vgt->state.cfg_space[offset], bytes);

	/* TODO: hooks */
	offset &= ~3;
	switch (offset) {
		case 0:
		case 4:
			break;
		case VGT_REG_CFG_SWSCI_TRIGGER:
			vgt_cfg_sci_read(vgt, offset, p_data, bytes);
			break;
		default:
			break;
	}
	return true;
}

bool vgt_emulate_cfg_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, int bytes)
{
	char *cfg_space = &vgt->state.cfg_space[0];
	uint32_t *cfg_reg, new;
	uint64_t size;
	u8 old_cmd, cmd_changed; /* we don't care the high 8 bits */
	bool rc = true;
	uint32_t low_mem_max_gpfn;

	ASSERT ((off + bytes) <= VGT_CFG_SPACE_SZ);
	cfg_reg = (uint32_t*)(cfg_space + (off & ~3));
	switch (off & ~3) {
		case VGT_REG_CFG_VENDOR_ID:
			low_mem_max_gpfn = *(uint32_t *)p_data;
			vgt_info("low_mem_max_gpfn: 0x%x\n", low_mem_max_gpfn);
			if (bytes != 4 ||
				low_mem_max_gpfn >= (1UL << (32-PAGE_SHIFT))) {
				vgt_warn("invalid low_mem_max_gpfn!\n");
				break;
			}
			if (vgt->low_mem_max_gpfn == 0)
				vgt->low_mem_max_gpfn = low_mem_max_gpfn;
			break;

		case VGT_REG_CFG_COMMAND:
			old_cmd = vgt->state.cfg_space[off];
			cmd_changed = old_cmd ^ (*(u8*)p_data);
			memcpy (&vgt->state.cfg_space[off], p_data, bytes);
			if (cmd_changed & _REGBIT_CFG_COMMAND_MEMORY) {
				if (old_cmd & _REGBIT_CFG_COMMAND_MEMORY) {
					 vgt_hvm_map_aperture(vgt, 0);
				} else {
					if(!vgt->state.bar_mapped[1]) {
						vgt_hvm_map_aperture(vgt, 1);
						vgt_hvm_set_trap_area(vgt, 1);
					}
				}
			} else {
				vgt_dbg(VGT_DBG_GENERIC, "need to trap the PIO BAR? "
					"old_cmd=0x%x, cmd_changed=%0x",
					old_cmd, cmd_changed);
			}
			break;
		case VGT_REG_CFG_SPACE_BAR0:	/* GTTMMIO */
		case VGT_REG_CFG_SPACE_BAR1:	/* GMADR */
		case VGT_REG_CFG_SPACE_BAR2:	/* IO */
			ASSERT((bytes == 4) && (off & 3) == 0);
			new = *(uint32_t *)p_data;
			printk("Programming bar 0x%x with 0x%x\n", off, new);
			size = vgt->state.bar_size[(off - VGT_REG_CFG_SPACE_BAR0)/8];
			if (new == 0xFFFFFFFF) {
				/*
				 * Power-up software can determine how much address
				 * space the device requires by writing a value of
				 * all 1's to the register and then reading the value
				 * back. The device will return 0's in all don't-care
				 * address bits.
				 */
				new = new & ~(size-1);
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR1)
					vgt_hvm_map_aperture(vgt, 0);
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR0)
					vgt_hvm_set_trap_area(vgt, 0);
				vgt_pci_bar_write_32(vgt, off, new);
			} else {
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR1)
					vgt_hvm_map_aperture(vgt, 0);
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR0)
					vgt_hvm_set_trap_area(vgt, 0);
				vgt_pci_bar_write_32(vgt, off, new);
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR1)
					vgt_hvm_map_aperture(vgt, 1);
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR0)
					vgt_hvm_set_trap_area(vgt, 1);
			}
			break;

		case VGT_REG_CFG_SPACE_MSAC:
			printk("Guest write MSAC %x, %d: Not supported yet\n",
					*(char *)p_data, bytes);
			break;

		case VGT_REG_CFG_SWSCI_TRIGGER:
			new = *(uint32_t *)p_data;
			if (vgt->vm_id == 0)
				rc = vgt_cfg_sci_write(vgt, off, p_data, bytes);
			else
				vgt_hvm_opregion_handle_request(vgt, new);
			break;

		case VGT_REG_CFG_OPREGION:
			new = *(uint32_t *)p_data;
			if (vgt->vm_id == 0) {
				/* normally domain 0 shouldn't write this reg */
				memcpy(&vgt->state.cfg_space[off], p_data, bytes);
			} else if (vgt->state.opregion_va == NULL) {
				vgt_hvm_opregion_init(vgt, new);
				memcpy(&vgt->state.cfg_space[off], p_data, bytes);
			} else
				vgt_warn("VM%d write OPREGION multiple times",
						vgt->vm_id);
			break;

		case VGT_REG_CFG_SPACE_BAR1+4:
		case VGT_REG_CFG_SPACE_BAR0+4:
		case VGT_REG_CFG_SPACE_BAR2+4:
			ASSERT((bytes == 4) && (off & 3) == 0);
			new = *(uint32_t *)p_data;
			printk("Programming bar 0x%x with 0x%x\n", off, new);
			size = vgt->state.bar_size[(off - (VGT_REG_CFG_SPACE_BAR0 + 4))/8];
			/* for 32bit mode bar it returns all-0 in upper 32 bit, for 64bit
			 * mode bar it will calculate the size with lower 32bit and return
			 * the corresponding value
			 */
			if (new == 0xFFFFFFFF) {
				if (VGT_GET_BITS(*(cfg_space + off - 4), 2, 1) == 2)
					new &= ~(size-1) >> 32;
				else
					new = 0;
				*cfg_reg = new;
			} else {
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR1 + 4)
					vgt_hvm_map_aperture(vgt, 0);
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR0 + 4)
					vgt_hvm_set_trap_area(vgt, 0);
				*cfg_reg = new;
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR1 + 4)
					vgt_hvm_map_aperture(vgt, 1);
				if ((off & ~3) == VGT_REG_CFG_SPACE_BAR0 + 4)
					vgt_hvm_set_trap_area(vgt, 1);
			}
			break;
		/* HSW/BDW */
		case 0x90:
		case 0x94:
		case 0x98:
		/* SKL */
		case 0xAC:
		case 0xB0:
		case 0xB4:
			printk("vGT: write to MSI capa(%x) with val (%x)\n", off, *(uint32_t *)p_data);
		default:
			memcpy (&vgt->state.cfg_space[off], p_data, bytes);
			break;
	}
	/*
	 * Assume most Dom0's cfg writes should be propagated to
	 * the real conf space. In the case where propagation is required
	 * but value needs be changed (sReg), do it here
	 */
	return rc;
}
