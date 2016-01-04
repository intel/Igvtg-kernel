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

#include <linux/acpi.h>

#include "gvt.h"

static bool init_virtual_opregion(struct vgt_device *vgt, u64 gpa)
{
	struct gvt_virtual_opregion_state *opregion = &vgt->state.opregion;
	void *host_va = vgt->pdev->opregion_va;
	u8 *buf;
	int i;

	if (opregion->va) {
		gvt_err("[vgt %d] is trying to init opregion multiple times",
				vgt->id);
		return false;
	}

	opregion->va = (void *)__get_free_pages(GFP_ATOMIC |
			GFP_DMA32 | __GFP_ZERO,
			GVT_OPREGION_PORDER);

	if (!opregion->va) {
		gvt_err("fail to allocate memory for opregion pages");
		return false;
	}

	memcpy_fromio(opregion->va, host_va, GVT_OPREGION_SIZE);

	for (i = 0; i < GVT_OPREGION_PAGES; i++)
		opregion->gfn[i] = (gpa >> PAGE_SHIFT) + i;

	/* for unknown reason, the value in LID field is incorrect
	 * which block the windows guest, so workaround it by force
	 * setting it to "OPEN"
	 */
	buf = (u8 *)opregion->va;
	buf[GVT_OPREGION_REG_CLID] = 0x3;

	return true;
}

static bool map_virtual_opregion(struct vgt_device *vgt, int map)
{
	struct gvt_virtual_opregion_state *opregion = &vgt->state.opregion;
	int r;
	int i;

	for (i = 0; i < GVT_OPREGION_PAGES; i++) {
		r = hypervisor_map_mfn_to_gpfn(vgt,
				opregion->gfn[i],
				hypervisor_virt_to_mfn(opregion->va + i * PAGE_SIZE),
				1,
				map,
				GVT_MAP_OPREGION);
		if (r != 0) {
			gvt_err("hypervisor_map_mfn_to_gpfn fail with %d!\n", r);
			return false;
		}
	}
	return true;
}

void gvt_clean_instance_opregion(struct vgt_device *vgt)
{
	struct gvt_host *host = &gvt_host;
	struct gvt_virtual_opregion_state *opregion = &vgt->state.opregion;
	int i;

	gvt_dbg_core("clean instance opregion, id %d", vgt->id);

	if (!opregion->va)
		return;

	if (host->hypervisor_type == GVT_HYPERVISOR_TYPE_KVM) {
		vunmap(opregion->va);
		for (i = 0; i < GVT_OPREGION_PAGES; i++) {
			if (opregion->pages[i]) {
				put_page(opregion->pages[i]);
				opregion->pages[i] = NULL;
			}
		}
	} else {
		map_virtual_opregion(vgt, 0);
		free_pages((unsigned long)opregion->va,
				GVT_OPREGION_PORDER);
	}

	opregion->va = NULL;
}

bool gvt_init_instance_opregion(struct vgt_device *vgt, u64 gpa)
{
	struct gvt_host *host = &gvt_host;

	gvt_dbg_core("init instance opregion, id %d", vgt->id);

	if (host->hypervisor_type == GVT_HYPERVISOR_TYPE_XEN) {
		gvt_dbg_core("emulate opregion from kernel");

		if (!init_virtual_opregion(vgt, gpa))
			return false;

		if (!map_virtual_opregion(vgt, 1))
			return false;
	} else {
		gvt_dbg_core("emulate opregion from userspace");

		/* If opregion pages are not allocated from host kenrel, most of
		 * the params are meaningless */
		hypervisor_map_mfn_to_gpfn(vgt,
				0, //not used
				0, //not used
				2, //not used
				1,
				GVT_MAP_OPREGION);
	}

	return true;
}

void gvt_clean_opregion(struct pgt_device *pdev)
{
	if (pdev->opregion_va) {
		iounmap(pdev->opregion_va);
		pdev->opregion_va = NULL;
	}
}

bool gvt_init_opregion(struct pgt_device *pdev)
{
	gvt_dbg_core("init host opregion");

	pci_read_config_dword(pdev->dev_priv->dev->pdev, GVT_REG_CFG_OPREGION,
			&pdev->opregion_pa);

	pdev->opregion_va = acpi_os_ioremap(pdev->opregion_pa,
			GVT_OPREGION_SIZE);

	if (!pdev->opregion_va) {
		gvt_err("fail to map host opregion");
		return false;
	}
	return true;
}

#define GVT_OPREGION_FUNC(scic)						\
	({								\
	 u32 __ret;						\
	 __ret = (scic & _REGBIT_OPREGION_SCIC_FUNC_MASK) >>	\
	 _REGBIT_OPREGION_SCIC_FUNC_SHIFT;			\
	 __ret;							\
	 })

#define GVT_OPREGION_SUBFUNC(scic)					\
	({								\
	 u32 __ret;						\
	 __ret = (scic & _REGBIT_OPREGION_SCIC_SUBFUNC_MASK) >>	\
	 _REGBIT_OPREGION_SCIC_SUBFUNC_SHIFT;		\
	 __ret;							\
	 })

static const char *opregion_func_name(u32 func)
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

static const char *opregion_subfunc_name(u32 subfunc)
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
static bool opregion_is_capability_get(u32 scic)
{
	u32 func, subfunc;

	func = GVT_OPREGION_FUNC(scic);
	subfunc = GVT_OPREGION_SUBFUNC(scic);

	if ((func == GVT_OPREGION_SCIC_F_GETBIOSDATA &&
				subfunc == GVT_OPREGION_SCIC_SF_SUPPRTEDCALLS) ||
			(func == GVT_OPREGION_SCIC_F_GETBIOSDATA &&
			 subfunc == GVT_OPREGION_SCIC_SF_REQEUSTEDCALLBACKS) ||
			(func == GVT_OPREGION_SCIC_F_GETBIOSCALLBACKS &&
			 subfunc == GVT_OPREGION_SCIC_SF_SUPPRTEDCALLS)) {
		return true;
	}

	return false;
}

/*
 * emulate multiple capability query requests
 */
void gvt_emulate_opregion_request(struct vgt_device *vgt, u32 swsci)
{
	struct gvt_virtual_opregion_state *opregion = &vgt->state.opregion;
	u32 *scic, *parm;
	u32 func, subfunc;
	scic = opregion->va + GVT_OPREGION_REG_SCIC;
	parm = opregion->va + GVT_OPREGION_REG_PARM;

	if (!(swsci & _REGBIT_CFG_SWSCI_SCI_SELECT)) {
		gvt_err("[vgt %d] requesting SMI service\n", vgt->id);
		return;
	}
	/* ignore non 0->1 trasitions */
	if ((vgt->state.cfg.space[GVT_REG_CFG_SWSCI_TRIGGER] &
				_REGBIT_CFG_SWSCI_SCI_TRIGGER) ||
			!(swsci & _REGBIT_CFG_SWSCI_SCI_TRIGGER)) {
		return;
	}

	func = GVT_OPREGION_FUNC(*scic);
	subfunc = GVT_OPREGION_SUBFUNC(*scic);
	if (!opregion_is_capability_get(*scic)) {
		gvt_err("[vgt %d] requesting runtime service: func \"%s\", subfunc \"%s\"\n",
				vgt->id,
				opregion_func_name(func),
				opregion_subfunc_name(subfunc));
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
