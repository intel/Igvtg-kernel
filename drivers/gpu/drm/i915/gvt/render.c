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

struct render_mmio {
	int ring_id;
	u32 reg;
	u32 value;
	u32 mask;
};

static struct render_mmio gen8_render_mmio_list[] = {
	{RCS, 0x229c, 0xffff},
	{RCS, 0x2248, 0x0},
	{RCS, 0x2098, 0x0},
	{RCS, 0x20c0, 0xffff},
	{RCS, 0x24d0, 0},
	{RCS, 0x24d4, 0},
	{RCS, 0x24d8, 0},
	{RCS, 0x24dc, 0},
	{RCS, 0x7004, 0xffff},
	{RCS, 0x7008, 0xffff},
	{RCS, 0x7000, 0xffff},
	{RCS, 0x7010, 0xffff},
	{RCS, 0x7300, 0xffff},
	{RCS, 0x83a4, 0xffff},

	{BCS, GVT_RING_MODE(BLT_RING_BASE), 0xffff},
	{BCS, _RING_MI_MODE(BLT_RING_BASE), 0xffff},
	{BCS, _RING_INSTPM(BLT_RING_BASE), 0xffff},
	{BCS, _RING_HWSTAM(BLT_RING_BASE), 0xffff},
	{BCS, 0x22028, 0x0},
};

void gvt_load_render_mmio(struct vgt_device *vgt, int ring_id)
{
	u32 v;
	int i;

	for (i = 0; i < ARRAY_SIZE(gen8_render_mmio_list); i++) {
		struct render_mmio *mmio = gen8_render_mmio_list + i;
		if (mmio->ring_id != ring_id)
			continue;

		mmio->value = gvt_mmio_read(vgt->pdev, mmio->reg);
		if (mmio->mask)
			v = __vreg(vgt, mmio->reg) | (mmio->mask << 16);
		else
			v = __vreg(vgt, mmio->reg);

		gvt_mmio_write(vgt->pdev, mmio->reg, v);
		gvt_mmio_posting_read(vgt->pdev, mmio->reg);

		gvt_dbg_render("reg %x value old %x new %x",
				mmio->reg, mmio->value, v);
	}
}

void gvt_restore_render_mmio(struct vgt_device *vgt, int ring_id)
{
	u32 v;
	int i;

	for (i = 0; i < ARRAY_SIZE(gen8_render_mmio_list); i++) {
		struct render_mmio *mmio = gen8_render_mmio_list + i;
		if (mmio->ring_id != ring_id)
			continue;

		__vreg(vgt, mmio->reg) = gvt_mmio_read(vgt->pdev, mmio->reg);

		if (mmio->mask) {
			__vreg(vgt, mmio->reg) &= ~(mmio->mask << 16);
			v = mmio->value | (mmio->mask << 16);
		} else
			v = mmio->value;

		gvt_mmio_write(vgt->pdev, mmio->reg, v);
		gvt_mmio_posting_read(vgt->pdev, mmio->reg);

		gvt_dbg_render("reg %x value old %x new %x",
				mmio->reg, mmio->value, v);
	}
}
