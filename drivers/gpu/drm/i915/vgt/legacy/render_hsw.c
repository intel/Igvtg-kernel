/*
 * Render context management for hsw
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

#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include "../vgt.h"

/* FIXME: need audit all render resources carefully */
vgt_reg_t vgt_gen7_render_regs[] = {
	/* Add IVB register, so they all got pass-through */

	ARB_MODE,

	BLT_HWS_PGA_GEN7,
	RENDER_HWS_PGA_GEN7,
	BSD_HWS_PGA_GEN7,
	_REG_VECS_HWS_PGA,

	_REG_BCS_MI_MODE,
	_REG_BCS_BLT_MODE_IVB,
	_REG_BCS_INSTPM,
	_REG_BCS_HWSTAM,
	_REG_BCS_EXCC,
	_REG_BCS_UHPTR,
	_REG_BRSYNC,
	_REG_BVSYNC,
	_REG_BVESYNC,

	GFX_MODE_GEN7,
	_REG_RCS_HWSTAM,
	_REG_RCS_UHPTR,
	_REG_RBSYNC,
	_REG_RVSYNC,
	_REG_RVESYNC,

	_REG_VCS_MI_MODE,
	_REG_VCS_MFX_MODE_IVB,
	_REG_VCS_INSTPM,
	_REG_VCS_HWSTAM,
	_REG_VCS_EXCC,
	_REG_VCS_UHPTR,
	_REG_VBSYNC,
	_REG_VRSYNC,
	_REG_VVESYNC,

	_REG_VECS_MI_MODE,
	_REG_VEBOX_MODE,
	_REG_VECS_INSTPM,
	_REG_VECS_HWSTAM,
	_REG_VECS_EXCC,
	_REG_VERSYNC,
	_REG_VEBSYNC,
	_REG_VEVSYNC,

	_REG_RCS_BB_PREEMPT_ADDR,

	/* more check for this group later */
	0x23bc,
	0x2448,
	0x244c,
	0x2450,
	0x2454,
	0x7034,
	0x2b00,
	0x91b8,
	0x91bc,
	0x91c0,
	0x91c4,
	0x9150,
	0x9154,
	0x9160,
	0x9164,

	0x4040,
	0xb010,
	0xb020,
	0xb024,

	//GEN6_UCGCTL1,
	//GEN6_UCGCTL2,
	//ILK_DISPLAY_CHICKEN1,
	//ILK_DSPCLK_GATE_D,
	//_REG_SUPER_QUEUE_CONFIG,
	GAM_ECOCHK,
	//GEN7_MISCCPCTL,

	0x2450,
	0x20dc,
	_3D_CHICKEN3,
	0x2088,
	0x20e4,
	GEN7_COMMON_SLICE_CHICKEN1,
	GEN7_L3CNTLREG1,
	GEN7_L3_CHICKEN_MODE_REGISTER,
	0x9030,
	0x20a0,
	0x20e8,
	0xb038,
};

vgt_reg_t *vgt_get_extra_ctx_regs_gen7(void)
{
	return &vgt_gen7_render_regs[0];
}

int vgt_get_extra_ctx_regs_num_gen7(void)
{
	return ARRAY_NUM(vgt_gen7_render_regs);
}

static u32 hsw_null_context_cmds[] = {
	0x7a000003, 0x01000000, 0x00000000, 0x00000000, 0x00000000, //0
	0x69040000, //0x14
	0x0, 0x0, 0x0, //0x18  //{0x12400001, 0x00138064, <addr>+0x124};
	0x11000001, 0x0000b020, 0x00800040, //0x24
	0x11000001, 0x0000b024, 0x00080410, //0x30
	0x11000001, 0x0000b010, 0x00610000, //0x3c
	0x78140001, 0x24000000, 0x80000000, //0x48
	0x78200006, 0x00000000, 0x80000000, 0x00000000,
	0x55800400, 0x00000000, 0x00000000, 0x00000000, //0x54
	0x78130005, 0x00000000, 0x20000000, 0x02001808,
	0x00000000, 0x00000000, 0x00000000, //0x74
	0x781f000c, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, //
	0x78100004, 0x00000000, 0x80010000, 0x0, 0x0, 0x0, //0x0c8
	0x781b0005, 0x00010023, 0x0, 0x0, 0x0, 0x00000800, 0x0, //0x0e0
	0x78110005, 0x0, 0x0, 0x0, 0x0, 0x01000000, 0x0, //0x0fc
	0x781e0001, 0x0, 0x0, //0x118
	0x781d0004, 0x0, 0x00010000, 0x0, 0x0, 0x0, //0x124
	0x78120002, 0x0, 0x20000001, 0x0, //0x13c
	0x781c0002, 0x0, 0x427c0000, 0x42800000, //0x14c
	0x780c0000, 0x0, //0x15c
	0x78300000, 0x00000018, //0x164
	0x78310000, 0x0, //0x16c
	0x78320000, 0x0, //0x174
	0x78330000, 0x0, //0x17c
	0x79120000, 0x0, //0x184
	0x79130000, 0x0, //0x18c
	0x79140000, 0x0, //0x194
	0x79150000, 0x0, //0x19c
	0x79160000, 0x0, //0x1a4
	0x78150005, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, //0x1ac
	0x78190005, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, //0x1c8
	0x781a0005, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, //0x1e4
	0x78160005, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, //0x200
	0x78170005, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, //0x21c
	0x79170101, 0x00000000, 0x80808080, 0x00000000,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, //0x238
	0x79180002, 0x00000000, 0x00000000, 0x00000000,
	0x79180002, 0x20000000, 0x00000000, 0x00000000,
	0x79180002, 0x40000000, 0x00000000, 0x00000000,
	0x79180002, 0x60000000, 0x00000000, 0x00000000, //0x644
	0x6101000a, 0x00000001, 0x00000001, 0x00000001, //{0x6101000a, <addr> | 0x1, <addr> | 0x1, <addr> | 0x1,
	0x00000001, 0x00000001, 0x00000001, 0x00000001, //0x1, <addr>|0x1, 0x1, 0x1,
	0x00000001, 0x00000001, 0x00000001, 0x00000001,  //0x1, 0x1, 0x0, 0x0}//0x684
	0x61020000, 0x00000000, //0x684
	0x79000002, 0x00000000, 0x1fff1fff, 0x00000000, //0x6bc
	0x78050005, 0xe0000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, //0x6cc
	0x79040002, 0x00000000, 0x00000000, 0x00000000,
	0x79040002, 0x40000000, 0x00000000, 0x00000000,
	0x79040002, 0x80000000, 0x00000000, 0x00000000,
	0x79040002, 0xc0000000, 0x00000000, 0x00000000, //0x6e8
	0x79080001, 0x00000000, 0x00000000, //0x728
	0x790a0001, 0x00000000, 0x00000000, //0x734
	0x8060001, 0x00000000, 0x00000000, //0x740
	0x78070001, 0x00000000, 0x00000000, //0x74c
	0x78040001, 0x0, 0x1, //0x758
	0x79110000, 0x0, //0x764
	0x79060000, 0x0, //0x76c
	0x7907001f, //0x774
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, //0x778
	0x790200ff,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, //0x7f8
	0x790c00ff,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, //0xbfc
	0x780a0001, 0x00000000, 0x00000000, //0x1000
	0x78080083, //0x100c
	0x00005000, 0x00000000, 0x00000000, 0x00000000,
	0x04005000, 0x00000000, 0x00000000, 0x00000000,
	0x08005000, 0x00000000, 0x00000000, 0x00000000,
	0x0c005000, 0x00000000, 0x00000000, 0x00000000,
	0x10005000, 0x00000000, 0x00000000, 0x00000000,
	0x14005000, 0x00000000, 0x00000000, 0x00000000,
	0x18005000, 0x00000000, 0x00000000, 0x00000000,
	0x1c005000, 0x00000000, 0x00000000, 0x00000000,
	0x20005000, 0x00000000, 0x00000000, 0x00000000,
	0x24005000, 0x00000000, 0x00000000, 0x00000000,
	0x28005000, 0x00000000, 0x00000000, 0x00000000,
	0x2c005000, 0x00000000, 0x00000000, 0x00000000,
	0x30005000, 0x00000000, 0x00000000, 0x00000000,
	0x34005000, 0x00000000, 0x00000000, 0x00000000,
	0x38005000, 0x00000000, 0x00000000, 0x00000000,
	0x3c005000, 0x00000000, 0x00000000, 0x00000000,
	0x40005000, 0x00000000, 0x00000000, 0x00000000,
	0x44005000, 0x00000000, 0x00000000, 0x00000000,
	0x48005000, 0x00000000, 0x00000000, 0x00000000,
	0x4c005000, 0x00000000, 0x00000000, 0x00000000,
	0x50005000, 0x00000000, 0x00000000, 0x00000000,
	0x54005000, 0x00000000, 0x00000000, 0x00000000,
	0x58005000, 0x00000000, 0x00000000, 0x00000000,
	0x5c005000, 0x00000000, 0x00000000, 0x00000000,
	0x60005000, 0x00000000, 0x00000000, 0x00000000,
	0x64005000, 0x00000000, 0x00000000, 0x00000000,
	0x68005000, 0x00000000, 0x00000000, 0x00000000,
	0x6c005000, 0x00000000, 0x00000000, 0x00000000,
	0x70005000, 0x00000000, 0x00000000, 0x00000000,
	0x74005000, 0x00000000, 0x00000000, 0x00000000,
	0x78005000, 0x00000000, 0x00000000, 0x00000000,
	0x7c005000, 0x00000000, 0x00000000, 0x00000000,
	0x80005000, 0x00000000, 0x00000000, 0x00000000, //0x1010
	0x78090043, //0x1220
	0x02000000, 0x22220000, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, //0x1224
	0x680b0001, //0x1334
	0x78260000, 0x0, //0x1338
	0x78270000, 0x0, //0x1340
	0x78280000, 0x0, //0x1348
	0x78290000, 0x0, //0x1350
	0x782a0000, 0x0, //0x1358
	0x79190001, 0x00000060, 0x00000000, //0x1360
	0x791a0001, 0x00000030, 0x00000000, //0x136c
	0x791b0001, 0x00000030, 0x00000000, //0x1378
	0x780e0000, 0x00000041, //0x1384
	0x78240000, 0x000000c1, //0x138c
	0x78250000, 0x00000081, //0x1394
	0x782b0000, 0x00000000, //0x139c
	0x782c0000, 0x00000000, //0x13a4
	0x782d0000, 0x00000000, //0x13ac
	0x782e0000, 0x00000000, //0x13b4
	0x782f0000, 0x00000000, //0x13bc
	0x790e0004, 0x0, 0x0, 0x0, 0x0, 0x0, //0x13c4
	0x780f0000, 0x0, //0x13dc
	0x78230000, 0x0, //0x13e4
	0x78210000, 0xe0, //0x13ec
	0x7b000005, 0x00000004, 0x00000001, 0x0, 0x1, 0x0, 0x0, //0x13f4
};

static u32 hsw_null_indirect_state[] = {
	0x0, 0x0, //0x0
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, //0x08
	0x0, 0x0, 0x3f800000, 0x3f800000, 0x3f800000, 0x3f800000, //0x040
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, //0x058
	0x0, 0x0, 0x08000000, //0x080
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, //0x08c
	0x00188221, 0x0, //0x0c0
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, //0x0c8
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, //0x0e0
	0x0, //0x120
	0x0, //0x124
};

static bool ring_wait_for_completion(struct pgt_device *pdev, int id)
{
	u32 *ptr;

	/* now a single magic number, because only RCS supports hw switch */
	if (id != RING_BUFFER_RCS)
		return true;

	ptr = (u32 *)(phys_aperture_vbase(pdev) + vgt_data_ctx_magic(pdev));
	if (wait_for_atomic((*ptr == pdev->magic), VGT_RING_TIMEOUT) != 0) {
		vgt_err("Timeout %d ms for CMD comletion on ring %d\n",
			VGT_RING_TIMEOUT, id);
		vgt_err("expected(%d), actual(%d)\n", pdev->magic, *ptr);
		return false;
	}

	return true;
}

static bool ring_is_stopped(struct pgt_device *pdev, int id)
{
	vgt_reg_t val;

	val = VGT_MMIO_READ(pdev, pdev->ring_mi_mode[id]);
	if ((val & (STOP_RING | MODE_IDLE)) ==
	    (STOP_RING | MODE_IDLE))
		return true;

	return false;
}

static inline bool stop_ring(struct pgt_device *pdev, int id)
{
	VGT_MMIO_WRITE(pdev, pdev->ring_mi_mode[id],
			STOP_RING | (STOP_RING << 16));

	if (wait_for_atomic(ring_is_stopped(pdev, id), VGT_RING_TIMEOUT)) {
		vgt_err("Timeout stop ring (%d) for %d ms\n",
			id, VGT_RING_TIMEOUT);
		return false;
	}

	return true;
}

static inline void start_ring(struct pgt_device *pdev, int id)
{
	VGT_MMIO_WRITE(pdev, pdev->ring_mi_mode[id],
			STOP_RING << 16);
	VGT_POST_READ(pdev, pdev->ring_mi_mode[id]);
}

/*
 * write to head is undefined when ring is enabled.
 *
 * so always invoke this disable action when recovering a new ring setting
 */
static inline void disable_ring(struct pgt_device *pdev, int id)
{
	/* disable the ring */
	VGT_WRITE_CTL(pdev, id, 0);
	/* by ktian1. no source for this trick */
	VGT_POST_READ_CTL(pdev, id);
}

static inline void restore_ring_ctl(struct pgt_device *pdev, int id,
		vgt_reg_t val)
{
	VGT_WRITE_CTL(pdev, id, val);
	VGT_POST_READ_CTL(pdev, id);
}

static inline void enable_ring(struct pgt_device *pdev, int id, vgt_reg_t val)
{
	ASSERT(val & _RING_CTL_ENABLE);
	restore_ring_ctl(pdev, id, val);
}

static void vgt_save_ringbuffer(struct vgt_device *vgt, int id)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_state_ring_t *rb = &vgt->rb[id];

	/* only head is HW updated */
	rb->sring.head = VGT_READ_HEAD(pdev, id);
	rb->vring.head = rb->sring.head;
}

/*
 * TODO: the context layout could be different on generations.
 * e.g. ring head/tail, ccid, etc. when PPGTT is enabled
 */
#define OFF_CACHE_MODE_0	0x4A
#define OFF_CACHE_MODE_1	0x4B
#define OFF_INSTPM		0x4D
#define OFF_EXCC		0x4E
#define OFF_MI_MODE		0x4F

void update_context(struct vgt_device *vgt, uint64_t context)
{
	struct pgt_device *pdev = vgt->pdev;
	uint64_t ptr;
	u32 *vptr;

	ptr = (uint64_t)phys_aperture_vbase(pdev) + context;
	vptr = (u32 *)ptr;
#define UPDATE_FIELD(off, reg) \
	*(vptr + off) = 0xFFFF0000 | (__sreg(vgt, reg) & 0xFFFF);

	UPDATE_FIELD(OFF_CACHE_MODE_0, _REG_CACHE_MODE_0);
	UPDATE_FIELD(OFF_CACHE_MODE_1, _REG_CACHE_MODE_1);
	UPDATE_FIELD(OFF_INSTPM, INSTPM);
	UPDATE_FIELD(OFF_EXCC, _REG_RCS_EXCC);
	UPDATE_FIELD(OFF_MI_MODE, MI_MODE);
}

static bool vgt_setup_rsvd_ring(struct vgt_rsvd_ring *ring)
{
	struct pgt_device *pdev = ring->pdev;
	u32 head;
	int id = ring->id;

	ring->head = 0;
	ring->tail = 0;

	disable_ring(pdev, id);

	/* execute our ring */
	VGT_WRITE_HEAD(pdev, id, 0);
	VGT_WRITE_TAIL(pdev, id, 0);

	head = VGT_READ_HEAD(pdev, id);
	if (head != 0) {
		VGT_WRITE_HEAD(pdev, id, 0);
	}

	VGT_WRITE_START(pdev, id, ring->start);

	enable_ring(pdev, id, ((ring->size - PAGE_SIZE) & 0x1FF000) | 1);

	if (wait_for_atomic(((VGT_READ_CTL(pdev, id) & 1) != 0 &&
			VGT_READ_START(pdev, id) == ring->start &&
			(VGT_READ_HEAD(pdev, id) & RB_HEAD_OFF_MASK) == 0),
			VGT_RING_TIMEOUT)) {
		vgt_err("Timeout setup rsvd ring-%d for %dms\n",
			VGT_RING_TIMEOUT, id);
		return false;
	}

	vgt_dbg(VGT_DBG_RENDER, "start vgt ring at 0x%x\n", ring->start);
	return true;
}

/*
 * ring buffer usage in vGT driver is simple. We allocate a ring buffer
 * big enough to contain all emitted CMDs in a render context switch,
 * and thus emit function is implemented simply by sequentially advancing
 * tail point, w/o the wrap handling requirement.
 */
static inline void vgt_ring_emit(struct vgt_rsvd_ring *ring,
				u32 data)
{
	ASSERT(ring->tail + 4 < ring->size);
	writel(data, ring->virtual_start + ring->tail);
	ring->tail += 4;
}

static void vgt_ring_emit_cmds(struct vgt_rsvd_ring *ring, char *buf, int size)
{
	ASSERT(ring->tail + size < ring->size);
	memcpy(ring->virtual_start + ring->tail, buf, size);
	ring->tail += size;
}

static void vgt_ring_advance(struct vgt_rsvd_ring *ring)
{
	ring->tail &= ring->size - 1;
	VGT_WRITE_TAIL(ring->pdev, ring->id, ring->tail);
}

static bool gen7_init_null_context(struct pgt_device *pdev, int id)
{
	struct vgt_rsvd_ring *ring = &pdev->ring_buffer[id];
	int i;
	vgt_reg_t	ccid;

	if (!IS_HSW(pdev))
		return false;

	/* only RCS support HW context on HSW */
	if ((id != RING_BUFFER_RCS) || ring->null_context)
		return true;

	/* assume no active usage so far */
	ccid = VGT_MMIO_READ (pdev, CCID);
	ASSERT(ccid == 0);
	ASSERT(!VGT_READ_TAIL(pdev, id));

	/* current ring buffer is dom0's. so switch first */
	if (!vgt_setup_rsvd_ring(ring))
		goto err;

	ring->null_context = aperture_2_gm(pdev,
			rsvd_aperture_alloc(pdev, SZ_CONTEXT_AREA_PER_RING));
	ring->indirect_state = aperture_2_gm(pdev,
			rsvd_aperture_alloc(pdev, SZ_INDIRECT_STATE));
	memcpy((char *)v_aperture(pdev, ring->indirect_state),
	       (char *)hsw_null_indirect_state,
	       sizeof(hsw_null_indirect_state));

	/* Make NULL context active */
	vgt_ring_emit(ring, MI_ARB_ON_OFF | MI_ARB_DISABLE);
	vgt_ring_emit(ring, MI_SET_CONTEXT);
	vgt_ring_emit(ring, ring->null_context |
			    MI_MM_SPACE_GTT |
			    MI_SAVE_EXT_STATE_EN |
			    MI_RESTORE_INHIBIT);
	vgt_ring_emit(ring, MI_NOOP);
	vgt_ring_emit(ring, MI_ARB_ON_OFF | MI_ARB_ENABLE);
	/* 64 Byte alignment */
	for (i = 5; i < 16; i++)
		vgt_ring_emit(ring, MI_NOOP);

	hsw_null_context_cmds[0x18/4] = 0x12400001;
	hsw_null_context_cmds[(0x18 + 0x4)/4] = 0x138064;
	hsw_null_context_cmds[(0x18 + 0x8)/4] = ring->indirect_state + 0x124;
	hsw_null_context_cmds[(0x684 + 0x4)/4] = ring->indirect_state | 0x1;
	hsw_null_context_cmds[(0x684 + 0x8)/4] = ring->indirect_state | 0x1;
	hsw_null_context_cmds[(0x684 + 0xc)/4] = ring->indirect_state | 0x1;
	hsw_null_context_cmds[(0x684 + 0x14)/4] = ring->indirect_state | 0x1;
	vgt_ring_emit_cmds(ring, (char *)hsw_null_context_cmds,
		sizeof(hsw_null_context_cmds));

#if 0
	{
		char *p_contents = ring->virtual_start;
		int i;
		for (i = 0; i < ring->tail/4; i++) {
			if (!(i % 8))
				printk("\n[%08x]:", i * 4);
			printk(" %8x", *((u32*)p_contents + i));
		}
		printk("\n");
	}
#endif

	vgt_ring_advance(ring);
	if (!idle_render_engine(pdev, id)) {
		vgt_err("Ring-%d hang in null context init\n", id);
		goto err;
	}

	vgt_ring_emit(ring, GFX_OP_PIPE_CONTROL(5));
	vgt_ring_emit(ring, PIPE_CONTROL_CS_STALL |
			    PIPE_CONTROL_MEDIA_STATE_CLEAR |
			    PIPE_CONTROL_RENDER_TARGET_CACHE_FLUSH |
			    PIPE_CONTROL_STATE_CACHE_INVALIDATE);
	vgt_ring_emit(ring, 0);
	vgt_ring_emit(ring, 0);
	vgt_ring_emit(ring, 0);
	/* save internal state to NULL context */
	vgt_ring_emit(ring, MI_ARB_ON_OFF | MI_ARB_DISABLE);
	vgt_ring_emit(ring, MI_SET_CONTEXT);
	vgt_ring_emit(ring, 0 |
			    MI_MM_SPACE_GTT |
			    MI_RESTORE_INHIBIT);
	vgt_ring_emit(ring, MI_NOOP);
	vgt_ring_emit(ring, MI_ARB_ON_OFF | MI_ARB_ENABLE);
	/* make sure no active context after this point */
	vgt_ring_emit(ring, MI_LRI_CMD |
			    MI_LRI_BYTE1_DISABLE |
			    MI_LRI_BYTE2_DISABLE |
			    MI_LRI_BYTE3_DISABLE);
	vgt_ring_emit(ring, CCID);
	vgt_ring_emit(ring, 0);
	vgt_ring_emit(ring, GFX_OP_PIPE_CONTROL(5));
	vgt_ring_emit(ring, PIPE_CONTROL_CS_STALL |
			    PIPE_CONTROL_RENDER_TARGET_CACHE_FLUSH |
			    PIPE_CONTROL_INDIRECT_STATE_DISABLE);
	vgt_ring_emit(ring, 0);
	vgt_ring_emit(ring, 0);
	vgt_ring_emit(ring, 0);

	vgt_ring_advance(ring);
	if (!idle_render_engine(pdev, id)) {
		vgt_err("Ring-%d hang in saving null context\n", id);
		goto err;
	}

	ccid = VGT_MMIO_READ (pdev, CCID);
	if (ccid != 0) {
		vgt_err("Fail to invalidate CCID after null context init\n");
		goto err;
	}

	/* Then recover dom0's ring structure */
	if (!stop_ring(pdev, id))
		goto err;
	vgt_restore_ringbuffer(vgt_dom0, id);
	start_ring(pdev, id);

	/* Update dom0's initial context area */
	memcpy((char *)v_aperture(pdev, vgt_dom0->rb[id].context_save_area),
	       (char *)v_aperture(pdev, ring->null_context),
	       SZ_CONTEXT_AREA_PER_RING);
	return true;

err:
	ring->null_context = 0;
	ring->indirect_state = 0;
	vgt_err("NULL context initialization fails!\n");
	return false;
}

static bool gen7_save_hw_context(int id, struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_state_ring_t *rb = &vgt->rb[id];
	struct vgt_rsvd_ring *ring = &pdev->ring_buffer[id];
	vgt_reg_t	ccid, new_ccid;

	if (test_bit(RESET_INPROGRESS, &vgt->reset_flags))
		return true;

	/* pipeline flush */
	vgt_ring_emit(ring, GFX_OP_PIPE_CONTROL(5));
	vgt_ring_emit(ring, PIPE_CONTROL_CS_STALL |
			    PIPE_CONTROL_TLB_INVALIDATE |
			    PIPE_CONTROL_FLUSH_ENABLE);
	vgt_ring_emit(ring, 0);
	vgt_ring_emit(ring, 0);
	vgt_ring_emit(ring, 0);

	vgt_ring_emit(ring, GFX_OP_PIPE_CONTROL(5));
	vgt_ring_emit(ring, PIPE_CONTROL_RENDER_TARGET_CACHE_FLUSH |
			    PIPE_CONTROL_FLUSH_ENABLE |
			    PIPE_CONTROL_VF_CACHE_INVALIDATE |
			    PIPE_CONTROL_CONST_CACHE_INVALIDATE |
			    PIPE_CONTROL_STATE_CACHE_INVALIDATE);
	vgt_ring_emit(ring, 0);
	vgt_ring_emit(ring, 0);
	vgt_ring_emit(ring, 0);

#if 0
	/*
	 * Activate XenGT context for prev
	 * Guest may have an active context already. Better to not clobber
	 * that area, and instead have full control on the context save
	 * area directly in XenGT driver.
	 */
	ccid = rb->context_save_area |
	       CCID_EXTENDED_STATE_SAVE_ENABLE |
	       CCID_EXTENDED_STATE_RESTORE_ENABLE |
	       CCID_VALID;
	vgt_ring_emit(ring, MI_LRI_CMD);
	vgt_ring_emit(ring, CCID);
	vgt_ring_emit(ring, ccid);

	/* pipeline flush */
	vgt_ring_emit(ring, GFX_OP_PIPE_CONTROL(5));
	vgt_ring_emit(ring, PIPE_CONTROL_CS_STALL |
			    PIPE_CONTROL_TLB_INVALIDATE |
			    PIPE_CONTROL_FLUSH_ENABLE |
			    PIPE_CONTROL_QW_WRITE |
			    PIPE_CONTROL_GLOBAL_GTT_IVB);
	vgt_ring_emit(ring, vgt_data_ctx_magic(pdev));
	vgt_ring_emit(ring, ++pdev->magic);
	vgt_ring_emit(ring, 0);

	vgt_ring_emit(ring, MI_NOOP);
	vgt_ring_emit(ring, MI_NOOP);
	vgt_ring_emit(ring, MI_NOOP);

	/* submit cmds */
	vgt_ring_advance(ring);

	if (!ring_wait_for_completion(pdev, id)) {
		vgt_err("change CCID to XenGT save context: commands unfinished\n");
		return false;
	}

	if (VGT_MMIO_READ(pdev, CCID) != ccid) {
		vgt_err("change CCID to XenGT save context: fail [%x, %x]\n",
			VGT_MMIO_READ(pdev, CCID), ccid);
		return false;
	}

	/* Save context and switch to NULL context */
	vgt_ring_emit(ring, MI_ARB_ON_OFF | MI_ARB_DISABLE);
	vgt_ring_emit(ring, MI_SET_CONTEXT);
	vgt_ring_emit(ring, ring->null_context |
			    MI_MM_SPACE_GTT |
			    MI_SAVE_EXT_STATE_EN |
			    MI_RESTORE_EXT_STATE_EN |
			    MI_FORCE_RESTORE);
	vgt_ring_emit(ring, MI_NOOP);
	vgt_ring_emit(ring, MI_ARB_ON_OFF | MI_ARB_ENABLE);
#else
	/* FIXME: too many CCID changes looks not working. So
	 * fall back to original style by using guest context directly
	 */
	if (vgt->has_context) {
		rb->active_vm_context = VGT_MMIO_READ(pdev, CCID);
		rb->active_vm_context &= 0xfffff000;
	}

	vgt_ring_emit(ring, MI_ARB_ON_OFF | MI_ARB_DISABLE);
	vgt_ring_emit(ring, MI_SET_CONTEXT);
	vgt_ring_emit(ring, rb->context_save_area |
			    MI_RESTORE_INHIBIT |
			    MI_SAVE_EXT_STATE_EN |
			    MI_RESTORE_EXT_STATE_EN);
	vgt_ring_emit(ring, MI_NOOP);
	vgt_ring_emit(ring, MI_ARB_ON_OFF | MI_ARB_ENABLE);
#endif

	/* pipeline flush */
	vgt_ring_emit(ring, GFX_OP_PIPE_CONTROL(5));
	vgt_ring_emit(ring, PIPE_CONTROL_CS_STALL |
			    PIPE_CONTROL_TLB_INVALIDATE |
			    PIPE_CONTROL_FLUSH_ENABLE |
			    PIPE_CONTROL_MEDIA_STATE_CLEAR |
			    PIPE_CONTROL_DC_FLUSH_ENABLE |
			    PIPE_CONTROL_RENDER_TARGET_CACHE_FLUSH |
			    PIPE_CONTROL_QW_WRITE |
			    PIPE_CONTROL_GLOBAL_GTT_IVB);
	vgt_ring_emit(ring, vgt_data_ctx_magic(pdev));
	vgt_ring_emit(ring, ++pdev->magic);
	vgt_ring_emit(ring, 0);

	vgt_ring_emit(ring, MI_NOOP);
	vgt_ring_emit(ring, MI_NOOP);
	vgt_ring_emit(ring, MI_NOOP);

	/* submit cmds */
	vgt_ring_advance(ring);

	if (!ring_wait_for_completion(pdev, id)) {
		vgt_err("save context commands unfinished\n");
		return false;
	}

	ccid = VGT_MMIO_READ (pdev, CCID);
#if 0
	new_ccid = ring->null_context;
#else
	new_ccid = rb->context_save_area;
#endif
	if ((ccid & GTT_PAGE_MASK) != (new_ccid & GTT_PAGE_MASK)) {
		vgt_err("vGT: CCID isn't changed [%x, %lx]\n", ccid, (unsigned long)new_ccid);
		return false;
	}

	return true;
}

static struct reg_mask_t rcs_reset_mmio[] = {
	{0x4080, 0},
	{0x2134, 0},
	{0x20c0, 1},
	{0x20a8, 0},

	{0x7000, 1},
	{0x209c, 1},
	{0x2090, 1},
	{0x9400, 0},

	{0x9404, 0},
	{0x42000, 0},
	{0x42020, 0},
	{0x902c, 0},

	{0x4090, 0},
	{0x9424, 0},
	{0x229c, 1},
	{0x2044, 0},

	{0x20a0, 0},
	{0x20e4, 1},
	{0x7004, 1},
	{0x20dc, 1},

	{0x2220, 0},
	{0x2228, 0},
	{0x2180, 0},

	{0x2054, 0},
};

static bool vgt_reset_engine(struct pgt_device *pdev, int id)
{
	int i;
	vgt_reg_t head, tail, start, ctl;
	vgt_reg_t val, val1;

	if (id != RING_BUFFER_RCS) {
		vgt_err("ring-%d reset unsupported\n", id);
		return false;
	}

	/* save reset context */
	for (i = 0; i < ARRAY_NUM(rcs_reset_mmio); i++) {
		struct reg_mask_t *r = &rcs_reset_mmio[i];

		if (r->reg == 0x2220 || r->reg == 0x2228)
			r->val = VGT_MMIO_READ(pdev, r->reg + 0x10000);
		else
			r->val = VGT_MMIO_READ(pdev, r->reg);

		if (r->mask)
			r->val |= 0xFFFF0000;
	}

	head = VGT_READ_HEAD(pdev, id);
	tail = VGT_READ_TAIL(pdev, id);
	start = VGT_READ_START(pdev, id);
	ctl = VGT_READ_CTL(pdev, id);

	/* trigger engine specific reset */
	VGT_MMIO_WRITE(pdev, GEN6_GDRST, GEN6_GRDOM_RENDER);

#define GDRST_COUNT 0x1000
	/* wait for reset complete */
	for (i = 0; i < GDRST_COUNT; i++) {
		if (!(VGT_MMIO_READ(pdev, GEN6_GDRST) &
			GEN6_GRDOM_RENDER))
			break;
	}

	if (i == GDRST_COUNT) {
		vgt_err("ring-%d engine reset incomplete\n", id);
		return false;
	}

	/* restore reset context */
	for (i = 0; i < ARRAY_NUM(rcs_reset_mmio); i++) {
		struct reg_mask_t *r = &rcs_reset_mmio[i];

		VGT_MMIO_WRITE(pdev, r->reg, r->val);
	}

	VGT_WRITE_CTL(pdev, id, 0);
	VGT_WRITE_START(pdev, id, start);
	VGT_WRITE_HEAD(pdev, id, head);
	VGT_WRITE_TAIL(pdev, id, tail);

	val = VGT_MMIO_READ(pdev, 0x2214);
	val &= 0xFFFFFFFE;
	val1 = VGT_MMIO_READ(pdev, 0x138064);
	if (val1 & 0x3) {
		if (val1 & 0x1)
			val |= 0x1;
	} else if (val1 & 0x8) {
		val |= 0x1;
	}
	VGT_MMIO_WRITE(pdev, 0x2214, val);

	VGT_WRITE_CTL(pdev, id, ctl);
	VGT_POST_READ_TAIL(pdev, id);
	VGT_POST_READ_HEAD(pdev, id);
	VGT_POST_READ_START(pdev, id);
	VGT_POST_READ_CTL(pdev, id);

	return true;
}

static bool gen7_restore_hw_context(int id, struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_state_ring_t	*rb = &vgt->rb[id];
	struct vgt_rsvd_ring *ring = &pdev->ring_buffer[id];

	/* sync between vReg and saved context */
	//update_context(vgt, rb->context_save_area);

	/* pipeline flush */
	vgt_ring_emit(ring, GFX_OP_PIPE_CONTROL(5));
	vgt_ring_emit(ring, PIPE_CONTROL_CS_STALL |
			    PIPE_CONTROL_TLB_INVALIDATE |
			    PIPE_CONTROL_FLUSH_ENABLE);
	vgt_ring_emit(ring, 0);
	vgt_ring_emit(ring, 0);
	vgt_ring_emit(ring, 0);

#if 0
	/*
	 * we don't want to clobber the null context. so invalidate
	 * the current context before restoring next instance
	 */
	vgt_ring_emit(ring, MI_LRI_CMD |
			    MI_LRI_BYTE1_DISABLE |
			    MI_LRI_BYTE2_DISABLE |
			    MI_LRI_BYTE3_DISABLE);
	vgt_ring_emit(ring, CCID);
	vgt_ring_emit(ring, 0);

	/* pipeline flush */
	vgt_ring_emit(ring, GFX_OP_PIPE_CONTROL(5));
	vgt_ring_emit(ring, PIPE_CONTROL_CS_STALL |
			    PIPE_CONTROL_TLB_INVALIDATE |
			    PIPE_CONTROL_FLUSH_ENABLE |
			    PIPE_CONTROL_QW_WRITE |
			    PIPE_CONTROL_GLOBAL_GTT_IVB);
	vgt_ring_emit(ring, vgt_data_ctx_magic(pdev));
	vgt_ring_emit(ring, ++pdev->magic);
	vgt_ring_emit(ring, 0);

	vgt_ring_emit(ring, MI_NOOP);
	vgt_ring_emit(ring, MI_NOOP);
	vgt_ring_emit(ring, MI_NOOP);

	/* submit cmds */
	vgt_ring_advance(ring);

	if (!ring_wait_for_completion(pdev, id)) {
		vgt_err("Invalidate CCID after NULL restore: commands unfinished\n");
		return false;
	}

	if (VGT_MMIO_READ(pdev, CCID) != 0) {
		vgt_err("Invalidate CCID after NULL restore: fail [%x, %x]\n",
			VGT_MMIO_READ(pdev, CCID), 0);
		return false;
	}
#endif

	/* restore HW context */
	vgt_ring_emit(ring, MI_ARB_ON_OFF | MI_ARB_DISABLE);
	vgt_ring_emit(ring, MI_SET_CONTEXT);
	vgt_ring_emit(ring, rb->context_save_area |
			    MI_MM_SPACE_GTT |
			    MI_SAVE_EXT_STATE_EN |
			    MI_RESTORE_EXT_STATE_EN |
			    MI_FORCE_RESTORE);
	vgt_ring_emit(ring, MI_NOOP);
	vgt_ring_emit(ring, MI_ARB_ON_OFF | MI_ARB_ENABLE);

#if 0
	vgt_ring_emit(ring, DUMMY_3D);
	vgt_ring_emit(ring, PRIM_TRILIST);
	vgt_ring_emit(ring, 0);
	vgt_ring_emit(ring, 0);
	vgt_ring_emit(ring, 0);
	vgt_ring_emit(ring, 0);
	vgt_ring_emit(ring, 0);
	vgt_ring_emit(ring, MI_NOOP);
#endif

	/* pipeline flush */
	vgt_ring_emit(ring, GFX_OP_PIPE_CONTROL(5));
	vgt_ring_emit(ring, PIPE_CONTROL_CS_STALL |
			    PIPE_CONTROL_TLB_INVALIDATE |
			    PIPE_CONTROL_FLUSH_ENABLE |
			    PIPE_CONTROL_MEDIA_STATE_CLEAR |
			    PIPE_CONTROL_QW_WRITE |
			    PIPE_CONTROL_GLOBAL_GTT_IVB);
	vgt_ring_emit(ring, vgt_data_ctx_magic(pdev));
	vgt_ring_emit(ring, ++pdev->magic);
	vgt_ring_emit(ring, 0);

	vgt_ring_emit(ring, MI_NOOP);
	vgt_ring_emit(ring, MI_NOOP);
	vgt_ring_emit(ring, MI_NOOP);

	/* submit CMDs */
	vgt_ring_advance(ring);

	if (!ring_wait_for_completion(pdev, id)) {
		vgt_err("restore context switch commands unfinished\n");
		return false;
	}

#if 0
	/* then restore current context to whatever VM expects */
	vgt_ring_emit(ring, MI_LRI_CMD);
	vgt_ring_emit(ring, CCID);
	vgt_ring_emit(ring, __vreg(vgt, CCID));

	/* pipeline flush */
	vgt_ring_emit(ring, GFX_OP_PIPE_CONTROL(5));
	vgt_ring_emit(ring, PIPE_CONTROL_CS_STALL |
			    PIPE_CONTROL_TLB_INVALIDATE |
			    PIPE_CONTROL_FLUSH_ENABLE |
			    PIPE_CONTROL_QW_WRITE |
			    PIPE_CONTROL_GLOBAL_GTT_IVB);
	vgt_ring_emit(ring, vgt_data_ctx_magic(pdev));
	vgt_ring_emit(ring, ++pdev->magic);
	vgt_ring_emit(ring, 0);

	vgt_ring_emit(ring, MI_NOOP);
	vgt_ring_emit(ring, MI_NOOP);
	vgt_ring_emit(ring, MI_NOOP);

	/* submit CMDs */
	vgt_ring_advance(ring);

	if (!ring_wait_for_completion(pdev, id)) {
		vgt_err("Restore VM CCID: commands unfinished\n");
		return false;
	}

	if (VGT_MMIO_READ(pdev, CCID) != __vreg(vgt, CCID)) {
		vgt_err("Restore VM CCID: fail [%x, %x]\n",
			VGT_MMIO_READ(pdev, CCID),
			__vreg(vgt, CCID));
		return false;
	}
#else
	if (vgt->has_context && rb->active_vm_context) {
		vgt_ring_emit(ring, MI_ARB_ON_OFF | MI_ARB_DISABLE);
		vgt_ring_emit(ring, MI_SET_CONTEXT);
		vgt_ring_emit(ring, rb->active_vm_context |
				MI_MM_SPACE_GTT |
				MI_SAVE_EXT_STATE_EN |
				MI_RESTORE_EXT_STATE_EN |
				MI_FORCE_RESTORE);
		vgt_ring_emit(ring, MI_NOOP);
		vgt_ring_emit(ring, MI_ARB_ON_OFF | MI_ARB_ENABLE);

		vgt_ring_emit(ring, DUMMY_3D);
		vgt_ring_emit(ring, PRIM_TRILIST);
		vgt_ring_emit(ring, 0);
		vgt_ring_emit(ring, 0);
		vgt_ring_emit(ring, 0);
		vgt_ring_emit(ring, 0);
		vgt_ring_emit(ring, 0);
		vgt_ring_emit(ring, MI_NOOP);

		vgt_ring_emit(ring, MI_STORE_DATA_IMM | MI_SDI_USE_GTT);
		vgt_ring_emit(ring, 0);
		vgt_ring_emit(ring, vgt_data_ctx_magic(pdev));
		vgt_ring_emit(ring, ++pdev->magic);
		vgt_ring_emit(ring, 0);
		vgt_ring_emit(ring, 0);
		vgt_ring_emit(ring, MI_NOOP);

		vgt_ring_advance(ring);

		if (!ring_wait_for_completion(pdev, id)) {
			vgt_err("change to VM context switch commands unfinished\n");
			return false;
		}
	}
#endif
	return true;
}

bool gen7_ring_switch(struct pgt_device *pdev,
		enum vgt_ring_id ring_id,
		struct vgt_device *prev,
		struct vgt_device *next)
{
	bool rc = false;
	struct vgt_rsvd_ring *ring = &pdev->ring_buffer[ring_id];

	/* STEP-a: stop the ring */
	if (!stop_ring(pdev, ring_id)) {
		vgt_err("Fail to stop ring (1st)\n");
		goto out;
	}
	/* STEP-b: save current ring buffer structure */
	vgt_save_ringbuffer(prev, ring_id);

	if (ring->stateless) {
		rc = true;
		goto out;
	}

	/* STEP-c: switch to vGT ring buffer */
	if (!vgt_setup_rsvd_ring(ring)) {
		vgt_err("Fail to setup rsvd ring\n");
		goto out;
	}

	start_ring(pdev, ring_id);

	/* STEP-d: save HW render context for prev */
	if (!gen7_save_hw_context(ring_id, prev)) {
		vgt_err("Fail to save context\n");
		goto out;
	}

	if (render_engine_reset && !vgt_reset_engine(pdev, ring_id)) {
		vgt_err("Fail to reset engine\n");
		goto out;
	}

	/* STEP-e: restore HW render context for next */
	if (!gen7_restore_hw_context(ring_id, next)) {
		vgt_err("Fail to restore context\n");
		goto out;
	}

	/* STEP-f: idle and stop ring at the end of HW switch */
	if (!idle_render_engine(pdev, ring_id)) {
		vgt_err("fail to idle ring-%d after ctx restore\n", ring_id);
		goto out;
	}

	if (!stop_ring(pdev, ring_id)) {
		vgt_err("Fail to stop ring (2nd)\n");
		goto out;
	}

	rc = true;
out:
	return rc;
}

bool ring_mmio_read_in_rb_mode(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int ring_id, rel_off;
	vgt_ringbuffer_t	*vring;
	struct vgt_statistics *stat = &vgt->stat;

	stat->ring_mmio_rcnt++;

	if ((hvm_render_owner && (vgt->vm_id != 0)) || reg_hw_access(vgt, off)){
		unsigned long data;
		data = VGT_MMIO_READ_BYTES(vgt->pdev, off, bytes);
		memcpy(p_data, &data, bytes);
		return true;
	}

	rel_off = off & ( sizeof(vgt_ringbuffer_t) - 1 );
	ring_id = tail_to_ring_id (vgt->pdev, _tail_reg_(off) );
	vring = &vgt->rb[ring_id].vring;

	memcpy(p_data, (char *)vring + rel_off, bytes);
	//ring_debug(vgt, ring_id);
	return true;
}

bool ring_mmio_write_in_rb_mode(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int ring_id, rel_off;
	vgt_state_ring_t	*rs;
	vgt_ringbuffer_t	*vring;
	vgt_ringbuffer_t	*sring;
	struct vgt_tailq *tailq = NULL;
	vgt_reg_t	oval;
	cycles_t	t0, t1;
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_statistics *stat = &vgt->stat;

	t0 = get_cycles();
	stat->ring_mmio_wcnt++;

	vgt_dbg(VGT_DBG_RENDER, "vGT:ring_mmio_write (0x%x) with val (0x%x)\n", off, *((u32 *)p_data));
	rel_off = off & ( sizeof(vgt_ringbuffer_t) - 1 );

	ring_id = tail_to_ring_id (pdev, _tail_reg_(off) );
	rs = &vgt->rb[ring_id];
	vring = &rs->vring;
	sring = &rs->sring;

	if (shadow_tail_based_qos)
		tailq = &vgt->rb_tailq[ring_id];

	if (ring_id == RING_BUFFER_VECS)
		vgt->vebox_support = 1;

	oval = *(vgt_reg_t *)((char *)vring + rel_off);
	memcpy((char *)vring + rel_off, p_data, bytes);

	switch (rel_off) {
	case RB_OFFSET_TAIL:
		stat->ring_tail_mmio_wcnt++;

		/* enable hvm tailq after the ring enabled */
		if (shadow_tail_based_qos) {
			if (test_bit(ring_id, vgt->enabled_rings))
				vgt_tailq_pushback(tailq, vring->tail, 0);
		} else
			sring->tail = vring->tail;

#if 0
		if (shadow_tail_based_qos) {
			if (vgt->vgt_id > 0) {
				if (enable_hvm_tailq && !vgt->force_removal)
					vgt_tailq_pushback(tailq, vring->tail, 0);
			} else
				vgt_tailq_pushback(tailq, vring->tail, 0);
		} else
			sring->tail = vring->tail;
#endif

		if (vring->ctl & _RING_CTL_ENABLE)
			vgt_scan_vring(vgt, ring_id);

		t1 = get_cycles();
		stat->ring_tail_mmio_wcycles += (t1-t0);

		if (shadow_tail_based_qos) {
			if (vgt_tailq_last_stail(tailq)
					&& !test_and_set_bit(ring_id, (void *)vgt->started_rings))
				printk("Ring-%d starts work for vgt-%d\n",
						ring_id, vgt->vgt_id);
			/* When a ring is enabled, tail value
			 * can never write to real hardware */
			return true;
		} else {
			if (sring->tail &&
					!test_and_set_bit(ring_id, (void *)vgt->started_rings))
				printk("Ring-%d starts work for vgt-%d\n",
						ring_id, vgt->vgt_id);
		}

		break;
	case RB_OFFSET_HEAD:
		//debug
		//vring->head |= 0x200000;
		sring->head = vring->head;
		break;
	case RB_OFFSET_START:
		sring->start = mmio_g2h_gmadr(vgt, off, vring->start);
		break;
	case RB_OFFSET_CTL:
		sring->ctl = vring->ctl;

		if ( (oval & _RING_CTL_ENABLE) &&
			!(vring->ctl & _RING_CTL_ENABLE) ) {
			printk("vGT: deactivate vgt (%d) on ring (%d)\n", vgt->vgt_id, ring_id);
			vgt_disable_ring(vgt, ring_id);
		}
		else if ( !(oval & _RING_CTL_ENABLE) &&
			(vring->ctl & _RING_CTL_ENABLE) ) {
			printk("vGT: activate vgt (%d) on ring (%d)\n", vgt->vgt_id, ring_id);
			vgt_enable_ring(vgt, ring_id);
			/*
			 * We rely on dom0 to init the render engine.
			 * So wait until dom0 enables ring for the 1st time,
			 * and then we can initialize the null context safely
			 */
			if (!hvm_render_owner && !pdev->ring_buffer[ring_id].null_context) {
				if (!gen7_init_null_context(pdev, ring_id))
					return false;
			}

			clear_bit(RESET_INPROGRESS, &vgt->reset_flags);
		}
		if (vring->ctl & _RING_CTL_ENABLE) {
			rs->last_scan_head =
				vring->head & RB_HEAD_OFF_MASK;
			vgt_scan_vring(vgt, ring_id);
		}
		break;
	default:
		return false;
		break;
	}

	/* TODO: lock with kthread? */
	/*
	 * FIXME: Linux VM doesn't read head register directly. Instead it relies on
	 * automatic head reporting mechanism. Later with command parser, there's no
	 * problem since all commands are translated and filled by command parser. for
	 * now it's possible for dom0 to fill over than a full ring in a scheduled
	 * quantum
	 */
	if (reg_hw_access(vgt, off)) {
		if (rel_off == RB_OFFSET_TAIL && (vring->ctl & _RING_CTL_ENABLE))
			vgt_submit_commands(vgt, ring_id);
		else
			VGT_MMIO_WRITE(pdev, off,
				*(vgt_reg_t*)((char *)sring + rel_off));
	}

	//ring_debug(vgt, ring_id);
	return true;
}

bool ring_uhptr_write_in_rb_mode(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	int ring_id;
	vgt_state_ring_t	*rs;
	vgt_reg_t val = *(vgt_reg_t *)p_data;

	switch (off) {
	case _REG_RCS_UHPTR:
		ring_id = RING_BUFFER_RCS;
		break;
	case _REG_VCS_UHPTR:
		ring_id = RING_BUFFER_VCS;
		break;
	case _REG_BCS_UHPTR:
		ring_id = RING_BUFFER_BCS;
		break;
	case _REG_VECS_UHPTR:
		ring_id = RING_BUFFER_VECS;
		break;
	default:
		ASSERT(0);
		break;
	}

	rs = &vgt->rb[ring_id];

	/* only cache the latest value */
	if (rs->uhptr & _REGBIT_UHPTR_VALID)
		vgt_info("VM(%d)-r%d: overwrite a valid uhptr (o:%x, n:%x)\n",
			vgt->vm_id, ring_id, rs->uhptr, val);

	rs->uhptr = val;
	rs->uhptr_id = rs->request_id;
	return true;
}

/* restore ring buffer structures to a empty state (head==tail) */
void vgt_restore_ringbuffer(struct vgt_device *vgt, int id)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_ringbuffer_t *srb = &vgt->rb[id].sring;
	struct vgt_rsvd_ring *ring = &pdev->ring_buffer[id];

	if (pdev->enable_execlist)
		return;

	if (!ring->need_switch)
		return;

	vgt_dbg(VGT_DBG_RENDER, "restore ring: [%x, %x, %x, %x] \n", srb->head, srb->tail,
		VGT_READ_HEAD(pdev, id),
		VGT_READ_TAIL(pdev, id));

	disable_ring(pdev, id);

	VGT_WRITE_START(pdev, id, srb->start);

	/* make head==tail when enabling the ring buffer */
	VGT_WRITE_HEAD(pdev, id, srb->head);
	VGT_WRITE_TAIL(pdev, id, srb->head);

	restore_ring_ctl(pdev, id, srb->ctl);
	/*
	 * FIXME: One weird issue observed when switching between dom0
	 * and win8 VM. The video ring #1 is not used by both dom0 and
	 * win8 (head=tail=0), however sometimes after switching back
	 * to win8 the video ring may enter a weird state that VCS cmd
	 * parser continues to parse the whole ring (fulled with ZERO).
	 * Sometimes it ends for one whole loop when head reaches back
	 * to 0. Sometimes it may parse indefinitely so that there's
	 * no way to wait for the ring empty.
	 *
	 * Add a posted read works around the issue. In the future we
	 * can further optimize by not switching unused ring.
	 */
	VGT_POST_READ_HEAD(pdev, id);
	vgt_dbg(VGT_DBG_RENDER, "restore ring: [%x, %x]\n",
		VGT_READ_HEAD(pdev, id),
		VGT_READ_TAIL(pdev, id));
}

void vgt_kick_off_ringbuffers(struct vgt_device *vgt)
{
	int i;
	struct pgt_device *pdev = vgt->pdev;

	for (i = 0; i < pdev->max_engines; i++) {
		struct vgt_rsvd_ring *ring = &pdev->ring_buffer[i];

		if (!ring->need_switch)
			continue;

		start_ring(pdev, i);
		vgt_submit_commands(vgt, i);
	}
}
