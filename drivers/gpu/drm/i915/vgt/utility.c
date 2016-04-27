/*
 * Various utility helpers.
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

#include <linux/delay.h>

#include "vgt.h"
#include <drm/intel-gtt.h>
#include <asm/cacheflush.h>

bool inline is_execlist_mode(struct pgt_device *pdev, int ring_id)
{
	unsigned long ring_mode = RB_TAIL(pdev, ring_id) - 0x30 + 0x29c;

	return VGT_MMIO_READ(pdev, ring_mode) & _REGBIT_EXECLIST_ENABLE;
}

void show_debug(struct pgt_device *pdev)
{
	int i, cpu;

	printk("========vGT DEBUG INFO==========\n");
	for_each_online_cpu(cpu)
		printk("CPU[%d]: %s\n", cpu,
			per_cpu(in_vgt, cpu) ? "in vgt" : "out of vgt");
	printk("DE_RRMR: %x\n", VGT_MMIO_READ(pdev, _REG_DE_RRMR));

	for (i = 0; i < pdev->max_engines; i++) {
		printk("-----------ring-%d info-------------\n", i);
		show_ring_debug(pdev, i);
		show_ring_buffer(pdev, i, 16 * sizeof(vgt_reg_t));
	}
}

/*
 * Print debug registers for CP
 *
 * Hope to introduce a sysfs interface to dump this information on demand
 * in the future
 */
void common_show_ring_debug(struct pgt_device *pdev, int ring_id)
{
	printk("debug registers,reg maked with <*>"
		" may not apply to every ring):\n");
	printk("....RING_EIR: %08x\n", VGT_MMIO_READ(pdev, RING_EIR(ring_id)));
	printk("....RING_EMR: %08x\n", VGT_MMIO_READ(pdev, RING_EMR(ring_id)));
	printk("....RING_ESR: %08x\n", VGT_MMIO_READ(pdev, RING_ESR(ring_id)));

	if (ring_id)
		printk("....%08x*: %08x\n", RING_REG_2064(ring_id),
				VGT_MMIO_READ(pdev, RING_REG_2064(ring_id)));

	printk("....%08x: %08x\n", RING_REG_2068(ring_id),
		VGT_MMIO_READ(pdev, RING_REG_2068(ring_id)));
	printk("....ACTHD(active header): %08x\n",
			VGT_MMIO_READ(pdev, VGT_ACTHD(ring_id)));
	printk("....UHPTR(pending header): %08x\n",
			VGT_MMIO_READ(pdev, VGT_UHPTR(ring_id)));
	printk("....%08x: %08x\n", RING_REG_2078(ring_id),
		VGT_MMIO_READ(pdev, RING_REG_2078(ring_id)));

	if (!ring_id) {
		printk("....INSTPS* (parser state): %08x :\n",
				VGT_MMIO_READ(pdev, 0x2070));
		printk("....CSCMDOP* (instruction DWORD): %08x\n",
				VGT_MMIO_READ(pdev, 0x220C));
		printk("....CSCMDVLD* (command buffer valid): %08x\n",
				VGT_MMIO_READ(pdev, 0x2210));
	}

	printk("(informative)\n");
	printk("....INSTDONE_1(FYI): %08x\n",
			VGT_MMIO_READ(pdev, RING_REG_206C(ring_id)));
	if (!ring_id)
		printk("....INSTDONE_2*: %08x\n",
				VGT_MMIO_READ(pdev, 0x207C));
}

void legacy_show_ring_debug(struct pgt_device *pdev, int ring_id)
{
	int i;

	for (i = 0; i < VGT_MAX_VMS; i++) {
		struct vgt_device *vgt;
		if (pdev->device[i]) {
			vgt = pdev->device[i];
			if (vgt == current_render_owner(pdev))
				printk("VM%d(*):", vgt->vm_id);
			else
				printk("VM%d   :", vgt->vm_id);

			printk("head(%x), tail(%x), start(%x), ctl(%x), uhptr(%x)\n",
				vgt->rb[ring_id].sring.head,
				vgt->rb[ring_id].sring.tail,
				vgt->rb[ring_id].sring.start,
				vgt->rb[ring_id].sring.ctl,
				__vreg(vgt, VGT_UHPTR(ring_id)));
		}
	}

	common_show_ring_debug(pdev, ring_id);
}

void execlist_show_ring_debug(struct pgt_device *pdev, int ring_id)
{
	int i;

	for (i = 0; i < VGT_MAX_VMS; i++) {
		struct vgt_device *vgt;

		if (!pdev->device[i])
			continue;

		vgt = pdev->device[i];

		if (vgt == current_render_owner(pdev)) {
			printk("VM%d(*):\n", vgt->vm_id);
			printk("stat(us): sche_in %lld, sche_out %lld, last_vblank %lld\n",
				vgt->stat.schedule_in_time/(cpu_khz/1000),
				vgt->stat.schedule_out_time/(cpu_khz/1000),
				vgt->stat.last_vblank_time/(cpu_khz/1000));
		} else {
			printk("VM%d   :\n", vgt->vm_id);
			printk("stat(us): sche_in %lld, sche_out %lld, last_vblank %lld\n",
				vgt->stat.schedule_in_time/(cpu_khz/1000),
				vgt->stat.schedule_out_time/(cpu_khz/1000),
				vgt->stat.last_vblank_time/(cpu_khz/1000));
		}
	}

	common_show_ring_debug(pdev, ring_id);
}

void show_ring_debug(struct pgt_device *pdev, int ring_id)
{
	is_execlist_mode(pdev, ring_id) ?
		execlist_show_ring_debug(pdev, ring_id) :
		legacy_show_ring_debug(pdev, ring_id);
}

/*
 * Show some global register settings, if we care about bits
 * in those registers.
 *
 * normally invoked from initialization phase, and mmio emulation
 * logic
 */
void show_mode_settings(struct pgt_device *pdev)
{
	vgt_reg_t val;
	struct vgt_device *vgt1 = default_device.device[1];

	if (current_render_owner(pdev))
		printk("Current render owner: %d\n", current_render_owner(pdev)->vgt_id);

#define SHOW_MODE(reg)		\
	do{				\
		val = VGT_MMIO_READ(pdev, reg);	\
		printk("vGT: "#reg"(%x): p(%x), 0(%x), 1(%x)\n",	\
			reg, val, __sreg(vgt_dom0, reg), vgt1 ? __sreg(vgt1, reg) : 0);	\
	} while (0);
	SHOW_MODE(MI_MODE);
	SHOW_MODE(_REG_VCS_MI_MODE);
	SHOW_MODE(_REG_BCS_MI_MODE);

	if (IS_IVB(pdev) || IS_HSW(pdev)) {
		SHOW_MODE(GFX_MODE_GEN7);
		SHOW_MODE(_REG_BCS_BLT_MODE_IVB);
		SHOW_MODE(_REG_VCS_MFX_MODE_IVB);
		SHOW_MODE(CACHE_MODE_0_GEN7);
		SHOW_MODE(CACHE_MODE_1);
		SHOW_MODE(GEN7_GT_MODE);
	} else if (IS_BDWGT3(pdev) || IS_SKLGT3(pdev) || IS_SKLGT4(pdev)) {
		SHOW_MODE(_REG_VCS2_MI_MODE);
		SHOW_MODE(_REG_VCS2_MFX_MODE_BDW);
		SHOW_MODE(_REG_VCS2_INSTPM);
	} else if (IS_SNB(pdev)) {
		SHOW_MODE(GFX_MODE);
		SHOW_MODE(ARB_MODE);
		SHOW_MODE(GEN6_GT_MODE);
		SHOW_MODE(_REG_CACHE_MODE_0);
		SHOW_MODE(_REG_CACHE_MODE_1);
	}

	SHOW_MODE(INSTPM);
	SHOW_MODE(_REG_VCS_INSTPM);
	SHOW_MODE(_REG_BCS_INSTPM);

	SHOW_MODE(TILECTL);
}

static void show_batchbuffer(struct pgt_device *pdev, int ring_id, u64 addr,
	int bytes, int ppgtt)
{
	struct vgt_device_info *info = &pdev->device_info;
	int i;
	char *ip_va;
	u64 start;
	struct vgt_device *vgt = current_render_owner(pdev);
	uint32_t val;
	struct vgt_mm *mm;

	if (!vgt) {
		vgt_err("no render owner at hanging point\n");
		return;
	}

	addr &= ~0x1;

	if ((addr & 0xFFF) < bytes) {
		bytes *= 2;
		start = addr & ~0xFFF;
	} else if (!ppgtt && (addr + bytes) >= info->max_gtt_gm_sz) {
		bytes *= 2;
		start = info->max_gtt_gm_sz - bytes;
	} else {
		start = addr - bytes;
		bytes *= 2;
	}

	if (!ppgtt) {
		mm = vgt->gtt.ggtt_mm;
	} else if (is_execlist_mode(pdev, ring_id)) {
		struct execlist_context *el_ctx;
		u32 lrca = VGT_MMIO_READ(pdev, _REG_CUR_DESC(ring_id));
		bool has_shadow = vgt_require_shadow_context(vgt) &&
					(!hvm_render_owner) &&
					(shadow_execlist_context != PATCH_WITHOUT_SHADOW);

		lrca >>= GTT_PAGE_SHIFT;

		if (has_shadow)
			el_ctx = execlist_shadow_context_find(vgt, lrca);
		else
			el_ctx = execlist_context_find(vgt, lrca);
		if (!el_ctx) {
			printk("cannot find ctx with lrca 0x%x\n", lrca);
			return;
		}
		mm = el_ctx->ppgtt_mm;
	} else {
		mm = vgt->rb[ring_id].active_ppgtt_mm;
	}

	if (!mm) {
		printk("cannot find mm for dump batch\n");
		return;
	}

	printk("Batch buffer contents: \n");
	for (i = 0; i < bytes; i += 4) {
		ip_va = vgt_gma_to_va(mm, start + i);

		if (!(i % 32))
			printk("\n[%08llx]:", start + i);

		if (ip_va == NULL)
			printk(" %8s", "N/A");
		else {
			hypervisor_read_va(vgt, ip_va, &val, sizeof(val), 0);
			printk(" %08x", val);
		}
		if (start + i == addr)
			printk("(*)");
	}
	printk("\n");
}


void mmio_show_batchbuffer(struct pgt_device *pdev, int ring_id, int
		bytes)
{
	u32 bb_state, sbb_state;
	u64 bb_start, bb_head;
	u64 sbb_head;
	int ppgtt;

	bb_state = VGT_MMIO_READ(pdev, _REG_BB_STATE(ring_id));

	bb_head = VGT_MMIO_READ(pdev, _REG_BB_HEAD_U(ring_id));
	bb_head &= 0xFFFF;
	bb_head <<= 32;
	bb_head |= VGT_MMIO_READ(pdev, _REG_BB_HEAD(ring_id));

	bb_start = VGT_MMIO_READ(pdev, _REG_BB_START_U(ring_id));
	bb_start &= 0xFFFF;
	bb_start <<= 32;
	bb_start |= VGT_MMIO_READ(pdev, _REG_BB_START(ring_id));

	sbb_state = VGT_MMIO_READ(pdev, _REG_SBB_STATE(ring_id));

	sbb_head = VGT_MMIO_READ(pdev, _REG_SBB_HEAD_U(ring_id));
	sbb_head &= 0xFFFF;
	sbb_head <<= 32;
	sbb_head |= VGT_MMIO_READ(pdev, _REG_SBB_HEAD(ring_id));

	printk("mmio batch info: state: 0x%x, head:0x%llx, start:0x%llx\n",
			bb_state, bb_head, bb_start);

	printk("mmio batch info SBB: state: 0x%x, head:0x%llx\n", sbb_state,
			sbb_head);

	if (bb_head & 0x1) {
		printk("dumping batch buffer contents\n");
		ppgtt = bb_state & (1 << 5);
		bb_head &= ~0x1;
		show_batchbuffer(pdev, ring_id, bb_head, bytes,
				ppgtt);

		if (sbb_head & 0x1) {
			printk("dumping second level batch buffer:\n");
			sbb_head &= ~0x1;
			show_batchbuffer(pdev, ring_id, sbb_head, bytes,
					ppgtt);

		}
	}
}
/*
 * Given a ring buffer, print out the current data [-bytes, bytes]
 */
void common_show_ring_buffer(struct pgt_device *pdev, int ring_id, int bytes,
	vgt_reg_t p_tail, vgt_reg_t p_head, vgt_reg_t p_start, vgt_reg_t p_ctl,
	unsigned long batch_head)
{
	char *p_contents;
	int i;
	struct vgt_device *vgt = current_render_owner(pdev);
	u32 *cur;
	u64 ring_len, off;
	u32 gpa;

	if (pdev->cur_reset_vm)
		vgt = pdev->cur_reset_vm;

	printk("ring xxx:(%d), mi_mode idle:(%d)\n",
		VGT_MMIO_READ(pdev, pdev->ring_xxx[ring_id]) & (1 << pdev->ring_xxx_bit[ring_id]),
		VGT_MMIO_READ(pdev, pdev->ring_mi_mode[ring_id]) & MODE_IDLE);

	if (!(p_ctl & _RING_CTL_ENABLE)) {
		printk("<NO CONTENT>\n");
		return;
	}

	p_head &= RB_HEAD_OFF_MASK;
	ring_len = _RING_CTL_BUF_SIZE(p_ctl);
	gpa = p_start >> PAGE_SHIFT;
	p_contents = vgt_gma_to_va(vgt->gtt.ggtt_mm, gpa<<PAGE_SHIFT);
	if (!p_contents) {
		if (pdev->enable_execlist)
			return;

		printk("Looks this ring buffer doesn't belong to current render owner.\n");
		printk("Try to dump it from aperture.\n");
		p_contents = phys_aperture_vbase(pdev) + (gpa<<PAGE_SHIFT);
	}
#define WRAP_OFF(off, size)			\
	({					\
		u64 val = off;			\
		if ((int64_t)val < 0)		\
			val += size;	\
		if (val >= size)		\
			val -= size;	\
		(val);				\
	})
	printk("p_contents(%lx)\n", (unsigned long)(p_contents + (p_start & (PAGE_SIZE-1))));
	/* length should be 4 bytes aligned */
	bytes &= ~0x3;
	for (i = -bytes; i < bytes; i += 4) {
		char *access;
		off = (p_head + i) % ring_len;
		off = WRAP_OFF(off, ring_len);
		/* print offset within the ring every 8 Dword */
		if (!((i + bytes) % 32))
			printk("\n[%08llx]:", off);

		/* handle Dom0 VA address 4K-page boundary */
		if (((p_start+off) >> PAGE_SHIFT) != gpa) {
			// cross page boundary.
			gpa = (p_start + off) >> PAGE_SHIFT;
			p_contents = vgt_gma_to_va(vgt->gtt.ggtt_mm, gpa << PAGE_SHIFT);
			if (!p_contents) return;
		}
		access = p_contents + ((p_start + off) & (PAGE_SIZE-1));

		printk(" %08x", *((u32*)access));
		if (!i)
			printk("(*)");
	}
	printk("\n");

	if (IS_PREBDW(pdev))
		off = WRAP_OFF(((int32_t)p_head) - 8, ring_len);
	else
		off = WRAP_OFF(((int32_t)p_head) - 12, ring_len);

	p_contents = vgt_gma_to_va(vgt->gtt.ggtt_mm, p_start + off);
	if (!p_contents) return;

	cur = (u32*)(p_contents);
	if ((*cur & 0xfff00000) == 0x18800000 && vgt) {
		int ppgtt = (*cur & _CMDBIT_BB_START_IN_PPGTT);

		if (ppgtt &&
			!test_bit(ring_id, &vgt->gtt.active_ppgtt_mm_bitmap)
			&& !is_execlist_mode(pdev, ring_id)) {
			printk("Batch buffer in PPGTT with PPGTT disabled?\n");
			return;
		}

		printk("Hang in (%s) batch buffer (%x)\n",
			ppgtt ? "PPGTT" : "GTT",
			*(cur + 1));

		show_batchbuffer(pdev, ring_id,
			batch_head,
			bytes,
			ppgtt);
	}

	if (pdev->cur_reset_vm == current_render_owner(pdev))
		mmio_show_batchbuffer(pdev, ring_id, bytes);

}

void legacy_show_ring_buffer(struct pgt_device *pdev, int ring_id, int bytes)
{
	vgt_reg_t p_tail, p_head, p_start, p_ctl;

	p_tail = VGT_MMIO_READ(pdev, RB_TAIL(pdev, ring_id));
	p_head = VGT_MMIO_READ(pdev, RB_HEAD(pdev, ring_id));
	p_start = VGT_MMIO_READ(pdev, RB_START(pdev, ring_id));
	p_ctl = VGT_MMIO_READ(pdev, RB_CTL(pdev, ring_id));

	printk("ring buffer(%d): head(0x%x) tail(0x%x), start(0x%x), ctl(0x%x)\n",
		ring_id, p_head, p_tail, p_start, p_ctl);

	common_show_ring_buffer(pdev, ring_id, bytes,
			p_tail, p_head, p_start, p_ctl,
			VGT_MMIO_READ(pdev, VGT_ACTHD(ring_id)));
}

void execlist_show_ring_buffer(struct pgt_device *pdev, int ring_id, int bytes)
{
	struct vgt_device *vgt = current_render_owner(pdev);
	vgt_reg_t p_tail, p_head, p_start, p_ctl; /* read from context */
	vgt_reg_t p_tail1, p_head1, p_start1, p_ctl1; /* read from MMIO */
	unsigned long reg, val;
	u64 bb_head;
	u32 *p;

	printk("Execlist:\n");

	reg = RB_TAIL(pdev, ring_id) - 0x30 + _EL_OFFSET_STATUS;
	val = VGT_MMIO_READ(pdev, reg);

	printk("....Current execlist status: %lx.\n", val);

	val = VGT_MMIO_READ(pdev, _REG_CUR_DESC(ring_id));

	printk("....Current element descriptor(low): %lx.\n", val);

	val &= ~0xfff;

	printk("....LRCA: %lx.\n", val);

	if (!val)
		return;

	p = vgt_gma_to_va(vgt->gtt.ggtt_mm, val + 4096);
	if (!p)
		return;

	if ((ring_id == RING_BUFFER_RCS && p[1] != 0x1100101b)
		|| (ring_id != RING_BUFFER_RCS && p[1] != 0x11000015)) {
		printk("Invalid signature: %x.\n", p[1]);
		return;
	}

	p_head = *(p + 0x4 + 1);
	p_tail = *(p + 0x6 + 1);
	p_start = *(p + 0x8 + 1);
	p_ctl = *(p + 0xa + 1);

	bb_head = *(p + 0xc + 1) & 0xFFFF;
	bb_head <<= 32;
	bb_head |= *(p + 0xe + 1);

	reg = RB_HEAD(pdev, ring_id);
	p_head1 = VGT_MMIO_READ(pdev, reg);
	reg = RB_TAIL(pdev, ring_id);
	p_tail1 = VGT_MMIO_READ(pdev, reg);
	reg = RB_START(pdev, ring_id);
	p_start1 = VGT_MMIO_READ(pdev, reg);
	reg = RB_CTL(pdev, ring_id);
	p_ctl1 = VGT_MMIO_READ(pdev, reg);

	printk("ring buffer(%d): head(0x%x) tail(0x%x), start(0x%x), ctl(0x%x)\n",
		ring_id, p_head, p_tail, p_start, p_ctl);

	if (p_head != p_head1 || p_tail != p_tail1) {
		/* under some condition, MMIO men will be cleared to zero */
		if (!(p_head1 == 0 && p_tail1 == 0 &&
			p_start1 == 0 && p_ctl1 == 0)) {
			p_head = p_head1;
			p_tail = p_tail1;
		}
		printk("rb from mmio(%d): head(0x%x) tail(0x%x), start(0x%x), ctl(0x%x)\n",
			ring_id, p_head1, p_tail1, p_start1, p_ctl1);
	}

	common_show_ring_buffer(pdev, ring_id, bytes,
			p_tail, p_head, p_start, p_ctl,
			bb_head);
}

void show_ring_buffer(struct pgt_device *pdev, int ring_id, int bytes)
{
	is_execlist_mode(pdev, ring_id) ?
		execlist_show_ring_buffer(pdev, ring_id, bytes) :
		legacy_show_ring_buffer(pdev, ring_id, bytes);
}

void show_interrupt_regs(struct pgt_device *pdev,
		struct seq_file *seq)
{
#define P(fmt, args...) \
	do { \
		if (!seq) \
			vgt_info(fmt, ##args); \
		else \
			seq_printf(seq, fmt, ##args); \
	}while(0)

	if (IS_PREBDW(pdev)) {
		P("vGT: DEISR is %x, DEIIR is %x, DEIMR is %x, DEIER is %x\n",
				VGT_MMIO_READ(pdev, DEISR),
				VGT_MMIO_READ(pdev, DEIIR),
				VGT_MMIO_READ(pdev, DEIMR),
				VGT_MMIO_READ(pdev, DEIER));
		P("vGT: GTISR is %x, GTIIR is %x, GTIMR is %x, GTIER is %x\n",
				VGT_MMIO_READ(pdev, GTISR),
				VGT_MMIO_READ(pdev, GTIIR),
				VGT_MMIO_READ(pdev, GTIMR),
				VGT_MMIO_READ(pdev, GTIER));
		P("vGT: PMISR is %x, PMIIR is %x, PMIMR is %x, PMIER is %x\n",
				VGT_MMIO_READ(pdev, GEN6_PMISR),
				VGT_MMIO_READ(pdev, GEN6_PMIIR),
				VGT_MMIO_READ(pdev, GEN6_PMIMR),
				VGT_MMIO_READ(pdev, GEN6_PMIER));
	} else {
		P("vGT: MASTER_IRQ: %x\n",
			VGT_MMIO_READ(pdev, GEN8_MASTER_IRQ));

#define P_GROUP_WHICH(group, w) do {\
		P("vGT: "#group"|"#w" ISR: %x IIR: %x IMR: %x IER: %x\n", \
			VGT_MMIO_READ(pdev, GEN8_##group##_ISR(w)), \
			VGT_MMIO_READ(pdev, GEN8_##group##_IIR(w)), \
			VGT_MMIO_READ(pdev, GEN8_##group##_IMR(w)), \
			VGT_MMIO_READ(pdev, GEN8_##group##_IER(w))); \
	}while(0)

#define P_GROUP(group) do {\
		P("vGT: "#group" ISR: %x IIR: %x IMR: %x IER: %x\n", \
			VGT_MMIO_READ(pdev, GEN8_##group##_ISR), \
			VGT_MMIO_READ(pdev, GEN8_##group##_IIR), \
			VGT_MMIO_READ(pdev, GEN8_##group##_IMR), \
			VGT_MMIO_READ(pdev, GEN8_##group##_IER)); \
	}while(0)

		P_GROUP_WHICH(DE_PIPE, PIPE_A);
		P_GROUP_WHICH(DE_PIPE, PIPE_B);
		P_GROUP_WHICH(DE_PIPE, PIPE_C);

		P_GROUP_WHICH(GT, 0);
		P_GROUP_WHICH(GT, 1);
		P_GROUP_WHICH(GT, 2);
		P_GROUP_WHICH(GT, 3);

		P_GROUP(DE_PORT);
		P_GROUP(DE_MISC);
		P_GROUP(PCU);
	}

	P("vGT: SDEISR is %x, SDEIIR is %x, SDEIMR is %x, SDEIER is %x\n",
			VGT_MMIO_READ(pdev, SDEISR),
			VGT_MMIO_READ(pdev, SDEIIR),
			VGT_MMIO_READ(pdev, SDEIMR),
			VGT_MMIO_READ(pdev, SDEIER));

	P("vGT: RCS_IMR is %x, VCS_IMR is %x, BCS_IMR is %x\n",
			VGT_MMIO_READ(pdev, IMR),
			VGT_MMIO_READ(pdev, _REG_VCS_IMR),
			VGT_MMIO_READ(pdev, _REG_BCS_IMR));
	return;
#undef P
#undef P_GROUP
#undef P_GROUP_WHICH
}

void show_virtual_interrupt_regs(struct vgt_device *vgt,
		struct seq_file *seq)
{
#define P(fmt, args...) \
	do { \
		if (!seq) \
			vgt_info(fmt, ##args); \
		else \
			seq_printf(seq, fmt, ##args); \
	}while(0)

	if (IS_PREBDW(vgt->pdev)) {
		P("....vreg (deier: %x, deiir: %x, deimr: %x, deisr: %x)\n",
				__vreg(vgt, DEIER),
				__vreg(vgt, DEIIR),
				__vreg(vgt, DEIMR),
				__vreg(vgt, DEISR));
		P("....vreg (gtier: %x, gtiir: %x, gtimr: %x, gtisr: %x)\n",
				__vreg(vgt, GTIER),
				__vreg(vgt, GTIIR),
				__vreg(vgt, GTIMR),
				__vreg(vgt, GTISR));
		P("....vreg (pmier: %x, pmiir: %x, pmimr: %x, pmisr: %x)\n",
				__vreg(vgt, GEN6_PMIER),
				__vreg(vgt, GEN6_PMIIR),
				__vreg(vgt, GEN6_PMIMR),
				__vreg(vgt, GEN6_PMISR));
	} else {
		P("....vreg: MASTER_IRQ: %x\n",
				__vreg(vgt, GEN8_MASTER_IRQ));

#define P_GROUP_WHICH(group, w) do {\
		P("....vreg "#group"|"#w" ISR: %x IIR: %x IMR: %x IER: %x\n", \
			__vreg(vgt, GEN8_##group##_ISR(w)), \
			__vreg(vgt, GEN8_##group##_IIR(w)), \
			__vreg(vgt, GEN8_##group##_IMR(w)), \
			__vreg(vgt, GEN8_##group##_IER(w))); \
	}while(0)

#define P_GROUP(group) do {\
		P("....vreg "#group" ISR: %x IIR: %x IMR: %x IER: %x\n", \
			__vreg(vgt, GEN8_##group##_ISR), \
			__vreg(vgt, GEN8_##group##_IIR), \
			__vreg(vgt, GEN8_##group##_IMR), \
			__vreg(vgt, GEN8_##group##_IER)); \
	}while(0)

		P_GROUP_WHICH(DE_PIPE, PIPE_A);
		P_GROUP_WHICH(DE_PIPE, PIPE_B);
		P_GROUP_WHICH(DE_PIPE, PIPE_C);

		P_GROUP_WHICH(GT, 0);
		P_GROUP_WHICH(GT, 1);
		P_GROUP_WHICH(GT, 2);
		P_GROUP_WHICH(GT, 3);

		P_GROUP(DE_PORT);
		P_GROUP(DE_MISC);
		P_GROUP(PCU);
	}

	P("....vreg (sdeier: %x, sdeiir: %x, sdeimr: %x, sdeisr: %x)\n",
			__vreg(vgt, SDEIER),
			__vreg(vgt, SDEIIR),
			__vreg(vgt, SDEIMR),
			__vreg(vgt, SDEISR));

	P("....vreg (rcs_imr: %x, vcs_imr: %x, bcs_imr: %x\n",
			__vreg(vgt, IMR),
			__vreg(vgt, _REG_VCS_IMR),
			__vreg(vgt, _REG_BCS_IMR));

	return;
#undef P
#undef P_GROUP
#undef P_GROUP_WHICH
}

uint64_t pci_bar_size(struct pgt_device *pdev, unsigned int bar_off)
{
	uint32_t bar_s;
	uint64_t bar_size, bar_upper_size = 0;
	struct pci_dev *dev = pdev->pdev;

	pci_read_config_dword(dev, bar_off, (uint32_t *)&bar_s);
	pci_write_config_dword(dev, bar_off, 0xFFFFFFFF);
	pci_read_config_dword(dev, bar_off, (uint32_t *)&bar_size);
	vgt_dbg(VGT_DBG_GENERIC, "read back lower bar size %x\n", (uint32_t)bar_size);
	bar_size &= ~0xf; /* bit 4-31 */
	pci_write_config_dword(dev, bar_off, bar_s);
	if (VGT_GET_BITS(bar_s, 2, 1) == 2) {
		pci_read_config_dword(dev, bar_off + 4, (uint32_t *)&bar_s);
		pci_write_config_dword(dev, bar_off + 4, 0xFFFFFFFF);
		pci_read_config_dword(dev, bar_off + 4, (uint32_t *)&bar_upper_size);
		vgt_dbg(VGT_DBG_GENERIC, "read back higher bar size %x\n", (uint32_t)bar_upper_size);
		bar_size |= (bar_upper_size << 32);
		pci_write_config_dword(dev, bar_off + 4, bar_s);
	}
	bar_size &= ~(bar_size - 1);
	return bar_size;
}

uint64_t vgt_get_gtt_size(struct pgt_device *pdev)
{
	struct pci_bus *bus = pdev->pbus;
	uint16_t gmch_ctrl;

	ASSERT(!bus->number);

	/* GTT size is within GMCH. */
	pci_bus_read_config_word(bus, 0, _REG_GMCH_CONTRL, &gmch_ctrl);

	if (IS_PREBDW(pdev)) {
		gmch_ctrl = (gmch_ctrl >> 8) & 3;
		switch (gmch_ctrl) {
			case 1:
			case 2:
				return gmch_ctrl << 20;
			default:
				vgt_err("Invalid GTT memory size: %d\n", gmch_ctrl);
				break;
		}
	} else {
		gmch_ctrl = (gmch_ctrl >> 6) & 3;
		if (gmch_ctrl)
			gmch_ctrl = 1 << gmch_ctrl;
		switch (gmch_ctrl) {
			case 2:
			case 4:
			case 8:
				return gmch_ctrl << 20;
			default:
				vgt_err("Invalid GTT memory size: %d\n", gmch_ctrl);
				break;
		}
	}

	return 0;
}

/*
 * random GTT entry check
 */
void check_gtt(struct pgt_device *pdev)
{
	static unsigned int addr[] = {
	0x00000000, 0x02000000, 0x04000000, 0x08000000,
	0x0C000000, 0x0FFFF000, 0x10000000, 0x20000000,
	0x40000000, 0x60000000, 0x7FFFF000 };

	int i;

	for (i = 0; i < ARRAY_SIZE(addr); i++)
		vgt_dbg(VGT_DBG_MEM, "GMADR: 0x08%x, GTT INDEX: %x, GTT VALUE: %x\n",
			addr[i], GTT_INDEX(pdev, addr[i]),
			vgt_read_gtt(pdev, GTT_INDEX(pdev, addr[i])));
}

static inline u64 dma_addr_to_pte_uc(struct pgt_device *pdev, dma_addr_t addr)
{
	u64 v;

	if (IS_BDWPLUS(pdev)) {
		v = addr & (0x7ffffff << 12);
	} else {
		if (IS_HSW(pdev)) {
			/* Haswell has new cache control bits */
			v = addr & ~0xfff;
			v |= (addr >> 28) & 0x7f0;
		} else {
			v = addr & ~0xfff;
			v |= (addr >> 28) & 0xff0;
			v |= (1 << 1); /* UC */
		}
	}
	v |= 1;
	return v;
}

void init_gm_space(struct pgt_device *pdev)
{
	struct vgt_gtt_pte_ops *ops = pdev->gtt.pte_ops;
	unsigned long i;

	/* clear all GM space, instead of only aperture */
	for (i = 0; i < gm_pages(pdev); i++)
		ops->set_entry(NULL, &pdev->dummy_gtt_entry, i, false, NULL);

	vgt_dbg(VGT_DBG_MEM, "content at 0x0: %lx\n",
			*(unsigned long *)((char *)phys_aperture_vbase(pdev) + 0x0));
	vgt_dbg(VGT_DBG_MEM, "content at 0x64000: %lx\n",
			*(unsigned long *)((char *)phys_aperture_vbase(pdev) + 0x64000));
	vgt_dbg(VGT_DBG_MEM, "content at 0x8064000: %lx\n",
			*(unsigned long *)((char *)phys_aperture_vbase(pdev) + 0x8064000));
}

static void vgt_free_gtt_pages(struct pgt_device *pdev)
{
	int i;
	struct page *dummy_page = pdev->dummy_page;
	struct page *(*pages)[VGT_APERTURE_PAGES] =
		pdev->rsvd_aperture_pages;

	if (pages != NULL) {
		for (i = 0; i < VGT_APERTURE_PAGES; i++) {
			if ((*pages)[i] == NULL)
				continue;
			put_page((*pages)[i]);
			__free_page((*pages)[i]);
		}
		kfree(pages);
	}

	if (dummy_page != NULL) {
		put_page(dummy_page);
		__free_page(dummy_page);
	}
}

void vgt_clear_gtt(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	struct vgt_gtt_pte_ops *ops = pdev->gtt.pte_ops;
	uint32_t index;
	uint32_t offset;
	uint32_t num_entries;

	index = vgt_visible_gm_base(vgt) >> PAGE_SHIFT;
	num_entries = vgt_aperture_sz(vgt) >> PAGE_SHIFT;
	for (offset = 0; offset < num_entries; offset++){
		ops->set_entry(NULL, &pdev->dummy_gtt_entry, index+offset, false, NULL);
	}

	index = vgt_hidden_gm_base(vgt) >> PAGE_SHIFT;
	num_entries = vgt_hidden_gm_sz(vgt) >> PAGE_SHIFT;
	for (offset = 0; offset < num_entries; offset++){
		ops->set_entry(NULL, &pdev->dummy_gtt_entry, index+offset, false, NULL);
	}
}

int setup_gtt(struct pgt_device *pdev)
{
	struct vgt_gtt_pte_ops *ops = pdev->gtt.pte_ops;
	struct page *dummy_page;
	struct page *(*pages)[VGT_APERTURE_PAGES];
	struct page *page;

	int i, ret, index;
	dma_addr_t dma_addr;
	gtt_entry_t e;
	u64 v;

	if (!pci_set_dma_mask(pdev->pdev, DMA_BIT_MASK(39)))
		pci_set_consistent_dma_mask(pdev->pdev, DMA_BIT_MASK(39));

	check_gtt(pdev);

	printk("vGT: clear all GTT entries.\n");

	dummy_page = alloc_page(GFP_KERNEL | __GFP_ZERO | GFP_DMA32);
	if (!dummy_page)
		return -ENOMEM;
	pdev->dummy_page = dummy_page;

	get_page(dummy_page);
	dma_addr = pci_map_page(pdev->pdev, dummy_page, 0, PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
	if (pci_dma_mapping_error(pdev->pdev, dma_addr)) {
		ret = -EINVAL;
		goto err_out;
	}

	printk("....dummy page (0x%llx, 0x%llx)\n", page_to_phys(dummy_page), dma_addr);

	/* for debug purpose */
	memset(pfn_to_kaddr(page_to_pfn(dummy_page)), 0x77, PAGE_SIZE);

	v = dma_addr_to_pte_uc(pdev, dma_addr);
	gtt_init_entry(&e, GTT_TYPE_GGTT_PTE, pdev, v);
	pdev->dummy_gtt_entry = e;

	init_gm_space(pdev);

	check_gtt(pdev);

	printk("vGT: allocate vGT aperture\n");
	/* Fill GTT range owned by vGT driver */

	ASSERT(sizeof(*pages) == VGT_APERTURE_PAGES * sizeof(struct page*));
	if ((pages = kzalloc(sizeof(*pages), GFP_KERNEL)) == NULL) {
		ret = -ENOMEM;
		goto err_out;
	}
	pdev->rsvd_aperture_pages = pages;


	index = GTT_INDEX(pdev, aperture_2_gm(pdev, pdev->rsvd_aperture_base));
	for (i = 0; i < VGT_APERTURE_PAGES; i++) {
		/* need a DMA flag? */
		page = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!page) {
			vgt_dbg(VGT_DBG_MEM, "vGT: Failed to create page for setup_gtt!\n");
			ret = -ENOMEM;
			goto err_out;
		}

		get_page(page);
		set_memory_wc((unsigned long)page_address(page), 1);

		(*pages)[i] = page;

		/* dom0 needs DMAR anyway */
		dma_addr = pci_map_page(pdev->pdev, page, 0, PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
		if (pci_dma_mapping_error(pdev->pdev, dma_addr)) {
			printk(KERN_ERR "vGT: Failed to do pci_dma_mapping while handling %d 0x%llx\n", i, dma_addr);
			ret = -EINVAL;
			goto err_out;
		}

		ops->set_pfn(&e, dma_addr >> GTT_PAGE_SHIFT);
		ops->set_entry(NULL, &e, index + i, false, NULL);

		if (!(i % 1024))
			vgt_dbg(VGT_DBG_MEM, "vGT: write GTT-%x phys: %llx, dma: %llx\n",
				index + i, page_to_phys(page), dma_addr);
	}

	check_gtt(pdev);
	/* any cache flush required here? */
	return 0;
err_out:
	printk("vGT: error in GTT initialization\n");
	vgt_free_gtt_pages(pdev);

	return ret;
}

void free_gtt(struct pgt_device *pdev)
{
	/* TODO: move this to host i915, when it is GVT-g aware */
	tmp_vgt_clear_gtt(pdev->gtt_size);
	vgt_free_gtt_pages(pdev);
}

void vgt_save_gtt_and_fence(struct pgt_device *pdev)
{
	int i;
	uint32_t *entry = pdev->saved_gtt;

	ASSERT(pdev->saved_gtt);
	vgt_info("Save GTT table...\n");
	for (i = 0; i < gm_pages(pdev); i++)
		*(entry + i) = vgt_read_gtt(pdev, i);

	for (i = 0; i < VGT_MAX_NUM_FENCES; i++)
		pdev->saved_fences[i] =
			VGT_MMIO_READ64(pdev, _REG_FENCE_0_LOW + 8 * i);
}

void vgt_restore_gtt_and_fence(struct pgt_device *pdev)
{
	int i;
	uint32_t *entry = pdev->saved_gtt;

	ASSERT(pdev->saved_gtt);
	vgt_info("Restore GTT table...\n");
	for (i = 0; i < gm_pages(pdev); i++)
		vgt_write_gtt(pdev, i, *(entry + i));

	for (i = 0; i < VGT_MAX_NUM_FENCES; i++)
		VGT_MMIO_WRITE_BYTES(pdev,
			_REG_FENCE_0_LOW + 8 * i,
			pdev->saved_fences[i], 8);
}

static void _hex_dump(const char *data, size_t size)
{
	char buf[74];
	size_t offset;
	int line;

	for (line = 0; line < ((size + 0xF) / 0x10); line++) {
		int byte;

		memset(buf, ' ', sizeof(buf));
		buf[73] = '\0';
		offset = 0;

		offset += snprintf(buf + offset, 74 - offset, "%07x: ", line * 0x10);

		for (byte = 0; byte < 0x10; byte++) {
			if (!(byte & 0x1)) {
				offset += snprintf(buf + offset, 74 - offset, " ");
			}

			if (((line * 0x10) + byte) >= size) {
				offset += snprintf(buf + offset, 74 - offset, "  ");
			} else {
				offset += snprintf(buf + offset, 74 - offset, "%02x",
					       data[byte + (line * 0x10)] & 0xFF);
			}
		}
		
		offset += snprintf(buf + offset, 74 - offset, "  ");

		for (byte = 0; byte < 0x10; byte++) {
			if (data[byte + (line * 0x10)] >= 0x20 &&
			    data[byte + (line * 0x10)] <= 0x7E) {
				offset += snprintf(buf + offset, 74 - offset, "%c",
				    data[byte + (line * 0x10)] & 0xFF);
			} else {
				offset += snprintf(buf + offset, 74 - offset, ".");
			}
		}

		offset += snprintf(buf + offset, 74 - offset, "\n");
		printk(buf);
	}
}

void vgt_print_edid(struct vgt_edid_data_t *edid)
{
	if (edid && edid->data_valid) {
		_hex_dump(edid->edid_block, EDID_SIZE);
	} else {
		printk("EDID is not available!\n");
	}

	return;
}

void vgt_print_dpcd(struct vgt_dpcd_data *dpcd)
{
	if (dpcd && dpcd->data_valid) {
		_hex_dump(dpcd->data, DPCD_SIZE);
	} else {
		printk("DPCD is not available!\n");
	}
}

int vgt_hvm_map_aperture (struct vgt_device *vgt, int map)
{
	char *cfg_space = &vgt->state.cfg_space[0];
	uint64_t bar_s;
	int r, nr_mfns;
	unsigned long first_gfn, first_mfn;

	if (!vgt_pci_mmio_is_enabled(vgt))
		return 0;

	/* guarantee the sequence of map -> unmap -> map -> unmap */
	if (map == vgt->state.bar_mapped[1])
		return 0;

	cfg_space += VGT_REG_CFG_SPACE_BAR1;	/* APERTUR */
	if (VGT_GET_BITS(*cfg_space, 2, 1) == 2){
		/* 64 bits MMIO bar */
		bar_s = * (uint64_t *) cfg_space;
	} else {
		/* 32 bits MMIO bar */
		bar_s = * (uint32_t*) cfg_space;
	}

	first_gfn = (bar_s + vgt_aperture_offset(vgt)) >> PAGE_SHIFT;
	first_mfn = vgt_aperture_base(vgt) >> PAGE_SHIFT;
	if (!vgt->ballooning)
		nr_mfns = vgt->state.bar_size[1] >> PAGE_SHIFT;
	else
		nr_mfns = vgt_aperture_sz(vgt) >> PAGE_SHIFT;

	printk("%s: domid=%d gfn_s=0x%lx mfn_s=0x%lx nr_mfns=0x%x\n", map==0? "remove_map":"add_map",
		vgt->vm_id, first_gfn, first_mfn, nr_mfns);

	r = hypervisor_map_mfn_to_gpfn(vgt, first_gfn, first_mfn,
		nr_mfns, map, VGT_MAP_APERTURE);

	if (r != 0)
		printk(KERN_ERR "vgt_hvm_map_aperture fail with %d!\n", r);
	else
		vgt->state.bar_mapped[1] = map;

	return r;
}

/*
 * Zap the GTTMMIO bar area for vGT trap and emulation.
 */
int vgt_hvm_set_trap_area(struct vgt_device *vgt, int map)
{
	char *cfg_space = &vgt->state.cfg_space[0];
	uint64_t bar_s, bar_e;

	if (!vgt->vm_id || !vgt_pci_mmio_is_enabled(vgt))
		return 0;

	cfg_space += VGT_REG_CFG_SPACE_BAR0;
	if (VGT_GET_BITS(*cfg_space, 2, 1) == 2) {
		/* 64 bits MMIO bar */
		bar_s = * (uint64_t *) cfg_space;
	} else {
		/* 32 bits MMIO bar */
		bar_s = * (uint32_t*) cfg_space;
	}

	bar_s &= ~0xF; /* clear the LSB 4 bits */
	bar_e = bar_s + vgt->state.bar_size[0] - 1;

	return hypervisor_set_trap_area(vgt, bar_s, bar_e, map);
}

/* EXECLIST dump functions */

void dump_ctx_desc(struct vgt_device *vgt, struct ctx_desc_format *desc)
{
	const char *addressing_string[4] = {
		"advanced context 64-bit no A/D",
		"legacy context 32-bit",
		"advanced context 64-bit A/D",
		"legacy context 64-bit" };

	printk("\nContext Descriptor ----\n");
	printk("\t ldw = 0x%x; udw = 0x%x\n", desc->elm_low, desc->elm_high);
	printk("\t context_id:0x%x\n", desc->context_id);
	printk("\t valid:%d\n", desc->valid);
	printk("\t force_pd_restore:%d\n", desc->force_pd_restore);
	printk("\t force_restore:%d\n", desc->force_restore);
	printk("\t addressing_mode:%d(%s)\n", desc->addressing_mode,
			addressing_string[desc->addressing_mode]);
	printk("\t llc_coherency:%d\n", desc->llc_coherency);
	printk("\t fault_handling:%d\n", desc->fault_handling);
	printk("\t privilege_access:%d\n", desc->privilege_access);
	printk("\t lrca:0x%x\n", desc->lrca);
}

void dump_execlist_status(struct execlist_status_format *status, enum vgt_ring_id ring_id)
{
	printk("-------- Current EXECLIST status of ring-%d --------\n", ring_id);
	printk("\tCurrent Context ID: 0x%x\n", status->context_id);
	printk("\tLDW: 0x%x\n", status->ldw);
	printk("\tEXECLIST queue full: %d\n", status->execlist_queue_full);
	printk("\tCurrent EXECLIST index: %d\n",
				status->current_execlist_pointer);
	printk("\tEXECLIST write index: %d\n", status->execlist_write_pointer);
	printk("\t  EXECLIST 0 status: %d(1 for valid)\t %d(1 for active)\n",
			status->execlist_0_valid, status->execlist_0_active);
	printk("\t  EXECLIST 1 status: %d(1 for valid)\t %d(1 for active)\n",
			status->execlist_1_valid, status->execlist_1_active);
	printk("\tActive context information:\n");
	printk("\t    %s is active\n", status->current_active_elm_status == 0 ?
			"no context" : (status->current_active_elm_status == 1 ?
						"context 0" : "context 1"));
	printk("\tLast ctx switch reason: 0x%x\n",
				status->last_ctx_switch_reason);
	printk("\tArbitration is: %s\n", status->arbitration_enable ?
						"enabled" : "disabled");
	if (status->current_active_elm_status == 2)
		vgt_err("EXECLIST status register has invalid active context value!\n");
}

void dump_execlist_info(struct pgt_device *pdev, enum vgt_ring_id ring_id)
{
	uint32_t status_reg = el_ring_mmio(ring_id, _EL_OFFSET_STATUS);
	struct execlist_status_format status;

	READ_STATUS_MMIO(pdev, status_reg, status);
	dump_execlist_status(&status, ring_id);
}

static void dump_ctx_status_buf_entry(struct vgt_device *vgt,
			enum vgt_ring_id ring_id, unsigned int buf_entry, bool hw_status)
{
	struct context_status_format status;
	uint32_t ctx_status_reg;

	if (buf_entry >= CTX_STATUS_BUF_NUM)
		return;

	ctx_status_reg = el_ring_mmio(ring_id, _EL_OFFSET_STATUS_BUF);
	ctx_status_reg += buf_entry * 8;

	if (hw_status) {
		READ_STATUS_MMIO(vgt->pdev, ctx_status_reg, status);
	} else {
		status.ldw = __vreg(vgt, ctx_status_reg);
		status.udw = __vreg(vgt, ctx_status_reg + 4);
	}

	printk("    ring-%d CSB[%d]: ctx(0x%08x) val(0x%08x) <set bits: ",
			ring_id, buf_entry, status.context_id, status.ldw);
	if (status.idle_to_active)
		printk("idle_to_active; ");
	if (status.preempted)
		printk("preemptedn; ");
	if (status.element_switch)
		printk("element_switch; ");
	if (status.active_to_idle)
		printk("active_to_idle; ");
	if (status.context_complete)
		printk("context_complete; ");
	if (status.wait_on_sync_flip)
		printk("wait_on_sync_flip; ");
	if (status.wait_on_vblank)
		printk("wait_on_vblank; ");
	if (status.wait_on_semaphore)
		printk("wait_on_semaphore; ");
	if (status.wait_on_scanline)
		printk("wait_on_scanline; ");
	if (status.semaphore_wait_mode)
		printk("semaphore_wait_mode; ");
	if (status.display_plane)
		printk("display_plane; ");
	if (status.lite_restore)
		printk("lite_restore; ");
	printk(">\n");
}

static void dump_ctx_st_ptr(struct vgt_device *vgt, struct ctx_st_ptr_format *ptr)
{
	printk("Context StatusBufPtr Value: 0x%x. ", ptr->dw);
	printk("(write_ptr: %d; ", ptr->status_buf_write_ptr);
	printk("read_ptr: %d; ", ptr->status_buf_read_ptr);
	printk("mask: 0x%x\n", ptr->mask);
}

void dump_ctx_status_buf(struct vgt_device *vgt,
			enum vgt_ring_id ring_id, bool hw_status)
{
	uint32_t ctx_ptr_reg;
	struct ctx_st_ptr_format ctx_st_ptr;
	unsigned read_idx;
	unsigned write_idx;
	int i;

	ctx_ptr_reg = el_ring_mmio(ring_id, _EL_OFFSET_STATUS_PTR);

	if (hw_status)
		ctx_st_ptr.dw = VGT_MMIO_READ(vgt->pdev, ctx_ptr_reg);
	else
		ctx_st_ptr.dw = __vreg(vgt, ctx_ptr_reg);

	if (hw_status)
		printk("---- Physical status Buffer of Ring %d ---- \n",
			ring_id);
	else
		printk("---- Virtual status Buffer for VM-%d of Ring %d ---- \n",
			vgt->vm_id, ring_id);

	dump_ctx_st_ptr(vgt, &ctx_st_ptr);

	read_idx = ctx_st_ptr.status_buf_read_ptr;
	write_idx = ctx_st_ptr.status_buf_write_ptr;

	if (read_idx == DEFAULT_INV_SR_PTR)
		read_idx = 0;

	if (write_idx == DEFAULT_INV_SR_PTR) {
		vgt_err("No writes happened and no interesting data "
			"in status buffer to show.\n");
		return;
	}

	/* show all contents in hw/virtual buffer */
	read_idx = 0;
	write_idx = CTX_STATUS_BUF_NUM - 1;

	if (read_idx > write_idx)
		write_idx += CTX_STATUS_BUF_NUM;

	for (i = read_idx; i <= write_idx; ++ i)
		dump_ctx_status_buf_entry(vgt, ring_id,
			i % CTX_STATUS_BUF_NUM, hw_status);
}

#define DUMP_CTX_MMIO(prefix, mmio)	\
printk("\t " #mmio ": <addr>0x%x - <val>0x%x\n", \
	prefix->mmio.addr, prefix->mmio.val);

void dump_regstate_ctx_header (struct reg_state_ctx_header *regstate)
{
	printk("\tlri_command:0x%x\n", regstate->lri_cmd_1);
	DUMP_CTX_MMIO(regstate, ctx_ctrl);
	DUMP_CTX_MMIO(regstate, ring_header);
	DUMP_CTX_MMIO(regstate, ring_tail);
	DUMP_CTX_MMIO(regstate, rb_start);
	DUMP_CTX_MMIO(regstate, rb_ctrl);
	DUMP_CTX_MMIO(regstate, bb_cur_head_UDW);
	DUMP_CTX_MMIO(regstate, bb_cur_head_LDW);
	DUMP_CTX_MMIO(regstate, bb_state);
	DUMP_CTX_MMIO(regstate, second_bb_addr_UDW);
	DUMP_CTX_MMIO(regstate, second_bb_addr_LDW);
	DUMP_CTX_MMIO(regstate, second_bb_state);
	DUMP_CTX_MMIO(regstate, bb_per_ctx_ptr);
	DUMP_CTX_MMIO(regstate, rcs_indirect_ctx);
	DUMP_CTX_MMIO(regstate, rcs_indirect_ctx_offset);
	printk("\tlri_command2:0x%x\n", regstate->lri_cmd_2);
	DUMP_CTX_MMIO(regstate, ctx_timestamp);
	DUMP_CTX_MMIO(regstate, pdp3_UDW);
	DUMP_CTX_MMIO(regstate, pdp3_LDW);
	DUMP_CTX_MMIO(regstate, pdp2_UDW);
	DUMP_CTX_MMIO(regstate, pdp2_LDW);
	DUMP_CTX_MMIO(regstate, pdp1_UDW);
	DUMP_CTX_MMIO(regstate, pdp1_LDW);
	DUMP_CTX_MMIO(regstate, pdp0_UDW);
	DUMP_CTX_MMIO(regstate, pdp0_LDW);
}

static inline struct reg_state_ctx_header *
vgt_get_reg_state_from_lrca(struct vgt_device *vgt, uint32_t lrca)
{
	struct reg_state_ctx_header *header;
	uint32_t state_gma = (lrca + 1) << GTT_PAGE_SHIFT;

	header = (struct reg_state_ctx_header *)
			vgt_gma_to_va(vgt->gtt.ggtt_mm, state_gma);
	return header;
}

void dump_el_context_information(struct vgt_device *vgt,
					struct execlist_context *el_ctx)
{
	struct reg_state_ctx_header *guest_state;
	struct reg_state_ctx_header *shadow_state;
	bool has_shadow;

	if (el_ctx == NULL)
		return;

	has_shadow = vgt_require_shadow_context(vgt) && ! hvm_render_owner;

	if (has_shadow)
		guest_state = (struct reg_state_ctx_header *)
				el_ctx->ctx_pages[1].guest_page.vaddr;
	else
		guest_state = vgt_get_reg_state_from_lrca(vgt,
					el_ctx->guest_context.lrca);

	printk("-- Context with guest ID 0x%x: --\n", el_ctx->guest_context.context_id);
	printk("Guest(LRCA 0x%x) register state in context<0x%llx> is:\n",
			el_ctx->guest_context.lrca,
			(unsigned long long)guest_state);
	dump_regstate_ctx_header(guest_state);

	printk("-- Ring Buffer from guest context --\n");
	common_show_ring_buffer(vgt->pdev, el_ctx->ring_id, 64 * 4,
				guest_state->ring_tail.val,
				guest_state->ring_header.val,
				guest_state->rb_start.val,
				guest_state->rb_ctrl.val,
				0);
	if (!has_shadow)
		return;

	shadow_state = (struct reg_state_ctx_header *)
				el_ctx->ctx_pages[1].shadow_page.vaddr;

	printk("Shadow(LRCA 0x%x) register state in context <0x%llx> is:\n",
			el_ctx->shadow_lrca,
			(unsigned long long)shadow_state);
	dump_regstate_ctx_header(shadow_state);
}

void dump_all_el_contexts(struct pgt_device *pdev)
{
	struct vgt_device *vgt;
	int i;
	printk("-------- dump all shadow contexts --------\n");
	for (i = 0; i < VGT_MAX_VMS; ++ i) {
		struct hlist_node *n;
		struct execlist_context *el_ctx;
		int j;

		vgt = pdev->device[i];
		if (!vgt || (pdev->cur_reset_vm &&
					vgt != pdev->cur_reset_vm))
			continue;
		printk("-- VM(%d) --\n", vgt->vm_id);
		hash_for_each_safe(vgt->gtt.el_ctx_hash_table, j, n, el_ctx, node) {
			dump_el_context_information(vgt, el_ctx);
			printk("^^^^^^^^\n");
		}
	}
}

static void dump_el_queue(struct vgt_device *vgt, int ring_id)
{
	int i;
	printk("---- VM(%d): ring-%d EL queue ---", vgt->vm_id, ring_id);
	printk("\thead: %d; tail: %d;\n", vgt_el_queue_head(vgt, ring_id),
			vgt_el_queue_tail(vgt, ring_id));
	for (i = 0; i < EL_QUEUE_SLOT_NUM; ++ i) {
		int j;
		struct vgt_exec_list *el_slot;
		el_slot = &vgt_el_queue_slot(vgt, ring_id, i);
		printk("[%d]: status: %d\n", i, el_slot->status);
		for (j = 0; j < 2; ++ j) {
			struct execlist_context *el_ctx;
			el_ctx = vgt_el_queue_ctx(vgt, ring_id, i, j);
			printk("|-ctx[%d]: ", j);
			if (el_ctx == NULL)
				printk("NULL\n");
			else
				printk("guest lrca:0x%x\n", el_ctx->guest_context.lrca);
		}
	}
}

void dump_el_status(struct pgt_device *pdev)
{
	enum vgt_ring_id ring_id;

	for (ring_id = RING_BUFFER_RCS; ring_id < MAX_ENGINES; ++ ring_id) {
		int i;
		dump_execlist_info(pdev, ring_id);
		dump_ctx_status_buf(vgt_dom0, ring_id, true);
		for (i = 0; i < VGT_MAX_VMS; ++ i) {
			struct vgt_device *vgt = pdev->device[i];
			if (!vgt || (pdev->cur_reset_vm &&
						vgt != pdev->cur_reset_vm))
				continue;
			dump_ctx_status_buf(vgt, ring_id, false);
			dump_el_queue(vgt, ring_id);
		}
	}
}

struct pgt_device *perf_pgt = NULL;

void vgt_gpu_perf_sample(void)
{
	int	ring_id = 0;

	if ( perf_pgt ) {
		if ( ring_is_empty(perf_pgt, ring_id) )
			perf_pgt->stat.ring_0_idle ++;
		else
			perf_pgt->stat.ring_0_busy ++;
	}
}

EXPORT_SYMBOL_GPL(vgt_gpu_perf_sample);
