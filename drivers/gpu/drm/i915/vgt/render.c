/*
 * Render context management
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
#include "vgt.h"

/*
 * NOTE list:
 *	- hook with i915 driver (now invoke vgt_initalize from i915_init directly)
 *	 also the hooks in AGP driver
 *	- need a check on "unsigned long" vs. "u64" usage
 *	- need consider cache related issues, e.g. Linux/Windows may have different
 *	 TLB invalidation mode setting, which may impact vGT's context switch logic
 */

int vgt_ctx_switch = 1;
bool vgt_validate_ctx_switch = false;

void vgt_toggle_ctx_switch(bool enable)
{
	/*
	 * No need to hold lock as this will be observed
	 * in the next check in kthread.
	 */
	if (enable)
		vgt_ctx_switch = 1;
	else
		vgt_ctx_switch = 0;
}

static bool ring_is_xxx(struct pgt_device *pdev,
	int id)
{
	if (pdev->ring_xxx_valid &&
	    !(VGT_MMIO_READ(pdev, pdev->ring_xxx[id]) &
		      (1 << pdev->ring_xxx_bit[id])))
		return false;

	return true;
}

/* make a render engine idle */
bool idle_render_engine(struct pgt_device *pdev, int id)
{
	if (wait_for_atomic(ring_is_empty(pdev, id), VGT_RING_TIMEOUT) != 0) {
		int i, busy = 1;
		vgt_reg_t acthd1, acthd2;
		vgt_warn("Timeout wait %d ms for ring(%d) empty\n",
			VGT_RING_TIMEOUT, id);

		/*
		 * TODO:
		 * The timeout value may be not big enough, for some specific
		 * workloads in the VM, if they submit a big trunk of commands
		 * in a batch. The problem in current implementation is, ctx
		 * switch request is made asynchronous to the cmd submission.
		 * it's possible to have a request handled right after the
		 * current owner submits a big trunk of commands, and thus
		 * need to wait for a long time for completion.
		 *
		 * a better way is to detect ring idle in a delayed fashion,
		 * e.g. in interrupt handler. That should remove this tricky
		 * multi-iteration logic simpler
		 */
		acthd1 = VGT_MMIO_READ(pdev, VGT_ACTHD(id));
		busy = wait_for_atomic(ring_is_empty(pdev, id), 50);
		for (i = 0; i < 3; i++) {
			if (!busy)
				break;

			vgt_info("(%d) check whether ring actually stops\n", i);
			acthd2 = VGT_MMIO_READ(pdev, VGT_ACTHD(id));
			if (acthd1 != acthd2) {
				vgt_info("ring still moves (%x->%x)\n",
					acthd1, acthd2);
				acthd1 = acthd2;
			}

			vgt_info("trigger another wait...\n");
			busy = wait_for_atomic(ring_is_empty(pdev, id),
				VGT_RING_TIMEOUT);
		}

		if (busy) {
			vgt_err("Ugh...it's a real hang!!!\n");
			return false;
		} else {
			vgt_warn("ring idle now... after extra wait\n");
		}
	}

	/* may do some jobs here to make sure ring idle */
	if (wait_for_atomic(ring_is_xxx(pdev, id), VGT_RING_TIMEOUT) != 0) {
		vgt_err("Timeout wait %d ms for ring(%d) xxx\n",
			VGT_RING_TIMEOUT, id);
		return false;
	}

	return true;
}

bool idle_rendering_engines(struct pgt_device *pdev, int *id)
{
	int i;

	/*
	 * Though engines are checked in order, no further CMDs
	 * are allowed from VM due to holding the big lock.
	 */
	for (i=0; i < pdev->max_engines; i++) {
		struct vgt_rsvd_ring *ring = &pdev->ring_buffer[i];

		if (!ring->need_switch)
			continue;

		if ( !idle_render_engine(pdev, i) ) {
			*id = i;
			return false;
		}
	}
	return true;
}

bool vgt_vrings_empty(struct vgt_device *vgt)
{
	int id;
	vgt_ringbuffer_t *vring;
	for (id = 0; id < vgt->pdev->max_engines; id++)
		if (test_bit(id, vgt->enabled_rings)) {
			vring = &vgt->rb[id].vring;
			if (vgt->pdev->enable_execlist) {
				int i;
				struct vgt_exec_list *el_slots;
				el_slots = vgt->rb[id].execlist_slots;
				for (i = 0; i < EL_QUEUE_SLOT_NUM; ++ i) {
					if (el_slots[i].status != EL_EMPTY)
						return false;
				}
			} else {
				if (!RB_HEAD_TAIL_EQUAL(vring->head, vring->tail))
					return false;
			}
		}

	return true;
}

void vgt_kick_off_execution(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	if (pdev->enable_execlist)
		vgt_kick_off_execlists(vgt);
	else
		vgt_kick_off_ringbuffers(vgt);
}

vgt_reg_t vgt_gen8_render_regs[] = {
	/* Hardware Status Page Address Registers. */
	_REG_HWS_PGA,
	0x12080,
	0x1a080,
	0x1c080,
	0x22080,

//	GEN8_PRIVATE_PAT,
//	GEN8_PRIVATE_PAT + 4,

	_REG_BCS_MI_MODE,
	_REG_BCS_BLT_MODE_IVB,
	_REG_BCS_INSTPM,
	_REG_BCS_HWSTAM,
	_REG_BCS_EXCC,

        /* Execlist Status Registers */
        _REG_RCS_EXECLIST_STATUS,
        _REG_VCS_EXECLIST_STATUS,
        _REG_VECS_EXECLIST_STATUS,
        _REG_VCS2_EXECLIST_STATUS,
        _REG_BCS_EXECLIST_STATUS,
	/*this register is from NONPRIV usage*/
	0x2248,
};

vgt_reg_t vgt_gen9_render_regs[] = {
	_REG_HWS_PGA,
	0x12080,
	0x1a080,
	0x1c080,
	0x22080,

	GEN8_PRIVATE_PAT_LO,
	GEN8_PRIVATE_PAT_HI,

	0x7004,
	0x2580,
	COMMON_SLICE_CHICKEN2,
	0x7300,
	0x20ec,

	/*this register is from NONPRIV usage*/
	GEN8_L3SQCREG4,

	0xe100,
	0xe180,
	0xe184,
	0xe188,
	0xe194,
	0xe4f0,

	0x4de0,
	0x4de4,
	0x4de8,
	0x4dec,
	0x4df0,
	0x4df4,

	0x24d0,
	0x24d4,
	0x24d8,
	0x24dc,

	_REG_VCS2_EXCC,
	_REG_VECS_EXCC,

	/* Execlist Status Registers */
	_REG_RCS_EXECLIST_STATUS,
	_REG_VCS_EXECLIST_STATUS,
	_REG_VECS_EXECLIST_STATUS,
	_REG_VCS2_EXECLIST_STATUS,
	_REG_BCS_EXECLIST_STATUS,
};

static vgt_reg_t *vgt_get_extra_ctx_regs(void)
{
	return &vgt_gen8_render_regs[0];
}

static int vgt_get_extra_ctx_regs_num(void)
{
	return ARRAY_NUM(vgt_gen8_render_regs);
}

static void __vgt_rendering_save(struct vgt_device *vgt, int num, vgt_reg_t *regs)
{
	vgt_reg_t	*sreg, *vreg;	/* shadow regs */
	int i;

	sreg = vgt->state.sReg;
	vreg = vgt->state.vReg;

	for (i=0; i<num; i++) {
		int reg = regs[i];
		//if (reg_hw_status(vgt->pdev, reg)) {
		/* FIXME: only hw update reg needs save */
		if (!reg_mode_ctl(vgt->pdev, reg))
		{
			__sreg(vgt, reg) = VGT_MMIO_READ(vgt->pdev, reg);
			__vreg(vgt, reg) = mmio_h2g_gmadr(vgt, reg, __sreg(vgt, reg));
			vgt_dbg(VGT_DBG_RENDER, "....save mmio (%x) with (%x)\n", reg, __sreg(vgt, reg));
		}
	}
}

static void gen9_save_mocs(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	u32 reg;

	for (reg = 0xc800; reg < 0xcff8; reg += 4)
		__vreg(vgt, reg) = VGT_MMIO_READ(pdev, reg);

	for (reg = 0xb020; reg < 0xb09c; reg += 4)
		__vreg(vgt, reg) = VGT_MMIO_READ(pdev, reg);
}

static void gen9_restore_mocs(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	u32 reg;

	for (reg = 0xc800; reg < 0xcff8; reg += 4) {
		VGT_MMIO_WRITE(pdev, reg, __vreg(vgt, reg));
		VGT_POST_READ(pdev, reg);
	}

	for (reg = 0xb020; reg < 0xb09c; reg += 4) {
		VGT_MMIO_WRITE(pdev, reg, __vreg(vgt, reg));
		VGT_POST_READ(pdev, reg);
	}
}

/* For save/restore global states difference between VMs.
 * Other context states should be covered by normal context switch later. */
static void vgt_rendering_save_mmio(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;

	/*
	 * both save/restore refer to the same array, so it's
	 * enough to track only save part
	 */
	pdev->in_ctx_switch = 1;

	if (IS_HSW(pdev))
		__vgt_rendering_save(vgt,
				vgt_get_extra_ctx_regs_num_gen7(),
				vgt_get_extra_ctx_regs_gen7());
	else if (IS_BDW(pdev))
		__vgt_rendering_save(vgt,
				vgt_get_extra_ctx_regs_num(),
				vgt_get_extra_ctx_regs());
	else if (IS_SKL(pdev)) {
		gen9_save_mocs(vgt);
		__vgt_rendering_save(vgt,
				ARRAY_NUM(vgt_gen9_render_regs),
				&vgt_gen9_render_regs[0]);
	}
	pdev->in_ctx_switch = 0;
}

static void __vgt_rendering_restore (struct vgt_device *vgt, int num_render_regs, vgt_reg_t *render_regs)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_reg_t	*sreg, *vreg;	/* shadow regs */
	vgt_reg_t  res_val; /*restored value of mmio register*/
	int i;

	sreg = vgt->state.sReg;
	vreg = vgt->state.vReg;

	for (i = 0; i < num_render_regs; i++) {
		int reg = render_regs[i];
		vgt_reg_t val = __sreg(vgt, reg);

		if (reg_mode_ctl(pdev, reg) && reg_aux_mode_mask(pdev, reg))
			val |= reg_aux_mode_mask(pdev, reg);

		/*
		 * FIXME: there's regs only with some bits updated by HW. Need
		 * OR vm's update with hw's bits?
		 */
		//if (!reg_hw_status(vgt->pdev, reg))
		VGT_MMIO_WRITE(vgt->pdev, reg, val);
		vgt_dbg(VGT_DBG_RENDER, "....restore mmio (%x) with (%x)\n", reg, val);

		/* Use this post-read as a workaround for a gpu hang issue */
		res_val = VGT_MMIO_READ(vgt->pdev, reg);

		if(!vgt_validate_ctx_switch)
			continue;
		if(res_val == val)
			continue;
		if (!reg_mode_ctl(pdev, reg) ||
			 ((res_val ^ val) & (reg_aux_mode_mask(pdev, reg) >> 16)))
			vgt_warn("restore %x: failed:  val=%x, val_read_back=%x\n",
				reg, val, res_val);
	}
}

/*
 * Restore MMIO registers per rendering context.
 * (Not include ring buffer registers).
 */
static void vgt_rendering_restore_mmio(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;

	if (IS_HSW(pdev))
		__vgt_rendering_restore(vgt,
				vgt_get_extra_ctx_regs_num_gen7(),
				vgt_get_extra_ctx_regs_gen7());
	else if (IS_BDW(pdev))
		__vgt_rendering_restore(vgt,
				vgt_get_extra_ctx_regs_num(),
				vgt_get_extra_ctx_regs());
	else if (IS_SKL(pdev)) {
		gen9_restore_mocs(vgt);
		__vgt_rendering_restore(vgt,
				ARRAY_NUM(vgt_gen9_render_regs),
				&vgt_gen9_render_regs[0]);
	}
}

void vgt_ring_init(struct pgt_device *pdev, int id)
{
	struct vgt_rsvd_ring *ring = &pdev->ring_buffer[id];

	ring->pdev = pdev;
	ring->id = id;
	ring->size = VGT_RSVD_RING_SIZE;

	if (IS_PREBDW(pdev)) {
		ring->start = aperture_2_gm(pdev,
				rsvd_aperture_alloc(pdev, ring->size));
		ring->virtual_start = v_aperture(pdev, ring->start);
	} else {
		ring->start = 0;
		ring->virtual_start = NULL;
	}
	ring->head = 0;
	ring->tail = 0;

	switch (id) {
	case RING_BUFFER_RCS:
		ring->stateless = 0;
		ring->need_switch = 1;
		break;
	case RING_BUFFER_VCS:
	case RING_BUFFER_VCS2:
	case RING_BUFFER_VECS:
		ring->stateless = 1;
		if (enable_video_switch)
			ring->need_switch = 1;
		else
			ring->need_switch = 0;
		break;
	case RING_BUFFER_BCS:
		ring->stateless = 1;
		ring->need_switch = 1;
		break;
	default:
		vgt_err("Unknown ring ID (%d)\n", id);
		ASSERT(0);
		break;
	}
}

static void dump_regs_on_err(struct pgt_device *pdev)
{
	static vgt_reg_t regs[] =  {
		0x2054,
		0x12054,
		0x22054,
		0x1A054,
		0xA098,
		0xA09C,
		0xA0A8,
		0xA0AC,
		0xA0B4,
		0xA0B8,
		0xA090,
		0xA094};

	int i;

	for (i = 0; i < ARRAY_SIZE(regs); i++)
		vgt_info("reg=0x%x, val=0x%x\n", regs[i],
			VGT_MMIO_READ(pdev, regs[i]));
}

static struct reg_mask_t gen8_rcs_reset_mmio[] = {
	{0x2098, 0},
	{0x20c0, 1},

	{0x24d0, 0},
	{0x24d4, 0},
	{0x24d8, 0},
	{0x24dc, 0},

#if 0
	{0xe4f0, 1},
	{0xe4f4, 1},
	{0xe184, 1},
#endif

	{0x7300, 1},
	{0x7004, 1},
	{0x7008, 1},

	{0x7000, 1},

	{0x7010, 1},

	{0x83a4, 1},
	{0x229c, 1},
};

static struct reg_mask_t gen9_rcs_reset_mmio[] = {
	{0x229c, 1},
};

static bool gen8plus_ring_switch(struct pgt_device *pdev,
		enum vgt_ring_id ring_id,
		struct vgt_device *prev,
		struct vgt_device *next)
{
	int count = 0;
	struct reg_mask_t *reset_mmio = NULL;
	int reg_num = 0;

	if (ring_id != RING_BUFFER_RCS)
		return true;

	if (IS_BDW(pdev)) {
		reg_num = ARRAY_SIZE(gen8_rcs_reset_mmio);
		reset_mmio = gen8_rcs_reset_mmio;
	}
	else if (IS_SKL(pdev)) {
		reg_num = ARRAY_SIZE(gen9_rcs_reset_mmio);
		reset_mmio = gen9_rcs_reset_mmio;
	}

	for (count = 0; count < reg_num; count++) {
		struct reg_mask_t *r = reset_mmio+count;
		__vreg(prev, r->reg) = VGT_MMIO_READ(pdev, r->reg);
	}

	/* Current policy:
	 * BDW render_engine_reset = 0
	 * SKL render_engine_reset = 1
	 */
	if (render_engine_reset) {

		VGT_MMIO_WRITE(pdev, 0x20d0, (1 << 16) | (1 << 0));

		for (count = 1000; count > 0; count --)
			if (VGT_MMIO_READ(pdev, 0x20d0) & (1 << 1))
				break;

		if (!count) {
			vgt_err("wait 0x20d0 timeout.\n");
			return false;
		}

		VGT_MMIO_WRITE(pdev, GEN6_GDRST, GEN6_GRDOM_RENDER);

		for (count = 1000; count > 0; count --)
			if (!(VGT_MMIO_READ(pdev, GEN6_GDRST) & GEN6_GRDOM_RENDER))
				break;

		if (!count) {
			vgt_err("wait gdrst timeout.\n");
			return false;
		}

		VGT_MMIO_WRITE(pdev, IMR, __sreg(vgt_dom0, IMR));
	}

	for (count = 0; count < reg_num; count++) {
		struct reg_mask_t *r = reset_mmio+count;
		vgt_reg_t v = __vreg(next, r->reg);
		if (r->mask)
			v |= 0xffff0000;

		VGT_MMIO_WRITE(pdev, r->reg, v);
		VGT_POST_READ(pdev, r->reg);
	}

	if (render_engine_reset)
		reset_phys_el_structure(pdev, ring_id);

	return true;
}

bool vgt_do_render_context_switch(struct pgt_device *pdev)
{
	int i = 0;
	int cpu;
	struct vgt_device *next, *prev;
	cycles_t t0, t1, t2;

	vgt_lock_dev(pdev, cpu);
	if (!ctx_switch_requested(pdev))
		goto out;

	ASSERT(spin_is_locked(&pdev->lock));
	vgt_force_wake_get();

	next = pdev->next_sched_vgt;
	prev = current_render_owner(pdev);
	ASSERT(pdev->next_sched_vgt);
	ASSERT(next != prev);

	t0 = vgt_get_cycles();
	if (!pdev->enable_execlist && !idle_rendering_engines(pdev, &i)) {
		int j;
		vgt_err("vGT: (%lldth switch<%d>)...ring(%d) is busy\n",
			vgt_ctx_switch(pdev),
			current_render_owner(pdev)->vgt_id, i);
		for (j = 0; j < 10; j++)
			printk("pHEAD(%x), pTAIL(%x)\n",
				VGT_READ_HEAD(pdev, i),
				VGT_READ_TAIL(pdev, i));
		goto err;
	}

	if (pdev->enable_execlist) {
		static int check_cnt = 0;
		int ring_id;
		for (ring_id = 0; ring_id < pdev->max_engines; ++ ring_id) {
			if (!pdev->ring_buffer[ring_id].need_switch)
				continue;
			if (!vgt_idle_execlist(pdev, ring_id)) {
				vgt_dbg(VGT_DBG_EXECLIST, "rendering ring is not idle. "
					"Ignore the context switch!\n");
				check_cnt++;
				vgt_force_wake_put();

				if (check_cnt > 500 && !idle_rendering_engines(pdev, &i)) {
					vgt_err("vGT: (%lldth switch<%d>)...ring(%d) is busy\n",
						vgt_ctx_switch(pdev),
					current_render_owner(pdev)->vgt_id, i);
					goto err;
				}

				goto out;
			}
			vgt_clear_submitted_el_record(pdev, ring_id);
		}

		check_cnt = 0;
	}

	vgt_dbg(VGT_DBG_RENDER, "vGT: next vgt (%d)\n", next->vgt_id);
	

	/* variable exported by debugfs */
	pdev->stat.context_switch_num ++;
	t1 = vgt_get_cycles();
	pdev->stat.ring_idle_wait += t1 - t0;
	prev->stat.schedule_out_time = t1;

	vgt_sched_update_prev(prev, t0);

	prev->stat.allocated_cycles +=
		(t0 - prev->stat.schedule_in_time);
	vgt_ctx_switch(pdev)++;

	/* STEP-1: manually save render context */
	vgt_rendering_save_mmio(prev);

	/* STEP-2: HW render context switch */
	for (i=0; i < pdev->max_engines; i++) {
		if (!pdev->ring_buffer[i].need_switch)
			continue;

		if (IS_PREBDW(pdev))
			gen7_ring_switch(pdev, i, prev, next);
		else
			gen8plus_ring_switch(pdev, i, prev, next);
	}

	/* STEP-3: manually restore render context */
	vgt_rendering_restore_mmio(next);

	/* STEP-4: restore ring buffer structure */
	for (i = 0; i < pdev->max_engines; i++)
		vgt_restore_ringbuffer(next, i);

	/* STEP-5: switch PPGTT */
	current_render_owner(pdev) = next;
	/* ppgtt switch must be done after render owner switch */
	if (!pdev->enable_execlist && pdev->enable_ppgtt && next->gtt.active_ppgtt_mm_bitmap)
		vgt_ppgtt_switch(next);

	/* STEP-6: ctx switch ends, and then kicks of new tail */
	vgt_kick_off_execution(next);

	/* NOTE: do NOT access MMIO after this PUT hypercall! */
	vgt_force_wake_put();

	/* request to check IRQ when ctx switch happens */
	if (prev->force_removal ||
		bitmap_empty(prev->enabled_rings, MAX_ENGINES)) {
		printk("Disable render for vgt(%d) from kthread\n",
			prev->vgt_id);
		vgt_disable_render(prev);
		wmb();
		if (prev->force_removal) {
			prev->force_removal = 0;
			if (waitqueue_active(&pdev->destroy_wq))
				wake_up(&pdev->destroy_wq);
		}
		/* no need to check if prev is to be destroyed */
	}

	next->stat.schedule_in_time = vgt_get_cycles();

	vgt_sched_update_next(next);

	t2 = vgt_get_cycles();
	pdev->stat.context_switch_cost += (t2-t1);
out:
	vgt_unlock_dev(pdev, cpu);
	return true;
err:
	dump_regs_on_err(pdev);
	/* TODO: any cleanup for context switch errors? */
	vgt_err("Ring-%d: (%lldth checks %lldth switch<%d->%d>)\n",
			i, vgt_ctx_check(pdev), vgt_ctx_switch(pdev),
			prev->vgt_id, next->vgt_id);
	vgt_err("FAIL on ring-%d\n", i);
	vgt_err("cur(%d): head(%x), tail(%x), start(%x)\n",
			current_render_owner(pdev)->vgt_id,
			current_render_owner(pdev)->rb[i].sring.head,
			current_render_owner(pdev)->rb[i].sring.tail,
			current_render_owner(pdev)->rb[i].sring.start);
	vgt_err("dom0(%d): head(%x), tail(%x), start(%x)\n",
			vgt_dom0->vgt_id,
			vgt_dom0->rb[i].sring.head,
			vgt_dom0->rb[i].sring.tail,
			vgt_dom0->rb[i].sring.start);
	show_ring_debug(pdev, i);
	show_ring_buffer(pdev, i, 16 * sizeof(vgt_reg_t));
	if (!enable_reset)
		/* crash system now, to avoid causing more confusing errors */
		ASSERT(0);

	/*
	 * put this after the ASSERT(). When ASSERT() tries to dump more
	 * CPU/GPU states: we want to hold the lock to prevent other
	 * vcpus' vGT related codes at this time.
	 */
	vgt_force_wake_put();

	vgt_unlock_dev(pdev, cpu);

	return false;
}
