/*
 * Render schedulers
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

/* Lets move context scheduler specific parameters here */
bool timer_based_qos = true;

struct vgt_hrtimer vgt_hrtimer;
struct pgt_device *vgt_hrtimer_pdev;
static void vgt_hrtimer_init(struct pgt_device *pdev,
	enum hrtimer_restart (*function)(struct hrtimer *),
	u64 period)
{
	struct vgt_hrtimer *hrtimer = &vgt_hrtimer;
	hrtimer_init(&hrtimer->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	hrtimer->timer.function = function;
	vgt_hrtimer_pdev = pdev;

	hrtimer_start(&hrtimer->timer,
			ktime_add_ns(ktime_get(), period),
			HRTIMER_MODE_ABS);
}

static void vgt_hrtimer_exit(struct pgt_device *pdev)
{
	hrtimer_cancel(&vgt_hrtimer.timer);
}

static inline bool phys_head_catch_tail(struct pgt_device *pdev)
{
	int ring_id;
	unsigned int reg_head, reg_tail;
	vgt_reg_t head, tail;

	for (ring_id = 0; ring_id < pdev->max_engines; ring_id++) {
		reg_head = RB_HEAD(pdev, ring_id);
		reg_tail = RB_TAIL(pdev, ring_id);
		head = VGT_MMIO_READ(pdev, reg_head);
		tail = VGT_MMIO_READ(pdev, reg_tail);
		if (!RB_HEAD_TAIL_EQUAL(head, tail))
			return false;
	}

	return true;
}


/* FIXME: Since it is part of "timer based scheduler",
 * move this from vgt_context.c here and renamed from
 * next_vgt() to tbs_next_vgt()
 */
static struct vgt_device *tbs_next_vgt(
	struct list_head *head, struct vgt_device *vgt)
{
	struct list_head *next = &vgt->list;
	struct vgt_device *next_vgt = NULL;
	struct pgt_device *pdev;

	if (vgt->force_removal)
		return vgt_dom0;

	pdev = vgt->pdev;
	if (ctx_switch_requested(pdev))
		return pdev->next_sched_vgt;

	do {
		next = next->next;
		/* wrap the list */
		if (next == head)
			next = head->next;
		next_vgt = list_entry(next, struct vgt_device, list);

		if (!vgt_vrings_empty(next_vgt))
			break;

	} while (next_vgt != vgt);

	return next_vgt;
}

/* safe to not use vgt_enter/vgt_exit, otherwise easily lead to deadlock */
static enum hrtimer_restart vgt_tbs_timer_fn(struct hrtimer *data)
{
	struct vgt_hrtimer *hrtimer = container_of(data,
			struct vgt_hrtimer, timer);
	struct pgt_device *pdev = vgt_hrtimer_pdev;

	ASSERT(pdev);

	if (vgt_nr_in_runq(pdev) > 1) {
		vgt_raise_request(pdev, VGT_REQUEST_SCHED);
	}
	/* we are safe to schedule next timeout with current vgt value
	 * (before ctx switch). If ctx switch successfully, we will cancel
	 * this timer and start new one with next vgt's tbs_period.
	 */
	hrtimer_add_expires_ns(&hrtimer->timer,
		ctx_tbs_period(current_render_owner(pdev)));
	return HRTIMER_RESTART;
}

/* tail queue operation */
static int init_vgt_tailq(struct vgt_tailq *tailq)
{
	int retval = 0;

	if (!tailq)
		return -EINVAL;

	tailq->__head = 0;
	tailq->__tail = 0;

	tailq->__buf_tail = vmalloc(VGT_TAILQ_SIZE);
	if (!tailq->__buf_tail)
		return -ENOMEM;

	tailq->__buf_cmdnr = vmalloc(VGT_TAILQ_SIZE);
	if (!tailq->__buf_cmdnr) {
		retval = -ENOMEM;
		goto free_buf_tail;
	}

	tailq->__buf_tail[0] = 0;
	tailq->__buf_cmdnr[0] = 0;

	return 0;

free_buf_tail:
	vfree(tailq->__buf_tail);
	return retval;
}

void destroy_vgt_tailq(struct vgt_tailq *tailq)
{
	/* FIXME: if need to check if it is empty ? */
	tailq->__head = 0;
	tailq->__tail = 0;

	if (tailq->__buf_tail) {
		vfree(tailq->__buf_tail);
		tailq->__buf_tail = NULL;
	}

	if (tailq->__buf_cmdnr) {
		vfree(tailq->__buf_cmdnr);
		tailq->__buf_cmdnr = NULL;
	}

}

static inline bool is_vgt_tailq_empty(struct vgt_tailq *tailq)
{
	return (tailq->__head == tailq->__tail);
}

static inline bool is_vgt_tailq_full(struct vgt_tailq *tailq)
{
	return (vgt_tailq_idx(tailq->__tail + 1) == tailq->__head);
}

static inline u32 vgt_tailq_cur_stail(struct vgt_tailq *tailq)
{
	return (tailq->__buf_tail[tailq->__head]);
}

inline u32 vgt_tailq_last_stail(struct vgt_tailq *tailq)
{
	return (tailq->__buf_tail[tailq->__tail]);
}

static inline int vgt_tailq_len(struct vgt_tailq *tailq)
{
	int len = tailq->__tail - tailq->__head;
	if (tailq->__tail < tailq->__head)
		len += VGT_TAILQ_MAX_ENTRIES;
	return len;
}

static inline void vgt_tailq_popall(struct vgt_tailq *tailq)
{
	tailq->__head = tailq->__tail;
}

/* catch each tail-writing */
int vgt_tailq_pushback(struct vgt_tailq *tailq, u32 tail, u32 cmdnr)
{
	u32 __tail;

	if (is_vgt_tailq_full(tailq))
		return -ENOSPC;

	__tail = vgt_tailq_idx(tailq->__tail + 1);
	tailq->__buf_tail[__tail] = tail;
	tailq->__buf_cmdnr[__tail] = cmdnr;

	tailq->__tail = __tail;
	return 0;
}

/* pop several tail-writing */
static int vgt_tailq_popfront(struct vgt_tailq *tailq, u32 nr)
{
	u32 __head = tailq->__head;

	if (nr > vgt_tailq_len(tailq))
		return -EACCES;

	tailq->__head = vgt_tailq_idx(__head + nr);
	return 0;
}

#define vgt_tailq_for_each_entry(idx, tailq) \
	for (idx = vgt_tailq_idx(tailq->__head + 1); \
		(tailq->__head != tailq->__tail) && (idx != vgt_tailq_idx(tailq->__tail + 1));\
		idx = vgt_tailq_idx(idx + 1))

/* Parameter:
 * @tailq:	Tail queue that we walk on
 * @cmd_nr:	Quantity of ring commands, that need to be
 *			executed in next round context switch.
 * @move_cnt:
 *			to get that quantity of ring commands, how many
 *			elements we need to pop out
 *
 * Return value:
 *			number of tail-writing that we need to pop-up
 */
static u32 __vgt_tailq_commit_cmd(struct vgt_tailq *tailq, u32 req_cmd_nr,
		u32 *move_cnt)
{
	u32 idx;
	u32 cmd_nr = 0, loop_cnt = 0;
	if (is_vgt_tailq_empty(tailq))
		return 0;

	vgt_tailq_for_each_entry(idx, tailq) {
		if (cmd_nr <= req_cmd_nr) {
			cmd_nr += tailq->__buf_cmdnr[idx];
			loop_cnt++;
		}
	}

	*move_cnt = loop_cnt;
	return cmd_nr;
}

static u32 vgt_tailq_commit_num_stail(struct vgt_device *vgt,
		int ring_id, u32 num)
{
	struct vgt_tailq *tailq = &vgt->rb_tailq[ring_id];
	vgt_reg_t stail;
	int len;

	len = vgt_tailq_len(tailq);
	if (num > len)
		num = len;

	vgt_tailq_popfront(tailq, num);
	stail = vgt_tailq_cur_stail(tailq);
	vgt->rb[ring_id].sring.tail = stail;

	return num;
}

static u32 vgt_tailq_commit_stail(struct vgt_device *vgt,
		int ring_id, u32 req_cmdnr)
{
	struct vgt_tailq *tailq = &vgt->rb_tailq[ring_id];
	u32 hmove;
	u32 actual_cmdnr;
	vgt_reg_t stail;

	actual_cmdnr = __vgt_tailq_commit_cmd(tailq, req_cmdnr, &hmove);
	if (actual_cmdnr == 0)
		return 0;

	if (vgt_tailq_popfront(tailq, hmove) != 0)
		return 0;

	stail = vgt_tailq_cur_stail(tailq);
	vgt->rb[ring_id].sring.tail = stail;

	return actual_cmdnr;
}



/* FIXME: default value of CMD number */
#define VGT_RCS_SCHED_DEFAULT_CMDNR  500
#define VGT_BCS_SCHED_DEFAULT_CMDNR  300

int vgt_init_rb_tailq(struct vgt_device *vgt)
{
	int retval = 0;
	int ring_id, from;

	for (ring_id = 0; ring_id < vgt->pdev->max_engines; ring_id++) {
		retval = init_vgt_tailq(&vgt->rb_tailq[ring_id]);
		if (retval < 0)
			goto cleanup;
	}
	return 0;

cleanup:
	for (from = 0; from < ring_id; from++)
		destroy_vgt_tailq(&vgt->rb_tailq[from]);

	return retval;
}

/* rb should be disabled before call this */
void vgt_destroy_rb_tailq(struct vgt_device *vgt)
{
	int ring_id;
	for (ring_id = 0; ring_id < vgt->pdev->max_engines; ring_id++)
		destroy_vgt_tailq(&vgt->rb_tailq[ring_id]);
}

//int rb_chk_set[] = {RING_BUFFER_RCS, RING_BUFFER_BCS, RING_BUFFER_VCS};
bool is_vgt_rb_tailq_empty(struct vgt_device *vgt, int max_engines)
{
	int ring_id;
	for (ring_id = 0; ring_id < max_engines; ring_id++) {
		if (test_bit(ring_id, (void *)vgt->started_rings)
				&& !is_vgt_tailq_empty(&vgt->rb_tailq[ring_id])) {
			/* check how many tail-writings can be cached */
			vgt_dbg(VGT_DBG_RENDER, "vGT(%d): rb(%d) tailq length(%d)\n",
					vgt->vgt_id, ring_id,
					vgt_tailq_len(&vgt->rb_tailq[ring_id]));
			return false;
		}
	}

	return true;
}


/* Use command(ring/batch buffer) number
 * to estimate how many stails
 * will be commited, this will be used soon
 */
bool vgt_rb_tailq_commit_stail(struct vgt_device *vgt,
		u32 *ring_req_cmdnr)
{
	u32 actual_cmdnr;
	int ring_id;
	for (ring_id = 0; ring_id < vgt->pdev->max_engines; ring_id++)
		if (test_bit(ring_id, (void*)vgt->enabled_rings))
			actual_cmdnr = vgt_tailq_commit_stail(vgt,
					ring_id,
					ring_req_cmdnr[ring_id]);

	return true;
}

bool vgt_rb_tailq_commit_num_stail(struct vgt_device *vgt,
		u32* req)
{
	u32 actual_move;
	int ring_id;
	for (ring_id = 0; ring_id < vgt->pdev->max_engines; ring_id++)
		if (test_bit(ring_id, (void *)vgt->enabled_rings))
			actual_move = vgt_tailq_commit_num_stail(vgt,
					ring_id,
					req[ring_id]);
	return true;
}

bool vgt_removal_req = false;
static struct vgt_device *ondemand_sched_next(struct pgt_device *pdev)
{
	struct vgt_device *cur_vgt = current_render_owner(pdev);
	struct vgt_device *next_vgt = NULL;
	struct list_head *next = &cur_vgt->list;
	struct list_head *head = &pdev->rendering_runq_head;

	do {
		next = next->next;
		/* wrap the list */
		if (next == head)
			next = head->next;
		next_vgt = list_entry(next, struct vgt_device, list);

		if (!cur_vgt->force_removal
				&& is_vgt_rb_tailq_empty(next_vgt, pdev->max_engines))
			continue;
		else
			break;
	} while (next_vgt != cur_vgt);

	if (cur_vgt->force_removal) {
		vgt_removal_req = true;
		printk("force_removal(%p), next_vgt(%p)\n", cur_vgt, next_vgt);
	}

	return next_vgt;
}

/* Command based scheduling */
static void ondemand_sched_ctx(struct pgt_device *pdev)
{
	struct vgt_device *cur_vgt = current_render_owner(pdev);
	struct vgt_device *next_vgt = ondemand_sched_next(pdev);
	/* default commit 5 tail writing at most */
	u32 tails_per_ring[MAX_ENGINES] = {5, 5, 5, 5};

	//if (is_vgt_rb_tailq_empty(next_vgt, pdev->max_engines))
	//	return;

	if (next_vgt == cur_vgt) {
		//FIXME: request 5 stails to be committed
		vgt_rb_tailq_commit_num_stail(next_vgt, tails_per_ring);
		vgt_kick_off_execution(next_vgt);
		return;
	}

	if (vgt_chk_raised_request(pdev, VGT_REQUEST_CTX_SWITCH)) {
		printk("Warning: last request for ctx_switch not handled yet!\n");
		/* this is a big change */
		//return;
	}

	/* set next vgt for ctx switch */
	vgt_rb_tailq_commit_num_stail(next_vgt, tails_per_ring);
	pdev->next_sched_vgt = next_vgt;
	vgt_raise_request(pdev, VGT_REQUEST_CTX_SWITCH);
}


//bool enable_tailq = false;
static enum hrtimer_restart vgt_poll_rb_tail(struct hrtimer *data)
{

	unsigned long flags;
	int active_nr;
	struct vgt_hrtimer *hrtimer = container_of(data,
			struct vgt_hrtimer, timer);
	struct pgt_device *pdev = vgt_hrtimer_pdev;
	int cpu;
	u64 hrtimer_period = VGT_TAILQ_RB_POLLING_PERIOD;

	ASSERT(pdev);

	vgt_lock_dev_flags(pdev, cpu, flags);
	/* TODO: if no more than 2 vgt in runqueue */
	active_nr = vgt_nr_in_runq(pdev);

	if (active_nr == 0)
		goto reload_timer;

#if 0
	if (((active_nr = vgt_nr_in_runq(pdev)) < 2)
			&& enable_tailq == true) {
		enable_tailq = false;
		// call disable tail queue
		// down the polling frequency ?
	} else if ((active_nr > 1)
			&& (enable_tailq == false)) {
		enable_tailq = true;
		// call enable tail queue
	}
#endif

	/* bspec said head and tail initially as 0 */
	if (phys_head_catch_tail(pdev))
		ondemand_sched_ctx(pdev);

reload_timer:
	vgt_unlock_dev_flags(pdev, cpu, flags);
	/* Slow down the polling as 16 ms to prevent the starvation
	 * of vgt_thread
	 */
	if (vgt_removal_req == true) {
		vgt_removal_req = false;
		hrtimer_period = (VGT_TAILQ_RB_POLLING_PERIOD << 3);
	} else
		hrtimer_period = VGT_TAILQ_RB_POLLING_PERIOD;

	hrtimer_add_expires_ns(&hrtimer->timer, hrtimer_period);

	return HRTIMER_RESTART;
}

void vgt_initialize_ctx_scheduler(struct pgt_device *pdev)
{
	ASSERT(pdev);
	/* If configured more than one,
	 * choose the one that has highest priority
	 */
	if (hvm_render_owner) {
		timer_based_qos = false;
		event_based_qos = false;
		shadow_tail_based_qos = false;
		return;
	}

	if (shadow_tail_based_qos) {
		vgt_hrtimer_init(pdev,
				vgt_poll_rb_tail,
				VGT_TAILQ_RB_POLLING_PERIOD);
		event_based_qos = false;
		timer_based_qos = false;
	} else if (event_based_qos)
		timer_based_qos = false;

	if (timer_based_qos) {
		ASSERT(current_render_owner(pdev));
		vgt_hrtimer_init(pdev,
				vgt_tbs_timer_fn,
				ctx_tbs_period(current_render_owner(pdev)));
	}
}

void vgt_cleanup_ctx_scheduler(struct pgt_device *pdev)
{
	ASSERT(pdev);

	if (event_based_qos)
		return;

	if (shadow_tail_based_qos || timer_based_qos)
		vgt_hrtimer_exit(pdev);

	pdev->next_sched_vgt = NULL;
}

/* internal facilities */
#if 0
static int64_t vgt_dec_tslice(struct vgt_device *vgt, vgt_tslice_t tslice)
{
	vgt_tslice_t *remained_tslice = &vgt->sched_info.time_slice;
	*remained_tslice -= tslice;
	return *remained_tslice;
}
#endif

static inline int64_t vgt_get_tslice(struct vgt_device *vgt)
{
	struct vgt_sched_info *sched_info = &vgt->sched_info;
	return sched_info->time_slice;
}
/* end of facilities functions */

static void vgt_alloc_tslice(struct vgt_device *vgt)
{
	/*
	 * Further this will rely on different policies
	 * */
	int64_t *ts = &(vgt->sched_info.time_slice);
	if (*ts > 0)
		return;
	vgt_dbg(VGT_DBG_RENDER, "vgt(%d): allocate tslice %lld\n",
			vgt->vgt_id,
			VGT_DEFAULT_TSLICE);

	*ts = VGT_DEFAULT_TSLICE;
#if 0
	/* Apply impact of statistics info */
	if (ctx_rb_empty_delay(vgt) > XXX )
		*ts = YYY;
	else
		*ts = ZZZ;
#endif
}


static void vgt_alloc_tslice_all(struct pgt_device *pdev)
{
	int i;
	struct vgt_device *vgt;

	/* FIXME: treat idle vgt differently
	 * 1) mark its idle state (add to statistics or something else)
	 * 2) If thought it is really idle, decrease its allocated time slice
	 * TODO: but the vgt triggered scheduler in its read/write handler,
	 * should not be regarded as idle.
	 */
	/* walk through the runqueue list */
	for (i = 0; i < VGT_MAX_VMS; i++) {
		vgt = pdev->device[i];
		if (vgt)
			vgt_alloc_tslice(vgt);
	}
}

/* pick up next vgt */
static struct vgt_device *vgt_sched_next(struct vgt_device *vgt,
		struct list_head *head)
{
	struct vgt_device *next_vgt = NULL;
	struct vgt_device *next_ctx_owner = vgt;
	struct vgt_sched_info *sched_info;
	struct list_head *next = &vgt->list;
	/* must used signed number for comparison */
	int64_t max_tslice = 0;

	do {
		next = next->next;
		if (next == head)
			next = head->next;
		next_vgt = list_entry(next, struct vgt_device, list);

		if (!vgt_vrings_empty(next_vgt)) {
			sched_info = &next_vgt->sched_info;
			if (sched_info->time_slice > max_tslice) {
				max_tslice = sched_info->time_slice;
				next_ctx_owner = next_vgt;
			}
		}
	} while (next_vgt != vgt);

	return next_ctx_owner;
}

void vgt_setup_countdown(struct vgt_device *vgt)
{
	vgt_tslice_t *start_time = &(vgt->sched_info.start_time);
	vgt_tslice_t *end_time = &(vgt->sched_info.end_time);
	int64_t *tslice = &(vgt->sched_info.time_slice);

	*start_time = vgt_get_cycles();

	ASSERT(*tslice > 0)
	*end_time = *start_time + (vgt_tslice_t)(*tslice);
}


static inline void vgt_sched_dump(struct vgt_device *cur_vgt, vgt_tslice_t cur_time)
{
	struct pgt_device *pdev = cur_vgt->pdev;
	struct vgt_device *vgt;
	printk("------------------------------------------\n");
	printk("     vgt scheduler dump vGT\n");
	printk("------------------------------------------\n");
	printk("....Current time (%llu)\n", cur_time);
	printk("....Current render owner vgt(%d))\n", (current_render_owner(pdev))->vgt_id);
	list_for_each_entry(vgt, &pdev->rendering_runq_head, list) {
		if (vgt == cur_vgt)
			printk("....vGT(%d) [dump caller]:\n", cur_vgt->vgt_id);
		else
			printk("....vGT(%d):\n", vgt->vgt_id);
		printk("........context start time (%llu)\n", ctx_start_time(vgt));
		printk("........context end time   (%llu)\n", ctx_end_time(vgt));
		printk("........context Remain time slice (%lld)\n", ctx_remain_time(vgt));
		printk("\n");
	}
}

/* TODO: call this for each eligible checkpoint */
int sched_next_failed = 0;

void vgt_sched_ctx(struct pgt_device *pdev)
{
	/* start of vgt context sheduling */
	vgt_tslice_t cur_time;
	struct vgt_device *next_vgt;
	struct vgt_device *cur_vgt = current_render_owner(pdev);
	struct list_head *head = &pdev->rendering_runq_head;

	/* TODO: before the first vgt was put into runqueue,
	 * the timestamp was used as the initial value of vgt_sched_tstamp
	 */
	if (vgt_nr_in_runq(pdev) <= 1)
		return;

	/* For the first time vgt_sched_ctx() called */
	if (ctx_start_time(cur_vgt) == 0) {
		vgt_setup_countdown(cur_vgt);
		return;
	}

	/* cycles counter will wrap in 126 years */
	cur_time = vgt_get_cycles();
	/* Two situations we need to consider
	 * 1) the vgt (render_owner) used up its time slice
	 * 2) the vgt (render_owner) become idle when its time slice left
	 *	  how to find out this:
	 *	  2.1) only check physical render engine
	 *	  2.2) check vhead/vtail of all rendering engines
	 * But now, for 2), lets just leave it TODO.
	 * */
	/* update remain time slice */
	ctx_remain_time(cur_vgt) = ctx_end_time(cur_vgt) - cur_time;

	/* time slice not used up */
	if (cur_time < ctx_end_time(cur_vgt)) {
		vgt_dbg(VGT_DBG_RENDER, "vgt(%d): cur_time(%lld), [%lld, %lld]\n",
				cur_vgt->vgt_id,
				cur_time,
				ctx_start_time(cur_vgt),
				ctx_end_time(cur_vgt)
				);
		return;
	}

	vgt_dbg(VGT_DBG_RENDER, "vgt(%d): tslice used up, cur_time(%lld), ctx_end_time(%lld)\n",
			cur_vgt->vgt_id,
			cur_time,
			ctx_end_time(cur_vgt));

	next_vgt = vgt_sched_next(cur_vgt, head);
	if (ctx_remain_time(next_vgt) <= 0) {
		vgt_alloc_tslice_all(pdev);
		next_vgt = vgt_sched_next(cur_vgt, head);
	}

	if (cur_vgt != next_vgt) {
		/* sometimes, it can be long to wait for the done of
		 * last context switch, so let's wait for it done
		 */
		if (vgt_chk_raised_request(pdev, VGT_REQUEST_CTX_SWITCH)) {
			return;
		}

		vgt_dbg(VGT_DBG_RENDER, "try to switch to vgt(%d), cur_time(%lld)\n", next_vgt->vgt_id, cur_time);
		vgt_dbg(VGT_DBG_RENDER, "vgt(%d): rb wait(%lld) to be empty\n",
				cur_vgt->vgt_id,
				ctx_rb_empty_delay(cur_vgt));

		/* set Global varaible next_sched_vgt for context switch */
		pdev->next_sched_vgt = next_vgt;
		vgt_raise_request(pdev, VGT_REQUEST_CTX_SWITCH);
	} else {
		/* setup countdown of cur_vgt for next round */
		vgt_setup_countdown(next_vgt);
	}
}

/* define which ring will be checked when trigger context scheduling */
void __raise_ctx_sched(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;

	if (vgt_runq_is_empty(pdev))
		return;

	/* we used to call scheduler until physical head equal to tail
	 * but it is unnecessary, the context switch logic helps to
	 * wait for head equal to tail
	 */
	vgt_sched_ctx(pdev);
}

/* cleanup for previous vgt instance */
void vgt_sched_update_prev(struct vgt_device *vgt, cycles_t time)
{
	/* cancel timer to avoid including context switch time */
	if (timer_based_qos)
		hrtimer_cancel(&vgt_hrtimer.timer);

	/* Records actual tsc when all rendering engines
	 * are stopped */
	if (event_based_qos) {
		ctx_actual_end_time(current_render_owner(vgt->pdev)) = time;
	}
}

/* prepare for next vgt instance */
void vgt_sched_update_next(struct vgt_device *vgt)
{
	if (timer_based_qos)
		hrtimer_start(&vgt_hrtimer.timer,
			ktime_add_ns(ktime_get(), ctx_tbs_period(vgt)),
			HRTIMER_MODE_ABS);

	/* setup countdown for next vgt context */
	if (event_based_qos) {
		vgt_setup_countdown(vgt);
	}
}

void vgt_schedule(struct pgt_device *pdev)
{
	ASSERT(spin_is_locked(&pdev->lock));

	if (vgt_nr_in_runq(pdev) < 2)
		return;

	pdev->next_sched_vgt = tbs_next_vgt(&pdev->rendering_runq_head,
			current_render_owner(pdev));
}


static int calculate_budget(struct vgt_device *vgt)
{
#if 0
	int budget;

	budget = vgt->allocated_cmds - vgt->submitted_cmds;
	/* call scheduler when budget is not enough */
	if (budget <= 0) {
		vgt_schedule(pdev);
		if (ctx_switch_requested(pdev))
			return;
	}
#endif

	return MAX_CMD_BUDGET;
}

void vgt_submit_commands(struct vgt_device *vgt, int ring_id)
{
	int cmd_nr;
	struct pgt_device *pdev = vgt->pdev;
	vgt_state_ring_t	*rs = &vgt->rb[ring_id];
	int budget;
	uint64_t submission_id;
	bool need_irq = rs->tail_list.cmd[rs->tail_list.tail].flags & F_CMDS_ISSUE_IRQ;
	unsigned long ip_offset = rs->tail_list.cmd[rs->tail_list.tail].ip_offset;

	/*
	 * No commands submision when context switch is in
	 * progress. Current owner is prevented from further
	 * submission to ensure quantum control, and new owner
	 * request will be recovered at end of ctx switch.
	 */
	if (ctx_switch_requested(pdev)) {
		vgt_dbg(VGT_DBG_RENDER, "<%d>: Hold commands in render ctx switch (%d->%d)\n",
			vgt->vm_id, current_render_owner(pdev)->vm_id,
			pdev->next_sched_vgt->vm_id);
		return;
	}

	/* kicks scheduler for non-owner write. */
	if (!is_current_render_owner(vgt)) {
		//vgt_schedule(pdev);
		return;
	}

	budget = calculate_budget(vgt);
	if (!budget)
		return;

	cmd_nr = get_submission_id(rs, budget, &submission_id);
	/* no valid cmd queued */
	if (cmd_nr == MAX_CMD_BUDGET) {
		vgt_dbg(VGT_DBG_RENDER, "VM(%d): tail write w/o cmd to submit\n",
			vgt->vm_id);
		return;
	}

	/*
	 * otherwise submit to GPU, even when cmd_nr is ZERO.
	 8 this is necessary, because sometimes driver may write
	 * old tail which must take real effect.
	 */
	apply_tail_list(vgt, ring_id, submission_id);
	pdev->ring_buffer[ring_id].need_irq = need_irq;
	pdev->ring_buffer[ring_id].ip_offset = ip_offset;
	vgt->total_cmds += cmd_nr;
	vgt->submitted_cmds += cmd_nr;
}

void vgt_request_force_removal(struct vgt_device *vgt)
{
	vgt->force_removal = 1;
	vgt->pdev->next_sched_vgt = vgt_dom0;
	vgt_raise_request(vgt->pdev, VGT_REQUEST_CTX_SWITCH);
	wmb();
}

static bool vgt_ring_check_offset_passed(struct pgt_device *pdev, int ring_id,
	u32 head, u32 tail, u32 offset)
{
	bool rc = true;
	/*
	 * Check if ring buffer is wrapped, otherwise there
	 * are remaining instruction in ringbuffer, but no
	 * interrupt anymore, so switch to polling mode.
	 */
	rc = (head > tail) ? false : true;

	if (head > offset)
		rc = (offset > tail) ? true : rc;
	else
		rc = (offset > tail) ? rc : false;

	return rc;
}

static bool vgt_rings_need_idle_notification(struct pgt_device *pdev)
{
	int i;
	u32 head, tail, offset;

	for (i=0; i < pdev->max_engines; i++) {
		if (pdev->ring_buffer[i].need_irq) {
			head = VGT_MMIO_READ(pdev, RB_HEAD(pdev, i)) & RB_HEAD_OFF_MASK;
			tail = VGT_MMIO_READ(pdev, RB_TAIL(pdev, i)) & RB_TAIL_OFF_MASK;
			if (head != tail) {
				offset = pdev->ring_buffer[i].ip_offset;
				if (!vgt_ring_check_offset_passed(pdev, i, head, tail, offset))
					break;
			}
		}
	}
	/*
	 * If all the rings has been checked, mean there are no more user
	 * interrupt instructions remain in the ring buffer, so switch to
	 * polling mode, otherwise return false and wait for next interrupt.
	 */
	return (i == pdev->max_engines) ? true : false;
}

void vgt_check_pending_context_switch(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;

	if (pdev->ctx_switch_pending) {
		if (vgt_rings_need_idle_notification(pdev)) {
			pdev->ctx_switch_pending = false;
			vgt_raise_request(pdev, VGT_REQUEST_CTX_SWITCH);
		}
	}
}

bool vgt_do_render_sched(struct pgt_device *pdev)
{
	int threshold = 500; /* print every 500 times */
	int cpu;
	bool rc = true;

	if (!(vgt_ctx_check(pdev) % threshold))
		vgt_dbg(VGT_DBG_RENDER, "vGT: %lldth checks, %lld switches\n",
			vgt_ctx_check(pdev), vgt_ctx_switch(pdev));
	vgt_ctx_check(pdev)++;

	ASSERT(!vgt_runq_is_empty(pdev));

	/*
	 * disable interrupt which is sufficient to prevent more
	 * cmds submitted by the current owner, when dom0 is UP.
	 * if the mmio handler for HVM is made into a thread,
	 * simply a spinlock is enough. IRQ handler is another
	 * race point
	 */
	vgt_lock_dev(pdev, cpu);

	vgt_schedule(pdev);

	if (ctx_switch_requested(pdev)) {
		if ((!irq_based_ctx_switch) || vgt_rings_need_idle_notification(pdev))
			vgt_raise_request(pdev, VGT_REQUEST_CTX_SWITCH);
		else
			pdev->ctx_switch_pending = true;
	}

	vgt_unlock_dev(pdev, cpu);
	return rc;
}
