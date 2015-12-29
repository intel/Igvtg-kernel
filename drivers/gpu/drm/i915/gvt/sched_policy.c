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

static bool instance_has_pending_workload(struct vgt_device *vgt)
{
	struct gvt_virtual_execlist_info *info;
	int i;

	for (i = 0; i < I915_NUM_RINGS; i++) {
		info = &vgt->virtual_execlist_info[i];
		if (!list_empty(workload_q_head(vgt, i)))
			return true;
	}

	return false;
}

static void try_to_schedule_next_instance(struct pgt_device *pdev)
{
	struct gvt_workload_scheduler *scheduler =
			&pdev->workload_scheduler;
	int i;

	/* no target to schedule */
	if (!scheduler->next_instance)
		return;

	gvt_dbg_sched("try to schedule next instance %d",
			scheduler->next_instance->id);

	/*
	 * after the flag is set, workload dispatch thread will
	 * stop dispatching workload for current instance
	 */
	scheduler->need_reschedule = true;

	/* still have uncompleted workload? */
	for (i = 0; i < I915_NUM_RINGS; i++) {
		if (scheduler->current_workload[i]) {
			gvt_dbg_sched("still have running workload");
			return;
		}
	}

	gvt_dbg_sched("switch to next instance %d",
			scheduler->next_instance->id);

	/* switch current instance */
	scheduler->current_instance = scheduler->next_instance;
	scheduler->next_instance = NULL;

	/* wake up workload dispatch thread */
	for (i = 0; i < I915_NUM_RINGS; i++)
		wake_up(&scheduler->waitq[i]);

	scheduler->need_reschedule = false;
}

struct tbs_instance_data {
	struct list_head list;
	struct vgt_device *vgt;
	/* put some per-instance sched stats here*/
};

struct tbs_sched_data {
	struct pgt_device *pdev;
	struct delayed_work work;
	unsigned long period;
	atomic_t runq_instance_num;
	struct list_head runq_head;
};

#define GVT_DEFAULT_TIME_SLICE (16 * HZ / 1000)

static void tbs_sched_func(struct work_struct *work)
{
	struct tbs_sched_data *sched_data = container_of(work,
			struct tbs_sched_data, work.work);
	struct tbs_instance_data *instance_data;

	struct pgt_device *pdev = sched_data->pdev;
	struct gvt_workload_scheduler *scheduler =
			&pdev->workload_scheduler;

	struct vgt_device *vgt = NULL;
	struct list_head *pos, *head;

	mutex_lock(&pdev->lock);

	/* no instance or has already had a target */
	if (list_empty(&sched_data->runq_head)|| scheduler->next_instance)
		goto out;

	if (scheduler->current_instance) {
		instance_data = scheduler->current_instance->sched_data;
		head = &instance_data->list;
	} else {
		gvt_dbg_sched("no current instance search from q head");
		head = &sched_data->runq_head;
	}

	/* search a instance with pending workload */
	list_for_each(pos, head) {
		if (pos == &sched_data->runq_head)
			continue;

		instance_data = container_of(pos, struct tbs_instance_data, list);
		if (!instance_has_pending_workload(instance_data->vgt))
			continue;

		vgt = instance_data->vgt;
		break;
	}

	if (vgt) {
		scheduler->next_instance = vgt;
		gvt_dbg_sched("pick next instance %d", vgt->id);
	}
out:
	if (scheduler->next_instance) {
		gvt_dbg_sched("try to schedule next instance %d",
				scheduler->next_instance->id);
		try_to_schedule_next_instance(pdev);
	}

	/*
	 * still have instance on runq
	 * or last schedule haven't finished due to running workload
	 */
	if (atomic_read(&sched_data->runq_instance_num) || scheduler->next_instance)
		schedule_delayed_work(&sched_data->work, sched_data->period);

	mutex_unlock(&pdev->lock);
}

static bool tbs_sched_init(struct pgt_device *pdev)
{
	struct gvt_workload_scheduler *scheduler =
		&pdev->workload_scheduler;

	struct tbs_sched_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		gvt_err("fail to allocate sched data");
		return false;
	}

	INIT_LIST_HEAD(&data->runq_head);
	INIT_DELAYED_WORK(&data->work, tbs_sched_func);
	data->period = GVT_DEFAULT_TIME_SLICE;
	data->pdev = pdev;

	atomic_set(&data->runq_instance_num, 0);
	scheduler->sched_data = data;

	return true;
}

static void tbs_sched_clean(struct pgt_device *pdev)
{
	struct gvt_workload_scheduler *scheduler =
		&pdev->workload_scheduler;
	struct tbs_sched_data *data = scheduler->sched_data;

	cancel_delayed_work(&data->work);
	kfree(data);
	scheduler->sched_data = NULL;
}

static bool tbs_sched_instance_init(struct vgt_device *vgt)
{
	struct tbs_instance_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		gvt_err("fail to allocate memory");
		return false;
	}

	data->vgt = vgt;
	INIT_LIST_HEAD(&data->list);

	vgt->sched_data = data;

	return true;
}

static void tbs_sched_instance_clean(struct vgt_device *vgt)
{
	kfree(vgt->sched_data);
	vgt->sched_data = NULL;
}

static void tbs_sched_start_schedule(struct vgt_device *vgt)
{
	struct tbs_sched_data *sched_data = vgt->pdev->workload_scheduler.sched_data;
	struct tbs_instance_data *instance_data = vgt->sched_data;

	if (!list_empty(&instance_data->list))
		return;

	list_add_tail(&instance_data->list, &sched_data->runq_head);
	atomic_inc(&sched_data->runq_instance_num);

	schedule_delayed_work(&sched_data->work, sched_data->period);
}

static void tbs_sched_stop_schedule(struct vgt_device *vgt)
{
	struct tbs_sched_data *sched_data = vgt->pdev->workload_scheduler.sched_data;
	struct tbs_instance_data *instance_data = vgt->sched_data;

	atomic_dec(&sched_data->runq_instance_num);
	list_del_init(&instance_data->list);
}

struct gvt_schedule_policy_ops tbs_schedule_ops = {
	.init = tbs_sched_init,
	.clean = tbs_sched_clean,
	.instance_init = tbs_sched_instance_init,
	.instance_clean = tbs_sched_instance_clean,
	.start_schedule = tbs_sched_start_schedule,
	.stop_schedule = tbs_sched_stop_schedule,
};

bool gvt_init_sched_policy(struct pgt_device *pdev)
{
	pdev->workload_scheduler.sched_ops = &tbs_schedule_ops;

	return pdev->workload_scheduler.sched_ops->init(pdev);
}

void gvt_clean_sched_policy(struct pgt_device *pdev)
{
	pdev->workload_scheduler.sched_ops->clean(pdev);
}

bool gvt_init_instance_sched_policy(struct vgt_device *vgt)
{
	return vgt->pdev->workload_scheduler.sched_ops->instance_init(vgt);
}

void gvt_clean_instance_sched_policy(struct vgt_device *vgt)
{
	vgt->pdev->workload_scheduler.sched_ops->instance_clean(vgt);
}

void gvt_start_schedule(struct vgt_device *vgt)
{
	gvt_info("[vgt %d] start schedule", vgt->id);

	vgt->pdev->workload_scheduler.sched_ops->start_schedule(vgt);
}

void gvt_stop_schedule(struct vgt_device *vgt)
{
	struct gvt_workload_scheduler *scheduler =
		&vgt->pdev->workload_scheduler;

	gvt_info("[vgt %d] stop schedule", vgt->id);

	scheduler->sched_ops->stop_schedule(vgt);

	if (scheduler->next_instance == vgt)
		scheduler->next_instance = NULL;

	if (scheduler->current_instance == vgt) {
		/* stop workload dispatching */
		scheduler->need_reschedule = true;
		scheduler->current_instance = NULL;
	}
}
