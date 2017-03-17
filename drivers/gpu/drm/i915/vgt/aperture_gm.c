/*
 * Aperture and Graphics Memory (GM) virtualization
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

#include <linux/highmem.h>
#include "vgt.h"

/*
 * Guest to host GMADR (include aperture) converting.
 *
 * handle in 4 bytes granule
 */
int mmio_g2h_gmadr(struct vgt_device *vgt, unsigned long reg,
		vgt_reg_t g_value, vgt_reg_t *h_value)
{
	struct pgt_device *pdev = vgt->pdev;
	uint64_t h_val;
	vgt_reg_t mask;
	uint32_t size;
	int ret = 0;

	if (!reg_addr_fix(pdev, reg)) {
		*h_value = g_value;
		return 0;
	}

	ASSERT((reg < _REG_FENCE_0_LOW) || (reg >= _REG_FENCE_0_LOW + VGT_FENCE_REGION_SIZE));

	mask = reg_aux_addr_mask(pdev, reg);
	vgt_dbg(VGT_DBG_MEM, "vGT: address fix g->h for reg (0x%lx) value (0x%x) mask (0x%x)\n", reg, g_value, mask);
	/*
	 * NOTE: address ZERO is special, and sometimes the driver may hard
	 * code address ZERO, e.g. in curbase setting (when the cursor becomes
	 * invisible). So we always translate address ZERO into the valid
	 * range of the VM. If this doesn't work, we need change the driver!
	 */
	if (!(g_value & mask)) {
		vgt_dbg(VGT_DBG_MEM, "vGT(%d): translate address ZERO for reg (%lx)\n",
			vgt->vgt_id, reg);
		g_value = (vgt_guest_visible_gm_base(vgt) & mask) |
			  (g_value & ~mask);
	}

	h_val = g_value & mask;
	size = reg_aux_addr_size(pdev, reg);
	ret = g2h_gm_range(vgt, &h_val, size);
	if (ret < 0) {
		vgt_err("vGT(%d): Failed to convert guest graphics memory address: g_value(0x%x), size(0x%x)\n",
				vgt->vgt_id, g_value, size);

		return ret;
	}

	vgt_dbg(VGT_DBG_MEM, "....(g)%x->(h)%llx\n", g_value, (h_val & mask) | (g_value & ~mask));

	*h_value =  (h_val & mask) | (g_value & ~mask);
	return 0;
}

/*
 * Host to guest GMADR (include aperture) converting.
 *
 * handle in 4 bytes granule
 */
vgt_reg_t mmio_h2g_gmadr(struct vgt_device *vgt, unsigned long reg, vgt_reg_t h_value)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_reg_t g_value;
	vgt_reg_t mask;

	if (!reg_addr_fix(pdev, reg))
		return h_value;

	vgt_dbg(VGT_DBG_MEM, "vGT: address fix h->g for reg (%lx)(%x)\n", reg, h_value);
	mask = reg_aux_addr_mask(pdev, reg);

	/*
	 * it's possible the initial state may not contain a valid address
	 * vm's range. In such case fake a valid address since the value there
	 * doesn't matter.
	 */
	if (!h_gm_is_valid(vgt, h_value & mask)) {
		vgt_dbg(VGT_DBG_MEM, "!!!vGT: reg (%lx) doesn't contain a valid host address (%x)\n", reg, h_value);
		h_value = (vgt_visible_gm_base(vgt) & mask) | (h_value & ~mask);
	}

	g_value = h2g_gm(vgt, h_value & mask);
	vgt_dbg(VGT_DBG_MEM, "....(h)%x->(g)%x\n", h_value, (g_value & mask) | (h_value & ~mask));
	return (g_value & mask) | (h_value & ~mask);
}

/* Allocate pages in reserved aperture.
 * Return 0 if failed.
 */
unsigned long rsvd_aperture_alloc(struct pgt_device *pdev, unsigned long size)
{
	unsigned long start, nr_pages;

	ASSERT(size > 0);

	nr_pages = (size + PAGE_SIZE - 1) >> PAGE_SHIFT;
	start = bitmap_find_next_zero_area( pdev->rsvd_aperture_bitmap,
			VGT_RSVD_APERTURE_BITMAP_BITS, 0, nr_pages, 0 );

	if (start >= VGT_RSVD_APERTURE_BITMAP_BITS) {
		vgt_err("Out of memory for reserved aperture allocation "
				"of size 0x%lx!\n", size);
		return 0;
	}

	bitmap_set(pdev->rsvd_aperture_bitmap, start, nr_pages);

	return pdev->rsvd_aperture_base + (start << PAGE_SHIFT);
}

/* free pages in reserved aperture */
void rsvd_aperture_free(struct pgt_device *pdev, unsigned long start, unsigned long size)
{
	unsigned long nr_pages = (size + PAGE_SIZE - 1) >> PAGE_SHIFT;

	if ( (start >= pdev->rsvd_aperture_base) &&
			(start + size <= pdev->rsvd_aperture_base + pdev->rsvd_aperture_sz) )
	{
		bitmap_clear(pdev->rsvd_aperture_bitmap,
				(start - pdev->rsvd_aperture_base)>>PAGE_SHIFT, nr_pages);
	} else {
		vgt_err("Out of range parameter for rsvd_aperture_free(pdev, "
			"start[0x%lx], size[0x%lx])!\n", start, size);
	}
}

void rsvd_aperture_runout_handler(struct pgt_device *pdev)
{
	pdev->dummy_vm_switch = true;
}

ssize_t get_avl_vm_aperture_gm_and_fence(struct pgt_device *pdev, char *buf,
		ssize_t buf_sz)
{
	unsigned long aperture_guard = phys_aperture_sz(pdev) / SIZE_1MB;
	unsigned long gm_guard = gm_sz(pdev) / SIZE_1MB;
	unsigned long fence_guard = VGT_FENCE_BITMAP_BITS;
	unsigned long available_low_gm_sz = 0;
	unsigned long available_high_gm_sz = 0;
	int i;
	ssize_t buf_len = 0;
#define MAX_NR_RES 2
	unsigned long *bitmap[MAX_NR_RES];
	int bitmap_sz[MAX_NR_RES];

	available_low_gm_sz = aperture_guard - bitmap_weight(pdev->gm_bitmap,
	  aperture_guard);
	available_high_gm_sz = gm_guard - bitmap_weight(pdev->gm_bitmap, gm_guard)
	  - available_low_gm_sz;
	buf_len = snprintf(buf, buf_sz, "0x%08lx, 0x%08lx, 0x%08lx, "
			"0x%08lx, 0x%08lx, 0x%08lx\n",
			aperture_guard,
			available_low_gm_sz,
			gm_guard - aperture_guard,
			available_high_gm_sz,
			fence_guard,
			fence_guard - bitmap_weight(pdev->fence_bitmap, fence_guard)
			);

#define init_resource_bitmap(i, map, sz) \
	ASSERT((i) <  MAX_NR_RES);	\
	bitmap[i] = map;	\
	bitmap_sz[i] = sz;
	/* gm */
	init_resource_bitmap(0, pdev->gm_bitmap, gm_guard);
	/* fence registers */
	init_resource_bitmap(1, pdev->fence_bitmap, fence_guard);

	for (i = 0; i < MAX_NR_RES; i++) {
		buf_len += snprintf(buf + buf_len, buf_sz - buf_len,
					"%*pb\n", bitmap_sz[i], bitmap[i]);
	}

	return buf_len;
}

/**
 * search hidden gm slots for a vGPU
 *
 * Physical hidden GM is partitioned into 64M-sized slots. Each vGPU can
 * take several contiguous slots to serve as a piece of continuous hidden
 * graphics memory.
 *
 * In this algorithm, if a vGPU is not willing to share slots with other
 * vGPUs, it will search from @start to @end for @nr_slots contigous slots
 * that are not yet taken by other vGPUs, i.e.,
 * slot->current_vgpu_num is 0;
 *
 * If a vGPU is willing to share slots with other vGPUs, it will search
 * from @start to @end for @nr_slots contigous slots which are least
 * shared with other vGPUs.
 * In order to find such contigous slots, a score is calculated for each
 * search. After Nth search, (N <= end - start - nr_slots + 1), slots
 * with the lowest score is found, which means those slots are current
 * least shared available. Then the index of the first slot of the found
 * slots is returned @index_found.
 *
 * An example for this algorithm is like this:
 * Suppose a vGPU is going to take 384M hidden GM, i.e. 6 hidden GM slots,
 * and total physical hidden GM is 3G, i.e., 48 slots.
 * First, score for slots 0-5 is calculated, then slots 1-6, 2-7, ...,
 * 42-47. slots with least score are chosen.
 *
 * The algorithm to calculate score is like this:
 * On the start of each search, score is set to 0.
 * 1. if a slot is not taken by any vGPU yet, so after it's chosen, no pte
 * copy is required. 0 is added to the score;
 * 2. if a slot is already taken by N vGPUs, so after it's chosen, ptes of
 * this slot are demanding for copy during context switch, therefore a
 * "weight" is added to the score. The "weight" is deliberately set to a
 * very large value to ensure that less slots shared is always favored.
 * After that, we add a fine-grained N to the score for balance
 * consideration. E.g.
 * suppose weight = 1024, and it is found that
 * a ) there are 6 slots taken by 1 1 1 1 1 1 vGPUs, then for the 6 slots,
 * score = 1024 * 6 + 6 = 6150;
 * b ) there are 6 slots taken by 0 0 0 0 0 6 vGPUs, which means the first
 * 5 slots are not taken by any vGPUs, and the last slot is taken by 6 slots
 * already, then score = 1024 + 6 = 1030.
 * The lower score means the less ptes need to be copied for GGTT during
 * context switch, and therefore is preferred.
 *
 * @vgt: structure for the target vGPU
 * @nr_slots: total count of hidden gm slots the vGPU is going to take
 * @start: an index that specifies from which slot to search from.
 * @end: an index that specifies to which slot the search is end.
 *	Usually it's the index of the last slot in hidden gm.
 * @index_found: return the slot index found.
 *	This index is the slot index of the vGPU's hidden GM base address
 */

static int vgt_find_hidden_gm_slots(struct vgt_device *vgt,
	unsigned int nr_slots, unsigned int start, unsigned int end,
	unsigned int *index_found)
{
	struct pgt_device *pdev = vgt->pdev;
	unsigned int weight = VGT_HIGHGM_SLOT_MAX_NUM * VGT_MAX_VMS;
	unsigned int min_score = -1U;

	ASSERT(start < VGT_HIGHGM_SLOT_MAX_NUM);
	ASSERT(end < VGT_HIGHGM_SLOT_MAX_NUM);

	*index_found = start;
	end = end - nr_slots + 1;

	if (start > end)
		return -ENOMEM;

	while (start <= end) {
		int i;
		unsigned int score = 0;

		for (i = 0; i < nr_slots; i++) {
			struct hidden_gm_slot *slot =
				hidden_gm_slotidx_to_slot(pdev, start + i);

			ASSERT(slot->current_vgpu_num <=
				slot->max_vgpu_num);

			/* slot is full or refuse to be taken, go to next
			 * one
			 */
			if (slot->current_vgpu_num == slot->max_vgpu_num)
				break;

			/* weight is deliberately set to a very large value
			 * to ensure that less slots shared is favored.
			 * if a slot is not taken by any vGPU yet, none pte
			 * copies are required. So 0 is added to the score;
			 * if a slot is already taken by "N" vGPUs, ptes
			 * copies are required, so weight is added to the
			 * score. After that, a fine-grained "N" is added
			 * for balance consideration.
			 */
			score += slot->current_vgpu_num;
			if (slot->current_vgpu_num)
				score += weight;

			/* if the vGPU is not willing to share high gm with
			 * others, only find N contiguous empty slots
			 */
			if (!vgt->share_hidden_gm && score)
				break;
		}
		if (i < nr_slots) {
			/* no suitable slots, go to next slot or return
			 * failure
			 */
			start += i + 1;

			if (start > end)
				return -ENOMEM;

			continue;
		}

		if (score < min_score) {
			min_score = score;
			*index_found = start;
		}
		if (score == 0)
			break;

		start++;

	}
	return 0;
}

static int allocate_hidden_gm(struct vgt_device *vgt,
	int size_in_mb, unsigned long *hidden_gm_start)
{
	struct pgt_device *pdev = vgt->pdev;
	int i;

	size_in_mb = (size_in_mb + VGT_HIGHGM_SLOT_SZ - 1) &
			(VGT_HIGHGM_SLOT_MASK);
	vgt->hidden_gm_slot_num = size_in_mb / VGT_HIGHGM_SLOT_SZ;

	if (vgt_find_hidden_gm_slots(vgt,
			vgt->hidden_gm_slot_num,
			0,
			hidden_gm_sz(pdev)/
				(VGT_HIGHGM_SLOT_SZ*SIZE_1MB) - 1,
			&vgt->hidden_gm_slot_start_idx))
		return -ENOMEM;

	*hidden_gm_start = hidden_gm_base(pdev)/SIZE_1MB +
		vgt->hidden_gm_slot_start_idx * VGT_HIGHGM_SLOT_SZ;

	for (i = 0; i < vgt->hidden_gm_slot_num; i++) {
		struct hidden_gm_slot *slot =
			vgt_hidden_gm_slotidx_to_slot(vgt, i);

		slot->current_vgpu_num++;
		if (!vgt->share_hidden_gm) {
			slot->max_vgpu_num = 1;
			/* if a slot is not shared, then its owner can be
			 * determined here, while if it's shared, its owner
			 * will be determined when a vGPU becomes a render
			 * owner
			 */
			slot->owner = vgt;
		}
	}

	vgt_info("vgt-%d hidden memory from %ldM, size %dM, %s shared\n",
		vgt->vm_id, *hidden_gm_start, size_in_mb,
		vgt->share_hidden_gm ? "" : "not");
	return 0;
}

int allocate_vm_aperture_gm_and_fence(struct vgt_device *vgt, vgt_params_t vp)
{
	struct pgt_device *pdev = vgt->pdev;
	unsigned long *gm_bitmap = pdev->gm_bitmap;
	unsigned long *fence_bitmap = pdev->fence_bitmap;
	unsigned long guard = hidden_gm_base(vgt->pdev)/SIZE_1MB;
	unsigned long aperture_search_start = 0;
	unsigned long visable_gm_start, hidden_gm_start = guard;
	unsigned long fence_base;
	int i=0;

	ASSERT(vgt->aperture_base == 0); /* not allocated yet*/

	if (vp.aperture_sz <= 0 || vp.aperture_sz > vp.gm_sz) {
		vgt_err("Aperture size error(%d).", vp.aperture_sz);
		return -EINVAL;
	}
	if (vp.fence_sz <= 0) {
		vgt_err("Fence size error(%d).", vp.fence_sz);
		return -EINVAL;
	}

	visable_gm_start = bitmap_find_next_zero_area(gm_bitmap, guard,
				aperture_search_start, vp.aperture_sz, 0);
	if (visable_gm_start >= guard)
		return -ENOMEM;

	if (vp.gm_sz > vp.aperture_sz) {
		int hidden_gm_sz = vp.gm_sz - vp.aperture_sz;

		if (allocate_hidden_gm(vgt, hidden_gm_sz,
			&hidden_gm_start))
			return -ENOMEM;
	}
	fence_base = bitmap_find_next_zero_area(fence_bitmap,
				VGT_FENCE_BITMAP_BITS, 0, vp.fence_sz, 0);
	if (fence_base >= VGT_MAX_NUM_FENCES)
		return -ENOMEM;

	vgt->aperture_base = phys_aperture_base(vgt->pdev) +
			(visable_gm_start * SIZE_1MB);
	vgt->aperture_sz = vp.aperture_sz * SIZE_1MB;
	vgt->gm_sz = vp.gm_sz * SIZE_1MB;
	vgt->hidden_gm_offset = hidden_gm_start * SIZE_1MB;
	vgt->fence_base = fence_base;
	vgt->fence_sz = vp.fence_sz;

	/* mark the related areas as BUSY. */
	bitmap_set(gm_bitmap, visable_gm_start, vp.aperture_sz);
	if ((vp.gm_sz > vp.aperture_sz) && !vgt->share_hidden_gm) {
		/* only mark non-shared hidden gm space in bitmap*/
		bitmap_set(gm_bitmap, hidden_gm_start, vp.gm_sz - vp.aperture_sz);
	}
	bitmap_set(fence_bitmap, fence_base, vp.fence_sz);

	for (i = vgt->fence_base; i < vgt->fence_base + vgt->fence_sz; i++){
		VGT_MMIO_WRITE_BYTES(pdev,
			_REG_FENCE_0_LOW + 8 * i,
			0, 8);
	}

	return 0;
}

void free_vm_aperture_gm_and_fence(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	unsigned long *gm_bitmap = pdev->gm_bitmap;
	unsigned long *fence_bitmap = pdev->fence_bitmap;
	unsigned long visable_gm_start =
		aperture_2_gm(vgt->pdev, vgt->aperture_base)/SIZE_1MB;
	unsigned long hidden_gm_start = vgt->hidden_gm_offset/SIZE_1MB;
	int i=0;

	/* mark the related areas as available */
	bitmap_clear(gm_bitmap, visable_gm_start, vgt->aperture_sz/SIZE_1MB);
	if (vgt->gm_sz > vgt->aperture_sz)
		bitmap_clear(gm_bitmap, hidden_gm_start,
			(vgt->gm_sz - vgt->aperture_sz)/SIZE_1MB);
	bitmap_clear(fence_bitmap, vgt->fence_base,  vgt->fence_sz);

	for (i = vgt->fence_base; i < vgt->fence_base + vgt->fence_sz; i++){
		VGT_MMIO_WRITE_BYTES(pdev,
			_REG_FENCE_0_LOW + 8 * i,
			0, 8);
	}

	for (i = 0; i < vgt->hidden_gm_slot_num; i++) {
		struct hidden_gm_slot *slot =
			vgt_hidden_gm_slotidx_to_slot(vgt, i);

		slot->current_vgpu_num--;
		if (!slot->current_vgpu_num)
			slot->owner = NULL;
		slot->max_vgpu_num = VGT_MAX_VMS;
	}
}

int alloc_vm_rsvd_aperture(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	int i;

	for (i=0; i< pdev->max_engines; i++) {
		vgt_state_ring_t *rb;
		struct vgt_rsvd_ring *ring = &pdev->ring_buffer[i];

		rb = &vgt->rb[i];
		if (IS_PREBDW(pdev)) {
			rb->context_save_area = aperture_2_gm(pdev,
					rsvd_aperture_alloc(pdev, SZ_CONTEXT_AREA_PER_RING));
			vgt_info("VM%d Ring%d context_save_area is allocated at gm(%llx)\n",
					vgt->vm_id, i, rb->context_save_area);
		} else {
			rb->context_save_area = 0;
		}

		rb->active_vm_context = 0;

		/*
		 * copy NULL context as the initial content. This update is
		 * only for non-dom0 instance. Dom0's context is updated when
		 * NULL context is created
		 */
		if (IS_PREBDW(pdev) && vgt->vgt_id && (i == RING_BUFFER_RCS)) {
			memcpy((char *)v_aperture(pdev, rb->context_save_area),
			       (char *)v_aperture(pdev, ring->null_context),
			       SZ_CONTEXT_AREA_PER_RING);
		}

		vgt_init_cmd_info(rb);
	}

	return 0;
}

void free_vm_rsvd_aperture(struct vgt_device *vgt)
{
	struct pgt_device *pdev = vgt->pdev;
	vgt_state_ring_t *rb;
	int i;

	if (IS_PREBDW(pdev)) {
		for (i = 0; i < pdev->max_engines; i++) {
			rb = &vgt->rb[i];
			rsvd_aperture_free(pdev, rb->context_save_area + phys_aperture_base(pdev),
					SZ_CONTEXT_AREA_PER_RING);
		}
	}
}

void initialize_gm_fence_allocation_bitmaps(struct pgt_device *pdev)
{
	struct vgt_device_info *info = &pdev->device_info;
	unsigned long *gm_bitmap = pdev->gm_bitmap;
	int i;

	vgt_info("total aperture: 0x%llx bytes, total GM space: 0x%llx bytes\n",
		phys_aperture_sz(pdev), gm_sz(pdev));

	ASSERT(phys_aperture_sz(pdev) % SIZE_1MB == 0);
	ASSERT(gm_sz(pdev) % SIZE_1MB == 0);
	ASSERT(phys_aperture_sz(pdev) <= gm_sz(pdev) && gm_sz(pdev) <= info->max_gtt_gm_sz);
	ASSERT(info->max_gtt_gm_sz <= VGT_MAX_GM_SIZE);

	for (i = 0; i < VGT_HIGHGM_SLOT_MAX_NUM; i++) {
		struct hidden_gm_slot *slot =
			hidden_gm_slotidx_to_slot(pdev, i);

		slot->current_vgpu_num = 0;
		slot->owner = NULL;
		slot->max_vgpu_num = VGT_MAX_VMS;
	}

	// mark the non-available space as non-available.
	if (gm_sz(pdev) < info->max_gtt_gm_sz)
		bitmap_set(gm_bitmap, gm_sz(pdev) / SIZE_1MB,
			(info->max_gtt_gm_sz - gm_sz(pdev)) / SIZE_1MB);

	pdev->rsvd_aperture_sz = VGT_RSVD_APERTURE_SZ;
	pdev->rsvd_aperture_base = phys_aperture_base(pdev) + hidden_gm_base(pdev) -
								pdev->rsvd_aperture_sz;

	// mark the rsvd aperture as not-available.
	bitmap_set(gm_bitmap, aperture_2_gm(pdev, pdev->rsvd_aperture_base)/SIZE_1MB,
				pdev->rsvd_aperture_sz/SIZE_1MB);

	vgt_info("reserved aperture: [0x%llx, 0x%llx)\n",
			pdev->rsvd_aperture_base,
			pdev->rsvd_aperture_base + pdev->rsvd_aperture_sz);
}

void vgt_init_reserved_aperture(struct pgt_device *pdev)
{
	/* setup the scratch page for the context switch */
	pdev->scratch_page = aperture_2_gm(pdev, rsvd_aperture_alloc(pdev,
				VGT_APERTURE_PER_INSTANCE_SZ));
	printk("scratch page is allocated at gm(0x%llx)\n", pdev->scratch_page);
	/* reserve the 1st trunk for vGT's general usage */
}
