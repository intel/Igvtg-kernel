/*
 * vGPU Live Migration Support
 *
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

#include "vgt.h"
#include "trace.h"

/******************************************************************************
 *	vGPU migration log dirty pages (LOGD)
 *
 * Since we do not have access/dirty bit in GPU PTE entries, we have to use
 * memory comparing for log vGPU touched pages. This patch enables the basic
 * framework for vGPU 'log dirty pages' feature.
 *
 * Please note:
 * Currently gfn memory comparing for vGPU pages is not yet implemented
 * All vGPU accessed gfns by default are marked as dirty.
 *
 * Interface:
 * KVMGT kernel LOGD (as well as XENGT Dom0) for SourceVM/TargetVM are
 * accessed by
 * /sys/kernel/vgt/vm#/dirty_bitmap
 *
 * dirty_bitmap is large binary table, that each guest page (gfn) is one of bit
 * in this table. gfn=0 is the bit0, and gfn=1 is the bit1, gfn=n is bit[n]in
 * this dirty_bitmap.
 * E.g The dirty_bitmap table size is 64KB for a guest VM with 2048MB memory
 *
 * Two steps to get the dirty pages from QEMU. First write corresponding gfn bit
 * to 1 and readback the same bit to get the dirty status. GVT-g will update the
 * dirty status when you write the bit. If the corresponding bit is still 1,
 * then this gfn page is used by vGPU and it is dirty, otherwise not.
 *
 * E.g:
 * QEMU want to get gfn[0-9] 's status, dirty or not
 * STEP1:
 *	QEMU: echo -e "\377\377\377" >  /sys/kernel/vgt/vm#/dirty_bitmap
 *	(write 0xFFF to dirty_bitmap)
 * STEP2:
 *	QEMU read back these bits to get the status
 *	cat /sys/kernel/vgt/vm#/dirty_bitmap | head -c 2 | xxd
 * Dump first two bytes from dirty_bitmap and the value is the dirty status.
 *
 * Of cause you should open dirty_bitmap and read only bytes you want instead of
 * read the whole table.
 ******************************************************************************/

#if 1
#define FUNC_ENTER vgt_dbg(VGT_DBG_MIGRATION, "----> ENTER\n")
#define FUNC_TAG vgt_dbg(VGT_DBG_MIGRATION, "----HIT\n")
#define FUNC_EXIT(x) vgt_dbg(VGT_DBG_MIGRATION, "<---- EXIT ret= %d\n", x)
#define DBP(x...) vgt_dbg(VGT_DBG_MIGRATION, x)
#else
#define FUNC_ENTER
#define FUNC_TAG
#define FUNC_EXIT
#define DBP
#endif

#define LOGD_BITMAP(slot, bit) \
(test_bit((bit), (slot)->dirty_bitmap))

#define LOGD_VGPU(slot, bit) \
(test_bit((bit), (slot)->vgpu_bitmap))

#define SLOT_OFFSET(gfn) \
((gfn)/LOGD_SLOT_SIZE)

#define BIT_OFFSET(gfn) \
((gfn) % LOGD_SLOT_SIZE)

#define GET_SLOT(vgt, gfn) \
(((vgt)->logd.logd_slot_head + SLOT_OFFSET(gfn))->slot)

static inline void
logd_hash_add_page(struct vgt_device *vgt, unsigned long gfn)
{
	logd_page_t *logd_page = kzalloc(sizeof(logd_page_t), GFP_ATOMIC);

	if (!logd_page) {
		vgt_err("Fail to alloc sizeof %d.\n", (int) sizeof(logd_page_t));
		vgt_logd_finit(vgt);
		logd_enable = false;
	}

	INIT_HLIST_NODE(&logd_page->node);
	logd_page->gfn = gfn;
	hash_add(vgt->logd.logd_page_hash_table,
			&logd_page->node, logd_page->gfn);
}

static inline void
logd_hash_remove_page(struct vgt_device *vgt, unsigned long gfn)
{
	logd_page_t *logd_page;
	bool found = false;

	hash_for_each_possible(vgt->logd.logd_page_hash_table,
			logd_page, node, gfn) {
		if (logd_page->gfn == gfn) {
			found = true;
			break;
		}
	}

	if (found) {
		hash_del(&logd_page->node);
		kfree(logd_page);
	}
}

static inline
logd_slot_t
*logd_alloc_slot(void)
{
	logd_slot_t *slot;
	slot = kzalloc(sizeof(logd_slot_t), GFP_ATOMIC);
	return slot;
}

static inline void
logd_free_slot(logd_slot_t *slot)
{
	if (slot != NULL)
		kfree(slot);
}

static inline bool
logd_page_is_dirty(struct vgt_device *vgt, unsigned long gfn)
{
	bool ret = false;
	logd_slot_t *slot;
	int bit_offset = gfn % LOGD_SLOT_SIZE;

	slot = GET_SLOT(vgt, gfn);
	if (slot == NULL) {
		/* page is clean */
		ret = false;
	} else if (LOGD_BITMAP(slot, bit_offset)
			|| LOGD_VGPU(slot, bit_offset)) {
		/* TODO: Calculate new logd_tag_t SHA1
		 * and comparing to old SHA1,
		 * if SHA1 is difference, mark as dirty
		 * and update with new SHA1
		 * currently we just always mark as dirty
		 */
		ret = true;
	} else {
		/* page is clean*/
		ret = false;
	}

	return ret;
}

static inline bool logd_slot_not_in_use(logd_slot_t *slot)
{
	if ((slot != NULL)
	&& (find_next_bit(slot->vgpu_bitmap,
			LOGD_SLOT_SIZE, 0) == LOGD_SLOT_SIZE)
	&& (find_next_bit(slot->dirty_bitmap,
			LOGD_SLOT_SIZE, 0) == LOGD_SLOT_SIZE))
		return true;
	else
		return false;
}

static inline bool logd_found_gfn(struct vgt_device *vgt, unsigned long gfn)
{
	logd_slot_t *slot;

	/* Instead of hash search in logd_page_hash_table,
	 * We keep vgpu_bitmap here, and search in this bitmap
	 * The gfn search could be very busy operation during migration
	 */
	slot = GET_SLOT(vgt, gfn);
	return (slot != NULL) && LOGD_VGPU(slot, BIT_OFFSET(gfn));
}

static inline bool
logd_increase_dirty_bitmap(struct vgt_device *vgt, unsigned long gfn)
{
	unsigned long max_gpfn =
		((gfn + INIT_BITMAP_GPFN - 1) / INIT_BITMAP_GPFN)
		* INIT_BITMAP_GPFN;

	int new_slot_count = SLOT_OFFSET(max_gpfn) + 1;
	int old_slot_count = SLOT_OFFSET(MAX_BITMAP_GPFN(vgt)) + 1;
	logd_slot_head_t *new_slot_head;

	new_slot_head =
		kzalloc(new_slot_count * sizeof(logd_slot_head_t), GFP_ATOMIC);

	if (new_slot_head == NULL) {
		vgt_err("Failed to increase dirty_bitmap. size=0x%lx\n",
			new_slot_count * sizeof(logd_slot_head_t));
		return false;
	}

	spin_lock(&vgt->logd.logd_lock);

	if (vgt->logd.logd_slot_head) {
		memcpy(new_slot_head,
			vgt->logd.logd_slot_head,
			old_slot_count * sizeof(logd_slot_head_t));

		kfree(vgt->logd.logd_slot_head);
	}

	vgt->logd.logd_slot_head = new_slot_head;
	MAX_BITMAP_GPFN(vgt) = max_gpfn;

	spin_unlock(&vgt->logd.logd_lock);
	return true;
}

/***********************************************
 * External Function for struct vgt_device
 ***********************************************/
void vgt_logd_init(struct vgt_device *vgt)
{
	if (vgt->vm_id == 0)
		return;

	spin_lock_init(&vgt->logd.logd_lock);

	/* set to initial status
	 * it is created when first gfn is added in
	 * logd_increase_dirty_bitmap(...)
	 */
	MAX_BITMAP_GPFN(vgt) = 0;
	vgt->logd.logd_slot_head = NULL;

	hash_init(vgt->logd.logd_page_hash_table);
}

void vgt_logd_finit(struct vgt_device *vgt)
{
	struct hlist_node *next;
	logd_page_t *logd_page;
	logd_slot_head_t *slot_head;
	int slot_count;
	int i;

	if (vgt->vm_id == 0)
		return;

	spin_lock(&vgt->logd.logd_lock);
	hash_for_each_safe(vgt->logd.logd_page_hash_table,
			i, next, logd_page, node) {
		hash_del(&logd_page->node);
		kfree(logd_page);
	}
	hash_init(vgt->logd.logd_page_hash_table);

	if (vgt->logd.logd_slot_head) {
		slot_count = SLOT_OFFSET(MAX_BITMAP_GPFN(vgt)) + 1;
		for(i = 0; i < slot_count; i++) {
			slot_head = vgt->logd.logd_slot_head + i;
			if (slot_head->slot)
				kfree(slot_head->slot);
		}

		kfree(vgt->logd.logd_slot_head);
		vgt->logd.logd_slot_head = NULL;
		MAX_BITMAP_GPFN(vgt) = 0;
	}
	spin_unlock(&vgt->logd.logd_lock);
}

/*
 * This function is designed very lite. Feel free to be called in any critical
 * path.
 */
void vgt_logd_add(struct vgt_device *vgt, unsigned long gfn)
{
	logd_slot_t *slot;
	int bit_offset = BIT_OFFSET(gfn);

	/* Enable logging according to kernel parameter*/
	if (!logd_enable)
		return;

	if (gfn > MAX_BITMAP_GPFN(vgt)) {
		if (!logd_increase_dirty_bitmap(vgt, gfn)) {
			trace_logd(vgt->vm_id,
				"create bitmap\0", gfn, "failed\0");
			/* disable logd if failed */
			vgt_logd_finit(vgt);
			logd_enable = false;
			return;
		} else {
			trace_logd(vgt->vm_id,
				"create bitmap\0", gfn, "success\0");
		}
	}

	if (!vgt->logd.logd_slot_head)
		return;

	/* TODO: ref count required. gfn could be shared in multiple PPGTTs */
	if (logd_found_gfn(vgt, gfn))
		return;

	slot = GET_SLOT(vgt, gfn);
	if (slot == NULL) {
		slot = logd_alloc_slot();
		GET_SLOT(vgt, gfn) = slot;
	}

	spin_lock(&vgt->logd.logd_lock);
	/* In order to have quick search of gfn, we keep vgpu_bitmap here */
	set_bit(bit_offset, slot->vgpu_bitmap);

	/* Previous gfn was removed from vgpu_bitmap only,
	 * and add this time before logsync, these gfns are already in hash
	 * we do not need to add to hash anymore.
	 * Here only add hash with bit not in bitmap and vgpu_bitmap
	 */
	if (!LOGD_BITMAP(slot, bit_offset)) {
		logd_hash_add_page(vgt, gfn);
		/* add to dirty_bitmap, thus gfn added next time can be detected
		 * if it is duplicated or not in hash
		 */
		set_bit(bit_offset, slot->dirty_bitmap);
	}

	spin_unlock(&vgt->logd.logd_lock);
	trace_logd(vgt->vm_id, "add\0", gfn, "success\0");
}

void vgt_logd_remove(struct vgt_device *vgt, unsigned long gfn)
{
	logd_slot_t *slot;
	int bit_offset = BIT_OFFSET(gfn);

	/* logd may already destroyed if logd disabled */
	if (!vgt->logd.logd_slot_head)
		return;

	if (gfn > MAX_BITMAP_GPFN(vgt)) {
		trace_logd(vgt->vm_id, "remove\0", gfn, "failed\0");
		return;
	}

	/* TODO: ref count required. gfn could be shared in multiple PPGTTs */
	slot = GET_SLOT(vgt, gfn);
	if (!slot)
		return;

	spin_lock(&vgt->logd.logd_lock);
	clear_bit(bit_offset, slot->vgpu_bitmap);
	/* set the dirty_bitmap again, since this bit may be cleared if logsync
	 * already called before gfn remove from vgpu_bitmap
	 */
	set_bit(bit_offset, slot->dirty_bitmap);
	
	spin_unlock(&vgt->logd.logd_lock);
	trace_logd(vgt->vm_id, "remove\0", gfn, "success\0");
}
/* Only logsync is responsible to release logd_page_t entry and logd_slot_t
 */
int vgt_logd_slot_sync(struct vgt_device *vgt,
		int slot_offset, int bit_offset, int nr_bits)
{
	logd_slot_t *slot;
	int bit;
	unsigned long n_dirty_pages = 0;
	unsigned long gfn_start = slot_offset*LOGD_SLOT_SIZE + bit_offset;

	slot = GET_SLOT(vgt, gfn_start);

	if (likely(slot == NULL))
		return n_dirty_pages;

	/* bit_offset + nr_bits <= LOGD_SLOT_SIZE */
	ASSERT(bit_offset + nr_bits <= LOGD_SLOT_SIZE);

	for (bit = bit_offset; bit < bit_offset + nr_bits; bit++) {
		/* logd_page_is_dirty is very cpu intensive workload
		 * please keep outside of spin_lock
		 */
		unsigned long gfn = slot_offset*LOGD_SLOT_SIZE + bit;
		bool is_dirty = logd_page_is_dirty(vgt, gfn);

		spin_lock(&vgt->logd.logd_lock);
		if (!is_dirty) {
			/* must handle before clear dirty_bitmap */
			if (LOGD_BITMAP(slot, bit) && !LOGD_VGPU(slot, bit)) {
				logd_hash_remove_page(vgt, gfn);
			}
			clear_bit(bit, slot->dirty_bitmap);
		} else {
			n_dirty_pages++;
		}
		spin_unlock(&vgt->logd.logd_lock);
	}

	spin_lock(&vgt->logd.logd_lock);
	if (logd_slot_not_in_use(slot)) {
		logd_free_slot(slot);
		GET_SLOT(vgt, gfn_start) = NULL;
	}

	spin_unlock(&vgt->logd.logd_lock);
	return n_dirty_pages;
}

/* return num of dirty pages in range [gfn_start..npages]
 * dirty page bitmaps are stored in logd_slot_t->dirty_bitmap
 */
int vgt_logd_sync(struct vgt_device *vgt, 
		unsigned long gfn_start, unsigned long npages)
{
	unsigned long n_dirty_pages = 0;
	int slot_start;
	int slot_end;
	int bit_offset = 0;
	int nr_bits = 0;
	int i;

	if (gfn_start > MAX_BITMAP_GPFN(vgt))
		return 0;

	if (gfn_start + npages > MAX_BITMAP_GPFN(vgt))
		npages = MAX_BITMAP_GPFN(vgt) - gfn_start;

	slot_start = SLOT_OFFSET(gfn_start);
	slot_end = SLOT_OFFSET(gfn_start + npages);

	for (i = slot_start; i <= slot_end; i++) {
		bit_offset = (i == slot_start) ?
			BIT_OFFSET(gfn_start) : 0;
		nr_bits = (i == slot_end) ?
			BIT_OFFSET(gfn_start + npages) : LOGD_SLOT_SIZE;
		nr_bits -= bit_offset;
		n_dirty_pages += vgt_logd_slot_sync(vgt, i,
				bit_offset, nr_bits);
	}

	return n_dirty_pages;
}

int vgt_logd_read_log(struct vgt_device *vgt, 
		char *buf, unsigned long off, unsigned long count)
{
	/* off is bitmap offset in unit of u8 */
	unsigned long gfn_start;
	unsigned long npages;
	logd_slot_t *slot;
	char *src, *dst;
	int slot_start;
	int slot_end;
	int bit_offset = 0;
	int nr_bits = 0;
	int i;

	gfn_start = off*BITS_PER_BYTE;
	npages = count*BITS_PER_BYTE;

	slot_start = SLOT_OFFSET(gfn_start);
	slot_end = SLOT_OFFSET(gfn_start + npages);

	/* clear all */
	memset(buf, 0, count);

	if (gfn_start >= MAX_BITMAP_GPFN(vgt))
		return 0;

	if (slot_start > SLOT_OFFSET(MAX_BITMAP_GPFN(vgt)))
		return 0;

	if (slot_end > SLOT_OFFSET(MAX_BITMAP_GPFN(vgt))) {
		slot_end = SLOT_OFFSET(MAX_BITMAP_GPFN(vgt));
		npages = MAX_BITMAP_GPFN(vgt) - gfn_start;
	}

	dst = buf;
	for (i = slot_start; i <= slot_end; i++) {
		bit_offset = (i == slot_start) ?
			BIT_OFFSET(gfn_start) : 0;
		nr_bits = (i == slot_end) ?
			BIT_OFFSET(gfn_start + npages) : LOGD_SLOT_SIZE;
		nr_bits -= bit_offset;
		slot = GET_SLOT(vgt, i*LOGD_SLOT_SIZE);
		if (slot == NULL) {
			/* no vgpu gfn and no dirty gfn */
		} else {
			src = (char *)slot->dirty_bitmap;
			src += BITS_TO_BYTES(bit_offset);
			memcpy(dst, src, BITS_TO_BYTES(nr_bits));
		}
		dst += BITS_TO_BYTES(nr_bits);
	}

	/* return number of pages it handled */
	return npages;
}

