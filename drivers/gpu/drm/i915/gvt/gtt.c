/*
 * GTT virtualization
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

#include "gvt.h"
#include "trace.h"

/*
 * Mappings between GTT_TYPE* enumerations.
 * Following informations can be found according to the given type:
 * - type of next level page table
 * - type of entry inside this level page table
 * - type of entry with PSE set
 *
 * If the given type doesn't have such a kind of information,
 * e.g. give a l4 root entry type, then request to get its PSE type,
 * give a PTE page table type, then request to get its next level page
 * table type, as we know l4 root entry doesn't have a PSE bit,
 * and a PTE page table doesn't have a next level page table type,
 * GTT_TYPE_INVALID will be returned. This is useful when traversing a
 * page table.
 */

struct gtt_type_table_entry {
	gtt_type_t entry_type;
	gtt_type_t next_pt_type;
	gtt_type_t pse_entry_type;
};

#define GTT_TYPE_TABLE_ENTRY(type, e_type, npt_type, pse_type) \
	[type] = { \
		.entry_type = e_type, \
		.next_pt_type = npt_type, \
		.pse_entry_type = pse_type, \
	}

static struct gtt_type_table_entry gtt_type_table[] = {
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_ROOT_L4_ENTRY,
			GTT_TYPE_PPGTT_ROOT_L4_ENTRY,
			GTT_TYPE_PPGTT_PML4_PT,
			GTT_TYPE_INVALID),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PML4_PT,
			GTT_TYPE_PPGTT_PML4_ENTRY,
			GTT_TYPE_PPGTT_PDP_PT,
			GTT_TYPE_INVALID),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PML4_ENTRY,
			GTT_TYPE_PPGTT_PML4_ENTRY,
			GTT_TYPE_PPGTT_PDP_PT,
			GTT_TYPE_INVALID),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PDP_PT,
			GTT_TYPE_PPGTT_PDP_ENTRY,
			GTT_TYPE_PPGTT_PDE_PT,
			GTT_TYPE_PPGTT_PTE_1G_ENTRY),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_ROOT_L3_ENTRY,
			GTT_TYPE_PPGTT_ROOT_L3_ENTRY,
			GTT_TYPE_PPGTT_PDE_PT,
			GTT_TYPE_PPGTT_PTE_1G_ENTRY),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PDP_ENTRY,
			GTT_TYPE_PPGTT_PDP_ENTRY,
			GTT_TYPE_PPGTT_PDE_PT,
			GTT_TYPE_PPGTT_PTE_1G_ENTRY),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PDE_PT,
			GTT_TYPE_PPGTT_PDE_ENTRY,
			GTT_TYPE_PPGTT_PTE_PT,
			GTT_TYPE_PPGTT_PTE_2M_ENTRY),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PDE_ENTRY,
			GTT_TYPE_PPGTT_PDE_ENTRY,
			GTT_TYPE_PPGTT_PTE_PT,
			GTT_TYPE_PPGTT_PTE_2M_ENTRY),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PTE_PT,
			GTT_TYPE_PPGTT_PTE_4K_ENTRY,
			GTT_TYPE_INVALID,
			GTT_TYPE_INVALID),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PTE_4K_ENTRY,
			GTT_TYPE_PPGTT_PTE_4K_ENTRY,
			GTT_TYPE_INVALID,
			GTT_TYPE_INVALID),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PTE_2M_ENTRY,
			GTT_TYPE_PPGTT_PDE_ENTRY,
			GTT_TYPE_INVALID,
			GTT_TYPE_PPGTT_PTE_2M_ENTRY),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_PPGTT_PTE_1G_ENTRY,
			GTT_TYPE_PPGTT_PDP_ENTRY,
			GTT_TYPE_INVALID,
			GTT_TYPE_PPGTT_PTE_1G_ENTRY),
	GTT_TYPE_TABLE_ENTRY(GTT_TYPE_GGTT_PTE,
			GTT_TYPE_GGTT_PTE,
			GTT_TYPE_INVALID,
			GTT_TYPE_INVALID),
};

static inline gtt_type_t get_next_pt_type(gtt_type_t type) {
	return gtt_type_table[type].next_pt_type;
}

static inline gtt_type_t get_entry_type(gtt_type_t type) {
	return gtt_type_table[type].entry_type;
}

static inline gtt_type_t get_pse_type(gtt_type_t type) {
	return gtt_type_table[type].pse_entry_type;
}

static inline gtt_entry_t *gtt_get_entry64(void *pt, gtt_entry_t *e,
		unsigned long index, bool hypervisor_access,
		struct vgt_device *vgt)
{
	struct gvt_device_info *info = &e->pdev->device_info;

	ASSERT(info->gtt_entry_size == 8);

	if (!pt)
		e->val64 = gvt_read_gtt64(e->pdev, index);
	else {
		if (!hypervisor_access)
			e->val64 = *((u64 *)pt + index);
		else
			hypervisor_read_va(vgt, (u64 *)pt + index, &e->val64, 8, 1);
	}
	return e;
}

static inline gtt_entry_t *gtt_set_entry64(void *pt, gtt_entry_t *e,
		unsigned long index, bool hypervisor_access,
		struct vgt_device *vgt)
{
	struct gvt_device_info *info = &e->pdev->device_info;

	ASSERT(info->gtt_entry_size == 8);

	if (!pt)
		gvt_write_gtt64(e->pdev, index, e->val64);
	else {
		if (!hypervisor_access)
			*((u64 *)pt + index) = e->val64;
		else
			hypervisor_write_va(vgt, (u64 *)pt + index, &e->val64, 8, 1);
	}
	return e;
}

static unsigned long gen8_gtt_get_pfn(gtt_entry_t *e)
{
	if (e->type == GTT_TYPE_PPGTT_PTE_1G_ENTRY)
		return (e->val64 & (0x1ff << 30)) >> 12;
	else if (e->type == GTT_TYPE_PPGTT_PTE_2M_ENTRY)
		return (e->val64 & (0x3ffff << 21)) >> 12;
	else
		return (e->val64 >> 12) & 0x7ffffff;
}

static void gen8_gtt_set_pfn(gtt_entry_t *e, unsigned long pfn)
{
	if (e->type == GTT_TYPE_PPGTT_PTE_1G_ENTRY) {
		e->val64 &= ~(0x1ff << 30);
		pfn &= ((0x1ff << 30) >> 12);
	} else if (e->type == GTT_TYPE_PPGTT_PTE_2M_ENTRY) {
		e->val64 &= ~(0x3ffff << 21);
		pfn &= ((0x3ffff << 21) >> 12);
	} else {
		e->val64 &= ~(0x7ffffff << 12);
		pfn &= 0x7ffffff;
	}

	e->val64 |= (pfn << 12);
}

static bool gen8_gtt_test_pse(gtt_entry_t *e)
{
	/* Entry doesn't have PSE bit. */
	if (get_pse_type(e->type) == GTT_TYPE_INVALID)
		return false;

	e->type = get_entry_type(e->type);
	if (!(e->val64 & (1 << 7)))
		return false;

	e->type = get_pse_type(e->type);
	return true;
}

static bool gen8_gtt_test_present(gtt_entry_t *e)
{
	/*
	 * i915 writes PDP root pointer registers without present bit,
	 * it also works, so we need to treat root pointer entry
	 * specifically.
	 */
	if (e->type == GTT_TYPE_PPGTT_ROOT_L3_ENTRY
			|| e->type == GTT_TYPE_PPGTT_ROOT_L4_ENTRY)
		return (e->val64 != 0);
	else
		return (e->val32[0] & (1 << 0));
}

static void gtt_entry_clear_present(gtt_entry_t *e)
{
	e->val32[0] &= ~(1 << 0);
}

/*
 * Per-platform GMA routines.
 */
static unsigned long gma_to_ggtt_pte_index(unsigned long gma)
{
	unsigned long x = (gma >> GTT_PAGE_SHIFT);
	trace_gma_index(__func__, gma, x);
	return x;
}

#define DEFINE_PPGTT_GMA_TO_INDEX(prefix, ename, exp) \
	static unsigned long prefix##_gma_to_##ename##_index(unsigned long gma) { \
		unsigned long x = (exp); \
		trace_gma_index(__func__, gma, x); \
		return x; \
	}

DEFINE_PPGTT_GMA_TO_INDEX(gen8, pte, (gma >> 12 & 0x1ff));
DEFINE_PPGTT_GMA_TO_INDEX(gen8, pde, (gma >> 21 & 0x1ff));
DEFINE_PPGTT_GMA_TO_INDEX(gen8, l3_pdp, (gma >> 30 & 0x3));
DEFINE_PPGTT_GMA_TO_INDEX(gen8, l4_pdp, (gma >> 30 & 0x1ff));
DEFINE_PPGTT_GMA_TO_INDEX(gen8, pml4, (gma >> 39 & 0x1ff));

struct gvt_gtt_pte_ops gen8_gtt_pte_ops = {
	.get_entry = gtt_get_entry64,
	.set_entry = gtt_set_entry64,
	.clear_present = gtt_entry_clear_present,
	.test_present = gen8_gtt_test_present,
	.test_pse = gen8_gtt_test_pse,
	.get_pfn = gen8_gtt_get_pfn,
	.set_pfn = gen8_gtt_set_pfn,
};

struct gvt_gtt_gma_ops gen8_gtt_gma_ops = {
	.gma_to_ggtt_pte_index = gma_to_ggtt_pte_index,
	.gma_to_pte_index = gen8_gma_to_pte_index,
	.gma_to_pde_index = gen8_gma_to_pde_index,
	.gma_to_l3_pdp_index = gen8_gma_to_l3_pdp_index,
	.gma_to_l4_pdp_index = gen8_gma_to_l4_pdp_index,
	.gma_to_pml4_index = gen8_gma_to_pml4_index,
};

static bool gtt_entry_p2m(struct vgt_device *vgt, gtt_entry_t *p, gtt_entry_t *m)
{
        struct gvt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
        unsigned long gfn, mfn;

        *m = *p;

        if (!ops->test_present(p))
                return true;

        gfn = ops->get_pfn(p);

        mfn = hypervisor_g2m_pfn(vgt, gfn);
        if (mfn == INVALID_MFN) {
                gvt_err("fail to translate gfn: 0x%lx", gfn);
                return false;
        }

        ops->set_pfn(m, mfn);

        return true;
}

/*
 * MM helpers.
 */
gtt_entry_t *gvt_mm_get_entry(struct gvt_mm *mm,
		void *page_table, gtt_entry_t *e,
		unsigned long index)
{
	struct pgt_device *pdev = mm->vgt->pdev;
	struct gvt_gtt_pte_ops *ops = pdev->gtt.pte_ops;

	e->pdev = pdev;
	e->type = mm->page_table_entry_type;

	ops->get_entry(page_table, e, index, false, NULL);
	ops->test_pse(e);

	return e;
}

gtt_entry_t *gvt_mm_set_entry(struct gvt_mm *mm,
		void *page_table, gtt_entry_t *e,
		unsigned long index)
{
	struct pgt_device *pdev = mm->vgt->pdev;
	struct gvt_gtt_pte_ops *ops = pdev->gtt.pte_ops;

	e->pdev = pdev;

	return ops->set_entry(page_table, e, index, false, NULL);
}

/*
 * PPGTT shadow page table helpers.
 */
static inline gtt_entry_t *ppgtt_spt_get_entry(ppgtt_spt_t *spt,
		void *page_table, gtt_type_t type,
		gtt_entry_t *e, unsigned long index,
		bool guest)
{
	struct pgt_device *pdev = spt->vgt->pdev;
	struct gvt_gtt_pte_ops *ops = pdev->gtt.pte_ops;

	e->pdev = pdev;
	e->type = get_entry_type(type);

	ASSERT(gtt_type_is_entry(e->type));

	ops->get_entry(page_table, e, index, guest, spt->vgt);
	ops->test_pse(e);

	return e;
}

static inline gtt_entry_t *ppgtt_spt_set_entry(ppgtt_spt_t *spt,
		void *page_table, gtt_type_t type,
		gtt_entry_t *e, unsigned long index,
		bool guest)
{
	struct pgt_device *pdev = spt->vgt->pdev;
	struct gvt_gtt_pte_ops *ops = pdev->gtt.pte_ops;

	e->pdev = pdev;

	ASSERT(gtt_type_is_entry(e->type));

	return ops->set_entry(page_table, e, index, guest, spt->vgt);
}

#define ppgtt_get_guest_entry(spt, e, index) \
	ppgtt_spt_get_entry(spt, spt->guest_page.vaddr, \
		spt->guest_page_type, e, index, true)

#define ppgtt_set_guest_entry(spt, e, index) \
	ppgtt_spt_set_entry(spt, spt->guest_page.vaddr, \
		spt->guest_page_type, e, index, true)

#define ppgtt_get_shadow_entry(spt, e, index) \
	ppgtt_spt_get_entry(spt, spt->shadow_page.vaddr, \
		spt->shadow_page.type, e, index, false)

#define ppgtt_set_shadow_entry(spt, e, index) \
	ppgtt_spt_set_entry(spt, spt->shadow_page.vaddr, \
		spt->shadow_page.type, e, index, false)


bool gvt_init_guest_page(struct vgt_device *vgt, guest_page_t *guest_page,
		unsigned long gfn, guest_page_handler_t handler, void *data)
{
	INIT_HLIST_NODE(&guest_page->node);

	guest_page->vaddr = hypervisor_gpa_to_va(vgt, gfn << GTT_PAGE_SHIFT);
	if (!guest_page->vaddr)
		return false;

	guest_page->writeprotection = false;
	guest_page->gfn = gfn;
	guest_page->handler = handler;
	guest_page->data = data;
	guest_page->oos_page = NULL;
	guest_page->write_cnt = 0;

	hash_add(vgt->gtt.guest_page_hash_table, &guest_page->node, guest_page->gfn);

	return true;
}

static bool gvt_detach_oos_page(struct vgt_device *vgt, oos_page_t *oos_page);

void gvt_clean_guest_page(struct vgt_device *vgt, guest_page_t *guest_page)
{
	if(!hlist_unhashed(&guest_page->node))
		hash_del(&guest_page->node);

	if (guest_page->oos_page)
		gvt_detach_oos_page(vgt, guest_page->oos_page);

	if (guest_page->writeprotection)
		hypervisor_unset_wp_pages(vgt, guest_page);

	if (guest_page == vgt->gtt.last_partial_ppgtt_access_gpt)
		vgt->gtt.last_partial_ppgtt_access_index = -1;
}

guest_page_t *gvt_find_guest_page(struct vgt_device *vgt, unsigned long gfn)
{
	guest_page_t *guest_page;
	struct gvt_statistics *stat = &vgt->stat;
	cycles_t t0, t1;

	t0 = get_cycles();

	hash_for_each_possible(vgt->gtt.guest_page_hash_table, guest_page, node, gfn) {
		if (guest_page->gfn == gfn) {
			t1 = get_cycles();
			stat->gpt_find_hit_cnt++;
			stat->gpt_find_hit_cycles += t1 - t0;
			return guest_page;
		}
	}

	t1 = get_cycles();
	stat->gpt_find_miss_cnt++;
	stat->gpt_find_miss_cycles += t1 - t0;

	return NULL;
}

/*
 * Shadow page manipulation routines.
 */
static inline bool gvt_init_shadow_page(struct vgt_device *vgt,
		shadow_page_t *sp, gtt_type_t type)
{
	sp->vaddr = page_address(sp->page);
	sp->type = type;
	memset(sp->vaddr, 0, PAGE_SIZE);

	INIT_HLIST_NODE(&sp->node);
	sp->mfn = hypervisor_virt_to_mfn(sp->vaddr);
	hash_add(vgt->gtt.shadow_page_hash_table, &sp->node, sp->mfn);

	return true;
}

static inline void gvt_clean_shadow_page(shadow_page_t *sp)
{
	if(!hlist_unhashed(&sp->node))
		hash_del(&sp->node);
}

static inline shadow_page_t *gvt_find_shadow_page(struct vgt_device *vgt,
		unsigned long mfn)
{
	shadow_page_t *shadow_page;
	struct gvt_statistics *stat = &vgt->stat;
	cycles_t t0, t1;

	t0 = get_cycles();

	hash_for_each_possible(vgt->gtt.shadow_page_hash_table, shadow_page, node, mfn) {
		if (shadow_page->mfn == mfn) {
			t1 = get_cycles();
			stat->spt_find_hit_cnt++;
			stat->spt_find_hit_cycles += t1 - t0;
			return shadow_page;
		}
	}

	t1 = get_cycles();
	stat->spt_find_miss_cnt++;
	stat->spt_find_miss_cycles += t1 - t0;

	return NULL;
}

#define guest_page_to_ppgtt_spt(ptr) \
	container_of(ptr, ppgtt_spt_t, guest_page)

#define shadow_page_to_ppgtt_spt(ptr) \
	container_of(ptr, ppgtt_spt_t, shadow_page)

static void ppgtt_free_shadow_page(ppgtt_spt_t *spt)
{
	trace_spt_free(spt->vgt->vm_id, spt, spt->shadow_page.type);

	gvt_clean_shadow_page(&spt->shadow_page);
	gvt_clean_guest_page(spt->vgt, &spt->guest_page);

	mempool_free(spt, spt->vgt->pdev->gtt.mempool);
}

static void ppgtt_free_all_shadow_page(struct vgt_device *vgt)
{
	struct hlist_node *n;
	shadow_page_t *sp;
	int i;

	hash_for_each_safe(vgt->gtt.shadow_page_hash_table, i, n, sp, node)
		ppgtt_free_shadow_page(shadow_page_to_ppgtt_spt(sp));

	return;
}

static bool ppgtt_handle_guest_write_page_table_bytes(void *gp,
		uint64_t pa, void *p_data, int bytes);

static bool ppgtt_write_protection_handler(void *gp, uint64_t pa, void *p_data, int bytes)
{
	guest_page_t *gpt = (guest_page_t *)gp;
	ppgtt_spt_t *spt = guest_page_to_ppgtt_spt(gpt);
	struct vgt_device *vgt = spt->vgt;
	struct gvt_statistics *stat = &vgt->stat;
	cycles_t t0, t1;

	if (bytes != 4 && bytes != 8)
		return false;

	if (!gpt->writeprotection)
		return false;

	t0 = get_cycles();

	if (!ppgtt_handle_guest_write_page_table_bytes(gp,
		pa, p_data, bytes))
		return false;

	t1 = get_cycles();
	stat->ppgtt_wp_cnt++;
	stat->ppgtt_wp_cycles += t1 - t0;

	return true;
}

static ppgtt_spt_t *ppgtt_alloc_shadow_page(struct vgt_device *vgt,
		gtt_type_t type, unsigned long gpt_gfn)
{
	ppgtt_spt_t *spt = NULL;

	spt = mempool_alloc(vgt->pdev->gtt.mempool, GFP_ATOMIC);
	if (!spt) {
		gvt_err("fail to allocate ppgtt shadow page.");
		return NULL;
	}

	spt->vgt = vgt;
	spt->guest_page_type = type;
	atomic_set(&spt->refcount, 1);

	/*
	 * TODO: Guest page may be different with shadow page type,
	 *	 if we support PSE page in future.
	 */
	if (!gvt_init_shadow_page(vgt, &spt->shadow_page, type)) {
		gvt_err("fail to initialize shadow_page_t for spt.");
		goto err;
	}

	if (!gvt_init_guest_page(vgt, &spt->guest_page,
				gpt_gfn, ppgtt_write_protection_handler, NULL)) {
		gvt_err("fail to initialize shadow_page_t for spt.");
		goto err;
	}

	trace_spt_alloc(vgt->vm_id, spt, type, spt->shadow_page.mfn, gpt_gfn);

	return spt;
err:
	ppgtt_free_shadow_page(spt);
	return NULL;
}

static ppgtt_spt_t *ppgtt_find_shadow_page(struct vgt_device *vgt, unsigned long mfn)
{
	shadow_page_t *sp = gvt_find_shadow_page(vgt, mfn);

	if (sp)
		return shadow_page_to_ppgtt_spt(sp);

	gvt_err("VM %d fail to find ppgtt shadow page: 0x%lx.",
			vgt->vm_id, mfn);

	return NULL;
}

#define pt_entry_size_shift(spt) \
	((spt)->vgt->pdev->device_info.gtt_entry_size_shift)

#define pt_entries(spt) \
	(PAGE_SIZE >> pt_entry_size_shift(spt))

#define for_each_present_guest_entry(spt, e, i) \
	for (i = 0; i < pt_entries(spt); i++) \
	if (spt->vgt->pdev->gtt.pte_ops->test_present(ppgtt_get_guest_entry(spt, e, i)))

#define for_each_present_shadow_entry(spt, e, i) \
	for (i = 0; i < pt_entries(spt); i++) \
	if (spt->vgt->pdev->gtt.pte_ops->test_present(ppgtt_get_shadow_entry(spt, e, i)))

static void ppgtt_get_shadow_page(ppgtt_spt_t *spt)
{
	int v = atomic_read(&spt->refcount);

	trace_spt_refcount(spt->vgt->vm_id, "inc", spt, v, (v + 1));

	atomic_inc(&spt->refcount);
}

static bool ppgtt_invalidate_shadow_page(ppgtt_spt_t *spt);

static bool ppgtt_invalidate_shadow_page_by_shadow_entry(struct vgt_device *vgt,
		gtt_entry_t *e)
{
	struct gvt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	ppgtt_spt_t *s;

	if (!gtt_type_is_pt(get_next_pt_type(e->type)))
		return false;

	s = ppgtt_find_shadow_page(vgt, ops->get_pfn(e));
	if (!s) {
		gvt_err("VM %d fail to find shadow page: mfn: 0x%lx.",
				vgt->vm_id, ops->get_pfn(e));
		return false;
	}

	return ppgtt_invalidate_shadow_page(s);
}

static bool ppgtt_invalidate_shadow_page(ppgtt_spt_t *spt)
{
	gtt_entry_t e;
	unsigned long index;
	int v = atomic_read(&spt->refcount);

	trace_spt_change(spt->vgt->vm_id, "die", spt,
			spt->guest_page.gfn, spt->shadow_page.type);

	trace_spt_refcount(spt->vgt->vm_id, "dec", spt, v, (v - 1));

	if (atomic_dec_return(&spt->refcount) > 0)
		return true;

	if (gtt_type_is_pte_pt(spt->shadow_page.type))
		goto release;

	for_each_present_shadow_entry(spt, &e, index) {
		if (!gtt_type_is_pt(get_next_pt_type(e.type))) {
			gvt_err("GVT doesn't support pse bit now.");
			return false;
		}
		if (!ppgtt_invalidate_shadow_page_by_shadow_entry(spt->vgt, &e))
			goto fail;
	}

release:
	trace_spt_change(spt->vgt->vm_id, "release", spt,
			spt->guest_page.gfn, spt->shadow_page.type);
	ppgtt_free_shadow_page(spt);
	return true;
fail:
	gvt_err("fail: shadow page %p shadow entry 0x%llx type %d.",
			spt, e.val64, e.type);
	return false;
}

static bool ppgtt_populate_shadow_page(ppgtt_spt_t *spt);

static ppgtt_spt_t *ppgtt_populate_shadow_page_by_guest_entry(struct vgt_device *vgt,
		gtt_entry_t *we)
{
	struct gvt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	ppgtt_spt_t *s = NULL;
	guest_page_t *g;

	if (!gtt_type_is_pt(get_next_pt_type(we->type)))
		goto fail;

	g = gvt_find_guest_page(vgt, ops->get_pfn(we));
	if (g) {
		s = guest_page_to_ppgtt_spt(g);
		ppgtt_get_shadow_page(s);
	} else {
		gtt_type_t type = get_next_pt_type(we->type);
		s = ppgtt_alloc_shadow_page(vgt, type, ops->get_pfn(we));
		if (!s)
			goto fail;

		if (!hypervisor_set_wp_pages(vgt, &s->guest_page))
			goto fail;

		if (!ppgtt_populate_shadow_page(s))
			goto fail;

		trace_spt_change(vgt->vm_id, "new", s, s->guest_page.gfn, s->shadow_page.type);
	}
	return s;
fail:
	gvt_err("fail: shadow page %p guest entry 0x%llx type %d.",
			s, we->val64, we->type);
	return NULL;
}

static inline void ppgtt_generate_shadow_entry(gtt_entry_t *se,
		ppgtt_spt_t *s, gtt_entry_t *ge)
{
	struct gvt_gtt_pte_ops *ops = s->vgt->pdev->gtt.pte_ops;

	se->type = ge->type;
	se->val64 = ge->val64;
	se->pdev = ge->pdev;

	ops->set_pfn(se, s->shadow_page.mfn);
}

static bool ppgtt_populate_shadow_page(ppgtt_spt_t *spt)
{
	struct vgt_device *vgt = spt->vgt;
	ppgtt_spt_t *s;
	gtt_entry_t se, ge;
	unsigned long i;

	trace_spt_change(spt->vgt->vm_id, "born", spt,
			spt->guest_page.gfn, spt->shadow_page.type);

	if (gtt_type_is_pte_pt(spt->shadow_page.type)) {
		for_each_present_guest_entry(spt, &ge, i) {
			if (!gtt_entry_p2m(vgt, &ge, &se))
				goto fail;
			ppgtt_set_shadow_entry(spt, &se, i);
		}
		return true;
	}

	for_each_present_guest_entry(spt, &ge, i) {
		if (!gtt_type_is_pt(get_next_pt_type(ge.type))) {
			gvt_err("GVT doesn't support pse bit now.");
			goto fail;
		}

		s = ppgtt_populate_shadow_page_by_guest_entry(vgt, &ge);
		if (!s)
			goto fail;
		ppgtt_get_shadow_entry(spt, &se, i);
		ppgtt_generate_shadow_entry(&se, s, &ge);
		ppgtt_set_shadow_entry(spt, &se, i);
	}
	return true;
fail:
	gvt_err("fail: shadow page %p guest entry 0x%llx type %d.",
			spt, ge.val64, ge.type);
	return false;
}

static bool ppgtt_handle_guest_entry_removal(guest_page_t *gpt,
		gtt_entry_t *we, unsigned long index)
{
	ppgtt_spt_t *spt = guest_page_to_ppgtt_spt(gpt);
	shadow_page_t *sp = &spt->shadow_page;
	struct vgt_device *vgt = spt->vgt;
	struct gvt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	gtt_entry_t e;

	trace_gpt_change(spt->vgt->vm_id, "remove", spt, sp->type, we->val64, index);

	ppgtt_get_shadow_entry(spt, &e, index);
	if (!ops->test_present(&e))
		return true;

	if (gtt_type_is_pt(get_next_pt_type(we->type))) {
		guest_page_t *g = gvt_find_guest_page(vgt, ops->get_pfn(we));
		if (!g) {
			gvt_err("fail to find guest page.");
			goto fail;
		}
		if (!ppgtt_invalidate_shadow_page(guest_page_to_ppgtt_spt(g)))
			goto fail;
	}
	e.val64 = 0;
	ppgtt_set_shadow_entry(spt, &e, index);
	return true;
fail:
	gvt_err("fail: shadow page %p guest entry 0x%llx type %d.",
			spt, we->val64, we->type);
	return false;
}

static bool ppgtt_handle_guest_entry_add(guest_page_t *gpt,
		gtt_entry_t *we, unsigned long index)
{
	ppgtt_spt_t *spt = guest_page_to_ppgtt_spt(gpt);
	shadow_page_t *sp = &spt->shadow_page;
	struct vgt_device *vgt = spt->vgt;
	gtt_entry_t m;
	ppgtt_spt_t *s;

	trace_gpt_change(spt->vgt->vm_id, "add", spt, sp->type, we->val64, index);

	if (gtt_type_is_pt(get_next_pt_type(we->type))) {
		s = ppgtt_populate_shadow_page_by_guest_entry(vgt, we);
		if (!s)
			goto fail;
		ppgtt_get_shadow_entry(spt, &m, index);
		ppgtt_generate_shadow_entry(&m, s, we);
		ppgtt_set_shadow_entry(spt, &m, index);
	} else {
		if (!gtt_entry_p2m(vgt, we, &m))
			goto fail;
		ppgtt_set_shadow_entry(spt, &m, index);
	}

	return true;

fail:
	gvt_err("fail: spt %p guest entry 0x%llx type %d.", spt, we->val64, we->type);
	return false;
}

static bool vgt_sync_oos_page(struct vgt_device *vgt, oos_page_t *oos_page)
{
	struct gvt_device_info *info = &vgt->pdev->device_info;
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_gtt_pte_ops *ops = pdev->gtt.pte_ops;
	ppgtt_spt_t *spt = guest_page_to_ppgtt_spt(oos_page->guest_page);
	gtt_entry_t old, new, m;
	int index;

	trace_oos_change(vgt->vm_id, "sync", oos_page->id,
			oos_page->guest_page, spt->guest_page_type);

	old.type = new.type = get_entry_type(spt->guest_page_type);
	old.pdev = new.pdev = pdev;
	old.val64 = new.val64 = 0;

	for (index = 0; index < (GTT_PAGE_SIZE >> info->gtt_entry_size_shift); index++) {
		ops->get_entry(oos_page->mem, &old, index, false, NULL);
		ops->get_entry(oos_page->guest_page->vaddr, &new, index, true, vgt);

		if (old.val64 == new.val64)
			continue;

		trace_oos_sync(vgt->vm_id, oos_page->id,
				oos_page->guest_page, spt->guest_page_type,
				new.val64, index);

		if (!gtt_entry_p2m(vgt, &new, &m))
			return false;

		ops->set_entry(oos_page->mem, &new, index, false, NULL);
		ppgtt_set_shadow_entry(spt, &m, index);
	}

	oos_page->guest_page->write_cnt = 0;

	return true;
}

static bool gvt_detach_oos_page(struct vgt_device *vgt, oos_page_t *oos_page)
{
	struct pgt_device *pdev = vgt->pdev;
	ppgtt_spt_t *spt = guest_page_to_ppgtt_spt(oos_page->guest_page);

	trace_oos_change(vgt->vm_id, "detach", oos_page->id,
			oos_page->guest_page, spt->guest_page_type);

	oos_page->guest_page->write_cnt = 0;
	oos_page->guest_page->oos_page = NULL;
	oos_page->guest_page = NULL;

	list_del_init(&oos_page->vm_list);
	list_move_tail(&oos_page->list, &pdev->gtt.oos_page_free_list_head);

	pdev->stat.oos_page_cur_avail_cnt++;
	pdev->stat.oos_page_detach_cnt++;

	return true;
}

static oos_page_t *gvt_attach_oos_page(struct vgt_device *vgt,
		oos_page_t *oos_page, guest_page_t *gpt)
{
	struct pgt_device *pdev = vgt->pdev;

	if (!hypervisor_read_va(vgt, gpt->vaddr, oos_page->mem, GTT_PAGE_SIZE, 1))
		return NULL;

	oos_page->guest_page = gpt;
	gpt->oos_page = oos_page;

	list_move_tail(&oos_page->list, &pdev->gtt.oos_page_use_list_head);

	if (--pdev->stat.oos_page_cur_avail_cnt < pdev->stat.oos_page_min_avail_cnt)
		pdev->stat.oos_page_min_avail_cnt = pdev->stat.oos_page_cur_avail_cnt;

	trace_oos_change(vgt->vm_id, "attach", gpt->oos_page->id,
			gpt, guest_page_to_ppgtt_spt(gpt)->guest_page_type);

	pdev->stat.oos_page_attach_cnt++;

	return oos_page;
}

static bool ppgtt_set_guest_page_sync(struct vgt_device *vgt, guest_page_t *gpt)
{
	if (!hypervisor_set_wp_pages(vgt, gpt))
		return false;

	trace_oos_change(vgt->vm_id, "set page sync", gpt->oos_page->id,
			gpt, guest_page_to_ppgtt_spt(gpt)->guest_page_type);

	list_del_init(&gpt->oos_page->vm_list);
	return vgt_sync_oos_page(vgt, gpt->oos_page);
}

static bool ppgtt_allocate_oos_page(struct vgt_device *vgt, guest_page_t *gpt)
{
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_gtt_info *gtt = &pdev->gtt;
	oos_page_t *oos_page = gpt->oos_page;

	/* oos_page should be NULL at this point */
	ASSERT(!oos_page);

	if (list_empty(&gtt->oos_page_free_list_head)) {
		oos_page = container_of(gtt->oos_page_use_list_head.next, oos_page_t, list);
		if (!ppgtt_set_guest_page_sync(vgt, oos_page->guest_page)
			|| !gvt_detach_oos_page(vgt, oos_page))
			return false;
		ASSERT(!list_empty(&gtt->oos_page_free_list_head));
		pdev->stat.oos_page_steal_cnt++;
	} else
		oos_page = container_of(gtt->oos_page_free_list_head.next, oos_page_t, list);

	return gvt_attach_oos_page(vgt, oos_page, gpt);
}

static bool ppgtt_set_guest_page_oos(struct vgt_device *vgt, guest_page_t *gpt)
{
	oos_page_t *oos_page = gpt->oos_page;

	ASSERT(oos_page);

	trace_oos_change(vgt->vm_id, "set page out of sync", gpt->oos_page->id,
			gpt, guest_page_to_ppgtt_spt(gpt)->guest_page_type);

	list_add_tail(&oos_page->vm_list, &vgt->gtt.oos_page_list_head);
	return hypervisor_unset_wp_pages(vgt, gpt);
}

bool ppgtt_sync_oos_pages(struct vgt_device *vgt)
{
	struct list_head *pos, *n;
	oos_page_t *oos_page;

	if (!gvt.spt_out_of_sync)
		return true;

	list_for_each_safe(pos, n, &vgt->gtt.oos_page_list_head) {
		oos_page = container_of(pos, oos_page_t, vm_list);
		if (!ppgtt_set_guest_page_sync(vgt, oos_page->guest_page))
			return false;
	}

	return true;
}

/*
 * The heart of PPGTT shadow page table.
 */
static bool ppgtt_handle_guest_write_page_table(guest_page_t *gpt, gtt_entry_t *we,
		unsigned long index)
{
	ppgtt_spt_t *spt = guest_page_to_ppgtt_spt(gpt);
	struct vgt_device *vgt = spt->vgt;
	struct gvt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	gtt_entry_t ge;

	int old_present, new_present;

	ppgtt_get_guest_entry(spt, &ge, index);

	old_present = ops->test_present(&ge);
	new_present = ops->test_present(we);

	ppgtt_set_guest_entry(spt, we, index);

	if (old_present && new_present) {
		if (!ppgtt_handle_guest_entry_removal(gpt, &ge, index)
		|| !ppgtt_handle_guest_entry_add(gpt, we, index))
			goto fail;
	} else if (!old_present && new_present) {
		if (!ppgtt_handle_guest_entry_add(gpt, we, index))
			goto fail;
	} else if (old_present && !new_present) {
		if (!ppgtt_handle_guest_entry_removal(gpt, &ge, index))
			goto fail;
	}
	return true;
fail:
	gvt_err("fail: shadow page %p guest entry 0x%llx type %d.",
			spt, we->val64, we->type);
	return false;
}

static inline bool can_do_out_of_sync(guest_page_t *gpt)
{
	return gvt.spt_out_of_sync
		&& gtt_type_is_pte_pt(guest_page_to_ppgtt_spt(gpt)->guest_page_type)
		&& gpt->write_cnt >= 2;
}

bool ppgtt_check_partial_access(struct vgt_device *vgt)
{
	struct gvt_vgtt_info *gtt = &vgt->gtt;

	if (gtt->last_partial_ppgtt_access_index == -1)
		return true;

	if (!gtt->warn_partial_ppgtt_access_once) {
		gvt_warn("Incomplete PPGTT page table access sequence.");
		gtt->warn_partial_ppgtt_access_once = true;
	}

	if (!ppgtt_handle_guest_write_page_table(
			gtt->last_partial_ppgtt_access_gpt,
			&gtt->last_partial_ppgtt_access_entry,
			gtt->last_partial_ppgtt_access_index))
		return false;

	gtt->last_partial_ppgtt_access_index = -1;
	return true;
}

static bool ppgtt_handle_guest_write_page_table_bytes(void *gp,
		uint64_t pa, void *p_data, int bytes)
{
	guest_page_t *gpt = (guest_page_t *)gp;
	ppgtt_spt_t *spt = guest_page_to_ppgtt_spt(gpt);
	struct vgt_device *vgt = spt->vgt;
	struct gvt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	struct gvt_device_info *info = &vgt->pdev->device_info;
	struct gvt_vgtt_info *gtt = &vgt->gtt;
	gtt_entry_t we, se;
	unsigned long index;

	bool partial_access = (bytes != info->gtt_entry_size);
	bool hi = (partial_access && (pa & (info->gtt_entry_size - 1)));

	index = (pa & (PAGE_SIZE - 1)) >> info->gtt_entry_size_shift;

	ppgtt_get_guest_entry(spt, &we, index);
	memcpy(&we.val64 + (pa & (info->gtt_entry_size - 1)), p_data, bytes);

	if (partial_access && !hi) {
		trace_gpt_change(vgt->vm_id, "partial access - LOW",
				NULL, we.type, *(u32 *)(p_data), index);

		ppgtt_check_partial_access(vgt);

		ppgtt_set_guest_entry(spt, &we, index);
		ppgtt_get_shadow_entry(spt, &se, index);

		if (!ops->test_present(&se))
			return true;

		if (gtt_type_is_pt(get_next_pt_type(se.type)))
			if (!ppgtt_invalidate_shadow_page_by_shadow_entry(vgt, &se))
				return false;

		se.val64 = 0;
		ppgtt_set_shadow_entry(spt, &se, index);

		gtt->last_partial_ppgtt_access_index = index;
		gtt->last_partial_ppgtt_access_gpt = gpt;
		gtt->last_partial_ppgtt_access_entry = we;

		return true;
	} else
		gtt->last_partial_ppgtt_access_index = -1;

	if (hi)
		trace_gpt_change(vgt->vm_id, "partial access - HIGH",
				NULL, we.type, *(u32 *)(p_data), index);

	ops->test_pse(&we);

	gpt->write_cnt++;

	if (!ppgtt_handle_guest_write_page_table(gpt, &we, index))
		return false;

	if (gvt.spt_out_of_sync) {
		if (gpt->oos_page) {
			/* 1. only GTT_PTE type has oos_page assocaited
			 * 2. update oos_page according to wp guest page change
			 */
			ops->set_entry(gpt->oos_page->mem, &we, index, false, NULL);
		}

		if (can_do_out_of_sync(gpt)) {
			if (!gpt->oos_page)
				ppgtt_allocate_oos_page(vgt, gpt);

			if (!ppgtt_set_guest_page_oos(vgt, gpt)) {
				/* should not return false since we can handle it*/
				ppgtt_set_guest_page_sync(vgt, gpt);
			}
		}

	}

	return true;
}

bool ppgtt_handle_guest_write_root_pointer(struct gvt_mm *mm,
		gtt_entry_t *we, unsigned long index)
{
	struct vgt_device *vgt = mm->vgt;
	struct gvt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	ppgtt_spt_t *spt = NULL;
	gtt_entry_t e;

	if (mm->type != GVT_MM_PPGTT || !mm->shadowed)
		return false;

	trace_gpt_change(vgt->vm_id, __func__, NULL,
			we->type, we->val64, index);

	ppgtt_get_guest_root_entry(mm, &e, index);

	if (ops->test_present(&e)) {
		ppgtt_get_shadow_root_entry(mm, &e, index);

		trace_gpt_change(vgt->vm_id, "destroy old root pointer",
				spt, e.type, e.val64, index);

		if (gtt_type_is_pt(get_next_pt_type(e.type))) {
			if (!ppgtt_invalidate_shadow_page_by_shadow_entry(vgt, &e))
				goto fail;
		} else {
			gvt_err("GVT doesn't support pse bit now.");
			goto fail;
		}
		e.val64 = 0;
		ppgtt_set_shadow_root_entry(mm, &e, index);
	}

	if (ops->test_present(we)) {
		if (gtt_type_is_pt(get_next_pt_type(we->type))) {
			spt = ppgtt_populate_shadow_page_by_guest_entry(vgt, we);
			if (!spt) {
				gvt_err("fail to populate root pointer.");
				goto fail;
			}
			ppgtt_generate_shadow_entry(&e, spt, we);
			ppgtt_set_shadow_root_entry(mm, &e, index);
		} else {
			gvt_err("GVT doesn't support pse bit now.");
			goto fail;
		}
		trace_gpt_change(vgt->vm_id, "populate root pointer",
				spt, e.type, e.val64, index);
	}
	return true;
fail:
	gvt_err("fail: shadow page %p guest entry 0x%llx type %d.",
			spt, we->val64, we->type);
	return false;
}

/*
 * mm page table allocation policy for pre-bdw:
 *  - for ggtt, a virtual page table will be allocated.
 *  - for ppgtt, the virtual page table(root entry) will use a part of
 *	virtual page table from ggtt.
 */
bool gen7_mm_alloc_page_table(struct gvt_mm *mm)
{
	struct vgt_device *vgt = mm->vgt;
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_vgtt_info *gtt = &vgt->gtt;
	struct gvt_device_info *info = &pdev->device_info;
	void *mem;

	if (mm->type == GVT_MM_PPGTT) {
		struct gvt_mm *ggtt_mm = gtt->ggtt_mm;
		if (!ggtt_mm) {
			gvt_err("ggtt mm hasn't been created.");
			return false;
		}
		mm->page_table_entry_cnt = 512;
		mm->page_table_entry_size = mm->page_table_entry_cnt *
			info->gtt_entry_size;
		mm->virtual_page_table = ggtt_mm->virtual_page_table +
			(mm->pde_base_index << info->gtt_entry_size_shift);
		/* shadow page table resides in the hw mmio entries. */
	} else if (mm->type == GVT_MM_GGTT) {
		mm->page_table_entry_cnt = (gm_sz(pdev) >> GTT_PAGE_SHIFT);
		mm->page_table_entry_size = mm->page_table_entry_cnt *
			info->gtt_entry_size;
		mem = vzalloc(mm->page_table_entry_size);
		if (!mem) {
			gvt_err("fail to allocate memory.");
			return false;
		}
		mm->virtual_page_table = mem;
	}
	return true;
}

void gen7_mm_free_page_table(struct gvt_mm *mm)
{
	if (mm->type == GVT_MM_GGTT) {
		if (mm->virtual_page_table)
			vfree(mm->virtual_page_table);
	}
	mm->virtual_page_table = mm->shadow_page_table = NULL;
}

/*
 * mm page table allocation policy for bdw+
 *  - for ggtt, only virtual page table will be allocated.
 *  - for ppgtt, dedicated virtual/shadow page table will be allocated.
 */
bool gen8_mm_alloc_page_table(struct gvt_mm *mm)
{
	struct vgt_device *vgt = mm->vgt;
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_device_info *info = &pdev->device_info;
	void *mem;

	if (mm->type == GVT_MM_PPGTT) {
		mm->page_table_entry_cnt = 4;
		mm->page_table_entry_size = mm->page_table_entry_cnt *
			info->gtt_entry_size;
		mem = kzalloc(mm->has_shadow_page_table ?
			mm->page_table_entry_size * 2 : mm->page_table_entry_size,
			GFP_ATOMIC);
		if (!mem) {
			gvt_err("fail to allocate memory.");
			return false;
		}
		mm->virtual_page_table = mem;
		if (!mm->has_shadow_page_table)
			return true;
		mm->shadow_page_table = mem + mm->page_table_entry_size;
	} else if (mm->type == GVT_MM_GGTT) {
		mm->page_table_entry_cnt = (gm_sz(pdev) >> GTT_PAGE_SHIFT);
		mm->page_table_entry_size = mm->page_table_entry_cnt *
			info->gtt_entry_size;
		mem = vzalloc(mm->page_table_entry_size);
		if (!mem) {
			gvt_err("fail to allocate memory.");
			return false;
		}
		mm->virtual_page_table = mem;
	}
	return true;
}

void gen8_mm_free_page_table(struct gvt_mm *mm)
{
	if (mm->type == GVT_MM_PPGTT) {
		if (mm->virtual_page_table)
			kfree(mm->virtual_page_table);
	} else if (mm->type == GVT_MM_GGTT) {
		if (mm->virtual_page_table)
			vfree(mm->virtual_page_table);
	}
	mm->virtual_page_table = mm->shadow_page_table = NULL;
}

void gvt_destroy_mm(struct gvt_mm *mm)
{
	struct vgt_device *vgt = mm->vgt;
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_gtt_info *gtt = &pdev->gtt;
	struct gvt_gtt_pte_ops *ops = gtt->pte_ops;
	gtt_entry_t se;
	int i;

	if (!mm->initialized)
		goto out;

	if (atomic_dec_return(&mm->refcount) > 0)
		return;

	list_del(&mm->list);

	if (mm->has_shadow_page_table && mm->shadowed) {
		for (i = 0; i < mm->page_table_entry_cnt; i++) {
			ppgtt_get_shadow_root_entry(mm, &se, i);
			if (!ops->test_present(&se))
				continue;
			ppgtt_invalidate_shadow_page_by_shadow_entry(vgt, &se);
			se.val64 = 0;
			ppgtt_set_shadow_root_entry(mm, &se, i);

			trace_gpt_change(vgt->vm_id, "destroy root pointer",
					NULL, se.type, se.val64, i);
		}
	}
	gtt->mm_free_page_table(mm);
out:
	kfree(mm);
}

struct gvt_mm *gvt_create_mm(struct vgt_device *vgt,
		gvt_mm_type_t mm_type, gtt_type_t page_table_entry_type,
		void *virtual_page_table, int page_table_level,
		u32 pde_base_index)
{
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_gtt_info *gtt = &pdev->gtt;
	struct gvt_gtt_pte_ops *ops = gtt->pte_ops;
	struct gvt_mm *mm;
	ppgtt_spt_t *spt;
	gtt_entry_t ge, se;
	int i;

	mm = kzalloc(sizeof(*mm), GFP_ATOMIC);
	if (!mm) {
		gvt_err("fail to allocate memory for mm.");
		goto fail;
	}

	mm->type = mm_type;
	mm->page_table_entry_type = page_table_entry_type;
	mm->page_table_level = page_table_level;
	mm->pde_base_index = pde_base_index;

	mm->vgt = vgt;
	mm->has_shadow_page_table = (vgt->vm_id != 0 && mm_type == GVT_MM_PPGTT);

	atomic_set(&mm->refcount, 1);
	INIT_LIST_HEAD(&mm->list);
	list_add_tail(&mm->list, &vgt->gtt.mm_list_head);

	if (!gtt->mm_alloc_page_table(mm)) {
		gvt_err("fail to allocate page table for mm.");
		goto fail;
	}

	mm->initialized = true;

	if (virtual_page_table)
		memcpy(mm->virtual_page_table, virtual_page_table,
				mm->page_table_entry_size);

	if (mm->has_shadow_page_table) {
		for (i = 0; i < mm->page_table_entry_cnt; i++) {
			ppgtt_get_guest_root_entry(mm, &ge, i);
			if (!ops->test_present(&ge))
				continue;

			trace_gpt_change(vgt->vm_id, __func__, NULL,
					ge.type, ge.val64, i);

			spt = ppgtt_populate_shadow_page_by_guest_entry(vgt, &ge);
			if (!spt) {
				gvt_err("fail to populate guest root pointer.");
				goto fail;
			}
			ppgtt_generate_shadow_entry(&se, spt, &ge);
			ppgtt_set_shadow_root_entry(mm, &se, i);

			trace_gpt_change(vgt->vm_id, "populate root pointer",
					NULL, se.type, se.val64, i);
		}
		mm->shadowed = true;
	}
	return mm;
fail:
	gvt_err("fail to create mm.");
	if (mm)
		gvt_destroy_mm(mm);
	return NULL;
}

/*
 * GMA translation APIs.
 */
static inline bool ppgtt_get_next_level_entry(struct gvt_mm *mm,
		gtt_entry_t *e, unsigned long index, bool guest)
{
	struct vgt_device *vgt = mm->vgt;
	struct gvt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
	ppgtt_spt_t *s;
	void *pt;

	if (mm->has_shadow_page_table) {
		if (!(s = ppgtt_find_shadow_page(vgt, ops->get_pfn(e))))
			return false;
		if (!guest)
			ppgtt_get_shadow_entry(s, e, index);
		else
			ppgtt_get_guest_entry(s, e, index);
	} else {
		pt = hypervisor_mfn_to_virt(ops->get_pfn(e));
		ops->get_entry(pt, e, index, false, NULL);
		e->type = get_entry_type(get_next_pt_type(e->type));
	}
	return true;
}

static inline unsigned long gvt_gma_to_gpa(struct gvt_mm *mm, unsigned long gma)
{
	struct vgt_device *vgt = mm->vgt;
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_gtt_pte_ops *pte_ops = pdev->gtt.pte_ops;
	struct gvt_gtt_gma_ops *gma_ops = pdev->gtt.gma_ops;

	unsigned long gpa = INVALID_ADDR;
	unsigned long gma_index[4];
	gtt_entry_t e;
	int i, index;

	if (mm->type != GVT_MM_GGTT && mm->type != GVT_MM_PPGTT)
		return INVALID_ADDR;

	if (mm->type == GVT_MM_GGTT) {
		if (!g_gm_is_valid(vgt, gma))
			goto err;

		ggtt_get_guest_entry(mm, &e, gma_ops->gma_to_ggtt_pte_index(gma));
		gpa = (pte_ops->get_pfn(&e) << GTT_PAGE_SHIFT) + (gma & ~GTT_PAGE_MASK);

		trace_gma_translate(vgt->vm_id, "ggtt", 0, 0, gma, gpa);

		return gpa;
	}

	switch (mm->page_table_level) {
		case 4:
			ppgtt_get_shadow_root_entry(mm, &e, 0);
			gma_index[0] = gma_ops->gma_to_pml4_index(gma);
			gma_index[1] = gma_ops->gma_to_l4_pdp_index(gma);
			gma_index[2] = gma_ops->gma_to_pde_index(gma);
			gma_index[3] = gma_ops->gma_to_pte_index(gma);
			index = 4;
			break;
		case 3:
			ppgtt_get_shadow_root_entry(mm, &e, gma_ops->gma_to_l3_pdp_index(gma));
			gma_index[0] = gma_ops->gma_to_pde_index(gma);
			gma_index[1] = gma_ops->gma_to_pte_index(gma);
			index = 2;
			break;
		case 2:
			ppgtt_get_shadow_root_entry(mm, &e, gma_ops->gma_to_pde_index(gma));
			gma_index[0] = gma_ops->gma_to_pte_index(gma);
			index = 1;
			break;
		default:
			BUG();
	}
	/* walk into the last level shadow page table and get gpa from guest entry */
	for (i = 0; i < index; i++)
		if (!ppgtt_get_next_level_entry(mm, &e, gma_index[i],
			(i == index - 1)))
			goto err;

	gpa = (pte_ops->get_pfn(&e) << GTT_PAGE_SHIFT) + (gma & ~GTT_PAGE_MASK);

	trace_gma_translate(vgt->vm_id, "ppgtt", 0, mm->page_table_level, gma, gpa);

	return gpa;
err:
	gvt_err("invalid mm type: %d, gma %lx", mm->type, gma);
	return INVALID_ADDR;
}

void *gvt_gma_to_va(struct gvt_mm *mm, unsigned long gma)
{
	struct vgt_device *vgt = mm->vgt;
	unsigned long gpa;

	gpa = gvt_gma_to_gpa(mm, gma);
	if (gpa == INVALID_ADDR) {
		gvt_warn("invalid gpa! gma 0x%lx, mm type %d", gma, mm->type);
		return NULL;
	}

	return hypervisor_gpa_to_va(vgt, gpa);
}

/*
 * GTT MMIO emulation.
 */
bool gtt_mmio_read(struct vgt_device *vgt,
	unsigned int off, void *p_data, unsigned int bytes)
{
	struct gvt_mm *ggtt_mm = vgt->gtt.ggtt_mm;
	struct gvt_device_info *info = &vgt->pdev->device_info;
	unsigned long index = off >> info->gtt_entry_size_shift;
	gtt_entry_t e;

	if (bytes != 4 && bytes != 8)
		return false;

	ggtt_get_guest_entry(ggtt_mm, &e, index);
	memcpy(p_data, &e.val64 + (off & (info->gtt_entry_size - 1)), bytes);

	return true;
}

bool gtt_emulate_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	struct gvt_device_info *info = &vgt->pdev->device_info;
	int ret;
	cycles_t t0, t1;
	struct gvt_statistics *stat = &vgt->stat;

	if (bytes != 4 && bytes != 8)
		return false;

	t0 = get_cycles();
	stat->gtt_mmio_rcnt++;

	off -= info->gtt_start_offset;

	ret = gtt_mmio_read(vgt, off, p_data, bytes);

	t1 = get_cycles();
	stat->gtt_mmio_rcycles += (u64) (t1 - t0);
	return ret;
}

bool gtt_mmio_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	struct pgt_device *pdev = vgt->pdev;
	struct gvt_device_info *info = &pdev->device_info;
	struct gvt_mm *ggtt_mm = vgt->gtt.ggtt_mm;
	unsigned long g_gtt_index = off >> info->gtt_entry_size_shift;
	bool partial_access = (bytes != info->gtt_entry_size);
	bool hi = (partial_access && (off & (info->gtt_entry_size - 1)));
	unsigned long gma;
	gtt_entry_t e, m;
	int rc;

	if (bytes != 4 && bytes != 8)
		return false;

	gma = g_gtt_index << GTT_PAGE_SHIFT;
	/* the VM may configure the whole GM space when ballooning is used */
	if (!g_gm_is_valid(vgt, gma)) {
		static int count = 0;

		/* print info every 32MB */
		if (!(count % 8192))
			gvt_dbg_mm("[vgt %d]capture ballooned write for %d times (%x)",
				vgt->vm_id, count, off);

		count++;
		/* in this case still return true since the impact is on vgtt only */
		return true;
	}

	ggtt_get_guest_entry(ggtt_mm, &e, g_gtt_index);

	memcpy(&e.val64 + (off & (info->gtt_entry_size - 1)), p_data, bytes);

	if (partial_access && !hi)
		goto out;

	rc = gtt_entry_p2m(vgt, &e, &m);
	if (!rc) {
		gvt_err("VM %d: failed to translate guest gtt entry", vgt->vm_id);
		return false;
	}

	ggtt_set_shadow_entry(ggtt_mm, &m, g_gtt_index);
out:
	ggtt_set_guest_entry(ggtt_mm, &e, g_gtt_index);
	return true;
}

bool gtt_emulate_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes)
{
	struct gvt_device_info *info = &vgt->pdev->device_info;
	int ret;
	cycles_t t0, t1;
	struct gvt_statistics *stat = &vgt->stat;

	if (bytes != 4 && bytes != 8)
		return false;

	t0 = get_cycles();
	stat->gtt_mmio_wcnt++;

	off -= info->gtt_start_offset;

	ret = gtt_mmio_write(vgt, off, p_data, bytes);

	t1 = get_cycles();
	stat->gtt_mmio_wcycles += (u64) (t1 - t0);
	return ret;
}

bool vgt_expand_shadow_page_mempool(struct pgt_device *pdev)
{
	mempool_t *mempool = pdev->gtt.mempool;
	bool rc = true;
	int new_min_nr;

	mutex_lock(&pdev->gtt.mempool_lock);

	if (mempool->curr_nr >= gvt.preallocated_shadow_pages / 3)
		goto out;

	/*
	 * Have to do this to let the pool expand directly.
	 */
	new_min_nr = gvt.preallocated_shadow_pages - 1;
	if (mempool_resize(mempool, new_min_nr)) {
		gvt_err("fail to resize the mempool.");
		rc = false;
		goto out;
	}

	new_min_nr = gvt.preallocated_shadow_pages;
	if (mempool_resize(mempool, new_min_nr)) {
		gvt_err("fail to resize the mempool.");
		rc = false;
		goto out;
	}

out:
	mutex_unlock(&pdev->gtt.mempool_lock);
	return rc;
}

static void *mempool_alloc_spt(gfp_t gfp_mask, void *pool_data)
{
	ppgtt_spt_t *spt;

	spt = kzalloc(sizeof(*spt), gfp_mask);
	if (!spt)
		return NULL;

	spt->shadow_page.page = alloc_page(gfp_mask);
	if (!spt->shadow_page.page) {
		kfree(spt);
		return NULL;
	}
	return spt;
}

static void mempool_free_spt(void *element, void *pool_data)
{
	ppgtt_spt_t *spt = element;

	__free_page(spt->shadow_page.page);
	kfree(spt);
}

bool gvt_init_vgtt(struct vgt_device *vgt)
{
	struct gvt_vgtt_info *gtt = &vgt->gtt;
	struct gvt_mm *ggtt_mm;

	hash_init(gtt->guest_page_hash_table);
	hash_init(gtt->shadow_page_hash_table);

	INIT_LIST_HEAD(&gtt->mm_list_head);
	INIT_LIST_HEAD(&gtt->oos_page_list_head);

	gtt->last_partial_ppgtt_access_index = -1;

	if (!vgt_expand_shadow_page_mempool(vgt->pdev)) {
		gvt_err("fail to expand the shadow page mempool.");
		return false;
	}

	ggtt_mm = gvt_create_mm(vgt, GVT_MM_GGTT,
			GTT_TYPE_GGTT_PTE, NULL, 1, 0);
	if (!ggtt_mm) {
		gvt_err("fail to create mm for ggtt.");
		return false;
	}

	gtt->ggtt_mm = ggtt_mm;
	return true;
}

void gvt_clean_vgtt(struct vgt_device *vgt)
{
	struct list_head *pos, *n;
	struct gvt_mm *mm;

	ppgtt_free_all_shadow_page(vgt);

	list_for_each_safe(pos, n, &vgt->gtt.mm_list_head) {
		mm = container_of(pos, struct gvt_mm, list);
		vgt->pdev->gtt.mm_free_page_table(mm);
		kfree(mm);
	}

	return;
}

static void gvt_clean_spt_oos(struct pgt_device *pdev)
{
	struct gvt_gtt_info *gtt = &pdev->gtt;
	struct list_head *pos, *n;
	oos_page_t *oos_page;

	ASSERT(list_empty(&gtt->oos_page_use_list_head));

	list_for_each_safe(pos, n, &gtt->oos_page_free_list_head) {
		oos_page = container_of(pos, oos_page_t, list);
		list_del(&oos_page->list);
		kfree(oos_page);
	}
}

static bool gvt_setup_spt_oos(struct pgt_device *pdev)
{
	struct gvt_gtt_info *gtt = &pdev->gtt;
	oos_page_t *oos_page;
	int i;

	INIT_LIST_HEAD(&gtt->oos_page_free_list_head);
	INIT_LIST_HEAD(&gtt->oos_page_use_list_head);

	for (i = 0; i < gvt.preallocated_oos_pages; i++) {
		oos_page = kzalloc(sizeof(*oos_page), GFP_KERNEL);
		if (!oos_page) {
			gvt_err("fail to pre-allocate oos page.");
			goto fail;
		}

		INIT_LIST_HEAD(&oos_page->list);
		INIT_LIST_HEAD(&oos_page->vm_list);
		oos_page->id = i;
		list_add_tail(&oos_page->list, &gtt->oos_page_free_list_head);
	}

	pdev->stat.oos_page_cur_avail_cnt = gvt.preallocated_oos_pages;
	pdev->stat.oos_page_min_avail_cnt = gvt.preallocated_oos_pages;
	pdev->stat.oos_page_steal_cnt = 0;
	pdev->stat.oos_page_attach_cnt = 0;
	pdev->stat.oos_page_detach_cnt = 0;

	gvt_info("%d oos pages preallocated", gvt.preallocated_oos_pages);

	return true;
fail:
	gvt_clean_spt_oos(pdev);
	return false;
}

bool gvt_init_avail_gtt_size(struct pgt_device *pdev)
{
	struct gvt_device_info *info = &pdev->device_info;
	struct drm_i915_private *dev_priv = pdev->dev_priv;
	u16 gmch_ctrl;
	u64 gtt_size;

	pci_read_config_word(dev_priv->dev->pdev, SNB_GMCH_CTRL, &gmch_ctrl);

	gmch_ctrl = (gmch_ctrl >> 6) & 3;
	if (gmch_ctrl)
		gmch_ctrl = 1 << gmch_ctrl;

	switch (gmch_ctrl) {
		case 2:
		case 4:
		case 8:
			gtt_size = gmch_ctrl << 20;
			break;
		default:
			gvt_err("invalid GTT memory size: %x", gmch_ctrl);
			return false;
	}

	info->gtt_end_offset = info->gtt_start_offset + gtt_size;
	pdev->total_gm_sz = gtt_size >> info->gtt_entry_size_shift << GTT_PAGE_SHIFT;

	gvt_info("Available GTT size: 0x%llx availible GM size: 0x%llx",
		gtt_size, pdev->total_gm_sz);

	return true;
}

bool gvt_init_gtt(struct pgt_device *pdev)
{
	gvt_dbg_core("init gtt");

	if (IS_BROADWELL(pdev->dev_priv)) {
		pdev->gtt.pte_ops = &gen8_gtt_pte_ops;
		pdev->gtt.gma_ops = &gen8_gtt_gma_ops;
		pdev->gtt.mm_alloc_page_table = gen8_mm_alloc_page_table;
		pdev->gtt.mm_free_page_table = gen8_mm_free_page_table;

		if (gvt.preallocated_shadow_pages == -1)
			gvt.preallocated_shadow_pages = 8192;
		if (gvt.preallocated_oos_pages == -1)
			gvt.preallocated_oos_pages = 4096;
	} else {
		gvt_err("Unsupported platform.");
		return false;
	}

	if (gvt.spt_out_of_sync) {
		if (!gvt_setup_spt_oos(pdev)) {
			gvt_err("fail to initialize SPT oos.");
			return false;
		}
	}

	mutex_init(&pdev->gtt.mempool_lock);

	pdev->gtt.mempool = mempool_create(gvt.preallocated_shadow_pages,
		mempool_alloc_spt, mempool_free_spt, pdev);
	if (!pdev->gtt.mempool) {
		gvt_err("fail to create mempool.");
		gvt_clean_spt_oos(pdev);
		return false;
	}

	return true;
}

void gvt_clean_gtt(struct pgt_device *pdev)
{
	if (gvt.spt_out_of_sync)
		gvt_clean_spt_oos(pdev);

	mempool_destroy(pdev->gtt.mempool);
}

struct gvt_mm *gen8_find_ppgtt_mm(struct vgt_device *vgt,
		int page_table_level, void *root_entry)
{
	struct list_head *pos;
	struct gvt_mm *mm;
	u64 *src, *dst;

	list_for_each(pos, &vgt->gtt.mm_list_head) {
		mm = container_of(pos, struct gvt_mm, list);
		if (mm->type != GVT_MM_PPGTT)
			continue;

		if (mm->page_table_level != page_table_level)
			continue;

		src = root_entry;
		dst = mm->virtual_page_table;

		if (page_table_level == 3) {
			if (src[0] == dst[0]
					&& src[1] == dst[1]
					&& src[2] == dst[2]
					&& src[3] == dst[3])
				return mm;
		} else {
			if (src[0] == dst[0])
				return mm;
		}
	}

	return NULL;
}

bool gvt_g2v_create_ppgtt_mm(struct vgt_device *vgt, int page_table_level)
{
	u64 *pdp = (u64 *)&__vreg64(vgt, _vgtif_reg(pdp[0].lo));
	gtt_type_t root_entry_type = page_table_level == 4 ?
		GTT_TYPE_PPGTT_ROOT_L4_ENTRY : GTT_TYPE_PPGTT_ROOT_L3_ENTRY;

	struct gvt_mm *mm;

	ASSERT(page_table_level == 4 || page_table_level == 3);

	mm = gen8_find_ppgtt_mm(vgt, page_table_level, pdp);
	if (mm) {
		atomic_inc(&mm->refcount);
	} else {
		mm = gvt_create_mm(vgt, GVT_MM_PPGTT, root_entry_type,
				pdp, page_table_level, 0);
		if (!mm)
			return false;
	}

	return true;
}

bool gvt_g2v_destroy_ppgtt_mm(struct vgt_device *vgt, int page_table_level)
{
	u64 *pdp = (u64 *)&__vreg64(vgt, _vgtif_reg(pdp[0].lo));
	struct gvt_mm *mm;

	ASSERT(page_table_level == 4 || page_table_level == 3);

	mm = gen8_find_ppgtt_mm(vgt, page_table_level, pdp);
	if (!mm) {
		gvt_err("fail to find ppgtt instance.");
		return false;
	}

	gvt_destroy_mm(mm);

	return true;
}
