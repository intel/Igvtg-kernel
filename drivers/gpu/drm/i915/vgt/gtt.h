/*
 * vGT gtt header
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

#ifndef _VGT_GTT_H_
#define _VGT_GTT_H_

struct vgt_mm;

#define	VGT_HASH_BITS	8

#define GTT_PAGE_SHIFT		12
#define GTT_PAGE_SIZE		(1UL << GTT_PAGE_SHIFT)
#define GTT_PAGE_MASK		(~(GTT_PAGE_SIZE-1))
#define GTT_PAE_MASK		((1UL <<12) - (1UL << 4)) /* bit 11:4 */

#define INVALID_ADDR (~0UL)

#define INVALID_MFN	(~0UL)

#define gtt_type_is_entry(type) \
	(type > GTT_TYPE_INVALID && type < GTT_TYPE_PPGTT_ENTRY \
	 && type != GTT_TYPE_PPGTT_PTE_ENTRY \
	 && type != GTT_TYPE_PPGTT_ROOT_ENTRY)

#define gtt_type_is_pt(type) \
	(type >= GTT_TYPE_PPGTT_PTE_PT && type < GTT_TYPE_MAX)

#define gtt_type_is_pte_pt(type) \
	(type == GTT_TYPE_PPGTT_PTE_PT)

#define gtt_type_is_root_pointer(type) \
	(gtt_type_is_entry(type) && type > GTT_TYPE_PPGTT_ROOT_ENTRY)

#define gtt_init_entry(e, t, p, v) do { \
	(e)->type = t; \
	(e)->pdev = p; \
	memcpy(&(e)->val64, &v, sizeof(v)); \
}while(0)

typedef enum {
	GTT_TYPE_INVALID = -1,

	GTT_TYPE_GGTT_PTE,

	GTT_TYPE_PPGTT_PTE_4K_ENTRY,
	GTT_TYPE_PPGTT_PTE_2M_ENTRY,
	GTT_TYPE_PPGTT_PTE_1G_ENTRY,

	GTT_TYPE_PPGTT_PTE_ENTRY,

	GTT_TYPE_PPGTT_PDE_ENTRY,
	GTT_TYPE_PPGTT_PDP_ENTRY,
	GTT_TYPE_PPGTT_PML4_ENTRY,

	GTT_TYPE_PPGTT_ROOT_ENTRY,

	GTT_TYPE_PPGTT_ROOT_L3_ENTRY,
	GTT_TYPE_PPGTT_ROOT_L4_ENTRY,

	GTT_TYPE_PPGTT_ENTRY,

	GTT_TYPE_PPGTT_PTE_PT,
	GTT_TYPE_PPGTT_PDE_PT,
	GTT_TYPE_PPGTT_PDP_PT,
	GTT_TYPE_PPGTT_PML4_PT,

	GTT_TYPE_MAX,
}gtt_type_t;

typedef struct {
	union {
		u32 val32[2];
		u64 val64;
	};
	gtt_type_t type;
	struct pgt_device *pdev;
}gtt_entry_t;

struct vgt_gtt_pte_ops {
	gtt_entry_t *(*get_entry)(void *pt, gtt_entry_t *e, unsigned long index,
			bool hypervisor_access, struct vgt_device *vgt);
	gtt_entry_t *(*set_entry)(void *pt, gtt_entry_t *e, unsigned long index,
			bool hypervisor_access, struct vgt_device *vgt);
	bool (*test_present)(gtt_entry_t *e);
	void (*clear_present)(gtt_entry_t *e);
	bool (*test_pse)(gtt_entry_t *e);
	void (*set_pfn)(gtt_entry_t *e, unsigned long pfn);
	unsigned long (*get_pfn)(gtt_entry_t *e);
};

struct vgt_gtt_gma_ops {
	unsigned long (*gma_to_ggtt_pte_index)(unsigned long gma);
	unsigned long (*gma_to_pte_index)(unsigned long gma);
	unsigned long (*gma_to_pde_index)(unsigned long gma);
	unsigned long (*gma_to_l3_pdp_index)(unsigned long gma);
	unsigned long (*gma_to_l4_pdp_index)(unsigned long gma);
	unsigned long (*gma_to_pml4_index)(unsigned long gma);
};

struct vgt_gtt_info {
	struct vgt_gtt_pte_ops *pte_ops;
	struct vgt_gtt_gma_ops *gma_ops;
	bool (*mm_alloc_page_table)(struct vgt_mm *mm);
	void (*mm_free_page_table)(struct vgt_mm *mm);
	mempool_t *mempool;
	struct mutex mempool_lock;
	struct list_head oos_page_use_list_head;
	struct list_head oos_page_free_list_head;
};

typedef struct {
	void *vaddr;
	struct page *page;
	gtt_type_t type;
	struct hlist_node node;
	unsigned long mfn;
}shadow_page_t;

typedef enum {
	VGT_MM_GGTT = 0,
	VGT_MM_PPGTT,
} vgt_mm_type_t;

struct vgt_mm {
	vgt_mm_type_t type;
	bool initialized;
	bool shadowed;

	gtt_type_t page_table_entry_type;
	u32 page_table_entry_size;
	u32 page_table_entry_cnt;
	void *virtual_page_table;
	void *shadow_page_table;

	int page_table_level;
	bool has_shadow_page_table;
	u32 pde_base_index;

	struct list_head list;
	atomic_t refcount;
	struct vgt_device *vgt;
};

extern struct vgt_gtt_pte_ops gen7_gtt_pte_ops;
extern struct vgt_gtt_pte_ops gen8_gtt_pte_ops;
extern struct vgt_gtt_gma_ops gen7_gtt_gma_ops;
extern struct vgt_gtt_gma_ops gen8_gtt_gma_ops;

extern gtt_entry_t *vgt_mm_get_entry(struct vgt_mm *mm,
                void *page_table, gtt_entry_t *e,
                unsigned long index);

extern gtt_entry_t *vgt_mm_set_entry(struct vgt_mm *mm,
                void *page_table, gtt_entry_t *e,
                unsigned long index);

#define ggtt_get_guest_entry(mm, e, index) \
	(mm->vgt->vm_id == 0) ? \
	vgt_mm_get_entry(mm, NULL, e, index) : \
	vgt_mm_get_entry(mm, mm->virtual_page_table, e, index)

#define ggtt_set_guest_entry(mm, e, index) \
	vgt_mm_set_entry(mm, mm->virtual_page_table, e, index)

#define ggtt_get_shadow_entry(mm, e, index) \
	vgt_mm_get_entry(mm, mm->shadow_page_table, e, index)

#define ggtt_set_shadow_entry(mm, e, index) \
	vgt_mm_set_entry(mm, mm->shadow_page_table, e, index)

#define ppgtt_get_guest_root_entry(mm, e, index) \
	vgt_mm_get_entry(mm, mm->virtual_page_table, e, index)

#define ppgtt_set_guest_root_entry(mm, e, index) \
	vgt_mm_set_entry(mm, mm->virtual_page_table, e, index)

#define ppgtt_get_shadow_root_entry(mm, e, index) \
	vgt_mm_get_entry(mm, mm->shadow_page_table, e, index)

#define ppgtt_set_shadow_root_entry(mm, e, index) \
	vgt_mm_set_entry(mm, mm->shadow_page_table, e, index)

extern struct vgt_mm *vgt_create_mm(struct vgt_device *vgt,
		vgt_mm_type_t mm_type, gtt_type_t page_table_entry_type,
		void *virtual_page_table, int page_table_level,
		u32 pde_base_index);
extern void vgt_destroy_mm(struct vgt_mm *mm);

extern bool gen7_mm_alloc_page_table(struct vgt_mm *mm);
extern void gen7_mm_free_page_table(struct vgt_mm *mm);
extern bool gen8_mm_alloc_page_table(struct vgt_mm *mm);
extern void gen8_mm_free_page_table(struct vgt_mm *mm);

struct guest_page;

struct vgt_vgtt_info {
	struct vgt_mm *ggtt_mm;
	unsigned long active_ppgtt_mm_bitmap;
	struct list_head mm_list_head;
	DECLARE_HASHTABLE(shadow_page_hash_table, VGT_HASH_BITS);
	DECLARE_HASHTABLE(guest_page_hash_table, VGT_HASH_BITS);
	DECLARE_HASHTABLE(el_ctx_hash_table, VGT_HASH_BITS);
	atomic_t n_write_protected_guest_page;
	struct list_head oos_page_list_head;
	int last_partial_ppgtt_access_index;
	gtt_entry_t last_partial_ppgtt_access_entry;
	struct guest_page *last_partial_ppgtt_access_gpt;
	bool warn_partial_ppgtt_access_once;
};

extern bool vgt_init_vgtt(struct vgt_device *vgt);
extern void vgt_clean_vgtt(struct vgt_device *vgt);

extern bool vgt_gtt_init(struct pgt_device *pdev);
extern void vgt_gtt_clean(struct pgt_device *pdev);

extern bool vgt_expand_shadow_page_mempool(struct pgt_device *pdev);

extern bool vgt_g2v_create_ppgtt_mm(struct vgt_device *vgt, int page_table_level);
extern bool vgt_g2v_destroy_ppgtt_mm(struct vgt_device *vgt, int page_table_level);

extern struct vgt_mm *gen8_find_ppgtt_mm(struct vgt_device *vgt,
                int page_table_level, void *root_entry);

extern bool ppgtt_check_partial_access(struct vgt_device *vgt);

typedef bool guest_page_handler_t(void *gp, uint64_t pa, void *p_data, int bytes);

struct oos_page;

struct guest_page {
	struct hlist_node node;
	int writeprotection;
	unsigned long gfn;
	void *vaddr;
	guest_page_handler_t *handler;
	void *data;
	unsigned long write_cnt;
	struct oos_page *oos_page;
};

typedef struct guest_page guest_page_t;

struct oos_page {
	guest_page_t *guest_page;
	struct list_head list;
	struct list_head vm_list;
	int id;
	unsigned char mem[GTT_PAGE_SIZE];
};
typedef struct oos_page oos_page_t;

typedef struct {
	shadow_page_t shadow_page;
	guest_page_t guest_page;
	gtt_type_t guest_page_type;
	atomic_t refcount;
	struct vgt_device *vgt;
} ppgtt_spt_t;

extern bool vgt_init_guest_page(struct vgt_device *vgt, guest_page_t *guest_page,
		unsigned long gfn, guest_page_handler_t handler, void *data);

extern void vgt_clean_guest_page(struct vgt_device *vgt, guest_page_t *guest_page);

extern bool vgt_set_guest_page_writeprotection(struct vgt_device *vgt,
		guest_page_t *guest_page);

extern bool vgt_clear_guest_page_writeprotection(struct vgt_device *vgt,
		guest_page_t *guest_page);

extern guest_page_t *vgt_find_guest_page(struct vgt_device *vgt, unsigned long gfn);

extern bool gen7_ppgtt_mm_setup(struct vgt_device *vgt, int ring_id);

extern bool ppgtt_sync_oos_pages(struct vgt_device *vgt);

extern void* vgt_gma_to_va(struct vgt_mm *mm, unsigned long gma);

extern void vgt_ppgtt_switch(struct vgt_device *vgt);

extern bool gtt_emulate_read(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes);

extern bool gtt_emulate_write(struct vgt_device *vgt, unsigned int off,
	void *p_data, unsigned int bytes);

#endif /* _VGT_GTT_H_ */
