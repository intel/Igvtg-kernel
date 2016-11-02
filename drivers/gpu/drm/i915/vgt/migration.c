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

/******************************************************************************
 *				README
 *          !!! IMPORTANT BASIC RULES FOR MIGRATION CODING !!!
 * 1. There is no global flag under vgt that indicate it is under migration
 *	This is to make sure all of the logic outside of this file is clean and
 *	run as before.
 *	E.g. migrate a gtt page table was inside of vgtt_info_ops, instead of
 *	modifying vgt_create_mm() function to support migration.
 * 2. Rules and Operations are defined cooperative together:
 *	struct vgt_device --> vgt_device_ops <--> vgt_device_rules[]
 *	struct vgt_vgtt_info_t --> vgtt_info_ops <--> vgtt_info_rules[]
 *	...
 * 3. All migration logic in operation function defined in:
 *	struct vgt_migration_operation_t
 *
 * Interfaces from GVT-g:
 * 1. /sys/kernel/vgt/vm#/state
 *	save/restore GVT-g state via this node
 * 2. /sys/kernel/vgt/vm#/start
 *	to stop/start a specific VM from render scheduling queue
 *	Example:
 *	echo "0" > /sys/kernel/vgt/vm#/start
 *	This will remove current VM from vGPU scheduling thus no workload from
 *	this VM will be submitted to HW
 *	echo "1" > /sys/kernel/vgt/vm#/start
 *	This will add back to vGPU scheduling
 * 3. /sys/kernel/vgt/vm#/schedule
 *	to get current VM scheduling status.
 *	Return:
 *	"ACTIVE": VM is current render owner
 *	"DEACTIVE": VM is not current render owner, however it is in queue
 *	"REMOVED": VM is removed from vGPU render scheduling
 * 4. /sys/kernel/vgt/vm#/dirty_bitmap
 *	To get vGPU dirty pages bitmap to userspace. Each bit in dirty_bitmap is
 *	one of gfn page in guest memory.
 *	Please refer to readme of logdirty.c
 *
 * Qemu for VGT internally do below steps during migration:
 * On Source host:
 * STEP1: echo "0" > /sys/kernel/vgt/vm<source>/start
 * STEP2: cat /sys/kernel/vgt/vm<source>/state > kernel.img
 *
 * On Target host:
 * STEP1: create vgt instance with same parameter as source
 * STEP2: cat kernel.img > /sys/kernel/vgt/vm<target>/state
 * STEP3: echo "1" > /sys/kernel/vgt/vm<target>/start
 *
 * kernel.mig file format:
 * 1. <file header>
 * 2. CRC
 * 3. <session1><session2>... <session-N>
 *
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

/* s - struct
 * m - member
 * t - type of obj (int, u64, void*, int*, list_head)
 * ops - operation override callback func
 * opt - operation tye: vgt_migration_type_t
 * img_s - imag_size
 */
#define MIGRATION_TAG(_s, _m, _t, _ops, _opt, _img_s) {	\
.mem.base	= NULL,					\
.mem.offset	= offsetof(_s, _m),			\
.mem.offset_end	= offsetof(_s, _m) + sizeof(_t),	\
.mem.size	= sizeof(_t),				\
.img.base = NULL,					\
.img.offset = INV,					\
.img.offset_end = INV,					\
.img.size = _img_s,					\
.type	= _opt,						\
.ops	= &(_ops),					\
.name	= "["#_s":"#_m"]\0"				\
}

#define MIGRATION_UNIT(s, m, t, ops) \
	MIGRATION_TAG(s, m, t, ops, VGT_MIGRATION_UNIT, sizeof(t))

#define MIGRATION_LIST(s, m, ops) \
	MIGRATION_TAG(s, m, struct list_head, ops, VGT_MIGRATION_LIST, INV)

#define MIGRATION_VARSIZE_BUF(s, m, ops) \
	MIGRATION_TAG(s, m, void*, ops, VGT_MIGRATION_VARSIZE_BUF, INV)

#define MIGRATION_FIXSIZE_BUF(s, m, img_sz, ops) \
	MIGRATION_TAG(s, m, void*, ops, VGT_MIGRATION_FIXSIZE_BUF, img_sz)

#define MIGRATION_END {	\
	{NULL, 0, 0, 0},	\
	{NULL, 0, 0, 0},	\
	VGT_MIGRATION_NONE,	\
	NULL,	\
	NULL	\
}

#define MIGRATION_BEGIN MIGRATION_END

#define INV (-1)

#define RULES_NUM(x) (sizeof(x)/sizeof(vgt_migration_obj_t))
#define FOR_EACH_OBJ(obj, rules) \
	for (obj = rules + 1; obj->type != VGT_MIGRATION_NONE; obj++)
#define FOR_EACH_OBJ_REVERSE(obj, rules) \
	for (obj = rules + RULES_NUM(rules) - 2; obj->type != VGT_MIGRATION_NONE; obj--)

#define VALID_BASE(obj)	 do {						\
	if (unlikely((obj) == NULL					\
		|| (obj)->mem.base == NULL \
		|| (obj)->img.base == NULL)) {	\
		vgt_err("obj was not initialized.\n");			\
		return INV;						\
	} } while (0)

static int default_save(const vgt_migration_obj_t *obj);
static int default_load(const vgt_migration_obj_t *obj);
static int image_header_save(const vgt_migration_obj_t *obj);
static int vggtt_load(const vgt_migration_obj_t *obj);
static int vggtt_save(const vgt_migration_obj_t *obj);
static int vports_load(const vgt_migration_obj_t *obj);
static int vReg_load(const vgt_migration_obj_t *obj);
static int vcfg_space_load(const vgt_migration_obj_t *obj);
static int vgt_device_pre_save(const vgt_migration_obj_t *obj);
static int vgt_device_pre_load(const vgt_migration_obj_t *obj);
static bool vgt_migration_init_obj(struct vgt_device *vgt);
static void vgt_migration_finit_obj(struct vgt_device *vgt);

/***********************************************
 * Internal Static Functions
 ***********************************************/
struct vgt_migration_operation_t default_ops = {
	.pre_copy = NULL,
	.pre_save = default_save,
	.pre_load = default_load,
	.post_load = NULL,
};

struct vgt_migration_operation_t vports_ops = {
	.pre_copy = NULL,
	.pre_save = default_save,
	.pre_load = vports_load,
	.post_load = NULL,
};

struct vgt_migration_operation_t vReg_ops = {
	.pre_copy = NULL,
	.pre_save = default_save,
	.pre_load = vReg_load,
	.post_load = NULL,
};

struct vgt_migration_operation_t vcfg_space_ops = {
	.pre_copy = NULL,
	.pre_save = default_save,
	.pre_load = vcfg_space_load,
	.post_load = NULL,
};

struct vgt_migration_operation_t vgtt_info_ops = {
	.pre_copy = NULL,
	.pre_save = vggtt_save,
	.pre_load = vggtt_load,
	.post_load = NULL,
};

struct vgt_migration_operation_t image_header_ops = {
	.pre_copy = NULL,
	.pre_save = image_header_save,
	.pre_load = default_load,
	.post_load = NULL,
};

struct vgt_migration_operation_t vgt_device_ops = {
	.pre_copy = NULL,
	.pre_save = vgt_device_pre_save,
	.pre_load = vgt_device_pre_load,
	.post_load = NULL,
};

static void migration_obj_print(const vgt_migration_obj_t *obj)
{
	vgt_info("OBJ DUMP %s\n", obj->name);
	vgt_info("|MEM base=0x%llx, offset=0x%x, offset_end=0x%x size=0x%x\n",
		(u64)obj->mem.base,
		obj->mem.offset,
		obj->mem.offset_end,
		obj->mem.size);
	vgt_info("|IMG base=0x%llx, offset=0x%x, offset_end=0x%x size=0x%x\n",
		(u64)obj->img.base,
		obj->img.offset,
		obj->img.offset_end,
		obj->img.size);
}

static int migration_search_for_magic_id(void *buf, int count)
{
	void *magic_id = buf + count - sizeof(u64);

	ASSERT((count%sizeof(u32)) == 0);
	ASSERT(count >= sizeof(u64));

	while ((*((u64 *)magic_id)) != (u64) MIGRATION_MAGIC_ID) {
		/* search with 32bit alignment */
		magic_id -= sizeof(u32);
		count -= sizeof(u32);
		if (count == 0)
			break;
	}

	return count; /* return entire size. return 0 if not find */
}

static inline void
update_mem_region_base(vgt_migration_obj_t *obj, void *base)
{
	obj->mem.base = base;
}

static inline void
update_image_region_base(vgt_migration_obj_t *obj, void *base)
{
	obj->img.base = base;
}

static inline void
update_image_region_start_pos(vgt_migration_obj_t *obj, int pos)
{
	obj->img.offset = pos;
}

static inline void
update_image_region_end_pos(vgt_migration_obj_t *obj, int pos)
{
	/* used for reverse obj operation */
	obj->img.offset_end = pos;
}

static void
update_image_region_size_and_pos(vgt_migration_obj_t *obj, int size)
{
	obj->img.size = size;
	if (obj->img.offset == INV && obj->img.offset_end == INV)
		return;

	if (obj->img.offset == INV)
		update_image_region_start_pos(obj, obj->img.offset_end - size);

	if (obj->img.offset_end == INV)
		update_image_region_end_pos(obj, obj->img.offset + size);
}

/* Handle migration tag for each member of struct vgt_device
 * Each member in struct vgt_device carry on certain vGPU state information.
 *
 * Rules are list of vgt_migration_obj_t objs
 * Each obj has its operation method, which write its data to buffer (mapping to
 * /sys/kernel/vgt/vm#/state)
 *
 * The reason we maintain a list of below table is:
 * 1. The order of how each member save/restroe to userspace files is crucial
 * 2. We can not directly copy vgt_device members from SourceVM to TargetVM
 * entirely.
 * Some members can be copied from SourceVM directly: e.g image_header_t
 * Some members must be re-generated based on original information on SourceVM:
 * e.g gtt save/restore
 * Some has to be freshly created for TargetVM: e.g max_gpfn
 * 3. Each member may have specific operations to save/restore.
 * E.g u32 members are different to a void* as below
 *
 * struct example_t {
 *   int* a;
 *   unsgined long b;
 * }
 *__________________  _________________   __________________
 *|x64 (Source)    |  |image region    |  |x64 (Target)    |
 *|________________|  |________________|  |________________|
 *|  a (8bytes)    |  | a (4096B)      |  |    a (8bytes)  |
 *|  pointing to   |  |   offset=0     |  | allocate a page|
 *|  4K page       |  |   size=4096B   |  | copy data here |
 *|----------------|  |     ...        |  |----------------|
 *|  b (8bytes)    |  |     ...        |  |    b (8bytes)  |
 *|                |  |----------------|  |                |
 *|                |  | b (8bytes)     |  |                |
 *|----------------|  |   offset=4096  |  |----------------|
 *                    |   size=8       |
 *                    |----------------|
 *
 * int *a points to a page, and the entire page is copied into image
 * region[0-4096].
 * Later in Target side, we allocate a page (4K size), and copy image[0-4096]
 * back, and let Target.a pointing to that new allocated page
 *
 */
static struct vgt_migration_obj_t vgt_device_rules[] = {
	/* save to image from top obj to the end
	 * load from image from end to top reversely.
	 */
	MIGRATION_BEGIN,
	MIGRATION_UNIT(struct vgt_device,
			sched_info, struct vgt_sched_info, default_ops),
	/* Restore Guest GTT required: 
	 * 1. cfg_space: GTT restore will use BAR1 address
	 * 2. vReg: GTT restore will read PVINFO
	 */
	MIGRATION_UNIT(struct vgt_device,
			ports, struct gt_port[I915_MAX_PORTS], vports_ops),
	MIGRATION_VARSIZE_BUF(struct vgt_device,
			gtt.ggtt_mm, vgtt_info_ops),
	MIGRATION_FIXSIZE_BUF(struct vgt_device,
			state.vReg, VGT_MMIO_SPACE_SZ, vReg_ops),
	MIGRATION_FIXSIZE_BUF(struct vgt_device,
			state.sReg, VGT_MMIO_SPACE_SZ, default_ops),
	MIGRATION_UNIT(struct vgt_device,
			rb, vgt_state_ring_t[MAX_ENGINES], default_ops),
	MIGRATION_UNIT(struct vgt_device,
			state.cfg_space, u8[VGT_CFG_SPACE_SZ], vcfg_space_ops),
	MIGRATION_FIXSIZE_BUF(struct vgt_device,
			state.opregion_va, VGT_OPREGION_SIZE, default_ops),

	/* Image header always put to last, till now we know actual data size */
	MIGRATION_TAG(struct vgt_device,
		image_header,
		vgt_image_header_t,
		image_header_ops,
		VGT_MIGRATION_UNIT,
		sizeof(vgt_image_header_t)
	),
	MIGRATION_END
};

static int vgt_device_pre_save(const vgt_migration_obj_t *obj)
{
	/*
	 * Suppose caller already prepared mem.base/img.base
	 * img.size is calculated runtime.
	 */
	vgt_migration_obj_t *node;
	int n_img_actual_saved = 0;

	FUNC_ENTER;
	VALID_BASE(obj);

	/* save struct vgt_device to image start from offset 0 */
	if (unlikely(obj->mem.offset != 0 || obj->img.offset != 0)) {
		vgt_err("offset must 0 at beginning.\n");
		return INV;
	}

	/* go by obj rules one by one */
	FOR_EACH_OBJ(node, vgt_device_rules) {
		int n_img = INV;

		/* obj will copy data to image file img.offset */
		update_image_region_start_pos(node, n_img_actual_saved);
		if (node->ops->pre_save == NULL) {
			n_img = 0;
		} else {
			n_img = node->ops->pre_save(node);
			if (n_img == INV) {
				/* Error occurred. colored as RED */
				vgt_err("Save obj %s failed\n",
						node->name);
				migration_obj_print(node);
				n_img_actual_saved = INV;
				break;
			}
		}
		/* show GREEN on screen with colorred term */
		vgt_info("Save obj %s success with %d bytes\n",
			       node->name, n_img);
		update_image_region_size_and_pos(node, n_img);
		n_img_actual_saved += n_img;

		if (n_img_actual_saved >= MIGRATION_IMG_MAX_SIZE) {
			vgt_err("Image size overflow!!! data=%d MAX=%ld\n",
				n_img_actual_saved,
				MIGRATION_IMG_MAX_SIZE);
			/* Mark as invalid */
			n_img_actual_saved = INV;
			break;
		}
	}

	FUNC_EXIT(n_img_actual_saved);

	return n_img_actual_saved;
}

static int vgt_device_pre_load(const vgt_migration_obj_t *obj)
{
	vgt_migration_obj_t *node;
	int n_img_actual_recv = 0;
	u32 n_img_actual_size;

	FUNC_ENTER;
	VALID_BASE(obj);

	/* save struct vgt_device to image start from offset 0 */
	if (unlikely(obj->img.offset != 0 || obj->mem.offset != 0))
		return INV;

	n_img_actual_size = migration_search_for_magic_id(obj->img.base,
		obj->img.size);
	if (n_img_actual_size <= 0
	 || n_img_actual_size >= MIGRATION_IMG_MAX_SIZE) {
		vgt_err("Invalid image. magic_id offset = 0x%x\n",
				n_img_actual_size);
		return INV;
	}

	/* go by obj rules reverse order one by one */
	FOR_EACH_OBJ_REVERSE(node, vgt_device_rules) {
		int n_img = INV;

		/* you do not know what is the size of obj img size
		 * until pre_load callled.
		 */
		update_image_region_end_pos(node,
			n_img_actual_size - n_img_actual_recv);
		if (node->ops->pre_load == NULL) {
			n_img = 0;
		} else {
			n_img = node->ops->pre_load(node);
			if (n_img == INV) {
				/* Error occurred. colored as RED */
				vgt_info("Load obj %s failed\n",
						node->name);
				migration_obj_print(node);
				n_img_actual_recv = INV;
				break;
			}
		}
		/* show GREEN on screen with colorred term */
		vgt_info("Load obj %s success with %d bytes.\n",
			       node->name, n_img);
		update_image_region_size_and_pos(node, n_img);
		n_img_actual_recv += n_img;
	}

	FUNC_EXIT(n_img_actual_recv);

	return n_img_actual_recv;
}

static inline void*
obj_mem_offset(const vgt_migration_obj_t *obj)
{
	void *p = NULL;

	if (obj->type == VGT_MIGRATION_UNIT) {
		p = obj->mem.base + obj->mem.offset;
	} else if (obj->type == VGT_MIGRATION_FIXSIZE_BUF
	|| obj->type == VGT_MIGRATION_VARSIZE_BUF) {
		p = *((void **)(obj->mem.base + obj->mem.offset));
	} else {
		vgt_err("migration object default operation only \
				applies for type of unit and fixsize\n");
		ASSERT(0);
	}

	return p;
}

static inline void*
obj_img_offset(const vgt_migration_obj_t *obj)
{
	void *p = NULL;

	if (unlikely((obj->img.offset == INV) && (obj->img.offset_end == INV)))
		/* should not happen, unless obj was not initialized*/
		ASSERT(0);

	if (obj->img.offset == INV)
		p = obj->img.base + obj->img.offset_end - obj->img.size;
	else
		p = obj->img.base + obj->img.offset;
	return p;
}

static int default_save(const vgt_migration_obj_t *obj)
{
	void *src = obj_mem_offset(obj);
	void *dest = obj_img_offset(obj);
	int n_transfer = INV;

	FUNC_ENTER;
	VALID_BASE(obj);

	if (unlikely((obj->type == VGT_MIGRATION_UNIT)
				&& (obj->mem.size != obj->img.size))) {
		/* It must be type of VGT_MIGRATION_UNIT, img.size and
		 * mem.size are generated from build time.i
		 */
		vgt_err("migration object size is not match between source \
				and image!!! memsize=%d imgsize=%d\n",
		obj->mem.size,
		obj->img.size);
		ASSERT(0);
	} else {
		n_transfer = obj->img.size;
		memcpy(dest, src, n_transfer);
	}

	FUNC_EXIT(n_transfer);
	return n_transfer;
}

static int default_load(const vgt_migration_obj_t *obj)
{
	void *src = obj_img_offset(obj);
	void *dest = obj_mem_offset(obj);
	int n_transfer = INV;

	FUNC_ENTER;
	VALID_BASE(obj);

	if (unlikely((obj->type == VGT_MIGRATION_UNIT)
				&& (obj->mem.size != obj->img.size))) {
		/* It must be type of VGT_MIGRATION_UNIT, img.size and
		 * mem.size are generated from build time.
		 */
		vgt_err("migration object size is not match between target \
				and image!!! memsize=%d imgsize=%d\n",
		obj->mem.size,
		obj->img.size);
		ASSERT(0);
	} else {
		n_transfer = obj->img.size;
		memcpy(dest, src, n_transfer);
	}

	FUNC_EXIT(n_transfer);
	return n_transfer;
}

#define MIG_VREG_RESTORE(reg)						\
	{								\
		u32 data = __vreg(vgt, (reg));				\
		u64 pa = vgt_mmio_offset_to_pa(vgt, (reg));		\
		if (reg_mode_ctl(pdev, (reg))) {			\
			data |= 0xFFFF0000;				\
		}							\
		vgt_emulate_write(vgt, pa, &data, 4);			\
	}

static int vports_load(const vgt_migration_obj_t *obj)
{
	struct vgt_device *vgt = (struct vgt_device *) obj->mem.base;
	struct pgt_device *pdev = vgt->pdev;
	int n_transfer = INV;

	FUNC_ENTER;
	VALID_BASE(obj);

	/* !!! suppose we already restored vReg before vport restore !!!
	 * TODO: we actually do not read any source data currently.
	 * Ideally, get source port override mapping and rebuild in target.
	 * Here we just rebuild in target side, ignore source mapping.
	 */
	vgt_check_and_fix_port_mapping(vgt);

	/* Restore DDI registers */
	MIG_VREG_RESTORE(TRANS_DDI_FUNC_CTL_A);
	MIG_VREG_RESTORE(TRANS_DDI_FUNC_CTL_B);
	MIG_VREG_RESTORE(TRANS_DDI_FUNC_CTL_C);
	MIG_VREG_RESTORE(TRANS_DDI_FUNC_CTL_EDP);

	n_transfer = obj->img.size;

	FUNC_EXIT(n_transfer);
	return n_transfer;
}

static int vReg_load(const vgt_migration_obj_t *obj)
{
	struct vgt_device *vgt = (struct vgt_device *) obj->mem.base;
	struct pgt_device *pdev = vgt->pdev;
	int n_transfer = INV;

	FUNC_ENTER;
	VALID_BASE(obj);

	/* calling buf load to copy Source vReg to Target entirely */
	n_transfer = default_load(obj);

	/*********************************************************************
	 *  !!!!! Be careful of any code change in below !!!!!
	 *
	 * Following code are manually added after screening hundrens vreg one
	 * by one.
	 * It took all most 4weeks to make it works for Windows guest and it is
	 *  very platform specific code.
	 *
	 * Any code change in below need sufficient testing with 3D benchmark:
	 * Heaven, 3Dmark06, Tropical ...etc
	 *
	 ********************************************************************/

	/*Restore Engine mode registers */
	MIG_VREG_RESTORE(GFX_MODE_GEN7);
	MIG_VREG_RESTORE(_REG_VCS_MFX_MODE_IVB);
	MIG_VREG_RESTORE(_REG_BCS_BLT_MODE_IVB);
	MIG_VREG_RESTORE(_REG_VEBOX_MODE);

	if ((IS_BDWGT3(pdev) || IS_SKLGT3(pdev) || IS_SKLGT4(pdev)))
		MIG_VREG_RESTORE(_REG_VCS2_MFX_MODE_BDW);

	/*Restore Display pipe_conf regsiters */
	MIG_VREG_RESTORE(_PIPEACONF);
	MIG_VREG_RESTORE(_REG_PIPEBCONF);
	MIG_VREG_RESTORE(_REG_PIPECCONF);
	MIG_VREG_RESTORE(_REG_PIPE_EDP_CONF);

	/*Restore PAT index */
	MIG_VREG_RESTORE(GEN8_PRIVATE_PAT_LO);
	MIG_VREG_RESTORE(GEN8_PRIVATE_PAT_HI);

	/*Restore snoop control reg*/
	MIG_VREG_RESTORE(GEN6_MBCUNIT_SNPCR);
	MIG_VREG_RESTORE(GEN7_MISCCPCTL);

	/* !!! */
	spin_lock(&pdev->lock);

	/*Recalculate IMR/IER for all engines */

	/* we still need ringbuffer mode ISR/IMR to be reprogrammed
	 * since Windows Guest will submit init_content before execlist mode
	 */
	recalculate_and_update_imr(pdev, IMR);
	recalculate_and_update_imr(pdev, _REG_BCS_IMR);
	recalculate_and_update_imr(pdev, _REG_VCS_IMR);
	recalculate_and_update_imr(pdev, _REG_VECS_IMR);
	recalculate_and_update_imr(pdev, _REG_VCS2_IMR);

	recalculate_and_update_imr(pdev, _REG_GT_IMR(0));
	recalculate_and_update_imr(pdev, _REG_GT_IMR(1));
	recalculate_and_update_imr(pdev, _REG_GT_IMR(2));
	recalculate_and_update_imr(pdev, _REG_GT_IMR(3));

	recalculate_and_update_ier(pdev, _REG_GT_IER(0));
	recalculate_and_update_ier(pdev, _REG_GT_IER(1));
	recalculate_and_update_ier(pdev, _REG_GT_IER(2));
	recalculate_and_update_ier(pdev, _REG_GT_IER(3));

	recalculate_and_update_imr(pdev, _REG_DE_PIPE_IMR(PIPE_A));
	recalculate_and_update_imr(pdev, _REG_DE_PIPE_IMR(PIPE_B));
	recalculate_and_update_imr(pdev, _REG_DE_PIPE_IMR(PIPE_C));

	recalculate_and_update_ier(pdev, _REG_DE_PIPE_IER(PIPE_A));
	recalculate_and_update_ier(pdev, _REG_DE_PIPE_IER(PIPE_B));
	recalculate_and_update_ier(pdev, _REG_DE_PIPE_IER(PIPE_C));

	recalculate_and_update_imr(pdev, GEN8_DE_PORT_IMR);
	recalculate_and_update_ier(pdev, GEN8_DE_PORT_IER);

	recalculate_and_update_imr(pdev, GEN8_DE_MISC_IMR);
	recalculate_and_update_ier(pdev, GEN8_DE_MISC_IER);

	recalculate_and_update_imr(pdev, GEN8_PCU_IMR);
	recalculate_and_update_ier(pdev, GEN8_PCU_IER);

	/* Restore DE_RRMR */
	recalculate_and_update_imr(pdev, _REG_DE_RRMR);

	spin_unlock(&pdev->lock);

	FUNC_EXIT(n_transfer);
	return n_transfer;
}

static int vcfg_space_load(const vgt_migration_obj_t *obj)
{
	struct vgt_device *vgt = (struct vgt_device *) obj->mem.base;
	u8 *cfg_space = obj_mem_offset(obj);
	u8 *src_cfg_space = obj_img_offset(obj);
	int n_transfer = obj->img.size;

	FUNC_ENTER;
	VALID_BASE(obj);

	/* Restore Guest aperture/hidden GM before config PCI Bar1 */
	__vreg(vgt, vgt_info_off(avail_rs.mappable_gmadr.base)) =
		vgt->image_header.global_data[0];
	__vreg(vgt, vgt_info_off(avail_rs.nonmappable_gmadr.base)) =
		vgt->image_header.global_data[1];

	/* override target VM's cfg space */
	memcpy(cfg_space, src_cfg_space, n_transfer);

#define MIG_CFG_SPACE_WRITE(off) {					\
	u32 data;							\
	data = *((u32 *)(src_cfg_space + (off)));			\
	vgt_emulate_cfg_write(vgt, (off), &data, sizeof(data));		\
	}

#define MIG_CFG_SPACE_WRITE_BAR(bar) {					\
	u32 data = 0xFFFFFFFF;						\
	vgt_emulate_cfg_write(vgt, (bar), &data, sizeof(data));		\
	data = *((u32 *)(src_cfg_space + (bar)));			\
	vgt_emulate_cfg_write(vgt, (bar), &data, sizeof(data));		\
	data = 0xFFFFFFFF;						\
	vgt_emulate_cfg_write(vgt, (bar)+4, &data, sizeof(data));	\
	data = *((u32 *)(src_cfg_space + (bar)+4));			\
	vgt_emulate_cfg_write(vgt, (bar)+4, &data, sizeof(data));	\
	}

	/* reconfig bar0,1,2 with source VM's base address.
	 * TargetVM and SourceVM must have same bar base.
	 */
	MIG_CFG_SPACE_WRITE_BAR(VGT_REG_CFG_SPACE_BAR0);
	MIG_CFG_SPACE_WRITE_BAR(VGT_REG_CFG_SPACE_BAR1);
	MIG_CFG_SPACE_WRITE_BAR(VGT_REG_CFG_SPACE_BAR2);

	/* restore OpRegion */
	MIG_CFG_SPACE_WRITE(VGT_REG_CFG_OPREGION);
	MIG_CFG_SPACE_WRITE(VGT_REG_CFG_SWSCI_TRIGGER);

	FUNC_EXIT(n_transfer);
	return n_transfer;
}

static int image_header_save(const vgt_migration_obj_t *obj)
{
	int ret = INV;
	struct vgt_device *vgt = (struct vgt_device *) obj->mem.base;
	vgt_image_header_t *header = (vgt_image_header_t *) obj_mem_offset(obj);

	FUNC_ENTER;

	header->version = 0;
	header->data_size = obj->img.offset + sizeof(vgt_image_header_t);
	header->magic_id = (u64) MIGRATION_MAGIC_ID;
	header->crc_check = 0; /* CRC check skipped for now*/

	/* get SourceVM's guest aperture/hidden offset*/
	header->global_data[0] = vgt_guest_visible_gm_base(vgt);
	header->global_data[1] = vgt_guest_hidden_gm_base(vgt);

	ret = default_save(obj);
	FUNC_EXIT(ret);

	return ret;
}

static int
mig_ggtt_save_restore(struct vgt_mm *ggtt_mm,
		void *data, u64 gm_offset,
		u64 gm_sz,
		bool save_to_image)
{
	struct vgt_device *vgt = ggtt_mm->vgt;
	struct vgt_gtt_gma_ops *gma_ops = vgt->pdev->gtt.gma_ops;

	void *ptable;
	int sz;
	int shift = vgt->pdev->device_info.gtt_entry_size_shift;

	ptable = ggtt_mm->virtual_page_table +
	    (gma_ops->gma_to_ggtt_pte_index(gm_offset) << shift);
	sz = (gm_sz >> GTT_PAGE_SHIFT) << shift;

	if (save_to_image)
		memcpy(data, ptable, sz);
	else
		memcpy(ptable, data, sz);

	return sz;
}

static int vggtt_save(const vgt_migration_obj_t *obj)
{
	int ret = INV;
	struct vgt_device *vgt = (struct vgt_device *) obj->mem.base;
	struct vgt_mm *ggtt_mm = (struct vgt_mm *) obj_mem_offset(obj);

	u64 aperture_offset = vgt_guest_visible_gm_base(vgt);
	u64 aperture_sz = vgt_aperture_sz(vgt);
	u64 hidden_gm_offset = vgt_guest_hidden_gm_base(vgt);
	u64 hidden_gm_sz = vgt_hidden_gm_sz(vgt);

	vgt_migration_obj_buf_t obuf;
	void *dest;
	int sz;

	/*TODO:512MB GTT takes total 1024KB page table size, optimization here*/
	FUNC_ENTER;

	vgt_info("Guest aperture=0x%llx (HW: 0x%llx) Guest Hidden=0x%llx (HW:0x%llx)\n",
		aperture_offset, vgt_aperture_offset(vgt),
		hidden_gm_offset, vgt_hidden_gm_offset(vgt));

	/*TODO:to be fixed after removal of address ballooning */
	ret = 0;
	dest = obj->img.base + obj->img.offset;

	/* aperture */
	sz = mig_ggtt_save_restore(ggtt_mm, dest,
		aperture_offset, aperture_sz, true);
	dest += sz;
	ret += sz;

	/* hidden gm */
	sz = mig_ggtt_save_restore(ggtt_mm, dest,
		hidden_gm_offset, hidden_gm_sz, true);
	dest += sz;
	ret += sz;

	/* Save the total size of this session,
	 * thus load func knows where is the beginning.
	 */
	obuf.buf_size = ret;
	memcpy(dest, (void *)&obuf, sizeof(obuf));

	ret = obuf.buf_size + sizeof(obuf);

	FUNC_EXIT(ret);
	return ret;
}

static int vggtt_load(const vgt_migration_obj_t *obj)
{
	int ret;
	int ggtt_index;
	void *src;
	int sz;
	vgt_migration_obj_buf_t obuf;

	struct vgt_device *vgt = (struct vgt_device *) obj->mem.base;
	struct vgt_mm *ggtt_mm = (struct vgt_mm *) obj_mem_offset(obj);

	int shift = vgt->pdev->device_info.gtt_entry_size_shift;

	/* offset to bar1 beginning */
	u64 dest_aperture_offset = vgt_guest_visible_gm_base(vgt);
	u64 aperture_sz = vgt_aperture_sz(vgt);
	u64 dest_hidden_gm_offset = vgt_guest_hidden_gm_base(vgt);
	u64 hidden_gm_sz = vgt_hidden_gm_sz(vgt);

	FUNC_ENTER;

	vgt_info("Guest aperture=0x%llx (HW: 0x%llx) Guest Hidden=0x%llx (HW:0x%llx)\n",
		dest_aperture_offset, vgt_aperture_offset(vgt),
		dest_hidden_gm_offset, vgt_hidden_gm_offset(vgt));

	ASSERT(ggtt_mm == vgt->gtt.ggtt_mm);

	memcpy((void *)&obuf, obj->img.base
			+ obj->img.offset_end - sizeof(obuf),
		sizeof(obuf));

	if ((obuf.buf_size>>shift) !=
			((aperture_sz + hidden_gm_sz) >> GTT_PAGE_SHIFT)) {
		vgt_err("ggtt restore failed due to page table size not match\n");
		return INV;
	}

	ret = 0;
	src = obj->img.base + obj->img.offset_end\
		  - sizeof(obuf) - obuf.buf_size;

	/* aperture */
	sz = mig_ggtt_save_restore(ggtt_mm,\
		src, dest_aperture_offset, aperture_sz, false);
	src += sz;
	ret += sz;

	/* hidden GM */
	sz = mig_ggtt_save_restore(ggtt_mm, src,
			dest_hidden_gm_offset, hidden_gm_sz, false);
	ret += sz;
	ret += sizeof(obuf);

	/* aperture/hidden GTT emulation from Source to Target */
	for (ggtt_index = 0; ggtt_index < ggtt_mm->page_table_entry_cnt;
			ggtt_index++) {

		if (g_gm_is_valid(vgt, ggtt_index<<GTT_PAGE_SHIFT)) {
			struct vgt_gtt_pte_ops *ops = vgt->pdev->gtt.pte_ops;
			gtt_entry_t e;
			u64 offset;
			u64 pa;

			/* TODO: hardcode to 64bit right now */
			offset = vgt->pdev->device_info.gtt_start_offset
				+ (ggtt_index<<shift);

			pa = vgt_mmio_offset_to_pa(vgt, offset);

			/* read out virtual GTT entity and
			 * trigger emulate write
			 */
			ggtt_get_guest_entry(ggtt_mm, &e, ggtt_index);
			if (ops->test_present(&e)) {
			/* same as gtt_emulate
			 * _write(vgt, offset, &e.val64, 1<<shift);
			 * Using vgt_emulate_write as to align with vReg load
			 */
				vgt_emulate_write(vgt, pa, &e.val64, 1<<shift);
			}
		}
	}

	FUNC_EXIT(ret);
	return ret;
}

static bool vgt_migration_init_obj(struct vgt_device *vgt)
{
	vgt_migration_obj_t *obj = &vgt->migration_obj;
	vgt_migration_obj_t *node;

	if (obj->img.base == NULL) {
		obj->img.base = vzalloc(MIGRATION_IMG_MAX_SIZE);
		if (obj->img.base == NULL) {
			vgt_err("Unable to allocate size: %ld\n",
					MIGRATION_IMG_MAX_SIZE);
			return false;
		}
	}

	obj->img.offset = 0;
	obj->img.offset_end = MIGRATION_IMG_MAX_SIZE;
	obj->img.size = MIGRATION_IMG_MAX_SIZE;

	obj->mem.base = (void *)vgt;
	obj->mem.offset = 0;
	obj->mem.size = sizeof(struct vgt_device);
	obj->type = VGT_MIGRATION_NONE;
	obj->ops = &vgt_device_ops;

	/* initial each obj(rules) */
	FOR_EACH_OBJ(node, vgt_device_rules) {
		update_mem_region_base(node, obj->mem.base);
		update_image_region_base(node, obj->img.base);
		update_image_region_start_pos(node, INV);
		update_image_region_end_pos(node, INV);
	}

	return true;
}

static void vgt_migration_finit_obj(struct vgt_device *vgt)
{
	vgt_migration_obj_t *obj = &vgt->migration_obj;

	if (obj->img.base != NULL) {
		vfree(obj->img.base);
		obj->img.base = NULL;
	}

	obj->mem.base = NULL;
}

/***********************************************
 * External Function for struct vgt_device
 ***********************************************/
void *vgt_migration_acquire_snapshot(struct vgt_device *vgt)
{
	if (vgt_migration_init_obj(vgt))
		return vgt->migration_obj.img.base;
	else
		return NULL;
}

void vgt_migration_release_snapshot(struct vgt_device *vgt)
{
	vgt_migration_finit_obj(vgt);
}

int vgt_migration_save_snapshot(struct vgt_device *vgt)
{
	int size;

	if (current_render_owner(vgt->pdev) == vgt)
		return INV;

	if (current_foreground_vm(vgt->pdev) == vgt)
		return INV;

	if (!vgt_migration_init_obj(vgt))
		return INV;

	/* called every time when we read to get updated data */
	size = vgt->migration_obj.ops->pre_save(&vgt->migration_obj);
	if (size == INV) {
		vgt_err("Failed to create snapshot\n");
		vgt_migration_finit_obj(vgt);
		return INV;
	}
	vgt_info("------- IMAGE INFO --------\n");
	vgt_info("vgt-%d: [%d] bytes saved to file\n", vgt->vm_id, size);
	vgt_info("version=0x%x, data_size=%d magic=0x%llx\n\n",
		vgt->image_header.version,
		vgt->image_header.data_size,
		vgt->image_header.magic_id);

	return size;
}

int vgt_migration_load_snapshot(struct vgt_device *vgt)
{
	int size;

	if (current_render_owner(vgt->pdev) == vgt)
		return INV;

	if (current_foreground_vm(vgt->pdev) == vgt)
		return INV;

	if (!vgt_migration_init_obj(vgt))
		return INV;

	/* load snapshot completed */
	size = vgt->migration_obj.ops->pre_load(&vgt->migration_obj);
	if (size == INV) {
		vgt_err("Failed to load snapshot\n");
		return INV;
	}

	vgt_info("------- IMAGE INFO --------\n");
	vgt_info("vgt-%d: [%d] bytes loaded\n", vgt->vm_id, size);
	vgt_info("version=0x%x, data_size=%d magic=0x%llx\n\n",
		vgt->image_header.version,
		vgt->image_header.data_size,
		vgt->image_header.magic_id);

	return size;
}
