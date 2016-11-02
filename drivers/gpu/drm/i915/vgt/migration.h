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

#ifndef _VGT_MIGRATION_H_
#define _VGT_MIGRATION_H_

/*
 * export node size: /sys/kernel/vgt/vm<target>/state
 * All vgt migration data read/write to this node
 */
/* Assume 9MB is eough to descript VM kernel state */
#define MIGRATION_IMG_MAX_SIZE (9*SIZE_1MB)
#define MIGRATION_MAGIC_ID \
((0xDEAD0BADll<<32) | ('_'<<24) | ('V'<<16) | ('G'<<8) | 'T')

typedef enum vgt_migration_type_t {
	/* Not to handle */
	VGT_MIGRATION_NONE,

	/* The sizeof(obj) is valid and static value
	 * the entire obj is copied into qemu img file.
	 */
	VGT_MIGRATION_UNIT,

	/* The obj is a buf, buf content is in source side, target required to
	 * allocate same size of buf, and copy from source.
	 */
	VGT_MIGRATION_VARSIZE_BUF,
	VGT_MIGRATION_FIXSIZE_BUF,
} vgt_migration_type_t;

typedef struct vgt_migration_obj_buf_t {
	u32 buf_size;
	u32 reserved; /* for 64bit alignment */
} vgt_migration_obj_buf_t;

typedef struct vgt_migration_obj_list_t {
	u32 element_num;
	u32 element_size;
} vgt_migration_obj_list_t;

typedef struct vgt_region_t {
	void *base;		/* obj read/write to (base + offset) */
	u32 offset;		/* offset to base. INVALID for pre_load */
	u32 offset_end;
	/* offset_end = offset + size. INVALID for pre_save */
	u32 size;		/* obj size of bytes to read/write */
} vgt_region_t;

struct vgt_migration_operation_t;

typedef struct vgt_migration_obj_t {
	vgt_region_t mem;		/* Source or Target region */
	vgt_region_t img;		/* snapshot image region */
	vgt_migration_type_t type;
	/* operation func defines how data transfer between mem<-->img region */
	struct vgt_migration_operation_t *ops;
	char *name;
} vgt_migration_obj_t;

typedef struct vgt_migration_operation_t {
	/* called during pre-copy stage, VM is still alive */
	int (*pre_copy)(const vgt_migration_obj_t *obj);
	/* called before when VM was paused,
	 * return bytes transferred
	 */
	int (*pre_save)(const vgt_migration_obj_t *obj);
	/* called before load the state of device,
	 * VM has not resume execution
	 */
	int (*pre_load)(const vgt_migration_obj_t *obj);
	/* called after load the state of device, VM already alive */
	int (*post_load)(const vgt_migration_obj_t *obj);
} vgt_migration_operation_t;

typedef struct vgt_image_header_t {
	int version;
	int data_size;
	u64 crc_check;
	u64 global_data[64];
	u64 magic_id;/* !!! please keep this at end of struct !!! */
} vgt_image_header_t;

/*
 * GVT Page modification logging support
 */
#define LOGD_SHA1_SIZE 20
#define BITS_TO_BYTES(bits) (((bits) + BITS_PER_BYTE - 1) / BITS_PER_BYTE)

/* create dirty_bitmap to cover 1GB guest memory at boot up
 * if we hit gpfn from guest > MAX_BITMAP_GPFN(vgt)
 * We will enlarge the dirty_bitmap size for additional INIT_BITMAP_GPFN
 */
#define INIT_BITMAP_GPFN ((1*1024*SIZE_1MB) >> PAGE_SHIFT)
#define MAX_BITMAP_GPFN(vgt) ((vgt)->logd.max_bitmap_gpfn)

typedef struct logd_tag_t {
	u8 sha1[LOGD_SHA1_SIZE];
} logd_tag_t;

/* each logd_slot_t cover 4M Guest memory range */
#define LOGD_SLOT_SIZE 1024
typedef struct logd_slot_t {
	/* 1024 bits to cover 4M gfn range */
	DECLARE_BITMAP(vgpu_bitmap, LOGD_SLOT_SIZE); /* gfn was vGPU referred */
	DECLARE_BITMAP(dirty_bitmap, LOGD_SLOT_SIZE); /* gfn is dirty*/
} logd_slot_t;

/* simple array instead of list_head for quick slot search */
typedef struct logd_slot_head_t {
	logd_slot_t *slot;
} logd_slot_head_t;

typedef struct logd_page_t {
	struct hlist_node node;
	unsigned long gfn;
	logd_tag_t tag;	/* tag change means dirty */
} logd_page_t;

typedef struct vgt_logd_t {
	/* max gpfn in dirty_map */
	unsigned long max_bitmap_gpfn;
	/* each logd_slot_t cover 4MB range */
	logd_slot_head_t *logd_slot_head;
	/* hash table to store all gfn vGPU refered */
	DECLARE_HASHTABLE(logd_page_hash_table, VGT_HASH_BITS);
	spinlock_t logd_lock;
} vgt_logd_t;

struct vgt_device;

void *vgt_migration_acquire_snapshot(struct vgt_device *vgt);
void vgt_migration_release_snapshot(struct vgt_device *vgt);
int vgt_migration_save_snapshot(struct vgt_device *vgt);
int vgt_migration_load_snapshot(struct vgt_device *vgt);
void vgt_logd_init(struct vgt_device *vgt);
void vgt_logd_finit(struct vgt_device *vgt);
void vgt_logd_add(struct vgt_device *vgt, unsigned long gfn);
void vgt_logd_remove(struct vgt_device *vgt, unsigned long gfn);
int vgt_logd_sync(struct vgt_device *vgt,
		unsigned long gfn_start, unsigned long npages);
int vgt_logd_read_log(struct vgt_device *vgt,
		char *buf, unsigned long off, unsigned long count);

#endif	/* _VGT_MIGRATION_H_ */
