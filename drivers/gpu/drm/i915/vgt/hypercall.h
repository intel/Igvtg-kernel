/*
 * Interface abstraction for hypervisor services
 *
 * Copyright(c) 2011-2015 Intel Corporation. All rights reserved.
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

#ifndef _VGT_HYPERCALL_H_
#define _VGT_HYPERCALL_H_

struct guest_page;
struct vgt_device;
enum map_type;
struct kernel_dm {
	const char *name;
	unsigned long (*g2m_pfn)(int vm_id, unsigned long g_pfn);
	int (*pause_domain)(int vm_id);
	int (*shutdown_domain)(int vm_id);
	int (*map_mfn_to_gpfn)(int vm_id, unsigned long gpfn,
		unsigned long mfn, int nr, int map, enum map_type type);
	int (*set_trap_area)(struct vgt_device *vgt, uint64_t start, uint64_t end, bool map);
	bool (*set_wp_pages)(struct vgt_device *vgt, struct guest_page *p);
	bool (*unset_wp_pages)(struct vgt_device *vgt, struct guest_page *p);
	int (*check_host)(void);
	int (*from_virt_to_mfn)(void *addr);
	void *(*from_mfn_to_virt)(int mfn);
	int (*inject_msi)(int vm_id, u32 addr, u16 data);
	int (*hvm_init)(struct vgt_device *vgt);
	void (*hvm_exit)(struct vgt_device *vgt);
	void *(*gpa_to_va)(struct vgt_device *vgt, unsigned long gap);
	bool (*read_va)(struct vgt_device *vgt, void *va, void *val, int len, int atomic);
	bool (*write_va)(struct vgt_device *vgt, void *va, void *val, int len, int atomic);
};

#endif /* _VGT_HYPERCALL_H_ */
