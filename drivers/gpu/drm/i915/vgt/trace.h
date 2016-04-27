/*
 * vGT ftrace header
 *
 * Copyright(c) 2011-2013 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of Version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#if !defined(_VGT_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _VGT_TRACE_H_

#include <linux/types.h>
#include <linux/stringify.h>
#include <linux/tracepoint.h>
#include <asm/tsc.h>

#undef TRACE_SYSTEM
#define TRACE_SYSTEM vgt
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace

TRACE_EVENT(vgt_mmio_rw,
		TP_PROTO(bool write, u32 vm_id, u32 offset, void *pd,
			int bytes),

		TP_ARGS(write, vm_id, offset, pd, bytes),

		TP_STRUCT__entry(
			__field(bool, write)
			__field(u32, vm_id)
			__field(u32, offset)
			__field(int, bytes)
			__field(u64, value)
			),

		TP_fast_assign(
			__entry->write = write;
			__entry->vm_id = vm_id;
			__entry->offset = offset;
			__entry->bytes = bytes;

			memset(&__entry->value, 0, sizeof(u64));
			memcpy(&__entry->value, pd, bytes);
		),

		TP_printk("VM%u %s offset 0x%x data 0x%llx byte %d\n",
				__entry->vm_id,
				__entry->write ? "write" : "read",
				__entry->offset,
				__entry->value,
				__entry->bytes)
);

#define MAX_CMD_STR_LEN	256
TRACE_EVENT(vgt_command,
		TP_PROTO(u8 vm_id, u8 ring_id, u32 ip_gma, u32 *cmd_va, u32 cmd_len, bool ring_buffer_cmd, cycles_t cost_pre_cmd_handler, cycles_t cost_cmd_handler),

		TP_ARGS(vm_id, ring_id, ip_gma, cmd_va, cmd_len, ring_buffer_cmd, cost_pre_cmd_handler, cost_cmd_handler),

		TP_STRUCT__entry(
			__field(u8, vm_id)
			__field(u8, ring_id)
			__field(int, i)
			__array(char,tmp_buf, MAX_CMD_STR_LEN)
			__array(char, cmd_str, MAX_CMD_STR_LEN)
			),

		TP_fast_assign(
			__entry->vm_id = vm_id;
			__entry->ring_id = ring_id;
			__entry->cmd_str[0] = '\0';
			snprintf(__entry->tmp_buf, MAX_CMD_STR_LEN, "VM(%d) Ring(%d): %s ip(%08x) pre handler cost (%llu), handler cost (%llu) ", vm_id, ring_id, ring_buffer_cmd ? "RB":"BB", ip_gma, cost_pre_cmd_handler, cost_cmd_handler);
			strcat(__entry->cmd_str, __entry->tmp_buf);
			entry->i = 0;
			while (cmd_len > 0) {
				if (cmd_len >= 8) {
					snprintf(__entry->tmp_buf, MAX_CMD_STR_LEN, "%08x %08x %08x %08x %08x %08x %08x %08x ",
						cmd_va[__entry->i], cmd_va[__entry->i+1], cmd_va[__entry->i+2], cmd_va[__entry->i+3],
						cmd_va[__entry->i+4],cmd_va[__entry->i+5],cmd_va[__entry->i+6],cmd_va[__entry->i+7]);
					__entry->i += 8;
					cmd_len -= 8;
					strcat(__entry->cmd_str, __entry->tmp_buf);
				} else if (cmd_len >= 4) {
					snprintf(__entry->tmp_buf, MAX_CMD_STR_LEN, "%08x %08x %08x %08x ",
						cmd_va[__entry->i], cmd_va[__entry->i+1], cmd_va[__entry->i+2], cmd_va[__entry->i+3]);
					__entry->i += 4;
					cmd_len -= 4;
					strcat(__entry->cmd_str, __entry->tmp_buf);
				} else if (cmd_len >= 2) {
					snprintf(__entry->tmp_buf, MAX_CMD_STR_LEN, "%08x %08x ", cmd_va[__entry->i], cmd_va[__entry->i+1]);
					__entry->i += 2;
					cmd_len -= 2;
					strcat(__entry->cmd_str, __entry->tmp_buf);
				} else if (cmd_len == 1) {
					snprintf(__entry->tmp_buf, MAX_CMD_STR_LEN, "%08x ", cmd_va[__entry->i]);
					__entry->i += 1;
					cmd_len -= 1;
					strcat(__entry->cmd_str, __entry->tmp_buf);
				}
			}
			strcat(__entry->cmd_str, "\n");
		),

		TP_printk("%s", __entry->cmd_str)
);

TRACE_EVENT(spt_alloc,
		TP_PROTO(int vm_id, void *spt, int type, unsigned long mfn, unsigned long gpt_gfn),

		TP_ARGS(vm_id, spt, type, mfn, gpt_gfn),

		TP_STRUCT__entry(
			__field(int, vm_id)
			__field(void *, spt)
			__field(int, type)
			__field(unsigned long, mfn)
			__field(unsigned long, gpt_gfn)
			),

		TP_fast_assign(
			__entry->vm_id = vm_id;
			__entry->spt = spt;
			__entry->type = type;
			__entry->mfn = mfn;
			__entry->gpt_gfn = gpt_gfn;
		),

		TP_printk("VM%d [alloc] spt %p type %d mfn 0x%lx gpt_gfn 0x%lx\n",
				__entry->vm_id,
				__entry->spt,
				__entry->type,
				__entry->mfn,
				__entry->gpt_gfn)
);

TRACE_EVENT(spt_free,
		TP_PROTO(int vm_id, void *spt, int type),

		TP_ARGS(vm_id, spt, type),

		TP_STRUCT__entry(
			__field(int, vm_id)
			__field(void *, spt)
			__field(int, type)
			),

		TP_fast_assign(
			__entry->vm_id = vm_id;
			__entry->spt = spt;
			__entry->type = type;
		),

		TP_printk("VM%u [free] spt %p type %d\n",
				__entry->vm_id,
				__entry->spt,
				__entry->type)
);

#define MAX_BUF_LEN 256

TRACE_EVENT(gma_index,
		TP_PROTO(const char *prefix, unsigned long gma, unsigned long index),

		TP_ARGS(prefix, gma, index),

		TP_STRUCT__entry(
			__array(char, buf, MAX_BUF_LEN)
		),

		TP_fast_assign(
			snprintf(__entry->buf, MAX_BUF_LEN, "%s gma 0x%lx index 0x%lx\n", prefix, gma, index);
		),

		TP_printk("%s", __entry->buf)
);

TRACE_EVENT(gma_translate,
		TP_PROTO(int vm_id, char *type, int ring_id, int pt_level, unsigned long gma, unsigned long gpa),

		TP_ARGS(vm_id, type, ring_id, pt_level, gma, gpa),

		TP_STRUCT__entry(
			__array(char, buf, MAX_BUF_LEN)
		),

		TP_fast_assign(
			snprintf(__entry->buf, MAX_BUF_LEN, "VM%d %s ring %d pt_level %d gma 0x%lx -> gpa 0x%lx\n",
					vm_id, type, ring_id, pt_level, gma, gpa);
		),

		TP_printk("%s", __entry->buf)
);

TRACE_EVENT(spt_refcount,
		TP_PROTO(int vm_id, char *action, void *spt, int before, int after),

		TP_ARGS(vm_id, action, spt, before, after),

		TP_STRUCT__entry(
			__array(char, buf, MAX_BUF_LEN)
		),

		TP_fast_assign(
			snprintf(__entry->buf, MAX_BUF_LEN, "VM%d [%s] spt %p before %d -> after %d\n",
					vm_id, action, spt, before, after);
		),

		TP_printk("%s", __entry->buf)
);

TRACE_EVENT(spt_change,
		TP_PROTO(int vm_id, char *action, void *spt, unsigned long gfn, int type),

		TP_ARGS(vm_id, action, spt, gfn, type),

		TP_STRUCT__entry(
			__array(char, buf, MAX_BUF_LEN)
		),

		TP_fast_assign(
			snprintf(__entry->buf, MAX_BUF_LEN, "VM%d [%s] spt %p gfn 0x%lx type %d\n",
					vm_id, action, spt, gfn, type);
		),

		TP_printk("%s", __entry->buf)
);

TRACE_EVENT(gpt_change,
		TP_PROTO(int vm_id, const char *tag, void *spt, int type, u64 v, unsigned long index),

		TP_ARGS(vm_id, tag, spt, type, v, index),

		TP_STRUCT__entry(
			__array(char, buf, MAX_BUF_LEN)
		),

		TP_fast_assign(
			snprintf(__entry->buf, MAX_BUF_LEN, "VM%d [%s] spt %p type %d entry 0x%llx index 0x%lx\n",
					vm_id, tag, spt, type, v, index);
		),

		TP_printk("%s", __entry->buf)
);

TRACE_EVENT(oos_change,
		TP_PROTO(int vm_id, const char *tag, int page_id, void *gpt, int type),

		TP_ARGS(vm_id, tag, page_id, gpt, type),

		TP_STRUCT__entry(
			__array(char, buf, MAX_BUF_LEN)
		),

		TP_fast_assign(
			snprintf(__entry->buf, MAX_BUF_LEN, "VM%d [oos %s] page id %d gpt %p type %d\n",
					vm_id, tag, page_id, gpt, type);
		),

		TP_printk("%s", __entry->buf)
);

TRACE_EVENT(oos_sync,
		TP_PROTO(int vm_id, int page_id, void *gpt, int type, u64 v, unsigned long index),

		TP_ARGS(vm_id, page_id, gpt, type, v, index),

		TP_STRUCT__entry(
			__array(char, buf, MAX_BUF_LEN)
		),

		TP_fast_assign(
			snprintf(__entry->buf, MAX_BUF_LEN, "VM%d [oos sync] page id %d gpt %p type %d entry 0x%llx index 0x%lx\n",
					vm_id, page_id, gpt, type, v, index);
		),

		TP_printk("%s", __entry->buf)
);

TRACE_EVENT(ctx_lifecycle,
		TP_PROTO(int vm_id, int ring_id,
				uint32_t guest_lrca, const char *action),

		TP_ARGS(vm_id, ring_id, guest_lrca, action),

		TP_STRUCT__entry(
			__array(char, buf, MAX_BUF_LEN)
		),

		TP_fast_assign(
			snprintf(__entry->buf, MAX_BUF_LEN,
				"VM-%d <ring-%d>: EXECLIST Context guest lrca 0x%x - %s\n",
				vm_id, ring_id, guest_lrca, action);
		),

		TP_printk("%s", __entry->buf)
);

TRACE_EVENT(ctx_protection,
		TP_PROTO(int vm_id, int ring_id, uint32_t guest_lrca,
			uint32_t page_idx, uint32_t gfn, const char *operation),

		TP_ARGS(vm_id, ring_id, guest_lrca, page_idx, gfn, operation),

		TP_STRUCT__entry(
			__array(char, buf, MAX_BUF_LEN)
		),

		TP_fast_assign(
			snprintf(__entry->buf, MAX_BUF_LEN,
				"VM-%d <ring-%d>: EXECLIST Context guest lrca[0x%x] "
				"page[%i] gfn[0x%x] - %s\n",
				vm_id, ring_id, guest_lrca, page_idx, gfn, operation);
		),

		TP_printk("%s", __entry->buf)
);

TRACE_EVENT(ctx_write_trap,
		TP_PROTO(uint32_t guest_lrca, uint32_t shadow_lrca,
			 uint64_t pa, int bytes, uint32_t val_32),

		TP_ARGS(guest_lrca, shadow_lrca, pa, bytes, val_32),

		TP_STRUCT__entry(
			__array(char, buf, MAX_BUF_LEN)
			),

		TP_fast_assign(
			snprintf(__entry->buf, MAX_BUF_LEN,
				 "EXECLIST Context write trapped: guest_lrca: "
				 "<0x%x>, shadow_lrca: <0x%x>, "
				 "addr: <0x%llx> idx[0x%x], bytes %i, val_32: <0x%x>\n",
				 guest_lrca, shadow_lrca, pa, (unsigned int)((pa & 0xfff) >> 2), bytes, val_32)
		),

		TP_printk("%s", __entry->buf)
);

TRACE_EVENT(shadow_rb_copy,
		TP_PROTO(int vm_id, int ring_id, uint32_t guest_lrca, uint32_t shadow_lrca, uint32_t size, unsigned long vbase, unsigned long sbase, uint32_t rb_head, uint32_t tail),

		TP_ARGS(vm_id, ring_id, guest_lrca, shadow_lrca, size, vbase, sbase, rb_head, tail),

		TP_STRUCT__entry(
			__array(char, buf, MAX_BUF_LEN)
		),

		TP_fast_assign(
			snprintf(__entry->buf, MAX_BUF_LEN,
				"VM-%d (ring <%d> ctx-0x%x(sctx-0x%x)): copy shadow ring. "
				"size:0x%x, base: 0x%lx(sbase:0x%lx), head: 0x%x, tail: 0x%x\n",
				vm_id, ring_id, guest_lrca, shadow_lrca,
				size, vbase, sbase, rb_head, tail);
		),

		TP_printk("%s", __entry->buf)
);

TRACE_EVENT(ctx_csb_emulate,
		TP_PROTO(int vm_id, int ring_id, int csb_entry, int tail, uint32_t udw, uint32_t ldw),

		TP_ARGS(vm_id, ring_id, csb_entry, tail, udw, ldw),

		TP_STRUCT__entry(
			__array(char, buf, MAX_BUF_LEN)
		),

		TP_fast_assign(
			snprintf(__entry->buf, MAX_BUF_LEN,"VM-%d <ring-%d>: Emulate CSB[%d](tail:%d): udw: 0x%x; ldw: 0x%x\n",
			vm_id, ring_id, csb_entry, tail, udw, ldw);
		),

		TP_printk("%s", __entry->buf)
);

TRACE_EVENT(shadow_bb_relocate,
		TP_PROTO(int vm_id, int ring_id, uint32_t guest_lrca, uint32_t source_gma, uint32_t value, uint32_t new_value, uint32_t bb_size),

		TP_ARGS(vm_id, ring_id, guest_lrca, source_gma, value, new_value, bb_size),

		TP_STRUCT__entry(
			__array(char, buf, MAX_BUF_LEN)
		),

		TP_fast_assign(
			snprintf(__entry->buf, MAX_BUF_LEN,
			"VM-%d(ring<%d>, lrca<0x%x>): Relocating source gma<0x%x> from <0x%x> to <0x%x> of size <0x%x>\n",
			vm_id, ring_id, guest_lrca, source_gma, value, new_value, bb_size);
		),

		TP_printk("%s", __entry->buf)
);
#endif /* _VGT_TRACE_H_ */

/* This part must be out of protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#include <trace/define_trace.h>
