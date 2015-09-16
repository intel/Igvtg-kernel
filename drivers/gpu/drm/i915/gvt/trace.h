/*
 * Copyright(c) 2011-2016 Intel Corporation. All rights reserved.
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

#if !defined(_GVT_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _GVT_TRACE_H_

#include <linux/types.h>
#include <linux/stringify.h>
#include <linux/tracepoint.h>
#include <asm/tsc.h>

#undef TRACE_SYSTEM
#define TRACE_SYSTEM gvt

#endif /* _GVT_TRACE_H_ */

/* This part must be out of protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace
#include <trace/define_trace.h>
