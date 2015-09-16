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

#ifndef _GVT_REG_H
#define _GVT_REG_H

#define GVT_CFG_SPACE_SZ	256
#define GVT_BAR_NUM		4

#define GVT_PCI_CLASS_VGA			0x03
#define GVT_PCI_CLASS_VGA_OTHER			0x80

#define GVT_REG_CFG_VENDOR_ID                   0x00
#define GVT_REG_CFG_COMMAND                     0x04
#define _REGBIT_CFG_COMMAND_IO                  (1 << 0)
#define _REGBIT_CFG_COMMAND_MEMORY              (1 << 1)
#define _REGBIT_CFG_COMMAND_MASTER              (1 << 2)
#define GVT_REG_CFG_CLASS_PROG_IF               0x09
#define GVT_REG_CFG_SUB_CLASS_CODE              0x0A
#define GVT_REG_CFG_CLASS_CODE                  0x0B
#define GVT_REG_CFG_SPACE_BAR0                  0x10
#define GVT_REG_CFG_SPACE_BAR1                  0x18
#define GVT_REG_CFG_SPACE_BAR2                  0x20
#define GVT_REG_CFG_SPACE_BAR_ROM               0x30
#define GVT_REG_CFG_SPACE_MSAC                  0x62
#define GVT_REG_CFG_SWSCI_TRIGGER               0xE8
#define _REGBIT_CFG_SWSCI_SCI_SELECT            (1 << 15)
#define _REGBIT_CFG_SWSCI_SCI_TRIGGER           1
#define GVT_REG_CFG_OPREGION                    0xFC

#define _REG_GMCH_CONTROL               0x50
#define    _REGBIT_BDW_GMCH_GMS_SHIFT   8
#define    _REGBIT_BDW_GMCH_GMS_MASK    0xff

#define _PCH_GMBUS2			0xc5108

#define _GEN6_GDRST			0x941c
#define _GEN6_GT_THREAD_STATUS_REG	0x13805c
#define _GEN6_GT_CORE_STATUS		0x138060

#endif
