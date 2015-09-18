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

#define GVT_OPREGION_PAGES                      2
#define GVT_OPREGION_PORDER                     1
#define GVT_OPREGION_SIZE                       (8 * 1024)
#define GVT_OPREGION_REG_CLID                   0x1AC
#define GVT_OPREGION_REG_SCIC                   0x200
#define _REGBIT_OPREGION_SCIC_FUNC_MASK         0x1E
#define _REGBIT_OPREGION_SCIC_FUNC_SHIFT        1
#define _REGBIT_OPREGION_SCIC_SUBFUNC_MASK      0xFF00
#define _REGBIT_OPREGION_SCIC_SUBFUNC_SHIFT     8
#define _REGBIT_OPREGION_SCIC_EXIT_MASK         0xE0
#define GVT_OPREGION_SCIC_F_GETBIOSDATA         4
#define GVT_OPREGION_SCIC_F_GETBIOSCALLBACKS    6
#define GVT_OPREGION_SCIC_SF_SUPPRTEDCALLS      0
#define GVT_OPREGION_SCIC_SF_REQEUSTEDCALLBACKS 1
#define GVT_OPREGION_REG_PARM			0x204

#define _REG_GMCH_CONTROL               0x50
#define    _REGBIT_BDW_GMCH_GMS_SHIFT   8
#define    _REGBIT_BDW_GMCH_GMS_MASK    0xff

#define _PCH_GMBUS2			0xc5108

#define _GEN6_GDRST			0x941c
#define _GEN6_GT_THREAD_STATUS_REG	0x13805c
#define _GEN6_GT_CORE_STATUS		0x138060

#define _GEN8_DE_PORT_IMR (0x44444)
#define _GEN8_DE_PORT_IER (0x4444c)
#define _GEN8_DE_PORT_IIR (0x44448)
#define _GEN8_DE_PORT_ISR (0x44440)

#define _GEN8_DE_MISC_IMR (0x44464)
#define _GEN8_DE_MISC_IER (0x4446c)
#define _GEN8_DE_MISC_IIR (0x44468)
#define _GEN8_DE_MISC_ISR (0x44460)

#define _GEN8_PCU_IMR (0x444e4)
#define _GEN8_PCU_IER (0x444ec)
#define _GEN8_PCU_IIR (0x444e8)
#define _GEN8_PCU_ISR (0x444e0)
#define _GEN8_MASTER_IRQ			(0x44200)

#define _SDEIMR  (0xc4004)
#define _SDEIER  (0xc400c)
#define _SDEIIR  (0xc4008)
#define _SDEISR  (0xc4000)

#define _GEN8_GT_ISR(which) (0x44300 + (0x10 * (which)))
#define _GEN8_GT_IMR(which) (0x44304 + (0x10 * (which)))
#define _GEN8_GT_IIR(which) (0x44308 + (0x10 * (which)))
#define _GEN8_GT_IER(which) (0x4430c + (0x10 * (which)))

#define _GEN8_DE_PIPE_ISR(pipe) (0x44400 + (0x10 * (pipe)))
#define _GEN8_DE_PIPE_IMR(pipe) (0x44404 + (0x10 * (pipe)))
#define _GEN8_DE_PIPE_IIR(pipe) (0x44408 + (0x10 * (pipe)))
#define _GEN8_DE_PIPE_IER(pipe) (0x4440c + (0x10 * (pipe)))

#endif
