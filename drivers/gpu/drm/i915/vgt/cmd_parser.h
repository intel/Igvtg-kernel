/*
 * cmd_parser.h: core header file for vGT command parser
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

#define VGT_UNHANDLEABLE 1

#define INVALID_OP    (~0U)

#define OP_LEN_MI           9
#define OP_LEN_2D           10
#define OP_LEN_3D_MEDIA     16
#define OP_LEN_MFX_VC       16
#define OP_LEN_VEBOX	    16

#define CMD_TYPE(cmd)	(((cmd) >> 29) & 7)

struct sub_op_bits{
	int hi;
	int low;
};
struct decode_info{
	char* name;
	int op_len;
	int nr_sub_op;
	struct sub_op_bits *sub_op;
};

/* Render Command Map */

/* MI_* command Opcode (28:23) */
#define OP_MI_NOOP                          0x0
#define OP_MI_SET_PREDICATE                 0x1  /* HSW+ */
#define OP_MI_USER_INTERRUPT                0x2
#define OP_MI_WAIT_FOR_EVENT                0x3
#define OP_MI_FLUSH                         0x4
#define OP_MI_ARB_CHECK                     0x5
#define OP_MI_RS_CONTROL                    0x6  /* HSW+ */
#define OP_MI_REPORT_HEAD                   0x7
#define OP_MI_ARB_ON_OFF                    0x8
#define OP_MI_URB_ATOMIC_ALLOC              0x9  /* HSW+ */
#define OP_MI_BATCH_BUFFER_END              0xA
#define OP_MI_SUSPEND_FLUSH                 0xB
#define OP_MI_PREDICATE                     0xC  /* IVB+ */
#define OP_MI_TOPOLOGY_FILTER               0xD  /* IVB+ */
#define OP_MI_SET_APPID                     0xE  /* IVB+ */
#define OP_MI_RS_CONTEXT                    0xF  /* HSW+ */
#define OP_MI_LOAD_SCAN_LINES_INCL          0x12 /* HSW+ */
#define OP_MI_DISPLAY_FLIP                  0x14
#define OP_MI_SEMAPHORE_MBOX                0x16
#define OP_MI_SET_CONTEXT                   0x18
#define OP_MI_MATH                          0x1A
#define OP_MI_URB_CLEAR                     0x19
#define OP_MI_SEMAPHORE_SIGNAL		    0x1B  /* BDW+ */
#define OP_MI_SEMAPHORE_WAIT		    0x1C  /* BDW+ */

#define OP_MI_STORE_DATA_IMM                0x20
#define OP_MI_STORE_DATA_INDEX              0x21
#define OP_MI_LOAD_REGISTER_IMM             0x22
#define OP_MI_UPDATE_GTT                    0x23
#define OP_MI_STORE_REGISTER_MEM            0x24
#define OP_MI_FLUSH_DW                      0x26
#define OP_MI_CLFLUSH                       0x27
#define OP_MI_REPORT_PERF_COUNT             0x28
#define OP_MI_LOAD_REGISTER_MEM             0x29  /* HSW+ */
#define OP_MI_LOAD_REGISTER_REG             0x2A  /* HSW+ */
#define OP_MI_RS_STORE_DATA_IMM             0x2B  /* HSW+ */
#define OP_MI_LOAD_URB_MEM                  0x2C  /* HSW+ */
#define OP_MI_STORE_URM_MEM                 0x2D  /* HSW+ */
#define OP_MI_2E			    0x2E  /* BDW+ */
#define OP_MI_2F			    0x2F  /* BDW+ */
#define OP_MI_BATCH_BUFFER_START            0x31

/* Bit definition for dword 0 */
#define _CMDBIT_BB_START_IN_PPGTT	(1UL << 8)

#define OP_MI_CONDITIONAL_BATCH_BUFFER_END  0x36

#define BATCH_BUFFER_ADDR_MASK ((1UL << 32) - (1U <<2))
#define BATCH_BUFFER_ADDR_HIGH_MASK ((1UL << 16) - (1U))
#define BATCH_BUFFER_ADR_SPACE_BIT(x)	(((x)>>8) & 1U)
#define BATCH_BUFFER_2ND_LEVEL_BIT(x)   ((x)>>22 & 1U)

/* 2D command: Opcode (28:22) */
#define OP_2D(x)    ((2<<7) | x)

#define OP_XY_SETUP_BLT                             OP_2D(0x1)
#define OP_XY_SETUP_CLIP_BLT                        OP_2D(0x3)
#define OP_XY_SETUP_MONO_PATTERN_SL_BLT             OP_2D(0x11)
#define OP_XY_PIXEL_BLT                             OP_2D(0x24)
#define OP_XY_SCANLINES_BLT                         OP_2D(0x25)
#define OP_XY_TEXT_BLT                              OP_2D(0x26)
#define OP_XY_TEXT_IMMEDIATE_BLT                    OP_2D(0x31)
#define OP_COLOR_BLT                                OP_2D(0x40)
#define OP_SRC_COPY_BLT                             OP_2D(0x43)
#define OP_XY_COLOR_BLT                             OP_2D(0x50)
#define OP_XY_PAT_BLT                               OP_2D(0x51)
#define OP_XY_MONO_PAT_BLT                          OP_2D(0x52)
#define OP_XY_SRC_COPY_BLT                          OP_2D(0x53)
#define OP_XY_MONO_SRC_COPY_BLT                     OP_2D(0x54)
#define OP_XY_FULL_BLT                              OP_2D(0x55)
#define OP_XY_FULL_MONO_SRC_BLT                     OP_2D(0x56)
#define OP_XY_FULL_MONO_PATTERN_BLT                 OP_2D(0x57)
#define OP_XY_FULL_MONO_PATTERN_MONO_SRC_BLT        OP_2D(0x58)
#define OP_XY_MONO_PAT_FIXED_BLT                    OP_2D(0x59)
#define OP_XY_MONO_SRC_COPY_IMMEDIATE_BLT           OP_2D(0x71)
#define OP_XY_PAT_BLT_IMMEDIATE                     OP_2D(0x72)
#define OP_XY_SRC_COPY_CHROMA_BLT                   OP_2D(0x73)
#define OP_XY_FULL_IMMEDIATE_PATTERN_BLT            OP_2D(0x74)
#define OP_XY_FULL_MONO_SRC_IMMEDIATE_PATTERN_BLT   OP_2D(0x75)
#define OP_XY_PAT_CHROMA_BLT                        OP_2D(0x76)
#define OP_XY_PAT_CHROMA_BLT_IMMEDIATE              OP_2D(0x77)

/* 3D/Media Command: Pipeline Type(28:27) Opcode(26:24) Sub Opcode(23:16) */
#define OP_3D_MEDIA(sub_type, opcode, sub_opcode) \
	( (3<<13) | ((sub_type)<<11) | ((opcode) <<8) | (sub_opcode))

#define OP_STATE_PREFETCH                       OP_3D_MEDIA(0x0, 0x0, 0x03)

#define OP_STATE_BASE_ADDRESS                   OP_3D_MEDIA(0x0, 0x1, 0x01)
#define OP_STATE_SIP                            OP_3D_MEDIA(0x0, 0x1, 0x02)
#define OP_3D_MEDIA_0_1_4			OP_3D_MEDIA(0x0, 0x1, 0x04)

#define OP_3DSTATE_VF_STATISTICS_GM45           OP_3D_MEDIA(0x1, 0x0, 0x0B)

#define OP_PIPELINE_SELECT                      OP_3D_MEDIA(0x1, 0x1, 0x04)

#define OP_MEDIA_VFE_STATE                      OP_3D_MEDIA(0x2, 0x0, 0x0)
#define OP_MEDIA_CURBE_LOAD                     OP_3D_MEDIA(0x2, 0x0, 0x1)
#define OP_MEDIA_INTERFACE_DESCRIPTOR_LOAD      OP_3D_MEDIA(0x2, 0x0, 0x2)
#define OP_MEDIA_GATEWAY_STATE                  OP_3D_MEDIA(0x2, 0x0, 0x3)
#define OP_MEDIA_STATE_FLUSH                    OP_3D_MEDIA(0x2, 0x0, 0x4)

#define OP_MEDIA_OBJECT                         OP_3D_MEDIA(0x2, 0x1, 0x0)
#define OP_MEDIA_OBJECT_PRT                     OP_3D_MEDIA(0x2, 0x1, 0x2)
#define OP_MEDIA_OBJECT_WALKER                  OP_3D_MEDIA(0x2, 0x1, 0x3)
#define OP_GPGPU_WALKER                         OP_3D_MEDIA(0x2, 0x1, 0x5)

#define OP_3DSTATE_BINDING_TABLE_POINTERS       OP_3D_MEDIA(0x3, 0x0, 0x01)
#define OP_3DSTATE_SAMPLER_STATE_POINTERS       OP_3D_MEDIA(0x3, 0x0, 0x02)
#define OP_3DSTATE_CLEAR_PARAMS                 OP_3D_MEDIA(0x3, 0x0, 0x04) /* IVB+ */
#define OP_3DSTATE_DEPTH_BUFFER                 OP_3D_MEDIA(0x3, 0x0, 0x05) /* IVB+ */
#define OP_3DSTATE_URB                          OP_3D_MEDIA(0x3, 0x0, 0x05) /* SNB  */
#define OP_3DSTATE_STENCIL_BUFFER               OP_3D_MEDIA(0x3, 0x0, 0x06) /* IVB+ */
#define OP_3DSTATE_HIER_DEPTH_BUFFER            OP_3D_MEDIA(0x3, 0x0, 0x07) /* IVB+ */
#define OP_3DSTATE_VERTEX_BUFFERS               OP_3D_MEDIA(0x3, 0x0, 0x08)
#define OP_3DSTATE_VERTEX_ELEMENTS              OP_3D_MEDIA(0x3, 0x0, 0x09)
#define OP_3DSTATE_INDEX_BUFFER                 OP_3D_MEDIA(0x3, 0x0, 0x0A)
#define OP_3DSTATE_VF_STATISTICS                OP_3D_MEDIA(0x3, 0x0, 0x0B)
#define OP_3DSTATE_VF                           OP_3D_MEDIA(0x3, 0x0, 0x0C)  /* HSW+ */
#define OP_3DSTATE_VIEWPORT_STATE_POINTERS      OP_3D_MEDIA(0x3, 0x0, 0x0D)
#define OP_3DSTATE_CC_STATE_POINTERS            OP_3D_MEDIA( 0x3 ,0x0, 0x0E )
#define OP_3DSTATE_SCISSOR_STATE_POINTERS       OP_3D_MEDIA( 0x3 ,0x0, 0x0F )
#define OP_3DSTATE_VS                           OP_3D_MEDIA( 0x3 ,0x0, 0x10)
#define OP_3DSTATE_GS                           OP_3D_MEDIA( 0x3 ,0x0, 0x11 )
#define OP_3DSTATE_CLIP                         OP_3D_MEDIA( 0x3 ,0x0, 0x12 )
#define OP_3DSTATE_SF                           OP_3D_MEDIA( 0x3 ,0x0, 0x13)
#define OP_3DSTATE_WM                           OP_3D_MEDIA( 0x3 ,0x0, 0x14 )
#define OP_3DSTATE_CONSTANT_VS                  OP_3D_MEDIA( 0x3 ,0x0, 0x15)
#define OP_3DSTATE_CONSTANT_GS                  OP_3D_MEDIA( 0x3 ,0x0, 0x16 )
#define OP_3DSTATE_CONSTANT_PS                  OP_3D_MEDIA( 0x3 ,0x0, 0x17 )
#define OP_3DSTATE_SAMPLE_MASK                  OP_3D_MEDIA( 0x3 ,0x0, 0x18 )
#define OP_3DSTATE_CONSTANT_HS                  OP_3D_MEDIA( 0x3 ,0x0, 0x19 ) /* IVB+ */
#define OP_3DSTATE_CONSTANT_DS                  OP_3D_MEDIA( 0x3 ,0x0, 0x1A ) /* IVB+ */
#define OP_3DSTATE_HS                           OP_3D_MEDIA( 0x3 ,0x0, 0x1B ) /* IVB+ */
#define OP_3DSTATE_TE                           OP_3D_MEDIA( 0x3 ,0x0, 0x1C ) /* IVB+ */
#define OP_3DSTATE_DS                           OP_3D_MEDIA( 0x3 ,0x0, 0x1D ) /* IVB+ */
#define OP_3DSTATE_STREAMOUT                    OP_3D_MEDIA( 0x3 ,0x0, 0x1E ) /* IVB+ */
#define OP_3DSTATE_SBE                          OP_3D_MEDIA( 0x3 ,0x0, 0x1F ) /* IVB+ */
#define OP_3DSTATE_PS                           OP_3D_MEDIA( 0x3 ,0x0, 0x20 ) /* IVB+ */
#define OP_3DSTATE_VIEWPORT_STATE_POINTERS_SF_CLIP OP_3D_MEDIA(0x3, 0x0, 0x21) /* IVB+ */
#define OP_3DSTATE_VIEWPORT_STATE_POINTERS_CC   OP_3D_MEDIA(0x3, 0x0, 0x23) /* IVB+ */
#define OP_3DSTATE_BLEND_STATE_POINTERS         OP_3D_MEDIA(0x3, 0x0, 0x24) /* IVB+ */
#define OP_3DSTATE_DEPTH_STENCIL_STATE_POINTERS OP_3D_MEDIA(0x3, 0x0, 0x25) /* IVB+ */
#define OP_3DSTATE_BINDING_TABLE_POINTERS_VS    OP_3D_MEDIA(0x3, 0x0, 0x26) /* IVB+ */
#define OP_3DSTATE_BINDING_TABLE_POINTERS_HS    OP_3D_MEDIA(0x3, 0x0, 0x27) /* IVB+ */
#define OP_3DSTATE_BINDING_TABLE_POINTERS_DS    OP_3D_MEDIA(0x3, 0x0, 0x28) /* IVB+ */
#define OP_3DSTATE_BINDING_TABLE_POINTERS_GS    OP_3D_MEDIA(0x3, 0x0, 0x29) /* IVB+ */
#define OP_3DSTATE_BINDING_TABLE_POINTERS_PS    OP_3D_MEDIA(0x3, 0x0, 0x2A) /* IVB+ */
#define OP_3DSTATE_SAMPLER_STATE_POINTERS_VS    OP_3D_MEDIA(0x3, 0x0, 0x2B) /* IVB+ */
#define OP_3DSTATE_SAMPLER_STATE_POINTERS_HS    OP_3D_MEDIA(0x3, 0x0, 0x2C) /* IVB+ */
#define OP_3DSTATE_SAMPLER_STATE_POINTERS_DS    OP_3D_MEDIA(0x3, 0x0, 0x2D) /* IVB+ */
#define OP_3DSTATE_SAMPLER_STATE_POINTERS_GS    OP_3D_MEDIA(0x3, 0x0, 0x2E) /* IVB+ */
#define OP_3DSTATE_SAMPLER_STATE_POINTERS_PS    OP_3D_MEDIA(0x3, 0x0, 0x2F) /* IVB+ */
#define OP_3DSTATE_URB_VS                       OP_3D_MEDIA(0x3, 0x0, 0x30) /* IVB+ */
#define OP_3DSTATE_URB_HS                       OP_3D_MEDIA(0x3, 0x0, 0x31) /* IVB+ */
#define OP_3DSTATE_URB_DS                       OP_3D_MEDIA(0x3, 0x0, 0x32) /* IVB+ */
#define OP_3DSTATE_URB_GS                       OP_3D_MEDIA(0x3, 0x0, 0x33) /* IVB+ */
#define OP_3DSTATE_GATHER_CONSTANT_VS           OP_3D_MEDIA(0x3, 0x0, 0x34) /* HSW+ */
#define OP_3DSTATE_GATHER_CONSTANT_GS           OP_3D_MEDIA(0x3, 0x0, 0x35) /* HSW+ */
#define OP_3DSTATE_GATHER_CONSTANT_HS           OP_3D_MEDIA(0x3, 0x0, 0x36) /* HSW+ */
#define OP_3DSTATE_GATHER_CONSTANT_DS           OP_3D_MEDIA(0x3, 0x0, 0x37) /* HSW+ */
#define OP_3DSTATE_GATHER_CONSTANT_PS           OP_3D_MEDIA(0x3, 0x0, 0x38) /* HSW+ */
#define OP_3DSTATE_DX9_CONSTANTF_VS             OP_3D_MEDIA(0x3, 0x0, 0x39) /* HSW+ */
#define OP_3DSTATE_DX9_CONSTANTF_PS             OP_3D_MEDIA(0x3, 0x0, 0x3A) /* HSW+ */
#define OP_3DSTATE_DX9_CONSTANTI_VS             OP_3D_MEDIA(0x3, 0x0, 0x3B) /* HSW+ */
#define OP_3DSTATE_DX9_CONSTANTI_PS             OP_3D_MEDIA(0x3, 0x0, 0x3C) /* HSW+ */
#define OP_3DSTATE_DX9_CONSTANTB_VS             OP_3D_MEDIA(0x3, 0x0, 0x3D) /* HSW+ */
#define OP_3DSTATE_DX9_CONSTANTB_PS             OP_3D_MEDIA(0x3, 0x0, 0x3E) /* HSW+ */
#define OP_3DSTATE_DX9_LOCAL_VALID_VS           OP_3D_MEDIA(0x3, 0x0, 0x3F) /* HSW+ */
#define OP_3DSTATE_DX9_LOCAL_VALID_PS           OP_3D_MEDIA(0x3, 0x0, 0x40) /* HSW+ */
#define OP_3DSTATE_DX9_GENERATE_ACTIVE_VS       OP_3D_MEDIA(0x3, 0x0, 0x41) /* HSW+ */
#define OP_3DSTATE_DX9_GENERATE_ACTIVE_PS       OP_3D_MEDIA(0x3, 0x0, 0x42) /* HSW+ */
#define OP_3DSTATE_BINDING_TABLE_EDIT_VS        OP_3D_MEDIA(0x3, 0x0, 0x43) /* HSW+ */
#define OP_3DSTATE_BINDING_TABLE_EDIT_GS        OP_3D_MEDIA(0x3, 0x0, 0x44) /* HSW+ */
#define OP_3DSTATE_BINDING_TABLE_EDIT_HS        OP_3D_MEDIA(0x3, 0x0, 0x45) /* HSW+ */
#define OP_3DSTATE_BINDING_TABLE_EDIT_DS        OP_3D_MEDIA(0x3, 0x0, 0x46) /* HSW+ */
#define OP_3DSTATE_BINDING_TABLE_EDIT_PS        OP_3D_MEDIA(0x3, 0x0, 0x47) /* HSW+ */

#define OP_3DSTATE_VF_INSTANCING 		OP_3D_MEDIA(0x3, 0x0, 0x49) /* BDW+ */
#define OP_3DSTATE_VF_SGVS  			OP_3D_MEDIA(0x3, 0x0, 0x4A) /* BDW+ */
#define OP_3DSTATE_VF_TOPOLOGY   		OP_3D_MEDIA(0x3, 0x0, 0x4B) /* BDW+ */
#define OP_3DSTATE_WM_CHROMAKEY   		OP_3D_MEDIA(0x3, 0x0, 0x4C) /* BDW+ */
#define OP_3DSTATE_PS_BLEND   			OP_3D_MEDIA(0x3, 0x0, 0x4D) /* BDW+ */
#define OP_3DSTATE_WM_DEPTH_STENCIL   		OP_3D_MEDIA(0x3, 0x0, 0x4E) /* BDW+ */
#define OP_3DSTATE_PS_EXTRA   			OP_3D_MEDIA(0x3, 0x0, 0x4F) /* BDW+ */
#define OP_3DSTATE_RASTER   			OP_3D_MEDIA(0x3, 0x0, 0x50) /* BDW+ */
#define OP_3DSTATE_SBE_SWIZ   			OP_3D_MEDIA(0x3, 0x0, 0x51) /* BDW+ */
#define OP_3DSTATE_WM_HZ_OP   			OP_3D_MEDIA(0x3, 0x0, 0x52) /* BDW+ */

#define OP_3DSTATE_DRAWING_RECTANGLE            OP_3D_MEDIA( 0x3 ,0x1, 0x00 )
#define OP_3DSTATE_SAMPLER_PALETTE_LOAD0        OP_3D_MEDIA( 0x3 ,0x1, 0x02 )
#define OP_3DSTATE_CHROMA_KEY                   OP_3D_MEDIA( 0x3 ,0x1, 0x04 )
#define OP_SNB_3DSTATE_DEPTH_BUFFER             OP_3D_MEDIA( 0x3 ,0x1, 0x05 )
#define OP_3DSTATE_POLY_STIPPLE_OFFSET          OP_3D_MEDIA( 0x3 ,0x1, 0x06 )
#define OP_3DSTATE_POLY_STIPPLE_PATTERN         OP_3D_MEDIA( 0x3 ,0x1, 0x07 )
#define OP_3DSTATE_LINE_STIPPLE                 OP_3D_MEDIA( 0x3 ,0x1, 0x08 )
#define OP_3DSTATE_AA_LINE_PARAMS               OP_3D_MEDIA( 0x3 ,0x1, 0x0A )
#define OP_3DSTATE_GS_SVB_INDEX                 OP_3D_MEDIA( 0x3 ,0x1, 0x0B )
#define OP_3DSTATE_SAMPLER_PALETTE_LOAD1        OP_3D_MEDIA( 0x3 ,0x1, 0x0C )
#define OP_3DSTATE_MULTISAMPLE                  OP_3D_MEDIA( 0x3 ,0x1, 0x0D )
#define OP_3DSTATE_MULTISAMPLE_BDW		OP_3D_MEDIA( 0x3 ,0x0, 0x0D )
#define OP_3DSTATE_RAST_MULTISAMPLE             OP_3D_MEDIA( 0x3 ,0x1, 0x0E )
#define OP_SNB_3DSTATE_STENCIL_BUFFER           OP_3D_MEDIA( 0x3 ,0x1, 0x0E )
#define OP_SNB_3DSTATE_HIER_DEPTH_BUFFER        OP_3D_MEDIA( 0x3 ,0x1, 0x0F )
#define OP_SNB_3DSTATE_CLEAR_PARAMS             OP_3D_MEDIA( 0x3 ,0x1, 0x10 )
#define OP_3DSTATE_MONOFILTER_SIZE              OP_3D_MEDIA( 0x3 ,0x1, 0x11 )
#define OP_3DSTATE_PUSH_CONSTANT_ALLOC_VS       OP_3D_MEDIA(0x3, 0x1, 0x12) /* IVB+ */
#define OP_3DSTATE_PUSH_CONSTANT_ALLOC_HS       OP_3D_MEDIA(0x3, 0x1, 0x13) /* IVB+ */
#define OP_3DSTATE_PUSH_CONSTANT_ALLOC_DS       OP_3D_MEDIA(0x3, 0x1, 0x14) /* IVB+ */
#define OP_3DSTATE_PUSH_CONSTANT_ALLOC_GS       OP_3D_MEDIA(0x3, 0x1, 0x15) /* IVB+ */
#define OP_3DSTATE_PUSH_CONSTANT_ALLOC_PS       OP_3D_MEDIA(0x3, 0x1, 0x16) /* IVB+ */
#define OP_3DSTATE_SO_DECL_LIST                 OP_3D_MEDIA( 0x3 ,0x1, 0x17 )
#define OP_3DSTATE_SO_BUFFER                    OP_3D_MEDIA( 0x3 ,0x1, 0x18 )
#define OP_3DSTATE_BINDING_TABLE_POOL_ALLOC     OP_3D_MEDIA( 0x3 ,0x1, 0x19 ) /* HSW+ */
#define OP_3DSTATE_GATHER_POOL_ALLOC            OP_3D_MEDIA( 0x3 ,0x1, 0x1A ) /* HSW+ */
#define OP_3DSTATE_DX9_CONSTANT_BUFFER_POOL_ALLOC OP_3D_MEDIA( 0x3 ,0x1, 0x1B ) /* HSW+ */
#define OP_3DSTATE_SAMPLE_PATTERN               OP_3D_MEDIA (0x3 ,0x1, 0x1C )
#define OP_3DSTATE_URB_CLEAR                    OP_3D_MEDIA (0x3 ,0x1, 0x1D )

#define OP_PIPE_CONTROL                         OP_3D_MEDIA( 0x3 ,0x2, 0x00 )

#define OP_3DPRIMITIVE                          OP_3D_MEDIA( 0x3 ,0x3, 0x00 )

/* VCCP Command Parser */

/*
 * Below MFX and VBE cmd definition is from vaapi intel driver project (BSD License)
 * git://anongit.freedesktop.org/vaapi/intel-driver
 * src/i965_defines.h
 *
 */

#define OP_MFX(pipeline, op, sub_opa, sub_opb)     \
     (3 << 13 |                                  \
     (pipeline) << 11 |                         \
     (op) << 8 |                               \
     (sub_opa) << 5 |                          \
     (sub_opb))

#define OP_MFX_PIPE_MODE_SELECT                    OP_MFX(2, 0, 0, 0)  /* ALL */
#define OP_MFX_SURFACE_STATE                       OP_MFX(2, 0, 0, 1)  /* ALL */
#define OP_MFX_PIPE_BUF_ADDR_STATE                 OP_MFX(2, 0, 0, 2)  /* ALL */
#define OP_MFX_IND_OBJ_BASE_ADDR_STATE             OP_MFX(2, 0, 0, 3)  /* ALL */
#define OP_MFX_BSP_BUF_BASE_ADDR_STATE             OP_MFX(2, 0, 0, 4)  /* ALL */
#define OP_2_0_0_5                                 OP_MFX(2, 0, 0, 5)  /* ALL */
#define OP_MFX_STATE_POINTER                       OP_MFX(2, 0, 0, 6)  /* ALL */
#define OP_MFX_QM_STATE                            OP_MFX(2, 0, 0, 7)  /* IVB+ */
#define OP_MFX_FQM_STATE                           OP_MFX(2, 0, 0, 8)  /* IVB+ */

#define OP_MFX_PAK_INSERT_OBJECT                   OP_MFX(2, 0, 2, 8)  /* IVB+ */
#define OP_MFX_STITCH_OBJECT                       OP_MFX(2, 0, 2, 0xA)  /* IVB+ */

#define OP_MFD_IT_OBJECT                           OP_MFX(2, 0, 1, 9) /* ALL */

#define OP_MFX_WAIT                                OP_MFX(1, 0, 0, 0) /* IVB+ */

#define OP_MFX_AVC_IMG_STATE                       OP_MFX(2, 1, 0, 0) /* ALL */
#define OP_MFX_AVC_QM_STATE                        OP_MFX(2, 1, 0, 1) /* ALL */
#define OP_MFX_AVC_DIRECTMODE_STATE                OP_MFX(2, 1, 0, 2) /* ALL */
#define OP_MFX_AVC_SLICE_STATE                     OP_MFX(2, 1, 0, 3) /* ALL */
#define OP_MFX_AVC_REF_IDX_STATE                   OP_MFX(2, 1, 0, 4) /* ALL */
#define OP_MFX_AVC_WEIGHTOFFSET_STATE              OP_MFX(2, 1, 0, 5) /* ALL */

#define OP_MFD_AVC_PICID_STATE                     OP_MFX(2, 1, 1, 5) /* HSW+ */
#define OP_MFD_AVC_DPB_STATE			   OP_MFX(2, 1, 1, 6) /* IVB+ */
#define OP_MFD_AVC_SLICEADDR                       OP_MFX(2, 1, 1, 7) /* IVB+ */
#define OP_MFD_AVC_BSD_OBJECT                      OP_MFX(2, 1, 1, 8) /* ALL */

#define OP_MFC_AVC_FQM_STATE                       OP_MFX(2, 1, 2, 2) /* SNB */
#define OP_MFC_AVC_PAK_INSERT_OBJECT               OP_MFX(2, 1, 2, 8) /* SNB */
#define OP_MFC_AVC_PAK_OBJECT                      OP_MFX(2, 1, 2, 9) /* ALL */

#define OP_MFX_VC1_PIC_STATE                       OP_MFX(2, 2, 0, 0) /* SNB */
#define OP_MFX_VC1_PRED_PIPE_STATE                 OP_MFX(2, 2, 0, 1) /* ALL */
#define OP_MFX_VC1_DIRECTMODE_STATE                OP_MFX(2, 2, 0, 2) /* ALL */

#define OP_MFD_VC1_SHORT_PIC_STATE                 OP_MFX(2, 2, 1, 0) /* IVB+ */
#define OP_MFD_VC1_LONG_PIC_STATE                  OP_MFX(2, 2, 1, 1) /* IVB+ */

#define OP_MFD_VC1_BSD_OBJECT                      OP_MFX(2, 2, 1, 8) /* ALL */

#define OP_MFX_MPEG2_PIC_STATE                     OP_MFX(2, 3, 0, 0) /* ALL */
#define OP_MFX_MPEG2_QM_STATE                      OP_MFX(2, 3, 0, 1) /* ALL */

#define OP_MFD_MPEG2_BSD_OBJECT                    OP_MFX(2, 3, 1, 8) /* ALL */

#define OP_MFC_MPEG2_SLICEGROUP_STATE              OP_MFX(2, 3, 2, 3) /* ALL */
#define OP_MFC_MPEG2_PAK_OBJECT                    OP_MFX(2, 3, 2, 9) /* ALL */

#define OP_MFX_2_6_0_0                             OP_MFX(2, 6, 0, 0) /* IVB+ */
#define OP_MFX_2_6_0_8                             OP_MFX(2, 6, 0, 8) /* IVB+ */
#define OP_MFX_2_6_0_9                             OP_MFX(2, 6, 0, 9) /* IVB+ */

#define OP_MFX_JPEG_PIC_STATE                      OP_MFX(2, 7, 0, 0)
#define OP_MFX_JPEG_HUFF_TABLE_STATE               OP_MFX(2, 7, 0, 2)

#define OP_MFD_JPEG_BSD_OBJECT                     OP_MFX(2, 7, 1, 8)

/* copy from vaapi, but not found definition in PRM yet */
#define OP_VEB(pipeline, op, sub_opa, sub_opb)     \
     (3 << 13 |                                 \
     (pipeline) << 11 |                         \
     (op) << 8 |                               \
     (sub_opa) << 5 |                          \
     (sub_opb))

#define OP_VEB_SURFACE_STATE                       OP_VEB(2, 4, 0, 0)
#define OP_VEB_STATE                               OP_VEB(2, 4, 0, 2)
#define OP_VEB_DNDI_IECP_STATE                     OP_VEB(2, 4, 0, 3)

extern int vgt_scan_vring_2(struct vgt_device *vgt, int ring_id);

struct parser_exec_state;

typedef int (*parser_cmd_handler)(struct parser_exec_state *s);

#define VGT_CMD_HASH_BITS   7

/* which DWords need address fix */
#define ADDR_FIX_1(x1)                  (1<<(x1))
#define ADDR_FIX_2(x1,x2)               (ADDR_FIX_1(x1) | ADDR_FIX_1(x2))
#define ADDR_FIX_3(x1,x2,x3)            (ADDR_FIX_1(x1) | ADDR_FIX_2(x2,x3))
#define ADDR_FIX_4(x1,x2,x3,x4)         (ADDR_FIX_1(x1) | ADDR_FIX_3(x2,x3,x4))
#define ADDR_FIX_5(x1,x2,x3,x4,x5)      (ADDR_FIX_1(x1) | ADDR_FIX_4(x2,x3,x4,x5))

struct cmd_info{
	char* name;
	uint32_t opcode;

#define F_LEN_MASK	(1U<<0)
#define F_LEN_CONST  1U
#define F_LEN_VAR    0U

/* command has its own ip advance logic
   e.g. MI_BATCH_START, MI_BATCH_END
*/
#define F_IP_ADVANCE_CUSTOM (1<<1)

#define F_POST_HANDLE	(1<<2)
	uint32_t flag;

#define R_RCS	(1 << RING_BUFFER_RCS )
#define R_VCS1  (1 << RING_BUFFER_VCS)
#define R_VCS2  (1 << RING_BUFFER_VCS2)
#define R_VCS	( R_VCS1 | R_VCS2)
#define R_BCS	(1 << RING_BUFFER_BCS )
#define R_VECS	(1 << RING_BUFFER_VECS )
#define R_ALL (R_RCS | R_VCS | R_BCS | R_VECS)
	/* rings that support this cmd: BLT/RCS/VCS/VECS */
	uint16_t rings;

	/* devices that support this cmd: SNB/IVB/HSW/... */
	uint16_t devices;

	/* which DWords are address that need fix up.
	 * bit 0 means a 32-bit non address operand in command
	 * bit 1 means address operand, which could be 32-bit
	 * or 64-bit depending on different architectures.(
	 * defined by "gmadr_bytes_in_cmd" in pgt_device.
	 * No matter the address length, each address only takes
	 * one bit in the bitmap.
	 */
	uint16_t addr_bitmap;

	/*	flag == F_LEN_CONST : command length
		flag == F_LEN_VAR : lenght bias bits
		Note: length is in DWord
	 */
	uint8_t	len;

	parser_cmd_handler handler;
};
#define VGT_MAX_CMD_LENGTH	20  /* In Dword */
struct vgt_cmd_entry {
	struct hlist_node hlist;
	struct cmd_info* info;
};

typedef enum {
	RING_BUFFER_INSTRUCTION,
	BATCH_BUFFER_INSTRUCTION,
	BATCH_BUFFER_2ND_LEVEL,
}cmd_buf_t;

typedef enum{
	GTT_BUFFER,
	PPGTT_BUFFER
}gtt_addr_t;

struct parser_exec_state {
	struct vgt_device *vgt;
	int ring_id;

	uint64_t request_id;

	cmd_buf_t buf_type;

	/* batch buffer address type */
	gtt_addr_t buf_addr_type;

	/* graphics memory address of ring buffer start */
	unsigned long ring_start;
	unsigned long ring_size;
	unsigned long ring_head;
	unsigned long ring_tail;

	/* instruction graphics memory address */
	unsigned long ip_gma;

	/* mapped va of the instr_gma */
	uint32_t *ip_va;

	/* length of free buffer in current page, in qword */
	unsigned long ip_buf_len;

	/* mapped va of the next page near instr_gma */
	uint32_t *ip_va_next_page;

	/* next instruction when return from  batch buffer to ring buffer */
	unsigned long ret_ip_gma_ring;

	/* next instruction when return from 2nd batch buffer to batch buffer */
	unsigned long ret_ip_gma_bb;

	/* batch buffer address type (GTT or PPGTT)
	   used when ret from 2nd level batch buffer */
	gtt_addr_t saved_buf_addr_type;

	/* indicate the command has user interrupt*/
	bool cmd_issue_irq;

	struct cmd_info* info;

	uint32_t *ip_buf_va;
	void *ip_buf;
};

#define CMD_TAIL_NUM	1024
#define CMD_HANDLER_NUM	1024
#define CMD_PATCH_NUM	CMD_HANDLER_NUM * 8
/* a DW based structure to avoid cross-page trickiness */
struct cmd_patch_info {
	uint64_t request_id;
	void *addr;
	uint32_t old_val;
	uint32_t new_val;
};

struct cmd_handler_info {
	uint64_t request_id;
	struct parser_exec_state exec_state;
	parser_cmd_handler handler;
};

struct cmd_tail_info {
	uint64_t request_id;
	uint32_t tail;
	uint32_t cmd_nr;
	uint32_t ip_offset;

#define F_CMDS_ISSUE_IRQ (1<<0)
	uint32_t flags;
};

struct cmd_general_info {
	union {
		struct cmd_patch_info patch[CMD_PATCH_NUM];
		struct cmd_handler_info handler[CMD_HANDLER_NUM];
		struct cmd_tail_info cmd[CMD_TAIL_NUM];
	};
	int	head;
	int	tail;
	int	count;
};

extern uint32_t vgt_get_opcode(uint32_t cmd, int ring_id );
extern void vgt_cmd_name(uint32_t cmd, int ring_id, int gen);
