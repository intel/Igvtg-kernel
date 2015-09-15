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

#ifndef _GVT_FB_DECODER_H_
#define _GVT_FB_DECODER_H_

typedef enum {
	FB_MODE_SET_START = 1,
	FB_MODE_SET_END,
	FB_DISPLAY_FLIP,
}gvt_fb_event_t;

typedef enum {
	DDI_PORT_NONE	= 0,
	DDI_PORT_B	= 1,
	DDI_PORT_C	= 2,
	DDI_PORT_D	= 3,
	DDI_PORT_E	= 4
} ddi_port_t;

struct pgt_device;

struct gvt_fb_notify_msg {
	unsigned vm_id;
	unsigned pipe_id; /* id starting from 0 */
	unsigned plane_id; /* primary, cursor, or sprite */
};

/* color space conversion and gamma correction are not included */
struct gvt_primary_plane_format {
	u8	enabled;	/* plane is enabled */
	u8	tiled;		/* X-tiled */
	u8	bpp;		/* bits per pixel */
	u32	hw_format;	/* format field in the PRI_CTL register */
	u32	drm_format;	/* format in DRM definition */
	u32	base;		/* framebuffer base in graphics memory */
	u32	x_offset;	/* in pixels */
	u32	y_offset;	/* in lines */
	u32	width;		/* in pixels */
	u32	height;		/* in lines */
	u32	stride;		/* in bytes */
};

struct gvt_sprite_plane_format {
	u8	enabled;	/* plane is enabled */
	u8	tiled;		/* X-tiled */
	u8	bpp;		/* bits per pixel */
	u32	hw_format;	/* format field in the SPR_CTL register */
	u32	drm_format;	/* format in DRM definition */
	u32	base;		/* sprite base in graphics memory */
	u32	x_pos;		/* in pixels */
	u32	y_pos;		/* in lines */
	u32	x_offset;	/* in pixels */
	u32	y_offset;	/* in lines */
	u32	width;		/* in pixels */
	u32	height;		/* in lines */
};

struct gvt_cursor_plane_format {
	u8	enabled;
	u8	mode;		/* cursor mode select */
	u8	bpp;		/* bits per pixel */
	u32	drm_format;	/* format in DRM definition */
	u32	base;		/* cursor base in graphics memory */
	u32	x_pos;		/* in pixels */
	u32	y_pos;		/* in lines */
	u8	x_sign;		/* X Position Sign */
	u8	y_sign;		/* Y Position Sign */
	u32	width;		/* in pixels */
	u32	height;		/* in lines */
	u32	x_hot;		/* in pixels */
	u32	y_hot;		/* in pixels */
};

struct gvt_pipe_format {
	struct gvt_primary_plane_format	primary;
	struct gvt_sprite_plane_format	sprite;
	struct gvt_cursor_plane_format	cursor;
	ddi_port_t ddi_port;  /* the DDI port that the pipe is connected to */
};

struct gvt_fb_format{
	struct gvt_pipe_format	pipes[I915_MAX_PIPES];
};

extern int gvt_fb_notifier_call_chain(unsigned long val, void *data);
extern int gvt_decode_fb_format(struct pgt_device *pdev, int vmid,
				struct gvt_fb_format *fb);

#endif
