#ifndef __FB_DECODER_H__
#define __FB_DECODER_H__
/*
 * Decode framebuffer attributes from raw vMMIO
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

/* color space conversion and gamma correction are not included */
struct vgt_primary_plane_format {
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

struct vgt_sprite_plane_format {
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

struct vgt_cursor_plane_format {
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

/* The virtual DDI port type definition.
 *
 * DDI port A for eDP is not supported.
 * DDI port E is for CRT.
 * DDI_PORT_NONE means no valid port information available. When getting
 * this return value from vgt_pipe_format, caller should stop using the
 * virtual pipe and retry later.
 */
typedef enum {
	DDI_PORT_NONE	= 0,
	DDI_PORT_B	= 1,
	DDI_PORT_C	= 2,
	DDI_PORT_D	= 3,
	DDI_PORT_E	= 4
} ddi_port_t;

struct vgt_pipe_format {
	struct vgt_primary_plane_format	primary;
	struct vgt_sprite_plane_format	sprite;
	struct vgt_cursor_plane_format	cursor;
	ddi_port_t ddi_port;  /* the DDI port that the pipe is connected to */
};

#define MAX_INTEL_PIPES	3
struct vgt_fb_format{
	struct vgt_pipe_format	pipes[MAX_INTEL_PIPES];
};

typedef enum {
	FB_MODE_SET_START = 1,
	FB_MODE_SET_END,
	FB_DISPLAY_FLIP,
}fb_event_t;

struct fb_notify_msg {
	unsigned vm_id;
	unsigned pipe_id; /* id starting from 0 */
	unsigned plane_id; /* primary, cursor, or sprite */
};

/*
 * Decode framebuffer information from raw vMMIO
 *
 * INPUT:
 *   [domid] - specify the VM
 * OUTPUT:
 *   [format] - contain the decoded format info
 *
 */
int vgt_decode_fb_format(int vmid, struct vgt_fb_format *fb);

/*
 * Register callback to get notification of frame buffer changes
 * "struct fb_notify_msg" will be the argument to the call back
 * function, from which user could get the changed frame buffer
 * information.
 *
 */
int vgt_register_fb_notifier(struct notifier_block *nb);

/*
 * Unregister the callback for notification
 */
int vgt_unregister_fb_notifier(struct notifier_block *nb);

#endif
