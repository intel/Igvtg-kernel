/*
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

#ifndef _VGT_DEVTABLE_H
#define _VGT_DEVTABLE_H

static inline int _is_sandybridge(int devid)
{
	int ret = 0;

	switch (devid) {
	case 0x0102:
	case 0x0112:
	case 0x0122:
	case 0x0106:
	case 0x0116:
	case 0x0126:
	case 0x010A:
		ret = 1;
		break;
	default:
		break;
	}
	return ret;
}

static inline int _is_ivybridge(int devid)
{
	int ret = 0;

	switch (devid) {
	case 0x0156:
	case 0x0166:
	case 0x0152:
	case 0x0162:
	case 0x015a:
	case 0x016a:
		ret = 1;
		break;
	default:
		break;
	}
	return ret;
}

static inline int _is_haswell(int devid)
{
	int ret = 0;

	switch (devid) {
	case 0x0400:
	case 0x0402:
	case 0x0404:
	case 0x0406:
	case 0x0408:
	case 0x040a:
	case 0x0412:
	case 0x0416:
	case 0x041a:
	case 0x0422:
	case 0x0426:
	case 0x042a:
	case 0x0a02:
	case 0x0a06:
	case 0x0a0a:
	case 0x0a12:
	case 0x0a16:
	case 0x0a1a:
	case 0x0a22:
	case 0x0a26:
	case 0x0a2a:
	case 0x0c02:
	case 0x0c04:
	case 0x0c06:
	case 0x0c0a:
	case 0x0c12:
	case 0x0c16:
	case 0x0c1a:
	case 0x0c22:
	case 0x0c26:
	case 0x0c2a:
	case 0x0d12:
	case 0x0d16:
	case 0x0d1a:
	case 0x0d22:
	case 0x0d26:
	case 0x0d2a:
	case 0x0d32:
	case 0x0d36:
	case 0x0d3a:
		ret = 1;
		break;
	default:
		break;
	}
	return ret;
}

static inline int _is_broadwell(int devid)
{
	switch ((devid >> 4) & 0xf) {
		case 0:
		case 1:
		case 2:
			break;
		default:
			return 0;
	}

	devid &= ~0xf0;

	switch (devid) {
		case 0x1602:
		case 0x1606:
		case 0x160B:
		case 0x160E:
		case 0x160A:
		case 0x160D:
			break;
		default:
			return 0;
	}

	return 1;
}

static inline int _is_skylake(int devid)
{
	switch ((devid >> 4) & 0xf) {
		case 0:
		case 1:
		case 2:
		case 3:
			break;
		default:
			return 0;
	}

	devid &= ~0xf0;

	switch (devid) {
		case 0x1901:
		case 0x1902:
		case 0x1906:
		case 0x190B:
		case 0x190E:
		case 0x190A:
		case 0x190D:
			break;
		default:
			return 0;
	}

	return 1;
}


#endif  /* _VGT_DEVTABLE_H */
