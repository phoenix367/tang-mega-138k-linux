/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * gowin-lcdfb: YUV 4:2:0 scan-out buffers of the Tang Mega 138K Pro LCD core.
 *
 * Besides the RGB565 framebuffer (fbdev as usual), the driver owns a few
 * planar YUV 4:2:0 buffers the scan-out core can display directly (gateware
 * Build 68+).  A video decoder writes a frame into one of them through
 * mmap() -- a plain cached mapping, no deferred I/O -- and asks the driver to
 * show it: the driver writes the buffer back from the CPU cache and programs
 * the core, which switches buffers at the next frame boundary (no tearing).
 * Closing the fbdev file descriptor, or GOWIN_LCD_IOC_RGB, returns to the
 * RGB565 framebuffer.
 */
#ifndef _UAPI_LINUX_GOWIN_LCDFB_H
#define _UAPI_LINUX_GOWIN_LCDFB_H

#include <linux/types.h>
#include <linux/ioctl.h>

struct gowin_lcd_yuv_info {
	__u32 nbuf;		/* YUV buffers; 0 = the gateware has no YUV mode */
	__u32 width, height;	/* the panel, pixels */
	__u32 y_stride;		/* bytes per Y line in the default layout (multiple of 32) */
	__u32 c_stride;		/* bytes per U or V line (multiple of 32) */
	__u32 u_off, v_off;	/* plane offsets of the default layout */
	__u32 buf_size;		/* bytes per buffer (page multiple) */
	__u32 mmap_off;		/* mmap() offset of buffer 0; buffer i is at mmap_off + i * buf_size */
};

struct gowin_lcd_yuv_show {
	__u32 buf;		/* buffer index */
	__u32 flags;		/* GOWIN_LCD_YUV_* */
	__u32 y_off, u_off, v_off;	/* plane starts within the buffer, 32-byte aligned */
	__u32 y_stride, c_stride;	/* multiples of 32; the planes must stay inside the buffer */
};

#define GOWIN_LCD_YUV_FULL_RANGE	0x1	/* JPEG-range YUV (yuvj420p), else BT.601 limited */
#define GOWIN_LCD_YUV_NOSYNC		0x2	/* a DMA master wrote the buffer, the CPU cache holds nothing of it: skip the write-back */

struct gowin_lcd_yuv_dma {
	__u32 buf;		/* in: buffer index */
	__u32 flags;		/* in: GOWIN_LCD_YUV_DMA_* */
	__u32 dma;		/* out: the buffer's DMA (bus) address, for a DMA master such as the IDCT core */
};
#define GOWIN_LCD_YUV_DMA_INVALIDATE	0x1	/* drop the buffer from the CPU cache (a DMA master will write it) */
#define GOWIN_LCD_YUV_DMA_WRITEBACK	0x2	/* first write the CPU's copy back (done before the invalidate) */

#define GOWIN_LCD_IOC_MAGIC		'G'
#define GOWIN_LCD_IOC_YUV_INFO		_IOR(GOWIN_LCD_IOC_MAGIC, 1, struct gowin_lcd_yuv_info)
#define GOWIN_LCD_IOC_YUV_SHOW		_IOW(GOWIN_LCD_IOC_MAGIC, 2, struct gowin_lcd_yuv_show)
#define GOWIN_LCD_IOC_RGB		_IO(GOWIN_LCD_IOC_MAGIC, 3)
#define GOWIN_LCD_IOC_FRAMES		_IOR(GOWIN_LCD_IOC_MAGIC, 4, __u32)	/* frames scanned out so far */
#define GOWIN_LCD_IOC_YUV_DMA		_IOWR(GOWIN_LCD_IOC_MAGIC, 5, struct gowin_lcd_yuv_dma)

#endif /* _UAPI_LINUX_GOWIN_LCDFB_H */
