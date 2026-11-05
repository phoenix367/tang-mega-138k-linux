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

/*
 * Scaler and OSD (gateware Build 71+, GOWIN_LCD_IOC_CAPS).  The scaler shows the
 * picture -- the YUV planes YUV_SHOW points at, src_w x src_h of them -- in a
 * window of the panel, nearest neighbour, any factor (YUV sources up to 2048
 * pixels wide), the bg colour around it.  The OSD is an ARGB8888 layer (a
 * little-endian 0xAARRGGBB word per pixel, straight alpha) from one of the
 * driver's OSD buffers, blended over the picture in its own window; buffer
 * pixel (0, 0) lands at (x, y).  Both apply from the next frame; GOWIN_LCD_IOC_RGB
 * and closing the file turn both off.  With the scaler the YUV buffers hold sources
 * larger than the panel (buf_size; module parameters yuv_max_w / yuv_max_h, default
 * 1280 x 768, 1024 x 768 before kernel #93): the caller lays the planes out itself
 * and passes them to YUV_SHOW, which then checks them for the scaler's source height.
 */
#define GOWIN_LCD_CAP_YUV		0x1
#define GOWIN_LCD_CAP_SCALE		0x2
#define GOWIN_LCD_CAP_OSD		0x4

struct gowin_lcd_caps {
	__u32 caps;		/* GOWIN_LCD_CAP_* */
	__u32 osd_nbuf;		/* OSD buffers, each panel-sized ARGB8888 */
	__u32 osd_stride;	/* bytes per OSD buffer line */
	__u32 osd_buf_size;	/* bytes per OSD buffer (page multiple) */
	__u32 osd_mmap_off;	/* mmap() offset of OSD buffer 0; buffer i at osd_mmap_off + i * osd_buf_size */
	__u32 max_src_w;	/* scaler: widest YUV source (RGB: half of it) */
	__u32 reserved[2];
};

struct gowin_lcd_scale {
	__u32 flags;		/* GOWIN_LCD_SCALE_ON; without it 1:1 over the whole panel */
	__u32 src_w, src_h;	/* the source picture */
	__u32 dst_x, dst_y, dst_w, dst_h;	/* its window on the panel */
	__u32 bg;		/* RGB565 around the window */
};
#define GOWIN_LCD_SCALE_ON		0x1

struct gowin_lcd_osd {
	__u32 flags;		/* GOWIN_LCD_OSD_ON (else the OSD is off), GOWIN_LCD_OSD_NOSYNC */
	__u32 buf;		/* OSD buffer index */
	__u32 x, y, w, h;	/* window on the panel, at most the panel size */
};
#define GOWIN_LCD_OSD_ON		0x1
#define GOWIN_LCD_OSD_NOSYNC		0x2	/* skip writing the buffer back from the CPU cache */

#define GOWIN_LCD_IOC_CAPS		_IOR(GOWIN_LCD_IOC_MAGIC, 6, struct gowin_lcd_caps)
#define GOWIN_LCD_IOC_SCALE		_IOW(GOWIN_LCD_IOC_MAGIC, 7, struct gowin_lcd_scale)
#define GOWIN_LCD_IOC_OSD		_IOW(GOWIN_LCD_IOC_MAGIC, 8, struct gowin_lcd_osd)

#endif /* _UAPI_LINUX_GOWIN_LCDFB_H */
