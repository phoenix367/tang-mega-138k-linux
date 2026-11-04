// SPDX-License-Identifier: GPL-2.0
/*
 * Gowin AE350 LCD framebuffer -- fbdev driver for the scan-out video core in
 * the Tang Mega 138K Pro gateware (video/rtl; registers at 0xE8001000).
 *
 * The core reads an RGB565 framebuffer straight from DDR3 (a read port on the
 * gateware's DDR bridge, beside the CPU's memory traffic) and drives a DE-mode
 * RGB666 panel (Sipeed 5" 800x480).  Rendering is all software: fbcon and
 * write() use the fb_sys drawing helpers, mmap() users draw directly.
 *
 * The framebuffer is ordinary kernel memory (alloc_pages_exact: physically
 * contiguous) under a streaming DMA mapping.  The A25 has no coherence
 * manager and rv32 has no uncached page attribute, so the CPU draws through
 * its D-cache and the driver writes changed bytes back before the core can
 * read them: the drawing helpers report the touched area/range and it is
 * written back at once; pages written through mmap() are tracked by fbdev
 * deferred I/O and written back GOWIN_LCD_DEFER later.
 *
 * YUV 4:2:0 scan-out (gateware Build 68+, include/uapi/linux/gowin-lcdfb.h):
 * the driver also owns `nbuf` planar YUV buffers the core can display
 * directly.  They are mmap()ed past the RGB framebuffer as plain cached
 * memory (no deferred I/O: a decoder writes whole frames), and
 * GOWIN_LCD_IOC_YUV_SHOW writes one back and points the core at it; the
 * core latches the new planes at the next frame boundary, so flips never
 * tear.  GOWIN_LCD_IOC_RGB, or closing the file, restores the RGB565 screen.
 *
 * Gateware Build 71+ adds a nearest-neighbour scaler (the picture in a window
 * of the panel) and an ARGB8888 OSD layer.  GOWIN_LCD_IOC_SCALE programs the
 * window and the 16.16 steps; the OSD comes from `osd_buffers` panel-sized
 * ARGB8888 buffers mmap()ed after the YUV buffers (cached, written back by
 * GOWIN_LCD_IOC_OSD).  Returning to RGB turns both off.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/dma-map-ops.h>
#include <linux/gfp.h>
#include <linux/uaccess.h>
#include <linux/math64.h>
#include <linux/gowin-lcdfb.h>

/* ---- video core registers (video/rtl/video_csr.v) ---- */
#define LCD_CTRL	0x00	/* [0] enable  [1] test pattern          */
#define LCD_FB_BASE	0x08
#define LCD_STRIDE	0x0c
#define LCD_HTIM	0x10	/* [15:0] active  [31:16] total          */
#define LCD_HSTART	0x14
#define LCD_VTIM	0x18
#define LCD_VSTART	0x1c
#define LCD_FRAMES	0x20
#define LCD_UNDERRUN	0x24
#define LCD_CRC		0x28
#define LCD_ID		0x2c
#define LCD_ID_VALUE	0x56494430	/* "VID0" */
#define LCD_FB_BASE_U	0x30	/* YUV mode (Build 68+): U and V planes, chroma stride */
#define LCD_FB_BASE_V	0x34
#define LCD_STRIDE_C	0x38
#define LCD_SRC_SIZE	0x3c	/* scaler / OSD (Build 71+) */
#define LCD_DST_POS	0x40
#define LCD_DST_SIZE	0x44
#define LCD_STEP_X	0x48
#define LCD_STEP_Y	0x4c
#define LCD_BG		0x50
#define LCD_OSD_BASE	0x54
#define LCD_OSD_STRIDE	0x58
#define LCD_OSD_POS	0x5c
#define LCD_OSD_SIZE	0x60
#define LCD_CAPS	0x64	/* [0] YUV [1] scaler [2] OSD; 0 on older cores */

#define CTRL_ENABLE	BIT(0)
#define CTRL_YUV	BIT(2)	/* planar YUV 4:2:0 framebuffer */
#define CTRL_FULL	BIT(3)	/* full-range (JPEG) YUV */
#define CTRL_SCALE	BIT(4)	/* scaler window */
#define CTRL_OSD	BIT(5)	/* OSD layer */

/* YUV buffers hold sources up to this size for the scaler (Build 71+), whatever the panel */
static unsigned int yuv_max_w = 1024, yuv_max_h = 768;
module_param(yuv_max_w, uint, 0444);
MODULE_PARM_DESC(yuv_max_w, "widest YUV 4:2:0 source a scan-out buffer holds (default 1024)");
module_param(yuv_max_h, uint, 0444);
MODULE_PARM_DESC(yuv_max_h, "tallest YUV 4:2:0 source a scan-out buffer holds (default 768)");

#define GOWIN_LCD_MAX_OSD	4
static unsigned int osd_buffers = 2;
module_param(osd_buffers, uint, 0444);
MODULE_PARM_DESC(osd_buffers, "panel-sized ARGB8888 OSD buffers to allocate (0..4, default 2)");

#define GOWIN_LCD_MAX_YUV	8
static unsigned int yuv_buffers = 6;	/* enough for an H.264 reference set + display */
module_param(yuv_buffers, uint, 0444);
MODULE_PARM_DESC(yuv_buffers, "planar YUV 4:2:0 scan-out buffers to allocate (0..8, default 6)");

#define GOWIN_LCD_DEFER	(HZ / 50)

struct gowin_lcdfb {
	void __iomem *base;
	struct device *dev;
	void *fb;			/* screen buffer (linear map)   */
	size_t size;
	dma_addr_t fb_dma;
	u32 stride, width, height;
	u32 pseudo_palette[16];
	/* YUV scan-out buffers */
	unsigned int nyuv;
	size_t yuv_size;		/* bytes per buffer (page multiple) */
	u32 y_stride, c_stride, u_off, v_off;
	void *yuv[GOWIN_LCD_MAX_YUV];
	dma_addr_t yuv_dma[GOWIN_LCD_MAX_YUV];
	bool yuv_on;			/* the core is scanning a YUV buffer */
	/* scaler / OSD */
	u32 caps;			/* GOWIN_LCD_CAP_* */
	u32 ctrl;			/* CTRL mode bits (enable, YUV, range) */
	u32 ctrl_x;			/* CTRL_SCALE / CTRL_OSD */
	u32 src_h;			/* the scaler's source height while it is on */
	unsigned int nosd;
	size_t osd_size;
	u32 osd_stride;
	void *osd[GOWIN_LCD_MAX_OSD];
	dma_addr_t osd_dma[GOWIN_LCD_MAX_OSD];
};

static void gowin_lcdfb_ctrl(struct gowin_lcdfb *p, u32 mode)
{
	p->ctrl = mode;
	writel(p->ctrl | p->ctrl_x, p->base + LCD_CTRL);
}

/* write back [off, off + len) of the framebuffer for the scan-out */
static void gowin_lcdfb_flush(struct fb_info *info, unsigned long off, size_t len)
{
	struct gowin_lcdfb *p = info->par;

	if (off >= p->size)
		return;
	len = min_t(size_t, len, p->size - off);
	dma_sync_single_range_for_device(p->dev, p->fb_dma, off, len, DMA_TO_DEVICE);
}

static void gowin_lcdfb_damage_range(struct fb_info *info, off_t off, size_t len)
{
	gowin_lcdfb_flush(info, off, len);
}

static void gowin_lcdfb_damage_area(struct fb_info *info, u32 x, u32 y, u32 width, u32 height)
{
	u32 bpp = info->var.bits_per_pixel / 8;
	u32 row;

	for (row = y; row < y + height; row++)
		gowin_lcdfb_flush(info, row * info->fix.line_length + x * bpp, width * bpp);
}

FB_GEN_DEFAULT_DEFERRED_SYSMEM_OPS(gowin_lcdfb, gowin_lcdfb_damage_range,
				   gowin_lcdfb_damage_area)

/* mmap() writers: deferred I/O hands over the pages they touched */
static void gowin_lcdfb_deferred_io(struct fb_info *info, struct list_head *pagereflist)
{
	struct fb_deferred_io_pageref *pageref;

	list_for_each_entry(pageref, pagereflist, list)
		gowin_lcdfb_flush(info, pageref->offset, PAGE_SIZE);
}

/* ---- YUV 4:2:0 scan-out buffers ---- */
static void gowin_lcdfb_rgb(struct gowin_lcdfb *p)
{
	writel(p->fb_dma, p->base + LCD_FB_BASE);
	writel(p->stride, p->base + LCD_STRIDE);
	p->ctrl_x = 0;				/* the desktop: 1:1, no OSD */
	gowin_lcdfb_ctrl(p, CTRL_ENABLE);
	p->yuv_on = false;
}

/* the picture in a window of the panel (or 1:1 again) */
static int gowin_lcdfb_scale(struct gowin_lcdfb *p, const struct gowin_lcd_scale *sc)
{
	u64 sx, sy;

	if (!(p->caps & GOWIN_LCD_CAP_SCALE))
		return -EOPNOTSUPP;
	if (!(sc->flags & GOWIN_LCD_SCALE_ON)) {
		p->ctrl_x &= ~CTRL_SCALE;
		p->src_h = 0;
		gowin_lcdfb_ctrl(p, p->ctrl);
		return 0;
	}
	if (!sc->src_w || sc->src_w > 2048 || !sc->src_h || sc->src_h > 4095 ||
	    !sc->dst_w || !sc->dst_h || sc->dst_x + sc->dst_w > p->width || sc->dst_y + sc->dst_h > p->height)
		return -EINVAL;
	sx = div_u64((u64)sc->src_w << 16, sc->dst_w);
	sy = div_u64((u64)sc->src_h << 16, sc->dst_h);
	if (sx > 0x3fffff || sy > 0x3fffff)		/* 22-bit steps: < 64x down */
		return -EINVAL;
	writel(sc->src_h << 16 | sc->src_w, p->base + LCD_SRC_SIZE);
	writel(sc->dst_y << 16 | sc->dst_x, p->base + LCD_DST_POS);
	writel(sc->dst_h << 16 | sc->dst_w, p->base + LCD_DST_SIZE);
	writel(sx, p->base + LCD_STEP_X);
	writel(sy, p->base + LCD_STEP_Y);
	writel(sc->bg & 0xffff, p->base + LCD_BG);
	p->ctrl_x |= CTRL_SCALE;
	p->src_h = sc->src_h;
	gowin_lcdfb_ctrl(p, p->ctrl);
	return 0;
}

/* show (or hide) an OSD buffer */
static int gowin_lcdfb_osd(struct gowin_lcdfb *p, const struct gowin_lcd_osd *o)
{
	if (!(p->caps & GOWIN_LCD_CAP_OSD) || !p->nosd)
		return -EOPNOTSUPP;
	if (!(o->flags & GOWIN_LCD_OSD_ON)) {
		p->ctrl_x &= ~CTRL_OSD;
		gowin_lcdfb_ctrl(p, p->ctrl);
		return 0;
	}
	if (o->buf >= p->nosd || !o->w || !o->h || o->x + o->w > p->width || o->y + o->h > p->height)
		return -EINVAL;
	/* the CPU drew through its cache; the core reads DRAM */
	if (!(o->flags & GOWIN_LCD_OSD_NOSYNC))
		dma_sync_single_range_for_device(p->dev, p->osd_dma[o->buf], 0,
						 (size_t)o->h * p->osd_stride, DMA_TO_DEVICE);
	writel(p->osd_dma[o->buf], p->base + LCD_OSD_BASE);
	writel(p->osd_stride, p->base + LCD_OSD_STRIDE);
	writel(o->y << 16 | o->x, p->base + LCD_OSD_POS);
	writel(o->h << 16 | o->w, p->base + LCD_OSD_SIZE);
	p->ctrl_x |= CTRL_OSD;
	gowin_lcdfb_ctrl(p, p->ctrl);
	return 0;
}

static int gowin_lcdfb_yuv_show(struct gowin_lcdfb *p, const struct gowin_lcd_yuv_show *sh)
{
	/* the planes must hold every line the core will fetch */
	u32 lines = max(p->height, (p->ctrl_x & CTRL_SCALE) ? p->src_h : 0), ch = (lines + 1) / 2;
	dma_addr_t b;

	if (!p->nyuv || sh->buf >= p->nyuv)
		return -EINVAL;
	if ((sh->y_off | sh->u_off | sh->v_off | sh->y_stride | sh->c_stride) & 31)
		return -EINVAL;
	if (!sh->y_stride || !sh->c_stride ||
	    sh->y_off + (size_t)sh->y_stride * lines > p->yuv_size ||
	    sh->u_off + (size_t)sh->c_stride * ch > p->yuv_size ||
	    sh->v_off + (size_t)sh->c_stride * ch > p->yuv_size)
		return -EINVAL;
	/* the decoder wrote through the cache; the core reads DRAM */
	if (!(sh->flags & GOWIN_LCD_YUV_NOSYNC))
		dma_sync_single_for_device(p->dev, p->yuv_dma[sh->buf], p->yuv_size, DMA_TO_DEVICE);
	b = p->yuv_dma[sh->buf];
	writel(b + sh->y_off, p->base + LCD_FB_BASE);
	writel(b + sh->u_off, p->base + LCD_FB_BASE_U);
	writel(b + sh->v_off, p->base + LCD_FB_BASE_V);
	writel(sh->y_stride, p->base + LCD_STRIDE);
	writel(sh->c_stride, p->base + LCD_STRIDE_C);
	gowin_lcdfb_ctrl(p, CTRL_ENABLE | CTRL_YUV | ((sh->flags & GOWIN_LCD_YUV_FULL_RANGE) ? CTRL_FULL : 0));
	p->yuv_on = true;
	return 0;
}

static int gowin_lcdfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct gowin_lcdfb *p = info->par;
	void __user *up = (void __user *)arg;

	switch (cmd) {
	case GOWIN_LCD_IOC_YUV_INFO: {
		struct gowin_lcd_yuv_info yi = {
			.nbuf = p->nyuv, .width = p->width, .height = p->height,
			.y_stride = p->y_stride, .c_stride = p->c_stride,
			.u_off = p->u_off, .v_off = p->v_off,
			.buf_size = p->yuv_size, .mmap_off = p->size,
		};
		return copy_to_user(up, &yi, sizeof(yi)) ? -EFAULT : 0;
	}
	case GOWIN_LCD_IOC_YUV_SHOW: {
		struct gowin_lcd_yuv_show sh;

		if (copy_from_user(&sh, up, sizeof(sh)))
			return -EFAULT;
		return gowin_lcdfb_yuv_show(p, &sh);
	}
	case GOWIN_LCD_IOC_YUV_DMA: {
		struct gowin_lcd_yuv_dma d;

		if (copy_from_user(&d, up, sizeof(d)))
			return -EFAULT;
		if (d.buf >= p->nyuv)
			return -EINVAL;
		/* a DMA master (the IDCT core) will write the buffer: stale CPU lines must
		 * neither be read back nor written back over its pixels */
		if (d.flags & GOWIN_LCD_YUV_DMA_WRITEBACK)
			dma_sync_single_for_device(p->dev, p->yuv_dma[d.buf], p->yuv_size, DMA_TO_DEVICE);
		if (d.flags & GOWIN_LCD_YUV_DMA_INVALIDATE)
			dma_sync_single_for_cpu(p->dev, p->yuv_dma[d.buf], p->yuv_size, DMA_FROM_DEVICE);
		d.dma = p->yuv_dma[d.buf];
		return copy_to_user(up, &d, sizeof(d)) ? -EFAULT : 0;
	}
	case GOWIN_LCD_IOC_RGB:
		gowin_lcdfb_rgb(p);
		return 0;
	case GOWIN_LCD_IOC_FRAMES: {
		u32 f = readl(p->base + LCD_FRAMES);

		return put_user(f, (u32 __user *)up);
	}
	case GOWIN_LCD_IOC_CAPS: {
		struct gowin_lcd_caps c = {
			.caps = p->caps, .osd_nbuf = p->nosd, .osd_stride = p->osd_stride,
			.osd_buf_size = p->osd_size, .osd_mmap_off = p->size + p->nyuv * p->yuv_size,
			.max_src_w = (p->caps & GOWIN_LCD_CAP_SCALE) ? 2048 : p->width,
		};
		return copy_to_user(up, &c, sizeof(c)) ? -EFAULT : 0;
	}
	case GOWIN_LCD_IOC_SCALE: {
		struct gowin_lcd_scale sc;

		if (copy_from_user(&sc, up, sizeof(sc)))
			return -EFAULT;
		return gowin_lcdfb_scale(p, &sc);
	}
	case GOWIN_LCD_IOC_OSD: {
		struct gowin_lcd_osd o;

		if (copy_from_user(&o, up, sizeof(o)))
			return -EFAULT;
		return gowin_lcdfb_osd(p, &o);
	}
	default:
		return -ENOTTY;
	}
}

/* the RGB framebuffer through deferred I/O; YUV and OSD buffers as plain cached pages */
static int gowin_lcdfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct gowin_lcdfb *p = info->par;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long len = vma->vm_end - vma->vm_start;
	unsigned int i;

	if (off < p->size) {
		if (off + len > p->size)
			return -EINVAL;
		return fb_deferred_io_mmap(info, vma);
	}
	off -= p->size;
	if (off < p->nyuv * p->yuv_size) {
		i = off / p->yuv_size;
		off -= i * p->yuv_size;
		if (off + len > p->yuv_size)
			return -EINVAL;
		vma->vm_pgoff = 0;
		return remap_pfn_range(vma, vma->vm_start,
				       page_to_pfn(virt_to_page(p->yuv[i] + off)), len, vma->vm_page_prot);
	}
	off -= p->nyuv * p->yuv_size;
	if (!p->osd_size)
		return -EINVAL;
	i = off / p->osd_size;
	off -= i * p->osd_size;
	if (i >= p->nosd || off + len > p->osd_size)
		return -EINVAL;
	vma->vm_pgoff = 0;
	return remap_pfn_range(vma, vma->vm_start,
			       page_to_pfn(virt_to_page(p->osd[i] + off)), len, vma->vm_page_prot);
}

/* a player that dies (or just exits) must not leave its last frame on the panel */
static int gowin_lcdfb_release(struct fb_info *info, int user)
{
	struct gowin_lcdfb *p = info->par;

	if (user && (p->yuv_on || p->ctrl_x))
		gowin_lcdfb_rgb(p);
	return 0;
}

static void gowin_lcdfb_yuv_free(struct gowin_lcdfb *p)
{
	unsigned int i;

	for (i = 0; i < p->nyuv; i++) {
		dma_unmap_single(p->dev, p->yuv_dma[i], p->yuv_size, DMA_TO_DEVICE);
		free_pages_exact(p->yuv[i], p->yuv_size);
	}
	p->nyuv = 0;
}

/* OSD buffers (Build 71+): panel-sized ARGB8888, transparent */
static void gowin_lcdfb_osd_init(struct gowin_lcdfb *p)
{
	unsigned int n = min(osd_buffers, (unsigned int)GOWIN_LCD_MAX_OSD), i;
	u8 *buf;

	if (!(p->caps & GOWIN_LCD_CAP_OSD) || !n)
		return;
	p->osd_stride = ALIGN(p->width * 4, 32);
	p->osd_size = PAGE_ALIGN(p->osd_stride * p->height);
	for (i = 0; i < n; i++) {
		buf = alloc_pages_exact(p->osd_size, GFP_KERNEL | __GFP_ZERO);
		if (!buf)
			break;
		p->osd_dma[i] = dma_map_single(p->dev, buf, p->osd_size, DMA_TO_DEVICE);
		if (dma_mapping_error(p->dev, p->osd_dma[i])) {
			free_pages_exact(buf, p->osd_size);
			break;
		}
		p->osd[i] = buf;
		p->nosd = i + 1;
	}
	dev_info(p->dev, "scaler%s, %u ARGB8888 OSD buffers of %zu KB\n",
		 (p->caps & GOWIN_LCD_CAP_SCALE) ? "" : " absent", p->nosd, p->osd_size / 1024);
}

static void gowin_lcdfb_osd_free(struct gowin_lcdfb *p)
{
	unsigned int i;

	for (i = 0; i < p->nosd; i++) {
		dma_unmap_single(p->dev, p->osd_dma[i], p->osd_size, DMA_TO_DEVICE);
		free_pages_exact(p->osd[i], p->osd_size);
	}
	p->nosd = 0;
}

/* allocate the YUV buffers if the core has the mode (its U-base register exists) */
static void gowin_lcdfb_yuv_init(struct gowin_lcdfb *p)
{
	unsigned int n = min(yuv_buffers, (unsigned int)GOWIN_LCD_MAX_YUV), i;
	u8 *buf;

	writel(0x12345680, p->base + LCD_FB_BASE_U);
	if (readl(p->base + LCD_FB_BASE_U) != 0x12345680 || !n) {
		dev_info(p->dev, "no YUV scan-out mode in this gateware\n");
		return;
	}
	writel(0, p->base + LCD_FB_BASE_U);
	p->y_stride = ALIGN(p->width, 32);
	p->c_stride = ALIGN(p->width / 2, 32);
	p->u_off = p->y_stride * p->height;
	p->v_off = p->u_off + p->c_stride * (p->height / 2);
	p->yuv_size = PAGE_ALIGN(p->v_off + p->c_stride * (p->height / 2));
	/* room for a larger source the scaler shrinks to the panel (own layout per stream) */
	if (readl(p->base + LCD_CAPS) & GOWIN_LCD_CAP_SCALE)
		p->yuv_size = max_t(size_t, p->yuv_size,
				    PAGE_ALIGN(ALIGN(yuv_max_w, 32) * yuv_max_h +
					       2 * ALIGN(yuv_max_w / 2, 32) * ((yuv_max_h + 1) / 2)));
	for (i = 0; i < n; i++) {
		buf = alloc_pages_exact(p->yuv_size, GFP_KERNEL);
		if (!buf)
			break;
		/* black in both ranges: Y 0, chroma 128 */
		memset(buf, 0, p->u_off);
		memset(buf + p->u_off, 128, p->yuv_size - p->u_off);
		p->yuv_dma[i] = dma_map_single(p->dev, buf, p->yuv_size, DMA_TO_DEVICE);
		if (dma_mapping_error(p->dev, p->yuv_dma[i])) {
			free_pages_exact(buf, p->yuv_size);
			break;
		}
		p->yuv[i] = buf;
		p->nyuv = i + 1;
	}
	dev_info(p->dev, "%u YUV 4:2:0 scan-out buffers of %zu KB\n", p->nyuv, p->yuv_size / 1024);
}

static int gowin_lcdfb_setcolreg(unsigned int regno, unsigned int red, unsigned int green,
				 unsigned int blue, unsigned int transp, struct fb_info *info)
{
	u32 *pal = info->pseudo_palette;

	if (regno >= 16)
		return -EINVAL;
	/* 16-bit colour components -> RGB565 (fbcon's palette) */
	pal[regno] = ((red >> 11) << 11) | ((green >> 10) << 5) | (blue >> 11);
	return 0;
}

static const struct fb_ops gowin_lcdfb_ops = {
	.owner		= THIS_MODULE,
	__FB_DEFAULT_DEFERRED_OPS_RDWR(gowin_lcdfb),
	__FB_DEFAULT_DEFERRED_OPS_DRAW(gowin_lcdfb),
	.fb_mmap	= gowin_lcdfb_mmap,
	.fb_ioctl	= gowin_lcdfb_ioctl,
	.fb_release	= gowin_lcdfb_release,
	.fb_setcolreg	= gowin_lcdfb_setcolreg,
};

static struct fb_deferred_io gowin_lcdfb_defio = {
	.delay		= GOWIN_LCD_DEFER,
	.deferred_io	= gowin_lcdfb_deferred_io,
};

static u32 gowin_lcdfb_prop(struct device *dev, const char *name, u32 def)
{
	u32 v;

	return of_property_read_u32(dev->of_node, name, &v) ? def : v;
}

static int gowin_lcdfb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	u32 width, height, hfp, hbp, vfp, vbp, stride;
	struct gowin_lcdfb *p;
	struct fb_info *info;
	void __iomem *base;
	int ret;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);
	if (readl(base + LCD_ID) != LCD_ID_VALUE)
		return dev_err_probe(dev, -ENODEV, "no video core (ID %08x)\n",
				     readl(base + LCD_ID));

	/* RISC-V treats DT devices as DMA-coherent; the A25 is not */
	if (dev_is_dma_coherent(dev))
		dev->dma_coherent = false;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/* panel: DE mode, so the porches are all the timing there is */
	width  = gowin_lcdfb_prop(dev, "width", 800);
	height = gowin_lcdfb_prop(dev, "height", 480);
	hfp    = gowin_lcdfb_prop(dev, "hfront-porch", 210);
	hbp    = gowin_lcdfb_prop(dev, "hback-porch", 182);
	vfp    = gowin_lcdfb_prop(dev, "vfront-porch", 45);
	vbp    = gowin_lcdfb_prop(dev, "vback-porch", 8);
	/* the core fetches 32 B at a time, up to 1024 pixels a line; YUV mode wants 32-pixel multiples */
	if (!width || width > 1024 || width % 32 || !height || height > 32767)
		return dev_err_probe(dev, -EINVAL, "unsupported size %ux%u\n", width, height);
	stride = width * 2;

	info = framebuffer_alloc(sizeof(*p), dev);
	if (!info)
		return -ENOMEM;
	p = info->par;
	p->base = base;
	p->dev = dev;
	p->stride = stride;
	p->width = width;
	p->height = height;
	p->size = PAGE_ALIGN(stride * height);
	p->fb = alloc_pages_exact(p->size, GFP_KERNEL | __GFP_ZERO);
	if (!p->fb) {
		ret = -ENOMEM;
		goto err_info;
	}
	p->fb_dma = dma_map_single(dev, p->fb, p->size, DMA_TO_DEVICE);	/* writes back the zeros */
	if (dma_mapping_error(dev, p->fb_dma)) {
		ret = -ENOMEM;
		goto err_pages;
	}

	strscpy(info->fix.id, "gowin-lcdfb", sizeof(info->fix.id));
	info->fix.smem_start  = page_to_phys(virt_to_page(p->fb));
	info->fix.smem_len    = p->size;
	info->fix.type        = FB_TYPE_PACKED_PIXELS;
	info->fix.visual      = FB_VISUAL_TRUECOLOR;
	info->fix.line_length = stride;
	info->fix.accel       = FB_ACCEL_NONE;

	info->var.xres = info->var.xres_virtual = width;
	info->var.yres = info->var.yres_virtual = height;
	info->var.bits_per_pixel = 16;
	info->var.red   = (struct fb_bitfield){ .offset = 11, .length = 5 };
	info->var.green = (struct fb_bitfield){ .offset = 5,  .length = 6 };
	info->var.blue  = (struct fb_bitfield){ .offset = 0,  .length = 5 };
	info->var.activate     = FB_ACTIVATE_NOW;
	info->var.height       = -1;
	info->var.width        = -1;
	info->var.pixclock     = 30000;			/* ps: 33.3 MHz */
	info->var.left_margin  = hbp;
	info->var.right_margin = hfp;
	info->var.upper_margin = vbp;
	info->var.lower_margin = vfp;
	info->var.vmode        = FB_VMODE_NONINTERLACED;

	info->fbops = &gowin_lcdfb_ops;
	info->flags = FBINFO_VIRTFB;
	info->screen_buffer = p->fb;
	info->screen_size = p->size;
	info->pseudo_palette = p->pseudo_palette;
	info->fbdefio = &gowin_lcdfb_defio;
	ret = fb_deferred_io_init(info);
	if (ret)
		goto err_unmap;

	/* program the core and start scanning out (the screen is black) */
	writel(0, base + LCD_CTRL);
	writel(p->fb_dma, base + LCD_FB_BASE);
	writel(stride, base + LCD_STRIDE);
	writel(((hbp + width + hfp) << 16) | width, base + LCD_HTIM);
	writel(hbp, base + LCD_HSTART);
	writel(((vbp + height + vfp) << 16) | height, base + LCD_VTIM);
	writel(vbp, base + LCD_VSTART);
	gowin_lcdfb_ctrl(p, CTRL_ENABLE);

	gowin_lcdfb_yuv_init(p);
	p->caps = (readl(base + LCD_CAPS) & (GOWIN_LCD_CAP_SCALE | GOWIN_LCD_CAP_OSD)) |
		  (p->nyuv ? GOWIN_LCD_CAP_YUV : 0);
	gowin_lcdfb_osd_init(p);

	ret = register_framebuffer(info);
	if (ret)
		goto err_stop;
	platform_set_drvdata(pdev, info);
	fb_info(info, "%ux%u RGB565 at %pad, %u Hz\n", width, height, &p->fb_dma,
		33333333 / ((hbp + width + hfp) * (vbp + height + vfp)));
	return 0;

err_stop:
	writel(0, base + LCD_CTRL);
	gowin_lcdfb_osd_free(p);
	gowin_lcdfb_yuv_free(p);
	fb_deferred_io_cleanup(info);
err_unmap:
	dma_unmap_single(dev, p->fb_dma, p->size, DMA_TO_DEVICE);
err_pages:
	free_pages_exact(p->fb, p->size);
err_info:
	framebuffer_release(info);
	return ret;
}

static void gowin_lcdfb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct gowin_lcdfb *p = info->par;

	unregister_framebuffer(info);
	writel(0, p->base + LCD_CTRL);
	gowin_lcdfb_osd_free(p);
	gowin_lcdfb_yuv_free(p);
	fb_deferred_io_cleanup(info);
	dma_unmap_single(p->dev, p->fb_dma, p->size, DMA_TO_DEVICE);
	free_pages_exact(p->fb, p->size);
	framebuffer_release(info);
}

static const struct of_device_id gowin_lcdfb_of_match[] = {
	{ .compatible = "gowin,ae350-lcd" },
	{ }
};
MODULE_DEVICE_TABLE(of, gowin_lcdfb_of_match);

static struct platform_driver gowin_lcdfb_driver = {
	.probe	= gowin_lcdfb_probe,
	.remove	= gowin_lcdfb_remove,
	.driver	= {
		.name		= "gowin-lcdfb",
		.of_match_table	= gowin_lcdfb_of_match,
	},
};
module_platform_driver(gowin_lcdfb_driver);

MODULE_DESCRIPTION("Gowin AE350 LCD framebuffer (software rendering)");
MODULE_LICENSE("GPL");
