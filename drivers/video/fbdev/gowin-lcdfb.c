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
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/dma-map-ops.h>
#include <linux/gfp.h>

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

#define CTRL_ENABLE	BIT(0)

#define GOWIN_LCD_DEFER	(HZ / 50)

struct gowin_lcdfb {
	void __iomem *base;
	struct device *dev;
	void *fb;			/* screen buffer (linear map)   */
	size_t size;
	dma_addr_t fb_dma;
	u32 pseudo_palette[16];
};

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
	FB_DEFAULT_DEFERRED_OPS(gowin_lcdfb),
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
	/* the core fetches 16 B (8 pixels) at a time, up to 1024 pixels a line */
	if (!width || width > 1024 || width % 8 || !height || height > 32767)
		return dev_err_probe(dev, -EINVAL, "unsupported size %ux%u\n", width, height);
	stride = width * 2;

	info = framebuffer_alloc(sizeof(*p), dev);
	if (!info)
		return -ENOMEM;
	p = info->par;
	p->base = base;
	p->dev = dev;
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
	writel(CTRL_ENABLE, base + LCD_CTRL);

	ret = register_framebuffer(info);
	if (ret)
		goto err_stop;
	platform_set_drvdata(pdev, info);
	fb_info(info, "%ux%u RGB565 at %pad, %u Hz\n", width, height, &p->fb_dma,
		33333333 / ((hbp + width + hfp) * (vbp + height + vfp)));
	return 0;

err_stop:
	writel(0, base + LCD_CTRL);
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
