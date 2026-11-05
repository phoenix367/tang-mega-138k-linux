// SPDX-License-Identifier: GPL-2.0
/*
 * gowin-idct: the hardware 8x8 inverse DCT / frame assembly core of the Tang
 * Mega 138K Pro AE350 gateware (src/rtl/idct/), a misc device.
 *
 * The core reads coefficient records from a ring in DRAM (one header word
 * and the coefficients, four int16 per 64-bit word), transforms them with
 * FFmpeg's integer IDCT and writes the pixels into YUV 4:2:0 planes as whole
 * 16/8-line bands.  This driver owns the ring (physically contiguous pages,
 * mmap()ed cached to one user at a time), writes it back from the CPU cache
 * when the user SUBMITs new records (the A25 has no coherence manager), and
 * sleeps on the core's done interrupt in WAIT.  Nothing in it is specific to
 * the MJPEG decoder that uses it (src/apps/vidplay).
 */
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/log2.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <uapi/linux/gowin-idct.h>

#define REG_CTRL	0x00
#define   CTRL_ENABLE		BIT(0)
#define   CTRL_RESET		BIT(1)
#define   CTRL_IRQ_DONE		BIT(2)
#define REG_STATUS	0x04
#define   ST_BUSY		BIT(0)
#define   ST_DONE		BIT(1)
#define   ST_BAND		BIT(2)
#define   ST_BAD		BIT(3)
#define   ST_WIDE		BIT(9)	/* frames up to 2048 wide (older cores: 1024) */
#define REG_RING_BASE	0x08
#define REG_RING_MASK	0x0c
#define REG_TAIL	0x10
#define REG_HEAD	0x14
#define REG_Y_BASE	0x18
#define REG_U_BASE	0x1c
#define REG_V_BASE	0x20
#define REG_STRIDES	0x24
#define REG_SIZE	0x28
#define REG_BLOCKS	0x2c
#define REG_BANDS	0x30
#define REG_ID		0x34
#define ID_IDCT		0x49444354

static unsigned int ring_kb = 2048;
module_param(ring_kb, uint, 0444);
MODULE_PARM_DESC(ring_kb, "record ring size in KB (power of two, default 2048)");

struct gowin_idct {
	struct device *dev;
	struct miscdevice misc;
	void __iomem *base;
	int irq;
	void *ring;
	dma_addr_t ring_dma;
	size_t ring_size;		/* allocated */
	u32 ring_used;			/* the core's ring: RING_MASK + 1 */
	u32 tail;			/* what the driver last handed to the core */
	struct mutex lock;		/* one opener; serialises the ioctls */
	bool open;
	wait_queue_head_t wq;
	u32 irqs;
	u32 max_width;			/* luma pixels a band line holds */
};

static inline u32 rd(struct gowin_idct *p, u32 r) { return readl(p->base + r); }
static inline void wr(struct gowin_idct *p, u32 r, u32 v) { writel(v, p->base + r); }

static irqreturn_t gowin_idct_irq(int irq, void *data)
{
	struct gowin_idct *p = data;
	u32 st = rd(p, REG_STATUS) & (ST_DONE | ST_BAND);

	if (!st)
		return IRQ_NONE;
	wr(p, REG_STATUS, st);
	p->irqs++;
	wake_up_interruptible(&p->wq);
	return IRQ_HANDLED;
}

static void gowin_idct_reset(struct gowin_idct *p)
{
	wr(p, REG_CTRL, CTRL_RESET);
	wr(p, REG_STATUS, ST_DONE | ST_BAND | ST_BAD);
	wr(p, REG_RING_BASE, p->ring_dma);
	wr(p, REG_RING_MASK, p->ring_used - 1);
	wr(p, REG_TAIL, 0);
	p->tail = 0;
	wr(p, REG_CTRL, CTRL_ENABLE | CTRL_IRQ_DONE);
}

static bool gowin_idct_idle(struct gowin_idct *p)
{
	return !(rd(p, REG_STATUS) & ST_BUSY) && rd(p, REG_HEAD) == p->tail;
}

/* idle = everything up to tail transformed and flushed; -ETIMEDOUT otherwise */
static int gowin_idct_wait(struct gowin_idct *p, unsigned int timeout_ms)
{
	long t = msecs_to_jiffies(timeout_ms ? timeout_ms : 1000);
	int ret;

	while (!gowin_idct_idle(p)) {
		ret = wait_event_interruptible_timeout(p->wq, gowin_idct_idle(p), t);
		if (ret == -ERESTARTSYS)
			return ret;
		if (ret == 0 && !gowin_idct_idle(p))
			return -ETIMEDOUT;
	}
	return 0;
}

static int gowin_idct_open(struct inode *inode, struct file *file)
{
	struct gowin_idct *p = container_of(file->private_data, struct gowin_idct, misc);

	mutex_lock(&p->lock);
	if (p->open) {
		mutex_unlock(&p->lock);
		return -EBUSY;
	}
	p->open = true;
	p->ring_used = p->ring_size;
	gowin_idct_reset(p);
	mutex_unlock(&p->lock);
	file->private_data = p;
	return 0;
}

static int gowin_idct_release(struct inode *inode, struct file *file)
{
	struct gowin_idct *p = file->private_data;

	mutex_lock(&p->lock);
	gowin_idct_wait(p, 200);		/* let a flush in flight land */
	wr(p, REG_CTRL, CTRL_RESET);
	p->open = false;
	mutex_unlock(&p->lock);
	return 0;
}

static long gowin_idct_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct gowin_idct *p = file->private_data;
	void __user *up = (void __user *)arg;
	int ret = 0;

	mutex_lock(&p->lock);
	switch (cmd) {
	case GOWIN_IDCT_IOC_INFO: {
		struct gowin_idct_info info = {
			.ring_size = p->ring_size, .id = rd(p, REG_ID), .max_width = p->max_width,
			.reserved = p->ring_used,
		};
		ret = copy_to_user(up, &info, sizeof(info)) ? -EFAULT : 0;
		break;
	}
	case GOWIN_IDCT_IOC_SETUP: {
		struct gowin_idct_setup s;

		if (copy_from_user(&s, up, sizeof(s))) {
			ret = -EFAULT;
			break;
		}
		if ((s.y_base | s.u_base | s.v_base | s.y_stride | s.c_stride) & 7 ||
		    !s.y_stride || !s.c_stride || !s.width || !s.height ||
		    s.width & 15 || s.width > p->max_width || s.height > 4095 ||
		    s.y_stride < s.width || s.c_stride < s.width / 2) {
			ret = -EINVAL;
			break;
		}
		ret = gowin_idct_wait(p, 1000);	/* the planes must not change under a flush */
		if (ret)
			break;
		wr(p, REG_Y_BASE, s.y_base);
		wr(p, REG_U_BASE, s.u_base);
		wr(p, REG_V_BASE, s.v_base);
		wr(p, REG_STRIDES, (s.c_stride << 16) | (s.y_stride & 0xffff));
		wr(p, REG_SIZE, (s.height << 16) | s.width);
		break;
	}
	case GOWIN_IDCT_IOC_SUBMIT: {
		struct gowin_idct_submit sb;
		u32 from, to;

		if (copy_from_user(&sb, up, sizeof(sb))) {
			ret = -EFAULT;
			break;
		}
		if (sb.tail & 7 || sb.tail >= p->ring_used) {
			ret = -EINVAL;
			break;
		}
		from = p->tail;
		to = sb.tail;
		if (to != from) {
			/* the records are in the CPU cache; the core reads DRAM */
			if (to > from) {
				dma_sync_single_for_device(p->dev, p->ring_dma + from, to - from, DMA_TO_DEVICE);
			} else {
				dma_sync_single_for_device(p->dev, p->ring_dma + from, p->ring_used - from, DMA_TO_DEVICE);
				if (to)
					dma_sync_single_for_device(p->dev, p->ring_dma, to, DMA_TO_DEVICE);
			}
			p->tail = to;
			wr(p, REG_TAIL, to);
		}
		sb.head = rd(p, REG_HEAD);
		ret = copy_to_user(up, &sb, sizeof(sb)) ? -EFAULT : 0;
		break;
	}
	case GOWIN_IDCT_IOC_WAIT: {
		u32 ms;

		if (get_user(ms, (u32 __user *)up)) {
			ret = -EFAULT;
			break;
		}
		ret = gowin_idct_wait(p, ms);
		if (!ret)
			ret = put_user(rd(p, REG_HEAD), (u32 __user *)up);
		break;
	}
	case GOWIN_IDCT_IOC_STATUS: {
		struct gowin_idct_status st = {
			.status = rd(p, REG_STATUS), .head = rd(p, REG_HEAD), .tail = p->tail,
			.blocks = rd(p, REG_BLOCKS), .bands = rd(p, REG_BANDS), .irqs = p->irqs,
		};
		ret = copy_to_user(up, &st, sizeof(st)) ? -EFAULT : 0;
		break;
	}
	case GOWIN_IDCT_IOC_RESET:
		gowin_idct_reset(p);
		break;
	case GOWIN_IDCT_IOC_RING: {
		u32 n;

		if (get_user(n, (u32 __user *)up)) {
			ret = -EFAULT;
			break;
		}
		if (n < 4096 || n > p->ring_size || !is_power_of_2(n)) {
			ret = -EINVAL;
			break;
		}
		gowin_idct_wait(p, 200);
		p->ring_used = n;
		gowin_idct_reset(p);
		break;
	}
	default:
		ret = -ENOTTY;
	}
	mutex_unlock(&p->lock);
	return ret;
}

/* the ring, as plain cached pages */
static int gowin_idct_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct gowin_idct *p = file->private_data;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long len = vma->vm_end - vma->vm_start;

	if (off + len > p->ring_size)
		return -EINVAL;
	vma->vm_pgoff = 0;
	return remap_pfn_range(vma, vma->vm_start, page_to_pfn(virt_to_page(p->ring + off)),
			       len, vma->vm_page_prot);
}

static const struct file_operations gowin_idct_fops = {
	.owner = THIS_MODULE,
	.open = gowin_idct_open,
	.release = gowin_idct_release,
	.unlocked_ioctl = gowin_idct_ioctl,
	.mmap = gowin_idct_mmap,
};

static int gowin_idct_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gowin_idct *p;
	u32 id;
	int ret;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->dev = dev;
	mutex_init(&p->lock);
	init_waitqueue_head(&p->wq);
	p->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);
	id = rd(p, REG_ID);
	if (id != ID_IDCT) {
		dev_err(dev, "no IDCT core (ID %08x)\n", id);
		return -ENODEV;
	}
	p->irq = platform_get_irq(pdev, 0);
	if (p->irq < 0)
		return p->irq;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	p->ring_size = roundup_pow_of_two(clamp(ring_kb, 64U, 16384U) * 1024);
	p->ring_used = p->ring_size;
	p->ring = alloc_pages_exact(p->ring_size, GFP_KERNEL | __GFP_ZERO);
	if (!p->ring)
		return -ENOMEM;
	p->ring_dma = dma_map_single(dev, p->ring, p->ring_size, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, p->ring_dma)) {
		free_pages_exact(p->ring, p->ring_size);
		return -ENOMEM;
	}

	wr(p, REG_CTRL, CTRL_RESET);
	wr(p, REG_STATUS, ST_DONE | ST_BAND | ST_BAD);
	p->max_width = (rd(p, REG_STATUS) & ST_WIDE) ? 2048 : 1024;
	ret = devm_request_irq(dev, p->irq, gowin_idct_irq, 0, dev_name(dev), p);
	if (ret)
		goto err_ring;

	p->misc.minor = MISC_DYNAMIC_MINOR;
	p->misc.name = "gowin-idct";
	p->misc.fops = &gowin_idct_fops;
	p->misc.parent = dev;
	ret = misc_register(&p->misc);
	if (ret)
		goto err_ring;
	platform_set_drvdata(pdev, p);
	dev_info(dev, "IDCT core, frames up to %u wide, %zu KB record ring, irq %d\n",
		 p->max_width, p->ring_size / 1024, p->irq);
	return 0;

err_ring:
	dma_unmap_single(dev, p->ring_dma, p->ring_size, DMA_TO_DEVICE);
	free_pages_exact(p->ring, p->ring_size);
	return ret;
}

static void gowin_idct_remove(struct platform_device *pdev)
{
	struct gowin_idct *p = platform_get_drvdata(pdev);

	misc_deregister(&p->misc);
	wr(p, REG_CTRL, CTRL_RESET);
	dma_unmap_single(p->dev, p->ring_dma, p->ring_size, DMA_TO_DEVICE);
	free_pages_exact(p->ring, p->ring_size);
}

static const struct of_device_id gowin_idct_of_match[] = {
	{ .compatible = "gowin,ae350-idct" },
	{ }
};
MODULE_DEVICE_TABLE(of, gowin_idct_of_match);

static struct platform_driver gowin_idct_driver = {
	.probe = gowin_idct_probe,
	.remove = gowin_idct_remove,
	.driver = {
		.name = "gowin-idct",
		.of_match_table = gowin_idct_of_match,
	},
};
module_platform_driver(gowin_idct_driver);

MODULE_DESCRIPTION("Gowin AE350 hardware IDCT core");
MODULE_LICENSE("GPL");
