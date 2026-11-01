// SPDX-License-Identifier: GPL-2.0
/*
 * Gowin AE350 (Tang Mega 138K Pro) gateware SD host -- MMC host driver for
 * sdhost_core (sdhost/rtl, registers at 0xE8020000): native 1/4-bit SD bus
 * with CRC engines and a 64-bit DMA master into DDR.
 *
 * A request is one command, optionally with BLKCNT blocks of BLKSIZE bytes
 * moved by the core's DMA to/from a contiguous bounce buffer (the SoC has
 * no coherence manager: the buffer is kmalloc'ed and streamed with
 * dma_map_single(), whose sync ops do the cache maintenance).  Completion
 * is signalled by the core's interrupt (sticky STATUS bits).  mrq->stop
 * (CMD12) is issued by the driver after the data phase.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>

#define REG_CTRL	0x00
#define  CTRL_CLK_EN	BIT(0)
#define  CTRL_BUS4	BIT(1)
#define  CTRL_RESET	BIT(2)
#define  CTRL_DIV(d)	(((d) & 0xff) << 8)
#define  CTRL_SMP_EARLY	BIT(16)			/* sample inputs on the SD rising edge (default: 20 ns later) */
#define REG_ARG		0x04
#define REG_CMD		0x08
#define  CMD_RESP48	(1 << 6)
#define  CMD_RESP136	(2 << 6)
#define  CMD_CRC	BIT(8)
#define  CMD_IDX	BIT(9)
#define  CMD_BUSY	BIT(10)
#define  CMD_DATA	BIT(11)
#define  CMD_WRITE	BIT(12)
#define REG_RESP(n)	(0x0C + 4 * (n))
#define REG_BLK		0x1C
#define REG_DMA		0x20
#define REG_STATUS	0x24
#define  ST_CMD_BUSY	BIT(0)
#define  ST_DAT_BUSY	BIT(1)
#define  ST_CARD	BIT(2)
#define  ST_CDONE	BIT(8)
#define  ST_DDONE	BIT(9)
#define  ST_CTMO	BIT(10)
#define  ST_CCRC	BIT(11)
#define  ST_CIDX	BIT(12)
#define  ST_DTMO	BIT(13)
#define  ST_DCRC	BIT(14)
#define  ST_DEND	BIT(15)
#define  ST_WST		BIT(16)
#define  ST_DMA		BIT(17)
#define  ST_FIFO	BIT(18)
#define  ST_ERRORS	(ST_CTMO | ST_CCRC | ST_CIDX | ST_DTMO | ST_DCRC | ST_DEND | ST_WST | ST_DMA | ST_FIFO)
#define  ST_ALL		GENMASK(18, 8)
#define REG_INT_EN	0x28
#define REG_TIMEOUT	0x2C
#define REG_ID		0x30
#define  ID_SDHC	0x53444843

#define BOUNCE_SIZE	(64 * 1024)

struct gowin_sd {
	struct device *dev;
	struct mmc_host *mmc;
	void __iomem *base;
	u32 bus_hz;
	int irq;
	u8 *bounce;
	struct completion done;
	u32 status;			/* sticky bits at completion */
	u32 need;			/* bits that mean "complete" */
	spinlock_t lock;
	/* clock tuning: the fastest (divider, sampling phase) that reads cleanly */
	u32 ios_clock;			/* what the MMC core asked for */
	u32 ctrl;			/* CTRL without the clock fields */
	u32 tuned_clock;		/* ios clock the tuning was done for, 0 = none */
	u32 tuned_div;
	bool tuned_early;
	bool need_tune;
};

static inline u32 sd_rd(struct gowin_sd *h, u32 r) { return readl(h->base + r); }
static inline void sd_wr(struct gowin_sd *h, u32 r, u32 v) { writel(v, h->base + r); }

static u32 clk_div(struct gowin_sd *h, u32 hz)
{
	u32 div = DIV_ROUND_UP(h->bus_hz, 2 * hz);

	return clamp_t(u32, div, 1, 256) - 1;
}

static void gowin_sd_apply_clock(struct gowin_sd *h, u32 div, bool early)
{
	u32 ctrl = h->ctrl | CTRL_CLK_EN | CTRL_DIV(div) | (early ? CTRL_SMP_EARLY : 0);

	sd_wr(h, REG_CTRL, ctrl);
	h->mmc->actual_clock = h->bus_hz / (2 * (div + 1));
}


static irqreturn_t gowin_sd_irq(int irq, void *data)
{
	struct gowin_sd *h = data;
	u32 st = sd_rd(h, REG_STATUS) & ST_ALL;

	if (!st)
		return IRQ_NONE;
	sd_wr(h, REG_STATUS, st);		/* clear what we took */
	spin_lock(&h->lock);
	h->status |= st;
	if ((h->status & ST_ERRORS) || (h->status & h->need) == h->need)
		complete(&h->done);
	spin_unlock(&h->lock);
	return IRQ_HANDLED;
}

/* wait for the engines to go idle (after an error, or a soft reset) */
static void gowin_sd_quiesce(struct gowin_sd *h)
{
	int i;

	for (i = 0; i < 100000; i++) {
		if (!(sd_rd(h, REG_STATUS) & (ST_CMD_BUSY | ST_DAT_BUSY)))
			return;
		udelay(1);
	}
	sd_wr(h, REG_CTRL, sd_rd(h, REG_CTRL) | CTRL_RESET);
	dev_warn(h->dev, "engines stuck, reset\n");
}

static int gowin_sd_error(struct gowin_sd *h, u32 st, bool data)
{
	if (st & (ST_CTMO | ST_DTMO))
		return -ETIMEDOUT;
	if (st & (ST_CCRC | ST_CIDX | ST_DCRC | ST_DEND | ST_WST))
		return -EILSEQ;
	if (st & (ST_DMA | ST_FIFO))
		return -EIO;
	return 0;
}

/* run one command (with its data, if any); returns the sticky status */
static u32 gowin_sd_do_cmd(struct gowin_sd *h, struct mmc_command *cmd, struct mmc_data *data)
{
	u32 c = cmd->opcode & 0x3f;
	unsigned long tmo;

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		break;
	case MMC_RSP_R2:
		c |= CMD_RESP136;			/* CRC is inside the CID/CSD */
		break;
	case MMC_RSP_R3:
		c |= CMD_RESP48;			/* no CRC, no index */
		break;
	case MMC_RSP_R1B:
		c |= CMD_RESP48 | CMD_CRC | CMD_IDX | CMD_BUSY;
		break;
	default:				/* R1, R6, R7 */
		c |= CMD_RESP48 | CMD_CRC | CMD_IDX;
		break;
	}
	if (data) {
		c |= CMD_DATA;
		if (data->flags & MMC_DATA_WRITE)
			c |= CMD_WRITE;
		sd_wr(h, REG_BLK, (data->blocks << 16) | data->blksz);
	}

	h->status = 0;
	h->need = ST_CDONE | (data ? ST_DDONE : 0);
	reinit_completion(&h->done);
	sd_wr(h, REG_STATUS, ST_ALL);
	sd_wr(h, REG_ARG, cmd->arg);
	sd_wr(h, REG_INT_EN, ST_ALL);
	sd_wr(h, REG_CMD, c);

	tmo = msecs_to_jiffies(data ? 2000 + data->blocks : 1000);
	if (!wait_for_completion_timeout(&h->done, tmo)) {
		dev_err(h->dev, "CMD%u: no completion (status %08x)\n", cmd->opcode, sd_rd(h, REG_STATUS));
		sd_wr(h, REG_INT_EN, 0);
		sd_wr(h, REG_CTRL, sd_rd(h, REG_CTRL) | CTRL_RESET);
		return ST_CTMO;
	}
	sd_wr(h, REG_INT_EN, 0);
	if (h->status & ST_ERRORS)
		gowin_sd_quiesce(h);
	return h->status;
}

/* one throw-away read of block 0 through the bounce buffer; true = clean */
static bool gowin_sd_probe_read(struct gowin_sd *h)
{
	struct mmc_command cmd = { .opcode = MMC_READ_SINGLE_BLOCK, .arg = 0, .flags = MMC_RSP_R1 };
	struct mmc_data data = { .blocks = 1, .blksz = 512, .flags = MMC_DATA_READ };
	dma_addr_t dma;
	u32 st;

	dma = dma_map_single(h->dev, h->bounce, 512, DMA_FROM_DEVICE);
	if (dma_mapping_error(h->dev, dma))
		return false;
	sd_wr(h, REG_DMA, dma);
	sd_wr(h, REG_TIMEOUT, 0xFFFFFF);
	st = gowin_sd_do_cmd(h, &cmd, &data);
	dma_unmap_single(h->dev, dma, 512, DMA_FROM_DEVICE);
	return (st & (ST_CDONE | ST_DDONE)) == (ST_CDONE | ST_DDONE) && !(st & ST_ERRORS);
}

/*
 * Find the fastest clock / sampling phase that reads cleanly.  Candidates,
 * from the requested clock down: late sampling first (the card's own output
 * delay is its hold margin), then on-edge sampling, then the next divider.
 * Each candidate must pass several reads in a row.
 */
static void gowin_sd_tune(struct gowin_sd *h)
{
	u32 base = clk_div(h, h->ios_clock), div;
	int phase, k;

	h->need_tune = false;
	for (div = base; div <= base + 4; div++) {
		for (phase = 0; phase < 2; phase++) {
			bool early = phase == 1;

			gowin_sd_apply_clock(h, div, early);
			usleep_range(50, 100);
			for (k = 0; k < 4; k++)
				if (!gowin_sd_probe_read(h))
					break;
			if (k == 4) {
				h->tuned_clock = h->ios_clock;
				h->tuned_div = div;
				h->tuned_early = early;
				dev_info(h->dev, "tuned: %u kHz, %s sampling%s\n", h->mmc->actual_clock / 1000,
					 early ? "on-edge" : "late", div == base ? "" : " (clock lowered)");
				return;
			}
		}
	}
	/* nothing clean: fall back to the slowest candidate, late sampling */
	h->tuned_clock = h->ios_clock;
	h->tuned_div = base + 4;
	h->tuned_early = false;
	gowin_sd_apply_clock(h, h->tuned_div, false);
	dev_warn(h->dev, "tuning found no clean setting, using %u kHz\n", h->mmc->actual_clock / 1000);
}

static void gowin_sd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct gowin_sd *h = mmc_priv(mmc);
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	dma_addr_t dma = 0;
	unsigned int len = 0;
	u32 st, i;

	if (!(sd_rd(h, REG_STATUS) & ST_CARD)) {
		cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}
	if (data) {
		len = data->blocks * data->blksz;
		if (len > BOUNCE_SIZE || (data->blksz & 7)) {
			cmd->error = -EINVAL;
			mmc_request_done(mmc, mrq);
			return;
		}
		if (data->flags & MMC_DATA_WRITE)
			sg_copy_to_buffer(data->sg, data->sg_len, h->bounce, len);
		dma = dma_map_single(h->dev, h->bounce, len,
				     data->flags & MMC_DATA_WRITE ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		if (dma_mapping_error(h->dev, dma)) {
			cmd->error = -ENOMEM;
			mmc_request_done(mmc, mrq);
			return;
		}
		sd_wr(h, REG_DMA, dma);
		sd_wr(h, REG_TIMEOUT, 0xFFFFFF);
	}

	if (h->need_tune && data && (data->flags & MMC_DATA_READ) && data->blksz == 512)
		gowin_sd_tune(h);
	st = gowin_sd_do_cmd(h, cmd, data);
	cmd->error = gowin_sd_error(h, st & ~(ST_DTMO | ST_DCRC | ST_DEND | ST_WST | ST_DMA | ST_FIFO), false);
	if (!cmd->error) {
		if (mmc_resp_type(cmd) == MMC_RSP_R2)
			for (i = 0; i < 4; i++)
				cmd->resp[i] = sd_rd(h, REG_RESP(i));
		else
			cmd->resp[0] = sd_rd(h, REG_RESP(0));
	}
	if (data) {
		dma_unmap_single(h->dev, dma, len, data->flags & MMC_DATA_WRITE ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		data->error = cmd->error ? cmd->error :
			      gowin_sd_error(h, st & (ST_DTMO | ST_DCRC | ST_DEND | ST_WST | ST_DMA | ST_FIFO), true);
		if (!data->error) {
			data->bytes_xfered = len;
			if (data->flags & MMC_DATA_READ)
				sg_copy_from_buffer(data->sg, data->sg_len, h->bounce, len);
		} else
			data->bytes_xfered = 0;
		if (mrq->stop) {
			u32 s2 = gowin_sd_do_cmd(h, mrq->stop, NULL);

			mrq->stop->error = gowin_sd_error(h, s2, false);
			if (!mrq->stop->error)
				mrq->stop->resp[0] = sd_rd(h, REG_RESP(0));
		}
	}
	mmc_request_done(mmc, mrq);
}

static void gowin_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct gowin_sd *h = mmc_priv(mmc);

	h->ctrl = ios->bus_width == MMC_BUS_WIDTH_4 ? CTRL_BUS4 : 0;
	h->ios_clock = ios->clock;
	if (!ios->clock) {
		sd_wr(h, REG_CTRL, h->ctrl);
		mmc->actual_clock = 0;
		h->tuned_clock = 0;			/* card off / re-identification: forget the tuning */
		h->need_tune = false;
		return;
	}
	if (ios->clock == h->tuned_clock) {
		gowin_sd_apply_clock(h, h->tuned_div, h->tuned_early);
	} else {
		gowin_sd_apply_clock(h, clk_div(h, ios->clock), false);
		/* identification runs at <= 400 kHz and needs no tuning */
		h->need_tune = ios->clock > 1000000;
	}
	usleep_range(100, 200);				/* let the clock run */
}

static int gowin_sd_get_cd(struct mmc_host *mmc)
{
	struct gowin_sd *h = mmc_priv(mmc);

	return !!(sd_rd(h, REG_STATUS) & ST_CARD);
}

static int gowin_sd_get_ro(struct mmc_host *mmc)
{
	return 0;
}

static const struct mmc_host_ops gowin_sd_ops = {
	.request = gowin_sd_request,
	.set_ios = gowin_sd_set_ios,
	.get_cd = gowin_sd_get_cd,
	.get_ro = gowin_sd_get_ro,
};

static int gowin_sd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mmc_host *mmc;
	struct gowin_sd *h;
	u32 id;
	int ret;

	mmc = devm_mmc_alloc_host(dev, sizeof(*h));
	if (!mmc)
		return -ENOMEM;
	h = mmc_priv(mmc);
	h->dev = dev;
	h->mmc = mmc;
	spin_lock_init(&h->lock);
	init_completion(&h->done);
	h->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(h->base))
		return PTR_ERR(h->base);
	id = sd_rd(h, REG_ID);
	if (id != ID_SDHC) {
		dev_err(dev, "no SD host core (ID %08x)\n", id);
		return -ENODEV;
	}
	h->bus_hz = 50000000;
	of_property_read_u32(dev->of_node, "clock-frequency", &h->bus_hz);
	h->irq = platform_get_irq(pdev, 0);
	if (h->irq < 0)
		return h->irq;
	h->bounce = devm_kmalloc(dev, BOUNCE_SIZE, GFP_KERNEL);
	if (!h->bounce)
		return -ENOMEM;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	sd_wr(h, REG_INT_EN, 0);
	sd_wr(h, REG_CTRL, CTRL_RESET);
	sd_wr(h, REG_STATUS, ST_ALL);
	h->ctrl = 0;
	ret = devm_request_irq(dev, h->irq, gowin_sd_irq, 0, dev_name(dev), h);
	if (ret)
		return ret;

	ret = mmc_of_parse(mmc);
	if (ret)
		return ret;
	mmc->ops = &gowin_sd_ops;
	mmc->f_min = h->bus_hz / 512;			/* 97 kHz */
	mmc->f_max = h->bus_hz / 2;			/* 25 MHz */
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps |= MMC_CAP_4_BIT_DATA | MMC_CAP_NEEDS_POLL;
	mmc->caps2 |= MMC_CAP2_NO_SDIO | MMC_CAP2_NO_MMC;
	mmc->max_blk_size = 512;
	mmc->max_blk_count = BOUNCE_SIZE / 512;
	mmc->max_req_size = BOUNCE_SIZE;
	mmc->max_segs = 128;
	mmc->max_seg_size = BOUNCE_SIZE;

	ret = mmc_add_host(mmc);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, mmc);
	dev_info(dev, "SD host, 4-bit, %u kHz..%u MHz, DMA bounce %u KB, irq %d, card %s\n",
		 mmc->f_min / 1000, mmc->f_max / 1000000, BOUNCE_SIZE / 1024, h->irq,
		 sd_rd(h, REG_STATUS) & ST_CARD ? "inserted" : "absent");
	return 0;
}

static void gowin_sd_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct gowin_sd *h = mmc_priv(mmc);

	mmc_remove_host(mmc);
	sd_wr(h, REG_INT_EN, 0);
	sd_wr(h, REG_CTRL, CTRL_RESET);
}

static const struct of_device_id gowin_sd_of_match[] = {
	{ .compatible = "gowin,ae350-sdhost" },
	{ }
};
MODULE_DEVICE_TABLE(of, gowin_sd_of_match);

static struct platform_driver gowin_sd_driver = {
	.probe = gowin_sd_probe,
	.remove = gowin_sd_remove,
	.driver = {
		.name = "gowin-sdhost",
		.of_match_table = gowin_sd_of_match,
	},
};
module_platform_driver(gowin_sd_driver);

MODULE_DESCRIPTION("Gowin AE350 gateware SD host (native 4-bit, DMA)");
MODULE_LICENSE("GPL");
