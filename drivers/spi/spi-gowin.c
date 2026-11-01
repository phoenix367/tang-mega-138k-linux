// SPDX-License-Identifier: GPL-2.0
/*
 * Gowin AE350 (Tang Mega 138K Pro) gateware SPI master -- spi_controller
 * driver for sdspi_core (sdspi/rtl, registers at 0xE8010000), the SPI-mode
 * host of the Dock's microSD slot.  Used under the mainline mmc_spi host.
 *
 * The core has a 2 KB TX buffer and a 2 KB RX buffer: a transfer is "fill
 * TX, write LEN, poll busy, read RX".  Chip select is a register bit.
 * SCK = bus clock / (2 * (DIV + 1)).  No interrupt: transfers are short
 * (a 2 KB chunk is 0.66 ms at 25 MHz) and the engine is polled.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/unaligned.h>
#include <linux/spi/spi.h>

#define REG_CTRL	0x0000
#define  CTRL_CS_N	BIT(0)
#define  CTRL_DIV(d)	(((d) & 0xff) << 8)
#define  CTRL_DISCARD	BIT(16)
#define REG_STATUS	0x0004
#define  ST_BUSY	BIT(0)
#define  ST_CARD	BIT(1)
#define REG_LEN		0x0008
#define REG_ID		0x000C
#define  ID_SDSP	0x53445350
#define TX_BUF		0x0800
#define RX_BUF		0x1000
#define BUF_SIZE	2048

struct gowin_spi {
	void __iomem *base;
	u32 bus_hz;
	u32 ctrl;		/* current CTRL value (cs, div) */
};



static void gowin_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct gowin_spi *gs = spi_controller_get_devdata(spi->controller);

	/* enable = the logic level of the CS line (the core already folded in
	 * SPI_CS_HIGH): true = line high = deasserted for the card */
	if (enable)
		gs->ctrl |= CTRL_CS_N;
	else
		gs->ctrl &= ~CTRL_CS_N;
	writel(gs->ctrl, gs->base + REG_CTRL);
}

static int gowin_spi_transfer_one(struct spi_controller *ctlr, struct spi_device *spi,
				  struct spi_transfer *t)
{
	struct gowin_spi *gs = spi_controller_get_devdata(ctlr);
	u32 hz = t->speed_hz ? t->speed_hz : spi->max_speed_hz;
	u32 div, done = 0, st;

	/* SCK = bus / (2 * (div + 1)), rounded down to <= hz */
	div = DIV_ROUND_UP(gs->bus_hz, 2 * hz);
	div = clamp_t(u32, div, 1, 256) - 1;
	gs->ctrl = (gs->ctrl & CTRL_CS_N) | CTRL_DIV(div) | (t->rx_buf ? 0 : CTRL_DISCARD);
	writel(gs->ctrl, gs->base + REG_CTRL);

	while (done < t->len) {
		u32 n = min_t(u32, t->len - done, BUF_SIZE);
		u32 words = DIV_ROUND_UP(n, 4), i;
		const u8 *tx = t->tx_buf ? (const u8 *)t->tx_buf + done : NULL;
		u8 *rx = t->rx_buf ? (u8 *)t->rx_buf + done : NULL;

		for (i = 0; i < words; i++) {
			u32 w = 0xffffffff;

			if (tx) {
				u32 left = n - 4 * i;

				if (left >= 4)
					w = get_unaligned_le32(tx + 4 * i);
				else
					memcpy(&w, tx + 4 * i, left);	/* rv32 is little-endian */
			}
			writel_relaxed(w, gs->base + TX_BUF + 4 * i);
		}
		/* writel's "fence w,o" costs ~1.6 us on the A25 (it waits for the
		 * posted write to land); fill the buffer with relaxed writes and
		 * let the single fence in the LEN write order them */
		writel(n, gs->base + REG_LEN);
		/* Short transfers (mmc_spi sends many 1..8 byte ones) spin on
		 * STATUS, one iteration ~0.2 us; long ones sleep between reads. */
		if (n * 8000 / (hz / 1000) < 400) {
			for (i = 0; i < 2000000; i++) {
				st = readl(gs->base + REG_STATUS);
				if (!(st & ST_BUSY))
					break;
				cpu_relax();
			}
		} else {
			for (i = 0; i < 20000; i++) {
				st = readl(gs->base + REG_STATUS);
				if (!(st & ST_BUSY))
					break;
				usleep_range(50, 200);
			}
		}
		if (st & ST_BUSY) {
			dev_err(&spi->dev, "transfer timeout (%u bytes)\n", n);
			return -ETIMEDOUT;
		}
		if (rx) {
			for (i = 0; i < words; i++) {
				u32 w = readl_relaxed(gs->base + RX_BUF + 4 * i);
				u32 left = n - 4 * i;

				if (left >= 4)
					put_unaligned_le32(w, rx + 4 * i);
				else
					memcpy(rx + 4 * i, &w, left);
			}
		}
		done += n;
	}
	return 0;
}

static int gowin_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *ctlr;
	struct gowin_spi *gs;
	u32 id;
	int ret;

	ctlr = devm_spi_alloc_host(dev, sizeof(*gs));
	if (!ctlr)
		return -ENOMEM;
	gs = spi_controller_get_devdata(ctlr);
	gs->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gs->base))
		return PTR_ERR(gs->base);
	id = readl(gs->base + REG_ID);
	if (id != ID_SDSP) {
		dev_err(dev, "no SPI core (ID %08x)\n", id);
		return -ENODEV;
	}
	gs->bus_hz = 50000000;
	of_property_read_u32(dev->of_node, "clock-frequency", &gs->bus_hz);
	gs->ctrl = CTRL_CS_N | CTRL_DIV(62);
	writel(gs->ctrl, gs->base + REG_CTRL);

	ctlr->dev.of_node = dev->of_node;
	ctlr->bus_num = -1;
	ctlr->num_chipselect = 1;
	/* mode 0 is what the core drives (mmc_spi asks for it); SPI_CS_HIGH is
	 * needed because mmc_spi flips it to clock the card with CS released */
	ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	ctlr->bits_per_word_mask = SPI_BPW_MASK(8);
	ctlr->min_speed_hz = gs->bus_hz / 512;
	ctlr->max_speed_hz = gs->bus_hz / 2;
	ctlr->set_cs = gowin_spi_set_cs;
	ctlr->transfer_one = gowin_spi_transfer_one;

	ret = devm_spi_register_controller(dev, ctlr);
	if (ret)
		return ret;
	dev_info(dev, "SPI master, %u kHz .. %u MHz, card %s\n", ctlr->min_speed_hz / 1000,
		 ctlr->max_speed_hz / 1000000,
		 readl(gs->base + REG_STATUS) & ST_CARD ? "inserted" : "absent");
	return 0;
}

static const struct of_device_id gowin_spi_of_match[] = {
	{ .compatible = "gowin,ae350-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, gowin_spi_of_match);

static struct platform_driver gowin_spi_driver = {
	.probe = gowin_spi_probe,
	.driver = {
		.name = "gowin-spi",
		.of_match_table = gowin_spi_of_match,
	},
};
module_platform_driver(gowin_spi_driver);

MODULE_DESCRIPTION("Gowin AE350 gateware SPI master (microSD in SPI mode)");
MODULE_LICENSE("GPL");
