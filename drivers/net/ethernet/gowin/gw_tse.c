// SPDX-License-Identifier: GPL-2.0
/*
 * Gowin Triple-Speed Ethernet -- descriptor-ring DMA NAPI driver for the
 * gw_eth_dma engine on the AE350 SoC (Tang Mega 138K Pro Dock).
 *
 * The gateware (eth_dma/rtl/csr.v, EXTS slave at 0xE8000000) is a real DMA
 * engine: it moves frames between the MAC and a ring of fixed 2 KB DRAM buffers
 * over the SoC's EXTM master port, via a custom AHB arbiter -- no more per-frame
 * CPU PIO. RX and TX each have a power-of-two ring of 2 KB buffers; head/tail
 * counters (cumulative, mod 2^16) give producer/consumer flow control:
 *
 *   RX: DMA writes buffer (RX_HEAD & (len-1)) = [hdr{[16]err,[15:0]len}][data],
 *       bumps RX_HEAD, IRQs rx_done; CPU consumes up to RX_HEAD, bumps RX_TAIL.
 *   TX: CPU writes buffer (TX_TAIL & (len-1)) = [len][data], bumps TX_TAIL;
 *       DMA transmits, bumps TX_HEAD, IRQs tx_done.
 *
 * The A25 has no coherence manager and rv32 has no uncached page attribute, so
 * dma_alloc_coherent() would hand back plain cached memory.  The rings are
 * therefore ordinary kmalloc memory under streaming DMA mappings: an RX slot is
 * invalidated (dma_sync_*_for_cpu) before the CPU reads it, a TX slot written
 * back (dma_sync_*_for_device) before the DMA is told about it.  The cache ops
 * are the A25 CCTL ones registered in arch/riscv/mm/cmo-andes-a25.c.
 *
 * Per-frame cost matters on this core (~1.4 us DRAM miss): frames are copied
 * with word accesses (gtse_copy), TX-done interrupts are only enabled while
 * the queue is stopped, the TX doorbell honours xmit_more, and TX_HEAD is only
 * read when the ring looks full.
 *
 * NAPI runs threaded, with interrupt deferral (GTSE_DEFER_IRQS polls on a
 * GTSE_GRO_FLUSH_NS timer before the RX interrupt is re-armed).  Processed in
 * softirq context, a UDP stream above ~3k frames/s (each ~250 us of stack
 * work on this single core) starves the receiving task and the datagrams die
 * in its full socket buffer (receive livelock); as a kernel thread NAPI
 * shares the CPU, and deferral batches frames even at moderate rates.  Both
 * remain tunable in /sys/class/net/<if>/{threaded,napi_defer_hard_irqs,
 * gro_flush_timeout}.
 *
 * Devicetree is identical to the PIO driver (same compatible/reg/irq/phy/mdio).
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/dma-mapping.h>
#include <linux/dma-map-ops.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/bitops.h>
#include <linux/log2.h>
#include <linux/slab.h>
#include <linux/rtnetlink.h>

/* ---- CSR map (eth_dma/rtl/csr.v) ---- */
#define GTSE_CTRL		0x00	/* [0] rx_en  [1] tx_en                  */
#define GTSE_IRQ_STAT		0x04	/* W1C [0] rx_done [1] tx_done           */
#define GTSE_IRQ_EN		0x08
#define GTSE_RX_BASE		0x10
#define GTSE_RX_LEN		0x14	/* number of 2 KB buffers (power of two) */
#define GTSE_RX_TAIL		0x18	/* frames consumed by the CPU            */
#define GTSE_RX_HEAD		0x1c	/* frames written by the DMA (ro)        */
#define GTSE_RX_DROP		0x20	/* (ro)                                  */
#define GTSE_TX_BASE		0x30
#define GTSE_TX_LEN		0x34
#define GTSE_TX_TAIL		0x38	/* frames queued by the CPU              */
#define GTSE_TX_HEAD		0x3c	/* frames transmitted (ro)               */
#define GTSE_MDIO_ADDR		0x50	/* [4:0] phyad  [9:5] regad              */
#define GTSE_MDIO_WD		0x54
#define GTSE_MDIO_CTRL		0x58	/* [0] start read  [1] start write       */
#define GTSE_MDIO_ST		0x5c	/* [15:0] rddata [16] rd_valid [17] busy */
#define GTSE_ETH_MODE		0x60	/* [1:0] 0=10 1=100 2=1000  [2] full-dup */

#define CTRL_RX_EN		BIT(0)
#define CTRL_TX_EN		BIT(1)
#define IRQ_RX_DONE		BIT(0)
#define IRQ_TX_DONE		BIT(1)
#define MDIO_START_READ		BIT(0)
#define MDIO_START_WRITE	BIT(1)
#define MDIO_RVALID		BIT(16)
#define MDIO_BUSY		BIT(17)
#define EMODE_10		0u
#define EMODE_100		1u
#define EMODE_1000		2u
#define EMODE_FD		BIT(2)

#define GTSE_BUF_SIZE		2048u		/* gateware BUF_SHIFT = 11        */
#define GTSE_HDR		4u		/* [hdr|len] word before the data*/
#define GTSE_FRAME_MAX		(GTSE_BUF_SIZE - GTSE_HDR)
#define GTSE_DEFER_IRQS		2		/* napi_defer_hard_irqs           */
#define GTSE_GRO_FLUSH_NS	200000		/* gro_flush_timeout: 200 us      */

/* ring depths: power of two, 4..1024 (a ring is depth x 2 KB of DRAM) */
static unsigned int rx_ring = 64;
module_param(rx_ring, uint, 0444);
MODULE_PARM_DESC(rx_ring, "RX ring depth in 2 KB buffers (power of two)");
static unsigned int tx_ring = 32;
module_param(tx_ring, uint, 0444);
MODULE_PARM_DESC(tx_ring, "TX ring depth in 2 KB buffers (power of two)");

struct gtse_priv {
	void __iomem *base;
	struct net_device *ndev;
	struct device *dev;
	struct mii_bus *mii;
	struct napi_struct napi;
	int irq;

	spinlock_t irq_lock;		/* IRQ_EN shadow: ISR, NAPI and xmit change it */
	u32 irq_en;

	unsigned int rx_len, tx_len;	/* ring depths (power of two) */
	u8 *rx_pool;			/* rx_len * 2 KB, cached, streaming-mapped */
	dma_addr_t rx_dma;
	u8 *tx_pool;
	dma_addr_t tx_dma;
	u16 rx_tail;			/* our consumer count (== CSR RX_TAIL) */
	u16 tx_tail;			/* our producer count (CSR TX_TAIL may lag) */
	u16 tx_head;			/* last TX_HEAD read; only refreshed when full */
};

static inline u32 gtse_rd(struct gtse_priv *p, u32 off) { return ioread32(p->base + off); }
static inline void gtse_wr(struct gtse_priv *p, u32 off, u32 v) { iowrite32(v, p->base + off); }

static inline unsigned int gtse_slot(u16 idx, unsigned int len)
{
	return (idx & (len - 1)) * GTSE_BUF_SIZE;
}

static void gtse_irq_update(struct gtse_priv *p, u32 clear, u32 set)
{
	unsigned long flags;

	spin_lock_irqsave(&p->irq_lock, flags);
	p->irq_en = (p->irq_en & ~clear) | set;
	gtse_wr(p, GTSE_IRQ_EN, p->irq_en);
	spin_unlock_irqrestore(&p->irq_lock, flags);
}

/*
 * Copy a frame between a ring slot (data at offset 4) and an skb (data at
 * NET_IP_ALIGN = 2 mod 4).  The riscv memcpy falls back to a byte loop when
 * source and destination are not co-aligned, so for that skew this copies
 * with aligned word loads merged by a 16-bit shift into aligned word stores.
 * An aligned word load never crosses a page, so reading the unused half
 * before the start or after the end of the source is harmless.
 */
static void gtse_copy(u8 *dst, const u8 *src, unsigned int len)
{
	const u32 *s;
	u32 *d, prev, next;

	if ((((unsigned long)dst - (unsigned long)src) & 3) != 2) {
		memcpy(dst, src, len);
		return;
	}
	for (; len && ((unsigned long)dst & 3); len--)
		*dst++ = *src++;
	/* dst is word aligned, so src is 2 mod 4 */
	s = (const u32 *)(src - 2);
	d = (u32 *)dst;
	prev = *s++;
	for (; len >= 4; len -= 4) {
		next = *s++;
		*d++ = (prev >> 16) | (next << 16);
		prev = next;
	}
	src = (const u8 *)s - 2;
	dst = (u8 *)d;
	while (len--)
		*dst++ = *src++;
}

/* ------------------------------------------------------------------ MDIO */
static int gtse_mdio_read(struct mii_bus *bus, int phy, int reg)
{
	struct gtse_priv *p = bus->priv;
	u32 v;
	int ret;

	gtse_wr(p, GTSE_MDIO_ADDR, ((reg & 0x1f) << 5) | (phy & 0x1f));
	gtse_wr(p, GTSE_MDIO_CTRL, MDIO_START_READ);
	ret = readl_poll_timeout(p->base + GTSE_MDIO_ST, v, v & MDIO_RVALID, 5, 10000);
	if (ret)
		return ret;
	return v & 0xffff;
}

static int gtse_mdio_write(struct mii_bus *bus, int phy, int reg, u16 val)
{
	struct gtse_priv *p = bus->priv;
	u32 v;

	gtse_wr(p, GTSE_MDIO_ADDR, ((reg & 0x1f) << 5) | (phy & 0x1f));
	gtse_wr(p, GTSE_MDIO_WD, val);
	gtse_wr(p, GTSE_MDIO_CTRL, MDIO_START_WRITE);
	return readl_poll_timeout(p->base + GTSE_MDIO_ST, v, !(v & MDIO_BUSY), 5, 10000);
}

/* ------------------------------------------------------------------ link */
static void gtse_adjust_link(struct net_device *ndev)
{
	struct gtse_priv *p = netdev_priv(ndev);
	struct phy_device *phy = ndev->phydev;
	u32 mode;

	if (!phy->link)
		return;
	switch (phy->speed) {
	case SPEED_1000: mode = EMODE_1000; break;
	case SPEED_10:   mode = EMODE_10;   break;
	default:         mode = EMODE_100;  break;
	}
	if (phy->duplex == DUPLEX_FULL)
		mode |= EMODE_FD;
	gtse_wr(p, GTSE_ETH_MODE, mode);
	phy_print_status(phy);
}

/* ----------------------------------------------------------- RX (NAPI) */
static int gtse_poll(struct napi_struct *napi, int budget)
{
	struct gtse_priv *p = container_of(napi, struct gtse_priv, napi);
	struct net_device *ndev = p->ndev;
	int done = 0;
	u16 head = gtse_rd(p, GTSE_RX_HEAD);

	while (done < budget && p->rx_tail != head) {
		unsigned int off = gtse_slot(p->rx_tail, p->rx_len);
		u8 *buf = p->rx_pool + off;
		struct sk_buff *skb;
		u32 hdr, len;

		/*
		 * The DMA wrote this slot behind the cache: invalidate the header
		 * line, then the rest of the frame.  The CPU never writes RX slots,
		 * so they stay clean and need no maintenance to go back to the DMA.
		 */
		dma_sync_single_range_for_cpu(p->dev, p->rx_dma, off, 32, DMA_FROM_DEVICE);
		hdr = le32_to_cpup((__le32 *)buf);
		len = hdr & 0xffff;
		if ((hdr & BIT(16)) || len == 0 || len > GTSE_FRAME_MAX) {
			ndev->stats.rx_errors++;
			goto next;
		}
		if (GTSE_HDR + len > 32)
			dma_sync_single_range_for_cpu(p->dev, p->rx_dma, off + 32,
						      GTSE_HDR + len - 32, DMA_FROM_DEVICE);
		skb = napi_alloc_skb(napi, len);
		if (!skb) {
			ndev->stats.rx_dropped++;
			goto next;
		}
		gtse_copy(skb_put(skb, len), buf + GTSE_HDR, len);
		skb->protocol = eth_type_trans(skb, ndev);
		napi_gro_receive(napi, skb);
		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += len;
next:
		p->rx_tail++;
		gtse_wr(p, GTSE_RX_TAIL, p->rx_tail);	/* free the buffer */
		done++;
		if (p->rx_tail == head)			/* re-check for late arrivals */
			head = gtse_rd(p, GTSE_RX_HEAD);
	}

	if (done < budget && napi_complete_done(napi, done))
		gtse_irq_update(p, 0, IRQ_RX_DONE);
	return done;
}

/* -------------------------------------------------------------------- TX */
/*
 * The ring looks full by our (lagging) copy of TX_HEAD.  Ring the doorbell,
 * clear any stale tx_done and read how far the DMA really got; if it is still
 * full, stop the queue and enable tx_done.  A frame completing after the read
 * latches tx_done again, so the wake-up cannot be missed.  Returns true if
 * the queue was stopped.
 */
static bool gtse_tx_full(struct gtse_priv *p)
{
	gtse_wr(p, GTSE_TX_TAIL, p->tx_tail);
	gtse_wr(p, GTSE_IRQ_STAT, IRQ_TX_DONE);
	p->tx_head = gtse_rd(p, GTSE_TX_HEAD);
	if ((u16)(p->tx_tail - p->tx_head) < p->tx_len)
		return false;
	netif_stop_queue(p->ndev);
	gtse_irq_update(p, 0, IRQ_TX_DONE);
	return true;
}

static netdev_tx_t gtse_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct gtse_priv *p = netdev_priv(ndev);
	unsigned int len = skb->len, off;
	u8 *buf;

	if (len > GTSE_FRAME_MAX) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	/* after a wake-up our TX_HEAD copy is stale until refreshed */
	if ((u16)(p->tx_tail - p->tx_head) >= p->tx_len && gtse_tx_full(p))
		return NETDEV_TX_BUSY;

	off = gtse_slot(p->tx_tail, p->tx_len);
	buf = p->tx_pool + off;
	*(__le32 *)buf = cpu_to_le32(len);
	gtse_copy(buf + GTSE_HDR, skb->data, len);
	dma_sync_single_range_for_device(p->dev, p->tx_dma, off, GTSE_HDR + len, DMA_TO_DEVICE);
	p->tx_tail++;
	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += len;
	dev_consume_skb_any(skb);			/* copied into the ring */

	if ((u16)(p->tx_tail - p->tx_head) >= p->tx_len)
		gtse_tx_full(p);			/* rings the doorbell too */
	else if (!netdev_xmit_more())
		gtse_wr(p, GTSE_TX_TAIL, p->tx_tail);	/* doorbell */
	return NETDEV_TX_OK;
}

/* ------------------------------------------------------------------- IRQ */
static irqreturn_t gtse_isr(int irq, void *dev_id)
{
	struct net_device *ndev = dev_id;
	struct gtse_priv *p = netdev_priv(ndev);
	/*
	 * Only acknowledge enabled sources: a masked rx_done that latched while
	 * NAPI was polling must survive to fire when the poll re-enables it.
	 */
	u32 stat = gtse_rd(p, GTSE_IRQ_STAT) & READ_ONCE(p->irq_en);

	if (!stat)
		return IRQ_NONE;
	gtse_wr(p, GTSE_IRQ_STAT, stat);		/* W1C */

	if (stat & IRQ_RX_DONE) {
		gtse_irq_update(p, IRQ_RX_DONE, 0);	/* masked during the poll */
		napi_schedule(&p->napi);
	}
	if (stat & IRQ_TX_DONE) {			/* only enabled when stopped */
		gtse_irq_update(p, IRQ_TX_DONE, 0);
		netif_wake_queue(ndev);
	}
	return IRQ_HANDLED;
}

/* ---------------------------------------------------------------- open/stop */
static void gtse_free_rings(struct gtse_priv *p)
{
	if (p->rx_pool && !dma_mapping_error(p->dev, p->rx_dma))
		dma_unmap_single(p->dev, p->rx_dma, p->rx_len * GTSE_BUF_SIZE, DMA_FROM_DEVICE);
	if (p->tx_pool && !dma_mapping_error(p->dev, p->tx_dma))
		dma_unmap_single(p->dev, p->tx_dma, p->tx_len * GTSE_BUF_SIZE, DMA_TO_DEVICE);
	kfree(p->rx_pool);
	kfree(p->tx_pool);
	p->rx_pool = p->tx_pool = NULL;
}

static int gtse_alloc_rings(struct gtse_priv *p)
{
	p->rx_pool = kmalloc(p->rx_len * GTSE_BUF_SIZE, GFP_KERNEL);
	p->tx_pool = kmalloc(p->tx_len * GTSE_BUF_SIZE, GFP_KERNEL);
	p->rx_dma = p->tx_dma = DMA_MAPPING_ERROR;
	if (!p->rx_pool || !p->tx_pool)
		goto err;
	p->rx_dma = dma_map_single(p->dev, p->rx_pool, p->rx_len * GTSE_BUF_SIZE, DMA_FROM_DEVICE);
	p->tx_dma = dma_map_single(p->dev, p->tx_pool, p->tx_len * GTSE_BUF_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(p->dev, p->rx_dma) || dma_mapping_error(p->dev, p->tx_dma))
		goto err;
	return 0;
err:
	gtse_free_rings(p);
	return -ENOMEM;
}

static int gtse_open(struct net_device *ndev)
{
	struct gtse_priv *p = netdev_priv(ndev);
	int ret;

	ret = gtse_alloc_rings(p);
	if (ret)
		return ret;

	ret = request_irq(p->irq, gtse_isr, 0, ndev->name, ndev);
	if (ret) {
		gtse_free_rings(p);
		return ret;
	}

	/* HEAD counters are cumulative + not CPU-resettable (firmware self-test
	 * already advanced them), so start the ring empty at the current HEAD. */
	gtse_wr(p, GTSE_CTRL, 0);
	gtse_wr(p, GTSE_RX_BASE, p->rx_dma);
	gtse_wr(p, GTSE_RX_LEN, p->rx_len);
	gtse_wr(p, GTSE_TX_BASE, p->tx_dma);
	gtse_wr(p, GTSE_TX_LEN, p->tx_len);
	p->rx_tail = gtse_rd(p, GTSE_RX_HEAD);
	p->tx_tail = p->tx_head = gtse_rd(p, GTSE_TX_HEAD);
	gtse_wr(p, GTSE_RX_TAIL, p->rx_tail);
	gtse_wr(p, GTSE_TX_TAIL, p->tx_tail);

	napi_enable(&p->napi);
	gtse_wr(p, GTSE_IRQ_STAT, IRQ_RX_DONE | IRQ_TX_DONE);	/* clear stale */
	gtse_irq_update(p, ~0u, IRQ_RX_DONE);	/* tx_done only while stopped */
	gtse_wr(p, GTSE_CTRL, CTRL_RX_EN | CTRL_TX_EN);

	phy_start(ndev->phydev);
	netif_start_queue(ndev);
	return 0;
}

static int gtse_stop(struct net_device *ndev)
{
	struct gtse_priv *p = netdev_priv(ndev);

	netif_stop_queue(ndev);
	phy_stop(ndev->phydev);
	gtse_irq_update(p, ~0u, 0);
	gtse_wr(p, GTSE_CTRL, 0);
	napi_disable(&p->napi);
	free_irq(p->irq, ndev);
	gtse_free_rings(p);
	return 0;
}

static const struct net_device_ops gtse_netdev_ops = {
	.ndo_open		= gtse_open,
	.ndo_stop		= gtse_stop,
	.ndo_start_xmit		= gtse_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
};

/* ----------------------------------------------------------- probe/remove */
static int gtse_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *mdio_np, *phy_np;
	struct net_device *ndev;
	struct gtse_priv *p;
	int ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	/*
	 * RISC-V treats DT devices as DMA-coherent unless marked otherwise, but
	 * the A25 has no coherence manager: make dma_sync_*() do the cache ops.
	 */
	if (dev_is_dma_coherent(dev))
		dev->dma_coherent = false;

	ndev = devm_alloc_etherdev(dev, sizeof(*p));
	if (!ndev)
		return -ENOMEM;
	SET_NETDEV_DEV(ndev, dev);
	platform_set_drvdata(pdev, ndev);

	p = netdev_priv(ndev);
	p->ndev = ndev;
	p->dev = dev;
	spin_lock_init(&p->irq_lock);
	p->rx_len = is_power_of_2(rx_ring) && rx_ring >= 4 && rx_ring <= 1024 ? rx_ring : 64;
	p->tx_len = is_power_of_2(tx_ring) && tx_ring >= 4 && tx_ring <= 1024 ? tx_ring : 32;

	p->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);

	p->irq = platform_get_irq(pdev, 0);
	if (p->irq < 0)
		return p->irq;

	if (of_get_ethdev_address(dev->of_node, ndev))
		eth_hw_addr_random(ndev);

	ndev->netdev_ops = &gtse_netdev_ops;
	ndev->irq = p->irq;
	netif_napi_add(ndev, &p->napi, gtse_poll);

	/* ---- MDIO bus ---- */
	p->mii = devm_mdiobus_alloc(dev);
	if (!p->mii)
		return -ENOMEM;
	p->mii->name = "gtse-mii";
	p->mii->read = gtse_mdio_read;
	p->mii->write = gtse_mdio_write;
	p->mii->priv = p;
	p->mii->parent = dev;
	snprintf(p->mii->id, MII_BUS_ID_SIZE, "%s", dev_name(dev));

	mdio_np = of_get_child_by_name(dev->of_node, "mdio");
	ret = of_mdiobus_register(p->mii, mdio_np);
	of_node_put(mdio_np);
	if (ret)
		return dev_err_probe(dev, ret, "MDIO register failed\n");

	/* ---- connect the PHY (RGMII-ID: delay is in the PHY) ---- */
	phy_np = of_parse_phandle(dev->of_node, "phy-handle", 0);
	if (!phy_np) {
		ret = -ENODEV;
		goto err_mdio;
	}
	ndev->phydev = of_phy_connect(ndev, phy_np, gtse_adjust_link, 0,
				      PHY_INTERFACE_MODE_RGMII_ID);
	of_node_put(phy_np);
	if (!ndev->phydev) {
		ret = -ENODEV;
		goto err_mdio;
	}

	ndev->napi_defer_hard_irqs = GTSE_DEFER_IRQS;
	ndev->gro_flush_timeout = GTSE_GRO_FLUSH_NS;
	ret = register_netdev(ndev);
	if (ret)
		goto err_phy;
	rtnl_lock();
	if (dev_set_threaded(ndev, true))
		netdev_warn(ndev, "threaded NAPI unavailable, using softirq\n");
	rtnl_unlock();

	netdev_info(ndev, "Gowin TSE (DMA, %u/%u-deep RX/TX rings, threaded NAPI) at %pa, irq %d\n",
		    p->rx_len, p->tx_len, &pdev->resource[0].start, p->irq);
	return 0;

err_phy:
	phy_disconnect(ndev->phydev);
err_mdio:
	mdiobus_unregister(p->mii);
	return ret;
}

static void gtse_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct gtse_priv *p = netdev_priv(ndev);

	unregister_netdev(ndev);
	phy_disconnect(ndev->phydev);
	mdiobus_unregister(p->mii);
}

static const struct of_device_id gtse_of_match[] = {
	{ .compatible = "gowin,ae350-tse" },
	{ }
};
MODULE_DEVICE_TABLE(of, gtse_of_match);

static struct platform_driver gtse_driver = {
	.probe	= gtse_probe,
	.remove	= gtse_remove,
	.driver	= {
		.name		= "gw-tse",
		.of_match_table	= gtse_of_match,
	},
};
module_platform_driver(gtse_driver);

MODULE_DESCRIPTION("Gowin Triple-Speed Ethernet (DMA) for AE350");
MODULE_LICENSE("GPL");
