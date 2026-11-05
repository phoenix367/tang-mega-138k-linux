// SPDX-License-Identifier: GPL-2.0
/*
 * Gowin AE350 (Tang Mega 138K Pro Dock) DDR bus performance counters -- the
 * ddr_perf block of the gateware (src/rtl/ddr_perf.v, SoC external APB at
 * 0xF8000000).  It watches the SoC's data-memory AHB bus, which every L1
 * line fill, eviction and uncached DDR access goes through, and carries the
 * DDR3 bridge's own statistics and its debug MODE register.
 *
 * The 32-bit hardware counters count in the 100 MHz bus clock and read
 * consistently only while counting is stopped, so the driver samples them:
 * stop, read them all, start again (a few microseconds of blind time), every
 * interval_ms and whenever a file is read more than 10 ms after the last
 * sample.  The differences go into 64-bit totals, so nothing wraps (the
 * cycle counter alone would every 43 s).  The block is never cleared on
 * probe: a trace captured before a watchdog reset stays readable.
 *
 * sysfs, in the platform device's directory:
 *   counters         every counter from one sample, "name value" per line:
 *                    64-bit totals since boot or the last clear; the two
 *                    *_wait_max are the longest single waits instead
 *   statistics/<counter>  the same one at a time; files read one after
 *                    another are not one sample (a file read on this CPU
 *                    takes longer than the 10 ms refresh), use counters
 *   clear            write 1: totals and maxima back to 0
 *   interval_ms      sampling period, 100..10000 (default 1000)
 *   clock_hz         the bus clock "cycles" counts (DT clock-frequency)
 *   config, mode     the block's CONFIG word, the bridge's MODE register
 *   trace_arm        1: capture the next 1024 bus transfers from entry 0
 *                    (keeps the totals), 0: stop capturing
 *   trace_count      entries captured
 *   trace            the entries, 8 bytes each: le32 HADDR, le32 {ts[15:0],
 *                    wait[7:0], write, HBURST[2:0], HSIZE[1:0], seq, prev_valid}
 */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#define REG_ID		0x000
#define  ID_MAGIC	0xDD3B
#define REG_CTRL	0x004
#define  CTRL_EN	BIT(0)
#define  CTRL_CLR	BIT(1)
#define  CTRL_ARM	BIT(2)
#define REG_STATUS	0x008
#define  STATUS_COUNT	GENMASK(15, 0)
#define REG_TRACE_IDX	0x00C
#define REG_TRACE_LO	0x010
#define REG_TRACE_HI	0x014
#define REG_CONFIG	0x018
#define REG_MODE	0x020
#define REG_CNT(i)	(0x100 + 4 * (i))

#define TRACE_ENTRIES	1024
#define TRACE_BYTES	8
#define REFRESH_MS	10

/* hardware counters 0..43 (44..51 = the bridge's read-BIST results, not counters) */
static const char * const ddr_perf_names[] = {
	"cycles", "rd_bursts", "wr_bursts", "rd_beats", "wr_beats",
	"rd_wait", "wr_wait", "rd_first_wait", "rd_wait_max", "wr_wait_max",
	"busy", "bus_active",
	/* NONSEQ transfers by HBURST */
	"rd_single", "rd_incr", "rd_wrap4", "rd_incr4",
	"rd_wrap8", "rd_incr8", "rd_wrap16", "rd_incr16",
	"wr_single", "wr_incr", "wr_wrap4", "wr_incr4",
	"wr_wrap8", "wr_incr8", "wr_wrap16", "wr_incr16",
	/* NONSEQ transfers by HSIZE */
	"rd_size8", "rd_size16", "rd_size32", "rd_size64",
	"wr_size8", "wr_size16", "wr_size32", "wr_size64",
	/* the DDR3 bridge (src/rtl/ddr_bridge) */
	"br_hit0", "br_demand", "br_prefetch", "br_wr_push",
	"mc_rd_cmds", "br_tag_mismatch", "mc_wr_cmds", "mc_rd_valid",
};
#define NCNT		ARRAY_SIZE(ddr_perf_names)
#define CNT_CYCLES	0
#define CNT_RD_WAIT_MAX	8
#define CNT_WR_WAIT_MAX	9

struct ddr_perf {
	void __iomem *base;
	struct mutex lock;		/* the block's CTRL / TRACE_IDX and the totals */
	struct delayed_work work;
	unsigned int interval_ms;
	u32 clock_hz;
	ktime_t last_time;
	u32 last[NCNT];			/* raw values at the last sample */
	u64 total[NCNT];
	struct dev_ext_attribute stat_attr[NCNT];
	struct attribute *stat_attrs[NCNT + 1];
	struct attribute_group stat_group;
};

static u32 dp_rd(struct ddr_perf *p, u32 reg) { return readl(p->base + reg); }
static void dp_wr(struct ddr_perf *p, u32 reg, u32 v) { writel(v, p->base + reg); }

/*
 * Stop counting (CTRL is written with ARM as given), read every counter, and
 * leave counting stopped.  EN reaches the bus clock through two flops and the
 * counters' byte carries settle a few cycles later: the read-back plus 1 us
 * cover both.
 */
static void ddr_perf_freeze_read(struct ddr_perf *p, u32 ctrl, u32 *raw)
{
	int i;

	dp_wr(p, REG_CTRL, ctrl & ~CTRL_EN);
	dp_rd(p, REG_CTRL);
	udelay(1);
	for (i = 0; i < NCNT; i++)
		raw[i] = dp_rd(p, REG_CNT(i));
}

/* lock held: fold what was counted since the last sample into the totals */
static void ddr_perf_sample(struct ddr_perf *p)
{
	u32 ctrl = dp_rd(p, REG_CTRL) & CTRL_ARM;
	ktime_t now = ktime_get();
	u64 expect;
	u32 raw[NCNT];
	bool cleared;
	int i;

	ddr_perf_freeze_read(p, ctrl, raw);
	dp_wr(p, REG_CTRL, ctrl | CTRL_EN);

	/*
	 * More cycles than the time since the last sample allows: someone
	 * cleared the block behind the driver's back (devmem, amotrace), so the
	 * raw values are what was counted since then.
	 */
	expect = div_u64((u64)ktime_to_us(ktime_sub(now, p->last_time)) * p->clock_hz, USEC_PER_SEC);
	cleared = raw[CNT_CYCLES] - p->last[CNT_CYCLES] > expect + p->clock_hz / 10;

	for (i = 0; i < NCNT; i++) {
		if (i == CNT_RD_WAIT_MAX || i == CNT_WR_WAIT_MAX)
			p->total[i] = max_t(u64, p->total[i], raw[i]);
		else
			p->total[i] += cleared ? raw[i] : raw[i] - p->last[i];
		p->last[i] = raw[i];
	}
	p->last_time = now;
}

static void ddr_perf_refresh(struct ddr_perf *p)
{
	if (ktime_ms_delta(ktime_get(), p->last_time) >= REFRESH_MS)
		ddr_perf_sample(p);
}

/*
 * lock held: clear the hardware counters, maxima and trace pointer, then count
 * again with ARM as given.  What was counted before goes into the totals first.
 */
static void ddr_perf_hw_clear(struct ddr_perf *p, u32 arm)
{
	ddr_perf_sample(p);
	dp_wr(p, REG_CTRL, CTRL_CLR | arm);
	ddr_perf_freeze_read(p, arm, p->last);
	dp_wr(p, REG_CTRL, arm | CTRL_EN);
	p->last_time = ktime_get();
}

static void ddr_perf_work(struct work_struct *w)
{
	struct ddr_perf *p = container_of(to_delayed_work(w), struct ddr_perf, work);

	mutex_lock(&p->lock);
	ddr_perf_sample(p);
	mutex_unlock(&p->lock);
	queue_delayed_work(system_power_efficient_wq, &p->work,
			   msecs_to_jiffies(READ_ONCE(p->interval_ms)));
}

static ssize_t stat_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ddr_perf *p = dev_get_drvdata(dev);
	unsigned long i = (unsigned long)container_of(attr, struct dev_ext_attribute, attr)->var;
	u64 v;

	mutex_lock(&p->lock);
	ddr_perf_refresh(p);
	v = p->total[i];
	mutex_unlock(&p->lock);
	return sysfs_emit(buf, "%llu\n", v);
}

static ssize_t counters_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ddr_perf *p = dev_get_drvdata(dev);
	int i, len = 0;

	mutex_lock(&p->lock);
	ddr_perf_sample(p);
	for (i = 0; i < NCNT; i++)
		len += sysfs_emit_at(buf, len, "%s %llu\n", ddr_perf_names[i], p->total[i]);
	mutex_unlock(&p->lock);
	return len;
}
static DEVICE_ATTR_RO(counters);

static ssize_t clear_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct ddr_perf *p = dev_get_drvdata(dev);
	bool on;
	int ret = kstrtobool(buf, &on);

	if (ret)
		return ret;
	if (on) {
		mutex_lock(&p->lock);
		ddr_perf_hw_clear(p, dp_rd(p, REG_CTRL) & CTRL_ARM);
		memset(p->total, 0, sizeof(p->total));
		mutex_unlock(&p->lock);
	}
	return count;
}
static DEVICE_ATTR_WO(clear);

static ssize_t interval_ms_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ddr_perf *p = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", READ_ONCE(p->interval_ms));
}

static ssize_t interval_ms_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct ddr_perf *p = dev_get_drvdata(dev);
	unsigned int ms;
	int ret = kstrtouint(buf, 0, &ms);

	if (ret)
		return ret;
	/* the 32-bit counters must not wrap twice between samples (43 s at 100 MHz) */
	if (ms < 100 || ms > 10000)
		return -EINVAL;
	WRITE_ONCE(p->interval_ms, ms);
	mod_delayed_work(system_power_efficient_wq, &p->work, msecs_to_jiffies(ms));
	return count;
}
static DEVICE_ATTR_RW(interval_ms);

static ssize_t clock_hz_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ddr_perf *p = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", p->clock_hz);
}
static DEVICE_ATTR_RO(clock_hz);

static ssize_t config_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ddr_perf *p = dev_get_drvdata(dev);

	return sysfs_emit(buf, "0x%08x\n", dp_rd(p, REG_CONFIG));
}
static DEVICE_ATTR_RO(config);

static ssize_t mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ddr_perf *p = dev_get_drvdata(dev);

	return sysfs_emit(buf, "0x%08x\n", dp_rd(p, REG_MODE));
}
static DEVICE_ATTR_RO(mode);

static ssize_t trace_arm_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ddr_perf *p = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", !!(dp_rd(p, REG_CTRL) & CTRL_ARM));
}

static ssize_t trace_arm_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct ddr_perf *p = dev_get_drvdata(dev);
	bool on;
	int ret = kstrtobool(buf, &on);

	if (ret)
		return ret;
	mutex_lock(&p->lock);
	if (on)
		ddr_perf_hw_clear(p, CTRL_ARM);		/* the trace starts again at entry 0 */
	else
		dp_wr(p, REG_CTRL, dp_rd(p, REG_CTRL) & CTRL_EN);
	mutex_unlock(&p->lock);
	return count;
}
static DEVICE_ATTR_RW(trace_arm);

static u32 ddr_perf_trace_count(struct ddr_perf *p)
{
	return min_t(u32, dp_rd(p, REG_STATUS) & STATUS_COUNT, TRACE_ENTRIES);
}

static ssize_t trace_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ddr_perf *p = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", ddr_perf_trace_count(p));
}
static DEVICE_ATTR_RO(trace_count);

static ssize_t trace_read(struct file *filp, struct kobject *kobj, struct bin_attribute *attr,
			  char *buf, loff_t off, size_t count)
{
	struct ddr_perf *p = dev_get_drvdata(kobj_to_dev(kobj));
	size_t len, done = 0;

	mutex_lock(&p->lock);
	len = (size_t)ddr_perf_trace_count(p) * TRACE_BYTES;
	count = off < len ? min_t(size_t, count, len - off) : 0;
	while (done < count) {
		u32 e = (off + done) / TRACE_BYTES, skip = (off + done) % TRACE_BYTES;
		size_t n = min_t(size_t, TRACE_BYTES - skip, count - done);
		__le32 ent[2];

		dp_wr(p, REG_TRACE_IDX, e);
		dp_rd(p, REG_TRACE_IDX);		/* the trace RAM's read port follows a cycle later */
		ent[0] = cpu_to_le32(dp_rd(p, REG_TRACE_LO));
		ent[1] = cpu_to_le32(dp_rd(p, REG_TRACE_HI));
		memcpy(buf + done, (u8 *)ent + skip, n);
		done += n;
	}
	mutex_unlock(&p->lock);
	return count;
}
static BIN_ATTR_RO(trace, TRACE_ENTRIES * TRACE_BYTES);

static struct attribute *ddr_perf_attrs[] = {
	&dev_attr_counters.attr,
	&dev_attr_clear.attr,
	&dev_attr_interval_ms.attr,
	&dev_attr_clock_hz.attr,
	&dev_attr_config.attr,
	&dev_attr_mode.attr,
	&dev_attr_trace_arm.attr,
	&dev_attr_trace_count.attr,
	NULL
};

static struct bin_attribute *ddr_perf_bin_attrs[] = {
	&bin_attr_trace,
	NULL
};

static const struct attribute_group ddr_perf_group = {
	.attrs = ddr_perf_attrs,
	.bin_attrs = ddr_perf_bin_attrs,
};

static void ddr_perf_stop(void *data)
{
	struct ddr_perf *p = data;

	cancel_delayed_work_sync(&p->work);
}

static int gowin_ddr_perf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ddr_perf *p;
	u32 id, cfg, arm;
	int i, ret;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);
	id = dp_rd(p, REG_ID);
	if (id >> 16 != ID_MAGIC)
		return dev_err_probe(dev, -ENODEV, "no ddr_perf block (ID 0x%08x)\n", id);
	if (of_property_read_u32(dev->of_node, "clock-frequency", &p->clock_hz) || !p->clock_hz)
		p->clock_hz = 100000000;
	p->interval_ms = 1000;
	mutex_init(&p->lock);
	INIT_DELAYED_WORK(&p->work, ddr_perf_work);
	platform_set_drvdata(pdev, p);

	/* totals count from here; an armed trace and the counters stay as they are */
	arm = dp_rd(p, REG_CTRL) & CTRL_ARM;
	ddr_perf_freeze_read(p, arm, p->last);
	dp_wr(p, REG_CTRL, arm | CTRL_EN);
	p->last_time = ktime_get();
	p->total[CNT_RD_WAIT_MAX] = p->last[CNT_RD_WAIT_MAX];
	p->total[CNT_WR_WAIT_MAX] = p->last[CNT_WR_WAIT_MAX];

	for (i = 0; i < NCNT; i++) {
		struct dev_ext_attribute *a = &p->stat_attr[i];

		sysfs_attr_init(&a->attr.attr);
		a->attr.attr.name = ddr_perf_names[i];
		a->attr.attr.mode = 0444;
		a->attr.show = stat_show;
		a->var = (void *)(unsigned long)i;
		p->stat_attrs[i] = &a->attr.attr;
	}
	p->stat_group.name = "statistics";
	p->stat_group.attrs = p->stat_attrs;

	ret = devm_add_action_or_reset(dev, ddr_perf_stop, p);
	if (ret)
		return ret;
	ret = devm_device_add_group(dev, &ddr_perf_group);
	if (!ret)
		ret = devm_device_add_group(dev, &p->stat_group);
	if (ret)
		return ret;
	queue_delayed_work(system_power_efficient_wq, &p->work, msecs_to_jiffies(p->interval_ms));

	cfg = dp_rd(p, REG_CONFIG);
	dev_info(dev, "DDR bus counters: bridge %u, DDR3 %u MHz, bus clock %u MHz, MODE 0x%08x%s\n",
		 (cfg >> 16) & 0xff, cfg & 0xffff, p->clock_hz / 1000000, dp_rd(p, REG_MODE),
		 arm ? ", trace armed" : "");
	return 0;
}

static const struct of_device_id gowin_ddr_perf_of_match[] = {
	{ .compatible = "gowin,ae350-ddr-perf" },
	{ }
};
MODULE_DEVICE_TABLE(of, gowin_ddr_perf_of_match);

static struct platform_driver gowin_ddr_perf_driver = {
	.probe = gowin_ddr_perf_probe,
	.driver = {
		.name = "gowin-ddr-perf",
		.of_match_table = gowin_ddr_perf_of_match,
	},
};
module_platform_driver(gowin_ddr_perf_driver);

MODULE_DESCRIPTION("Gowin AE350 DDR bus performance counters");
MODULE_LICENSE("GPL");
