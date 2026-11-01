// SPDX-License-Identifier: GPL-2.0
/*
 * Gowin AE350 PT8211 audio -- ALSA playback driver for the audio core in the
 * Tang Mega 138K Pro gateware (audio/rtl; registers at 0xE8002000).
 *
 * The core is a 2048-frame FIFO in front of a serialiser for the Dock's PT8211
 * stereo DAC (16-bit, headphone jack / speaker).  The hardware position
 * (.pointer) is the core's FRAMES register: frames taken from the FIFO since
 * the stream start.  Two ways of feeding the FIFO:
 *
 * DMA mode (cores with CAP[0], the default): the ALSA buffer is physically
 * contiguous memory (SNDRV_DMA_TYPE_CONTINUOUS) mapped for streaming DMA; the
 * core's own AHB master reads it as a ring (DMA_ADDR/SIZE) and keeps the FIFO
 * at DMA_FILL frames.  The driver only tells the core how far the application
 * has written: .ack writes back the dirty cache lines of the newly committed
 * range (the A25 is not cache coherent; the sync is a clean, no invalidate)
 * and advances DMA_LIMIT, a free-running byte count.  Interrupts: DMA_IRQ[0]
 * every DMA_PERIOD bytes fetched (-> period elapsed; the fetch leads playback
 * by at most DMA_FILL frames, kept well below a period), DMA_IRQ[1] on a bus
 * error, and the FIFO-level interrupt with THRESH = 1, i.e. "FIFO empty": that
 * is how the end of the stream (or a real starvation) is noticed.  When the
 * fetch has caught up with DMA_LIMIT the handler sets STOP_EMPTY so the core
 * stops cleanly when the last frame is out, and masks the interrupt once the
 * FIFO is empty; .ack re-enables both when more data arrives.
 *
 * PIO mode (older cores, or pio=1): the ALSA buffer is vmalloc memory and the
 * interrupt handler copies frames into the FIFO with 32-bit DATA writes when
 * the level drops below THRESH = depth/2; when the buffer has no more frames,
 * THRESH = 1 + STOP_EMPTY as above.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/irqdomain.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/property.h>
#include <linux/math64.h>
#include <linux/dma-mapping.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/info.h>

/* ---- audio core registers (audio/rtl/audio_csr.v) ---- */
#define AUD_CTRL	0x00
#define   CTRL_ENABLE	BIT(0)
#define   CTRL_IRQ_EN	BIT(1)
#define   CTRL_FIFO_RST	BIT(2)
#define   CTRL_SWAP	BIT(3)
#define   CTRL_STOP_EMPTY BIT(4)	/* clear ENABLE at the first empty frame */
#define   CTRL_AMP_EN	BIT(5)	/* headphone amplifier (PA_EN pin), resets to 1 */
#define   CTRL_DMA_EN	BIT(6)	/* FIFO fed by the DMA engine, DATA writes ignored */
#define AUD_STATUS	0x04	/* [15:0] level  [16] level<thresh  [17] full */
#define AUD_DATA	0x08	/* {R, L} */
#define AUD_THRESH	0x0c
#define AUD_RATE	0x10	/* NCO increment: BCK = RATE / 2^32 * bus clock */
#define AUD_UNDERRUN	0x14
#define AUD_OVERRUN	0x18
#define AUD_FRAMES	0x1c
#define AUD_CRC		0x20
#define AUD_DEPTH	0x24
#define AUD_CAP		0x28	/* [0] DMA engine present */
#define AUD_ID		0x2c
#define AUD_ID_VALUE	0x41554430	/* "AUD0" */
#define AUD_DMA_ADDR	0x30	/* ring buffer bus address, 8-byte aligned */
#define AUD_DMA_SIZE	0x34	/* ring bytes, multiple of 8 */
#define AUD_DMA_PERIOD	0x38	/* DMA_IRQ[0] every this many bytes fetched */
#define AUD_DMA_LIMIT	0x3c	/* committed bytes, free running */
#define AUD_DMA_FETCH	0x40	/* fetched bytes, free running */
#define AUD_DMA_IRQ	0x44	/* [0] period  [1] bus error; W1C */
#define   DMA_IRQ_PERIOD BIT(0)
#define   DMA_IRQ_ERR	BIT(1)
#define AUD_DMA_FILL	0x48	/* [15:0] fill level (frames)  [23:16] max beats per burst */

#define DMA_FILL_FRAMES	256	/* fetch lead over playback: 5.3 ms at 48 kHz */
#define DMA_MAX_BEATS	32	/* 64 frames per burst (INCR16 x 2) */

static bool swap_lr;
module_param(swap_lr, bool, 0644);
MODULE_PARM_DESC(swap_lr, "send the right channel during WS low (default: left)");

static bool pio;
module_param(pio, bool, 0644);
MODULE_PARM_DESC(pio, "feed the FIFO from the interrupt handler even when the core has a DMA engine (read when a stream is opened)");

static int hwirq;
module_param(hwirq, int, 0444);
MODULE_PARM_DESC(hwirq, "override the PLIC source number from the device tree (0: use DT). The AE350 maps EXT_INT[0] to PLIC source 4 and EXT_INT[1..15] to 11..25");

struct pt8211 {
	struct device *dev;
	void __iomem *base;
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct snd_pcm_substream *substream;
	spinlock_t lock;
	u32 bus_hz;
	u32 depth;		/* FIFO frames */
	u32 thresh;		/* PIO refill threshold */
	bool dma_cap;		/* the core has the DMA engine */
	bool dma;		/* DMA mode in use for the open stream */
	bool running;
	/* stream state (under lock) */
	snd_pcm_uframes_t push_pos;	/* PIO: next buffer frame to push */
	snd_pcm_uframes_t pushed;	/* PIO: frames pushed since start, mod boundary */
	snd_pcm_uframes_t last_appl;	/* DMA: appl_ptr at the last commit */
	u32 committed;			/* DMA: bytes committed (DMA_LIMIT) */
	u32 ring_pos;			/* DMA: committed % ring bytes */
	u32 last_frames;		/* FRAMES at the last period check */
	unsigned long irqs, fills, dma_errs;
	/* DMA mapping of the ALSA buffer (hw_params .. hw_free) */
	dma_addr_t dma_buf;
	size_t dma_bytes;
};

static const struct snd_pcm_hardware pt8211_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_DRAIN_TRIGGER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 |
		 SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
		 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000,
	.rate_min = 8000,
	.rate_max = 96000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 256 * 1024,
	.period_bytes_min = 4096,
	.period_bytes_max = 64 * 1024,
	.periods_min = 2,
	.periods_max = 64,
	.fifo_size = 2048 * 4,
};

static inline u32 aud_rd(struct pt8211 *p, u32 off)
{
	return readl(p->base + off);
}

static inline void aud_wr(struct pt8211 *p, u32 off, u32 v)
{
	writel(v, p->base + off);
}

/* CTRL value for a running stream; every CTRL write keeps the amplifier on */
static u32 pt8211_ctrl(struct pt8211 *p)
{
	return CTRL_AMP_EN | CTRL_ENABLE | CTRL_IRQ_EN | (swap_lr ? CTRL_SWAP : 0) |
	       (p->dma ? CTRL_DMA_EN : 0);
}

/* ---- PIO ------------------------------------------------------------------ */

/*
 * Push frames from the ALSA buffer (up to appl_ptr) into the FIFO (up to its
 * depth).  Lock held.  Returns true when frames remain to be pushed.
 */
static bool pt8211_fill(struct pt8211 *p)
{
	struct snd_pcm_runtime *rt = p->substream->runtime;
	snd_pcm_uframes_t appl = READ_ONCE(rt->control->appl_ptr);
	snd_pcm_uframes_t avail, n, chunk;
	u32 level = aud_rd(p, AUD_STATUS) & 0xffff;
	u32 room = level < p->depth ? p->depth - level : 0;

	avail = appl >= p->pushed ? appl - p->pushed : appl + rt->boundary - p->pushed;
	n = min_t(snd_pcm_uframes_t, avail, room);
	while (n) {
		chunk = min(n, rt->buffer_size - p->push_pos);
		writesl(p->base + AUD_DATA,
			rt->dma_area + frames_to_bytes(rt, p->push_pos), chunk);
		p->push_pos += chunk;
		if (p->push_pos == rt->buffer_size)
			p->push_pos = 0;
		p->pushed += chunk;
		if (p->pushed >= rt->boundary)
			p->pushed -= rt->boundary;
		avail -= chunk;
		n -= chunk;
	}
	p->fills++;
	return avail != 0;
}

/* ---- DMA ------------------------------------------------------------------ */

/*
 * Tell the core how far the application has written: clean the cache over the
 * newly committed range of the ring and advance DMA_LIMIT.  Lock held.
 */
static void pt8211_commit(struct pt8211 *p)
{
	struct snd_pcm_runtime *rt = p->substream->runtime;
	snd_pcm_uframes_t appl = READ_ONCE(rt->control->appl_ptr);
	snd_pcm_uframes_t delta;
	u32 bytes, first;

	delta = appl >= p->last_appl ? appl - p->last_appl
				     : appl + rt->boundary - p->last_appl;
	p->last_appl = appl;
	if (!delta)
		return;
	bytes = frames_to_bytes(rt, delta);
	first = min_t(u32, bytes, p->dma_bytes - p->ring_pos);
	dma_sync_single_for_device(p->dev, p->dma_buf + p->ring_pos, first,
				   DMA_TO_DEVICE);
	if (bytes > first)
		dma_sync_single_for_device(p->dev, p->dma_buf, bytes - first,
					   DMA_TO_DEVICE);
	p->ring_pos = (p->ring_pos + bytes) % p->dma_bytes;
	p->committed += bytes;
	aud_wr(p, AUD_DMA_LIMIT, p->committed);
	p->fills++;
}

/* ---- interrupt ------------------------------------------------------------ */

static irqreturn_t pt8211_irq(int irq, void *dev_id)
{
	struct pt8211 *p = dev_id;
	struct snd_pcm_substream *ss;
	bool elapsed = false, more, error = false;
	u32 frames, level, ps, st = 0;

	spin_lock(&p->lock);
	p->irqs++;
	ss = p->substream;
	if (!p->running || !ss) {
		aud_wr(p, AUD_CTRL, (aud_rd(p, AUD_CTRL) & ~CTRL_IRQ_EN) | CTRL_AMP_EN);
		spin_unlock(&p->lock);
		return IRQ_HANDLED;
	}
	if (p->dma) {
		st = aud_rd(p, AUD_DMA_IRQ);
		if (st)
			aud_wr(p, AUD_DMA_IRQ, st);
		if (st & DMA_IRQ_ERR) {
			p->dma_errs++;
			p->running = false;
			aud_wr(p, AUD_CTRL, CTRL_AMP_EN);
			error = true;
		} else {
			level = aud_rd(p, AUD_STATUS) & 0xffff;
			if (aud_rd(p, AUD_DMA_FETCH) == p->committed) {
				/* the whole committed range is in the FIFO or
				 * played: stop cleanly at the last frame; once
				 * the FIFO is empty (level irq) mask until
				 * .ack commits more */
				if (level) {
					aud_wr(p, AUD_CTRL, pt8211_ctrl(p) | CTRL_STOP_EMPTY);
				} else {
					aud_wr(p, AUD_CTRL, pt8211_ctrl(p) & ~(CTRL_ENABLE | CTRL_IRQ_EN));
					elapsed = true;
				}
			}
			if (st & DMA_IRQ_PERIOD)
				elapsed = true;
		}
	} else {
		more = pt8211_fill(p);
		level = aud_rd(p, AUD_STATUS) & 0xffff;
		if (more) {
			aud_wr(p, AUD_THRESH, p->thresh);
		} else if (level) {
			/* the stream's last frames are in the FIFO: have the
			 * core stop itself when they are out (no underrun count
			 * for the tail) and interrupt once more when empty */
			aud_wr(p, AUD_THRESH, 1);
			aud_wr(p, AUD_CTRL, pt8211_ctrl(p) | CTRL_STOP_EMPTY);
		} else {
			/* everything the application gave us has been played:
			 * stop the serialiser (it would send zeros and count
			 * them as underruns until ALSA stops the stream) and
			 * mask; .ack re-enables both */
			aud_wr(p, AUD_CTRL, pt8211_ctrl(p) & ~(CTRL_ENABLE | CTRL_IRQ_EN));
			elapsed = true;
		}
	}
	if (!error) {
		frames = aud_rd(p, AUD_FRAMES);
		ps = ss->runtime->period_size;
		if (frames / ps != p->last_frames / ps)
			elapsed = true;
		p->last_frames = frames;
	}
	spin_unlock(&p->lock);

	if (error) {
		dev_err_ratelimited(p->dev, "DMA bus error, stream stopped\n");
		snd_pcm_stop_xrun(ss);
	} else if (elapsed) {
		snd_pcm_period_elapsed(ss);
	}
	return IRQ_HANDLED;
}

/* ---- PCM ops -------------------------------------------------------------- */

static int pt8211_open(struct snd_pcm_substream *ss)
{
	struct pt8211 *p = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *rt = ss->runtime;
	int ret;

	rt->hw = pt8211_hw;
	rt->hw.fifo_size = p->depth * 4;
	p->dma = p->dma_cap && !pio;	/* the buffer type suits both modes */
	ret = snd_pcm_hw_constraint_integer(rt, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;
	if (p->dma) {
		/* the engine reads 8 bytes = 2 frames per beat: ring and
		 * period are multiples of 2 frames (commits may be odd) */
		ret = snd_pcm_hw_constraint_step(rt, 0, SNDRV_PCM_HW_PARAM_PERIOD_SIZE, 2);
		if (ret < 0)
			return ret;
		ret = snd_pcm_hw_constraint_step(rt, 0, SNDRV_PCM_HW_PARAM_BUFFER_SIZE, 2);
		if (ret < 0)
			return ret;
	}
	spin_lock_irq(&p->lock);
	p->substream = ss;
	spin_unlock_irq(&p->lock);
	return 0;
}

static int pt8211_close(struct snd_pcm_substream *ss)
{
	struct pt8211 *p = snd_pcm_substream_chip(ss);

	spin_lock_irq(&p->lock);
	p->running = false;
	aud_wr(p, AUD_CTRL, CTRL_AMP_EN);
	p->substream = NULL;
	spin_unlock_irq(&p->lock);
	return 0;
}

static void pt8211_unmap(struct pt8211 *p)
{
	if (p->dma_bytes) {
		dma_unmap_single(p->dev, p->dma_buf, p->dma_bytes, DMA_TO_DEVICE);
		p->dma_bytes = 0;
	}
}

/* DMA mode: map the (physically contiguous) ALSA buffer for the core's master */
static int pt8211_hw_params(struct snd_pcm_substream *ss,
			    struct snd_pcm_hw_params *params)
{
	struct pt8211 *p = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *rt = ss->runtime;
	size_t bytes = params_buffer_bytes(params);

	if (!p->dma)
		return 0;
	pt8211_unmap(p);
	if (bytes & 7 || (uintptr_t)rt->dma_area & 7)
		return -EINVAL;
	p->dma_buf = dma_map_single(p->dev, rt->dma_area, bytes, DMA_TO_DEVICE);
	if (dma_mapping_error(p->dev, p->dma_buf))
		return -ENOMEM;
	p->dma_bytes = bytes;
	return 0;
}

static int pt8211_hw_free(struct snd_pcm_substream *ss)
{
	struct pt8211 *p = snd_pcm_substream_chip(ss);

	if (p->dma) {
		spin_lock_irq(&p->lock);
		p->running = false;
		aud_wr(p, AUD_CTRL, CTRL_AMP_EN);
		spin_unlock_irq(&p->lock);
		pt8211_unmap(p);
	}
	return 0;
}

static u32 pt8211_rate_inc(struct pt8211 *p, unsigned int rate)
{
	/* inc = rate * 32 * 2^32 / bus_hz, rounded */
	return div_u64(((u64)rate << 37) + p->bus_hz / 2, p->bus_hz);
}

/* ALSA requires .prepare; the real setup happens at the start trigger */
static int pt8211_prepare(struct snd_pcm_substream *ss)
{
	struct pt8211 *p = snd_pcm_substream_chip(ss);
	unsigned long flags;

	spin_lock_irqsave(&p->lock, flags);
	p->running = false;
	aud_wr(p, AUD_CTRL, CTRL_AMP_EN | CTRL_FIFO_RST);
	aud_wr(p, AUD_RATE, pt8211_rate_inc(p, ss->runtime->rate));
	spin_unlock_irqrestore(&p->lock, flags);
	return 0;
}

static int pt8211_trigger(struct snd_pcm_substream *ss, int cmd)
{
	struct pt8211 *p = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *rt = ss->runtime;
	unsigned long flags;
	u32 v;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		spin_lock_irqsave(&p->lock, flags);
		/* stop, empty, clear the statistics and the DMA counters */
		aud_wr(p, AUD_CTRL, CTRL_AMP_EN | CTRL_FIFO_RST);
		aud_wr(p, AUD_RATE, pt8211_rate_inc(p, rt->rate));
		p->push_pos = 0;
		p->pushed = 0;
		p->last_appl = 0;
		p->committed = 0;
		p->ring_pos = 0;
		p->last_frames = 0;
		p->running = true;
		if (p->dma) {
			aud_wr(p, AUD_THRESH, 1);		/* level irq = FIFO empty */
			aud_wr(p, AUD_DMA_ADDR, p->dma_buf);
			aud_wr(p, AUD_DMA_SIZE, p->dma_bytes);
			aud_wr(p, AUD_DMA_PERIOD, snd_pcm_lib_period_bytes(ss));
			aud_wr(p, AUD_DMA_FILL, (DMA_MAX_BEATS << 16) | DMA_FILL_FRAMES);
			pt8211_commit(p);
			/* the engine fetches with the serialiser off: let the
			 * first frames arrive before enabling it (no start-up
			 * underrun) and the interrupt (the empty-FIFO level
			 * interrupt would fire right away) */
			aud_wr(p, AUD_CTRL, pt8211_ctrl(p) & ~(CTRL_ENABLE | CTRL_IRQ_EN));
			readl_poll_timeout_atomic(p->base + AUD_STATUS, v, v & 0xffff, 1, 500);
			aud_wr(p, AUD_CTRL, pt8211_ctrl(p));
		} else {
			aud_wr(p, AUD_THRESH, p->thresh);
			pt8211_fill(p);
			aud_wr(p, AUD_CTRL, pt8211_ctrl(p));
		}
		spin_unlock_irqrestore(&p->lock, flags);
		return 0;
	case SNDRV_PCM_TRIGGER_DRAIN:
		/* no more data will come: let the core stop itself at the first
		 * empty frame (DMA mode has no "fetch reached the limit" event,
		 * so without this the tail costs interrupt latency in underruns) */
		spin_lock_irqsave(&p->lock, flags);
		if (p->running && p->dma)
			aud_wr(p, AUD_CTRL, pt8211_ctrl(p) | CTRL_STOP_EMPTY);
		spin_unlock_irqrestore(&p->lock, flags);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		spin_lock_irqsave(&p->lock, flags);
		p->running = false;
		/* stop + mask; the FIFO and the FRAMES/CRC/UNDERRUN/OVERRUN
		 * statistics stay readable until the next start */
		aud_wr(p, AUD_CTRL, CTRL_AMP_EN);
		spin_unlock_irqrestore(&p->lock, flags);
		return 0;
	default:
		return -EINVAL;
	}
}

static snd_pcm_uframes_t pt8211_pointer(struct snd_pcm_substream *ss)
{
	struct pt8211 *p = snd_pcm_substream_chip(ss);

	return aud_rd(p, AUD_FRAMES) % ss->runtime->buffer_size;
}

/* the application advanced appl_ptr: hand the data over and make sure the
 * core and its interrupt are live */
static int pt8211_ack(struct snd_pcm_substream *ss)
{
	struct pt8211 *p = snd_pcm_substream_chip(ss);
	unsigned long flags;

	spin_lock_irqsave(&p->lock, flags);
	if (p->running) {
		if (p->dma)
			pt8211_commit(p);
		else
			aud_wr(p, AUD_THRESH, p->thresh);
		aud_wr(p, AUD_CTRL, pt8211_ctrl(p));
	}
	spin_unlock_irqrestore(&p->lock, flags);
	return 0;
}

static const struct snd_pcm_ops pt8211_ops = {
	.open = pt8211_open,
	.close = pt8211_close,
	.hw_params = pt8211_hw_params,
	.hw_free = pt8211_hw_free,
	.prepare = pt8211_prepare,
	.trigger = pt8211_trigger,
	.pointer = pt8211_pointer,
	.ack = pt8211_ack,
};

static void pt8211_proc_read(struct snd_info_entry *entry,
			     struct snd_info_buffer *buffer)
{
	struct pt8211 *p = entry->private_data;

	snd_iprintf(buffer, "mode %s\n", p->dma ? "dma" : "pio");
	snd_iprintf(buffer, "ctrl 0x%x status 0x%x thresh %u rate 0x%08x\n",
		    aud_rd(p, AUD_CTRL), aud_rd(p, AUD_STATUS),
		    aud_rd(p, AUD_THRESH), aud_rd(p, AUD_RATE));
	snd_iprintf(buffer, "frames %u crc 0x%08x underrun %u overrun %u\n",
		    aud_rd(p, AUD_FRAMES), aud_rd(p, AUD_CRC),
		    aud_rd(p, AUD_UNDERRUN), aud_rd(p, AUD_OVERRUN));
	snd_iprintf(buffer, "irqs %lu fills %lu depth %u bus %u Hz\n",
		    p->irqs, p->fills, p->depth, p->bus_hz);
	if (p->dma)
		snd_iprintf(buffer, "dma addr 0x%08x size %u period %u limit %u fetch %u irq 0x%x fill 0x%x errors %lu\n",
			    aud_rd(p, AUD_DMA_ADDR), aud_rd(p, AUD_DMA_SIZE),
			    aud_rd(p, AUD_DMA_PERIOD), aud_rd(p, AUD_DMA_LIMIT),
			    aud_rd(p, AUD_DMA_FETCH), aud_rd(p, AUD_DMA_IRQ),
			    aud_rd(p, AUD_DMA_FILL), p->dma_errs);
}

static int pt8211_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_card *card;
	struct pt8211 *p;
	struct snd_pcm *pcm;
	int irq, ret;
	u32 id;

	ret = snd_devm_card_new(dev, SNDRV_DEFAULT_IDX1, "PT8211", THIS_MODULE,
				sizeof(*p), &card);
	if (ret < 0)
		return ret;
	p = card->private_data;
	p->dev = dev;
	p->card = card;
	spin_lock_init(&p->lock);

	p->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);
	id = aud_rd(p, AUD_ID);
	if (id != AUD_ID_VALUE) {
		dev_err(dev, "no audio core (ID 0x%08x)\n", id);
		return -ENODEV;
	}
	p->depth = aud_rd(p, AUD_DEPTH) & 0xffff;
	if (!p->depth)
		p->depth = 2048;
	p->thresh = p->depth / 2;
	p->bus_hz = 50000000;
	device_property_read_u32(dev, "clock-frequency", &p->bus_hz);
	p->dma_cap = aud_rd(p, AUD_CAP) & BIT(0);
	p->dma = p->dma_cap && !pio;
	if (p->dma_cap) {
		ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret)
			return ret;
	}
	/* amplifier on from here (not toggled per stream: that would pop) */
	aud_wr(p, AUD_CTRL, CTRL_AMP_EN | CTRL_FIFO_RST);

	if (hwirq > 0) {
		/* the interrupt line's PLIC source, bypassing the DT 'interrupts' */
		struct device_node *parent = of_irq_find_parent(dev->of_node);
		struct irq_fwspec fwspec = {
			.fwnode = of_node_to_fwnode(parent),
			.param_count = 2,
			.param = { hwirq, IRQ_TYPE_LEVEL_HIGH },
		};
		irq = parent ? irq_create_fwspec_mapping(&fwspec) : 0;
		of_node_put(parent);
		if (irq <= 0)
			return -ENXIO;
		dev_info(dev, "using PLIC source %d (module parameter)\n", hwirq);
	} else {
		irq = platform_get_irq(pdev, 0);
		if (irq < 0)
			return irq;
	}
	ret = devm_request_irq(dev, irq, pt8211_irq, 0, dev_name(dev), p);
	if (ret)
		return ret;

	ret = snd_pcm_new(card, "PT8211", 0, 1, 0, &pcm);
	if (ret < 0)
		return ret;
	p->pcm = pcm;
	pcm->private_data = p;
	strscpy(pcm->name, "PT8211 DAC", sizeof(pcm->name));
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &pt8211_ops);
	/* DMA: physically contiguous pages, streaming-mapped in hw_params */
	snd_pcm_set_managed_buffer_all(pcm, p->dma_cap ? SNDRV_DMA_TYPE_CONTINUOUS
						       : SNDRV_DMA_TYPE_VMALLOC,
				       NULL, 64 * 1024, 256 * 1024);

	strscpy(card->driver, "gowin-pt8211", sizeof(card->driver));
	strscpy(card->shortname, "PT8211", sizeof(card->shortname));
	snprintf(card->longname, sizeof(card->longname),
		 "Gowin AE350 PT8211 audio at %pa irq %d (%s)",
		 &pdev->resource[0].start, irq, p->dma_cap ? "DMA" : "PIO");
	snd_card_ro_proc_new(card, "pt8211", p, pt8211_proc_read);

	ret = snd_card_register(card);
	if (ret < 0)
		return ret;
	platform_set_drvdata(pdev, p);
	dev_info(dev, "PT8211 audio: FIFO %u frames, bus %u Hz, irq %d, %s\n",
		 p->depth, p->bus_hz, irq, p->dma_cap ? "DMA engine" : "PIO only");
	return 0;
}

static void pt8211_remove(struct platform_device *pdev)
{
	struct pt8211 *p = platform_get_drvdata(pdev);

	aud_wr(p, AUD_CTRL, 0);		/* serialiser, interrupt and amplifier off */
}

static const struct of_device_id pt8211_of_match[] = {
	{ .compatible = "gowin,ae350-pt8211" },
	{ }
};
MODULE_DEVICE_TABLE(of, pt8211_of_match);

static struct platform_driver pt8211_driver = {
	.probe = pt8211_probe,
	.remove = pt8211_remove,
	.driver = {
		.name = "gowin-pt8211",
		.of_match_table = pt8211_of_match,
	},
};
module_platform_driver(pt8211_driver);

MODULE_DESCRIPTION("Gowin AE350 PT8211 audio output (FIFO + DMA ring)");
MODULE_LICENSE("GPL");
