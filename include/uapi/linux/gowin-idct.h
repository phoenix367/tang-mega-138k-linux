/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * gowin-idct: the hardware IDCT / frame assembly core of the Tang Mega 138K
 * Pro AE350 gateware (src/rtl/idct/), /dev/gowin-idct.
 *
 * The user mmap()s the record ring (cached memory, offset 0, GOWIN_IDCT_INFO
 * gives its size), writes records as the gateware defines them (a header
 * word, then the coefficients four per 64-bit word), and SUBMITs the new
 * tail: the driver writes the ring range back from the CPU cache and hands
 * it to the core.  Flow control is the user's: a record must not be written
 * over ring space the core has not consumed (head from SUBMIT / STATUS /
 * WAIT).  SETUP (plane addresses from the LCD driver's YUV buffers,
 * GOWIN_LCD_IOC_YUV_DMA) waits for the core to go idle first.  WAIT sleeps
 * until everything submitted has been transformed and flushed to DRAM.
 */
#ifndef _UAPI_LINUX_GOWIN_IDCT_H
#define _UAPI_LINUX_GOWIN_IDCT_H

#include <linux/types.h>
#include <linux/ioctl.h>

/* record header word (the rest zero): n coefficients follow, four per word */
#define GOWIN_IDCT_HDR(n, flush, plane, x, y, irq) \
	((__u64)((n) & 0x7f) | ((__u64)((flush) & 1) << 7) | ((__u64)((plane) & 3) << 8) | \
	 ((__u64)((x) & 0xfff) << 10) | ((__u64)((y) & 0xfff) << 22) | ((__u64)((irq) & 1) << 34))
/* flush-only record: the plane's band if it is dirty, no block (ends a frame) */
#define GOWIN_IDCT_HDR_FLUSH(plane, irq) \
	(((__u64)((plane) & 3) << 8) | ((__u64)((irq) & 1) << 34) | ((__u64)1 << 35))

/*
 * JPEG records (cores with GOWIN_IDCT_ST_JPEG in the STATUS register): the core's
 * entropy decoder takes baseline 4:2:0 scans. Payload bytes follow the header,
 * eight per word, the first in bits [7:0]:
 *   HUFF  id = slot (0 DC 0, 1 DC 1, 2 AC 0, 3 AC 1): the DHT counts (16 bytes) + values
 *   QUANT id = table 0..3: 64 bytes, zigzag order (8-bit DQT)
 *   SCAN  one parameter word (GOWIN_IDCT_SCAN_PARAM), then the entropy-coded bytes
 *         exactly as in the file (between the SOS header and the EOI marker)
 */
#define GOWIN_IDCT_ST_JPEG		(1u << 8)
#define GOWIN_IDCT_REC_HUFF		1
#define GOWIN_IDCT_REC_QUANT		2
#define GOWIN_IDCT_REC_SCAN		3
#define GOWIN_IDCT_HDR_JPEG(type, id, bytes) \
	(((__u64)((type) & 15) << 36) | ((__u64)((id) & 3) << 8) | ((__u64)((bytes) & 0xffffff) << 40))
/* tsel: {Cr AC, Cr DC, Cb AC, Cb DC, Y AC, Y DC} table bits; tq: {Cr, Cb, Y} 2 bits each */
#define GOWIN_IDCT_SCAN_PARAM(mcux, mcuy, restart, tsel, tq) \
	((__u64)((mcux) & 0xff) | ((__u64)((mcuy) & 0xff) << 8) | ((__u64)((restart) & 0xffff) << 16) | \
	 ((__u64)((tsel) & 0x3f) << 32) | ((__u64)((tq) & 0x3f) << 38))

struct gowin_idct_info {
	__u32 ring_size;	/* bytes allocated, a power of two; mmap() offset 0 (GOWIN_IDCT_IOC_RING can use less) */
	__u32 id;		/* the core's ID register (0x49444354) */
	__u32 max_width;	/* luma pixels the band RAM holds per line */
	__u32 reserved;
};

struct gowin_idct_setup {
	__u32 y_base, u_base, v_base;	/* DMA addresses of the planes, 8-byte aligned */
	__u32 y_stride, c_stride;	/* bytes, multiples of 8 */
	__u32 width, height;		/* luma pixels: width a multiple of 16 and <= max_width */
};

struct gowin_idct_submit {
	__u32 tail;		/* in: byte offset in the ring past the last record written (8-aligned) */
	__u32 head;		/* out: offset the core has consumed up to */
};

struct gowin_idct_status {
	__u32 status;		/* the STATUS register: [0] busy [3] a bad record was skipped */
	__u32 head, tail;
	__u32 blocks, bands;	/* the core's counters */
	__u32 irqs;		/* done interrupts taken by the driver */
};

#define GOWIN_IDCT_IOC_MAGIC	'J'
#define GOWIN_IDCT_IOC_INFO	_IOR(GOWIN_IDCT_IOC_MAGIC, 1, struct gowin_idct_info)
#define GOWIN_IDCT_IOC_SETUP	_IOW(GOWIN_IDCT_IOC_MAGIC, 2, struct gowin_idct_setup)
#define GOWIN_IDCT_IOC_SUBMIT	_IOWR(GOWIN_IDCT_IOC_MAGIC, 3, struct gowin_idct_submit)
#define GOWIN_IDCT_IOC_WAIT	_IOWR(GOWIN_IDCT_IOC_MAGIC, 4, __u32)	/* in: timeout ms; out: head */
#define GOWIN_IDCT_IOC_STATUS	_IOR(GOWIN_IDCT_IOC_MAGIC, 5, struct gowin_idct_status)
#define GOWIN_IDCT_IOC_RESET	_IO(GOWIN_IDCT_IOC_MAGIC, 6)	/* abort: pointers to 0, core idle */
#define GOWIN_IDCT_IOC_RING	_IOW(GOWIN_IDCT_IOC_MAGIC, 7, __u32)	/* use only the first N bytes of the ring (power of two, >= 4096; resets the pointers) */

#endif /* _UAPI_LINUX_GOWIN_IDCT_H */
