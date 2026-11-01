/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Andes A25 (Gowin AE350): page-table stores are written back to DRAM.
 *
 * The A25 hardware page-table walker issues its own (uncached) bus reads, and
 * the AE350 has no coherence manager, so a page-table line that is still dirty
 * in the L1 D-cache can be walked from stale DRAM.  Every store to a page table
 * is therefore followed by an L1D VA writeback (the same S-mode CCTL CSRs as
 * arch/riscv/mm/cmo-andes-a25.c, which need mcache_ctl.CCTL_SUEN from M-mode).
 *
 * Runtime switch: /sys/module/kernel/parameters/a25_ptwb (boot: a25_ptwb=0).
 */
#ifndef _ASM_RISCV_A25_PTWB_H
#define _ASM_RISCV_A25_PTWB_H

#include <linux/compiler.h>
#include <asm/csr.h>
#include <asm/barrier.h>

#define A25_UCCTLBEGINADDR	0x80b
#define A25_UCCTLCOMMAND	0x80c
#define A25_CCTL_L1D_VA_WB	1
#define A25_PTWB_LINE		32

extern int a25_ptwb;

/* write back the D-cache lines covering [p, p + size) */
static inline void a25_ptwb_range(const void *p, unsigned long size)
{
	unsigned long va, end;

	if (!READ_ONCE(a25_ptwb))
		return;
	va  = (unsigned long)p & ~(A25_PTWB_LINE - 1UL);
	end = (unsigned long)p + size;
	mb();				/* the page-table stores are in the D-cache */
	for (; va < end; va += A25_PTWB_LINE) {
		csr_write(A25_UCCTLBEGINADDR, va);
		csr_write(A25_UCCTLCOMMAND, A25_CCTL_L1D_VA_WB);
	}
	mb();				/* ... and the writebacks are out */
}

static inline void a25_ptwb_entry(const void *p)
{
	a25_ptwb_range(p, sizeof(unsigned long));
}

#endif /* _ASM_RISCV_A25_PTWB_H */
