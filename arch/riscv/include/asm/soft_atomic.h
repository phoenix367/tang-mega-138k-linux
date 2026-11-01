/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Gowin AE350 (Andes A25) single-hart soft atomics.
 *
 * This SoC has no coherency agent (mcache_ctl.DC_COHEN is hardwired 0), so
 * RISC-V AMO / LR-SC exclusive accesses to DRAM never complete: amoswap hangs
 * and sc never succeeds. The system is single-hart, so the only concurrency is
 * with traps. Every read-modify-write is therefore made uninterruptible by
 * clearing sstatus.SIE, performed as a plain load/store, then SIE is restored.
 * (Ordering vs. devices/DMA is still provided by the fences in the callers.)
 *
 * See CACHE_ISSUES.md in the gateware project.
 */
#ifndef _ASM_RISCV_SOFT_ATOMIC_H
#define _ASM_RISCV_SOFT_ATOMIC_H

#define __SOFT_SR_SIE 0x2UL	/* sstatus.SIE */

#define __soft_atomic_begin(__sf)					\
	__asm__ __volatile__("csrrc %0, sstatus, %1"			\
			     : "=r"(__sf) : "r"(__SOFT_SR_SIE) : "memory")

#define __soft_atomic_end(__sf)						\
	__asm__ __volatile__("csrs sstatus, %0"				\
			     :: "r"((__sf) & __SOFT_SR_SIE) : "memory")

/* Map an asm op-mnemonic token to the equivalent C operator. */
#define __soft_op_add(a, b)	((a) + (b))
#define __soft_op_and(a, b)	((a) & (b))
#define __soft_op_or(a, b)	((a) | (b))
#define __soft_op_xor(a, b)	((a) ^ (b))

#endif /* _ASM_RISCV_SOFT_ATOMIC_H */
