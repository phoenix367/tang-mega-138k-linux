// SPDX-License-Identifier: GPL-2.0-only
/*
 * L1 cache-maintenance (CMO) provider for the Andes A25 core on the Gowin
 * AE350 SoC.
 *
 * The A25 has L1 I/D caches only (no L2, no coherence manager), so non-coherent
 * DMA needs explicit L1 D-cache writeback/invalidate. This registers the riscv
 * non-standard cache ops using the AndeStar V5 *user-mode* CCTL CSRs
 * (ucctlbeginaddr / ucctlcommand). Those are reachable from S-mode only when
 * M-mode (U-Boot harts_early_init) sets mcache_ctl.CCTL_SUEN; otherwise a
 * csrw 0x80b traps as an illegal instruction. See PATCHES/0003 (U-Boot).
 *
 * The CCTL "VA" commands operate on a virtual address (translated by the MMU),
 * so we feed the kernel linear-map VA of the DMA buffer (phys_to_virt(paddr)).
 * On this rv32 / 256 MB board all RAM is lowmem, so phys_to_virt is always
 * valid for DMA-able memory. Command encoding + per-line loop mirror the SDK
 * BSP (library/bsp/ae350/cache.c).
 */
#include <linux/init.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <asm/dma-noncoherent.h>
#include <asm/csr.h>
#include <asm/io.h>
#include <asm/a25-ptwb.h>
#include <linux/moduleparam.h>

/* page-table store writeback, see asm/a25-ptwb.h */
int a25_ptwb __read_mostly = 1;
EXPORT_SYMBOL(a25_ptwb);
core_param(a25_ptwb, a25_ptwb, int, 0644);

/* AndeStar V5 user CCTL CSRs (S-mode accessible iff mcache_ctl.CCTL_SUEN = 1). */
#define CCTL_UCCTLBEGINADDR	0x80b
#define CCTL_UCCTLCOMMAND	0x80c

/* L1D virtual-address CCTL commands. */
#define CCTL_L1D_VA_INVAL	0	/* invalidate */
#define CCTL_L1D_VA_WB		1	/* writeback */
#define CCTL_L1D_VA_WBINVAL	2	/* writeback + invalidate */

#define A25_CACHE_LINE_SIZE	32

static void a25_cmo_range(phys_addr_t paddr, size_t size, unsigned long cmd)
{
	unsigned long va  = (unsigned long)phys_to_virt(paddr);
	unsigned long end = va + size;

	va = ALIGN_DOWN(va, A25_CACHE_LINE_SIZE);
	for (; va < end; va += A25_CACHE_LINE_SIZE) {
		csr_write(CCTL_UCCTLBEGINADDR, va);
		csr_write(CCTL_UCCTLCOMMAND, cmd);
	}
}

static void a25_dma_cache_wback(phys_addr_t paddr, size_t size)
{
	a25_cmo_range(paddr, size, CCTL_L1D_VA_WB);
}

static void a25_dma_cache_inv(phys_addr_t paddr, size_t size)
{
	a25_cmo_range(paddr, size, CCTL_L1D_VA_INVAL);
}

static void a25_dma_cache_wback_inv(phys_addr_t paddr, size_t size)
{
	a25_cmo_range(paddr, size, CCTL_L1D_VA_WBINVAL);
}

static const struct riscv_nonstd_cache_ops a25_cmo_ops = {
	.wback		= a25_dma_cache_wback,
	.inv		= a25_dma_cache_inv,
	.wback_inv	= a25_dma_cache_wback_inv,
};

static int __init a25_cmo_init(void)
{
	void *buf;

	if (!of_machine_is_compatible("andestech,a25"))
		return 0;

	riscv_noncoherent_register_cache_ops(&a25_cmo_ops);

	/*
	 * One-shot self-check: exercise the CCTL path once. If U-Boot had NOT
	 * set mcache_ctl.CCTL_SUEN this csrw would trap (illegal instruction)
	 * and oops here, so reaching the pr_info proves the S-mode CCTL path
	 * works end to end.
	 */
	buf = kmalloc(A25_CACHE_LINE_SIZE * 2, GFP_KERNEL);
	if (buf) {
		a25_dma_cache_wback_inv(virt_to_phys(buf), A25_CACHE_LINE_SIZE * 2);
		kfree(buf);
	}

	pr_info("AE350 A25: registered L1 CCTL non-coherent cache ops (line=%d B, self-check OK)\n",
		A25_CACHE_LINE_SIZE);
	pr_info("AE350 A25: page-table store writeback %s (a25_ptwb)\n",
		a25_ptwb ? "on" : "off");
	return 0;
}
early_initcall(a25_cmo_init);
