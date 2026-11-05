// SPDX-License-Identifier: GPL-2.0-only

#include <asm/cpufeature.h>
#include <asm/sbi.h>
#include <asm/vendor_extensions.h>
#include <asm/vendor_extensions/andes.h>
#include <asm/vendorid_list.h>

#include <linux/array_size.h>
#include <linux/cache.h>
#include <linux/init.h>
#include <linux/types.h>

/*
 * All Andes vendor extensions supported in Linux.  The instruction extensions are
 * advertised (/proc/cpuinfo) for userspace; the kernel does not use them itself.
 * Names as the Andes toolchain spells them in -march.
 */
static const struct riscv_isa_ext_data riscv_isa_vendor_ext_andes[] = {
	__RISCV_ISA_EXT_DATA(xandespmu, RISCV_ISA_VENDOR_EXT_XANDESPMU),
	/* AndeStar V5 performance extension (mmsc_cfg.EV5PE): lea, bbc/bbs, beqc/bnec,
	 * bfos/bfoz, ffb/ffzmism/ffmism/flmism, addigp/l*gp/s*gp */
	__RISCV_ISA_EXT_DATA(xandesperf, RISCV_ISA_VENDOR_EXT_XANDESPERF),
	/* CoDense code-size extension (mmsc_cfg.ECD): exec.it */
	__RISCV_ISA_EXT_DATA(xandescodense, RISCV_ISA_VENDOR_EXT_XANDESCODENSE),
};

struct riscv_isa_vendor_ext_data_list riscv_isa_vendor_ext_list_andes = {
	.ext_data_count = ARRAY_SIZE(riscv_isa_vendor_ext_andes),
	.ext_data = riscv_isa_vendor_ext_andes,
};

bool riscv_andes_dsp __ro_after_init;

/* decided once, before the first user task: switch_to() then only tests a bool */
static int __init riscv_andes_dsp_init(void)
{
	riscv_andes_dsp = riscv_isa_extension_available(NULL, p) &&
			  riscv_cached_mvendorid(0) == ANDES_VENDOR_ID;
	return 0;
}
arch_initcall(riscv_andes_dsp_init);
