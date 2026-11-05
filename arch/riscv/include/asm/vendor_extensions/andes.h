/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_RISCV_VENDOR_EXTENSIONS_ANDES_H
#define _ASM_RISCV_VENDOR_EXTENSIONS_ANDES_H

#include <asm/vendor_extensions.h>

#include <linux/types.h>

#define RISCV_ISA_VENDOR_EXT_XANDESPMU		0
#define RISCV_ISA_VENDOR_EXT_XANDESPERF		1
#define RISCV_ISA_VENDOR_EXT_XANDESCODENSE	2

/*
 * Extension keys should be strictly less than max.
 * It is safe to increment this when necessary.
 */
#define RISCV_ISA_VENDOR_EXT_MAX_ANDES			32

extern struct riscv_isa_vendor_ext_data_list riscv_isa_vendor_ext_list_andes;

/* user CSRs of the instruction extensions, switched per task (asm/switch_to.h) */
#define CSR_ANDES_UITB		0x800	/* CoDense: exec.it table base */
#define CSR_ANDES_UCODE		0x801	/* DSP (P draft v0.5): overflow / saturation flag */

/* P on an Andes core: v0.5, whose state is ucode (other P drafts keep it elsewhere) */
extern bool riscv_andes_dsp;

#endif
