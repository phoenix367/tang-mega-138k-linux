/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 Regents of the University of California
 */

#ifndef _ASM_RISCV_SWITCH_TO_H
#define _ASM_RISCV_SWITCH_TO_H

#include <linux/jump_label.h>
#include <linux/sched/task_stack.h>
#include <linux/mm_types.h>
#include <asm/vector.h>
#include <asm/cpufeature.h>
#include <asm/processor.h>
#include <asm/ptrace.h>
#include <asm/csr.h>

#ifdef CONFIG_FPU
extern void __fstate_save(struct task_struct *save_to);
extern void __fstate_restore(struct task_struct *restore_from);

static inline void __fstate_clean(struct pt_regs *regs)
{
	regs->status = (regs->status & ~SR_FS) | SR_FS_CLEAN;
}

static inline void fstate_off(struct task_struct *task,
			      struct pt_regs *regs)
{
	regs->status = (regs->status & ~SR_FS) | SR_FS_OFF;
}

static inline void fstate_save(struct task_struct *task,
			       struct pt_regs *regs)
{
	if ((regs->status & SR_FS) == SR_FS_DIRTY) {
		__fstate_save(task);
		__fstate_clean(regs);
	}
}

static inline void fstate_restore(struct task_struct *task,
				  struct pt_regs *regs)
{
	if ((regs->status & SR_FS) != SR_FS_OFF) {
		__fstate_restore(task);
		__fstate_clean(regs);
	}
}

static inline void __switch_to_fpu(struct task_struct *prev,
				   struct task_struct *next)
{
	struct pt_regs *regs;

	regs = task_pt_regs(prev);
	fstate_save(prev, regs);
	fstate_restore(next, task_pt_regs(next));
}

static __always_inline bool has_fpu(void)
{
	return riscv_has_extension_likely(RISCV_ISA_EXT_f) ||
		riscv_has_extension_likely(RISCV_ISA_EXT_d);
}
#else
static __always_inline bool has_fpu(void) { return false; }
#define fstate_save(task, regs) do { } while (0)
#define fstate_restore(task, regs) do { } while (0)
#define __switch_to_fpu(__prev, __next) do { } while (0)
#endif

extern struct task_struct *__switch_to(struct task_struct *,
				       struct task_struct *);

static inline bool switch_to_should_flush_icache(struct task_struct *task)
{
#ifdef CONFIG_SMP
	bool stale_mm = task->mm && task->mm->context.force_icache_flush;
	bool stale_thread = task->thread.force_icache_flush;
	bool thread_migrated = smp_processor_id() != task->thread.prev_cpu;

	return thread_migrated && (stale_mm || stale_thread);
#else
	return false;
#endif
}

#ifdef CONFIG_SMP
#define __set_prev_cpu(thread) ((thread).prev_cpu = smp_processor_id())
#else
#define __set_prev_cpu(thread)
#endif

#ifdef CONFIG_RISCV_ISA_VENDOR_EXT_ANDES
#include <asm/vendor_extensions.h>
#include <asm/vendor_extensions/andes.h>
#include <asm/vendorid_list.h>

/*
 * The user CSRs of the Andes extensions /proc/cpuinfo advertises belong to the
 * task, like the FPU registers: without this, a process saw another's DSP
 * overflow flag or exec.it table (src/apps/tests/ucsr_test: 31-86 % of reads).
 */
static __always_inline bool andes_has_uitb(void)
{
	return riscv_has_vendor_extension_unlikely(ANDES_VENDOR_ID, RISCV_ISA_VENDOR_EXT_XANDESCODENSE);
}

static inline void andes_ustate_save(struct task_struct *t)
{
	if (riscv_andes_dsp)
		t->thread.andes_ucode = csr_read(CSR_ANDES_UCODE);
	if (andes_has_uitb())
		t->thread.andes_uitb = csr_read(CSR_ANDES_UITB);
}

static inline void andes_ustate_restore(struct task_struct *t)
{
	if (riscv_andes_dsp)
		csr_write(CSR_ANDES_UCODE, t->thread.andes_ucode);
	if (andes_has_uitb())
		csr_write(CSR_ANDES_UITB, t->thread.andes_uitb);
}

/* exec: a new program starts with a clear flag and no exec.it table */
static inline void andes_ustate_reset(struct task_struct *t)
{
	t->thread.andes_ucode = 0;
	t->thread.andes_uitb = 0;
	andes_ustate_restore(t);
}
#else
static inline void andes_ustate_save(struct task_struct *t) { }
static inline void andes_ustate_restore(struct task_struct *t) { }
static inline void andes_ustate_reset(struct task_struct *t) { }
#endif

#define switch_to(prev, next, last)			\
do {							\
	struct task_struct *__prev = (prev);		\
	struct task_struct *__next = (next);		\
	__set_prev_cpu(__prev->thread);			\
	if (has_fpu())					\
		__switch_to_fpu(__prev, __next);	\
	if (has_vector())					\
		__switch_to_vector(__prev, __next);	\
	andes_ustate_save(__prev);			\
	andes_ustate_restore(__next);			\
	if (switch_to_should_flush_icache(__next))	\
		local_flush_icache_all();		\
	((last) = __switch_to(__prev, __next));		\
} while (0)

#endif /* _ASM_RISCV_SWITCH_TO_H */
