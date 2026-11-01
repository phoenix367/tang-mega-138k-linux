/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2006  Ralf Baechle (ralf@linux-mips.org)
 * Copyright (c) 2018  Jim Wilson (jimw@sifive.com)
 */

#ifndef _ASM_RISCV_FUTEX_H
#define _ASM_RISCV_FUTEX_H

#include <linux/futex.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <asm/asm.h>
#include <asm/asm-extable.h>
#include <asm/soft_atomic.h>

/* We don't even really need the extable code, but for now keep it simple */
#ifndef CONFIG_MMU
#define __enable_user_access()		do { } while (0)
#define __disable_user_access()		do { } while (0)
#endif

/*
 * Gowin AE350 single-hart soft futex atomics (no hardware atomics; see
 * soft_atomic.h). The user RMW is made atomic w.r.t. traps by clearing
 * sstatus.SIE around an extable-protected load + store.
 */
static inline int
arch_futex_atomic_op_inuser(int op, int oparg, int *oval, u32 __user *uaddr)
{
	int oldval = 0, newval, ret = 0;
	unsigned long sf;

	if (!access_ok(uaddr, sizeof(u32)))
		return -EFAULT;

	__enable_user_access();
	__soft_atomic_begin(sf);
	__asm__ __volatile__ (
	"1:	lw %[ov], %[u]				\n"
	"2:						\n"
	_ASM_EXTABLE_UACCESS_ERR(1b, 2b, %[r])
	: [r] "+r" (ret), [ov] "=&r" (oldval)
	: [u] "m" (*uaddr)
	: "memory");
	if (!ret) {
		switch (op) {
		case FUTEX_OP_SET:  newval = oparg; break;
		case FUTEX_OP_ADD:  newval = oldval + oparg; break;
		case FUTEX_OP_OR:   newval = oldval | oparg; break;
		case FUTEX_OP_ANDN: newval = oldval & ~oparg; break;
		case FUTEX_OP_XOR:  newval = oldval ^ oparg; break;
		default:	    newval = oldval; ret = -ENOSYS; break;
		}
		if (!ret)
			__asm__ __volatile__ (
			"1:	sw %[nv], %[u]			\n"
			"2:						\n"
			_ASM_EXTABLE_UACCESS_ERR(1b, 2b, %[r])
			: [r] "+r" (ret), [u] "+m" (*uaddr)
			: [nv] "r" (newval)
			: "memory");
	}
	__soft_atomic_end(sf);
	__disable_user_access();

	if (!ret)
		*oval = oldval;

	return ret;
}

static inline int
futex_atomic_cmpxchg_inatomic(u32 *uval, u32 __user *uaddr,
			      u32 oldval, u32 newval)
{
	int ret = 0;
	u32 val = 0;
	unsigned long sf;

	if (!access_ok(uaddr, sizeof(u32)))
		return -EFAULT;

	__enable_user_access();
	__soft_atomic_begin(sf);
	__asm__ __volatile__ (
	"1:	lw %[v], %[u]				\n"
	"	bne %[v], %z[ov], 2f			\n"
	"3:	sw %z[nv], %[u]				\n"
	"2:						\n"
		_ASM_EXTABLE_UACCESS_ERR(1b, 2b, %[r])
		_ASM_EXTABLE_UACCESS_ERR(3b, 2b, %[r])
	: [r] "+r" (ret), [v] "=&r" (val), [u] "+m" (*uaddr)
	: [ov] "Jr" (oldval), [nv] "Jr" (newval)
	: "memory");
	__soft_atomic_end(sf);
	__disable_user_access();

	*uval = val;
	return ret;
}

#endif /* _ASM_RISCV_FUTEX_H */
