// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2012 Regents of the University of California
 */

#include <linux/reboot.h>
#include <linux/pm.h>
#include <linux/irqflags.h>

static void default_power_off(void)
{
	/*
	 * Final halt (e.g. `halt`, or poweroff when SBI SRST is unavailable).
	 * Disable interrupts first: a wfi loop with interrupts enabled keeps
	 * waking for timer ticks while the scheduler/watchdog can't run, which
	 * trips the soft-lockup detector. With interrupts off the hart parks
	 * silently. (Single-hart: nothing else needs servicing.)
	 */
	local_irq_disable();
	while (1)
		wait_for_interrupt();
}

void (*pm_power_off)(void) = NULL;
EXPORT_SYMBOL(pm_power_off);

void machine_restart(char *cmd)
{
	do_kernel_restart(cmd);
	while (1);
}

void machine_halt(void)
{
	do_kernel_power_off();
	default_power_off();
}

void machine_power_off(void)
{
	do_kernel_power_off();
	default_power_off();
}
