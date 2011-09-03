/*
 * arch/arm/mach-lpc32xx/pm.c
 *
 * Original authors: Vitaly Wool, Dmitry Chigirev <source@mvista.com>
 * Modified by Kevin Wells <kevin.wells@nxp.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

/*
 * LPC32XX CPU and system power management
 *
 * The LCP32XX has three CPU modes for controlling system power: run,
 * direct-run, and halt modes. When switching between halt and run modes,
 * the CPU transistions through direct-run mode. For Linux, direct-run
 * mode is not used in normal operation. Halt mode is used when the
 * system is fully suspended.
 *
 * Run mode:
 * The ARM CPU clock (HCLK_PLL), HCLK bus clock, and PCLK bus clocks are
 * derived from the HCLK PLL. The HCLK and PCLK bus rates are divided from
 * the HCLK_PLL rate. Linux runs in this mode.
 *
 * Direct-run mode:
 * The ARM CPU clock, HCLK bus clock, and PCLK bus clocks are driven from
 * SYSCLK. SYSCLK is usually around 13MHz, but may vary based on SYSCLK
 * source or the frequency of the main oscillator. In this mode, the
 * HCLK_PLL can be safely enabled, changed, or disabled.
 *
 * Halt mode:
 * SYSCLK is gated off and the CPU and system clocks are halted.
 * Peripherals based on the 32KHz oscillator clock (ie, RTC, touch,
 * key scanner, etc.) still operate if enabled. In this state, an enabled
 * system event (ie, GPIO state change, RTC match, key press, etc.) will
 * wake the system up back into direct-run mode.
 *
 * DRAM refresh
 * DRAM clocking and refresh are slightly different for systems with DDR
 * DRAM or regular SDRAM devices. If SDRAM is used in the system, the
 * SDRAM will still be accessible in direct-run mode. In DDR based systems,
 * a transistion to direct-run mode will stop all DDR accesses (no clocks).
 * Because of this, the code to switch power modes and the code to enter
 * and exit DRAM self-refresh modes must not be executed in DRAM. A small
 * section of IRAM is used instead for this.
 *
 * Suspend is handled with the following logic:
 *  Backup a small area of IRAM used for the suspend code
 *  Copy suspend code to IRAM
 *  Transfer control to code in IRAM
 *  Places DRAMs in self-refresh mode
 *  Enter direct-run mode
 *  Save state of HCLK_PLL PLL
 *  Disable HCLK_PLL PLL
 *  Enter halt mode - CPU and buses will stop
 *  System enters direct-run mode when an enabled event occurs
 *  HCLK PLL state is restored
 *  Run mode is entered
 *  DRAMS are placed back into normal mode
 *  Code execution returns from IRAM
 *  IRAM code are used for suspend is restored
 *  Suspend mode is exited
 */

#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <asm/cacheflush.h>

#include <mach/hardware.h>
#include <mach/platform.h>
#include "common.h"
#include "clock.h"

#define TEMP_IRAM_AREA  IO_ADDRESS(LPC32XX_IRAM_BASE)
static void *iram_swap_area;
static int (*lpc32xx_suspend_ptr) (void);

static inline int lpc32xx_suspend(void)
 {
	/* Backup a small area of IRAM used for the suspend code */
	memcpy(iram_swap_area, (void *) TEMP_IRAM_AREA,
		lpc32xx_sys_suspend_sz);

	/*
	 * Copy code to suspend system into IRAM. The suspend code
	 * needs to run from IRAM as DRAM may no longer be available
	 * when the PLL is stopped.
	 */
	memcpy((void *) TEMP_IRAM_AREA, &lpc32xx_sys_suspend,
		lpc32xx_sys_suspend_sz);
	flush_cache_all();
	flush_icache_range((unsigned long)TEMP_IRAM_AREA,
		(unsigned long)(TEMP_IRAM_AREA) + lpc32xx_sys_suspend_sz);

	/* Transfer to suspend code in IRAM */
	(void) lpc32xx_suspend_ptr();

	/* Restore original IRAM contents */
	memcpy((void *) TEMP_IRAM_AREA, iram_swap_area,
		lpc32xx_sys_suspend_sz);

	return 0;
}

static int lpc32xx_pm_enter(suspend_state_t state)
{
	int ret = 0;

	lpc32xx_irq_suspend(state);

	switch (state)
	{
		case PM_SUSPEND_STANDBY:
			asm("mcr p15, 0, r0, c7, c0, 4");
			break;
		case PM_SUSPEND_MEM:
			ret = lpc32xx_suspend();
			break;
	}

	lpc32xx_irq_resume(state);

	return ret;
}

static int lpc32xx_pm_valid(suspend_state_t state)
{
	return (state == PM_SUSPEND_STANDBY) ||
	       (state == PM_SUSPEND_MEM);
}


static struct platform_suspend_ops lpc32xx_pm_ops = {
	.valid	= lpc32xx_pm_valid,
	.enter	= lpc32xx_pm_enter,
};

#define EMC_DYN_MEM_CTRL_OFS 0x20
#define EMC_SRMMC           (1 << 3)
#define EMC_CTRL_REG io_p2v(LPC32XX_EMC_BASE + EMC_DYN_MEM_CTRL_OFS)
static int __init lpc32xx_pm_init(void)
{
	/*
	 * Setup SDRAM self-refresh clock to automatically disable on
	 * start of self-refresh. This only needs to be done once.
	 */
	__raw_writel(__raw_readl(EMC_CTRL_REG) | EMC_SRMMC, EMC_CTRL_REG);

	/* Allocate some space for temporary IRAM storage */
	iram_swap_area = kmalloc(lpc32xx_sys_suspend_sz, GFP_KERNEL);
	if (!iram_swap_area) {
		printk(KERN_ERR
		       "PM Suspend: cannot allocate memory to save portion "
			"of SRAM\n");
		return -ENOMEM;
	}

	lpc32xx_suspend_ptr = (void *) TEMP_IRAM_AREA;
	suspend_set_ops(&lpc32xx_pm_ops);

	return 0;
}
arch_initcall(lpc32xx_pm_init);
