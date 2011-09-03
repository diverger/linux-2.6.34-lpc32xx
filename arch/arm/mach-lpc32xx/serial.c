/*
 * arch/arm/mach-lpc32xx/serial.c
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2010 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/serial_8250.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/board.h>
#include "common.h"

#define LPC32XX_SUART_FIFO_SIZE	64

/* Standard 8250/16550 compatible serial ports */
static struct plat_serial8250_port serial_std_platform_data[] = {
#ifdef CONFIG_ARCH_LPC32XX_UART5_SELECT
	{
		.membase        = io_p2v(LPC32XX_UART5_BASE),
		.mapbase        = LPC32XX_UART5_BASE,
		.irq		= IRQ_LPC32XX_UART_IIR5,
		.regshift	= 2,
		.iotype		= UPIO_MEM32,
		.flags		= UPF_BOOT_AUTOCONF | UPF_BUGGY_UART |
					UPF_SKIP_TEST,
	},
#endif
#ifdef CONFIG_ARCH_LPC32XX_UART3_SELECT
	{
		.membase	= io_p2v(LPC32XX_UART3_BASE),
		.mapbase        = LPC32XX_UART3_BASE,
		.irq		= IRQ_LPC32XX_UART_IIR3,
		.regshift	= 2,
		.iotype		= UPIO_MEM32,
		.flags		= UPF_BOOT_AUTOCONF | UPF_BUGGY_UART |
					UPF_SKIP_TEST,
	},
#endif
#ifdef CONFIG_ARCH_LPC32XX_UART4_SELECT
	{
		.membase	= io_p2v(LPC32XX_UART4_BASE),
		.mapbase        = LPC32XX_UART4_BASE,
		.irq		= IRQ_LPC32XX_UART_IIR4,
		.regshift	= 2,
		.iotype		= UPIO_MEM32,
		.flags		= UPF_BOOT_AUTOCONF | UPF_BUGGY_UART |
					UPF_SKIP_TEST,
	},
#endif
#ifdef CONFIG_ARCH_LPC32XX_UART6_SELECT
	{
		.membase	= io_p2v(LPC32XX_UART6_BASE),
		.mapbase        = LPC32XX_UART6_BASE,
		.irq		= IRQ_LPC32XX_UART_IIR6,
		.regshift	= 2,
		.iotype		= UPIO_MEM32,
		.flags		= UPF_BOOT_AUTOCONF | UPF_BUGGY_UART |
					UPF_SKIP_TEST,
	},
#endif
	{ },
};

struct uartinit {
	char *uart_ck_name;
	u32 ck_mode_mask;
	void __iomem *pdiv_clk_reg;
	resource_size_t mapbase;
	bool enabled;
};

static struct uartinit uartinit_data[] __initdata = {
	{
		.uart_ck_name = "uart5_ck",
		.ck_mode_mask =
			LPC32XX_UART_CLKMODE_LOAD(LPC32XX_UART_CLKMODE_ON, 5),
		.pdiv_clk_reg = LPC32XX_CLKPWR_UART5_CLK_CTRL,
		.mapbase = LPC32XX_UART5_BASE,
#ifdef CONFIG_ARCH_LPC32XX_UART5_SELECT
		.enabled = true,
#endif
	},
	{
		.uart_ck_name = "uart3_ck",
		.ck_mode_mask =
			LPC32XX_UART_CLKMODE_LOAD(LPC32XX_UART_CLKMODE_ON, 3),
		.pdiv_clk_reg = LPC32XX_CLKPWR_UART3_CLK_CTRL,
		.mapbase = LPC32XX_UART3_BASE,
#ifdef CONFIG_ARCH_LPC32XX_UART3_SELECT
		.enabled = true,
#endif
	},
	{
		.uart_ck_name = "uart4_ck",
		.ck_mode_mask =
			LPC32XX_UART_CLKMODE_LOAD(LPC32XX_UART_CLKMODE_ON, 4),
		.pdiv_clk_reg = LPC32XX_CLKPWR_UART4_CLK_CTRL,
		.mapbase = LPC32XX_UART4_BASE,
#ifdef CONFIG_ARCH_LPC32XX_UART4_SELECT
		.enabled = true,
#endif
	},
	{
		.uart_ck_name = "uart6_ck",
		.ck_mode_mask =
			LPC32XX_UART_CLKMODE_LOAD(LPC32XX_UART_CLKMODE_ON, 6),
		.pdiv_clk_reg = LPC32XX_CLKPWR_UART6_CLK_CTRL,
		.mapbase = LPC32XX_UART6_BASE,
#ifdef CONFIG_ARCH_LPC32XX_UART6_SELECT
		.enabled = true,
#endif
	},
};

static struct platform_device serial_std_platform_device = {
	.name			= "serial8250",
	.id			= 0,
	.dev			= {
		.platform_data	= serial_std_platform_data,
	},
};

/* High speed serial ports */
static struct lpc32xx_hsuart_port serial_hspd_platform_data[] = {
#ifdef CONFIG_ARCH_LPC32XX_HSUART1_SELECT
	{
		.port 					= {
			.membase        = io_p2v(LPC32XX_HS_UART1_BASE),
			.mapbase        = LPC32XX_HS_UART1_BASE,
			.irq            = IRQ_LPC32XX_UART_IIR1,
			.regshift       = 2,
			.iotype         = UPIO_MEM32,
			.flags          = UPF_BOOT_AUTOCONF,
		},
		.fbit_sam       = 20,
	},
#endif
#ifdef CONFIG_ARCH_LPC32XX_HSUART2_SELECT
	{
		.port 					= {
			.membase        = io_p2v(LPC32XX_HS_UART2_BASE),
			.mapbase        = LPC32XX_HS_UART2_BASE,
			.irq            = IRQ_LPC32XX_UART_IIR2,
			.regshift       = 2,
			.iotype         = UPIO_MEM32,
			.flags          = UPF_BOOT_AUTOCONF,
		},
		.fbit_sam       = 20,
	},
#endif
#ifdef CONFIG_ARCH_LPC32XX_HSUART7_SELECT
	{
		.port 					= {
			.membase        = io_p2v(LPC32XX_HS_UART7_BASE),
			.mapbase        = LPC32XX_HS_UART7_BASE,
			.irq            = IRQ_LPC32XX_UART_IIR7,
			.regshift       = 2,
			.iotype         = UPIO_MEM32,
			.flags          = UPF_BOOT_AUTOCONF,
		},
		.fbit_sam       = 20,
	},
#endif
	{ },
};

static struct platform_device serial_hspd_platform_device = {
	.name                   = "lpc32xx_hsuart",
	.id                     = 0,
	.dev                    = {
		.platform_data  = serial_hspd_platform_data,
	},
};

static struct platform_device *lpc32xx_serial_devs[] __initdata = {
	&serial_std_platform_device,
	&serial_hspd_platform_device,
};

void __init lpc32xx_serial_init(void)
{
	u32 tmp, tmpclk, clkmodes = 0, clkmodesdone = 0;
	struct clk *clk;
	resource_size_t puart;
	int i, j, k = 0;

	/*
	 * For each standard UART, enable the UART clock, and force
	 * FIFO flush. This works around a HW bug with the 8250 where
	 * the FIFOs may not work correctly. This should be done even if
	 * the standard UARTs are not used. The clocks will be disabled
	 * when done.
	 */
	for (i = 0; i < ARRAY_SIZE(uartinit_data); i++) {
		clk = clk_get(NULL, uartinit_data[i].uart_ck_name);
		if (!IS_ERR(clk)) {
			clk_enable(clk);
			tmpclk = clk_get_rate(clk);

			/*
			 * Fall back on main osc rate if clock rate return
			 * fails
			 */
			if (tmpclk == 0)
				tmpclk = LPC32XX_MAIN_OSC_FREQ;

			/* pre-UART clock divider set to 1 */
			__raw_writel(0x0101, uartinit_data[i].pdiv_clk_reg);

			/* Setup UART clock modes for all UART to always on */
			clkmodes = uartinit_data[i].ck_mode_mask;
			__raw_writel(clkmodes, LPC32XX_UARTCTL_CLKMODE);

			/*
			 * Force a flush of the RX FIFOs to work around a
			 * HW bug
			 */
			puart = uartinit_data[i].mapbase;
			__raw_writel(0xC1, LPC32XX_UART_IIR_FCR(puart));
			__raw_writel(0x00, LPC32XX_UART_DLL_FIFO(puart));
			j = LPC32XX_SUART_FIFO_SIZE;
			while (j--)
				tmp = __raw_readl(
					LPC32XX_UART_DLL_FIFO(puart));
			__raw_writel(0, LPC32XX_UART_IIR_FCR(puart));

			/*
			 * The 8250 serial drivers do not use any type of
			 * clock management, so the clocks need to be
			 * pre-enabled here before the 8250 driver attempts
			 * to access the peripherals. If a UART is enabled,
			 * just leave the clock on.
			 */
			if (uartinit_data[i].enabled) {
				clkmodesdone |= clkmodes;
				serial_std_platform_data[k++].uartclk = tmpclk;
			} else {
				clk_disable(clk);
				clk_put(clk);
			}
		}
	}

	/* Enable clocks only for enabled UARTs */
	__raw_writel(clkmodesdone, LPC32XX_UARTCTL_CLKMODE);

	/*
	 * Get current PCLK rate and use it for the base clock for all the
	 * high speed UARTS
	 */
	tmpclk = 0;
	clk = clk_get(NULL, "pclk_ck");
	if (!IS_ERR(clk)) {
		tmpclk = clk_get_rate(clk);
		clk_put(clk);
	}
	if (!tmpclk)
		tmpclk = LPC32XX_MAIN_OSC_FREQ;

        /* Setup of HSUART devices */
        for (i = 0; i < ARRAY_SIZE(serial_hspd_platform_data) - 1; i++) {
                serial_hspd_platform_data[i].port.line = i;
                serial_hspd_platform_data[i].port.uartclk = tmpclk;
	}

	/* Disable UART5->USB transparent mode or USB won't work */
	tmp = __raw_readl(LPC32XX_UARTCTL_CTRL);
	tmp &= ~LPC32XX_UART_U5_ROUTE_TO_USB;
	__raw_writel(tmp, LPC32XX_UARTCTL_CTRL);

	platform_add_devices(lpc32xx_serial_devs,
		ARRAY_SIZE(lpc32xx_serial_devs));
}
