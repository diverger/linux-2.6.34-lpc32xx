/*
 * arch/arm/mach-lpc32xx/include/mach/debug-macro.S
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

/*
 * Debug output is hardcoded to standard UART 5
*/

#if defined(CONFIG_SERIAL_HS_LPC32XX_CONSOLE)
		.macro	addruart,rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		ldreq	\rx, =0x40014000	@ physical
		ldrne	\rx, =0xF4014000	@ virtual
1003:
		.endm

		.macro	senduart,rd,rx
		str	\rd, [\rx, #0]
		.endm

		.macro	busyuart,rd,rx
1002:		ldr	\rd, [\rx, #0x0]
		ldr	\rd, [\rx, #0x4]
		cmp	\rd, #0
		bne	1002b
		.endm

		.macro	waituart,rd,rx
1001:		ldr	\rd, [\rx, #0x0]
		ldr	\rd, [\rx, #0x4]
		cmp	\rd, #0
		bne	1001b
		.endm

#else
	.macro	addruart,rx, tmp
	mrc	p15, 0, \rx, c1, c0
	tst	\rx, #1				@ MMU enabled?
	ldreq	\rx, =0x40090000
	ldrne	\rx, =0xF4090000
	.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
#endif

