/*
 * arm/arch/mach-lpc32xx/include/mach/board.h
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


#ifndef __ASM_ARCH_BOARD_H
#define __ASM_ARCH_BOARD_H

#include "platform.h"
#include <linux/mtd/partitions.h>
#include <linux/serial_core.h>

/*
 * NAND platform configuration structure
*/
typedef int (*en_wp)(int);
struct lpc32XX_nand_cfg
{
        u32             wdr_clks;
        u32             wwidth;
        u32             whold;
        u32             wsetup;
        u32             rdr_clks;
        u32             rwidth;
        u32             rhold;
        u32             rsetup;
	bool		use_bbt;
	bool		polled_completion;
        en_wp           enable_write_prot;
        struct mtd_partition* (*partition_info)(int, int*);
};

/*
 * Key scanner platform configuration structure
 */
struct lpc32XX_kscan_cfg {
	u32	matrix_sz;	/* Size of matrix in XxY, ie. 3 = 3x3 */
	int	*keymap;	/* Pointer to key map for the scan matrix */
	u32	deb_clks;	/* Debounce clocks (based on 32KHz clock) */
	u32	scan_delay;	/* Scan delay (based on 32KHz clock) */
};

/*
 * Network configuration structure
 */
struct lpc_net_cfg {
	int	phy_irq;	/* PHY IRQ number, or -1 for polling */
	u32	phy_mask;	/* PHY mask value */
};

/*
 * USB device configuration structure
 */
typedef void (*usc_chg_event)(int);
struct lpc32xx_usbd_cfg
{
        int     vbus_drv_pol;   /* 0=active low drive for VBUS via ISP1301 */
        usc_chg_event conn_chgb; /* Connection change event callback (optional) */
        usc_chg_event susp_chgb; /* Suspend/resume event callback (optional) */
        usc_chg_event rmwk_chgb; /* Enable/disable remote wakeup */
};

/*
 * High Speed UART configuration structure
 */
struct lpc32xx_hsuart_port {
	struct uart_port port;
	unsigned int fbit_sam;
};

#endif	/* __ASM_ARCH_BOARD_H */

