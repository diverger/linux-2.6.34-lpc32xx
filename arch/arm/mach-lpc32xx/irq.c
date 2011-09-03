/*
 * arch/arm/mach-lpc32xx/irq.c
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

#ifdef CONFIG_PM_DEBUG
#define DEBUG
#endif

#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kobject.h>

#include <mach/irqs.h>
#include <mach/hardware.h>
#include <mach/platform.h>
#include "common.h"

unsigned int wakeupirq = -1;

struct lpc32xx_event_group_regs {
	void __iomem *enab_reg;
	void __iomem *edge_reg;
	void __iomem *maskstat_reg;
	void __iomem *rawstat_reg;
};

static const struct lpc32xx_event_group_regs lpc32xx_event_int_regs = {
	.enab_reg = LPC32XX_CLKPWR_INT_ER,
	.edge_reg = LPC32XX_CLKPWR_INT_AP,
	.maskstat_reg = LPC32XX_CLKPWR_INT_SR,
	.rawstat_reg = LPC32XX_CLKPWR_INT_RS,
};

static const struct lpc32xx_event_group_regs lpc32xx_event_pin_regs = {
	.enab_reg = LPC32XX_CLKPWR_PIN_ER,
	.edge_reg = LPC32XX_CLKPWR_PIN_AP,
	.maskstat_reg = LPC32XX_CLKPWR_PIN_SR,
	.rawstat_reg = LPC32XX_CLKPWR_PIN_RS,
};

struct lpc32xx_event_info {
	const struct lpc32xx_event_group_regs *event_group;
	u32 mask;
};

static const char *lpc32xx_irqname[NR_IRQS] = {
	[IRQ_LPC32XX_GPI_08] = "GPI_08",
	[IRQ_LPC32XX_GPI_09] = "GPI_09",
	[IRQ_LPC32XX_GPI_19] = "GPI_19",
	[IRQ_LPC32XX_GPI_07] = "GPI_07",
	[IRQ_LPC32XX_GPI_00] = "GPI_00",
	[IRQ_LPC32XX_GPI_01] = "GPI_01",
	[IRQ_LPC32XX_GPI_02] = "GPI_02",
	[IRQ_LPC32XX_GPI_03] = "GPI_03",
	[IRQ_LPC32XX_GPI_04] = "GPI_04",
	[IRQ_LPC32XX_GPI_05] = "GPI_05",
	[IRQ_LPC32XX_GPI_06] = "GPI_06",
	[IRQ_LPC32XX_GPI_28] = "GPI_28",
	[IRQ_LPC32XX_UART_IIR1] = "Uart_1",
	[IRQ_LPC32XX_UART_IIR2] = "Uart_2",
	[IRQ_LPC32XX_UART_IIR3] = "Uart_3",
	[IRQ_LPC32XX_UART_IIR4] = "Uart_4",
	[IRQ_LPC32XX_UART_IIR5] = "Uart_5",
	[IRQ_LPC32XX_UART_IIR6] = "Uart_6",
	[IRQ_LPC32XX_UART_IIR7] = "Uart_7",
	[IRQ_LPC32XX_GPIO_00] = "GPIO_00",
	[IRQ_LPC32XX_GPIO_01] = "GPIO_01",
	[IRQ_LPC32XX_GPIO_02] = "GPIO_02",
	[IRQ_LPC32XX_GPIO_03] = "GPIO_03",
	[IRQ_LPC32XX_GPIO_04] = "GPIO_04",
	[IRQ_LPC32XX_GPIO_05] = "GPIO_05",
	[IRQ_LPC32XX_ETHERNET]    = "Ethernet",
	[IRQ_LPC32XX_KEY]         = "Keyboard",
	[IRQ_LPC32XX_USB_OTG_ATX] = "USB_OTG",
	[IRQ_LPC32XX_USB_HOST]    = "USB_Host",
	[IRQ_LPC32XX_RTC]     = "RTC",
	[IRQ_LPC32XX_MSTIMER] = "MS_Timer",
	[IRQ_LPC32XX_TS_AUX]  = "TS_AUX",
	[IRQ_LPC32XX_TS_P]    = "TS_P",
	[IRQ_LPC32XX_TS_IRQ]  = "TS_IRQ",
};

/*
 * Maps an IRQ number to and event mask and register
 * All IRQs are event based wakeup IRQs except the UARTs. The UART RX
 * wakeup is based on the pin state, not the UART IRQ state.
 */
static const struct lpc32xx_event_info lpc32xx_events[NR_IRQS] = {
	[IRQ_LPC32XX_GPI_08] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_GPI_08_BIT,
	},
	[IRQ_LPC32XX_GPI_09] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_GPI_09_BIT,
	},
	[IRQ_LPC32XX_GPI_19] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_GPI_19_BIT,
	},
	[IRQ_LPC32XX_GPI_07] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_GPI_07_BIT,
	},
	[IRQ_LPC32XX_GPI_00] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_GPI_00_BIT,
	},
	[IRQ_LPC32XX_GPI_01] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_GPI_01_BIT,
	},
	[IRQ_LPC32XX_GPI_02] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_GPI_02_BIT,
	},
	[IRQ_LPC32XX_GPI_03] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_GPI_03_BIT,
	},
	[IRQ_LPC32XX_GPI_04] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_GPI_04_BIT,
	},
	[IRQ_LPC32XX_GPI_05] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_GPI_05_BIT,
	},
	[IRQ_LPC32XX_GPI_06] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_GPI_06_BIT,
	},
	[IRQ_LPC32XX_GPI_28] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_GPI_28_BIT,
	},
  /* UART */
	[IRQ_LPC32XX_UART_IIR1] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_U1_RX_BIT,
	},
	[IRQ_LPC32XX_UART_IIR2] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_U2_RX_BIT,
	},
	[IRQ_LPC32XX_UART_IIR3] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_U3_RX_BIT,
	},
	[IRQ_LPC32XX_UART_IIR4] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_U4_RX_BIT,
	},
	[IRQ_LPC32XX_UART_IIR5] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_U5_RX_BIT,
	},
	[IRQ_LPC32XX_UART_IIR6] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_U6_IRRX_BIT,
	},
	[IRQ_LPC32XX_UART_IIR7] = {
		.event_group = &lpc32xx_event_pin_regs,
		.mask = LPC32XX_CLKPWR_EXTSRC_U7_RX_BIT,
	},
	[IRQ_LPC32XX_GPIO_00] = {
		.event_group = &lpc32xx_event_int_regs,
		.mask = LPC32XX_CLKPWR_INTSRC_GPIO_00_BIT,
	},
	[IRQ_LPC32XX_GPIO_01] = {
		.event_group = &lpc32xx_event_int_regs,
		.mask = LPC32XX_CLKPWR_INTSRC_GPIO_01_BIT,
	},
	[IRQ_LPC32XX_GPIO_02] = {
		.event_group = &lpc32xx_event_int_regs,
		.mask = LPC32XX_CLKPWR_INTSRC_GPIO_02_BIT,
	},
	[IRQ_LPC32XX_GPIO_03] = {
		.event_group = &lpc32xx_event_int_regs,
		.mask = LPC32XX_CLKPWR_INTSRC_GPIO_03_BIT,
	},
	[IRQ_LPC32XX_GPIO_04] = {
		.event_group = &lpc32xx_event_int_regs,
		.mask = LPC32XX_CLKPWR_INTSRC_GPIO_04_BIT,
	},
	[IRQ_LPC32XX_GPIO_05] = {
		.event_group = &lpc32xx_event_int_regs,
		.mask = LPC32XX_CLKPWR_INTSRC_GPIO_05_BIT,
	},
	[IRQ_LPC32XX_KEY] = {
		.event_group = &lpc32xx_event_int_regs,
		.mask = LPC32XX_CLKPWR_INTSRC_KEY_BIT,
	},
	[IRQ_LPC32XX_ETHERNET] = {
		.event_group = &lpc32xx_event_int_regs,
		.mask = LPC32XX_CLKPWR_INTSRC_MAC_BIT,
	},
	[IRQ_LPC32XX_USB_OTG_ATX] = {
		.event_group = &lpc32xx_event_int_regs,
		.mask = LPC32XX_CLKPWR_INTSRC_USBATXINT_BIT,
	},
	[IRQ_LPC32XX_USB_HOST] = {
		.event_group = &lpc32xx_event_int_regs,
		.mask = LPC32XX_CLKPWR_INTSRC_USB_BIT,
	},
	[IRQ_LPC32XX_RTC] = {
		.event_group = &lpc32xx_event_int_regs,
		.mask = LPC32XX_CLKPWR_INTSRC_RTC_BIT,
	},
	[IRQ_LPC32XX_MSTIMER] = {
		.event_group = &lpc32xx_event_int_regs,
		.mask = LPC32XX_CLKPWR_INTSRC_MSTIMER_BIT,
	},
	[IRQ_LPC32XX_TS_AUX] = {
		.event_group = &lpc32xx_event_int_regs,
		.mask = LPC32XX_CLKPWR_INTSRC_TS_AUX_BIT,
	},
	[IRQ_LPC32XX_TS_P] = {
		.event_group = &lpc32xx_event_int_regs,
		.mask = LPC32XX_CLKPWR_INTSRC_TS_P_BIT,
	},
	[IRQ_LPC32XX_TS_IRQ] = {
		.event_group = &lpc32xx_event_int_regs,
		.mask = LPC32XX_CLKPWR_INTSRC_ADC_BIT,
	},
};

static void get_controller(unsigned int irq, unsigned int *base,
	unsigned int *irqbit)
{
	if (irq < 32) {
		*base = LPC32XX_MIC_BASE;
		*irqbit = 1 << irq;
	} else if (irq < 64) {
		*base = LPC32XX_SIC1_BASE;
		*irqbit = 1 << (irq - 32);
	} else {
		*base = LPC32XX_SIC2_BASE;
		*irqbit = 1 << (irq - 64);
	}
}

static void lpc32xx_mask_irq(unsigned int irq)
{
	unsigned int reg, ctrl, mask;

	get_controller(irq, &ctrl, &mask);

	reg = __raw_readl(LPC32XX_INTC_MASK(ctrl)) & ~mask;
	__raw_writel(reg, LPC32XX_INTC_MASK(ctrl));
}

static void lpc32xx_unmask_irq(unsigned int irq)
{
	unsigned int reg, ctrl, mask;

	get_controller(irq, &ctrl, &mask);

	reg = __raw_readl(LPC32XX_INTC_MASK(ctrl)) | mask;
	__raw_writel(reg, LPC32XX_INTC_MASK(ctrl));
}

static void lpc32xx_ack_irq(unsigned int irq)
{
	unsigned int ctrl, mask;

	get_controller(irq, &ctrl, &mask);

	__raw_writel(mask, LPC32XX_INTC_RAW_STAT(ctrl));

	/* Also need to clear pending wake event */
	if (lpc32xx_events[irq].mask != 0)
		__raw_writel(lpc32xx_events[irq].mask,
			lpc32xx_events[irq].event_group->rawstat_reg);
}

static void __lpc32xx_set_irq_type(unsigned int irq, int use_high_level,
	int use_edge)
{
	unsigned int reg, ctrl, mask;

	get_controller(irq, &ctrl, &mask);

	/* Activation level, high or low */
	reg = __raw_readl(LPC32XX_INTC_POLAR(ctrl));
	if (use_high_level)
		reg |= mask;
	else
		reg &= ~mask;
	__raw_writel(reg, LPC32XX_INTC_POLAR(ctrl));

	/* Activation type, edge or level */
	reg = __raw_readl(LPC32XX_INTC_ACT_TYPE(ctrl));
	if (use_edge)
		reg |= mask;
	else
		reg &= ~mask;
	__raw_writel(reg, LPC32XX_INTC_ACT_TYPE(ctrl));

	/* Use same polarity for the wake events */
	if (lpc32xx_events[irq].mask != 0) {
		reg = __raw_readl(lpc32xx_events[irq].event_group->edge_reg);

		if (use_high_level)
			reg |= lpc32xx_events[irq].mask;
		else
			reg &= ~lpc32xx_events[irq].mask;

		__raw_writel(reg, lpc32xx_events[irq].event_group->edge_reg);
	}
}

static int lpc32xx_set_irq_type(unsigned int irq, unsigned int type)
{
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		/* Rising edge sensitive */
		__lpc32xx_set_irq_type(irq, 1, 1);
		break;

	case IRQ_TYPE_EDGE_FALLING:
		/* Falling edge sensitive */
		__lpc32xx_set_irq_type(irq, 0, 1);
		break;

	case IRQ_TYPE_LEVEL_LOW:
		/* Low level sensitive */
		__lpc32xx_set_irq_type(irq, 0, 0);
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		/* High level sensitive */
		__lpc32xx_set_irq_type(irq, 1, 0);
		break;

	/* Other modes are not supported */
	default:
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_PM
static u32 backup_irqs[NR_IRQ_CTRLS];
static u32 wakeup_irqs[NR_IRQ_CTRLS];

static ssize_t wakeup_event_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	if(wakeupirq < 0) {
		return 0;
	}

	return sprintf(buf, "%s\n", lpc32xx_irqname[wakeupirq]);
}
static struct kobj_attribute wakeup_event_attr =
			__ATTR(wakeup_event, 0644, wakeup_event_show, NULL);
#endif

void lpc32xx_irq_suspend(suspend_state_t state)
{
#ifdef CONFIG_PM
	unsigned int i, ctrl, mask;

	for( i=0; i < NR_IRQ_CTRLS; i++) {
		get_controller((i*32), &ctrl, &mask);

		/* Backup programmed IRQs */
		backup_irqs[i] = __raw_readl(LPC32XX_INTC_MASK(ctrl));

		/* Disable all interrupts */
		__raw_writel(0, LPC32XX_INTC_MASK(ctrl));

		if(state == PM_SUSPEND_STANDBY) {
			pr_debug("Wakeup_irq[%d] = 0x%08X\n", i, wakeup_irqs[i] );

			/* Program interrupts only in standby because of DDR wakeup issues*/
			__raw_writel( wakeup_irqs[i], LPC32XX_INTC_RAW_STAT(ctrl));
			__raw_writel( wakeup_irqs[i], LPC32XX_INTC_MASK(ctrl));
		}
	}

	if(state == PM_SUSPEND_MEM)	{
		pr_debug("Start_enable_pin = 0x%X\n",
				__raw_readl(lpc32xx_event_pin_regs.enab_reg));
		pr_debug("Start_enable_internal = 0x%X\n",
				__raw_readl(lpc32xx_event_int_regs.enab_reg));
	}

	/* Clear Raw Status registers */
	__raw_writel(LPC32XX_CLKPWR_EXTSRC_MASK,
			lpc32xx_event_pin_regs.rawstat_reg);
	__raw_writel(LPC32XX_CLKPWR_INTSRC_MASK,
			lpc32xx_event_int_regs.rawstat_reg);

	sysfs_remove_file(power_kobj, &wakeup_event_attr.attr);
#endif
}

void lpc32xx_irq_resume(suspend_state_t state)
{
#ifdef CONFIG_PM
	unsigned int i, ctrl, mask;
	unsigned int pinRegMask, intRegMask;
	unsigned int statusReg = 0;

	pinRegMask = __raw_readl(lpc32xx_event_pin_regs.maskstat_reg);
	intRegMask = __raw_readl(lpc32xx_event_int_regs.maskstat_reg);
	pr_debug("Pin Register wakeup mask = 0x%X\n", pinRegMask);
	pr_debug("Int Register wakeup mask = 0x%X\n", intRegMask);

	/* Scan all lpc32xx_events to find who woke up */
	for(i=0; i< NR_IRQS; i++) {
		/* Check IRQ can be defined as wakeup IRQ */
		if (lpc32xx_events[i].mask != 0) {
			pr_debug("Scan IRQ %d\n",i);
			if((lpc32xx_events[i].event_group->enab_reg == lpc32xx_event_pin_regs.enab_reg)
					&& pinRegMask) {
				pr_debug("Test mask 0x%X\n",lpc32xx_events[i].mask);
				if(lpc32xx_events[i].mask == pinRegMask) {
					wakeupirq = i;
					break;
				}
			}

			if((lpc32xx_events[i].event_group->enab_reg == lpc32xx_event_int_regs.enab_reg)
					&& intRegMask) {
				pr_debug("Test mask 0x%X\n",lpc32xx_events[i].mask);
				if(lpc32xx_events[i].mask == intRegMask) {
					wakeupirq = i;
					break;
				}
			}
		}
	}

	/* Clear Raw Status registers */
	__raw_writel(LPC32XX_CLKPWR_EXTSRC_MASK,
			lpc32xx_event_pin_regs.rawstat_reg);
	__raw_writel(LPC32XX_CLKPWR_INTSRC_MASK,
			lpc32xx_event_int_regs.rawstat_reg);

	for( i=0; i < NR_IRQ_CTRLS; i++) {
		get_controller( (i*32), &ctrl, &mask);

		statusReg = __raw_readl(LPC32XX_INTC_STAT(ctrl));

		pr_debug("statusReg %d = 0x%08X\n",i,statusReg);

		/* Disable all interrupts */
		__raw_writel( 0, LPC32XX_INTC_MASK(ctrl));

		/* Clear Wakeup pending interrupts */
		__raw_writel(wakeup_irqs[i], LPC32XX_INTC_RAW_STAT(ctrl));

		/* Restore old controller config */
		__raw_writel(backup_irqs[i], LPC32XX_INTC_MASK(ctrl));
	}

	if(wakeupirq > 0) {
		pr_debug("Wakeup source: %s\n", lpc32xx_irqname[wakeupirq]);
		sysfs_create_file(power_kobj, &wakeup_event_attr.attr);
	}
#endif
}

static int lpc32xx_irq_wake(unsigned int irqno, unsigned int state)
{
	unsigned long eventreg;
	unsigned int ctrl_nr, ctrl, mask;

	if (unlikely(irqno >= (32 * NR_IRQ_CTRLS))) {
		return -EINVAL;
	}

	get_controller(irqno, &ctrl, &mask);
	ctrl_nr = irqno / 32;

#ifdef CONFIG_PM_DEBUG
	if (state) {
		pr_debug("Set irq %d as wakeup IRQ\n", irqno);
		wakeup_irqs[ctrl_nr] |= mask;
		if( ctrl_nr == 1 ) {
			wakeup_irqs[0] |= ( 1<<IRQ_LPC32XX_SUB1IRQ );
		}
		if( ctrl_nr == 2 ) {
			wakeup_irqs[0] |= ( 1<<IRQ_LPC32XX_SUB2IRQ );
		}
	}
	else
	{
		pr_debug("Disable wakeup from irq %d \n", irqno);
		wakeup_irqs[ctrl_nr] &= ~mask;
		if( (ctrl_nr == 1) && (wakeup_irqs[1] == 0) )
			wakeup_irqs[0] &= ~( 1<<IRQ_LPC32XX_SUB1IRQ );
		if( (ctrl_nr == 2) && (wakeup_irqs[2] == 0) )
				wakeup_irqs[0] &= ~( 1<<IRQ_LPC32XX_SUB2IRQ );
	}
#endif

	if ( lpc32xx_events[irqno].mask == 0 ) {
		pr_warning("Can't configure irq %d as wakeup source\n",irqno);
		return 0;
	}

	if (lpc32xx_events[irqno].mask != 0) {
		eventreg = __raw_readl(lpc32xx_events[irqno].
			event_group->enab_reg);

		if (state)
			eventreg |= lpc32xx_events[irqno].mask;
		else {
			eventreg &= ~lpc32xx_events[irqno].mask;

			/*
			 * When disabling the wakeup, clear the latched
			 * event
			 */
			__raw_writel(lpc32xx_events[irqno].mask,
				lpc32xx_events[irqno].
				event_group->rawstat_reg);
		}

		__raw_writel(eventreg,
			lpc32xx_events[irqno].event_group->enab_reg);

		return 0;
	}

	__raw_writel(lpc32xx_events[irqno].mask,
		lpc32xx_events[irqno].event_group->rawstat_reg);

	return -ENODEV;
}

static void __init lpc32xx_set_default_mappings(unsigned int apr,
	unsigned int atr, unsigned int offset)
{
	unsigned int i;

	/* Set activation levels for each interrupt */
	i = 0;
	while (i < 32) {
		__lpc32xx_set_irq_type(offset + i, ((apr >> i) & 0x1),
			((atr >> i) & 0x1));
		i++;
	}
}

static struct irq_chip lpc32xx_irq_chip = {
	.ack = lpc32xx_ack_irq,
	.mask = lpc32xx_mask_irq,
	.unmask = lpc32xx_unmask_irq,
	.set_type = lpc32xx_set_irq_type,
	.set_wake = lpc32xx_irq_wake
};

static void lpc32xx_sic1_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned long ints = __raw_readl(LPC32XX_INTC_STAT(LPC32XX_SIC1_BASE));

	while (ints != 0) {
		int irqno = fls(ints) - 1;

		ints &= ~(1 << irqno);

		generic_handle_irq(LPC32XX_SIC1_IRQ(irqno));
	}
}

static void lpc32xx_sic2_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned long ints = __raw_readl(LPC32XX_INTC_STAT(LPC32XX_SIC2_BASE));

	while (ints != 0) {
		int irqno = fls(ints) - 1;

		ints &= ~(1 << irqno);

		generic_handle_irq(LPC32XX_SIC2_IRQ(irqno));
	}
}

void __init lpc32xx_init_irq(void)
{
	unsigned int i;

	/* Setup MIC */
	__raw_writel(0, LPC32XX_INTC_MASK(LPC32XX_MIC_BASE));
	__raw_writel(MIC_APR_DEFAULT, LPC32XX_INTC_POLAR(LPC32XX_MIC_BASE));
	__raw_writel(MIC_ATR_DEFAULT, LPC32XX_INTC_ACT_TYPE(LPC32XX_MIC_BASE));

	/* Setup SIC1 */
	__raw_writel(0, LPC32XX_INTC_MASK(LPC32XX_SIC1_BASE));
	__raw_writel(SIC1_APR_DEFAULT, LPC32XX_INTC_POLAR(LPC32XX_SIC1_BASE));
	__raw_writel(SIC1_ATR_DEFAULT, LPC32XX_INTC_ACT_TYPE(LPC32XX_SIC1_BASE));

	/* Setup SIC2 */
	__raw_writel(0, LPC32XX_INTC_MASK(LPC32XX_SIC2_BASE));
	__raw_writel(SIC2_APR_DEFAULT, LPC32XX_INTC_POLAR(LPC32XX_SIC2_BASE));
	__raw_writel(SIC2_ATR_DEFAULT, LPC32XX_INTC_ACT_TYPE(LPC32XX_SIC2_BASE));

	/* Configure supported IRQ's */
	for (i = 0; i < NR_IRQS; i++) {
		set_irq_chip(i, &lpc32xx_irq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	/* Set default mappings */
	lpc32xx_set_default_mappings(MIC_APR_DEFAULT, MIC_ATR_DEFAULT, 0);
	lpc32xx_set_default_mappings(SIC1_APR_DEFAULT, SIC1_ATR_DEFAULT, 32);
	lpc32xx_set_default_mappings(SIC2_APR_DEFAULT, SIC2_ATR_DEFAULT, 64);

	/* mask all interrupts except SUBIRQ */
	__raw_writel((1 << IRQ_LPC32XX_SUB1IRQ) | (1 << IRQ_LPC32XX_SUB2IRQ) |
			(1 << IRQ_LPC32XX_SUB1FIQ) | (1 << IRQ_LPC32XX_SUB2FIQ),
			LPC32XX_INTC_MASK(LPC32XX_MIC_BASE));
	__raw_writel(0, LPC32XX_INTC_MASK(LPC32XX_SIC1_BASE));
	__raw_writel(0, LPC32XX_INTC_MASK(LPC32XX_SIC2_BASE));

	/* MIC SUBIRQx interrupts will route handling to the chain handlers */
	set_irq_chained_handler(IRQ_LPC32XX_SUB1IRQ, lpc32xx_sic1_handler);
	set_irq_chained_handler(IRQ_LPC32XX_SUB2IRQ, lpc32xx_sic2_handler);

	/* Initially disable all wake events */
	__raw_writel(0, LPC32XX_CLKPWR_P01_ER);
	__raw_writel(0, LPC32XX_CLKPWR_INT_ER);
	__raw_writel(0, LPC32XX_CLKPWR_PIN_ER);

	/* Clear latched wake event states */
	__raw_writel(__raw_readl(LPC32XX_CLKPWR_PIN_RS),
		LPC32XX_CLKPWR_PIN_RS);
	__raw_writel(__raw_readl(LPC32XX_CLKPWR_INT_RS),
		LPC32XX_CLKPWR_INT_RS);
}
