/*
 * arch/arm/mach-lpc32xx/phy3250.c
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

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/amba/pl022.h>
#include <linux/amba/mmci.h>
#include <sound/uda1380.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/board.h>
#include "common.h"

/*
 * Mapped GPIOLIB GPIOs
 */
#define SPI0_CS_GPIO		LPC32XX_GPIO(LPC32XX_GPIO_P3_GRP, 5)
#define LCD_POWER_GPIO		LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 0)
#define BKL_POWER_GPIO		LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 4)
#define LED_GPIO		LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 1)
#define NAND_WP_GPIO		LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 19)
#define	MMC_PWR_ENABLE_GPIO	LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 5)
#define	MMC_CD_GPIO		LPC32XX_GPIO(LPC32XX_GPIO_P3_GRP, 1)
#define	MMC_WP_GPIO		LPC32XX_GPIO(LPC32XX_GPIO_P3_GRP, 0)

/*
 * AMBA LCD controller
 */
static struct clcd_panel conn_lcd_panel = {
	.mode		= {
		.name		= "QVGA portrait",
		.refresh	= 60,
		.xres		= 240,
		.yres		= 320,
		.pixclock	= 191828,
		.left_margin	= 22,
		.right_margin	= 11,
		.upper_margin	= 2,
		.lower_margin	= 1,
		.hsync_len	= 5,
		.vsync_len	= 2,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= (TIM2_IVS | TIM2_IHS),
	.cntl		= (CNTL_BGR | CNTL_LCDTFT | CNTL_LCDVCOMP(1) |
				CNTL_LCDBPP16_565),
	.bpp		= 16,
};
#define PANEL_SIZE (3 * SZ_64K)

static int lpc32xx_clcd_setup(struct clcd_fb *fb)
{
	dma_addr_t dma;

	fb->fb.screen_base = dma_alloc_writecombine(&fb->dev->dev,
		PANEL_SIZE, &dma, GFP_KERNEL);
	if (!fb->fb.screen_base) {
		printk(KERN_ERR "CLCD: unable to map framebuffer\n");
		return -ENOMEM;
	}

	fb->fb.fix.smem_start = dma;
	fb->fb.fix.smem_len = PANEL_SIZE;
	fb->panel = &conn_lcd_panel;

	if (gpio_request(LCD_POWER_GPIO, "LCD power"))
		printk(KERN_ERR "Error requesting gpio %u",
			LCD_POWER_GPIO);
	else if (gpio_direction_output(LCD_POWER_GPIO, 1))
		printk(KERN_ERR "Error setting gpio %u to output",
			LCD_POWER_GPIO);

	if (gpio_request(BKL_POWER_GPIO, "LCD backlight power"))
		printk(KERN_ERR "Error requesting gpio %u",
			BKL_POWER_GPIO);
	else if (gpio_direction_output(BKL_POWER_GPIO, 1))
		printk(KERN_ERR "Error setting gpio %u to output",
			BKL_POWER_GPIO);

	return 0;
}

static int lpc32xx_clcd_mmap(struct clcd_fb *fb, struct vm_area_struct *vma)
{
	return dma_mmap_writecombine(&fb->dev->dev, vma,
		fb->fb.screen_base, fb->fb.fix.smem_start,
		fb->fb.fix.smem_len);
}

static void lpc32xx_clcd_remove(struct clcd_fb *fb)
{
	dma_free_writecombine(&fb->dev->dev, fb->fb.fix.smem_len,
		fb->fb.screen_base, fb->fb.fix.smem_start);
}

/*
 * On some early LCD modules (1307.0), the backlight logic is inverted.
 * For those board variants, swap the disable and enable states for
 * BKL_POWER_GPIO.
*/
static void clcd_disable(struct clcd_fb *fb)
{
	gpio_set_value(BKL_POWER_GPIO, 0);
	gpio_set_value(LCD_POWER_GPIO, 0);
}

static void clcd_enable(struct clcd_fb *fb)
{
	gpio_set_value(BKL_POWER_GPIO, 1);
	gpio_set_value(LCD_POWER_GPIO, 1);
}

static struct clcd_board lpc32xx_clcd_data = {
	.name		= "Phytec LCD",
	.check		= clcdfb_check,
	.decode		= clcdfb_decode,
	.disable	= clcd_disable,
	.enable		= clcd_enable,
	.setup		= lpc32xx_clcd_setup,
	.mmap		= lpc32xx_clcd_mmap,
	.remove		= lpc32xx_clcd_remove,
};

static struct amba_device lpc32xx_clcd_device = {
	.dev				= {
		.coherent_dma_mask	= ~0,
		.init_name		= "dev:clcd",
		.platform_data		= &lpc32xx_clcd_data,
	},
	.res				= {
		.start			= LPC32XX_LCD_BASE,
		.end			= (LPC32XX_LCD_BASE + SZ_4K - 1),
		.flags			= IORESOURCE_MEM,
	},
	.dma_mask			= ~0,
	.irq				= {IRQ_LPC32XX_LCD, NO_IRQ},
};

/*
 * AMBA SSP (SPI)
 */
static void phy3250_spi_cs_set(u32 control)
{
	gpio_set_value(SPI0_CS_GPIO, (int) control);
}

static struct pl022_config_chip spi0_chip_info = {
	.lbm			= LOOPBACK_DISABLED,
	.com_mode		= INTERRUPT_TRANSFER,
	.iface			= SSP_INTERFACE_MOTOROLA_SPI,
	.hierarchy		= SSP_MASTER,
	.slave_tx_disable	= 0,
	.endian_tx		= SSP_TX_LSB,
	.endian_rx		= SSP_RX_LSB,
	.data_size		= SSP_DATA_BITS_8,
	.rx_lev_trig		= SSP_RX_4_OR_MORE_ELEM,
	.tx_lev_trig		= SSP_TX_4_OR_MORE_EMPTY_LOC,
	.clk_phase		= SSP_CLK_FIRST_EDGE,
	.clk_pol		= SSP_CLK_POL_IDLE_LOW,
	.ctrl_len		= SSP_BITS_8,
	.wait_state		= SSP_MWIRE_WAIT_ZERO,
	.duplex			= SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
	.cs_control		= phy3250_spi_cs_set,
};

static struct pl022_ssp_controller lpc32xx_ssp0_data = {
	.bus_id			= 0,
	.num_chipselect		= 1,
	.enable_dma		= 0,
};

static struct amba_device lpc32xx_ssp0_device = {
	.dev				= {
		.coherent_dma_mask	= ~0,
		.init_name		= "dev:ssp0",
		.platform_data		= &lpc32xx_ssp0_data,
	},
	.res				= {
		.start			= LPC32XX_SSP0_BASE,
		.end			= (LPC32XX_SSP0_BASE + SZ_4K - 1),
		.flags			= IORESOURCE_MEM,
	},
	.dma_mask			= ~0,
	.irq				= {IRQ_LPC32XX_SSP0, NO_IRQ},
};

/* AT25 driver registration */
static int __init phy3250_spi_board_register(void)
{
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
	static struct spi_board_info info[] = {
		{
			.modalias = "spidev",
			.max_speed_hz = 5000000,
			.bus_num = 0,
			.chip_select = 0,
			.controller_data = &spi0_chip_info,
		},
	};

#else
	static struct spi_eeprom eeprom = {
		.name = "at25256a",
		.byte_len = 0x8000,
		.page_size = 64,
		.flags = EE_ADDR2,
	};

	static struct spi_board_info info[] = {
		{
			.modalias = "at25",
			.max_speed_hz = 5000000,
			.bus_num = 0,
			.chip_select = 0,
			.platform_data = &eeprom,
			.controller_data = &spi0_chip_info,
		},
	};
#endif
	return spi_register_board_info(info, ARRAY_SIZE(info));
}
arch_initcall(phy3250_spi_board_register);

/*
 * Platform Data for UDA1380 Audiocodec.
 * As there are no GPIOs for codec power & reset pins,
 * dummy GPIO numbers are used.
 */
static struct uda1380_platform_data uda1380_info = {
        .gpio_power = LPC32XX_GPIO(LPC32XX_GPO_P3_GRP,10),
        .gpio_reset = LPC32XX_GPIO(LPC32XX_GPO_P3_GRP,2),
        .dac_clk    = UDA1380_DAC_CLK_WSPLL,
};

static struct i2c_board_info __initdata phy3250_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("pcf8563", 0x51),
	},
        {
                I2C_BOARD_INFO("uda1380", 0x18),
                .platform_data = &uda1380_info,
        },
};

static struct gpio_led phy_leds[] = {
	{
		.name			= "led0",
		.gpio			= LED_GPIO,
		.active_low		= 1,
		.default_trigger	= "heartbeat",
	},
};

static struct gpio_led_platform_data led_data = {
	.leds = phy_leds,
	.num_leds = ARRAY_SIZE(phy_leds),
};

static struct platform_device lpc32xx_gpio_led_device = {
	.name			= "leds-gpio",
	.id			= -1,
	.dev.platform_data	= &led_data,
};

/*
 * Board specific key scanner driver data
 */
#define PHY3250_KMATRIX_SIZE 1
static int lpc32xx_keymaps[] = {
	KEY_1,  /* 1, 1 */
};

static struct lpc32XX_kscan_cfg lpc32xx_kscancfg = {
	.matrix_sz      = PHY3250_KMATRIX_SIZE,
	.keymap         = lpc32xx_keymaps,
	/* About a 30Hz scan rate based on a 32KHz clock */
	.deb_clks       = 3,
	.scan_delay     = 34,
};

static struct resource kscan_resources[] = {
	[0] = {
		.start  = LPC32XX_KSCAN_BASE,
		.end    = LPC32XX_KSCAN_BASE + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_LPC32XX_KEY,
		.end    = IRQ_LPC32XX_KEY,
		.flags  = IORESOURCE_IRQ,
	},

};

static struct platform_device lpc32xx_kscan_device = {
	.name           = "lpc32xx_keys",
	.id             = 0,
	.dev            = {
		.platform_data  = &lpc32xx_kscancfg,
	},
	.num_resources  = ARRAY_SIZE(kscan_resources),
	.resource       = kscan_resources,
};

#if defined (CONFIG_MMC_ARMMMCI)
static u32 mmc_translate_vdd(struct device *dev, unsigned int vdd)
{
	/* Only on and off are supported */
	if (vdd != 0)
		gpio_set_value(MMC_PWR_ENABLE_GPIO,1);
	else
		gpio_set_value(MMC_PWR_ENABLE_GPIO,0);

	return 0;
}

/*
 * Board specific MMC driver data
 */
struct mmci_platform_data lpc32xx_plat_data = {
        .ocr_mask       = MMC_VDD_30_31|MMC_VDD_31_32|MMC_VDD_32_33|MMC_VDD_33_34,
	.translate_vdd	= mmc_translate_vdd,
	.capabilities   = MMC_CAP_4_BIT_DATA,
        .gpio_wp        = MMC_WP_GPIO,
        .gpio_cd        = MMC_CD_GPIO,
};

/*
 * SD card controller resources
 */
struct amba_device lpc32xx_mmc_device = {
        .dev                            = {
                .coherent_dma_mask      = ~0,
                .init_name                 = "dev:mmc0",
                .platform_data          = &lpc32xx_plat_data,
        },
        .res                            = {
                .start                  = LPC32XX_SD_BASE,
                .end                    = (LPC32XX_SD_BASE + SZ_4K - 1),
                .flags                  = IORESOURCE_MEM,
        },
        .dma_mask                       = ~0,
        .irq                            = {IRQ_LPC32XX_SD0, IRQ_LPC32XX_SD1},
};
#endif


#if defined(CONFIG_MTD_NAND_SLC_LPC32XX)
/*
 * Board specific NAND setup data
 */
static int nandwp_enable(int enable)
{
        if (enable != 0)
		gpio_set_value(NAND_WP_GPIO,0);
        else 
		gpio_set_value(NAND_WP_GPIO,1);

        return 1;
}
#define BLK_SIZE (512 * 32)
static struct mtd_partition __initdata phy3250_nand_partition[] = {
        {
                .name   = "phy3250-boot",
                .offset = 0,
                .size   = (BLK_SIZE * 25)
        },
        {
                .name   = "phy3250-uboot",
                .offset = MTDPART_OFS_APPEND,
                .size   = (BLK_SIZE * 100)
        },
        {
                .name   = "phy3250-ubt-prms",
                .offset = MTDPART_OFS_APPEND,
                .size   = (BLK_SIZE * 4)
        },
        {
                .name   = "phy3250-kernel",
                .offset = MTDPART_OFS_APPEND,
                .size   = (BLK_SIZE * 256)
        },
        {
                .name   = "phy3250-rootfs",
                .offset = MTDPART_OFS_APPEND,
                .size   = MTDPART_SIZ_FULL
        },
};
static struct mtd_partition * __init phy3250_nand_partitions(int size, int *num_partitions)
{
        *num_partitions = ARRAY_SIZE(phy3250_nand_partition);
        return phy3250_nand_partition;
}
struct lpc32XX_nand_cfg lpc32xx_nandcfg =
{
        .wdr_clks               = 3,
        .wwidth                 = 28571428,
        .whold                  = 100000000,
        .wsetup                 = 66666666,
        .rdr_clks               = 3,
        .rwidth                 = 28571428,
        .rhold                  = 100000000,
        .rsetup                 = 66666666,
        .use16bus               = 0,
        .enable_write_prot      = nandwp_enable,
        .partition_info         = phy3250_nand_partitions,
};

/*
 * SLC NAND resources
 */
static struct resource slc_nand_resources[] = {
        [0] = {
                .start  = LPC32XX_SLC_BASE,
                .end    = LPC32XX_SLC_BASE + SZ_4K - 1,
                .flags  = IORESOURCE_MEM,
        },

        [1] = {
                .start  = IRQ_LPC32XX_FLASH,
                .end    = IRQ_LPC32XX_FLASH,
                .flags  = IORESOURCE_IRQ,
        },

};
static u64 lpc32xx_slc_dma_mask = 0xffffffffUL;
static struct platform_device lpc32xx_slc_nand_device = {
        .name           = "lpc32xx-nand",
        .id             = 0,
        .dev            = {
                                .platform_data  = &lpc32xx_nandcfg,
                                .dma_mask    = &lpc32xx_slc_dma_mask,
                                .coherent_dma_mask = ~0UL,
        },
        .num_resources  = ARRAY_SIZE(slc_nand_resources),
        .resource       = slc_nand_resources,
};
#endif

/*
 * Network Support
 */
static struct lpc_net_cfg lpc32xx_netdata =
{
	.phy_irq        = -1,
	.phy_mask       = 0xFFFFFFF0,
};

static struct resource net_resources[] = {
	[0] = {
		.start  = LPC32XX_ETHERNET_BASE,
		.end    = LPC32XX_ETHERNET_BASE + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},

	[1] = {
		.start  = IRQ_LPC32XX_ETHERNET,
		.end    = IRQ_LPC32XX_ETHERNET,
		.flags  = IORESOURCE_IRQ,
	},

};

static u64 lpc32xx_mac_dma_mask = 0xffffffffUL;
static struct platform_device lpc32xx_net_device = {
	.name           = "lpc-net",
	.id             = 0,
	.dev            = {
		.dma_mask = &lpc32xx_mac_dma_mask,
		.coherent_dma_mask = 0xffffffffUL,
		.platform_data  = &lpc32xx_netdata,
	},
	.num_resources  = ARRAY_SIZE(net_resources),
	.resource       = net_resources,
};

static struct platform_device *phy3250_devs[] __initdata = {
	&lpc32xx_i2c0_device,
	&lpc32xx_i2c1_device,
	&lpc32xx_i2c2_device,
	&lpc32xx_watchdog_device,
	&lpc32xx_gpio_led_device,
	&lpc32xx_rtc_device,
	&lpc32xx_tsc_device,
	&lpc32xx_kscan_device,
	&lpc32xx_net_device,
#if defined(CONFIG_MTD_NAND_SLC_LPC32XX)
	&lpc32xx_slc_nand_device,
#endif
#if defined(CONFIG_USB_OHCI_HCD)
	&lpc32xx_ohci_device,
#endif
#if defined(CONFIG_USB_GADGET_LPC32XX)
	&lpc32xx_usbd_device,
#endif
};

static struct amba_device *amba_devs[] __initdata = {
	&lpc32xx_clcd_device,
	&lpc32xx_ssp0_device,
#if defined(CONFIG_MMC_ARMMMCI)
	&lpc32xx_mmc_device,
#endif
};

/*
 * Board specific functions
 */
static void __init phy3250_board_init(void)
{
	u32 tmp;
	int i;

	lpc32xx_gpio_init();

	/* Register GPIOs used on this board */
	if (gpio_request(SPI0_CS_GPIO, "spi0 cs"))
		printk(KERN_ERR "Error requesting gpio %u",
				SPI0_CS_GPIO);
	else if (gpio_direction_output(SPI0_CS_GPIO, 1))
		printk(KERN_ERR "Error setting gpio %u to output",
				SPI0_CS_GPIO);

	if (gpio_request(MMC_PWR_ENABLE_GPIO, "mmc_power_en"))
		printk(KERN_ERR "Error requesting gpio %u",
				MMC_PWR_ENABLE_GPIO);
	else if (gpio_direction_output(MMC_PWR_ENABLE_GPIO, 1))
		printk(KERN_ERR "Error setting gpio %u to output",
				MMC_PWR_ENABLE_GPIO);

	/* Setup network interface for RMII mode */
	tmp = __raw_readl(LPC32XX_CLKPWR_MACCLK_CTRL);
	tmp &= ~LPC32XX_CLKPWR_MACCTRL_PINS_MSK;
	tmp |= LPC32XX_CLKPWR_MACCTRL_USE_RMII_PINS;
	__raw_writel(tmp, LPC32XX_CLKPWR_MACCLK_CTRL);

	/* Setup SLC NAND controller muxing */
	__raw_writel(LPC32XX_CLKPWR_NANDCLK_SEL_SLC,
		LPC32XX_CLKPWR_NAND_CLK_CTRL);

	/* Setup LCD muxing to RGB565 */
	tmp = __raw_readl(LPC32XX_CLKPWR_LCDCLK_CTRL) &
		~(LPC32XX_CLKPWR_LCDCTRL_LCDTYPE_MSK |
		LPC32XX_CLKPWR_LCDCTRL_PSCALE_MSK);
	tmp |= LPC32XX_CLKPWR_LCDCTRL_LCDTYPE_TFT16;
	__raw_writel(tmp, LPC32XX_CLKPWR_LCDCLK_CTRL);

	/* Set up I2C pull levels */
	tmp = __raw_readl(LPC32XX_CLKPWR_I2C_CLK_CTRL);
	tmp |= LPC32XX_CLKPWR_I2CCLK_USBI2CHI_DRIVE |
		LPC32XX_CLKPWR_I2CCLK_I2C2HI_DRIVE;
	__raw_writel(tmp, LPC32XX_CLKPWR_I2C_CLK_CTRL);

	/* Disable IrDA pulsing support on UART6 */
	tmp = __raw_readl(LPC32XX_UARTCTL_CTRL);
	tmp |= LPC32XX_UART_UART6_IRDAMOD_BYPASS;
	__raw_writel(tmp, LPC32XX_UARTCTL_CTRL);

	/* Enable DMA for I2S1 channel */
	tmp = __raw_readl(LPC32XX_CLKPWR_I2S_CLK_CTRL);
	tmp = LPC32XX_CLKPWR_I2SCTRL_I2S1_USE_DMA;
	__raw_writel(tmp, LPC32XX_CLKPWR_I2S_CLK_CTRL);

	lpc32xx_serial_init();

	/*
	 * AMBA peripheral clocks need to be enabled prior to AMBA device
	 * detection or a data fault will occur, so enable the clocks
	 * here. However, we don't want to enable them if the peripheral
	 * isn't included in the image
	 */
#if defined(CONFIG_MMC_ARMMMCI)
        tmp = __raw_readl(LPC32XX_CLKPWR_MS_CTRL);
        tmp |= LPC32XX_CLKPWR_MSCARD_SDCARD_EN | LPC32XX_CLKPWR_MSCARD_MSDIO_PU_EN;
        __raw_writel(tmp, LPC32XX_CLKPWR_MS_CTRL);
#endif

#ifdef CONFIG_FB_ARMCLCD
	tmp = __raw_readl(LPC32XX_CLKPWR_LCDCLK_CTRL);
	__raw_writel((tmp | LPC32XX_CLKPWR_LCDCTRL_CLK_EN),
		LPC32XX_CLKPWR_LCDCLK_CTRL);
#endif
#ifdef CONFIG_SPI_PL022
	tmp = __raw_readl(LPC32XX_CLKPWR_SSP_CLK_CTRL);
	__raw_writel((tmp | LPC32XX_CLKPWR_SSPCTRL_SSPCLK0_EN),
		LPC32XX_CLKPWR_SSP_CLK_CTRL);
#endif

	platform_add_devices(phy3250_devs, ARRAY_SIZE(phy3250_devs));
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++) {
		struct amba_device *d = amba_devs[i];
		amba_device_register(d, &iomem_resource);
	}

	/* Test clock needed for UDA1380 initial init */
	__raw_writel(LPC32XX_CLKPWR_TESTCLK2_SEL_MOSC |
		LPC32XX_CLKPWR_TESTCLK_TESTCLK2_EN,
		LPC32XX_CLKPWR_TEST_CLK_SEL);

	i2c_register_board_info(0, phy3250_i2c_board_info,
		ARRAY_SIZE(phy3250_i2c_board_info));
}

static int __init lpc32xx_display_uid(void)
{
	u32 uid[4];

	lpc32xx_get_uid(uid);

	printk(KERN_INFO "LPC32XX unique ID: %08x%08x%08x%08x\n",
		uid[3], uid[2], uid[1], uid[0]);

	return 1;
}
arch_initcall(lpc32xx_display_uid);

/*
 * Example code for setting up the BTN1 button (on GPI3) for system
 * wakeup and IRQ support. This will allow the GPI3 input to wake
 * up the system on a low edge. Edge based interrupts won't be
 * registered in the interrupt controller when the system is asleep,
 * although they will be registered in the event manager. For this,
 * reason, a level based interrupt state is recommended for GPIOs when
 * using IRQ and wakeup from GPI edge state.
 * 
 */
#define BTN1_GPIO		LPC32XX_GPIO(LPC32XX_GPI_P3_GRP, 3)
static irqreturn_t phy3250_btn1_irq(int irq, void *dev)
{
	printk(KERN_INFO "GPIO IRQ!\n");

	return IRQ_HANDLED;
}

static int __init phy3250_button_setup(void)
{
	int ret;

	if (gpio_request(BTN1_GPIO, "Button 1")) {
		printk(KERN_ERR "Error requesting gpio %u", BTN1_GPIO);
		return 0;
	}

	/*
	 * Wakeup/irq on low edge - the wakeup state will use the same
	 * state as the IRQ edge state.
	 */
	set_irq_type(IRQ_LPC32XX_GPI_03, IRQ_TYPE_EDGE_FALLING);
	ret = request_irq(IRQ_LPC32XX_GPI_03, phy3250_btn1_irq,
		IRQF_DISABLED, "gpio_btn1_irq", NULL);
	if (ret < 0) {
		printk(KERN_ERR "Can't request interrupt\n");
		return 0;
	}

	enable_irq_wake(IRQ_LPC32XX_GPI_03);

	return 1;
}
device_initcall(phy3250_button_setup);

MACHINE_START(PHY3250, "Phytec 3250 board with the LPC3250 Microcontroller")
	/* Maintainer: Kevin Wells, NXP Semiconductors */
	.phys_io	= LPC32XX_UART5_BASE,
	.io_pg_offst	= ((IO_ADDRESS(LPC32XX_UART5_BASE))>>18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= lpc32xx_map_io,
	.init_irq	= lpc32xx_init_irq,
	.timer		= &lpc32xx_timer,
	.init_machine	= phy3250_board_init,
MACHINE_END

/* For backwards compatibility with older bootloaders only */
MACHINE_START(LPC3XXX, "Phytec 3250 board with the LPC3250 Microcontroller")
	/* Maintainer: Kevin Wells, NXP Semiconductors */
	.phys_io	= LPC32XX_UART5_BASE,
	.io_pg_offst	= ((IO_ADDRESS(LPC32XX_UART5_BASE))>>18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= lpc32xx_map_io,
	.init_irq	= lpc32xx_init_irq,
	.timer		= &lpc32xx_timer,
	.init_machine	= phy3250_board_init,
MACHINE_END
