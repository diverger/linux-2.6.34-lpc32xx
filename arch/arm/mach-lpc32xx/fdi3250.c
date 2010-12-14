/*
 * arch/arm/mach-lpc32xx/fdi3250.c
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
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/amba/pl022.h>
#include <linux/amba/mmci.h>

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
#define	LED_GPIO		LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 3)
#define	SPI_CS_GPIO		LPC32XX_GPIO(LPC32XX_GPIO_P3_GRP, 5)
#define	NAND_WP_GPIO		LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 19)

/*
 * Tick LED
 */
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
 * AMBA SSP (SPI)
 */
static struct pl022_ssp_controller lpc32xx_ssp0_data = {
	.bus_id			= 0,
	.num_chipselect		= 2,
	.enable_dma		= 0,
};

static struct amba_device lpc32xx_ssp0_device = {
	.dev	= {
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

/* SPIDEV chip select function */
static void fdi3250_spi_cs_set(u32 control)
{
	gpio_set_value(SPI_CS_GPIO, (int) control);
}

/* SPIDEV parameters */
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
	.cs_control		= fdi3250_spi_cs_set,
};

/* SPI devices registration */
static int __init fdi3250_spi_devices_register(void)
{
	static struct spi_board_info info[] = {
		{
			.modalias = "spidev",
			.max_speed_hz = 2500000,
			.bus_num = 0,
			.chip_select = 0,
			.controller_data = &spi0_chip_info,
		},
	};

	return spi_register_board_info(info, ARRAY_SIZE(info));
}
arch_initcall(fdi3250_spi_devices_register);

#if defined (CONFIG_FB_ARMCLCD)
/*
 * Board specific LCD setup and functions
 */
#if defined (CONFIG_SOM9DIMM3250_LCD_PANEL)
/*
 * Support for QVGA portrait panel 
 */
#if defined (CONFIG_SOM9DIMM3250_LCD_TOSHIBA_QVGA_35)
static struct clcd_panel conn_lcd_panel = {
	.mode		= {
		.name		= "QVGA portrait",
		.refresh	= 30,
		.xres		= 320,
		.yres		= 240,
		.pixclock	= 158730,
		.left_margin	= 11,
		.right_margin	= 3,
		.upper_margin	= 7,
		.lower_margin	= 7,
		.hsync_len	= 69,
		.vsync_len	= 45,
		.sync		= FB_SYNC_HOR_HIGH_ACT|FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= 0, 
	.cntl		= (CNTL_BGR | CNTL_LCDTFT | CNTL_LCDVCOMP(1) |
				CNTL_LCDBPP16_565),
	.bpp		= 16,
};
#define PANEL_SIZE (3 * SZ_64K)
#endif
#if defined (CONFIG_SOM9DIMM3250_LCD_OKAYA_VGA_35)
static struct clcd_panel conn_lcd_panel = {
	.mode		= {
		.name		= "VGA portrait",
		.refresh	= 30,
		.xres		= 640,
		.yres		= 480,
		.pixclock	= 41666,
		.left_margin	= 10,
		.right_margin	= 120,
		.upper_margin	= 7,
		.lower_margin	= 35,
		.hsync_len	= 30,
		.vsync_len	= 3,
		.sync		= FB_SYNC_HOR_HIGH_ACT|FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= 0, 
	.cntl		= (CNTL_BGR | CNTL_LCDTFT | CNTL_LCDVCOMP(1) |
				CNTL_LCDBPP16_565),
	.bpp		= 16,
};
#define PANEL_SIZE (10 * SZ_64K)
#endif

#endif // CONFIG_SOM9DIMM3250_LCD_PANEL

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

        return 0;
}

static int lpc32xx_clcd_mmap(struct clcd_fb *fb, struct vm_area_struct *vma)
{
        return dma_mmap_writecombine(&fb->dev->dev, vma,
                                     fb->fb.screen_base,
                                     fb->fb.fix.smem_start,
                                     fb->fb.fix.smem_len);
}

static void lpc32xx_clcd_remove(struct clcd_fb *fb)
{
        dma_free_writecombine(&fb->dev->dev, fb->fb.fix.smem_len,
                              fb->fb.screen_base, fb->fb.fix.smem_start);
}

void clcd_disable(struct clcd_fb *fb)
{
#if defined (CONFIG_SOM9DIMM3250_LCD_TOSHIBA_QVGA_35)
	__raw_writel(0x40000000, io_p2v(LPC32XX_PWM1_BASE));
#elif defined (CONFIG_SOM9DIMM3250_LCD_OKAYA_VGA_35)
	__raw_writel(0x00000000, io_p2v(LPC32XX_PWM1_BASE));
#endif
}

void clcd_enable(struct clcd_fb *fb)
{
#if defined (CONFIG_SOM9DIMM3250_LCD_TOSHIBA_QVGA_35)
	__raw_writel(0x00000000, io_p2v(LPC32XX_PWM1_BASE));
#elif defined (CONFIG_SOM9DIMM3250_LCD_OKAYA_VGA_35)
	__raw_writel(0x40000000, io_p2v(LPC32XX_PWM1_BASE));
#endif
}

struct clcd_board lpc32xx_clcd_data = {
#if defined (CONFIG_SOM9DIMM3250_LCD_TOSHIBA_QVGA_35)
        .name           = "Toshiba 3.5 inch LCD",
#elif defined (CONFIG_SOM9DIMM3250_LCD_OKAYA_VGA_35)
        .name           = "Okaya 3.5 inch LCD",
#else
        .name           = "Unknown Display",
#endif
        .check          = clcdfb_check,
        .decode         = clcdfb_decode,
        .disable        = clcd_disable,
        .enable         = clcd_enable,
        .setup          = lpc32xx_clcd_setup,
        .mmap           = lpc32xx_clcd_mmap,
        .remove         = lpc32xx_clcd_remove,
};

struct amba_device lpc32xx_clcd_device = {
        .dev                            = {
                .coherent_dma_mask      = ~0,
                .init_name              = "dev:clcd",
                .platform_data          = &lpc32xx_clcd_data,
        },
        .res                            = {
                .start                  = LPC32XX_LCD_BASE,
                .end                    = (LPC32XX_LCD_BASE + SZ_4K - 1),
                .flags                  = IORESOURCE_MEM,
        },
        .dma_mask                       = ~0,
        .irq                            = {IRQ_LPC32XX_LCD, NO_IRQ},
};
#endif

/* AMBA based devices list */
static struct amba_device *amba_devs[] __initdata = {
	&lpc32xx_ssp0_device,
#if defined (CONFIG_FB_ARMCLCD)
	&lpc32xx_clcd_device,
#endif
};

/*
 * Register AMBA BUS Devices.
 * Call AMBA device restration after SPI driver probe(),
 * as LCD controller uses SPI driver for initialization
 */
static int __init fdi3250_amba_devices_register(void)
{
	u32 i = 0;

	/* Add AMBA devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++) {
		struct amba_device *d = amba_devs[i];
		amba_device_register(d, &iomem_resource);
	}
	
	return 0;
}
device_initcall_sync(fdi3250_amba_devices_register);

#if defined(CONFIG_MTD_NAND_SLC_LPC32XX)
/*
 *  * Board specific NAND setup data
 *   */
static int nandwp_enable(int enable)
{
        if (enable != 0)
		gpio_set_value(NAND_WP_GPIO,0);
        else
		gpio_set_value(NAND_WP_GPIO,1);

        return 1;
}
#define BLK_SIZE (2048 * 64)
static struct mtd_partition __initdata fdi3250_nand_partition[] = {
        {
                .name   = "fdi3250-boot",
                .offset = 0,
                .size   = (BLK_SIZE * 25)
        },
        {
                .name   = "fdi3250-uboot",
                .offset = MTDPART_OFS_APPEND,
                .size   = (BLK_SIZE * 102)
        },
        {
                .name   = "fdi3250-kernel",
                .offset = MTDPART_OFS_APPEND,
                .size   = (BLK_SIZE * 32)
        },
        {
                .name   = "fdi3250-jffs2",
                .offset = MTDPART_OFS_APPEND,
                .size   = MTDPART_SIZ_FULL
        },
};
static struct mtd_partition * __init fdi3250_nand_partitions(int size, int *num_partitions)
{
        *num_partitions = ARRAY_SIZE(fdi3250_nand_partition);
        return fdi3250_nand_partition;
}
struct lpc32XX_nand_cfg lpc32xx_nandcfg =
{
        .wdr_clks               = 14,
        .wwidth                 = 260000000,
        .whold                  = 104000000,
        .wsetup                 = 200000000,
        .rdr_clks               = 14,
        .rwidth                 = 34666666,
        .rhold                  = 104000000,
        .rsetup                 = 200000000,
        .use16bus               = 0,
        .enable_write_prot      = nandwp_enable,
        .partition_info         = fdi3250_nand_partitions,
};

/*
 *  * SLC NAND resources
 *   */
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

static struct platform_device* fdi3250_devs[] __initdata = {
	&lpc32xx_i2c0_device,
	&lpc32xx_i2c1_device,
	&lpc32xx_i2c2_device,
	&lpc32xx_watchdog_device,
	&lpc32xx_gpio_led_device,
	&lpc32xx_rtc_device,
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

/*
 * Board specific functions
 */
void __init fdi3250_board_init(void)
{
	u32 tmp;

	/* Intiliase GPIO */
	lpc32xx_gpio_init();

	/* Set SPI CS GPIO to output */
	gpio_request(SPI_CS_GPIO, "spi0 cs");
	gpio_direction_output(SPI_CS_GPIO, 1);

	gpio_request(NAND_WP_GPIO, "NAND WP GPIO");
	gpio_direction_input(NAND_WP_GPIO);

	/* Setup network interface for RMII mode */
	tmp = __raw_readl(LPC32XX_CLKPWR_MACCLK_CTRL);
	tmp &= ~LPC32XX_CLKPWR_MACCTRL_PINS_MSK;
	tmp |= LPC32XX_CLKPWR_MACCTRL_USE_RMII_PINS;
	__raw_writel(tmp, LPC32XX_CLKPWR_MACCLK_CTRL);

	/* Setup SLC NAND controller */
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

	/* Initalise Serial device */
	lpc32xx_serial_init();

	/*
	 * AMBA peripheral clocks need to be enabled prior to AMBA device
	 * detection or a data fault will occur, so enable the clocks
	 * here. However, we don't want to enable them if the peripheral
	 * isn't included in the image
	 */
	/* Initialise SSP clock */
	tmp = __raw_readl(LPC32XX_CLKPWR_SSP_CLK_CTRL);
	__raw_writel((tmp | LPC32XX_CLKPWR_SSPCTRL_SSPCLK0_EN),
			LPC32XX_CLKPWR_SSP_CLK_CTRL);

	/* Initialise LCD clock */
	tmp = __raw_readl(LPC32XX_CLKPWR_LCDCLK_CTRL);
	__raw_writel((tmp | LPC32XX_CLKPWR_LCDCTRL_CLK_EN),
			LPC32XX_CLKPWR_LCDCLK_CTRL);

	/* Enable SD card clock so AMBA driver will work correctly. The
	   AMBA driver needs the clock before the SD card controller
	   driver initializes it. The clock will turn off once the driver
	   has been initialized. */
	tmp = __raw_readl(LPC32XX_CLKPWR_MS_CTRL);
	tmp |= LPC32XX_CLKPWR_MSCARD_SDCARD_EN |
		LPC32XX_CLKPWR_MSCARD_MSDIO_PU_EN;
	__raw_writel(tmp, LPC32XX_CLKPWR_MS_CTRL);

	/* Disable UART5->USB transparent mode or USB won't work */
	tmp = __raw_readl(LPC32XX_UARTCTL_CTRL);
	tmp &= ~LPC32XX_UART_U5_ROUTE_TO_USB;
	__raw_writel(tmp, LPC32XX_UARTCTL_CTRL);

	/* Add platform devcies */
	platform_add_devices(fdi3250_devs, ARRAY_SIZE(fdi3250_devs));
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

MACHINE_START (FDI3250, "Future Designs board with the LPC3250 Microcontroller")
	.phys_io	= LPC32XX_UART5_BASE,
	.io_pg_offst	= ((IO_ADDRESS(LPC32XX_HS_UART1_BASE))>>18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= lpc32xx_map_io,
	.init_irq	= lpc32xx_init_irq,
	.timer		= &lpc32xx_timer,
	.init_machine	= fdi3250_board_init,
	MACHINE_END

/* For backwards compatibility with older bootloaders only */
MACHINE_START (LPC3XXX, "Future Designs board with the LPC3250 Microcontroller")
	.phys_io	= LPC32XX_UART5_BASE,
	.io_pg_offst	= ((IO_ADDRESS(LPC32XX_HS_UART1_BASE))>>18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= lpc32xx_map_io,
	.init_irq	= lpc32xx_init_irq,
	.timer		= &lpc32xx_timer,
	.init_machine	= fdi3250_board_init,
	MACHINE_END
