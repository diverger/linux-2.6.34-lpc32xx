/*
 * arch/arm/mach-lpc32xx/ea3250.c
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

#include <linux/spi/ads7846.h>

#define I2C_PCA9532_ADDR 0x60
#define I2C_24LC256_ADDR 0x50

/*
 * Mapped GPIOLIB GPIOs
 */
#define	LED_GPIO	LPC32XX_GPIO(LPC32XX_GPIO_P2_GRP, 1)
#define	SPI0_CS_GPIO	LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 11)
#define	ADS_TS_GPIO	LPC32XX_GPIO(LPC32XX_GPIO_P3_GRP, 0)
#define	NAND_WP_GPIO	LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 19)
#define	LCD_CS_GPIO	LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 4)
#define	LCD_RS_GPIO	LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 5)
#define	BKL_POW_GPIO	LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 14)
#define	SSEL0_GPIO5	LPC32XX_GPIO(LPC32XX_GPIO_P3_GRP, 5)

/*
 * LCD controller functions
 */
#define SET_RS		(gpio_set_value(LCD_RS_GPIO, 1))
#define RESET_RS	(gpio_set_value(LCD_RS_GPIO, 0))
#define PANEL_SIZE	(3 * SZ_64K)

/* SPI LCDC device structure */
struct spi_device *ea3250_spi_lcd_dev = NULL;

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

/*
 * Touchscreen device
 */
/* Touch screen chip select function */
static void ea3250_spi_cs_set(u32 control)
{
	gpio_set_value(SPI0_CS_GPIO, (int) control);
}

/* Touch screen SPI parameters */
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
	.cs_control		= ea3250_spi_cs_set,
};

/* Touch screen interrupt status function */
static int ea3250_ads7846_pendown_state(void)
{
	u32 tmp = gpio_get_value(ADS_TS_GPIO);
	return (tmp == 0);
}

/* Touch screen platform data */
static struct ads7846_platform_data ea_ads7846_platform_data __initdata = {
	.debounce_max	= 10,
	.debounce_tol	= 3,
	.pressure_max	= 1024,
	.get_pendown_state = ea3250_ads7846_pendown_state,
};

/*
 * SPI based LCDC data
 */
/* LCDC chip select function */
static void ea3250_spi_lcdc_cs_set(u32 control)
{
	gpio_set_value(LCD_CS_GPIO, (int) control);
}

/* LCDC SPI parameters */
static struct pl022_config_chip spi0_chip_info1 = {
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
	.cs_control		= ea3250_spi_lcdc_cs_set,
};

/* SPI devices registration */
static int __init ea3250_spi_devices_register(void)
{
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
	static struct spi_board_info info[] = {
		{
			.modalias = "spidev",
			.max_speed_hz = 2500000,
			.bus_num = 0,
			.chip_select = 0,
			.controller_data = &spi0_chip_info,
		},
	};
#else
	struct spi_board_info info[] = {
		{
			.modalias      = "ads7846",
			.max_speed_hz  = 2500000,
			.chip_select   = 0,
			.irq           = IRQ_LPC32XX_GPIO_00,
			.platform_data = &ea_ads7846_platform_data,
			.controller_data = &spi0_chip_info,
		},
		{
			.modalias      = "ea3250_lcdc",
			.max_speed_hz  = 10000000,
			.chip_select   = 1,
			.controller_data = &spi0_chip_info1,
		},
	};
#endif

	/* Configure ADS TS INT GPIO pin as input */
	if (gpio_request(ADS_TS_GPIO, "ADS7846 TS INT"))
		return -EIO;
	if(gpio_direction_input(ADS_TS_GPIO))
		return -EIO;

	/* Configure LCDC CS GPIO pin */
	if (gpio_request(LCD_CS_GPIO, "LCDC CS"))
		return -EIO;
	if(gpio_direction_output(LCD_CS_GPIO, 1))
		return -EIO;

	return spi_register_board_info(info, ARRAY_SIZE(info));
}
arch_initcall(ea3250_spi_devices_register);

#if defined (CONFIG_FB_ARMCLCD)
/*
 * LCDC AMBA Driver Board Functions
 */
#if defined (CONFIG_EA3250_QVGA_3_2_LCD)
/*
 * Support for Embedded Artists 3.2 inch QVGA LCD panel
 */
static struct clcd_panel conn_lcd_panel = {
        .mode           = {
                .name           = "QVGA portrait",
                .refresh        = 60,
                .xres           = 240,
                .yres           = 320,
                .pixclock       = 121654,
                .left_margin    = 28,
                .right_margin   = 10,
                .upper_margin   = 2,
                .lower_margin   = 2,
                .hsync_len      = 3,
                .vsync_len      = 2,
                .sync           = 0,
                .vmode          = FB_VMODE_NONINTERLACED,
        },
        .width          = -1,
        .height         = -1,
        .tim2           = (TIM2_IVS | TIM2_IHS),
        .cntl           = (CNTL_BGR | CNTL_LCDTFT | CNTL_LCDVCOMP(1) |
                                CNTL_LCDBPP16_565),
        .bpp            = 16,
};

#elif defined (CONFIG_EA3250_QVGA_2_8_OLED)
/*
 * Support for Embedded Artists 2.8 inch QVGA OLED panel
*/
static struct clcd_panel conn_lcd_panel = {
        .mode           = {
                .name           = "QVGA portrait",
                .refresh        = 60,
                .xres           = 240,
                .yres           = 320,
                .pixclock       = 176366,
                .left_margin    = 33,
                .right_margin   = 26,
                .upper_margin   = 3,
                .lower_margin   = 8,
                .hsync_len      = 32,
                .vsync_len      = 4,
                .sync           = 0,
                .vmode          = FB_VMODE_NONINTERLACED,
        },
        .width          = -1,
        .height         = -1,
        .tim2           = (TIM2_IVS | TIM2_IHS),
        .cntl           = (CNTL_BGR | CNTL_LCDTFT | CNTL_LCDVCOMP(1) |
                                CNTL_LCDBPP16_565),
        .bpp            = 16,
};
#endif

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

	if (gpio_request(SSEL0_GPIO5, "Unused GPIO5 input"))
		return -EIO;
	if(gpio_direction_input(SSEL0_GPIO5))
		return -EIO;

        /* Configure LCDC RS GPIO pin */
        if (gpio_request(LCD_RS_GPIO, "LCDC RS"))
                return -EIO;

        if(gpio_direction_output(LCD_RS_GPIO, 1))
                return -EIO;

        /* Configure LCDC Backlight GPIO pin */
        if (gpio_request(BKL_POW_GPIO, "LCDC BKL"))
                return -EIO;

#if defined (CONFIG_EA3250_QVGA_3_2_LCD)
        if(gpio_direction_output(BKL_POW_GPIO, 0)) {
#else
        if(gpio_direction_output(BKL_POW_GPIO, 1)) {
#endif
                return -EIO;
        }

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

static void spiSend(u8 *buf, size_t len)
{
        BUG_ON(ea3250_spi_lcd_dev == NULL);
        spi_write(ea3250_spi_lcd_dev, buf, len);
}

#if defined (CONFIG_EA3250_QVGA_2_8_OLED)
static void writeToDisp(u16 data)
{
        u8 buf[3];

        /* Initiliase buffer */
        buf[0] = 0x72;
        buf[1] = data >> 8;
        buf[2] = data & 0xff;
        spiSend(buf, 3);
}
#endif

static void writeToReg(u16 addr, u16 data)
{
        u8 buf[3];

#if defined (CONFIG_EA3250_QVGA_3_2_LCD)
        RESET_RS;
        buf[0] = 0x00;
        buf[1] = addr & 0xff;
        spiSend(buf, 2);

        SET_RS;
        buf[0] = data >> 8;
        buf[1] = data & 0xff;
        spiSend(buf, 2);

        RESET_RS;
        buf[0] = 0x00;
        buf[1] = 0x22;
        spiSend(buf, 2);
#elif defined (CONFIG_EA3250_QVGA_2_8_OLED)
        buf[0] = 0x70;
        buf[1] = data >> 8;
        buf[2] = data & 0xff;
        spiSend(buf, 3);
#endif

}

static void clcd_display_init(void)
{
        u32 tmp;

        /* setup MUX register to use SSP0 */
        __raw_writel(( _BIT(12) | _BIT(10) | _BIT(9) ), LPC32XX_GPIO_P_MUX_SET);
        tmp = __raw_readl(LPC32XX_GPIO_P_MUX_SET);

#if defined (CONFIG_EA3250_QVGA_3_2_LCD)

        writeToReg (0x00,0x0001);
        mdelay(20);
        writeToReg (0x03,0xA2A4);
        writeToReg (0x0C,0x0004);
        writeToReg (0x0D,0x0308);
        writeToReg (0x0E,0x3000);
        mdelay(50);
        writeToReg (0x1E,0x00AF);
        writeToReg (0x01,0x2B3F);
        writeToReg (0x02,0x0600);
        writeToReg (0x10,0x0000);
        writeToReg (0x07,0x0233);
        writeToReg (0x0B,0x0039);
        writeToReg (0x0F,0x0000);
        mdelay(50);

        writeToReg (0x30,0x0707);
        writeToReg (0x31,0x0204);
        writeToReg (0x32,0x0204);
        writeToReg (0x33,0x0502);
        writeToReg (0x34,0x0507);
        writeToReg (0x35,0x0204);
        writeToReg (0x36,0x0204);
        writeToReg (0x37,0x0502);
        writeToReg (0x3A,0x0302);
        writeToReg (0x3B,0x0302);

        writeToReg (0x23,0x0000);
        writeToReg (0x24,0x0000);

        writeToReg (0x48,0x0000);
        writeToReg (0x49,0x013F);
        writeToReg (0x4A,0x0000);
        writeToReg (0x4B,0x0000);

        writeToReg (0x41,0x0000);
        writeToReg (0x42,0x0000);

        writeToReg (0x44,0xEF00);
        writeToReg (0x45,0x0000);
        writeToReg (0x46,0x013F);
        mdelay(50);

        writeToReg (0x44,0xEF00);
        writeToReg (0x45,0x0000);
        writeToReg (0x4E,0x0000);
        writeToReg (0x4F,0x0000);
        writeToReg (0x46,0x013F);

#elif defined (CONFIG_EA3250_QVGA_2_8_OLED)

        writeToReg(0,0x02);
        writeToDisp(0x0192);

        writeToReg(0,0x03);
        writeToDisp(0x0130);

        /* set standby off */
        writeToReg(0,0x10);
        writeToDisp(0x0000);

        mdelay(100);

        /* set display on */
        writeToReg(0,0x05);
        writeToDisp(0x0001);

        /* enable image data transfer */
        writeToReg(0,0x22);
#endif
}

void clcd_disable(struct clcd_fb *fb)
{
        /* Disable the backlight */
#if defined (CONFIG_EA3250_QVGA_3_2_LCD)
        gpio_set_value(BKL_POW_GPIO, 1);
#elif defined (CONFIG_EA3250_QVGA_2_8_OLED)
        gpio_set_value(BKL_POW_GPIO, 0);
#endif
}

void clcd_enable(struct clcd_fb *fb)
{
        clcd_display_init();

        /* Enable the backlight */
#if defined (CONFIG_EA3250_QVGA_3_2_LCD)
        gpio_set_value(BKL_POW_GPIO, 0);
#elif defined (CONFIG_EA3250_QVGA_2_8_OLED)
        gpio_set_value(BKL_POW_GPIO, 1);
#endif

}

struct clcd_board lpc32xx_clcd_data = {
#if defined (CONFIG_EA3250_QVGA_3_2_LCD)
        .name           = "Embedded Artists 3.2 inch LCD",
#elif defined (CONFIG_EA3250_QVGA_2_8_OLED)
        .name           = "Embedded Artists 2.8 inch OLED",
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

/*
 * SPI LCDC Driver Probe function
 */
static int ea3250_spi_lcdc_probe(struct spi_device *spi)
{
        int err;

        spi->mode = SPI_MODE_0;
	ea3250_spi_lcd_dev = spi;

        /* SPI settings */
        err = spi_setup(spi);
        if (err < 0) {
                dev_err(&spi->dev, "Err in setting SPI \n");
                return err;
        }
        return 0;
}

/*
 *  * SPI LCDC Driver remove function
 *   * */
static int ea3250_spi_lcdc_remove(struct spi_device *spi)
{
        return 0;
}

static struct spi_driver ea3250_spi_lcdc_driver = {
        .driver = {
                .name   = "ea3250_lcdc",
                .bus    = &spi_bus_type,
                .owner  = THIS_MODULE,
        },
        .probe  = ea3250_spi_lcdc_probe,
        .remove = __devexit_p(ea3250_spi_lcdc_remove),
};

void __init ea3250_spi_lcdc_drv_init(void)
{
        spi_register_driver(&ea3250_spi_lcdc_driver);
}

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
static int __init ea3250_amba_devices_register(void)
{
	u32 i = 0;

	/* Add AMBA devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++) {
		struct amba_device *d = amba_devs[i];
		amba_device_register(d, &iomem_resource);
	}
	
	return 0;
}
device_initcall_sync(ea3250_amba_devices_register);

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
static struct mtd_partition __initdata ea3250_nand_partition[] = {
        {
                .name   = "ea3250-boot",
                .offset = 0,
                .size   = (BLK_SIZE * 25)
        },
        {
                .name   = "ea3250-uboot",
                .offset = MTDPART_OFS_APPEND,
                .size   = (BLK_SIZE * 100)
        },
        {
                .name   = "ea3250-ubt-prms",
                .offset = MTDPART_OFS_APPEND,
                .size   = (BLK_SIZE * 2)
        },
        {
                .name   = "ea3250-kernel",
                .offset = MTDPART_OFS_APPEND,
                .size   = (BLK_SIZE * 32)
        },
        {
                .name   = "ea3250-jffs2",
                .offset = MTDPART_OFS_APPEND,
                .size   = MTDPART_SIZ_FULL
        },
};
static struct mtd_partition * __init ea3250_nand_partitions(int size, int *num_partitions)
{
        *num_partitions = ARRAY_SIZE(ea3250_nand_partition);
        return ea3250_nand_partition;
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
        .partition_info         = ea3250_nand_partitions,
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

/*
 * I2C devices support
 */
#if defined (CONFIG_SENSORS_PCA9532) || defined (CONFIG_AT24)
	static struct i2c_board_info __initdata ea3250_i2c_board_info [] = {
#if defined (CONFIG_SENSORS_PCA9532)
		{
			I2C_BOARD_INFO("pca9532", I2C_PCA9532_ADDR),

		},
#endif
#if defined (CONFIG_AT24)
		{
			I2C_BOARD_INFO("24c256", I2C_24LC256_ADDR),
		},
#endif
	};
#endif

static struct platform_device* ea3250_devs[] __initdata = {
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

extern void __init ea3250_spi_lcdc_drv_init(void);

/*
 * Board specific functions
 */
void __init ea3250_board_init(void)
{
	u32 tmp;

	/* Intiliase GPIO */
	lpc32xx_gpio_init();

	/* Set SPI CS GPIO to output */
	gpio_request(SPI0_CS_GPIO, "spi0 cs");
	gpio_direction_output(SPI0_CS_GPIO, 1);

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

	/* Enable DMA for I2S1 channel */
	tmp = __raw_readl(LPC32XX_CLKPWR_I2S_CLK_CTRL);
	tmp = LPC32XX_CLKPWR_I2SCTRL_I2S1_USE_DMA;
	__raw_writel(tmp, LPC32XX_CLKPWR_I2S_CLK_CTRL);

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
	platform_add_devices(ea3250_devs, ARRAY_SIZE(ea3250_devs));

	/* Register SPI driver */
	ea3250_spi_lcdc_drv_init();
	
	/* Test clock needed for UDA1380 initial init */
	__raw_writel(LPC32XX_CLKPWR_TESTCLK2_SEL_MOSC |
			LPC32XX_CLKPWR_TESTCLK_TESTCLK2_EN,
			LPC32XX_CLKPWR_TEST_CLK_SEL);

#if defined (CONFIG_SENSORS_PCA9532) || defined (CONFIG_AT24)
	i2c_register_board_info(0, ea3250_i2c_board_info,
			ARRAY_SIZE(ea3250_i2c_board_info));
#endif
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

MACHINE_START (EA3250, "Embedded Artists LPC3250 OEM board with the LPC3250 Microcontroller")
	/* Maintainer: Embedded Artists */
	.phys_io	= LPC32XX_UART5_BASE,
	.io_pg_offst	= ((IO_ADDRESS(LPC32XX_UART5_BASE))>>18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= lpc32xx_map_io,
	.init_irq	= lpc32xx_init_irq,
	.timer		= &lpc32xx_timer,
	.init_machine	= ea3250_board_init,
	MACHINE_END

/* For backwards compatibility with older bootloaders only */
MACHINE_START (LPC3XXX, "Embedded Artists LPC3250 OEM board with the LPC3250 Microcontroller")
	/* Maintainer: Embedded Artists */
	.phys_io	= LPC32XX_UART5_BASE,
	.io_pg_offst	= ((IO_ADDRESS(LPC32XX_UART5_BASE))>>18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= lpc32xx_map_io,
	.init_irq	= lpc32xx_init_irq,
	.timer		= &lpc32xx_timer,
	.init_machine	= ea3250_board_init,
	MACHINE_END
