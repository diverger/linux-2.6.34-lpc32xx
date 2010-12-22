/*
 * Header file for I2C support on PNX010x/4008.
 *
 * Author: Dennis Kovalev <dkovalev@ru.mvista.com>
 *
 * 2004-2006 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __I2C_PNX_H__
#define __I2C_PNX_H__
#include <linux/i2c.h>

struct platform_device;
struct clk;

struct i2c_pnx_mif {
	int			ret;		/* Return value */
	int			mode;		/* Interface mode */
	struct completion	complete;	/* I/O completion */
	struct timer_list	timer;		/* Timeout */
	u8 *			buf;		/* Data buffer */
	int			len;		/* Length of data buffer */
};

struct i2c_pnx_smbus {
	int ret; /* Return value from ISR */
	int index; /* ISR use it for tx */
	int rx_cnt; /* Number of data received */
	u16 tx_buf[I2C_SMBUS_BLOCK_MAX+8]; /* Tx buffer */
	u8 *rx_buf; /* Rx buffer */
	int max_rx_len; /* Max. len. of expected Rx data */
	struct completion complete;
	int len; /* Length of data to be transmitted */
#define I2C_PNX_SMBUS_NEED_RESET  1 /* Bus needs reset */
#define I2C_PNX_SMBUS_ACTIVE      2 /* SMBus active */
#define I2C_PNX_SMBUS_BLOCK_RX    4 /* Block transfer */
#define I2C_PNX_SMBUS_WORD_RX     8 /* Word transfer */
	u32 flags; /* Updated by ISR */
	int slave_addr;
};

struct i2c_pnx_algo_data {
	void __iomem		*ioaddr;
	struct i2c_pnx_mif	mif;
	struct i2c_pnx_smbus smb;
	int			last;
	struct clk		*clk;
	struct i2c_pnx_data	*i2c_pnx;
	struct i2c_adapter	adapter;
};

struct i2c_pnx_data {
	const char *name;
	u32 base;
	int irq;
};

#endif /* __I2C_PNX_H__ */
