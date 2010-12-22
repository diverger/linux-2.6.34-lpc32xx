/*
 * Provides I2C support for Philips PNX010x/PNX4008 boards.
 *
 * Authors: Dennis Kovalev <dkovalev@ru.mvista.com>
 *	    Vitaly Wool <vwool@ru.mvista.com>
 *
 * 2004-2006 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/i2c-pnx.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/slab.h>

#include <mach/hardware.h>
#include <mach/i2c.h>

#define I2C_PNX_TIMEOUT		10 /* msec */
#define I2C_PNX_SPEED_KHZ	100
#define I2C_PNX_REGION_SIZE	0x100

static inline int wait_timeout(long timeout, struct i2c_pnx_algo_data *data)
{
	while (timeout > 0 &&
			(ioread32(I2C_REG_STS(data)) & mstatus_active)) {
		mdelay(1);
		timeout--;
	}
	return (timeout <= 0);
}

static inline int wait_reset(long timeout, struct i2c_pnx_algo_data *data)
{
	while (timeout > 0 &&
			(ioread32(I2C_REG_CTL(data)) & mcntrl_reset)) {
		mdelay(1);
		timeout--;
	}
	return (timeout <= 0);
}

static inline void i2c_pnx_arm_timer(struct i2c_pnx_algo_data *alg_data)
{
	struct timer_list *timer = &alg_data->mif.timer;
	unsigned long expires = msecs_to_jiffies(I2C_PNX_TIMEOUT);

	if (expires <= 1)
		expires = 2;

	del_timer_sync(timer);

	dev_dbg(&alg_data->adapter.dev, "Timer armed at %lu plus %lu jiffies.\n",
		jiffies, expires);

	timer->expires = jiffies + expires;
	timer->data = (unsigned long)&alg_data;

	add_timer(timer);
}

/**
 * i2c_pnx_start - start a device
 * @slave_addr:		slave address
 * @adap:		pointer to adapter structure
 *
 * Generate a START signal in the desired mode.
 */
static int i2c_pnx_start(unsigned char slave_addr,
	struct i2c_pnx_algo_data *alg_data)
{
	dev_dbg(&alg_data->adapter.dev, "%s(): addr 0x%x mode %d\n", __func__,
		slave_addr, alg_data->mif.mode);

	/* Check for 7 bit slave addresses only */
	if (slave_addr & ~0x7f) {
		dev_err(&alg_data->adapter.dev,
			"%s: Invalid slave address %x. Only 7-bit addresses are supported\n",
			alg_data->adapter.name, slave_addr);
		return -EINVAL;
	}

	/* First, make sure bus is idle */
	if (wait_timeout(I2C_PNX_TIMEOUT, alg_data)) {
		/* Somebody else is monopolizing the bus */
		dev_err(&alg_data->adapter.dev,
			"%s: Bus busy. Slave addr = %02x, cntrl = %x, stat = %x\n",
			alg_data->adapter.name, slave_addr,
			ioread32(I2C_REG_CTL(alg_data)),
			ioread32(I2C_REG_STS(alg_data)));
		return -EBUSY;
	} else if (ioread32(I2C_REG_STS(alg_data)) & mstatus_afi) {
		/* Sorry, we lost the bus */
		dev_err(&alg_data->adapter.dev,
		        "%s: Arbitration failure. Slave addr = %02x\n",
			alg_data->adapter.name, slave_addr);
		return -EIO;
	}

	/*
	 * OK, I2C is enabled and we have the bus.
	 * Clear the current TDI and AFI status flags.
	 */
	iowrite32(ioread32(I2C_REG_STS(alg_data)) | mstatus_tdi | mstatus_afi,
		  I2C_REG_STS(alg_data));

	dev_dbg(&alg_data->adapter.dev, "%s(): sending %#x\n", __func__,
		(slave_addr << 1) | start_bit | alg_data->mif.mode);

	/* Write the slave address, START bit and R/W bit */
	iowrite32((slave_addr << 1) | start_bit | alg_data->mif.mode,
		  I2C_REG_TX(alg_data));

	dev_dbg(&alg_data->adapter.dev, "%s(): exit\n", __func__);

	return 0;
}

/**
 * i2c_pnx_stop - stop a device
 * @adap:		pointer to I2C adapter structure
 *
 * Generate a STOP signal to terminate the master transaction.
 */
static void i2c_pnx_stop(struct i2c_pnx_algo_data *alg_data)
{
	/* Only 1 msec max timeout due to interrupt context */
	long timeout = 1000;

	dev_dbg(&alg_data->adapter.dev, "%s(): entering: stat = %04x.\n",
		__func__, ioread32(I2C_REG_STS(alg_data)));

	/* Write a STOP bit to TX FIFO */
	iowrite32(0xff | stop_bit, I2C_REG_TX(alg_data));

	/* Wait until the STOP is seen. */
	while (timeout > 0 &&
	       (ioread32(I2C_REG_STS(alg_data)) & mstatus_active)) {
		/* may be called from interrupt context */
		udelay(1);
		timeout--;
	}

	dev_dbg(&alg_data->adapter.dev, "%s(): exiting: stat = %04x.\n",
		__func__, ioread32(I2C_REG_STS(alg_data)));
}

/**
 * i2c_pnx_master_xmit - transmit data to slave
 * @adap:		pointer to I2C adapter structure
 *
 * Sends one byte of data to the slave
 */
static int i2c_pnx_master_xmit(struct i2c_pnx_algo_data *alg_data)
{
	u32 val;

	dev_dbg(&alg_data->adapter.dev, "%s(): entering: stat = %04x.\n",
		__func__, ioread32(I2C_REG_STS(alg_data)));

	if (alg_data->mif.len > 0) {
		/* We still have something to talk about... */
		val = *alg_data->mif.buf++;

		if (alg_data->mif.len == 1)
			val |= stop_bit;

		alg_data->mif.len--;
		iowrite32(val, I2C_REG_TX(alg_data));

		dev_dbg(&alg_data->adapter.dev, "%s(): xmit %#x [%d]\n",
			__func__, val, alg_data->mif.len + 1);

		if (alg_data->mif.len == 0) {
			if (alg_data->last) {
				/* Wait until the STOP is seen. */
				if (wait_timeout(I2C_PNX_TIMEOUT, alg_data))
					dev_err(&alg_data->adapter.dev,
						"The bus is still active after timeout\n");
			}
			/* Disable master interrupts */
			iowrite32(ioread32(I2C_REG_CTL(alg_data)) &
				~(mcntrl_afie | mcntrl_naie | mcntrl_drmie),
				  I2C_REG_CTL(alg_data));

			del_timer_sync(&alg_data->mif.timer);

			dev_dbg(&alg_data->adapter.dev,
				"%s(): Waking up xfer routine.\n",
				__func__);

			complete(&alg_data->mif.complete);
		}
	} else if (alg_data->mif.len == 0) {
		/* zero-sized transfer */
		i2c_pnx_stop(alg_data);

		/* Disable master interrupts. */
		iowrite32(ioread32(I2C_REG_CTL(alg_data)) &
			~(mcntrl_afie | mcntrl_naie | mcntrl_drmie),
			  I2C_REG_CTL(alg_data));

		/* Stop timer. */
		del_timer_sync(&alg_data->mif.timer);
		dev_dbg(&alg_data->adapter.dev,
			"%s(): Waking up xfer routine after zero-xfer.\n",
			__func__);

		complete(&alg_data->mif.complete);
	}

	dev_dbg(&alg_data->adapter.dev, "%s(): exiting: stat = %04x.\n",
		__func__, ioread32(I2C_REG_STS(alg_data)));

	return 0;
}

/**
 * i2c_pnx_master_rcv - receive data from slave
 * @adap:		pointer to I2C adapter structure
 *
 * Reads one byte data from the slave
 */
static int i2c_pnx_master_rcv(struct i2c_pnx_algo_data *alg_data)
{
	unsigned int val = 0;
	u32 ctl = 0;

	dev_dbg(&alg_data->adapter.dev, "%s(): entering: stat = %04x.\n",
		__func__, ioread32(I2C_REG_STS(alg_data)));

	/* Check, whether there is already data,
	 * or we didn't 'ask' for it yet.
	 */
	if (ioread32(I2C_REG_STS(alg_data)) & mstatus_rfe) {
		dev_dbg(&alg_data->adapter.dev,
			"%s(): Write dummy data to fill Rx-fifo...\n",
			__func__);

		if (alg_data->mif.len == 1) {
			/* Last byte, do not acknowledge next rcv. */
			val |= stop_bit;

			/*
			 * Enable interrupt RFDAIE (data in Rx fifo),
			 * and disable DRMIE (need data for Tx)
			 */
			ctl = ioread32(I2C_REG_CTL(alg_data));
			ctl |= mcntrl_rffie | mcntrl_daie;
			ctl &= ~mcntrl_drmie;
			iowrite32(ctl, I2C_REG_CTL(alg_data));
		}

		/*
		 * Now we'll 'ask' for data:
		 * For each byte we want to receive, we must
		 * write a (dummy) byte to the Tx-FIFO.
		 */
		iowrite32(val, I2C_REG_TX(alg_data));

		return 0;
	}

	/* Handle data. */
	if (alg_data->mif.len > 0) {
		val = ioread32(I2C_REG_RX(alg_data));
		*alg_data->mif.buf++ = (u8) (val & 0xff);
		dev_dbg(&alg_data->adapter.dev, "%s(): rcv 0x%x [%d]\n",
			__func__, val, alg_data->mif.len);

		alg_data->mif.len--;
		if (alg_data->mif.len == 0) {
			if (alg_data->last)
				/* Wait until the STOP is seen. */
				if (wait_timeout(I2C_PNX_TIMEOUT, alg_data))
					dev_err(&alg_data->adapter.dev,
						"The bus is still active after timeout\n");

			/* Disable master interrupts */
			ctl = ioread32(I2C_REG_CTL(alg_data));
			ctl &= ~(mcntrl_afie | mcntrl_naie | mcntrl_rffie |
				 mcntrl_drmie | mcntrl_daie);
			iowrite32(ctl, I2C_REG_CTL(alg_data));

			/* Kill timer. */
			del_timer_sync(&alg_data->mif.timer);
			complete(&alg_data->mif.complete);
		}
	}

	dev_dbg(&alg_data->adapter.dev, "%s(): exiting: stat = %04x.\n",
		__func__, ioread32(I2C_REG_STS(alg_data)));

	return 0;
}

/* SMBus receive handler, called from ISR */
static int i2c_pnx_smbus_rx(struct i2c_pnx_algo_data *smbus)
{
	struct i2c_pnx_smbus *smb = &smbus->smb;
	while (!(ioread32(I2C_REG_STS(smbus)) & mstatus_rfe)) {
		smb->rx_buf[smb->rx_cnt++] =
			(u8) ioread32(I2C_REG_RX(smbus));
		dev_dbg(&smbus->adapter.dev, "Rx Char: %02x\n",
				smb->rx_buf[smb->rx_cnt - 1]);

		/* If Rx char is not length then continue receiving */
		if ((smb->rx_cnt != 1) ||
			!(smb->flags & I2C_PNX_SMBUS_BLOCK_RX))
			continue;

		/* If Received char is length, check for validity */
		if (unlikely(smb->rx_buf[0] <= 0 &&
			smb->rx_buf[0] > smb->max_rx_len)) {
			dev_dbg(&smbus->adapter.dev, "ERR: SMBus received "
					"invalid transfer length %d from slave"
					" %#02x during a block transfer.\n",
					smb->rx_buf[0],
					smb->slave_addr);
			smb->ret = -EIO;
			complete(&smb->complete);
			smb->flags |= I2C_PNX_SMBUS_NEED_RESET;
			return 1;
		}

		/* There is a hardware BUG, that makes receiving
		 * only length not possible, so we receive length
		 * and a byte of data, if that is the only byte to
		 * be received the the transfer must stop right away
		 **/
		if (smb->rx_buf[0] == 1) {
			/* Stop xfer right away */
			iowrite32(stop_bit, I2C_REG_TX(smbus));
			iowrite32(ioread32(I2C_REG_CTL(smbus)) &
					~(mcntrl_drmie | mcntrl_tffie),
					I2C_REG_CTL(smbus));
			return 1;
		}

		dev_dbg(&smbus->adapter.dev, "Set Len:%d\n",
				smb->rx_buf[0]);
		smb->len += smb->rx_buf[0] - 1;
		smb->tx_buf[smb->len - 1] |= stop_bit;
		iowrite32(ioread32(I2C_REG_CTL(smbus)) |
				  mcntrl_tffie, I2C_REG_CTL(smbus));
	}
	return 0;
}

/* SMBUs interrupt handler */
static irqreturn_t i2c_pnx_smbus_isr(int irq, void *dev_id)
{
	struct i2c_pnx_algo_data *smbus = dev_id;
	struct i2c_pnx_smbus *smb = &smbus->smb;
	u32 stat, ctl;
	stat = ioread32(I2C_REG_STS(smbus));
	ctl = ioread32(I2C_REG_CTL(smbus));

	dev_dbg(&smbus->adapter.dev, "ISR: stat = %#08x, "
			"ctrl = %#08x\r\n", stat, ctl);

	/* Handle Rx data */
	if (((stat & mstatus_rff) && (ctl & mcntrl_rffie)) ||
	    (!(stat & mstatus_rfe) && (ctl & mcntrl_daie))) {
		if (i2c_pnx_smbus_rx(smbus))
			return IRQ_HANDLED;
		stat = ioread32(I2C_REG_STS(smbus));
		ctl = ioread32(I2C_REG_CTL(smbus));
	}

	/* Handle Transmit */
	if (((stat & mstatus_drmi) && (ctl & mcntrl_drmie)) ||
	    (!(stat & mstatus_rff) && (ctl & mcntrl_tffie))) {

		/* Push data into FIFO until we run out of data
		 * or TX/RX fifo is full
		 **/
		for (; (smb->index < smb->len) &&
		     !(ioread32(I2C_REG_STS(smbus)) &
				 (mstatus_tff | mstatus_rff));
		     smb->index++) {
			iowrite32(smb->tx_buf[smb->index], I2C_REG_TX(smbus));
			dev_dbg(&smbus->adapter.dev, "Tx Char: %03x\n",
					smb->tx_buf[smb->index]);
		}

		/* Stop further transmit if we run out of data */
		if (smb->index >= smb->len) {
			iowrite32(ctl & ~(mcntrl_drmie | mcntrl_tffie),
					I2C_REG_CTL(smbus));
		}
		return IRQ_HANDLED;
	}

	/* Handle Arbitration loss */
	if (unlikely((stat & mstatus_afi) && (ctl & mcntrl_afie))) {
		dev_dbg(&smbus->adapter.dev, "Aribitration lost during"
				" transfer to/from slave addr %02x\r\n",
				smb->slave_addr);
		smb->ret = -EAGAIN;
		complete(&smb->complete);
		smb->flags |= I2C_PNX_SMBUS_NEED_RESET;
		/* We are done! */
		iowrite32(0, I2C_REG_CTL(smbus));
		return IRQ_HANDLED;
	}

	/* Handle NACK reception */
	if (unlikely((stat & mstatus_nai) && (ctl & mcntrl_naie))) {
		dev_dbg(&smbus->adapter.dev, "Nack received!\n");
		smb->ret = -EIO;
		complete(&smb->complete);
		smb->flags |= I2C_PNX_SMBUS_NEED_RESET;
		/* We are done! */
		iowrite32(0, I2C_REG_CTL(smbus));
		return IRQ_HANDLED;
	}

	/* Handle Xfer Done */
	if ((stat & mstatus_tdi) &&
	    (mcntrl_tdie & ctl)) {
		dev_dbg(&smbus->adapter.dev, "SMBus Xfer Done!\r\n");
		/* Transmission is done */
		smb->ret = 0;
		complete(&smb->complete);
		iowrite32(mstatus_tdi, I2C_REG_STS(smbus));
		iowrite32(0, I2C_REG_CTL(smbus));
		return IRQ_HANDLED;
	}

	return IRQ_HANDLED;
}

static irqreturn_t i2c_pnx_interrupt(int irq, void *dev_id)
{
	struct i2c_pnx_algo_data *alg_data = dev_id;
	u32 stat, ctl;

	/* If it is an SMBUS xfer let another handler do the task */
	if (alg_data->smb.flags & I2C_PNX_SMBUS_ACTIVE)
		return i2c_pnx_smbus_isr(irq, dev_id);

	dev_dbg(&alg_data->adapter.dev,
		"%s(): mstat = %x mctrl = %x, mode = %d\n",
		__func__,
		ioread32(I2C_REG_STS(alg_data)),
		ioread32(I2C_REG_CTL(alg_data)),
		alg_data->mif.mode);
	stat = ioread32(I2C_REG_STS(alg_data));

	/* let's see what kind of event this is */
	if (stat & mstatus_afi) {
		/* We lost arbitration in the midst of a transfer */
		alg_data->mif.ret = -EIO;

		/* Disable master interrupts. */
		ctl = ioread32(I2C_REG_CTL(alg_data));
		ctl &= ~(mcntrl_afie | mcntrl_naie | mcntrl_rffie |
			 mcntrl_drmie);
		iowrite32(ctl, I2C_REG_CTL(alg_data));

		/* Stop timer, to prevent timeout. */
		del_timer_sync(&alg_data->mif.timer);
		complete(&alg_data->mif.complete);
	} else if (stat & mstatus_nai) {
		/* Slave did not acknowledge, generate a STOP */
		dev_dbg(&alg_data->adapter.dev,
			"%s(): Slave did not acknowledge, generating a STOP.\n",
			__func__);
		i2c_pnx_stop(alg_data);

		/* Disable master interrupts. */
		ctl = ioread32(I2C_REG_CTL(alg_data));
		ctl &= ~(mcntrl_afie | mcntrl_naie | mcntrl_rffie |
			 mcntrl_drmie);
		iowrite32(ctl, I2C_REG_CTL(alg_data));

		/* Our return value. */
		alg_data->mif.ret = -EIO;

		/* Stop timer, to prevent timeout. */
		del_timer_sync(&alg_data->mif.timer);
		complete(&alg_data->mif.complete);
	} else {
		/*
		 * Two options:
		 * - Master Tx needs data.
		 * - There is data in the Rx-fifo
		 * The latter is only the case if we have requested for data,
		 * via a dummy write. (See 'i2c_pnx_master_rcv'.)
		 * We therefore check, as a sanity check, whether that interrupt
		 * has been enabled.
		 */
		if ((stat & mstatus_drmi) || !(stat & mstatus_rfe)) {
			if (alg_data->mif.mode == I2C_SMBUS_WRITE) {
				i2c_pnx_master_xmit(alg_data);
			} else if (alg_data->mif.mode == I2C_SMBUS_READ) {
				i2c_pnx_master_rcv(alg_data);
			}
		}
	}

	/* Clear TDI and AFI bits */
	stat = ioread32(I2C_REG_STS(alg_data));
	iowrite32(stat | mstatus_tdi | mstatus_afi, I2C_REG_STS(alg_data));

	dev_dbg(&alg_data->adapter.dev,
		"%s(): exiting, stat = %x ctrl = %x.\n",
		 __func__, ioread32(I2C_REG_STS(alg_data)),
		 ioread32(I2C_REG_CTL(alg_data)));

	return IRQ_HANDLED;
}

static void i2c_pnx_timeout(unsigned long data)
{
	struct i2c_pnx_algo_data *alg_data = (struct i2c_pnx_algo_data *)data;
	u32 ctl;

	dev_err(&alg_data->adapter.dev,
		"Master timed out. stat = %04x, cntrl = %04x. Resetting master...\n",
		ioread32(I2C_REG_STS(alg_data)),
		ioread32(I2C_REG_CTL(alg_data)));

	/* Reset master and disable interrupts */
	ctl = ioread32(I2C_REG_CTL(alg_data));
	ctl &= ~(mcntrl_afie | mcntrl_naie | mcntrl_rffie | mcntrl_drmie);
	iowrite32(ctl, I2C_REG_CTL(alg_data));

	ctl |= mcntrl_reset;
	iowrite32(ctl, I2C_REG_CTL(alg_data));
	wait_reset(I2C_PNX_TIMEOUT, alg_data);
	alg_data->mif.ret = -EIO;
	complete(&alg_data->mif.complete);
}

static inline void bus_reset_if_active(struct i2c_pnx_algo_data *alg_data)
{
	u32 stat;

	if ((stat = ioread32(I2C_REG_STS(alg_data))) & mstatus_active) {
		dev_err(&alg_data->adapter.dev,
			"%s: Bus is still active after xfer. Reset it...\n",
			alg_data->adapter.name);
		iowrite32(ioread32(I2C_REG_CTL(alg_data)) | mcntrl_reset,
			  I2C_REG_CTL(alg_data));
		wait_reset(I2C_PNX_TIMEOUT, alg_data);
	} else if (!(stat & mstatus_rfe) || !(stat & mstatus_tfe)) {
		/* If there is data in the fifo's after transfer,
		 * flush fifo's by reset.
		 */
		iowrite32(ioread32(I2C_REG_CTL(alg_data)) | mcntrl_reset,
			  I2C_REG_CTL(alg_data));
		wait_reset(I2C_PNX_TIMEOUT, alg_data);
	} else if (stat & mstatus_nai) {
		iowrite32(ioread32(I2C_REG_CTL(alg_data)) | mcntrl_reset,
			  I2C_REG_CTL(alg_data));
		wait_reset(I2C_PNX_TIMEOUT, alg_data);
	}
}

/**
 * i2c_pnx_xfer - I2C Protocol Transfer routine
 * @adap:		pointer to I2C adapter structure
 * @msgs:		array of messages
 * @num:		number of messages
 *
 * Initiates the transfer
 */
static int
i2c_pnx_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct i2c_msg *pmsg;
	int rc = 0, completed = 0, i;
	struct i2c_pnx_algo_data *alg_data = adap->algo_data;
	u32 stat = ioread32(I2C_REG_STS(alg_data));

	dev_dbg(&alg_data->adapter.dev,
		"%s(): entering: %d messages, stat = %04x.\n",
		__func__, num, ioread32(I2C_REG_STS(alg_data)));

	bus_reset_if_active(alg_data);

	/* Process transactions in a loop. */
	for (i = 0; rc >= 0 && i < num; i++) {
		u8 addr;

		pmsg = &msgs[i];
		addr = pmsg->addr;

		if (pmsg->flags & I2C_M_TEN) {
			dev_err(&alg_data->adapter.dev,
				"%s: 10 bits addr not supported!\n",
				alg_data->adapter.name);
			rc = -EINVAL;
			break;
		}

		alg_data->mif.buf = pmsg->buf;
		alg_data->mif.len = pmsg->len;
		alg_data->mif.mode = (pmsg->flags & I2C_M_RD) ?
			I2C_SMBUS_READ : I2C_SMBUS_WRITE;
		alg_data->mif.ret = 0;
		alg_data->last = (i == num - 1);

		dev_dbg(&alg_data->adapter.dev, "%s(): mode %d, %d bytes\n",
			__func__, alg_data->mif.mode, alg_data->mif.len);

		i2c_pnx_arm_timer(alg_data);

		/* initialize the completion var */
		init_completion(&alg_data->mif.complete);

		/* Enable master interrupt */
		iowrite32(ioread32(I2C_REG_CTL(alg_data)) | mcntrl_afie |
				mcntrl_naie | mcntrl_drmie,
			  I2C_REG_CTL(alg_data));

		/* Put start-code and slave-address on the bus. */
		rc = i2c_pnx_start(addr, alg_data);
		if (rc < 0)
			break;

		/* Wait for completion */
		wait_for_completion(&alg_data->mif.complete);

		if (!(rc = alg_data->mif.ret))
			completed++;
		dev_dbg(&alg_data->adapter.dev,
			"%s(): Complete, return code = %d.\n",
			__func__, rc);

		/* Clear TDI and AFI bits in case they are set. */
		if ((stat = ioread32(I2C_REG_STS(alg_data))) & mstatus_tdi) {
			dev_dbg(&alg_data->adapter.dev,
				"%s: TDI still set... clearing now.\n",
				alg_data->adapter.name);
			iowrite32(stat, I2C_REG_STS(alg_data));
		}
		if ((stat = ioread32(I2C_REG_STS(alg_data))) & mstatus_afi) {
			dev_dbg(&alg_data->adapter.dev,
				"%s: AFI still set... clearing now.\n",
				alg_data->adapter.name);
			iowrite32(stat, I2C_REG_STS(alg_data));
		}
	}

	bus_reset_if_active(alg_data);

	/* Cleanup to be sure... */
	alg_data->mif.buf = NULL;
	alg_data->mif.len = 0;

	dev_dbg(&alg_data->adapter.dev, "%s(): exiting, stat = %x\n",
		__func__, ioread32(I2C_REG_STS(alg_data)));

	if (completed != num)
		return ((rc < 0) ? rc : -EREMOTEIO);

	return num;
}

/* Checks for the state of I2C BUS */
static int i2c_pnx_smbus_check(struct i2c_pnx_algo_data *smbus)
{
	u32 stat;
	int need_reset;

	stat = ioread32(I2C_REG_STS(smbus));

	/* Reset if bus is still active or a NACK from prev xfer pending */
	need_reset = stat & (mstatus_active | mstatus_nai);

	/* Reset if TXFIFO or RXFIFO is not empty */
	need_reset |= ~stat & (mstatus_rfe | mstatus_tfe);

	if (unlikely(need_reset)) {
		dev_dbg(&smbus->adapter.dev, "SMBus is not in idle state"
				" before transfer, resetting it...\r\n");
		iowrite32(ioread32(I2C_REG_CTL(smbus)) | mcntrl_reset,
			  I2C_REG_CTL(smbus));
		wait_reset(I2C_PNX_TIMEOUT, smbus);
	}

	stat = ioread32(I2C_REG_STS(smbus));

	/* if the bus is still busy ask i2c-core to retry after sometime */
	if (stat & mstatus_active) {
		dev_dbg(&smbus->adapter.dev, "SMBus is still active!\r\n");
		return -EAGAIN;
	}

	return 0;
}

/* Initialize SMBus structure */
static void i2c_pnx_smbus_init(struct i2c_pnx_algo_data *smbus)
{
	struct i2c_pnx_smbus *smb = &smbus->smb;
	smb->index = 0;
	smb->ret = 0;
	smb->flags = I2C_PNX_SMBUS_ACTIVE;
	smb->len = 0;
	smb->rx_cnt = 0;
	iowrite32(mstatus_tdi, I2C_REG_STS(smbus));
	init_completion(&smb->complete);
}

static void i2c_pnx_fill_buffer(u16 *to, u8 *from, int cnt)
{
	int i;
	for (i = 0; i < cnt; i++)
		to[i] = from ? from[i] : 0;
}

/**
 * i2c_pnx_smbus_xfer - SMBUS protocol transfer routine
 * @adapter:	pointer to I2C adapter structure
 * @msgs:		array of messages
 * @num:		number of messages
 *
 * Initiates the transfer
 */
static int i2c_pnx_smbus_xfer(struct i2c_adapter *adapter,
		u16 addr, unsigned short flags, char read_write, u8 command,
		int size, union i2c_smbus_data *data)
{
	struct i2c_pnx_algo_data *smbus = adapter->algo_data;
	struct i2c_pnx_smbus *smb = &smbus->smb;
	u16 *tx_buf = smb->tx_buf;
	int read_flag, err;
	int len = 0, i = 0;

	dev_dbg(&adapter->dev, "SMBus xfer request: Slave addr %#02x,"
			"command=%d, operation=%d\r\n", addr, command, size);

	smb->slave_addr = addr;
	/* All our ops take 8-bit shifted addresses */
	addr <<= 1;
	read_flag = read_write == I2C_SMBUS_READ;

	err = i2c_pnx_smbus_check(smbus);
	if (unlikely(err))
		return err;

	i2c_pnx_smbus_init(smbus);

	smb->rx_buf = data->block;
	switch (size) {
	case I2C_SMBUS_QUICK:
		tx_buf[0] = addr | start_bit | stop_bit | read_flag;
		read_flag = 0;
		smb->len = 1;
		break;

	case I2C_SMBUS_BYTE:
		tx_buf[0] = addr | start_bit | read_flag;
		tx_buf[1] = command | stop_bit;
		smb->len = 2;
		break;

	case I2C_SMBUS_BYTE_DATA:
		i = 0;
		tx_buf[i++] = addr | start_bit;
		tx_buf[i++] = command;
		if (read_flag)
			tx_buf[i++] = addr | start_bit | 1;
		tx_buf[i++] = data->byte | stop_bit;
		smb->len = i;
		break;

	case I2C_SMBUS_WORD_DATA:
		i = 0;
		tx_buf[i++] = addr | start_bit;
		tx_buf[i++] = command;
		if (read_flag)
			tx_buf[i++] = addr | start_bit | 1;
		tx_buf[i++] = (data->word & 0xFF); /* Low Word */
		tx_buf[i++] = ((data->word >> 8) & 0xFF) | stop_bit;
		smb->len = i;
		smb->flags |= I2C_PNX_SMBUS_WORD_RX;
		break;

	case I2C_SMBUS_BLOCK_DATA:
		len = data->block[0];
		tx_buf[i++] = addr | start_bit;
		tx_buf[i++] = command;
		if (read_flag) {
			tx_buf[i++] = addr | start_bit | 1;
			i2c_pnx_fill_buffer(&tx_buf[i],
					(u8 *)NULL, I2C_SMBUS_BLOCK_MAX + 1);
			tx_buf[I2C_SMBUS_BLOCK_MAX + i] |= stop_bit;
			smb->rx_buf = data->block;
			smb->flags |= I2C_PNX_SMBUS_BLOCK_RX;
			smb->len = i + 2;
			smb->max_rx_len = I2C_SMBUS_BLOCK_MAX;
		} else {
			if (!len)
				return -EIO;
			i2c_pnx_fill_buffer(&tx_buf[i],
			    data->block, len + 1);
			tx_buf[len + i] |= stop_bit;
			smb->len = len + i + 1;
		}
		break;

	case I2C_SMBUS_PROC_CALL:
		tx_buf[0] = addr | start_bit;
		tx_buf[1] = command;
		tx_buf[2] = data->word & 0xFF;
		tx_buf[3] = (data->word >> 8) & 0xFF;
		tx_buf[4] = addr | start_bit | 1;
		tx_buf[5] = 0;
		tx_buf[6] = 0 | stop_bit;
		smb->len = 7;
		smb->max_rx_len = 2;
		smb->flags |= I2C_PNX_SMBUS_WORD_RX;
		read_flag = 1;
		break;

	case I2C_SMBUS_BLOCK_PROC_CALL:
		len = data->block[0];
		if (!len)
			return -EINVAL;
		tx_buf[0] = addr | start_bit;
		tx_buf[1] = command;
		i2c_pnx_fill_buffer(&tx_buf[2],
			data->block, len + 1);
		i = 3 + len;
		tx_buf[i++] = addr | start_bit | 1;
		len = I2C_SMBUS_BLOCK_MAX - len;
		i2c_pnx_fill_buffer(&tx_buf[i],
		    NULL, len + 1);
		tx_buf[i+len] = stop_bit;
		smb->flags |= I2C_PNX_SMBUS_BLOCK_RX;
		smb->max_rx_len = len;
		smb->len = 2 + i;
		read_flag = 1;
		break;

	default:
		dev_warn(&adapter->dev, "Unsupported transaction %d\n", size);
		return -EINVAL;
	}
	/* Enable interrupts and wait for completion of xfer */
	iowrite32(ioread32(I2C_REG_CTL(smbus)) | 0xEF, I2C_REG_CTL(smbus));

	err = wait_for_completion_interruptible_timeout(&smb->complete, HZ);

	/* Disable interrupts */
	iowrite32(ioread32(I2C_REG_CTL(smbus)) & ~0xEF, I2C_REG_CTL(smbus));
	smb->flags &= ~I2C_PNX_SMBUS_ACTIVE;

	if (err == 0) { /* Xfer timedout */
		dev_dbg(&adapter->dev, "SMBus Xfer timedout"
				"[Slave Addr: %02x]\n", addr >> 1);
		err = -ETIMEDOUT;
		smb->flags |= I2C_PNX_SMBUS_NEED_RESET;
	} else if (err > 0) { /* No error */
		err = smb->ret;
	} else { /* < 0 Possibly interrupted */
		smb->flags |= I2C_PNX_SMBUS_NEED_RESET;
	}

	/* Handle post processing for a Rx xfer */
	if (!err && read_flag) {
		len = (smb->flags & I2C_PNX_SMBUS_BLOCK_RX) ?
			data->block[0] + 1 : 1;

		if (smb->flags & I2C_PNX_SMBUS_WORD_RX) {
			len = 2;
			/* Return endian independent data */
			data->word = (data->block[0] & 0xFF) |
				((data->block[1] & 0xFF) << 8);
		}
		if (unlikely(len > smb->rx_cnt)) {
			dev_err(&adapter->dev, "SMBus: Rx count error "
					"[Expected:%d, Got:%d] slave: %#02x\n",
					len, smb->rx_cnt, addr >> 1);
			err = -EIO;
		}
	}

	if (unlikely(smb->flags & I2C_PNX_SMBUS_NEED_RESET)) {
		iowrite32(ioread32(I2C_REG_CTL(smbus)) | mcntrl_reset,
			  I2C_REG_CTL(smbus));
	}
	return err;
}

/**
 * i2c_pnx_func - SMBUS protocol transfer routine
 * @adapt:	pointer to I2C adapter structure
 *
 * Provides the list of functionality provided by pnx-i2c
 *
 * I2C_FUNC_10BIT_ADDR - is supported by hardware but
 * this driver does not implement it!
 */
static u32 i2c_pnx_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_QUICK | I2C_FUNC_SMBUS_BYTE |
	       I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
	       I2C_FUNC_SMBUS_BLOCK_DATA | I2C_FUNC_SMBUS_PROC_CALL |
	       I2C_FUNC_SMBUS_BLOCK_PROC_CALL | I2C_FUNC_I2C;
}

static const struct i2c_algorithm pnx_algorithm = {
	.master_xfer = i2c_pnx_xfer,
	.smbus_xfer  = i2c_pnx_smbus_xfer,
	.functionality = i2c_pnx_func,
};

#ifdef CONFIG_PM
static int i2c_pnx_controller_suspend(struct platform_device *pdev,
				      pm_message_t state)
{
	struct i2c_pnx_algo_data *alg_data = platform_get_drvdata(pdev);

	/* FIXME: shouldn't this be clk_disable? */
	clk_enable(alg_data->clk);

	return 0;
}

static int i2c_pnx_controller_resume(struct platform_device *pdev)
{
	struct i2c_pnx_algo_data *alg_data = platform_get_drvdata(pdev);

	return clk_enable(alg_data->clk);
}
#else
#define i2c_pnx_controller_suspend	NULL
#define i2c_pnx_controller_resume	NULL
#endif

static int __devinit i2c_pnx_probe(struct platform_device *pdev)
{
	unsigned long tmp;
	int ret = 0;
	struct i2c_pnx_algo_data *alg_data;
	unsigned long freq;
	struct i2c_pnx_data *i2c_pnx = pdev->dev.platform_data;

	if (!i2c_pnx || !i2c_pnx->name) {
		dev_err(&pdev->dev, "%s: no platform data supplied\n",
		       __func__);
		ret = -EINVAL;
		goto out;
	}

	alg_data = kzalloc(sizeof(*alg_data), GFP_KERNEL);
	if (!alg_data) {
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	platform_set_drvdata(pdev, alg_data);

	strlcpy(alg_data->adapter.name, i2c_pnx->name,
		sizeof(alg_data->adapter.name));
	alg_data->adapter.dev.parent = &pdev->dev;
	alg_data->adapter.algo = &pnx_algorithm;
	alg_data->adapter.algo_data = alg_data;
	alg_data->adapter.nr = pdev->id;
	alg_data->i2c_pnx = i2c_pnx;

	alg_data->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(alg_data->clk)) {
		ret = PTR_ERR(alg_data->clk);
		goto out_drvdata;
	}

	init_timer(&alg_data->mif.timer);
	alg_data->mif.timer.function = i2c_pnx_timeout;
	alg_data->mif.timer.data = (unsigned long)alg_data;

	/* Register I/O resource */
	if (!request_mem_region(i2c_pnx->base, I2C_PNX_REGION_SIZE,
				pdev->name)) {
		dev_err(&pdev->dev,
		       "I/O region 0x%08x for I2C already in use.\n",
		       i2c_pnx->base);
		ret = -ENODEV;
		goto out_clkget;
	}

	alg_data->ioaddr = ioremap(i2c_pnx->base, I2C_PNX_REGION_SIZE);
	if (!alg_data->ioaddr) {
		dev_err(&pdev->dev, "Couldn't ioremap I2C I/O region\n");
		ret = -ENOMEM;
		goto out_release;
	}

	ret = clk_enable(alg_data->clk);
	if (ret)
		goto out_unmap;

	freq = clk_get_rate(alg_data->clk);

	/*
	 * Clock Divisor High This value is the number of system clocks
	 * the serial clock (SCL) will be high.
	 * For example, if the system clock period is 50 ns and the maximum
	 * desired serial period is 10000 ns (100 kHz), then CLKHI would be
	 * set to 0.5*(f_sys/f_i2c)-2=0.5*(20e6/100e3)-2=98. The actual value
	 * programmed into CLKHI will vary from this slightly due to
	 * variations in the output pad's rise and fall times as well as
	 * the deglitching filter length.
	 */

	tmp = ((freq / 1000) / I2C_PNX_SPEED_KHZ) / 2 - 2;
	if (tmp > 0x3FF)
		tmp = 0x3FF;
	iowrite32(tmp, I2C_REG_CKH(alg_data));
	iowrite32(tmp, I2C_REG_CKL(alg_data));

	iowrite32(mcntrl_reset, I2C_REG_CTL(alg_data));
	if (wait_reset(I2C_PNX_TIMEOUT, alg_data)) {
		ret = -ENODEV;
		goto out_clock;
	}
	init_completion(&alg_data->mif.complete);

	ret = request_irq(i2c_pnx->irq, i2c_pnx_interrupt,
			0, pdev->name, alg_data);
	if (ret)
		goto out_clock;

	/* Register this adapter with the I2C subsystem */
	ret = i2c_add_numbered_adapter(&alg_data->adapter);
	if (ret < 0) {
		dev_err(&pdev->dev, "I2C: Failed to add bus\n");
		goto out_irq;
	}

	dev_dbg(&pdev->dev, "%s: Master at %#8x, irq %d.\n",
	       alg_data->adapter.name, i2c_pnx->base, i2c_pnx->irq);

	return 0;

out_irq:
	free_irq(i2c_pnx->irq, alg_data);
out_clock:
	clk_disable(alg_data->clk);
out_unmap:
	iounmap(alg_data->ioaddr);
out_release:
	release_mem_region(i2c_pnx->base, I2C_PNX_REGION_SIZE);
out_clkget:
	clk_put(alg_data->clk);
out_drvdata:
	kfree(alg_data);
err_kzalloc:
	platform_set_drvdata(pdev, NULL);
out:
	return ret;
}

static int __devexit i2c_pnx_remove(struct platform_device *pdev)
{
	struct i2c_pnx_algo_data *alg_data = platform_get_drvdata(pdev);
	struct i2c_pnx_data *i2c_pnx = alg_data->i2c_pnx;

	free_irq(i2c_pnx->irq, alg_data);
	i2c_del_adapter(&alg_data->adapter);
	clk_disable(alg_data->clk);
	iounmap(alg_data->ioaddr);
	release_mem_region(i2c_pnx->base, I2C_PNX_REGION_SIZE);
	clk_put(alg_data->clk);
	kfree(alg_data);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver i2c_pnx_driver = {
	.driver = {
		.name = "pnx-i2c",
		.owner = THIS_MODULE,
	},
	.probe = i2c_pnx_probe,
	.remove = __devexit_p(i2c_pnx_remove),
	.suspend = i2c_pnx_controller_suspend,
	.resume = i2c_pnx_controller_resume,
};

static int __init i2c_adap_pnx_init(void)
{
	return platform_driver_register(&i2c_pnx_driver);
}

static void __exit i2c_adap_pnx_exit(void)
{
	platform_driver_unregister(&i2c_pnx_driver);
}

MODULE_AUTHOR("Vitaly Wool, Dennis Kovalev <source@mvista.com>");
MODULE_DESCRIPTION("I2C driver for Philips IP3204-based I2C busses");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pnx-i2c");

/* We need to make sure I2C is initialized before USB */
subsys_initcall(i2c_adap_pnx_init);
module_exit(i2c_adap_pnx_exit);
