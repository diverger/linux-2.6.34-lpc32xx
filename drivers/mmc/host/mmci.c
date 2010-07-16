/*
 *  linux/drivers/mmc/host/mmci.c - ARM PrimeCell MMCI PL180/1 driver
 *
 *  Copyright (C) 2003 Deep Blue Solutions, Ltd, All Rights Reserved.
 *  Copyright (C) 2010 ST-Ericsson AB.
 *  Copyright (C) 2010 NXP Semiconductors (LPC32xx DMA modifications)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/log2.h>
#include <linux/mmc/host.h>
#include <linux/amba/bus.h>
#include <linux/clk.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/amba/mmci.h>
#include <linux/regulator/consumer.h>

#include <asm/cacheflush.h>
#include <asm/div64.h>
#include <asm/io.h>
#include <asm/sizes.h>

#include <mach/clkdev.h>
#include <mach/dmac.h>
#include <mach/sdcard.h>
#include <mach/dma.h>
#include <mach/hardware.h>

#include "mmci.h"

#define DRIVER_NAME "mmci-pl18x"

static unsigned int fmax = 26000000; /* 26MHz bit rate max */

#define DMA_BUFF_SIZE SZ_64K

struct LPC32XX_SDDRV_DATA {
	struct dma_config dmacfgtx;
	struct dma_config dmacfgrx;
	struct device *dev;
	int lastch;
	dma_addr_t dma_handle_tx;
	void *dma_v_base;
	int mapped;
};
static struct LPC32XX_SDDRV_DATA lpc32xx_drvdat;

#define MCI_WIDEBUS (1 << 11)
#undef MCI_IRQENABLE
#define MCI_IRQENABLE   \
        (MCI_CMDCRCFAILMASK|MCI_DATACRCFAILMASK|MCI_CMDTIMEOUTMASK|     \
        MCI_DATATIMEOUTMASK|MCI_TXUNDERRUNMASK|MCI_RXOVERRUNMASK|       \
        MCI_CMDRESPENDMASK|MCI_CMDSENTMASK)

static int mmc_dma_setup(void)
{
	u32 llptrrx, llptrtx;
	int ret = 0;

	/*
	 * There is a quirk with the LPC32XX and SD burst DMA. DMA sg
	 * transfers where DMA is the flow controller will not transfer
	 * the last few bytes to or from the SD card controller and
	 * memory. For RX, the last few bytes in the SD transfer can be
	 * forced out with a software DMA burst request. For TX, this
	 * can't be done, so TX sg support cannot be supported. For TX,
	 * a temporary bouncing buffer is used if more than 1 sg segment
	 * is passed in the data request. The bouncing buffer will get a
	 * contiguous copy of the TX data and it will be used instead.
	 */

        /* Allocate a chunk of memory for the DMA TX buffers */
        lpc32xx_drvdat.dma_v_base = dma_alloc_coherent(lpc32xx_drvdat.dev,
		DMA_BUFF_SIZE, &lpc32xx_drvdat.dma_handle_tx, GFP_KERNEL);
        if (lpc32xx_drvdat.dma_v_base == NULL) {
                dev_err(lpc32xx_drvdat.dev, "error getting DMA region\n");
                ret = -ENOMEM;
                goto dma_no_tx_buff;
        }
        dev_info(lpc32xx_drvdat.dev, "DMA buffer: phy:%p, virt:%p\n",
                (void *) lpc32xx_drvdat.dma_handle_tx, lpc32xx_drvdat.dma_v_base);

        /* Setup TX DMA channel */
        lpc32xx_drvdat.dmacfgtx.ch = DMA_CH_SDCARD_TX;
        lpc32xx_drvdat.dmacfgtx.tc_inten = 0;
        lpc32xx_drvdat.dmacfgtx.err_inten = 0;
        lpc32xx_drvdat.dmacfgtx.src_size = 4;
        lpc32xx_drvdat.dmacfgtx.src_inc = 1;
        lpc32xx_drvdat.dmacfgtx.src_ahb1 = 0;
        lpc32xx_drvdat.dmacfgtx.src_bsize = DMAC_CHAN_SRC_BURST_8;
        lpc32xx_drvdat.dmacfgtx.src_prph = DMAC_SRC_PERIP(DMA_PERID_SDCARD);
        lpc32xx_drvdat.dmacfgtx.dst_size = 4;
        lpc32xx_drvdat.dmacfgtx.dst_inc = 0;
        lpc32xx_drvdat.dmacfgtx.dst_ahb1 = 0;
        lpc32xx_drvdat.dmacfgtx.dst_bsize = DMAC_CHAN_DEST_BURST_8;
        lpc32xx_drvdat.dmacfgtx.dst_prph = DMAC_DEST_PERIP(DMA_PERID_SDCARD);
        lpc32xx_drvdat.dmacfgtx.flowctrl = DMAC_CHAN_FLOW_P_M2P;
        if (lpc32xx_dma_ch_get(&lpc32xx_drvdat.dmacfgtx, "dma_sd_tx", NULL, NULL) < 0)
        {
                dev_err(lpc32xx_drvdat.dev, "Error setting up SD card TX DMA channel\n");
                ret = -ENODEV;
		goto dma_no_txch;
        }

	/* Allocate a linked list for DMA support */
	llptrtx = lpc32xx_dma_alloc_llist(lpc32xx_drvdat.dmacfgtx.ch, NR_SG * 2);
	if (llptrtx == 0) {
		dev_err(lpc32xx_drvdat.dev, "Error allocating list buffer (MMC TX)\n");
		ret = -ENOMEM;
		goto dma_no_txlist;
	}

        /* Setup RX DMA channel */
        lpc32xx_drvdat.dmacfgrx.ch = DMA_CH_SDCARD_RX;
        lpc32xx_drvdat.dmacfgrx.tc_inten = 0;
        lpc32xx_drvdat.dmacfgrx.err_inten = 0;
        lpc32xx_drvdat.dmacfgrx.src_size = 4;
        lpc32xx_drvdat.dmacfgrx.src_inc = 0;
        lpc32xx_drvdat.dmacfgrx.src_ahb1 = 0;
        lpc32xx_drvdat.dmacfgrx.src_bsize = DMAC_CHAN_SRC_BURST_8;
        lpc32xx_drvdat.dmacfgrx.src_prph = DMAC_SRC_PERIP(DMA_PERID_SDCARD);
        lpc32xx_drvdat.dmacfgrx.dst_size = 4;
        lpc32xx_drvdat.dmacfgrx.dst_inc = 1;
        lpc32xx_drvdat.dmacfgrx.dst_ahb1 = 0;
        lpc32xx_drvdat.dmacfgrx.dst_bsize = DMAC_CHAN_DEST_BURST_8;
        lpc32xx_drvdat.dmacfgrx.dst_prph = DMAC_DEST_PERIP(DMA_PERID_SDCARD);
        lpc32xx_drvdat.dmacfgrx.flowctrl = DMAC_CHAN_FLOW_D_P2M;
        if (lpc32xx_dma_ch_get(&lpc32xx_drvdat.dmacfgrx, "dma_sd_rx", NULL, NULL) < 0)
        {
                dev_err(lpc32xx_drvdat.dev, "Error setting up SD card RX DMA channel\n");
                ret = -ENODEV;
		goto dma_no_rxch;
        }

	/* Allocate a linked list for DMA support */
	llptrrx = lpc32xx_dma_alloc_llist(lpc32xx_drvdat.dmacfgrx.ch, NR_SG * 2);
	if (llptrrx == 0) {
		dev_err(lpc32xx_drvdat.dev, "Error allocating list buffer (MMC RX)\n");
		ret = -ENOMEM;
		goto dma_no_rxlist;
	}

	return 0;

dma_no_rxlist:
	lpc32xx_dma_ch_put(lpc32xx_drvdat.dmacfgrx.ch);
	lpc32xx_drvdat.dmacfgrx.ch = -1;
dma_no_rxch:
	lpc32xx_dma_dealloc_llist(lpc32xx_drvdat.dmacfgtx.ch);
dma_no_txlist:
	lpc32xx_dma_ch_put(lpc32xx_drvdat.dmacfgtx.ch);
	lpc32xx_drvdat.dmacfgtx.ch = -1;
dma_no_txch:
        dma_free_coherent(lpc32xx_drvdat.dev, DMA_BUFF_SIZE,
                lpc32xx_drvdat.dma_v_base, lpc32xx_drvdat.dma_handle_tx);
dma_no_tx_buff:
        return ret;
}

static void mmc_dma_dealloc(void)
{
	lpc32xx_dma_dealloc_llist(lpc32xx_drvdat.dmacfgrx.ch);
	lpc32xx_dma_ch_put(lpc32xx_drvdat.dmacfgrx.ch);
	lpc32xx_drvdat.dmacfgrx.ch = -1;
	lpc32xx_dma_dealloc_llist(lpc32xx_drvdat.dmacfgtx.ch);
	lpc32xx_dma_ch_put(lpc32xx_drvdat.dmacfgtx.ch);
	lpc32xx_drvdat.dmacfgtx.ch = -1;
        dma_free_coherent(lpc32xx_drvdat.dev, DMA_BUFF_SIZE,
                lpc32xx_drvdat.dma_v_base, lpc32xx_drvdat.dma_handle_tx);
}

/* Supports scatter/gather */
static void mmc_dma_rx_start(struct mmci_host *host)
{
	unsigned int len;
	int i, dma_len;
	struct scatterlist *sg;
	struct mmc_request *mrq = host->mrq;
	struct mmc_data *reqdata = mrq->data;
	void *dmaaddr;
	u32 dmalen, dmaxferlen;

	sg = reqdata->sg;
	len = reqdata->sg_len;

	dma_len = dma_map_sg(mmc_dev(host->mmc), reqdata->sg, reqdata->sg_len, DMA_FROM_DEVICE);
	if (dma_len == 0)
		return;

	/* Setup transfer */
        for (i = 0; i < len; i++) {
		dmalen = (u32) sg_dma_len(&sg[i]);
		dmaaddr = (void *) sg_dma_address(&sg[i]);

		/* Build a list with a max size if 15872 bytes per seg */
		while (dmalen > 0) {
			dmaxferlen = dmalen;
			if (dmaxferlen > 15872)
				dmaxferlen = 15872;

			lpc32xx_dma_queue_llist_entry(lpc32xx_drvdat.lastch,
				(void *) SD_FIFO(LPC32XX_SD_BASE),
				dmaaddr, dmaxferlen);

				dmaaddr += dmaxferlen;
				dmalen -= dmaxferlen;
		}
	}

//printk("DMARX %d\n", len);

}

/* May need to reorganize buffer for scatter/gather */
static void mmc_dma_tx_start(struct mmci_host *host)
{
	unsigned int len;
	int dma_len;
	struct scatterlist *sg;
	struct mmc_request *mrq = host->mrq;
	struct mmc_data *reqdata = mrq->data;
	void *dmaaddr;
        char *src_buffer, *dst_buffer;
        unsigned long flags;

	sg = reqdata->sg;
	len = reqdata->sg_len;

	/* Only 1 segment? */
	if (len == 1) {
		dma_len = dma_map_sg(mmc_dev(host->mmc), reqdata->sg,
			reqdata->sg_len, DMA_TO_DEVICE);
		if (dma_len == 0)
			return;

		dmaaddr = (void *) sg_dma_address(&sg[0]);
		lpc32xx_drvdat.mapped = 1;
	}
	else {
		/* Move data to contiguous buffer first, then transfer it */
	        dst_buffer = (char *) lpc32xx_drvdat.dma_v_base;
	        do
	        {
	                /*
			 * Map the current scatter buffer, copy data, and unmap
			 */
        	        src_buffer = mmci_kmap_atomic(host, &flags) + host->sg_off;
        	        memcpy(dst_buffer, src_buffer, host->sg_ptr->length);
        	        dst_buffer += host->sg_ptr->length;
        	        mmci_kunmap_atomic(host, src_buffer, &flags);

        	        if (!mmci_next_sg(host))
        	                break;
        	} while (1);

		lpc32xx_drvdat.mapped = 0;
		dmaaddr = (void *) lpc32xx_drvdat.dma_handle_tx;
	}

	lpc32xx_dma_start_pflow_xfer(DMA_CH_SDCARD_TX, dmaaddr,
		(void *) SD_FIFO(LPC32XX_SD_BASE), 1);
}

/*
 * This must be called with host->lock held
 */
static void mmci_set_clkreg(struct mmci_host *host, unsigned int desired)
{
	u32 clk = 0;

	if (desired) {
		if (desired >= host->mclk) {
			clk = MCI_CLK_BYPASS;
			host->cclk = host->mclk;
		} else {
			clk = host->mclk / (2 * desired) - 1;
			if (clk >= 256)
				clk = 255;
			host->cclk = host->mclk / (2 * (clk + 1));
		}
		if (host->hw_designer == AMBA_VENDOR_ST)
			clk |= MCI_FCEN; /* Bug fix in ST IP block */
		clk |= MCI_CLK_ENABLE;
		/* This hasn't proven to be worthwhile */
		clk |= MCI_CLK_PWRSAVE;
	}

	if (host->mmc->ios.bus_width == MMC_BUS_WIDTH_4)
		clk |= MCI_WIDE_BUS;

	writel(clk, host->base + MMCICLOCK);
}

static void
mmci_request_end(struct mmci_host *host, struct mmc_request *mrq)
{
	writel(0, host->base + MMCICOMMAND);

	BUG_ON(host->data);

	host->mrq = NULL;
	host->cmd = NULL;

	if (mrq->data)
		mrq->data->bytes_xfered = host->data_xfered;

	/*
	 * Need to drop the host lock here; mmc_request_done may call
	 * back into the driver...
	 */
	spin_unlock(&host->lock);
	mmc_request_done(host->mmc, mrq);
	spin_lock(&host->lock);
}

static void mmci_stop_data(struct mmci_host *host)
{
	writel(0, host->base + MMCIDATACTRL);
	writel(0, host->base + MMCIMASK1);
	host->data = NULL;
}

static void mmci_start_data(struct mmci_host *host, struct mmc_data *data)
{
	unsigned int datactrl, timeout, irqmask = 0;
	unsigned long long clks;
	void __iomem *base;
	int blksz_bits;

	dev_dbg(mmc_dev(host->mmc), "blksz %04x blks %04x flags %08x\n",
		data->blksz, data->blocks, data->flags);

	host->data = data;
	host->size = data->blksz;
	host->data_xfered = 0;

	mmci_init_sg(host, data);

	clks = (unsigned long long)data->timeout_ns * host->cclk;
	do_div(clks, 1000000000UL);

	timeout = data->timeout_clks + (unsigned int)clks;

	base = host->base;
	writel(timeout, base + MMCIDATATIMER);
        writel((host->size * data->blocks), base + MMCIDATALENGTH);
        blksz_bits = ffs(data->blksz) - 1;
        BUG_ON(1 << blksz_bits != data->blksz);

        datactrl = MCI_DPSM_ENABLE | MCI_DPSM_DMAENABLE | blksz_bits << 4;

        if (data->flags & MMC_DATA_READ) {
                datactrl |= MCI_DPSM_DIRECTION;
                lpc32xx_drvdat.lastch = DMA_CH_SDCARD_RX;
		mmc_dma_rx_start(host);
        }
	else {
                lpc32xx_drvdat.lastch = DMA_CH_SDCARD_TX;
		mmc_dma_tx_start(host);
	}

        writel(datactrl, base + MMCIDATACTRL);
        datactrl = readl(base + MMCIMASK0) & ~MCI_DATABLOCKENDMASK;
        writel(datactrl | MCI_DATAENDMASK, base + MMCIMASK0);

	writel(irqmask, base + MMCIMASK1);
}

static void
mmci_start_command(struct mmci_host *host, struct mmc_command *cmd, u32 c)
{
	void __iomem *base = host->base;

	dev_dbg(mmc_dev(host->mmc), "op %02x arg %08x flags %08x\n",
	    cmd->opcode, cmd->arg, cmd->flags);

	if (readl(base + MMCICOMMAND) & MCI_CPSM_ENABLE) {
		writel(0, base + MMCICOMMAND);
		udelay(1);
	}

	c |= cmd->opcode | MCI_CPSM_ENABLE;
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			c |= MCI_CPSM_LONGRSP;
		c |= MCI_CPSM_RESPONSE;
	}
	if (/*interrupt*/0)
		c |= MCI_CPSM_INTERRUPT;

	host->cmd = cmd;

	writel(cmd->arg, base + MMCIARGUMENT);
	writel(c, base + MMCICOMMAND);
}

static void
mmci_data_irq(struct mmci_host *host, struct mmc_data *data,
	      unsigned int status)
{
        if (status & MCI_DATAEND) {
                host->data_xfered += data->blksz * data->blocks;

#ifdef CONFIG_ARCH_U300
		/*
		 * On the U300 some signal or other is
		 * badly routed so that a data write does
		 * not properly terminate with a MCI_DATAEND
		 * status flag. This quirk will make writes
		 * work again.
		 */
		if (data->flags & MMC_DATA_WRITE)
			status |= MCI_DATAEND;
#endif
	}
	if (status & (MCI_DATACRCFAIL|MCI_DATATIMEOUT|MCI_TXUNDERRUN|MCI_RXOVERRUN)) {
		dev_dbg(mmc_dev(host->mmc), "MCI ERROR IRQ (status %08x)\n", status);
		if (status & MCI_DATACRCFAIL)
			data->error = -EILSEQ;
		else if (status & MCI_DATATIMEOUT)
			data->error = -ETIMEDOUT;
		else if (status & (MCI_TXUNDERRUN|MCI_RXOVERRUN))
			data->error = -EIO;
		status |= MCI_DATAEND;

		/*
		 * We hit an error condition.  Ensure that any data
		 * partially written to a page is properly coherent.
		 */
		if (host->sg_len && data->flags & MMC_DATA_READ)
			flush_dcache_page(sg_page(host->sg_ptr));
	}

	if (status & MCI_DATAEND) {
		if (data->flags & MMC_DATA_READ) {
			lpc32xx_dma_force_burst(lpc32xx_drvdat.lastch, DMA_PERID_SDCARD);
			lpc32xx_dma_flush_llist(lpc32xx_drvdat.lastch);
			dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len, DMA_FROM_DEVICE);
		}
		else {
			lpc32xx_dma_ch_disable(lpc32xx_drvdat.lastch);
			if (lpc32xx_drvdat.mapped)
				dma_unmap_sg(mmc_dev(host->mmc), data->sg,
					data->sg_len, DMA_TO_DEVICE);
		}

		mmci_stop_data(host);

		if (!data->stop) {
			mmci_request_end(host, data->mrq);
		} else {
			mmci_start_command(host, data->stop, 0);
		}
	}
}

static void
mmci_cmd_irq(struct mmci_host *host, struct mmc_command *cmd,
	     unsigned int status)
{
	void __iomem *base = host->base;

	host->cmd = NULL;

	cmd->resp[0] = readl(base + MMCIRESPONSE0);
	cmd->resp[1] = readl(base + MMCIRESPONSE1);
	cmd->resp[2] = readl(base + MMCIRESPONSE2);
	cmd->resp[3] = readl(base + MMCIRESPONSE3);

	if (status & MCI_CMDTIMEOUT) {
		cmd->error = -ETIMEDOUT;
	} else if (status & MCI_CMDCRCFAIL && cmd->flags & MMC_RSP_CRC) {
		cmd->error = -EILSEQ;
	}

	if (!cmd->data || cmd->error) {
		if (host->data)
			mmci_stop_data(host);
		mmci_request_end(host, cmd->mrq);
	} else if (!(cmd->data->flags & MMC_DATA_READ))
		mmci_start_data(host, cmd->data);
}

static int mmci_pio_read(struct mmci_host *host, char *buffer, unsigned int remain)
{
	void __iomem *base = host->base;
	char *ptr = buffer;
	u32 status;
	int host_remain = host->size;

	do {
		int count = host_remain - (readl(base + MMCIFIFOCNT) << 2);

		if (count > remain)
			count = remain;

		if (count <= 0)
			break;

		readsl(base + MMCIFIFO, ptr, count >> 2);

		ptr += count;
		remain -= count;
		host_remain -= count;

		if (remain == 0)
			break;

		status = readl(base + MMCISTATUS);
	} while (status & MCI_RXDATAAVLBL);

	return ptr - buffer;
}

static int mmci_pio_write(struct mmci_host *host, char *buffer, unsigned int remain, u32 status)
{
	void __iomem *base = host->base;
	char *ptr = buffer;

	do {
		unsigned int count, maxcnt;

		maxcnt = status & MCI_TXFIFOEMPTY ? MCI_FIFOSIZE : MCI_FIFOHALFSIZE;
		count = min(remain, maxcnt);

		writesl(base + MMCIFIFO, ptr, count >> 2);

		ptr += count;
		remain -= count;

		if (remain == 0)
			break;

		status = readl(base + MMCISTATUS);
	} while (status & MCI_TXFIFOHALFEMPTY);

	return ptr - buffer;
}

/*
 * PIO data transfer IRQ handler.
 */
static irqreturn_t mmci_pio_irq(int irq, void *dev_id)
{
	struct mmci_host *host = dev_id;
	void __iomem *base = host->base;
	u32 status;

	status = readl(base + MMCISTATUS);

	dev_dbg(mmc_dev(host->mmc), "irq1 (pio) %08x\n", status);

	do {
		unsigned long flags;
		unsigned int remain, len;
		char *buffer;

		/*
		 * For write, we only need to test the half-empty flag
		 * here - if the FIFO is completely empty, then by
		 * definition it is more than half empty.
		 *
		 * For read, check for data available.
		 */
		if (!(status & (MCI_TXFIFOHALFEMPTY|MCI_RXDATAAVLBL)))
			break;

		/*
		 * Map the current scatter buffer.
		 */
		buffer = mmci_kmap_atomic(host, &flags) + host->sg_off;
		remain = host->sg_ptr->length - host->sg_off;

		len = 0;
		if (status & MCI_RXACTIVE)
			len = mmci_pio_read(host, buffer, remain);
		if (status & MCI_TXACTIVE)
			len = mmci_pio_write(host, buffer, remain, status);

		/*
		 * Unmap the buffer.
		 */
		mmci_kunmap_atomic(host, buffer, &flags);

		host->sg_off += len;
		host->size -= len;
		remain -= len;

		if (remain)
			break;

		/*
		 * If we were reading, and we have completed this
		 * page, ensure that the data cache is coherent.
		 */
		if (status & MCI_RXACTIVE)
			flush_dcache_page(sg_page(host->sg_ptr));

		if (!mmci_next_sg(host))
			break;

		status = readl(base + MMCISTATUS);
	} while (1);

	/*
	 * If we're nearing the end of the read, switch to
	 * "any data available" mode.
	 */
	if (status & MCI_RXACTIVE && host->size < MCI_FIFOSIZE)
		writel(MCI_RXDATAAVLBLMASK, base + MMCIMASK1);

	/*
	 * If we run out of data, disable the data IRQs; this
	 * prevents a race where the FIFO becomes empty before
	 * the chip itself has disabled the data path, and
	 * stops us racing with our data end IRQ.
	 */
	if (host->size == 0) {
		writel(0, base + MMCIMASK1);
		writel(readl(base + MMCIMASK0) | MCI_DATAENDMASK, base + MMCIMASK0);
	}

	return IRQ_HANDLED;
}

/*
 * Handle completion of command and data transfers.
 */
static irqreturn_t mmci_irq(int irq, void *dev_id)
{
	struct mmci_host *host = dev_id;
	u32 status;
	int ret = 0;

	spin_lock(&host->lock);

	do {
		struct mmc_command *cmd;
		struct mmc_data *data;

		status = readl(host->base + MMCISTATUS);
		status &= readl(host->base + MMCIMASK0);
                writel((status | MCI_DATABLOCKEND), host->base + MMCICLEAR);

		dev_dbg(mmc_dev(host->mmc), "irq0 (data+cmd) %08x\n", status);

		data = host->data;
		if (status & (MCI_DATACRCFAIL|MCI_DATATIMEOUT|MCI_TXUNDERRUN|
			      MCI_RXOVERRUN|MCI_DATAEND|MCI_DATABLOCKEND) && data)
			mmci_data_irq(host, data, status);

		cmd = host->cmd;
		if (status & (MCI_CMDCRCFAIL|MCI_CMDTIMEOUT|MCI_CMDSENT|MCI_CMDRESPEND) && cmd)
			mmci_cmd_irq(host, cmd, status);

		ret = 1;
	} while (status);

	spin_unlock(&host->lock);

	return IRQ_RETVAL(ret);
}

static void mmci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct mmci_host *host = mmc_priv(mmc);
	unsigned long flags;

	WARN_ON(host->mrq != NULL);

	if (mrq->data && !is_power_of_2(mrq->data->blksz)) {
		dev_err(mmc_dev(mmc), "unsupported block size (%d bytes)\n",
			mrq->data->blksz);
		mrq->cmd->error = -EINVAL;
		mmc_request_done(mmc, mrq);
		return;
	}

	spin_lock_irqsave(&host->lock, flags);

	host->mrq = mrq;

	if (mrq->data && mrq->data->flags & MMC_DATA_READ)
		mmci_start_data(host, mrq->data);

	mmci_start_command(host, mrq->cmd, 0);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void mmci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct mmci_host *host = mmc_priv(mmc);
	u32 pwr = 0;
	unsigned long flags;

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		if(host->vcc &&
		   regulator_is_enabled(host->vcc))
			regulator_disable(host->vcc);
		break;
	case MMC_POWER_UP:
#ifdef CONFIG_REGULATOR
		if (host->vcc)
			/* This implicitly enables the regulator */
			mmc_regulator_set_ocr(host->vcc, ios->vdd);
#endif
		/*
		 * The translate_vdd function is not used if you have
		 * an external regulator, or your design is really weird.
		 * Using it would mean sending in power control BOTH using
		 * a regulator AND the 4 MMCIPWR bits. If we don't have
		 * a regulator, we might have some other platform specific
		 * power control behind this translate function.
		 */
		if (!host->vcc && host->plat->translate_vdd)
			pwr |= host->plat->translate_vdd(mmc_dev(mmc), ios->vdd);
		/* The ST version does not have this, fall through to POWER_ON */
		if (host->hw_designer != AMBA_VENDOR_ST) {
			pwr |= MCI_PWR_UP;
			break;
		}
	case MMC_POWER_ON:
		pwr |= MCI_PWR_ON;
		break;
	}

	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN) {
		if (host->hw_designer != AMBA_VENDOR_ST)
			pwr |= MCI_ROD;
		else {
			/*
			 * The ST Micro variant use the ROD bit for something
			 * else and only has OD (Open Drain).
			 */
			pwr |= MCI_OD;
		}
	}

	spin_lock_irqsave(&host->lock, flags);

	mmci_set_clkreg(host, ios->clock);

	if (host->pwr != pwr) {
		host->pwr = pwr;
		writel(pwr, host->base + MMCIPOWER);
	}

	spin_unlock_irqrestore(&host->lock, flags);
}

static int mmci_get_ro(struct mmc_host *mmc)
{
	struct mmci_host *host = mmc_priv(mmc);

	if (host->gpio_wp == -ENOSYS)
		return -ENOSYS;

	return gpio_get_value(host->gpio_wp);
}

static int mmci_get_cd(struct mmc_host *mmc)
{
	struct mmci_host *host = mmc_priv(mmc);
	unsigned int status;

	if (host->gpio_cd == -ENOSYS)
		status = host->plat->status(mmc_dev(host->mmc));
	else
		status = gpio_get_value(host->gpio_cd);

	return !status;
}

static const struct mmc_host_ops mmci_ops = {
	.request	= mmci_request,
	.set_ios	= mmci_set_ios,
	.get_ro		= mmci_get_ro,
	.get_cd		= mmci_get_cd,
};

static void mmci_check_status(unsigned long data)
{
	struct mmci_host *host = (struct mmci_host *)data;
	unsigned int status = mmci_get_cd(host->mmc);

	if (status ^ host->oldstat)
		mmc_detect_change(host->mmc, 0);

	host->oldstat = status;
	mod_timer(&host->timer, jiffies + HZ);
}

static int __devinit mmci_probe(struct amba_device *dev, struct amba_id *id)
{
	struct mmci_platform_data *plat = dev->dev.platform_data;
	struct mmci_host *host;
	struct mmc_host *mmc;
	int ret;

	/* must have platform data */
	if (!plat) {
		ret = -EINVAL;
		goto out;
	}

	ret = amba_request_regions(dev, DRIVER_NAME);
	if (ret)
                goto out;
	mmc = mmc_alloc_host(sizeof(struct mmci_host), &dev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto rel_regions;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;

	host->gpio_wp = -ENOSYS;
	host->gpio_cd = -ENOSYS;

	host->hw_designer = amba_manf(dev);
	host->hw_revision = amba_rev(dev);
	dev_dbg(mmc_dev(mmc), "designer ID = 0x%02x\n", host->hw_designer);
	dev_dbg(mmc_dev(mmc), "revision = 0x%01x\n", host->hw_revision);

	host->clk = clk_get(&dev->dev, NULL);
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		host->clk = NULL;
		goto host_free;
	}

	ret = clk_enable(host->clk);
	if (ret)
		goto clk_free;

	host->plat = plat;
	host->mclk = clk_get_rate(host->clk);
	/*
	 * According to the spec, mclk is max 100 MHz,
	 * so we try to adjust the clock down to this,
	 * (if possible).
	 */
	if (host->mclk > 100000000) {
		ret = clk_set_rate(host->clk, 100000000);
		if (ret < 0)
			goto clk_disable;
		host->mclk = clk_get_rate(host->clk);
		dev_dbg(mmc_dev(mmc), "eventual mclk rate: %u Hz\n",
			host->mclk);
	}
	host->base = ioremap(dev->res.start, resource_size(&dev->res));
	if (!host->base) {
		ret = -ENOMEM;
		goto clk_disable;
	}

	mmc->ops = &mmci_ops;
	mmc->f_min = (host->mclk + 511) / 512;
	mmc->f_max = min(host->mclk, fmax);
	dev_dbg(mmc_dev(mmc), "clocking block at %u Hz\n", mmc->f_max);

#ifdef CONFIG_REGULATOR
	/* If we're using the regulator framework, try to fetch a regulator */
	host->vcc = regulator_get(&dev->dev, "vmmc");
	if (IS_ERR(host->vcc))
		host->vcc = NULL;
	else {
		int mask = mmc_regulator_get_ocrmask(host->vcc);

		if (mask < 0)
			dev_err(&dev->dev, "error getting OCR mask (%d)\n",
				mask);
		else {
			host->mmc->ocr_avail = (u32) mask;
			if (plat->ocr_mask)
				dev_warn(&dev->dev,
				 "Provided ocr_mask/setpower will not be used "
				 "(using regulator instead)\n");
		}
	}
#endif
	/* Fall back to platform data if no regulator is found */
	if (host->vcc == NULL)
		mmc->ocr_avail = plat->ocr_mask;
	mmc->caps = plat->capabilities;

	/*
	 * We can do SGIO
	 */
	mmc->max_hw_segs = 16;
	mmc->max_phys_segs = NR_SG;

	/*
	 * Since we only have a 16-bit data length register, we must
	 * ensure that we don't exceed 2^16-1 bytes in a single request.
	 */
	mmc->max_req_size = 65535;

	/*
	 * Set the maximum segment size.  Since we aren't doing DMA
	 * (yet) we are only limited by the data length register.
	 */
	/*
	 * The LPC32x0 DMA controller can handle up to a 16383 byte DMA
	 * transfer. We'll rely on the mmc core to make sure the passed
	 * size for a request is block aligned.
	 */
	mmc->max_seg_size = 65535;

	/*
	 * Block size can be up to 2048 bytes, but must be a power of two.
	 */
	mmc->max_blk_size = 2048;

	/*
	 * No limit on the number of blocks transferred.
	 */
	mmc->max_blk_count = mmc->max_req_size;

        /*
	 * Setup DMA for the interface
	 */
	lpc32xx_drvdat.dev = &dev->dev;
        if (mmc_dma_setup())
		goto err_dma_setup;

	spin_lock_init(&host->lock);

	writel(0, host->base + MMCIMASK0);
	writel(0, host->base + MMCIMASK1);
	writel(0xfff, host->base + MMCICLEAR);

	if (gpio_is_valid(plat->gpio_cd)) {
		ret = gpio_request(plat->gpio_cd, DRIVER_NAME " (cd)");
		if (ret == 0)
			ret = gpio_direction_input(plat->gpio_cd);
		if (ret == 0)
			host->gpio_cd = plat->gpio_cd;
		else if (ret != -ENOSYS)
			goto err_gpio_cd;
	}
	if (gpio_is_valid(plat->gpio_wp)) {
		ret = gpio_request(plat->gpio_wp, DRIVER_NAME " (wp)");
		if (ret == 0)
			ret = gpio_direction_input(plat->gpio_wp);
		if (ret == 0)
			host->gpio_wp = plat->gpio_wp;
		else if (ret != -ENOSYS)
			goto err_gpio_wp;
	}

	ret = request_irq(dev->irq[0], mmci_irq, IRQF_SHARED, DRIVER_NAME " (cmd)", host);
	if (ret)
		goto unmap;

	ret = request_irq(dev->irq[1], mmci_pio_irq, IRQF_SHARED, DRIVER_NAME " (pio)", host);
	if (ret)
		goto irq0_free;
	writel(MCI_IRQENABLE, host->base + MMCIMASK0);
	amba_set_drvdata(dev, mmc);
	host->oldstat = mmci_get_cd(host->mmc);

	mmc_add_host(mmc);

	dev_info(&dev->dev, "%s: MMCI rev %x cfg %02x at 0x%016llx irq %d,%d\n",
		mmc_hostname(mmc), amba_rev(dev), amba_config(dev),
		(unsigned long long)dev->res.start, dev->irq[0], dev->irq[1]);

	init_timer(&host->timer);
	host->timer.data = (unsigned long)host;
	host->timer.function = mmci_check_status;
	host->timer.expires = jiffies + HZ;
	add_timer(&host->timer);

	return 0;

 irq0_free:
	free_irq(dev->irq[0], host);
 unmap:
	if (host->gpio_wp != -ENOSYS)
		gpio_free(host->gpio_wp);
 err_gpio_wp:
	if (host->gpio_cd != -ENOSYS)
		gpio_free(host->gpio_cd);
 err_gpio_cd:
	mmc_dma_dealloc();
 err_dma_setup:
	iounmap(host->base);
 clk_disable:
	clk_disable(host->clk);
 clk_free:
	clk_put(host->clk);
 host_free:
	mmc_free_host(mmc);
 rel_regions:
	amba_release_regions(dev);
 out:
	return ret;
}

static int __devexit mmci_remove(struct amba_device *dev)
{
	struct mmc_host *mmc = amba_get_drvdata(dev);

	amba_set_drvdata(dev, NULL);

	if (mmc) {
		struct mmci_host *host = mmc_priv(mmc);

		del_timer_sync(&host->timer);

		mmc_remove_host(mmc);

		writel(0, host->base + MMCIMASK0);
		writel(0, host->base + MMCIMASK1);

		writel(0, host->base + MMCICOMMAND);
		writel(0, host->base + MMCIDATACTRL);

		free_irq(dev->irq[0], host);
		free_irq(dev->irq[1], host);

		if (host->gpio_wp != -ENOSYS)
			gpio_free(host->gpio_wp);
		if (host->gpio_cd != -ENOSYS)
			gpio_free(host->gpio_cd);

		mmc_dma_dealloc();

		iounmap(host->base);
		clk_disable(host->clk);
		clk_put(host->clk);

		if (regulator_is_enabled(host->vcc))
			regulator_disable(host->vcc);
		regulator_put(host->vcc);

		mmc_free_host(mmc);

		amba_release_regions(dev);
	}

	return 0;
}

#ifdef CONFIG_PM
static int mmci_suspend(struct amba_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = amba_get_drvdata(dev);
	int ret = 0;

	if (mmc) {
		struct mmci_host *host = mmc_priv(mmc);

		ret = mmc_suspend_host(mmc, state);
		if (ret == 0)
			writel(0, host->base + MMCIMASK0);
	}

	return ret;
}

static int mmci_resume(struct amba_device *dev)
{
	struct mmc_host *mmc = amba_get_drvdata(dev);
	int ret = 0;

	if (mmc) {
		struct mmci_host *host = mmc_priv(mmc);

		writel(MCI_IRQENABLE, host->base + MMCIMASK0);

		ret = mmc_resume_host(mmc);
	}

	return ret;
}
#else
#define mmci_suspend	NULL
#define mmci_resume	NULL
#endif

static struct amba_id mmci_ids[] = {
	{
		.id	= 0x00041180,
		.mask	= 0x000fffff,
	},
	{
		.id	= 0x00041181,
		.mask	= 0x000fffff,
	},
	/* ST Micro variants */
	{
		.id     = 0x00180180,
		.mask   = 0x00ffffff,
	},
	{
		.id     = 0x00280180,
		.mask   = 0x00ffffff,
	},
	{ 0, 0 },
};

static struct amba_driver mmci_driver = {
	.drv		= {
		.name	= DRIVER_NAME,
	},
	.probe		= mmci_probe,
	.remove		= __devexit_p(mmci_remove),
	.suspend	= mmci_suspend,
	.resume		= mmci_resume,
	.id_table	= mmci_ids,
};

static int __init mmci_init(void)
{
	return amba_driver_register(&mmci_driver);
}

static void __exit mmci_exit(void)
{
	amba_driver_unregister(&mmci_driver);
}

module_init(mmci_init);
module_exit(mmci_exit);
module_param(fmax, uint, 0444);

MODULE_DESCRIPTION("ARM PrimeCell PL180/181 Multimedia Card Interface driver");
MODULE_LICENSE("GPL");
