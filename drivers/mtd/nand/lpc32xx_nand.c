/*
 * drivers/mtd/nand/lpc32xx_nand.c
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2011 NXP Semiconductors
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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <linux/io.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/nand_ecc.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/slcnand.h>
#include <mach/dmac.h>
#include <mach/dma.h>

#define LPC32XX_MODNAME			"lpc32xx-nand"

/*
 * DMA requires storage space for the DMA local buffer and the hardware ECC
 * storage area. The DMA local buffer is only used if DMA mapping fails
 * during runtime.
 */
#define LPC32XX_DMA_DATA_SIZE		4096
#define LPC32XX_ECC_SAVE_SIZE		((4096 / 256) * 4)

/* Number of bytes used for ECC stored in NAND per 256 bytes */
#define LPC32XX_SLC_DEV_ECC_BYTES	3

/*
 * 2 DMA descriptors are needed for every 256 byte data transfer, 1 descriptor
 * is for the data, and the other is for the saved ECC data. These are not
 * needed for OOB data.
 */
#define LPC32XX_MAX_DMA_DESCRIPTORS	(((4096 / 256) * 2) + 1)

/*
 * If the NAND base clock frequency can't be fetched, this frequency will be
 * used instead as the base. This rate is used to setup the timing registers
 * used for NAND accesses.
 */
#define LPC32XX_DEF_BUS_RATE		133250000

/* DMA transfer completion failure timeout */
#define LPC32XX_DMA_WAIT_TIMEOUT_MS	20

/*
 * This timeout is used for verifying the NAND buffer has commited it's
 * FIFO to memory or FLASH, or verifying the DMA transfer has completed.
 * The timeout is used as a count for simple polled checks of the hardware.
 * For most hardware, the actual timeouts are much lower than this, but
 * very slow hardware may use most of this time.
 */
#define LPC32XX_DMA_SIMPLE_TIMEOUT	10000

/*
 * This is the number of reads of the ECC register after a DMA write to
 * the NAND device. On writes, the DMA data is buffered in the NAND controller
 * prior to ECC calculation, so the DMA transfer 'completes' prior to the
 * NAND controller completing the transfer and ECC calculation for the write
 * data. Because of this, the initial reads of the ECC register by the DMA
 * controller may be incorrect as the data is still in transfer, so multiple
 * reads are needed. With very slow NAND devices, this count may need to be
 * increased. This doesn't apply to read operations.
 */
#define LPC32XX_DMA_ECC_REP_READ	10

/*
 * NAND ECC Layout for small page NAND devices
 * Note: For large and huge page devices, the default layouts are used
 */
static struct nand_ecclayout lpc32xx_nand_oob_16 = {
	.eccbytes = 6,
	.eccpos = {10, 11, 12, 13, 14, 15},
	.oobfree = {
	        {.offset = 0,
	         . length = 4},
	        {.offset = 6,
	         . length = 4}}
};

static uint8_t bbt_pattern[] = {'B', 'b', 't', '0' };
static uint8_t mirror_pattern[] = {'1', 't', 'b', 'B' };

/*
 * Small page FLASH BBT descriptors, marker at offset 0, version at offset 6
 * Note: Large page devices used the default layout
 */
static struct nand_bbt_descr bbt_smallpage_main_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs =	0,
	.len = 4,
	.veroffs = 6,
	.maxblocks = 4,
	.pattern = bbt_pattern
};

static struct nand_bbt_descr bbt_smallpage_mirror_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs =	0,
	.len = 4,
	.veroffs = 6,
	.maxblocks = 4,
	.pattern = mirror_pattern
};

struct lpc32xx_nand_host {
	struct nand_chip	nand_chip;
	struct clk		*clk;
	struct mtd_info		mtd;
	void __iomem		*io_base;
	struct lpc32XX_nand_cfg	*ncfg;

	wait_queue_head_t	dma_waitq;
	volatile u32		dmapending;
	struct dma_config	dmacfg;
	int			dmach;
	volatile uint32_t	dma_xfer_status;
	uint32_t		llptr;
	uint32_t		dma_buf_len;
	/*
	 * DMA and CPU addresses of ECC work area and data buffer
	 */
	dma_addr_t		ecc_buf_dma;
	uint32_t		*ecc_buf;
	dma_addr_t		data_buf_dma;
	uint8_t			*data_buf;
	dma_addr_t		io_base_dma;
};

static void lpc32xx_nand_setup(struct lpc32xx_nand_host *host)
{
	u32 clkrate, tmp;

	/* Reset SLC controller */
	__raw_writel(SLCCTRL_SW_RESET, SLC_CTRL(host->io_base));
	udelay(1000);

	/* Basic setup */
	__raw_writel(0, SLC_CFG(host->io_base));
	__raw_writel(0, SLC_IEN(host->io_base));
	__raw_writel((SLCSTAT_INT_TC | SLCSTAT_INT_RDY_EN),
		SLC_ICR(host->io_base));

	/* Get base clock for SLC block */
	clkrate = clk_get_rate(host->clk);
	if (clkrate == 0)
		clkrate = LPC32XX_DEF_BUS_RATE;

	/* Compute clock setup values */
	tmp = SLCTAC_WDR(host->ncfg->wdr_clks) |
		SLCTAC_WWIDTH(1 + (clkrate / host->ncfg->wwidth)) |
		SLCTAC_WHOLD(1 + (clkrate / host->ncfg->whold)) |
		SLCTAC_WSETUP(1 + (clkrate / host->ncfg->wsetup)) |
		SLCTAC_RDR(host->ncfg->rdr_clks) |
		SLCTAC_RWIDTH(1 + (clkrate / host->ncfg->rwidth)) |
		SLCTAC_RHOLD(1 + (clkrate / host->ncfg->rhold)) |
		SLCTAC_RSETUP(1 + (clkrate / host->ncfg->rsetup));
	__raw_writel(tmp, SLC_TAC(host->io_base));
}

/*
 * Hardware specific access to control lines
 */
static void lpc32xx_nand_cmd_ctrl(struct mtd_info *mtd, int cmd,
	unsigned int ctrl)
{
	u32 tmp;
	struct nand_chip *chip = mtd->priv;
	struct lpc32xx_nand_host *host = chip->priv;

	/* Does CE state need to be changed? */
	tmp = __raw_readl(SLC_CFG(host->io_base));
	if (ctrl & NAND_NCE)
		tmp |= SLCCFG_CE_LOW;
	else
		tmp &= ~SLCCFG_CE_LOW;
	__raw_writel(tmp, SLC_CFG(host->io_base));

	if (cmd != NAND_CMD_NONE) {
		if (ctrl & NAND_CLE)
			__raw_writel(cmd, SLC_CMD(host->io_base));
		else
			__raw_writel(cmd, SLC_ADDR(host->io_base));
	}
}

/*
 * Read the Device Ready pin
 */
static int lpc32xx_nand_device_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct lpc32xx_nand_host *host = chip->priv;
	int rdy = 0;

	if ((__raw_readl(SLC_STAT(host->io_base)) & SLCSTAT_NAND_READY) != 0)
		rdy = 1;

	return rdy;
}

/*
 * Enable NAND write protect
 */
static void lpc32xx_wp_enable(struct lpc32xx_nand_host *host)
{
	if (host->ncfg->enable_write_prot != NULL)
		/* Disable write protection */
		host->ncfg->enable_write_prot(1);
}

/*
 * Disable NAND write protect
 */
static void lpc32xx_wp_disable(struct lpc32xx_nand_host *host)
{
	if (host->ncfg->enable_write_prot != NULL)
		/* Enable write protection */
		host->ncfg->enable_write_prot(0);
}

/* 
 * Prepares SLC for transfers with H/W ECC enabled
 */
static void lpc32xx_nand_ecc_enable(struct mtd_info *mtd, int mode)
{
	(void)mtd;
	(void)mode;

	/* Hardware ECC is enabled automatically in hardware as needed */
}

/*
 * Calculates the ECC for the data
 */
static int lpc32xx_nand_ecc_calculate(struct mtd_info *mtd,
	const unsigned char *buf, unsigned char *code)
{
	(void) mtd;
	(void) buf;
	(void) code;

	/*
	 * ECC is calculated automatically in hardware during syndrome read
	 * and write operations, so it doesn't need to be calculated here.
	 */

	return 0;
}

/*
 * Read a single byte from NAND device
 */
static uint8_t lpc32xx_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct lpc32xx_nand_host *host = chip->priv;

	return (uint8_t) __raw_readl(SLC_DATA(host->io_base));
}

/*
 * Simple device read without ECC
 */
static void lpc32xx_nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct lpc32xx_nand_host *host = chip->priv;

	/* Direct device read with no ECC */
	while (len-- > 0)
		*buf++ = (uint8_t) __raw_readl(SLC_DATA(host->io_base));
}

/*
 * Simple device write without ECC
 */
static void lpc32xx_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf,
	int len)
{
	struct nand_chip *chip = mtd->priv;
	struct lpc32xx_nand_host *host = chip->priv;

	/* Direct device write with no ECC */
	while (len-- > 0)
		__raw_writel((u32) *buf++, SLC_DATA(host->io_base));
}

/*
 * Verify data in buffer to data on device
 */
static int lpc32xx_verify_buf(struct mtd_info *mtd, const uint8_t *buf, 
	int len)
{
	struct nand_chip *chip = mtd->priv;
	struct lpc32xx_nand_host *host = chip->priv;
	int i;

	/* DATA register must be read as 32 bits or it will fail */
	for (i = 0; i < len; i++) {
		if (buf[i] != (uint8_t) __raw_readl(SLC_DATA(host->io_base)))
			return -EFAULT;
	}

	return 0;
}

/*
 * Read the OOB data from the device without ECC using FIFO method
 */
static int lpc32xx_nand_read_oob_syndrome(struct mtd_info *mtd,
	struct nand_chip *chip, int page, int sndcmd)
{
	(void)page;

	if (sndcmd) {
		chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
		sndcmd = 0;
	}
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);

	return sndcmd;
}

/*
 * Write the OOB data to the device without ECC using FIFO method
 */
static int lpc32xx_nand_write_oob_syndrome(struct mtd_info *mtd,
	struct nand_chip *chip, int page)
{
	int status;

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, mtd->writesize, page);
	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);

	/* Send command to program the OOB data */
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

/*
 * Fills in the ECC fields in the OOB buffer with the hardware generated ECC
 */
static void lpc32xx_slc_ecc_copy(uint8_t *spare, const uint32_t *ecc, 
	int count)
{
	int i;

	for (i = 0; i < (count * 3); i += 3) {
		uint32_t ce = ecc[i / 3];
		ce = ~(ce << 2) & 0xFFFFFF;
		spare[i + 2] = (uint8_t)(ce & 0xFF);
		ce >>= 8;
		spare[i + 1] = (uint8_t)(ce & 0xFF);
		ce >>= 8;
		spare[i] = (uint8_t)(ce & 0xFF);
	}
}

/*
 * Configure DMA descriptors and enable DMA channel for data and ECC reads
 */
static void lpc32xx_nand_dma_configure(struct mtd_info *mtd,
	dma_addr_t databuf, int eccsubpages, int read)
{
	struct nand_chip *chip = mtd->priv;
	struct lpc32xx_nand_host *host = chip->priv;
	uint32_t ecc_ctrl, *ecc_buf;
	uint32_t dataaddr, data_ctrl;
	int i;

	/* DMA buffer pointer for calculated ECC values */
	ecc_buf = (uint32_t *)host->ecc_buf_dma;

	/*
	 * ctrl descriptor entry for reading ECC
	 */
	ecc_ctrl = DMAC_CHAN_SRC_BURST_1 |
		DMAC_CHAN_DEST_BURST_1 |
		DMAC_CHAN_SRC_WIDTH_32 |
		DMAC_CHAN_DEST_WIDTH_32 |
		DMAC_CHAN_DEST_AHB1;

	/* data descriptor entry for reading/writing data */
	data_ctrl = ((mtd->writesize / eccsubpages) / 4) |
		DMAC_CHAN_SRC_BURST_4 |
		DMAC_CHAN_DEST_BURST_4 |
		DMAC_CHAN_SRC_WIDTH_32 |
		DMAC_CHAN_DEST_WIDTH_32 |
		DMAC_CHAN_DEST_AHB1;

	if (read) {
		data_ctrl |= DMAC_CHAN_DEST_AUTOINC;
		ecc_ctrl |= DMAC_CHAN_TRANSFER_SIZE(1);
	} else {
		data_ctrl |= DMAC_CHAN_SRC_AUTOINC;
		ecc_ctrl |= DMAC_CHAN_TRANSFER_SIZE(LPC32XX_DMA_ECC_REP_READ);
	}

	/*
	 * Only transfer the data areas plus ECC from hardware. The last ECC
	 * from hardware and OOB area will be transferred later.
	 */
	dataaddr = (uint32_t)databuf;

	for (i = 0; i < eccsubpages; i++) {
		if ((i == (eccsubpages - 1)) &&
			(!host->ncfg->polled_completion))
			data_ctrl |= DMAC_CHAN_INT_TC_EN;

		if (read)
			lpc32xx_dma_queue_llist(host->dmach,
				(void *)SLC_DMA_DATA(host->io_base_dma),
				(void *)dataaddr, -1, data_ctrl);
		else
			lpc32xx_dma_queue_llist(host->dmach, (void *)dataaddr,
				(void *)SLC_DMA_DATA(host->io_base_dma), -1,
				data_ctrl);

		dataaddr += (uint32_t)chip->ecc.size;;

		if (i != (eccsubpages - 1)) {
			lpc32xx_dma_queue_llist(host->dmach,
				(void *)SLC_ECC(host->io_base_dma),
				(void *)ecc_buf, -1, ecc_ctrl);
			ecc_buf++;;
		}
	}
}

/*
 * DMA read/write transfers with ECC support
 */
static int lpc32xx_dma_xfer(struct mtd_info *mtd, uint8_t *buf,
	int eccsubpages, int read)
{
	struct nand_chip *chip = mtd->priv;
	struct lpc32xx_nand_host *host = chip->priv;
	uint32_t config, tmpreg;
	dma_addr_t buf_phy;
	int i, timeout, dma_mapped = 0, status = 0;

	/* Map DMA buffer */
	if (likely((void *) buf < high_memory)) {
		buf_phy = dma_map_single(mtd->dev.parent, buf, mtd->writesize,
			read ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(mtd->dev.parent, buf_phy))) {
			dev_err(mtd->dev.parent,
				"Unable to map DMA buffer\n");
			dma_mapped = 0;
		} else
			dma_mapped = 1;
	}

	/* If a buffer can't be mapped, use the local buffer */
	if (!dma_mapped) {
		buf_phy = host->data_buf_dma;
		if (!read)
			memcpy(host->data_buf, buf, mtd->writesize);
	}

	if (read)
		config = DMAC_CHAN_ITC | DMAC_CHAN_IE | DMAC_CHAN_FLOW_D_P2M |
			DMAC_DEST_PERIP (0) |
			DMAC_SRC_PERIP(DMA_PERID_NAND1) | DMAC_CHAN_ENABLE;
	else
		config = DMAC_CHAN_ITC | DMAC_CHAN_IE | DMAC_CHAN_FLOW_D_M2P |
			DMAC_DEST_PERIP(DMA_PERID_NAND1) |
			DMAC_SRC_PERIP (0) | DMAC_CHAN_ENABLE;

	/* DMA mode with ECC enabled */
	tmpreg = __raw_readl(SLC_CFG(host->io_base));
	__raw_writel(SLCCFG_ECC_EN | SLCCFG_DMA_ECC | tmpreg,
		SLC_CFG(host->io_base));

	/* Clear initial ECC */
	__raw_writel(SLCCTRL_ECC_CLEAR, SLC_CTRL(host->io_base));

	/* Prepare DMA descriptors */
	lpc32xx_nand_dma_configure(mtd, buf_phy, chip->ecc.steps, read);

	/* Setup DMA direction and burst mode */
	if (read)
		__raw_writel(__raw_readl(SLC_CFG(host->io_base)) |
			SLCCFG_DMA_DIR, SLC_CFG(host->io_base));
	else
		__raw_writel(__raw_readl(SLC_CFG(host->io_base)) &
			~SLCCFG_DMA_DIR, SLC_CFG(host->io_base));
	__raw_writel(__raw_readl(SLC_CFG(host->io_base)) | SLCCFG_DMA_BURST,
		SLC_CFG(host->io_base));

	/* Transfer size is data area only */
	__raw_writel(mtd->writesize, SLC_TC(host->io_base));

	/* Start transfer in the NAND controller */
	__raw_writel(__raw_readl(SLC_CTRL(host->io_base)) | SLCCTRL_DMA_START,
		SLC_CTRL(host->io_base));

	/* Start DMA to process NAND controller DMA FIFO */
	host->dmapending = 0;
	lpc32xx_dma_start_xfer(host->dmach, config);

	/*
	 * On some systems, the DMA transfer will be very fast, so there is no
	 * point in waiting for the transfer to complete using the interrupt
	 * method. It's best to just poll the transfer here to prevent several
	 * costly context changes. This is especially true for systems that
	 * use small page devices or NAND devices with very fast access.
	 */
	if (host->ncfg->polled_completion) {
		timeout = LPC32XX_DMA_SIMPLE_TIMEOUT;
		while ((timeout > 0) && lpc32xx_dma_is_active(host->dmach))
			timeout--;
		if (timeout == 0) {
			dev_err(mtd->dev.parent,
				"DMA transfer timeout error\n");
			status = -EIO;

			/* Switch to non-polled mode */
			host->ncfg->polled_completion = false;
		}
	}

	if (!host->ncfg->polled_completion) {
		/* Wait till DMA transfer is done or timeout occurs */
		wait_event_timeout(host->dma_waitq, host->dmapending,
			msecs_to_jiffies(LPC32XX_DMA_WAIT_TIMEOUT_MS));
		if (host->dma_xfer_status != 0) {
			dev_err(mtd->dev.parent, "DMA transfer error\n");
			status = -EIO;
		}
	}

	/*
	 * The DMA is finished, but the NAND controller may still have
	 * buffered data. Wait until all the data is sent.
	 */
	timeout = LPC32XX_DMA_SIMPLE_TIMEOUT;
	while ((__raw_readl(SLC_STAT(host->io_base)) & SLCSTAT_DMA_FIFO)
		&& (timeout > 0))
		timeout--;
	if (timeout == 0) {
		dev_err(mtd->dev.parent, "FIFO held data too long\n");
		status = -EIO;
	}

	/* Read last calculated ECC value */
	if (read)
		host->ecc_buf[chip->ecc.steps - 1] =
			__raw_readl(SLC_ECC(host->io_base));
	else {
		for (i = 0; i < LPC32XX_DMA_ECC_REP_READ; i++)
			host->ecc_buf[chip->ecc.steps - 1] =
				__raw_readl(SLC_ECC(host->io_base));
	}

	/*
	 * For reads, get the OOB data. For writes, the data will be written
	 * later
	 */
	if (read)
		chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);

	/* Flush DMA link list */
	lpc32xx_dma_flush_llist(host->dmach);

	if (__raw_readl(SLC_STAT(host->io_base)) & SLCSTAT_DMA_FIFO ||
		__raw_readl(SLC_TC(host->io_base))) {
		/* Something is left in the FIFO, something is wrong */
		dev_err(mtd->dev.parent, "DMA FIFO failure\n");
		status = -EIO;
	}

	if (dma_mapped)
		dma_unmap_single(mtd->dev.parent, buf_phy, mtd->writesize,
			read ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
	else if (read)
		memcpy(buf, host->data_buf, mtd->writesize);

	/* Stop DMA & HW ECC */
	__raw_writel(__raw_readl(SLC_CTRL(host->io_base)) &
		~SLCCTRL_DMA_START, SLC_CTRL(host->io_base));
	__raw_writel(tmpreg, SLC_CFG(host->io_base));

	return status;
}

/*
 * Read the data and OOB data from the device, use ECC correction with the
 * data, disable ECC for the OOB data
 */
static int lpc32xx_nand_read_page_syndrome(struct mtd_info *mtd,
	struct nand_chip *chip, uint8_t *buf, int page)
{
	struct lpc32xx_nand_host *host = chip->priv;
	int stat, i, status;
	uint8_t *oobecc, tmpecc[LPC32XX_ECC_SAVE_SIZE];

	/* Issue read command */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);

	/* Read data and oob, calculate ECC */
	status = lpc32xx_dma_xfer(mtd, buf, chip->ecc.steps, 1);

	/* Convert to stored ECC format */
	lpc32xx_slc_ecc_copy(tmpecc, (uint32_t *) host->ecc_buf,
		chip->ecc.steps);

	/* Pointer to ECC data retrieved from NAND spare area */
	oobecc = chip->oob_poi + chip->ecc.layout->eccpos[0];

	for (i = 0; i < chip->ecc.steps; i++) {
		stat = chip->ecc.correct(mtd, buf, oobecc,
			&tmpecc[i * chip->ecc.bytes]);
		if (stat < 0)
			mtd->ecc_stats.failed++;
		else
			mtd->ecc_stats.corrected += stat;

		buf += chip->ecc.size;
		oobecc += chip->ecc.bytes;
	}

	return status;
}

/*
 * Read the data and OOB data from the device, no ECC correction with the
 * data or OOB data
 */
static int lpc32xx_nand_read_page_raw_syndrome(struct mtd_info *mtd,
	struct nand_chip *chip, uint8_t *buf, int page)
{
	/* Issue read command */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);

	/* Raw reads can just use the FIFO interface */
	chip->read_buf(mtd, buf, chip->ecc.size * chip->ecc.steps);
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);

	return 0;
}

/*
 * Write the data and OOB data to the device, use ECC with the data,
 * disable ECC for the OOB data
 */
static void lpc32xx_nand_write_page_syndrome(struct mtd_info *mtd,
	struct nand_chip *chip, const uint8_t *buf)
{
	struct lpc32xx_nand_host *host = chip->priv;
	uint8_t *pb = chip->oob_poi + chip->ecc.layout->eccpos[0];

	/* Write data, calculate ECC on outbound data */
	lpc32xx_dma_xfer(mtd, (uint8_t *)buf, chip->ecc.steps, 0);

	/*
	 * The calculated ECC needs some manual work done to it before
	 * committing it to NAND. Process the calculated ECC and place
	 * the resultant values directly into the OOB buffer. */
	lpc32xx_slc_ecc_copy(pb, (uint32_t *) host->ecc_buf, chip->ecc.steps);

	/* Write ECC data to device */
	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);
}

/*
 * Write the data and OOB data to the device, no ECC correction with the
 * data or OOB data
 */
static void lpc32xx_nand_write_page_raw_syndrome(struct mtd_info *mtd,
	struct nand_chip *chip, const uint8_t *buf)
{
	/* Raw writes can just use the FIFO interface */
	chip->write_buf(mtd, buf, chip->ecc.size * chip->ecc.steps);
	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);
}

/*
 * DMA ISR - occurs when DMA transfer complete.
 */
static void lpc3xxx_nand_dma_irq(int channel, int cause,
			struct lpc32xx_nand_host *host)
{
	host->dma_xfer_status = (cause & DMA_TC_INT) ? 0: 1;
	host->dmapending = 1;
	wake_up(&host->dma_waitq);
}

/*
 * Get DMA channel and allocate DMA descriptors memory.
 * Prepare DMA descriptors link lists
 */
static int lpc32xx_nand_dma_setup(struct lpc32xx_nand_host *host,
	int num_entries)
{
	int ret = 0;

	host->dmach = DMA_CH_SLCNAND;
	host->dmacfg.ch = DMA_CH_SLCNAND;

	/*
	 * All the DMA configuration parameters will
	 * be overwritten in lpc32xx_nand_dma_configure().
	 */
	host->dmacfg.tc_inten = 1;
	host->dmacfg.err_inten = 1;
	host->dmacfg.src_size = 4;
	host->dmacfg.src_inc = 1;
	host->dmacfg.src_ahb1 = 1;
	host->dmacfg.src_bsize = DMAC_CHAN_SRC_BURST_4;
	host->dmacfg.src_prph = 0;
	host->dmacfg.dst_size = 4;
	host->dmacfg.dst_inc = 0;
	host->dmacfg.dst_bsize = DMAC_CHAN_DEST_BURST_4;
	host->dmacfg.dst_ahb1 = 0;
	host->dmacfg.dst_prph = DMAC_DEST_PERIP(DMA_PERID_NAND1);
	host->dmacfg.flowctrl = DMAC_CHAN_FLOW_D_M2P;
	if (lpc32xx_dma_ch_get(&host->dmacfg, LPC32XX_MODNAME,
		&lpc3xxx_nand_dma_irq, host) < 0) {
		dev_err(host->mtd.dev.parent, "Error setting up SLC NAND "
			"DMA channel\n");
		ret = -ENODEV;
		goto dma_ch_err;
	}

	/*
	 * Allocate Linked list of DMA Descriptors
	 */
	host->llptr = lpc32xx_dma_alloc_llist(host->dmach, num_entries);
	if (host->llptr == 0) {
		lpc32xx_dma_ch_put(host->dmach);
		host->dmach = -1;
		dev_err(host->mtd.dev.parent,
			"Error allocating list buffer for SLC NAND\n");
		ret = -ENOMEM;
		goto dma_alloc_err;
	}

	return ret;
dma_alloc_err:
	lpc32xx_dma_ch_put(host->dmach);
dma_ch_err:
	return ret;
}

static int __init lpc32xx_add_partitions(struct lpc32xx_nand_host *host)
{
#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_info *mtd = &host->mtd;
	struct mtd_partition *partitions = NULL;
	int num_partitions = 0;

#ifdef CONFIG_MTD_CMDLINE_PARTS
	static const char *part_probes[] = { "cmdlinepart", NULL };

	mtd->name = LPC32XX_MODNAME;
	num_partitions = parse_mtd_partitions(mtd, part_probes,
					      &partitions, 0);
#endif
	if ((num_partitions <= 0) && (host->ncfg->partition_info))
		partitions = host->ncfg->partition_info(mtd->size,
			&num_partitions);

	if ((!partitions) || (num_partitions == 0)) {
		dev_err(mtd->dev.parent,"No parititions defined,"
			" or unsupported device.\n");
		return ENXIO;
	}

	return add_mtd_partitions(mtd, partitions, num_partitions);
#else
	return add_mtd_device(mtd);
#endif
}

/*
 * Probe for NAND controller
 */
static int __init lpc32xx_nand_probe(struct platform_device *pdev)
{
	struct lpc32xx_nand_host *host;
	struct mtd_info *mtd;
	struct nand_chip *chip;
	struct resource *rc;
	int res;

	rc = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (rc == NULL) {
		dev_err(&pdev->dev,"No memory resource found for"
			" device\n");
		return -ENXIO;
	}

	/* Allocate memory for the device structure (and zero it) */
	host = kzalloc(sizeof(struct lpc32xx_nand_host), GFP_KERNEL);
	if (!host) {
		dev_err(&pdev->dev,"failed to allocate device structure\n");
		return -ENOMEM;
	}
	host->io_base_dma = (dma_addr_t) rc->start;

	host->io_base = ioremap(rc->start, rc->end - rc->start + 1);
	if (host->io_base == NULL) {
		dev_err(&pdev->dev,"ioremap failed\n");
		res = -EIO;
		goto err_exit1;
	}

	host->ncfg = pdev->dev.platform_data;
	if (!host->ncfg) {
		dev_err(&pdev->dev,"Missing platform data\n");
		res = -ENOENT;
		goto err_exit1;
	}

	mtd = &host->mtd;
	chip = &host->nand_chip;
	chip->priv = host;
	mtd->priv = chip;
	mtd->owner = THIS_MODULE;
	mtd->dev.parent = &pdev->dev;

	/* Get NAND clock */
	host->clk = clk_get(&pdev->dev, "nand_ck");
	if (IS_ERR(host->clk)) {
		 dev_err(&pdev->dev,"Clock failure\n");
		res = -ENOENT;
		goto err_exit2;
	}
	clk_enable(host->clk);

	/* Set NAND IO addresses and command/ready functions */
	chip->IO_ADDR_R = SLC_DATA(host->io_base);
	chip->IO_ADDR_W = SLC_DATA(host->io_base);
	chip->cmd_ctrl = lpc32xx_nand_cmd_ctrl;
	chip->dev_ready = lpc32xx_nand_device_ready;
	chip->chip_delay = 20; /* 20us command delay time */

	/* Init NAND controller */
	lpc32xx_nand_setup(host);
	lpc32xx_wp_disable(host);

	platform_set_drvdata(pdev, host);

	/* NAND callbacks for LPC32xx SLC hardware */
	chip->ecc.mode = NAND_ECC_HW_SYNDROME;
	chip->read_byte = lpc32xx_nand_read_byte;
	chip->read_buf = lpc32xx_nand_read_buf;
	chip->write_buf = lpc32xx_nand_write_buf;
	chip->ecc.read_page_raw = lpc32xx_nand_read_page_raw_syndrome;
	chip->ecc.read_page = lpc32xx_nand_read_page_syndrome;
	chip->ecc.write_page_raw = lpc32xx_nand_write_page_raw_syndrome;
	chip->ecc.write_page = lpc32xx_nand_write_page_syndrome;
	chip->ecc.write_oob = lpc32xx_nand_write_oob_syndrome;
	chip->ecc.read_oob = lpc32xx_nand_read_oob_syndrome;
	chip->ecc.calculate = lpc32xx_nand_ecc_calculate;
	chip->ecc.correct   = nand_correct_data;
	chip->ecc.hwctl = lpc32xx_nand_ecc_enable;
	chip->verify_buf = lpc32xx_verify_buf;

	/*
	 * Allocate a large enough buffer for a single huge page plus
	 * extra space for the spare area and ECC storage area
	 */
	host->dma_buf_len = LPC32XX_DMA_DATA_SIZE + LPC32XX_ECC_SAVE_SIZE;
	host->data_buf = dma_alloc_coherent(&pdev->dev, host->dma_buf_len,
		&host->data_buf_dma, GFP_KERNEL);
	if (host->data_buf == NULL) {
		dev_err(&pdev->dev, "Error allocating memory\n");
		res = -ENOMEM;
		goto err_exit3;
	}

	/* Get free DMA channel and alloc DMA descriptor link list */
	res = lpc32xx_nand_dma_setup(host, LPC32XX_MAX_DMA_DESCRIPTORS);
	if(res) {
		res = -EIO;
		goto err_exit4;
	}

	init_waitqueue_head(&host->dma_waitq);

	/* Find NAND device */
	if (nand_scan_ident(mtd, 1)) {
		res = -ENXIO;
		goto err_exit5;
	}

	/* OOB and ECC CPU and DMA work areas */
	host->ecc_buf_dma = host->data_buf_dma + LPC32XX_DMA_DATA_SIZE;
	host->ecc_buf = (uint32_t *) (host->data_buf + LPC32XX_DMA_DATA_SIZE);

	/*
	 * Small page FLASH has a unique OOB layout, but large and huge
	 * page FLASH use the standard layout. Small page FLASH uses a
	 * custom BBT marker layout.
	 */
	if (mtd->writesize <= 512)
		chip->ecc.layout = &lpc32xx_nand_oob_16;

	/* These sizes remain the same regardless of page size */
	chip->ecc.size = 256;
	chip->ecc.bytes = LPC32XX_SLC_DEV_ECC_BYTES;
	chip->ecc.prepad = chip->ecc.postpad = 0;

	/* Avoid extra scan if using BBT, setup BBT support */
	if (host->ncfg->use_bbt) {
		chip->options |= NAND_USE_FLASH_BBT | NAND_SKIP_BBTSCAN;

		/*
		 * Use a custom BBT marker setup for small page FLASH that
		 * won't interfere with the ECC layout. Large and huge page
		 * FLASH use the standard layout.
		 */
		if (mtd->writesize <= 512) {
			chip->bbt_td = &bbt_smallpage_main_descr;
			chip->bbt_md = &bbt_smallpage_mirror_descr;
		}
	}

	/*
	 * Fills out all the uninitialized function pointers with the defaults
	 */
	if (nand_scan_tail(mtd)) {
		res = -ENXIO;
		goto err_exit5;
	}

	/* Standard layout in FLASH for bad block tables */
	if (host->ncfg->use_bbt) {
		if (nand_default_bbt(mtd) < 0)
			dev_err(&pdev->dev, "Error initializing default bad"
				" block tables\n");
	}

	res = lpc32xx_add_partitions(host);
	if (!res)
		return res;

	nand_release(mtd);

err_exit5:
	/* Free the DMA channel used by us */
	lpc32xx_dma_ch_disable(host->dmach);
	lpc32xx_dma_dealloc_llist(host->dmach);
	lpc32xx_dma_ch_put(host->dmach);
	host->dmach = -1;
err_exit4:
	dma_free_coherent(&pdev->dev, host->dma_buf_len,
		host->data_buf, host->data_buf_dma);
err_exit3:
	clk_disable(host->clk);
	clk_put(host->clk);
	platform_set_drvdata(pdev, NULL);
err_exit2:
	lpc32xx_wp_enable(host);
	iounmap(host->io_base);
err_exit1:
	kfree(host);

	return res;
}

/*
 * Remove NAND device.
 */
static int __devexit lpc32xx_nand_remove(struct platform_device *pdev)
{
	u32 tmp;
	struct lpc32xx_nand_host *host = platform_get_drvdata(pdev);
	struct mtd_info *mtd = &host->mtd;

	nand_release(mtd);

	/* Free the DMA channel used by us */
	lpc32xx_dma_ch_disable(host->dmach);
	lpc32xx_dma_dealloc_llist(host->dmach);
	lpc32xx_dma_ch_put(host->dmach);
	host->dmach = -1;

	dma_free_coherent(&pdev->dev, host->dma_buf_len,
		host->data_buf, host->data_buf_dma);

	/* Force CE high */
	tmp = __raw_readl(SLC_CTRL(host->io_base));
	tmp &= ~SLCCFG_CE_LOW;
	__raw_writel(tmp, SLC_CTRL(host->io_base));

	lpc32xx_wp_enable(host);
	clk_disable(host->clk);
	clk_put(host->clk);

	iounmap(host->io_base);

	kfree(host);

	return 0;
}

#if defined (CONFIG_PM)
static int lpc32xx_nand_resume(struct platform_device *pdev)
{
	struct lpc32xx_nand_host *host = platform_get_drvdata(pdev);

	/* Re-enable NAND clock */
	clk_enable(host->clk);

	/* Fresh init of NAND controller */
	lpc32xx_nand_setup(host);

	/* Disable write protect */
	lpc32xx_wp_disable(host);

	return 0;
}

static int lpc32xx_nand_suspend(struct platform_device *pdev, pm_message_t pm)
{
	u32 tmp;
	struct lpc32xx_nand_host *host = platform_get_drvdata(pdev);

	/* Force CE high */
	tmp = __raw_readl(SLC_CTRL(host->io_base));
	tmp &= ~SLCCFG_CE_LOW;
	__raw_writel(tmp, SLC_CTRL(host->io_base));

	/* Enable write protect for safety */
	lpc32xx_wp_enable(host);

	/* Disable clock */
	clk_disable(host->clk);

	return 0;
}

#else
#define lpc32xx_nand_resume NULL
#define lpc32xx_nand_suspend NULL
#endif

static struct platform_driver lpc32xx_nand_driver = {
	.probe		= lpc32xx_nand_probe,
	.remove		= __devexit_p(lpc32xx_nand_remove),
	.resume		= lpc32xx_nand_resume,
	.suspend	= lpc32xx_nand_suspend,
	.driver		= {
		.name	= LPC32XX_MODNAME,
		.owner	= THIS_MODULE,
	},
};

static int __init lpc32xx_nand_init(void)
{
	return platform_driver_register(&lpc32xx_nand_driver);
}

static void __exit lpc32xx_nand_exit(void)
{
	platform_driver_unregister(&lpc32xx_nand_driver);
}

module_init(lpc32xx_nand_init);
module_exit(lpc32xx_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin Wells(kevin.wells@nxp.com)");
MODULE_DESCRIPTION("NAND driver for the NXP LPC32XX SLC controller");

