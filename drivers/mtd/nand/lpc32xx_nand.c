/*
 * drivers/mtd/nand/lpc32xx_nand.c
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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/completion.h>

#include <asm/io.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/nand_ecc.h>

#include <asm/sizes.h>
#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/slcnand.h>
#include <mach/dmac.h>
#include <mach/dma.h>

/*
 * LPC3250 has 3 bytes of ECC data but due to DMA
 * word transfer limitation, we'll use 4 bytes
 */
#define NAND_ECC_LEN_PER_SUBPAGE        0x4
#define NAND_ECC_SUBPAGE_LEN            256

#define NAND_LARGE_BLOCK_PAGE_SIZE      2048
#define NAND_SMALL_BLOCK_PAGE_SIZE      512

#define NAND_ERASED_BLOCK_ECC_VALUE	0xFFFFFFFF

static struct nand_ecclayout lpc32xx_nand_oob_16 = {
        .eccbytes = 8,
        .eccpos = {8, 9, 10, 11, 12, 13, 14, 15},
        .oobfree = {
                {.offset = 0,
                 . length = 5},
                {.offset = 6,
                 . length = 2}}
};

static struct nand_ecclayout lpc32xx_nand_oob_64 = {
        .eccbytes = 32,
        .eccpos = { 8, 9, 10, 11, 12, 13, 14, 15,
                   24, 25, 26, 27, 28, 29, 30, 31,
                   40, 41, 42, 43, 44, 45, 46, 47,
                   56, 57, 58, 59, 60, 61, 62, 63},
        .oobfree = {
                {.offset = 2,
                 . length = 6},
                {.offset = 16,
                 . length = 8},
                {.offset = 32,
                 . length = 8},
                {.offset = 48,
                 . length = 8}}
};

struct lpc32xx_nand_host {
	struct nand_chip	nand_chip;
	struct clk		*clk;
	struct mtd_info		mtd;
	void __iomem		*io_base;
	struct lpc32XX_nand_cfg	*ncfg;
	struct completion       comp;
	struct dma_config dmacfg;
	int dmach;
	uint32_t dma_xfer_status;
	uint32_t llptr;
	uint32_t dma_buf_len;
	/*
	 * Physical addresses of ECC buffer,DMA data buffers,OOB data buffer
	 */
	dma_addr_t oob_buf_phy;
	dma_addr_t ecc_calc_buf_phy;
	dma_addr_t dma_buf_phy;
	/*
	 * Virtual addresses of ECC buffer,DMA data buffers,OOB data buffer
	 */
	uint8_t *oob_buf;
	uint8_t *ecc_calc_buf;
	uint8_t * dma_buf;
	/* Physical address of DMA base address */
	dma_addr_t io_base_phy;
	uint8_t *erase_buf_data;
};

#ifdef CONFIG_MTD_PARTITIONS
const char *part_probes[] = { "cmdlinepart", NULL };
#endif

static uint8_t nand_slc_bit_cnt16(uint16_t ch)
{
        ch = (ch & 0x5555) + ((ch & ~0x5555) >> 1);
        ch = (ch & 0x3333) + ((ch & ~0x3333) >> 2);
        ch = (ch & 0x0F0F) + ((ch & ~0x0F0F) >> 4);
        return (ch + (ch >> 8)) & 0xFF;
}

static uint8_t bit_cnt32(uint32_t val)
{
        return nand_slc_bit_cnt16(val & 0xFFFF) +
                nand_slc_bit_cnt16(val >> 16);
}

static void lpc32xx_nand_setup(struct lpc32xx_nand_host *host)
{
	u32 clkrate, tmp;

	/* Reset SLC controller */
	__raw_writel(SLCCTRL_SW_RESET, SLC_CTRL(host->io_base));
	udelay(1000);

	/* Basic setup */
	__raw_writel(0, SLC_CFG(host->io_base));
	__raw_writel(0, SLC_IEN(host->io_base));
	__raw_writel((SLCSTAT_INT_TC | SLCSTAT_INT_RDY_EN), SLC_ICR(host->io_base));

	/* Get base clock for SLC block */
	clkrate = clk_get_rate(host->clk);
	if (clkrate == 0)
		clkrate = 133000000;

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
static void lpc32xx_nand_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	u32 tmp;
	struct nand_chip *nand_chip = mtd->priv;
	struct lpc32xx_nand_host *host = nand_chip->priv;

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
 * Read the Device Ready pin.
 */
static int lpc32xx_nand_device_ready(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct lpc32xx_nand_host *host = nand_chip->priv;
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

static uint8_t lpc32xx_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct lpc32xx_nand_host *host = nand_chip->priv;

	return (uint8_t) __raw_readl(SLC_DATA(host->io_base));
}

static void lpc32xx_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct lpc32xx_nand_host *host = nand_chip->priv;
	int i;

	for (i = 0; i < len; i++)
		buf[i] = (uint8_t) __raw_readl(SLC_DATA(host->io_base));
}

static int lpc32xx_verify_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct lpc32xx_nand_host *host = nand_chip->priv;
	int i;

	for (i = 0; i < len; i++) {
		if (buf[i] != (uint8_t) __raw_readl(SLC_DATA(host->io_base)))
			return -EFAULT;
	}

	return 0;
}

static void lpc32xx_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct lpc32xx_nand_host *host = nand_chip->priv;
	int i;

	for (i = 0; i < len; i++)
		__raw_writel((u32) buf[i], SLC_DATA(host->io_base));
}

/*
 * DMA ISR - occurs when DMA transfer complete.
 */
static void lpc3xxx_nand_dma_irq(int channel, int cause,
			struct lpc32xx_nand_host *host)
{
	/* Flush DMA link list */
	lpc32xx_dma_flush_llist(host->dmach);

	host->dma_xfer_status = (cause & DMA_TC_INT)? 0: 1;
	complete(&host->comp);
}

/*
 * Get DMA channel and allocate DMA descriptors memory.
 * Prepare DMA descriptors link lists
 */
static int lpc32xx_nand_dma_setup(struct lpc32xx_nand_host *host, int num_entries)
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
	if (lpc32xx_dma_ch_get(&host->dmacfg, "dma_slcnand",
				&lpc3xxx_nand_dma_irq, host) < 0) {
		printk(KERN_ERR "Error setting up SLC NAND DMA channel\n");
		ret = -ENODEV;
		goto dma_ch_err;
	}

	/* 
	 * Allocate Linked list of total DMA Descriptors.
	 * For Large Block: 17 descriptors = ((16 Data and ECC Read) + 1 Spare Area)
	 * For Small Block: 5 descriptors = ((4 Data and ECC Read) + 1 Spare Area)
	 */
	host->llptr = lpc32xx_dma_alloc_llist(host->dmach, num_entries);
	if (host->llptr == 0) {
		lpc32xx_dma_ch_put(host->dmach);
		host->dmach = -1;
		printk(KERN_ERR "Error allocating list buffer for SLC NAND\n");
		ret = -ENOMEM;
		goto dma_alloc_err;
	}

	return ret;
dma_alloc_err:
	lpc32xx_dma_ch_put(host->dmach);
dma_ch_err:
	return ret;
}

/*
 * Configure DMA descriptors and start DMA x'fer
 */
static void lpc32xx_nand_dma_configure(struct mtd_info *mtd,
		dma_addr_t buffer, int size, int read)
{
	struct nand_chip *chip = mtd->priv;
	struct lpc32xx_nand_host *host = chip->priv;
	uint32_t page_divider = (size == NAND_LARGE_BLOCK_PAGE_SIZE) ? 8: 2;
	uint32_t dmasrc, dmadst, ctrl, ecc_ctrl, oob_ctrl;
	int i;
	uint32_t *eccpos = chip->ecc.layout->eccpos;

	/* 
	 * CTRL descriptor entry for reading ECC
	 * Copy Multiple times to sync DMA with Flash Controller
	 */
	ecc_ctrl =  (0x5 |
			DMAC_CHAN_SRC_BURST_1 |
			DMAC_CHAN_DEST_BURST_1 |
			DMAC_CHAN_SRC_WIDTH_32 |
			DMAC_CHAN_DEST_WIDTH_32 |
			DMAC_CHAN_DEST_AHB1);

	/* CTRL descriptor entry for reading/writing data */
	ctrl =     ((mtd->writesize / page_divider) / 4) |
		DMAC_CHAN_SRC_BURST_4 |
		DMAC_CHAN_DEST_BURST_4 |
		DMAC_CHAN_SRC_WIDTH_32 |
		DMAC_CHAN_DEST_WIDTH_32 |
		DMAC_CHAN_DEST_AHB1;

	/* CTRL descriptor entry for reading/writing Spare Area */
	oob_ctrl =  ((mtd->oobsize / 4) |
                        DMAC_CHAN_SRC_BURST_4 |
                        DMAC_CHAN_DEST_BURST_4 |
                        DMAC_CHAN_SRC_WIDTH_32 |
                        DMAC_CHAN_DEST_WIDTH_32 |
                        DMAC_CHAN_DEST_AHB1);

        if (read) {
                dmasrc = (uint32_t) SLC_DMA_DATA(host->io_base_phy);
                dmadst = (uint32_t) (buffer);
                ctrl |= DMAC_CHAN_DEST_AUTOINC;
        } else {
                dmadst = (uint32_t) SLC_DMA_DATA(host->io_base_phy);
                dmasrc = (uint32_t) (buffer);
                ctrl |= DMAC_CHAN_SRC_AUTOINC;
        }

	/*
	 * Write Operation Sequence for Small Block NAND
	 * ----------------------------------------------------------
	 * 1. X'fer 256 bytes of data from Memory to Flash.
	 * 2. Copy generated ECC data from Register to Spare Area
	 * 3. X'fer next 256 bytes of data from Memory to Flash.
	 * 4. Copy generated ECC data from Register to Spare Area.
	 * 5. X'fer 16 byets of Spare area from Memory to Flash.
	 *
	 * Read Operation Sequence for Small Block NAND
	 * ----------------------------------------------------------
	 * 1. X'fer 256 bytes of data from Flash to Memory.
	 * 2. Copy generated ECC data from Register to ECC calc Buffer.
	 * 3. X'fer next 256 bytes of data from Flash to Memory.
	 * 4. Copy generated ECC data from Register to ECC calc Buffer.
	 * 5. X'fer 16 bytes of Spare area from Flash to Memory.
	 *
	 * Write Operation Sequence for Large Block NAND
	 * ----------------------------------------------------------
	 * 1. Steps(1-4) of Write Operations repeate for four times
	 * which generates 16 DMA descriptors to X'fer 2048 byets of
	 * data & 32 bytes of ECC data.
	 * 2. X'fer 64 bytes of Spare area from Memory to Flash.
	 *
	 * Read Operation Sequence for Large Block NAND
	 * ----------------------------------------------------------
	 * 1. Steps(1-4) of Read Operations repeate for four times
	 * which generates 16 DMA descriptors to X'fer 2048 byets of 
	 * data & 32 bytes of ECC data.
	 * 2. X'fer 64 bytes of Spare area from Flash to Memory.
	 */
        for (i = 0; i < size/256; i++) {
		lpc32xx_dma_queue_llist(host->dmach,
				(void *)(read ?(dmasrc) :(dmasrc + (i*256))), 
				(void *)(read ?(dmadst + (i*256)) :dmadst),
				-1, ctrl);
		lpc32xx_dma_queue_llist(host->dmach,
				(void *)SLC_ECC(host->io_base_phy),
				(void *)(read ?((uint32_t) host->ecc_calc_buf_phy + (i*4)):
				 ((uint32_t) host->oob_buf_phy + eccpos[i*4])),
				-1, ecc_ctrl);
        }

        if (read) {
                dmasrc = (uint32_t) (uint32_t) SLC_DMA_DATA(host->io_base_phy);
                dmadst = (uint32_t) (host->oob_buf_phy);
                oob_ctrl |= DMAC_CHAN_DEST_AUTOINC;
        } else {
                dmadst = (uint32_t) (uint32_t) SLC_DMA_DATA(host->io_base_phy);
                dmasrc = (uint32_t) (host->oob_buf_phy);
                oob_ctrl |= DMAC_CHAN_SRC_AUTOINC;
        }

        /* Read/ Write Spare Area Data To/From Flash */
	lpc32xx_dma_queue_llist(host->dmach, (void *)dmasrc, (void *)dmadst, -1,
			oob_ctrl | DMAC_CHAN_INT_TC_EN);
}

static void lpc32xx_nand_dma_xfer(struct mtd_info *mtd, u_char *buf, int len, int read)
{
	struct nand_chip *this = mtd->priv;
	uint32_t config;
	struct lpc32xx_nand_host *host = this->priv;
	dma_addr_t buf_phy = (dma_addr_t) 0;
	int dma_mapped = 0;

	/* Calculate the physical address of the Buffer */
	/* Check if memory not allocated by vmalloc */
	if (likely((void *) buf < high_memory)) {
		buf_phy = dma_map_single(mtd->dev.parent,
				buf, len, read ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(mtd->dev.parent, buf_phy))) {
			dev_err(mtd->dev.parent, "Unable to DMA map a buffer "
					"of size %d\r\n", len);
			dma_mapped = 0;
		}else {
			dma_mapped = 1;
		}
	}

	if (!dma_mapped) {
		memcpy(host->dma_buf, buf, len);
		buf_phy = host->dma_buf_phy;
	}

	config = DMAC_CHAN_ITC | DMAC_CHAN_IE |
		(read ? DMAC_CHAN_FLOW_D_P2M : DMAC_CHAN_FLOW_D_M2P) |
		(read ? DMAC_DEST_PERIP(0) : DMAC_DEST_PERIP(DMA_PERID_NAND1)) |
		(read ? DMAC_SRC_PERIP(DMA_PERID_NAND1) : DMAC_SRC_PERIP(0)) |
		DMAC_CHAN_ENABLE;

	/* Prepare descriptors for read transfer */
	lpc32xx_nand_dma_configure(mtd, buf_phy, len, read);

	/* This should start the DMA transfers */
	lpc32xx_dma_start_xfer(host->dmach, config);
	__raw_writel(__raw_readl(SLC_CTRL(host->io_base)) | SLCCTRL_DMA_START,
			SLC_CTRL(host->io_base));

	/* Wait for NAND to be ready */
	nand_wait_ready(mtd);
	
	/* Wait till DMA transfer is DONE! */
	wait_for_completion(&host->comp);
	if (unlikely(host->dma_xfer_status != 0)) {
		dev_err(mtd->dev.parent, "DMA transfer error!\r\n");
		WARN_ON(1);
	}

	if (dma_mapped)
		dma_unmap_single(mtd->dev.parent, buf_phy, len,
				read ? DMA_FROM_DEVICE : DMA_TO_DEVICE);

        /* Stop DMA & HW ECC */
	__raw_writel(__raw_readl(SLC_CTRL(host->io_base)) & ~SLCCTRL_DMA_START,
			SLC_CTRL(host->io_base));
	__raw_writel( __raw_readl(SLC_CFG(host->io_base)) &
			~(SLCCFG_DMA_BURST | SLCCFG_ECC_EN |
			  SLCCFG_DMA_ECC | SLCCFG_DMA_DIR),
			SLC_CFG(host->io_base));
}

static int lpc32xx_nand_correct_data(struct mtd_info *mtd, u_char *dat,
                u_char *read_ecc, u_char *calc_ecc)
{
        int ret = 0;
        uint32_t tmp, err;
        uint32_t *ecc_stored = (uint32_t*)read_ecc;
        uint32_t *ecc_gen = (uint32_t*)calc_ecc;


        err = *ecc_stored ^ *ecc_gen;
        /* Only perform ECC processing if an error is detected */
        if (err) {
                /* ECC Failure in i-th block */
                tmp = bit_cnt32(err);
                if (tmp == 11) {
                        uint32_t byte = err >> 6;
                        uint32_t bit = 0;
                        bit = ((err & _BIT(1)) >> 1)|((err & _BIT(3)) >> 2)|
                                ((err & _BIT(5)) >> 3);

                        /* Calculate Byte offset */
                        byte = ((byte & _BIT(1)) >> 1)|((byte & _BIT(3)) >> 2)|
                                ((byte & _BIT(5)) >> 3)|((byte & _BIT(7)) >> 4)|
                                ((byte & _BIT(9)) >> 5)|((byte & _BIT(11)) >> 6)|
                                ((byte & _BIT(13)) >> 7)|((byte & _BIT(15)) >> 8);

                        /* Do the correction */
                        dat[byte] ^= _BIT(bit);
                        ret = 1;
                }else {
                        /* Non-corrrectable */
                        ret = -1;
                }
        }
        return ret;
}

/* Prepares SLC for transfers with H/W ECC enabled */
static void lpc32xx_ecc_enable(struct mtd_info *mtd, int mode)
{
	struct nand_chip *this = mtd->priv;
	struct lpc32xx_nand_host *host = this->priv;

	/* Clear ECC, start DMA */
	__raw_writel(SLCCTRL_ECC_CLEAR, SLC_CTRL(host->io_base));

	if (mode == NAND_ECC_READ) {
		__raw_writel( __raw_readl(SLC_CFG(host->io_base)) |
		        SLCCFG_DMA_DIR, SLC_CFG(host->io_base));
	}
	else  { /* NAND_ECC_WRITE */
		__raw_writel( __raw_readl(SLC_CFG(host->io_base)) &
		        ~SLCCFG_DMA_DIR, SLC_CFG(host->io_base));
	}

	__raw_writel( __raw_readl(SLC_CFG(host->io_base)) |
			SLCCFG_DMA_BURST | SLCCFG_ECC_EN | SLCCFG_DMA_ECC,
			SLC_CFG(host->io_base));

	/* Set transfer count */
	__raw_writel(this->ecc.size + mtd->oobsize, SLC_TC(host->io_base));
}

/* Function to calculate inverted ECC from the ECC got from H/W */
static int lpc32xx_ecc_calculate(struct mtd_info *mtd, const uint8_t *dat,
				 uint8_t *ecc_code)
{
	return 0;
}

static void lpc32xx_nand_write_page_hwecc(struct mtd_info *mtd,
				struct nand_chip *chip, const uint8_t *buf)
{
	struct nand_chip *this = mtd->priv;
	struct lpc32xx_nand_host *host = this->priv;
        int eccsize = chip->ecc.size;

	/* 
	 * Skip writting page which has all 0xFF data as this will
	 * generate 0x0 value.
	 */
        if(memcmp(buf, host->erase_buf_data, mtd->writesize) == 0)
                return;

        /* Enable H/W ECC & DMA */
        chip->ecc.hwctl(mtd, NAND_ECC_WRITE);
	
	/* Copy OOB data from kernel buffer to DMA memory */
	memcpy(host->oob_buf, chip->oob_poi,mtd->oobsize);

        /* Configure DMA Desriptor for NAND Write Operation */
        lpc32xx_nand_dma_xfer(mtd, (uint8_t *)buf, eccsize, 0);
}

static int lpc32xx_nand_read_page_hwecc(struct mtd_info *mtd,
                                 struct nand_chip *chip, uint8_t *buf, int page)
{
	struct nand_chip *this = mtd->priv;
	struct lpc32xx_nand_host *host = this->priv;
        int i, eccsize = chip->ecc.size;
        int eccsteps = (mtd->writesize/NAND_ECC_SUBPAGE_LEN);
        uint8_t *p = buf;
        uint8_t *ecc_calc = chip->buffers->ecccalc;
        uint8_t *ecc_code = chip->buffers->ecccode;
        uint32_t *eccpos = chip->ecc.layout->eccpos;

	memset(host->ecc_calc_buf, 0x0, this->ecc.bytes);

        /* Enable HW ECC & DMA */
        chip->ecc.hwctl(mtd, NAND_ECC_READ);

        /* Configure DMA Desriptor for NAND Read Operation */
        lpc32xx_nand_dma_xfer(mtd, buf, eccsize, 1);

	/* Copy OOB data from DMA memory to kernel buffer */
	memcpy(chip->oob_poi, host->oob_buf, mtd->oobsize);
	
        /* Copy only ECC data which are stored into Flash */
        for (i = 0; i < chip->ecc.total; i++) {
                ecc_code[i] = chip->oob_poi[eccpos[i]];
                ecc_calc[i] = host->ecc_calc_buf[i];
	}

        /*
	 * LPC3250 has 4 bytes of ECC data per 256 bytes of data block
	 * As eccsteps are calucated based on subpage size.
	 */
        for (i = 0; eccsteps; eccsteps--, i += NAND_ECC_LEN_PER_SUBPAGE,
                         p += NAND_ECC_SUBPAGE_LEN) {
                int stat;

		/*
		 * Once block is erased, all the data including OOB data are 0xFF.
		 * ECC generator always generate zero value ECC for such page while,
		 * stored value is 0xFFFFFFFF.
		 */ 
		if(*((uint32_t *)&ecc_code[i]) == NAND_ERASED_BLOCK_ECC_VALUE)
			continue;
		
		stat = chip->ecc.correct(mtd, p, &ecc_code[i], &ecc_calc[i]);
                if (stat == -1)
                        mtd->ecc_stats.failed++;
                else
                        mtd->ecc_stats.corrected += stat;
        }
        return 0;
}

/*
 * Probe for NAND controller
 */
static int __init lpc32xx_nand_probe(struct platform_device *pdev)
{
	struct lpc32xx_nand_host *host;
	struct mtd_info *mtd;
	struct nand_chip *nand_chip;
	struct resource *rc;
	int res;

#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *partitions = NULL;
	int num_partitions = 0;
#endif

	/* Allocate memory for the device structure (and zero it) */
	host = kzalloc(sizeof(struct lpc32xx_nand_host), GFP_KERNEL);
	if (!host) {
		 dev_err(&pdev->dev,"lpc32xx_nand: failed to allocate device structure.\n");
		return -ENOMEM;
	}
	
	rc = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (rc == NULL) {
		dev_err(&pdev->dev,"No memory resource found for device!\r\n");
		res = -ENXIO;
		goto err_exit1;
	}

	host->io_base = ioremap(rc->start, rc->end - rc->start + 1);
	if (host->io_base == NULL) {
		 dev_err(&pdev->dev,"lpc32xx_nand: ioremap failed\n");
		res = -EIO;
		goto err_exit1;
	}

	mtd = &host->mtd;
	nand_chip = &host->nand_chip;
	host->ncfg = pdev->dev.platform_data;

	nand_chip->priv = host;		/* link the private data structures */
	mtd->priv = nand_chip;
	mtd->owner = THIS_MODULE;
	mtd->dev.parent = &pdev->dev;

	/* Get NAND clock */
	host->clk = clk_get(&pdev->dev, "nand_ck");
	if (IS_ERR(host->clk)) {
		 dev_err(&pdev->dev,"lpc32xx_nand: Clock failure\n");
		res = -ENOENT;
		goto err_exit2;
	}
	clk_enable(host->clk);

	/* Set address of NAND IO lines */
	nand_chip->IO_ADDR_R = SLC_DATA(host->io_base);
	nand_chip->IO_ADDR_W = SLC_DATA(host->io_base);
	nand_chip->cmd_ctrl = lpc32xx_nand_cmd_ctrl;
	nand_chip->dev_ready = lpc32xx_nand_device_ready;
	nand_chip->chip_delay = 20;		/* 20us command delay time */
	nand_chip->read_byte = lpc32xx_read_byte;
	nand_chip->read_buf = lpc32xx_read_buf;
	nand_chip->verify_buf = lpc32xx_verify_buf;
	nand_chip->write_buf = lpc32xx_write_buf;

	/* Init NAND controller */
	lpc32xx_nand_setup(host);
	lpc32xx_wp_disable(host);

	platform_set_drvdata(pdev, host);

	/*
	 * Scan to find existance of the device and
	 * Get the type of NAND device SMALL block or LARGE block
	 */
	if (nand_scan_ident(mtd, 1)) {
		res = -ENXIO;
		goto err_exit3;
	}

	nand_chip->ecc.mode = NAND_ECC_HW;
	nand_chip->ecc.size = mtd->writesize;
	nand_chip->ecc.bytes = (mtd->writesize / 256) * 4;
        nand_chip->ecc.read_page_raw = lpc32xx_nand_read_page_hwecc;
        nand_chip->ecc.read_page = lpc32xx_nand_read_page_hwecc;
        nand_chip->ecc.write_page = lpc32xx_nand_write_page_hwecc;

	switch (mtd->oobsize) {
		case 16:
			nand_chip->ecc.layout = &lpc32xx_nand_oob_16;
			break;
		case 64:
			nand_chip->ecc.layout = &lpc32xx_nand_oob_64;
			break;
		default:
			 dev_err(&pdev->dev, "No oob scheme defined for "
					"oobsize %d\n", mtd->oobsize);
			BUG();
	}

	/* H/W ECC specific functions */
	nand_chip->ecc.hwctl = lpc32xx_ecc_enable;
	nand_chip->ecc.correct = lpc32xx_nand_correct_data;
	nand_chip->ecc.calculate = lpc32xx_ecc_calculate;

	/*
	 * Fills out all the uninitialized function pointers with the defaults
	 * And scans for a bad block table if appropriate.
	 */
	if (nand_scan_tail(mtd)) {
		res = -ENXIO;
		goto err_exit3;
	}

	/* Get free DMA channel and alloc DMA descriptor link list */
	res = lpc32xx_nand_dma_setup(host,((mtd->writesize/128) + 1));
	if(res) {
		res = -EIO;	
		goto err_exit3;
	}

	/* allocate DMA buffer */
	host->dma_buf_len = 
		(/* OOB size area for storing OOB data including ECC */
		 mtd->oobsize + 
		 /* Page Size area for storing Page RAW data */
		 mtd->writesize +
		 /* ECC bytes area for storing Calculated ECC at the time reading page */
		 nand_chip->ecc.bytes);

	host->oob_buf = dmam_alloc_coherent(&pdev->dev, host->dma_buf_len,
			&host->oob_buf_phy, GFP_KERNEL);
	if (host->oob_buf == NULL) {
		dev_err(&pdev->dev, "Unable to allocate DMA memory!\r\n");
		res = -ENOMEM;
		goto err_exit4;
	}

	host->dma_buf = (uint8_t *)host->oob_buf + mtd->oobsize;
	host->ecc_calc_buf = (uint8_t *)host->dma_buf + mtd->writesize;
 
	host->dma_buf_phy = host->oob_buf_phy + mtd->oobsize;
	host->ecc_calc_buf_phy = host->dma_buf_phy + mtd->writesize;

	host->io_base_phy = platform_get_resource(pdev, IORESOURCE_MEM, 0)->start;

	/* 
	 * Allocate a page size buffer to check all 0xFF data
	 * at the time page writting.
	 */
	host->erase_buf_data = kmalloc(mtd->writesize, GFP_KERNEL);
	if (!host->erase_buf_data) {
		 dev_err(&pdev->dev,"lpc32xx_nand: failed to allocate device structure.\n");
		return -ENOMEM;
		goto err_exit5;
	}
        memset(host->erase_buf_data, 0xFF, mtd->writesize);
	init_completion(&host->comp);

#ifdef CONFIG_MTD_PARTITIONS
#ifdef CONFIG_MTD_CMDLINE_PARTS
	mtd->name = "lpc32xx_nand";
	num_partitions = parse_mtd_partitions(mtd, part_probes,
					      &partitions, 0);
#endif
	if ((num_partitions <= 0) && (host->ncfg->partition_info)) {
		partitions = host->ncfg->partition_info(mtd->size,
							 &num_partitions);
	}

	if ((!partitions) || (num_partitions == 0)) {
		 dev_err(&pdev->dev,"lpc32xx_nand: No parititions defined, or unsupported device.\n");
		res = ENXIO;
		goto err_exit6;
	}

	res = add_mtd_partitions(mtd, partitions, num_partitions);
#else
	res = add_mtd_device(mtd);
#endif
	if (!res)
		return res;

	nand_release(mtd);
err_exit6:
	kfree(host->erase_buf_data);
err_exit5:
	dma_free_coherent(&pdev->dev, host->dma_buf_len,
                                host->oob_buf, host->oob_buf_phy);
err_exit4:
	/* Free the DMA channel used by us */
	lpc32xx_dma_ch_disable(host->dmach);
	lpc32xx_dma_dealloc_llist(host->dmach);
	lpc32xx_dma_ch_put(host->dmach);
	host->dmach = -1;
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

	/* Free the DMA channel used by us */
	lpc32xx_dma_ch_disable(host->dmach);
	lpc32xx_dma_dealloc_llist(host->dmach);
	lpc32xx_dma_ch_put(host->dmach);
	host->dmach = -1;

	dma_free_coherent(&pdev->dev, host->dma_buf_len,
                                host->oob_buf, host->oob_buf_phy);
	nand_release(mtd);

	/* Force CE high */
	tmp = __raw_readl(SLC_CTRL(host->io_base));
	tmp &= ~SLCCFG_CE_LOW;
	__raw_writel(tmp, SLC_CTRL(host->io_base));

	lpc32xx_wp_enable(host);
	clk_disable(host->clk);
	clk_put(host->clk);

	iounmap(host->io_base);
	
	kfree(host->erase_buf_data);
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
		.name	= "lpc32xx-nand",
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

