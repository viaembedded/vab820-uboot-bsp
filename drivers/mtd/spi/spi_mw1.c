/* steven: MW1=m25p32, W25Q32FV, WX25L3206E */
#define MW1_WREN 0x06
#define MW1_RDSR 0x05
#define MW1_NORM_READ 0x03
#define MW1_PP 0x02
#define MW1_ERASE_64K 0xD8
#define MW1_ERASE_4K 0x20
#define	mw1_write_enable(a) mw1_spi_nor_cmd_1byte(a, MW1_WREN)
#ifndef SPI_FIFOSIZE
 #define SPI_FIFOSIZE 24
#endif

/* global variables: g_tx_buf, g_rx_buf */


static s32 mw1_spi_nor_status(struct spi_flash *flash)
{
	g_tx_buf[1] = MW1_RDSR;

	if (spi_xfer(flash->spi, 2 << 3, g_tx_buf, g_rx_buf,
			SPI_XFER_BEGIN | SPI_XFER_END) != 0) {
		printf("Error: %s(): %d\n", __func__, __LINE__);
		return 0;
	}
	return g_rx_buf[0];
}

static int mw1_wait_till_ready(struct spi_flash *flash)
{
	int sr;
	int times = 10000;

	do {
		sr = mw1_spi_nor_status(flash);
		if (sr < 0)
			break;
		else if (!(sr & SR_WIP))
			return 0;

		udelay(1000);

	} while (times--);

	return 1;
}

static s32 mw1_spi_nor_cmd_1byte(struct spi_flash *flash, u8 cmd)
{
	g_tx_buf[0] = cmd;
	g_tx_buf[1] = 0;
	g_tx_buf[2] = 0;
	g_tx_buf[3] = 0;

	if (spi_xfer(flash->spi, (1 << 3), g_tx_buf, g_rx_buf,
			SPI_XFER_BEGIN | SPI_XFER_END) != 0) {
		printf("Error: %s(): %d\n", __func__, __LINE__);
		return -1;
	}
	return 0;
}


static int mw1_spi_nor_flash_read(struct spi_flash *flash, u32 from,
		size_t len, void *buf)
{
	struct imx_spi_flash *imx_sf = to_imx_spi_flash(flash);
	int rx_len = 0, count = 0, i = 0;
	int addr, cmd_len;
	u8 txer[SPI_FIFOSIZE] = { 0 };
	u8 *s = txer;
	u8 *d = buf;

	debug("%s: %s 0x%08x, len %zd\n", __func__, "from", (u32)from, len);

	/* sanity checks */
	if (!len)
		return 0;

	if (from + len > imx_sf->params->device_size)
		return -EINVAL;

	/* Wait till previous write/erase is done. */
	if (mw1_wait_till_ready(flash)) {
		/* REVISIT status return?? */
		return 1;
	}

	printf("Reading SPI NOR flash 0x%x [0x%x bytes] -> ram 0x%p\n",
		from, len, buf);

	cmd_len = 4;

	addr = from;

	while (len > 0) {

		rx_len = len > (SPI_FIFOSIZE - cmd_len) ?
		    SPI_FIFOSIZE - cmd_len : len;

		/* Set up the write data buffer. */
		txer[3] = MW1_NORM_READ;
		txer[2] = addr >> 16;
		txer[1] = addr >> 8;
		txer[0] = addr;

		if (spi_xfer(flash->spi, (roundup(rx_len, 4) + cmd_len) << 3,
			txer, txer,
			SPI_XFER_BEGIN | SPI_XFER_END) != 0) {
			printf("Error: %s(%d): failed\n",
				__FILE__, __LINE__);
			return -1;
		}

		s = txer + cmd_len;

		for (i = rx_len; i >= 0; i -= 4, s += 4) {
			if (i < 4) {
				if (i == 1) {
					*d = s[3];
				} else if (i == 2) {
					*d++ = s[3];
					*d++ = s[2];
				} else if (i == 3) {
					*d++ = s[3];
					*d++ = s[2];
					*d++ = s[1];
				}

				break;
			}

			*d++ = s[3];
			*d++ = s[2];
			*d++ = s[1];
			*d++ = s[0];
		}

		/* updaate */
		len -= rx_len;
		addr += rx_len;
		count += rx_len;

		if ((len % imx_sf->params->block_size) == 0)
			printf(".");
		debug("%s: left:0x%x, from:0x%08x, to:0x%p, done: 0x%x\n",
		      __func__, len, (u32) addr, d, count);
	}

	printf("SUCCESS\n\n");
	return 0;
}



static int _mw1_fsl_spi_write(struct spi_flash *flash, const void *buf, int len, int addr)
{
	u8 txer[SPI_FIFOSIZE] = { 0 };
	u8 *d = txer;
	u8 *s = (u8 *) buf;
	int delta = 0, l = 0, i = 0, count = 0;

	count = len;
	delta = count % 4;
	if (delta)
		count -= delta;

	while (count) {
		d = txer;
		l = count > (SPI_FIFOSIZE - 4) ?
		    SPI_FIFOSIZE - 4 : count;

		d[3] = MW1_PP;
		d[2] = addr >> 16;
		d[1] = addr >> 8;
		d[0] = addr;

		for (i = 0, d += 4; i < l / 4; i++, d += 4) {
			d[3] = *s++;
			d[2] = *s++;
			d[1] = *s++;
			d[0] = *s++;
		}

		debug("WRITEBUF: (%x) %x %x %x\n",
		      txer[3], txer[2], txer[1], txer[0]);

		mw1_wait_till_ready(flash);

		mw1_write_enable(flash);

		if (spi_xfer(flash->spi, (l + 4) << 3,
				txer, txer,
				SPI_XFER_BEGIN | SPI_XFER_END) != 0) {
			printf("Error: %s(%d): failed\n",
				__FILE__, __LINE__);
			return -1;
		}

		/* update */
		count -= l;
		addr += l;
	}

	if (delta) {
		d = txer;
		/* to keep uninterested bytes untouched */
		for (i = 4; i < 8; i++)
			d[i] = 0xff;

		d[3] = MW1_PP;
		d[2] = (addr >> 16) & 0xff;
		d[1] = (addr >> 8) & 0xff;
		d[0] = (addr) & 0xff;

		switch (delta) {
		case 1:
			d[7] = *s++;
			break;
		case 2:
			d[7] = *s++;
			d[6] = *s++;
			break;
		case 3:
			d[7] = *s++;
			d[6] = *s++;
			d[5] = *s++;
			break;
		default:
			break;
		}

		debug("WRITEBUF: (%x) %x %x %x\n",
		      txer[3], txer[2], txer[1], txer[0]);

		mw1_wait_till_ready(flash);

		mw1_write_enable(flash);

		if (spi_xfer(flash->spi, (4 + 4) << 3,
				txer, txer,
				SPI_XFER_BEGIN | SPI_XFER_END) != 0) {
			printf("Error: %s(%d): failed\n",
				__FILE__, __LINE__);
			return -1;
		}
	}

	return len;
}

static int mw1_spi_nor_flash_write(struct spi_flash *flash, u32 to,
		size_t len, const void *buf)
{
	struct imx_spi_flash *imx_sf = to_imx_spi_flash(flash);
	u32 page_offset, page_size;

	/* sanity checks */
	if (!len)
		return 0;

	printf("Writing SPI NOR flash 0x%x [0x%x bytes] <- ram 0x%p\n",
		to, len, buf);

	if (to + len > imx_sf->params->device_size)
		return -EINVAL;

	/* Wait until finished previous write command. */
	if (mw1_wait_till_ready(flash))
		return 1;

	mw1_write_enable(flash);

	page_offset = to & (imx_sf->params->page_size - 1);

	/* do all the bytes fit onto one page? */
	if (page_offset + len <= imx_sf->params->page_size) {
		_mw1_fsl_spi_write(flash, buf, len, to);

	} else {
		u32 i;

		/* the size of data remaining on the first page */
		page_size = imx_sf->params->page_size - page_offset;

		_mw1_fsl_spi_write(flash, buf, page_size, to);

		/* write everything in flash->page_size chunks */
		for (i = page_size; i < len; i += page_size) {
			page_size = len - i;
			if (page_size > imx_sf->params->page_size)
				page_size = imx_sf->params->page_size;

			mw1_wait_till_ready(flash);

			mw1_write_enable(flash);

			_mw1_fsl_spi_write(flash, buf + i, page_size, to + i);
			if (page_size % imx_sf->params->block_size == 0)
				printf(".");
		}
	}

	printf("SUCCESS\n\n");

	return 0;
}

static int mw1_erase_sector(struct spi_flash *flash, u32 offset)
{
	/* Wait until finished previous write command. */
	if (mw1_wait_till_ready(flash))
		return 1;

	/* Send write enable, then erase commands. */
	mw1_write_enable(flash);

	/* Set up command buffer. */
	g_tx_buf[3] = MW1_ERASE_64K;
	g_tx_buf[2] = offset >> 16;
	g_tx_buf[1] = offset >> 8;
	g_tx_buf[0] = offset;

	if (spi_xfer(flash->spi, (4 << 3), g_tx_buf, g_rx_buf,
				SPI_XFER_BEGIN | SPI_XFER_END)) {
		return -1;
	}

	return 0;
}

static int mw1_spi_nor_flash_erase(struct spi_flash *flash, u32 offset,
		size_t len)
{
	struct imx_spi_flash *imx_sf = to_imx_spi_flash(flash);

	/* whole-chip erase? */
	/*
	if (len == imx_sf->params->device_size) {
		if (erase_chip(flash))
			return -EIO;
		else
			return 0;
	*/

	/* REVISIT in some cases we could speed up erasing large regions
	 * by using MW1_ERASE_64K instead of MW1_ERASE_4K.  We may have set up
	 * to use "small sector erase", but that's not always optimal.
	 */

	printf("Erasing SPI NOR flash 0x%x [0x%x bytes]\n",
		offset, len);

	/* "sector"-at-a-time erase */
	len = roundup(len, imx_sf->params->block_size);
	while (len) {
		if (mw1_erase_sector(flash, offset))
			return -EIO;

		offset += imx_sf->params->block_size;
		len -= imx_sf->params->block_size;
		printf(".");
	}

	printf("SUCCESS\n\n");
	return 0;
}
