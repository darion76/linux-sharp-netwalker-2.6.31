/*
 * Copyright 2004-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 * (c) 2005 MontaVista Software, Inc.
 *
 * This code is based on m25p80.c by adding FSL spi access.
 *
 * modification information
 * ------------------------
 * 2009/07/01 : W25Q16BV support.
 * 2009/07/03 : support AAI-not-support device.
 *              CHECK_SPI_FIFOSIZE add.
 * 2009/08/02 : mxspinor_read() printk -> DEBUG(MTD_DEBUG_LEVEL2.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#define FLASH_PAGESIZE		256
#define	SPI_FIFOSIZE		8	/* Bust size in bytes */
#ifdef CONFIG_MACH_MX51_ERDOS
#define	CHECK_SPI_FIFOSIZE	(32)
#endif /* CONFIG_MACH_MX51_ERDOS */

/* Flash opcodes. */
#define	OPCODE_WREN		0x06	/* Write enable */
#define	OPCODE_RDSR		0x05	/* Read status register */
#define	OPCODE_NORM_READ	0x03	/* Read data bytes (low frequency) */
#define	OPCODE_FAST_READ	0x0b	/* Read data bytes (high frequency) */
#define	OPCODE_PP		0x02	/* Page program (up to 256 bytes) */
#define	OPCODE_BE_4K 		0x20	/* Erase 4KiB block */
#define	OPCODE_BE_32K		0x52	/* Erase 32KiB block */
#define	OPCODE_SE		0xd8	/* Sector erase (usually 64KiB) */
#define	OPCODE_RDID		0x9f	/* Read JEDEC ID */
#define OPCODE_WRSR		0x01	/* write status register */
#define OPCODE_EWSR		0x50	/* enable write status register */
#define OPCODE_AAI_PROG		0xad	/* Auto Address Increment (AAI) Word-Program */
#define OPCODE_WRDI		0x04	/* Write disable */

/* Status Register bits. */
#define	SR_WIP			1	/* Write in progress */
#define	SR_WEL			2	/* Write enable latch */
/* meaning of other SR_* bits may differ between vendors */
#define	SR_BP0			4	/* Block protect 0 */
#define	SR_BP1			8	/* Block protect 1 */
#define	SR_BP2			0x10	/* Block protect 2 */
#define	SR_SRWD			0x80	/* SR write protect */

/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_COUNT	100000
#define	CMD_SIZE		4

#ifdef CONFIG_MTD_PARTITIONS
#define	mtd_has_partitions()	(1)
#else
#define	mtd_has_partitions()	(0)
#endif

#ifdef CONFIG_MACH_MX51_ERDOS
struct mtd_info *mxc_spi_nor_mtd = NULL;
EXPORT_SYMBOL(mxc_spi_nor_mtd);
#endif

/****************************************************************************/

struct mxspinor {
	struct spi_device *spi;
	struct mutex lock;
	struct mtd_info mtd;
	unsigned partitioned:1;
	u8 erase_opcode;
#ifdef CONFIG_MACH_MX51_ERDOS
	u8 unsupport_cmd_AAI;	/* UNsupport Auto Address Increment */
#endif /* CONFIG_MACH_MX51_ERDOS */
};

static inline struct mxspinor *mtd_to_mxspinor(struct mtd_info *mtd)
{
	return container_of(mtd, struct mxspinor, mtd);
}

/*
 * This function initializes the SPI device parameters.
 */
static inline int spi_nor_setup(struct spi_device *spi, u8 bst_len)
{
	spi->bits_per_word = bst_len << 3;

	return spi_setup(spi);
}

/*
 * This function perform spi read/write transfer.
 */
static int spi_read_write(struct spi_device *spi, u8 * buf, u32 len)
{
	struct spi_message m;
	struct spi_transfer t;

#ifdef CONFIG_MACH_MX51_ERDOS
	if (len > CHECK_SPI_FIFOSIZE || len <= 0)
#else
	if (len > SPI_FIFOSIZE || len <= 0)
#endif /* CONFIG_MACH_MX51_ERDOS */
		return -1;

	spi_nor_setup(spi, len);

	spi_message_init(&m);
	memset(&t, 0, sizeof t);

	t.tx_buf = buf;
	t.rx_buf = buf;
	t.len = ((len - 1) >> 2) + 1;

	spi_message_add_tail(&t, &m);

	if (spi_sync(spi, &m) != 0 || m.status != 0){
		printk("%s: error\n", __func__); 	
		return -1;
	}

	DEBUG(MTD_DEBUG_LEVEL2,"%s: len: 0x%x success\n", __func__, len);

	return 0;

}

/*
 * Enable write status regist.
 * Returns negative if error occurred.
 */
static inline int spi_nor_enable_wrsr(struct mxspinor *flash)
{
        u8 code = OPCODE_EWSR;

        return spi_read_write(flash->spi, &code, 1);
} 


/*
 * Read the status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int spi_nor_read_status(struct mxspinor *flash)
{
	ssize_t retval;

	u16 val = OPCODE_RDSR << 8;

	retval = spi_read_write(flash->spi, (u8 *) & val, 2);

	if (retval < 0) {
		dev_err(&flash->spi->dev, "error %d reading SR\n", (int)retval);
		return retval;
	}

	DEBUG(MTD_DEBUG_LEVEL2,"%s: status: 0x%x\n", __func__, val & 0xff);

	return val & 0xff;
}

/*
 * write stuatus regist.
 * Returns negative if error occurred.
 */
static inline int spi_nor_write_status(struct mxspinor *flash, u8 flag)
{
	u16 data = OPCODE_WRSR << 8 | flag;

	spi_nor_enable_wrsr(flash);

	return spi_read_write(flash->spi, (u8 *) & data, 2);
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static inline int spi_nor_write_enable(struct mxspinor *flash)
{
	u8 code = OPCODE_WREN;

	return spi_read_write(flash->spi, &code, 1);
}

/*
 * Set write disable latch with Write disable command.
 * Returns negative if error occurred.
 */
static inline int spi_nor_write_disable(struct mxspinor *flash)
{
	u8 code = OPCODE_WRDI;

	return spi_read_write(flash->spi, &code, 1);
} 


/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int wait_till_ready(struct mxspinor *flash)
{
	int count;
	int sr;

	/* one chip guarantees max 5 msec wait here after page writes,
	 * but potentially three seconds (!) after page erase.
	 */
	for (count = 0; count < MAX_READY_WAIT_COUNT; count++) {
		if ((sr = spi_nor_read_status(flash)) < 0)
			break;
		else if (!(sr & SR_WIP))
			return 0;

	DEBUG(MTD_DEBUG_LEVEL3, "%s: sr is %x,retry %d\n", __func__, sr, count);

		/* REVISIT sometimes sleeping would be best */
	}

	return 1;
}

/*
 * Erase one sector of flash memory at offset ``offset'' which is any
 * address within the sector which should be erased.
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_sector(struct mxspinor *flash, u32 offset)
{

	u32 cmd = flash->erase_opcode << 24 | offset;

	DEBUG(MTD_DEBUG_LEVEL3, "%s: %s %dKiB at 0x%08x\n",
	      dev_name(&flash->spi->dev), __func__,
	      flash->mtd.erasesize / 1024, offset);

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return 1;

	spi_nor_write_status(flash, 0);

	/* Send write enable, then erase commands. */
	spi_nor_write_enable(flash);

	spi_read_write(flash->spi, (u8 *) & cmd, CMD_SIZE);

	return 0;
}

/*
 * MTD implementation
 */

/*
 * Erase an address range on the flash chip.  The address range may extend
 * one or more erase sectors.  Return an error is there is a problem erasing.
 */
static int mxspinor_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct mxspinor *flash = mtd_to_mxspinor(mtd);
	u32 addr, len, tmp;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%08x, len %d\n",
	      dev_name(&flash->spi->dev), __func__, "at",
	      (u32) instr->addr, instr->len);

	/* sanity checks */
	if (instr->addr + instr->len > flash->mtd.size)
		return -EINVAL;

/*	if ((instr->addr % mtd->erasesize) != 0
	    || (instr->len % mtd->erasesize) != 0) {
		return -EINVAL;
	}*/
	tmp = do_div(instr->addr, mtd->erasesize);
	if (tmp*mtd->erasesize != instr->addr) return -EINVAL;
	
	tmp = do_div(instr->len, mtd->erasesize);
	if (tmp*mtd->erasesize != instr->addr) return -EINVAL;
	
	addr = instr->addr;
	len = instr->len;

	mutex_lock(&flash->lock);

	/* REVISIT in some cases we could speed up erasing large regions
	 * by using OPCODE_SE instead of OPCODE_BE_4K
	 */

	/* now erase those sectors */
	while (len) {
		if (erase_sector(flash, addr)) {
			instr->state = MTD_ERASE_FAILED;
			mutex_unlock(&flash->lock);
			return -EIO;
		}
//		asm("" : "+r"(addr)); // Prevent wrong optimization
//		asm("" : "+r"(len)); // Prevent wrong optimization
		addr += mtd->erasesize;
		len -= mtd->erasesize;
	}


	mutex_unlock(&flash->lock);

	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);


	return 0;
}

/*
 * Read an address range from the flash chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static int mxspinor_read(struct mtd_info *mtd, loff_t from, size_t len,
			 size_t * retlen, u_char * buf)
{
	struct mxspinor *flash = mtd_to_mxspinor(mtd);
	int rx_len = 0, count = 0, i = 0;
	u_char txer[SPI_FIFOSIZE];
	u_char *s = txer;
	u_char *d = buf;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%08x, to 0x%p, len %zd\n",
	      dev_name(&flash->spi->dev), __func__, "from", (u32) from, buf, len);

	/* sanity checks */
	if (!len)
		return 0;

	if (from + len > flash->mtd.size)
		return -EINVAL;

	if (retlen)
		*retlen = 0;

	mutex_lock(&flash->lock);

	while (len > 0) {

		rx_len = len > (SPI_FIFOSIZE - CMD_SIZE) ?
		    SPI_FIFOSIZE - CMD_SIZE : len;

		/* Wait till previous write/erase is done. */
		if (wait_till_ready(flash)) {

			DEBUG(MTD_DEBUG_LEVEL2,"%s: error\n", __func__); 
			/* REVISIT status return?? */
			mutex_unlock(&flash->lock);
			return 1;
		}
		
		if ((rx_len & 0x3) == 1) {
			DEBUG(MTD_DEBUG_LEVEL2,"%d: only %d bytes left\n", 
							1, rx_len);
			txer[0] = OPCODE_NORM_READ;
			txer[7] = (from >> 16) & 0xff;
			txer[6] = (from >> 8) & 0xff;
			txer[5] = from & 0xff;
 		} else if ((rx_len & 0x3) == 2) {
			DEBUG(MTD_DEBUG_LEVEL2,"%d: only %d bytes left\n", 
							2, rx_len);
			txer[1] = OPCODE_NORM_READ;
			txer[0] = (from >> 16) & 0xff;
			txer[7] = (from >> 8) & 0xff;
			txer[6] = from & 0xff;
		} else if ((rx_len & 0x3) == 3) {
			DEBUG(MTD_DEBUG_LEVEL2,"%d: only %d bytes left\n",
							3, rx_len);
			txer[2] = OPCODE_NORM_READ;
			txer[1] = (from >> 16) & 0xff;
			txer[0] = (from >> 8) & 0xff;
			txer[7] = from & 0xff;
		} else {
			txer[3] = OPCODE_NORM_READ;
			txer[2] = (from >> 16) & 0xff;
			txer[1] = (from >> 8) & 0xff;
			txer[0] = from & 0xff;
		}
 
		if (spi_read_write(flash->spi, txer, rx_len + 4)) {
			mutex_unlock(&flash->lock);
			return 1;
		}

		s = txer + CMD_SIZE;

		if (rx_len >= (SPI_FIFOSIZE - 4)) {
			for (i = 0; i < (SPI_FIFOSIZE - 4); i += 4, s += 4) {
				*d++ = s[3];
				*d++ = s[2];
				*d++ = s[1];
				*d++ = s[0];
			}
		} else {
			for (i = rx_len; i >= 0; i -= 4, s += 4) {
				if (i < 4) {
					if (i == 1) {
						*d = s[0];
					} else if (i == 2) {
						*d++ = s[1];
						*d++ = s[0];
					} else if (i == 3) {
						*d++ = s[2];
						*d++ = s[1];
						*d++ = s[0];
					}

					break;
				}

				*d++ = s[3];
				*d++ = s[2];
				*d++ = s[1];
				*d++ = s[0];
			}
		}

		len -= rx_len;
		from += rx_len;
		count += rx_len;

		DEBUG(MTD_DEBUG_LEVEL2,"%s: left:0x%x, from:0x%08x, to:0x%p,  done: 0x%x\n",
				 __func__, len, (u32)from, d,  count);
	}

	*retlen = count;

	DEBUG(MTD_DEBUG_LEVEL2,"%s: %d bytes read\n", __func__, count);

	mutex_unlock(&flash->lock);

	return 0;
}

/*
 * Write an address range to the flash chip.  Data must be written in
 * FLASH_PAGESIZE chunks.  The address range may be any size provided
 * it is within the physical boundaries.
 */
static int mxspinor_write(struct mtd_info *mtd, loff_t to, size_t len,
			  size_t * retlen, const u_char * buf)
{
	struct mxspinor *flash = mtd_to_mxspinor(mtd);
	u8 cmd[8];
	int i = 0;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%08x, len %zd\n",
	      dev_name(&flash->spi->dev), __func__, "to", (u32) to, len);

	if (retlen)
		*retlen = 0;

	/* sanity checks */
	if (!len)
		return (0);

	if (to + len > flash->mtd.size)
		return -EINVAL;

	mutex_lock(&flash->lock);

	spi_nor_write_status(flash, 0);

#ifdef CONFIG_MACH_MX51_ERDOS
	/*
	 * check UNsupported AAI
	 */
	if (flash->unsupport_cmd_AAI == 1) {
		int s_ofs = 0;
		u8  cmdu [32];
		/*
		 * SPI write data ex)10bytes (data 6bytes)
		 *      +--------+--------+--------+--------+
		 *   +0 |   -    |    -   |   cmd  | Adr(H) |
		 *      +--------+--------+--------+--------+
		 *   +4 | Adr(M) | Adr(L) | Data[0]| Data[1]|
		 *      +--------+--------+--------+--------+
		 *   +8 | Data[2]| Data[3]| Data[4]| Data[5]| *must fill 4bytes
		 *      +--------+--------+--------+--------+
		 */
		while ( 0 < len ) {
			int          slen, sz;
			unsigned int val;
			if (16 <= len) {
				sz = 16;
			} else {
				sz = len;
			}
			switch ( sz ) {
			case 16:
			case 12:
			case 8:
			case 4:
				/*
				 * cmd, address
				 */
				val = (OPCODE_PP << 24) | (to & 0x00FFFFFF);
				memcpy (&(cmdu [0]), &val, 4);
				/*
				 * data
				 */
				val = (buf [s_ofs + 0] << 24) | (buf [s_ofs + 1] << 16) |
				      (buf [s_ofs + 2] <<  8) | (buf [s_ofs + 3]);
				memcpy (&(cmdu [4]), &val, 4);
				slen = 4;
				break;
			case 15:
			case 11:
			case 7:
			case 3:
				/*
				 * cmd, address
				 */
				val = (OPCODE_PP << 16) | ((to & 0x00FFFF00) >> 8);
				memcpy (&(cmdu [0]), &val, 4);
				/*
				 * address, data
				 */
				val = ((to & 0x000000FF) << 24) | (buf [s_ofs + 0] << 16)|
				      (buf [s_ofs + 1] <<  8)   | (buf [s_ofs + 2]);
				memcpy (&(cmdu [4]), &val, 4);
				slen = 3;
				break;
			case 14:
			case 10:
			case 6:
			case 2:
				/*
				 * cmd, address
				 */
				val = (OPCODE_PP << 8) | ((to & 0x00FF0000) >> 16);
				memcpy (&(cmdu [0]), &val, 4);
				/*
				 * address, data
				 */
				val = ((to & 0x0000FFFF) << 16) |
				      (buf [s_ofs + 0] <<  8)   | (buf [s_ofs + 1]);
				memcpy (&(cmdu [4]), &val, 4);
				slen = 2;
				break;
			default:
				/*
				 * cmd
				 */
				val = OPCODE_PP;
				memcpy (&(cmdu [0]), &val, 4);
				/*
				 * address, data
				 */
				val = ((to & 0x00FFFFFF) << 8) | buf [s_ofs + 0];
				memcpy (&(cmdu [4]), &val, 4);
				slen = 1;
				break;
			}
			while ( slen < sz ) {
				val = (buf [s_ofs + slen + 0] << 24) |
				      (buf [s_ofs + slen + 1] << 16) |
				      (buf [s_ofs + slen + 2] <<  8) |
				      (buf [s_ofs + slen + 3]);
				slen += 4;
				memcpy (&(cmdu [((slen + 3) / 4) * 4]), &val, 4);
			}

			/* Wait until finished previous write command. */
                	if (wait_till_ready(flash)) {
                        	mutex_unlock(&flash->lock);
                        	return 1;
                	}

			spi_nor_write_enable(flash);
			if (spi_read_write(flash->spi, (u8 *)cmdu, (4 + slen))) {
				mutex_unlock(&flash->lock);
				return 1;
			}

			/*
			 * write execute
			 */
			spi_nor_write_status(flash, 0);

			/*
			 * next data offset
			 */
			len    -= slen;
			s_ofs  += slen;
			to     += slen;
		}
		*retlen = s_ofs;
		mutex_unlock(&flash->lock);
		return 0;
	}
#endif /* CONFIG_MACH_MX51_ERDOS */
	

	if (len > 1) {

		/* Wait until finished previous write command. */
                if (wait_till_ready(flash)) {
                        mutex_unlock(&flash->lock);
                        return 1;
                }

		cmd[0] = (to >> 16) & 0xff;
		cmd[1] = OPCODE_AAI_PROG;
		cmd[4] = buf[1];
		cmd[5] = buf[0];
		cmd[6] = to & 0xff;
		cmd[7] = (to >> 8) & 0xff;

		spi_nor_write_enable(flash);

		if (spi_read_write(flash->spi, (u8 *) cmd, 6)) {
			mutex_unlock(&flash->lock);
			return 1;
		}

		i += 2;

	}

	DEBUG(MTD_DEBUG_LEVEL2,"%s: left:0x%x, done:0x%x\n",
				 __func__, len - i , i);

	for (; len - i > 1; i += 2) {

		/* Wait until finished previous write command. */
		if (wait_till_ready(flash)) {
			mutex_unlock(&flash->lock);
			return 1;
		}

		cmd[0] = buf[i + 1];
		cmd[1] = buf[i];
		cmd[2] = OPCODE_AAI_PROG;

		if (spi_read_write(flash->spi, (u8 *) cmd, 3)) {
			mutex_unlock(&flash->lock);
			return 1;
		}

	}

	DEBUG(MTD_DEBUG_LEVEL2,"%s: left:0x%x, done:0x%x\n",
					 __func__, len - i, i);

	spi_nor_write_disable(flash);

	if (len - i == 1) {

		/* Wait until finished previous write command. */
		if (wait_till_ready(flash)) {
			mutex_unlock(&flash->lock);
			return 1;
		}

		to += i;

		cmd[0] = OPCODE_PP;
		cmd[4] = buf[i];
		cmd[5] = to & 0xff;
		cmd[6] = (to >> 8) & 0xff;
		cmd[7] = (to >> 16) & 0xff;
	
		spi_nor_write_enable(flash);

		if (spi_read_write(flash->spi, (u8 *) cmd, 5)) {
			mutex_unlock(&flash->lock);
			 return 1;
		}

		i++;
	}

	DEBUG(MTD_DEBUG_LEVEL2,"%s: left:0x%x, done:0x%x\n",
					 __func__, len - i, i); 

	*retlen = i;
	mutex_unlock(&flash->lock);

	return 0;
}

/*
 * SPI device driver setup and teardown
 */

struct flash_info {
	char *name;

	/* JEDEC id zero means "no ID" (most older chips); otherwise it has
	 * a high byte of zero plus three data bytes: the manufacturer id,
	 * then a two byte device id.
	 */
	u32 jedec_id;

	/* The size listed here is what works with OPCODE_SE, which isn't
	 * necessarily called a "sector" by the vendor.
	 */
	unsigned sector_size;
	u16 n_sectors;

	u16 flags;
#define	SECT_4K		0x01	/* OPCODE_BE_4K works uniformly */
#ifdef CONFIG_MACH_MX51_ERDOS
	u16 unsupport_cmd_AAI;	/* UNsupport Auto Address Increment */
#endif /* CONFIG_MACH_MX51_ERDOS */
};

/* NOTE: double check command sets and memory organization when you add
 * more flash chips.  This current list focusses on newer chips, which
 * have been converging on command sets which including JEDEC ID.
 */
static struct flash_info __devinitdata mxspinor_data[] = {

	/* Atmel -- some are (confusingly) marketed as "DataFlash" */
	{"at25fs010", 0x1f6601, 32 * 1024, 4, SECT_4K,},
	{"at25fs040", 0x1f6604, 64 * 1024, 8, SECT_4K,},

	{"at25df041a", 0x1f4401, 64 * 1024, 8, SECT_4K,},
	{"at25df641", 0x1f4800, 64 * 1024, 128, SECT_4K,},

	{"at26f004", 0x1f0400, 64 * 1024, 8, SECT_4K,},
	{"at26df081a", 0x1f4501, 64 * 1024, 16, SECT_4K,},
	{"at26df161a", 0x1f4601, 64 * 1024, 32, SECT_4K,},
	{"at26df321", 0x1f4701, 64 * 1024, 64, SECT_4K,},

	/* Spansion -- single (large) sector size only, at least
	 * for the chips listed here (without boot sectors).
	 */
	{"s25sl004a", 0x010212, 64 * 1024, 8,},
	{"s25sl008a", 0x010213, 64 * 1024, 16,},
	{"s25sl016a", 0x010214, 64 * 1024, 32,},
	{"s25sl032a", 0x010215, 64 * 1024, 64,},
	{"s25sl064a", 0x010216, 64 * 1024, 128,},

	/* SST -- large erase sizes are "overlays", "sectors" are 4K */
	{"sst25vf040b", 0xbf258d, 64 * 1024, 8, SECT_4K,},
	{"sst25vf080b", 0xbf258e, 64 * 1024, 16, SECT_4K,},
	{"sst25vf016b", 0xbf2541, 64 * 1024, 32, SECT_4K,},
	{"sst25vf032b", 0xbf254a, 64 * 1024, 64, SECT_4K,},

	/* ST Microelectronics -- newer production may have feature updates */
	{"m25p05", 0x202010, 32 * 1024, 2,},
	{"m25p10", 0x202011, 32 * 1024, 4,},
	{"m25p20", 0x202012, 64 * 1024, 4,},
	{"m25p40", 0x202013, 64 * 1024, 8,},
	{"m25p80", 0, 64 * 1024, 16,},
	{"m25p16", 0x202015, 64 * 1024, 32,},
	{"m25p32", 0x202016, 64 * 1024, 64,},
	{"m25p64", 0x202017, 64 * 1024, 128,},
	{"m25p128", 0x202018, 256 * 1024, 64,},

	{"m45pe80", 0x204014, 64 * 1024, 16,},
	{"m45pe16", 0x204015, 64 * 1024, 32,},

	{"m25pe80", 0x208014, 64 * 1024, 16,},
	{"m25pe16", 0x208015, 64 * 1024, 32, SECT_4K,},

	/* Winbond -- w25x "blocks" are 64K, "sectors" are 4KiB */
	{"w25x10", 0xef3011, 64 * 1024, 2, SECT_4K,},
	{"w25x20", 0xef3012, 64 * 1024, 4, SECT_4K,},
	{"w25x40", 0xef3013, 64 * 1024, 8, SECT_4K,},
	{"w25x80", 0xef3014, 64 * 1024, 16, SECT_4K,},
	{"w25x16", 0xef3015, 64 * 1024, 32, SECT_4K,},
	{"w25x32", 0xef3016, 64 * 1024, 64, SECT_4K,},
	{"w25x64", 0xef3017, 64 * 1024, 128, SECT_4K,},
	{"w25q16", 0xef4015, 64 * 1024, 32, SECT_4K, 1, },
};

static struct flash_info *__devinit jedec_probe(struct spi_device *spi)
{
	int tmp;
	u32 code = OPCODE_RDID << 24;
	u32 jedec;
	struct flash_info *info;

	/* JEDEC also defines an optional "extended device information"
	 * string for after vendor-specific data, after the three bytes
	 * we use here.  Supporting some chips might require using it.
	 */
	tmp = spi_read_write(spi, (u8 *) & code, 4);
	if (tmp < 0) {
		DEBUG(MTD_DEBUG_LEVEL0, "%s: error %d reading JEDEC ID\n",
		      dev_name(&spi->dev), tmp);
		return NULL;
	}

	jedec = code & 0xFFFFFF;

	for (tmp = 0, info = mxspinor_data;
	     tmp < ARRAY_SIZE(mxspinor_data); tmp++, info++) {
		if (info->jedec_id == jedec)
			return info;
	}
	dev_err(&spi->dev, "unrecognized JEDEC id %06x\n", jedec);
	return NULL;

}

/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int __devinit mxspinor_probe(struct spi_device *spi)
{
	struct flash_platform_data *data;
	struct mxspinor *flash;
	struct flash_info *info;
	unsigned i;

	/* Platform data helps sort out which chip type we have, as
	 * well as how this board partitions it.  If we don't have
	 * a chip ID, try the JEDEC id commands; they'll work for most
	 * newer chips, even if we don't recognize the particular chip.
	 */

	data = spi->dev.platform_data;
	if (data && data->type) {
		for (i = 0, info = mxspinor_data;
		     i < ARRAY_SIZE(mxspinor_data); i++, info++) {
			if (strcmp(data->type, info->name) == 0)
				break;
		}

		/* unrecognized chip? */
		if (i == ARRAY_SIZE(mxspinor_data)) {
			DEBUG(MTD_DEBUG_LEVEL0, "%s: unrecognized id %s\n",
			      dev_name(&spi->dev), data->type);
			info = NULL;

			/* recognized; is that chip really what's there? */
		} else if (info->jedec_id) {
			struct flash_info *chip = jedec_probe(spi);

			if (!chip || chip != info) {
				dev_warn(&spi->dev, "found %s, expected %s\n",
					 chip ? chip->name : "UNKNOWN",
					 info->name);
				info = NULL;
			}
		}
	} else
		info = jedec_probe(spi);

	if (!info)
		return -ENODEV;

	flash = kzalloc(sizeof *flash, GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	flash->spi = spi;
	mutex_init(&flash->lock);
	dev_set_drvdata(&spi->dev, flash);

	/* unlock the proctection */
	spi_nor_write_status(flash, 0);

	if (data && data->name)
		flash->mtd.name = data->name;
	else
		flash->mtd.name = dev_name(&spi->dev);

	flash->mtd.type = MTD_NORFLASH;
	flash->mtd.writesize = 1;
	flash->mtd.flags = MTD_CAP_NORFLASH;
	flash->mtd.size = info->sector_size * info->n_sectors;
	flash->mtd.erase = mxspinor_erase;
	flash->mtd.read = mxspinor_read;
	flash->mtd.write = mxspinor_write;

#ifdef CONFIG_MACH_MX51_ERDOS
	flash->unsupport_cmd_AAI = info->unsupport_cmd_AAI;
#endif /* CONFIG_MACH_MX51_ERDOS */

	/* prefer "small sector" erase if possible */
	if (info->flags & SECT_4K) {
		flash->erase_opcode = OPCODE_BE_4K;
		flash->mtd.erasesize = 4096;
	} else {
		flash->erase_opcode = OPCODE_SE;
		flash->mtd.erasesize = info->sector_size;
	}

	dev_info(&spi->dev, "%s (%d Kbytes)\n", info->name,
		 flash->mtd.size / 1024);

	DEBUG(MTD_DEBUG_LEVEL2,
	      "mtd .name = %s, .size = 0x%.8x (%uMiB) "
	      ".erasesize = 0x%.8x (%uKiB) .numeraseregions = %d\n",
	      flash->mtd.name,
	      flash->mtd.size, flash->mtd.size / (1024 * 1024),
	      flash->mtd.erasesize, flash->mtd.erasesize / 1024,
	      flash->mtd.numeraseregions);

	if (flash->mtd.numeraseregions)
		for (i = 0; i < flash->mtd.numeraseregions; i++)
			DEBUG(MTD_DEBUG_LEVEL2,
			      "mtd.eraseregions[%d] = { .offset = 0x%.8x, "
			      ".erasesize = 0x%.8x (%uKiB), "
			      ".numblocks = %d }\n",
			      i, flash->mtd.eraseregions[i].offset,
			      flash->mtd.eraseregions[i].erasesize,
			      flash->mtd.eraseregions[i].erasesize / 1024,
			      flash->mtd.eraseregions[i].numblocks);

	/* partitions should match sector boundaries; and it may be good to
	 * use readonly partitions for writeprotected sectors (BP2..BP0).
	 */
	if (mtd_has_partitions()) {
		struct mtd_partition *parts = NULL;
		int nr_parts = 0;

#ifdef CONFIG_MTD_CMDLINE_PARTS
		static const char *part_probes[] = { "cmdlinepart", NULL, };

		nr_parts = parse_mtd_partitions(&flash->mtd,
						part_probes, &parts, 0);
#endif

		if (nr_parts <= 0 && data && data->parts) {
			parts = data->parts;
			nr_parts = data->nr_parts;
		}

		if (nr_parts > 0) {
			for (i = 0; i < nr_parts; i++) {
				DEBUG(MTD_DEBUG_LEVEL2, "partitions[%d] = "
				      "{.name = %s, .offset = 0x%.8x, "
				      ".size = 0x%.8x (%uKiB) }\n",
				      i, parts[i].name,
				      parts[i].offset,
				      parts[i].size, parts[i].size / 1024);
			}
			flash->partitioned = 1;
#ifdef CONFIG_MACH_MX51_ERDOS
			mxc_spi_nor_mtd = &flash->mtd;
#endif
			return add_mtd_partitions(&flash->mtd, parts, nr_parts);
		}
	} else if (data->nr_parts)
		dev_warn(&spi->dev, "ignoring %d default partitions on %s\n",
			 data->nr_parts, data->name);

	return add_mtd_device(&flash->mtd) == 1 ? -ENODEV : 0;
}

static int __devexit mxspinor_remove(struct spi_device *spi)
{
	struct mxspinor *flash = dev_get_drvdata(&spi->dev);
	int status;

	/* Clean up MTD stuff. */
	if (mtd_has_partitions() && flash->partitioned)
		status = del_mtd_partitions(&flash->mtd);
	else
		status = del_mtd_device(&flash->mtd);
	if (status == 0)
		kfree(flash);

#ifdef CONFIG_MACH_MX51_ERDOS
			mxc_spi_nor_mtd = NULL;
#endif
	return 0;
}

static struct spi_driver mxspinor_driver = {
	.driver = {
		   .name = "mxc_spi_nor",
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   },
	.probe = mxspinor_probe,
	.remove = __devexit_p(mxspinor_remove),

	/* REVISIT: many of these chips have deep power-down modes, which
	 * should clearly be entered on suspend() to minimize power use.
	 * And also when they're otherwise idle...
	 */
};

static int mxspinor_init(void)
{
	return spi_register_driver(&mxspinor_driver);
}

static void mxspinor_exit(void)
{
	spi_unregister_driver(&mxspinor_driver);
}

module_init(mxspinor_init);
module_exit(mxspinor_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jason Liu");
MODULE_DESCRIPTION("MTD SPI driver for SST M25Pxx flash chips");
