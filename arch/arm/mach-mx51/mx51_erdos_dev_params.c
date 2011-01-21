/**
 * mach-mx51/mx51_erdos_dev_params.c
 *
 * This file contains the board specific device parameter routines.
 *
 * Copyright (C) 2008 Nissin Systems Co.,Ltd.
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * modification information
 * ------------------------
 * 2009/07/29 : created.
 * 2010/10/25 : ported to 2.6.31 by Andrey Zhornyak darion76@gmail.com
 */

#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/flash.h>
#include <mach/mx51_erdos_dev_params.h>

#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)


extern struct mtd_info	    * mxc_spi_nor_mtd;
extern struct mtd_partition mxc_spi_flash_partitions[];
extern struct flash_platform_data mxc_spi_flash_data;

static int mx51_erdos_dev_params_cache_read(u_char ** cache_buf, loff_t * mtd_offset)
{
	struct mtd_info      * mtd;
	struct mtd_partition * parts;
	int		       nr_parts;
	int		       ret, retlen, i;

	static u_char	      * dev_params_cache = NULL;
	static loff_t		dev_params_mtd_offset = 0;


	mtd	 = mxc_spi_nor_mtd;
	parts	 = mxc_spi_flash_partitions;
	nr_parts = mxc_spi_flash_data.nr_parts;

	if (mtd == NULL || nr_parts < 3)
		return (-ENODEV);

	if (dev_params_cache == NULL) {
		loff_t offset = 0;

		dev_params_cache = kmalloc(DEV_PARAMS_ERASE_SIZE, GFP_KERNEL);
		if (dev_params_cache == NULL)
			return (-ENOMEM);

		memset(dev_params_cache, 0, DEV_PARAMS_ERASE_SIZE);

		for (i=0; i < nr_parts; i++) {
			if (strcmp(parts[i].name, DEV_PARAMS_PART_NAME) == 0) {
				dev_params_mtd_offset = offset;
				break;
			}
			offset += parts[i].size;
		}
	}

	ret = mtd->read(mtd, dev_params_mtd_offset,
	                DEV_PARAMS_ERASE_SIZE,
	                &retlen, dev_params_cache);
	if (ret || retlen != DEV_PARAMS_ERASE_SIZE) {
		printk(KERN_ERR
		       "%s:%s(%d):Connot read mtd. ret=%d retlen=%d\n",
		       __FILE__, __FUNCTION__, __LINE__, ret, retlen);
		return (ret);
	}

	*cache_buf  = dev_params_cache;
	*mtd_offset = dev_params_mtd_offset;

	return (0);
}

int mx51_erdos_dev_params_read(loff_t from, size_t len, u_char * buf)
{
	struct mtd_info      * mtd;
	loff_t		       mtd_offset = 0;
	u_char		     * cache_buf = NULL;
	int		       ret;


	mtd	 = mxc_spi_nor_mtd;

	if (mtd == NULL)
		return (-ENODEV);

	if (from + len >= DEV_PARAMS_AREA_SIZE)
		return (-EINVAL);

	ret = mx51_erdos_dev_params_cache_read(&cache_buf, &mtd_offset);
	if (ret || cache_buf == NULL || mtd_offset == 0) {
		return (-ENODEV);
	}

	memcpy(buf, cache_buf + from, len);

	return (len);
}
EXPORT_SYMBOL(mx51_erdos_dev_params_read);

int mx51_erdos_dev_params_write(loff_t to, size_t len, u_char * buf)
{
	struct mtd_info      * mtd;
	struct erase_info      instr;
	loff_t		       mtd_offset = 0;
	u_char		     * cache_buf = NULL;
	int		       ret, retlen;

	mtd = mxc_spi_nor_mtd;

	if (mtd == NULL)
		return (-ENODEV);

	if (to + len > DEV_PARAMS_AREA_SIZE) {
		return (-EINVAL);
	}

	ret = mx51_erdos_dev_params_cache_read(&cache_buf, &mtd_offset);
	if (ret || cache_buf == NULL || mtd_offset == 0) {
		return (-ENODEV);
	}

	memcpy(cache_buf + to, buf, len);

	memset(&instr, 0, sizeof(instr));
	instr.mtd  = mtd;
	instr.addr = mtd_offset;
	instr.len  = DEV_PARAMS_ERASE_SIZE;

	ret = mtd->erase(mtd, &instr);
	if (ret) {
		printk(KERN_ERR "%s(%d):Cannot erase rfsinfo partition.\n",
			__FUNCTION__, __LINE__);
		return (ret);
	}

	ret = mtd->write(mtd, mtd_offset, DEV_PARAMS_ERASE_SIZE,
			 &retlen, cache_buf);
	if (ret || retlen != DEV_PARAMS_ERASE_SIZE) {
		printk(KERN_ERR "%s:%s(%d):Connot write mtd. "
			"offset(0x%08x) cache_buf(0x%08x) ret(%d) retlen(%d)\n",		       __FILE__, __FUNCTION__, __LINE__,
		       	(int) mtd_offset, (int)cache_buf, ret, retlen);
		return (ret);
	}

	return (len);
}
EXPORT_SYMBOL(mx51_erdos_dev_params_write);

#endif
