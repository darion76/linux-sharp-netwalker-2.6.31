/**
 * mach-mx51/mx51_erdos_dev_params.h
 *
 * This file contains all the board level device parameter interface.
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
 */

#ifndef __ASM_ARCH_MXC_MX51_ERDOS_DEV_PARAMS_H__
#define __ASM_ARCH_MXC_MX51_ERDOS_DEV_PARAMS_H__

/* Area offset defines */
#define	DEV_PARAMS_OFFSET_SPIDER_TYPE	  0
#define	DEV_PARAMS_OFFSET_DRM_ID	 16
#define	DEV_PARAMS_OFFSET_RESERVE1	 22
#define	DEV_PARAMS_OFFSET_TOUCH_MINX	 32
#define	DEV_PARAMS_OFFSET_TOUCH_MINY	 34
#define	DEV_PARAMS_OFFSET_TOUCH_MAXX	 36
#define	DEV_PARAMS_OFFSET_TOUCH_MAXY	 38
#define	DEV_PARAMS_OFFSET_TOUCH_X0	 40
#define	DEV_PARAMS_OFFSET_TOUCH_Y0	 42
#define	DEV_PARAMS_OFFSET_TOUCH_X1	 44
#define	DEV_PARAMS_OFFSET_TOUCH_Y1	 46
#define	DEV_PARAMS_OFFSET_TOUCH_X2	 48
#define	DEV_PARAMS_OFFSET_TOUCH_Y2	 50
#define	DEV_PARAMS_OFFSET_TOUCH_X3	 52
#define	DEV_PARAMS_OFFSET_TOUCH_Y3	 54
#define	DEV_PARAMS_OFFSET_TOUCH_X4	 56
#define	DEV_PARAMS_OFFSET_TOUCH_Y4	 58
#define	DEV_PARAMS_OFFSET_TOUCH_X5	 60
#define	DEV_PARAMS_OFFSET_TOUCH_Y5	 62
#define	DEV_PARAMS_OFFSET_TOUCH_X6	 64
#define	DEV_PARAMS_OFFSET_TOUCH_Y6	 66
#define	DEV_PARAMS_OFFSET_TOUCH_X7	 68
#define	DEV_PARAMS_OFFSET_TOUCH_Y7	 70
#define	DEV_PARAMS_OFFSET_TOUCH_X8	 72
#define	DEV_PARAMS_OFFSET_TOUCH_Y8	 74
#define	DEV_PARAMS_OFFSET_RESERVE2	 76
#define	DEV_PARAMS_OFFSET_SENSOR	 80
#define	DEV_PARAMS_OFFSET_RESERVE3	 81
#define	DEV_PARAMS_OFFSET_SELF_TEST_VER	 96
#define	DEV_PARAMS_OFFSET_WLAN		112
#define	DEV_PARAMS_OFFSET_RESERVE4	113

/* Area size defines */
#define	DEV_PARAMS_SIZE_SPIDER_TYPE	 0
#define	DEV_PARAMS_SIZE_DRM_ID		16
#define	DEV_PARAMS_SIZE_RESERVE1	 6
#define	DEV_PARAMS_SIZE_TOUCH_MINX	10
#define	DEV_PARAMS_SIZE_TOUCH_MINY	 2
#define	DEV_PARAMS_SIZE_TOUCH_MAXX	 2
#define	DEV_PARAMS_SIZE_TOUCH_MAXY	 2
#define	DEV_PARAMS_SIZE_TOUCH_X0	 2
#define	DEV_PARAMS_SIZE_TOUCH_Y0	 2
#define	DEV_PARAMS_SIZE_TOUCH_X1	 2
#define	DEV_PARAMS_SIZE_TOUCH_Y1	 2
#define	DEV_PARAMS_SIZE_TOUCH_X2	 2
#define	DEV_PARAMS_SIZE_TOUCH_Y2	 2
#define	DEV_PARAMS_SIZE_TOUCH_X3	 2
#define	DEV_PARAMS_SIZE_TOUCH_Y3	 2
#define	DEV_PARAMS_SIZE_TOUCH_X4	 2
#define	DEV_PARAMS_SIZE_TOUCH_Y4	 2
#define	DEV_PARAMS_SIZE_TOUCH_X5	 2
#define	DEV_PARAMS_SIZE_TOUCH_Y5	 2
#define	DEV_PARAMS_SIZE_TOUCH_X6	 2
#define	DEV_PARAMS_SIZE_TOUCH_Y6	 2
#define	DEV_PARAMS_SIZE_TOUCH_X7	 2
#define	DEV_PARAMS_SIZE_TOUCH_Y7	 2
#define	DEV_PARAMS_SIZE_TOUCH_X8	 2
#define	DEV_PARAMS_SIZE_TOUCH_Y8	 2
#define	DEV_PARAMS_SIZE_RESERVE2	 4
#define	DEV_PARAMS_SIZE_SENSOR		 1
#define	DEV_PARAMS_SIZE_RESERVE3	15
#define	DEV_PARAMS_SIZE_SELF_TEST_VER	16
#define	DEV_PARAMS_SIZE_WLAN		 1
#define	DEV_PARAMS_SIZE_RESERVE4	15

/* Configure Option */
#define	DEV_PARAMS_PART_NAME		 "info"	/* MTD partition name */
#define	DEV_PARAMS_AREA_SIZE		128
#define	DEV_PARAMS_ERASE_SIZE		0x1000
#define	DEV_PARAMS_USE_CACHE		1

extern int mx51_erdos_dev_params_read (loff_t from, size_t len, u_char * buf);
extern int mx51_erdos_dev_params_write(loff_t to,   size_t len, u_char * buf);

#endif /* __ASM_ARCH_MXC_MX51_ERDOS_DEV_PARAMS_H__ */
