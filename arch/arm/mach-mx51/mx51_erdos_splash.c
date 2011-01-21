/**
 * @file  arch/arm/mach-mx51/mx51_erdos_splash.c
 *
 * @brief display splash logo
 *
 * Copyright (C) 2009 Nissin Systems Co.,Ltd.
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
 * 2009/07/21 : create.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/fb.h>

#include "splash_dat.inc"

static volatile char *fb_show_spider_logo_mem;
/*
 * fb_show_spider_logo - display splash logo
 */
int fb_show_spider_logo(struct fb_info *info, int rotate)
{
	volatile unsigned short *p;
	struct _splash_bmap     *tbl;
	int                      x, y, ofst;
	int                      omax = sizeof(splash_bmap) / sizeof(struct _splash_bmap);

	fb_show_spider_logo_mem = (char *)(info->screen_base);
	ofst = 0;
	for ( y = 0 ; y < SPLASH_HEIGHT ; y++ ) {
		/*
		 * y : reverse
		 */
		p   = (volatile unsigned short *)((char *)(info->screen_base) +
			(SPLASH_HEIGHT - y - 1) * SPLASH_WIDTH * sizeof(short));
		tbl = 0;
		if ((ofst < omax) && (splash_bmap [ofst].y == y)) {
			tbl = &(splash_bmap [ofst]);
			ofst++;
		}
		if (tbl != 0) {
			unsigned short *src = &(tbl->x [0]);
			for ( x = 0 ; x < SPLASH_WIDTH ; x++ ) {
				*p++ = *src++;
			}
		} else {
			for ( x = 0 ; x < SPLASH_WIDTH ; x++ ) {
				*p++ = 0xFFFF;		/* white */
			}
		}
	}
	return 0;
}

MODULE_DESCRIPTION("splash display");
MODULE_AUTHOR("Nissin Systems Co.,Ltd");
MODULE_LICENSE("GPL");
