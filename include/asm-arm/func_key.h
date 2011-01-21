/*!
 * @file  include/asm-arm/func_key.h
 *
 * @brief Allocation function of function key for Spider.
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
 */

#ifndef __FUNC_KEY_H__
#define __FUNC_KEY_H__

#ifdef __KERNEL__

#define FNKEY_TABLE_MAX  9

#endif /* __KERNEL__ */

#define FNKEY_DEV_VEBDOR			0x04dd	/* Device Vendor ID of corresponding function key */
#define FNKEY_DEV_PRODUCT_QA0		0x9410	/* Device Product ID(QA0) of corresponding function key */
#define FNKEY_DEV_PRODUCT_QA1		0x92e7	/* Device Product ID(QA1) of corresponding function key */

#define FNKEY_ID_WIRELESS			0x01	/* Wireless LAN ON/OFF */
#define FNKEY_ID_BLUETOOTH			0x02	/* Bluetooth ON/OFF */
#define FNKEY_ID_VOL_UP				0x03	/* Volume UP */
#define FNKEY_ID_VOL_DOWN			0x04	/* Volume Down */
#define FNKEY_ID_VOL_MUTE			0x05	/* Mute */
#define FNKEY_ID_BRIGHT_UP			0x06	/* Brightness UP */
#define FNKEY_ID_BRIGHT_DOWN		0x07	/* Brightness Down */
#define FNKEY_ID_LCD_BL				0x08	/* LCD Backlight ON/OFF */
#define FNKEY_ID_TRACK_BALL			0x09	/* Cursor/Wheel */
#define FNKEY_ID_MAX				0x0a	/* ID MAX */

typedef void (*fnkey_handler_t)(int id, int value, void *arg);

#ifdef __KERNEL__

struct fnkey_handle {
	fnkey_handler_t handler;
	void *arg;
};

struct fnkey_regist_table {
	int id;
	unsigned int ev_code;
	unsigned int fn_code;
	struct fnkey_handle handle;
};

#endif /* __KERNEL__ */

extern void fnkey_pass_event(struct input_dev *dev,unsigned int type, unsigned int code, int value);
extern int fnkey_register(int id, fnkey_handler_t handler, void *arg);
extern int fnkey_unregister(int id);

#endif				/* __FUNC_KEY_H__ */
