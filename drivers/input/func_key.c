/*!
 * @file  drivers/input/func_key.c
 *
 * @brief  Allocation function of function key for Spider.
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

/*!
 * Comment FNKEY_DEBUG to disable debug messages
 */
#define FNKEY_DEBUG	0

#if FNKEY_DEBUG
#define	DEBUG
#include <linux/kernel.h>
#endif

#include <linux/init.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/major.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/rcupdate.h>
#include <linux/smp_lock.h>

/*
 * Module header file
 */
#include <asm/func_key.h>

static struct fnkey_regist_table fnkey_tbl[FNKEY_TABLE_MAX] = {
	{ FNKEY_ID_WIRELESS,	0x96, KEY_WLAN,				{NULL, NULL}},
	{ FNKEY_ID_BLUETOOTH,	0x9E, KEY_BLUETOOTH,		{NULL, NULL}},
	{ FNKEY_ID_VOL_UP,		0x73, KEY_VOLUMEUP,			{NULL, NULL}},
	{ FNKEY_ID_VOL_DOWN,	0x72, KEY_VOLUMEDOWN,		{NULL, NULL}},
	{ FNKEY_ID_VOL_MUTE,	0x71, KEY_MUTE,				{NULL, NULL}},
	{ FNKEY_ID_BRIGHT_UP,	0x9F, KEY_BRIGHTNESSUP,		{NULL, NULL}},
	{ FNKEY_ID_BRIGHT_DOWN,	0x80, KEY_BRIGHTNESSDOWN,	{NULL, NULL}},
	{ FNKEY_ID_LCD_BL,		0x88, 0x00,					{NULL, NULL}},
	{ FNKEY_ID_TRACK_BALL,	0x8E, KEY_F23,				{NULL, NULL}}
};

/*!
 * Processing for the Spider allocated in the function key is called. 
 * Other usual events are processed.
 *
 * @param	dev		device that generated the event  
 * @param	type	type of the event
 * @param	code	event code
 * @param	value	value of the event
 */
void fnkey_pass_event(struct input_dev * dev, unsigned int type,
		      unsigned int code, int value)
{
	unsigned int put_code = code;
	struct input_handle * handle;
	int		      i;

	for (i=0; i < FNKEY_TABLE_MAX; i++) {
		if (type == EV_KEY && fnkey_tbl[i].ev_code == code){
			if (fnkey_tbl[i].handle.handler != NULL)
				fnkey_tbl[i].handle.handler(fnkey_tbl[i].id,
					value, fnkey_tbl[i].handle.arg);
			put_code = fnkey_tbl[i].fn_code;

			pr_debug("func_key: %s: "
				 "func ID = 0x%02X put code = 0x%02X\n",
				 __func__, fnkey_tbl[i].id, put_code);
			break;
		}
	}

	if (type != EV_KEY || put_code != 0) {
		rcu_read_lock();

		handle = rcu_dereference(dev->grab);
		if (handle) {
			handle->handler->event(handle, type, put_code, value);
		} else {
			list_for_each_entry_rcu(handle, &dev->h_list, d_node) {
				if (handle->open)
				    handle->handler->event(handle, type,
				    			   put_code, value);
			}
		}

		rcu_read_unlock();
	}
}
EXPORT_SYMBOL(fnkey_pass_event);

/*!
 * The acquisition demand of the event of the function key 
 * is registered.  
 *
 * @param	id			Function ID to demand  
 * @param	handler		Handler which acquires an input event
 * @param	arg			Argument passed when handler is called
 *
 * @return	The function returns 0 on success and errno on failure
 */
int fnkey_register(int id, fnkey_handler_t handler, void * arg)
{
	int ret = -EINVAL;
	int i;

	if ( (id <= 0)	|| (id >= FNKEY_ID_MAX) || (handler == NULL) ) {
		pr_debug("func_key: %s: Illegal argument\n", __func__);
		goto end;
	}

	for (i=0; i < FNKEY_TABLE_MAX; i++){
		if (fnkey_tbl[i].id == id) {
			if (fnkey_tbl[i].handle.handler != NULL) {
				pr_debug("func_key: %s: Selected ID is busy\n",
					 __func__);
				goto end;
		    	}

			fnkey_tbl[i].handle.handler = handler;
			fnkey_tbl[i].handle.arg     = arg;
			ret = 0;
			pr_debug("func_key: %s: ID = %d was registered\n",
				__func__, id);
			break;
		}
	}
end:
	return (ret);
}
EXPORT_SYMBOL(fnkey_register);

/*!
 * The acquisition demand of the event of the function key
 * is deleted.   
 *
 * @param	id			Function ID to demand  
 *
 * @return	The function returns 0 on success and errno on failure
 */
int fnkey_unregister(int id)
{
    int ret = -EINVAL;
    int i;

	if ((id <= 0)  || (id >= FNKEY_ID_MAX)){
		pr_debug("func_key: %s: Illegal argument\n", __func__);
		goto end;
	}

	for(i=0; i < FNKEY_TABLE_MAX; i++) {
		if (fnkey_tbl[i].id == id) {
			if (fnkey_tbl[i].handle.handler == NULL) {
				pr_debug("func_key: %s: "
					 "Selected ID is not registered\n",
					  __func__);
				goto end;
			}
			fnkey_tbl[i].handle.handler    = NULL;
			fnkey_tbl[i].handle.arg        = NULL;
			ret = 0;
			pr_debug("func_key: %s: ID = %d was deleted\n",
				 __func__, id);
			break;
		}
	}
end:
	return (ret);
}
EXPORT_SYMBOL(fnkey_unregister);

MODULE_DESCRIPTION("Allocation function of function key for Spider");
MODULE_AUTHOR("Nissin Systems Co.,Ltd");
MODULE_LICENSE("GPL");

