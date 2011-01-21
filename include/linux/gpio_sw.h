/**
 * @file  include/linux/gpio_sw.h
 *
 * @brief Header file for the OJ6SH-T25 Optical Joystick Driver.
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

#ifndef __GPIO_SW_H__
#define __GPIO_SW_H__

#ifdef __KERNEL__

struct gpio_sw_platform_data {
	int  irq;
	int  output_pol;
	int  event_type;
	int  event_code;
	int  wake;
	char *input_name;
	char *irq_name;
	int (*get_value)(void);
};

#endif

#endif /* __GPIO_SW_H__ */
