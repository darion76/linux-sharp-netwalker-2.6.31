/*!
 * @file  include/asm-armmxc_touch_key.h
 *
 * @brief Header for the CAP1014 Capacitive Touch Sensor.
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

#ifndef __MXC_ETK_H__
#define __MXC_ETK_H__

#ifdef __KERNEL__

#define MOD_NAME  "mxc_etk"
#define CODE_TABLE_MAX  7
#define ETK_BTN_MAX  7
#define ETK_ALT_PIN (2 * 32 + 25)

#endif /* __KERNEL__ */

#define ETK_REG_MAIN_STAT			0x00	/* Main Status Control */
#define ETK_REG_BTN_STAT1			0x03	/* Button Status 1 */
#define ETK_REG_BTN_STAT2			0x04	/* Button Status 2 */
#define ETK_REG_DATA_SENS			0x1F	/* Data Sensitivity */
#define ETK_REG_BTN_SET_CNF			0x20	/* Configuration */
#define ETK_REG_SENS_ENBL			0x21	/* Sensor Enable */
#define ETK_REG_BTN_CYCL_CNF		0x22	/* Button Configuration */
#define ETK_REG_CALIB_ENBL			0x25	/* Calibration Enable */
#define ETK_REG_CALIB_ACT			0x26	/* Calibration Activate */
#define ETK_REG_INT_ENBL			0x27	/* Interrupt Enable 1 */
#define ETK_REG_MULTI_TCH_CNF		0x2A	/* Multiple Press Configuration */
#define ETK_REG_RECALIB_CNF			0x2F	/* Recalibration Configuration */
#define ETK_REG_SENS_THREHOLDE1		0x30	/* Sensor 1 Threshold */
#define ETK_REG_SENS_THREHOLDE2		0x31	/* Sensor 2 Threshold */
#define ETK_REG_SENS_THREHOLDE3		0x32	/* Sensor 3 Threshold */
#define ETK_REG_SENS_THREHOLDE4		0x33	/* Sensor 4 Threshold */
#define ETK_REG_SENS_THREHOLDE5		0x34	/* Sensor 5 Threshold */
#define ETK_REG_SENS_THREHOLDE6		0x35	/* Sensor 6 Threshold */
#define ETK_REG_SENS_THREHOLDE7		0x36	/* Sensor 7 Threshold */
#define ETK_REG_GROUP_THREHOLDE		0x37	/* Group Threshold */

#ifdef __KERNEL__

#define ETK_INT_BIT					0x01
#define ETK_DSLEEP_BIT				0x10
#define ETK_CALB_ALL_BIT			0x80

#endif /* __KERNEL__ */

#define ETK_GET_REGISTER	_IO('t', 0x01)	/* ioctl Get register deta*/
#define ETK_SET_REGISTER	_IO('t', 0x02)	/* ioctl Set register deta*/
#define ETK_EXEC_CALIB		_IO('t', 0x03)	/* ioctl Execution of calibration*/

typedef unsigned char 	BYTE;
typedef enum { TRUE, FALSE, FUZZY } BOOL;
typedef unsigned short int 	SCANCODE;

struct etk_reg_op_pram {
	BYTE reg_add;
	BYTE reg_val;
};

#endif				/* __MXC_ETK_H__ */
