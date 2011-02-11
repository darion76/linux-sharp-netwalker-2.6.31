/**
 * mach-mx51/mx51_erdos_gpio.c
 *
 * This file contains all the GPIO setup functions for the board.
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
 * 2009/07/21 : get_gpio_sw_status() add.
 *              MX51_PIN_CSI2_D19(cover open) add.
 *              MX51_PIN_EIM_A27(power sw) add.
 * 2009/07/22 : get_gpio_sw_status() AC/BATT add.
 * 2009/07/27 : change CONFIG_MACH_MX51_ERDOS_Q0 -> is_board_qa0().
 *              don't PULLUP REFON, CHGCTRL, CHG_PG, CHG_STAT1, CHG_STAT2
 *              get_gpio_sw_status() COVER-SW fixed.
 *              gpio_ac_power() delete.
 *              gpio_charge_status(), gpio_refon() add.
 *              gpio_battery_enable(), gpio_poweroff() add.
 *              get_gpio_sw_status() change ACadapter -> DCinput OK.
 * 2009/08/02 : merge L2.6.28_4.4.0_SS_Jul2009_source.
 *               gpio_spi_chipselect_inactive/_active() ->
 *                mx51_erdos_gpio_spi_chipselect_inactive/_active().
 * 2009/08/07 : mx51_erdos_gpio_spi_chipselect_all_inactive() add.
 * 2009/08/09 : I2C iomux fixed(SCL don't OPENDRAIN)
 *              gpio_wlan_exit() add check already init.
 *              gpio_cap1014_wakeup() delete 30ms delay.
 * 2009/08/27 : MX51_PIN_NANDF_D10/D12 battery_charge_status 1/0 change.
 * 2009/08/28 : gpio_usb_power() arg change.
 *              delete USB HUB reset in mx51_erdos_io_init().
 *              change USB HUB RESET usb_reset() -> gpio_usb_power().
 * 2009/09/01 : gpio_usb_power() DR(otg-host) support.
 * 2009/09/06 : iopad change.
 *               USB_HUB_RESET, SD1_CMD, SD1_DATAx, BB_RXD, BB_CLK,
 *               BB_FS, Audio_clk_en, EIM_A19/A20/A24, CSPI-MOSI,MISO,SCLK
 * 2010/11/06 : ported to 2.6.31 by Andrey Zhornyak darion76@gmail.com
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

#include "iomux.h"

int is_fec(void);
int is_board_qa0(void);

static struct mxc_iomux_pin_cfg __initdata mxc_iomux_pins[] = {
	/* Battery pin settings */
	{
		/* REFON */
		.pin       = MX51_PIN_DISPB2_SER_DIO,
		.mux_mode  = IOMUX_CONFIG_ALT4,
		.pad_cfg   = (PAD_CTL_DRV_HIGH),
		.in_select = MUX_IN_GPIO3_IPP_IND_G_IN_6_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH1,
	}, {
		/* CHGCTRL */
		.pin       = MX51_PIN_DISPB2_SER_CLK,
		.mux_mode  = IOMUX_CONFIG_ALT4,
		.pad_cfg   = (PAD_CTL_DRV_HIGH),
		.in_select = MUX_IN_GPIO3_IPP_IND_G_IN_7_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH1,
	}, {
		/* CHG_PG */
		.pin       = MX51_PIN_NANDF_D13,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_DRV_HIGH   | PAD_CTL_HYS_ENABLE),
	}, {
		/* CHG_STAT1 */
		.pin       = MX51_PIN_NANDF_D12,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_DRV_HIGH   | PAD_CTL_HYS_ENABLE),
	}, {
		/* CHG_STAT2 */
		.pin       = MX51_PIN_NANDF_D10,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_DRV_HIGH   | PAD_CTL_HYS_ENABLE),
	},

	/* USB pin settings */
	{
		/* USBH1_STP */
		.pin	   = MX51_PIN_USBH1_STP,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_SRE_FAST	 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			      PAD_CTL_HYS_ENABLE),
	}, {
		/* USBH1_CLK */
		.pin	   = MX51_PIN_USBH1_CLK,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_SRE_FAST	 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			      PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS),
	}, {
		/* USBH1_DIR */
		.pin	   = MX51_PIN_USBH1_DIR,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_SRE_FAST	 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			      PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS),
	}, {
		/* USBH1_NXT */
		.pin	   = MX51_PIN_USBH1_NXT,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_SRE_FAST	 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			      PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS),
	}, {
		/* USBH1_DATA0 */
		.pin	   = MX51_PIN_USBH1_DATA0,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_SRE_FAST	 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_100K_PU	 | PAD_CTL_PUE_KEEPER |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	}, {
		/* USBH1_DATA1 */
		.pin	   = MX51_PIN_USBH1_DATA1,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_SRE_FAST	 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_100K_PU	 | PAD_CTL_PUE_KEEPER |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	}, {
		/* USBH1_DATA2 */
		.pin	   = MX51_PIN_USBH1_DATA2,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_SRE_FAST	 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_100K_PU	 | PAD_CTL_PUE_KEEPER |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	}, {
		/* USBH1_DATA3 */
		.pin	   = MX51_PIN_USBH1_DATA3,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_SRE_FAST	 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_100K_PU	 | PAD_CTL_PUE_KEEPER |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	}, {
		/* USBH1_DATA4 */
		.pin	   = MX51_PIN_USBH1_DATA4,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_SRE_FAST	 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_100K_PU	 | PAD_CTL_PUE_KEEPER |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	}, {
		/* USBH1_DATA5 */
		.pin	   = MX51_PIN_USBH1_DATA5,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_SRE_FAST	 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_100K_PU	 | PAD_CTL_PUE_KEEPER |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	}, {
		/* USBH1_DATA6 */
		.pin	   = MX51_PIN_USBH1_DATA6,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_SRE_FAST	 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_100K_PU	 | PAD_CTL_PUE_KEEPER |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	}, {
		/* USBH1_DATA7 */
		.pin	   = MX51_PIN_USBH1_DATA7,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_SRE_FAST	 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_100K_PU	 | PAD_CTL_PUE_KEEPER |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	}, {
		/* USB_CLK_EN_B line low */
		.pin	   = MX51_PIN_EIM_D17,
		.mux_mode  = IOMUX_CONFIG_ALT1,
		.pad_cfg   = (PAD_CTL_DRV_HIGH	 | PAD_CTL_PKE_ENABLE	|
			      PAD_CTL_SRE_FAST),
	}, {
		/* USB PHY RESETB */
		.pin	   = MX51_PIN_EIM_D21,
		.mux_mode  = IOMUX_CONFIG_ALT1,
		.pad_cfg   = (PAD_CTL_DRV_HIGH	 | PAD_CTL_HYS_NONE |
			      PAD_CTL_PUE_KEEPER | PAD_CTL_100K_PU  |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST |
			      PAD_CTL_ODE_OPENDRAIN_NONE),
	}, {
		/* USB HUB RESET */
		.pin	   = MX51_PIN_GPIO1_7,
		.mux_mode  = IOMUX_CONFIG_GPIO,
		.pad_cfg   = (PAD_CTL_DRV_HIGH | PAD_CTL_PKE_NONE |
			      PAD_CTL_SRE_FAST),
	},

	/* UART pin settings */
	{
		/* UART1 RXD */
		.pin	   = MX51_PIN_UART1_RXD,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE |
			      PAD_CTL_PUE_PULL	 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_SRE_FAST),
		.in_select = MUX_IN_UART1_IPP_UART_RXD_MUX_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH0,
	}, {
		/* UART1 TXD */
		.pin	   = MX51_PIN_UART1_TXD,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE |
			      PAD_CTL_PUE_PULL	 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_SRE_FAST),
	}, {
		/* UART1 RTS */
		.pin	   = MX51_PIN_UART1_RTS,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE |
			      PAD_CTL_PUE_PULL	 | PAD_CTL_DRV_HIGH),
		.in_select = MUX_IN_UART1_IPP_UART_RTS_B_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH0,
	}, {
		/* UART1 CTS */
		.pin	   = MX51_PIN_UART1_CTS,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE |
			      PAD_CTL_PUE_PULL	 | PAD_CTL_DRV_HIGH),
	}, {
		/* UART3 RXD */
		.pin	   = MX51_PIN_EIM_D25,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_DRV_HIGH	 | PAD_CTL_HYS_NONE |
			      PAD_CTL_PUE_KEEPER | PAD_CTL_100K_PU  |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST |
			      PAD_CTL_ODE_OPENDRAIN_NONE),
		.in_select = MUX_IN_UART3_IPP_UART_RXD_MUX_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH0,
	}, {
		/* UART3 TXD */
		.pin	   = MX51_PIN_EIM_D26,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_DRV_HIGH	 | PAD_CTL_HYS_NONE |
			      PAD_CTL_PUE_KEEPER | PAD_CTL_100K_PU  |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST |
			      PAD_CTL_ODE_OPENDRAIN_NONE),
	}, {
		/* UART3 RTS */
		.pin	   = MX51_PIN_EIM_D27,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_DRV_HIGH	 | PAD_CTL_HYS_NONE |
			      PAD_CTL_PUE_KEEPER | PAD_CTL_100K_PU  |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST |
			      PAD_CTL_ODE_OPENDRAIN_NONE),
		.in_select = MUX_IN_UART3_IPP_UART_RTS_B_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH3,
	}, {
		/* UART3 CTS */
		.pin	   = MX51_PIN_EIM_D24,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_DRV_HIGH	 | PAD_CTL_HYS_NONE |
			      PAD_CTL_PUE_KEEPER | PAD_CTL_100K_PU  |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST |
			      PAD_CTL_ODE_OPENDRAIN_NONE),
	},

	/* SD1 pin settings */
	{
		/* SD1 CMD */
		.pin	   = MX51_PIN_SD1_CMD,
		.mux_mode  = IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		.pad_cfg   = (PAD_CTL_PKE_NONE | PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST),
	}, {
		/* SD1 CLK */
		.pin	   = MX51_PIN_SD1_CLK,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			      PAD_CTL_DRV_HIGH	 | PAD_CTL_47K_PU     |
			      PAD_CTL_SRE_FAST),
	}, {
		/* SD1 DATA0 */
		.pin	   = MX51_PIN_SD1_DATA0,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_PKE_NONE | PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST),
	}, {
		/* SD1 DATA1 */
		.pin	   = MX51_PIN_SD1_DATA1,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_PKE_NONE | PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST),
	}, {
		/* SD1 DATA2 */
		.pin	   = MX51_PIN_SD1_DATA2,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_PKE_NONE | PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST),
	}, {
		/* SD1 DATA3 */
		.pin	   = MX51_PIN_SD1_DATA3,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_PKE_NONE | PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST),
	},

	/* SD2 pin settings */
	{
		/* SD2 CMD */
		.pin	   = MX51_PIN_SD2_CMD,
		.mux_mode  = IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		.pad_cfg   = (PAD_CTL_DRV_MAX  | PAD_CTL_22K_PU  |
		              PAD_CTL_SRE_FAST | PAD_CTL_DRV_VOT_LOW),
	}, {
		/* SD2 CLK */
		.pin	   = MX51_PIN_SD2_CLK,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_MAX  | PAD_CTL_22K_PU  |
		              PAD_CTL_SRE_FAST | PAD_CTL_DRV_VOT_LOW),
	}, {
		/* SD2 DATA0 */
		.pin	   = MX51_PIN_SD2_DATA0,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_MAX  | PAD_CTL_22K_PU  |
		              PAD_CTL_SRE_FAST | PAD_CTL_DRV_VOT_LOW),
	}, {
		/* SD2 DATA1 */
		.pin	   = MX51_PIN_SD2_DATA1,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_MAX  | PAD_CTL_22K_PU  |
		              PAD_CTL_SRE_FAST | PAD_CTL_DRV_VOT_LOW),
	}, {
		/* SD2 DATA2 */
		.pin	   = MX51_PIN_SD2_DATA2,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_MAX  | PAD_CTL_22K_PU  |
		              PAD_CTL_SRE_FAST | PAD_CTL_DRV_VOT_LOW),
	}, {
		/* SD2 DATA3 */
		.pin	   = MX51_PIN_SD2_DATA3,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_MAX  | PAD_CTL_22K_PU  |
		              PAD_CTL_SRE_FAST | PAD_CTL_DRV_VOT_LOW),
	},

	/* MMC pin settings */
	{
		/* BOOT PORT - SD/MMC CD_B */
		.pin	   = MX51_PIN_GPIO1_0,
		.mux_mode  = IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION,
		.pad_cfg   = (PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU),
	},

	/* Audio pin settings */
	{
		/* AUD3_BB_TXD */
		.pin	   = MX51_PIN_AUD3_BB_TXD,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_SRE_FAST	     | PAD_CTL_DRV_HIGH |
			      PAD_CTL_100K_PU	     | PAD_CTL_HYS_NONE |
			      PAD_CTL_DDR_INPUT_CMOS | PAD_CTL_DRV_VOT_LOW),
	}, {
		/* AUD3_BB_RXD */
		.pin	   = MX51_PIN_AUD3_BB_RXD,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_SRE_FAST	 | PAD_CTL_DRV_HIGH |
			      PAD_CTL_HYS_NONE   |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER),
	}, {
		/* AUD3_BB_CK */
		.pin	   = MX51_PIN_AUD3_BB_CK,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_SRE_FAST	 | PAD_CTL_DRV_HIGH |
			      PAD_CTL_HYS_NONE   |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER),
	}, {
		/* AUD3_BB_FS */
		.pin	   = MX51_PIN_AUD3_BB_FS,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_SRE_FAST	 | PAD_CTL_DRV_HIGH |
			      PAD_CTL_HYS_NONE   |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER),
	}, {
		/* hphone_det_b */
		.pin	   = MX51_PIN_NANDF_D14,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_100K_PU    | PAD_CTL_HYS_ENABLE |
		              PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL),
	}, {
		/* audio_clk_en_b */
		.pin	   = MX51_PIN_CSPI1_RDY,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_DRV_HIGH	 | PAD_CTL_HYS_NONE |
			      PAD_CTL_PKE_NONE   | PAD_CTL_SRE_FAST),
	}, {
		/* AUDAMP_CTRL1 */
		.pin       = MX51_PIN_EIM_A23,
		.mux_mode  = IOMUX_CONFIG_ALT1,
		.pad_cfg   = ( PAD_CTL_HYS_NONE   | PAD_CTL_PKE_NONE ),
	}, {
		/* AUDAMP_CTRL2 */
		.pin       = MX51_PIN_EIM_A25,
		.mux_mode  = IOMUX_CONFIG_ALT1,
		.pad_cfg   = ( PAD_CTL_HYS_NONE   | PAD_CTL_PKE_NONE ),
	},

	/* I2C pin settings */
	{
		/* i2c1 SDA */
		.pin	   = MX51_PIN_EIM_D16,
		.mux_mode  = IOMUX_CONFIG_ALT4 | IOMUX_CONFIG_SION,
		.pad_cfg   = (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_HYS_ENABLE | PAD_CTL_ODE_OPENDRAIN_ENABLE),
		.in_select = MUX_IN_I2C1_IPP_SDA_IN_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH0,
	}, {
		/* i2c1 SCL */
		.pin	   = MX51_PIN_EIM_D19,
		.mux_mode  = IOMUX_CONFIG_ALT4 | IOMUX_CONFIG_SION,
		.pad_cfg   = (PAD_CTL_SRE_FAST),
		.in_select = MUX_IN_I2C1_IPP_SCL_IN_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH0,
	}, {
		/* i2c2 SDA */
		.pin	   = MX51_PIN_KEY_COL5,
		.mux_mode  = IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION,
		.pad_cfg   = (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_HYS_ENABLE | PAD_CTL_ODE_OPENDRAIN_ENABLE),
		.in_select = MUX_IN_I2C2_IPP_SDA_IN_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH1,
	}, {
		/* i2c2 SCL */
		.pin	   = MX51_PIN_KEY_COL4,
		.mux_mode  = IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION,
		.pad_cfg   = (PAD_CTL_SRE_FAST),
		.in_select = MUX_IN_I2C2_IPP_SCL_IN_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH1,
	}, {
		/* BT_LPB[1] */
		.pin	   = MX51_PIN_EIM_A19,
		.mux_mode  = IOMUX_CONFIG_GPIO,
		.pad_cfg   = (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH |
			      PAD_CTL_HYS_NONE | PAD_CTL_PKE_NONE),
	},

	/* PMIC pin settings */
	{
		/* PMIC INT */
		.pin	   = MX51_PIN_GPIO1_8,
		.mux_mode  = IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION,
		.pad_cfg   = (PAD_CTL_SRE_SLOW | PAD_CTL_DRV_MEDIUM |
			      PAD_CTL_100K_PU  | PAD_CTL_HYS_ENABLE |
			      PAD_CTL_DRV_VOT_HIGH),
	},

	/* 26M_OSC pin settings */
	{
		.pin	   = MX51_PIN_DI1_PIN12,
		.mux_mode  = IOMUX_CONFIG_ALT4,
		.pad_cfg   = (PAD_CTL_DRV_HIGH | PAD_CTL_PKE_ENABLE |
			      PAD_CTL_SRE_FAST),
	},

	/* LCD pin settings */
	{
		/* DISP1 HSYNC */
		.pin	   = MX51_PIN_DI1_PIN2,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 VSYNC */
		.pin	   = MX51_PIN_DI1_PIN3,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[6] */
		.pin	   = MX51_PIN_DISP1_DAT6,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[7] */
		.pin	   = MX51_PIN_DISP1_DAT7,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[8] */
		.pin	   = MX51_PIN_DISP1_DAT8,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[9] */
		.pin	   = MX51_PIN_DISP1_DAT9,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[10] */
		.pin	   = MX51_PIN_DISP1_DAT10,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[11] */
		.pin	   = MX51_PIN_DISP1_DAT11,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[12] */
		.pin	   = MX51_PIN_DISP1_DAT12,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[13] */
		.pin	   = MX51_PIN_DISP1_DAT13,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[14] */
		.pin	   = MX51_PIN_DISP1_DAT14,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[15] */
		.pin	   = MX51_PIN_DISP1_DAT15,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[16] */
		.pin	   = MX51_PIN_DISP1_DAT16,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[17] */
		.pin	   = MX51_PIN_DISP1_DAT17,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[18] */
		.pin	   = MX51_PIN_DISP1_DAT18,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[19] */
		.pin	   = MX51_PIN_DISP1_DAT19,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[20] */
		.pin	   = MX51_PIN_DISP1_DAT20,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[21] */
		.pin	   = MX51_PIN_DISP1_DAT21,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[22] */
		.pin	   = MX51_PIN_DISP1_DAT22,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP1 DAT[23] */
		.pin	   = MX51_PIN_DISP1_DAT23,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* LVDS PWR DWN */
		.pin	   = MX51_PIN_DI1_D0_CS,
		.mux_mode  = IOMUX_CONFIG_ALT4,
		.pad_cfg   = (PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER |
		              PAD_CTL_DRV_HIGH   | PAD_CTL_SRE_FAST),
	}, {
		/* LCD 3V3 V5A PWRGD */
		.pin	   = MX51_PIN_CSI2_D12,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_HYS_NONE | PAD_CTL_PKE_ENABLE |
		              PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST),
	}, {
		/* LCD 5V ON */
		.pin	   = MX51_PIN_CSI2_D13,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_HYS_NONE | PAD_CTL_PKE_ENABLE |
		              PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST),
	},

	/* VGA pin settings */
	{
		/* DISP2 DRDY */
		.pin	   = MX51_PIN_DI_GP4,
		.mux_mode  = IOMUX_CONFIG_ALT4,
	}, {
		/* DISP2 HSYNC */
		.pin	   = MX51_PIN_DI2_PIN2,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 VSYNC */
		.pin	   = MX51_PIN_DI2_PIN3,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 CLK */
		.pin	   = MX51_PIN_DI2_DISP_CLK,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 DAT[0] */
		.pin	   = MX51_PIN_DISP2_DAT0,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 DAT[1] */
		.pin	   = MX51_PIN_DISP2_DAT1,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 DAT[2] */
		.pin	   = MX51_PIN_DISP2_DAT2,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 DAT[3] */
		.pin	   = MX51_PIN_DISP2_DAT3,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 DAT[4] */
		.pin	   = MX51_PIN_DISP2_DAT4,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 DAT[5] */
		.pin	   = MX51_PIN_DISP2_DAT5,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 DAT[6] */
		.pin	   = MX51_PIN_DISP2_DAT6,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 DAT[7] */
		.pin	   = MX51_PIN_DISP2_DAT7,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 DAT[8] */
		.pin	   = MX51_PIN_DISP2_DAT8,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 DAT[9] */
		.pin	   = MX51_PIN_DISP2_DAT9,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 DAT[10] */
		.pin	   = MX51_PIN_DISP2_DAT10,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 DAT[11] */
		.pin	   = MX51_PIN_DISP2_DAT11,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 DAT[12] */
		.pin	   = MX51_PIN_DISP2_DAT12,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 DAT[13] */
		.pin	   = MX51_PIN_DISP2_DAT13,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 DAT[14] */
		.pin	   = MX51_PIN_DISP2_DAT14,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	}, {
		/* DISP2 DAT[15] */
		.pin	   = MX51_PIN_DISP2_DAT15,
		.mux_mode  = IOMUX_CONFIG_ALT0,
	},

	/* PWM pin settings */
	{
		/* PWM1 PWMO */
		.pin	   = MX51_PIN_GPIO1_2,
		.mux_mode  = IOMUX_CONFIG_ALT1,
		.pad_cfg   = (PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_ENABLE),
	},

	/* OJ6SH-T25 Optical Joystick GPIO pin settings */
	{
		/* DOME */
		.pin	   = MX51_PIN_CSI1_D9,
		.mux_mode  = IOMUX_CONFIG_GPIO,
	} , {
		/* SHUTDOWN */
		.pin	   = MX51_PIN_CSI1_VSYNC,
		.mux_mode  = IOMUX_CONFIG_GPIO,
	} , {
		/* MOTION */
		.pin	   = MX51_PIN_CSI1_HSYNC,
		.mux_mode  = IOMUX_CONFIG_GPIO,
	},

	/* Electrostatic Touch Key pin settings */
	{
		/* ALERT pin */
		.pin	   = MX51_PIN_NANDF_D15,
		.mux_mode  = IOMUX_CONFIG_GPIO,
	}, {
		/* RESET pin */
		.pin	   = MX51_PIN_GPIO_NAND,
		.mux_mode  = IOMUX_CONFIG_GPIO,
	},

	/* BlueTooth GPIO pin settings */
	{
		/* BT_HOST_WAKEUP pin */
		.pin	   = MX51_PIN_EIM_D20,
		.mux_mode  = IOMUX_CONFIG_GPIO,
	}, {
		/* BT_RESETN pin */
		.pin	   = MX51_PIN_EIM_D22,
		.mux_mode  = IOMUX_CONFIG_GPIO,
	}, {
		/* BT_WAKEUP pin */
		.pin	   = MX51_PIN_EIM_D23,
		.mux_mode  = IOMUX_CONFIG_GPIO,
	},

	/* LEFT/WRITE Switch pin setting */
	{
		/* LEFT_CLICK_KEY (GPIO2_GPIO[22]) */
		.pin	   = MX51_PIN_EIM_EB2,
		.mux_mode  = IOMUX_CONFIG_GPIO,
		.pad_cfg   = (PAD_CTL_HYS_ENABLE),
	}, {
		/* RIGHT_CLICK_KEY (GPIO2_GPIO[23]) */
		.pin	   = MX51_PIN_EIM_EB3,
		.mux_mode  = IOMUX_CONFIG_GPIO,
		.pad_cfg   = (PAD_CTL_HYS_ENABLE),
	},

	/* Cover Switch pin setting(GPIO4_GPIO[12] */
	{
		.pin       = MX51_PIN_CSI2_D19,
		.mux_mode  = IOMUX_CONFIG_GPIO,
		.pad_cfg   = (PAD_CTL_HYS_ENABLE),
	},

	/* Power Switch pin setting(GPIO2_GPIO[21] */
	{
		.pin       = MX51_PIN_EIM_A27,
		.mux_mode  = IOMUX_CONFIG_GPIO,
		.pad_cfg   = (PAD_CTL_HYS_ENABLE),
	},

	/* CSPI pin settings */
	{
		.pin       = MX51_PIN_CSPI1_MOSI,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE |
			      PAD_CTL_PUE_KEEPER | PAD_CTL_100K_PD    |
			      PAD_CTL_DRV_HIGH   | PAD_CTL_SRE_FAST),
	}, {
		.pin       = MX51_PIN_CSPI1_MISO,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE |
			      PAD_CTL_PUE_KEEPER | PAD_CTL_100K_PD    |
			      PAD_CTL_DRV_HIGH   | PAD_CTL_SRE_FAST),
	}, {
		.pin       = MX51_PIN_CSPI1_SCLK,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE |
			      PAD_CTL_PUE_KEEPER | PAD_CTL_100K_PD    |
			      PAD_CTL_DRV_HIGH   | PAD_CTL_SRE_FAST),
	}, {
		.pin       = MX51_PIN_CSPI1_SS1,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_HYS_ENABLE | PAD_CTL_DRV_HIGH |
		              PAD_CTL_SRE_FAST   | PAD_CTL_PKE_ENABLE),
	}, {
		.pin       = MX51_PIN_DI1_PIN11,
		.mux_mode  = IOMUX_CONFIG_ALT7,
		.pad_cfg   = (PAD_CTL_HYS_ENABLE | PAD_CTL_DRV_HIGH   |
		              PAD_CTL_SRE_FAST   | PAD_CTL_PKE_ENABLE |
		              PAD_CTL_PUE_PULL),
	},

	/* NAND Flash pin setting */
	{
		/* NAND_WE_B(GPIO3[3]) */
		.pin	   = MX51_PIN_NANDF_WE_B,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_VOT_LOW  | PAD_CTL_HYS_NONE |
			      PAD_CTL_PKE_ENABLE   | PAD_CTL_47K_PU   |
			      PAD_CTL_DRV_HIGH),
	}, {
		/* NANDF_RE_B(GPIO3[4]) */
		.pin	   = MX51_PIN_NANDF_RE_B,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_VOT_LOW  | PAD_CTL_HYS_NONE |
			      PAD_CTL_PKE_ENABLE   | PAD_CTL_47K_PU   |
			      PAD_CTL_DRV_HIGH),
	}, {
		/* NANDF_ALE(GPIO3[5]) */
		.pin	   = MX51_PIN_NANDF_ALE,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_VOT_LOW  | PAD_CTL_PKE_ENABLE |
			      PAD_CTL_DRV_HIGH),
	}, {
		/* NANDF_CLE(GPIO3[6]) */
		.pin	   = MX51_PIN_NANDF_CLE,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_VOT_LOW  | PAD_CTL_PKE_ENABLE |
			      PAD_CTL_DRV_HIGH),
	}, {
		/* NANDF_WP_B(GPIO3[7]) */
		.pin	   = MX51_PIN_NANDF_WP_B,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_VOT_LOW  | PAD_CTL_HYS_NONE |
			      PAD_CTL_PKE_ENABLE   | PAD_CTL_100K_PU  |
			      PAD_CTL_DRV_HIGH),
	}, {
		/* NANDF_RB0(GPIO3[8]) */
		.pin	   = MX51_PIN_NANDF_RB0,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_VOT_LOW  | PAD_CTL_HYS_NONE |
			      PAD_CTL_PKE_ENABLE   | PAD_CTL_PUE_PULL |
 			      PAD_CTL_100K_PU      | PAD_CTL_DRV_LOW),
	}, {
		/* NANDF_RB1(GPIO3[9]) */
		.pin	   = MX51_PIN_NANDF_RB1,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_VOT_LOW  | PAD_CTL_HYS_NONE |
			      PAD_CTL_PKE_ENABLE   | PAD_CTL_PUE_PULL |
 			      PAD_CTL_100K_PU      | PAD_CTL_ODE_OPENDRAIN_NONE |
 			      PAD_CTL_DRV_LOW),
	}, {
		/* NANDF_CS0(GPIO3[16]) */
		.pin	   = MX51_PIN_NANDF_CS0,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_PKE_ENABLE   | PAD_CTL_DRV_HIGH),
	}, {
		/* NANDF_CS1(GPIO3[17]) */
		.pin	   = MX51_PIN_NANDF_CS1,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_PKE_ENABLE   | PAD_CTL_DRV_HIGH),
	}, {
		/* NANDF_D7(GPIO4[1]) */
		.pin	   = MX51_PIN_NANDF_D7,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_VOT_LOW  | PAD_CTL_HYS_NONE |
			      PAD_CTL_PKE_ENABLE   | PAD_CTL_PUE_KEEPER |
			      PAD_CTL_100K_PU      | PAD_CTL_DRV_HIGH),
	}, {
		/* NANDF_D6(GPIO4[2]) */
		.pin	   = MX51_PIN_NANDF_D6,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_VOT_LOW  | PAD_CTL_HYS_NONE |
			      PAD_CTL_PKE_ENABLE   | PAD_CTL_PUE_KEEPER |
			      PAD_CTL_100K_PU      | PAD_CTL_ODE_OPENDRAIN_NONE |
			      PAD_CTL_DRV_HIGH),
	}, {
		/* NANDF_D5(GPIO4[3]) */
		.pin	   = MX51_PIN_NANDF_D5,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_VOT_LOW  | PAD_CTL_HYS_NONE |
			      PAD_CTL_PKE_ENABLE   | PAD_CTL_PUE_KEEPER |
			      PAD_CTL_100K_PU      | PAD_CTL_DRV_HIGH),
	}, {
		/* NANDF_D4(GPIO4[4]) */
		.pin	   = MX51_PIN_NANDF_D4,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_VOT_LOW  | PAD_CTL_HYS_NONE |
			      PAD_CTL_PKE_ENABLE   | PAD_CTL_PUE_KEEPER |
			      PAD_CTL_100K_PU      | PAD_CTL_DRV_HIGH),
	}, {
		/* NANDF_D3(GPIO4[5]) */
		.pin	   = MX51_PIN_NANDF_D3,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_VOT_LOW  | PAD_CTL_HYS_NONE |
			      PAD_CTL_PKE_ENABLE   | PAD_CTL_PUE_KEEPER |
			      PAD_CTL_100K_PU      | PAD_CTL_DRV_HIGH),
	}, {
		/* NANDF_D2(GPIO4[6]) */
		.pin	   = MX51_PIN_NANDF_D2,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_VOT_LOW  | PAD_CTL_HYS_NONE |
			      PAD_CTL_PKE_ENABLE   | PAD_CTL_PUE_KEEPER |
			      PAD_CTL_100K_PU      | PAD_CTL_DRV_HIGH),
	}, {
		/* NANDF_D1(GPIO4[7]) */
		.pin	   = MX51_PIN_NANDF_D1,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_VOT_LOW  | PAD_CTL_HYS_NONE |
			      PAD_CTL_PKE_ENABLE   | PAD_CTL_PUE_KEEPER |
			      PAD_CTL_100K_PU      | PAD_CTL_DRV_HIGH),
	}, {
		/* NANDF_D0(GPIO4[8]) */
		.pin	   = MX51_PIN_NANDF_D0,
		.mux_mode  = IOMUX_CONFIG_ALT0,
		.pad_cfg   = (PAD_CTL_DRV_VOT_LOW  | PAD_CTL_HYS_NONE |
			      PAD_CTL_PKE_ENABLE   | PAD_CTL_PUE_KEEPER |
			      PAD_CTL_100K_PU      | PAD_CTL_DRV_HIGH),
	}, {
		/* NOT USE (FEC RESET) */
		.pin	   = MX51_PIN_EIM_A20,
		.mux_mode  = IOMUX_CONFIG_GPIO,
		.pad_cfg   = (PAD_CTL_HYS_NONE | PAD_CTL_PKE_NONE),
	}, {
		/* NOT USE (LID_CLOSE) */
		.pin	   = MX51_PIN_EIM_A24,
		.mux_mode  = IOMUX_CONFIG_GPIO,
		.pad_cfg   = (PAD_CTL_HYS_NONE | PAD_CTL_PKE_NONE),
	},
};

static struct mxc_iomux_pin_cfg __initdata mxc_fec_pins[] = {
	/* FEC pin settings */
	{
		/* FEC MDIO */
		.pin	   = MX51_PIN_EIM_EB2,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_22K_PU   | PAD_CTL_PKE_ENABLE |
			      PAD_CTL_PUE_PULL | PAD_CTL_HYS_ENABLE |
			      PAD_CTL_ODE_OPENDRAIN_ENABLE),
		.in_select = MUX_IN_FEC_FEC_MDI_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH0,
	}, {
		/* FEC MDC */
		.pin	   = MX51_PIN_NANDF_CS3,
		.mux_mode  = IOMUX_CONFIG_ALT2,
		.pad_cfg   = (PAD_CTL_DRV_HIGH),
	}, {
		/* FEC RDATA[3] */
		.pin	   = MX51_PIN_EIM_CS3,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
		.in_select = MUX_IN_FEC_FEC_RDATA_3_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH0,
	}, {
		/* FEC RDATA[2] */
		.pin	   = MX51_PIN_EIM_CS2,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
		.in_select = MUX_IN_FEC_FEC_RDATA_2_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH0,
	}, {
		/* FEC RDATA[1]*/
		.pin	   = MX51_PIN_EIM_EB3,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
		.in_select = MUX_IN_FEC_FEC_RDATA_1_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH0,
	}, {
		/* FEC RDATA[0]*/
		.pin	   = MX51_PIN_NANDF_D9,
		.mux_mode  = IOMUX_CONFIG_ALT2,
		.pad_cfg   = (PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
		.in_select = MUX_IN_FEC_FEC_RDATA_0_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH0,
	}, {
		/* FEC TDATA[3]*/
		.pin	   = MX51_PIN_NANDF_CS6,
		.mux_mode  = IOMUX_CONFIG_ALT2,
		.pad_cfg   = (PAD_CTL_DRV_HIGH),
	}, {
		/* FEC TDATA[2]*/
		.pin	   = MX51_PIN_NANDF_CS5,
		.mux_mode  = IOMUX_CONFIG_ALT2,
		.pad_cfg   = (PAD_CTL_DRV_HIGH),
	}, {
		/* FEC TDATA[1]*/
		.pin	   = MX51_PIN_NANDF_CS4,
		.mux_mode  = IOMUX_CONFIG_ALT2,
		.pad_cfg   = (PAD_CTL_DRV_HIGH),
	}, {
		/* FEC TDATA[0]*/
		.pin	   = MX51_PIN_NANDF_D8,
		.mux_mode  = IOMUX_CONFIG_ALT2,
		.pad_cfg   = (PAD_CTL_DRV_HIGH),
	}, {
		/* FEC TX_EN */
		.pin	   = MX51_PIN_NANDF_CS7,
		.mux_mode  = IOMUX_CONFIG_ALT1,
		.pad_cfg   = (PAD_CTL_DRV_HIGH),
	}, {
		/* FEC TX_ER */
		.pin	   = MX51_PIN_NANDF_CS2,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_DRV_HIGH),
	}, {
		/* FEC TX_CLK */
		.pin	   = MX51_PIN_NANDF_RDY_INT,
		.mux_mode  = IOMUX_CONFIG_ALT1,
		.pad_cfg   = (PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
		.in_select = MUX_IN_FEC_FEC_TX_CLK_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH0,
	}, {
		/* FEC COL */
		.pin	   = MX51_PIN_NANDF_RB2,
		.mux_mode  = IOMUX_CONFIG_ALT1,
		.pad_cfg   = (PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
		.in_select = MUX_IN_FEC_FEC_COL_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH0,
	}, {
		/* FEC RX_CLK */
		.pin	   = MX51_PIN_NANDF_RB3,
		.mux_mode  = IOMUX_CONFIG_ALT1,
		.pad_cfg   = (PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
		.in_select = MUX_IN_FEC_FEC_RX_CLK_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH0,
	}, {
		/* FEC CRS */
		.pin	   = MX51_PIN_EIM_CS5,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
		.in_select = MUX_IN_FEC_FEC_CRS_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH0,
	}, {
		/* FEC RX_ER */
		.pin	   = MX51_PIN_EIM_CS4,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
		.in_select = MUX_IN_FEC_FEC_RX_ER_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH0,
	}, {
		/* FEC RX_DV */
		.pin	   = MX51_PIN_NANDF_D11,
		.mux_mode  = IOMUX_CONFIG_ALT2,
		.pad_cfg   = (PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
		.in_select = MUX_IN_FEC_FEC_RX_DV_SELECT_INPUT,
		.in_mode   = INPUT_CTL_PATH0,
	}, {
		/* FEC PHY RESET */
		.pin	   = MX51_PIN_EIM_A20,
		.mux_mode  = IOMUX_CONFIG_GPIO,
		.pad_cfg   = (PAD_CTL_PKE_ENABLE),
	}, {
		/* FEC PHY INT */
		.pin	   = MX51_PIN_EIM_A21,
		.mux_mode  = IOMUX_CONFIG_GPIO,
	},
};

static struct mxc_iomux_pin_cfg __initdata mxc_wlan_pins[] = {
	/* WLAN pin setting */
	{
		/* EIM_OE(GPIO2[24]) -> WL_RESETN */
		.pin	   = MX51_PIN_EIM_OE,
		.mux_mode  = IOMUX_CONFIG_ALT1,
		.pad_cfg   = (PAD_CTL_SRE_FAST		 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_PKE_ENABLE),
	}, {
		/* EIM_CS0(GPIO2[25]) <- WL_WKU_REQ */
		.pin	   = MX51_PIN_EIM_CS0,
		.mux_mode  = IOMUX_CONFIG_ALT1		 | IOMUX_CONFIG_SION,
		.pad_cfg   = (PAD_CTL_SRE_FAST		 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_PKE_ENABLE),
	}, {
		/* EIM_CS1(GPIO2[26]) -> WL_WKU */
		.pin	   = MX51_PIN_EIM_CS1,
		.mux_mode  = IOMUX_CONFIG_ALT1,
		.pad_cfg   = (PAD_CTL_SRE_FAST		 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_PKE_ENABLE),
	}, {
		/* EIM_DTACK(GPIO2[31]) <- WL_RESON */
		.pin	   = MX51_PIN_EIM_DTACK,
		.mux_mode  = IOMUX_CONFIG_ALT1		 | IOMUX_CONFIG_SION,
		.pad_cfg   = (PAD_CTL_SRE_SLOW		 | PAD_CTL_DRV_HIGH   |
			      PAD_CTL_PKE_ENABLE),
	}, {
		/* NANDF_CS6(GPIO3[23]) -> WLBT_CLK_EN */
		.pin	   = MX51_PIN_NANDF_CS7,
		.mux_mode  = IOMUX_CONFIG_ALT3,
		.pad_cfg   = (PAD_CTL_DRV_HIGH	 | PAD_CTL_100K_PD  |
			      PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_NONE |
			      PAD_CTL_DRV_VOT_LOW),
	}
};

void __init mx51_erdos_io_init(void)
{
	int i;
	struct mxc_iomux_pin_cfg *tbl;

	enum iomux_pins gpios[] = {
		MX51_PIN_CSI1_D9,
		MX51_PIN_CSI1_HSYNC,
		MX51_PIN_CSI1_VSYNC,
		MX51_PIN_CSI2_D12,
		MX51_PIN_CSI2_D13,
		MX51_PIN_CSI2_D19,
		MX51_PIN_CSPI1_RDY,
		MX51_PIN_DISPB2_SER_CLK,
		MX51_PIN_DISPB2_SER_CLK,
		MX51_PIN_DISPB2_SER_CLK,
		MX51_PIN_DISPB2_SER_DIO,
		MX51_PIN_DI1_D0_CS,
		MX51_PIN_DI1_PIN12,
		MX51_PIN_EIM_A19,
		MX51_PIN_EIM_A21,
		MX51_PIN_EIM_A23,
		MX51_PIN_EIM_A25,
		MX51_PIN_EIM_A27,
		MX51_PIN_EIM_D17,
		MX51_PIN_EIM_D20,
		MX51_PIN_EIM_D21,
		MX51_PIN_EIM_D22,
		MX51_PIN_EIM_D23,
		MX51_PIN_EIM_EB2,
		MX51_PIN_EIM_EB3,
		MX51_PIN_GPIO_NAND,
		MX51_PIN_GPIO1_0,
		MX51_PIN_GPIO1_7,
		MX51_PIN_GPIO1_7,
		MX51_PIN_GPIO1_7,
		MX51_PIN_GPIO1_8,
		MX51_PIN_NANDF_D10,
		MX51_PIN_NANDF_D12,
		MX51_PIN_NANDF_D13,
		MX51_PIN_NANDF_D14,
		MX51_PIN_NANDF_D15,
	};

	for (i = 0; i < ARRAY_SIZE(gpios); i++)
		gpio_request(IOMUX_TO_GPIO(gpios[i]), "init");
	/*
	 * USB HUB RESET-OFF
	 *  Output data setting before IOMUX setting.
	 */
//	mxc_set_gpio_dataout(MX51_PIN_GPIO1_7, 1);
//	mxc_set_gpio_direction(MX51_PIN_GPIO1_7, 0);
//	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_7), 1);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_7), 1);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_GPIO1_7),1);

	/* GPIO config */
	tbl = &(mxc_iomux_pins[0]);
	for (i = 0; i < ARRAY_SIZE(mxc_iomux_pins); i++ , tbl++ ) {
		/*
		 * check board QA0
		 */
		if (is_board_qa0() == 1) {
			if (tbl->pin == MX51_PIN_NANDF_CS7) {
				;	/* don't setting */
			} else {
				u16 pad_cfg = tbl->pad_cfg;
				if ((tbl->pin == MX51_PIN_SD2_CMD) ||
				    (tbl->pin == MX51_PIN_SD2_CLK) ||
				    (tbl->pin == MX51_PIN_SD2_DATA0) ||
				    (tbl->pin == MX51_PIN_SD2_DATA1) ||
				    (tbl->pin == MX51_PIN_SD2_DATA2) ||
				    (tbl->pin == MX51_PIN_SD2_DATA3)) {
					pad_cfg &= ~PAD_CTL_DRV_VOT_LOW;
				}
				mxc_request_iomux(tbl->pin, tbl->mux_mode);
				if (pad_cfg) {
					mxc_iomux_set_pad(tbl->pin, pad_cfg);
				}
				if (tbl->in_select) {
					mxc_iomux_set_input(tbl->in_select, tbl->in_mode);
				}
			}
		} else {
			mxc_request_iomux(tbl->pin, tbl->mux_mode);
			if (tbl->pad_cfg) {
				mxc_iomux_set_pad(tbl->pin, tbl->pad_cfg);
			}
			if (tbl->in_select) {
				mxc_iomux_set_input(tbl->in_select, tbl->in_mode);
			}
		}
	}

	/**
	 * Battery
	 *   bq24105
	 */

	/* Battery REFON(gpio3[6]) to low (OFF) */
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIO),0);

	/* Battery CHGCTRL(gpio3[7]) to low (allow charging) */
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_CLK), 1);	/* stop charging */
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_CLK),1);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_CLK), 0);	/* allow charging */

	/* Battery status pins */
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_NANDF_D13));	/* PG    */
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_NANDF_D12));	/* STAT1 */
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_NANDF_D10));	/* STAT2 */

	/* GPIO dir config */
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_GPIO1_8));	/* PMIC INT */
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_GPIO1_0));	/* SD1 CD */

	/**
	 * MX51_PIN_EIM_A21 : PHY_IRQ/TXERR
	 * setup for GPIO input mode, not used TX_ERR
	 */
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_EIM_A21));

	/* Drive 26M_OSC_EN line high */
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN12),1);

	/* Drive USB_CLK_EN_B line low */
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_D17),0);

	/* MX51_PIN_EIM_D21 - De-assert USB PHY RESETB */
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_D21),1);

	/* hphone_det_b */
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_NANDF_D14));

	/* audio_clk_en_b */
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSPI1_RDY),0);

	/* AUDAMP_CTRL1 */
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_A23),0);

	/* AUDAMP_CTRL2 */
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_A25),0);

	/* Deassert VGA reset to free i2c bus */
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_A19),1);

	/* LCD VCC pin setting */ /* LCD LVDS Enable */
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DI1_D0_CS), 1);

	/* LCD VCC pin setting */ /* LCD 3V3 on */
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSI2_D12), 1);
	mdelay(180);
	/* LCD 5V on */
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSI2_D13), 1);

	/* OJ6SH-T25 Optical Joystick GPIO pin Direction Setting */
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_CSI1_D9));
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSI1_VSYNC), 0);
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_CSI1_HSYNC));

	/* mxc_etk ALERT pin */
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_NANDF_D15));

	/* mxc_etk RESET pin */
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_GPIO_NAND),0);

	/* BlueTooth Reset */
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_EIM_D20));
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_D22),0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_D23),0);

	/* Left/Right click switch pin config */
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_EIM_EB2));
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_EIM_EB3));

	/* Cover Switch pin config */
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_CSI2_D19));

	/* Power Switch pin config */
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_EIM_A27));

	for (i = 0; i < ARRAY_SIZE(gpios); i++)
		gpio_free(IOMUX_TO_GPIO(gpios[i]));
}

static volatile int gpio_wlan_initialized = 0;	/* gpio_wlan_init() call mark */
int gpio_wlan_start(void)
{
	int i;
	static int iomux_requested = 0;

	/*
	 * check already initialize
	 */
	if (gpio_wlan_initialized) {
		return (0);
	}

	gpio_wlan_initialized = 1;

	/* setup WLAN_PIN */
	if (!iomux_requested) {
	for (i = 0; i < ARRAY_SIZE(mxc_wlan_pins); i++) {
		mxc_request_iomux(mxc_wlan_pins[i].pin,
				  mxc_wlan_pins[i].mux_mode);
		if (mxc_wlan_pins[i].pad_cfg) {
			mxc_iomux_set_pad(mxc_wlan_pins[i].pin,
					  mxc_wlan_pins[i].pad_cfg);
		}
		if (mxc_wlan_pins[i].in_select) {
			mxc_iomux_set_input(mxc_wlan_pins[i].in_select,
					    mxc_wlan_pins[i].in_mode);
		}
	}
		iomux_requested = 1;
	}

	if (is_board_qa0() == 0) {
		/* WLBT_CLK_EN */
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_NANDF_CS7), 0);
	}
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_OE),0);	/* WL_RESETN */
	gpio_direction_input (IOMUX_TO_GPIO(MX51_PIN_EIM_CS0));	/* WL_WKU_REQ */
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_CS1),0);	/* WL_WKU */
	gpio_direction_input (IOMUX_TO_GPIO(MX51_PIN_EIM_DTACK));	/* WL_RESON */

	if (is_board_qa0() == 0) {	/* for board qa1 */
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_OE), 0);   /* Reset */
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_NANDF_CS7), 0);/* Clk Disable */
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_NANDF_CS7), 1);/* Clk Enable */
		mdelay(3);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_OE), 1);   /* Reset release */
	} else {
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_OE), 0);   /* Reset */
		mdelay(1);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_OE), 1);   /* Reset release */
	}

	gpio_wlan_initialized = 1;

	return (0);
}
EXPORT_SYMBOL(gpio_wlan_start);

void gpio_wlan_stop(void)
{
	int i;

	/*
	 * check already initialize
	 */
	if (!gpio_wlan_initialized) {
		return;
	}

	if (is_board_qa0() == 0) {	/* for board qa1 */
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_OE), 0);   /* Reset */
		i=100;
		while(i--) {
			if (!gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_EIM_DTACK))) {
				break;
			}
		}
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_NANDF_CS7), 0);/* Clk Disable */
	} else {
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_OE), 0);   /* Reset */
	}

	gpio_wlan_initialized = 0;
}
EXPORT_SYMBOL(gpio_wlan_stop);

static int __init gpio_fec_config(void)
{
	int i;

	if (is_fec()) {
		/* setup FEC_PIN */
		mxc_free_iomux(MX51_PIN_EIM_EB2, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX51_PIN_EIM_EB3, IOMUX_CONFIG_GPIO);
		for (i = 0; i < ARRAY_SIZE(mxc_fec_pins); i++) {
			mxc_request_iomux(mxc_fec_pins[i].pin,
					  mxc_fec_pins[i].mux_mode);
			if (mxc_fec_pins[i].pad_cfg) {
				mxc_iomux_set_pad(mxc_fec_pins[i].pin,
						  mxc_fec_pins[i].pad_cfg);
			}
			if (mxc_fec_pins[i].in_select) {
				mxc_iomux_set_input(mxc_fec_pins[i].in_select,
						    mxc_fec_pins[i].in_mode);
			}
		}
		/* reset FEC PHY */
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_A20), 0);
		msleep(10);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A20), 1);
	}
	return 0;
}
subsys_initcall(gpio_fec_config);

void gpio_lcd_power(int on)
{
	if (on) {
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSI2_D12),1);
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DI1_D0_CS),1);
		mdelay(180);
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSI2_D13),1);

	} else {
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI2_D13), 0); /* 4_10 */
		mdelay(5);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI2_D12), 0); /* 4_9 */
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_D0_CS), 0); /* 3_3 */
	}
}
EXPORT_SYMBOL(gpio_lcd_power);

/*
 * Battery Charge status
 */
int gpio_charge_status(void)
{
	int stat1  = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_NANDF_D12));
	int stat2  = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_NANDF_D10));

	return ( (stat1 << 1) | stat2 );
}
EXPORT_SYMBOL(gpio_charge_status);

/*
 * Battery/DCin voltage select for measurement
 *  val : 1(ON) / 0(OFF)
 */
void gpio_refon(int val)
{
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIO), val);
}
EXPORT_SYMBOL(gpio_refon);

void mx51_erdos_gpio_spi_chipselect_active(int cspi_mode, int status, int chipselect)
{
	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			mxc_request_iomux(MX51_PIN_CSPI1_SS0,
			                  IOMUX_CONFIG_ALT0);
			mxc_iomux_set_pad(MX51_PIN_CSPI1_SS0,
			                  PAD_CTL_HYS_ENABLE |
			                  PAD_CTL_PKE_ENABLE |
			                  PAD_CTL_DRV_HIGH   |
			                  PAD_CTL_SRE_FAST);
			break;
		default:
			mxc_request_iomux(MX51_PIN_CSPI1_SS0,
			                  IOMUX_CONFIG_GPIO);
			gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS0),1 & (~status));
			break;
		}
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(mx51_erdos_gpio_spi_chipselect_active);

void mx51_erdos_gpio_spi_chipselect_inactive(int cspi_mode, int status, int chipselect)
{
	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			mxc_free_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_GPIO);
			mxc_free_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_GPIO);
			break;
		default:
			mxc_free_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_GPIO);
			break;
		}
		break;
	}
}
EXPORT_SYMBOL(mx51_erdos_gpio_spi_chipselect_inactive);

/*
 * all CSPI SSx inactive when reset
 */
void mx51_erdos_gpio_spi_chipselect_all_inactive (void)
{
	mxc_free_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_GPIO);
}

void gpio_cap1014_wakeup(void)
{
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_NANDF_D15), 1);
	mdelay(5);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_NANDF_D15), 0);
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_NANDF_D15));
	mdelay(30);
}
EXPORT_SYMBOL(gpio_cap1014_wakeup);

void gpio_cap1014_reset(void)
{
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_GPIO_NAND), 1);
	mdelay(5);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_GPIO_NAND), 0);
	mdelay(30);
}
EXPORT_SYMBOL(gpio_cap1014_reset);

void gpio_oj6sh_shutdown(int shutdown)
{
	if (shutdown) {
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI1_VSYNC), 1);
	} else {
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI1_VSYNC), 0);
	}
}
EXPORT_SYMBOL(gpio_oj6sh_shutdown);


void gpio_audio_sgtl_clk(int isEnable)
{
	static int clk_en = 0;

	if (isEnable != clk_en){
		if (isEnable == 0) {
			/* do disable */
			/* audio_clk_en_b */
			gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSPI1_RDY), 1);
			clk_en = isEnable;
		} else {
			/* do enable */
			/* audio_clk_en_b */
			gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSPI1_RDY), 0);
			udelay(1);
			clk_en = isEnable;
		}
	}
}
EXPORT_SYMBOL(gpio_audio_sgtl_clk);

void gpio_usb_power(struct platform_device *pdev, int on)
{
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	if (pdev == 0) {
		return;
	}
	if (strcmp (pdata->name, "Host 1") == 0) {
		/*
		 * check HOST-1
		 */
		;
	} else if (strcmp (pdata->name, "DR") == 0) {
		/*
		 * DR otg-host
		 */
		extern void dr_udc_suspend (void);
		extern void dr_udc_resume (void);
		if (on) {
			dr_udc_resume ();
		} else {
			dr_udc_suspend ();
		}
		return;
	} else {
		return;
	}
	if (on) {
		/* do resume */
		/* HUB RESET release */
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_7), 1);

		/* Drive 26M_OSC_EN line high 3_1 */
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN12), 1);

		/* Drive USB_CLK_EN_B line low  2_1 */
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D17), 0);

		/* MX51_PIN_EIM_D21 - De-assert USB PHY RESETB */
		mdelay ( 10 );
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D21), 1);
		mdelay ( 5 );
	} else {
		/* do suspend */
		/* MX51_PIN_EIM_D21 - assert USB PHY RESETB */
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D21), 0);

		/* Drive USB_CLK_EN_B line high 2_1 */
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D17), 1);
			
		/* Drive 26M_OSC_EN line low 3_1 */
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN12), 0);

		/* MX51_PIN_EIM_D21 - assert USB PHY RESETB */
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_7), 0);
	}
}
EXPORT_SYMBOL(gpio_usb_power);

/*
 * gpio_battery_enable - Battery Power enable/disable
 *  enable : 1(Enable) / 0(Disable)
 */
void gpio_battery_enable (int enable)
{
	/*
	 * BAT_AC2(GPIO1_9) High(Battery Power) / Low(DCinput Power)
	 */
	gpio_set_value (IOMUX_TO_GPIO(MX51_PIN_GPIO1_9), enable);
}
EXPORT_SYMBOL(gpio_battery_enable);

/*
 * gpio_poweroff - prepare gpio setting for poweroff
 */
void gpio_poweroff (void)
{
	/*
	 * setting DCinput Power mode
	 */
	gpio_battery_enable ( 0 );

	/*
	 * SYS_ON_OFF_CTL(GPIO1_23) OFF
	 */
	gpio_set_value (IOMUX_TO_GPIO(MX51_PIN_UART3_TXD), 0);
}

/*
 * get_gpio_sw_status - current sw status
 */
int get_gpio_sw_status (void)
{
	extern int pmic_get_dcinput_voltage (unsigned short *voltage);
	unsigned short dcin;
	int rc;
	int val = 0;
	/*
	 *  b07 -
	 *  b06 - BATT COMP (1:COMPLETE)
	 *  b05 - BATT CHARG(1:CHARGE)
	 *  b04 - DCinput(AC adapter)(1:GOOD)
	 *  b03 - HeadPhone (1:CONNECT)
	 *  b02 - COVER-SW  (1:OPEN)
	 *  b01 - LEFT-SW   (1:ON)
	 *  b00 - RIGHT-SW  (1:ON)
	 */
	if (gpio_get_value (IOMUX_TO_GPIO(MX51_PIN_NANDF_D10)) == 1) {	// BATT COMP
		val |= (1 << 6);
	}
	if (gpio_get_value (IOMUX_TO_GPIO(MX51_PIN_NANDF_D12)) == 1) {	// BATT CHARG
		val |= (1 << 5);
	}
	rc = pmic_get_dcinput_voltage (&dcin);			// DCinput
	if (rc == 0) {
		val |= (1 << 4);
	}
	if (gpio_get_value (IOMUX_TO_GPIO(MX51_PIN_NANDF_D14)) == 0) {	// HeadPhone
		val |= (1 << 3);
	}
	if (gpio_get_value (IOMUX_TO_GPIO(MX51_PIN_CSI2_D19)) == 1) {	// COVER-SW
		val |= (1 << 2);
	}
	if (gpio_get_value (IOMUX_TO_GPIO(MX51_PIN_EIM_EB2)) == 0) {	// LEFT-SW
		val |= (1 << 1);
	}
	if (gpio_get_value (IOMUX_TO_GPIO(MX51_PIN_EIM_EB3)) == 0) {	// RIGHT-SW
		val |= (1 << 0);
	}
	return val;
}
EXPORT_SYMBOL(get_gpio_sw_status);

void usb_reset(int on)
{
	/*
	 * do not need reset, already reset at gpio_usb_power().
	 */
	return;
}
EXPORT_SYMBOL(usb_reset);

int get_gpio_cover_sw(void)
{
	return gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_CSI2_D19));
}
EXPORT_SYMBOL(get_gpio_cover_sw);

int get_gpio_power_sw(void)
{
	return gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A27));
}
EXPORT_SYMBOL(get_gpio_power_sw);

#ifdef CONFIG_BT_HCIUART
#ifdef CONFIG_ALLOC_FUNC_KEY
#include <linux/input.h>
#include <asm/func_key.h>

static void gpio_bt_reset(int id, int value, void *arg)
{
	static int bt_on = 0;
	/* press */
	if (value == 1) {
		if (bt_on) {
			/* reset */
			gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D22), 0);
			gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D23), 0);
			bt_on = 0;
		} else {
			/* reset release */
			gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D22), 1);
			gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D23), 1);
			bt_on = 1;
		}
	}
}

static int __init input_init(void)
{
	return fnkey_register(FNKEY_ID_BLUETOOTH, gpio_bt_reset, NULL);
}
subsys_initcall(input_init);
#endif
#endif
