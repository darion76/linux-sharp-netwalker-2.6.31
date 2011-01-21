/*
 * Copyright 2007-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

#include "iomux.h"

/*!
 * @file mach-mx51/mx51_babbage_gpio.c
 *
 * @brief This file contains all the GPIO setup functions for the board.
 *
 * @ingroup GPIO
 */

static struct mxc_iomux_pin_cfg __initdata mxc_iomux_pins[] = {
	{
	 MX51_PIN_EIM_A16, IOMUX_CONFIG_GPIO,
	 },
	{
	 MX51_PIN_EIM_A17, IOMUX_CONFIG_GPIO,
	 },
	{
	 MX51_PIN_EIM_A18, IOMUX_CONFIG_GPIO,
	 },
	{
	 MX51_PIN_EIM_A19, IOMUX_CONFIG_GPIO,
	 },
	{
	 MX51_PIN_EIM_A20, IOMUX_CONFIG_GPIO,
	 (PAD_CTL_PKE_ENABLE),
	 },
	{
	 MX51_PIN_EIM_A21, IOMUX_CONFIG_GPIO,
	 },
	{
	 MX51_PIN_EIM_A22, IOMUX_CONFIG_GPIO,
	 },
	{
	 MX51_PIN_EIM_A23, IOMUX_CONFIG_GPIO,
	 },
	{			/*MDIO */
	 MX51_PIN_EIM_EB2, IOMUX_CONFIG_ALT3,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_ENABLE |
	  PAD_CTL_22K_PU | PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE |
	  PAD_CTL_PUE_PULL),
	 },
	{			/*RDATA[1] */

	 MX51_PIN_EIM_EB3, IOMUX_CONFIG_ALT3,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE),
	 },
	{			/*RDATA[2] */
	 MX51_PIN_EIM_CS2, IOMUX_CONFIG_ALT3,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE),
	 },
	{			/*RDATA[3] */
	 MX51_PIN_EIM_CS3, IOMUX_CONFIG_ALT3,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE),
	 },
	{			/*RX_ER */
	 MX51_PIN_EIM_CS4, IOMUX_CONFIG_ALT3,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE),
	 },
	{			/*CRS */
	 MX51_PIN_EIM_CS5, IOMUX_CONFIG_ALT3,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE),
	 },
	{
	 MX51_PIN_EIM_DTACK, IOMUX_CONFIG_GPIO,
	 (PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PU),
	 },
	{
	 MX51_PIN_EIM_LBA, IOMUX_CONFIG_GPIO,
	 },
	{
	 MX51_PIN_NANDF_RB2, IOMUX_CONFIG_ALT1,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE),
	 },
	{
	 MX51_PIN_NANDF_RB3, IOMUX_CONFIG_ALT1,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE),
	 },
	{
	 MX51_PIN_NANDF_RB4, IOMUX_CONFIG_ALT1,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE),
	 },
	{			/*RDATA[0] */
	 MX51_PIN_NANDF_RB6, IOMUX_CONFIG_ALT1,
	 (PAD_CTL_DRV_VOT_HIGH | PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE),
	 },
	{			/*TDATA[0] */
	 MX51_PIN_NANDF_RB7, IOMUX_CONFIG_ALT1,
	 (PAD_CTL_DRV_VOT_HIGH | PAD_CTL_DRV_HIGH),
	 },
	{
	 MX51_PIN_NANDF_CS0, IOMUX_CONFIG_GPIO,
	 PAD_CTL_100K_PU,
	 },
	{
	 MX51_PIN_NANDF_CS1, IOMUX_CONFIG_GPIO,
	 },
	{			/*TX_ER */
	 MX51_PIN_NANDF_CS2, IOMUX_CONFIG_ALT2,
	 (PAD_CTL_DRV_VOT_HIGH | PAD_CTL_DRV_HIGH),
	 },
	{
	 MX51_PIN_NANDF_CS3, IOMUX_CONFIG_ALT2,
	 (PAD_CTL_DRV_VOT_HIGH | PAD_CTL_DRV_HIGH),
	 },
	{			/*TDATA[1] */
	 MX51_PIN_NANDF_CS4, IOMUX_CONFIG_ALT2,
	 (PAD_CTL_DRV_VOT_HIGH | PAD_CTL_DRV_HIGH),
	 },
	{			/*TDATA[2] */
	 MX51_PIN_NANDF_CS5, IOMUX_CONFIG_ALT2,
	 (PAD_CTL_DRV_VOT_HIGH | PAD_CTL_DRV_HIGH),
	 },
	{			/*TDATA[3] */
	 MX51_PIN_NANDF_CS6, IOMUX_CONFIG_ALT2,
	 (PAD_CTL_DRV_VOT_HIGH | PAD_CTL_DRV_HIGH),
	 },
	{			/*TX_EN */
	 MX51_PIN_NANDF_CS7, IOMUX_CONFIG_ALT1,
	 (PAD_CTL_DRV_VOT_HIGH | PAD_CTL_DRV_HIGH),
	 },
	{			/*TX_CLK */
	 MX51_PIN_NANDF_RDY_INT, IOMUX_CONFIG_ALT1,
	 (PAD_CTL_DRV_VOT_HIGH | PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE),
	 },
	{
	 MX51_PIN_GPIO1_8, IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION,
	 (PAD_CTL_SRE_SLOW | PAD_CTL_DRV_MEDIUM | PAD_CTL_100K_PU |
	  PAD_CTL_HYS_ENABLE | PAD_CTL_DRV_VOT_HIGH),
	 },
	{
	 MX51_PIN_DI_GP4, IOMUX_CONFIG_ALT4,
	 },
	{
	 MX51_PIN_DISPB2_SER_DIN, IOMUX_CONFIG_GPIO,
	 0,
	 MUX_IN_GPIO3_IPP_IND_G_IN_5_SELECT_INPUT,
	 INPUT_CTL_PATH1,
	 },
#ifdef CONFIG_FB_MXC_CLAA_WVGA_SYNC_PANEL
	{	/* DISP2_DAT16 */
	 MX51_PIN_DISP1_DAT22, IOMUX_CONFIG_ALT5,
	 },
	{	/* DISP2_DAT17 */
	 MX51_PIN_DISP1_DAT23, IOMUX_CONFIG_ALT5,
	 },
	{
	 MX51_PIN_DI1_D1_CS, IOMUX_CONFIG_ALT4,
	 (PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER | PAD_CTL_DRV_HIGH |
	  PAD_CTL_SRE_FAST),
	 MUX_IN_GPIO3_IPP_IND_G_IN_4_SELECT_INPUT, INPUT_CTL_PATH1,
	 },
#endif
	 /* LVDS GPIO control */
	 {
	 MX51_PIN_DI1_D0_CS, IOMUX_CONFIG_ALT4,
	 (PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER | PAD_CTL_DRV_HIGH |
	  PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_CSI2_D12, IOMUX_CONFIG_ALT3,
	 (PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER | PAD_CTL_DRV_HIGH |
	  PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_CSI2_D13, IOMUX_CONFIG_ALT3,
	 (PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER | PAD_CTL_DRV_HIGH |
	  PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_NANDF_D12, IOMUX_CONFIG_GPIO,
	 0,
	 },
	{
	 MX51_PIN_I2C1_CLK, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
	 0x1E4,
	 },
	{
	 MX51_PIN_I2C1_DAT, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
	 0x1E4,
	 },
	{
	 MX51_PIN_GPIO1_2, IOMUX_CONFIG_ALT1,
	 },
	{
	 MX51_PIN_GPIO1_3, IOMUX_CONFIG_ALT2 | IOMUX_CONFIG_SION,
	 (PAD_CTL_SRE_FAST | PAD_CTL_ODE_OPENDRAIN_ENABLE | PAD_CTL_DRV_HIGH |
	  PAD_CTL_100K_PU | PAD_CTL_HYS_ENABLE),
	 MUX_IN_I2C2_IPP_SDA_IN_SELECT_INPUT, INPUT_CTL_PATH3,
	 },
	{
	 MX51_PIN_USBH1_STP, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
	  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	 },
	{			/* USBH1_CLK */
	 MX51_PIN_USBH1_CLK, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
	  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS),
	 },
	{			/* USBH1_DIR */
	 MX51_PIN_USBH1_DIR, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
	  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS),
	 },
	{			/* USBH1_NXT */
	 MX51_PIN_USBH1_NXT, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
	  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS),
	 },
	{			/* USBH1_DATA0 */
	 MX51_PIN_USBH1_DATA0, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
	  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	 },
	{			/* USBH1_DATA1 */
	 MX51_PIN_USBH1_DATA1, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
	  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	 },
	{			/* USBH1_DATA2 */
	 MX51_PIN_USBH1_DATA2, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
	  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	 },
	{			/* USBH1_DATA3 */
	 MX51_PIN_USBH1_DATA3, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
	  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	 },
	{			/* USBH1_DATA4 */
	 MX51_PIN_USBH1_DATA4, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
	  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	 },
	{			/* USBH1_DATA5 */
	 MX51_PIN_USBH1_DATA5, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
	  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	 },
	{			/* USBH1_DATA6 */
	 MX51_PIN_USBH1_DATA6, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
	  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	 },
	{			/* USBH1_DATA7 */
	 MX51_PIN_USBH1_DATA7, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
	  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	 },
	{
	 MX51_PIN_SD1_CMD, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
	 (PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_SD1_CLK, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
	 (PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_SD1_DATA0, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_SD1_DATA1, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_SD1_DATA2, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_SD1_DATA3, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_GPIO1_0, IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU),
	 },
	{
	 MX51_PIN_GPIO1_1, IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU),
	 },
	{
	 MX51_PIN_SD2_CMD, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
	 (PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_SD2_CLK, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
	 (PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_SD2_DATA0, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_SD2_DATA1, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_SD2_DATA2, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_SD2_DATA3, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_GPIO1_4, IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU),
	 },
	{
	 MX51_PIN_GPIO1_5, IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU),
	 },
	{
	 MX51_PIN_GPIO1_6, IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU),
	 },
	{			/* Detect pin GPIO BB2.0 and BB2.5 */
	 MX51_PIN_UART3_RXD, IOMUX_CONFIG_ALT3,
	 (PAD_CTL_HYS_NONE | PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER |
	  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_UART1_RXD, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
	  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST),
	 MUX_IN_UART1_IPP_UART_RXD_MUX_SELECT_INPUT,
	 INPUT_CTL_PATH0,
	 },
	{
	 MX51_PIN_UART1_TXD, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
	  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST),
	 },
	{
	 MX51_PIN_UART1_RTS, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
	  PAD_CTL_DRV_HIGH),
	 MUX_IN_UART1_IPP_UART_RTS_B_SELECT_INPUT,
	 INPUT_CTL_PATH0,
	 },
	{
	 MX51_PIN_UART1_CTS, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
	  PAD_CTL_DRV_HIGH),
	 },
	{
	 MX51_PIN_AUD3_BB_TXD, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
	  PAD_CTL_100K_PU | PAD_CTL_HYS_NONE | PAD_CTL_DDR_INPUT_CMOS |
	  PAD_CTL_DRV_VOT_LOW),
	 },
	{
	 MX51_PIN_AUD3_BB_RXD, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
	  PAD_CTL_100K_PU | PAD_CTL_HYS_NONE | PAD_CTL_DDR_INPUT_CMOS |
	  PAD_CTL_DRV_VOT_LOW),
	 },
	{
	 MX51_PIN_AUD3_BB_CK, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
	  PAD_CTL_100K_PU | PAD_CTL_HYS_NONE | PAD_CTL_DDR_INPUT_CMOS |
	  PAD_CTL_DRV_VOT_LOW),
	 },
	{
	 MX51_PIN_AUD3_BB_FS, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
	  PAD_CTL_100K_PU | PAD_CTL_HYS_NONE | PAD_CTL_DDR_INPUT_CMOS |
	  PAD_CTL_DRV_VOT_LOW),
	 },
	{
	 MX51_PIN_CSPI1_SS1, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
	  PAD_CTL_SRE_FAST),
	 },
	/* Camera on expansion board */
	{	/* camera reset */
	 MX51_PIN_EIM_D23, IOMUX_CONFIG_ALT1,
	 (PAD_CTL_HYS_NONE | PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER |
	  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST),
	 },
	{	/* camera low power */
	 MX51_PIN_CSI2_D19, IOMUX_CONFIG_ALT3,
	 (PAD_CTL_HYS_NONE | PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_LOW |
	  PAD_CTL_SRE_SLOW),
	 },
	{	/* CSI1_DATA_EN need to be pulled up */
	 MX51_PIN_DI_GP3, IOMUX_CONFIG_ALT3,
	 (PAD_CTL_HYS_NONE | PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
	  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST),
	 MUX_IN_HSC_MIPI_MIX_IPP_IND_SENS2_DATA_EN_SELECT_INPUT,
	 INPUT_CTL_PATH1,
	 },
	{
	 MX51_PIN_CSI1_D10, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	 },
	{
	 MX51_PIN_CSI1_D11, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	 },
	{
	 MX51_PIN_CSI1_D12, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	 },
	{
	 MX51_PIN_CSI1_D13, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	 },
	{
	 MX51_PIN_CSI1_D14, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	 },
	{
	 MX51_PIN_CSI1_D15, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	 },
	{
	 MX51_PIN_CSI1_D16, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	 },
	{
	 MX51_PIN_CSI1_D17, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	 },
	{
	 MX51_PIN_CSI1_D18, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	 },
	{
	 MX51_PIN_CSI1_D19, IOMUX_CONFIG_ALT0, PAD_CTL_HYS_NONE,
	 },
	{
	 MX51_PIN_CSI1_VSYNC, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_HYS_NONE | PAD_CTL_SRE_SLOW),
	 },
	{
	 MX51_PIN_CSI1_HSYNC, IOMUX_CONFIG_ALT0,
	 (PAD_CTL_HYS_NONE | PAD_CTL_SRE_SLOW),
	 },
	{
	 MX51_PIN_EIM_D18, IOMUX_CONFIG_GPIO,
	 (PAD_CTL_DRV_HIGH | PAD_CTL_PKE_ENABLE |
	  PAD_CTL_PUE_KEEPER | PAD_CTL_100K_PU | PAD_CTL_SRE_FAST),
	 },
};

static int __initdata enable_w1 = { 0 };
static int __init w1_setup(char *__unused)
{
	enable_w1 = 1;
	return 1;
}

__setup("w1", w1_setup);

void __init mx51_babbage_io_init(void)
{
	int i;

	/* Work-around For external USB HUB chip to use default configuration
	   by reseting hub with i2c lines pulled low */
	mxc_request_iomux(MX51_PIN_GPIO1_7, IOMUX_CONFIG_GPIO);
	mxc_iomux_set_pad(MX51_PIN_GPIO1_7, PAD_CTL_DRV_HIGH |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_7), "gpio1_7");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_GPIO1_7), 0);

	if (cpu_is_mx51_rev(CHIP_REV_1_1) == 1) {
		/* Drive I2C1 SDA line low */
		mxc_request_iomux(MX51_PIN_GPIO1_3, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_GPIO1_3, PAD_CTL_DRV_HIGH |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_3), "gpio1_3");
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_GPIO1_3), 0);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_3), 0);

		/* Drive I2C1 SCL line low */
		mxc_request_iomux(MX51_PIN_GPIO1_2, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_GPIO1_2, PAD_CTL_DRV_HIGH |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_2), "gpio1_2");
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_GPIO1_2), 0);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_2), 0);

		msleep(5);
		mxc_free_iomux(MX51_PIN_GPIO1_2, IOMUX_CONFIG_ALT2);
		mxc_free_iomux(MX51_PIN_GPIO1_3, IOMUX_CONFIG_ALT2);
	}

	/* USB HUB RESET - De-assert USB HUB RESET_N */
	msleep(1);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_7), 0);
	msleep(1);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_7), 1);

	for (i = 0; i < ARRAY_SIZE(mxc_iomux_pins); i++) {
		mxc_request_iomux(mxc_iomux_pins[i].pin,
				  mxc_iomux_pins[i].mux_mode);
		if (mxc_iomux_pins[i].pad_cfg)
			mxc_iomux_set_pad(mxc_iomux_pins[i].pin,
					  mxc_iomux_pins[i].pad_cfg);
		if (mxc_iomux_pins[i].in_select)
			mxc_iomux_set_input(mxc_iomux_pins[i].in_select,
					    mxc_iomux_pins[i].in_mode);
	}

	gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_8), "gpio1_8");
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_0), "gpio1_0");
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_1), "gpio1_1");
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_GPIO1_8));
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_GPIO1_0));	/* SD1 CD */
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_GPIO1_1));	/* SD1 WP */
	if (board_is_rev(BOARD_REV_2)) {
		/* SD2 CD for BB2.5 */
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_6), "gpio1_6");
		gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_GPIO1_6));
	} else {
		/* SD2 CD for BB2.0 */
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_4), "gpio1_4");
		gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_GPIO1_4));
	}
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_5), "gpio1_5");
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_GPIO1_5));	/* SD2 WP */

	/* reset FEC PHY */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_A20), "eim_a20");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_A20), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A20), 0);
	msleep(10);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A20), 1);

	/* reset FM */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_A21), "eim_a21");
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A21), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_A21), 0);
	msleep(10);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A21), 1);

	if (cpu_is_mx51_rev(CHIP_REV_1_1) == 1) {
		/* MX51_PIN_EIM_CRE - De-assert USB PHY RESETB */
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_CRE), "eim_cre");
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_CRE), 0);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_CRE), 1);

		/* hphone_det_b */
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_NANDF_CS0), "nandf_cs0");
		gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_NANDF_CS0));
	} else {
		mxc_free_iomux(MX51_PIN_EIM_D21, IOMUX_CONFIG_ALT2);
		mxc_free_iomux(MX51_PIN_EIM_A24, IOMUX_CONFIG_ALT2);
		mxc_free_iomux(MX51_PIN_EIM_A25, IOMUX_CONFIG_ALT2);
		mxc_free_iomux(MX51_PIN_EIM_D18, IOMUX_CONFIG_ALT2);
		mxc_free_iomux(MX51_PIN_EIM_D20, IOMUX_CONFIG_ALT2);
		mxc_free_iomux(MX51_PIN_EIM_D21, IOMUX_CONFIG_ALT2);
		mxc_free_iomux(MX51_PIN_EIM_D16, IOMUX_CONFIG_ALT2);
		mxc_free_iomux(MX51_PIN_EIM_D17, IOMUX_CONFIG_ALT2);
		mxc_free_iomux(MX51_PIN_EIM_D19, IOMUX_CONFIG_ALT2);
		mxc_free_iomux(MX51_PIN_GPIO1_2, IOMUX_CONFIG_ALT2);
		mxc_free_iomux(MX51_PIN_GPIO1_3, IOMUX_CONFIG_ALT2);
		mxc_free_iomux(MX51_PIN_EIM_LBA, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX51_PIN_NANDF_CS0, IOMUX_CONFIG_GPIO);

		/* i2c1 SDA */
		mxc_request_iomux(MX51_PIN_EIM_D16,
				  IOMUX_CONFIG_ALT4 | IOMUX_CONFIG_SION);
		mxc_iomux_set_input(MUX_IN_I2C1_IPP_SDA_IN_SELECT_INPUT,
				    INPUT_CTL_PATH0);
		mxc_iomux_set_pad(MX51_PIN_EIM_D16, PAD_CTL_SRE_FAST |
				  PAD_CTL_ODE_OPENDRAIN_ENABLE |
				  PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
				  PAD_CTL_HYS_ENABLE);

		/* i2c1 SCL */
		mxc_request_iomux(MX51_PIN_EIM_D19,
				  IOMUX_CONFIG_ALT4 | IOMUX_CONFIG_SION);
		mxc_iomux_set_input(MUX_IN_I2C1_IPP_SCL_IN_SELECT_INPUT,
				    INPUT_CTL_PATH0);
		mxc_iomux_set_pad(MX51_PIN_EIM_D19, PAD_CTL_SRE_FAST |
				  PAD_CTL_ODE_OPENDRAIN_ENABLE |
				  PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
				  PAD_CTL_HYS_ENABLE);

		/* i2c2 SDA */
		mxc_request_iomux(MX51_PIN_KEY_COL5,
				  IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION);
		mxc_iomux_set_input(MUX_IN_I2C2_IPP_SDA_IN_SELECT_INPUT,
				    INPUT_CTL_PATH1);
		mxc_iomux_set_pad(MX51_PIN_KEY_COL5,
				  PAD_CTL_SRE_FAST |
				  PAD_CTL_ODE_OPENDRAIN_ENABLE |
				  PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
				  PAD_CTL_HYS_ENABLE);

		/* i2c2 SCL */
		mxc_request_iomux(MX51_PIN_KEY_COL4,
				  IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION);
		mxc_iomux_set_input(MUX_IN_I2C2_IPP_SCL_IN_SELECT_INPUT,
				    INPUT_CTL_PATH1);
		mxc_iomux_set_pad(MX51_PIN_KEY_COL4,
				  PAD_CTL_SRE_FAST |
				  PAD_CTL_ODE_OPENDRAIN_ENABLE |
				  PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
				  PAD_CTL_HYS_ENABLE);

		/* Drive 26M_OSC_EN line high */
		mxc_request_iomux(MX51_PIN_DI1_PIN12, IOMUX_CONFIG_ALT4);
		mxc_iomux_set_pad(MX51_PIN_DI1_PIN12, PAD_CTL_DRV_HIGH |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN12), "di1_pin12");
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN12), 0);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN12), 1);

		/* Drive USB_CLK_EN_B line low */
		mxc_request_iomux(MX51_PIN_EIM_D17, IOMUX_CONFIG_ALT1);
		mxc_iomux_set_pad(MX51_PIN_EIM_D17, PAD_CTL_DRV_HIGH |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_D17), "eim_d17");
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_D17), 0);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D17), 0);

		/* MX51_PIN_EIM_D21 - De-assert USB PHY RESETB */
		mxc_request_iomux(MX51_PIN_EIM_D21, IOMUX_CONFIG_ALT1);
		mxc_iomux_set_pad(MX51_PIN_EIM_D21, PAD_CTL_DRV_HIGH |
				  PAD_CTL_HYS_NONE | PAD_CTL_PUE_KEEPER |
				  PAD_CTL_100K_PU | PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_D21), "eim_d21");
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_D21), 0);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D21), 1);

		/* hphone_det_b */
		mxc_request_iomux(MX51_PIN_NANDF_D14, IOMUX_CONFIG_ALT3);
		mxc_iomux_set_pad(MX51_PIN_NANDF_D14, PAD_CTL_100K_PU);
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_NANDF_D14), "nandf_d14");
		gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_NANDF_D14));

		/* audio_clk_en_b */
		mxc_request_iomux(MX51_PIN_CSPI1_RDY, IOMUX_CONFIG_ALT3);
		mxc_iomux_set_pad(MX51_PIN_CSPI1_RDY, PAD_CTL_DRV_HIGH |
				  PAD_CTL_HYS_NONE | PAD_CTL_PUE_KEEPER |
				  PAD_CTL_100K_PU | PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_CSPI1_RDY), "cspi1_rdy");
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSPI1_RDY), 0);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSPI1_RDY), 0);

		/* power key */
		mxc_request_iomux(MX51_PIN_EIM_A27, IOMUX_CONFIG_ALT1);
		mxc_iomux_set_pad(MX51_PIN_EIM_A27, PAD_CTL_SRE_FAST |
				  PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
				  PAD_CTL_HYS_NONE);
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_A27), "eim_a27");
		gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_EIM_A27));
	}

	if (cpu_is_mx51_rev(CHIP_REV_3_0) > 0) {
		/* DVI_I2C_ENB = 0 tristates the DVI I2C level shifter */
		mxc_request_iomux(MX51_PIN_CSI2_HSYNC, IOMUX_CONFIG_ALT3);
		mxc_iomux_set_pad(MX51_PIN_CSI2_HSYNC, PAD_CTL_DRV_HIGH |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_CSI2_HSYNC), "csi2_hsync");
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSI2_HSYNC), 0);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI2_HSYNC), 0);
		/* TO3 doesn't need pad to drive CSI_DATA_EN[0] high */
		mxc_request_iomux(MX51_PIN_DI_GP3, IOMUX_CONFIG_ALT0);
	}

	/* Deassert VGA reset to free i2c bus */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_A19), "eim_a19");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_A19), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A19), 1);

	/* LCD related gpio */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_DI1_D1_CS), "di1_d1_cs");
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_DI1_D0_CS), "di1_d0_cs");
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_CSI2_D12), "csi2_d12");
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_CSI2_D13), "csi2_d13");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DI1_D1_CS), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DI1_D0_CS), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSI2_D12), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSI2_D13), 0);

	/* Camera reset */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_D23), "eim_d23");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_D23), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D23), 1);

	/* Camera low power */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_CSI2_D19), "csi2_d19");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSI2_D19), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI2_D19), 0);

	/* OSC_EN */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_D18), "eim_d18");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_D18), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D18), 1);

	if (enable_w1) {
		/* OneWire */
		mxc_request_iomux(MX51_PIN_OWIRE_LINE, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_OWIRE_LINE, PAD_CTL_HYS_ENABLE |
				PAD_CTL_PKE_ENABLE |
				PAD_CTL_ODE_OPENDRAIN_ENABLE |
				PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST |
				PAD_CTL_100K_PU | PAD_CTL_PUE_PULL);
	} else {
		/* SPDIF Out */
		mxc_request_iomux(MX51_PIN_OWIRE_LINE, IOMUX_CONFIG_ALT6);
		mxc_iomux_set_pad(MX51_PIN_OWIRE_LINE, PAD_CTL_DRV_HIGH |
				PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				PAD_CTL_100K_PU | PAD_CTL_SRE_FAST);
	}
}

/* workaround for ecspi chipselect pin may not keep correct level when idle */
void mx51_babbage_gpio_spi_chipselect_active(int cspi_mode, int status,
					     int chipselect)
{
	u32 gpio;

	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			mxc_request_iomux(MX51_PIN_CSPI1_SS0,
					  IOMUX_CONFIG_ALT0);
			mxc_iomux_set_pad(MX51_PIN_CSPI1_SS0,
					  PAD_CTL_HYS_ENABLE |
					  PAD_CTL_PKE_ENABLE |
					  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
			break;
		case 0x2:
			gpio = IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS0);
			mxc_request_iomux(MX51_PIN_CSPI1_SS0,
					  IOMUX_CONFIG_GPIO);
			gpio_request(gpio, "cspi1_ss0");
			gpio_direction_output(gpio, 0);
			gpio_set_value(gpio, 1 & (~status));
			break;
		default:
			break;
		}
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(mx51_babbage_gpio_spi_chipselect_active);

void mx51_babbage_gpio_spi_chipselect_inactive(int cspi_mode, int status,
					       int chipselect)
{
	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			mxc_free_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX51_PIN_CSPI1_SS0,
					  IOMUX_CONFIG_GPIO);
			mxc_free_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_GPIO);
			break;
		case 0x2:
			mxc_free_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_GPIO);
			break;
		default:
			break;
		}
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(mx51_babbage_gpio_spi_chipselect_inactive);
