/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __ASM_ARCH_MXC_BOARD_MX51_EFIKAMX_H__
#define __ASM_ARCH_MXC_BOARD_MX51_EFIKAMX_H__

/*!
 * @defgroup BRDCFG_MX51 Board Configuration Options
 * @ingroup MSL_MX51
 */

/*!
 * @file mach-mx51/board-mx51_efikamx.h
 *
 * @brief This file contains all the board level configuration options.
 *
 * It currently hold the options defined for the Genesi Efika MX Platform.
 *
 * @ingroup BRDCFG_MX51
 */

/*
 * Include Files
 */
#include <mach/mxc_uart.h>

/*!
 * @name MXC UART board level configurations
 */
/*! @{ */
/*!
 * Specifies if the Irda transmit path is inverting
 */
#define MXC_IRDA_TX_INV	0
/*!
 * Specifies if the Irda receive path is inverting
 */
#define MXC_IRDA_RX_INV	0

/* UART 1 configuration */
/*!
 * This define specifies if the UART port is configured to be in DTE or
 * DCE mode. There exists a define like this for each UART port. Valid
 * values that can be used are \b MODE_DTE or \b MODE_DCE.
 */
#define UART1_MODE		MODE_DCE
/*!
 * This define specifies if the UART is to be used for IRDA. There exists a
 * define like this for each UART port. Valid values that can be used are
 * \b IRDA or \b NO_IRDA.
 */
#define UART1_IR		NO_IRDA
/*!
 * This define is used to enable or disable a particular UART port. If
 * disabled, the UART will not be registered in the file system and the user
 * will not be able to access it. There exists a define like this for each UART
 * port. Specify a value of 1 to enable the UART and 0 to disable it.
 */
#define UART1_ENABLED		1
/*! @} */
/* UART 2 configuration */
#define UART2_MODE		MODE_DCE
#define UART2_IR		IRDA
#define UART2_ENABLED		0
/* UART 3 configuration */
#define UART3_MODE		MODE_DTE
#define UART3_IR		NO_IRDA
#define UART3_ENABLED		0

#define MXC_LL_UART_PADDR	UART1_BASE_ADDR
#define MXC_LL_UART_VADDR	AIPS1_IO_ADDRESS(UART1_BASE_ADDR)

/* ron: add LED pin definition */
#define WLAN_LED_PIN            MX51_PIN_CSI1_D9
#define POWER_LED_PIN        MX51_PIN_CSI1_VSYNC
#define SLEEP_LED_PIN           MX51_PIN_CSI1_HSYNC

extern int __init mx51_efikamx_init_mc13892(void);

#endif				/* __ASM_ARCH_MXC_BOARD_MX51_EFIKAMX_H__ */
