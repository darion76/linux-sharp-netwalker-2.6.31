/**
 * mach-mx51/board-mx51_erdos.h
 *
 * This file contains all the board level configuration options.
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

#ifndef __ASM_ARCH_MXC_BOARD_MX51_ERDOS_H__
#define __ASM_ARCH_MXC_BOARD_MX51_ERDOS_H__

#include <mach/mxc_uart.h>

/**
 * MXC UART board level configurations
 */
#define MXC_IRDA_TX_INV 0
#define MXC_IRDA_RX_INV 0

/* UART 1 configuration */
#define UART1_MODE    MODE_DCE
#define UART1_IR      NO_IRDA
#define UART1_ENABLED 1

/* UART 2 configuration */
#define UART2_MODE    MODE_DCE
#define UART2_IR      IRDA
#define UART2_ENABLED 1

/* UART 3 configuration */
#define UART3_MODE    MODE_DCE
#define UART3_IR      NO_IRDA
#define UART3_ENABLED 1

#define MXC_LL_UART_PADDR UART1_BASE_ADDR
#define MXC_LL_UART_VADDR AIPS1_IO_ADDRESS(UART1_BASE_ADDR)

extern int __init mx51_erdos_init_mc13892(void);
extern void __init mx51_erdos_io_init(void);

#endif /* __ASM_ARCH_MXC_BOARD_MX51_ERDOS_H__ */
