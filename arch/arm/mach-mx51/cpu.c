/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * 2010 Andrey Zhornyak darion76@gmail.com
 *     - added code to prevent NEON initialization on SoCs with broken implementation
 */

/*!
 * @file mach-mx51/cpu.c
 *
 * @brief This file contains the CPU initialization code.
 *
 * @ingroup MSL_MX51
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include "crm_regs.h"

/*!
 * CPU initialization. It is called by fixup_mxc_board()
 */
void __init mxc_cpu_init(void)
{
	if (!system_rev)
		mxc_set_system_rev(0x51, CHIP_REV_1_0);
}

#ifdef CONFIG_NEON

/* All versions of the silicon before Rev. 3 have broken NEON implementations.
 * Dependent on link order - so the assumption is that vfp_init is called before us
 */
static int __init mx51_neon_fixup(void)
{
       if (cpu_is_mx51_rev(CHIP_REV_3_0) < 1 ) {
               if (elf_hwcap & HWCAP_NEON) {
                       elf_hwcap &= ~HWCAP_NEON;
                       pr_info("Turning off NEON support, detected broken NEON implemention\n");
               }
       }
       return 0;
}

late_initcall(mx51_neon_fixup);
#endif


static int __init post_cpu_init(void)
{
	void __iomem *base;
	unsigned int reg;

	/* Set ALP bits to 000. Set ALP_EN bit in Arm Memory Controller reg. */
	reg = 0x8;
	__raw_writel(reg, MXC_CORTEXA8_PLAT_AMC);

	base = IO_ADDRESS(AIPS1_BASE_ADDR);
	__raw_writel(0x0, base + 0x40);
	__raw_writel(0x0, base + 0x44);
	__raw_writel(0x0, base + 0x48);
	__raw_writel(0x0, base + 0x4C);
	reg = __raw_readl(base + 0x50) & 0x00FFFFFF;
	__raw_writel(reg, base + 0x50);

	base = IO_ADDRESS(AIPS2_BASE_ADDR);
	__raw_writel(0x0, base + 0x40);
	__raw_writel(0x0, base + 0x44);
	__raw_writel(0x0, base + 0x48);
	__raw_writel(0x0, base + 0x4C);
	reg = __raw_readl(base + 0x50) & 0x00FFFFFF;
	__raw_writel(reg, base + 0x50);

	return 0;
}

postcore_initcall(post_cpu_init);
