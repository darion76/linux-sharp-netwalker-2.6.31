/*
 *  Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * 2010/11/10 ported to kernel 2.6.31 by Andrey Zhornyak darion76@gmail.com
 */
 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/cpufreq.h>
#include <asm/cacheflush.h>
#include <asm/tlb.h>
#include <asm/mach/map.h>
#include <mach/hardware.h>
#include "crm_regs.h"
#include <asm/mach-types.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>

static struct cpu_wp *cpu_wp_tbl;
static struct clk *cpu_clk;

#if defined(CONFIG_CPU_FREQ)
static int org_freq;
extern int cpufreq_suspended;
extern int set_cpu_freq(int wp);
#endif

#define PWGT1SPIEN (1<<15)
#define PWGT2SPIEN (1<<16)

static struct device *pm_dev;
struct clk *gpc_dvfs_clk;
extern void cpu_do_suspend_workaround(u32 sdclk_iomux_addr);
extern void cpu_cortexa8_do_idle(void *);
extern struct cpu_wp *(*get_cpu_wp)(int *wp);

extern int iram_ready;
void *suspend_iram_base;
void (*suspend_in_iram)(void *sdclk_iomux_addr) = NULL;

#ifdef CONFIG_MACH_MX51_ERDOS
/*
 * is_near_wakeup - check wakeup
 *  sleep -> Power-SW -> wakeup
 *   for cut Power-SW interrupt
 */
static volatile unsigned long wakeup_jiffies;
int is_near_wakeup (void)
{
	unsigned long jif = jiffies;
	unsigned long dif;

	/*
	 * check wakeup judge band time
	 */
	if (wakeup_jiffies == 0) {
		return 0;	/* no near wakeup */
	}
	if (wakeup_jiffies <= jif) {
		dif = jif - wakeup_jiffies;
	} else {
		dif = 0xffffffff - wakeup_jiffies + jif;
	}

	if (dif < (HZ * 5)) {
		return 1;	/* near wakeup(wakeup 5sec) */
	}
	wakeup_jiffies = 0;
	return 0;		/* no near wakeup */
}
#endif /* CONFIG_MACH_MX51_ERDOS */


static int mx51_suspend_enter(suspend_state_t state)
{
	void __iomem *sdclk_iomux_addr = IO_ADDRESS(IOMUXC_BASE_ADDR + 0x4b8);

	if (gpc_dvfs_clk == NULL)
		gpc_dvfs_clk = clk_get(NULL, "gpc_dvfs_clk");
	/* gpc clock is needed for SRPG */
	clk_enable(gpc_dvfs_clk);
	
	/* ron: Enable power gate */
	pmic_write_reg(REG_POWER_MISC, (PWGT1SPIEN|PWGT2SPIEN),
		(PWGT1SPIEN|PWGT2SPIEN));
	
	switch (state) {
	case PM_SUSPEND_MEM:
		mxc_cpu_lp_set(STOP_POWER_OFF);
		break;
	case PM_SUSPEND_STANDBY:
		mxc_cpu_lp_set(WAIT_UNCLOCKED_POWER_OFF);
		break;
	default:
#ifdef CONFIG_MACH_MX51_ERDOS
		clk_disable(gpc_dvfs_clk);
#endif /* CONFIG_MACH_MX51_ERDOS */
		return -EINVAL;
	}
#ifdef CONFIG_MACH_MX51_ERDOS
	if (tzic_enable_wake(0) != 0) {
		__raw_writel(0, MXC_SRPG_EMPGC0_SRPGCR);
		__raw_writel(0, MXC_SRPG_EMPGC1_SRPGCR);
		clk_disable(gpc_dvfs_clk);
		return -EAGAIN;
	}
#else
	if (tzic_enable_wake(0) != 0)
		return -EAGAIN;
#endif /* CONFIG_MACH_MX51_ERDOS */

	if (state == PM_SUSPEND_MEM) {
#ifdef CONFIG_MACH_MX51_ERDOS
		wakeup_jiffies = 0;
		local_flush_tlb_all();
		flush_cache_all();

		/* Run the suspend code from iRAM. */
		suspend_in_iram(sdclk_iomux_addr);
		wakeup_jiffies = jiffies;
		if (wakeup_jiffies == 0) {
			wakeup_jiffies = 1;
		}
#else
		local_flush_tlb_all();
		flush_cache_all();

		/* Run the suspend code from iRAM. */
		suspend_in_iram(sdclk_iomux_addr);
#endif /* CONFIG_MACH_MX51_ERDOS */
		/*clear the EMPGC0/1 bits */
		__raw_writel(0, MXC_SRPG_EMPGC0_SRPGCR);
		__raw_writel(0, MXC_SRPG_EMPGC1_SRPGCR);
	} else {
		if ((mxc_cpu_is_rev(CHIP_REV_2_0)) < 0) {
			/* do cpu_idle_workaround */
			u32 l2_iram_addr = IDLE_IRAM_BASE_ADDR;
			if (!iram_ready)
				return 0;
			if (l2_iram_addr > 0x1FFE8000)
				cpu_cortexa8_do_idle(IO_ADDRESS(l2_iram_addr));
		} else {
			cpu_do_idle();
		}
	}
	clk_disable(gpc_dvfs_clk);

	return 0;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int mx51_suspend_prepare(void)
{
#if defined(CONFIG_CPU_FREQ)
	struct cpufreq_freqs freqs;
	org_freq = clk_get_rate(cpu_clk);
	freqs.old = org_freq / 1000;
	freqs.new = cpu_wp_tbl[0].cpu_rate / 1000;
	freqs.cpu = 0;
	freqs.flags = 0;

	cpufreq_suspended = 1;
	if (clk_get_rate(cpu_clk) != cpu_wp_tbl[0].cpu_rate) {
		set_cpu_freq(cpu_wp_tbl[0].cpu_rate);
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}
#endif
	return 0;
}

/*
 * Called before devices are re-setup.
 */
static void mx51_suspend_finish(void)
{
#if defined(CONFIG_CPU_FREQ)
	struct cpufreq_freqs freqs;

	freqs.old = clk_get_rate(cpu_clk) / 1000;
	freqs.new = org_freq / 1000;
	freqs.cpu = 0;
	freqs.flags = 0;

	cpufreq_suspended = 0;

	if (org_freq != clk_get_rate(cpu_clk)) {
		set_cpu_freq(org_freq);
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}
#endif

	/* ron: disable power gate */
	pmic_write_reg(REG_POWER_MISC, ~(PWGT1SPIEN|PWGT2SPIEN),
		(PWGT1SPIEN|PWGT2SPIEN));
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void mx51_suspend_end(void)
{
}

static int mx51_pm_valid(suspend_state_t state)
{
	return (state > PM_SUSPEND_ON && state <= PM_SUSPEND_MAX);
}

struct platform_suspend_ops mx51_suspend_ops = {
	.valid = mx51_pm_valid,
	.prepare = mx51_suspend_prepare,
	.enter = mx51_suspend_enter,
	.finish = mx51_suspend_finish,
	.end = mx51_suspend_end,
};


static int __devinit mx51_pm_probe(struct platform_device *pdev)
{
	pm_dev = &pdev->dev;
	return 0;
}

static struct platform_driver mx51_pm_driver = {
	.driver = {
		   .name = "mx51_pm",
		   },
	.probe = mx51_pm_probe,
};

static int __init pm_init(void)
{
	int cpu_wp_nr;

	pr_info("Static Power Management for Freescale i.MX51\n");
	if (platform_driver_register(&mx51_pm_driver) != 0) {
		printk(KERN_ERR "mx51_pm_driver register failed\n");
		return -ENODEV;
	}
	suspend_set_ops(&mx51_suspend_ops);
	/* Move suspend routine into iRAM */
	suspend_iram_base = IO_ADDRESS(SUSPEND_IRAM_BASE_ADDR);
	memcpy(suspend_iram_base, cpu_do_suspend_workaround, SZ_4K);
	/* Need to remap the area here since we want the memory region
		 to be executable. */
	suspend_iram_base = __arm_ioremap(SUSPEND_IRAM_BASE_ADDR, SZ_4K,
										MT_HIGH_VECTORS);
	suspend_in_iram = (void *)suspend_iram_base;

	cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);

	cpu_clk = clk_get(NULL, "cpu_clk");
	if (IS_ERR(cpu_clk)) {
		printk(KERN_DEBUG "%s: failed to get cpu_clk\n", __func__);
		return PTR_ERR(cpu_clk);
	}
	printk(KERN_INFO "PM driver module loaded\n");

	return 0;
}


static void __exit pm_cleanup(void)
{
	/* Unregister the device structure */
	platform_driver_unregister(&mx51_pm_driver);
}

module_init(pm_init);
module_exit(pm_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("PM driver");
MODULE_LICENSE("GPL");
