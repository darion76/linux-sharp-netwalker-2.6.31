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
 */

/*!
 * @file dvfs_core.c
 *
 * @brief A simplied driver for the Freescale Semiconductor MXC DVFS module.
 *
 * Upon initialization, the DVFS driver initializes the DVFS hardware
 * sets up driver nodes attaches to the DVFS interrupt and initializes internal
 * data structures. When the DVFS interrupt occurs the driver checks the cause
 * of the interrupt (lower frequency, increase frequency or emergency) and
 * changes the CPU voltage according to translation table that is loaded into
 * the driver.
 *
 * @ingroup PM
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>

#define MXC_DVFSTHRS_UPTHR_MASK               0x0FC00000
#define MXC_DVFSTHRS_UPTHR_OFFSET             22
#define MXC_DVFSTHRS_DNTHR_MASK               0x003F0000
#define MXC_DVFSTHRS_DNTHR_OFFSET             16
#define MXC_DVFSTHRS_PNCTHR_MASK              0x0000003F
#define MXC_DVFSTHRS_PNCTHR_OFFSET            0

#define MXC_DVFSCOUN_DNCNT_MASK               0x00FF0000
#define MXC_DVFSCOUN_DNCNT_OFFSET             16
#define MXC_DVFSCOUN_UPCNT_MASK              0x000000FF
#define MXC_DVFSCOUN_UPCNT_OFFSET            0

#define MXC_DVFSEMAC_EMAC_MASK               0x000001FF
#define MXC_DVFSEMAC_EMAC_OFFSET             0

#define MXC_DVFSCNTR_DVFEV                   0x10000000
#define MXC_DVFSCNTR_LBMI                    0x08000000
#define MXC_DVFSCNTR_LBFL                    0x06000000
#define MXC_DVFSCNTR_DVFIS                   0x01000000
#define MXC_DVFSCNTR_FSVAIM                  0x00400000
#define MXC_DVFSCNTR_FSVAI_MASK              0x00300000
#define MXC_DVFSCNTR_FSVAI_OFFSET            20
#define MXC_DVFSCNTR_WFIM                    0x00080000
#define MXC_DVFSCNTR_WFIM_OFFSET             19
#define MXC_DVFSCNTR_MAXF_MASK               0x00040000
#define MXC_DVFSCNTR_MAXF_OFFSET             18
#define MXC_DVFSCNTR_MINF_MASK               0x00020000
#define MXC_DVFSCNTR_MINF_OFFSET             17
#define MXC_DVFSCNTR_LTBRSR_MASK             0x00000018
#define MXC_DVFSCNTR_LTBRSR_OFFSET           3
#define MXC_DVFSCNTR_DVFEN                   0x00000001

extern int dvfs_core_is_active;
extern void setup_pll(void);
static struct mxc_dvfs_platform_data *dvfs_data;
static struct device *dvfs_dev;
static struct cpu_wp *cpu_wp_tbl;
int dvfs_core_resume;
int curr_wp;
int old_wp;

extern int cpufreq_trig_needed;
struct timeval core_prev_intr;

void dump_dvfs_core_regs(void);
static struct delayed_work dvfs_core_handler;

/*
 * Clock structures
 */
static struct clk *pll1_sw_clk;
static struct clk *cpu_clk;
static struct clk *dvfs_clk;
static struct regulator *core_regulator;

extern int cpu_wp_nr;
#ifdef CONFIG_ARCH_MX51
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
#endif

enum {
	FSVAI_FREQ_NOCHANGE = 0x0,
	FSVAI_FREQ_INCREASE,
	FSVAI_FREQ_DECREASE,
	FSVAI_FREQ_EMERG,
};

/*
 * Load tracking buffer source: 1 for ld_add; 0 for pre_ld_add; 2 for after EMA
 */
#define DVFS_LTBRSR		(2 << MXC_DVFSCNTR_LTBRSR_OFFSET)

extern struct dvfs_wp dvfs_core_setpoint[2];
extern int low_bus_freq_mode;
extern int high_bus_freq_mode;
extern int set_low_bus_freq(void);
extern int set_high_bus_freq(int high_bus_speed);
extern int low_freq_bus_used(void);
extern void propagate_rate(struct clk *tclk);

DEFINE_SPINLOCK(mxc_dvfs_core_lock);

static void dvfs_load_config(int set_point)
{
	u32 reg;
	reg = 0;

	reg |= dvfs_core_setpoint[set_point].upthr << MXC_DVFSTHRS_UPTHR_OFFSET;
	reg |= dvfs_core_setpoint[set_point].downthr <<
	    MXC_DVFSTHRS_DNTHR_OFFSET;
	reg |= dvfs_core_setpoint[set_point].panicthr;
	__raw_writel(reg, dvfs_data->dvfs_thrs_reg_addr);

	reg = 0;
	reg |= dvfs_core_setpoint[set_point].downcnt <<
	    MXC_DVFSCOUN_DNCNT_OFFSET;
	reg |= dvfs_core_setpoint[set_point].upcnt << MXC_DVFSCOUN_UPCNT_OFFSET;
	__raw_writel(reg, dvfs_data->dvfs_coun_reg_addr);

	/* Set EMAC value */
	__raw_writel((dvfs_core_setpoint[set_point].emac <<
					MXC_DVFSEMAC_EMAC_OFFSET),
					dvfs_data->dvfs_emac_reg_addr);


}

static int set_cpu_freq(int wp)
{
	int arm_podf;
	int podf;
	int vinc = 0;
	int ret = 0;
	int org_cpu_rate;
	unsigned long rate = 0;
	int gp_volt = 0;
	u32 reg;
	u32 reg1;
	unsigned long flags;

	if (cpu_wp_tbl[wp].pll_rate != cpu_wp_tbl[old_wp].pll_rate) {
		org_cpu_rate = clk_get_rate(cpu_clk);
		rate = cpu_wp_tbl[wp].cpu_rate;

		if (org_cpu_rate == rate)
			return ret;

		gp_volt = cpu_wp_tbl[wp].cpu_voltage;
		if (gp_volt == 0)
			return ret;

		/*Set the voltage for the GP domain. */
		if (rate > org_cpu_rate) {
			ret = regulator_set_voltage(core_regulator, gp_volt,
						    gp_volt);
			if (ret < 0) {
				printk(KERN_DEBUG "COULD NOT SET GP VOLTAGE\n");
				return ret;
			}
			udelay(dvfs_data->delay_time);
		}
		spin_lock_irqsave(&mxc_dvfs_core_lock, flags);
		/* PLL_RELOCK, set ARM_FREQ_SHIFT_DIVIDER */
		reg = __raw_readl(dvfs_data->ccm_cdcr_reg_addr);
		reg &= 0xFFFFFFFB;
		__raw_writel(reg, dvfs_data->ccm_cdcr_reg_addr);

		setup_pll();
		/* START the GPC main control FSM */
		/* set VINC */
		reg = __raw_readl(dvfs_data->gpc_vcr_reg_addr);
		reg &= ~(MXC_GPCVCR_VINC_MASK | MXC_GPCVCR_VCNTU_MASK |
			 MXC_GPCVCR_VCNT_MASK);

		if (rate > org_cpu_rate)
			reg |= 1 << MXC_GPCVCR_VINC_OFFSET;

		reg |= (1 << MXC_GPCVCR_VCNTU_OFFSET) |
		    (1 << MXC_GPCVCR_VCNT_OFFSET);
		__raw_writel(reg, dvfs_data->gpc_vcr_reg_addr);

		reg = __raw_readl(dvfs_data->gpc_cntr_reg_addr);
		reg &= ~(MXC_GPCCNTR_ADU_MASK | MXC_GPCCNTR_FUPD_MASK);
		reg |= MXC_GPCCNTR_FUPD;
		reg |= MXC_GPCCNTR_ADU;
		__raw_writel(reg, dvfs_data->gpc_cntr_reg_addr);

		reg |= MXC_GPCCNTR_STRT;
		__raw_writel(reg, dvfs_data->gpc_cntr_reg_addr);
		while (__raw_readl(dvfs_data->gpc_cntr_reg_addr) & 0x4000)
			udelay(10);
		spin_unlock_irqrestore(&mxc_dvfs_core_lock, flags);

		if (rate < org_cpu_rate) {
			ret = regulator_set_voltage(core_regulator,
						    gp_volt, gp_volt);
			if (ret < 0) {
				printk(KERN_DEBUG
				       "COULD NOT SET GP VOLTAGE!!!!\n");
				return ret;
			}
			udelay(dvfs_data->delay_time);
		}
		clk_set_rate(cpu_clk, rate);
	} else {
		podf = cpu_wp_tbl[wp].cpu_podf;
		gp_volt = cpu_wp_tbl[wp].cpu_voltage;

		/* Change arm_podf only */
		/* set ARM_FREQ_SHIFT_DIVIDER */
		reg = __raw_readl(dvfs_data->ccm_cdcr_reg_addr);
		reg &= 0xFFFFFFFB;
		reg |= 1 << 2;
		__raw_writel(reg, dvfs_data->ccm_cdcr_reg_addr);

		/* Get ARM_PODF */
		reg = __raw_readl(dvfs_data->ccm_cacrr_reg_addr);
		arm_podf = reg & 0x07;
		if (podf == arm_podf) {
			printk(KERN_DEBUG
			       "No need to change freq and voltage!!!!\n");
			return 0;
		}

		/* Check if FSVAI indicate freq up */
		if (podf < arm_podf) {
			ret = regulator_set_voltage(core_regulator,
						    gp_volt, gp_volt);
			if (ret < 0) {
				printk(KERN_DEBUG
				       "COULD NOT SET GP VOLTAGE!!!!\n");
				return 0;
			}
			udelay(dvfs_data->delay_time);
			vinc = 1;
		} else {
			vinc = 0;
		}

		arm_podf = podf;
		/* Set ARM_PODF */
		reg &= 0xFFFFFFF8;
		reg |= arm_podf;

		reg1 = __raw_readl(dvfs_data->ccm_cdhipr_reg_addr);
		if ((reg1 & 0x00010000) == 0)
			__raw_writel(reg, dvfs_data->ccm_cacrr_reg_addr);
		else {
			printk(KERN_DEBUG "ARM_PODF still in busy!!!!\n");
			return 0;
		}

		/* START the GPC main control FSM */
		reg = __raw_readl(dvfs_data->gpc_cntr_reg_addr);
		reg |= MXC_GPCCNTR_FUPD;
		/* ADU=1, select ARM domain */
		reg |= MXC_GPCCNTR_ADU;
		__raw_writel(reg, dvfs_data->gpc_cntr_reg_addr);
		/* set VINC */
		reg = __raw_readl(dvfs_data->gpc_vcr_reg_addr);
		reg &=
		    ~(MXC_GPCVCR_VINC_MASK | MXC_GPCVCR_VCNTU_MASK |
		      MXC_GPCVCR_VCNT_MASK);
		reg |= (1 << MXC_GPCVCR_VCNTU_OFFSET) |
		    (100 << MXC_GPCVCR_VCNT_OFFSET) |
		    (vinc << MXC_GPCVCR_VINC_OFFSET);
		__raw_writel(reg, dvfs_data->gpc_vcr_reg_addr);

		reg = __raw_readl(dvfs_data->gpc_cntr_reg_addr);
		reg &= (~(MXC_GPCCNTR_ADU | MXC_GPCCNTR_FUPD));
		reg |= MXC_GPCCNTR_ADU | MXC_GPCCNTR_FUPD | MXC_GPCCNTR_STRT;
		__raw_writel(reg, dvfs_data->gpc_cntr_reg_addr);

		/* Wait for arm podf Enable */
		while ((__raw_readl(dvfs_data->gpc_cntr_reg_addr) &
			MXC_GPCCNTR_STRT) == MXC_GPCCNTR_STRT) {
			printk(KERN_DEBUG "Waiting arm_podf enabled!\n");
			udelay(10);
		}

		if (vinc == 0) {
			ret = regulator_set_voltage(core_regulator,
						    gp_volt, gp_volt);
			if (ret < 0) {
				printk(KERN_DEBUG
				       "COULD NOT SET GP VOLTAGE!!!!\n");
				return ret;
			}
			udelay(dvfs_data->delay_time);
		}

		propagate_rate(pll1_sw_clk);
	}
#if defined(CONFIG_CPU_FREQ)
		cpufreq_trig_needed = 1;
#endif
	old_wp = wp;

	return ret;
}

static int start_dvfs(void)
{
	u32 reg;
	unsigned long flags;

	printk(KERN_DEBUG "Try to start DVFS \n");
	
	if (dvfs_core_is_active)
		return 0;

	spin_lock_irqsave(&mxc_dvfs_core_lock, flags);

	clk_enable(dvfs_clk);

	dvfs_load_config(0);

	/* config reg GPC_CNTR */
	reg = __raw_readl(dvfs_data->gpc_cntr_reg_addr);

	reg &= ~MXC_GPCCNTR_GPCIRQM;
	/* GPCIRQ=1, select ARM IRQ */
	reg |= MXC_GPCCNTR_GPCIRQ_ARM;
	/* ADU=1, select ARM domain */
	reg |= MXC_GPCCNTR_ADU;
	__raw_writel(reg, dvfs_data->gpc_cntr_reg_addr);

	/* Set PREDIV bits */
	reg = __raw_readl(dvfs_data->dvfs_cntr_reg_addr);
	reg = (reg & ~(dvfs_data->prediv_mask));
	reg |= (dvfs_data->prediv_val) << (dvfs_data->prediv_offset);
	__raw_writel(reg, dvfs_data->dvfs_cntr_reg_addr);

	/* Enable DVFS interrupt */
	reg = __raw_readl(dvfs_data->dvfs_cntr_reg_addr);
	/* FSVAIM=0 */
	reg = (reg & ~MXC_DVFSCNTR_FSVAIM);
	/* Set MAXF, MINF */
	reg = (reg & ~(MXC_DVFSCNTR_MAXF_MASK | MXC_DVFSCNTR_MINF_MASK));
	reg |= 1 << MXC_DVFSCNTR_MAXF_OFFSET;
	/* Select ARM domain */
	reg |= MXC_DVFSCNTR_DVFIS;
	/* Enable DVFS frequency adjustment interrupt */
	reg = (reg & ~MXC_DVFSCNTR_FSVAIM);
	/* Set load tracking buffer register source */
	reg = (reg & ~MXC_DVFSCNTR_LTBRSR_MASK);
	reg |= DVFS_LTBRSR;
	/* Set DIV3CK */
	reg = (reg & ~(dvfs_data->div3ck_mask));
	reg |= (dvfs_data->div3ck_val) << (dvfs_data->div3ck_offset);
	__raw_writel(reg, dvfs_data->dvfs_cntr_reg_addr);

	/* Enable DVFS */
	reg = __raw_readl(dvfs_data->dvfs_cntr_reg_addr);
	reg |= MXC_DVFSCNTR_DVFEN;
	__raw_writel(reg, dvfs_data->dvfs_cntr_reg_addr);

	dvfs_core_is_active = 1;

	spin_unlock_irqrestore(&mxc_dvfs_core_lock, flags);

	printk(KERN_DEBUG "DVFS is started\n");

	return 0;
}

/*!
 * This function is called for module initialization.
 * It sets up the DVFS hardware.
 * It sets default values for DVFS thresholds and counters. The default
 * values was chosen from a set of different reasonable values. They was tested
 * and the default values in the driver gave the best results.
 * More work should be done to find optimal values.
 *
 * @return   0 if successful; non-zero otherwise.
 *
 */
static int init_dvfs_controller(void)
{
	/* DVFS loading config */
	dvfs_load_config(0);

	return 0;
}

static irqreturn_t dvfs_irq(int irq, void *dev_id)
{
	u32 reg;

	/* Check if DVFS0 (ARM) id requesting for freqency/voltage update */
	if ((__raw_readl(dvfs_data->gpc_cntr_reg_addr) & MXC_GPCCNTR_DVFS0CR) ==
	    0)
		return IRQ_NONE;

	/* Mask DVFS irq */
	reg = __raw_readl(dvfs_data->dvfs_cntr_reg_addr);
	/* FSVAIM=1 */
	reg |= MXC_DVFSCNTR_FSVAIM;
	__raw_writel(reg, dvfs_data->dvfs_cntr_reg_addr);

	/* Mask GPC1 irq */
	reg = __raw_readl(dvfs_data->gpc_cntr_reg_addr);
	reg |= MXC_GPCCNTR_GPCIRQM | 0x1000000;
	__raw_writel(reg, dvfs_data->gpc_cntr_reg_addr);

	schedule_delayed_work(&dvfs_core_handler, 0);
	return IRQ_HANDLED;
}

static void dvfs_core_work_handler(struct work_struct *work)
{
	u32 fsvai;
	u32 reg;
	u32 curr_cpu;
	int ret = 0;
	int maxf = 0, minf = 0;
	int low_freq_bus_ready = 0;
	int bus_incr = 0, cpu_dcr = 0;

	low_freq_bus_ready = low_freq_bus_used();

	/* Check DVFS frequency adjustment interrupt status */
	reg = __raw_readl(dvfs_data->dvfs_cntr_reg_addr);
	fsvai = (reg & MXC_DVFSCNTR_FSVAI_MASK) >> MXC_DVFSCNTR_FSVAI_OFFSET;
	/* Check FSVAI, FSVAI=0 is error */
	if (fsvai == FSVAI_FREQ_NOCHANGE) {
		/* Do nothing. Freq change is not required */
		goto END;
	}
	curr_cpu = clk_get_rate(cpu_clk);

	/* If FSVAI indicate freq down,
	   check arm-clk is not in lowest frequency 200 MHz */
	if (fsvai == FSVAI_FREQ_DECREASE) {
		if (curr_cpu == cpu_wp_tbl[cpu_wp_nr - 1].cpu_rate) {
			minf = 1;
			if (low_bus_freq_mode)
				goto END;
		} else {
			/* freq down */
			curr_wp++;
			if (curr_wp >= cpu_wp_nr) {
				curr_wp = cpu_wp_nr - 1;
				goto END;
			}

			if (curr_wp == cpu_wp_nr - 1 && !low_freq_bus_ready) {
				minf = 1;
				dvfs_load_config(1);
			} else {
				cpu_dcr = 1;
			}
		}
	} else {
		if (curr_cpu == cpu_wp_tbl[0].cpu_rate) {
			maxf = 1;
			goto END;
		} else {
			if (low_bus_freq_mode) {
				/* bump up LP freq first. */
				bus_incr = 1;
				dvfs_load_config(2);
			} else {
				/* freq up */
				curr_wp = 0;
				maxf = 1;
				dvfs_load_config(0);
			}
		}
	}

	low_freq_bus_ready = low_freq_bus_used();
	if ((curr_wp == cpu_wp_nr - 1) && (!low_bus_freq_mode)
	    && (low_freq_bus_ready) && !bus_incr) {
		if (cpu_dcr)
			ret = set_cpu_freq(curr_wp);
		if (!cpu_dcr) {
			set_low_bus_freq();
			dvfs_load_config(3);
		} else {
			dvfs_load_config(2);
			cpu_dcr = 0;
		}
	} else {
		if (low_bus_freq_mode)
			set_high_bus_freq(0);

		if (!bus_incr)
			ret = set_cpu_freq(curr_wp);
		bus_incr = 0;
	}


END:	/* Set MAXF, MINF */
	reg = __raw_readl(dvfs_data->dvfs_cntr_reg_addr);
	reg = (reg & ~(MXC_DVFSCNTR_MAXF_MASK | MXC_DVFSCNTR_MINF_MASK));
	reg |= maxf << MXC_DVFSCNTR_MAXF_OFFSET;
	reg |= minf << MXC_DVFSCNTR_MINF_OFFSET;

	/* Enable DVFS interrupt */
	/* FSVAIM=0 */
	reg = (reg & ~MXC_DVFSCNTR_FSVAIM);
	reg |= FSVAI_FREQ_NOCHANGE;
	/* LBFL=1 */
	reg = (reg & ~MXC_DVFSCNTR_LBFL);
	reg |= MXC_DVFSCNTR_LBFL;
	__raw_writel(reg, dvfs_data->dvfs_cntr_reg_addr);
	/*Unmask GPC1 IRQ */
	reg = __raw_readl(dvfs_data->gpc_cntr_reg_addr);
	reg &= ~MXC_GPCCNTR_GPCIRQM;
	__raw_writel(reg, dvfs_data->gpc_cntr_reg_addr);

#if defined(CONFIG_CPU_FREQ)
	if (cpufreq_trig_needed == 1) {
		cpufreq_trig_needed = 0;
		cpufreq_update_policy(0);
	}
#endif
}


/*!
 * This function disables the DVFS module.
 */
static void stop_dvfs(void)
{
	u32 reg = 0;
	unsigned long flags;
	u32 curr_cpu;

	if (dvfs_core_is_active) {

		/* Mask dvfs irq, disable DVFS */
		reg = __raw_readl(dvfs_data->dvfs_cntr_reg_addr);
		/* FSVAIM=1 */
		reg |= MXC_DVFSCNTR_FSVAIM;
		__raw_writel(reg, dvfs_data->dvfs_cntr_reg_addr);

		curr_wp = 0;
		if (!high_bus_freq_mode)
			set_high_bus_freq(1);

		curr_cpu = clk_get_rate(cpu_clk);
		if (curr_cpu != cpu_wp_tbl[curr_wp].cpu_rate) {
			set_cpu_freq(curr_wp);
#if defined(CONFIG_CPU_FREQ)
			if (cpufreq_trig_needed == 1) {
				cpufreq_trig_needed = 0;
				cpufreq_update_policy(0);
			}
#endif
		}
		spin_lock_irqsave(&mxc_dvfs_core_lock, flags);

		reg = __raw_readl(dvfs_data->dvfs_cntr_reg_addr);
		reg = (reg & ~MXC_DVFSCNTR_DVFEN);
		__raw_writel(reg, dvfs_data->dvfs_cntr_reg_addr);

		spin_unlock_irqrestore(&mxc_dvfs_core_lock, flags);

		dvfs_core_is_active = 0;

		clk_disable(dvfs_clk);
	}

	printk(KERN_DEBUG "DVFS is stopped\n");
}

void dump_dvfs_core_regs()
{
	struct timeval cur;
	u32 diff = 0;
	if (core_prev_intr.tv_sec == 0)
		do_gettimeofday(&core_prev_intr);
	else {
		do_gettimeofday(&cur);
		diff = (cur.tv_sec - core_prev_intr.tv_sec)*1000000
			 + (cur.tv_usec - core_prev_intr.tv_usec);
		core_prev_intr = cur;
	}
	if (diff < 90000)
		printk(KERN_DEBUG "diff = %d\n", diff);

	printk(KERN_INFO "THRS = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr));
	printk(KERN_INFO "COUNT = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr+0x04));
	printk(KERN_INFO "SIG1 = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr+0x08));
	printk(KERN_INFO "SIG0 = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr+0x0c));
	printk(KERN_INFO "GPC0 = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr+0x10));
	printk(KERN_INFO "GPC1 = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr+0x14));
	printk(KERN_INFO "GPBT = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr+0x18));
	printk(KERN_INFO "EMAC = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr+0x1c));
	printk(KERN_INFO "CNTR = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr+0x20));
	printk(KERN_INFO "LTR0_0 = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr+0x24));
	printk(KERN_INFO "LTR0_1 = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr+0x28));
	printk(KERN_INFO "LTR1_0 = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr+0x2c));
	printk(KERN_DEBUG "LTR1_1 = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr+0x30));
	printk(KERN_INFO "PT0 = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr+0x34));
	printk(KERN_INFO "PT1 = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr+0x38));
	printk(KERN_INFO "PT2 = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr+0x3c));
	printk(KERN_INFO "PT3 = 0x%08x\n",
			__raw_readl(dvfs_data->dvfs_thrs_reg_addr+0x40));
}

static ssize_t downthreshold_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", dvfs_core_setpoint[0].downthr);
}

static ssize_t downthreshold_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	int ret = 0;
	int val;
	ret = sscanf(buf, "%u", &val);
	dvfs_core_setpoint[0].downthr = val;

	return size;
}

static ssize_t downcount_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", dvfs_core_setpoint[0].downcnt);
}

static ssize_t downcount_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	int ret = 0;
	int val;
	ret = sscanf(buf, "%u", &val);
	dvfs_core_setpoint[0].downcnt = val;

	return size;
}


static ssize_t dvfs_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dvfs_core_is_active)
		return sprintf(buf, "DVFS is enabled\n");
	else
		return sprintf(buf, "DVFS is disabled\n");
}

static ssize_t dvfs_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	if (strstr(buf, "1") != NULL) {
		if (start_dvfs() != 0)
			printk(KERN_ERR "Failed to start DVFS\n");
	} else if (strstr(buf, "0") != NULL)
		stop_dvfs();

	return size;
}

static ssize_t dvfs_regs_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dvfs_core_is_active)
		dump_dvfs_core_regs();
	return 0;
}

static ssize_t dvfs_regs_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	if (dvfs_core_is_active)
		dump_dvfs_core_regs();
	return 0;

	return size;
}

static DEVICE_ATTR(enable, 0644, dvfs_enable_show, dvfs_enable_store);
static DEVICE_ATTR(show_regs, 0644, dvfs_regs_show, dvfs_regs_store);

static DEVICE_ATTR(down_threshold, 0644, downthreshold_show,
						downthreshold_store);
static DEVICE_ATTR(down_count, 0644, downcount_show, downcount_store);

/*!
 * This is the probe routine for the DVFS driver.
 *
 * @param   pdev   The platform device structure
 *
 * @return         The function returns 0 on success
 */
static int __devinit mxc_dvfs_core_probe(struct platform_device *pdev)
{
	int err = 0;
	struct resource *res;
	int irq;

	printk(KERN_INFO "mxc_dvfs_core_probe\n");
	dvfs_dev = &pdev->dev;
	dvfs_data = pdev->dev.platform_data;

	INIT_DELAYED_WORK(&dvfs_core_handler, dvfs_core_work_handler);

	pll1_sw_clk = clk_get(NULL, "pll1_sw_clk");
	if (IS_ERR(pll1_sw_clk)) {
		printk(KERN_INFO "%s: failed to get pll1_sw_clk\n", __func__);
		return PTR_ERR(pll1_sw_clk);
	}

	cpu_clk = clk_get(NULL, dvfs_data->clk1_id);
	if (IS_ERR(cpu_clk)) {
		printk(KERN_ERR "%s: failed to get cpu clock\n", __func__);
		return PTR_ERR(cpu_clk);
	}

	dvfs_clk = clk_get(NULL, dvfs_data->clk2_id);
	if (IS_ERR(dvfs_clk)) {
		printk(KERN_ERR "%s: failed to get dvfs clock\n", __func__);
		return PTR_ERR(dvfs_clk);
	}

	core_regulator = regulator_get(NULL, dvfs_data->reg_id);
	if (IS_ERR(core_regulator)) {
		clk_put(cpu_clk);
		clk_put(dvfs_clk);
		printk(KERN_ERR "%s: failed to get gp regulator\n", __func__);
		return PTR_ERR(core_regulator);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		err = -ENODEV;
		goto err1;
	}

	/*
	 * Request the DVFS interrupt
	 */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		err = irq;
		goto err1;
	}

	/* request the DVFS interrupt */
	err = request_irq(irq, dvfs_irq, IRQF_SHARED, "dvfs", dvfs_dev);
	if (err)
		printk(KERN_ERR
		       "DVFS: Unable to attach to DVFS interrupt,err = %d",
		       err);

	clk_enable(dvfs_clk);
	err = init_dvfs_controller();
	if (err) {
		printk(KERN_ERR "DVFS: Unable to initialize DVFS");
		return err;
	}
	clk_disable(dvfs_clk);

	err = sysfs_create_file(&dvfs_dev->kobj, &dev_attr_enable.attr);
	if (err) {
		printk(KERN_ERR
		       "DVFS: Unable to register sysdev entry for DVFS");
		return err;
	}

	err = sysfs_create_file(&dvfs_dev->kobj, &dev_attr_show_regs.attr);
	if (err) {
		printk(KERN_ERR
		       "DVFS: Unable to register sysdev entry for DVFS");
		return err;
	}


	err = sysfs_create_file(&dvfs_dev->kobj, &dev_attr_down_threshold.attr);
	if (err) {
		printk(KERN_ERR
		       "DVFS: Unable to register sysdev entry for DVFS");
		return err;
	}

	err = sysfs_create_file(&dvfs_dev->kobj, &dev_attr_down_count.attr);
	if (err) {
		printk(KERN_ERR
		       "DVFS: Unable to register sysdev entry for DVFS");
		return err;
	}

	/* Set the current working point. */
	cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);
	old_wp = 0;
	curr_wp = 0;
	dvfs_core_resume = 0;
	cpufreq_trig_needed = 0;

	return err;

err1:
	dev_err(&pdev->dev, "Failed to probe DVFS CORE\n");
	return err;
}

/*!
 * This function is called to put DVFS in a low power state.
 *
 * @param   pdev  the device structure
 * @param   state the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int mxc_dvfs_core_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	if (dvfs_core_is_active) {
		dvfs_core_resume = 1;
		stop_dvfs();
	}

	return 0;
}

/*!
 * This function is called to resume the MU from a low power state.
 *
 * @param   dev   the device structure
 * @param   level the stage in device suspension process that we want the
 *                device to be put in
 *
 * @return  The function always returns 0.
 */
static int mxc_dvfs_core_resume(struct platform_device *pdev)
{
	if (dvfs_core_resume) {
		dvfs_core_resume = 0;
		start_dvfs();
	}

	return 0;
}

static struct platform_driver mxc_dvfs_core_driver = {
	.driver = {
		   .name = "mxc_dvfs_core",
		   },
	.probe = mxc_dvfs_core_probe,
	.suspend = mxc_dvfs_core_suspend,
	.resume = mxc_dvfs_core_resume,
};

static int __init dvfs_init(void)
{
	if (platform_driver_register(&mxc_dvfs_core_driver) != 0) {
		printk(KERN_ERR "mxc_dvfs_core_driver register failed\n");
		return -ENODEV;
	}

	dvfs_core_is_active = 0;
	printk(KERN_INFO "DVFS driver module loaded\n");
	return 0;
}

static void __exit dvfs_cleanup(void)
{
	stop_dvfs();

	/* release the DVFS interrupt */
	free_irq(MXC_INT_GPC1, NULL);

	sysfs_remove_file(&dvfs_dev->kobj, &dev_attr_enable.attr);

	/* Unregister the device structure */
	platform_driver_unregister(&mxc_dvfs_core_driver);

	clk_put(cpu_clk);
	clk_put(dvfs_clk);

	dvfs_core_is_active = 0;
	printk(KERN_INFO "DVFS driver module unloaded\n");

}

module_init(dvfs_init);
module_exit(dvfs_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("DVFS driver");
MODULE_LICENSE("GPL");
