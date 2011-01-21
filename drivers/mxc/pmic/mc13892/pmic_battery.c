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
 *
 * modification information
 * ------------------------
 * 2009/07/27 : for custom board(CONFIG_MACH_MX51_ERDOS).
 *              POWER_SUPPLY_PROP_VOLTAGE_NOW=Batt-Volt
 *              POWER_SUPPLY_PROP_CAPACITY=Batt-Life
 * 2009/08/06 : pmic_get_voltage() raw-value add.
 *              CAPACITY level fixed.
 *              /proc/battery support.
 *              BATT scale change.
 * 2009/08/07 : VOLT_MEASURE_INTERVAL 20*HZ -> 2*HZ
 * 2009/08/11 : BATT volt average of some samples.
 * 2009/08/12 : suspend/resume support LOBATLI/HI interrupt.
 *              online/offline judge check error.
 *              BATT volt 10msx5 sample, average, 60sec/5sec input.
 *              ADIN2 support.
 *              ALWAY_REFON mode.
 * 2009/08/21 : suspend/resume change.
 *               delete LOBATLI/HI interrupt.
 *               support TODAI.
 *              ALWAY_REFON delete.
 *              mc13892_battery_sample() arg "wait force" add.
 * 2009/08/24 : DC volt scale adjust.
 * 2009/08/25 : BATT_CAP_SHUTDOWN_LEVEL change.
 * 2009/08/26 : samp_raw, samp_volt add.
 * 2009/08/27 : every cycle update gpio_battery_enable().
 * 2009/08/27 : samp_raw, samp_volt del.
 *              /proc/battery called pmic_get_voltage()
 *              batt_sample_rate add.
 * 2009/08/31 : battery_low_resume() called battery_low_suspend_cancel().
 *               if don't call battery_low_resume_check(), then don't restore pmic_event.
 * 2009/09/01 : change shutdown judge level(800 -> 780)
 * 2009/09/04 : change batt_cap[] value.
 *              change batt_poll_interval (60*60) -> (30*60)
 * 2009/09/09 : Power-SW flag clean at suspend. 
 *              battery_poll_statistics [6] (check count) add.
 * 2009/09/19 : check not yet power_supply_register on mc13892_charger_update_status()
 *              at mc13892_battery_get_property().
 * 2010/11/09 : ported to kernel 2.6.31 by Andrey Zhornyak darion76@gmail.com
 */

/*
 * Includes
 */
#include <linux/platform_device.h>
#include <linux/power_supply.h>

#include <linux/delay.h>
#include <linux/pmic_battery.h>
#include <linux/pmic_adc.h>
#include <linux/pmic_status.h>
#ifdef CONFIG_MACH_MX51_ERDOS
#include <linux/input.h>
#endif /* CONFIG_MACH_MX51_ERDOS */

#define BIT_CHG_VOL_LSH		0
#define BIT_CHG_VOL_WID		3

#define BIT_CHG_CURR_LSH		3
#define BIT_CHG_CURR_WID		4

#define BIT_CHG_PLIM_LSH		15
#define BIT_CHG_PLIM_WID		2

#define BIT_CHG_DETS_LSH 6
#define BIT_CHG_DETS_WID 1
#define BIT_CHG_CURRS_LSH 11
#define BIT_CHG_CURRS_WID 1

#define TRICKLE_CHG_EN_LSH	7
#define	LOW_POWER_BOOT_ACK_LSH	8
#define BAT_TH_CHECK_DIS_LSH	9
#define	BATTFET_CTL_EN_LSH	10
#define BATTFET_CTL_LSH		11
#define	REV_MOD_EN_LSH		13
#define PLIM_DIS_LSH		17
#define	CHG_LED_EN_LSH		18
#define RESTART_CHG_STAT_LSH	20
#define	AUTO_CHG_DIS_LSH	21
#define CYCLING_DIS_LSH		22
#define	VI_PROGRAM_EN_LSH	23

#define TRICKLE_CHG_EN_WID	1
#define	LOW_POWER_BOOT_ACK_WID	1
#define BAT_TH_CHECK_DIS_WID	1
#define	BATTFET_CTL_EN_WID	1
#define BATTFET_CTL_WID		1
#define	REV_MOD_EN_WID		1
#define PLIM_DIS_WID		1
#define	CHG_LED_EN_WID		1
#define RESTART_CHG_STAT_WID	1
#define	AUTO_CHG_DIS_WID	1
#define CYCLING_DIS_WID		1
#define	VI_PROGRAM_EN_WID	1

#define ACC_STARTCC_LSH		0
#define ACC_STARTCC_WID		1
#define ACC_RSTCC_LSH		1
#define ACC_RSTCC_WID		1
#define ACC_CCFAULT_LSH		7
#define ACC_CCFAULT_WID		7
#define ACC_CCOUT_LSH		8
#define ACC_CCOUT_WID		16
#define ACC1_ONEC_LSH		0
#define ACC1_ONEC_WID		15

#define ACC_CALIBRATION 0x17
#define ACC_START_COUNTER 0x07
#define ACC_STOP_COUNTER 0x2
#define ACC_CONTROL_BIT_MASK 0x1f
#define ACC_ONEC_VALUE 2621
#define ACC_COULOMB_PER_LSB 1
#define ACC_CALIBRATION_DURATION_MSECS 20

#define BAT_VOLTAGE_UNIT_UV 4692
#define BAT_CURRENT_UNIT_UA 5870
#define CHG_VOLTAGE_UINT_UV 23474
#define CHG_MIN_CURRENT_UA 3500

#define COULOMB_TO_UAH(c) (10000 * c / 36)


#ifdef CONFIG_MACH_MX51_ERDOS
//#define PMIC_BATT_DEBUG
//#define PMIC_BATT_DEBUG_DETAIL

#undef  USE_BATTERY_LOW_INTERRUPT	// LOBATLI/HI
#define USE_BATTERY_LOW_POLLING		// TODAI

#ifdef USE_BATTERY_LOW_POLLING
/*
 * BATTERY POLLING interval (sec)
 */
static int batt_poll_interval = (30 * 60);
#endif /* USE_BATTERY_LOW_POLLING */

#ifdef USE_BATTERY_LOW_INTERRUPT
/*
 * battery low event key
 */
#define BATTERY_INPUT_NAME  ("battery_stat")
struct _batt_input_stat {
    int  event_type;		/* EV_KEY/... */
    int  event_code;		/* KEY_xxx/... */
}; 
static struct _batt_input_stat batt_input_stat [2] = {
    { EV_KEY, KEY_POWER },	/* LOBATLI */
    { EV_KEY, KEY_BATTERY },	/* LOBATHI */
};
#endif /* USE_BATTERY_LOW_INTERRUPT */

/*
 * BATT capacity level
 */
#define BATT_CAP_CAUTION_LEVEL    2
#define BATT_CAP_SHUTDOWN_LEVEL   4
struct _batt_cap {
    int  raw;		/* BATT raw value */
    int  cap;		/* BATT capacity level (%) */
};
static struct _batt_cap batt_cap [] = {
  { 947, 90 },		/* [0] */
  { 892, 50 },		/* [1] */
  { 873, 20 },		/* [2] caution judge level */
  { 771,  8 },		/* [3] */
  { 770,  2 },		/* [4] shutdown judge level */
  {   0,  2 } };	/* [5] */

#ifdef PMIC_BATT_DEBUG
#define PMIC_BATT_PRINTK	printk
#else
#define PMIC_BATT_PRINTK(fmt, arg...)
#endif

#define DCIN_GOOD_VOLT			(8500)		// DCinput Good Voltage level
/*
 * Battery 8.3V - 991
 */
#define BATT_RAW_TO_VOLT(raw)	((raw) * 8300 / 991)

/*
 * Battery/DCinput voltage input data
 *  [0] : BATTERY(ADIN5)
 *  [1] : DCinput(ADIN7)
 *  [2] : BPSNS (ADIN2)
 */
#define VOLT_MEASURE_INTERVAL (2 * HZ)
struct _volt_input_value {
    unsigned long jif;		/* jiffies at measure */
    int           volt;		/* voltage(mV) */
    int           raw;		/* A/D raw value */
};
static struct _volt_input_value volt_input_value [3];

/* BQ24105 maximum voltage (about 3.517[V]) */
#define BQ24105_BATTERY_VOLTAGE_MAX	(int)((47 * 11000000) / 147)

/* BQ24105 minimum voltage */
#define BQ24105_BATTERY_VOLTAGE_MIN	(int)( 0 )

#undef  ALWAY_REFON		/* alway REFON on */

/* GPIO functions prototype */
extern int  gpio_charge_status(void);
extern void gpio_refon(int val);
extern void gpio_battery_enable (int enable);
#endif /* CONFIG_MACH_MX51_ERDOS */


enum chg_setting {
       TRICKLE_CHG_EN,
       LOW_POWER_BOOT_ACK,
       BAT_TH_CHECK_DIS,
       BATTFET_CTL_EN,
       BATTFET_CTL,
       REV_MOD_EN,
       PLIM_DIS,
       CHG_LED_EN,
       RESTART_CHG_STAT,
       AUTO_CHG_DIS,
       CYCLING_DIS,
       VI_PROGRAM_EN
};

#ifndef CONFIG_MACH_MX51_ERDOS
static int pmic_set_chg_current(unsigned short curr)
{
	unsigned int mask;
	unsigned int value;

	value = BITFVAL(BIT_CHG_CURR, curr);
	mask = BITFMASK(BIT_CHG_CURR);
	CHECK_ERROR(pmic_write_reg(REG_CHARGE, value, mask));

	return 0;
}

static int pmic_set_chg_misc(enum chg_setting type, unsigned short flag)
{

	unsigned int reg_value = 0;
	unsigned int mask = 0;

	switch (type) {
	case TRICKLE_CHG_EN:
		reg_value = BITFVAL(TRICKLE_CHG_EN, flag);
		mask = BITFMASK(TRICKLE_CHG_EN);
		break;
	case LOW_POWER_BOOT_ACK:
		reg_value = BITFVAL(LOW_POWER_BOOT_ACK, flag);
		mask = BITFMASK(LOW_POWER_BOOT_ACK);
		break;
	case BAT_TH_CHECK_DIS:
		reg_value = BITFVAL(BAT_TH_CHECK_DIS, flag);
		mask = BITFMASK(BAT_TH_CHECK_DIS);
		break;
	case BATTFET_CTL_EN:
		reg_value = BITFVAL(BATTFET_CTL_EN, flag);
		mask = BITFMASK(BATTFET_CTL_EN);
		break;
	case BATTFET_CTL:
		reg_value = BITFVAL(BATTFET_CTL, flag);
		mask = BITFMASK(BATTFET_CTL);
		break;
	case REV_MOD_EN:
		reg_value = BITFVAL(REV_MOD_EN, flag);
		mask = BITFMASK(REV_MOD_EN);
		break;
	case PLIM_DIS:
		reg_value = BITFVAL(PLIM_DIS, flag);
		mask = BITFMASK(PLIM_DIS);
		break;
	case CHG_LED_EN:
		reg_value = BITFVAL(CHG_LED_EN, flag);
		mask = BITFMASK(CHG_LED_EN);
		break;
	case RESTART_CHG_STAT:
		reg_value = BITFVAL(RESTART_CHG_STAT, flag);
		mask = BITFMASK(RESTART_CHG_STAT);
		break;
	case AUTO_CHG_DIS:
		reg_value = BITFVAL(AUTO_CHG_DIS, flag);
		mask = BITFMASK(AUTO_CHG_DIS);
		break;
	case CYCLING_DIS:
		reg_value = BITFVAL(CYCLING_DIS, flag);
		mask = BITFMASK(CYCLING_DIS);
		break;
	case VI_PROGRAM_EN:
		reg_value = BITFVAL(VI_PROGRAM_EN, flag);
		mask = BITFMASK(VI_PROGRAM_EN);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(REG_CHARGE, reg_value, mask));

	return 0;
}
#endif /* CONFIG_MACH_MX51_ERDOS */

#ifdef CONFIG_MACH_MX51_ERDOS
#define GEN_PURPOSE_AD2 APPLICATION_SUPPLY	/* AD2 */
#if 1 /* add 090731 */
extern int g_apm_spi_disable;
#endif
/*
 * get Battery/DCinput voltage from PMIC
 *  voltage [mV]
 */
static int pmic_get_voltage(t_channel channel, unsigned short *voltage, int *rawval,
			    int force)
{
	unsigned short            result[8];
	struct _volt_input_value *info;
	unsigned long             dif, jif = jiffies;
	int                       rc, volt, isBat = 0;

	if ((channel == BATTERY_VOLTAGE) || (channel == GEN_PURPOSE_AD5)) {
		/*
		 * Battery Voltage
		 */
		info    = &(volt_input_value [0]);
		channel = GEN_PURPOSE_AD5;
		isBat   = 1;
	} else if (channel == GEN_PURPOSE_AD7) {
		/*
		 * DCinput Voltage
		 */
		info = &(volt_input_value [1]);
	} else if (channel == GEN_PURPOSE_AD2) {
		/*
		 * BPSNS Voltage
		 */
		info  = &(volt_input_value [2]);
		isBat = 2;
	} else {
		return -1;
	}

#if 1 /* add 090731 */
	if (g_apm_spi_disable) {
		if (channel == BATTERY_VOLTAGE || channel == GEN_PURPOSE_AD5) {
			if (voltage != 0) {
				*voltage = 8300;
			}
			if (rawval != 0) {
				*rawval = 991;
			}
			return 0;
		} else if (channel == GEN_PURPOSE_AD7) {
			if (voltage != 0) {
				*voltage = 11000;
			}
			if (rawval != 0) {
				*rawval = 1020;
			}
			return 0;
		} else if (channel == GEN_PURPOSE_AD2) {
			if (voltage != 0) {
				*voltage = 3450;
			}
			if (rawval != 0) {
				*rawval = 735;
			}
			return 0;
		} else {
			return -1;
		}
	}
#endif

	/*
	 * check measure timing
	 */
	if (force == 0) {
	if (info->jif <= jif) {
		dif = jif - info->jif;
	} else {
		dif = 0xffffffff - info->jif + jif;
	}
	if (dif < VOLT_MEASURE_INTERVAL) {
		/*
		 * using already measure value
		 */
		if (voltage != 0) {
			*voltage = info->volt;
		}
		if (rawval != 0) {
			*rawval = info->raw;
		}
		return 0;
	}
	}

#ifdef ALWAY_REFON
	rc = pmic_adc_convert(channel, result);
#else
	gpio_refon(1);		// select measurement
	rc = pmic_adc_convert(channel, result);
	gpio_refon(0);		// release measurement
#endif /* ALWAY_REFON */
	if (rc == 0) {
	/*
	 * scale convert(mV)
	 */
	if (isBat == 1) {
		/*
		 * Battery
		 */
			volt = (int)BATT_RAW_TO_VOLT(result[0]);
		} else if (isBat == 2) {
			/*
			 * BPSNS
			 *  4.80V - 1023
			 */
			volt = (int)(result[0] * 4800 / 1023);
	} else {
		/*
		 * DCinput
		 */
		volt = (int)(result[0] * 20398) / 0x3FF;
	}
	info->volt = volt;
	info->raw  = result[0];
		info->jif  = jif;

	if (voltage != 0) {
		*voltage = volt;
	}
	if (rawval != 0) {
		*rawval = result[0];
	}
	}

	return rc;
}

/*
 * mc13892_battery_sample (void)
 *  battery volatge sample for average
 *   return  N - sample count
 *   return -1 - error
 */
struct _batt_sample {
    int     mode_caution;		/* mode mormal(0)/caution(1) */
    int     interval;			/* sample interval */
    int     raw;			/* sample raw A/D */
    int     volt;			/* sample volt */
    int     prev_use;			/* previous sample use count */
    int     error;			/* sample error */
    int     update;			/* sample update */
};
static struct _batt_sample batt_sample;
static int                 batt_sample_initialized;
static int                 batt_sample_rate = 60;	/* sample rate (sec) */
static int mc13892_battery_sample (int online, int wait, int force)
{
	int   i, raw, rc;
	int   rdata [5], rcnt, rmin, rmax, rttl, fmin, fmax;

	/*
	 * check sample interval
	 *  mormal - 60sec, caution - 5sec
	 */
	rc = 0;
	if (batt_sample_initialized != 0) {
		if (batt_sample.mode_caution == 0) {
			batt_sample.interval++;
			if ((force == 0) &&
			    (batt_sample.interval < (batt_sample_rate / 5))) {
				/*
				 * not sample timing
				 */
				return rc;
			}
			batt_sample.interval = 0;
		}
	}

	/*
	 * BATT voltage
         *  lowest  (1)   (2)   (3)  highest
         *    *      +- average -+      *    (* ignore)
	 */
	rmin = 0x7fffffff;
	rmax = 0;
	rcnt = 0;
	for ( i = 0 ; i < 5 ; i++ ) {
		if (pmic_get_voltage (GEN_PURPOSE_AD5, 0, &raw, 1) == 0) {
			rdata [rcnt++] = raw;
			if (raw < rmin) {
				rmin = raw;
			} else if (rmax < raw) {
				rmax = raw;
			}
		} else {
			batt_sample.error++;
		}
		if (wait != 0) {
			mdelay ( 10 );	/* 10ms measure interval */
		}
	}
	rttl = 0;
	fmin = 0;
	fmax = 0;
	for ( i = 0 ; i < rcnt ; i++ ) {
		if ((fmin == 0) && (rmin == rdata [i])) {
			fmin++;		/* lowest value ignore */
		} else if ((fmax == 0) && (rmax == rdata [i])) {
			fmax++;		/* highest value ignore */
		} else {
			rttl += rdata [i];
		}
	}
	if (0 < (rcnt - fmin - fmax)) {
		raw = rttl / (rcnt - fmin - fmax);
		batt_sample.update++;
		rc = rcnt;
	} else {
		raw = batt_sample.raw;
		batt_sample.prev_use++;
		rc = -1;
	}

	/*
	 * check BATT use(AC adapter disconnect)
	 *  voltage of BATT use is decrease only.
	 */
	if ((online == 0) && (batt_sample_initialized != 0) && (batt_sample.raw < raw)) {
		raw = batt_sample.raw;	/* prev sample use */
	}

	/*
	 * check normal/caution band
	 */
	if (raw <= batt_cap [BATT_CAP_CAUTION_LEVEL].raw) {
		batt_sample.mode_caution = 1;
	} else {
		batt_sample.mode_caution = 0;
	}

	batt_sample.raw = raw;
	/*
	 * Convert voltage
	 */
	batt_sample.volt = BATT_RAW_TO_VOLT(raw);

	batt_sample_initialized = 1;

	return rc;
}

/*
 * pmic_get_battery_voltage (unsigned short *voltage, int *raw)
 *  sampled battery voltage input
 */
static int pmic_get_battery_voltage (unsigned short *voltage, int *raw)
{
	if (voltage != 0) {
		*voltage = batt_sample.volt;
	}
	if (raw != 0) {
		*raw = batt_sample.raw;
	}
	return 0;
}

/*
 * pmic_get_dcinput_voltage (unsigned short *voltage)
 *  rc 0(DCinput OK) / 1(DCinput Low) / -1(error)
 */
int pmic_get_dcinput_voltage (unsigned short *voltage)
{
	unsigned short vv;
	int rc = pmic_get_voltage (GEN_PURPOSE_AD7, &vv, 0, 0);

	if (voltage != 0) {
		*voltage = vv;
	}
	/*
	 * judge DCinput good/low
	 */
	if (rc == 0) {
		if (DCIN_GOOD_VOLT <= vv) {
			rc = 0;		// Good
		} else {
			rc = 1;		// Low
		}
	}
	return rc;
}
EXPORT_SYMBOL(pmic_get_dcinput_voltage);
#endif /* CONFIG_MACH_MX51_ERDOS */

#ifdef CONFIG_MACH_MX51_ERDOS
static int pmic_get_batt_voltage(unsigned short *voltage, int *raw)
{
	return ( pmic_get_battery_voltage (voltage, raw) );
}
#else
static int pmic_get_batt_voltage(unsigned short *voltage)
{
	t_channel channel;
	unsigned short result[8];

	channel = BATTERY_VOLTAGE;
	CHECK_ERROR(pmic_adc_convert(channel, result));
	*voltage = result[0];

	return 0;
}
#endif /* CONFIG_MACH_MX51_ERDOS */

#ifndef CONFIG_MACH_MX51_ERDOS
static int pmic_get_batt_current(unsigned short *curr)
{
	t_channel channel;
	unsigned short result[8];

	channel = BATTERY_CURRENT;
	CHECK_ERROR(pmic_adc_convert(channel, result));
	*curr = result[0];

	return 0;
}

static int coulomb_counter_calibration;
static unsigned int coulomb_counter_start_time_msecs;

static int pmic_start_coulomb_counter(void)
{
	/* set scaler */
	CHECK_ERROR(pmic_write_reg(REG_ACC1,
		ACC_COULOMB_PER_LSB * ACC_ONEC_VALUE, BITFMASK(ACC1_ONEC)));

	CHECK_ERROR(pmic_write_reg(
		REG_ACC0, ACC_START_COUNTER, ACC_CONTROL_BIT_MASK));
	coulomb_counter_start_time_msecs = jiffies_to_msecs(jiffies);
	pr_debug("coulomb counter start time %u\n",
		coulomb_counter_start_time_msecs);
	return 0;
}

static int pmic_stop_coulomb_counter(void)
{
	CHECK_ERROR(pmic_write_reg(
		REG_ACC0, ACC_STOP_COUNTER, ACC_CONTROL_BIT_MASK));
	return 0;
}

static int pmic_calibrate_coulomb_counter(void)
{
	int ret;
	unsigned int value;

	/* set scaler */
	CHECK_ERROR(pmic_write_reg(REG_ACC1,
		0x1, BITFMASK(ACC1_ONEC)));

	CHECK_ERROR(pmic_write_reg(
		REG_ACC0, ACC_CALIBRATION, ACC_CONTROL_BIT_MASK));
	msleep(ACC_CALIBRATION_DURATION_MSECS);

	ret = pmic_read_reg(REG_ACC0, &value, BITFMASK(ACC_CCOUT));
	if (ret != 0)
		return -1;
	value = BITFEXT(value, ACC_CCOUT);
	pr_debug("calibrate value = %x\n", value);
	coulomb_counter_calibration = (int)((s16)((u16) value));
	pr_debug("coulomb_counter_calibration = %d\n",
		coulomb_counter_calibration);

	return 0;

}

static int pmic_get_charger_coulomb(int *coulomb)
{
	int ret;
	unsigned int value;
	int calibration;
	unsigned int time_diff_msec;

	ret = pmic_read_reg(REG_ACC0, &value, BITFMASK(ACC_CCOUT));
	if (ret != 0)
		return -1;
	value = BITFEXT(value, ACC_CCOUT);
	pr_debug("counter value = %x\n", value);
	*coulomb = ((s16)((u16)value)) * ACC_COULOMB_PER_LSB;

	if (abs(*coulomb) >= ACC_COULOMB_PER_LSB) {
			/* calibrate */
		time_diff_msec = jiffies_to_msecs(jiffies);
		time_diff_msec =
			(time_diff_msec > coulomb_counter_start_time_msecs) ?
			(time_diff_msec - coulomb_counter_start_time_msecs) :
			(0xffffffff - coulomb_counter_start_time_msecs
			+ time_diff_msec);
		calibration = coulomb_counter_calibration * (int)time_diff_msec
			/ (ACC_ONEC_VALUE * ACC_CALIBRATION_DURATION_MSECS);
		*coulomb -= calibration;
	}

	return 0;
}

static int pmic_restart_charging(void)
{
	pmic_set_chg_misc(BAT_TH_CHECK_DIS, 1);
	pmic_set_chg_misc(AUTO_CHG_DIS, 0);
	pmic_set_chg_misc(VI_PROGRAM_EN, 1);
	pmic_set_chg_current(0x8);
	pmic_set_chg_misc(RESTART_CHG_STAT, 1);
	return 0;
}
#endif /* CONFIG_MACH_MX51_ERDOS */

struct mc13892_dev_info {
	struct device *dev;

	unsigned short voltage_raw;
	int voltage_uV;
#ifndef CONFIG_MACH_MX51_ERDOS
	unsigned short current_raw;
	int current_uA;
#endif /* CONFIG_MACH_MX51_ERDOS */
	int battery_status;
	int full_counter;
	int charger_online;
	int charger_voltage_uV;
#ifndef CONFIG_MACH_MX51_ERDOS
	int accum_current_uAh;
#endif /* CONFIG_MACH_MX51_ERDOS */

	struct power_supply bat;
	struct power_supply charger;

	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
};

#define mc13892_SENSER	25
#define to_mc13892_dev_info(x) container_of((x), struct mc13892_dev_info, \
					      bat);

static enum power_supply_property mc13892_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
#ifndef CONFIG_MACH_MX51_ERDOS
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
#endif /* CONFIG_MACH_MX51_ERDOS */
	POWER_SUPPLY_PROP_STATUS,
};

static enum power_supply_property mc13892_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
#ifdef CONFIG_MACH_MX51_ERDOS
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_STATUS,
#endif /* CONFIG_MACH_MX51_ERDOS */
};

static int mc13892_charger_update_status(struct mc13892_dev_info *di)
{
	int ret;
#ifndef CONFIG_MACH_MX51_ERDOS
	unsigned int value;
#endif /* CONFIG_MACH_MX51_ERDOS */
	int online;

#ifdef CONFIG_MACH_MX51_ERDOS
	ret = pmic_get_dcinput_voltage ((unsigned short *)0);
	if (ret == 0) {
		online = 1;
	} else if (ret == 1) {
		online = 0;
	} else {
		online = di->charger_online;	/* keep previous */
	}
	ret = 0;
		/*
	 * Battery/DCinput update
		 */
		if (online == 1) {
			gpio_battery_enable ( 0 );
	} else if (online == 0) {
			gpio_battery_enable ( 1 );
		}
	if (online != di->charger_online) {
		di->charger_online = online;
		/*
		 * check power_supply_register.
		 */
		if (di->charger.dev != 0) {
		dev_info(di->charger.dev, "charger status: %s\n",
			online ? "online" : "offline");
		power_supply_changed(&di->charger);
		} else {
			printk ("mc13892_charger_update_status: charger status: %s\n",
				online ? "online" : "offline");
		}
	}
#else
	ret = pmic_read_reg(REG_INT_SENSE0, &value, BITFMASK(BIT_CHG_DETS));

	if (ret == 0) {
		online = BITFEXT(value, BIT_CHG_DETS);
		if (online != di->charger_online) {
			di->charger_online = online;
			dev_info(di->charger.dev, "charger status: %s\n",
				online ? "online" : "offline");
			power_supply_changed(&di->charger);

			cancel_delayed_work(&di->monitor_work);
			queue_delayed_work(di->monitor_wqueue,
				&di->monitor_work, HZ / 10);
			if (online) {
				pmic_start_coulomb_counter();
				pmic_restart_charging();
			} else
				pmic_stop_coulomb_counter();
		}
	}
#endif /* CONFIG_MACH_MX51_ERDOS */

	return ret;
}

static int mc13892_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct mc13892_dev_info *di =
		container_of((psy), struct mc13892_dev_info, charger);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->charger_online;
		return 0;
#ifdef CONFIG_MACH_MX51_ERDOS
	case POWER_SUPPLY_PROP_STATUS:
		if ( di->charger_online == 0 ) {	/* AC off */
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		} else {				/* AC on */
			int stat = gpio_charge_status();
			switch (stat) {
			case 0x1: /* STAT2: charged */
				val->intval = POWER_SUPPLY_STATUS_FULL;
				break;
			case 0x2: /* STAT1: now charging */
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
				break;
			default:
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
				break;
			}
		}
 		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = BQ24105_BATTERY_VOLTAGE_MAX;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = BQ24105_BATTERY_VOLTAGE_MIN;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		{
			unsigned short voltage = 0;
			pmic_get_batt_voltage (&voltage, 0);
			val->intval = voltage;		/* mV */
			return 0;
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		{
			/*
			 * BATT life temp
			 */
			unsigned short    voltage = 0;
			int               raw = batt_cap [0].raw;
			struct _batt_cap *tbl = &(batt_cap [0]);
			int               i, mx;
			pmic_get_batt_voltage (&voltage, &raw);
			mx = sizeof(batt_cap) / sizeof(struct _batt_cap);
			for ( i = 0 ; i < mx ; i++ , tbl++ ) {
				if (tbl->raw <= raw) {
					val->intval = tbl->cap;
					break;
				}
			}
			return 0;
		}
		break;
#endif /* CONFIG_MACH_MX51_ERDOS */
	default:
		break;
	}
	return -EINVAL;
}

static int mc13892_battery_read_status(struct mc13892_dev_info *di)
{
	int retval;
#ifdef CONFIG_MACH_MX51_ERDOS
	/* set voltage */
 	retval = pmic_get_batt_voltage(&(di->voltage_raw), 0);
 	if (retval == 0) {
 		di->voltage_uV = di->voltage_raw * 1000;	/* mV -> uV */
	}
#else
	int coulomb;
	retval = pmic_get_batt_voltage(&(di->voltage_raw));
	if (retval == 0)
		di->voltage_uV = di->voltage_raw * BAT_VOLTAGE_UNIT_UV;

	retval = pmic_get_batt_current(&(di->current_raw));
	if (retval == 0) {
		if (di->current_raw & 0x200)
			di->current_uA =
				(0x1FF - (di->current_raw & 0x1FF)) *
				BAT_CURRENT_UNIT_UA * (-1);
		else
			di->current_uA =
				(di->current_raw & 0x1FF) * BAT_CURRENT_UNIT_UA;
	}
	retval = pmic_get_charger_coulomb(&coulomb);
	if (retval == 0)
		di->accum_current_uAh = COULOMB_TO_UAH(coulomb);
#endif /* CONFIG_MACH_MX51_ERDOS */

	return retval;
}

static void mc13892_battery_update_status(struct mc13892_dev_info *di)
{
#ifndef CONFIG_MACH_MX51_ERDOS
	unsigned int value;
#endif /* CONFIG_MACH_MX51_ERDOS */
	int retval;
	int old_battery_status = di->battery_status;

	if (di->battery_status == POWER_SUPPLY_STATUS_UNKNOWN)
		di->full_counter = 0;

	if (di->charger_online) {
#ifdef CONFIG_MACH_MX51_ERDOS
		retval = gpio_charge_status();
		switch (retval) {
		case 0x1: /* STAT2: charged */
			di->battery_status = POWER_SUPPLY_STATUS_FULL;
			break;
		case 0x2: /* STAT1: now charging */
			di->battery_status = POWER_SUPPLY_STATUS_CHARGING;
			break;
		default:
			di->battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
 		}
		if (di->battery_status == POWER_SUPPLY_STATUS_FULL)
			di->full_counter++;
		else
			di->full_counter = 0;
#else
		retval = pmic_read_reg(REG_INT_SENSE0,
					&value, BITFMASK(BIT_CHG_CURRS));

		if (retval == 0) {
			value = BITFEXT(value, BIT_CHG_CURRS);
			if (value)
				di->battery_status =
					POWER_SUPPLY_STATUS_CHARGING;
			else
				di->battery_status =
					POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		if (di->battery_status == POWER_SUPPLY_STATUS_NOT_CHARGING)
			di->full_counter++;
		else
			di->full_counter = 0;
#endif /* CONFIG_MACH_MX51_ERDOS */
	} else {
		di->battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
		di->full_counter = 0;
	}

	dev_dbg(di->bat.dev, "bat status: %d\n",
		di->battery_status);

	if (di->battery_status != old_battery_status)
		power_supply_changed(&di->bat);
}

static void mc13892_battery_work(struct work_struct *work)
{
	struct mc13892_dev_info *di = container_of(work,
						     struct mc13892_dev_info,
						     monitor_work.work);
#ifdef CONFIG_MACH_MX51_ERDOS
	const int interval = HZ * 5;
	mc13892_charger_update_status(di);
	mc13892_battery_sample(di->charger_online, 1, 0);
#else
	const int interval = HZ * 60;
#endif /* CONFIG_MACH_MX51_ERDOS */

	dev_dbg(di->dev, "%s\n", __func__);

	mc13892_battery_update_status(di);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, interval);
}

#ifdef CONFIG_MACH_MX51_ERDOS
#ifdef USE_BATTERY_LOW_INTERRUPT

/*
 * battery_create_inputdevice : create BATTERY key
 */
static struct input_dev *battery_input_dev;
static void battery_create_inputdevice (void)
{
	int                      i, ret, inited;
	struct input_dev        *input = 0;
	struct _batt_input_stat *tbl;

	inited = 0;
	tbl = &(batt_input_stat [0]);
	for ( i = 0 ; i < 2 ; i++ , tbl++ ) {
		if (0 < tbl->event_type) {
			if (inited == 0) {
				input = input_allocate_device ();
				if (input == 0) {
					printk ("battery_create_inputdevice: can't create input device\n");
					return;
				}
				input->name = BATTERY_INPUT_NAME;
				inited = 1;
			}
			input_set_capability (input, tbl->event_type, tbl->event_code);
		}
	}
	if (inited == 1) {
		ret = input_register_device (input);
		if (ret != 0) {
			printk ("battery_create_inputdevice: can't register input device\n");
			input_free_device (input);
		} else {
			battery_input_dev = input;
		}
	}
	return;
}

/*
 * battery_remove_inputdevice : delete BATTERY key
 */
static void battery_remove_inputdevice (void)
{
	if (battery_input_dev != 0) {
		input_unregister_device (battery_input_dev);
		input_free_device (battery_input_dev);
	}
}

/*
 * battery_input_event : event report
 */
static void battery_input_event (struct _batt_input_stat *ev)
{
	if ((battery_input_dev != 0) && (0 < ev->event_type)) {
		input_event (battery_input_dev, ev->event_type, ev->event_code, 1);
		input_sync (battery_input_dev);
		input_event (battery_input_dev, ev->event_type, ev->event_code, 0);
		input_sync (battery_input_dev);
	}
}

/*
 * LOBATLI interrupt
 */
static void battery_lobatli_callback (void *para)
{
	unsigned int value;
	int          ret;

	/*
	 * check BATLI level lower
	 */
	ret = pmic_read_reg (REG_INT_SENSE0, &value, 0x00ffffff);
	printk ("battery_lobatli_callback: reg2 (%d %x)\n", ret, value);
	if ((ret == PMIC_SUCCESS) && ((value & (0x1 << SENSE_LOBATLS)) != 0x00000000)) {
		battery_input_event (&(batt_input_stat [0]));
	}
}

/*
 * LOBATHI interrupt
 */
static void battery_lobathi_callback (void *para)
{
	unsigned int value;
	int          ret;

	/*
	 * check BATHI level lower
	 */
	ret = pmic_read_reg (REG_INT_SENSE0, &value, 0x00ffffff);
	printk ("battery_lobathi_callback: reg2 (%d %x)\n", ret, value);
	if ((ret == PMIC_SUCCESS) && ((value & (0x1 << SENSE_LOBATHS)) == 0x00000000)) {
		/*
		 * caution mode
		 */
		batt_sample.interval = 999;
		battery_input_event (&(batt_input_stat [1]));
	}
}

/*
 * Battery low suspend
 */
static void battery_low_suspend (void)
{
	/*
	 * interrupt enable LOBATLI only
	 */
	extern int pmic_event_for_suspend (type_event event);
	pmic_event_for_suspend (EVENT_LOBATLI);

	/*
	 * DCinput Power mode
	 */
	gpio_battery_enable ( 0 );
}

/*
 * Battery low resume
 */
static void battery_low_resume (void)
{
	/*
	 * interrupt restore
	 */
	extern int pmic_event_for_resume (void);
	pmic_event_for_resume ();
}

/*
 * Battery low check initial
 */
static void battery_low_init (struct mc13892_dev_info *di)
{
	int retval = 0;
	/*
	 * LOBATLI/HI level setting
	 *  BPSNS1/0 0:0 -> 2.8V 3.0V
	 */
	retval = pmic_write_reg (REG_POWER_CTL0, 0x00000000, 0x00030000);
	if (retval != PMIC_SUCCESS) {
		printk ("pmic_battery_probe: can't set BPSNS1/0 %d\n", retval);
	}
	battery_create_inputdevice ();
	bat_event_callback.func  = battery_lobatli_callback;
	bat_event_callback.param = (void *) di;
	pmic_event_subscribe (EVENT_LOBATLI, bat_event_callback);

	bat_event_callback.func  = battery_lobathi_callback;
	bat_event_callback.param = (void *) di;
	pmic_event_subscribe (EVENT_LOBATHI, bat_event_callback);
}

/*
 * Battery low check remove
 */
static void battery_low_remove (struct mc13892_dev_info *di)
{
	pmic_event_callback_t bat_event_callback;

	bat_event_callback.func  = battery_lobatli_callback;
	bat_event_callback.param = (void *) di;
	pmic_event_unsubscribe(EVENT_LOBATLI, bat_event_callback);

	bat_event_callback.func  = battery_lobathi_callback;
	bat_event_callback.param = (void *) di;
	pmic_event_unsubscribe(EVENT_LOBATHI, bat_event_callback);
	battery_remove_inputdevice ();
}
#endif /* USE_BATTERY_LOW_INTERRUPT */

#ifdef USE_BATTERY_LOW_POLLING

extern void resume_power_sw_clean (void);

static volatile int battery_poll_statistics [16];
static volatile int battpollC;
static volatile int battpoll [260][4];
static void LogIn (int a, int b, int c)
{
    int *p;
    if (256 <= battpollC)
        battpollC = 0;
    p = (int *)&(battpoll [battpollC++][0]);
    p [0] = jiffies;
    p [1] = a;
    p [2] = b;
    p [3] = c;
}

/*
 * PMIC alarm set
 */
static int battery_set_alarm (void)
{
	int          rc;
	unsigned int toda, daya;

	/*
	 * wakeup after check interval
	 */
	rc = pmic_read_reg (REG_RTC_TIME, &toda, 0x01FFFF);
	if (rc == 0) {
		rc = pmic_read_reg (REG_RTC_DAY, &daya, 0x007FFF);
	}
	if (rc != 0) {
		printk ("battery_set_alarm: get TODA/DAYA error %d\n", rc);
		return -1;
	}
	toda += batt_poll_interval;
	if ((24 * 60 * 60) <= toda) {
		toda -= (24 * 60 * 60);
		daya++;
	}
	rc = pmic_write_reg (REG_RTC_ALARM, toda, 0x01FFFF);
	if (rc == 0) {
		rc = pmic_write_reg (REG_RTC_DAY_ALARM, daya, 0x007FFF);
	}
	if (rc != 0) {
		printk ("battery_set_alarm: set TODA/DAYA error %d\n", rc);
		return -1;
	}
	return 0;
}

/*
 * poll wake interrupt
 */
static void battery_poll_callback (void *para)
{
	battery_poll_statistics [0]++;
	/*
	 * wakeup after check interval
	 */
	battery_set_alarm ();
}

/*
 * Battery low suspend
 *  called by PM(suspend)
 */
extern int pmic_event_for_suspend (type_event event);
extern int pmic_event_for_resume (void);
static unsigned int pre_alarm [3];
static void battery_low_suspend (void)
{
	int  rc;
	pmic_event_callback_t bat_event_callback;

	battery_poll_statistics [1]++;
	battery_poll_statistics [6] = 0;
	LogIn (0x00010000, battery_poll_statistics [1], battery_poll_statistics [0]);
	/*
	 * previous alarm setting
	 */
	pre_alarm [0] = 1;
	pre_alarm [1] = 0x01FFFF;
	pre_alarm [2] = 0x007FFF;
	pmic_read_reg (REG_RTC_ALARM,     &(pre_alarm [1]), 0x01FFFF);
	pmic_read_reg (REG_RTC_DAY_ALARM, &(pre_alarm [2]), 0x007FFF);

	/*
	 * wakeup after check interval
	 */
	rc = battery_set_alarm ();
	if (rc == 0) {
		/*
		 * interrupt enable TODAI only
		 */
		bat_event_callback.func  = battery_poll_callback;
		bat_event_callback.param = (void *)0;
		pmic_event_subscribe (EVENT_TODAI, bat_event_callback);
		pmic_event_for_suspend (EVENT_TODAI);
	}

	/*
	 * DCinput Power mode
	 */
	gpio_battery_enable ( 0 );

	/*
	 * cleanup Power-SW flag
	 */
	resume_power_sw_clean ();
}

/*
 * Battery low cancel_suspend
 *  cancel suspend mode
 */
static void battery_low_suspend_cancel (void)
{
	int        rc;
	pmic_event_callback_t bat_event_callback;

	/*
	 * interrupt restore
	 */
	pmic_event_for_resume ();

	/*
	 * restore alarm
	 */
	if (pre_alarm [0] == 1) {
		pre_alarm [0] = 0;
		rc = pmic_write_reg (REG_RTC_ALARM, pre_alarm [1], 0x01FFFF);
		if (rc == 0) {
			rc = pmic_write_reg (REG_RTC_DAY_ALARM, pre_alarm [2], 0x007FFF);
		}
		if (rc != 0) {
			printk ("battery_low_resume: restore TODA/DAYA error %d\n", rc);
		}

	bat_event_callback.func  = battery_poll_callback;
	bat_event_callback.param = (void *)0;
	pmic_event_unsubscribe (EVENT_TODAI, bat_event_callback);
	}
}

/*
 * Battery low resume
 *  called by PM(resume)
 */
static void battery_low_resume (void)
{
	battery_poll_statistics [2]++;
	LogIn (0x00020000, battery_poll_statistics [2], battery_poll_statistics [0]);
	battery_low_suspend_cancel ();
	resume_power_sw_clean ();
}

/*
 * check resume condition
 *  called by exit suspend
 *   return 0 - re-suspend
 *   return 1 - wakeup
 */
extern struct platform_driver pmic_adc_driver_ldm;
static int battery_low_resume_check (void)
{
	extern int is_resume_power_sw (void);
	int rc, raw;
	int resume_by_sw = is_resume_power_sw ();

	battery_poll_statistics [3]++;
	battery_poll_statistics [6]++;
	/*
	 * check resume by Power-SW
	 */
	if (resume_by_sw != 0) {
		battery_low_suspend_cancel ();
		battery_poll_statistics [4]++;
		LogIn (0x00070099, (battery_poll_statistics [4]<<16)|(battery_poll_statistics [6]&0xFFFF), resume_by_sw);
		return 1;		/* wakeup by Power-SW */
	}

	/*
	 * check BATTERY shutdown voltage
	 */
	if (pmic_adc_driver_ldm.resume != 0) {
		(*pmic_adc_driver_ldm.resume)((struct platform_device *)0);
	}
	pmic_event_for_suspend (EVENT_ADCBISDONEI);	/* ONLY enable ADC interrupt */
	rc = mc13892_battery_sample (1, 0, 1);		/* fake ONLINE/noWAIT/FORCE */
	raw = batt_sample.raw;
	pmic_event_for_suspend (EVENT_TODAI);		/* ONLY enable ALARM interrupt */
	if (pmic_adc_driver_ldm.suspend != 0) {
		(*pmic_adc_driver_ldm.suspend)((struct platform_device *)0, PMSG_SUSPEND);
	}
	if (0 < rc) {
		if (raw <= batt_cap[BATT_CAP_SHUTDOWN_LEVEL].raw) {
			battery_low_suspend_cancel ();
			battery_poll_statistics [5]++;
			LogIn (0x00070098, (battery_poll_statistics [5]<<16)|(battery_poll_statistics [6]&0xFFFF), (raw<<16)|batt_cap[BATT_CAP_SHUTDOWN_LEVEL].raw);
			return 1;	/* shutdown voltage */
		}
	}

	return 0;
}

/*
 * Battery low check initial
 */
static void battery_low_init (struct mc13892_dev_info *di)
{
	extern int resume_check_register_ops (int (*func)(void));
	int rc;

	rc = resume_check_register_ops (battery_low_resume_check);
	if (rc != 0) {
		printk ("battery_low_init: resume_check_register_ops error %d\n", rc);
	}
}

/*
 * Battery low check remove
 */
static void battery_low_remove (struct mc13892_dev_info *di)
{
	extern int resume_check_unregister_ops (int (*func)(void));
	int rc;

	rc = resume_check_unregister_ops (battery_low_resume_check);
	if (rc != 0) {
		printk ("battery_low_remove: resume_check_unregister_ops error %d\n", rc);
	}
}

#endif /* USE_BATTERY_LOW_POLLING */

#else

static void charger_online_event_callback(void *para)
{
	struct mc13892_dev_info *di = (struct mc13892_dev_info *) para;
	pr_info("\n\n DETECTED charger plug/unplug event\n");
	mc13892_charger_update_status(di);
}
#endif /* CONFIG_MACH_MX51_ERDOS */


static int mc13892_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct mc13892_dev_info *di = to_mc13892_dev_info(psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (di->battery_status == POWER_SUPPLY_STATUS_UNKNOWN) {

#ifdef CONFIG_MACH_MX51_ERDOS
			mc13892_charger_update_status(di);	/* first call */
			mc13892_battery_update_status(di);
#else
			mc13892_battery_update_status(di);
			mc13892_charger_update_status(di);
#endif /* CONFIG_MACH_MX51_ERDOS */
		}
		val->intval = di->battery_status;
		return 0;
	default:
		break;
	}

	mc13892_battery_read_status(di);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->voltage_uV;
		break;
#ifndef CONFIG_MACH_MX51_ERDOS
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_uA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = di->accum_current_uAh;
		break;
#endif /* CONFIG_MACH_MX51_ERDOS */
	default:
		return -EINVAL;
	}

	return 0;
}

static int pmic_battery_remove(struct platform_device *pdev)
{
#ifdef CONFIG_MACH_MX51_ERDOS
	struct mc13892_dev_info *di = platform_get_drvdata(pdev);

	battery_low_remove (di);
#else
	pmic_event_callback_t bat_event_callback;
	struct mc13892_dev_info *di = platform_get_drvdata(pdev);

	bat_event_callback.func = charger_online_event_callback;
	bat_event_callback.param = (void *) di;
	pmic_event_unsubscribe(EVENT_CHGDETI, bat_event_callback);
#endif /* CONFIG_MACH_MX51_ERDOS */

	cancel_rearming_delayed_workqueue(di->monitor_wqueue,
					  &di->monitor_work);
	destroy_workqueue(di->monitor_wqueue);
	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->charger);

	kfree(di);

	return 0;
}

#ifdef CONFIG_MACH_MX51_ERDOS
#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
static int proc_battery_show(struct seq_file *m, void *v)
{
	unsigned short    mV;
	int               raw;
	int               i, mx;
	struct _batt_cap *tbl = &(batt_cap [0]);

	if (pmic_get_voltage(GEN_PURPOSE_AD5, &mV, &raw, 1) == 0) {
	seq_printf (m, "%dmV (%d)  ", mV, raw);
	} else {
		seq_printf (m, "---mV (---)  ");
	}
	mx = sizeof(batt_cap) / sizeof(struct _batt_cap);
	for ( i = 0 ; i < mx ; i++ , tbl++ ) {
		seq_printf (m, " %d:%d", tbl->raw, tbl->cap);
	}

	seq_printf (m, " caution(%d:%d - %d)", batt_cap[BATT_CAP_CAUTION_LEVEL].raw,
		    batt_cap[BATT_CAP_CAUTION_LEVEL].cap, batt_sample.mode_caution);

#ifdef USE_BATTERY_LOW_INTERRUPT
	if (pmic_get_voltage (GEN_PURPOSE_AD2, &mV, &raw, 0) == 0) {
		seq_printf (m, " bpsns:%dmV(%d)", mV, raw);
	} else {
		seq_printf (m, " bpsns:---mV(---)");
	}
#endif /* USE_BATTERY_LOW_INTERRUPT */
#ifdef USE_BATTERY_LOW_POLLING
	seq_printf (m, " shutdown(%d:%d) poll:%dsec",
		    batt_cap[BATT_CAP_SHUTDOWN_LEVEL].raw,
		    batt_cap[BATT_CAP_SHUTDOWN_LEVEL].cap, batt_poll_interval);
#endif /* USE_BATTERY_LOW_POLLING */

	seq_printf (m, "\n");
	return 0;
}

static int proc_battery_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_battery_show, NULL);
}

/*
 * update batt_cap[] table
 *  capacity rate
 *  echo "raw:cap raw:cap ..." >/proc/battery
 */
static ssize_t proc_battery_write(struct file *file, const char __user *user,
                                  size_t count, loff_t *data)
{
	int   ret, offset, id, mx, raw, cap, mode;
	char  c, str [64];

	if ((count == 0) || (sizeof(str) <= count)) {
		return -EINVAL;
	}
	ret = copy_from_user(str, user, count);
	if (ret != 0) {
		return -EFAULT;
	}

	/*
	 * front space cut
	 */
	for ( offset = 0 ; offset < count ; offset++ ) {
		c = str [offset];
		if (c != ' ') {
			break;
		}
	}
#ifdef USE_BATTERY_LOW_POLLING
	if (strncmp (&(str [offset]), "poll", 4) == 0) {
		int  val;
		char mm [16];
		if (sscanf (&(str [offset]), "%s %d", mm, &val) == 2) {
			if (0 < val) {
				batt_poll_interval = val;
			}
		}
		return count;
	}
#endif /* USE_BATTERY_LOW_POLLING */

	/*
	 * check sample rate
	 */
	if (strncmp (&(str [offset]), "rate", 4) == 0) {
		int  val;
		char mm [16];
		if (sscanf (&(str [offset]), "%s %d", mm, &val) == 2) {
			if (0 < val) {
				batt_sample_rate = val;
			}
		}
		return count;
	}

	raw  = 0;
	cap  = 0;
	mode = 0;
	id   = 0;
	mx   = sizeof(batt_cap) / sizeof(struct _batt_cap);
	for ( offset = 0 ; offset < count ; offset++ ) {
		c = str [offset];
		if (('0' <= c) && (c <= '9')) {
			if (mode == 0) {
				raw = raw * 10 + (c - '0');
			} else {
				cap = cap * 10 + (c - '0');
			}
		} else if (c == ':') {
			mode = 1;
		}
		if ((c == ' ') || ((offset + 1) == count)) {
			if (mx <= id) {
				break;
			}
			batt_cap [id].raw = raw;
			batt_cap [id].cap = cap;
			id++;
			raw  = 0;
			cap  = 0;
			mode = 0;
		}
	}
	return count;
}

static const struct file_operations battery_proc_fops = {
	.owner   = THIS_MODULE,
	.open    = proc_battery_open,
	.read    = seq_read,
	.write   = proc_battery_write,
	.llseek  = seq_lseek,
	.release = single_release,
};
#endif /* CONFIG_PROC_FS */
#endif /* CONFIG_MACH_MX51_ERDOS */

static int pmic_battery_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct mc13892_dev_info *di;
#ifndef CONFIG_MACH_MX51_ERDOS
	pmic_event_callback_t bat_event_callback;
#endif /* CONFIG_MACH_MX51_ERDOS */
	pmic_version_t pmic_version;

	/* Only apply battery driver for MC13892 V2.0 due to ENGR108085 */
	pmic_version = pmic_get_version();
	if (pmic_version.revision < 20) {
		pr_debug("Battery driver is only applied for MC13892 V2.0\n");
		return -1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		retval = -ENOMEM;
		goto di_alloc_failed;
	}

	platform_set_drvdata(pdev, di);

	di->dev	= &pdev->dev;
	di->bat.name	= "mc13892_bat";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = mc13892_battery_props;
	di->bat.num_properties = ARRAY_SIZE(mc13892_battery_props);
	di->bat.get_property = mc13892_battery_get_property;

	di->battery_status = POWER_SUPPLY_STATUS_UNKNOWN;

	retval = power_supply_register(&pdev->dev, &di->bat);
	if (retval) {
		dev_err(di->dev, "failed to register battery\n");
		goto batt_failed;
	}
	di->charger.name	= "mc13892_charger";
	di->charger.type = POWER_SUPPLY_TYPE_MAINS;
	di->charger.properties = mc13892_charger_props;
	di->charger.num_properties = ARRAY_SIZE(mc13892_charger_props);
	di->charger.get_property = mc13892_charger_get_property;
	retval = power_supply_register(&pdev->dev, &di->charger);
	if (retval) {
		dev_err(di->dev, "failed to register charger\n");
		goto charger_failed;
	}
	INIT_DELAYED_WORK(&di->monitor_work, mc13892_battery_work);
	di->monitor_wqueue = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!di->monitor_wqueue) {
		retval = -ESRCH;
		goto workqueue_failed;
	}
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ * 10);

#ifdef CONFIG_MACH_MX51_ERDOS
#ifdef ALWAY_REFON
	gpio_refon(1);		// select measurement
#endif /* ALWAY_REFON */
	/*
	 * initial value (BATT voltage raw)
	 */
#ifdef CONFIG_PROC_FS
	proc_create ("battery", 0, NULL, &battery_proc_fops);
#endif /* CONFIG_PROC_FS */
	battery_low_init (di);
#else
	bat_event_callback.func = charger_online_event_callback;
	bat_event_callback.param = (void *) di;
	pmic_event_subscribe(EVENT_CHGDETI, bat_event_callback);

	pmic_stop_coulomb_counter();
	pmic_calibrate_coulomb_counter();
#endif /* CONFIG_MACH_MX51_ERDOS */
	goto success;

workqueue_failed:
	power_supply_unregister(&di->charger);
charger_failed:
	power_supply_unregister(&di->bat);
batt_failed:
	kfree(di);
di_alloc_failed:
success:
	dev_dbg(di->dev, "%s battery probed!\n", __func__);
	return retval;


	return 0;
}

#ifdef CONFIG_PM
static int pmic_battery_suspend (struct platform_device *pdev, pm_message_t state)
{
#ifdef CONFIG_MACH_MX51_ERDOS
	battery_low_suspend ();
#endif /* CONFIG_MACH_MX51_ERDOS */
        return 0;
}

static int pmic_battery_resume (struct platform_device *pdev)
{
#ifdef CONFIG_MACH_MX51_ERDOS
	battery_low_resume ();
#endif /* CONFIG_MACH_MX51_ERDOS */

	return 0;
}
#else
#define pmic_battery_suspend 0
#define pmic_battery_resume  0
#endif /* CONFIG_PM */

static struct platform_driver pmic_battery_driver_ldm = {
	.driver = {
		   .name = "pmic_battery",
		   .bus = &platform_bus_type,
		   },
	.probe = pmic_battery_probe,
	.remove = pmic_battery_remove,
	.suspend = pmic_battery_suspend,
	.resume = pmic_battery_resume,
};

static int __init pmic_battery_init(void)
{
	pr_debug("PMIC Battery driver loading...\n");
	return platform_driver_register(&pmic_battery_driver_ldm);
}

static void __exit pmic_battery_exit(void)
{
	platform_driver_unregister(&pmic_battery_driver_ldm);
	pr_debug("PMIC Battery driver successfully unloaded\n");
}

module_init(pmic_battery_init);
module_exit(pmic_battery_exit);

MODULE_DESCRIPTION("pmic_battery driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
