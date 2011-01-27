/**
 * mach-mx51/mx51_erdos_dev_params.c
 *
 * This file contains the board specific controll SDIO status routines.
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
 * 2009/07/29 : created.
 * 2009/08/01 : LED R/G change.
 *		 later QA1.5 Green-LED [LED-G]
 * 2009/08/21 : sdio_detect_resume() check sdio_detect_enable is 1, delete LED on.
 *               LED suspend/resume -> LED driver.
 * 2010/11/25 : ported to 2.6.31 by Andrey Zhornyak darion76@gmail.com
 */

#include <linux/input.h>
#include <linux/sysdev.h>
#include <linux/pmic_light.h>
#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#endif	/* CONFIG_PROC_FS */
#ifdef CONFIG_ALLOC_FUNC_KEY
#include <asm/func_key.h>
#endif
#include <mach/mx51_erdos_dev_params.h>
#include <linux/pmic_light.h>

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)

DEFINE_MUTEX(sdio_detect_mutex);
EXPORT_SYMBOL(sdio_detect_mutex);

u_char sdio_detect_enable = 0; /* user controll flags (0:dis 1:ena) */
EXPORT_SYMBOL(sdio_detect_enable);

#ifdef CONFIG_ALLOC_FUNC_KEY
static struct work_struct sdio_detect_change_task;
#endif
#ifdef CONFIG_PROC_FS
static struct work_struct sdio_detect_change_task_for_proc;
#endif

#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
extern void esdhc_foce_cd_local(int id);
#endif
extern int gpio_wlan_start(void);
extern void gpio_wlan_stop(void);

extern void mxc_mmc_force_detect(int id);

#define	SDIO_DETECT_LED_CH	4	/* LED-G / 2:[LED-KP] */
#define	SDIO_DETECT_LED_LVL	4

void sdio_detect_led_on(void)
{
	PMIC_STATUS	 status;

	if (sdio_detect_enable)
		return;

	status = mc13892_bklit_set_current(SDIO_DETECT_LED_CH,
					   5);
	if (status < 0)
		printk("%s(%d):. cannot set led (%d)\n",
			__FUNCTION__, __LINE__, (int) status);

	status = mc13892_bklit_set_dutycycle(SDIO_DETECT_LED_CH,
					     SDIO_DETECT_LED_LVL);
	if (status < 0)
		printk("%s(%d):. cannot set led (%d)\n",
			__FUNCTION__, __LINE__, (int) status);
}
EXPORT_SYMBOL(sdio_detect_led_on);

void sdio_detect_led_off(void)
{
	PMIC_STATUS	 status;

	if (sdio_detect_enable)
		return;

	status = mc13892_bklit_set_current(SDIO_DETECT_LED_CH, 0);
	if (status < 0)
		printk("%s(%d):. cannot set led (%d)\n",
			__FUNCTION__, __LINE__, (int) status);

	status = mc13892_bklit_set_dutycycle(SDIO_DETECT_LED_CH, 0);
	if (status < 0)
		printk("%s(%d):. cannot set led (%d)\n",
			__FUNCTION__, __LINE__, (int) status);
}
EXPORT_SYMBOL(sdio_detect_led_off);

void sdio_detect_led(void)
{
	PMIC_STATUS	 status;

	if (sdio_detect_enable) {
		status = mc13892_bklit_set_current(SDIO_DETECT_LED_CH,
						   5);
		if (status < 0)
			printk("%s(%d):. cannot set led (%d)\n",
				__FUNCTION__, __LINE__, (int) status);

		status = mc13892_bklit_set_dutycycle(SDIO_DETECT_LED_CH,
						     SDIO_DETECT_LED_LVL);
		if (status < 0)
			printk("%s(%d):. cannot set led (%d)\n",
				__FUNCTION__, __LINE__, (int) status);
	}
	else {
		status = mc13892_bklit_set_current(SDIO_DETECT_LED_CH, 0);
		if (status < 0)
			printk("%s(%d):. cannot set led (%d)\n",
				__FUNCTION__, __LINE__, (int) status);

		status = mc13892_bklit_set_dutycycle(SDIO_DETECT_LED_CH, 0);
		if (status < 0)
			printk("%s(%d):. cannot set led (%d)\n",
				__FUNCTION__, __LINE__, (int) status);
	}
}
EXPORT_SYMBOL(sdio_detect_led);

#ifdef CONFIG_PROC_FS
#define	GPIO_WLAN_PROC_NAME	"wlan_sw"
static struct proc_dir_entry *sdio_detect_proc = NULL;

static int sdio_detect_proc_read(char *buffer, char **start, off_t offset,
				 int count, int *eof, void *data)

{
	int	       len;
	char	      *p = buffer;
	extern int get_gpio_sw_status (void);

	/*
	 *  b00 - WLAN_KEY  (1:ON)
	 */
	p += sprintf (p,"%d\n", sdio_detect_enable);

	len = (p - buffer) - offset;
	if (len < 0) {
		len = 0;
	}

	*start = buffer + offset;

	return len;
}


static int sdio_detect_proc_write(struct file *file, const char __user *buffer,
				unsigned long count, void *data)
{
	char * p   = (char *)buffer;
	char * ep;
	int    val;

	if (count > 3)
		return (-EINVAL);

	val = (simple_strtol(p, &ep, 2) & 0x1);
	if (val < 0)
		return count;

	if (val != 1)
		val = 0;

	if ((val && !sdio_detect_enable) || (!val &&  sdio_detect_enable)) {
		schedule_work(&sdio_detect_change_task_for_proc);
	}

	return count;
}

#endif	/* GPIO_WLAN_PROC_NAME */

static int sdio_detect_proc_entry(void)
{
	sdio_detect_proc = create_proc_entry(GPIO_WLAN_PROC_NAME,
					     S_IRUGO, NULL);
	if (sdio_detect_proc == NULL) {
		printk(KERN_ERR "Cannot init proc entry for wlan_sw\n");
		return (-1);
	}

//	sdio_detect_proc->owner	     = THIS_MODULE;
	sdio_detect_proc->data	     = NULL;
	sdio_detect_proc->read_proc  = sdio_detect_proc_read;
	sdio_detect_proc->write_proc = sdio_detect_proc_write;

	return 0;
}

static void sdio_detect_change(struct work_struct *work)
{
	int		 ret;

	mutex_lock(&sdio_detect_mutex);

	sdio_detect_enable = ~(sdio_detect_enable) & 0x1;

	if (sdio_detect_enable) {
		gpio_wlan_start();
	}

	sdio_detect_led();

	mxc_mmc_force_detect(1);

#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
	ret = mx51_erdos_dev_params_write(DEV_PARAMS_OFFSET_WLAN,
					  DEV_PARAMS_SIZE_WLAN,
					  &sdio_detect_enable);
	if (ret != DEV_PARAMS_SIZE_WLAN)
		printk(KERN_ERR
		       "%s(%d): Failed to write device param.\n",
			__FUNCTION__, __LINE__);
#endif

	if (!sdio_detect_enable) {
		gpio_wlan_stop();
	}

	mutex_unlock(&sdio_detect_mutex);
}

void sdio_detect_disable_force(void)
{
	int		 ret;

	mutex_lock(&sdio_detect_mutex);

	sdio_detect_enable = 0;

	sdio_detect_led();

#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
	esdhc_foce_cd_local(1); //darion76

	ret = mx51_erdos_dev_params_write(DEV_PARAMS_OFFSET_WLAN,
					  DEV_PARAMS_SIZE_WLAN,
					  &sdio_detect_enable);
	if (ret != DEV_PARAMS_SIZE_WLAN)
		printk(KERN_ERR
		       "%s(%d): Failed to write device param.\n",
			__FUNCTION__, __LINE__);
#endif

	gpio_wlan_stop();

	mutex_unlock(&sdio_detect_mutex);
}
EXPORT_SYMBOL(sdio_detect_disable_force);

#ifdef CONFIG_PROC_FS
static void sdio_detect_change_no_write(struct work_struct *work)
{
	mutex_lock(&sdio_detect_mutex);

	sdio_detect_enable = ~(sdio_detect_enable) & 0x1;

	if (sdio_detect_enable) {
		gpio_wlan_start();
	}

	sdio_detect_led();

	mxc_mmc_force_detect(1);

	if (!sdio_detect_enable) {
		gpio_wlan_stop();
	}

	mutex_unlock(&sdio_detect_mutex);
}
#endif

#ifdef CONFIG_ALLOC_FUNC_KEY
static void wlan_fnkey_handler(int id, int value, void *arg)
{
	if (id != FNKEY_ID_WIRELESS)
		return;

	if (value == 1)	{	/* case in PRESS */
			schedule_work(&sdio_detect_change_task);
	}
}
#endif

#ifdef CONFIG_PM

#include <linux/delay.h>

int sdio_detect_suspend(struct sys_device *dev, pm_message_t state)
{
	if (sdio_detect_enable) {
		gpio_wlan_stop();
	}

	return 0;
}

static int sdio_detect_resume(struct sys_device *dev)
{
	if (sdio_detect_enable) {
		gpio_wlan_start();
		printk(KERN_INFO "SDIO detect resumed.\n");
	}

	return 0;
}

static struct sysdev_class sdio_detect_sysclass = {
	.name		= "sdio_detect",
	.suspend	= sdio_detect_suspend,
	.resume		= sdio_detect_resume,
};

static struct sys_device sdio_detect_sysdev = {
	.id		= 0,
	.cls		= &sdio_detect_sysclass,
};


static int __init sdio_detect_init(void)
{
	int ret = 0;

	ret = sysdev_class_register(&sdio_detect_sysclass);
	if (ret)
		return ret;

	ret = sysdev_register(&sdio_detect_sysdev);

	printk("SDIO detect port functions initialized.\n");

	return ret;
}
subsys_initcall(sdio_detect_init);
#endif/* CONFIG_PM */

static int sdio_detect_late_init(void)
{
	int ret;

#ifdef CONFIG_ALLOC_FUNC_KEY
	INIT_WORK(&sdio_detect_change_task, sdio_detect_change);
#endif
#ifdef CONFIG_PROC_FS
	INIT_WORK(&sdio_detect_change_task_for_proc,
	          sdio_detect_change_no_write);
#endif

	ret = sdio_detect_proc_entry();
	if (ret)
		printk(KERN_ERR "Can not setup proc entry for wlan.\n");

#ifdef CONFIG_ALLOC_FUNC_KEY
	ret = fnkey_register(FNKEY_ID_WIRELESS, wlan_fnkey_handler, NULL);
	if (ret)
		printk(KERN_ERR "Can not register fucntion key for wlan.\n");
#endif	/* CONFIG_ALLOC_FUNC_KEY */

#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
	ret = mx51_erdos_dev_params_read(DEV_PARAMS_OFFSET_WLAN,
					 DEV_PARAMS_SIZE_WLAN,
					 &sdio_detect_enable);

	if (ret != DEV_PARAMS_SIZE_WLAN)
		printk(KERN_ERR "%s(%d): Failed to read device param.\n",
			__FUNCTION__, __LINE__);
#endif

	/* device parameter is not erase state AND
	   device parameter = "ON" AND now detect status = "OFF" */
	if (sdio_detect_enable == 1) {
		sdio_detect_led();
		gpio_wlan_start();
		mxc_mmc_force_detect(1);
	} else {
		sdio_detect_enable = 0;
	}

	return 0;
}

late_initcall(sdio_detect_late_init);
#endif
