/**
 * drivers/video/backlight/mxc_pwm.c
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <mach/clock.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/input.h>
#include <asm/func_key.h>

#define MXC_PWM_DEV_NAME       "mxc_pwm_backlight"
#define MXC_MAX_INTENSITY      100
#define MXC_DEFAULT_INTENSITY  100
#define MXC_INTENSITY_OFF      0

#define PWMCR  0x00
#define PWMSR  0x04
#define PWMIR  0x08
#define PWMSAR 0x0c
#define PWMPR  0x10
#define PWMCNR 0x14

#define PWM_REG(reg) (IO_ADDRESS(PWM1_BASE_ADDR) + reg)

#define PWMCR_ENABLE   0x00810641  /* ipg_clk, perscaler=10, enable */
#define PWMCR_DISABLE  0x00000000  /* disable         */
#define PWM_TARGET_CLK 300         /* 400Hz           */

struct mxc_pwm_dev_data {
	struct backlight_device *bd;
	int                     intensity;
};
static struct mxc_pwm_dev_data *mxc_pwm_dev_data;

/**
 * mxc_pwm_set_duty - set the duty ratio
 *
 * @level : duty ratio [%]
 */
void mxc_pwm_set_duty(u8 level)
{
	u32 period;
	u32 sample;

	if (level > 100) {
		level = 100;
	}

	period = __raw_readl(PWM_REG(PWMPR));
	sample = level * (period / 100);
	sample &= 0xffff;
	__raw_writel(sample, PWM_REG(PWMSAR));
}
EXPORT_SYMBOL(mxc_pwm_set_duty);

static int mxc_pwm_send_intensity(struct backlight_device *bd)
{
	struct mxc_pwm_dev_data *devdata  = dev_get_drvdata(&bd->dev);
	int                     intensity = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK) {
		intensity = 0;
	}
	if (bd->props.fb_blank != FB_BLANK_UNBLANK) {
		intensity = 0;
	}

	mxc_pwm_set_duty(intensity);
	devdata->intensity = intensity;
	return 0;
}

static int mxc_pwm_get_intensity(struct backlight_device *bd)
{
	struct mxc_pwm_dev_data *devdata = dev_get_drvdata(&bd->dev);
	return devdata->intensity;
}

static int mxc_pwm_check_fb(struct fb_info *info)
{
	if (strcmp(info->fix.id, "DISP3 BG")) {
		return 0;
	}
	return 1;
}

static struct backlight_ops mxc_pwm_ops = {
	.get_brightness = mxc_pwm_get_intensity,
	.update_status  = mxc_pwm_send_intensity,
	.check_fb       = mxc_pwm_check_fb,
};

static void mxc_pwm_bd_change(int id, int value, void *arg)
{
	struct mxc_pwm_dev_data *devdata = arg;
	struct backlight_device *bd      = devdata->bd;

	/* press */
	if (value == 1) {
		bd->props.power = (bd->props.power == FB_BLANK_UNBLANK) ?
		                   FB_BLANK_NORMAL : FB_BLANK_UNBLANK;
		mxc_pwm_send_intensity(devdata->bd);
	}
}

/**
 * mxc_pwm_init - initialize the pwm devices
 *
 * return codes:
 *   0     : succeeded
 *   others: failed
 */
static int __init mxc_pwm_init(void)
{
	struct platform_device  pdev;
	struct backlight_device *bd;
	struct mxc_pwm_dev_data *devdata;
	struct clk              *pwm_ipg_clk;
	u32                     rate;
	u32                     period;
	int                     ret;

	pdev.dev.bus  = &platform_bus_type;
	pdev.id       = 0;
	pwm_ipg_clk = clk_get(&pdev.dev, "pwm_ipg_clk");
	if (IS_ERR(pwm_ipg_clk)) {
		return -EIO;
	}
	clk_enable(pwm_ipg_clk);

	rate = clk_get_rate(pwm_ipg_clk) / (((PWMCR_ENABLE >> 4) & 0xff) + 1);
	period = rate / PWM_TARGET_CLK;
	period &= 0xffff;
	__raw_writel(period, PWM_REG(PWMPR));
	__raw_writel(PWMCR_ENABLE, PWM_REG(PWMCR));

	devdata = kzalloc(sizeof(*devdata), GFP_KERNEL);
	if (!devdata) {
		return -ENOMEM;
	}
	mxc_pwm_dev_data = devdata;

	bd = backlight_device_register(MXC_PWM_DEV_NAME, NULL, devdata,
	                               &mxc_pwm_ops);
	if (IS_ERR(bd)) {
		ret = PTR_ERR(bd);
		goto err0;
	}
	devdata->bd = bd;

	ret = fnkey_register(FNKEY_ID_LCD_BL, mxc_pwm_bd_change, devdata);
	if (ret) {
		goto err1;
	}

	bd->props.brightness     = MXC_DEFAULT_INTENSITY;
	bd->props.max_brightness = MXC_MAX_INTENSITY;

	printk(KERN_INFO "PWM: Rate=%u [Hz] Period=%u [cnt]\n",
	       rate, period);

	return 0;
err1:
	backlight_device_unregister(bd);
err0:
	kfree(devdata);
	mxc_pwm_dev_data = NULL;
	return ret;
}

/**
 * mxc_pwm_exit - remove the pwm devices
 */
static void __exit mxc_pwm_exit(void)
{
	__raw_writel(PWMCR_DISABLE, PWM_REG(PWMCR));
	fnkey_unregister(FNKEY_ID_LCD_BL);
	mxc_pwm_dev_data->bd->props.brightness = MXC_INTENSITY_OFF;
	backlight_update_status(mxc_pwm_dev_data->bd);
	backlight_device_unregister(mxc_pwm_dev_data->bd);
}

module_init(mxc_pwm_init);
module_exit(mxc_pwm_exit);

MODULE_DESCRIPTION("PWM driver for Sharp LCD devices");
MODULE_AUTHOR("Nissin Systems Co.,Ltd");
MODULE_LICENSE("GPL");
