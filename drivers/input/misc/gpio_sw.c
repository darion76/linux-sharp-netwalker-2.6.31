/**
 * @file  drivers/input/misc/gpio_sw.c
 *
 * @brief Driver for the gpio Switch.
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
 *
 * modification information
 * ------------------------
 * 2009/08/07 : check wakeup by Power-SW, ignore event.
 * 2009/08/21 : add resume_power_sw_flag.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/gpio_sw.h>
#include <mach/gpio.h>

struct gpio_sw_dev {
	int                          irq_init;
	struct input_dev             *input;
	struct gpio_sw_platform_data *data;
};

/*
 * mark resume by Power-SW
 */
static int resume_power_sw_flag;

/*
 * check resume by Power-SW
 *  return 0 - none
 *  return 1 - by Power-SW
 */
int is_resume_power_sw (void)
{
	return resume_power_sw_flag;
}
void resume_power_sw_clean (void)
{
	resume_power_sw_flag = 0;
}

static int gpio_sw_create_inputdevice(struct gpio_sw_dev *dev)
{
	int ret = 0;

	dev->input = input_allocate_device();
	if (!dev->input) {
		ret = -ENOMEM;
		goto err;
	}

	dev->input->name = dev->data->input_name;
	input_set_capability(dev->input, dev->data->event_type,
	                     dev->data->event_code);

	ret = input_register_device(dev->input);
	if (ret) {
		goto err;
	}
err:
	if (ret) {
		input_free_device(dev->input);
		dev = NULL;
	}
	return ret;
}

static void gpio_sw_remove_inputdevice(struct gpio_sw_dev *dev)
{
	input_unregister_device(dev->input);
	input_free_device(dev->input);
}

static irqreturn_t gpio_sw_irq(int irq, void *dev_id)
{
	struct gpio_sw_dev *dev = dev_id;
	int                 type;
	int                 value;
	int                 raw_value;
	int                 ignore;

	raw_value = value = dev->data->get_value();
	if (dev->data->output_pol) {
		value = value ? 0 : 1;
	}
	/*
	 * check for after Power-SW(wakeup by Power-SW)
	 */
	ignore = 0;
	if (dev->data->event_code == KEY_POWER) {
		extern int is_near_wakeup (void);
		if (is_near_wakeup () == 1) {
			resume_power_sw_flag = 1;
			ignore = 1;	/* near wakeup supress event */
		} else {
			resume_power_sw_flag = 0;
		}
	}

	if (ignore == 0) {
	input_event(dev->input, dev->data->event_type,
	            dev->data->event_code, value);
	input_sync(dev->input);
	}

	if (raw_value) {
		type = IRQ_TYPE_LEVEL_LOW;
	} else {
		type = IRQ_TYPE_LEVEL_HIGH;
	}
	set_irq_type(dev->data->irq, type);

	return IRQ_HANDLED;
}

static int gpio_sw_setup_irq(struct platform_device *pdev,
                              struct gpio_sw_dev *dev)
{
	int ret;
	int type;
	int value;

	value = dev->data->get_value();
	if (value) {
		type = IRQ_TYPE_LEVEL_LOW;
	} else {
		type = IRQ_TYPE_LEVEL_HIGH;
	}

	set_irq_type(dev->data->irq, type);
	ret = request_irq(dev->data->irq, gpio_sw_irq, 0,
	                  dev->data->irq_name, dev);
	if (ret) {
		goto err;
	}
	if (dev->data->wake) {
		enable_irq_wake(dev->data->irq);
	}
	dev->irq_init = 1;
err:
	return ret;
}

static void gpio_sw_free_irq(struct gpio_sw_dev *dev)
{
	free_irq(dev->data->irq, dev);
	dev->irq_init = 0;
}

static int __devinit gpio_sw_probe(struct platform_device *pdev)
{
	int                 ret = 0;
	struct gpio_sw_dev *dev;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		printk(KERN_ERR "%s : Cannot allocate memory\n", __FUNCTION__);
		ret = -ENOMEM;
		goto err;
	}
	dev_set_drvdata(&pdev->dev, dev);
	dev->data = pdev->dev.platform_data;

	ret = gpio_sw_setup_irq(pdev, dev);
	if (ret) {
		printk(KERN_ERR "%s : Cannot requeset irq\n",
		       __FUNCTION__);
		goto err;
	}

	ret = gpio_sw_create_inputdevice(dev);
	if (ret) {
		printk(KERN_ERR "%s : Cannot create inputdevice\n",
		       __FUNCTION__);
		goto err;
	}
err:
	if (ret) {
		if (dev) {
			if (dev->input) {
				gpio_sw_remove_inputdevice(dev);
			}
			if (dev->irq_init) {
				gpio_sw_free_irq(dev);
			}
			kfree(dev);
			dev_set_drvdata(&pdev->dev, NULL);
		}
	}
	return ret;
}

static int __devexit gpio_sw_remove(struct platform_device *pdev)
{
	struct gpio_sw_dev *dev = dev_get_drvdata(&pdev->dev);

	gpio_sw_remove_inputdevice(dev);
	dev_set_drvdata(&pdev->dev, NULL);
	return 0;
}

static struct platform_driver gpio_sw_driver = {
	.probe  = gpio_sw_probe,
	.remove = gpio_sw_remove,
	.driver = {
		.name  = "gpio_sw",
		.owner = THIS_MODULE,
	},
};

static int __init gpio_sw_init(void)
{
	return platform_driver_register(&gpio_sw_driver);
}

static void __exit gpio_sw_exit(void)
{
	platform_driver_unregister(&gpio_sw_driver);
}

module_init(gpio_sw_init);
module_exit(gpio_sw_exit);

MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_AUTHOR("Nissin Systems Co.,Ltd");
MODULE_LICENSE("GPL");
