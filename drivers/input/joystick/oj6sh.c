/**
 * @file  drivers/input/joystick/oj6sh.c
 *
 * @brief Driver for the OJ6SH-T25 Optical Joystick.
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
 * 2009/08/09 : oj6sh_power_up() delay add for resume(some wait after hard-reset)
 *
 */

#include <linux/module.h>
#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#endif
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/oj6sh.h>
#include <mach/gpio.h>
#include <asm/uaccess.h>
#include <asm/func_key.h>

enum oj6sh_reg {
	OJ6SH_PRODUCT = 0x0,
	OJ6SH_REVISION,
	OJ6SH_MOTION,
	OJ6SH_DELTA_X,
	OJ6SH_DELTA_Y,
	OJ6SH_SQUAL,
	OJ6SH_SHUT_UPPER,
	OJ6SH_SHUT_LOWER,
	OJ6SH_PIX_GRAB = 0x0b,
	OJ6SH_CRC0,
	OJ6SH_CRC1,
	OJ6SH_CRC2,
	OJ6SH_CRC3,
	OJ6SH_SELF_TEST,
	OJ6SH_CFG,
	OJ6SH_POWER_UP_RESET = 0x3a,
	OJ6SH_INV_REVISION = 0x3e,
	OJ6SH_INV_PRODUCT,
};

static const struct self_test_crc {
	enum oj6sh_reg reg;
	u8             value;
} oj6sh_test_crc[] = {
	{ OJ6SH_CRC0, 0xaf },
	{ OJ6SH_CRC1, 0x4e },
	{ OJ6SH_CRC2, 0x31 },
	{ OJ6SH_CRC3, 0x22 },
};

struct oj6sh_dev {
	int                        irq_init;
	int                        wheel_mode;
	int                        wheel_wait;
	int                        wheel_move;
	int                        wheel_disable;
	int                        left_clk;
	int                        right_clk;
	spinlock_t                 mode_lock;
	struct work_struct         init_work;
	struct work_struct         event_work;
	struct spi_device          *spi;
	struct input_dev           *input;
	struct proc_dir_entry      *proc;
	struct oj6sh_platform_data *data;
	struct class               *class;
	struct cdev                cdev;
	struct timer_list          timer;
	int                        major;
};
static struct oj6sh_dev *oj6sh_device;

static const char oj6sh_spi_name[]   = "oj6sh_spi";
static const char oj6sh_input_name[] = "oj6sh";
static const char oj6sh_irq_name[]   = "oj6sh_irq";
static const char oj6sh_cdev_name[]  = "oj6sh";
#ifdef CONFIG_PROC_FS
static const char oj6sh_proc_name[]  = "oj6sh_wheel";
#endif

#define OJ6SH_DEF_WHEEL_MOVE 1
#define OJ6SH_DEF_WHEEL_WAIT 10

/**
 *  spi_rw cannot be used by interrupt handler and probe function.
 */
static inline int spi_rw(struct spi_device *spi, void *buf, size_t len)
{
	struct spi_message  m;
	struct spi_transfer t = {
		.tx_buf      = (const void *)buf,
		.rx_buf      = buf,
		.len         = len,
		.cs_change   = 0,
		.delay_usecs = 0,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	if (spi_sync(spi, &m) || m.status) {
		return -EIO;
	}

	return (len - m.actual_length);
}

/**
 *  Spi operation frame is 16 bit.
 *    bit 15        0:read 1:write
 *    bit 8 to 14   register index
 *    bit 0 to 7    read/write value
 */
static int oj6sh_reg_read(struct spi_device *spi, enum oj6sh_reg reg_num,
                          u8 *reg_val)
{
	int ret   = 0;
	u16 frame = 0;

	frame |= reg_num << 8;
	ret = spi_rw(spi, &frame, 1);
	*reg_val = (u8)(frame & 0xff);
	return ret;
}

static int oj6sh_reg_write(struct spi_device *spi, enum oj6sh_reg reg_num,
                           u8 reg_val)
{
	u16 frame = 0;

	frame |= 0x8000;
	frame |= reg_num << 8;
	frame |= reg_val;
	return spi_rw(spi, &frame, 1);
}

static int oj6sh_spi_setup(struct spi_device *spi)
{
	spi->bits_per_word = 16;

	return spi_setup(spi);
}

static void oj6sh_wheel_timer(unsigned long data)
{
	struct oj6sh_dev  *dev = (struct oj6sh_dev *)data;

	dev->wheel_disable = 0;
}

static void oj6sh_wheel_event(struct oj6sh_dev *dev, s8 x, s8 y)
{
	int           wait;
	int           move;
	unsigned long flags;

	spin_lock_irqsave(&dev->mode_lock, flags);
	wait = dev->wheel_wait;
	move = dev->wheel_move;
	spin_unlock_irqrestore(&dev->mode_lock, flags);

	/* It decreases it because there are a lot of wheel events. */
	if (dev->wheel_disable) {
		return;
	}

	if (abs(abs(x) - abs(y)) < 5) {
		return;
	}

	if (abs(x) > abs(y)) {
		if (move) {
			if (x < 0) {
				move = -move;
			}
		} else {
			move = x;
		}
		input_report_rel(dev->input, REL_HWHEEL, move);
	} else {
		if (move) {
			if (y > 0) {
				move = -move;
			}
		} else {
			move = -y;
		}
		input_report_rel(dev->input, REL_WHEEL, move);
	}

	input_sync(dev->input);

	if (wait) {
		dev->wheel_disable = 1;
		dev->timer.expires = jiffies + wait * (HZ / 100);
		add_timer(&dev->timer);
	}
}


static void oj6sh_event(struct work_struct *work)
{
	u8               mot;
	u8               x;
	u8               y;
	u8               squal;
	u8               shut_temp;
	u16              shutter;
	int              wheel;
	unsigned long    flags;
	struct oj6sh_dev *dev = container_of(work, struct oj6sh_dev,
	                                     event_work);

	/**
	 * OJ6SH_MOTION
	 *  bit 7 : MOT
	 */
	oj6sh_reg_read(dev->spi, OJ6SH_MOTION,  &mot);
	if (!(mot & 0x80)) {
		goto ignore;
	}

	/**
	 * Correction of the direction
	 *   DELTA_X : top and bottom
	 *   DELTA_Y : right and left
	 */
	oj6sh_reg_read(dev->spi, OJ6SH_DELTA_X, &y);
	oj6sh_reg_read(dev->spi, OJ6SH_DELTA_Y, &x);
	oj6sh_reg_read(dev->spi, OJ6SH_SQUAL,      &squal);
	oj6sh_reg_read(dev->spi, OJ6SH_SHUT_UPPER, &shut_temp);
	shutter = shut_temp << 8;
	oj6sh_reg_read(dev->spi, OJ6SH_SHUT_LOWER, &shut_temp);
	shutter |= shut_temp;

	/* for jitter cancel */
	if ((x == 0 && y == 0) || (squal <= 25 && shutter >= 600)) {
		goto ignore;
	}

	spin_lock_irqsave(&dev->mode_lock, flags);
	wheel = dev->wheel_mode;
	spin_unlock_irqrestore(&dev->mode_lock, flags);
	if (wheel) {
		oj6sh_wheel_event(dev, (s8)-x, (s8)-y);
	} else {
		input_report_rel(dev->input, REL_X, (s8)-x);
		input_report_rel(dev->input, REL_Y, (s8)-y);
	input_sync(dev->input);
	}
ignore:
	enable_irq(dev->spi->irq);
}

static irqreturn_t oj6sh_irq(int irq, void *dev_id)
{
	struct spi_device *spi = dev_id;
	struct oj6sh_dev  *dev = spi_get_drvdata(spi);

	if (irq == dev->data->left_btn_irq) {
		dev->left_clk = (dev->left_clk) ? 0 : 1;
		input_report_key(dev->input, BTN_LEFT, dev->left_clk);
		if (dev->left_clk) {
			set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
		} else {
			set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
		}
	}

	if (irq == dev->data->right_btn_irq) {
		dev->right_clk = (dev->right_clk) ? 0 : 1;
		input_report_key(dev->input, BTN_RIGHT, dev->right_clk);
		if (dev->right_clk) {
			set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
		} else {
			set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
		}
	}

	if (irq == spi->irq) {
		schedule_work(&dev->event_work);
		disable_irq(irq);
	}

	return IRQ_HANDLED;
}


static void oj6sh_setup_irq(struct oj6sh_dev *dev)
{
	const struct irq_setup_data {
		int irq;
		int irq_type;
	} irq_def[] = {
		{ dev->spi->irq,            IRQ_TYPE_LEVEL_LOW },
		{ dev->data->left_btn_irq,  IRQ_TYPE_LEVEL_LOW },
		{ dev->data->right_btn_irq, IRQ_TYPE_LEVEL_LOW },
	};

	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(irq_def); i++) {
		set_irq_type(irq_def[i].irq, irq_def[i].irq_type);
		ret = request_irq(irq_def[i].irq, oj6sh_irq, IRQF_DISABLED,
		                  oj6sh_irq_name, dev->spi);
		if (ret) {
			printk(KERN_ERR "%s : request_irq failed. irq no%d\n",
			       __FUNCTION__, dev->spi->irq);
			goto err;
		}
	}
	dev->irq_init = 1;
	return;
err:
	for (i = i - 1; i >= 0; i--) {
		free_irq(irq_def[i].irq, dev->spi);
	}
}

static void oj6sh_free_irq(struct oj6sh_dev *dev)
{
	int irq_def[] = {
		dev->spi->irq,
		dev->data->left_btn_irq,
		dev->data->right_btn_irq,
	};
	int i;

	if (!dev->irq_init) {
		return;
	}
	for (i = 0; i < ARRAY_SIZE(irq_def); i++) {
		free_irq(irq_def[i], dev->spi);
	}
	dev->irq_init = 0;
}


/* timeout : 10msec */
static void oj6sh_sleep(int timeout)
{
	int hz = timeout * (HZ / 100);

	do {
		hz = schedule_timeout(hz);
	} while (hz);
}

static int oj6sh_self_test(struct spi_device *spi, char buf[OJ6SH_BUF_SIZE])
{
	struct oj6sh_dev  *dev = spi_get_drvdata(spi);
	int               i;
	u8                crc;
	int               ret = 0;
	u8                *b  = (u8 *)buf;

	oj6sh_free_irq(dev);
	flush_scheduled_work();

	/* reset */
	oj6sh_reg_write(dev->spi, OJ6SH_POWER_UP_RESET, 0x5a);
	oj6sh_sleep(100);

	/* self test start */
	oj6sh_reg_write(spi, OJ6SH_SELF_TEST, 0x01);

	/* the test takes 250ms */
	oj6sh_sleep(25);

	for (i = 0; i < ARRAY_SIZE(oj6sh_test_crc); i++) {
		oj6sh_reg_read(spi, oj6sh_test_crc[i].reg, &crc);
		*b++ = crc;
		if (oj6sh_test_crc[i].value != crc) {
			ret = 1;
		}
	}

	schedule_work(&dev->init_work);

	return ret;
}

static int oj6sh_pixel_grab(struct spi_device *spi, char buf[OJ6SH_BUF_SIZE])
{
	struct oj6sh_dev  *dev = spi_get_drvdata(spi);
	int               ret  = 0;
	int               cont = 1;
	u8                mot;
	u8                grab;
	u8                *b   = (u8 *)buf;

	oj6sh_free_irq(dev);
	flush_scheduled_work();

	/* reset */
	oj6sh_reg_write(dev->spi, OJ6SH_POWER_UP_RESET, 0x5a);
	oj6sh_sleep(100);

	/* pixel grab start */
	oj6sh_reg_write(spi, OJ6SH_PIX_GRAB, 0x0);
	oj6sh_sleep(10);

	do {
		/**
		 * OJ6SH_MOTION register
		 * bit 6 : PIXRDY
		 *     5 : PIXFIRST
		 */
		oj6sh_reg_read(spi, OJ6SH_MOTION, &mot);
		if (!(mot & 0x40)) {
			ret  = 1;
			cont = 0;
		}
		if (mot & 0x20) {
			*b   = grab;
			ret  = 0;
			cont = 0;
		}
		oj6sh_reg_read(spi, OJ6SH_PIX_GRAB, &grab);
		oj6sh_sleep(10);
	} while (cont);

	schedule_work(&dev->init_work);

	return ret;
}

static void oj6sh_power_up(struct work_struct *work)
{
	int               ret;
	u8                reg;
	u8                product;
	u8                revision;
	u8                inv_product;
	u8                inv_revision;
	struct oj6sh_dev  *dev = container_of(work, struct oj6sh_dev,
	                                     init_work);

	oj6sh_sleep(10);	/* delay after hard-reset */

	/* All settings will revert to default values */
	ret = oj6sh_reg_write(dev->spi, OJ6SH_POWER_UP_RESET, 0x5a);
	if (ret) {
		printk("oj6sh_power_up: RESET error %d\n", ret);
		return;
	}
	oj6sh_sleep(1);

	/* Sets resolution 800 */
	oj6sh_reg_write(dev->spi, OJ6SH_CFG, 0x80);

	/* check product id and revision id */
	oj6sh_reg_read(dev->spi, OJ6SH_PRODUCT,      &product);
	oj6sh_reg_read(dev->spi, OJ6SH_REVISION,     &revision);
	oj6sh_reg_read(dev->spi, OJ6SH_INV_PRODUCT,  &inv_product);
	oj6sh_reg_read(dev->spi, OJ6SH_INV_REVISION, &inv_revision);
	if ((product != (~inv_product & 0xff)) ||
	    (revision != (~inv_revision & 0xff))) {
		printk("oj6sh_power_up: unmatch product(%x:%x) revision(%x:%x)\n",
		       product, ~inv_product & 0xff, revision, ~inv_revision & 0xff);
		return;
	}

	/* oj6sh register clear */
	oj6sh_reg_read(dev->spi, OJ6SH_MOTION,  &reg);
	oj6sh_reg_read(dev->spi, OJ6SH_DELTA_X, &reg);
	oj6sh_reg_read(dev->spi, OJ6SH_DELTA_Y, &reg);

	oj6sh_setup_irq(dev);
	printk("oj6sh_power_up: product:%02x revision:%02x\n", product, revision);
}

#ifdef CONFIG_PROC_FS
static int oj6sh_proc_read(char *buffer, char **start, off_t offset,
                           int count, int *eof, void *data)
{
	struct oj6sh_dev *dev = (struct oj6sh_dev *)data;
	char             *p   = buffer;
	int              len;
	unsigned long    flags;

	spin_lock_irqsave(&dev->mode_lock, flags);
	p += sprintf(p, "%d %d %d\n", dev->wheel_mode,
	             dev->wheel_move, dev->wheel_wait);
	spin_unlock_irqrestore(&dev->mode_lock, flags);

	len = (p - buffer) - offset;

	if (len < 0) {
		len = 0;
	}

	*start = buffer + offset;

	return len;
}

static int oj6sh_next_str(char **p, char *ep)
{
	*p = ep;
	while ((**p != '\0') && (!isdigit(**p))) {
		(*p)++;
	}
	if (**p == '\0') {
		return -1;
	}
	return 0;
}

static int oj6sh_proc_write(struct file *file, const char __user *buffer,
                            unsigned long count, void *data)
{
	struct oj6sh_dev *dev = (struct oj6sh_dev *)data;
	char             *p   = (char *)buffer;
	char             *ep;
	int              mode;
	int              wait = -1;
	int              move = -1;
	unsigned long    flags;

	mode = simple_strtol(p, &ep, 10) ? 1 : 0;
	if (oj6sh_next_str(&p, ep)) {
		goto end;
	}

	move = simple_strtol(p, &ep, 10);
	if (oj6sh_next_str(&p, ep)) {
		goto end;
	}

	wait = simple_strtol(p, &ep, 10);
end:
	spin_lock_irqsave(&dev->mode_lock, flags);
	dev->wheel_mode = mode;
	if (wait >= 0) {
		dev->wheel_wait = wait;
	}
	if (move >= 0) {
		dev->wheel_move = move;
	}
	spin_unlock_irqrestore(&dev->mode_lock, flags);

	return count;
}

static void oj6sh_create_proc_entry(struct oj6sh_dev *dev)
{
	dev->proc = create_proc_entry(oj6sh_proc_name,
	                              S_IRUGO | S_IWUGO, NULL);
	if (!dev->proc) {
		return;
	}
//	dev->proc->owner      = THIS_MODULE;
	dev->proc->data       = dev;
	dev->proc->read_proc  = oj6sh_proc_read;
	dev->proc->write_proc = oj6sh_proc_write;
}

static void oj6sh_remove_proc_entry(struct oj6sh_dev *dev)
{
	remove_proc_entry(oj6sh_proc_name, NULL);
}
#else
static void oj6sh_create_proc_entry(struct oj6sh_dev *dev)
{
}
static void oj6sh_remove_proc_entry(struct oj6sh_dev *dev)
{
}
#endif

static void oj6sh_wheel_change(int id, int value, void *arg)
{
	struct oj6sh_dev *dev = arg;
	unsigned long    flags;

	/* press */
	if (value == 1) {
		spin_lock_irqsave(&dev->mode_lock, flags);
		dev->wheel_mode = dev->wheel_mode ? 0 : 1;
		spin_unlock_irqrestore(&dev->mode_lock, flags);
	}
}

static int oj6sh_create_inputdevice(struct oj6sh_dev *dev)
{
	int ret = 0;

	dev->input = input_allocate_device();
	if (!dev->input) {
		ret = -ENOMEM;
		goto err;
	}

	dev->input->name = oj6sh_input_name;
	input_set_capability(dev->input, EV_REL, REL_X);
	input_set_capability(dev->input, EV_REL, REL_Y);
	input_set_capability(dev->input, EV_REL, REL_HWHEEL);
	input_set_capability(dev->input, EV_REL, REL_WHEEL);
	input_set_capability(dev->input, EV_KEY, BTN_LEFT);
	input_set_capability(dev->input, EV_KEY, BTN_RIGHT);

	ret = input_register_device(dev->input);
	if (ret) {
		goto err;
	}

err:
	if (ret) {
		input_free_device(dev->input);
		dev->input = NULL;
	}

	return ret;
}

static void oj6sh_remove_inputdevice(struct oj6sh_dev *dev)
{
	input_unregister_device(dev->input);
	input_free_device(dev->input);
}

static int oj6sh_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int oj6sh_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int oj6sh_ioctl(struct inode *inode, struct file *filp,
                       unsigned int cmd, unsigned long arg)
{
	int  ret;
	int  tmp;
	char buf[OJ6SH_BUF_SIZE] = {0};
	char *usr_buf            = (char *)arg;

	switch (cmd) {
	case IO_OJ6SH_SELFTEST:
		ret = oj6sh_self_test(oj6sh_device->spi, buf);
		break;
	case IO_OJ6SH_PIXELGRAB:
		ret = oj6sh_pixel_grab(oj6sh_device->spi, buf);
		break;
	default:
		ret = -EBADRQC;
		break;
	}

	if (usr_buf) {
		tmp = copy_to_user(usr_buf, buf, sizeof(buf));
		if (tmp) {
			ret = tmp;
		}
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long oj6sh_compat_ioctl(struct file *filp, unsigned int cmd,
                               unsigned long arg)
{
	return (long)oj6sh_ioctl((struct inode *)0, filp, cmd, arg);
}
#endif

static struct file_operations oj6sh_char_ops = {
//	.owner   = THIS_MODULE,
	.open    = oj6sh_open,
	.release = oj6sh_release,
	.ioctl   = oj6sh_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = oj6sh_compat_ioctl,
#endif
};

static int oj6sh_register_cdev(struct oj6sh_dev *dev)
{
	int                 ret;
	int                 major_num;
	dev_t               device;
	struct cdev         *cdev;
	struct class        *class;
	struct device       *class_device;

	ret = alloc_chrdev_region(&device, 0, 1, oj6sh_cdev_name);
	major_num = MAJOR(device);
	if (ret) {
		goto exit;
	}

	cdev = &dev->cdev;
	cdev_init(cdev, &oj6sh_char_ops);
//	cdev->owner = THIS_MODULE;

	ret = cdev_add(cdev, device, 1);
	if (ret) {
		goto exit_unregister_chrdev_region;
	}

	class = class_create(THIS_MODULE, oj6sh_cdev_name);
	if (IS_ERR(class)) {
		ret = PTR_ERR(class);
		goto exit_cdev_del;
	}

	class_device = device_create(class, NULL, device, NULL,
	                             oj6sh_cdev_name);
	if (IS_ERR(class_device)) {
		ret = PTR_ERR(class_device);
		goto exit_class_destroy;
	}
	dev->class = class;
	dev->major = major_num;

	return 0;

exit_class_destroy:
	class_destroy(class);
exit_cdev_del:
	cdev_del(cdev);
exit_unregister_chrdev_region:
	unregister_chrdev_region(device, 1);
exit:
	return ret;
}

static void oj6sh_unregister_cdev(struct oj6sh_dev *dev)
{
	int          major_num = dev->major;
	struct class *class    = dev->class;
	struct cdev  *cdev     = &dev->cdev;

	if (class) {
		device_destroy(class, MKDEV(major_num, 0));
		class_destroy(class);
		cdev_del(cdev);
		unregister_chrdev_region(MKDEV(major_num, 0), 1);
	}
	dev->class = NULL;
}

static int __devinit oj6sh_probe(struct spi_device *spi)
{
	struct oj6sh_dev *dev = NULL;
	int               ret = 0;

	ret = oj6sh_spi_setup(spi);
	if (ret) {
		goto err;
	}

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		printk(KERN_ERR "%s : kzalloc failed.\n", __FUNCTION__);
		ret = -ENOMEM;
		goto err;
	}
	dev->spi  = spi;
	dev->data = (struct oj6sh_platform_data *)spi->dev.platform_data;
	dev->wheel_wait = OJ6SH_DEF_WHEEL_WAIT;
	dev->wheel_move = OJ6SH_DEF_WHEEL_MOVE;
	spi_set_drvdata(spi, dev);
	oj6sh_device = dev;

	spin_lock_init(&dev->mode_lock);

	ret = fnkey_register(FNKEY_ID_TRACK_BALL, oj6sh_wheel_change, dev);
	if (ret) {
		printk(KERN_ERR "%s : Cannot register fnkey\n", __FUNCTION__);
		goto err;
	}

	ret = oj6sh_create_inputdevice(dev);
	if (ret) {
		printk(KERN_ERR "%s : Cannot create inputdevice\n",
		       __FUNCTION__);
		goto err;
	}

	ret = oj6sh_register_cdev(dev);
	if (ret) {
		printk(KERN_ERR "%s : Cannot register cdev\n", __FUNCTION__);
		goto err;
	}

	oj6sh_create_proc_entry(dev);

	INIT_WORK(&dev->init_work, oj6sh_power_up);
	INIT_WORK(&dev->event_work, oj6sh_event);
	schedule_work(&dev->init_work);
	init_timer(&dev->timer);
	dev->timer.function = oj6sh_wheel_timer;
	dev->timer.data     = (unsigned long)dev;

//	printk(KERN_INFO "%s Device %s probed\n", __FUNCTION__,
//	       spi->dev.dev_name(dev));
err:
	if (ret) {
		if (dev) {
			if (dev->input) {
				oj6sh_remove_inputdevice(dev);
			}
			if (dev->proc) {
				oj6sh_remove_proc_entry(dev);
			}
			if (dev->class) {
				oj6sh_unregister_cdev(dev);
			}
			kfree(dev);
			spi_set_drvdata(spi, NULL);
		}
	}

	return ret;
}

static int __devexit oj6sh_remove(struct spi_device *spi)
{
	struct oj6sh_dev *dev = spi_get_drvdata(spi);

	oj6sh_free_irq(dev);
	flush_scheduled_work();
	del_timer(&dev->timer);
	fnkey_unregister(FNKEY_ID_TRACK_BALL);
	oj6sh_unregister_cdev(dev);
	oj6sh_remove_inputdevice(dev);
	oj6sh_remove_proc_entry(dev);
	kfree(dev);
	spi_set_drvdata(spi, NULL);

	return 0;
}

static int oj6sh_suspend(struct spi_device *spi, pm_message_t mesg)
{
	struct oj6sh_dev *dev = spi_get_drvdata(spi);

	oj6sh_free_irq(dev);
	dev->data->shutdown(1);
	return 0;
}

static int oj6sh_resume(struct spi_device *spi)
{
	struct oj6sh_dev *dev = spi_get_drvdata(spi);

	dev->data->shutdown(0);
	schedule_work(&dev->init_work);
	return 0;
}

static struct spi_driver oj6sh_spi_driver = {
	.driver = {
		.name  = oj6sh_spi_name,
		.bus   = &spi_bus_type,
//		.owner = THIS_MODULE,
	},
	.probe   = oj6sh_probe,
	.remove  = oj6sh_remove,
	.suspend = oj6sh_suspend,
	.resume  = oj6sh_resume,
};

static int __init oj6sh_init(void)
{
	return spi_register_driver(&oj6sh_spi_driver);
}

static void __exit oj6sh_exit(void)
{
	spi_unregister_driver(&oj6sh_spi_driver);
}

module_init(oj6sh_init);
module_exit(oj6sh_exit);

MODULE_DESCRIPTION("Optical Joystick driver for OJ6SH-T25 devices");
MODULE_AUTHOR("Nissin Systems Co.,Ltd");
MODULE_LICENSE("GPL");
