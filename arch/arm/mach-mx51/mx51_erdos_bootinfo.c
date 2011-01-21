/**
 * @file  arch/arm/mach-mx51/mx51_erdos_bootinfo.c
 *
 * @brief boot information
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
 * 2009/07/06 : version support.
 * 2009/07/21 : /proc/sw support.
 * 2009/07/22 : AC/BATT add.
 * 2009/07/27 : is_board_qa0() add.
 * 2009/07/29 : 0.011
 * 2009/08/01 : 0.013, 0.014
 * 2009/08/03 : 0.015
 * 2009/09/09 : 0.107 -> 0.108 -> 1.000
 * 2009/09/17 : 1.001
 * 2009/09/24 : 1.002
 * 2010/11/05 : ported to 2.6.31 by Andrey Zhornyak darion@gmail.com
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>

#define SPIDER_KERNEL_VERSION    "1.220"
#define SPIDER_PROC_NAME_BOOT    "ver_boot"
#define SPIDER_PROC_NAME_KERNEL  "ver_kernel"
#define SPIDER_PROC_NAME_SW      "sw"

/*
 * bootsw
 *  b31 -
 *  b30 -
 *  ...
 *  b03 - Board QA0    (1:QA0)
 *  b02 - have LAN-PHY (TG-board)
 *  b01 - LEFT-SW  at boot (1:ON)
 *  b00 - RIGHT-SW at boot (1:ON)
 */
static unsigned int bootsw;

/*
 * is_fec - check have CAP1014
 */
int is_fec (void)
{
	if ( bootsw & 0x0004 ) {
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(is_fec);

/*
 * is_cap1014 - check have CAP1014
 */
int is_cap1014 (void)
{
	if ( bootsw & 0x0004 ) {
		return 0;
	}
	return 1;
}
EXPORT_SYMBOL(is_cap1014);

/*
 * is_oj6sh - check have OJ6SH
 */
int is_oj6sh (void)
{
	if ( bootsw & 0x0004 ) {
		return 0;
	}
	return 1;
}
EXPORT_SYMBOL(is_oj6sh);

/*
 * is_board_qa0 - check board type QA0/TG
 */
int is_board_qa0 (void)
{
	if ( bootsw & 0x0008 ) {
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(is_board_qa0);

/*
 * bootsw - parse cmdline for bootsw=0xXXXX
 */
static int __init get_bootsw (char *str)
{
	if (str) {
		sscanf (&(str [1]), "%x", &bootsw);
	}
	return 1;
}
__setup("bootsw", get_bootsw);

/*
 * bootVer - parse cmdline for bootVer=XXXX
 */
static char spider_ver_boot   [16];
static char spider_ver_kernel [16];
static int bootVer_setup(char *p)
{
        memset (spider_ver_boot, 0, sizeof(spider_ver_boot));
        strcpy (spider_ver_boot, p);

        memset (spider_ver_kernel, 0, sizeof(spider_ver_kernel));
        strcpy (spider_ver_kernel, SPIDER_KERNEL_VERSION);

        return 0;
}
__setup("bootVer=", bootVer_setup);

#ifdef CONFIG_PROC_FS
/*
 * spider_ver_proc_read - show version
 */
static struct proc_dir_entry *proc_spider_ver_boot = NULL;
static struct proc_dir_entry *proc_spider_ver_kernel = NULL;
static int spider_ver_proc_read(char *buffer, char **start, off_t offset,
				int count, int *eof, void *data)
{
	int   len;
	char *p = buffer;

	p += sprintf(p, "%s\n", (char *) data);

	len = (p - buffer) -offset;
	if (len < 0) {
		len = 0;
	}
	*start = buffer + offset;

	return len;
}

/*
 * spider_proc_sw_read - sw information
 */
static struct proc_dir_entry *proc_spider_sw = NULL;
static int spider_proc_sw_read(char *buffer, char **start, off_t offset,
			       int count, int *eof, void *data)
{
	int            len, i;
	unsigned short val = 0;
	char          *p = buffer;
	extern int get_gpio_sw_status (void);

	/*
	 *  b15 - 
	 *  b14 - 
	 *  b13 - 
	 *  b12 - 
	 *  b11 - 
	 *  b10 - 
	 *  b09 - LEFT-SW  at boot (1:ON)
	 *  b08 - RIGHT-SW at boot (1:ON)
	 *  b07 - 
	 *  b06 - BATT COMP (1:COMPLETE)
	 *  b05 - BATT CHARG(1:CHARGE)
	 *  b04 - AC adapter(1:CONNECT)
	 *  b03 - HeadPhone (1:CONNECT)
	 *  b02 - COVER-SW  (1:OPEN)
	 *  b01 - LEFT-SW   (1:ON)
	 *  b00 - RIGHT-SW  (1:ON)
	 */
	val  = (bootsw & 3) << 8;	/* boot */
	val |= get_gpio_sw_status ();	/* current sw */

	for ( i = 0 ; i < 16 ; i++ ) {
		p += sprintf (p, "%d ", (val >> (15 - i)) & 1);
	}
	p  += sprintf (p, "\n");
	len = (p - buffer) - offset;
	if (len < 0) {
		len = 0;
	}
	*start = buffer + offset;

	return len;
}

/*
 * spider_ver_setup_proc_entry - show version initialize
 */
static int spider_ver_setup_proc_entry(void)
{
	proc_spider_ver_boot = create_proc_entry(SPIDER_PROC_NAME_BOOT, S_IRUGO, NULL);
	if (proc_spider_ver_boot == NULL) {
		printk(KERN_ERR "Cannot init proc entry for spider_ver_boot\n");
		return (-1);
	}
//	proc_spider_ver_boot->owner     = THIS_MODULE;
	proc_spider_ver_boot->data      = (void *) spider_ver_boot;
	proc_spider_ver_boot->read_proc = spider_ver_proc_read;

	proc_spider_ver_kernel = create_proc_entry(SPIDER_PROC_NAME_KERNEL, S_IRUGO, NULL);
	if (proc_spider_ver_kernel == NULL) {
		printk(KERN_ERR "Cannot init proc entry for spider_ver_kernel\n");
		return (-1);
	}
//	proc_spider_ver_kernel->owner     = THIS_MODULE;
	proc_spider_ver_kernel->data      = (void *) spider_ver_kernel;
	proc_spider_ver_kernel->read_proc = spider_ver_proc_read;

	proc_spider_sw = create_proc_entry(SPIDER_PROC_NAME_SW, S_IRUGO, NULL);
	if (proc_spider_sw == NULL) {
		printk(KERN_ERR "Cannot init proc entry for spider_sw\n");
		return (-1);
	}
//	proc_spider_sw->owner     = THIS_MODULE;
	proc_spider_sw->read_proc = spider_proc_sw_read;

	return 0;
}
late_initcall(spider_ver_setup_proc_entry);
#endif /* CONFIG_PROC_FS */

static int __init bootinfo_init(void)
{
	return 0;
}

static void __exit bootinfo_exit(void)
{
#ifdef CONFIG_PROC_FS
	if (proc_spider_ver_boot != 0) {
		remove_proc_entry (SPIDER_PROC_NAME_BOOT, proc_spider_ver_boot);
	}
	if (proc_spider_ver_kernel != 0) {
		remove_proc_entry (SPIDER_PROC_NAME_KERNEL, proc_spider_ver_kernel);
	}
	if (proc_spider_sw != 0) {
		remove_proc_entry (SPIDER_PROC_NAME_SW, proc_spider_sw);
	}
#endif /* CONFIG_PROC_FS */
}

module_init(bootinfo_init);
module_exit(bootinfo_exit);

MODULE_DESCRIPTION("boot information");
MODULE_AUTHOR("Nissin Systems Co.,Ltd");
MODULE_LICENSE("GPL");
