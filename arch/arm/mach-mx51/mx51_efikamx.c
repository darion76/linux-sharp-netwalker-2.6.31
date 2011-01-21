/*
 * Copyright 2009 Pegatron Corporation. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/flash.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pci_ids.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/spba.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/keypad.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include "board-mx51_efikamx.h"
#include "iomux.h"
#include "crm_regs.h"

#include <linux/syscalls.h>
#include <linux/reboot.h>
#include "mx51_efikamx.h"

extern int board_id[2];

int sii9022_reinit(struct fb_var_screeninfo *var);
int cs8556_reinit(struct fb_var_screeninfo *var);

#define VIDEO_MODE_HDMI_DEF	4
#define VIDEO_MODE_VGA_DEF	2

#define MEGA              1000000

int __initdata video_output = { VIDEO_OUT_STATIC_AUTO };
int __initdata video_mode = { VIDEO_OUT_STATIC_AUTO };
int __initdata hdmi_audio = { 1 };
int __initdata enable_hdmi_spdif = { 0 }; // spdif doesn't work
int __initdata clock_auto = { 1 };
char __initdata vmode[32] = { 0 };
int __initdata mxc_debug = { 1 };
int __initdata extsync = { 1 };
int __initdata sink_dvi = { 0 };
int __initdata sink_monitor = { 0 };
int __initdata pixclk_limit = { KHZ2PICOS(133200) };
int __initdata video_max_res = { 1 };
u8 edid[256];

struct fb_videomode __initdata mxcfb_preferred;
EXPORT_SYMBOL(hdmi_audio);
EXPORT_SYMBOL(enable_hdmi_spdif);
EXPORT_SYMBOL(clock_auto);
EXPORT_SYMBOL(vmode);
EXPORT_SYMBOL(mxc_debug);
EXPORT_SYMBOL(extsync);
EXPORT_SYMBOL(video_mode);


/*!
 * @file mach-mx51/mx51_efikamx.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX51
 */
int (*sii9022_func)( const char *procfs_buffer );
extern int sii9022_reinit(struct fb_var_screeninfo *var);
extern void __init mx51_efikamx_io_init(void);
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);
static int num_cpu_wp = 3;
int mxc_fb_initialized = 0;

/* working point(wp): 0 - 800MHz; 1 - 166.25MHz; */
static struct cpu_wp cpu_wp_auto[] = {
	{
	 .pll_rate = 1000000000,
	 .cpu_rate = 1000000000,
	 .pdf = 0,
	 .mfi = 10,
	 .mfd = 11,
	 .mfn = 5,
	 .cpu_podf = 0,
	 .cpu_voltage = 1175000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1100000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 166250000,
	 .pdf = 4,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 4,
	 .cpu_voltage = 1000000,},
};

struct cpu_wp *mx51_efikamx_get_cpu_wp(int *wp)
{
	*wp = num_cpu_wp;
	return cpu_wp_auto;
}

void mx51_efikamx_set_num_cpu_wp(int num)
{
	num_cpu_wp = num;
	return;
}
static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
static struct resource mxcfb_resources[] = {
	[0] = {
	       .flags = IORESOURCE_MEM,
	       },
};

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB24,
	 .mode_str = "1280x720-24@60",	//hdmi
	 },
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB565,
	 .mode_str = "1024x768-16@60", 	//vga
	 },
};

static struct platform_device mxc_fb_device[] = {
	{
	 .name = "mxc_sdc_fb",	//hdmi
	 .id = 0,
	 .dev = {
		 .release = mxc_nop_release,
		 .coherent_dma_mask = 0xFFFFFFFF,
		 .platform_data = &fb_data[0],
		 },
	 .num_resources = ARRAY_SIZE(mxcfb_resources),
	 .resource = mxcfb_resources,
	 },	 
	{
	 .name = "mxc_sdc_fb",	//vga
	 .id = 1,
	 .dev = {
		 .release = mxc_nop_release,
		 .coherent_dma_mask = 0xFFFFFFFF,
		 .platform_data = &fb_data[1],
		 },
	 },
	{
	 .name = "mxc_sdc_fb", // overlay
	 .id = 2,
	 .dev = {
		 .release = mxc_nop_release,
		 .coherent_dma_mask = 0xFFFFFFFF,
		 },
	 },
};

int read_edid2(struct i2c_adapter *adp,
	      char *edid, u16 len, struct fb_var_screeninfo *einfo,
	      int *dvi)
{
	int i;
	u8 buf0[2] = {0, 0};
	int dat = 0;
	u16 addr = 0x50;
	struct i2c_msg msg[2] = {
		{
		.addr	= addr,
		.flags	= 0,
		.len	= 1,
		.buf	= buf0,
		}, {
		.addr	= addr,
		.flags	= I2C_M_RD,
		.len	= 128,
		.buf	= edid,
		},
	};

	printk("%s \n", __func__ );
	
	if (adp == NULL || einfo == NULL)
		return -EINVAL;

	buf0[0] = 0x00;
	memset(edid, 0, len);
	memset(einfo, 0, sizeof(struct fb_var_screeninfo));
	msg[1].len = len;
	dat = i2c_transfer(adp, msg, 2);

	/* If 0x50 fails, try 0x37. */
	if (edid[1] == 0x00) {
		msg[0].addr = msg[1].addr = 0x37;
		dat = i2c_transfer(adp, msg, 2);
	}

	if (edid[1] == 0x00)
		return -ENOENT;

	*dvi = 0;
	if ((edid[20] == 0x80) || (edid[20] == 0x88) || (edid[20] == 0))
		*dvi = 1;

	dat = fb_parse_edid(edid, einfo);
	if (dat)
		return -dat;

	/* This is valid for version 1.3 of the EDID */
	if ((edid[18] == 1) && (edid[19] == 3)) {
		einfo->height = edid[21] * 10;
		einfo->width = edid[22] * 10;
	}

	for(i=0; i< len; i+=32) {
	   int j=0;
	   
	   for(j=0; j<32; j++) {
		  printk("%02x", edid[i+j] );
	   }
	   printk("\n");
	}

	return 0;
}

void fb_dump_modeline( struct fb_videomode *modedb, int num)
{
	int i;
	struct fb_videomode *mode;
	
	for (i = 0; i < num; i++) {

		mode = &modedb[i];

		printk("   \"%dx%d%s%d\" %lu.%02lu ",  
			mode->xres, mode->yres, (mode->vmode & FB_VMODE_INTERLACED) ? "i@" : "@", mode->refresh,
			(PICOS2KHZ(mode->pixclock) * 1000UL)/1000000,
			(PICOS2KHZ(mode->pixclock) ) % 1000);
		printk("%d %d %d %d ", 
			mode->xres, 
			mode->xres + mode->right_margin,
			mode->xres + mode->right_margin + mode->hsync_len, 
			mode->xres + mode->right_margin + mode->hsync_len + mode->left_margin );
		printk("%d %d %d %d ", 
			mode->yres, 
			mode->yres + mode->lower_margin,
			mode->yres + mode->lower_margin + mode->vsync_len, 
			mode->yres + mode->lower_margin + mode->vsync_len + mode->upper_margin );
		printk("%shsync %svsync\n", (mode->sync & FB_SYNC_HOR_HIGH_ACT) ? "+" : "-",
			   (mode->sync & FB_SYNC_VERT_HIGH_ACT) ? "+" : "-" );

	}
	
}

static int fb_dump_mode( const char *func_char, const struct fb_videomode *mode)
{
	if ( mode == NULL )
		return -1;

	printk(KERN_INFO "%s geometry %u %u %u\n", 
	  func_char,  mode->xres, mode->yres, mode->pixclock);
	printk(KERN_INFO "%s timings %u %u %u %u %u %u %u\n",
	  func_char, mode->pixclock, mode->left_margin, mode->right_margin, 
	  mode->upper_margin, mode->lower_margin, mode->hsync_len, mode->vsync_len );
	printk(KERN_INFO "%s flag %u sync %u vmode %u %s\n",
	  func_char, mode->flag, mode->sync, mode->vmode, mode->flag & FB_MODE_IS_FIRST ? "preferred" : "" );

	return 0;
}

 // silence compiler warning because we don't use this right now
static void fb_dump_var( const char *func_char, struct fb_var_screeninfo *var)
{
	if ( var == NULL )	
		return;

	printk(KERN_INFO "%s geometry %u %u %u %u\n", 
	  func_char,  var->xres, var->yres, var->xres_virtual, var->yres_virtual);
	printk(KERN_INFO "%s offset %u %u %u %u %u\n",
	  func_char, var->xoffset, var->yoffset, var->height, var->width, var->bits_per_pixel);
	printk(KERN_INFO "%s timings %u %u %u %u %u %u %u\n",
	  func_char, var->pixclock, var->left_margin, var->right_margin, 
	  var->upper_margin, var->lower_margin, var->hsync_len, var->vsync_len ); 	
	printk(KERN_INFO "%s accel_flags %u sync %u vmode %u\n",
	  func_char, var->accel_flags, var->sync, var->vmode );	
}


void mxcfb_videomode_to_modelist(const struct fb_info *info, const struct fb_videomode *modedb, int num,
			      struct list_head *head)
{
	int i, del = 0;
	INIT_LIST_HEAD(head);

	for (i = 0; i < num; i++) {

		struct list_head *pos, *n;
		struct fb_modelist *modelist;
		
		del = 0;
		
		if ( modedb[i].flag & FB_MODE_IS_FIRST ) {
			printk(KERN_INFO "found preferred video mode %ux%u%s%u pclk=%u\n",
				modedb[i].xres, modedb[i].yres, 
				(modedb[i].vmode & FB_VMODE_INTERLACED ) ? "i@" : "@",
				modedb[i].refresh,
				modedb[i].pixclock );
				
			memcpy((void *) &mxcfb_preferred, (const void *) &modedb[i], sizeof(struct fb_videomode));	
			// and we carry on because the next few lines may delete it
		}
		
		if ( modedb[i].pixclock < pixclk_limit ) {
			printk(KERN_INFO "%ux%u%s%u pclk=%u removed (exceed %u limit)\n",
				modedb[i].xres, modedb[i].yres, 
				(modedb[i].vmode & FB_VMODE_INTERLACED ) ? "i@" : "@",
				modedb[i].refresh,
				modedb[i].pixclock,
				pixclk_limit );
			continue; // next i, practically deleted
		}
		else if ( (modedb[i].vmode & FB_VMODE_INTERLACED) ) {
			printk(KERN_INFO "%ux%u%s%u pclk=%u removed (interlaced)\n",
				modedb[i].xres, modedb[i].yres, 
				(modedb[i].vmode & FB_VMODE_INTERLACED ) ? "i@" : "@",
				modedb[i].refresh,
				modedb[i].pixclock );
			continue; // next i, practically deleted
		}

				/* if candidate is a detail timing, delete existing one in modelist !
				  * note: some TV has 1280x720@60 in standard timings but also has 1280x720 in detail timing block 
				  *         in this case, use detail timing !
				  */
		list_for_each_safe(pos, n, head) {
			modelist = list_entry(pos, struct fb_modelist, list);
			
			if ( res_matches_refresh(modelist->mode,
				modedb[i].xres, modedb[i].yres, modedb[i].refresh) ) {

				if ( ( modedb[i].flag & FB_MODE_IS_DETAILED ) ) {

					printk(KERN_INFO "%ux%u%s%u pclk=%u removed (duplicate)\n",
						modelist->mode.xres, modelist->mode.yres, 
						(modelist->mode.vmode & FB_VMODE_INTERLACED ) ? "i@" : "@",
						modelist->mode.refresh,
						modelist->mode.pixclock );
					list_del(pos);
					kfree(pos);
					del = 1;
				}
			}
		}

		if (del == 0)
			fb_add_videomode(&modedb[i], head);
	}
}

int handle_edid2(struct i2c_adapter *adp, char *buffer, u16 len)
{
	int err = 0;
	int dvi = 0;
	struct fb_var_screeninfo screeninfo;
	
	memset(&screeninfo, 0, sizeof(screeninfo));

	if (cpu_is_mx51_rev(CHIP_REV_3_0) > 0) {
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI2_HSYNC), 1);
		msleep(1);
    }

	err = read_edid2(adp, buffer, len, &screeninfo, &dvi);

	if (cpu_is_mx51_rev(CHIP_REV_3_0) > 0)
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI2_HSYNC), 0);


	if ( err )	
		printk("read_edid error!\n");

	return err;
}


/* this make console & x window refresh rate fixed to 60Hz if video mode set to 1280x720@60Hz */
void clock_update(const char *clk_name_parent, unsigned long parent_rate, 
					const char *clk_name_this, unsigned long divider)
{
	struct clk *clk, *di_clk;
	int ret=0, ret1=0;
	
	clk = clk_get(NULL, clk_name_parent);
	di_clk = clk_get(NULL, clk_name_this);

	clk_disable(clk);
	clk_disable(di_clk);

	printk("orig: parent=%s clk=%lu ", clk_name_parent, clk_get_rate(clk) );
	printk("this=%s di_clk=%lu\n", clk_name_this, clk_get_rate(di_clk));
	ret = clk_set_rate(clk, parent_rate);
	ret1 = clk_set_rate(di_clk, parent_rate / divider);

	printk("adjust: parent=%s clk=%lu ", clk_name_parent, clk_get_rate(clk));
	printk("this=%s di_clk=%lu ret=%d ret=%d\n", clk_name_this, clk_get_rate(di_clk), ret, ret1);
	clk_enable(clk);
	clk_enable(di_clk);

	clk_put(di_clk);
	clk_put(clk);
}

int mxcfb_di_clock_adjust(int video_output, u32 pixel_clock)
{
	char *clk_di = "ipu_di0_clk";
	u32 rate = 0;
	static u32 pixel_clock_last = 0;
	static int video_output_last = 0;
	
	if( clock_auto == 0 )
		return 0;

	/* avoid uncessary clock change to reduce unknown impact chance */
	if ( pixel_clock && (pixel_clock == pixel_clock_last) && video_output == video_output_last )
	{
		printk(KERN_INFO "pclk %u same to previous one, skipping!\n", pixel_clock );
		return 0;
	}
	if ( pixel_clock < 6000 || pixel_clock > 40000 ) { /* 25Mhz~148Mhz */
		printk(KERN_INFO "pclk %u exceed limitation (6000~40000)!\n", pixel_clock );
		return -1;
	}
	pixel_clock_last = pixel_clock;
	video_output_last = video_output;	
	
			if ( pixel_clock == 0 ) {
		printk(KERN_INFO "%s incorrect clock rate (%u), reset to %u\n", 
					__func__, rate, 260*MEGA );
				rate = 260*MEGA;
			}
			else
		rate = (u32) (PICOS2KHZ(pixel_clock) * 1000UL)*1;

			printk("%s pixelclk=%u rate=%u\n", __func__, pixel_clock, rate );
			
	clock_update("pll3", rate, clk_di, 1);

	return 0;
}
	
void mxcfb_adjust(struct fb_var_screeninfo *var )
{
	if( clock_auto )
		mxcfb_di_clock_adjust( video_output, var->pixclock );
	if ( 1 /*(video_output == VIDEO_OUT_STATIC_HDMI) */) {
		sii9022_reinit( var );
	}
}

void mxcfb_update_default_var(struct fb_var_screeninfo *var, 
									struct fb_info *info, 
									const struct fb_videomode *def_mode )
{
	struct fb_monspecs *specs = &info->monspecs;
	const struct fb_videomode *mode = NULL;
//	struct fb_var_screeninfo var_tmp;
	int modeidx = 0;

	printk(KERN_INFO "%s mode_opt=%s vmode=%s\n", __func__ , fb_mode_option, vmode );

	fb_dump_mode("preferred mode", &mxcfb_preferred);

	/* user specified vmode,  ex: support reduce blanking, such as 1280x768MR-16@60 */
	if ( vmode[0] ) {
		/* use edid support modedb or modedb in modedb.c */
		modeidx = fb_find_mode(var, info, vmode, specs->modedb, specs->modedb_len, def_mode, 16);
	}
	else if ( specs->modedb != NULL) {
		mode = fb_find_best_display(specs, &info->modelist);

		if ( mode ) {
			fb_videomode_to_var(var, mode);
			printk(KERN_INFO "best mode is %ux%u@%u pclk=%u\n",
				mode->xres, mode->yres, mode->refresh, mode->pixclock );
			fb_dump_mode("best mode", mode);
		} else {
			mode = fb_find_nearest_mode( def_mode, &info->modelist );
			if ( mode ) {
				fb_videomode_to_var(var, mode);
				printk(KERN_INFO "nearest default (%ux%u@%u pclk=%u) is %ux%u@%u pclk=%u\n",
					def_mode->xres, def_mode->yres, def_mode->refresh, def_mode->pixclock,
					mode->xres, mode->yres, mode->refresh, mode->pixclock );
				fb_dump_mode("nearest default", mode);
			}
		}
	}

	if ( modeidx == 0 && mode == NULL ) { /* no best monitor support mode timing found, use def_video_mode timing ! */
		fb_videomode_to_var(var, def_mode);
		fb_dump_var( __func__, var );
	}
}

void mxc_init_fb(void)
{	
	if ( mxc_fb_initialized )
		return;
	
	mxc_fb_initialized = 1;
	
	printk("*** %s vmode=%s video-mode=%d clock_auto=%d\n", 
		  __func__, vmode, video_mode, clock_auto);

		(void)platform_device_register(&mxc_fb_device[0]);	// HDMI
	//(void)platform_device_register(&mxc_fb_device[1]);	// VGA
	(void)platform_device_register(&mxc_fb_device[2]);		// Overlay for VPU

}


static int __init hdmi_setup(char *__unused)
{
	video_output = VIDEO_OUT_STATIC_HDMI;	
	return 1;
}

static int __init hdmi_spdif_setup(char *__unused)
{
	enable_hdmi_spdif = 1;
	return 1;
}

static int __init video_mode_setup(char *options)
{
	if (!options || !*options)
		return 1;

	video_mode = simple_strtol(options, NULL, 10);
	
	printk("video mode=%d\n", video_mode);

	return 1;
}

static int __init clock_setup(char *options)
{	
	if (!options || !*options)
		return 1;
	
	clock_auto = simple_strtol(options, NULL, 10);
	printk("clock_auto=%d\n", clock_auto);
	
	return 1;
}

static int __init max_res_setup(char *options)
{
	if (!options || !*options)
		return 1;
		
	video_max_res = simple_strtol(options, NULL, 10);
	printk("video_max_res=%d\n", video_max_res);
	
	return 1;
}

static int __init vmode_setup(char *options)
{
	if (!options || !*options)
		return 1;

	memset( vmode, 0, sizeof(vmode));
	strncpy( vmode, options, sizeof(vmode)-1 );
	printk("vmode=%s\n", vmode );
	
	return 1;
}

__setup("hdmi", hdmi_setup);
__setup("spdif", hdmi_spdif_setup);
__setup("video_mode=", video_mode_setup);
__setup("clock_auto=", clock_setup);
__setup("vmode=", vmode_setup);
__setup("max_res=", max_res_setup);



#else
static inline void mxc_init_fb(void)
{
}
#endif

static void dvi_reset(void)
{
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIN), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIN), 0);
	msleep(50);

	/* do reset */
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIN), 1);
	msleep(20);		/* tRES >= 50us */

	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIN), 0);
}

static struct mxc_lcd_platform_data dvi_data = {
	.core_reg = "VGEN1",
	.io_reg = "VGEN3",
	.reset = dvi_reset,
};

static void vga_reset(void)
{
	#if 0
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_A19), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A19), 0);
	msleep(50);
	/* do reset */
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A19), 1);
	msleep(10);		/* tRES >= 50us */
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A19), 0);
	#endif
}

static struct mxc_lcd_platform_data vga_data = {
	.core_reg = "VCAM",
	.io_reg = "VGEN3",
	.analog_reg = "VAUDIO",
	.reset = vga_reset,
};

#if defined(CONFIG_I2C_MXC) || defined(CONFIG_I2C_MXC_MODULE)

#ifdef CONFIG_I2C_MXC_SELECT1
static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
};
#endif
#ifdef CONFIG_I2C_MXC_SELECT2
static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
	{
	 .type = "cs8556",
	 .addr = 0x3d,
	 .platform_data = &vga_data,
	 },
	{
	 .type = "sii9022",
	 .addr = 0x39,
	 .platform_data = &dvi_data,
	 },
};
#endif

#if defined(CONFIG_I2C_MXC_HS) || defined(CONFIG_I2C_MXC_HS_MODULE)
static struct i2c_board_info mxc_i2c_hs_board_info[] __initdata = {
	/*
	{
	 .type = "si4702",
	 .addr = 0x10,
	 .platform_data = (void *)&si4702_data,
	 },
	*/
};
#endif

#endif

#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)

static struct mtd_partition mxc_spi_nor_partitions[] = {
	{
	 .name = "flash",
	 .offset = 0,
	 .size = 0x200000,
	},
};

static struct flash_platform_data mxc_spi_flash_data = {
	.name = "mxc_spi_nor",
	.parts = mxc_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(mxc_spi_nor_partitions),
	.type = "sst25vf032b",
};
#endif

static struct spi_board_info mxc_spi_board_info[] __initdata = {
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
	{
	 .modalias = "mxc_spi_nor",
	 .max_speed_hz = 25000000, /* max spi clock (SCK) speed in HZ */
	 .bus_num = 1,
	 .chip_select = 1,
	 .platform_data = &mxc_spi_flash_data,
	},
#endif
};

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
unsigned int expio_intr_fec;

EXPORT_SYMBOL(expio_intr_fec);
#endif

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
static int sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;

	if (to_platform_device(dev)->id == 0)
		rc = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_1));
	else	//ron: change WP gpio to GPIO1_7
		rc = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_7));
		//rc = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_5));

	return rc;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret;

	if (to_platform_device(dev)->id == 0) {
		// only detects on 1.2, on 1.1 there is no CD pin..
		ret = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_0));
		return ret;
	} else {		/* config the det pin for SDHC2 */
		//ron: SDHC2 CD gpio
		ret = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_8));

		return ret;
	}
}

static struct mxc_mmc_platform_data mmc_data = {
	.ocr_mask = MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 52000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

/*!
 * Resource definition for the SDHC1
 */
static struct resource mxcsdhc1_resources[] = {
	[0] = {
	       .start = MMC_SDHC1_BASE_ADDR,
	       .end = MMC_SDHC1_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_MMC_SDHC1,
	       .end = MXC_INT_MMC_SDHC1,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .start = IOMUX_TO_IRQ(MX51_PIN_GPIO1_0),
	       .end = IOMUX_TO_IRQ(MX51_PIN_GPIO1_0),
	       .flags = IORESOURCE_IRQ,
	       },
};

/*!
 * Resource definition for the SDHC2
 */
static struct resource mxcsdhc2_resources[] = {
	[0] = {
	       .start = MMC_SDHC2_BASE_ADDR,
	       .end = MMC_SDHC2_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_MMC_SDHC2,
	       .end = MXC_INT_MMC_SDHC2,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       /*
		.start = IOMUX_TO_IRQ(MX51_PIN_GPIO1_4),
	       	.end = IOMUX_TO_IRQ(MX51_PIN_GPIO1_4),
		*/
	       .start = IOMUX_TO_IRQ(MX51_PIN_GPIO1_8),
	       .end = IOMUX_TO_IRQ(MX51_PIN_GPIO1_8),
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Device Definition for MXC SDHC1 */
static struct platform_device mxcsdhc1_device = {
	.name = "mxsdhci",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mmc_data,
		},
	.num_resources = ARRAY_SIZE(mxcsdhc1_resources),
	.resource = mxcsdhc1_resources,
};

/*! Device Definition for MXC SDHC2 */
static struct platform_device mxcsdhc2_device = {
	.name = "mxsdhci",
	.id = 1,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mmc_data,
		},
	.num_resources = ARRAY_SIZE(mxcsdhc2_resources),
	.resource = mxcsdhc2_resources,
};

static inline void mxc_init_mmc(void)
{
	(void)platform_device_register(&mxcsdhc1_device);
	(void)platform_device_register(&mxcsdhc2_device);
}
#else
static inline void mxc_init_mmc(void)
{
}
#endif

#if defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000) \
    || defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000_MODULE)
static int mxc_sgtl5000_amp_enable(int enable);

static int headphone_det_status(void)
{
	return gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_RS));
}

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_irq = IOMUX_TO_IRQ(MX51_PIN_DISPB2_SER_RS),
	.hp_status = headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
	.sysclk = 12288000,
};

static struct platform_device mxc_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &sgtl5000_data,
		},
};

static int mxc_sgtl5000_amp_enable(int enable)
{
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A23), enable ? 1 : 0);
	return 0;
}

static void mxc_init_sgtl5000(void)
{
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_A23), "amp_enable");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_A23), 0);

	platform_device_register(&mxc_sgtl5000_device);
}
#else
static inline void mxc_init_sgtl5000(void)
{
}
#endif

#if 0

#if defined(CONFIG_GPIO_BUTTON_MXC) || \
	defined(CONFIG_GPIO_BUTTON_MXC_MODULE)

#define MXC_BUTTON_GPIO_PIN MX51_PIN_EIM_DTACK

static struct mxc_gpio_button_data gpio_button_data = {
	.name = "Power Button (CM)",
	.gpio = MXC_BUTTON_GPIO_PIN,
	.irq = IOMUX_TO_IRQ(MXC_BUTTON_GPIO_PIN),
	.key = KEY_POWER,
};

static struct platform_device gpio_button_device = {
	.name = "gpio_button",
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &gpio_button_data,
		},
};

static inline void mxc_init_gpio_button(void)
{
	gpio_direction_input(IOMUX_TO_GPIO(MXC_BUTTON_GPIO_PIN));
	platform_device_register(&gpio_button_device);
}
#else
static inline void mxc_init_gpio_button(void)
{
}
#endif

#endif
/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	int size = SZ_512M - SZ_32M;
	struct tag *t;

	mxc_cpu_init();

	get_cpu_wp = mx51_efikamx_get_cpu_wp;
	set_num_cpu_wp = mx51_efikamx_set_num_cpu_wp;

	for_each_tag(t, tags) {
		if (t->hdr.tag != ATAG_CMDLINE)
			continue;
		str = t->u.cmdline.cmdline;
		str = strstr(str, "mem=");
		if (str != NULL) {
			str += 4;
			size = memparse(str, &str);
			if (size == 0 || size == SZ_512M)
				return;
		}
	}

	for_each_tag(t, tags) {
		if (t->hdr.tag != ATAG_MEM)
			continue;

		t->u.mem.size = size;
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
		mxcfb_resources[0].start = t->u.mem.start + size;
		mxcfb_resources[0].end = t->u.mem.start + SZ_512M - 1;
#endif
	}
}

int mxc_reboot(void)
{
	/* wdog reset workaround, result power reset! */
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN13), 0);

	return 0;
}

#define PWGT1SPIEN (1<<15)
#define PWGT2SPIEN (1<<16)
#define USEROFFSPI (1<<3)

static void mxc_power_off(void)
{
	/* We can do power down one of two ways:
	   Set the power gating
	   Set USEROFFSPI */

	/* Set the power gate bits to power down */
	pmic_write_reg(REG_POWER_MISC, (PWGT1SPIEN|PWGT2SPIEN),
		(PWGT1SPIEN|PWGT2SPIEN));
	/* pmic_write_reg(REG_POWER_MISC, (PWGT1SPIEN|PWGT2SPIEN), */
	/* 		(PWGT1SPIEN|PWGT2SPIEN)); */
	//robin: CLR_DFF
	mxc_request_iomux(MX51_PIN_CSI2_VSYNC, IOMUX_CONFIG_GPIO);
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_CSI2_VSYNC), "reset_thingy");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSI2_VSYNC), 0);
	msleep(10);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI2_VSYNC), 1);
}

static struct input_dev *pwr_inputdev;

static void do_poweroff(struct work_struct *dummy)
{
	struct task_struct *p;

	for_each_process(p) {
		if (p->mm && !is_global_init(p))
			/* Not swapper, init nor kernel thread */
			force_sig(SIGKILL, p);
	}
	msleep(1000);
	sys_reboot(LINUX_REBOOT_MAGIC1, LINUX_REBOOT_MAGIC2, LINUX_REBOOT_CMD_POWER_OFF, NULL);
}

static DECLARE_WORK(poweroff_work, do_poweroff);

/*!
 * Power Key interrupt handler.
 */
static irqreturn_t power_key_int(int irq, void *dev_id)
{
	int pk_pressed;
	static unsigned int pk_press_count = 0;
	static unsigned long pk_press_time = 0;
	
	pk_pressed = !gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_EIM_DTACK));
	if(pk_pressed) {
		/* accumulate power key press time */
		if ( jiffies - pk_press_time < 1*HZ )
			pk_press_count++;
		else
			pk_press_count = 0;
			
		pk_press_time = jiffies;
		pr_info("Power Key pressed #%u\n", pk_press_count);
			
		/* ron: input_report_key(pk_dev, KEY_POWER, pk_pressed); */
		set_irq_type(irq, IRQF_TRIGGER_RISING); /* ron: detect falling edge */
		input_report_key(pwr_inputdev, KEY_POWER, 1);
		input_sync(pwr_inputdev);
	} else {
		pr_info("PWR Key released\n");
		/* ron: input_report_key(pk_dev, KEY_POWER, pk_released); */
		set_irq_type(irq, IRQF_TRIGGER_FALLING); /* ron: detect falling edge */
		input_report_key(pwr_inputdev, KEY_POWER, 0);
		input_sync(pwr_inputdev);
	}

	/* software shutdown if power keypress >= 4 */
	if ( pk_press_count >= 3 ) {
		schedule_work_on(first_cpu(cpu_online_map), &poweroff_work);
	}
	return 0;
}

/*!
 * Power Key initialization.
 */
static int __init mxc_init_power_key(void)
{
	/* Set power key as wakeup resource */
	int irq, ret;

	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_EIM_DTACK));
	irq = IOMUX_TO_IRQ(MX51_PIN_EIM_DTACK);
	set_irq_type(irq, IRQF_TRIGGER_FALLING); /* ron: detect rising & falling edge */

	pwr_inputdev = input_allocate_device();
	if (!pwr_inputdev) {
		pr_err("Failed to allocate hotkey input device\n");
		return -ENOMEM;
	}
	pwr_inputdev->name = "Genesi Efika MX Buttons";
	pwr_inputdev->phys = "efikamx/input0";
	pwr_inputdev->uniq = "efikamx";
	pwr_inputdev->id.bustype = BUS_HOST;
	pwr_inputdev->id.vendor = PCI_VENDOR_ID_FREESCALE;
	set_bit(KEY_POWER, pwr_inputdev->keybit);
	set_bit(EV_KEY, pwr_inputdev->evbit);
	ret = input_register_device(pwr_inputdev);
	if (ret) {
		input_free_device(pwr_inputdev);
		pr_err("Failed to register hotkey input device\n");
		return -ENODEV;
	}

	ret = request_irq(irq, power_key_int, 0, "power_key", 0);
	if (ret)
		pr_info("register on-off key interrupt failed\n");
	else
		enable_irq_wake(irq);
	return ret;
}

late_initcall(mxc_init_power_key);

extern void gpio_ata_active(void);
extern void gpio_ata_inactive(void);

static int ata_init(struct platform_device *pdev)
{
	/* Configure the pins */
	gpio_ata_active();
	return 0;
}

static void ata_exit(void)
{
	/* Free the pins */
	gpio_ata_inactive();
}

static struct fsl_ata_platform_data ata_data = {
	.udma_mask = ATA_UDMA3,
	.mwdma_mask = ATA_MWDMA2,
	.pio_mask = ATA_PIO4,
	.fifo_alarm = MXC_IDE_DMA_WATERMARK / 2,
	.max_sg = MXC_IDE_DMA_BD_NR,
	.init = ata_init,
	.exit = ata_exit,
	.core_reg = NULL,
	.io_reg = NULL,
};

static struct resource pata_fsl_resources[] = {
	[0] = {
	       .start = ATA_BASE_ADDR,
	       .end = ATA_BASE_ADDR + 0x000000C8,
	       .flags = IORESOURCE_MEM,},
	[2] = {
	       .start = MXC_INT_ATA,
	       .end = MXC_INT_ATA,
	       .flags = IORESOURCE_IRQ,},
};

static struct platform_device pata_fsl_device = {
	.name = "pata_fsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(pata_fsl_resources),
	.resource = pata_fsl_resources,
	.dev = {
		.platform_data = &ata_data,
		.coherent_dma_mask = ~0,},
};

static void __init mxc_init_pata(void)
{
	(void)platform_device_register(&pata_fsl_device);
}

/* proc --begin */

#include <linux/module.h>       /* Specifically, a module */
#include <linux/kernel.h>       /* We're doing kernel work */
#include <linux/proc_fs.h>      /* Necessary because we use the proc fs */
#include <asm/uaccess.h>        /* for copy_from_user */

#define PROCFS_MAX_SIZE         512
#define PROCFS_NAME             "ioctl"

/**
 * This structure hold information about the /proc file
 *
 */
static struct proc_dir_entry *Our_Proc_File;

/**
 * The buffer used to store character for this module
 *
 */
static char procfs_buffer[PROCFS_MAX_SIZE];

/**
 * The size of the buffer
 *
 */
static unsigned long procfs_buffer_size = 0;

/** 
 * This function is called then the /proc file is read
 *
 */
int 
procfile_read(char *buffer,
              char **buffer_location,
              off_t offset, int buffer_length, int *eof, void *data)
{
	int ret;
	int len = 0;
	        
	if (offset > 0) {
	        /* we have finished to read, return 0 */
	        ret  = 0;
	} else {
	        /* fill the buffer, return the buffer size */
			len +=sprintf(procfs_buffer+len, "board id		= %x.%x\n", board_id[0], board_id[1] );			
			len +=sprintf(procfs_buffer+len, "mxc_debug		= %d\n", mxc_debug );
			len +=sprintf(procfs_buffer+len, "clock_auto	= %d\n", clock_auto );
			len +=sprintf(procfs_buffer+len, "extsync		= %d\n", extsync );

			len +=sprintf(procfs_buffer+len, "----HDMI-----\n");
			len +=sprintf(procfs_buffer+len, "video_mode	= %d\n", video_mode );
			len +=sprintf(procfs_buffer+len, "audio 		= %d\n", hdmi_audio);
			len +=sprintf(procfs_buffer+len, "spdif		= %d\n", enable_hdmi_spdif);

			procfs_buffer_size = len;
			
			memcpy(buffer, procfs_buffer, procfs_buffer_size);
			ret = procfs_buffer_size;
	}

	return ret;
}

/**
 * This function is called with the /proc file is written
 *
 */
int procfile_write(struct file *file, const char *buffer, unsigned long count,
                   void *data)
{
	/* get buffer size */
	procfs_buffer_size = count;
	if (procfs_buffer_size > PROCFS_MAX_SIZE ) {
	        procfs_buffer_size = PROCFS_MAX_SIZE;
	}

	/* write data to the buffer */
	if ( copy_from_user(procfs_buffer, buffer, procfs_buffer_size) ) {
	        return -EFAULT;
	}

	if ( strncmp( procfs_buffer, "hdmi_", 5) == 0 ) {
		if( sii9022_func )
			sii9022_func( procfs_buffer );
	} else if ( strncmp( procfs_buffer, "mxc_debug_on", 12) == 0 )
		mxc_debug = 1;
	else if ( strncmp( procfs_buffer, "mxc_debug_off", 13) == 0 )
		mxc_debug = 0;
	else if ( strncmp( procfs_buffer, "clock_auto_on", 13) == 0 )
		clock_auto = 1;
	else if ( strncmp( procfs_buffer, "clock_auto_off", 14) == 0 )
		clock_auto = 0;
	else if ( strncmp( procfs_buffer, "extsync_on", 10) == 0 )
		extsync = 1;
	else if ( strncmp( procfs_buffer, "extsync_off", 11) == 0 )
		extsync = 0;
	
	return procfs_buffer_size;
}

/**
 *This function is called when the module is loaded
 *
 */
int procfile_init(void)
{
	/* create the /proc file */
	Our_Proc_File = create_proc_entry(PROCFS_NAME, 0666, NULL);

	if (Our_Proc_File == NULL) {
	        remove_proc_entry(PROCFS_NAME, NULL);
	        printk(KERN_ALERT "Error: Could not initialize /proc/%s\n",
	                PROCFS_NAME);
	        return -ENOMEM;
	}

	Our_Proc_File->read_proc  = procfile_read;
	Our_Proc_File->write_proc = procfile_write;
	//Our_Proc_File->owner          = THIS_MODULE;
	Our_Proc_File->mode           = S_IFREG |S_IRUGO|S_IWUSR;
	Our_Proc_File->uid    = 0;
	Our_Proc_File->gid    = 0;
	Our_Proc_File->size           = 37;

	printk(KERN_INFO "/proc/%s created\n", PROCFS_NAME);    
	return 0;       /* everything is ok */
}

/**
 *This function is called when the module is unloaded
 *
 */
void procfile_cleanup(void)
{
	remove_proc_entry(PROCFS_NAME, NULL);
	printk(KERN_INFO "/proc/%s removed\n", PROCFS_NAME);
}

/* proc --end */

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	procfile_init();

	mxc_cpu_common_init();
	mxc_register_gpios();
	mx51_efikamx_io_init();
	early_console_setup(saved_command_line);

	mxc_init_devices();

//	mxc_init_pata();

	//mxc_init_bl();
	mxc_init_mmc();
	mxc_init_gpio_button();
	mx51_efikamx_init_mc13892();

	spi_register_board_info(mxc_spi_board_info,
				ARRAY_SIZE(mxc_spi_board_info));

#if defined(CONFIG_I2C_MXC) || defined(CONFIG_I2C_MXC_MODULE)

#ifdef CONFIG_I2C_MXC_SELECT1
	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
#endif
#ifdef CONFIG_I2C_MXC_SELECT2
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
#endif
#if defined(CONFIG_I2C_MXC_HS) || defined(CONFIG_I2C_MXC_HS_MODULE)
	if (cpu_is_mx51_rev(CHIP_REV_2_0) >= 1) {
		vga_data.core_reg = NULL;
		vga_data.io_reg = NULL;
		vga_data.analog_reg = NULL;
	}
	i2c_register_board_info(3, mxc_i2c_hs_board_info,
				ARRAY_SIZE(mxc_i2c_hs_board_info));
#endif

#endif
	pm_power_off = mxc_power_off;
	mxc_init_sgtl5000();
	
	printk(KERN_INFO "board ID: %d.%d\n", board_id[0], board_id[1] );	
}

static void __init mx51_efikamx_timer_init(void)
{
	if (cpu_is_mx51_rev(CHIP_REV_2_0) <= 1) {
		cpu_wp_auto[0].cpu_voltage = 1175000;
		cpu_wp_auto[1].cpu_voltage = 1100000;
		cpu_wp_auto[2].cpu_voltage = 1000000;
	}
	
	mx51_clocks_init(32768, 24000000, 22579200, 24576000);
	//mx51_timer_init("gpt_clk.0");
}

static struct sys_timer mxc_timer = {
	.init	= mx51_efikamx_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX51_EFIKAMX data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX51_EFIKAMX, "Efika MX")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.phys_io = AIPS1_BASE_ADDR,
	.io_pg_offst = ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx51_map_io,
	.init_irq = mxc_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
