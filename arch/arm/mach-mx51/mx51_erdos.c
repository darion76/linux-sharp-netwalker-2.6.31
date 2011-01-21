/**
 * mach-mx51/mx51_erdos.c
 *
 * This file contains the board specific initialization routines.
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
 * 2009/07/03 : SPI nor not specify device name.
 * 2009/07/21 : num_cpu_wp/mxc_init_power_key() add.
 * 2009/07/27 : gpio_poweroff() add.
 *		mxc_init_fb() DI1_D1_CS, DISPB2_SER_DIN, DISPB2_SER_DIO, GPIO1_9 delete.
 *			      NANDF_D12, NANDF_D10, NANDF_D13 delete.
 * 2009/08/13 : machine_name fixed.
 * 2009/10/16 : mxc_sgtl5000_line_mute() add.
 *              mxc_sgtl5000_amp_enable() condition add mute.
 * 2010/12/3  : ported to 2.6.31 by Andrey Zhornyak darion76@gmail.com
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
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#endif	/* CONFIG_PROC_FS */
#ifdef CONFIG_ALLOC_FUNC_KEY
#include <asm/func_key.h>
#endif
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
#include "board-mx51_erdos.h"
#include "iomux.h"
#include "crm_regs.h"

/* Ported from efikamx */
//int __initdata clock_auto = { 1 };
//int __initdata extsync = { 0 };
int clock_auto = { 0 };
int extsync = { 0 };

EXPORT_SYMBOL(clock_auto);
EXPORT_SYMBOL(extsync);

static int __init clock_setup(char *options)
{	
    if (!options || !*options)
    return 1;

    clock_auto = simple_strtol(options, NULL, 10);
    printk("dumb clock_auto=%d\n", clock_auto);

    return 1;
}

__setup("clock_auto=", clock_setup);

void mxcfb_adjust(struct fb_var_screeninfo *var )
{
    return;
}
/* End Efikamx port */

extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);
extern void sgtl5000_dap_enable(int jack, int event);

static int num_cpu_wp = 3;

/* working point(wp): 0 - 800MHz; 1 - 200MHz; */
static struct cpu_wp cpu_wp_auto[] = {
	{
		 .pll_rate = 1000000000,
		 .cpu_rate = 1000000000,
		 .pdf = 0,
		 .mfi = 10,
		 .mfd = 11,
		 .mfn = 5,
		 .cpu_podf = 0,
		 .cpu_voltage = 1175000,
	}, {
		.pll_rate    = 800000000,
		.cpu_rate    = 800000000,
		.pdf	     = 0,
		.mfi	     = 8,
		.mfd	     = 2,
		.mfn	     = 1,
		.cpu_podf    = 0,
		.cpu_voltage = 1100000,
	}, {
		.pll_rate    = 800000000,
		.cpu_rate    = 166250000,
		.pdf	     = 4,
		.mfi	     = 8,
		.mfd	     = 2,
		.mfn	     = 1,
		.cpu_podf    = 4,
		.cpu_voltage = 1000000,
	},
};

struct cpu_wp *mx51_erdos_get_cpu_wp(int *wp)
{
	*wp = num_cpu_wp;
	return cpu_wp_auto;
}

void mx51_erdos_set_num_cpu_wp(int num)
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
		.interface_pix_fmt = IPU_PIX_FMT_LVDS666,
		.mode_str	   = "1024x768M-16@60",
	}, {
		.interface_pix_fmt = IPU_PIX_FMT_RGB565,
		.mode_str	   = "1024x768M-16@60",
	},
};

static struct platform_device mxc_fb_device[] = {
	{
		.name = "mxc_sdc_fb",
		.id   = 0,
		.dev  = {
			.release	   = mxc_nop_release,
			.coherent_dma_mask = 0xFFFFFFFF,
			.platform_data	   = &fb_data[0],
		},
	}, {
		.name = "mxc_sdc_fb",
		.id   = 1,
		.dev  = {
			.release	   = mxc_nop_release,
			.coherent_dma_mask = 0xFFFFFFFF,
			.platform_data	   = &fb_data[1],
		},
	}, {
		.name = "mxc_sdc_fb",
		.id   = 2,
		.dev  = {
			.release	   = mxc_nop_release,
			.coherent_dma_mask = 0xFFFFFFFF,
		},
	},
};

static int __initdata enable_vga = {0};

static void __init mxc_init_fb(void)
{
	/**
	 * don't use DVI
	 */

	if (!enable_vga) {
		platform_device_register(&mxc_fb_device[0]);
	} else {
		platform_device_register(&mxc_fb_device[1]);
	}
	platform_device_register(&mxc_fb_device[2]);
}

static int __init vga_setup(char *__unused)
{
	enable_vga = 1;
	return 1;
}

__setup("vga", vga_setup);
#else
static inline void mxc_init_fb(void)
{
}
#endif

#if defined(CONFIG_FB_MXC_SHARP_WSVGA_SYNC_PANEL) || \
    defined(CONFIG_FB_MXC_SHARP_WSVGA_SYNC_PANEL_MODULE)

static void lcd_reset(void)
{
}

static struct mxc_lcd_platform_data lcd_data = {
	.reset	    = lcd_reset,
};

static struct platform_device mxc_lcd_device = {
	.name = "lcd_sharp",
	.id   = 0,
	.dev  = {
		.release	   = mxc_nop_release,
		.coherent_dma_mask = 0xFFFFFFFF,
		.platform_data	   = &lcd_data,
	},
};

static void __init mxc_init_lcd(void)
{
	if (!enable_vga) {
		platform_device_register(&mxc_lcd_device);
	}
}
#else
static inline void mxc_init_lcd(void)
{
}
#endif

static struct platform_device mxcbl_device = {
	.name = "mxc_mc13892_bl",
};

static inline void mxc_init_bl(void)
{
	platform_device_register(&mxcbl_device);
}

#if defined(CONFIG_I2C_MXC) || defined(CONFIG_I2C_MXC_MODULE)

#ifdef CONFIG_I2C_MXC_SELECT1
static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		.type = "mxc_etk",
		.addr = 0x28,
	},
};
#endif

#ifdef CONFIG_I2C_MXC_SELECT2
static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		.type = "sgtl5000-i2c",
		.addr = 0x0a,
	},
};
#endif

static void __init mxc_init_i2c(void)
{
#ifdef CONFIG_I2C_MXC_SELECT1
	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
#endif

#ifdef CONFIG_I2C_MXC_SELECT2
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
#endif
}
#else
static inline void mxc_init_i2c(void)
{
}
#endif

#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/flash.h>
#include <mach/mx51_erdos_dev_params.h>


struct mtd_partition mxc_spi_flash_partitions[] = {
	{
		.name	    = "Redboot",
		.offset     = 0,
		.size	    = 0x00080000,
		.mask_flags = MTD_CAP_ROM
	}, {
		.name	    = "Redboot fis",
		.offset     = MTDPART_OFS_APPEND,
		.size	    = 0x00010000
	}, {
		.name	    = DEV_PARAMS_PART_NAME,
		.offset     = MTDPART_OFS_APPEND,
		.size	    = MTDPART_SIZ_FULL
	},
};



struct flash_platform_data mxc_spi_flash_data = {
	.name	  = "mxc_spi_nor",
	.parts	  = mxc_spi_flash_partitions,
	.nr_parts = ARRAY_SIZE(mxc_spi_flash_partitions),
	.type	  = 0,		/* not specify device, ex)"sst25vf016b" */
};

#endif


static struct spi_board_info mxc_spi_board_info[] __initdata = {
	{
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
		.modalias      = "mxc_spi_nor",
#if 1
		/*
		 * tempolary setting for can't get correct ChipID
		 * We have to need re-checking (CLK, Timing).
		 */
		.max_speed_hz  = 12500000,
		.bus_num       = 1,
		.chip_select   = 1,
		.mode	       = SPI_MODE_0,
		.platform_data = &mxc_spi_flash_data,
#else
		.max_speed_hz  = 2500000,
		.bus_num       = 1,
		.chip_select   = 1,
		.mode	       = SPI_MODE_0,
		.platform_data = &mxc_spi_flash_data,
#endif
#endif
	},
};

#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)

static void __init mxc_init_spi_nor(void)
{
	spi_register_board_info(mxc_spi_board_info,
				ARRAY_SIZE(mxc_spi_board_info));
}

#else

static inline void mxc_init_spi_nor(void)
{
}

#endif

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
unsigned int expio_intr_fec;
EXPORT_SYMBOL(expio_intr_fec);
#endif

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)

extern u_char sdio_detect_enable;

static int sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;

	/* because QA0 board not assigned wp port,
	   always return no protection status */
	return rc;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret = 0;

	if (to_platform_device(dev)->id == 0) {
//		ret = mxc_get_gpio_datain(MX51_PIN_GPIO1_0); //darion
		ret = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_0));
	} else if (to_platform_device(dev)->id == 1) {
		/* because QA0 board not assigned detect port,
		   always return inserted status */
		ret = ~(sdio_detect_enable) & 0x1;
	}

	return ret;
}

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask	     = MMC_VDD_31_32,
	.caps		     = MMC_CAP_4_BIT_DATA,
	.min_clk	     = 400000,
	.max_clk	     = 25000000,
	.card_inserted_state = 1,
	.status 	     = sdhc_get_card_det_status,
	.wp_status	     = sdhc_write_protect,
	.clock_mmc	     = "esdhc_clk",
	.power_mmc	     = NULL,
};

static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
	    MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 17500000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};

/*!
 * Resource definition for the SDHC1
 */
static struct resource mxcsdhc1_resources[] = {
	[0] = {
		.start = MMC_SDHC1_BASE_ADDR,
		.end   = MMC_SDHC1_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MXC_INT_MMC_SDHC1,
		.end   = MXC_INT_MMC_SDHC1,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IOMUX_TO_IRQ(MX51_PIN_GPIO1_0),
		.end   = IOMUX_TO_IRQ(MX51_PIN_GPIO1_0),
		.flags = IORESOURCE_IRQ,
	},
};

/*!
 * Resource definition for the SDHC2
 */
static struct resource mxcsdhc2_resources[] = {
	[0] = {
		.start = MMC_SDHC2_BASE_ADDR,
		.end   = MMC_SDHC2_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MXC_INT_MMC_SDHC2,
		.end   = MXC_INT_MMC_SDHC2,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = 0,	/* not connect */
		.end   = 0,	/* not connect */
		.flags = IORESOURCE_IRQ,
	},
};

/*! Device Definition for MXC SDHC1 */
static struct platform_device mxcsdhc1_device = {
	.name	       = "mxsdhci",
	.id	       = 0,
	.dev	       = {
		.release       = mxc_nop_release,
		.platform_data = &mmc1_data,
	},
	.num_resources = ARRAY_SIZE(mxcsdhc1_resources),
	.resource      = mxcsdhc1_resources,
};

/*! Device Definition for MXC SDHC2 */
static struct platform_device mxcsdhc2_device = {
	.name	       = "mxsdhci",
	.id	       = 1,
	.dev	       = {
		.release       = mxc_nop_release,
		.platform_data = &mmc2_data,
	},
	.num_resources = ARRAY_SIZE(mxcsdhc2_resources),
	.resource      = mxcsdhc2_resources,
};

static void __init mxc_init_mmc(void)
{
	platform_device_register(&mxcsdhc1_device);
	platform_device_register(&mxcsdhc2_device);
}
#else
static inline void mxc_init_mmc(void)
{
}
#endif

#if defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000) || \
    defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000_MODULE)

static int headphone_det_status(void)
{
//	return (mxc_get_gpio_datain(MX51_PIN_NANDF_D14) == 0);
	return (gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_NANDF_D14)) == 0);
}

static int line_mute_flag = 1;		/* mute flag */
static int enable_f = 0;		/* amp enable flag */
/*
 * mxc_sgtl5000_line_mute - mute ON/OFF
 */
void mxc_sgtl5000_line_mute (int mute)
{
	if (line_mute_flag != mute) {
		int jack = headphone_det_status();
		if ((jack == 0) && (enable_f == 1) && (mute == 0)) {
			/*
			 * LINE-ON, AMP-enable, MUTE-OFF
			 */
//			mxc_set_gpio_dataout(MX51_PIN_EIM_A23, 1);
//			mxc_set_gpio_dataout(MX51_PIN_EIM_A25, 1);darion
			gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A23),1);
			gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A25),1);
		} else {
//			mxc_set_gpio_dataout(MX51_PIN_EIM_A23, 0);
//			mxc_set_gpio_dataout(MX51_PIN_EIM_A25, 0);
			gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A23),0);
			gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A25),0);
		}
		line_mute_flag = mute;
	}
}

static int mxc_sgtl5000_amp_enable(int enable)
{
	int jack;

	if (enable == 0 || enable == 1 || enable == -1){
	} else {
		return -EINVAL;
	}

	if (enable == 0 || enable == 1){
		enable_f = enable;
	}

	jack = headphone_det_status();
	if ((jack == 0) && (enable_f == 1) && (line_mute_flag == 0)) {
		/*
		 * LINE-ON, AMP-enable, MUTE-OFF
		 */
//		sgtl5000_dap_enable(jack, 1);
//		mxc_set_gpio_dataout(MX51_PIN_EIM_A23, 1);
//		mxc_set_gpio_dataout(MX51_PIN_EIM_A25, 1);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A23),1);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A25),1);
	} else {
//		mxc_set_gpio_dataout(MX51_PIN_EIM_A23, 0);
//		mxc_set_gpio_dataout(MX51_PIN_EIM_A25, 0);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A23),0);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A25),0);
//		sgtl5000_dap_enable(jack, 0);
	}

	return jack;
}

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num    = 1,
	.src_port   = 2,
	.ext_port   = 3,
	.hp_irq     = IOMUX_TO_IRQ(MX51_PIN_NANDF_D14),
	.hp_status  = headphone_det_status,
//	.vddio_reg  = "VVIDEO", //darion
//	.vdda_reg   = "VDIG",
//	.vddd_reg   = "VGEN1",
	.amp_enable = mxc_sgtl5000_amp_enable,
//	.vddio	    = 2775000,
//	.vdda	    = 1650000,
//	.vddd	    = 1200000,
	.sysclk     = 12288000,
};

static struct platform_device mxc_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
	.dev  = {
		.release       = mxc_nop_release,
		.platform_data = &sgtl5000_data,
	},
};

static void __init mxc_init_sgtl5000(void)
{
	sgtl5000_data.sysclk   = 26000000;
//	sgtl5000_data.vddd_reg = NULL;//darion
//	sgtl5000_data.vddd     = 0;

//	mxc_set_gpio_direction(MX51_PIN_EIM_A23, 0); //darion
//	mxc_set_gpio_direction(MX51_PIN_EIM_A25, 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_A23), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_A25), 0);

	platform_device_register(&mxc_sgtl5000_device);
}
#else
static inline void mxc_init_sgtl5000(void)
{
}
#endif

#if defined(CONFIG_MTD_NAND_MXC)	   || \
    defined(CONFIG_MTD_NAND_MXC_MODULE)    || \
    defined(CONFIG_MTD_NAND_MXC_V2)	   || \
    defined(CONFIG_MTD_NAND_MXC_V2_MODULE) || \
    defined(CONFIG_MTD_NAND_MXC_V3)	   || \
    defined(CONFIG_MTD_NAND_MXC_V3_MODULE)

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

struct nand_flash_platform_data {
	const char	     *map_name;
	const char	     *name;
	unsigned int	     width;
	int		     (*init)(void);
	void		     (*exit)(void);
	void		     (*set_vpp)(int on);
	void		     (*mmcontrol)(struct mtd_info *mtd, int sync_read);
	struct mtd_partition *parts;
	unsigned int	     nr_parts;
};

#define MBTOB (1024 * 1024UL)
static struct mtd_partition mxc_nand_partitions[] = {
	{
		.name	= "kernel",
		.offset = 0,
		.size	= 6 * MBTOB,
	}, {
		.name	= "rfs",
		.offset = MTDPART_OFS_APPEND,
		.size	= (4096 - 6 - 4 - 1) * MBTOB,	/* -6(Kernel)-4(BBT)-1(not_use) */
	},
	/*
	 * last 4 block (FFB00000)/(FFE00000) - BBT
	 * last 1 block (FFF00000)	      - not use block
	 */
};

static struct nand_flash_platform_data mxc_nand_data = {
	.parts	  = mxc_nand_partitions,
	.nr_parts = ARRAY_SIZE(mxc_nand_partitions),
	.width	  = 1,
};

static struct platform_device mxc_nandv2_mtd_device = {
	.name = "mxc_nandv2_flash",
	.id   = 0,
	.dev  = {
		.release       = mxc_nop_release,
		.platform_data = &mxc_nand_data,
	},
};

static void mxc_init_nand_mtd(void)
{
	platform_device_register(&mxc_nandv2_mtd_device);
}
#else
static inline void mxc_init_nand_mtd(void)
{
}
#endif

#if defined(CONFIG_GPIO_BUTTON_MXC) || \
    defined(CONFIG_GPIO_BUTTON_MXC_MODULE)

#define MXC_BUTTON_GPIO_PIN MX51_PIN_EIM_DTACK

static struct mxc_gpio_button_data gpio_button_data = {
	.name = "Power Button (CM)",
	.gpio = MXC_BUTTON_GPIO_PIN,
	.irq  = IOMUX_TO_IRQ(MXC_BUTTON_GPIO_PIN),
	.key  = KEY_POWER,
};

static struct platform_device gpio_button_device = {
	.name = "gpio_button",
	.dev  = {
		.release = mxc_nop_release,
		.platform_data = &gpio_button_data,
	},
};

static void __init mxc_init_gpio_button(void)
{
	mxc_set_gpio_direction(MXC_BUTTON_GPIO_PIN, 1);
	platform_device_register(&gpio_button_device);
}
#else
static inline void mxc_init_gpio_button(void)
{
}
#endif

#ifdef CONFIG_MXC_PMIC

#define PWGT1SPIEN (1<<15)
#define PWGT2SPIEN (1<<16)
#define USEROFFSPI (1<<3)

static void mxc_power_off(void)
{
	extern void gpio_poweroff (void);
	gpio_poweroff ();

	/**
	 * We can do power down one of two ways:
	 * Set the power gating
	 * Set USEROFFSPI
	 */

	/* Set the power gate bits to power down */
	pmic_write_reg(REG_POWER_MISC, (PWGT1SPIEN|PWGT2SPIEN),
		       (PWGT1SPIEN|PWGT2SPIEN));
}

static void __init mxc_init_power_off(void)
{
	pm_power_off = mxc_power_off;
}
#else
static inline void mxc_init_power_off(void)
{
}
#endif

#ifdef CONFIG_JOYSTICK_OJ6SH

#include <linux/oj6sh.h>
extern void gpio_oj6sh_shutdown(int shutdown);

static struct oj6sh_platform_data oj6sh_data = {
	.left_btn_irq  = IOMUX_TO_IRQ(MX51_PIN_EIM_EB2),
	.right_btn_irq = IOMUX_TO_IRQ(MX51_PIN_EIM_EB3),
	.shutdown      = gpio_oj6sh_shutdown,
};

static struct spi_board_info mxc_spi_oj6sh_device __initdata = {
	.modalias      = "oj6sh_spi",
	.max_speed_hz  = 2500000,
	.bus_num       = 1,
	.chip_select   = 2,
	.mode          = SPI_MODE_0,
	.irq           = IOMUX_TO_IRQ(MX51_PIN_CSI1_HSYNC),
	.platform_data = (void *)&oj6sh_data,
};

static void __init mxc_init_oj6sh(void)
{
	spi_register_board_info(&mxc_spi_oj6sh_device,
	                        ARRAY_SIZE(mxc_spi_board_info));
}
#else
static inline void mxc_init_oj6sh(void)
{
}
#endif

#ifdef CONFIG_GPIO_SW
#include <linux/gpio_sw.h>
#include <linux/input.h>
extern int get_gpio_cover_sw(void);
static struct gpio_sw_platform_data cover_sw_data = {
	.irq	    = IOMUX_TO_IRQ(MX51_PIN_CSI2_D19),
	.output_pol = 1,
	.event_type = EV_SW,
	.event_code = SW_LID,
	.wake	    = 0,
	.input_name = "cover_sw",
	.irq_name   = "cover_sw_irq",
	.get_value  = get_gpio_cover_sw,
};

static struct platform_device cover_sw_device = {
	.name	       = "gpio_sw",
	.id	       = 0,
	.dev = {
		.platform_data = &cover_sw_data,
	},
};

extern int get_gpio_power_sw(void);
static struct gpio_sw_platform_data power_sw_data = {
	.irq	    = IOMUX_TO_IRQ(MX51_PIN_EIM_A27),
	.output_pol = 0,
	.event_type = EV_KEY,
	.event_code = KEY_POWER,
	.wake	    = 1,
	.input_name = "power_sw",
	.irq_name   = "power_sw_irq",
	.get_value  = get_gpio_power_sw,
};

static struct platform_device power_sw_device = {
	.name	       = "gpio_sw",
	.id	       = 1,
	.dev = {
		.platform_data = &power_sw_data,
	},
};

static void __init mxc_init_gpio_sw(void)
{
	platform_device_register(&cover_sw_device);
	platform_device_register(&power_sw_device);
}
#else
static inline void mxc_init_gpio_sw(void)
{
}
#endif

#if defined CONFIG_TOUCHKEY_MXC

/* mxc touch key driver */
static struct platform_device mxc_etk_device = {
	.name = "mxc_touch_key",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
};
static void mxc_init_mxc_etk(void)
{
	platform_device_register(&mxc_etk_device);
}
#else
static inline void mxc_init_mxc_etk(void)
{
}
#endif

/**
 * Board specific fixup function. It is called by setup_arch() in setup.c file
 * very early on during kernel starts. It allows the user to statically fill
 * in the proper values for the passed-in parameters. None of the parameters
 * is used currently.
 *
 * @param  desc     pointer to struct machine_desc
 * @param  tags     pointer to struct tag
 * @param  cmdline  pointer to the command line
 * @param  mi	    pointer to struct meminfo
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	struct tag *t;

	mxc_cpu_init();

	get_cpu_wp = mx51_erdos_get_cpu_wp;
	set_num_cpu_wp = mx51_erdos_set_num_cpu_wp;

	for_each_tag(t, tags) {
		if (t->hdr.tag != ATAG_MEM) {
			continue;
		}
		if (t->u.mem.size == SZ_512M) {
			t->u.mem.size -= SZ_32M;
		}
		mxcfb_resources[0].start = t->u.mem.start + t->u.mem.size;
		mxcfb_resources[0].end	 = t->u.mem.start + SZ_512M - 1;
	}
}

/**
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	mxc_cpu_common_init();
//	mxc_gpio_init(); //darion
	mxc_register_gpios();
	mx51_erdos_io_init();
	early_console_setup(saved_command_line);

	mxc_init_devices();
	mxc_init_fb();
	mxc_init_lcd();
//	mxc_init_bl();
	printk("darion: mxc_init_mmc started\n");
	mxc_init_mmc();
//	mxc_init_gpio_button();
	printk("darion: mx51_erdos_init_mc13892 started\n");
	mx51_erdos_init_mc13892();
#if defined(CONFIG_MTD_NAND_MXC)	   || \
    defined(CONFIG_MTD_NAND_MXC_MODULE)    || \
    defined(CONFIG_MTD_NAND_MXC_V2)	   || \
    defined(CONFIG_MTD_NAND_MXC_V2_MODULE) || \
    defined(CONFIG_MTD_NAND_MXC_V3)	   || \
    defined(CONFIG_MTD_NAND_MXC_V3_MODULE)
//	mxc_init_nand_mtd();
//	mxc_init_spi_nor();
#endif
	mxc_init_i2c();
	printk("darion: mxc_init_power_off started\n");
	mxc_init_power_off();
	printk("darion: mxc_init_sgtl5000 started\n");
	mxc_init_sgtl5000();
//	printk("darion: mxc_init_oj6sh started\n");
//	mxc_init_oj6sh();
//	mxc_init_gpio_sw();
//	mxc_init_mxc_etk();
}

static void __init mx51_erdos_timer_init(void)
{
	printk("darion: erdos_timer_init started\n");
	if (cpu_is_mx51_rev(CHIP_REV_2_0) <= 1) {
	    cpu_wp_auto[0].cpu_voltage = 1175000;
	    cpu_wp_auto[1].cpu_voltage = 1100000;
	    cpu_wp_auto[2].cpu_voltage = 1000000;
	}

//	mxc_clocks_init(32768, 24000000, 22579200, 24576000);
	mx51_clocks_init(32768, 24000000, 22579200, 24576000);
//	mx51_timer_init("gpt_clk.0");
}

static struct sys_timer mxc_timer = {
	.init = mx51_erdos_timer_init,
};

int mxc_reboot (void)
{
    return 0;
}

MACHINE_START(MX51_BABBAGE, "SHARP PC-Z1")
	.phys_io      = AIPS1_BASE_ADDR,
	.io_pg_offst  = ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params  = PHYS_OFFSET + 0x100,
	.fixup	      = fixup_mxc_board,
	.map_io       = mx51_map_io,
	.init_irq     = mxc_init_irq,
	.init_machine = mxc_board_init,
	.timer	      = &mxc_timer,
MACHINE_END
