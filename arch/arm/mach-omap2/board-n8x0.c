/*
 * linux/arch/arm/mach-omap2/board-n8x0.c
 *
 * Copyright (C) 2005-2009 Nokia Corporation
 * Author: Juha Yrjola <juha.yrjola@nokia.com>
 *
 * Modified from mach-omap2/board-generic.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/stddef.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc2005.h>
#include <linux/input.h>
#include <linux/usb/musb.h>
#ifdef CONFIG_MENELAUS
#include <linux/i2c.h>
#endif
#include <linux/i2c/lm8323.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <plat/board.h>
#include <plat/common.h>
#include <mach/irqs.h>
#include <plat/mcspi.h>
#ifdef CONFIG_MENELAUS
#include <plat/menelaus.h>
#endif
#include <plat/onenand.h>
#include <plat/serial.h>
#include <plat/cbus.h>

#define	RX51_TSC2005_RESET_GPIO	94
#define	RX51_TSC2005_IRQ_GPIO	106
#define OMAP_TAG_NOKIA_BT	0x4e01

static s16 rx44_keymap[LM8323_KEYMAP_SIZE] = {
	[0x01] = KEY_Q,
	[0x02] = KEY_K,
	[0x03] = KEY_O,
	[0x04] = KEY_P,
	[0x05] = KEY_BACKSPACE,
	[0x06] = KEY_A,
	[0x07] = KEY_S,
	[0x08] = KEY_D,
	[0x09] = KEY_F,
	[0x0a] = KEY_G,
	[0x0b] = KEY_H,
	[0x0c] = KEY_J,

	[0x11] = KEY_W,
	[0x12] = KEY_F4,
	[0x13] = KEY_L,
	[0x14] = KEY_APOSTROPHE,
	[0x16] = KEY_Z,
	[0x17] = KEY_X,
	[0x18] = KEY_C,
	[0x19] = KEY_V,
	[0x1a] = KEY_B,
	[0x1b] = KEY_N,
	[0x1c] = KEY_LEFTSHIFT, /* Actually, this is both shift keys */
	[0x1f] = KEY_F7,

	[0x21] = KEY_E,
	[0x22] = KEY_SEMICOLON,
	[0x23] = KEY_MINUS,
	[0x24] = KEY_EQUAL,
	[0x2b] = KEY_FN,
	[0x2c] = KEY_M,
	[0x2f] = KEY_F8,

	[0x31] = KEY_R,
	[0x32] = KEY_RIGHTCTRL,
	[0x34] = KEY_SPACE,
	[0x35] = KEY_COMMA,
	[0x37] = KEY_UP,
	[0x3c] = KEY_COMPOSE,
	[0x3f] = KEY_F6,

	[0x41] = KEY_T,
	[0x44] = KEY_DOT,
	[0x46] = KEY_RIGHT,
	[0x4f] = KEY_F5,
	[0x51] = KEY_Y,
	[0x53] = KEY_DOWN,
	[0x55] = KEY_ENTER,
	[0x5f] = KEY_ESC,

	[0x61] = KEY_U,
	[0x64] = KEY_LEFT,

	[0x71] = KEY_I,
	[0x75] = KEY_KPENTER,
};

static struct lm8323_platform_data lm8323_pdata = {
	.repeat		= 0, /* Repeat is handled in userspace for now. */
	.keymap		= rx44_keymap,
	.size_x		= 8,
	.size_y		= 12,
	.debounce_time	= 12,
	.active_time	= 500,

	.name		= NULL,  // Let the module determine the device name.
	.pwm_names[0] 	= "n810::keyboard",
	.pwm_names[1] 	= "n810::cover",
	//.pwm1_name	= "n810::keyboard",
	//.pwm2_name	= "n810::cover",
};

struct omap_bluetooth_config {
	u8    chip_type;
	u8    bt_wakeup_gpio;
	u8    host_wakeup_gpio;
	u8    reset_gpio;
	u8    bt_uart;
	u8    bd_addr[6];
	u8    bt_sysclk;
};

static struct platform_device n8x0_bt_device = {
	.name           = "hci_h4p",
	.id             = -1,
	.num_resources  = 0,
};

void __init n8x0_bt_init(void)
{
	const struct omap_bluetooth_config *bt_config;

	bt_config = (void *) omap_get_config(OMAP_TAG_NOKIA_BT,
					     struct omap_bluetooth_config);
	n8x0_bt_device.dev.platform_data = (void *) bt_config;
	if (platform_device_register(&n8x0_bt_device) < 0)
		BUG();
}

static struct omap2_mcspi_device_config mipid_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,
};

static struct omap2_mcspi_device_config p54spi_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,
};

#ifdef CONFIG_MACH_NOKIA_N8X0_LCD
extern struct mipid_platform_data n8x0_mipid_platform_data;
#endif

#ifdef CONFIG_TOUCHSCREEN_TSC2005
static struct tsc2005_platform_data tsc2005_config;
static void rx51_tsc2005_set_reset(bool enable)
{
	gpio_set_value(RX51_TSC2005_RESET_GPIO, enable);
}

static struct omap2_mcspi_device_config tsc2005_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,
};
#endif

static struct spi_board_info n800_spi_board_info[] __initdata = {
#ifdef CONFIG_MACH_NOKIA_N8X0_LCD
	{
		.modalias	= "lcd_mipid",
		.bus_num	= 1,
		.chip_select	= 1,
		.max_speed_hz	= 4000000,
		.controller_data= &mipid_mcspi_config,
		.platform_data	= &n8x0_mipid_platform_data,
	},
#endif
	{
		.modalias	= "p54spi",
		.bus_num	= 2,
		.chip_select	= 0,
		.max_speed_hz   = 48000000,
		.controller_data = &p54spi_mcspi_config,
	},
	{
		.modalias	 = "tsc2005",
		.bus_num	 = 1,
		.chip_select	 = 0,
		.irq		 = OMAP_GPIO_IRQ(RX51_TSC2005_IRQ_GPIO),
		.max_speed_hz    = 6000000,
		.controller_data = &tsc2005_mcspi_config,
		.platform_data   = &tsc2005_config,
	},
};

static void __init tsc2005_set_config(void)
{
	const struct omap_lcd_config *conf;

	conf = omap_get_config(OMAP_TAG_LCD, struct omap_lcd_config);
	if (conf != NULL) {
#ifdef CONFIG_TOUCHSCREEN_TSC2005
		if (strcmp(conf->panel_name, "lph8923") == 0) {
			tsc2005_config.ts_x_plate_ohm = 180;
			tsc2005_config.ts_hw_avg = 0;
			tsc2005_config.ts_ignore_last = 0;
			tsc2005_config.ts_touch_pressure = 1500;
			tsc2005_config.ts_stab_time = 100;
			tsc2005_config.ts_pressure_max = 2048;
			tsc2005_config.ts_pressure_fudge = 2;
			tsc2005_config.ts_x_max = 4096;
			tsc2005_config.ts_x_fudge = 4;
			tsc2005_config.ts_y_max = 4096;
			tsc2005_config.ts_y_fudge = 7;
			tsc2005_config.set_reset = rx51_tsc2005_set_reset;
		} else if (strcmp(conf->panel_name, "ls041y3") == 0) {
			tsc2005_config.ts_x_plate_ohm = 280;
			tsc2005_config.ts_hw_avg = 0;
			tsc2005_config.ts_ignore_last = 0;
			tsc2005_config.ts_touch_pressure = 1500;
			tsc2005_config.ts_stab_time = 1000;
			tsc2005_config.ts_pressure_max = 2048;
			tsc2005_config.ts_pressure_fudge = 2;
			tsc2005_config.ts_x_max = 4096;
			tsc2005_config.ts_x_fudge = 4;
			tsc2005_config.ts_y_max = 4096;
			tsc2005_config.ts_y_fudge = 7;
			tsc2005_config.set_reset = rx51_tsc2005_set_reset;
		} else {
			printk(KERN_ERR "Unknown panel type, set default "
			       "touchscreen configuration\n");
			tsc2005_config.ts_x_plate_ohm = 200;
			tsc2005_config.ts_stab_time = 100;
		}
#endif
	}
}

#ifdef CONFIG_MENELAUS
static int n8x0_auto_sleep_regulators(void)
{
	u32 val;
	int ret;

	val = EN_VPLL_SLEEP | EN_VMMC_SLEEP    \
		| EN_VAUX_SLEEP | EN_VIO_SLEEP \
		| EN_VMEM_SLEEP | EN_DC3_SLEEP \
		| EN_VC_SLEEP | EN_DC2_SLEEP;

	ret = menelaus_set_regulator_sleep(1, val);
	if (ret < 0) {
		printk(KERN_ERR "Could not set regulators to sleep on "
			"menelaus: %u\n", ret);
		return ret;
	}
	return 0;
}

static int n8x0_auto_voltage_scale(void)
{
	int ret;

	ret = menelaus_set_vcore_hw(1400, 1050);
	if (ret < 0) {
		printk(KERN_ERR "Could not set VCORE voltage on "
			"menelaus: %u\n", ret);
		return ret;
	}
	return 0;
}

static int n8x0_menelaus_init(struct device *dev)
{
	int ret;

	ret = n8x0_auto_voltage_scale();
	if (ret < 0)
		return ret;
	ret = n8x0_auto_sleep_regulators();
	if (ret < 0)
		return ret;
	return 0;
}

static struct menelaus_platform_data n8x0_menelaus_platform_data = {
	.late_init = n8x0_menelaus_init,
};

static struct i2c_board_info __initdata n8x0_i2c_board_info_menelaus[] = {
	{
		I2C_BOARD_INFO("menelaus", 0x72),
		.irq = INT_24XX_SYS_NIRQ,
		.platform_data = &n8x0_menelaus_platform_data,
	},
};
#endif

static struct i2c_board_info __initdata_or_module n8x0_i2c_board_info_2[] = {};

static struct i2c_board_info __initdata_or_module n810_i2c_board_info_2[] = {
	{
		I2C_BOARD_INFO("lm8323", 0x45),
		.irq		= OMAP_GPIO_IRQ(109),
		.platform_data	= &lm8323_pdata,
	},
};

#if defined(CONFIG_MTD_ONENAND_OMAP2) || \
	defined(CONFIG_MTD_ONENAND_OMAP2_MODULE)

static struct mtd_partition onenand_partitions[] = {
	{
		.name           = "bootloader",
		.offset         = 0,
		.size           = 0x20000,
		.mask_flags     = MTD_WRITEABLE,	/* Force read-only */
	},
	{
		.name           = "config",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 0x60000,
	},
	{
		.name           = "kernel",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 0x200000,
	},
	{
		.name           = "initfs",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 0x400000,
	},
	{
		.name           = "rootfs",
		.offset         = MTDPART_OFS_APPEND,
		.size           = MTDPART_SIZ_FULL,
	},
};

static struct cbus_host_platform_data n8x0_cbus_data = {
	.clk_gpio	= 66,
	.dat_gpio	= 65,
	.sel_gpio	= 64,
};

static struct platform_device n8x0_cbus_device = {
	.name		= "cbus",
	.id		= -1,
	.dev		= {
		.platform_data = &n8x0_cbus_data,
	},
};

static struct omap_onenand_platform_data board_onenand_data = {
	.cs		= 0,
	.gpio_irq	= 26,
	.parts		= onenand_partitions,
	.nr_parts	= ARRAY_SIZE(onenand_partitions),
	.flags		= ONENAND_SYNC_READ,
};

static void __init n8x0_onenand_init(void)
{
	gpmc_onenand_init(&board_onenand_data);
}

#else

static void __init n8x0_onenand_init(void) {}

#endif

static void __init n8x0_map_io(void)
{
	omap2_set_globals_242x();
	omap2_map_common_io();
}

static void __init n8x0_init_irq(void)
{
	omap2_init_common_hw(NULL, NULL);
	omap_init_irq();
	omap_gpio_init();
}

#if defined(CONFIG_MMC_OMAP) || defined(CONFIG_MMC_OMAP_MODULE)
extern void n8x0_mmc_init(void);
#endif
#ifdef CONFIG_MACH_NOKIA_N8X0_LCD
extern void n8x0_mipid_init(void);
extern void n8x0_blizzard_init(void);
#else
#define n8x0_mipid_init() 0
#define n8x0_blizzard_init() 0
#endif
#ifdef CONFIG_MACH_NOKIA_N8X0_USB
extern void n8x0_usb_init(void);
#else
#	define n8x0_usb_init() 0
#endif

static void __init n8x0_init_machine(void)
{
	platform_device_register(&n8x0_cbus_device);

#if defined(CONFIG_MMC_OMAP) || defined(CONFIG_MMC_OMAP_MODULE)
	n8x0_mmc_init();
#endif
	n8x0_bt_init();
	n8x0_usb_init();

	/* FIXME: add n810 spi devices */
	tsc2005_set_config();
	spi_register_board_info(n800_spi_board_info,
				ARRAY_SIZE(n800_spi_board_info));

	omap_serial_init();

#ifdef CONFIG_MENELAUS
	omap_register_i2c_bus(1, 400, n8x0_i2c_board_info_menelaus,
			      ARRAY_SIZE(n8x0_i2c_board_info_menelaus));
#endif

	omap_register_i2c_bus(2, 400, n8x0_i2c_board_info_2,
			      ARRAY_SIZE(n8x0_i2c_board_info_2));

	i2c_register_board_info(2, n810_i2c_board_info_2,
    				ARRAY_SIZE(n810_i2c_board_info_2));

	n8x0_mipid_init();
	n8x0_blizzard_init();

	n8x0_onenand_init();
}

MACHINE_START(NOKIA_N800, "Nokia N800")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= n8x0_map_io,
	.init_irq	= n8x0_init_irq,
	.init_machine	= n8x0_init_machine,
	.timer		= &omap_timer,
MACHINE_END

MACHINE_START(NOKIA_N810, "Nokia N810")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= n8x0_map_io,
	.init_irq	= n8x0_init_irq,
	.init_machine	= n8x0_init_machine,
	.timer		= &omap_timer,
MACHINE_END

MACHINE_START(NOKIA_N810_WIMAX, "Nokia N810 WiMAX")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= n8x0_map_io,
	.init_irq	= n8x0_init_irq,
	.init_machine	= n8x0_init_machine,
	.timer		= &omap_timer,
MACHINE_END
