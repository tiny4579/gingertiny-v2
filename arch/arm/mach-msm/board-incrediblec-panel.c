/* arch/arm/mach-msm/board-incrediblec-panel.c
 *
 * Copyright (c) 2009 Google Inc.
 * Copyright (c) 2009 HTC.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author: Chad0989
 *
 * Based on the Samsung tl2786a AMOLED and Sony s6d16a0x21 SLCD
 * drivers originally authored by HTC
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <asm/io.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>
#include <mach/vreg.h>
#include <linux/spi/spi.h>
#include <mach/atmega_microp.h>
#include "proc_comm.h"

#include "board-incrediblec.h"
#include "devices.h"

#define SPI_CONFIG		(0x00000000)
#define SPI_IO_CONTROL		(0x00000004)
#define SPI_OPERATIONAL		(0x00000030)
#define SPI_ERROR_FLAGS_EN	(0x00000038)
#define SPI_ERROR_FLAGS		(0x00000038)
#define SPI_OUTPUT_FIFO		(0x00000100)

#define SAMSUNG_PANEL		0
/*Bitwise mask for SONY PANEL ONLY*/
#define SONY_PANEL		0x1		/*Set bit 0 as 1 when it is SONY PANEL*/
#define SONY_PWM_SPI		0x2		/*Set bit 1 as 1 as PWM_SPI mode, otherwise it is PWM_MICROP mode*/
#define SONY_GAMMA		0x4		/*Set bit 2 as 1 when panel contains GAMMA table in its NVM*/
#define SONY_RGB666		0x8		/*Set bit 3 as 1 when panel is 18 bit, otherwise it is 16 bit*/

extern int panel_type;
extern int qspi_send(uint32_t id, uint8_t data);
extern int qspi_send_9bit(unsigned char id, unsigned data);

static int is_sony_spi(void)
{
	return (panel_type & SONY_PWM_SPI ? 1 : 0);
}

static int is_sony_with_gamma(void)
{
	return (panel_type & SONY_GAMMA ? 1 : 0);
}

static int is_sony_RGB666(void)
{
	return (panel_type & SONY_RGB666 ? 1 : 0);
}

static int lcm_write_cmd(uint32_t reg, uint32_t data)
{
	int ret = -1;

	if (reg < 0)
		return -EIO;
	ret = qspi_send(0x0, reg);
	if (ret)
		goto err_lcm_writecmd;

	ret = qspi_send(0x1, data);
	if (ret)
		goto err_lcm_writecmd;
	return 0;

err_lcm_writecmd:
	printk(KERN_ERR "%s: Failure on sending SPI commands", __func__);
	return ret;
}

struct lcm_cmd {
        int reg;
        uint32_t val;
};

static int lcm_write_tb(struct lcm_cmd cmd_table[], unsigned size)
{
        int i;

        for (i = 0; i < size; i++) {
                if (panel_type != SAMSUNG_PANEL)
                qspi_send_9bit(cmd_table[i].reg, cmd_table[i].val);
                else
                lcm_write_cmd(cmd_table[i].reg, cmd_table[i].val);
        }
        return 0;
}

static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct lcm_cmd samsung_oled_rgb565_init_table[] = {
	{ 0x31, 0x08 },
	{ 0x32, 0x14 },
	{ 0x30, 0x2 },
	{ 0x27, 0x1 },
	{ 0x12, 0x8 },
	{ 0x13, 0x8 },
	{ 0x15, 0x0 },
	{ 0x16, 0x02 },
	{ 0x39, 0x44 },
	{ 0x17, 0x22 },
	{ 0x18, 0x33 },
	{ 0x19, 0x3 },
	{ 0x1A, 0x1 },
	{ 0x22, 0xA4 },
	{ 0x23, 0x0 },
	{ 0x26, 0xA0 },
	{ 0x1D, 0xA0 },
};

static struct lcm_cmd samsung_oled_rgb666_init_table[] = {
	{ 0x31, 0x08 },
	{ 0x32, 0x14 },
	{ 0x30, 0x2 },
	{ 0x27, 0x1 },
	{ 0x12, 0x8 },
	{ 0x13, 0x8 },
	{ 0x15, 0x0 },
	{ 0x16, 0x01 },
	{ 0x16, 0x01 },
	{ 0x39, 0x44 },
	{ 0x17, 0x22 },
	{ 0x18, 0x33 },
	{ 0x19, 0x3 },
	{ 0x1A, 0x1 },
	{ 0x22, 0xA4 },
	{ 0x23, 0x0 },
	{ 0x26, 0xA0 },
	{ 0x1D, 0xA0 },
};

#define OLED_GAMMA_TABLE_SIZE		(7 * 3)
static struct lcm_cmd samsung_oled_gamma_table[][OLED_GAMMA_TABLE_SIZE] = {
	/* level 10 */
	{
		{0x40, 0x0},
		{0x41, 0x3},
		{0x42, 0x40},
		{0x43, 0x39},
		{0x44, 0x32},
		{0x45, 0x2e},
		{0x46, 0xc },
		{0x50, 0x0 },
		{0x51, 0x0 },
		{0x52, 0x0 },
		{0x53, 0x00},
		{0x54, 0x26},
		{0x55, 0x2d},
		{0x56, 0xb },
		{0x60, 0x0 },
		{0x61, 0x3f},
		{0x62, 0x40},
		{0x63, 0x38},
		{0x64, 0x31},
		{0x65, 0x2d},
		{0x66, 0x12},
	},

	/*level 40*/
	{
		{0x40, 0x0 },
		{0x41, 0x3f},
		{0x42, 0x3e},
		{0x43, 0x2e},
		{0x44, 0x2d},
		{0x45, 0x28},
		{0x46, 0x21},
		{0x50, 0x0 },
		{0x51, 0x0 },
		{0x52, 0x0 },
		{0x53, 0x21},
		{0x54, 0x2a},
		{0x55, 0x28},
		{0x56, 0x20},
		{0x60, 0x0 },
		{0x61, 0x3f},
		{0x62, 0x3e},
		{0x63, 0x2d},
		{0x64, 0x2b},
		{0x65, 0x26},
		{0x66, 0x2d},
	},

	/*level 70*/
	{
		{0x40, 0x0 },
		{0x41, 0x3f},
		{0x42, 0x35},
		{0x43, 0x2c},
		{0x44, 0x2b},
		{0x45, 0x26},
		{0x46, 0x29},
		{0x50, 0x0 },
		{0x51, 0x0 },
		{0x52, 0x0 },
		{0x53, 0x25},
		{0x54, 0x29},
		{0x55, 0x26},
		{0x56, 0x28},
		{0x60, 0x0 },
		{0x61, 0x3f},
		{0x62, 0x34},
		{0x63, 0x2b},
		{0x64, 0x2a},
		{0x65, 0x23},
		{0x66, 0x37},
	},

	/*level 100*/
	{
		{0x40, 0x0 },
		{0x41, 0x3f},
		{0x42, 0x30},
		{0x43, 0x2a},
		{0x44, 0x2b},
		{0x45, 0x24},
		{0x46, 0x2f},
		{0x50, 0x0 },
		{0x51, 0x0 },
		{0x52, 0x0 },
		{0x53, 0x25},
		{0x54, 0x29},
		{0x55, 0x24},
		{0x56, 0x2e},
		{0x60, 0x0 },
		{0x61, 0x3f},
		{0x62, 0x2f},
		{0x63, 0x29},
		{0x64, 0x29},
		{0x65, 0x21},
		{0x66, 0x3f},
	},

	/*level 130*/
	{
		{0x40, 0x0 },
		{0x41, 0x3f},
		{0x42, 0x2e},
		{0x43, 0x29},
		{0x44, 0x2a},
		{0x45, 0x23},
		{0x46, 0x34},
		{0x50, 0x0 },
		{0x51, 0x0 },
		{0x52, 0xa },
		{0x53, 0x25},
		{0x54, 0x28},
		{0x55, 0x23},
		{0x56, 0x33},
		{0x60, 0x0 },
		{0x61, 0x3f},
		{0x62, 0x2d},
		{0x63, 0x28},
		{0x64, 0x27},
		{0x65, 0x20},
		{0x66, 0x46},
	},

	/*level 160*/
	{
		{0x40, 0x0 },
		{0x41, 0x3f},
		{0x42, 0x2b},
		{0x43, 0x29},
		{0x44, 0x28},
		{0x45, 0x23},
		{0x46, 0x38},
		{0x50, 0x0 },
		{0x51, 0x0 },
		{0x52, 0xb },
		{0x53, 0x25},
		{0x54, 0x27},
		{0x55, 0x23},
		{0x56, 0x37},
		{0x60, 0x0 },
		{0x61, 0x3f},
		{0x62, 0x29},
		{0x63, 0x28},
		{0x64, 0x25},
		{0x65, 0x20},
		{0x66, 0x4b},
	},

	/*level 190*/
	{
		{0x40, 0x0 },
		{0x41, 0x3f},
		{0x42, 0x29},
		{0x43, 0x29},
		{0x44, 0x27},
		{0x45, 0x22},
		{0x46, 0x3c},
		{0x50, 0x0 },
		{0x51, 0x0 },
		{0x52, 0x10},
		{0x53, 0x26},
		{0x54, 0x26},
		{0x55, 0x22},
		{0x56, 0x3b},
		{0x60, 0x0 },
		{0x61, 0x3f},
		{0x62, 0x28},
		{0x63, 0x28},
		{0x64, 0x24},
		{0x65, 0x1f},
		{0x66, 0x50},
	},

	/*level 220*/
	{
		{0x40, 0x0 },
		{0x41, 0x3f},
		{0x42, 0x28},
		{0x43, 0x28},
		{0x44, 0x28},
		{0x45, 0x20},
		{0x46, 0x40},
		{0x50, 0x0 },
		{0x51, 0x0 },
		{0x52, 0x11},
		{0x53, 0x25},
		{0x54, 0x27},
		{0x55, 0x20},
		{0x56, 0x3f},
		{0x60, 0x0 },
		{0x61, 0x3f},
		{0x62, 0x27},
		{0x63, 0x26},
		{0x64, 0x26},
		{0x65, 0x1c},
		{0x66, 0x56},
	},

	/*level 250*/
	{
		{0x40, 0x0 },
		{0x41, 0x3f},
		{0x42, 0x2a},
		{0x43, 0x27},
		{0x44, 0x27},
		{0x45, 0x1f},
		{0x46, 0x44},
		{0x50, 0x0 },
		{0x51, 0x0 },
		{0x52, 0x17},
		{0x53, 0x24},
		{0x54, 0x26},
		{0x55, 0x1f},
		{0x56, 0x43},
		{0x60, 0x0 },
		{0x61, 0x3f},
		{0x62, 0x2a},
		{0x63, 0x25},
		{0x64, 0x24},
		{0x65, 0x1b},
		{0x66, 0x5c},
	},
};

#define SAMSUNG_OLED_NUM_LEVELS		ARRAY_SIZE(samsung_oled_gamma_table)

#define SAMSUNG_OLED_MIN_VAL		10
#define SAMSUNG_OLED_MAX_VAL		250
#define SAMSUNG_OLED_DEFAULT_VAL	(SAMSUNG_OLED_MIN_VAL +		\
					 (SAMSUNG_OLED_MAX_VAL -	\
					  SAMSUNG_OLED_MIN_VAL) / 2)

#define SAMSUNG_OLED_LEVEL_STEP		((SAMSUNG_OLED_MAX_VAL -	\
					  SAMSUNG_OLED_MIN_VAL) /	\
					 (SAMSUNG_OLED_NUM_LEVELS - 1))

#define LCM_GPIO_CFG(gpio, func) \
PCOM_GPIO_CFG(gpio, func, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)
static uint32_t display_on_gpio_table[] = {
	LCM_GPIO_CFG(INCREDIBLEC_LCD_R0, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_R1, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_R2, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_R3, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_R4, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_R5, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_G0, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_G1, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_G2, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_G3, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_G4, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_G5, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_B0, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_B1, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_B2, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_B3, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_B4, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_B5, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_PCLK, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_VSYNC, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_HSYNC, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_DE, 1),
};

static uint32_t display_off_gpio_table[] = {
	LCM_GPIO_CFG(INCREDIBLEC_LCD_R0, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_R1, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_R2, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_R3, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_R4, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_R5, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_G0, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_G1, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_G2, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_G3, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_G4, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_G5, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_B0, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_B1, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_B2, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_B3, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_B4, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_B5, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_PCLK, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_VSYNC, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_HSYNC, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_DE, 0),
};

static uint32_t sony_display_on_gpio_table[] = {
	LCM_GPIO_CFG(INCREDIBLEC_SPI_CLK, 1),
	LCM_GPIO_CFG(INCREDIBLEC_SPI_CS, 1),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_ID0, 1),
	LCM_GPIO_CFG(INCREDIBLEC_SPI_DO, 1),
};

static uint32_t sony_display_off_gpio_table[] = {
	LCM_GPIO_CFG(INCREDIBLEC_SPI_CLK, 0),
	LCM_GPIO_CFG(INCREDIBLEC_SPI_CS, 0),
	LCM_GPIO_CFG(INCREDIBLEC_LCD_ID0, 0),
	LCM_GPIO_CFG(INCREDIBLEC_SPI_DO, 0),
};
#undef LCM_GPIO_CFG

#define SONYWVGA_BR_DEF_USER_PWM         143
#define SONYWVGA_BR_MIN_USER_PWM         30
#define SONYWVGA_BR_MAX_USER_PWM         255
#define SONYWVGA_BR_DEF_PANEL_PWM        153
#define SONYWVGA_BR_MIN_PANEL_PWM        8
#define SONYWVGA_BR_MAX_PANEL_PWM        255
#define SONYWVGA_BR_DEF_PANEL_UP_PWM    132
#define SONYWVGA_BR_MIN_PANEL_UP_PWM    9
#define SONYWVGA_BR_MAX_PANEL_UP_PWM    255

static DEFINE_MUTEX(panel_lock);
static uint8_t new_val = SAMSUNG_OLED_DEFAULT_VAL;
static uint8_t last_val = SAMSUNG_OLED_DEFAULT_VAL;
static uint8_t last_val_pwm = SONYWVGA_BR_DEF_PANEL_PWM;
static uint8_t table_sel_vals[] = { 0x43, 0x34 };
static int table_sel_idx;
static uint8_t tft_panel_on;
static struct wake_lock panel_idle_lock;

static int panel_gpio_switch(int on)
{
	if (on) {
		config_gpio_table(display_on_gpio_table,
			ARRAY_SIZE(display_on_gpio_table));

		if(panel_type != SAMSUNG_PANEL) {
			config_gpio_table(sony_display_on_gpio_table,
							  ARRAY_SIZE(sony_display_on_gpio_table));
		}
	}
	else {
		int i;

		config_gpio_table(display_off_gpio_table,
			ARRAY_SIZE(display_off_gpio_table));

		for (i = INCREDIBLEC_LCD_R0; i <= INCREDIBLEC_LCD_R5; i++)
			gpio_set_value(i, 0);
		for (i = INCREDIBLEC_LCD_G0; i <= INCREDIBLEC_LCD_G5; i++)
			gpio_set_value(i, 0);
		for (i = INCREDIBLEC_LCD_B0; i <= INCREDIBLEC_LCD_DE; i++)
			gpio_set_value(i, 0);

		if(panel_type != SAMSUNG_PANEL) {
			config_gpio_table(sony_display_off_gpio_table,
							  ARRAY_SIZE(sony_display_off_gpio_table));
		}
	}
	return 0;
}

static void gamma_table_bank_select(void)
{
	lcm_write_cmd(0x39, table_sel_vals[table_sel_idx]);
	table_sel_idx ^= 1;
}

static void samsung_oled_set_gamma_val(int val)
{
	int i;
	int level;
	int frac;

	val = clamp(val, SAMSUNG_OLED_MIN_VAL, SAMSUNG_OLED_MAX_VAL);
	val = (val / 2) * 2;

	if (val < 31) {
		val = 20;
	}

	level = (val - SAMSUNG_OLED_MIN_VAL) / SAMSUNG_OLED_LEVEL_STEP;
	frac = (val - SAMSUNG_OLED_MIN_VAL) % SAMSUNG_OLED_LEVEL_STEP;

	for (i = 0; i < OLED_GAMMA_TABLE_SIZE; ++i) {
		unsigned int v1;
		unsigned int v2 = 0;
		u8 v;
		if (frac == 0) {
			v = samsung_oled_gamma_table[level][i].val;
		} else {

			v1 = samsung_oled_gamma_table[level][i].val;
			v2 = samsung_oled_gamma_table[level+1][i].val;
			v = (v1 * (SAMSUNG_OLED_LEVEL_STEP - frac) +
			     v2 * frac) / SAMSUNG_OLED_LEVEL_STEP;
		}
		lcm_write_cmd(samsung_oled_gamma_table[level][i].reg, v);
	}
	gamma_table_bank_select();
	last_val = val;
}

static int panel_power(int on)
{
	static struct vreg *vreg_lcm_2v6;
	if (!vreg_lcm_2v6) {
		vreg_lcm_2v6 = vreg_get(0, "gp1");
		if (IS_ERR(vreg_lcm_2v6))
			return -EINVAL;
	}

	if (on) {
		unsigned id, on = 1;
		panel_gpio_switch(1);
		id = PM_VREG_PDOWN_CAM_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on, &id);
		vreg_enable(vreg_lcm_2v6);
		udelay(50);
		gpio_set_value(INCREDIBLEC_LCD_RST_ID1, 1);
		udelay(50);
		gpio_set_value(INCREDIBLEC_LCD_RST_ID1, 0);
		udelay(20);
		gpio_set_value(INCREDIBLEC_LCD_RST_ID1, 1);
		usleep_range(1000, 1050);
	} else {
		unsigned id, on = 0;

		gpio_set_value(INCREDIBLEC_LCD_RST_ID1, 0);
		id = PM_VREG_PDOWN_CAM_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on, &id);
		vreg_disable(vreg_lcm_2v6);
		panel_gpio_switch(0);
	}

return 0;

}

static int samsung_oled_panel_init(struct msm_lcdc_panel_ops *ops)
{
	pr_info("%s: +()\n", __func__);
	panel_power(1);
	wake_lock(&panel_idle_lock);
	mutex_lock(&panel_lock);
	if (system_rev < 1)
		lcm_write_tb(samsung_oled_rgb565_init_table, ARRAY_SIZE(samsung_oled_rgb565_init_table));
	else
		lcm_write_tb(samsung_oled_rgb666_init_table, ARRAY_SIZE(samsung_oled_rgb666_init_table));
	usleep_range(1000, 1050);
	gamma_table_bank_select();
	mutex_unlock(&panel_lock);
        wake_unlock(&panel_idle_lock);
	pr_info("%s: -()\n", __func__);
	return 0;
}

static int samsung_oled_panel_unblank(struct msm_lcdc_panel_ops *ops)
{
	pr_info("%s: +()\n", __func__);

        wake_lock(&panel_idle_lock);
	mutex_lock(&panel_lock);
	table_sel_idx = 0;
	gamma_table_bank_select();
	samsung_oled_set_gamma_val(last_val);
	qspi_send(0, 0xef);
        qspi_send(1, 0xd0);
        qspi_send(1, 0xe8);
	lcm_write_cmd(0x14, 0x03);
	mutex_unlock(&panel_lock);
	wake_unlock(&panel_idle_lock);

	pr_info("%s: -()\n", __func__);
	return 0;
}

static int samsung_oled_panel_blank(struct msm_lcdc_panel_ops *ops)
{
	pr_info("%s: +()\n", __func__);
	mutex_lock(&panel_lock);
	lcm_write_cmd(0x14, 0x0);
	lcm_write_cmd(0x1d, 0xa1);
        mutex_unlock(&panel_lock);
	panel_power(0);
	mdelay(5);
	pr_info("%s: -()\n", __func__);
	return 0;
}

static int
sonywvga_panel_shrink_pwm(int brightness)
{
	int level;
	unsigned int min_pwm, def_pwm, max_pwm;

	if(!is_sony_spi()) {
		min_pwm = SONYWVGA_BR_MIN_PANEL_UP_PWM;
		def_pwm = SONYWVGA_BR_DEF_PANEL_UP_PWM;
		max_pwm = SONYWVGA_BR_MAX_PANEL_UP_PWM;
	} else {
		min_pwm = SONYWVGA_BR_MIN_PANEL_PWM;
		def_pwm = SONYWVGA_BR_DEF_PANEL_PWM;
		max_pwm = SONYWVGA_BR_MAX_PANEL_PWM;
	}

	if (brightness <= SONYWVGA_BR_DEF_USER_PWM) {
		if (brightness <= SONYWVGA_BR_MIN_USER_PWM)
			level = min_pwm;
		else
			level = (def_pwm - min_pwm) *
			(brightness - SONYWVGA_BR_MIN_USER_PWM) /
			(SONYWVGA_BR_DEF_USER_PWM - SONYWVGA_BR_MIN_USER_PWM) +
			min_pwm;
	} else
		level = (max_pwm - def_pwm) *
		(brightness - SONYWVGA_BR_DEF_USER_PWM) /
		(SONYWVGA_BR_MAX_USER_PWM - SONYWVGA_BR_DEF_USER_PWM) +
		def_pwm;

	return level;
}

//2010-5-21 Rev May21-2(Wx, Wy)=(0.306, 0.315) Gamma = 2.2
static struct lcm_cmd SONY_TFT_INIT_TABLE[]={
	//Change to level 2
	{0, 0xF1},
	{1, 0x5A},
	{1, 0x5A},
	// FAh RGB
	{0, 0xFA},
	// Red
	{1, 0x32},
	{1, 0x3F},
	{1, 0x3F},
	{1, 0x29},
	{1, 0x3E},
	{1, 0x3C},
	{1, 0x3D},
	{1, 0x2C},
	{1, 0x27},
	{1, 0x3D},
	{1, 0x2E},
	{1, 0x31},
	{1, 0x3A},
	{1, 0x34},
	{1, 0x36},
	// Green
	{1, 0x1A},
	{1, 0x3F},
	{1, 0x3F},
	{1, 0x2E},
	{1, 0x40},
	{1, 0x3C},
	{1, 0x3C},
	{1, 0x2B},
	{1, 0x25},
	{1, 0x39},
	{1, 0x25},
	{1, 0x23},
	{1, 0x2A},
	{1, 0x20},
	{1, 0x22},
	// Blue
	{1, 0x00},
	{1, 0x3F},
	{1, 0x3F},
	{1, 0x2F},
	{1, 0x3E},
	{1, 0x3C},
	{1, 0x3C},
	{1, 0x2A},
	{1, 0x23},
	{1, 0x35},
	{1, 0x1E},
	{1, 0x18},
	{1, 0x1C},
	{1, 0x0C},
	{1, 0x0E},
	// FBh RGB
	{0, 0xFB},
	// Red
	{1, 0x00},
	{1, 0x0D},
	{1, 0x09},
	{1, 0x0C},
	{1, 0x26},
	{1, 0x2E},
	{1, 0x31},
	{1, 0x22},
	{1, 0x19},
	{1, 0x33},
	{1, 0x22},
	{1, 0x23},
	{1, 0x21},
	{1, 0x17},
	{1, 0x00},
	// Green
	{1, 0x00},
	{1, 0x25},
	{1, 0x1D},
	{1, 0x1F},
	{1, 0x35},
	{1, 0x3C},
	{1, 0x3A},
	{1, 0x26},
	{1, 0x1B},
	{1, 0x34},
	{1, 0x23},
	{1, 0x23},
	{1, 0x1F},
	{1, 0x12},
	{1, 0x00},
	// Blue
	{1, 0x00},
	{1, 0x3F},
	{1, 0x31},
	{1, 0x33},
	{1, 0x43},
	{1, 0x48},
	{1, 0x41},
	{1, 0x2A},
	{1, 0x1D},
	{1, 0x35},
	{1, 0x23},
	{1, 0x23},
	{1, 0x21},
	{1, 0x10},
	{1, 0x00},
	// F3h Power control
	{0, 0xF3},
	{1, 0x00},
	{1, 0x10},
	{1, 0x25},
	{1, 0x01},
	{1, 0x2D},
	{1, 0x2D},
	{1, 0x24},
	{1, 0x2D},
	{1, 0x10},
	{1, 0x10},
	{1, 0x0A},
	{1, 0x37},
	// F4h VCOM Control
	{0, 0xF4},
	{1, 0x88},
	{1, 0x20},
	{1, 0x00},
	{1, 0xAF},
	{1, 0x64},
	{1, 0x00},
	{1, 0xAA},
	{1, 0x64},
	{1, 0x00},
	{1, 0x00},
	//Change to level 1
	{0, 0xF0},
	{1, 0x5A},
	{1, 0x5A},
};

static struct lcm_cmd SONY_TFT_GAMMA_TABLE[]={
	{0, 0xf0},
	{1, 0x5a},
	{1, 0x5a},
	{0, 0xf1},
	{1, 0x5a},
	{1, 0x5a},
	{0, 0xd0},
	{1, 0x5a},
	{1, 0x5a},
	{0, 0xc2},
	{1, 0x53},
	{1, 0x12},
	{0, 0x51},
};

static void sonywvga_set_gamma_val(int val)
{
	uint8_t data[4] = {0, 0, 0, 0};

	if (!is_sony_spi()) {
		//turn on backlight
		data[0] = 5;
		data[1] = sonywvga_panel_shrink_pwm(val);
		data[3] = 1;
		microp_i2c_write(0x25, data, 4);
	} else {
		lcm_write_tb(SONY_TFT_GAMMA_TABLE, ARRAY_SIZE(SONY_TFT_GAMMA_TABLE));
		qspi_send_9bit(0x1, sonywvga_panel_shrink_pwm(val));
		qspi_send_9bit(0x0, 0x53);
		qspi_send_9bit(0x1, 0x24);
	}
	last_val_pwm = val;
}

static int sony_tft_panel_unblank(struct msm_lcdc_panel_ops *ops)
{
	wake_lock(&panel_idle_lock);
	mutex_lock(&panel_lock);
	qspi_send_9bit(0x0, 0x29);
	usleep_range(1000, 5000);
	//init gamma setting
	if(!is_sony_with_gamma())
		lcm_write_tb(SONY_TFT_INIT_TABLE,
					 ARRAY_SIZE(SONY_TFT_INIT_TABLE));
	sonywvga_set_gamma_val(last_val_pwm);
	tft_panel_on = 1;

	mutex_unlock(&panel_lock);
	wake_unlock(&panel_idle_lock);
	return 0;
}

static int sony_tft_panel_blank(struct msm_lcdc_panel_ops *ops)
{
	uint8_t data[4] = {0, 0, 0, 0};
	pr_info("%s: +()\n", __func__);

	mutex_lock(&panel_lock);
	qspi_send_9bit(0x0, 0x28);
	qspi_send_9bit(0x0, 0x10);
	usleep_range(1000, 5000);
	tft_panel_on = 0;
	mutex_unlock(&panel_lock);
	panel_power(0);

	if (!is_sony_spi()) {
		data[0] = 5;
		data[1] = 0;
		data[3] = 1;
		microp_i2c_write(0x25, data, 4);
	}

	return 0;
}

static int sony_tft_panel_init(struct msm_lcdc_panel_ops *ops)
{
	wake_lock(&panel_idle_lock);
	mutex_lock(&panel_lock);
	panel_power(1);
	usleep_range(1000, 5000);
	qspi_send_9bit(0x0, 0x11);
	usleep_range(1000, 5000);
	qspi_send_9bit(0x0, 0x3a);
	if (is_sony_RGB666())
		qspi_send_9bit(0x1, 0x06);
	else
		qspi_send_9bit(0x1, 0x05);
	mutex_unlock(&panel_lock);
	wake_unlock(&panel_idle_lock);

	return 0;
}

static struct msm_lcdc_panel_ops incrediblec_lcdc_amoled_panel_ops = {
	.init		= samsung_oled_panel_init,
	.blank		= samsung_oled_panel_blank,
	.unblank	= samsung_oled_panel_unblank,
};

static struct msm_lcdc_panel_ops incrediblec_lcdc_tft_panel_ops = {
	.init		= sony_tft_panel_init,
	.blank		= sony_tft_panel_blank,
	.unblank	= sony_tft_panel_unblank,
};


static struct msm_lcdc_timing incrediblec_lcdc_amoled_timing = {
		.clk_rate		= 24576000,
		.hsync_pulse_width	= 4,
		.hsync_back_porch	= 4,
		.hsync_front_porch	= 8,
		.hsync_skew		= 0,
		.vsync_pulse_width	= 2,
		.vsync_back_porch	= 6,
		.vsync_front_porch	= 8,
		.vsync_act_low		= 1,
		.hsync_act_low		= 1,
		.den_act_low		= 1,
};

static struct msm_lcdc_timing incrediblec_lcdc_tft_timing = {
		.clk_rate		= 24576000,
		.hsync_pulse_width	= 2,
		.hsync_back_porch	= 18,
		.hsync_front_porch	= 20,
		.hsync_skew		= 0,
		.vsync_pulse_width	= 2,
		.vsync_back_porch	= 4,
		.vsync_front_porch	= 4,
		.vsync_act_low		= 1,
		.hsync_act_low		= 1,
		.den_act_low		= 0,
};

static struct msm_fb_data incrediblec_lcdc_fb_data = {
		.xres		= 480,
		.yres		= 800,
		.width		= 48,
		.height		= 80,
		.output_format	= MSM_MDP_OUT_IF_FMT_RGB565,
};

static struct msm_lcdc_platform_data incrediblec_lcdc_amoled_platform_data = {
	.panel_ops	= &incrediblec_lcdc_amoled_panel_ops,
	.timing		= &incrediblec_lcdc_amoled_timing,
	.fb_id		= 0,
	.fb_data	= &incrediblec_lcdc_fb_data,
	.fb_resource	= &resources_msm_fb[0],
};

static struct msm_lcdc_platform_data incrediblec_lcdc_tft_platform_data = {
	.panel_ops	= &incrediblec_lcdc_tft_panel_ops,
	.timing		= &incrediblec_lcdc_tft_timing,
	.fb_id		= 0,
	.fb_data	= &incrediblec_lcdc_fb_data,
	.fb_resource	= &resources_msm_fb[0],
};

static struct platform_device incrediblec_lcdc_amoled_device = {
	.name	= "msm_mdp_lcdc",
	.id	= -1,
	.dev	= {
		.platform_data = &incrediblec_lcdc_amoled_platform_data,
	},
};

static struct platform_device incrediblec_lcdc_tft_device = {
	.name	= "msm_mdp_lcdc",
	.id	= -1,
	.dev	= {
		.platform_data = &incrediblec_lcdc_tft_platform_data,
	},
};

static void incrediblec_brightness_set(struct led_classdev *led_cdev,
				    enum led_brightness val)
{
	led_cdev->brightness = val;
	if(panel_type != SAMSUNG_PANEL) {
		mutex_lock(&panel_lock);
		if(tft_panel_on)
			sonywvga_set_gamma_val(val);
		else
			last_val_pwm = val;
		mutex_unlock(&panel_lock);
	} else {
	mutex_lock(&panel_lock);
	new_val = val;
	samsung_oled_set_gamma_val(new_val);
	mutex_unlock(&panel_lock);
	}
}

static struct led_classdev incrediblec_brightness_led = {
	.name = "lcd-backlight",
	.brightness = LED_FULL,
	.brightness_set = incrediblec_brightness_set,
};

int __init incrediblec_init_panel(void)
{
	int ret;

	if (system_rev >= 1) {
		/* CDMA version (except for EVT1) supports RGB666 */
		incrediblec_lcdc_fb_data.output_format = MSM_MDP_OUT_IF_FMT_RGB666;
	}

	ret = platform_device_register(&msm_device_mdp);
	if (ret != 0)
		return ret;

	if (gpio_get_value(INCREDIBLEC_LCD_ID0)) {
		pr_info("%s: tft panel\n", __func__);
		if (gpio_get_value(INCREDIBLEC_LCD_RST_ID1))
		ret = platform_device_register(&incrediblec_lcdc_tft_device);
	} else {
		pr_info("%s: amoled panel\n", __func__);
		ret = platform_device_register(&incrediblec_lcdc_amoled_device);	
	}

	panel_gpio_switch(1);

	wake_lock_init(&panel_idle_lock, WAKE_LOCK_SUSPEND,
			"backlight_present");

	if (ret != 0)
		return ret;

	ret = led_classdev_register(NULL, &incrediblec_brightness_led);
	if (ret != 0) {
		pr_err("%s: Cannot register brightness led\n", __func__);
		return ret;
	}

	return 0;
}

device_initcall(incrediblec_init_panel);
