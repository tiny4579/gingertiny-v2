/*
 *
 * /arch/arm/mach-msm/include/mach/htc_headset_h2w.h
 *
 * HTC 2 Wire headset driver.
 *
 * Copyright (C) 2010 HTC, Inc.
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
 */

#ifndef HTC_HEADSET_H2W_H
#define HTC_HEADSET_H2W_H

#define H2W_DETECT_DELAY	msecs_to_jiffies(200)
#define BUTTON_H2W_DELAY	msecs_to_jiffies(10)

#define RESEND_DELAY		(3)	/* ms */
#define MAX_ACK_RESEND_TIMES	(6)	/* Follow specification */
#define MAX_HOST_RESEND_TIMES	(3)	/* Follow specification */
#define MAX_HYGEIA_RESEND_TIMES	(5)

#define H2W_ASCR_DEVICE_INI	(0x01)
#define H2W_ASCR_ACT_EN		(0x02)
#define H2W_ASCR_PHONE_IN	(0x04)
#define H2W_ASCR_RESET		(0x08)
#define H2W_ASCR_AUDIO_IN	(0x10)

#define H2W_LED_OFF		(0x0)
#define H2W_LED_BKL		(0x1)
#define H2W_LED_MTL		(0x2)

#define H2W_PhoneIn		(0x01)
#define H2W_MuteLed		(0x02)

enum {
	H2W_GPIO	= 0,
	H2W_UART1	= 1,
	H2W_UART3	= 2,
	H2W_BT		= 3
};

enum {
	/* === system group 0x0000~0x00FF === */
	/* (R) Accessory type register */
	H2W_SYSTEM		= 0x0000,
	/* (R) Maximum group address */
	H2W_MAX_GP_ADD		= 0x0001,
	/* (R/W) Accessory system control register0 */
	H2W_ASCR0		= 0x0002,

	/* === key group 0x0100~0x01FF === */
	/* (R) Key group maximum sub address */
	H2W_KEY_MAXADD		= 0x0100,
	/* (R) ASCII key press down flag */
	H2W_ASCII_DOWN		= 0x0101,
	/* (R) ASCII key release up flag */
	H2W_ASCII_UP		= 0x0102,
	/* (R) Function key status flag */
	H2W_FNKEY_UPDOWN	= 0x0103,
	/* (R/W) Key device status */
	H2W_KD_STATUS		= 0x0104,

	/* === led group 0x0200~0x02FF === */
	/* (R) LED group maximum sub address */
	H2W_LED_MAXADD		= 0x0200,
	/* (R/W) LED control register0 */
	H2W_LEDCT0		= 0x0201,

	/* === crdl group 0x0300~0x03FF === */
	/* (R) Cardle group maximum sub address */
	H2W_CRDL_MAXADD		= 0x0300,
	/* (R/W) Cardle group function control register0 */
	H2W_CRDLCT0		= 0x0301,

	/* === car kit group 0x0400~0x04FF === */
	H2W_CARKIT_MAXADD	= 0x0400,

	/* === usb host group 0x0500~0x05FF === */
	H2W_USBHOST_MAXADD	= 0x0500,

	/* === medical group 0x0600~0x06FF === */
	H2W_MED_MAXADD		= 0x0600,
	H2W_MED_CONTROL		= 0x0601,
	H2W_MED_IN_DATA		= 0x0602,
};

enum {
	H2W_500KHz	= 1,
	H2W_250KHz	= 2,
	H2W_166KHz	= 3,
	H2W_125KHz	= 4,
	H2W_100KHz	= 5,
	H2W_83KHz	= 6,
	H2W_71KHz	= 7,
	H2W_62KHz	= 8,
	H2W_55KHz	= 9,
	H2W_50KHz	= 10,
};

enum {
	HS_H2W_KEY_INVALID	= -1,
	HS_H2W_KEY_PLAY		= 0,
	HS_H2W_KEY_FORWARD	= 1,
	HS_H2W_KEY_BACKWARD	= 2,
	HS_H2W_KEY_VOLUP	= 3,
	HS_H2W_KEY_VOLDOWN	= 4,
	HS_H2W_KEY_PICKUP	= 5,
	HS_H2W_KEY_HANGUP	= 6,
	HS_H2W_KEY_MUTE		= 7,
	HS_H2W_KEY_HOLD		= 8,
	HS_H2W_NUM_KEYFUNC	= 9,
};

struct htc_h2w_info {
	/* System */
	unsigned char CLK_SP;
	int SLEEP_PR;
	unsigned char HW_REV;
	int AUDIO_DEVICE;
	unsigned char ACC_CLASS;
	unsigned char MAX_GP_ADD;

	/* Key */
	int KEY_MAXADD;
	int ASCII_DOWN;
	int ASCII_UP;
	int FNKEY_UPDOWN;
	int KD_STATUS;

	/* LED */
	int LED_MAXADD;
	int LEDCT0;

	/* Medical */
	int MED_MAXADD;
	unsigned char AP_ID;
	unsigned char AP_EN;
	unsigned char DATA_EN;
};

struct htc_headset_h2w_platform_data {
	int cable_in1;
	int cable_in2;
	int h2w_clk;
	int h2w_data;
	int debug_uart;

	void (*power)(int);
	void (*config)(int);
	void (*set_dat)(int);
	void (*set_clk)(int);
	void (*set_dat_dir)(int);
	void (*set_clk_dir)(int);
	int (*get_dat)(void);
	int (*get_clk)(void);
};

struct htc_headset_h2w_info {
	struct htc_headset_h2w_platform_data pdata;
	int speed;

	unsigned int irq;
	unsigned int irq_btn;

	int h2w_dev_type;

	int mic_switch_flag;
	int rc_flag;

	struct class *htc_accessory_class;
	struct device *mic_dev;
	struct device *mute_dev;
	struct device *phonein_dev;

	struct htc_h2w_info h2w_info;

	struct mutex mutex_lock;
	struct mutex mutex_h2w_lock;
	struct wake_lock hs_wake_lock;
};

#ifdef CONFIG_MSM_SERIAL_DEBUGGER
extern void msm_serial_debug_enable(int);
#endif

#endif
