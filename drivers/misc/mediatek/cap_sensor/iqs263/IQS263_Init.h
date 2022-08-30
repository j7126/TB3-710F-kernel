/*
 *	IQS263_init.h - IQS263 Registers and Bit Definitions
 *
 *	Copyright (C) 2013 Azoteq (Pty) Ltd
 *	Author: Alwino van der Merwe <alwino.vandermerwe@azoteq.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *	Azoteq (Pty) Ltd does not take responsibility for the use of
 *	this driver.
 *
 *	This header file contains the setup for the IQS263 for this demo code
 *	of the IQS263 SAR Sensor. This file needs to be edited for each
 *	specific implementation of this driver. Use the IQS263 GUI from
 *	www.azoteq.com to change these settings and tune for a specific
 *	implementation.
 */

#ifndef IQS263_INIT_H
#define IQS263_INIT_H

/* Used to switch Projected mode & set Global Filter Halt (0x01 Byte1) */
#define SYSTEM_FLAGS_VAL					0x00

/* Enable / Disable system events (0x01 Byte2)*/
#define SYSTEM_EVENTS_VAL					0x00

/*
* Change the Multipliers & Base values (0x07 in this order)
* Please note that these values are affected by the Auto ATI routine and should
* only be written in the case of a specific setup.  Alternatively Auto ATI
* should be manually called after writing these settings.
* This is applicable for both Multipliers and Compensation.
*/
#define MULTIPLIERS_CH0						0x3F
#define MULTIPLIERS_CH1						0x2A
#define MULTIPLIERS_CH2						0x37
#define MULTIPLIERS_CH3						0x00
#define BASE_VAL							0x44

/*
* Change the Compensation for each channel (0x08 in this order)
* Please note that these values are affected by the Auto ATI routine and should
* only be written in the case of a specific setup.  Alternatively Auto ATI
* should be manually called after writing these settings.
* This is applicable for both Multipliers and Compensation.
*/
#define COMPENSATION_CH0					0x74
#define COMPENSATION_CH1					0xA9
#define COMPENSATION_CH2					0xA5
#define COMPENSATION_CH3					0x00

/* Change the Prox Settings or setup of the IQS263 (0x09 in this order) */
#define PROXSETTINGS0_VAL					0x10
#define PROXSETTINGS1_VAL					0x11
#define PROXSETTINGS1_VAL_SLEEP				0x51
#define PROXSETTINGS2_VAL					0x00
#define PROXSETTINGS3_VAL					0x01
#define EVENT_MASK_VAL						0x02
#define EVENT_MASK_VAL_SLEEP				0x00

/* Change the Thresholds for each channel (0x0A in this order) */
#define PROX_THRESHOLD						0x05
#define TOUCH_THRESHOLD_CH1					0x02
#define TOUCH_THRESHOLD_CH2					0x10
#define TOUCH_THRESHOLD_CH3					0x10
#define MOVEMENT_THRESHOLD					0x03
#define RESEED_BLOCK						0x00
#define HALT_TIME							0xFF
#define I2C_TIMEOUT							0x04

/* Change the Timing settings (0x0B in this order) */
#define LOW_POWER							0x00
#define ATI_TARGET_TOUCH					0xaa
#define ATI_TARGET_PROX						0xaa
#define TAP_TIMER							0x00
#define FLICK_TIMER							0x14
#define FLICK_THRESHOLD						0x32

/* Set Active Channels (0x0D) */
#define ACTIVE_CHS						0x03

#endif	/* IQS263_INIT_H */
