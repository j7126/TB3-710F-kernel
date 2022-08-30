/*
 *	iqs263.h - IQS263 Registers and Bit Definitions
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
 *	This header file contains the register and bit definitions for the
 *	IQS263 SAR Sensor, to be used in the driver. This makes the code
 *	cleaner to read.
 */
#ifndef IQS263_H
#define IQS263_H

/*	i2c slave device address	*/
#define IQS263_ADDR	0x44

/*	Definitions for the driver	*/
#define IQS_MAJOR				248
#define IQS_MINOR				0
#define IQS_DEV_NUMS			1
#define IQS_NUM_RW_REGS         26
#define DEVICE_NAME             "iqs263"

/*	Addresses of Registers on IQS263	*/
#define DEV_INF_263				0x00
#define SYSFLAGS_263			0x01
#define COORDINATE_263   		0x02
#define TOUCH_STAT_263			0x03
#define COUNTS_263				0x04
#define LTA_263 				0x05
#define DELTAS_263				0x06
#define MULTIPLIERS_263			0x07
#define COMPENSATION_263		0x08
#define PROXSETTING_263			0x09
#define THRESHOLD_263			0x0a
#define TIMING_TARGET_263		0x0b
#define GESTURE_TIMER_263		0x0c
#define ACTIVE_CHANNEL_263		0x0d

/*	Indicates reset has occurred	*/
#define	SHOW_RESET			0x20

/*	Bit definitions - Touch Bytes	*/
/*	Indicates a Prox and Touch on CH1	*/
#define	CH_1_EVENT			0x02
/*	Delays	*/
#define HANDSHAKE_DELAY_HOLD	11		/*	11ms	*/
#define HANDSHAKE_DELAY_SET		200		/*	200渭s	*/
#endif
