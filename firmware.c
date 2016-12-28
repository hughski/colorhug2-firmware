/* -*- Mode: C; tab-width: 8; indent-tabs-mode: t; c-basic-offset: 8 -*-
 *
 * Copyright (C) 2011-2015 Richard Hughes <richard@hughsie.com>
 *
 * Licensed under the GNU General Public License Version 2
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Additionally, some constants and code snippets have been taken from
 * freely available datasheets which are:
 *
 * Copyright (C) Microchip Technology, Inc.
 */

#include "ColorHug.h"
#include "usb_config.h"
#include "ch-math.h"
#include "ch-common.h"
#include "ch-flash.h"
#include "ch-sram.h"
#include "ch-temp.h"
#include "ch-mcdc04.h"

#include <i2c.h>

#include <USB/usb.h>
#include <USB/usb_common.h>
#include <USB/usb_device.h>
#include <USB/usb_function_hid.h>

/* configuration */
#pragma config XINST	= OFF		/* turn off extended instruction set */
#pragma config STVREN	= ON		/* Stack overflow reset */
#pragma config PLLDIV	= 6		/* (24 MHz crystal used on this board) */
#pragma config WDTEN	= ON		/* Watch Dog Timer (WDT) */
#pragma config CP0	= OFF		/* Code protect */
#pragma config OSC	= HSPLL		/* HS oscillator, PLL enabled, HSPLL used by USB */
#pragma config CPUDIV	= OSC2_PLL2	/* OSC1 = divide by 2 mode */
#pragma config IESO	= OFF		/* Internal External (clock) Switchover */
#pragma config FCMEN	= ON		/* Fail Safe Clock Monitor */
#pragma config T1DIG	= ON		/* secondary clock Source */
#pragma config LPT1OSC	= OFF		/* low power timer*/
#pragma config WDTPS	= 2048		/* Watchdog Timer Postscaler */
#pragma config DSWDTOSC	= INTOSCREF	/* DSWDT uses INTOSC/INTRC as reference clock */
#pragma config RTCOSC	= T1OSCREF	/* RTCC uses T1OSC/T1CKI as reference clock */
#pragma config DSBOREN	= OFF		/* Zero-Power BOR disabled in Deep Sleep */
#pragma config DSWDTEN	= OFF		/* Deep Sleep Watchdog Timer Enable */
#pragma config DSWDTPS	= 8192		/* Deep Sleep Watchdog Timer Postscale Select 1:8,192 (8.5 seconds) */
#pragma config IOL1WAY	= OFF		/* The IOLOCK bit (PPSCON<0>) can be set and cleared as needed */
#pragma config MSSP7B_EN = MSK7		/* 7 Bit address masking */
#pragma config WPFP	= PAGE_1	/* Write Protect Program Flash Page 0 */
#pragma config WPEND	= PAGE_0	/* Write/Erase protect Flash Memory pages */
#pragma config WPCFG	= OFF		/* Write/Erase Protection of last page Disabled */
#pragma config WPDIS	= OFF		/* Write Protect Disable */

#pragma rom

extern void _startup (void);
void CHugHighPriorityISRCode();
void CHugLowPriorityISRCode();

#pragma code REMAPPED_RESET_VECTOR = CH_EEPROM_ADDR_RUNCODE
void _reset (void)
{
	_asm goto _startup _endasm
}
#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = CH_EEPROM_ADDR_HIGH_INTERRUPT
void Remapped_High_ISR (void)
{
	_asm goto CHugHighPriorityISRCode _endasm
}
#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = CH_EEPROM_ADDR_LOW_INTERRUPT
void Remapped_Low_ISR (void)
{
	_asm goto CHugLowPriorityISRCode _endasm
}

/* actual interupt handlers */
#pragma interrupt CHugHighPriorityISRCode
void CHugHighPriorityISRCode()
{
}

#pragma interruptlow CHugLowPriorityISRCode
void CHugLowPriorityISRCode()
{
}

/* If we program the firmware to the device without a bootloader then
 * this sets up the interrupt vectors properly.
 * The colorhug-inhx32-to-bin will ignore any code < 0x4000 */
#pragma code HIGH_INTERRUPT_VECTOR = 0x08
void CHugHighPriorityISRPlaceholder (void)
{
	_asm goto CH_EEPROM_ADDR_HIGH_INTERRUPT _endasm
}
#pragma code LOW_INTERRUPT_VECTOR = 0x18
void CHugLowPriorityISRPlaceholder (void)
{
	_asm goto CH_EEPROM_ADDR_LOW_INTERRUPT _endasm
}

#pragma udata

static uint32_t		SensorSerial = 0x00000000;
static uint16_t		CalibrationMap[6];
static uint16_t		PcbErrata = CH_PCB_ERRATA_NONE;
static ChSha1		remote_hash;

#pragma udata udata1
static uint8_t OwnerName[CH_OWNER_LENGTH_MAX];
static uint8_t OwnerEmail[CH_OWNER_LENGTH_MAX];

/* this is used to map the firmware to a hardware version */
static const char flash_id[] = CH_FIRMWARE_ID_TOKEN;

#pragma udata
/* USB idle support */
static uint8_t		idle_command = 0x00;
static uint8_t		idle_counter = 0x00;

/* USB buffers */
static uint8_t RxBuffer[CH_USB_HID_EP_SIZE];
static uint8_t TxBuffer[CH_USB_HID_EP_SIZE];
static uint8_t FlashBuffer[CH_FLASH_WRITE_BLOCK_SIZE];

USB_HANDLE	USBOutHandle = 0;
USB_HANDLE	USBInHandle = 0;

/* protect against a deactivated device changing its serial number */
static uint8_t		flash_success = 0xff;

/* sensor context */
CHugMcdc04Context ctx;

#pragma code

/**
 * CHugGetLEDs:
 **/
uint8_t
CHugGetLEDs(void)
{
	return (PORTEbits.RE1 << 1) + PORTEbits.RE0;
}

/**
 * CHugSetLEDsInternal:
 **/
static void
CHugSetLEDsInternal(uint8_t leds)
{
	/* the first few boards on the P&P machines had the
	 * LEDs soldered the wrong way around */
	if ((PcbErrata & CH_PCB_ERRATA_SWAPPED_LEDS) > 0) {
		PORTEbits.RE0 = (leds & CH_STATUS_LED_GREEN);
		PORTEbits.RE1 = (leds & CH_STATUS_LED_RED) >> 1;
	} else {
		PORTEbits.RE0 = (leds & CH_STATUS_LED_RED) >> 1;
		PORTEbits.RE1 = (leds & CH_STATUS_LED_GREEN);
	}
}

/**
 * CHugSetLEDsDutyCycle:
 **/
static void
CHugSetLEDsDutyCycle(uint8_t leds, uint8_t length, uint8_t duty)
{
	uint8_t i, j;
	for (j = 0; j < length; j++) {
		ClrWdt();
		CHugSetLEDsInternal(leds);
		for (i = 0; i < duty; i++);
		CHugSetLEDsInternal(0);
		for (i = 0; i < 0xff - duty; i++);
	}
}

/**
 * CHugWelcomeFlash:
 **/
static void
CHugWelcomeFlash(void)
{
	uint8_t i;

	/* do not to the startup flash */
	if ((PcbErrata & CH_PCB_ERRATA_NO_WELCOME) > 0)
		return;

	for (i = 0; i < 0xaf; i++)
		CHugSetLEDsDutyCycle(CH_STATUS_LED_RED, 0x05, i);
	for (i = 0; i < 0xaf; i++)
		CHugSetLEDsDutyCycle(CH_STATUS_LED_RED, 0x05, 0xff-i);
	for (i = 0; i < 0xaf; i++)
		CHugSetLEDsDutyCycle(CH_STATUS_LED_GREEN, 0x05, i);
	for (i = 0; i < 0xaf; i++)
		CHugSetLEDsDutyCycle(CH_STATUS_LED_GREEN, 0x05, 0xff-i);
	CHugSetLEDsInternal(0);
}

/**
 * CHugSetLEDs:
 **/
static void
CHugSetLEDs(uint8_t leds,
	    uint8_t repeat,
	    uint8_t on_time,
	    uint8_t off_time)
{
	uint8_t i;

	/* trivial case */
	if (repeat == 0) {
		CHugSetLEDsInternal (leds);
		return;
	}

	/* run in a loop */
	for (i = 0; i < repeat; i++) {
		CHugSetLEDsInternal (leds);
		Delay10KTCYx(on_time);
		CHugSetLEDsInternal (0);
		Delay10KTCYx(off_time);

		/* clear watchdog */
		ClrWdt();
	}
}

/**
 * CHugReadEEprom:
 **/
static void
CHugReadEEprom(void)
{
	/* read this into RAM so it can be changed */
	CHugFlashRead(CH_EEPROM_ADDR_CONFIG + CH_EEPROM_OFFSET_SERIAL,
		      sizeof(uint32_t),
		      (uint8_t *) &SensorSerial);
	CHugFlashRead(CH_EEPROM_ADDR_CONFIG + CH_EEPROM_OFFSET_CALIBRATION_MAP,
		      6 * sizeof(uint16_t),
		      (uint8_t *) &CalibrationMap);
	CHugFlashRead(CH_EEPROM_ADDR_CONFIG + CH_EEPROM_OFFSET_PCB_ERRATA,
		      1 * sizeof(uint16_t),
		      (uint8_t *) &PcbErrata);
	CHugFlashRead(CH_EEPROM_ADDR_CONFIG + CH_EEPROM_OFFSET_REMOTE_HASH,
		      1 * sizeof(ChSha1),
		      (uint8_t *) &remote_hash);
	CHugFlashRead(CH_EEPROM_ADDR_OWNER + CH_EEPROM_OFFSET_NAME,
		      CH_OWNER_LENGTH_MAX * sizeof(char),
		      (uint8_t *) OwnerName);
	CHugFlashRead(CH_EEPROM_ADDR_OWNER + CH_EEPROM_OFFSET_EMAIL,
		      CH_OWNER_LENGTH_MAX * sizeof(char),
		      (uint8_t *) OwnerEmail);

	/* the default value in flash is 0xff */
	if (OwnerName[0] == 0xff)
		OwnerName[0] = '\0';
	if (OwnerEmail[0] == 0xff)
		OwnerEmail[0] = '\0';
	if (PcbErrata == 0xffff)
		PcbErrata = CH_PCB_ERRATA_NONE;

	/* fix up some PCBs we know about */
	if (SensorSerial >= 2732 && SensorSerial <= 3088)
		PcbErrata ^= CH_PCB_ERRATA_SWAPPED_LEDS;
}

/**
 * CHugWriteEEprom:
 **/
static void
CHugWriteEEprom(void)
{
	/* we can't call this more than 10,000 times otherwise we'll
	 * burn out the device */
	CHugFlashErase(CH_EEPROM_ADDR_CONFIG, CH_FLASH_WRITE_BLOCK_SIZE);

	/* write this in one fell swoop */
	memcpy(FlashBuffer + CH_EEPROM_OFFSET_SERIAL,
	       (void *) &SensorSerial,
	       sizeof(uint32_t));
	memcpy(FlashBuffer + CH_EEPROM_OFFSET_CALIBRATION_MAP,
	       (void *) &CalibrationMap,
	       CH_CALIBRATION_INDEX_MAX * sizeof(uint16_t));
	memcpy(FlashBuffer + CH_EEPROM_OFFSET_PCB_ERRATA,
	       (void *) &PcbErrata,
	       1 * sizeof(uint16_t));
	memcpy(FlashBuffer + CH_EEPROM_OFFSET_REMOTE_HASH,
	       (void *) &remote_hash,
	       1 * sizeof(ChSha1));
	CHugFlashWrite(CH_EEPROM_ADDR_CONFIG,
		       CH_FLASH_WRITE_BLOCK_SIZE,
		       FlashBuffer);

	CHugFlashErase(CH_EEPROM_ADDR_OWNER, 2 * CH_FLASH_WRITE_BLOCK_SIZE);
	CHugFlashWrite(CH_EEPROM_ADDR_OWNER + CH_EEPROM_OFFSET_NAME,
		       CH_FLASH_WRITE_BLOCK_SIZE,
		       (uint8_t *) OwnerName);
	CHugFlashWrite(CH_EEPROM_ADDR_OWNER + CH_EEPROM_OFFSET_EMAIL,
		       CH_FLASH_WRITE_BLOCK_SIZE,
		       (uint8_t *) OwnerEmail);
}

/**
 * CHugIsMagicUnicorn:
 **/
static char
CHugIsMagicUnicorn(const char *text)
{
	if (text[0] == 'U' &&
	    text[1] == 'n' &&
	    text[2] == '1' &&
	    text[3] == 'c' &&
	    text[4] == '0' &&
	    text[5] == 'r' &&
	    text[6] == 'n' &&
	    text[7] == '2')
		return TRUE;
	return FALSE;
}

/**
 * CHugDotProduct:
 * @vec1: An input vector
 * @vec2: Another input vector
 * @scalar: the output value
 *
 * Find the dot product of two vectors
 **/
static uint8_t
CHugDotProduct(const CHugPackedFloat *vec1,
	       const CHugPackedFloat *vec2,
	       CHugPackedFloat *scalar)
{
	CHugPackedFloat tmp;
	uint8_t rc;

	/* 1 */
	rc = CHugPackedFloatMultiply (&vec1[0],
				      &vec2[0],
				      scalar);
	if (rc != CH_ERROR_NONE)
		goto out;

	/* 2 */
	rc = CHugPackedFloatMultiply (&vec1[1],
				      &vec2[1],
				      &tmp);
	if (rc != CH_ERROR_NONE)
		goto out;

	/* add to result */
	rc = CHugPackedFloatAdd(scalar, &tmp, scalar);
	if (rc != CH_ERROR_NONE)
		goto out;

	/* 3 */
	rc = CHugPackedFloatMultiply (&vec1[2],
				      &vec2[2],
				      &tmp);
	if (rc != CH_ERROR_NONE)
		goto out;

	/* add to result */
	rc = CHugPackedFloatAdd(scalar, &tmp, scalar);
	if (rc != CH_ERROR_NONE)
		goto out;
out:
	return rc;
}

/**
 * CHugCalibrationMultiply:
 * @cal: calibration matrix
 * @device_rgb: device floating point values
 * @xyz: the output value
 *
 * Multiply the device vector by the calibration matrix.
 **/
static uint8_t
CHugCalibrationMultiply (const CHugPackedFloat *cal,
			 const CHugPackedFloat *device_rgb,
			 CHugPackedFloat *xyz)
{
	uint8_t rc;

	/* X */
	rc = CHugDotProduct(cal + 0, device_rgb, &xyz[0]);
	if (rc != CH_ERROR_NONE)
		goto out;

	/* Y */
	rc = CHugDotProduct(cal + 3, device_rgb, &xyz[1]);
	if (rc != CH_ERROR_NONE)
		goto out;

	/* Z */
	rc = CHugDotProduct(cal + 6, device_rgb, &xyz[2]);
	if (rc != CH_ERROR_NONE)
		goto out;
out:
	return rc;
}

/**
 * CHugSwitchCalibrationMatrix:
 **/
static uint8_t
CHugSwitchCalibrationMatrix(uint16_t calibration_index,
			    CHugPackedFloat *calibration)
{
	uint32_t addr;
	uint8_t calibration_type;

	addr = CH_CALIBRATION_ADDR + (calibration_index * 0x40);
	CHugFlashRead(addr, 9 * sizeof(CHugPackedFloat),
		      (uint8_t *) calibration);
	if (calibration[0].raw == 0xffffffff)
		return CH_ERROR_NO_CALIBRATION;

	/* check the calibration matrix is valid */
	CHugFlashRead(addr + 0x24, 1, (uint8_t *) &calibration_type);
	if (calibration_index == 0) {
		if (calibration_type != CH_CALIBRATION_TYPE_ALL)
			return CH_ERROR_INVALID_CALIBRATION;
	} else {
		if (calibration_type == 0)
			return CH_ERROR_INVALID_CALIBRATION;
	}

	return CH_ERROR_NONE;
}

/**
 * CHugTakeReadingsXYZ:
 **/
static uint8_t
CHugTakeReadingsXYZ (uint8_t calibration_index,
		     CHugPackedFloat *x,
		     CHugPackedFloat *y,
		     CHugPackedFloat *z)
{
	CHugPackedFloat calibration[9];
	CHugPackedFloat readings[3];
	CHugPackedFloat readings_tmp[3];
	uint8_t i;
	uint8_t rc;

	/* get integer readings */
	rc = CHugMcdc04TakeReadingsAuto(&ctx,
					&readings[CH_COLOR_OFFSET_RED],
					&readings[CH_COLOR_OFFSET_GREEN],
					&readings[CH_COLOR_OFFSET_BLUE]);
	if (rc != CH_ERROR_NONE)
		return rc;

	/* convert to xyz using the factory calibration */
	rc = CHugSwitchCalibrationMatrix(CH_CALIBRATION_INDEX_FACTORY_ONLY,
					 calibration);
	if (rc != CH_ERROR_NONE)
		return rc;
	rc = CHugCalibrationMultiply(calibration,
				     readings,
				     readings_tmp);
	if (rc != CH_ERROR_NONE)
		return rc;

	/* use the specified correction matrix */
	if (calibration_index != CH_CALIBRATION_INDEX_FACTORY_ONLY) {
		rc = CHugSwitchCalibrationMatrix(calibration_index,
						 calibration);
		if (rc != CH_ERROR_NONE)
			return rc;
		rc = CHugCalibrationMultiply(calibration,
					     readings_tmp,
					     readings);
		if (rc != CH_ERROR_NONE)
			return rc;
	} else {
		memcpy(readings,
		       (void *) readings_tmp,
		       sizeof(readings));
	}

	/* copy values */
	*x = readings[0];
	*y = readings[1];
	*z = readings[2];
	return CH_ERROR_NONE;
}

/**
 * CHugDeviceIdle:
 **/
static void
CHugDeviceIdle(void)
{
	switch (idle_command) {
	case CH_CMD_RESET:
		Reset();
		break;
	}
	idle_command = 0x00;
}

/**
 * CHugGetCalibrationMatrix:
 **/
static uint8_t
CHugGetCalibrationMatrix(uint16_t calibration_index,
			 CHugPackedFloat *calibration,
			 uint8_t *types,
			 uint8_t *description)
{
	uint32_t addr;
	uint8_t matrix_size;

	matrix_size = 9 * sizeof(CHugPackedFloat);
	addr = CH_CALIBRATION_ADDR + (calibration_index * 0x40);
	CHugFlashRead(addr, matrix_size, (uint8_t *) calibration);
	CHugFlashRead(addr + matrix_size, sizeof(uint8_t), (uint8_t *) types);
	CHugFlashRead(addr + matrix_size + 1,
		      CH_CALIBRATION_DESCRIPTION_LEN,
		      (uint8_t *) description);
	if (description[0] == 0xff)
		return CH_ERROR_NO_CALIBRATION;
	return CH_ERROR_NONE;
}

/**
 * CHugCopyFlash:
 **/
static void
CHugCopyFlash(uint32_t src, uint32_t dest, uint16_t len)
{
	uint32_t addr;

	/* nothing to do */
	if (src == dest)
		return;

	/* copy in 64 byte chunks */
	for (addr = 0; addr < len; addr += CH_FLASH_WRITE_BLOCK_SIZE) {
		CHugFlashRead(src + addr, CH_FLASH_WRITE_BLOCK_SIZE, FlashBuffer);
		CHugFlashWrite(dest + addr, CH_FLASH_WRITE_BLOCK_SIZE, FlashBuffer);
	}
}

/**
 * CHugSetCalibrationMatrix:
 **/
static uint8_t
CHugSetCalibrationMatrix(uint16_t calibration_index,
			 CHugPackedFloat *calibration,
			 uint8_t types,
			 uint8_t *description)
{
	uint32_t addr_block_start;
	uint32_t offset;
	uint8_t matrix_size;

	/* calculate offsets */
	addr_block_start = CH_CALIBRATION_ADDR +
		((calibration_index / 16) * CH_FLASH_ERASE_BLOCK_SIZE);
	offset = (calibration_index % 16) * 0x40;

	/* erase the destination block */
	CHugFlashErase(CH_CALIBRATION_ADDR_TMP, CH_FLASH_ERASE_BLOCK_SIZE);

	/* copy the block to the temporary area */
	CHugCopyFlash(addr_block_start,
		      CH_CALIBRATION_ADDR_TMP,
		      CH_FLASH_ERASE_BLOCK_SIZE);

	/* erase the block */
	CHugFlashErase(addr_block_start, CH_FLASH_ERASE_BLOCK_SIZE);

	/* write matrixes before the offset */
	CHugCopyFlash(CH_CALIBRATION_ADDR_TMP,
		      addr_block_start,
		      offset);

	/* write the current matrix in one chunk */
	matrix_size = 9 * sizeof(CHugPackedFloat);
	memcpy(FlashBuffer,
	       (void *) calibration,
	       matrix_size);
	memcpy(FlashBuffer + matrix_size,
	       (void *) &types,
	       1);
	memcpy(FlashBuffer + matrix_size + 1,
	       (void *) description,
	       CH_CALIBRATION_DESCRIPTION_LEN);
	CHugFlashWrite(addr_block_start + offset,
		       CH_FLASH_WRITE_BLOCK_SIZE,
		       (uint8_t *) FlashBuffer);

	/* write matrixes after the offset */
	CHugCopyFlash(CH_CALIBRATION_ADDR_TMP + offset + 0x40,
		      addr_block_start + offset + 0x40,
		      CH_FLASH_ERASE_BLOCK_SIZE - (offset + 0x40));

	return CH_ERROR_NONE;
}

/**
 * CHugTakeReadingArray:
 **/
static uint8_t
CHugTakeReadingArray(uint8_t *data)
{
	CHugPackedFloat xyz[3];
	uint16_t i;
	uint16_t tmp[3];
	uint8_t rc = CH_ERROR_NONE;

	/* take a quick non-adaptive reading without using the factory
	 * calibration matrix */
	CHugMcdc04SetTINT(&ctx, CH_MCDC04_TINT_1);
	CHugMcdc04SetIREF(&ctx, CH_MCDC04_IREF_20);
	CHugMcdc04SetDIV(&ctx, CH_MCDC04_DIV_DISABLE);
	rc = CHugMcdc04WriteConfig(&ctx);
	if (rc != CH_ERROR_NONE)
		goto out;

	/* fully populating the SRAM, 2 bytes per reading x 3 channels is:
	 * 8192 / (2 * 3) = 1365 measurements */
	for (i = 0; i < 1365; i++) {
		rc = CHugMcdc04TakeReadingsRaw(&ctx, &xyz[0], &xyz[1], &xyz[2]);
		if (rc != CH_ERROR_NONE)
			goto out;

		/* squeeze a 32 bit number down into 16 bits, as even the
		 * brightest display cannot come close to saturating that in
		 * 1ms sample duration */
		if (xyz[0].raw > 0xffff ||
		    xyz[1].raw > 0xffff ||
		    xyz[2].raw > 0xffff / 2)
			return CH_ERROR_OVERFLOW_SENSOR;
		tmp[0] = xyz[0].raw;
		tmp[1] = xyz[1].raw;
		tmp[2] = xyz[2].raw * 2;
		CHugSramDmaFromCpu((uint8_t *) tmp, 6 * i, 6);
	}

	/* copy the first 30 'Y' values scaled to one byte */
	for (i = 0; i < 30; i++) {
		CHugSramDmaToCpu((6 * i) + 2, (uint8_t *) tmp, 2);
		data[i] = tmp[0] / 4;
	}
out:
	return rc;
}

/**
 * ProcessIO:
 **/
static void
ProcessIO(void)
{
	CHugPackedFloat readings[3];
	CHugPackedFloat temp;
	uint16_t address;
	uint16_t calibration_index;
	uint32_t reading;
	uint8_t checksum;
	uint8_t i;
	uint8_t length;
	uint8_t cmd;
	uint8_t rc = CH_ERROR_NONE;

	/* User Application USB tasks */
	if ((USBDeviceState < CONFIGURED_STATE) ||
	    (USBSuspendControl == 1))
		return;

	/* no data was received */
	if (HIDRxHandleBusy(USBOutHandle)) {
		if (idle_counter++ == 0xff &&
		    idle_command != 0x00)
			CHugDeviceIdle();
		return;
	}

	/* we're waiting for a read from the host */
	if (HIDTxHandleBusy(USBInHandle)) {
		/* hijack the pending read with the new error */
		TxBuffer[CH_BUFFER_OUTPUT_RETVAL] = CH_ERROR_INCOMPLETE_REQUEST;
		goto re_arm_rx;
	}

	/* got data, reset idle counter */
	idle_counter = 0;

	/* clear for debugging */
	memset (TxBuffer, 0xff, sizeof (TxBuffer));

	/* are we lost or stolen */
	if (flash_success == 0xff) {
		rc = CH_ERROR_DEVICE_DEACTIVATED;
		goto out;
	}

	cmd = RxBuffer[CH_BUFFER_INPUT_CMD];
	switch(cmd) {
	case CH_CMD_GET_HARDWARE_VERSION:
		TxBuffer[CH_BUFFER_OUTPUT_DATA] = PORTB & 0x0f;
		break;
	case CH_CMD_GET_LEDS:
		TxBuffer[CH_BUFFER_OUTPUT_DATA] = CHugGetLEDs();
		break;
	case CH_CMD_SET_LEDS:
		CHugSetLEDs(RxBuffer[CH_BUFFER_INPUT_DATA + 0],
			    RxBuffer[CH_BUFFER_INPUT_DATA + 1],
			    RxBuffer[CH_BUFFER_INPUT_DATA + 2],
			    RxBuffer[CH_BUFFER_INPUT_DATA + 3]);
		break;
	case CH_CMD_GET_PCB_ERRATA:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(void *) &PcbErrata,
			2);
		break;
	case CH_CMD_SET_PCB_ERRATA:
		memcpy (&PcbErrata,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			sizeof(uint16_t));
		break;
	case CH_CMD_GET_REMOTE_HASH:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(void *) &remote_hash,
			sizeof(ChSha1));
		break;
	case CH_CMD_SET_REMOTE_HASH:
		memcpy (&remote_hash,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			sizeof(ChSha1));
		break;
	case CH_CMD_GET_FIRMWARE_VERSION:
		*((uint16_t *) &TxBuffer[CH_BUFFER_OUTPUT_DATA + 0]) = CH_VERSION_MAJOR;
		*((uint16_t *) &TxBuffer[CH_BUFFER_OUTPUT_DATA + 2]) = CH_VERSION_MINOR;
		*((uint16_t *) &TxBuffer[CH_BUFFER_OUTPUT_DATA + 4]) = CH_VERSION_MICRO;
		break;
	case CH_CMD_READ_FLASH:
		/* are we lost or stolen */
		if (flash_success == 0xff) {
			rc = CH_ERROR_DEVICE_DEACTIVATED;
			break;
		}
		/* allow to read any firmware address */
		memcpy (&address,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA+0],
			2);
		length = RxBuffer[CH_BUFFER_INPUT_DATA+2];
		if (length > 60) {
			rc = CH_ERROR_INVALID_LENGTH;
			break;
		}
		if (address < CH_EEPROM_ADDR_RUNCODE ||
		    address > CH_EEPROM_ADDR_MAX) {
			rc = CH_ERROR_INVALID_ADDRESS;
			break;
		}
		CHugFlashRead(address, length,
			      &TxBuffer[CH_BUFFER_OUTPUT_DATA+1]);
		checksum = CHugCalculateChecksum (&TxBuffer[CH_BUFFER_OUTPUT_DATA+1],
						  length);
		TxBuffer[CH_BUFFER_OUTPUT_DATA+0] = checksum;
		break;
	case CH_CMD_GET_CALIBRATION:
		/* get the chosen calibration matrix */
		memcpy (&calibration_index,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			sizeof(uint16_t));
		if (calibration_index > CH_CALIBRATION_MAX) {
			rc = CH_ERROR_INVALID_VALUE;
			break;
		}
		rc = CHugGetCalibrationMatrix(calibration_index,
					      (CHugPackedFloat *)&TxBuffer[CH_BUFFER_OUTPUT_DATA],
					      &TxBuffer[CH_BUFFER_OUTPUT_DATA] + 0x24,
					      &TxBuffer[CH_BUFFER_OUTPUT_DATA] + 0x25);
		if (rc != CH_ERROR_NONE)
			break;
		break;
	case CH_CMD_SET_CALIBRATION:
		/* set the chosen calibration matrix */
		memcpy (&calibration_index,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			sizeof(uint16_t));
		if (calibration_index > CH_CALIBRATION_MAX) {
			rc = CH_ERROR_INVALID_VALUE;
			break;
		}
		CHugSetCalibrationMatrix(calibration_index,
					 (CHugPackedFloat *) &RxBuffer[CH_BUFFER_INPUT_DATA + 2],
					 RxBuffer[CH_BUFFER_INPUT_DATA + 2 + 0x24],
					 &RxBuffer[CH_BUFFER_INPUT_DATA + 3 + 0x24]);
		break;
	case CH_CMD_GET_CALIBRATION_MAP:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			&CalibrationMap,
			6 * sizeof(uint16_t));
		break;
	case CH_CMD_SET_CALIBRATION_MAP:
		memcpy ((void *) &CalibrationMap,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			6 * sizeof(uint16_t));
		break;
	case CH_CMD_GET_SERIAL_NUMBER:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) &SensorSerial,
			4);
		break;
	case CH_CMD_SET_SERIAL_NUMBER:
		memcpy (&SensorSerial,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			4);
		break;
	case CH_CMD_WRITE_EEPROM:
		/* verify the magic matched */
		if (CHugIsMagicUnicorn ((const char *) &RxBuffer[CH_BUFFER_INPUT_DATA])) {
			CHugWriteEEprom();
		} else {
			rc = CH_ERROR_WRONG_UNLOCK_CODE;
		}
		break;
	case CH_CMD_TAKE_READINGS:
		/* take a quick non-adaptive reading without using the factory
		 * calibration matrix */
		CHugMcdc04SetTINT(&ctx, CH_MCDC04_TINT_64);
		CHugMcdc04SetIREF(&ctx, CH_MCDC04_IREF_80);
		CHugMcdc04SetDIV(&ctx, CH_MCDC04_DIV_DISABLE);
		rc = CHugMcdc04WriteConfig(&ctx);
		if (rc != CH_ERROR_NONE)
			break;
		rc = CHugMcdc04TakeReadings(&ctx,
					    &readings[CH_COLOR_OFFSET_RED],
					    &readings[CH_COLOR_OFFSET_GREEN],
					    &readings[CH_COLOR_OFFSET_BLUE]);
		if (rc != CH_ERROR_NONE)
			break;
		readings[CH_COLOR_OFFSET_RED].raw *= 64;
		readings[CH_COLOR_OFFSET_GREEN].raw *= 64;
		readings[CH_COLOR_OFFSET_BLUE].raw *= 64;
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) readings,
			3 * sizeof(CHugPackedFloat));
		break;
	case CH_CMD_TAKE_READING_XYZ:
		/* take multiple readings and multiply with the
		 * calibration matrix */
		memcpy (&calibration_index,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			sizeof(uint16_t));
		if (calibration_index > CH_CALIBRATION_MAX + 6) {
			rc = CH_ERROR_INVALID_VALUE;
			break;
		}

		/* need to remap the index to reality */
		if (calibration_index >= CH_CALIBRATION_MAX) {
			calibration_index = CalibrationMap[calibration_index -
							   CH_CALIBRATION_MAX];
		}
		rc = CHugTakeReadingsXYZ(calibration_index,
					 &readings[CH_COLOR_OFFSET_RED],
					 &readings[CH_COLOR_OFFSET_GREEN],
					 &readings[CH_COLOR_OFFSET_BLUE]);
		if (rc != CH_ERROR_NONE)
			break;
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) readings,
			3 * sizeof(CHugPackedFloat));
		break;
	case CH_CMD_READ_SRAM:
		/* allow to read any SRAM address as only 64k is
		 * adressable by a uint16 */
		memcpy (&address,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA+0],
			2);
		length = RxBuffer[CH_BUFFER_INPUT_DATA+2];
		if (length > 60) {
			rc = CH_ERROR_INVALID_LENGTH;
			break;
		}
		CHugSramDmaToCpu(address,
				 &TxBuffer[CH_BUFFER_OUTPUT_DATA],
				 length);
		CHugSramDmaWait();
		break;
	case CH_CMD_WRITE_SRAM:
		/* allow to write any SRAM address as only 64k is
		 * adressable by a uint16 */
		memcpy (&address,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA+0],
			2);
		length = RxBuffer[CH_BUFFER_INPUT_DATA+2];
		if (length > 60) {
			rc = CH_ERROR_INVALID_LENGTH;
			break;
		}
		CHugSramDmaFromCpu(&RxBuffer[CH_BUFFER_INPUT_DATA+3],
				   address,
				   length);
		CHugSramDmaWait();
		break;
	case CH_CMD_RESET:
		/* only reset when USB stack is not busy */
		idle_command = CH_CMD_RESET;
		break;
	case CH_CMD_SET_FLASH_SUCCESS:
		if (RxBuffer[CH_BUFFER_INPUT_DATA] != 0x01 &&
		    RxBuffer[CH_BUFFER_INPUT_DATA] != 0xff) {
			rc = CH_ERROR_INVALID_VALUE;
			break;
		}
		flash_success = RxBuffer[CH_BUFFER_INPUT_DATA];
		rc = CHugFlashErase(CH_EEPROM_ADDR_FLASH_SUCCESS, 1);
		if (rc != CH_ERROR_NONE)
			break;
		rc = CHugFlashWrite(CH_EEPROM_ADDR_FLASH_SUCCESS, 1,
				    &RxBuffer[CH_BUFFER_INPUT_DATA]);
		break;
	case CH_CMD_GET_OWNER_NAME:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) OwnerName,
			CH_OWNER_LENGTH_MAX);
		break;
	case CH_CMD_GET_OWNER_EMAIL:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) OwnerEmail,
			CH_OWNER_LENGTH_MAX);
		break;
	case CH_CMD_SET_OWNER_NAME:
		memcpy ((void *) OwnerName,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			CH_OWNER_LENGTH_MAX);
		break;
	case CH_CMD_SET_OWNER_EMAIL:
		memcpy ((void *) OwnerEmail,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			CH_OWNER_LENGTH_MAX);
		break;
	case CH_CMD_GET_TEMPERATURE:
		rc = CHugTempGetAmbient(&temp);
		if (rc != CH_ERROR_NONE)
			goto out;
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) &temp,
			sizeof(CHugPackedFloat));
		break;
	case CH_CMD_TAKE_READING_ARRAY:
		rc = CHugTakeReadingArray(&TxBuffer[CH_BUFFER_OUTPUT_DATA]);
		break;
	case CH_CMD_SELF_TEST:
		rc = CHugSelfTest();
		break;
	default:
		rc = CH_ERROR_UNKNOWN_CMD;
		break;
	}
out:
	/* always send return code */
	if(!HIDTxHandleBusy(USBInHandle)) {
		TxBuffer[CH_BUFFER_OUTPUT_RETVAL] = rc;
		TxBuffer[CH_BUFFER_OUTPUT_CMD] = cmd;
		USBInHandle = HIDTxPacket(HID_EP,
					  (BYTE*)&TxBuffer[0],
					  CH_USB_HID_EP_SIZE);
	}
re_arm_rx:
	/* re-arm the OUT endpoint for the next packet */
	USBOutHandle = HIDRxPacket(HID_EP,
				   (BYTE*)&RxBuffer,
				   CH_USB_HID_EP_SIZE);
}

/**
 * UserInit:
 **/
static void
UserInit(void)
{
	/* set some defaults to power down the sensor */
	CHugSetLEDs(CH_STATUS_LED_RED | CH_STATUS_LED_GREEN,
		    0, 0x00, 0x00);

	/* read out the sensor data from EEPROM */
	CHugReadEEprom();
}

/**
 * InitializeSystem:
 **/
static void
InitializeSystem(void)
{
	/* enable the PLL and wait 2+ms until the PLL locks
	 * before enabling USB module */
	uint16_t pll_startup_counter = 1200;
	OSCTUNEbits.PLLEN = 1;
	while (pll_startup_counter--);

	/* default all pins to digital */
	ANCON0 = 0xFF;
	ANCON1 = 0xFF;

	/* set RA0 to input (READY)
	 * set RA1 to input (unused)
	 * set RA2 to input (unused)
	 * set RA3 to input (unused)
	 * set RA4 to input (missing)
	 * set RA5 to input (frequency counter),
	 * (RA6 is "don't care" in OSC2 mode)
	 * set RA7 to input (OSC1, HSPLL in) */
	TRISA = 0b11111111;

	/* set RB0 to input (h/w revision),
	 * set RB1 to input (h/w revision),
	 * set RB2 to input (h/w revision),
	 * set RB3 to input (h/w revision),
	 * set RB4 to input (SCL),
	 * set RB5 to input (SDA),
	 * set RB6 to input (PGC),
	 * set RB7 to input (PGD) */
	TRISB = 0b11111111;

	/* set RC0 to input (unused),
	 * set RC1 to output (SCK2),
	 * set RC2 to output (SDO2)
	 * set RC3 to input (unused)
	 * set RC4 to input (unused)
	 * set RC5 to input (unused)
	 * set RC6 to input (unused
	 * set RC7 to input (unused) */
	TRISC = 0b11111001;

	/* set RD0 to input (unused),
	/* set RD1 to input (unused),
	 * set RD2 to input (SDI2),
	 * set RD3 to output (SS2) [SSDMA?],
	 * set RD4-RD7 to input (unused) */
	TRISD = 0b11110111;

	/* set RE0, RE1 output (LEDs) others input (unused) */
	TRISE = 0x3c;

	/* assign remappable input and outputs */
	RPINR21 = 19;			/* RP19 = SDI2 */
	RPINR22 = 12;			/* RP12 = SCK2 (input and output) */
	RPOR12 = 0x0a;			/* RP12 = SCK2 */
	RPOR13 = 0x09;			/* RP13 = SDO2 */
	RPOR20 = 0x0c;			/* RP20 = SS2 (SSDMA) */

	/* turn on the SPI bus */
	SSP2STATbits.CKE = 1;		/* enable SMBus-specific inputs */
	SSP2STATbits.SMP = 0;		/* enable slew rate for HS mode */
	SSP2CON1bits.SSPEN = 1;		/* enables the serial port */
	SSP2CON1bits.SSPM = 0x0;	/* SPI master mode, clk = Fosc / 4 */

	/* set up the DMA controller */
	DMACON1bits.SSCON0 = 0;		/* SSDMA (_CS) is not DMA controlled */
	DMACON1bits.SSCON1 = 0;
	DMACON1bits.DLYINTEN = 0;	/* don't interrupt after each byte */
	DMACON2bits.INTLVL = 0x0;	/* interrupt only when complete */
	DMACON2bits.DLYCYC = 0x02;	/* minimum delay between bytes */

	/* clear base SRAM memory */
	CHugSramWipe(0x0000, 0xffff);

	/* set up the I2C controller */
	SSP1ADD = 0x3e;
	OpenI2C1(MASTER, SLEW_ON);

	/* set up TCN75A */
	CHugTempSetResolution(CH_TEMP_RESOLUTION_1_16C);

	/* set up MCDC04 */
	CHugMcdc04Init (&ctx);
	CHugMcdc04SetTINT(&ctx, CH_MCDC04_TINT_512);
	CHugMcdc04SetIREF(&ctx, CH_MCDC04_IREF_20);
	CHugMcdc04SetDIV(&ctx, CH_MCDC04_DIV_DISABLE);

	/* The USB module will be enabled if the bootloader has booted,
	 * so we soft-detach from the host. */
	if(UCONbits.USBEN == 1) {
		UCONbits.SUSPND = 0;
		UCON = 0;
		Delay10KTCYx(0xff);
	}

	/* only turn on the USB module when the device has power */
#if defined(USE_USB_BUS_SENSE_IO)
	tris_usb_bus_sense = INPUT_PIN;
#endif

	/* we're self powered */
#if defined(USE_SELF_POWER_SENSE_IO)
	tris_self_power = INPUT_PIN;
#endif

	/* do all user init code */
	UserInit();

	/* Initializes USB module SFRs and firmware variables to known states */
	USBDeviceInit();
}

/**
 * USBCBSuspend:
 *
 * Callback that is invoked when a USB suspend is detected
 **/
void
USBCBSuspend(void)
{
	/* power down LEDs */
	CHugSetLEDs(0, 0, 0x00, 0x00);
}

/**
 * USBCBWakeFromSuspend:
 *
 * The host may put USB peripheral devices in low power
 * suspend mode (by "sending" 3+ms of idle).  Once in suspend
 * mode, the host may wake the device back up by sending non-
 * idle state signalling.
 *
 * This call back is invoked when a wakeup from USB suspend
 * is detected.
 **/
static void
USBCBWakeFromSuspend(void)
{
}

/**
 * USBCBCheckOtherReq:
 *
 * Process the SETUP request and fulfill the request.
 **/
static void
USBCBCheckOtherReq(void)
{
	USBCheckHIDRequest();
}

/**
 * USBCBInitEP:
 *
 * Called when the host sends a SET_CONFIGURATION.
 **/
static void
USBCBInitEP(void)
{
	/* enable the HID endpoint */
	USBEnableEndpoint(HID_EP,
			  USB_IN_ENABLED|
			  USB_OUT_ENABLED|
			  USB_HANDSHAKE_ENABLED|
			  USB_DISALLOW_SETUP);

	/* re-arm the OUT endpoint for the next packet */
	USBOutHandle = HIDRxPacket(HID_EP,
				   (BYTE*)&RxBuffer,
				   CH_USB_HID_EP_SIZE);
}

/**
 * USER_USB_CALLBACK_EVENT_HANDLER:
 * @event: the type of event
 * @pdata: pointer to the event data
 * @size: size of the event data
 *
 * This function is called from the USB stack to
 * notify a user application that a USB event
 * occured.  This callback is in interrupt context
 * when the USB_INTERRUPT option is selected.
 **/
BOOL
USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
	switch(event) {
	case EVENT_TRANSFER:
		break;
	case EVENT_SUSPEND:
		USBCBSuspend();
		break;
	case EVENT_RESUME:
		USBCBWakeFromSuspend();
		break;
	case EVENT_CONFIGURED:
		USBCBInitEP();
		break;
	case EVENT_EP0_REQUEST:
		USBCBCheckOtherReq();
		break;
	case EVENT_TRANSFER_TERMINATED:
		break;
	default:
		break;
	}
	return TRUE;
}

/**
 * main:
 **/
void
main(void)
{
	InitializeSystem();

	/* read the flash success */
	CHugFlashRead(CH_EEPROM_ADDR_FLASH_SUCCESS, 1,
		      (uint8_t *) &flash_success);

	/* the watchdog saved us from our doom */
	if (!RCONbits.NOT_TO)
		CHugFatalError(CH_ERROR_WATCHDOG);

	/* do the welcome flash */
	CHugWelcomeFlash();

	while(1) {

		/* clear watchdog */
		ClrWdt();

		/* check bus status and service USB interrupts */
		USBDeviceTasks();

		ProcessIO();
	}
}
