/* -*- Mode: C; tab-width: 8; indent-tabs-mode: t; c-basic-offset: 8 -*-
 *
 * Copyright (C) 2011-2012 Richard Hughes <richard@hughsie.com>
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
#include "HardwareProfile.h"
#include "usb_config.h"
#include "ch-math.h"
#include "ch-common.h"

#include <flash.h>

#include <USB/usb.h>
#include <USB/usb_common.h>
#include <USB/usb_device.h>
#include <USB/usb_function_hid.h>

/* configuration */
#if defined(COLORHUG)
#pragma config XINST	= OFF		/* turn off extended instruction set */
#pragma config STVREN	= ON		/* Stack overflow reset */
#pragma config PLLDIV	= 3		/* (12 MHz crystal used on this board) */
#pragma config WDTEN	= ON		/* Watch Dog Timer (WDT) */
#pragma config CP0	= OFF		/* Code protect */
#pragma config OSC	= HSPLL		/* HS oscillator, PLL enabled, HSPLL used by USB */
#pragma config CPUDIV	= OSC1		/* OSC1 = divide by 1 mode */
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
#else
#error No hardware board defined, see "HardwareProfile.h" and __FILE__
#endif

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

/* ensure this is incremented on each released build */
static uint16_t		FirmwareVersion[3] = { 1, 2, 0 };

#pragma udata

static uint32_t		SensorSerial = 0x00000000;
static uint16_t		DarkCalibration[3] = { 0x0000, 0x0000, 0x0000 };
static uint16_t		CalibrationMap[6];
static uint16_t		SensorIntegralTime = 0xffff;
static CHugPackedFloat	PostScale;
static CHugPackedFloat	PreScale;
static uint16_t		PcbErrata = CH_PCB_ERRATA_NONE;
static uint8_t		MeasureMode = CH_MEASURE_MODE_FREQUENCY;
static ChSha1		remote_hash;

#pragma udata udata1
static uint8_t OwnerName[CH_OWNER_LENGTH_MAX];
static uint8_t OwnerEmail[CH_OWNER_LENGTH_MAX];

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

/* need to save this so we can power down in USB suspend and then power
 * back up in the same mode */
static ChFreqScale	multiplier_old = CH_FREQ_SCALE_0;

/* protect against a deactivated device changing its serial number */
static uint8_t		flash_success = 0xff;

#pragma code

/**
 * CHugGetLEDs:
 **/
unsigned char
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
		PORTEbits.RE0 = (leds & CH_STATUS_LED_RED) >> 1;
		PORTEbits.RE1 = (leds & CH_STATUS_LED_GREEN);
	} else {
		PORTEbits.RE0 = (leds & CH_STATUS_LED_GREEN);
		PORTEbits.RE1 = (leds & CH_STATUS_LED_RED) >> 1;
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

	for (i = 0; i < 0xff; i++)
		CHugSetLEDsDutyCycle(CH_STATUS_LED_RED, 0x05, i);
	for (i = 0; i < 0xff; i++)
		CHugSetLEDsDutyCycle(CH_STATUS_LED_RED, 0x05, 0xff-i);
	for (i = 0; i < 0xff; i++)
		CHugSetLEDsDutyCycle(CH_STATUS_LED_GREEN, 0x05, i);
	for (i = 0; i < 0xff; i++)
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
		delay10ktcy(on_time);
		CHugSetLEDsInternal (0);
		delay10ktcy(off_time);

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
	ReadFlash(CH_EEPROM_ADDR_CONFIG + CH_EEPROM_OFFSET_SERIAL,
		  sizeof(uint32_t),
		  (unsigned char *) &SensorSerial);
	ReadFlash(CH_EEPROM_ADDR_CONFIG + CH_EEPROM_OFFSET_DARK_OFFSET_RED,
		  3 * sizeof(uint16_t),
		  (unsigned char *) DarkCalibration);
	ReadFlash(CH_EEPROM_ADDR_CONFIG + CH_EEPROM_OFFSET_PRE_SCALE,
		  sizeof(CHugPackedFloat),
		  (unsigned char *) &PreScale);
	ReadFlash(CH_EEPROM_ADDR_CONFIG + CH_EEPROM_OFFSET_POST_SCALE,
		  sizeof(CHugPackedFloat),
		  (unsigned char *) &PostScale);
	ReadFlash(CH_EEPROM_ADDR_CONFIG + CH_EEPROM_OFFSET_CALIBRATION_MAP,
		  6 * sizeof(uint16_t),
		  (unsigned char *) &CalibrationMap);
	ReadFlash(CH_EEPROM_ADDR_CONFIG + CH_EEPROM_OFFSET_PCB_ERRATA,
		  1 * sizeof(uint16_t),
		  (unsigned char *) &PcbErrata);
	ReadFlash(CH_EEPROM_ADDR_CONFIG + CH_EEPROM_OFFSET_REMOTE_HASH,
		  1 * sizeof(ChSha1),
		  (unsigned char *) &remote_hash);
	ReadFlash(CH_EEPROM_ADDR_OWNER + CH_EEPROM_OFFSET_NAME,
		  CH_OWNER_LENGTH_MAX * sizeof(char),
		  (unsigned char *) OwnerName);
	ReadFlash(CH_EEPROM_ADDR_OWNER + CH_EEPROM_OFFSET_EMAIL,
		  CH_OWNER_LENGTH_MAX * sizeof(char),
		  (unsigned char *) OwnerEmail);

	/* the default value in flash is 0xff */
	if (OwnerName[0] == 0xff)
		OwnerName[0] = '\0';
	if (OwnerEmail[0] == 0xff)
		OwnerEmail[0] = '\0';
	if (PcbErrata == 0xffff)
		PcbErrata = CH_PCB_ERRATA_NONE;

	/* fix up some PCBs we know about */
	if (PcbErrata == CH_PCB_ERRATA_NONE) {
		if (SensorSerial == 15 ||
		    SensorSerial == 72 ||
		    SensorSerial == 76 ||
		    SensorSerial == 82 ||
		    SensorSerial == 114 ||
		    SensorSerial == 120 ||
		    SensorSerial == 103 ||
		    SensorSerial == 104) {
			PcbErrata &= CH_PCB_ERRATA_SWAPPED_LEDS;
		}
	}
}

/**
 * CHugWriteEEprom:
 **/
static void
CHugWriteEEprom(void)
{
	/* we can't call this more than 10,000 times otherwise we'll
	 * burn out the device */
	EraseFlash(CH_EEPROM_ADDR_CONFIG,
		   CH_EEPROM_ADDR_CONFIG + CH_FLASH_WRITE_BLOCK_SIZE);

	/* write this in one fell swoop */
	memcpy(FlashBuffer + CH_EEPROM_OFFSET_SERIAL,
	       (void *) &SensorSerial,
	       sizeof(uint32_t));
	memcpy(FlashBuffer + CH_EEPROM_OFFSET_DARK_OFFSET_RED,
	       (void *) DarkCalibration,
	       3 * sizeof(uint16_t));
	memcpy(FlashBuffer + CH_EEPROM_OFFSET_PRE_SCALE,
	       (void *) &PreScale,
	       sizeof(CHugPackedFloat));
	memcpy(FlashBuffer + CH_EEPROM_OFFSET_POST_SCALE,
	       (void *) &PostScale,
	       sizeof(CHugPackedFloat));
	memcpy(FlashBuffer + CH_EEPROM_OFFSET_CALIBRATION_MAP,
	       (void *) &CalibrationMap,
	       6 * sizeof(uint16_t));
	memcpy(FlashBuffer + CH_EEPROM_OFFSET_PCB_ERRATA,
	       (void *) &PcbErrata,
	       1 * sizeof(uint16_t));
	memcpy(FlashBuffer + CH_EEPROM_OFFSET_REMOTE_HASH,
	       (void *) &remote_hash,
	       1 * sizeof(ChSha1));
	WriteBytesFlash(CH_EEPROM_ADDR_CONFIG,
			CH_FLASH_WRITE_BLOCK_SIZE,
			(unsigned char *) FlashBuffer);

	EraseFlash(CH_EEPROM_ADDR_OWNER,
		   CH_EEPROM_ADDR_OWNER + 2 * CH_FLASH_WRITE_BLOCK_SIZE);
	WriteBytesFlash(CH_EEPROM_ADDR_OWNER + CH_EEPROM_OFFSET_NAME,
			CH_FLASH_WRITE_BLOCK_SIZE,
			(unsigned char *) OwnerName);
	WriteBytesFlash(CH_EEPROM_ADDR_OWNER + CH_EEPROM_OFFSET_EMAIL,
			CH_FLASH_WRITE_BLOCK_SIZE,
			(unsigned char *) OwnerEmail);
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
 * CHugScaleByIntegral:
 **/
static void
CHugScaleByIntegral (uint16_t *pulses, uint32_t actual_integral_time)
{
	uint32_t tmp;

	/* no point, so optimize */
	if (actual_integral_time == 0xffff)
		return;

	/* do this as integer math for speed */
	tmp = (uint32_t) *pulses * 0xffff;
	*pulses = tmp / actual_integral_time;
}

/**
 * CHugTakeReadingsFrequencyRaw:
 * @integral_time: The integral time of the sample
 * @last_rising_edge: (out): The last rising edge value, or 0
 * Return value: the number of detected edges for the sample
 *
 * The TAOS3200 sensor with the external IR filter and in the rubber
 * aperture gives the following rough outputs with red selected at 100%:
 *
 *   Frequency   |   Condition
 * --------------|-------------------
 *    10Hz       | Aperture covered
 *    160Hz      | TFT backlight on, but masked to black
 *    1.24KHz    | 100 white at 170cd/m2
 *
 * So when we sample for 100ms at ~160Hz we actually need to care about
 * starting and stopping the sample at the same point in the waveform,
 * otherwise we could be counting 15, 16 or 17 pulses.
 *
 * By triggering and stopping on the rising edge, we can improve the
 * accuracy by 12.5% for low light readings, at the slight expense of
 * making the calibration duration 6.25ms longer.
 **/
static uint16_t
CHugTakeReadingsFrequencyRaw (uint32_t integral_time, uint32_t *last_rising_edge)
{
	uint32_t i;
	uint32_t last_rising_edge_tmp = 0;
	uint16_t number_edges = 0;
	unsigned char ra_tmp = PORTA;

	/* clear watchdog */
	ClrWdt();

	/* wait for the output to change so we start on a new pulse
	 * rising edge, which means more accurate black readings */
	for (i = 0; i < integral_time; i++) {
		if (ra_tmp != PORTA) {
			/* ___      ____
			 *    |____|    |___
			 *
			 *         ^- START HERE
			 */
			if (PORTAbits.RA5 == 1)
				break;
			ra_tmp = PORTA;
		}
	}

	/* we got no change */
	if (i == integral_time)
		goto out;

	/* count how many times we get a rising edge */
	for (i = 0; i < integral_time; i++) {
		if (ra_tmp != PORTA) {
			if (PORTAbits.RA5 == 1) {
				number_edges++;
				/* _      ____
				 *  |____|    |_
				 *
				 *       ^- SAVE LOOP COUNTER
				 */
				last_rising_edge_tmp = i;
			}
			ra_tmp = PORTA;
		}
	}

	/* no second edge found */
	if (last_rising_edge_tmp == 0)
		goto out;

	/* return last rising edge */
	if (last_rising_edge != 0)
		*last_rising_edge = last_rising_edge_tmp;
out:
	return number_edges;
}

/**
 * CHugWaitForPulse:
 *
 * Wait for the next rising pulse.
 *
 * Return value: the number of ticks spent waiting, where a tick is
 * the clock frequency / 32, or zero for no rising edge detected.
 **/
static uint32_t
CHugWaitForPulse (uint32_t integral_time)
{
	uint32_t i;
	unsigned char ra_tmp = PORTA;

	/* clear watchdog */
	ClrWdt();

	/* wait for rising or falling edge */
	for (i = 0; i < integral_time; i++) {
		/* __      ____
		 *   |____|    |___
		 *
		 *   ^----^- START HERE
		 */
		if (ra_tmp != PORTA)
			goto out;
	}

	/* we never got a pulse */
	i = 0;
out:
	return i;
}

/**
 * CHugTakeReadingDuration:
 *
 * Take a reading from the sensor using the pulse width
 * where black ~= 10Hz and white ~= 1kHz
 **/
static uint32_t
CHugTakeReadingDuration (uint32_t integral_time)
{
	uint32_t tmp;
	uint32_t total = 0;
	uint32_t total_check = 0;
	uint8_t i;
	uint8_t rc = CH_ERROR_NONE;

	/* wait for initial rising edge */
	tmp = CHugWaitForPulse (integral_time);
	if (tmp == 0)
		goto out;

	/* wait for rising edges */
	for (i = 0; i < 0xffff; i++) {

		/* if we don't get a pulse then just assume maximum */
		tmp = CHugWaitForPulse(integral_time);
		if (tmp == 0) {
			total = 0;
			goto out;
		}

		/* if we would overflow a uint32 then abort */
		total_check += tmp / 0xff;
		if (total_check > integral_time / 0xff)
			break;

		total += tmp;
	}

	/* average out */
	total /= i;
out:
	return total;
}

/**
 * CHugTakeReadingRaw:
 *
 * Take a reading from the sensor. For bright readings we use the
 * SensorIntegralTime taken from the user, but for dark readings we
 * multiply this by a constant to get a more accurate reading.
 **/
static uint32_t
CHugTakeReadingRaw (uint32_t integral_time)
{
	uint32_t val = 0;
	if (MeasureMode == CH_MEASURE_MODE_FREQUENCY) {
		val = CHugTakeReadingsFrequencyRaw(integral_time, 0);
		goto out;
	}
	if (MeasureMode == CH_MEASURE_MODE_DURATION) {
		val = CHugTakeReadingDuration(integral_time);
		goto out;
	}
out:
	return val;
}

/**
 * CHugTakeReadingFrequency:
 *
 * Take a reading from the sensor. For bright readings we use the
 * SensorIntegralTime taken from the user, but for dark readings we
 * multiply this by a constant to get a more accurate reading.
 **/
static uint16_t
CHugTakeReadingFrequency (void)
{
	const uint32_t edges_min = 10; /* this is a dim CRT screen */
	uint16_t number_edges;
	uint32_t integral;
	uint32_t integral_max;
	uint32_t last_rising_edge;

	/* set the maximum permitted reading time
	 * FIXME: make this configurable? */
	integral_max = (uint32_t) SensorIntegralTime * 3;

	/* get a 10% test reading */
	integral = SensorIntegralTime / 10;
	number_edges = CHugTakeReadingsFrequencyRaw(integral, 0);

	/* if we got any reading, scale the integral time to get at
	 * least the minimum number of edges */
	if (number_edges == 0) {
		/* perfect black, so try the hardest we can to get an
		 * accurate reading */
		integral = integral_max;
	} else {
		integral = (edges_min * (uint32_t) SensorIntegralTime) / (uint32_t) number_edges;

		/* we're expected to read for this much time, so for
		 * white light get the best precision available */
		if (integral < SensorIntegralTime)
			integral = SensorIntegralTime;

		/* we can't go higher than this or we'll time out the
		 * USB read */
		else if (integral > integral_max)
			integral = integral_max;
	}

	/* get the number of pulses with the new threshold */
	number_edges = CHugTakeReadingsFrequencyRaw(integral,
						    &last_rising_edge);
	if (number_edges == 0)
		goto out;

	/* pre-multiply so 100% intensity is near 1.0 */
	number_edges *= PreScale.offset;

	/* scale by the integral time */
	CHugScaleByIntegral(&number_edges, last_rising_edge);
out:
	return number_edges;
}

/**
 * CHugTakeReadingsFrequency:
 **/
static uint8_t
CHugTakeReadingsFrequency (CHugPackedFloat *red,
			   CHugPackedFloat *green,
			   CHugPackedFloat *blue)
{
	uint16_t reading;
	uint8_t rc = CH_ERROR_NONE;

	/* set to zero */
	red->raw = 0;
	green->raw = 0;
	blue->raw = 0;

	/* check the device is sane */
	if (SensorSerial == 0xffffffff) {
		rc = CH_ERROR_NO_SERIAL;
		goto out;
	}
	if (DarkCalibration[CH_COLOR_OFFSET_RED] == 0xffff) {
		rc = CH_ERROR_NO_CALIBRATION;
		goto out;
	}

	/* do red */
	CHugSetColorSelect(CH_COLOR_SELECT_RED);
	reading = CHugTakeReadingFrequency();
	if (reading > 0x7fff) {
		rc = CH_ERROR_OVERFLOW_SENSOR;
		goto out;
	}
	if (reading > DarkCalibration[CH_COLOR_OFFSET_RED])
		red->fraction = reading - DarkCalibration[CH_COLOR_OFFSET_RED];

	/* do green */
	CHugSetColorSelect(CH_COLOR_SELECT_GREEN);
	reading = CHugTakeReadingFrequency();
	if (reading > 0x7fff) {
		rc = CH_ERROR_OVERFLOW_SENSOR;
		goto out;
	}
	if (reading > DarkCalibration[CH_COLOR_OFFSET_GREEN])
		green->fraction = reading - DarkCalibration[CH_COLOR_OFFSET_GREEN];

	/* do blue */
	CHugSetColorSelect(CH_COLOR_SELECT_BLUE);
	reading = CHugTakeReadingFrequency();
	if (reading > 0x7fff) {
		rc = CH_ERROR_OVERFLOW_SENSOR;
		goto out;
	}
	if (reading > DarkCalibration[CH_COLOR_OFFSET_BLUE])
		blue->fraction = reading - DarkCalibration[CH_COLOR_OFFSET_BLUE];
out:
	return rc;
}

/**
 * CHugTakeReadingsDuration:
 **/
static uint8_t
CHugTakeReadingsDuration (CHugPackedFloat *red,
			  CHugPackedFloat *green,
			  CHugPackedFloat *blue)
{
	uint32_t tmp;
	uint8_t rc = CH_ERROR_NONE;

	/* clear */
	red->raw = 0;
	green->raw = 0;
	blue->raw = 0;

	/* check the device is sane */
	if (SensorSerial == 0xffffffff) {
		rc = CH_ERROR_NO_SERIAL;
		goto out;
	}

	/* do red */
	CHugSetColorSelect(CH_COLOR_SELECT_RED);
	tmp = CHugTakeReadingDuration (SensorIntegralTime);
	if (tmp > 0)
		red->raw = 0xffffffff / tmp;

	/* do green */
	CHugSetColorSelect(CH_COLOR_SELECT_GREEN);
	tmp = CHugTakeReadingDuration (SensorIntegralTime);
	if (tmp > 0)
		green->raw = 0xffffffff / tmp;

	/* do blue */
	CHugSetColorSelect(CH_COLOR_SELECT_BLUE);
	tmp = CHugTakeReadingDuration (SensorIntegralTime);
	if (tmp > 0)
		blue->raw = 0xffffffff / tmp;
out:
	return rc;
}

/**
 * CHugTakeReadings:
 **/
static uint8_t
CHugTakeReadings (CHugPackedFloat *red,
		  CHugPackedFloat *green,
		  CHugPackedFloat *blue)
{
	uint8_t rc = CH_ERROR_INVALID_VALUE;
	if (MeasureMode == CH_MEASURE_MODE_FREQUENCY) {
		rc = CHugTakeReadingsFrequency(red, green, blue);
		if (rc != CH_ERROR_NONE)
			goto out;
		rc = CHugPackedFloatMultiply(red, &PostScale, red);
		if (rc != CH_ERROR_NONE)
			goto out;
		rc = CHugPackedFloatMultiply(green, &PostScale, green);
		if (rc != CH_ERROR_NONE)
			goto out;
		rc = CHugPackedFloatMultiply(blue, &PostScale, blue);
		if (rc != CH_ERROR_NONE)
			goto out;
		goto out;
	}
	if (MeasureMode == CH_MEASURE_MODE_DURATION) {
		rc = CHugTakeReadingsDuration(red, green, blue);
		goto out;
	}
out:
	return rc;
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
	ReadFlash(addr,
		  9 * sizeof(CHugPackedFloat),
		  (unsigned char *) calibration);
	if (calibration[0].raw == 0xffffffff)
		return CH_ERROR_NO_CALIBRATION;

	/* check the calibration matrix is valid */
	ReadFlash(addr + 0x24, 1,
		  (unsigned char *) &calibration_type);
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
	rc = CHugTakeReadings(&readings[CH_COLOR_OFFSET_RED],
			      &readings[CH_COLOR_OFFSET_GREEN],
			      &readings[CH_COLOR_OFFSET_BLUE]);
	if (rc != CH_ERROR_NONE)
		goto out;

	/* convert to xyz using the factory calibration */
	rc = CHugSwitchCalibrationMatrix(0, calibration);
	if (rc != CH_ERROR_NONE)
		goto out;
	rc = CHugCalibrationMultiply(calibration,
				     readings,
				     readings_tmp);
	if (rc != CH_ERROR_NONE)
		goto out;

	/* use the specified correction matrix */
	if (calibration_index != 0) {
		rc = CHugSwitchCalibrationMatrix(calibration_index,
						 calibration);
		if (rc != CH_ERROR_NONE)
			goto out;
		rc = CHugCalibrationMultiply(calibration,
					     readings_tmp,
					     readings);
		if (rc != CH_ERROR_NONE)
			goto out;
	} else {
		memcpy(readings,
		       (void *) readings_tmp,
		       sizeof(readings));
	}

	/* copy values */
	*x = readings[0];
	*y = readings[1];
	*z = readings[2];
out:
	return rc;
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
	ReadFlash(addr,
		  matrix_size,
		  (unsigned char *) calibration);
	ReadFlash(addr + matrix_size,
		  sizeof(uint8_t),
		  (unsigned char *) types);
	ReadFlash(addr + matrix_size + 1,
		  CH_CALIBRATION_DESCRIPTION_LEN,
		  (unsigned char *) description);
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
		ReadFlash(src + addr,
			  CH_FLASH_WRITE_BLOCK_SIZE,
			  (unsigned char *) FlashBuffer);
		WriteBytesFlash(dest + addr,
				CH_FLASH_WRITE_BLOCK_SIZE,
				(unsigned char *) FlashBuffer);
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
	EraseFlash(CH_CALIBRATION_ADDR_TMP,
		   CH_CALIBRATION_ADDR_TMP + CH_FLASH_ERASE_BLOCK_SIZE);

	/* copy the block to the temporary area */
	CHugCopyFlash(addr_block_start,
		      CH_CALIBRATION_ADDR_TMP,
		      CH_FLASH_ERASE_BLOCK_SIZE);

	/* erase the block */
	EraseFlash(addr_block_start,
		   addr_block_start + CH_FLASH_ERASE_BLOCK_SIZE);

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
	WriteBytesFlash(addr_block_start + offset,
			CH_FLASH_WRITE_BLOCK_SIZE,
			(unsigned char *) FlashBuffer);

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
	uint32_t i;
	uint32_t integral_time = SensorIntegralTime;
	uint8_t idx;
	uint8_t rc;
	uint32_t chunk;
	unsigned char ra_tmp = PORTA;

	/* set all buckets to zero */
	memset(data, 0x00, 30);

	/* do a long integral time to pick up multiple refreshes */
	integral_time *= 10;
	chunk = integral_time / 30;
	for (i = 0; i < integral_time; i++) {
		if (ra_tmp == PORTA)
			continue;
		if (PORTAbits.RA5 != 1)
			continue;
		idx = i / chunk;
		if (data[idx] < 0xff)
			data[idx]++;
		ra_tmp = PORTA;
	}

	/* success */
	rc = CH_ERROR_NONE;
out:
	return rc;
}

/**
 * ChSha1Valid:
 *
 * A hash is only valid when it is first set. It is invalid when all
 * bytes of the hash are 0xff.
 **/
static uint8_t
ChSha1Valid(ChSha1 *sha1)
{
	uint8_t i;
	for (i = 0; i < 20; i++) {
		if (sha1->bytes[i] != 0xff)
			return CH_ERROR_NONE;
	}
	return CH_ERROR_INVALID_VALUE;
}

/**
 * ProcessIO:
 **/
static void
ProcessIO(void)
{
	CHugPackedFloat readings[3];
	uint16_t address;
	uint16_t calibration_index;
	uint32_t reading;
	uint8_t checksum;
	uint8_t i;
	uint8_t length;
	unsigned char cmd;
	unsigned char rc = CH_ERROR_NONE;
	unsigned char reply_len = CH_BUFFER_OUTPUT_DATA;

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
		reply_len += 1;
		break;
	case CH_CMD_GET_COLOR_SELECT:
		TxBuffer[CH_BUFFER_OUTPUT_DATA] = CHugGetColorSelect();
		reply_len += 1;
		break;
	case CH_CMD_SET_COLOR_SELECT:
		CHugSetColorSelect(RxBuffer[CH_BUFFER_INPUT_DATA]);
		break;
	case CH_CMD_GET_LEDS:
		TxBuffer[CH_BUFFER_OUTPUT_DATA] = CHugGetLEDs();
		reply_len += 1;
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
		reply_len += sizeof(uint16_t);
		break;
	case CH_CMD_SET_PCB_ERRATA:
		memcpy (&PcbErrata,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			sizeof(uint16_t));
		break;
	case CH_CMD_GET_MEASURE_MODE:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(void *) &MeasureMode,
			sizeof(uint8_t));
		reply_len += sizeof(uint8_t);
		break;
	case CH_CMD_SET_MEASURE_MODE:
		memcpy (&MeasureMode,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			sizeof(uint8_t));
		break;
	case CH_CMD_GET_REMOTE_HASH:

		/* check is valid */
		rc = ChSha1Valid(&remote_hash);
		if (rc != CH_ERROR_NONE)
			break;
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(void *) &remote_hash,
			sizeof(ChSha1));
		reply_len += sizeof(ChSha1);
		break;
	case CH_CMD_SET_REMOTE_HASH:
		memcpy (&remote_hash,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			sizeof(ChSha1));
		break;
	case CH_CMD_GET_MULTIPLIER:
		TxBuffer[CH_BUFFER_OUTPUT_DATA] = CHugGetMultiplier();
		reply_len += 1;
		break;
	case CH_CMD_SET_MULTIPLIER:
		CHugSetMultiplier(RxBuffer[CH_BUFFER_INPUT_DATA]);
		break;
	case CH_CMD_GET_INTEGRAL_TIME:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(void *) &SensorIntegralTime,
			2);
		reply_len += 2;
		break;
	case CH_CMD_SET_INTEGRAL_TIME:
		memcpy (&SensorIntegralTime,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			2);
		break;
	case CH_CMD_GET_FIRMWARE_VERSION:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			&FirmwareVersion,
			2 * 3);
		reply_len += 2 * 3;
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
		reply_len += (9 * sizeof(CHugPackedFloat)) + 1 + CH_CALIBRATION_DESCRIPTION_LEN;
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
	case CH_CMD_GET_POST_SCALE:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) &PostScale,
			sizeof(CHugPackedFloat));
		reply_len += sizeof(CHugPackedFloat);
		break;
	case CH_CMD_SET_POST_SCALE:
		memcpy ((void *) &PostScale,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			sizeof(CHugPackedFloat));
		break;
	case CH_CMD_GET_PRE_SCALE:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) &PreScale,
			sizeof(CHugPackedFloat));
		reply_len += sizeof(CHugPackedFloat);
		break;
	case CH_CMD_SET_PRE_SCALE:
		memcpy ((void *) &PreScale,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			sizeof(CHugPackedFloat));
		break;
	case CH_CMD_GET_DARK_OFFSETS:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			&DarkCalibration,
			3 * sizeof(uint16_t));
		reply_len += 3 * sizeof(uint16_t);
		break;
	case CH_CMD_SET_DARK_OFFSETS:
		memcpy ((void *) &DarkCalibration,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			3 * sizeof(uint16_t));
		break;
	case CH_CMD_GET_CALIBRATION_MAP:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			&CalibrationMap,
			6 * sizeof(uint16_t));
		reply_len += 6 * sizeof(uint16_t);
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
		reply_len += 4;
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
	case CH_CMD_TAKE_READING_RAW:
		/* take a single reading */
		reading = CHugTakeReadingRaw(SensorIntegralTime);
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) &reading,
			sizeof(uint32_t));
		reply_len += sizeof(uint32_t);
		break;
	case CH_CMD_TAKE_READINGS:
		/* take multiple readings without using the factory
		 * calibration matrix but using post scaling */
		rc = CHugTakeReadings(&readings[CH_COLOR_OFFSET_RED],
				      &readings[CH_COLOR_OFFSET_GREEN],
				      &readings[CH_COLOR_OFFSET_BLUE]);
		if (rc != CH_ERROR_NONE)
			break;
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) readings,
			3 * sizeof(CHugPackedFloat));
		reply_len += 3 * sizeof(CHugPackedFloat);
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
		reply_len += 3 * sizeof(CHugPackedFloat);
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
		EraseFlash(CH_EEPROM_ADDR_FLASH_SUCCESS,
			   CH_EEPROM_ADDR_FLASH_SUCCESS + 1);
		WriteBytesFlash(CH_EEPROM_ADDR_FLASH_SUCCESS, 1,
				(unsigned char *) &RxBuffer[CH_BUFFER_INPUT_DATA]);
		break;
	case CH_CMD_GET_OWNER_NAME:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) OwnerName,
			CH_OWNER_LENGTH_MAX);
		reply_len += CH_OWNER_LENGTH_MAX;
		break;
	case CH_CMD_GET_OWNER_EMAIL:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) OwnerEmail,
			CH_OWNER_LENGTH_MAX);
		reply_len += CH_OWNER_LENGTH_MAX;
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
	case CH_CMD_TAKE_READING_ARRAY:
		rc = CHugTakeReadingArray(&TxBuffer[CH_BUFFER_OUTPUT_DATA]);
		reply_len += 30;
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
					  reply_len);
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
	CHugSetColorSelect(CH_COLOR_SELECT_WHITE);
	CHugSetMultiplier(CH_FREQ_SCALE_0);

	/* read out the sensor data from EEPROM */
	CHugReadEEprom();
}

/**
 * InitializeSystem:
 **/
static void
InitializeSystem(void)
{
#if defined(__18F46J50)
	/* Enable the PLL and wait 2+ms until the PLL locks
	 * before enabling USB module */
	unsigned int pll_startup_counter = 600;
	OSCTUNEbits.PLLEN = 1;
	while (pll_startup_counter--);

	/* default all pins to digital */
	ANCON0 = 0xFF;
	ANCON1 = 0xFF;
#elif defined(__18F4550)
	/* default all pins to digital */
	ADCON1 = 0x0F;
#endif

	/* set RA0, RA1 to output (freq scaling),
	 * set RA2, RA3 to output (color select),
	 * set RA5 to input (frequency counter),
	 * (RA6 is "don't care" in OSC2 mode)
	 * set RA7 to input (OSC1, HSPLL in) */
	TRISA = 0xf0;

	/* set RB0 to RB3 to input (h/w revision) others input (unused) */
	TRISB = 0xff;

	/* set RC0 to RC2 to input (unused) */
	TRISC = 0xff;

	/* set RD0 to RD7 to input (unused) */
	TRISD = 0xff;

	/* set RE0, RE1 output (LEDs) others input (unused) */
	TRISE = 0x3c;

	/* The USB module will be enabled if the bootloader has booted,
	 * so we soft-detach from the host. */
	if(UCONbits.USBEN == 1) {
		UCONbits.SUSPND = 0;
		UCON = 0;
		delay10ktcy(0xff);
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
	/* need to reduce power to < 2.5mA, so power down sensor */
	multiplier_old = CHugGetMultiplier();
	CHugSetMultiplier(CH_FREQ_SCALE_0);

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
	/* restore full power mode */
	CHugSetMultiplier(multiplier_old);
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
	ReadFlash(CH_EEPROM_ADDR_FLASH_SUCCESS, 1,
		  (unsigned char *) &flash_success);

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
