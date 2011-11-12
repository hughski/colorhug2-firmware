/* -*- Mode: C; tab-width: 8; indent-tabs-mode: t; c-basic-offset: 8 -*-
 *
 * Copyright (C) 2011 Richard Hughes <richard@hughsie.com>
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

#include <p18cxxx.h>
#include <delays.h>
#include <flash.h>
#include <GenericTypeDefs.h>

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

/* ensure this is incremented on each released build */
static UINT16	FirmwareVersion[3] = { 0, 0, 3 };

#pragma udata

static UINT32	SensorSerial = 0x00000000;
static UINT16	DarkCalibration[3] = { 0x0000, 0x0000, 0x0000 };
static float	SensorCalibration[16] = { 1.0f, 0.0f, 0.0f,
					  0.0f, 1.0f, 0.0f,
					  0.0f, 0.0f, 1.0f };
static UINT16	SensorIntegralTime = 0xffff;

/* USB idle support */
static UINT8 idle_command = 0x00;
static UINT8 idle_counter = 0x00;

/* USB buffers */
unsigned char RxBuffer[CH_USB_HID_EP_SIZE];
unsigned char TxBuffer[CH_USB_HID_EP_SIZE];

USB_HANDLE	USBOutHandle = 0;
USB_HANDLE	USBInHandle = 0;

/* need to save this so we can power down in USB suspend and then power
 * back up in the same mode */
static ChFreqScale multiplier_old = CH_FREQ_SCALE_0;

#pragma code

/* suitable for TDSDB146J50 or TDSDB14550 demo board */
#define LED0			PORTEbits.RE0
#define LED1			PORTEbits.RE1

/**
 * CHugBootFlash:
 **/
static void
CHugBootFlash(void)
{
	_asm
	goto CH_EEPROM_ADDR_RUNCODE
	_endasm
}

/**
 * CHugGetLEDs:
 **/
unsigned char
CHugGetLEDs(void)
{
	return (PORTEbits.RE1 << 1) + PORTEbits.RE0;
}

/**
 * CHugSetLEDs:
 **/
void
CHugSetLEDs(unsigned char leds)
{
	PORTEbits.RE0 = leds;
	PORTEbits.RE1 = leds >> 1;
}

/**
 * CHugGetColorSelect:
 **/
ChColorSelect
CHugGetColorSelect(void)
{
	return (PORTAbits.RA3 << 1) + PORTAbits.RA2;
}

/**
 * CHugSetColorSelect:
 **/
void
CHugSetColorSelect(ChColorSelect color_select)
{
	PORTAbits.RA2 = (color_select & 0x01);
	PORTAbits.RA3 = (color_select & 0x02) >> 1;
}

/**
 * CHugGetMultiplier:
 **/
ChFreqScale
CHugGetMultiplier(void)
{
	return (PORTAbits.RA1 << 1) + PORTAbits.RA0;
}

/**
 * CHugSetMultiplier:
 **/
void
CHugSetMultiplier(ChFreqScale multiplier)
{
	PORTAbits.RA0 = (multiplier & 0x01);
	PORTAbits.RA1 = (multiplier & 0x02) >> 1;
}

/**
 * CHugFatalError:
 **/
static void
CHugFatalError (ChFatalError fatal_error)
{
	char i;

	/* turn off watchdog */
	WDTCONbits.SWDTEN = 0;

	while (1) {
		for (i = 0; i < fatal_error + 2; i++) {
			LED0 = 1;
			Delay10KTCYx(0xFF);
			LED0 = 0;
			Delay10KTCYx(0xFF);
		}
		Delay10KTCYx(0xFF);
		Delay10KTCYx(0xFF);
	}
}

/**
 * CHugReadEEprom:
 **/
void
CHugReadEEprom(void)
{
	/* read this into RAM so it can be changed */
	ReadFlash(CH_EEPROM_ADDR_SERIAL,
		  4, (unsigned char *) &SensorSerial);
	ReadFlash(CH_EEPROM_ADDR_CALIBRATION_MATRIX,
		  9 * 4, (unsigned char *) SensorCalibration);
	ReadFlash(CH_EEPROM_ADDR_DARK_OFFSET_RED,
		  2 * 3, (unsigned char *) DarkCalibration);
}

/**
 * CHugWriteEEprom:
 **/
void
CHugWriteEEprom(void)
{
	/* we can't call this more than 10,000 times otherwise we'll
	 * burn out the device */
	EraseFlash(CH_EEPROM_ADDR,
		   CH_EEPROM_ADDR + 0x400);
	WriteBytesFlash(CH_EEPROM_ADDR_SERIAL,
			4, (unsigned char *) &SensorSerial);
	WriteBytesFlash(CH_EEPROM_ADDR_DARK_OFFSET_RED,
			2 * 3, (unsigned char *) DarkCalibration);
	WriteBytesFlash(CH_EEPROM_ADDR_CALIBRATION_MATRIX,
			9 * sizeof(float), (unsigned char *) SensorCalibration);
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
CHugScaleByIntegral (UINT16 *pulses)
{
	UINT32 tmp;

	/* do this as integer math for speed */
	tmp = (UINT32) *pulses * 0xffff;
	*pulses = tmp / (UINT32) SensorIntegralTime;
}

/**
 * CHugTakeReading:
 **/
static UINT16
CHugTakeReading (void)
{
	UINT16 i;
	UINT16 cnt = 0;
	unsigned char ra_tmp = PORTA;

	/* count how many times the output state changes */
	for (i = 0; i < SensorIntegralTime; i++) {
		if (ra_tmp != PORTA) {
			cnt++;
			ra_tmp = PORTA;
		}
	}

	/* scale by the integral time */
	CHugScaleByIntegral(&cnt);

	return cnt;
}

/**
 * CHugTakeReadings:
 **/
static UINT8
CHugTakeReadings (UINT16 *red, UINT16 *green, UINT16 *blue)
{
	UINT16 reading;
	UINT8 retval = CH_FATAL_ERROR_UNDERFLOW;

	/* check the device is sane */
	if (SensorSerial == 0xffffffff) {
		retval = CH_FATAL_ERROR_NO_SERIAL;
		goto out;
	}
	if (DarkCalibration[CH_COLOR_OFFSET_RED] == 0xffff) {
		retval = CH_FATAL_ERROR_NO_SERIAL;
		goto out;
	}

	/* do red */
	CHugSetColorSelect(CH_COLOR_SELECT_RED);
	reading = CHugTakeReading();
	if (reading < DarkCalibration[CH_COLOR_OFFSET_RED])
		goto out;
	*red = reading - DarkCalibration[CH_COLOR_OFFSET_RED];

	/* do green */
	CHugSetColorSelect(CH_COLOR_SELECT_GREEN);
	reading = CHugTakeReading();
	if (reading < DarkCalibration[CH_COLOR_OFFSET_GREEN])
		goto out;
	*green = reading - DarkCalibration[CH_COLOR_OFFSET_GREEN];

	/* do blue */
	CHugSetColorSelect(CH_COLOR_SELECT_BLUE);
	reading = CHugTakeReading();
	if (reading < DarkCalibration[CH_COLOR_OFFSET_BLUE])
		goto out;
	*blue = reading - DarkCalibration[CH_COLOR_OFFSET_BLUE];

	/* success */
	retval = CH_FATAL_ERROR_NONE;
out:
	return retval;
}

/**
 * CHugVectorMultiply:
 **/
static void
CHugVectorMultiply (const float *cal,
		    const float *device_rgb,
		    float *xyz)
{
	xyz[0] = cal[0] * device_rgb[0] +
		 cal[1] * device_rgb[1] +
		 cal[2] * device_rgb[2];
	xyz[1] = cal[3] * device_rgb[0] +
		 cal[4] * device_rgb[1] +
		 cal[5] * device_rgb[2];
	xyz[2] = cal[6] * device_rgb[0] +
		 cal[7] * device_rgb[1] +
		 cal[8] * device_rgb[2];
}

/**
 * CHugTakeReadingsXYZ:
 **/
static UINT8
CHugTakeReadingsXYZ (float *red, float *green, float *blue)
{
	float readings_f[3];
	float readings_xyz[3];
	UINT16 readings[3];
	UINT8 i;
	UINT8 retval;

	/* get integer readings */
	retval = CHugTakeReadings(&readings[CH_COLOR_OFFSET_RED],
				  &readings[CH_COLOR_OFFSET_GREEN],
				  &readings[CH_COLOR_OFFSET_BLUE]);

	/* convert to floats */
	for (i = 0; i < 3; i++)
		readings_f[i] = (float) readings[i] / (float) 0xffff;

	/* convert to xyz */
	CHugVectorMultiply(SensorCalibration,
			   readings_f,
			   readings_xyz);

	/* copy values */
	*red = readings_xyz[CH_COLOR_OFFSET_RED];
	*green = readings_xyz[CH_COLOR_OFFSET_GREEN];
	*blue = readings_xyz[CH_COLOR_OFFSET_BLUE];

	return retval;
}

/**
 * CHugCalculateChecksum:
 **/
static UINT8
CHugCalculateChecksum(UINT8 *data, UINT8 length)
{
	int i;
	UINT8 checksum = 0xff;
	for (i = 0; i < length; i++)
		checksum ^= data[i];
	return checksum;
}

/**
 * CHugDeviceIdle:
 **/
static void
CHugDeviceIdle(void)
{
	if (idle_command == CH_CMD_RESET)
		Reset();
}

/**
 * ProcessIO:
 **/
void
ProcessIO(void)
{
	float readings_xyz[3];
	UINT16 address;
	UINT16 readings[3];
	UINT8 length;
	UINT8 checksum;
	unsigned char cmd;
	unsigned char reply_len = CH_BUFFER_OUTPUT_DATA;
	unsigned char retval = CH_FATAL_ERROR_NONE;

	LATD++;

	/* User Application USB tasks */
	if ((USBDeviceState < CONFIGURED_STATE) ||
	    (USBSuspendControl == 1))
		return;

	/* no data was received */
	if(HIDRxHandleBusy(USBOutHandle)) {
		if (idle_counter++ == 0xff)
			CHugDeviceIdle();
		return;
	}

	/* got data, reset idle counter */
	idle_counter = 0;

#ifdef __DEBUG
	/* clear for debugging */
	memset (TxBuffer, 0xff, sizeof (TxBuffer));
#endif

	cmd = RxBuffer[CH_BUFFER_INPUT_CMD];
	switch(cmd) {
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
		CHugSetLEDs(RxBuffer[CH_BUFFER_INPUT_DATA]);
		break;
	case CH_CMD_GET_MULTIPLIER:
		TxBuffer[CH_BUFFER_OUTPUT_DATA] = CHugGetMultiplier();
		reply_len += 1;
		break;
	case CH_CMD_SET_MULTIPLIER:
		CHugSetMultiplier(RxBuffer[CH_BUFFER_INPUT_DATA]);
		break;
	case CH_CMD_GET_INTERGRAL_TIME:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(void *) &SensorIntegralTime,
			2);
		reply_len += 2;
		break;
	case CH_CMD_SET_INTERGRAL_TIME:
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
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) SensorCalibration,
			9 * sizeof(float));
		reply_len += 9 * sizeof(float);
		break;
	case CH_CMD_SET_CALIBRATION:
		memcpy ((void *) SensorCalibration,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			9 * sizeof(float));
		break;
	case CH_CMD_GET_DARK_OFFSETS:
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			&DarkCalibration,
			2 * 3);
		reply_len += 2 * 3;
		break;
	case CH_CMD_SET_DARK_OFFSETS:
		memcpy ((void *) &DarkCalibration,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
			2 * 3);
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
			/* copy the magic for debugging */
			memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
				(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA],
				8);
			retval = 1;
		}
		break;
	case CH_CMD_TAKE_READING_RAW:
		/* take a single reading */
		readings[0] = CHugTakeReading();
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) &readings[0],
			2);
		reply_len += 2;
		break;
	case CH_CMD_TAKE_READINGS:
		/* take multiple readings */
		retval = CHugTakeReadings(&readings[CH_COLOR_OFFSET_RED],
					  &readings[CH_COLOR_OFFSET_GREEN],
					  &readings[CH_COLOR_OFFSET_BLUE]);
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) readings,
			2 * 3);
		reply_len += 6;
		break;
	case CH_CMD_TAKE_READING_XYZ:
		/* take multiple readings and multiply with the
		 * calibration matrix */
		retval = CHugTakeReadingsXYZ(&readings_xyz[CH_COLOR_OFFSET_RED],
					     &readings_xyz[CH_COLOR_OFFSET_GREEN],
					     &readings_xyz[CH_COLOR_OFFSET_BLUE]);
		memcpy (&TxBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) readings_xyz,
			4*3);
		reply_len += 4*3;
		break;
	case CH_CMD_RESET:
		/* only reset when USB stack is not busy */
		idle_command = CH_CMD_RESET;
		break;
	case CH_CMD_READ_FLASH:
		/* allow to read any address */
		memcpy (&address,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA+0],
			2);
		length = RxBuffer[CH_BUFFER_INPUT_DATA+2];
		if (length > 60) {
			retval = CH_FATAL_ERROR_INVALID_LENGTH;
			break;
		}
		ReadFlash(address, length,
			  (unsigned char *) &TxBuffer[CH_BUFFER_OUTPUT_DATA+1]);
		checksum = CHugCalculateChecksum (&TxBuffer[CH_BUFFER_OUTPUT_DATA+1],
						  length);
		TxBuffer[CH_BUFFER_OUTPUT_DATA+0] = checksum;
		reply_len += length + 1;
		break;
	case CH_CMD_WRITE_FLASH:
		/* write to flash that's not the bootloader */
		memcpy (&address,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA+0],
			2);
		if (address < CH_EEPROM_ADDR_RUNCODE) {
			retval = CH_FATAL_ERROR_INVALID_ADDRESS;
			break;
		}
		length = RxBuffer[CH_BUFFER_INPUT_DATA+2];
		if (length > 59) {
			retval = CH_FATAL_ERROR_INVALID_LENGTH;
			break;
		}
		checksum = CHugCalculateChecksum(&RxBuffer[CH_BUFFER_INPUT_DATA+4],
						 length);
		if (checksum != RxBuffer[CH_BUFFER_INPUT_DATA+3]) {
			retval = CH_FATAL_ERROR_INVALID_CHECKSUM;
			break;
		}
		EraseFlash(address, address + length);
		WriteBytesFlash(address, length,
				(unsigned char *) &RxBuffer[CH_BUFFER_INPUT_DATA+4]);
		break;
	default:
		retval = CH_FATAL_ERROR_UNKNOWN_CMD;
		break;
	}

	/* always send return code */
	if(!HIDTxHandleBusy(USBInHandle)) {
		TxBuffer[CH_BUFFER_OUTPUT_RETVAL] = retval;
		TxBuffer[CH_BUFFER_OUTPUT_CMD] = cmd;
		USBInHandle = HIDTxPacket(HID_EP,
					  (BYTE*)&TxBuffer[0],
					  reply_len);
	}

	/* re-arm the OUT endpoint for the next packet */
	USBOutHandle = HIDRxPacket(HID_EP,
				   (BYTE*)&RxBuffer,
				   CH_USB_HID_EP_SIZE);
}

/**
 * UserInit:
 **/
void
UserInit(void)
{
	/* set some defaults to power down the sensor */
	CHugSetLEDs(3);
	CHugSetColorSelect(CH_COLOR_SELECT_WHITE);
	CHugSetMultiplier(CH_FREQ_SCALE_0);

	/* read out the sensor data from EEPROM */
	CHugReadEEprom();
}

/**
 * InitializeSystem:
 **/
void
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
	 * set RA4 to input (frequency counter),
	 * set RA5 to input (unused) */
	TRISA = 0xf0;

	/* set RB2, RB3 to input (switches) others input (unused) */
	TRISB = 0xff;

	/* set RC0 to RC2 to input (unused) */
	TRISC = 0xff;

	/* set RD0 to RD7 to output (freq test) */
	TRISD = 0x00;

	/* set RE0, RE1 output (LEDs) others input (unused) */
	TRISE = 0x3c;

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
	CHugSetLEDs(0);
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
void
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
void
USBCBCheckOtherReq(void)
{
	USBCheckHIDRequest();
}

/**
 * USBCBInitEP:
 *
 * Called when the host sends a SET_CONFIGURATION.
 **/
void
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
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
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
	UINT16 runcode_start = 0xffff;

	/* boot into the flashed program if we didn't do soft-reset
	 * and the flashed program exists */
	ReadFlash(CH_EEPROM_ADDR_RUNCODE, 2,
		  (unsigned char *) &runcode_start);
	if (RCONbits.NOT_RI && runcode_start != 0xffff)
		CHugBootFlash();

	InitializeSystem();

	/* the watchdog saved us from our doom */
	if (!RCONbits.NOT_TO)
		CHugFatalError(CH_FATAL_ERROR_WATCHDOG);

	while(1) {

		/* clear watchdog */
		ClrWdt();

#if defined(USB_POLLING)
		/* check bus status and service USB interrupts */
		USBDeviceTasks();
#endif

		ProcessIO();
	}
}
