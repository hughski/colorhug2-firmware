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
#if defined(PIC18F46J50_PIM)
#pragma config XINST	= OFF		/* turn off extended instruction set */
#pragma config STVREN	= ON		/* Stack overflow reset */
#pragma config PLLDIV	= 3		/* (12 MHz crystal used on this board) */
#pragma config WDTEN	= OFF		/* Watch Dog Timer (WDT) */
#pragma config CP0	= OFF		/* Code protect */
#pragma config OSC	= HSPLL		/* HS oscillator, PLL enabled, HSPLL used by USB */
#pragma config CPUDIV	= OSC1		/* OSC1 = divide by 1 mode */
#pragma config IESO	= OFF		/* Internal External (clock) Switchover */
#pragma config FCMEN	= ON		/* Fail Safe Clock Monitor */
#pragma config T1DIG	= ON		/* secondary clock Source */
#pragma config LPT1OSC	= OFF		/* low power timer*/
#pragma config WDTPS	= 32768		/* Watchdog Timer Postscaler */
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

#pragma interrupt	CHugHighPriorityISRCode
#pragma interruptlow	CHugLowPriorityISRCode

/* The 18F46J50 does not have real EEPROM, so we fake some by using the
 * program flash. This is rated at 10,000 erase cycles which will be
 * fine, considering a device will be calibrated usually only once */
#define	EEPROM_ADDR		0xfbf8

#pragma udata

static DWORD	SensorSerial = 0;
static WORD	SensorVersion[3] = { 0, 0, 0 };
static float	SensorCalibration[16] = { 1.0f, 0.0f, 0.0f,
					  0.0f, 1.0f, 0.0f,
					  0.0f, 0.0f, 1.0f };
static WORD	SensorIntegralTime = 0;

/* USB buffers */
unsigned char ReceivedDataBuffer[CH_USB_HID_EP_SIZE];
unsigned char ToSendDataBuffer[CH_USB_HID_EP_SIZE];

USB_HANDLE	USBOutHandle = 0;
USB_HANDLE	USBInHandle = 0;

/* need to save this so we can power down in USB suspend and then power
 * back up in the same mode */
static ChFreqScale multiplier_old = CH_FREQ_SCALE_0;

#pragma code

/* suitable for TDSDB146J50 or TDSDB14550 demo board */
#define BUTTON2			(PORTBbits.RB2 == 0)
#define BUTTON3			(PORTBbits.RB3 == 0)
#define LED0			PORTEbits.RE0
#define LED1			PORTEbits.RE1

/**
 * CHugHighPriorityISRCode:
 *
 * Returns with "retfie fast" as interrupt
 **/
void
CHugHighPriorityISRCode()
{
#if defined(USB_INTERRUPT)
	USBDeviceTasks();
#endif
}

/**
 * CHugLowPriorityISRCode:
 *
 * Returns with "retfie" as interruptlow
 **/
void
CHugLowPriorityISRCode()
{
	/* nothing to do */
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
	ReadFlash(EEPROM_ADDR + CH_EEPROM_ADDR_SERIAL,
		  4, (unsigned char *) &SensorSerial);
	ReadFlash(EEPROM_ADDR + CH_EEPROM_ADDR_FIRMWARE_MAJOR,
		  2, (unsigned char *) &SensorVersion[0]);
	ReadFlash(EEPROM_ADDR + CH_EEPROM_ADDR_FIRMWARE_MINOR,
		  4, (unsigned char *) &SensorVersion[1]);
	ReadFlash(EEPROM_ADDR + CH_EEPROM_ADDR_FIRMWARE_MICRO,
		  4, (unsigned char *) &SensorVersion[2]);
	ReadFlash(EEPROM_ADDR + CH_EEPROM_ADDR_CALIBRATION_MATRIX,
		  9 * 4, (unsigned char *) SensorCalibration);
}

/**
 * CHugWriteEEprom:
 **/
void
CHugWriteEEprom(void)
{
	/* we can't call this more than 10,000 times otherwise we'll
	 * burn out the device */
	EraseFlash(EEPROM_ADDR,
		   EEPROM_ADDR + 0x400);
	WriteBytesFlash(EEPROM_ADDR + CH_EEPROM_ADDR_SERIAL,
			4, (unsigned char *) &SensorSerial);
	WriteBytesFlash(EEPROM_ADDR + CH_EEPROM_ADDR_FIRMWARE_MAJOR,
			2, (unsigned char *) &SensorVersion[0]);
	WriteBytesFlash(EEPROM_ADDR + CH_EEPROM_ADDR_FIRMWARE_MINOR,
			2, (unsigned char *) &SensorVersion[1]);
	WriteBytesFlash(EEPROM_ADDR + CH_EEPROM_ADDR_FIRMWARE_MICRO,
			2, (unsigned char *) &SensorVersion[2]);
	WriteBytesFlash(EEPROM_ADDR + CH_EEPROM_ADDR_CALIBRATION_MATRIX,
			9 * sizeof(float), (unsigned char *) SensorCalibration);
}

/**
 * IsMagicUnicorn:
 **/
static char
IsMagicUnicorn(const char *text)
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
 * ProcessIO:
 **/
void
ProcessIO(void)
{
	unsigned char cmd;
	unsigned char reply_len = CH_BUFFER_OUTPUT_DATA;
	unsigned char retval = CH_FATAL_ERROR_NONE;

	if (BUTTON3) {
		CHugFatalError(CH_FATAL_ERROR_UNKNOWN_CMD);
		CHugWriteEEprom();
	}
	LATD++;

	/* User Application USB tasks */
	if ((USBDeviceState < CONFIGURED_STATE) ||
	    (USBSuspendControl == 1))
		return;

	/* no data was received */
	if(HIDRxHandleBusy(USBOutHandle))
		return;

#ifdef __DEBUG
	/* clear for debugging */
	memset (ToSendDataBuffer, 0xff, sizeof (ToSendDataBuffer));
#endif

	cmd = ReceivedDataBuffer[CH_BUFFER_INPUT_CMD];
	switch(cmd) {
	case CH_CMD_GET_COLOR_SELECT:
		ToSendDataBuffer[CH_BUFFER_OUTPUT_DATA] = CHugGetColorSelect();
		reply_len += 1;
		break;
	case CH_CMD_SET_COLOR_SELECT:
		CHugSetColorSelect(ReceivedDataBuffer[CH_BUFFER_INPUT_DATA]);
		break;
	case CH_CMD_GET_LEDS:
		ToSendDataBuffer[CH_BUFFER_OUTPUT_DATA] = CHugGetLEDs();
		reply_len += 1;
		break;
	case CH_CMD_SET_LEDS:
		CHugSetLEDs(ReceivedDataBuffer[CH_BUFFER_INPUT_DATA]);
		break;
	case CH_CMD_GET_MULTIPLIER:
		ToSendDataBuffer[CH_BUFFER_OUTPUT_DATA] = CHugGetMultiplier();
		reply_len += 1;
		break;
	case CH_CMD_SET_MULTIPLIER:
		CHugSetMultiplier(ReceivedDataBuffer[CH_BUFFER_INPUT_DATA]);
		break;
	case CH_CMD_GET_INTERGRAL_TIME:
		memcpy (&ToSendDataBuffer[CH_BUFFER_OUTPUT_DATA],
			(void *) &SensorIntegralTime,
			2);
		reply_len += 2;
		break;
	case CH_CMD_SET_INTERGRAL_TIME:
		memcpy (&SensorIntegralTime,
			(const void *) &ReceivedDataBuffer[CH_BUFFER_INPUT_DATA],
			2);
		break;
	case CH_CMD_GET_FIRMWARE_VERSION:
		memcpy (&ToSendDataBuffer[CH_BUFFER_OUTPUT_DATA],
			&SensorVersion,
			2 * 3);
		reply_len += 2 * 3;
		break;
	case CH_CMD_SET_FIRMWARE_VERSION:
		memcpy ((void *) &SensorVersion,
			(const void *) &ReceivedDataBuffer[CH_BUFFER_INPUT_DATA],
			2 * 3);
		break;
	case CH_CMD_GET_CALIBRATION:
		memcpy (&ToSendDataBuffer[CH_BUFFER_OUTPUT_DATA],
			&SensorCalibration,
			9 * sizeof(float));
		reply_len += 9 * sizeof(float);
		break;
	case CH_CMD_SET_CALIBRATION:
		memcpy ((void *) &SensorCalibration,
			(const void *) &ReceivedDataBuffer[CH_BUFFER_INPUT_DATA],
			9 * sizeof(float));
		break;
	case CH_CMD_GET_SERIAL_NUMBER:
		memcpy (&ToSendDataBuffer[CH_BUFFER_OUTPUT_DATA],
			(const void *) &SensorSerial,
			4);
		reply_len += 4;
		break;
	case CH_CMD_SET_SERIAL_NUMBER:
		memcpy (&SensorSerial,
			(const void *) &ReceivedDataBuffer[CH_BUFFER_INPUT_DATA],
			4);
		break;
	case CH_CMD_WRITE_EEPROM:
		/* verify the magic matched */
		if (IsMagicUnicorn ((const char *) &ReceivedDataBuffer[CH_BUFFER_INPUT_DATA])) {
			CHugWriteEEprom();
		} else {
			/* copy the magic for debugging */
			memcpy (&ToSendDataBuffer[CH_BUFFER_OUTPUT_DATA],
				(const void *) &ReceivedDataBuffer[CH_BUFFER_INPUT_DATA],
				8);
			retval = 1;
		}
		break;
	case CH_CMD_TAKE_READING:
		/* TODO */
		reply_len += 0;
		break;
	case CH_CMD_TAKE_READING_XYZ:
		/* TODO */
		retval = CH_FATAL_ERROR_NOT_IMPLEMENTED;
		reply_len += 4*3*3;
		break;
	default:
		retval = CH_FATAL_ERROR_UNKNOWN_CMD;
		break;
	}

	/* always send return code */
	if(!HIDTxHandleBusy(USBInHandle)) {
		ToSendDataBuffer[CH_BUFFER_OUTPUT_RETVAL] = retval;
		ToSendDataBuffer[CH_BUFFER_OUTPUT_CMD] = cmd;
		USBInHandle = HIDTxPacket(HID_EP,
					  (BYTE*)&ToSendDataBuffer[0],
					  reply_len);
	}

	/* re-arm the OUT endpoint for the next packet */
	USBOutHandle = HIDRxPacket(HID_EP,
				   (BYTE*)&ReceivedDataBuffer,
				   CH_USB_HID_EP_SIZE);
}

/**
 * UserInit:
 **/
void
UserInit(void)
{
	/* set some defaults to power down the sensor */
	CHugSetLEDs(0);
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
	tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif

	/* we're self powered */
#if defined(USE_SELF_POWER_SENSE_IO)
	tris_self_power = INPUT_PIN;  // See HardwareProfile.h
#endif

	/* do all user init code */
	UserInit();

	/* Initializes USB module SFRs and firmware variables to known states */
	USBDeviceInit();

#if defined(USB_INTERRUPT)
	USBDeviceAttach();
#endif
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
 * USBCB_SOF_Handler:
 *
 * The USB host sends out a SOF packet to full-speed devices every 1 ms.
 **/
void
USBCB_SOF_Handler(void)
{
}

/**
 * USBCBErrorHandler:
 *
 * The purpose of this callback is mainly for debugging during
 * development. Check UEIR to see which error causes the interrupt.
 **/
void
USBCBErrorHandler(void)
{
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
 * USBCBStdSetDscHandler:
 *
 * SET_DESCRIPTOR request, not used
 **/
void
USBCBStdSetDscHandler(void)
{
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
				   (BYTE*)&ReceivedDataBuffer,
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
	case EVENT_SOF:
		USBCB_SOF_Handler();
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
	case EVENT_SET_DESCRIPTOR:
		USBCBStdSetDscHandler();
		break;
	case EVENT_EP0_REQUEST:
		USBCBCheckOtherReq();
		break;
	case EVENT_BUS_ERROR:
		USBCBErrorHandler();
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

	while(1) {

#if defined(USB_POLLING)
		/* check bus status and service USB interrupts */
		USBDeviceTasks();
#endif

		ProcessIO();
	}
}
