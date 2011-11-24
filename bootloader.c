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

#pragma code High_ISR = 0x08
void
CHugHighPriorityISRCode (void)
{
	_asm goto CH_EEPROM_ADDR_HIGH_INTERRUPT _endasm
}

#pragma code Low_ISR = 0x18
void
CHugLowPriorityISRCode (void)
{
	_asm goto CH_EEPROM_ADDR_LOW_INTERRUPT _endasm
}

#pragma rom

#pragma udata

/* USB idle support */
static UINT8 idle_command = 0x00;
static UINT8 idle_counter = 0x00;

/* USB buffers */
unsigned char RxBuffer[CH_USB_HID_EP_SIZE];
unsigned char TxBuffer[CH_USB_HID_EP_SIZE];
unsigned char FlashBuffer[CH_FLASH_WRITE_BLOCK_SIZE];

USB_HANDLE	USBOutHandle = 0;
USB_HANDLE	USBInHandle = 0;

/* flash the LEDs when in bootloader mode */
#define	BOOTLOADER_FLASH_INTERVAL	0x2fff
static UINT16 led_counter = 0x0;

#pragma code

#define CHugBootFlash()		(((int(*)(void))(CH_EEPROM_ADDR_RUNCODE))())

/**
 * CHugFatalError:
 **/
static void
CHugFatalError (ChError error)
{
	char i;

	/* turn off watchdog */
	WDTCONbits.SWDTEN = 0;
	TRISE = 0x3c;

	while (1) {
		for (i = 0; i < error; i++) {
			PORTE = 0x01;
			Delay10KTCYx(0xff);
			PORTE = 0x00;
			Delay10KTCYx(0xff);
		}
		Delay10KTCYx(0xff);
		Delay10KTCYx(0xff);
	}
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
	switch (idle_command) {
	case CH_CMD_RESET:
		Reset();
		break;
	case CH_CMD_BOOT_FLASH:
		CHugBootFlash();
		CHugFatalError(CH_ERROR_NOT_IMPLEMENTED);
		break;
	}
	idle_command = 0x00;
}

/**
 * ProcessIO:
 **/
void
ProcessIO(void)
{
	UINT16 address;
	UINT8 length;
	UINT8 checksum;
	unsigned char cmd;
	unsigned char rc = CH_ERROR_NONE;
	unsigned char reply_len = CH_BUFFER_OUTPUT_DATA;

	/* reset the LED state */
	if (PORTE != 0x01 && PORTE != 0x02)
		PORTE = 0x01;
	led_counter--;
	if (led_counter == 0) {
		PORTE ^= 0x03;
		led_counter = BOOTLOADER_FLASH_INTERVAL;
	}

	/* User Application USB tasks */
	if ((USBDeviceState < CONFIGURED_STATE) ||
	    (USBSuspendControl == 1))
		return;

	/* no data was received */
	if(HIDRxHandleBusy(USBOutHandle)) {
		if (idle_counter++ == 0xff &&
		    idle_command != 0x00)
			CHugDeviceIdle();
		return;
	}

	/* got data, reset idle counter */
	idle_counter = 0;

	/* clear for debugging */
	memset (TxBuffer, 0xff, sizeof (TxBuffer));

	cmd = RxBuffer[CH_BUFFER_INPUT_CMD];
	switch(cmd) {
	case CH_CMD_RESET:
		/* only reset when USB stack is not busy */
		idle_command = CH_CMD_RESET;
		break;
	case CH_CMD_ERASE_FLASH:
		/* allow to read any address */
		memcpy (&address,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA+0],
			2);
		length = RxBuffer[CH_BUFFER_INPUT_DATA+2];
		EraseFlash(address, address + length);
		break;
	case CH_CMD_READ_FLASH:
		/* allow to read any address */
		memcpy (&address,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA+0],
			2);
		length = RxBuffer[CH_BUFFER_INPUT_DATA+2];
		if (length > 60) {
			rc = CH_ERROR_INVALID_LENGTH;
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
			rc = CH_ERROR_INVALID_ADDRESS;
			break;
		}
		length = RxBuffer[CH_BUFFER_INPUT_DATA+2];
		if (length > CH_FLASH_TRANSFER_BLOCK_SIZE) {
			rc = CH_ERROR_INVALID_LENGTH;
			break;
		}
		checksum = CHugCalculateChecksum(&RxBuffer[CH_BUFFER_INPUT_DATA+4],
						 length);
		if (checksum != RxBuffer[CH_BUFFER_INPUT_DATA+3]) {
			rc = CH_ERROR_INVALID_CHECKSUM;
			break;
		}

		/* copy low 32 bytes into flash buffer, and only write
		 * in 64 byte chunks as this is a limitation of the
		 * hardware */
		if ((address & CH_FLASH_TRANSFER_BLOCK_SIZE) == 0) {
			memset (FlashBuffer,
				0xff,
				CH_FLASH_WRITE_BLOCK_SIZE);
			memcpy (FlashBuffer,
				(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA+4],
				length);
		} else {
			memcpy (FlashBuffer + CH_FLASH_TRANSFER_BLOCK_SIZE,
				(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA+4],
				length);
			WriteBytesFlash(address - CH_FLASH_TRANSFER_BLOCK_SIZE,
					CH_FLASH_WRITE_BLOCK_SIZE,
					(unsigned char *) FlashBuffer);
		}
		break;
	case CH_CMD_BOOT_FLASH:
		/* only boot when USB stack is not busy */
		idle_command = CH_CMD_BOOT_FLASH;
		break;
	case CH_CMD_SET_FLASH_SUCCESS:
		if (RxBuffer[CH_BUFFER_INPUT_DATA] != 0x00) {
			rc = CH_ERROR_INVALID_VALUE;
			break;
		}
		EraseFlash(CH_EEPROM_ADDR_FLASH_SUCCESS,
			   CH_EEPROM_ADDR_FLASH_SUCCESS + 1);
		WriteBytesFlash(CH_EEPROM_ADDR_FLASH_SUCCESS, 1,
				(unsigned char *) &RxBuffer[CH_BUFFER_INPUT_DATA]);
		break;
	default:
		rc = CH_ERROR_UNKNOWN_CMD_FOR_BOOTLOADER;
		break;
	}

	/* always send return code */
	if(!HIDTxHandleBusy(USBInHandle)) {
		TxBuffer[CH_BUFFER_OUTPUT_RETVAL] = rc;
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

	/* set RD0 to RD7 to input (unused) */
	TRISD = 0xff;

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

	/* Initializes USB module SFRs and firmware variables to known states */
	USBDeviceInit();
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
	UINT8 flash_success = 0xff;

	/* stack overflow / underflow */
	if (STKPTRbits.STKFUL || STKPTRbits.STKUNF)
		CHugFatalError(CH_ERROR_OVERFLOW_STACK);

	/* the watchdog saved us from our doom */
	if (!RCONbits.NOT_TO)
		CHugFatalError(CH_ERROR_WATCHDOG);

	/*
	 * Boot into the flashed program if all of these are true:
	 *  1. we didn't do soft-reset
	 *  2. the flashed program exists
	 *  3. the flash success is 0x01
	 */
	ReadFlash(CH_EEPROM_ADDR_RUNCODE, 2,
		  (unsigned char *) &runcode_start);
	ReadFlash(CH_EEPROM_ADDR_FLASH_SUCCESS, 1,
		  (unsigned char *) &flash_success);
	if (RCONbits.NOT_RI &&
	    runcode_start != 0xffff &&
	    flash_success == 0x01)
		CHugBootFlash();

	InitializeSystem();

	while(1) {

		/* clear watchdog */
		ClrWdt();

		/* check bus status and service USB interrupts */
		USBDeviceTasks();

		ProcessIO();
	}
}

/**
 * CHugBootFlashTemplate:
 *
 * Placeholder incase the user uses BootFlash when there is no firmware
 * image. The real firmware will overwrite this with the custom code.
 **/
#pragma code user_app_vector=CH_EEPROM_ADDR_RUNCODE
void
CHugBootFlashTemplate(void)
{
	CHugFatalError(CH_ERROR_NOT_IMPLEMENTED);
}
