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
#include "HardwareProfile.h"
#include "usb_config.h"
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
static uint8_t idle_command = 0x00;
static uint8_t idle_counter = 0x00;

/* USB buffers */
uint8_t RxBuffer[CH_USB_HID_EP_SIZE];
uint8_t TxBuffer[CH_USB_HID_EP_SIZE];
uint8_t FlashBuffer[CH_FLASH_WRITE_BLOCK_SIZE];

USB_HANDLE	USBOutHandle = 0;
USB_HANDLE	USBInHandle = 0;

/* flash the LEDs when in bootloader mode */
#define	BOOTLOADER_FLASH_INTERVAL	0x1fff
static uint16_t led_counter = 0x0;

/* protect against having a bad flash */
static uint8_t flash_success = 0xff;

/* sensor context */
CHugMcdc04Context ctx;

#pragma code

#define CHugBootFlash()		(((int(*)(void))(CH_EEPROM_ADDR_RUNCODE))())

/**
 * CHugCalculateChecksum:
 **/
static uint8_t
CHugCalculateChecksum(uint8_t *data, uint8_t length)
{
	int i;
	uint8_t checksum = 0xff;
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
static void
ProcessIO(void)
{
	uint16_t address;
	uint16_t erase_length;
	uint8_t length;
	uint8_t checksum;
	uint8_t cmd;
	uint8_t rc = CH_ERROR_NONE;

	/* reset the LED state */
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
	if (HIDRxHandleBusy(USBOutHandle)) {
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
	case CH_CMD_GET_HARDWARE_VERSION:
		TxBuffer[CH_BUFFER_OUTPUT_DATA] = PORTB & 0x0f;
		break;
	case CH_CMD_RESET:
		/* only reset when USB stack is not busy */
		idle_command = CH_CMD_RESET;
		break;
	case CH_CMD_GET_FIRMWARE_VERSION:
		*((uint16_t *) &TxBuffer[CH_BUFFER_OUTPUT_DATA + 0]) = CH_VERSION_MAJOR;
		*((uint16_t *) &TxBuffer[CH_BUFFER_OUTPUT_DATA + 2]) = CH_VERSION_MINOR;
		*((uint16_t *) &TxBuffer[CH_BUFFER_OUTPUT_DATA + 4]) = CH_VERSION_MICRO;
		break;
	case CH_CMD_ERASE_FLASH:
		/* are we lost or stolen */
		if (flash_success == 0xff) {
			rc = CH_ERROR_DEVICE_DEACTIVATED;
			break;
		}
		memcpy (&address,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA+0],
			2);
		/* allow to erase any address but not the bootloader */
		if (address < CH_EEPROM_ADDR_RUNCODE ||
		    address > CH_EEPROM_ADDR_MAX) {
			rc = CH_ERROR_INVALID_ADDRESS;
			break;
		}
		memcpy (&erase_length,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA+2],
			2);
		rc = CHugFlashErase(address, erase_length);
		break;
	case CH_CMD_READ_FLASH:
		/* are we lost or stolen */
		if (flash_success == 0xff) {
			rc = CH_ERROR_DEVICE_DEACTIVATED;
			break;
		}
		/* allow to read any address */
		memcpy (&address,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA+0],
			2);
		length = RxBuffer[CH_BUFFER_INPUT_DATA+2];
		if (length > 60) {
			rc = CH_ERROR_INVALID_LENGTH;
			break;
		}
		CHugFlashRead(address, length,
			      &TxBuffer[CH_BUFFER_OUTPUT_DATA+1]);
		checksum = CHugCalculateChecksum (&TxBuffer[CH_BUFFER_OUTPUT_DATA+1],
						  length);
		TxBuffer[CH_BUFFER_OUTPUT_DATA+0] = checksum;
		break;
	case CH_CMD_WRITE_FLASH:
		/* are we lost or stolen */
		if (flash_success == 0xff) {
			rc = CH_ERROR_DEVICE_DEACTIVATED;
			break;
		}
		/* write to flash that's not the bootloader */
		memcpy (&address,
			(const void *) &RxBuffer[CH_BUFFER_INPUT_DATA+0],
			2);
		if (address < CH_EEPROM_ADDR_RUNCODE ||
		    address > CH_EEPROM_ADDR_MAX) {
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
			rc = CHugFlashWrite(address - CH_FLASH_TRANSFER_BLOCK_SIZE,
					    CH_FLASH_WRITE_BLOCK_SIZE,
					    FlashBuffer);
		}
		break;
	case CH_CMD_BOOT_FLASH:
		/* are we lost or stolen */
		if (flash_success == 0xff) {
			rc = CH_ERROR_DEVICE_DEACTIVATED;
			break;
		}
		/* only boot when USB stack is not busy */
		idle_command = CH_CMD_BOOT_FLASH;
		break;
	case CH_CMD_SET_FLASH_SUCCESS:
		if (RxBuffer[CH_BUFFER_INPUT_DATA] != 0x00) {
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
	case CH_CMD_SELF_TEST:
		rc = CHugSelfTest();
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
					  CH_USB_HID_EP_SIZE);
	}

	/* re-arm the OUT endpoint for the next packet */
	USBOutHandle = HIDRxPacket(HID_EP,
				   (BYTE*)&RxBuffer,
				   CH_USB_HID_EP_SIZE);
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

	/* set the LED state initially */
	PORTE = CH_STATUS_LED_RED;

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
	uint16_t runcode_start = 0xffff;

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
	CHugFlashRead(CH_EEPROM_ADDR_RUNCODE, 2,
		      (uint8_t *) &runcode_start);
	CHugFlashRead(CH_EEPROM_ADDR_FLASH_SUCCESS, 1,
		      (uint8_t *) &flash_success);
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
