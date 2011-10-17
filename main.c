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
 */

#include <p18cxxx.h>
#include <delays.h>

#include "ColorHug.h"

#pragma config XINST	= OFF		/* turn off extended instruction set */
#pragma config STVREN	= ON		/* Stack overflow reset */
#pragma config PLLDIV	= 3		/* (12 MHz crystal used on this board) */
#pragma config WDTEN	= OFF		/* Watch Dog Timer (WDT) */
#pragma config CP0	= OFF		/* Code protect */
#pragma config CPUDIV	= OSC1		/* OSC1 = divide by 1 mode */
#pragma config IESO	= OFF		/* Internal External (clock) Switchover */
#pragma config FCMEN	= OFF		/* Fail Safe Clock Monitor */

#pragma code

/* suitable for TDSDB146J50 or TDSDB14550 demo board */
#define BUTTON2			LATBbits.LATB2
#define BUTTON3			LATBbits.LATB3
#define LED0			LATEbits.LATE0
#define LED1			LATEbits.LATE1

/**
 * CHugSetColorSelect:
 **/
void
CHugSetColorSelect(ChColorSelect color_select)
{
	LATAbits.LATA0 = color_select & 0x01;
	LATAbits.LATA1 = color_select & 0x02;
}

/**
 * CHugSetMultiplier:
 **/
void
CHugSetMultiplier(ChFreqScale multiplier)
{
	LATAbits.LATA2 = multiplier & 0x01;
	LATAbits.LATA3 = multiplier & 0x02;
}

/**
 * CHugSetMultiplier:
 **/
static void
CHugFatalError (ChFatalError fatal_error)
{
	char i;
	while (1) {
		for (i = 0; i < fatal_error + 2; i++) {
			LED0 = 1;
			Delay10KTCYx(1);
			LED0 = 0;
			Delay10KTCYx(1);
		}
		Delay10KTCYx(10);
	}
}

/**
 * ProcessIO:
 **/
void
ProcessIO(void)
{
	if (BUTTON2)
		LED0 = 1;
	else
		LED0 = 0;
	if (BUTTON3)
		CHugFatalError(CH_FATAL_ERROR_UNKNOWN_CMD);
	LATD++;
}

/**
 * UserInit:
 **/
void
UserInit(void)
{
	/* turn off LEDs */
	LED0 = 0;
	LED1 = 0;

	/* set some defaults to power down the sensor */
	CHugSetColorSelect(CH_COLOR_SELECT_WHITE);
	CHugSetMultiplier(CH_FREQ_SCALE_0);
}

/**
 * InitializeSystem:
 **/
void
InitializeSystem(void)
{
// processor specific startup code
#if defined(__18F46J50)
	{
		/* Enable the PLL and wait 2+ms until the PLL locks
		 * before enabling USB module */
		unsigned int pll_startup_counter = 600;
		OSCTUNEbits.PLLEN = 1;
		while (pll_startup_counter--);

		/* default all pins to digital */
		ANCON0 = 0xFF;
		ANCON1 = 0xFF;
	}
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

	/* do all user init code */
	UserInit();
}

/**
 * main:
 **/
void
main(void)
{
	InitializeSystem();

	while(1) {
		ProcessIO();
	}
}
