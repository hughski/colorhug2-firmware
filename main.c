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

/* turn off extended mode */
#pragma config XINST = OFF

#pragma code

/* suitable for TDSDB146J50 or TDSDB14550 demo board */
#define BUTTON2			LATBbits.LATB2
#define BUTTON3			LATBbits.LATB3
#define LED0			LATEbits.LATE0
#define LED1			LATEbits.LATE1

/**
 * set_color_select:
 **/
void
set_color_select (ChColorSelect color_select)
{
	LATAbits.LATA0 = color_select & 0x01;
	LATAbits.LATA1 = color_select & 0x02;
}

/**
 * set_multiplier:
 **/
void
set_multiplier (ChFreqScale multiplier)
{
	LATAbits.LATA2 = multiplier & 0x01;
	LATAbits.LATA3 = multiplier & 0x02;
}

/**
 * main:
 **/
void
main (void)
{
// processor specific startup code
#if defined(__18F46J50)
	{
		/* Enable the PLL and wait 2+ms until the PLL locks
		 * before enabling USB module */
		unsigned int pll_startup_counter = 600;
		OSCTUNEbits.PLLEN = 1;
		while (pll_startup_counter--);
			ANCON0 = 0xFF;
		/* default all pins to digital */
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

	/* set some defaults to power down the sensor */
	set_color_select (CH_COLOR_SELECT_WHITE);
	set_multiplier (CH_FREQ_SCALE_0);

	while(1) {
		if (BUTTON2)
			LED0 = 1;
		else
			LED0 = 0;
		LATD++;
	}
}
