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

/**
 * Code for TDSDB14550 or TDSDB146J50 using C18
 * This program demonstrates how to compile code for either the bootloader or using a PICKIT2/3 or similar programmer.
 **/

#include <p18cxxx.h>
#include <delays.h>

/* turn off extended mode */
#pragma config XINST = OFF

#pragma code

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
			ANCON0 = 0xFF;		/* Default all pins to digital */
		ANCON1 = 0xFF;			/* Default all pins to digital */
	}
#elif defined(__18F4550)
	ADCON1 = 0x0F;				/* Default all pins to digital */
#endif

	TRISA = 0xc0;		/* Set RA0 to RA5 to output */
	TRISB = 0x00;		/* Set RB0 to RB7 to output */
	TRISC = 0xF8;		/* Set RC0 to RC2 to output */
	TRISD = 0x00;		/* Set RD0 to RD7 to output */
	TRISE = 0xF8;		/* Set RE0 TO RE2 to output */

	while(1) {
#if defined(__18F46J50)
		/* on the 46j50 as RA4 and LAT4 do not exist LATA++ would never
		 * toggle RA5 as RA4=0 always */
		LATA+= 0x21;
#elif defined(__18F4550)
		LATA++;
#endif
		LATB++;
		LATC++;
		LATD++;
		LATE++;
	}
}
