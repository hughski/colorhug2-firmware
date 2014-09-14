/* -*- Mode: C; tab-width: 8; indent-tabs-mode: t; c-basic-offset: 8 -*-
 *
 * Copyright (C) 2014 Richard Hughes <richard@hughsie.com>
 *
 * Multi-Channel Programmable Analog Current Integrator with Digital Converter
 * Manufactured by MAZeT GmbH, designed for the JENCOLOR sensor
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

#ifndef __CH_MCDC04_H
#define __CH_MCDC04_H

#include "ColorHug.h"

#include "ch-math.h"

/* Integration Time */
typedef enum {
	CH_MCDC04_TINT_1,	/* ms, 10 bit precision */
	CH_MCDC04_TINT_2,	/* ms, 11 bit precision */
	CH_MCDC04_TINT_4,	/* ms, 12 bit precision */
	CH_MCDC04_TINT_8,	/* ms, 13 bit precision */
	CH_MCDC04_TINT_16,	/* ms, 14 bit precision */
	CH_MCDC04_TINT_32,	/* ms, 15 bit precision */
	CH_MCDC04_TINT_64,	/* ms, 16 bit precision */
	CH_MCDC04_TINT_128,	/* ms, 17 bit precision */
	CH_MCDC04_TINT_256,	/* ms, 18 bit precision */
	CH_MCDC04_TINT_512,	/* ms, 19 bit precision */
	CH_MCDC04_TINT_1024	/* ms, 20 bit precision */
} CHugMcdc04Tint;

typedef enum {
	CH_MCDC04_IREF_20,	/* nA */
	CH_MCDC04_IREF_80,	/* nA */
	CH_MCDC04_IREF_320,	/* nA */
	CH_MCDC04_IREF_1280,	/* nA */
	CH_MCDC04_IREF_5120	/* nA */
} CHugMcdc04Iref;

typedef enum {
	CH_MCDC04_DIV_2,
	CH_MCDC04_DIV_4,
	CH_MCDC04_DIV_8,
	CH_MCDC04_DIV_16,
	CH_MCDC04_DIV_DISABLE
} CHugMcdc04Div;

typedef struct {
	CHugMcdc04Tint		tint;
	CHugMcdc04Iref		iref;
	CHugMcdc04Div		div;
} CHugMcdc04Context;

void	 CHugMcdc04Init			(CHugMcdc04Context	*ctx);
void	 CHugMcdc04SetTINT		(CHugMcdc04Context	*ctx,
					 CHugMcdc04Tint		 tint);
void	 CHugMcdc04SetIREF		(CHugMcdc04Context	*ctx,
					 CHugMcdc04Iref		 iref);
void	 CHugMcdc04SetDIV		(CHugMcdc04Context	*ctx,
					 CHugMcdc04Div		 div);
uint8_t	 CHugMcdc04WriteConfig		(CHugMcdc04Context	*ctx);
uint8_t	 CHugMcdc04TakeReadings		(CHugMcdc04Context	*ctx,
					 CHugPackedFloat	*x,
					 CHugPackedFloat	*y,
					 CHugPackedFloat	*z);

#endif /* __CH_MCDC04_H */
