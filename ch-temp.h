/* -*- Mode: C; tab-width: 8; indent-tabs-mode: t; c-basic-offset: 8 -*-
 *
 * Copyright (C) 2013 Richard Hughes <richard@hughsie.com>
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

#ifndef __CH_TEMP_H
#define __CH_TEMP_H

#include "ColorHug.h"

#include "ch-math.h"

/* this is for the TCN75A */
typedef enum {
	CH_TEMP_RESOLUTION_1_2C,
	CH_TEMP_RESOLUTION_1_4C,
	CH_TEMP_RESOLUTION_1_8C,
	CH_TEMP_RESOLUTION_1_16C
} ChTempResolution;

uint8_t	 CHugTempSetResolution	(ChTempResolution	 resolution);
uint8_t	 CHugTempGetAmbient	(CHugPackedFloat	*result);

#endif /* __CH_TEMP_H */
