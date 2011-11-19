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

#ifndef __CH_MATH_H
#define __CH_MATH_H

#include <GenericTypeDefs.h>

#include "ColorHug.h"

/* a 32 bit struct to hold numbers from the range -32767 to +32768
 * with a precision of at least 0.000015 */
typedef union {
	struct {
		UINT16	fraction;
		INT16	offset;
	};
	struct {
		INT32	raw;
	};
} CHugPackedFloat;

ChFatalError	 CHugPackedFloatAdd		(const CHugPackedFloat	*pf1,
						 const CHugPackedFloat	*pf2,
						 CHugPackedFloat	*result);
ChFatalError	 CHugPackedFloatMultiply	(const CHugPackedFloat	*pf1,
						 const CHugPackedFloat	*pf2,
						 CHugPackedFloat	*result);

#endif /* __CH_MATH_H */
