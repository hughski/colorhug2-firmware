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

#include "ColorHug.h"

#include "ch-math.h"

/**
 * CHugPackedFloatAdd:
 *
 * @pf1: A %CHugPackedFloat
 * @pf1: A %CHugPackedFloat
 * @result: A %CHugPackedFloat
 *
 * Adds two packed floats together using only integer maths.
 *
 * @return: an error code
 **/
ChError
CHugPackedFloatAdd (const CHugPackedFloat *pf1,
		    const CHugPackedFloat *pf2,
		    CHugPackedFloat *result)
{
	INT32 pf1_tmp;
	INT32 pf2_tmp;

	/* check overflow */
	pf1_tmp = pf1->raw / 0xffff;
	pf2_tmp = pf2->raw / 0xffff;
	if (pf1_tmp + pf2_tmp > 0xffff)
		return CH_ERROR_OVERFLOW_ADDITION;

	/* do the proper result */
	result->raw = pf1->raw + pf2->raw;
	return CH_ERROR_NONE;
}

/* not in stdlib.h */
#define ABS(a)	((a) > 0 ? (a) : (-a))

/**
 * CHugPackedFloatMultiply:
 *
 * @pf1: A %CHugPackedFloat
 * @pf1: A %CHugPackedFloat
 * @result: A %CHugPackedFloat
 *
 * Multiplies two packed floats together using only integer maths.
 *
 * @return: an error code
 **/
ChError
CHugPackedFloatMultiply (const CHugPackedFloat *pf1,
			 const CHugPackedFloat *pf2,
			 CHugPackedFloat *result)
{
	INT32 mult_result;
	INT32 mult_divisor;
	INT16 i;

	/* trivial: two numbers < 1.0 can be safely handled
	 * within 32 bits */
	if (pf1->raw < 0x10000 && pf2->raw < 0x10000)
		result->raw = (pf1->raw * pf2->raw) / 0x10000;

	/* find a divisor that can multiply these numbers with the
	 * greatest precision and with the temporary result still
	 * staying within 32 bits */
	for (i = 2; i < 0xff; i *= 2) {

		/* just do the multiplication */
		mult_result = (pf1->raw / i) * (pf2->raw / i);

		/* detect overflow */
		if (ABS((mult_result / pf1->raw) - (pf2->raw / (i * i))) > 1)
			continue;

		/* calculate post-multiply divisor */
		mult_divisor = 0x10000 / (i * i);
		result->raw = mult_result / mult_divisor;
		return CH_ERROR_NONE;
	}

	return CH_ERROR_OVERFLOW_MULTIPLY;
}
