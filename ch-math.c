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
	int32_t pf1_tmp;
	int32_t pf2_tmp;

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
	CHugPackedFloat pf1_tmp;
	CHugPackedFloat pf2_tmp;

	/* make positive */
	pf1_tmp.raw = ABS(pf1->raw);
	pf2_tmp.raw = ABS(pf2->raw);

	/* check for overflow */
	if (pf1_tmp.offset > 0 &&
	    0x8000 / pf1_tmp.offset < pf2_tmp.offset)
		return CH_ERROR_OVERFLOW_MULTIPLY;

	/* do long multiplication on each 16 bit part */
	result->raw = ((uint32_t) pf1_tmp.fraction *
		       (uint32_t) pf2_tmp.fraction) / 0x10000;
	result->raw += ((uint32_t) pf1_tmp.offset *
			(uint32_t) pf2_tmp.offset) * 0x10000;
	result->raw += (uint32_t) pf1_tmp.fraction *
		       (uint32_t) pf2_tmp.offset;
	result->raw += (uint32_t) pf1_tmp.offset *
		       (uint32_t) pf2_tmp.fraction;

	/* correct sign bit */
	if ((pf1->raw < 0) ^ (pf2->raw < 0))
		result->raw = -result->raw;
	return CH_ERROR_NONE;
}
