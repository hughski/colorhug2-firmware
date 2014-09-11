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

#include <i2c.h>

#include "ColorHug.h"

#include "ch-temp.h"

/**
 * CHugTempSetResolution:
 **/
uint8_t
CHugTempSetResolution (ChTempResolution resolution)
{
	uint8_t tmp;
	uint8_t rc;

	/* start */
	StartI2C1();

	/* send slave address (write) */
	rc = WriteI2C1(0b10010000);
	if (rc != 0x00) {
		rc = CH_ERROR_I2C_SLAVE_ADDRESS;
		goto out;
	}

	/* set config pointer */
	rc = WriteI2C1(0b00000001);
	if (rc != 0x00) {
		rc = CH_ERROR_I2C_SLAVE_CONFIG;
		goto out;
	}

	/* send config */
	tmp = 0x00;
	tmp |= resolution << 5;
	WriteI2C1(tmp);

	/* stop */
	StopI2C1();
out:
	return rc;
}

/**
 * CHugTempGetAmbient:
 **/
uint8_t
CHugTempGetAmbient (CHugPackedFloat *result)
{
	uint16_t tmp;
	uint8_t rc;
	uint8_t tmp_lsb = 4;
	uint8_t tmp_msb = 2;

	/* start */
	StartI2C1();

	/* send slave address (write) */
	rc = WriteI2C1(0b10010000);
	if (rc != 0x00) {
		rc = CH_ERROR_I2C_SLAVE_ADDRESS;
		goto out;
	}

	/* set ambient temperature pointer */
	rc = WriteI2C1(0b00000000);

	/* reset the bus */
	RestartI2C1();

	/* send slave address (read) */
	WriteI2C1(0b10010001);

	/* read the temperature */
	tmp_msb = ReadI2C1();
	AckI2C1();
	tmp_lsb = ReadI2C1();
	NotAckI2C1();

	/* stop */
	StopI2C1();

	/* format result */
	result->offset = tmp_msb;
	result->fraction = ((uint16_t) tmp_lsb) << 8;
out:
	return rc;
}
