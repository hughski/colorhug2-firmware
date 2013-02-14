/* -*- Mode: C; tab-width: 8; indent-tabs-mode: t; c-basic-offset: 8 -*-
 *
 * Copyright (C) 2011-2012 Richard Hughes <richard@hughsie.com>
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

#include "ch-common.h"

/**
 * CHugGetColorSelect:
 **/
ChColorSelect
CHugGetColorSelect(void)
{
	return (PORTAbits.RA3 << 1) + PORTAbits.RA2;
}

/**
 * CHugSetColorSelect:
 **/
void
CHugSetColorSelect(ChColorSelect color_select)
{
	PORTAbits.RA2 = (color_select & 0x01);
	PORTAbits.RA3 = (color_select & 0x02) >> 1;
}

/**
 * CHugGetMultiplier:
 **/
ChFreqScale
CHugGetMultiplier(void)
{
	return (PORTAbits.RA1 << 1) + PORTAbits.RA0;
}

/**
 * CHugSetMultiplier:
 **/
void
CHugSetMultiplier(ChFreqScale multiplier)
{
	PORTAbits.RA0 = (multiplier & 0x01);
	PORTAbits.RA1 = (multiplier & 0x02) >> 1;
}

/**
 * CHugFatalError:
 **/
void
CHugFatalError (ChError error)
{
	uint8_t i;

	/* turn off watchdog */
	WDTCONbits.SWDTEN = 0;
	TRISE = 0x3c;

	while (1) {
		for (i = 0; i < error; i++) {
			PORTE = CH_STATUS_LED_RED;
			Delay10KTCYx(0xff);
			PORTE = 0x00;
			Delay10KTCYx(0xff);
		}
		Delay10KTCYx(0xff);
		Delay10KTCYx(0xff);
	}
}

/**
 * CHugSelfTestSensor:
 **/
static uint8_t
CHugSelfTestSensor(uint8_t min_pulses)
{
	uint16_t i;
	uint8_t pulses = 0;
	unsigned char ra_tmp = PORTA;

	/* check sensor reports some values */
	for (i = 0; i < 0xffff && pulses < min_pulses; i++) {
		if (ra_tmp != PORTA)
			pulses++;
	}
	return pulses;
}

/**
 * CHugSelfTest:
 *
 * Tests the device in the following ways:
 *  - Tests the RED sensor
 *  - Tests the GREEN sensor
 *  - Tests the BLUE sensor
 **/
uint8_t
CHugSelfTest(void)
{
	const uint8_t min_pulses = 3;
	uint8_t pulses[3];
	uint8_t rc;

	/* check multiplier can be set and read */
	CHugSetMultiplier(CH_FREQ_SCALE_0);
	if (CHugGetMultiplier() != CH_FREQ_SCALE_0) {
		rc = CH_ERROR_SELF_TEST_MULTIPLIER;
		goto out;
	}
	CHugSetMultiplier(CH_FREQ_SCALE_100);
	if (CHugGetMultiplier() != CH_FREQ_SCALE_100) {
		rc = CH_ERROR_SELF_TEST_MULTIPLIER;
		goto out;
	}

	/* check color select can be set and read */
	CHugSetColorSelect(CH_COLOR_SELECT_RED);
	if (CHugGetColorSelect() != CH_COLOR_SELECT_RED) {
		rc = CH_ERROR_SELF_TEST_COLOR_SELECT;
		goto out;
	}
	CHugSetColorSelect(CH_COLOR_SELECT_GREEN);
	if (CHugGetColorSelect() != CH_COLOR_SELECT_GREEN) {
		rc = CH_ERROR_SELF_TEST_COLOR_SELECT;
		goto out;
	}

	/* check red, green and blue */
	CHugSetColorSelect(CH_COLOR_SELECT_RED);
	pulses[CH_COLOR_OFFSET_RED] = CHugSelfTestSensor(min_pulses);
	CHugSetColorSelect(CH_COLOR_SELECT_GREEN);
	pulses[CH_COLOR_OFFSET_GREEN] = CHugSelfTestSensor(min_pulses);
	CHugSetColorSelect(CH_COLOR_SELECT_BLUE);
	pulses[CH_COLOR_OFFSET_BLUE] = CHugSelfTestSensor(min_pulses);

	/* are all the results invalid? */
	if (pulses[CH_COLOR_OFFSET_RED] != min_pulses &&
	    pulses[CH_COLOR_OFFSET_GREEN] != min_pulses &&
	    pulses[CH_COLOR_OFFSET_BLUE] != min_pulses) {
		rc = CH_ERROR_SELF_TEST_SENSOR;
		goto out;
	}

	/* one sensor color invalid */
	if (pulses[CH_COLOR_OFFSET_RED] != min_pulses) {
		rc = CH_ERROR_SELF_TEST_RED;
		goto out;
	}
	if (pulses[CH_COLOR_OFFSET_GREEN] != min_pulses) {
		rc = CH_ERROR_SELF_TEST_GREEN;
		goto out;
	}
	if (pulses[CH_COLOR_OFFSET_BLUE] != min_pulses) {
		rc = CH_ERROR_SELF_TEST_BLUE;
		goto out;
	}

	/* success */
	rc = CH_ERROR_NONE;
out:
	return rc;
}
