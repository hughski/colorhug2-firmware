/* -*- Mode: C; tab-width: 8; indent-tabs-mode: t; c-basic-offset: 8 -*-
 *
 * Copyright (C) 2011-2013 Richard Hughes <richard@hughsie.com>
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
#include "ch-sram.h"
#include "ch-temp.h"

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
 * CHugSelfTestSram:
 **/
static uint8_t
CHugSelfTestSram(void)
{
	uint8_t rc;
	uint8_t sram_dma[4];
	uint8_t sram_tmp;

	/* check SRAM */
	CHugSramWriteByte(0x0000, 0xde);
	CHugSramWriteByte(0x0001, 0xad);
	CHugSramWriteByte(0x0002, 0xbe);
	CHugSramWriteByte(0x0003, 0xef);
	sram_tmp = CHugSramReadByte(0x0000);
	if (sram_tmp != 0xde) {
		rc = CH_ERROR_SRAM_FAILED;
		goto out;
	}
	sram_tmp = CHugSramReadByte(0x0001);
	if (sram_tmp != 0xad) {
		rc = CH_ERROR_SRAM_FAILED;
		goto out;
	}

	/* test DMA to and from SRAM */
	sram_dma[0] = 0xde;
	sram_dma[1] = 0xad;
	sram_dma[2] = 0xbe;
	sram_dma[3] = 0xef;
	CHugSramDmaFromCpu(sram_dma, 0x0010, 3);
	CHugSramDmaWait();
	sram_dma[0] = 0x00;
	sram_dma[1] = 0x00;
	sram_dma[2] = 0x00;
	sram_dma[3] = 0x00;
	CHugSramDmaToCpu(0x0010, sram_dma, 3);
	CHugSramDmaWait();

	if (sram_dma[0] != 0xde &&
	    sram_dma[1] != 0xad &&
	    sram_dma[2] != 0xbe &&
	    sram_dma[3] != 0xef) {
		rc = CH_ERROR_SRAM_FAILED;
		goto out;
	}

	/* success */
	rc = CH_ERROR_NONE;
out:
	return rc;
}

/**
 * CHugSelfTestSensor:
 **/
static uint8_t
CHugSelfTestSensor(uint8_t min_pulses)
{
	uint16_t i;
	uint8_t pulses = 0;
	uint8_t ra_tmp = PORTA;

	/* check sensor reports some values */
	for (i = 0; i < 0xffff && pulses < min_pulses; i++) {
		if (ra_tmp != PORTA)
			pulses++;
	}
	return pulses;
}

/**
 * CHugSelfTestI2C:
 **/
static uint8_t
CHugSelfTestI2C(void)
{
	uint8_t i = 0x00;
	uint8_t rc;

	/* test the i2c bus */
	SSP1CON2bits.SEN = 1;
	while (SSP1CON2bits.SEN) {
		if (++i == 0) {
			rc = CH_ERROR_SELF_TEST_I2C;
			goto out;
		}
	}
	PIR1bits.SSP1IF = 0;
	SSP1BUF = 0x00;
	while (!PIR1bits.SSP1IF) {
		if (++i == 0) {
			rc = CH_ERROR_SELF_TEST_I2C;
			goto out;
		}
	}
	SSP1CON2bits.PEN = 1;
	while (SSP1CON2bits.PEN) {
		if (++i == 0) {
			rc = CH_ERROR_SELF_TEST_I2C;
			goto out;
		}
	}
	rc = CH_ERROR_NONE;
	goto out;

	/* success */
	rc = CH_ERROR_NONE;
out:
	return rc;
}

/**
 * CHugSelfTest:
 *
 * Tests the device in the following ways:
 *  - Tests the RED sensor
 *  - Tests the GREEN sensor
 *  - Tests the BLUE sensor
 *  - Tests the SRAM functionality
 *  - Checks the I2C bus
 *  - Tests the ambient sensor
 **/
uint8_t
CHugSelfTest(void)
{
	const uint8_t min_pulses = 3;
	uint8_t pulses[3];
	uint8_t rc;
	CHugPackedFloat tmp;

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

	/* check SRAM */
	rc = CHugSelfTestSram();
	if (rc != CH_ERROR_NONE)
		goto out;

	/* check I2C */
	rc = CHugSelfTestI2C();
	if (rc != CH_ERROR_NONE)
		goto out;

	/* get the sensor temperature */
	rc = CHugTempGetAmbient(&tmp);
	if (rc != CH_ERROR_NONE)
		goto out;
	if (tmp.offset > 50 || tmp.offset < 10) {
		rc = CH_ERROR_SELF_TEST_TEMPERATURE;
		goto out;
	}

	/* success */
	rc = CH_ERROR_NONE;
out:
	return rc;
}
