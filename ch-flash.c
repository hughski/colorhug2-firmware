/* -*- Mode: C; tab-width: 8; indent-tabs-mode: t; c-basic-offset: 8 -*-
 *
 * Copyright (C) 2015 Richard Hughes <richard@hughsie.com>
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

#include "ch-flash.h"

/**
 * CHugFlashLoadTableAtAddr:
 *
 * TBLPTR is a special 22 bit register, and we can't use memcpy
 **/
static void
CHugFlashLoadTableAtAddr(uint32_t addr)
{
	DWORD_VAL tmp;
	tmp.Val = addr;
	TBLPTRU = tmp.byte.UB;
	TBLPTRH = tmp.byte.HB;
	TBLPTRL = tmp.byte.LB;
}

/**
 * CHugFlashErase:
 **/
uint8_t
CHugFlashErase(uint32_t addr, uint16_t len)
{
	uint32_t i;
	uint8_t enable_int = FALSE;

	/* check this is aligned */
	if (addr % CH_FLASH_ERASE_BLOCK_SIZE > 0)
		return CH_ERROR_INVALID_ADDRESS;

	/* disable interrupts if set */
	if (INTCONbits.GIE) {
		INTCONbits.GIE = 0;
		enable_int = TRUE;
	}

	/* erase in chunks */
	for (i = addr; i < addr + len; i += CH_FLASH_ERASE_BLOCK_SIZE) {
		CHugFlashLoadTableAtAddr(i);
		EECON1bits.WREN = 1;
		EECON1bits.FREE = 1;
		EECON2 = 0x55;
		EECON2 = 0xAA;
		EECON1bits.WR = 1;
	}

	/* re-enable interrupts */
	if (enable_int)
		INTCONbits.GIE = 1;
	return CH_ERROR_NONE;
}


/**
 * CHugFlashWrite:
 **/
uint8_t
CHugFlashWrite(uint32_t addr, uint16_t len, const uint8_t *data)
{
	uint16_t cnt = 0;
	uint32_t i;
	uint8_t enable_int = FALSE;

	/* check this is aligned */
	if (addr % CH_FLASH_WRITE_BLOCK_SIZE > 0)
		return CH_ERROR_INVALID_ADDRESS;

	/* disable interrupts if set */
	if (INTCONbits.GIE) {
		INTCONbits.GIE = 0;
		enable_int = TRUE;
	}

	/* write in chunks */
	for (i = addr; len != 0; i += CH_FLASH_WRITE_BLOCK_SIZE) {
		CHugFlashLoadTableAtAddr(i);
		for (cnt = 0; cnt < CH_FLASH_WRITE_BLOCK_SIZE; cnt++) {
			TABLAT = *data++;
			_asm TBLWTPOSTINC _endasm
			if (--len == 0)
				break;
		}
		CHugFlashLoadTableAtAddr(i);
		EECON1bits.WREN = 1;
		EECON2 = 0x55;
		EECON2 = 0xAA;
		EECON1bits.WR = 1;
		EECON1bits.WREN = 0;
	}

	/* re-enable interrupts */
	if (enable_int)
		INTCONbits.GIE = 1;
	return CH_ERROR_NONE;
}

/**
 * CHugFlashRead:
 **/
void
CHugFlashRead(uint32_t addr, uint16_t len, uint8_t *data)
{
	CHugFlashLoadTableAtAddr(addr);
	while (len--) {
		_asm TBLRDPOSTINC _endasm
		*data++ = TABLAT;
	}
}
