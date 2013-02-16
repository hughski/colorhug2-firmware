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

#include "ColorHug.h"

#include "ch-common.h"

/* this is for the 23K640 */
typedef enum {
	CH_SRAM_MODE_BYTE = 0x01,
	CH_SRAM_MODE_PAGE = 0x81,
	CH_SRAM_MODE_SEQUENTIAL = 0x41
} ChSramMode;

/* this is for the 23K640 */
typedef enum {
	CH_SRAM_COMMAND_STATUS_WRITE = 0x01,
	CH_SRAM_COMMAND_DATA_WRITE = 0x02,
	CH_SRAM_COMMAND_DATA_READ = 0x03,
	CH_SRAM_COMMAND_STATUS_READ = 0x05
} ChSramCommand;

/**
 * CHugSramEnable:
 *
 * This deasserts the SS2/SSDMA/_CS line.
 * We handle this manually, as we want to issue commands before starting
 * the DMA transfer.
 **/
static void
CHugSramEnable (void)
{
	PORTDbits.RD3 = 0;
}

/**
 * CHugSramDisable:
 *
 * This asserts the SS2/SSDMA/_CS line.
 **/
static void
CHugSramDisable (void)
{
	PORTDbits.RD3 = 1;
}

/**
 * CHugSramSetMode:
 *
 * The SRAM has different modes to access and write the data.
 * byte: one byte can be transferred.
 * page: 32 bytes at a time within 1024 byte pages
 * sequential: 1024 bytes at a time over multiple pages
 **/
static void
CHugSramSetMode (ChSramMode mode)
{
	CHugSramEnable();
	SSP2BUF = CH_SRAM_COMMAND_STATUS_WRITE;
	while(!SSP2STATbits.BF);
	SSP2BUF = mode;
	while(!SSP2STATbits.BF);
	CHugSramDisable();
}

/**
 * CHugSramWriteByte:
 *
 * This switches the SRAM to byte mode and reads one byte.
 * This blocks for the duration of the transfer.
 **/
void
CHugSramWriteByte (uint16_t address, uint8_t data)
{
	/* change mode */
	CHugSramSetMode(CH_SRAM_MODE_BYTE);

	/* byte-write, high-addr, low-addr, data */
	CHugSramEnable();
	SSP2BUF = CH_SRAM_COMMAND_DATA_WRITE;
	while(!SSP2STATbits.BF);
	SSP2BUF = address >> 8;
	while(!SSP2STATbits.BF);
	SSP2BUF = address & 0xff;
	while(!SSP2STATbits.BF);
	SSP2BUF = data;
	while(!SSP2STATbits.BF);
	CHugSramDisable();
}

/**
 * CHugSramReadByte:
 * This blocks for the duration of the transfer.
 **/
uint8_t
CHugSramReadByte (uint16_t address)
{
	uint8_t data;

	/* change mode */
	CHugSramSetMode(CH_SRAM_MODE_BYTE);

	/* byte-read, high-addr, low-addr, data */
	CHugSramEnable();
	SSP2BUF = CH_SRAM_COMMAND_DATA_READ;
	while(!SSP2STATbits.BF);
	SSP2BUF = address >> 8;
	while(!SSP2STATbits.BF);
	SSP2BUF = address & 0xff;
	while(!SSP2STATbits.BF);
	SSP2BUF = 0xff;
	while(!SSP2STATbits.BF);
	data = SSP2BUF;
	CHugSramDisable();

	return data;
}

/**
 * CHugSramDmaWait:
 *
 * Hang the CPU waiting for the DMA transfer to finish,
 * then disable the SRAM.
 **/
void
CHugSramDmaWait (void)
{
	while (DMACON1bits.DMAEN)
		ClrWdt();
	CHugSramDisable();
}

/**
 * CHugSramDmaCheck:
 *
 * Check to see if the DMA transfer has finished, and if so then
 * disable the SRAM.
 *
 * Return value: TRUE if the DMA transfer has finished.
 **/
uint8_t
CHugSramDmaCheck (void)
{
	if (DMACON1bits.DMAEN)
		return FALSE;
	CHugSramDisable();
	return TRUE;
}

/**
 * CHugSramDmaFromCpuPrep:
 **/
void
CHugSramDmaFromCpuPrep (void)
{
	/* set the SPI DMA engine to send-only data mode*/
	DMACON1bits.DUPLEX1 = 0;
	DMACON1bits.DUPLEX0 = 1;
	DMACON1bits.RXINC = 0;
	DMACON1bits.TXINC = 1;

	/* change mode */
	CHugSramSetMode(CH_SRAM_MODE_SEQUENTIAL);
}

/**
 * CHugSramDmaFromCpuExec:
 **/
void
CHugSramDmaFromCpuExec (uint8_t *address_cpu, uint16_t address_ram, uint16_t length)
{
	/* byte-write, high-addr, low-addr, data */
	CHugSramEnable();
	SSP2BUF = CH_SRAM_COMMAND_DATA_WRITE;
	while(!SSP2STATbits.BF);
	SSP2BUF = address_ram >> 8;
	while(!SSP2STATbits.BF);
	SSP2BUF = address_ram & 0xff;
	while(!SSP2STATbits.BF);

	/* actual bytes transferred is DMABC + 1 */
	DMABCH = (length - 1) >> 8;
	DMABCL = (length - 1) & 0xff;

	/* set transmit address */
	TXADDRH = (uint16_t) address_cpu >> 8;
	TXADDRL = (uint16_t) address_cpu & 0xff;

	/* initiate a DMA transaction */
	DMACON1bits.DMAEN = 1;
}

/**
 * CHugSramDmaFromCpu:
 *
 * DMA up to 1024 bytes of data from the CPU->SRAM.
 * This method does not block and leaves the SRAM enabled.
 **/
void
CHugSramDmaFromCpu (uint8_t *address_cpu, uint16_t address_ram, uint16_t length)
{
	CHugSramDmaFromCpuPrep();
	CHugSramDmaFromCpuExec(address_cpu, address_ram, length);
}

/**
 * CHugSramDmaToCpuPrep:
 **/
void
CHugSramDmaToCpuPrep (void)
{
	/* set the SPI DMA engine to receive-only data mode */
	DMACON1bits.DUPLEX1 = 0;
	DMACON1bits.DUPLEX0 = 0;
	DMACON1bits.RXINC = 1;
	DMACON1bits.TXINC = 0;

	/* change mode */
	CHugSramSetMode(CH_SRAM_MODE_SEQUENTIAL);
}

/**
 * CHugSramDmaToCpuExec:
 **/
void
CHugSramDmaToCpuExec (uint16_t address_ram, uint8_t *address_cpu, uint16_t length)
{
	/* byte-read, high-addr, low-addr, data */
	CHugSramEnable();
	SSP2BUF = CH_SRAM_COMMAND_DATA_READ;
	while(!SSP2STATbits.BF);
	SSP2BUF = address_ram >> 8;
	while(!SSP2STATbits.BF);
	SSP2BUF = address_ram & 0xff;
	while(!SSP2STATbits.BF);

	/* actual bytes transferred is DMABC + 1 */
	DMABCH = (length - 1) >> 8;
	DMABCL = (length - 1) & 0xff;

	/* set recieve address */
	RXADDRH = (uint16_t) address_cpu >> 8;
	RXADDRL = (uint16_t) address_cpu & 0xff;

	/* initiate a DMA transaction */
	DMACON1bits.DMAEN = 1;
}

/**
 * CHugSramDmaToCpu:
 *
 * DMA up to 1024 bytes of data from the SRAM->CPU.
 * This method does not block and leaves the SRAM enabled.
 **/
void
CHugSramDmaToCpu (uint16_t address_ram, uint8_t *address_cpu, uint16_t length)
{
	CHugSramDmaToCpuPrep();
	CHugSramDmaToCpuExec(address_ram, address_cpu, length);
}

/**
 * CHugSramWipe:
 *
 * Wipe the SRAM so that it contains known data.
 **/
void
CHugSramWipe (uint16_t address, uint16_t length)
{
	uint8_t buffer[8];
	uint16_t i;

	/* use 0xff as 'clear' */
	for (i = 0; i < 8; i++)
		buffer[i] = 0xff;

	/* blit this to the entire area specified */
	CHugSramDmaFromCpuPrep();
	for (i = 0; i < length / 8; i++) {
		ClrWdt();
		CHugSramDmaFromCpuExec(buffer, address + i * 8, 8);
		CHugSramDmaWait();
	}
}
