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

#ifndef USB_CONFIG_H
#define USB_CONFIG_H

/* we only do small amounts of data */
#define USB_EP0_BUFF_SIZE		8

/* no alternative setting */
#define USB_MAX_NUM_INT			1
#define USB_MAX_EP_NUMBER		1

//#define USB_PING_PONG_MODE USB_PING_PONG__NO_PING_PONG
#define USB_PING_PONG_MODE USB_PING_PONG__FULL_PING_PONG
//#define USB_PING_PONG_MODE USB_PING_PONG__EP0_OUT_ONLY
//#define USB_PING_PONG_MODE USB_PING_PONG__ALL_BUT_EP0

/* we don't want to poll */
//#define USB_POLLING
#define USB_INTERRUPT

/* Parameter definitions are defined in usb_device.h */
#define USB_PULLUP_OPTION		USB_PULLUP_ENABLE
//#define USB_PULLUP_OPTION		USB_PULLUP_DISABLED

#define USB_TRANSCEIVER_OPTION		USB_INTERNAL_TRANSCEIVER
//#define USB_TRANSCEIVER_OPTION	USB_EXTERNAL_TRANSCEIVER

#define USB_SPEED_OPTION		USB_FULL_SPEED
//#define USB_SPEED_OPTION		USB_LOW_SPEED

#define USB_SUPPORT_DEVICE

#define USB_NUM_STRING_DESCRIPTORS 	3

#define USB_ENABLE_ALL_HANDLERS

/* device class */
#define USB_USE_GEN

/* generic endpoint */
#define USBGEN_EP_SIZE			64
#define USBGEN_EP_NUM			1

#endif
