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

#ifndef __USB_CONFIG_H
#define __USB_CONFIG_H

/* we only do small amounts of data */
#define USB_EP0_BUFF_SIZE		8

/* no alternative setting */
#define USB_MAX_NUM_INT			1
#define USB_MAX_EP_NUMBER		1

//Device descriptor - if these two definitions are not defined then
//  a ROM USB_DEVICE_DESCRIPTOR variable by the exact name of device_dsc
//  must exist.
#define USB_USER_DEVICE_DESCRIPTOR &device_dsc
#define USB_USER_DEVICE_DESCRIPTOR_INCLUDE extern ROM USB_DEVICE_DESCRIPTOR device_dsc

//Configuration descriptors - if these two definitions do not exist then
//  a ROM BYTE *ROM variable named exactly USB_CD_Ptr[] must exist.
#define USB_USER_CONFIG_DESCRIPTOR USB_CD_Ptr
#define USB_USER_CONFIG_DESCRIPTOR_INCLUDE extern ROM BYTE *ROM USB_CD_Ptr[]

#define USB_PING_PONG_MODE USB_PING_PONG__FULL_PING_PONG

/* we want to poll */
#define USB_POLLING

/* Parameter definitions are defined in usb_device.h */
#define USB_PULLUP_OPTION		USB_PULLUP_ENABLE
#define USB_TRANSCEIVER_OPTION		USB_INTERNAL_TRANSCEIVER
#define USB_SPEED_OPTION		USB_FULL_SPEED
#define USB_NUM_STRING_DESCRIPTORS 	4
#define USB_ENABLE_ALL_HANDLERS

/* device class */
#define USB_SUPPORT_DEVICE
#define USB_USE_HID

/* HID */
#define HID_INTF_ID			0x00
#define HID_EP				0x01
#define HID_INT_OUT_EP_SIZE		3
#define HID_INT_IN_EP_SIZE		3
#define HID_NUM_OF_DSC			1
#define HID_RPT01_SIZE			29

#endif
