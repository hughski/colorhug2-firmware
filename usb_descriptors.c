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
 *
 * Additionally, some constants and code snippets have been taken from
 * freely available datasheets which are:
 *
 * Copyright (C) Microchip Technology, Inc.
 */

#include "ColorHug.h"

#include <USB/usb.h>
#include <USB/usb_function_hid.h>

#if defined(__18CXX)
#pragma romdata
#endif

/* device descriptor, see usb_ch9.h */
ROM USB_DEVICE_DESCRIPTOR device_dsc=
{
	0x12,				/* Size of this descriptor in bytes */
	USB_DESCRIPTOR_DEVICE,		/* DEVICE descriptor type */
	0x0200,				/* USB Spec Release Number (BCD format) */
	0x00,				/* Class Code */
	0x00,				/* Subclass */
	0x00,				/* Protocol */
	USB_EP0_BUFF_SIZE,		/* Max packet size for EP0, see usb_config.h */
	CH_USB_VID,			/* Vendor ID */
	CH_USB_PID,			/* Product ID */
	0x0002,				/* Device release number (BCD format) */
	0x01,				/* Manufacturer string index */
	0x02,				/* Product string index */
	0x00,				/* Device serial number string index */
	0x01				/* Number of possible configurations */
};

/* Configuration 1 Descriptor */
ROM BYTE configDescriptor1[]={
	/* Configuration Descriptor */
	0x09,// sizeof(USB_CFG_DSC),	/* Size of this descriptor in bytes */
	USB_DESCRIPTOR_CONFIGURATION,	/* CONFIGURATION descriptor type */
	0x29,0x00,			/* Total length of data */
	1,				/* Number of interfaces */
	1,				/* Index value of this configuration */
	0,				/* Configuration string index */
	_DEFAULT | _SELF,		/* Attributes (this device is self-powered, but has no remote wakeup), see usb_device.h */
	150,				/* Max power consumption (2X mA)

	/* Interface Descriptor */
	0x09,// sizeof(USB_INTF_DSC),	/* Size of this descriptor in bytes */
	USB_DESCRIPTOR_INTERFACE,	/* INTERFACE descriptor type */
	0,				/* Interface Number */
	0,				/* Alternate Setting Number */
	2,				/* Number of endpoints in this intf */
	HID_INTF,			/* Class code */
	0,				/* Subclass code */
	0,				/* Protocol code */
	0,				/* Interface string index */

	/* HID Class-Specific Descriptor */
	0x09,// sizeof(USB_HID_DSC)+3,	/* Size of this descriptor in bytes */
	DSC_HID,			/* HID descriptor type */
	0x11,0x01,			/* HID Spec Release Number (BCD format) */
	0x00,				/* Country Code (0x00 for Not supported) */
	HID_NUM_OF_DSC,			/* Number of class descriptors, see usbcfg.h */
	DSC_RPT,			/* Report descriptor type */
	HID_RPT01_SIZE,0x00,// sizeof(hid_rpt01), /* Size of the report descriptor (with extra byte) */

	/* Endpoint Descriptor */
	0x07,/*sizeof(USB_EP_DSC)*/
	USB_DESCRIPTOR_ENDPOINT,	/* Endpoint Descriptor */
	HID_EP | _EP_IN,		/* EndpointAddress */
	_INTERRUPT,			/* Attributes */
	0x40,0x00,			/* size (with extra byte) */
	0x01,				/* polling interval */

	/* Endpoint Descriptor */
	0x07,/*sizeof(USB_EP_DSC)*/
	USB_DESCRIPTOR_ENDPOINT,	/* Endpoint Descriptor */
	HID_EP | _EP_OUT,		/* EndpointAddress */
	_INTERRUPT,			/* Attributes */
	0x40,0x00,			/* size (with extra byte) */
	0x01				/* polling interval */
};

/* Language code string descriptor */
ROM struct{BYTE bLength;BYTE bDscType;WORD string[1];}sd000={
sizeof(sd000),USB_DESCRIPTOR_STRING,{0x0409
}};

/* Manufacturer string descriptor (unicode) */
ROM struct{BYTE bLength;BYTE bDscType;WORD string[12];}sd001={
sizeof(sd001),USB_DESCRIPTOR_STRING,
{'H','u','g','h','s','k','i',' ','L','t','d','.',
}};

#ifdef COLORHUG_BOOTLOADER

/* Product string descriptor (unicode) */
ROM struct{BYTE bLength;BYTE bDscType;WORD string[21];}sd002={
sizeof(sd002),USB_DESCRIPTOR_STRING,
{'C','o','l','o','r','H','u','g',' ','(','b','o','o','t','l','o','a','d','e','r',')'
}};

#else

/* Product string descriptor (unicode) */
ROM struct{BYTE bLength;BYTE bDscType;WORD string[8];}sd002={
sizeof(sd002),USB_DESCRIPTOR_STRING,
{'C','o','l','o','r','H','u','g'
}};

#endif

/* HID descriptor -- see http://www.usb.org/developers/hidpage#HID%20Descriptor%20Tool */
ROM struct{BYTE report[HID_RPT01_SIZE];}hid_rpt01={
{
	0x06, 0x00, 0xFF,		/* Usage Page = 0xFF00 (Vendor Defined Page 1) */
	0x09, 0x01,			/* Usage (Vendor Usage 1) */
	0xA1, 0x01,			/* Collection (Application) */
	0x19, 0x01,			/* Usage Minimum */
	0x29, 0x40,			/* Usage Maximum -- 64 input usages total (0x01 to 0x40) */
	0x15, 0x00,			/* Logical Minimum (Vendor Usage = 0) */
	0x26, 0xFF, 0x00,		/* Logical Maximum (Vendor Usage = 255) */
	0x75, 0x08,			/* Report Size: 8-bit field size */
	0x95, 0x40,			/* Report Count: Make sixty-four 8-bit fields (the next time the parser hits an "Input", "Output", or "Feature" item) */
	0x81, 0x02,			/* Input (Data, Array, Abs): Instantiates input packet fields based on the above report size, count, logical min/max, and usage. */
	0x19, 0x01,			/* Usage Minimum */
	0x29, 0x40,			/* Usage Maximum - 64 output usages total (0x01 to 0x40) */
	0x91, 0x02,			/* Output (Data, Array, Abs): Instantiates output packet fields.  Uses same report size and count as "Input" fields, since nothing new/different was specified to the parser since the "Input" item. */
	0xC0}				/* End Collection */
};

/* only one configuration descriptor */
ROM BYTE *ROM USB_CD_Ptr[]=
{
	(ROM BYTE *ROM)&configDescriptor1
};

/* string descriptors */
ROM BYTE *ROM USB_SD_Ptr[]=
{
	(ROM BYTE *ROM)&sd000,
	(ROM BYTE *ROM)&sd001,
	(ROM BYTE *ROM)&sd002
};
