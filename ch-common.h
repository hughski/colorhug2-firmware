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

#ifndef __CH_COMMON_H
#define __CH_COMMON_H

#include <GenericTypeDefs.h>

#include "ColorHug.h"

ChColorSelect	 CHugGetColorSelect	(void);
void		 CHugSetColorSelect	(ChColorSelect	 color_select);
ChFreqScale	 CHugGetMultiplier	(void);
void		 CHugSetMultiplier	(ChFreqScale	 multiplier);
void		 CHugFatalError		(ChError	 error);

#endif /* __CH_COMMON_H */
