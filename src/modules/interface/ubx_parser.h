/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * exptest.c - Testing of expansion port.
 */

#include <stdint.h>

void ubxpaserInit(void);

typedef enum
{
	INVALID,
	NEWDATA,
	PROCESSED
} Status_t;

typedef struct {

	uint8_t			NA;		//
	int8_t			NA2;		//
	int16_t			NA3;		//
	uint32_t		NA4;		//
	int32_t			relPosN;		// North component of relative position vector cm
	int32_t			relPosE;		// East component of relative position vector cm
	int32_t			relPosD;		// Down component of relative position vector cm
	int8_t			NA5;
	int8_t			NA6;
	int8_t			NA7;
	uint8_t			NA8;
	uint32_t		NA9;
	uint32_t		NA10;
	uint32_t		NA11;
	uint32_t		Flags;
	Status_t		Status;
} UBX_RELPOSNED_t;
