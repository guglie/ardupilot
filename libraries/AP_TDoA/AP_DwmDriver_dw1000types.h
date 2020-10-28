/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * DW1000 Driver by Guglielmo Cassinelli
 * 
 * ported to ArduPilot from Libdw1000 by Bitcraze AB
 * Converted to C from the Decawave DW1000 library for arduino by Thomas Trojer
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "AP_DwmDriver_dw1000.h"

struct dwOps_s;
struct dwDevice_s;

typedef union dwTime_u {
	uint8_t raw[5];
	uint64_t full;
	struct {
		uint32_t low32;
		uint8_t high8;
	} __attribute__((packed));
	struct {
		uint8_t low8;
		uint32_t high32;
	} __attribute__((packed));
} dwTime_t;

typedef void (*dwHandler_t)(struct dwDevice_s *dev);

/**
 * DW device type. Contains the context of a dw1000 device and should be passed
 * as first argument of most of the driver functions.
 */
typedef struct dwDevice_s {
	struct dwOps_s *ops;
	void *userdata;

	/* State */
	uint8_t sysctrl[LEN_SYS_CTRL];
	uint8_t deviceMode;
	uint8_t networkAndAddress[LEN_PANADR];
	uint8_t syscfg[LEN_SYS_CFG];
	uint8_t sysmask[LEN_SYS_MASK];
	uint8_t chanctrl[LEN_CHAN_CTRL];
	uint8_t sysstatus[LEN_SYS_STATUS];
	uint8_t txfctrl[LEN_TX_FCTRL];

	uint8_t extendedFrameLength;
	uint8_t pacSize;
	uint8_t pulseFrequency;
	uint8_t dataRate;
	uint8_t preambleLength;
	uint8_t preambleCode;
	uint8_t channel;
	bool smartPower;
	bool frameCheck;
	bool permanentReceive;
	bool wait4resp;

	dwTime_t antennaDelay;

	// Callback handles
	dwHandler_t handleSent;
	dwHandler_t handleError;
	dwHandler_t handleReceived;
	dwHandler_t handleReceiveTimeout;
	dwHandler_t handleReceiveFailed;
	dwHandler_t handleReceiveTimestampAvailable;

	// settings
	uint32_t txPower;
	bool forceTxPower;
} dwDevice_t;

typedef enum {dwSpiSpeedLow, dwSpiSpeedHigh} dwSpiSpeed_t;

typedef enum {dwClockAuto = 0x00, dwClockXti = 0x01, dwClockPll = 0x02} dwClock_t;



