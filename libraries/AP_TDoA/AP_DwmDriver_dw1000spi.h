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
#include "AP_DwmDriver_dw1000types.h"

/**
 * Read from the dw1000 SPI interface
 */
void dwSpiRead(dwDevice_t *dev, uint8_t regid, uint32_t address, void* data, size_t length);
uint16_t dwSpiRead16(dwDevice_t *dev, uint8_t regid, uint32_t address);
uint32_t dwSpiRead32(dwDevice_t *dev, uint8_t regid, uint32_t address);

/**
 * Write to the dw1000 SPI interface
 */
void dwSpiWrite(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                 const void* data, size_t length);

void dwSpiWrite8(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                   uint8_t data);

void dwSpiWrite32(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                  uint32_t data);
