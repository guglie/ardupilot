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
 * TDoA Tag Driver by Guglielmo Cassinelli
 * 
 * ported to ArduPilot from Crazyflie firmware by Bitcraze AB
 */
#pragma once

#include "AP_DwmDriver_loco.h"
#include <stdint.h>

// Packet format with compressed PAN and 64Bit addresses
// Maximum 128 bytes payload
typedef struct packet_s {
    union {
      uint16_t fcf;
      struct {
        uint16_t type:3;
        uint16_t security:1;
        uint16_t framePending:1;
        uint16_t ack:1;
        uint16_t ipan:1;
        uint16_t reserved:3;
        uint16_t destAddrMode:2;
        uint16_t version:2;
        uint16_t srcAddrMode:2;
      } fcf_s;
    };

    uint8_t seq;
    uint16_t pan;
    locoAddress_t destAddress;
    locoAddress_t sourceAddress;

    uint8_t payload[128];
} __attribute__((packed)) packet_t;

#define MAC80215_PACKET_INIT(packet, TYPE) packet.fcf_s.type = (TYPE); \
  packet.fcf_s.security = 0; \
  packet.fcf_s.framePending = 0; \
  packet.fcf_s.ack = 0; \
  packet.fcf_s.ipan = 1; \
  packet.fcf_s.destAddrMode = 3; \
  packet.fcf_s.version = 1; \
  packet.fcf_s.srcAddrMode = 3;

#define MAC802154_TYPE_BEACON 0
#define MAC802154_TYPE_DATA 1
#define MAC802154_TYPE_ACK 2
#define MAC802154_TYPE_CMD 3

#define MAC802154_HEADER_LENGTH 21