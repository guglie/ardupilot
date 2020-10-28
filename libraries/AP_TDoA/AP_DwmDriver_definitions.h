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


#include "AP_DwmDriver_dw1000.h"
#include "AP_DwmDriver_packet.h"
#include "AP_DwmDriver_dw1000types.h"

#include <cstdint> 



typedef uint64_t locoAddress_t;


///////////////////////// TDoA /////////////////////////

// Protocol version
#define PACKET_TYPE_TDOA2 0x22

// Positions in payload for received LPP packets
#define LPS_TDOA2_LPP_HEADER (sizeof(rangePacket2_t))
#define LPS_TDOA2_LPP_TYPE (sizeof(rangePacket2_t) + 1)
#define LPS_TDOA2_LPP_PAYLOAD (sizeof(rangePacket2_t) + 2)

// Positions for sent LPP packets
#define LPS_TDOA2_TYPE_INDEX 0
#define LPS_TDOA2_SEND_LPP_PAYLOAD_INDEX 1

#define SHORT_LPP 0xF0

#define LPP_SHORT_ANCHOR_POSITION 0x01
#define LPP_SHORT_REBOOT 0x02
#define LPP_SHORT_MODE 0x03
#define LPP_SHORT_UWB 0x04
#define LPP_SHORT_UWB_MODE 0x05

struct lppShortAnchorPosition_s {
  float position[3];
} __attribute__((packed));

// data stored in the circular array
typedef struct{
    uint8_t anchor;
    packet_t rxPacket;
    uint64_t rxtime;
} recvData;




