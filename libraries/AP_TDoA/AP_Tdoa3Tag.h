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

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#include "AP_DwmDriverBackend.h"
#include "AP_DwmDriver_loco.h"
#include "AP_DwmDriver_packet.h"
#include "AP_TdoaEngine.h"
#include "AP_TdoaStorage.h"
#include "TDOA_EKF.h"

// Positions for sent LPP packets
#define LPS_TDOA3_TYPE 0
#define LPS_TDOA3_SEND_LPP_PAYLOAD 1

#define PACKET_TYPE_TDOA3 0x30

#define TDOA3_RECEIVE_TIMEOUT 10000

#define TDOA3TAG_DEBUG true

typedef struct {
    uint8_t type;
    uint8_t seq;
    uint32_t txTimeStamp;
    uint8_t remoteCount;
} __attribute__((packed)) rangePacketHeader3_t;

typedef struct {
    uint8_t id;
    uint8_t seq;
    uint32_t rxTimeStamp;
    uint16_t distance;
} __attribute__((packed)) remoteAnchorDataFull_t;

typedef struct {
    uint8_t id;
    uint8_t seq;
    uint32_t rxTimeStamp;
} __attribute__((packed)) remoteAnchorDataShort_t;

typedef struct {
    rangePacketHeader3_t header;
    uint8_t remoteAnchorData;
} __attribute__((packed)) rangePacket3_t;



class AP_Tdoa3Tag {
  private:
    static AP_Tdoa3Tag* _singleton;

    // Outgoing LPP packet
    static lpsLppShortPacket_t lppPacket;

    static bool rangingOk;
    
    uint32_t last_update_ms; // pos estimator loop timer
    static uint32_t last_update_tdoa_ms;
    static uint64_t last_update_tdoa_us;


    static AP_TdoaStorage* storage;
    static AP_TdoaEngine engine_instance;
    static AP_TdoaEngine* engine;

    TDOA_EKF* ekf;
    bool thread_started;

  public:

    static uint32_t tdoaPushedToEkf;

    AP_Tdoa3Tag();

    //~AP_Tdoa3Tag() { delete ekf; }

    static AP_Tdoa3Tag* get_singleton() { return _singleton; }

    // a loop running in the timer thread to calculate position
    bool start_thread();
    void pos_estimator_loop();
    void pos_estimator_thread();

    static bool isValidTimeStamp(const int64_t anchorRxTime);

    static int updateRemoteData(tdoaAnchorContext_t* anchorCtx, const void* payload);

    static void handleLppShortPacket(tdoaAnchorContext_t* anchorCtx, const uint8_t* data, const int length);

    static void handleLppPacket(const int dataLength, int rangePacketLength, const packet_t* rxPacket, tdoaAnchorContext_t* anchorCtx);

    static void rxcallback(unsigned int dataLength);

    static void setRadioInReceiveMode(dwDevice_t* dev);

    static void sendLppShort(dwDevice_t* dev, lpsLppShortPacket_t* packet);

    static bool sendLpp(dwDevice_t* dev);

    /* static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
    switch(event) {
      case eventPacketReceived:
        rxcallback(dev);
        break;
      case eventTimeout:
        break;
      case eventReceiveTimeout:
        break;
      case eventPacketSent:
        // Service packet sent, the radio is back to receive automatically
        break;
      default:
        ASSERT_FAILED();
    }

    if(!sendLpp(dev)) {
      setRadioInReceiveMode(dev);
    }

    tdoaStatsUpdate(&tdoaEngineState.stats, AP_HAL::millis());

    return MAX_TIMEOUT;
  }*/

    //static void pushEkfUpdate();

    static bool getAnchorPosition(const uint8_t anchorId, point_t* position);

    static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize);

    static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize);

};  // class AP_Tdoa3Tag
