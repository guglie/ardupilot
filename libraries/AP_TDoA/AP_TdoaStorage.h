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

#include <AP_HAL/AP_HAL.h>

#include "AP_DwmDriver_stabTypes.h"
#include "AP_ClockCorrectionEngine.h"

using AP_ClockCorrectionEngine::clockCorrectionStorage_t;

#define ANCHOR_STORAGE_COUNT 16
#define REMOTE_ANCHOR_DATA_COUNT 16
#define TOF_PER_ANCHOR_COUNT 16


typedef struct {
  uint8_t id; // Id of remote remote anchor
  uint8_t seqNr; // Sequence number of the packet received in the remote anchor (7 bits)
  int64_t rxTime; // Receive time of packet from anchor id in the remote anchor, in remote DWM clock
  uint32_t endOfLife;
} tdoaRemoteAnchorData_t;

typedef struct {
  uint8_t id;
  int64_t tof;
  uint32_t endOfLife; // Time stamp when the tof data is outdated, local system time in ms
} tdoaTimeOfFlight_t;

typedef struct {
  bool isInitialized;
  uint32_t lastUpdateTime; // The time when this anchor was updated the last time
  uint8_t id; // Anchor id

  int64_t txTime; // Transmit time of last packet, in remote DWM clock
  int64_t rxTime; // Receive time of last packet, in local DWM clock
  uint8_t seqNr; // Sequence nr of last packet (7 bits)

  clockCorrectionStorage_t clockCorrectionStorage;

  point_t position; // The coordinates of the anchor

  tdoaTimeOfFlight_t tof[TOF_PER_ANCHOR_COUNT];
  tdoaRemoteAnchorData_t remoteAnchorData[REMOTE_ANCHOR_DATA_COUNT];
} tdoaAnchorInfo_t;

typedef tdoaAnchorInfo_t tdaoAnchorInfoArray_t[ANCHOR_STORAGE_COUNT];


// The anchor context is used to pass information about an anchor as well as
// the current time to functions.
// The context should not be stored.
typedef struct {
  tdoaAnchorInfo_t* anchorInfo;
  uint32_t currentTime_ms;
} tdoaAnchorContext_t;




class AP_TdoaStorage {
private:
    static AP_TdoaStorage* _singleton;

    static tdoaAnchorInfo_t* initializeSlot(const uint8_t slot, const uint8_t anchor);

public:
    AP_TdoaStorage() {
        _singleton = this;
    }

    static tdaoAnchorInfoArray_t anchorInfoArray; //aka tdoaAnchorInfo_t anchorStorage[]


    static AP_TdoaStorage* get_singleton() { return _singleton; }

    /*AP_TdoaStorage& storage() {
      return *AP_TdoaStorage::get_singleton();
    }*/


    bool getCreateAnchorCtx(const uint8_t anchor, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx);
    bool getAnchorCtx(const uint8_t anchor, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx);
    uint8_t getListOfAnchorIds(uint8_t unorderedAnchorList[], const int maxListSize);
    uint8_t getListOfActiveAnchorIds(uint8_t unorderedAnchorList[], const int maxListSize, const uint32_t currentTime_ms);

    uint8_t getId(const tdoaAnchorContext_t* anchorCtx);
    int64_t getRxTime(const tdoaAnchorContext_t* anchorCtx);
    int64_t getTxTime(const tdoaAnchorContext_t* anchorCtx);
    uint8_t getSeqNr(const tdoaAnchorContext_t* anchorCtx);
    uint32_t getLastUpdateTime(const tdoaAnchorContext_t* anchorCtx);
    clockCorrectionStorage_t* getClockCorrectionStorage(const tdoaAnchorContext_t* anchorCtx);
    bool getAnchorPosition(const tdoaAnchorContext_t* anchorCtx, point_t* position);
    void setAnchorPosition(tdoaAnchorContext_t* anchorCtx, const float x, const float y, const float z);
    void setRxTxData(tdoaAnchorContext_t* anchorCtx, int64_t rxTime, int64_t txTime, uint8_t seqNr);
    double getClockCorrection(const tdoaAnchorContext_t* anchorCtx);
    int64_t getRemoteRxTime(const tdoaAnchorContext_t* anchorCtx, const uint8_t remoteAnchor);
    void setRemoteRxTime(tdoaAnchorContext_t* anchorCtx, const uint8_t remoteAnchor, const int64_t remoteRxTime, const uint8_t remoteSeqNr);
    void getRemoteSeqNrList(const tdoaAnchorContext_t* anchorCtx, int* remoteCount, uint8_t seqNr[], uint8_t id[]);
    int64_t getTimeOfFlight(const tdoaAnchorContext_t* anchorCtx, const uint8_t otherAnchor);
    void setTimeOfFlight(tdoaAnchorContext_t* anchorCtx, const uint8_t remoteAnchor, const int64_t tof);



    // Mainly for test
    bool isAnchorInStorage(const uint8_t anchor);

    
};