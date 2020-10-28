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

#include "AP_TdoaStorage.h"
//#include "tdoaStats.h"
#include "AP_DwmDriver_definitions.h"
//#include "AP_Tdoa3Tag.h"

#include <AP_HAL/utility/RingBuffer.h>


extern const AP_HAL::HAL& hal; //TODO only for debug print

// standar deviation of TDoA measurement
// in practice this varies a lot depending on position
#define MEASUREMENT_NOISE_STD 0.15f

#define TDOA_QUEUE_SIZE 64

typedef void (*tdoaEngineSendTdoaToEstimator)(tdoaMeasurement_t* tdoaMeasurement, const uint8_t idA, const uint8_t idB, uint64_t tdoa_systime_us);

typedef enum {
  TdoaEngineMatchingAlgorithmNone = 0,
  TdoaEngineMatchingAlgorithmRandom = 1,
  TdoaEngineMatchingAlgorithmYoungest = 2,
} tdoaEngineMatchingAlgorithm_t;

typedef struct {
    uint8_t seqNr[REMOTE_ANCHOR_DATA_COUNT];
    uint8_t id[REMOTE_ANCHOR_DATA_COUNT];
    uint8_t offset;
} matchingAlgoData;

class AP_TdoaEngine {
private:
    static AP_TdoaEngine* _singleton;
    static AP_TdoaStorage* storage;

    static void enqueueTDOA(const tdoaAnchorContext_t* anchorACtx, const tdoaAnchorContext_t* anchorBCtx, double distanceDiff, uint64_t tdoa_systime_us);
    static bool updateClockCorrection(tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T/*, tdoaStats_t* stats*/);
    static int64_t calcTDoA(const tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T);
    static double calcDistanceDiff(const tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T);
    static bool matchRandomAnchor(tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx);
    static bool matchYoungestAnchor(tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx);
    static bool findSuitableAnchor(tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx);

public:

    static uint32_t tdoa_enqueued;
    static uint32_t packets_processed;
    static uint32_t packets_timeOk;
    static uint32_t packets_anchorOk;
    static uint32_t matched_seqs;
    static uint32_t tof_found_num;
    static uint8_t last_seen_anchor;
    static uint32_t contextHitCount;
    static uint32_t contextMissCount;
    static uint32_t suitableDataFound;
    static uint32_t clockCorrectionOkCount;


    //static tdoaMeasurement_t tdoaQueue [TDOA_QUEUE_SIZE]; 
    //static uint16_t currentTdoaInQueue;

    static ObjectBuffer<tdoaMeasurement_t> tdoaQueue;

    // State
    static tdaoAnchorInfoArray_t anchorInfoArray;
    //tdoaStats_t stats; TODO re enable stats

    // Configuration
    static tdoaEngineSendTdoaToEstimator sendTdoaToEstimator;
    static tdoaEngineMatchingAlgorithm_t matchingAlgorithm;

    // Matching algorithm data
    static matchingAlgoData matching;
    // end state


    AP_TdoaEngine(const uint32_t now_ms, tdoaEngineSendTdoaToEstimator sendTdoaToEstimator, const tdoaEngineMatchingAlgorithm_t initialMatchingAlgorithm) {
        _singleton = this;
        storage = AP_TdoaStorage::get_singleton();

        //tdoaStorageInitialize(anchorInfoArray) //TODO this was a memset
        //tdoaStatsInit(&stats, now_ms);
        this->sendTdoaToEstimator = sendTdoaToEstimator;
        this->matchingAlgorithm = TdoaEngineMatchingAlgorithmRandom; //TODO debug initialMatchingAlgorithm;

        matching.offset = 0;

        //tdoa_enqueued = 0;
    }

    static AP_TdoaEngine* get_singleton() { return _singleton; }

    void getAnchorCtxForPacketProcessing(const uint8_t anchorId, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx);
    void processPacket(tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, uint64_t now_us);

};



