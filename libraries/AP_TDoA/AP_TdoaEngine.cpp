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

#include "AP_TdoaEngine.h"

AP_TdoaEngine* AP_TdoaEngine::_singleton;

//tdoaMeasurement_t AP_TdoaEngine::tdoaQueue [TDOA_QUEUE_SIZE]; 


ObjectBuffer<tdoaMeasurement_t> AP_TdoaEngine::tdoaQueue(TDOA_QUEUE_SIZE);


matchingAlgoData AP_TdoaEngine::matching;
AP_TdoaStorage* AP_TdoaEngine::storage;

tdaoAnchorInfoArray_t AP_TdoaEngine::anchorInfoArray;
tdoaEngineSendTdoaToEstimator AP_TdoaEngine::sendTdoaToEstimator;
tdoaEngineMatchingAlgorithm_t AP_TdoaEngine::matchingAlgorithm;

uint32_t AP_TdoaEngine::tdoa_enqueued;
uint32_t AP_TdoaEngine::packets_processed;
uint32_t AP_TdoaEngine::packets_timeOk;
uint32_t AP_TdoaEngine::packets_anchorOk;
uint32_t AP_TdoaEngine::matched_seqs;
uint32_t AP_TdoaEngine::tof_found_num;
uint8_t AP_TdoaEngine::last_seen_anchor;
uint32_t AP_TdoaEngine::contextHitCount;
uint32_t AP_TdoaEngine::contextMissCount;
uint32_t AP_TdoaEngine::suitableDataFound;
uint32_t AP_TdoaEngine::clockCorrectionOkCount;

#define TRUNCATE_TO_ANCHOR_TS_BITMAP 0x00FFFFFFFF
static uint64_t truncateToAnchorTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & TRUNCATE_TO_ANCHOR_TS_BITMAP;
}

void AP_TdoaEngine::enqueueTDOA(const tdoaAnchorContext_t* anchorACtx, const tdoaAnchorContext_t* anchorBCtx, double distanceDiff, uint64_t tdoa_systime_us) {
  //tdoaStats_t* stats = &stats;

  tdoaMeasurement_t tdoa = {
    anchorPosition: {0},
    distanceDiff : (float)distanceDiff, //TODO float
    stdDev : MEASUREMENT_NOISE_STD,
  };

  if (storage->getAnchorPosition(anchorACtx, &tdoa.anchorPosition[0]) && storage->getAnchorPosition(anchorBCtx, &tdoa.anchorPosition[1])) {
    //STATS_CNT_RATE_EVENT(&stats->packetsToEstimator);

    uint8_t idA = storage->getId(anchorACtx);
    uint8_t idB = storage->getId(anchorBCtx);

    tdoa.idA = idA;
    tdoa.idB = idB;
    tdoa.systime_us = tdoa_systime_us;
    
    /*if (idA == stats->anchorId && idB == stats->remoteAnchorId) {
      stats->tdoa = distanceDiff;
    }
    if (idB == stats->anchorId && idA == stats->remoteAnchorId) {
      stats->tdoa = -distanceDiff;
    }*/

    tdoa_enqueued++; //TODO remove debug
    
    tdoaQueue.push(tdoa);
    //TODO error is HERE vvvvvvvvvvvvvvvv

    //sendTdoaToEstimator(&tdoa, idA, idB, tdoa_systime_us);    //TODO error is HERE <<<<<<<<<<<<<<<<

    //TODO error is HERE ^^^^^^^^^^^^^^^^
  }
}

bool AP_TdoaEngine::updateClockCorrection(tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T/*, tdoaStats_t* stats*/) {
  bool sampleIsReliable = false;

  const int64_t latest_rxAn_by_T_in_cl_T = storage->getRxTime(anchorCtx);
  const int64_t latest_txAn_in_cl_An = storage->getTxTime(anchorCtx);

  if (latest_rxAn_by_T_in_cl_T != 0 && latest_txAn_in_cl_An != 0) {
    double clockCorrectionCandidate = AP_ClockCorrectionEngine::calculate(rxAn_by_T_in_cl_T, latest_rxAn_by_T_in_cl_T, txAn_in_cl_An, latest_txAn_in_cl_An, TRUNCATE_TO_ANCHOR_TS_BITMAP);
    sampleIsReliable = AP_ClockCorrectionEngine::update(storage->getClockCorrectionStorage(anchorCtx), clockCorrectionCandidate);

    /*if (sampleIsReliable){
      if (storage->getId(anchorCtx) == stats->anchorId) {
        stats->clockCorrection = storage->getClockCorrection(anchorCtx);
        STATS_CNT_RATE_EVENT(&stats->clockCorrectionCount);
      }
    }*/
  }

  return sampleIsReliable;
}

int64_t AP_TdoaEngine::calcTDoA(const tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  const uint8_t otherAnchorId = storage->getId(otherAnchorCtx);

  const int64_t tof_Ar_to_An_in_cl_An = storage->getTimeOfFlight(anchorCtx, otherAnchorId);
  const int64_t rxAr_by_An_in_cl_An = storage->getRemoteRxTime(anchorCtx, otherAnchorId);
  const double clockCorrection = storage->getClockCorrection(anchorCtx);

  const int64_t rxAr_by_T_in_cl_T = storage->getRxTime(otherAnchorCtx);

  const int64_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + truncateToAnchorTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An));
  const int64_t timeDiffOfArrival_in_cl_T =  truncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) - delta_txAr_to_txAn_in_cl_An  * clockCorrection;

  return timeDiffOfArrival_in_cl_T;
}

double AP_TdoaEngine::calcDistanceDiff(const tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  const int64_t tdoa = calcTDoA(otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
  return SPEED_OF_LIGHT_IN_AIR * tdoa / DWM1000_TS_FREQ; //tdoa * SPEED_OF_LIGHT_ON_COUNTER_FREQ; 
  // TODO is "SPEED_OF_LIGHT * tdoa / DWM1000_TS_FREQ" more numerically stable? 
}

bool AP_TdoaEngine::matchRandomAnchor(tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx) {
  matching.offset++;
  int remoteCount = 0;
  storage->getRemoteSeqNrList(anchorCtx, &remoteCount, matching.seqNr, matching.id); //TODO this is always 0, why?

  if(remoteCount > matched_seqs) matched_seqs = remoteCount; // TODO remove debug

  uint32_t now_ms = anchorCtx->currentTime_ms;

  // Loop over the candidates and pick the first one that is useful
  // An offset (updated for each call) is added to make sure we start at
  // different positions in the list and vary which candidate to choose
  for (int i = matching.offset; i < (remoteCount + matching.offset); i++) {
    uint8_t index = i % remoteCount;
    const uint8_t candidateAnchorId = matching.id[index];
    if (storage->getCreateAnchorCtx(candidateAnchorId, now_ms, otherAnchorCtx)) { //why getcreate?
      //if(matching.seqNr[index] == storage->getSeqNr(otherAnchorCtx)) matched_seqs++;   //TODO remove debug
      //if(storage->getTimeOfFlight(anchorCtx, candidateAnchorId)) tof_found_num++;   //TODO remove debug
      

      if (matching.seqNr[index] == storage->getSeqNr(otherAnchorCtx) && storage->getTimeOfFlight(anchorCtx, candidateAnchorId)) {
        return true;
      }
    }
  }

  otherAnchorCtx->anchorInfo = 0;
  return false;
}

bool AP_TdoaEngine::matchYoungestAnchor(tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx) {
    int remoteCount = 0;
    storage->getRemoteSeqNrList(anchorCtx, &remoteCount, matching.seqNr, matching.id);

    uint32_t now_ms = anchorCtx->currentTime_ms;
    uint32_t youmgestUpdateTime = 0;
    int bestId = -1;

    for (int index = 0; index < remoteCount; index++) {
      const uint8_t candidateAnchorId = matching.id[index];
      if (storage->getTimeOfFlight(anchorCtx, candidateAnchorId)) {
        if (storage->getCreateAnchorCtx(candidateAnchorId, now_ms, otherAnchorCtx)) {
          uint32_t updateTime = otherAnchorCtx->anchorInfo->lastUpdateTime;
          if (updateTime > youmgestUpdateTime) {
            if (matching.seqNr[index] == storage->getSeqNr(otherAnchorCtx)) {
              youmgestUpdateTime = updateTime;
              bestId = candidateAnchorId;
            }
          }
        }
      }
    }

    if (bestId >= 0) {
      storage->getCreateAnchorCtx(bestId, now_ms, otherAnchorCtx);
      return true;
    }

    otherAnchorCtx->anchorInfo = 0;
    return false;
}

bool AP_TdoaEngine::findSuitableAnchor(tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx) {
  bool result = false;

  if (storage->getClockCorrection(anchorCtx) > 0.0) {
    clockCorrectionOkCount++; // ok

    result = matchRandomAnchor(otherAnchorCtx, anchorCtx); //TODO use the switch when matchingAlgorithm enum works

    /*switch(matchingAlgorithm) {
      case TdoaEngineMatchingAlgorithmRandom:
        result = matchRandomAnchor(otherAnchorCtx, anchorCtx);
        break;

      case TdoaEngineMatchingAlgorithmYoungest:
        result = matchYoungestAnchor(otherAnchorCtx, anchorCtx);
        break;

      default:
        // Do nothing
        break;
    }
    */
  }

  return result;
}

void AP_TdoaEngine::getAnchorCtxForPacketProcessing(const uint8_t anchorId, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx) {
  if (storage->getCreateAnchorCtx(anchorId, currentTime_ms, anchorCtx)) {
    //STATS_CNT_RATE_EVENT(&stats.contextHitCount);
    contextHitCount++;
  } else {
    //STATS_CNT_RATE_EVENT(&stats.contextMissCount);
    contextMissCount++;
  }
}

void AP_TdoaEngine::processPacket(tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, uint64_t now_us) {
  packets_processed++; //TODO stats  //TODO remove debug
  
  bool timeIsGood = updateClockCorrection(anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T/*, &stats*/);
  if (timeIsGood) {
    //STATS_CNT_RATE_EVENT(&stats.timeIsGood);
    packets_timeOk++;  //TODO remove debug
    last_seen_anchor = anchorCtx->anchorInfo->id;  //TODO remove debug

    tdoaAnchorContext_t otherAnchorCtx;
    if (findSuitableAnchor(&otherAnchorCtx, anchorCtx)) {
      suitableDataFound++;
      //STATS_CNT_RATE_EVENT(&stats.suitableDataFound);
      double tdoaDistDiff = calcDistanceDiff(&otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
      enqueueTDOA(&otherAnchorCtx, anchorCtx, tdoaDistDiff, now_us);
    }
  }
}