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

#include "AP_TdoaStorage.h"


AP_TdoaStorage* AP_TdoaStorage::_singleton;
tdaoAnchorInfoArray_t AP_TdoaStorage::anchorInfoArray;




#define TOF_VALIDITY_PERIOD (2 * 1000)
#define REMOTE_DATA_VALIDITY_PERIOD 30
#define ANCHOR_POSITION_VALIDITY_PERIOD (2 * 1000)
#define ANCHOR_ACTIVE_VALIDITY_PERIOD (2 * 1000)


/*void AP_TdoaStorage::initialize(tdoaAnchorInfo_t anchorStorage[]) {
  memset(anchorStorage, 0, sizeof(tdoaAnchorInfo_t) * ANCHOR_STORAGE_COUNT);
}*/

bool AP_TdoaStorage::getCreateAnchorCtx(const uint8_t anchor, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx) {
  anchorCtx->currentTime_ms = currentTime_ms;
  uint32_t oldestUpdateTime = currentTime_ms;
  int firstUninitializedSlot = -1;
  int oldestSlot = 0;

  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    if (anchorInfoArray[i].isInitialized) {
      if (anchor == anchorInfoArray[i].id) {
        anchorCtx->anchorInfo = &anchorInfoArray[i];
        return true;
      }

      if (anchorInfoArray[i].lastUpdateTime < oldestUpdateTime) {
        oldestUpdateTime = anchorInfoArray[i].lastUpdateTime;
        oldestSlot = i;
      }
    } else {
      if (firstUninitializedSlot == -1) {
        firstUninitializedSlot = i;
      }
    }
  }

  // The anchor was not found in storage
  tdoaAnchorInfo_t* newAnchorInfo = 0;
  if (firstUninitializedSlot != -1) {
    newAnchorInfo = initializeSlot(firstUninitializedSlot, anchor);
  } else {
    newAnchorInfo = initializeSlot(oldestSlot, anchor);
  }

  anchorCtx->anchorInfo = newAnchorInfo;
  return false;
}

bool AP_TdoaStorage::getAnchorCtx(const uint8_t anchor, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx) {
  anchorCtx->currentTime_ms = currentTime_ms;

  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    if (anchorInfoArray[i].isInitialized) {
      if (anchor == anchorInfoArray[i].id) {
        anchorCtx->anchorInfo = &anchorInfoArray[i];
        return true;
      }
    }
  }

  anchorCtx->anchorInfo = 0;
  return false;
}

uint8_t AP_TdoaStorage::getListOfAnchorIds(uint8_t unorderedAnchorList[], const int maxListSize) {
  int count = 0;

  for (int i = 0; i < ANCHOR_STORAGE_COUNT && count < maxListSize; i++) {
    if (anchorInfoArray[i].isInitialized) {
      unorderedAnchorList[count] = anchorInfoArray[i].id;
      count++;
    }
  }

  return count;
}

uint8_t AP_TdoaStorage::getListOfActiveAnchorIds(uint8_t unorderedAnchorList[], const int maxListSize, const uint32_t currentTime_ms) {
  int count = 0;

  const uint32_t expiryTime = currentTime_ms - ANCHOR_ACTIVE_VALIDITY_PERIOD;
  for (int i = 0; i < ANCHOR_STORAGE_COUNT && count < maxListSize; i++) {
    if (anchorInfoArray[i].isInitialized && anchorInfoArray[i].lastUpdateTime > expiryTime) {
      unorderedAnchorList[count] = anchorInfoArray[i].id;
      count++;
    }
  }

  return count;
}

uint8_t AP_TdoaStorage::getId(const tdoaAnchorContext_t* anchorCtx) {
  return anchorCtx->anchorInfo->id;
}

int64_t AP_TdoaStorage::getRxTime(const tdoaAnchorContext_t* anchorCtx) {
  return anchorCtx->anchorInfo->rxTime;
}

int64_t AP_TdoaStorage::getTxTime(const tdoaAnchorContext_t* anchorCtx) {
  return anchorCtx->anchorInfo->txTime;
}

uint8_t AP_TdoaStorage::getSeqNr(const tdoaAnchorContext_t* anchorCtx) {
  return anchorCtx->anchorInfo->seqNr;
}

uint32_t AP_TdoaStorage::getLastUpdateTime(const tdoaAnchorContext_t* anchorCtx) {
  return anchorCtx->anchorInfo->lastUpdateTime;
}

clockCorrectionStorage_t* AP_TdoaStorage::getClockCorrectionStorage(const tdoaAnchorContext_t* anchorCtx) {
  return &anchorCtx->anchorInfo->clockCorrectionStorage;
}

bool AP_TdoaStorage::getAnchorPosition(const tdoaAnchorContext_t* anchorCtx, point_t* position) {
  uint32_t now = anchorCtx->currentTime_ms;

  int32_t validCreationTime = now - ANCHOR_POSITION_VALIDITY_PERIOD;
  const tdoaAnchorInfo_t* anchorInfo = anchorCtx->anchorInfo;
  if ((int32_t)anchorInfo->position.timestamp > validCreationTime) {
    position->timestamp = anchorInfo->position.timestamp;
    position->x = anchorInfo->position.x;
    position->y = anchorInfo->position.y;
    position->z = anchorInfo->position.z;
    return true;
  }

  return false;
}

void AP_TdoaStorage::setAnchorPosition(tdoaAnchorContext_t* anchorCtx, const float x, const float y, const float z) {
  uint32_t now = anchorCtx->currentTime_ms;
  tdoaAnchorInfo_t* anchorInfo = anchorCtx->anchorInfo;

  anchorInfo->position.timestamp = now;
  anchorInfo->position.x = x;
  anchorInfo->position.y = y;
  anchorInfo->position.z = z;
}

void AP_TdoaStorage::setRxTxData(tdoaAnchorContext_t* anchorCtx, int64_t rxTime, int64_t txTime, uint8_t seqNr) {
  uint32_t now = anchorCtx->currentTime_ms;
  tdoaAnchorInfo_t* anchorInfo = anchorCtx->anchorInfo;

  anchorInfo->rxTime = rxTime;
  anchorInfo->txTime = txTime;
  anchorInfo->seqNr = seqNr;
  anchorInfo->lastUpdateTime = now;
}

double AP_TdoaStorage::getClockCorrection(const tdoaAnchorContext_t* anchorCtx) {
  return AP_ClockCorrectionEngine::get(&anchorCtx->anchorInfo->clockCorrectionStorage);
}

int64_t AP_TdoaStorage::getRemoteRxTime(const tdoaAnchorContext_t* anchorCtx, const uint8_t remoteAnchor) {
  const tdoaAnchorInfo_t* anchorInfo = anchorCtx->anchorInfo;

  for (int i = 0; i < REMOTE_ANCHOR_DATA_COUNT; i++) {
    if (remoteAnchor == anchorInfo->remoteAnchorData[i].id) {
      uint32_t now = anchorCtx->currentTime_ms;
      if (anchorInfo->remoteAnchorData[i].endOfLife > now) {
        return anchorInfo->remoteAnchorData[i].rxTime;
      }
      break;
    }
  }

  return 0;
}

void AP_TdoaStorage::setRemoteRxTime(tdoaAnchorContext_t* anchorCtx, const uint8_t remoteAnchor, const int64_t remoteRxTime, const uint8_t remoteSeqNr) {
  tdoaAnchorInfo_t* anchorInfo = anchorCtx->anchorInfo;

  int indexToUpdate = 0;
  uint32_t now = anchorCtx->currentTime_ms;
  uint32_t oldestTime = 0xFFFFFFFF;

  for (int i = 0; i < REMOTE_ANCHOR_DATA_COUNT; i++) {
    if (remoteAnchor == anchorInfo->remoteAnchorData[i].id) {
      indexToUpdate = i;
      break;
    }

    if (anchorInfo->remoteAnchorData[i].endOfLife < oldestTime) {
      oldestTime = anchorInfo->remoteAnchorData[i].endOfLife;
      indexToUpdate = i;
    }
  }

  anchorInfo->remoteAnchorData[indexToUpdate].id = remoteAnchor;
  anchorInfo->remoteAnchorData[indexToUpdate].rxTime = remoteRxTime;
  anchorInfo->remoteAnchorData[indexToUpdate].seqNr = remoteSeqNr;
  anchorInfo->remoteAnchorData[indexToUpdate].endOfLife = now + REMOTE_DATA_VALIDITY_PERIOD;
}

void AP_TdoaStorage::getRemoteSeqNrList(const tdoaAnchorContext_t* anchorCtx, int* remoteCount, uint8_t seqNr[], uint8_t id[]) {
  const tdoaAnchorInfo_t* anchorInfo = anchorCtx->anchorInfo;
  uint32_t now = anchorCtx->currentTime_ms;

  int count = 0;

  for (int i = 0; i < REMOTE_ANCHOR_DATA_COUNT; i++) {
    if (anchorInfo->remoteAnchorData[i].endOfLife > now) { //TODO endOfLife is never > now, why?
      id[count] = anchorInfo->remoteAnchorData[i].id;
      seqNr[count] = anchorInfo->remoteAnchorData[i].seqNr;
      count++;
    }
  }

  *remoteCount = count;
}

int64_t AP_TdoaStorage::getTimeOfFlight(const tdoaAnchorContext_t* anchorCtx, const uint8_t otherAnchor) {
  const tdoaAnchorInfo_t* anchorInfo = anchorCtx->anchorInfo;

  for (int i = 0; i < TOF_PER_ANCHOR_COUNT; i++) {
    if (otherAnchor == anchorInfo->tof[i].id) {
      uint32_t now = anchorCtx->currentTime_ms;
      if (anchorInfo->tof[i].endOfLife > now) {
        return anchorInfo->tof[i].tof;
      }
      break;
    }
  }

  return 0;
}

void AP_TdoaStorage::setTimeOfFlight(tdoaAnchorContext_t* anchorCtx, const uint8_t remoteAnchor, const int64_t tof) {
  tdoaAnchorInfo_t* anchorInfo = anchorCtx->anchorInfo;

  int indexToUpdate = 0;
  uint32_t now = anchorCtx->currentTime_ms;
  uint32_t oldestTime = 0xFFFFFFFF;

  for (int i = 0; i < TOF_PER_ANCHOR_COUNT; i++) {
    if (remoteAnchor == anchorInfo->tof[i].id) {
      indexToUpdate = i;
      break;
    }

    if (anchorInfo->tof[i].endOfLife < oldestTime) {
      oldestTime = anchorInfo->tof[i].endOfLife;
      indexToUpdate = i;
    }
  }

  anchorInfo->tof[indexToUpdate].id = remoteAnchor;
  anchorInfo->tof[indexToUpdate].tof = tof;
  anchorInfo->tof[indexToUpdate].endOfLife = now + TOF_VALIDITY_PERIOD;
}

bool AP_TdoaStorage::isAnchorInStorage(const uint8_t anchor) {
  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    if (anchorInfoArray[i].isInitialized) {
      if (anchor == anchorInfoArray[i].id) {
        return true;
      }
    }
  }

  return false;
}

tdoaAnchorInfo_t* AP_TdoaStorage::initializeSlot(const uint8_t slot, const uint8_t anchor) {
  anchorInfoArray[slot] = {0};
  anchorInfoArray[slot].isInitialized = true;
  anchorInfoArray[slot].id = anchor;

  return &anchorInfoArray[slot];
}

