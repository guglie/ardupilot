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

#include "AP_Tdoa3Tag.h"


// stats
uint32_t AP_Tdoa3Tag::tdoaPushedToEkf;

// object pointers
AP_Tdoa3Tag* AP_Tdoa3Tag::_singleton;
AP_TdoaStorage* AP_Tdoa3Tag::storage;
AP_TdoaEngine* AP_Tdoa3Tag::engine;

bool AP_Tdoa3Tag::rangingOk;

uint32_t AP_Tdoa3Tag::last_update_tdoa_ms;
uint64_t AP_Tdoa3Tag::last_update_tdoa_us;

static void noOp(tdoaMeasurement_t* tdoaMeasurement, const uint8_t idA, const uint8_t idB, uint64_t tdoa_systime_us) {};


AP_Tdoa3Tag::AP_Tdoa3Tag() /*: ekf()*/ {
    _singleton = this;

    //ekf = TDOA_EKF(0);  //TODO check initial height
    ekf = TDOA_EKF();

    engine_instance = AP_TdoaEngine(AP_HAL::millis(), &noOp /*&sendTdoaToEstimatorCallback*/, TdoaEngineMatchingAlgorithmRandom);
    engine = &engine_instance;
    storage = AP_TdoaStorage::get_singleton();

    last_update_ms = AP_HAL::millis();
    //last_update_tdoa_ms = AP_HAL::millis();

    //TODO Tdoa3 tag sets this
    //dwSetReceiveWaitTimeout(dev, TDOA3_RECEIVE_TIMEOUT);
    //dwCommitConfiguration(dev);

    rangingOk = false;

    thread_started = false;
}

bool AP_Tdoa3Tag::start_thread() {
    //hal.scheduler->register_timer_process(FUNCTOR_BIND(AP_Tdoa3Tag::get_singleton(), &AP_Tdoa3Tag::pos_estimator_loop, void));

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Tdoa3Tag::pos_estimator_loop, void));

    /*if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Tdoa3Tag::pos_estimator_thread, void),
                                      "tdoa", 8192, AP_HAL::Scheduler::PRIORITY_TIMER, -1)) {
        return true;
    }*/

    return false;
}

void AP_Tdoa3Tag::pos_estimator_thread() {
    ekf = TDOA_EKF();

    while (true) {
        pos_estimator_loop();
    }
}

void AP_Tdoa3Tag::pos_estimator_loop() {
    if(!thread_started) {
        thread_started = true;
        ekf = TDOA_EKF();
    }

    // reduce update frequency
    const uint32_t now = AP_HAL::millis();
    if ((now - last_update_ms) < 500) {
        return;
    }

    Vector3f pos = ekf.getPosition();

    /*
    uint8_t sequence_of_last_seen = 0;
    float x_last_seen = 0, y_last_seen = 0, z_last_seen = 0;

    tdoaAnchorContext_t lastAnchorCtx;
    bool lastAnchorCtxFound = false;

    for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
        if (storage->anchorInfoArray[i].isInitialized) {
            if (engine->last_seen_anchor == storage->anchorInfoArray[i].id) {
                lastAnchorCtx.anchorInfo = &engine->anchorInfoArray[i];

                sequence_of_last_seen = lastAnchorCtx.anchorInfo->seqNr;
                x_last_seen = lastAnchorCtx.anchorInfo->position.x;
                y_last_seen = lastAnchorCtx.anchorInfo->position.y;
                z_last_seen = lastAnchorCtx.anchorInfo->position.z;
                lastAnchorCtxFound = true;
                break;
            }
        }
    }*/

    //uint8_t activeAnchors[16];
    //uint8_t activeAnchorsCount = storage->getListOfActiveAnchorIds(activeAnchors, 16, now);  // 10

    if(TDOA3TAG_DEBUG) hal.console->printf("\n AP_Tdoa3Tag pos %f %f %f - tdoas %ld  pkts %ld  enqueued %ld  EKFlastObTime %.3f  NeedReset %d  OutFiltLev %.1f\n",
                        pos.x, pos.y, pos.z, engine->tdoa_enqueued, engine->packets_processed, engine->tdoa_enqueued, ekf.lastObservationTime, ekf.needReset, ekf.outlierFilter.acceptanceLevel);

    if (engine->tdoaQueue.available() > 0) {
        tdoaMeasurement_t tdoa;
        //engine->tdoaQueue.pop(&tdoa);
        engine->tdoaQueue.pop(tdoa);

        /*Vector3f posA = Vector3f(tdoa.anchorPosition[0].x, tdoa.anchorPosition[0].y, tdoa.anchorPosition[0].z);
        Vector3f posB = Vector3f(tdoa.anchorPosition[1].x, tdoa.anchorPosition[1].y, tdoa.anchorPosition[1].z);

        ekf.update(tdoa.systime_us, tdoa.distanceDiff*100, posA, posB, tdoa.stdDev, tdoa.idA, tdoa.idB);//*/

        if (TDOA3TAG_DEBUG && engine->tdoa_enqueued > 1000) {
            ekf.test();
        }

        if(TDOA3TAG_DEBUG) hal.console->printf("  --- tdoa %d %d  diff: %.2f   %lld \n", tdoa.idA, tdoa.idB, tdoa.distanceDiff, tdoa.systime_us);
    }

    last_update_ms = now;

    // read tdoa from queue and pass them to the TDOA EKF

    // send position to AP ekf
}

bool AP_Tdoa3Tag::isValidTimeStamp(const int64_t anchorRxTime) {
    return anchorRxTime != 0;
}

int AP_Tdoa3Tag::updateRemoteData(tdoaAnchorContext_t* anchorCtx, const void* payload) {
    const rangePacket3_t* packet = (rangePacket3_t*)payload;
    const uint8_t* anchorDataPtr = &packet->remoteAnchorData;  //TODO void* ??
    for (uint8_t i = 0; i < packet->header.remoteCount; i++) {
        remoteAnchorDataFull_t* anchorData = (remoteAnchorDataFull_t*)anchorDataPtr;

        uint8_t remoteId = anchorData->id;
        int64_t remoteRxTime = anchorData->rxTimeStamp;
        uint8_t remoteSeqNr = anchorData->seq & 0x7f;

        if (isValidTimeStamp(remoteRxTime)) {
            storage->setRemoteRxTime(anchorCtx, remoteId, remoteRxTime, remoteSeqNr);
        }

        bool hasDistance = ((anchorData->seq & 0x80) != 0);
        if (hasDistance) {
            int64_t tof = anchorData->distance;
            if (isValidTimeStamp(tof)) {
                storage->setTimeOfFlight(anchorCtx, remoteId, tof);
                /* TODO stats
                uint8_t anchorId = storage->getId(anchorCtx);
                tdoaStats_t* stats = &tdoaEngineState.stats;
                if (anchorId == stats->anchorId && remoteId == stats->remoteAnchorId) {
                    stats->tof = (uint16_t)tof;
                }*/
            }

            anchorDataPtr += sizeof(remoteAnchorDataFull_t);
        } else {
            anchorDataPtr += sizeof(remoteAnchorDataShort_t);
        }
    }

    return (uint8_t*)anchorDataPtr - (uint8_t*)packet;
}

void AP_Tdoa3Tag::handleLppShortPacket(tdoaAnchorContext_t* anchorCtx, const uint8_t* data, const int length) {
    uint8_t type = data[0];

    if (type == LPP_SHORT_ANCHORPOS) {
        struct lppShortAnchorPos_s* newpos = (struct lppShortAnchorPos_s*)&data[1];
        storage->setAnchorPosition(anchorCtx, newpos->x, newpos->y, newpos->z);
    }
}

void AP_Tdoa3Tag::handleLppPacket(const int dataLength, int rangePacketLength, const packet_t* rxPacket, tdoaAnchorContext_t* anchorCtx) {
    const int32_t payloadLength = dataLength - MAC802154_HEADER_LENGTH;
    const int32_t startOfLppDataInPayload = rangePacketLength;
    const int32_t lppDataLength = payloadLength - startOfLppDataInPayload;
    const int32_t lppTypeInPayload = startOfLppDataInPayload + 1;

    if (lppDataLength > 0) {
        const uint8_t lppPacketHeader = rxPacket->payload[startOfLppDataInPayload];
        if (lppPacketHeader == LPP_HEADER_SHORT_PACKET) {
            const int32_t lppTypeAndPayloadLength = lppDataLength - 1;
            handleLppShortPacket(anchorCtx, &rxPacket->payload[lppTypeInPayload], lppTypeAndPayloadLength);
        }
    }
}

void AP_Tdoa3Tag::rxcallback(unsigned int dataLength) {
    /* TODO stats
    tdoaStats_t* stats = &tdoaEngineState.stats;
    STATS_CNT_RATE_EVENT(&stats->packetsReceived);
    */

    packet_t rxPacket;
    //spidev->read_registers(RX_BUFFER, (uint8_t*)&rxPacket, dataLength);
    AP_DwmDriverBackend::get_singleton()->dwGetData((uint8_t*)&rxPacket, dataLength);

    dwTime_t arrival = {.full = 0};
    AP_DwmDriverBackend::get_singleton()->dwGetReceiveTimestamp(&arrival);

    uint8_t anchorId = rxPacket.sourceAddress & 0xff;

    const int64_t rxAn_by_T_in_cl_T = arrival.full;

    const rangePacket3_t* packet = (rangePacket3_t*)rxPacket.payload;

    if (packet->header.type == PACKET_TYPE_TDOA3) {
        const int64_t txAn_in_cl_An = packet->header.txTimeStamp;
        const uint8_t seqNr = packet->header.seq & 0x7f;

        tdoaAnchorContext_t anchorCtx;
        uint32_t now_ms = AP_HAL::millis();  //these timings are working correctly
        uint64_t now_us = AP_HAL::micros64();

        engine->getAnchorCtxForPacketProcessing(anchorId, now_ms, &anchorCtx);
        int rangeDataLength = updateRemoteData(&anchorCtx, packet);
        engine->processPacket(&anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, now_us);
        storage->setRxTxData(&anchorCtx, rxAn_by_T_in_cl_T, txAn_in_cl_An, seqNr);
        handleLppPacket(dataLength, rangeDataLength, &rxPacket, &anchorCtx);
        rangingOk = true;
    }
}

void AP_Tdoa3Tag::setRadioInReceiveMode(dwDevice_t* dev) {
    AP_DwmDriverBackend::get_singleton()->newReceive();
    AP_DwmDriverBackend::get_singleton()->dwSetDefaults();
    AP_DwmDriverBackend::get_singleton()->dwStartReceive();
}

void AP_Tdoa3Tag::sendLppShort(dwDevice_t* dev, lpsLppShortPacket_t* packet) {
    static packet_t txPacket;
    AP_DwmDriverBackend::get_singleton()->dwIdle();

    MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);

    txPacket.payload[LPS_TDOA3_TYPE] = LPP_HEADER_SHORT_PACKET;
    memcpy(&txPacket.payload[LPS_TDOA3_SEND_LPP_PAYLOAD], packet->data, packet->length);

    txPacket.pan = 0xbccf;
    txPacket.sourceAddress = 0xbccf000000000000 | 0xff;
    txPacket.destAddress = 0xbccf000000000000 | packet->dest;

    AP_DwmDriverBackend::get_singleton()->dwNewTransmit();
    AP_DwmDriverBackend::get_singleton()->dwSetDefaults();
    AP_DwmDriverBackend::get_singleton()->dwSetData((uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 1 + packet->length);

    AP_DwmDriverBackend::get_singleton()->dwStartTransmit();
}

bool AP_Tdoa3Tag::sendLpp(dwDevice_t* dev) {
    bool lppPacketToSend = lpsGetLppShort(&lppPacket);
    if (lppPacketToSend) {
        sendLppShort(dev, &lppPacket);
        return true;
    }

    return false;
}

/*  uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
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

    uint32_t now_ms = AP_HAL::millis();
    tdoaStatsUpdate(&tdoaEngineState.stats, now_ms);

    return MAX_TIMEOUT;
  }*/

bool AP_Tdoa3Tag::getAnchorPosition(const uint8_t anchorId, point_t* position) {
    tdoaAnchorContext_t anchorCtx;
    uint32_t now_ms = AP_HAL::millis();

    bool contextFound = storage->getAnchorCtx(anchorId, now_ms, &anchorCtx);
    if (contextFound) {
        storage->getAnchorPosition(&anchorCtx, position);
        return true;
    }

    return false;
}

uint8_t AP_Tdoa3Tag::getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
    return storage->getListOfAnchorIds(unorderedAnchorList, maxListSize);
}

uint8_t AP_Tdoa3Tag::getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
    uint32_t now_ms = AP_HAL::millis();
    return storage->getListOfActiveAnchorIds(unorderedAnchorList, maxListSize, now_ms);
}
