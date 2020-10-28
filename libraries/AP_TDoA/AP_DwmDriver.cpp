

#include "AP_DwmDriver.h"


AP_DwmDriver* AP_DwmDriver::_singleton;



/*
void AP_DwmDriver::txCallback(dwDevice_t *dev) {

}

void AP_DwmDriver::rxCallback(dwDevice_t *dev) {

    //TODO this is TDoA3
    int dataLength = dwGetDataLength(dev);
    packet_t rxPacket;

    dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
    const uint8_t anchorId = rxPacket.sourceAddress & 0xff;

    dwTime_t arrival = {.full = 0};
    dwGetReceiveTimestamp(dev, &arrival);

    const rangePacket3_t* packet = (rangePacket3_t*)rxPacket.payload;
    if (packet->header.type == PACKET_TYPE_TDOA3) {
        const uint8_t seqNr = packet->header.seq & 0x7f;;

        hal.console->printf(" DwmDriver rxCallback: from %d seq %d\n", anchorId, seqNr);
    }
}

void AP_DwmDriver::rxTimeoutCallback(dwDevice_t *dev) {

}*/


