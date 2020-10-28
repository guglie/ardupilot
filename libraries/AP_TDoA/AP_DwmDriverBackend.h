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


#include <AP_HAL/AP_HAL.h>
#include "AP_DwmDriver_definitions.h"


#define UWB_READ_CALLBACK_FREQ 2000

#define DWM1000_ID 0xDECA0130

// timeout... check RX_FWTO register in dw1000 documentation
#define RX_TIMEOUT 10000


#define DWMDRIVER_DEBUG false




extern const AP_HAL::HAL& hal;


class AP_DwmDriverBackend {
private:
    static AP_DwmDriverBackend* _singleton;


    dwDevice_t dwm_device;
    static AP_HAL::OwnPtr<AP_HAL::SPIDevice> spidev;
    AP_HAL::Semaphore *spi_semaphore;

    // flags
    bool _dwm_read_loop_started = false;


    //timestamps
    uint32_t last_print_ms = 0;


public:
    dwDevice_t *dwm;

    AP_DwmDriverBackend() {
        _singleton = this;

        dwm = &dwm_device;
    }

    static AP_DwmDriverBackend* get_singleton() { return _singleton; }
    
    ~AP_DwmDriverBackend() {}
    
    void driver_init();

    // checks SPI device and starts SPI read loop
    void init_dwm();

    // dwm1000 setup
    bool setup_dwm();

    // a 2us loop to read dwm data on spi
    void dwm_read_loop();

    void handleReceive();
    void packetReceived(unsigned int dataLength);


    // Dwm1000 driver functions
    void dwNewConfiguration();
    void dwCommitConfiguration();
    void dwSetDefaults();

    // settings
    void dwUseExtendedFrameLength(bool val);
    void dwUseSmartPower(bool smartPower);
    void dwSuppressFrameCheck(bool val);
    void dwEnableMode(const uint8_t mode[]);
    void dwSetDataRate(uint8_t rate);
    void dwSetPulseFrequency(uint8_t freq);
    uint8_t dwGetPulseFrequency();
    void dwSetPreambleLength(uint8_t prealen);
    void dwSetChannel(uint8_t channel);
    void dwSetPreambleCode(uint8_t preacode);



    void conf_dwm();
    void newReceive();
    void getTime(dwTime_t* time);
    void dwSoftReset();
    void dwIdle();
    bool set_sys_reg();
    void dwInit();
    void dwEnableAllLeds();
    void newConfig();
    void dwTune();  //TODO remove old

    void dwEnableClock(dwClock_t clock);
    void dwConfigure();

    bool dwIsReceiveFailed();
    bool dwIsReceiveTimeout();

    void dwClearInterrupts();
    void dwManageLDE();

    void dwNewTransmit();
    void sendPower();
    void dwClearTransmitStatus();

    void dwSetAntenaDelay(dwTime_t delay);


    void dwSetData(uint8_t data[], unsigned int n);
    void dwStartTransmit();
    void dwStartReceive();

    void dwClearReceiveStatus();

    unsigned int dwGetDataLength();
    void dwGetData(uint8_t data[], unsigned int n);

    // timestamps
    void dwGetTransmitTimestamp(dwTime_t* time);
    void dwGetReceiveTimestamp(dwTime_t* time);
    void dwGetRawReceiveTimestamp(dwTime_t* time);
    void dwCorrectTimestamp(dwTime_t* timestamp);
    void dwGetSystemTimestamp(dwTime_t* time);
    bool dwIsTransmitDone();
    bool dwIsReceiveTimestampAvailable();
    float dwGetReceiveQuality();
    static float spiReadRxInfo();
    static float calculatePower(float base, float N, uint8_t pulseFrequency);
    float dwGetFirstPathPower();
    float dwGetReceivePower();


    static void writeValueToBytes(uint8_t data[], long val, unsigned int n);
    static void setBit(uint8_t data[], unsigned int n, unsigned int bit, bool val);
    static bool getBit(uint8_t data[], unsigned int n, unsigned int bit);

    static void readBytesOTP(uint16_t address, uint8_t data[]);


    bool dwIsReceiveDone();

    void dwRxSoftReset();

    // register read/write
    void dwReadSystemConfigurationRegister();
    void dwWriteSystemConfigurationRegister();

    void dwReadSystemEventStatusRegister();

    void dwReadNetworkIdAndDeviceAddress();
    void dwWriteNetworkIdAndDeviceAddress();

    void dwReadSystemEventMaskRegister();
    void dwWriteSystemEventMaskRegister();

    void dwReadChannelControlRegister();
    void dwWriteChannelControlRegister();

    void dwReadTransmitFrameControlRegister();
    void dwWriteTransmitFrameControlRegister();

    // ---
    void dwSetReceiveWaitTimeout(uint16_t timeout);
    void dwSetFrameFilter(bool val);
    void dwSetFrameFilterBehaveCoordinator(bool val);
    void dwSetFrameFilterAllowBeacon(bool val);
    void dwSetFrameFilterAllowData(bool val);
    void dwSetFrameFilterAllowAcknowledgement(bool val);
    void dwSetFrameFilterAllowMAC(bool val);
    void dwSetFrameFilterAllowReserved(bool val);
    void dwSetDoubleBuffering(bool val);
    void dwSetInterruptPolarity(bool val);
    void dwSetReceiverAutoReenable(bool val);
    void dwInterruptOnSent(bool val);
    void dwInterruptOnReceived(bool val);
    void dwInterruptOnReceiveFailed(bool val);
    void dwInterruptOnReceiveTimeout(bool val);
    void dwInterruptOnReceiveTimestampAvailable(bool val);
    void dwInterruptOnAutomaticAcknowledgeTrigger(bool val);

    // Utility wrappers to the transfer() function
    // Bitwise operations are used to set 8bit header frames (see dw1000 documentation)
    static void dwRead(uint8_t reg, uint8_t* val, uint8_t len);
    static void dwRead(uint8_t reg, uint16_t sub , uint8_t* val, uint8_t len);
    uint16_t dwSpiRead16(uint8_t regid, uint32_t address);
    static void dwWrite(uint8_t reg, uint8_t* val, uint8_t len);
    static void dwWrite(uint8_t reg, uint16_t sub , uint8_t* val, uint8_t len);
    static void dwSpiWrite8(uint8_t regid, uint32_t address, uint8_t data);


    // delay wrapper
    static void delayms(unsigned int delay) {
        hal.scheduler->delay(delay);
    }
};