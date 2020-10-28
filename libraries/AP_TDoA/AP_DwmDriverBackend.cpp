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

#include "AP_DwmDriverBackend.h"
#include "AP_DwmDriver.h"

AP_HAL::OwnPtr<AP_HAL::SPIDevice> AP_DwmDriverBackend::spidev;
AP_DwmDriverBackend* AP_DwmDriverBackend::_singleton;


static const uint8_t BIAS_500_16_ZERO = 10;
static const uint8_t BIAS_500_64_ZERO = 8;
static const uint8_t BIAS_900_16_ZERO = 7;
static const uint8_t BIAS_900_64_ZERO = 7;

// range bias tables (500 MHz in [mm] and 900 MHz in [2mm] - to fit into bytes)
static const uint8_t BIAS_500_16[] = {198, 187, 179, 163, 143, 127, 109, 84, 59, 31,   0,  36,  65,  84,  97, 106, 110, 112};
static const uint8_t BIAS_500_64[] = {110, 105, 100,  93,  82,  69,  51, 27,  0, 21,  35,  42,  49,  62,  71,  76,  81,  86};
static const uint8_t BIAS_900_16[] = {137, 122, 105, 88, 69,  47,  25,  0, 21, 48, 79, 105, 127, 147, 160, 169, 178, 197};
static const uint8_t BIAS_900_64[] = {147, 133, 117, 99, 75, 50, 29,  0, 24, 45, 63, 76, 87, 98, 116, 122, 132, 142};

// Default Mode of operation
const uint8_t MODE_LONGDATA_RANGE_LOWPOWER[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_2048};
const uint8_t MODE_SHORTDATA_FAST_LOWPOWER[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_128};
const uint8_t MODE_LONGDATA_FAST_LOWPOWER[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_1024};
const uint8_t MODE_SHORTDATA_FAST_ACCURACY[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_128};
const uint8_t MODE_LONGDATA_FAST_ACCURACY[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_1024};
const uint8_t MODE_LONGDATA_RANGE_ACCURACY[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_2048};
const uint8_t MODE_SHORTDATA_MID_ACCURACY[] = {TRX_RATE_850KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_128};
const uint8_t MODE_LONGDATA_MID_ACCURACY[] = {TRX_RATE_850KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_1024};

uint32_t statsPacketsReceived; //TODO remove

void AP_DwmDriverBackend::driver_init() {
    // try initialization a few times
    //for(uint8_t i=0; i<10; i++) {
        //hal.scheduler->delay(5);
        //if(DWMDRIVER_DEBUG) hal.console->printf("\n Trying to initialize DWM1000 %d\n",i);

        spidev = hal.spi->get_device("dwm1000");
        if (!spidev) {
            if(DWMDRIVER_DEBUG) hal.console->printf(" dwm1000 spi device not found \n");
            return; //continue; // dwm1000 spi device not found: retry
        }
        
        
        // set environmental values
        spidev->set_read_flag(0x00);
        spidev->set_speed(AP_HAL::Device::SPEED_HIGH); // with low speed doesn't work
        spi_semaphore = spidev->get_semaphore();


        // block the semaphore
        if (!spi_semaphore->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            if(DWMDRIVER_DEBUG) hal.console->printf(" dwm1000 init can't take sempahore \n");
            return;
        }
        //check if the device id is correct
        uint32_t id = 0x00000000;
        if(!spidev->read_registers(DEV_ID, (uint8_t*)&id, LEN_DEV_ID)) {
            if(DWMDRIVER_DEBUG) hal.console->printf(" dwm1000 init can't read id \n");
            spi_semaphore->give();
            return;
        }

        if(id != 0xDECA0130) { // 0xBC950360 is read with spi set to low speed
            if(DWMDRIVER_DEBUG) hal.console->printf(" dwm1000 init ID ERROR, read id is %08lx \n", id);
            spi_semaphore->give();
            return;
        }

        // release semaphore
        spi_semaphore->give();
        if(DWMDRIVER_DEBUG) hal.console->printf(" dwm1000 init - id checked\n");

        // setup the DWM1000
        if(!setup_dwm()) {
            if(DWMDRIVER_DEBUG) hal.console->printf(" dwm1000 setup failed \n");
            return; //continue; // setup failed: retry
        }
        
        if(!_dwm_read_loop_started) {
            // create the thread which reads the packets it runs every UWB_READ_CALLBACK_FREQ usec (1000 usec is the upper bound to how slow we can check (1000 usec = 8anchor period / 2))
            if(spidev->register_periodic_callback( UWB_READ_CALLBACK_FREQ, FUNCTOR_BIND_MEMBER(&AP_DwmDriverBackend::dwm_read_loop, void)) == nullptr) {
                return; //continue; // can't start spi periodic callback: retry
            }
            _dwm_read_loop_started = true;
        }

        //break; // initialization done, no need to retry
    //}

    if(_dwm_read_loop_started) {
        if(DWMDRIVER_DEBUG) hal.console->printf(" dwm1000 starting pos estimator thread \n");

        // start position estimator loop in another thread
        AP_Tdoa3Tag::get_singleton()->start_thread();
    }

    // TODO create LPP out queue

    // TODO set LPS mode auto to allow mode auto-detect
}



bool AP_DwmDriverBackend::setup_dwm() {
    // CHECK DEVICE AND TAKE SEMAPHORE
    if (!spidev) {
        if(DWMDRIVER_DEBUG) hal.console->printf(" dwm1000 setup - spi error \n");
        return false;
    }
    if (!spi_semaphore->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if(DWMDRIVER_DEBUG) hal.console->printf(" dwm1000 setup - semaphore error \n");
        return false;
    }

    dwInit();

    dwConfigure();

    // configure SYS_CFG and SYS_CTRL
    if (!set_sys_reg()) {
        // configure failed
        spi_semaphore->give();
        if(DWMDRIVER_DEBUG) hal.console->printf(" dwm1000 setup - set_sys_reg error \n");
        return false;
    }

    // enable LEDs
    dwEnableAllLeds();

    dwTime_t delay = {.full = 0};
    dwSetAntenaDelay(delay);

    //dwAttachSentHandler(txCallback);
    //dwAttachReceivedHandler(rxCallback);
    //dwAttachReceiveTimeoutHandler(rxTimeoutCallback);

    // Read Network Id And Device Address 
	// Read System Configuration Register 
	// Read Channel Control Register 
	// Read Transmit Frame Control Register 
	// Read System Event Mask Register
    dwNewConfiguration();
    dwSetDefaults();

    dwEnableMode(MODE_SHORTDATA_FAST_ACCURACY);
    dwSetChannel(CHANNEL_2);
    dwSetPreambleCode(PREAMBLE_CODE_64MHZ_9);

    dwUseSmartPower(true);

    dwSetReceiveWaitTimeout(RX_TIMEOUT);

    dwCommitConfiguration();

    // TODO ?
    uint8_t send2[5] = {0x8D, 0x00, 0x01, 0x00, 0x00};
    spidev->transfer(send2, 5 ,nullptr, 0);

    // setup done
    spi_semaphore->give();
    return true;
}


void AP_DwmDriverBackend::dwm_read_loop() {

    /*handleModeSwitch();
    do
    {
        //xSemaphoreTake(algoSemaphore, portMAX_DELAY);
        dwHandleInterrupt(dwm);
        //xSemaphoreGive(algoSemaphore);
    } while (digitalRead(GPIO_PIN_IRQ) != 0);*/

    /*if(AP_HAL::millis() - last_print_ms > 1000) {
        if(DWMDRIVER_DEBUG) hal.console->printf(" dwm1000 dwm_read_loop. Packets %d \n", statsPacketsReceived);
        last_print_ms = AP_HAL::millis();
    }*/

    //---- READ vvv

    handleReceive();
}


void AP_DwmDriverBackend::handleReceive() {
    // check in the status register for a new message
    dwReadSystemEventStatusRegister(); //dwRead(SYS_STATUS, dwm->sysstatus, LEN_SYS_STATUS);
    // if it's there then read it
    if(dwIsReceiveDone()) {
    	statsPacketsReceived++;

		unsigned int dataLength = dwGetDataLength();
        if(dataLength <= 0) {
        	//if no data return in receive mode and exit current read loop
			newReceive();
            return;
        }
		
        // TODO here we could support other algorithms
        AP_Tdoa3Tag::get_singleton()->rxcallback(dataLength);

        // setup the DWM1000 to receive again
        newReceive();
    } else {
    	//if there's no messages, check if there's corrupted data occupying the buffer and get back to receive state
        if( dwIsReceiveTimeout() || dwIsReceiveFailed() ) {
            dwRxSoftReset();
            newReceive();
        }
    }
}


void AP_DwmDriverBackend::newReceive() {
    dwIdle();
    // OLD memset(dwm->sysctrl, 0, LEN_SYS_CTRL);
    dwClearReceiveStatus();
    dwm->deviceMode = RX_MODE;
    dwStartReceive();
}

//K
void AP_DwmDriverBackend::dwInit() {

    dwm->userdata = nullptr;

    /* Device default state */
    dwm->extendedFrameLength = FRAME_LENGTH_NORMAL;
    dwm->pacSize = PAC_SIZE_8;
    dwm->pulseFrequency = TX_PULSE_FREQ_16MHZ;
    dwm->dataRate = TRX_RATE_6800KBPS;
    dwm->preambleLength = TX_PREAMBLE_LEN_128;
    dwm->preambleCode = PREAMBLE_CODE_16MHZ_4;
    dwm->channel = CHANNEL_5;
    dwm->smartPower = false;
    dwm->frameCheck = true;
    dwm->permanentReceive = false;
    dwm->deviceMode = IDLE_MODE;
    dwm->forceTxPower = false;

    writeValueToBytes(dwm->antennaDelay.raw, 16384, LEN_STAMP);
}

//K
void AP_DwmDriverBackend::dwConfigure() {
    dwEnableClock(dwClockAuto);
    delayms(5);

    //  RESET DWM1000, TODO support hard pin reset
    dwSoftReset();

    // Set default address
    memset(dwm->networkAndAddress, 0xff, LEN_PANADR);
    dwWrite(PANADR, dwm->networkAndAddress, LEN_PANADR);

    //default configuration
    memset(dwm->syscfg, 0, LEN_SYS_CFG);
    dwSetDoubleBuffering(false);
    dwSetInterruptPolarity(true);
    dwWriteSystemConfigurationRegister();
    // default interrupt mask, i.e. no interrupts
    dwClearInterrupts();
    dwWriteSystemEventMaskRegister();
    // load LDE micro-code
    dwEnableClock(dwClockXti);
    delayms(5);  //delay 5ms
    dwManageLDE();
    delayms(5);  //delay 5ms
    dwEnableClock(dwClockPll);
    delayms(5);  //delay 5ms
}

//K
void AP_DwmDriverBackend::dwNewConfiguration() {
	dwIdle();
	dwReadNetworkIdAndDeviceAddress();
	dwReadSystemConfigurationRegister();
	dwReadChannelControlRegister();
	dwReadTransmitFrameControlRegister();
	dwReadSystemEventMaskRegister();
}
//K
void AP_DwmDriverBackend::dwCommitConfiguration() {
	// write all configurations back to device
	dwWriteNetworkIdAndDeviceAddress();
	dwWriteSystemConfigurationRegister();
	dwWriteChannelControlRegister();
	dwWriteTransmitFrameControlRegister();
	dwWriteSystemEventMaskRegister();
	// tune according to configuration
	dwTune();
	// TODO clean up code + antenna delay/calibration API
	// TODO setter + check not larger two bytes integer
	// uint8_t antennaDelayBytes[LEN_STAMP];
	// writeValueToBytes(antennaDelayBytes, 16384, LEN_STAMP);
	// dev->antennaDelay.setTimestamp(antennaDelayBytes);
	// dwSpiRead(dev, TX_ANTD, antennaDelayBytes, LEN_TX_ANTD);
	// dwSpiRead(dev, LDE_IF, LDE_RXANTD_SUB, antennaDelayBytes, LEN_LDE_RXANTD);
	dwWrite(TX_ANTD, dwm->antennaDelay.raw, LEN_TX_ANTD);
	dwWrite(LDE_IF, LDE_RXANTD_SUB, dwm->antennaDelay.raw, LEN_LDE_RXANTD);
}

//K
void AP_DwmDriverBackend::dwSetDefaults() {
	if(dwm->deviceMode == TX_MODE) {

	} else if(dwm->deviceMode == RX_MODE) {

	} else if(dwm->deviceMode == IDLE_MODE) {
		dwUseExtendedFrameLength(false);
		dwUseSmartPower(false);
		dwSuppressFrameCheck(false);
		//for global frame filtering
		dwSetFrameFilter(false);
		//for data frame (poll, poll_ack, range, range report, range failed) filtering
		dwSetFrameFilterAllowData(false);
		//for reserved (blink) frame filtering
		dwSetFrameFilterAllowReserved(false);
		//setFrameFilterAllowMAC(true);
		//setFrameFilterAllowBeacon(true);
		//setFrameFilterAllowAcknowledgement(true);
		dwInterruptOnSent(true);
		dwInterruptOnReceived(true);
		dwInterruptOnReceiveTimeout(true);
		dwInterruptOnReceiveFailed(false);
		dwInterruptOnReceiveTimestampAvailable(false);
		dwInterruptOnAutomaticAcknowledgeTrigger(false);
		dwSetReceiverAutoReenable(true);
		// default mode when powering up the chip
		// still explicitly selected for later tuning
		dwEnableMode(MODE_LONGDATA_RANGE_LOWPOWER);
	}
}
//K
void AP_DwmDriverBackend::dwUseExtendedFrameLength(bool val) {
	dwm->extendedFrameLength = (val ? FRAME_LENGTH_EXTENDED : FRAME_LENGTH_NORMAL);
	dwm->syscfg[2] &= 0xFC;
	dwm->syscfg[2] |= dwm->extendedFrameLength;
}
//K
void AP_DwmDriverBackend::dwUseSmartPower(bool smartPower) {
	dwm->smartPower = smartPower;
	setBit(dwm->syscfg, LEN_SYS_CFG, DIS_STXP_BIT, !smartPower);
}
//K
void AP_DwmDriverBackend::dwSuppressFrameCheck(bool val) {
	dwm->frameCheck = !val;
}


//K
void AP_DwmDriverBackend::dwEnableMode(const uint8_t mode[]) {
	dwSetDataRate(mode[0]);
	dwSetPulseFrequency(mode[1]);
	dwSetPreambleLength(mode[2]);
	// TODO add channel and code to mode tuples
	// TODO add channel and code settings with checks (see Table 58)
	dwSetChannel(CHANNEL_5);
	if(mode[1] == TX_PULSE_FREQ_16MHZ) {
		dwSetPreambleCode(PREAMBLE_CODE_16MHZ_4);
	} else {
		dwSetPreambleCode(PREAMBLE_CODE_64MHZ_10);
	}
}
//K
void AP_DwmDriverBackend::dwSetDataRate(uint8_t rate) {
	rate &= 0x03;
	dwm->txfctrl[1] &= 0x83;
	dwm->txfctrl[1] |= (uint8_t)((rate << 5) & 0xFF);
	// special 110kbps flag
	if(rate == TRX_RATE_110KBPS) {
		setBit(dwm->syscfg, LEN_SYS_CFG, RXM110K_BIT, true);
	} else {
		setBit(dwm->syscfg, LEN_SYS_CFG, RXM110K_BIT, false);
	}
	// SFD mode and type (non-configurable, as in Table )
	if(rate == TRX_RATE_6800KBPS) {
		setBit(dwm->chanctrl, LEN_CHAN_CTRL, DWSFD_BIT, false);
		setBit(dwm->chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, false);
		setBit(dwm->chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, false);
	} else {
		setBit(dwm->chanctrl, LEN_CHAN_CTRL, DWSFD_BIT, true);
		setBit(dwm->chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, true);
		setBit(dwm->chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, true);

	}
	uint8_t sfdLength;
	if(rate == TRX_RATE_6800KBPS) {
		sfdLength = 0x08;
	} else if(rate == TRX_RATE_850KBPS) {
		sfdLength = 0x10;
	} else {
		sfdLength = 0x40;
	}
	dwWrite(USR_SFD, SFD_LENGTH_SUB, &sfdLength, LEN_SFD_LENGTH);
	dwm->dataRate = rate;
}
//K
void AP_DwmDriverBackend::dwSetPulseFrequency(uint8_t freq) {
	freq &= 0x03;
	dwm->txfctrl[2] &= 0xFC;
	dwm->txfctrl[2] |= (uint8_t)(freq & 0xFF);
	dwm->chanctrl[2] &= 0xF3;
	dwm->chanctrl[2] |= (uint8_t)((freq << 2) & 0xFF);
	dwm->pulseFrequency = freq;

}
//K
uint8_t AP_DwmDriverBackend::dwGetPulseFrequency() {
	return dwm->pulseFrequency;
}
//K
void AP_DwmDriverBackend::dwSetPreambleLength(uint8_t prealen) {
	prealen &= 0x0F;
	dwm->txfctrl[2] &= 0xC3;
	dwm->txfctrl[2] |= (uint8_t)((prealen << 2) & 0xFF);
	if(prealen == TX_PREAMBLE_LEN_64 || prealen == TX_PREAMBLE_LEN_128) {
		dwm->pacSize = PAC_SIZE_8;
	} else if(prealen == TX_PREAMBLE_LEN_256 || prealen == TX_PREAMBLE_LEN_512) {
		dwm->pacSize = PAC_SIZE_16;
	} else if(prealen == TX_PREAMBLE_LEN_1024) {
		dwm->pacSize = PAC_SIZE_32;
	} else {
		dwm->pacSize = PAC_SIZE_64;
	}
	dwm->preambleLength = prealen;
}
//K
void AP_DwmDriverBackend::dwSetChannel(uint8_t channel) {
	channel &= 0xF;
	dwm->chanctrl[0] = ((channel | (channel << 4)) & 0xFF);
	dwm->channel = channel;
}
//K
void AP_DwmDriverBackend::dwSetPreambleCode(uint8_t preacode) {
	preacode &= 0x1F;
	dwm->chanctrl[2] &= 0x3F;
	dwm->chanctrl[2] |= ((preacode << 6) & 0xFF);
	dwm->chanctrl[3] = 0x00;
	dwm->chanctrl[3] = ((((preacode >> 2) & 0x07) | (preacode << 3)) & 0xFF);
	dwm->preambleCode = preacode;
}

//TODO compare with tune
/*void AP_DwmDriverBackend::dwTune_NEW() {
	// these registers are going to be tuned/configured
	uint8_t agctune1[LEN_AGC_TUNE1] = {0};
    uint8_t agctune2[LEN_AGC_TUNE2] = {0};
    uint8_t agctune3[LEN_AGC_TUNE3] = {0};
    uint8_t drxtune0b[LEN_DRX_TUNE0b] = {0};
    uint8_t drxtune1a[LEN_DRX_TUNE1a] = {0};
    uint8_t drxtune1b[LEN_DRX_TUNE1b] = {0};
    uint8_t drxtune2[LEN_DRX_TUNE2] = {0};
    uint8_t drxtune4H[LEN_DRX_TUNE4H] = {0};
    uint8_t ldecfg1[LEN_LDE_CFG1] = {0};
    uint8_t ldecfg2[LEN_LDE_CFG2] = {0};
    uint8_t lderepc[LEN_LDE_REPC] = {0};
    uint8_t txpower[LEN_TX_POWER] = {0};
    uint8_t rfrxctrlh[LEN_RF_RXCTRLH] = {0};
    uint8_t rftxctrl[LEN_RF_TXCTRL] = {0};
    uint8_t tcpgdelay[LEN_TC_PGDELAY] = {0};
    uint8_t fspllcfg[LEN_FS_PLLCFG] = {0};
    uint8_t fsplltune[LEN_FS_PLLTUNE] = {0};
    uint8_t fsxtalt[LEN_FS_XTALT] = {0};
	// AGC_TUNE1
	if(dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		writeValueToBytes(agctune1, 0x8870, LEN_AGC_TUNE1);
	} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		writeValueToBytes(agctune1, 0x889B, LEN_AGC_TUNE1);
	} else {
		// TODO proper error/warning handling
	}
	// AGC_TUNE2
	writeValueToBytes(agctune2, 0x2502A907L, LEN_AGC_TUNE2);
	// AGC_TUNE3
	writeValueToBytes(agctune3, 0x0035, LEN_AGC_TUNE3);
	// DRX_TUNE0b (already optimized according to Table 20 of user manual)
	if(dev->dataRate == TRX_RATE_110KBPS) {
		writeValueToBytes(drxtune0b, 0x0016, LEN_DRX_TUNE0b);
	} else if(dev->dataRate == TRX_RATE_850KBPS) {
		writeValueToBytes(drxtune0b, 0x0006, LEN_DRX_TUNE0b);
	} else if(dev->dataRate == TRX_RATE_6800KBPS) {
		writeValueToBytes(drxtune0b, 0x0001, LEN_DRX_TUNE0b);
	} else {
		// TODO proper error/warning handling
	}
	// DRX_TUNE1a
	if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		writeValueToBytes(drxtune1a, 0x0087, LEN_DRX_TUNE1a);
	} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		writeValueToBytes(drxtune1a, 0x008D, LEN_DRX_TUNE1a);
	} else {
		// TODO proper error/warning handling
	}
	// DRX_TUNE1b
	if(dev->preambleLength ==  TX_PREAMBLE_LEN_1536 || dev->preambleLength ==  TX_PREAMBLE_LEN_2048 ||
			dev->preambleLength ==  TX_PREAMBLE_LEN_4096) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(drxtune1b, 0x0064, LEN_DRX_TUNE1b);
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->preambleLength != TX_PREAMBLE_LEN_64) {
		if(dev->dataRate == TRX_RATE_850KBPS || dev->dataRate == TRX_RATE_6800KBPS) {
			writeValueToBytes(drxtune1b, 0x0020, LEN_DRX_TUNE1b);
		} else {
			// TODO proper error/warning handling
		}
	} else {
		if(dev->dataRate == TRX_RATE_6800KBPS) {
			writeValueToBytes(drxtune1b, 0x0010, LEN_DRX_TUNE1b);
		} else {
			// TODO proper error/warning handling
		}
	}
	// DRX_TUNE2
	if(dev->pacSize == PAC_SIZE_8) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x311A002DL, LEN_DRX_TUNE2);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x313B006BL, LEN_DRX_TUNE2);
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->pacSize == PAC_SIZE_16) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x331A0052L, LEN_DRX_TUNE2);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x333B00BEL, LEN_DRX_TUNE2);
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->pacSize == PAC_SIZE_32) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x351A009AL, LEN_DRX_TUNE2);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x353B015EL, LEN_DRX_TUNE2);
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->pacSize == PAC_SIZE_64) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x371A011DL, LEN_DRX_TUNE2);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x373B0296L, LEN_DRX_TUNE2);
		} else {
			// TODO proper error/warning handling
		}
	} else {
		// TODO proper error/warning handling
	}
	// DRX_TUNE4H
	if(dev->preambleLength == TX_PREAMBLE_LEN_64) {
		writeValueToBytes(drxtune4H, 0x0010, LEN_DRX_TUNE4H);
	} else {
		writeValueToBytes(drxtune4H, 0x0028, LEN_DRX_TUNE4H);
	}
	// RF_RXCTRLH
	if(dev->channel != CHANNEL_4 && dev->channel != CHANNEL_7) {
		writeValueToBytes(rfrxctrlh, 0xD8, LEN_RF_RXCTRLH);
	} else {
		writeValueToBytes(rfrxctrlh, 0xBC, LEN_RF_RXCTRLH);
	}
	// RX_TXCTRL
	if(dev->channel == CHANNEL_1) {
		writeValueToBytes(rftxctrl, 0x00005C40L, LEN_RF_TXCTRL);
	} else if(dev->channel == CHANNEL_2) {
		writeValueToBytes(rftxctrl, 0x00045CA0L, LEN_RF_TXCTRL);
	} else if(dev->channel == CHANNEL_3) {
		writeValueToBytes(rftxctrl, 0x00086CC0L, LEN_RF_TXCTRL);
	} else if(dev->channel == CHANNEL_4) {
		writeValueToBytes(rftxctrl, 0x00045C80L, LEN_RF_TXCTRL);
	} else if(dev->channel == CHANNEL_5) {
		writeValueToBytes(rftxctrl, 0x001E3FE0L, LEN_RF_TXCTRL);
	} else if(dev->channel == CHANNEL_7) {
		writeValueToBytes(rftxctrl, 0x001E7DE0L, LEN_RF_TXCTRL);
	} else {
		// TODO proper error/warning handling
	}
	// TC_PGDELAY
	if(dev->channel == CHANNEL_1) {
		writeValueToBytes(tcpgdelay, 0xC9, LEN_TC_PGDELAY);
	} else if(dev->channel == CHANNEL_2) {
		writeValueToBytes(tcpgdelay, 0xC2, LEN_TC_PGDELAY);
	} else if(dev->channel == CHANNEL_3) {
		writeValueToBytes(tcpgdelay, 0xC5, LEN_TC_PGDELAY);
	} else if(dev->channel == CHANNEL_4) {
		writeValueToBytes(tcpgdelay, 0x95, LEN_TC_PGDELAY);
	} else if(dev->channel == CHANNEL_5) {
		writeValueToBytes(tcpgdelay, 0xC0, LEN_TC_PGDELAY);
	} else if(dev->channel == CHANNEL_7) {
		writeValueToBytes(tcpgdelay, 0x93, LEN_TC_PGDELAY);
	} else {
		// TODO proper error/warning handling
	}
	// FS_PLLCFG and FS_PLLTUNE
	if(dev->channel == CHANNEL_1) {
		writeValueToBytes(fspllcfg, 0x09000407L, LEN_FS_PLLCFG);
		writeValueToBytes(fsplltune, 0x1E, LEN_FS_PLLTUNE);
	} else if(dev->channel == CHANNEL_2 || dev->channel == CHANNEL_4) {
		writeValueToBytes(fspllcfg, 0x08400508L, LEN_FS_PLLCFG);
		writeValueToBytes(fsplltune, 0x26, LEN_FS_PLLTUNE);
	} else if(dev->channel == CHANNEL_3) {
		writeValueToBytes(fspllcfg, 0x08401009L, LEN_FS_PLLCFG);
		writeValueToBytes(fsplltune, 0x56, LEN_FS_PLLTUNE);
	} else if(dev->channel == CHANNEL_5 || dev->channel == CHANNEL_7) {
		writeValueToBytes(fspllcfg, 0x0800041DL, LEN_FS_PLLCFG);
		writeValueToBytes(fsplltune, 0xA6, LEN_FS_PLLTUNE);
	} else {
		// TODO proper error/warning handling
	}
	// LDE_CFG1
	writeValueToBytes(ldecfg1, 0xD, LEN_LDE_CFG1);
	// LDE_CFG2
	if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		writeValueToBytes(ldecfg2, 0x1607, LEN_LDE_CFG2);
	} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		writeValueToBytes(ldecfg2, 0x0607, LEN_LDE_CFG2);
	} else {
		// TODO proper error/warning handling
	}
	// LDE_REPC
	if(dev->preambleCode == PREAMBLE_CODE_16MHZ_1 || dev->preambleCode == PREAMBLE_CODE_16MHZ_2) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x5998 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x5998, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_3 || dev->preambleCode == PREAMBLE_CODE_16MHZ_8) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x51EA >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x51EA, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_4) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x428E >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x428E, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_5) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x451E >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x451E, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_6) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x2E14 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x2E14, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_7) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x8000 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x8000, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_9) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x28F4 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x28F4, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_10 || dev->preambleCode == PREAMBLE_CODE_64MHZ_17) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x3332 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x3332, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_11) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x3AE0 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x3AE0, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_12) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x3D70 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x3D70, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_18 || dev->preambleCode == PREAMBLE_CODE_64MHZ_19) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x35C2 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x35C2, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_20) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x47AE >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x47AE, LEN_LDE_REPC);
		}
	} else {
		// TODO proper error/warning handling
	}
	// TX_POWER (enabled smart transmit power control)
	if(dev->forceTxPower) {
		writeValueToBytes(txpower, dev->txPower, LEN_TX_POWER);
	} else if(dev->channel == CHANNEL_1 || dev->channel == CHANNEL_2) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x15355575L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x75757575L, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x07274767L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x67676767L, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_3) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x0F2F4F6FL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x6F6F6F6FL, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x2B4B6B8BL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x8B8B8B8BL, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_4) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x1F1F3F5FL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x5F5F5F5FL, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x3A5A7A9AL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x9A9A9A9AL, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_5) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x0E082848L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x48484848L, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x25456585L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x85858585L, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_7) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x32527292L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x92929292L, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x5171B1D1L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0xD1D1D1D1L, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else {
		// TODO proper error/warning handling
	}
	// Crystal calibration from OTP (if available)
	uint8_t buf_otp[4];
	readBytesOTP(dev, 0x01E, buf_otp);
	if (buf_otp[0] == 0) {
		// No trim value available from OTP, use midrange value of 0x10
		writeValueToBytes(fsxtalt, ((0x10 & 0x1F) | 0x60), LEN_FS_XTALT);
	} else {
		writeValueToBytes(fsxtalt, ((buf_otp[0] & 0x1F) | 0x60), LEN_FS_XTALT);
	}
	// write configuration back to chip
	dwSpiWrite(dev, AGC_TUNE, AGC_TUNE1_SUB, agctune1, LEN_AGC_TUNE1);
	dwSpiWrite(dev, AGC_TUNE, AGC_TUNE2_SUB, agctune2, LEN_AGC_TUNE2);
	dwSpiWrite(dev, AGC_TUNE, AGC_TUNE3_SUB, agctune3, LEN_AGC_TUNE3);
	dwSpiWrite(dev, DRX_TUNE, DRX_TUNE0b_SUB, drxtune0b, LEN_DRX_TUNE0b);
	dwSpiWrite(dev, DRX_TUNE, DRX_TUNE1a_SUB, drxtune1a, LEN_DRX_TUNE1a);
	dwSpiWrite(dev, DRX_TUNE, DRX_TUNE1b_SUB, drxtune1b, LEN_DRX_TUNE1b);
	dwSpiWrite(dev, DRX_TUNE, DRX_TUNE2_SUB, drxtune2, LEN_DRX_TUNE2);
	dwSpiWrite(dev, DRX_TUNE, DRX_TUNE4H_SUB, drxtune4H, LEN_DRX_TUNE4H);
	dwSpiWrite(dev, LDE_IF, LDE_CFG1_SUB, ldecfg1, LEN_LDE_CFG1);
	dwSpiWrite(dev, LDE_IF, LDE_CFG2_SUB, ldecfg2, LEN_LDE_CFG2);
	dwSpiWrite(dev, LDE_IF, LDE_REPC_SUB, lderepc, LEN_LDE_REPC);
	dwSpiWrite(dev, TX_POWER, txpower, LEN_TX_POWER);
	dwSpiWrite(dev, RF_CONF, RF_RXCTRLH_SUB, rfrxctrlh, LEN_RF_RXCTRLH);
	dwSpiWrite(dev, RF_CONF, RF_TXCTRL_SUB, rftxctrl, LEN_RF_TXCTRL);
	dwSpiWrite(dev, TX_CAL, TC_PGDELAY_SUB, tcpgdelay, LEN_TC_PGDELAY);
	dwSpiWrite(dev, FS_CTRL, FS_PLLTUNE_SUB, fsplltune, LEN_FS_PLLTUNE);
	dwSpiWrite(dev, FS_CTRL, FS_PLLCFG_SUB, fspllcfg, LEN_FS_PLLCFG);
	dwSpiWrite(dev, FS_CTRL, FS_XTALT_SUB, fsxtalt, LEN_FS_XTALT);
}
*/


/**
 Reset the receiver. Needed after errors or timeouts.
 From the DW1000 User Manual, v2.13 page 35:
 "Due to an issue in the re-initialisation of the receiver, it is necessary to apply
 a receiver reset after certain receiver error or timeout events (i.e. RXPHE (PHY Header Error),
 RXRFSL (Reed Solomon error), RXRFTO (Frame wait timeout), etc.).
 This ensures that the next good frame will have correctly calculated timestamp.
 It is not necessary to do this in the cases of RXPTO (Preamble detection Timeout)
 and RXSFDTO (SFD timeout). For details on how to apply a receiver-only reset see SOFTRESET
 field of Sub- Register 0x36:00 – PMSC_CTRL0."
 */
void AP_DwmDriverBackend::dwRxSoftReset() {
    uint8_t pmscctrl0[LEN_PMSC_CTRL0];
    dwRead(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);

    pmscctrl0[3] = pmscctrl0[3] & 0xEF;
    dwWrite(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    pmscctrl0[3] = pmscctrl0[3] | 0x10;
    dwWrite(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
}

bool AP_DwmDriverBackend::dwIsReceiveFailed() {
    bool ldeErr = getBit(dwm->sysstatus, LEN_SYS_STATUS, LDEERR_BIT);
    bool rxCRCErr = getBit(dwm->sysstatus, LEN_SYS_STATUS, RXFCE_BIT);
    bool rxHeaderErr = getBit(dwm->sysstatus, LEN_SYS_STATUS, RXPHE_BIT);
    bool rxDecodeErr = getBit(dwm->sysstatus, LEN_SYS_STATUS, RXRFSL_BIT);

    bool rxSfdto = getBit(dwm->sysstatus, LEN_SYS_STATUS, RXSFDTO_BIT);
    bool affrej = getBit(dwm->sysstatus, LEN_SYS_STATUS, AFFREJ_BIT);

    return (ldeErr || rxCRCErr || rxHeaderErr || rxDecodeErr || rxSfdto || affrej);
}

bool AP_DwmDriverBackend::dwIsReceiveTimeout() {
    return getBit(dwm->sysstatus, LEN_SYS_STATUS, RXRFTO_BIT);
}


//K
void AP_DwmDriverBackend::dwClearInterrupts() {
    memset(dwm->sysmask, 0, LEN_SYS_MASK);
}

//K, but clock TODO
void AP_DwmDriverBackend::dwManageLDE() {
    // transfer any ldo tune values
    // tell the chip to load the LDE microcode
    // TODO remove clock-related code (PMSC_CTRL) as handled separately
    uint8_t pmscctrl0[LEN_PMSC_CTRL0]; //clock
    uint8_t otpctrl[LEN_OTP_CTRL];
    memset(pmscctrl0, 0, LEN_PMSC_CTRL0); //clock
    memset(otpctrl, 0, LEN_OTP_CTRL);
    dwRead(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0); //clock
    dwRead(OTP_IF, OTP_CTRL_SUB, otpctrl, LEN_OTP_CTRL);
    pmscctrl0[0] = 0x01; //clock
    pmscctrl0[1] = 0x03; //clock
    otpctrl[0] = 0x00;
    otpctrl[1] = 0x80;
    dwWrite(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0); //clock
    dwWrite(OTP_IF, OTP_CTRL_SUB, otpctrl, LEN_OTP_CTRL);
    delayms(5);
    pmscctrl0[0] = 0x00; //clock
    pmscctrl0[1] = 0x02; //clock
    dwWrite(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0); //clock
}

/*void AP_DwmDriverBackend::newReceive() {
    // IDLE
    memset(dwm->sysctrl, 0, LEN_SYS_CTRL);
    dwm->sysctrl[0] |= 1 << TRXOFF_BIT;
    dwm->deviceMode = IDLE_MODE;
    dwWrite(SYS_CTRL, dwm->sysctrl, LEN_SYS_CTRL);

    memset(dwm->sysctrl, 0, LEN_SYS_CTRL);

    dwClearReceiveStatus();

    dwm->deviceMode = RX_MODE;

    // Start Receive
    dwStartReceive();
}
*/

void AP_DwmDriverBackend::dwClearReceiveStatus() {
    // clear latched RX bits (i.e. write 1 to clear)
    //uint32_t regData = SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_GOOD;
    //dwWrite32(SYS_STATUS, regData);

    uint8_t reg[LEN_SYS_STATUS] = {0};
    setBit(reg, LEN_SYS_STATUS, RXDFR_BIT, true);
    setBit(reg, LEN_SYS_STATUS, LDEDONE_BIT, true);
    setBit(reg, LEN_SYS_STATUS, LDEERR_BIT, true);
    setBit(reg, LEN_SYS_STATUS, RXPHE_BIT, true);
    setBit(reg, LEN_SYS_STATUS, RXFCE_BIT, true);
    setBit(reg, LEN_SYS_STATUS, RXFCG_BIT, true);
    setBit(reg, LEN_SYS_STATUS, RXRFSL_BIT, true);
    setBit(reg, LEN_SYS_STATUS, RXRFTO_BIT, true);

    setBit(reg, LEN_SYS_STATUS, RXSFDTO_BIT, true);
    setBit(reg, LEN_SYS_STATUS, AFFREJ_BIT, true);
    setBit(reg, LEN_SYS_STATUS, RXPTO_BIT, true);
    setBit(reg, LEN_SYS_STATUS, RXPRD_BIT, true);
    setBit(reg, LEN_SYS_STATUS, MRXSFDD_BIT, true);
    setBit(reg, LEN_SYS_STATUS, MRXPHD_BIT, true);
    dwWrite(SYS_STATUS, reg, LEN_SYS_STATUS);
}

//TODO refactor using correct functions
/*void AP_DwmDriverBackend::getTime(dwTime_t *time) {
    // "dwGetReceivePower()"
    // base line dBm, which is -61, 2 dBm steps, total 18 data points (down to -95 dBm)
    double C, N, twoPower17 = 131072.0;
    dwRead(RX_FQUAL, CIR_PWR_SUB, (uint8_t *)&C, LEN_CIR_PWR);

    // "spiReadRxInfo()"
    uint8_t rxFrameInfo[LEN_RX_FINFO];
    spidev->read_registers(RX_FINFO, rxFrameInfo, LEN_RX_FINFO);
    N = (double)((((unsigned int)rxFrameInfo[2] >> 4) & 0xFF) | ((unsigned int)rxFrameInfo[3] << 4));

    // "calculatePower()"
    double A, corrFac;

    if (TX_PULSE_FREQ_16MHZ == dwm->pulseFrequency) {
        A = 115.72;
        corrFac = 2.3334;
    } else {
        A = 121.74;
        corrFac = 1.1667;
    }

    double estFpPwr = 10.0 * log10f((C * twoPower17) / (N * N)) - A;

    if (estFpPwr <= -88) {
        //return estFpPwr;
    } else {
        // approximation of Fig. 22 in user manual for dbm correction
        estFpPwr += (estFpPwr + 88) * corrFac;
    }

    // "dwCorrectTimestamp()"
    double rxPowerBase = -(estFpPwr + 61.0) * 0.5;
    if (!isfinite(rxPowerBase)) {
        return;
    }
    int rxPowerBaseLow = (int)rxPowerBase;
    int rxPowerBaseHigh = rxPowerBaseLow + 1;
    if (rxPowerBaseLow < 0) {
        rxPowerBaseLow = 0;
        rxPowerBaseHigh = 0;
    } else if (rxPowerBaseHigh > 17) {
        rxPowerBaseLow = 17;
        rxPowerBaseHigh = 17;
    }
    // select range low/high values from corresponding table
    int rangeBiasHigh = 0;
    int rangeBiasLow = 0;
    if (dwm->channel == CHANNEL_4 || dwm->channel == CHANNEL_7) {
        // 900 MHz receiver bandwidth
        if (dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            rangeBiasHigh = (rxPowerBaseHigh < BIAS_900_16_ZERO ? -BIAS_900_16[rxPowerBaseHigh] : BIAS_900_16[rxPowerBaseHigh]);
            rangeBiasHigh <<= 1;
            rangeBiasLow = (rxPowerBaseLow < BIAS_900_16_ZERO ? -BIAS_900_16[rxPowerBaseLow] : BIAS_900_16[rxPowerBaseLow]);
            rangeBiasLow <<= 1;
        } else if (dwm->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            rangeBiasHigh = (rxPowerBaseHigh < BIAS_900_64_ZERO ? -BIAS_900_64[rxPowerBaseHigh] : BIAS_900_64[rxPowerBaseHigh]);
            rangeBiasHigh <<= 1;
            rangeBiasLow = (rxPowerBaseLow < BIAS_900_64_ZERO ? -BIAS_900_64[rxPowerBaseLow] : BIAS_900_64[rxPowerBaseLow]);
            rangeBiasLow <<= 1;
        } else {
            // TODO proper error handling
        }
    } else {
        // 500 MHz receiver bandwidth
        if (dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            rangeBiasHigh = (rxPowerBaseHigh < BIAS_500_16_ZERO ? -BIAS_500_16[rxPowerBaseHigh] : BIAS_500_16[rxPowerBaseHigh]);
            rangeBiasLow = (rxPowerBaseLow < BIAS_500_16_ZERO ? -BIAS_500_16[rxPowerBaseLow] : BIAS_500_16[rxPowerBaseLow]);
        } else if (dwm->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            rangeBiasHigh = (rxPowerBaseHigh < BIAS_500_64_ZERO ? -BIAS_500_64[rxPowerBaseHigh] : BIAS_500_64[rxPowerBaseHigh]);
            rangeBiasLow = (rxPowerBaseLow < BIAS_500_64_ZERO ? -BIAS_500_64[rxPowerBaseLow] : BIAS_500_64[rxPowerBaseLow]);
        } else {
            // TODO proper error handling
        }
    }
    // linear interpolation of bias values
    double rangeBias = rangeBiasLow + (rxPowerBase - rxPowerBaseLow) * (rangeBiasHigh - rangeBiasLow);
    // range bias [mm] to timestamp modification value conversion
    dwTime_t adjustmentTime;
    adjustmentTime.full = rangeBias * DISTANCE_OF_RADIO_INV * 0.001;  //(int)
    // apply correction
    time->full += adjustmentTime.full;
}*/

//K
void AP_DwmDriverBackend::dwSoftReset() {	
    uint8_t pmscctrl0[LEN_PMSC_CTRL0];
    dwRead(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    pmscctrl0[0] = 0x01;
    dwWrite(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    pmscctrl0[3] = 0x00;
    dwWrite(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    delayms(10);
    pmscctrl0[0] = 0x00;
    pmscctrl0[3] = 0xF0;
    dwWrite(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    // force into idle mode
    dwIdle();
}

void AP_DwmDriverBackend::dwIdle() {
    memset(dwm->sysctrl, 0, LEN_SYS_CTRL);
    dwm->sysctrl[0] |= 1 << TRXOFF_BIT;
    dwm->deviceMode = IDLE_MODE;
    dwWrite(SYS_CTRL, dwm->sysctrl, LEN_SYS_CTRL);
}

//K
void AP_DwmDriverBackend::dwEnableAllLeds() {
    uint32_t reg;

    // Set all 4 GPIO in LED mode
    dwRead(GPIO_CTRL, GPIO_MODE_SUB, (uint8_t *)&reg, LEN_GPIO_MODE);
    reg &= ~0x00003FC0ul;
    reg |= 0x00001540ul;
    dwWrite(GPIO_CTRL, GPIO_MODE_SUB, (uint8_t *)&reg, LEN_GPIO_MODE);

    // Enable debounce clock (used to clock the LED blinking)
    dwRead(PMSC, PMSC_CTRL0_SUB, (uint8_t *)&reg, LEN_PMSC_CTRL0);
    reg |= 0x00840000ul;
    dwWrite(PMSC, PMSC_CTRL0_SUB, (uint8_t *)&reg, LEN_PMSC_CTRL0);

    // Enable LED blinking and set the rate
    reg = 0x00000110ul;
    dwWrite(PMSC, PMSC_LEDC, (uint8_t *)&reg, LEN_PMSC_LEDC);

    // Trigger a manual blink of the LEDs for test
    reg |= 0x000f0000ul;
    dwWrite(PMSC, PMSC_LEDC, (uint8_t *)&reg, LEN_PMSC_LEDC);
    reg &= ~0x000f0000ul;
    dwWrite(PMSC, PMSC_LEDC, (uint8_t *)&reg, LEN_PMSC_LEDC);
}

void AP_DwmDriverBackend::newConfig() {
    dwm->antennaDelay.full = 0;
    spidev->read_registers(PANADR, dwm->networkAndAddress, LEN_PANADR);
    spidev->read_registers(SYS_CFG, dwm->syscfg, LEN_SYS_CFG);
    spidev->read_registers(CHAN_CTRL, dwm->chanctrl, LEN_CHAN_CTRL);
    spidev->read_registers(SYS_MASK, dwm->sysmask, LEN_SYS_MASK);

    //  SET DATA RATE
    uint8_t rate = MODE_SHORTDATA_FAST_ACCURACY[0];
    rate &= 0x03;
    dwm->txfctrl[1] &= 0x83;
    dwm->txfctrl[1] |= (uint8_t)((rate << 5) & 0xFF);
    if (rate == TRX_RATE_110KBPS) {
        setBit(dwm->syscfg, LEN_SYS_CFG, RXM110K_BIT, true);
    } else {
        setBit(dwm->syscfg, LEN_SYS_CFG, RXM110K_BIT, false);
    }
    // SFD mode and type (non-configurable, as in Table )
    if (rate == TRX_RATE_6800KBPS) {
        setBit(dwm->chanctrl, LEN_CHAN_CTRL, DWSFD_BIT, false);
        setBit(dwm->chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, false);
        setBit(dwm->chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, false);
    } else {
        setBit(dwm->chanctrl, LEN_CHAN_CTRL, DWSFD_BIT, true);
        setBit(dwm->chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, true);
        setBit(dwm->chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, true);
    }
    uint8_t sfdLength;
    if (rate == TRX_RATE_6800KBPS) {
        sfdLength = 0x08;
    } else if (rate == TRX_RATE_850KBPS) {
        sfdLength = 0x10;
    } else {
        sfdLength = 0x40;
    }
    dwWrite(USR_SFD, SFD_LENGTH_SUB, (uint8_t *)&sfdLength, LEN_SFD_LENGTH);
    dwm->dataRate = rate;

    //  SET PULSE FREQUENCY
    uint8_t freq = MODE_SHORTDATA_FAST_ACCURACY[1];
    freq &= 0x03;
    dwm->txfctrl[2] &= 0xFC;
    dwm->txfctrl[2] |= (uint8_t)(freq & 0xFF);
    dwm->chanctrl[2] &= 0xF3;
    dwm->chanctrl[2] |= (uint8_t)((freq << 2) & 0xFF);
    dwm->pulseFrequency = freq;

    //  SET PREAMBLE LENGTH
    uint8_t prealen = MODE_SHORTDATA_FAST_ACCURACY[2];
    prealen &= 0x0F;
    dwm->txfctrl[2] &= 0xC3;
    dwm->txfctrl[2] |= (uint8_t)((prealen << 2) & 0xFF);
    if (prealen == TX_PREAMBLE_LEN_64 || prealen == TX_PREAMBLE_LEN_128) {
        dwm->pacSize = PAC_SIZE_8;
    } else if (prealen == TX_PREAMBLE_LEN_256 || prealen == TX_PREAMBLE_LEN_512) {
        dwm->pacSize = PAC_SIZE_16;
    } else if (prealen == TX_PREAMBLE_LEN_1024) {
        dwm->pacSize = PAC_SIZE_32;
    } else {
        dwm->pacSize = PAC_SIZE_64;
    }
    dwm->preambleLength = prealen;

    //  SET CHANNEL
    uint8_t channel = CHANNEL_2;
    channel &= 0xF;
    dwm->chanctrl[0] = ((channel | (channel << 4)) & 0xFF);
    dwm->channel = channel;

    // SET PREAMBLE CODE
    uint8_t preacode = PREAMBLE_CODE_64MHZ_9;
    preacode &= 0x1F;
    dwm->chanctrl[2] &= 0x3F;
    dwm->chanctrl[2] |= ((preacode << 6) & 0xFF);
    dwm->chanctrl[3] = 0x00;
    dwm->chanctrl[3] = ((((preacode >> 2) & 0x07) | (preacode << 3)) & 0xFF);
    dwm->preambleCode = preacode;

    //  USE SMART POWER
    bool smartPower = true;
    dwm->smartPower = smartPower;
    setBit(dwm->syscfg, LEN_SYS_CFG, DIS_STXP_BIT, !smartPower);

    //  SET RX TIMEOUT
    uint16_t timeout = RX_TIMEOUT;
    dwWrite(RX_FWTO, (uint8_t *)&timeout, 2);
    setBit(dwm->syscfg, LEN_SYS_CFG, RXWTOE_BIT, timeout != 0);
    setBit(dwm->syscfg, LEN_SYS_CFG, RXAUTR_BIT, true);

    if(DWMDRIVER_DEBUG) hal.console->printf("\n newConfig finished \n");
}


// subs dwTune  TODO remove  this is the old one
void AP_DwmDriverBackend::dwTune() {
    // these registers are going to be tuned/configured
    uint8_t agctune1[LEN_AGC_TUNE1] = {0};
    uint8_t agctune2[LEN_AGC_TUNE2] = {0};
    uint8_t agctune3[LEN_AGC_TUNE3] = {0};
    uint8_t drxtune0b[LEN_DRX_TUNE0b] = {0};
    uint8_t drxtune1a[LEN_DRX_TUNE1a] = {0};
    uint8_t drxtune1b[LEN_DRX_TUNE1b] = {0};
    uint8_t drxtune2[LEN_DRX_TUNE2] = {0};
    uint8_t drxtune4H[LEN_DRX_TUNE4H] = {0};
    uint8_t ldecfg1[LEN_LDE_CFG1] = {0};
    uint8_t ldecfg2[LEN_LDE_CFG2] = {0};
    uint8_t lderepc[LEN_LDE_REPC] = {0};
    uint8_t txpower[LEN_TX_POWER] = {0};
    uint8_t rfrxctrlh[LEN_RF_RXCTRLH] = {0};
    uint8_t rftxctrl[LEN_RF_TXCTRL] = {0};
    uint8_t tcpgdelay[LEN_TC_PGDELAY] = {0};
    uint8_t fspllcfg[LEN_FS_PLLCFG] = {0};
    uint8_t fsplltune[LEN_FS_PLLTUNE] = {0};
    uint8_t fsxtalt[LEN_FS_XTALT] = {0};

    // AGC_TUNE1
    if (dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
        writeValueToBytes(agctune1, 0x8870, LEN_AGC_TUNE1);
    } else if (dwm->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
        writeValueToBytes(agctune1, 0x889B, LEN_AGC_TUNE1);
    } else {
        // TODO proper error/warning handling
    }

    // AGC_TUNE2
    writeValueToBytes(agctune2, 0x2502A907L, LEN_AGC_TUNE2);
    // AGC_TUNE3
    writeValueToBytes(agctune3, 0x0035, LEN_AGC_TUNE3);
    // DRX_TUNE0b (already optimized according to Table 20 of user manual)
    if (dwm->dataRate == TRX_RATE_110KBPS) {
        writeValueToBytes(drxtune0b, 0x0016, LEN_DRX_TUNE0b);
    } else if (dwm->dataRate == TRX_RATE_850KBPS) {
        writeValueToBytes(drxtune0b, 0x0006, LEN_DRX_TUNE0b);
    } else if (dwm->dataRate == TRX_RATE_6800KBPS) {
        writeValueToBytes(drxtune0b, 0x0001, LEN_DRX_TUNE0b);
    } else {
        // TODO proper error/warning handling
    }
    // DRX_TUNE1a
    if (dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
        writeValueToBytes(drxtune1a, 0x0087, LEN_DRX_TUNE1a);
    } else if (dwm->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
        writeValueToBytes(drxtune1a, 0x008D, LEN_DRX_TUNE1a);
    } else {
        // TODO proper error/warning handling
    }
    // DRX_TUNE1b
    if (dwm->preambleLength == TX_PREAMBLE_LEN_1536 || dwm->preambleLength == TX_PREAMBLE_LEN_2048 ||
        dwm->preambleLength == TX_PREAMBLE_LEN_4096) {
        if (dwm->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(drxtune1b, 0x0064, LEN_DRX_TUNE1b);
        } else {
            // TODO proper error/warning handling
        }
    } else if (dwm->preambleLength != TX_PREAMBLE_LEN_64) {
        if (dwm->dataRate == TRX_RATE_850KBPS || dwm->dataRate == TRX_RATE_6800KBPS) {
            writeValueToBytes(drxtune1b, 0x0020, LEN_DRX_TUNE1b);
        } else {
            // TODO proper error/warning handling
        }
    } else {
        if (dwm->dataRate == TRX_RATE_6800KBPS) {
            writeValueToBytes(drxtune1b, 0x0010, LEN_DRX_TUNE1b);
        } else {
            // TODO proper error/warning handling
        }
    }
    // DRX_TUNE2
    if (dwm->pacSize == PAC_SIZE_8) {
        if (dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            writeValueToBytes(drxtune2, 0x311A002DL, LEN_DRX_TUNE2);
        } else if (dwm->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            writeValueToBytes(drxtune2, 0x313B006BL, LEN_DRX_TUNE2);
        } else {
            // TODO proper error/warning handling
        }
    } else if (dwm->pacSize == PAC_SIZE_16) {
        if (dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            writeValueToBytes(drxtune2, 0x331A0052L, LEN_DRX_TUNE2);
        } else if (dwm->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            writeValueToBytes(drxtune2, 0x333B00BEL, LEN_DRX_TUNE2);
        } else {
            // TODO proper error/warning handling
        }
    } else if (dwm->pacSize == PAC_SIZE_32) {
        if (dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            writeValueToBytes(drxtune2, 0x351A009AL, LEN_DRX_TUNE2);
        } else if (dwm->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            writeValueToBytes(drxtune2, 0x353B015EL, LEN_DRX_TUNE2);
        } else {
            // TODO proper error/warning handling
        }
    } else if (dwm->pacSize == PAC_SIZE_64) {
        if (dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            writeValueToBytes(drxtune2, 0x371A011DL, LEN_DRX_TUNE2);
        } else if (dwm->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            writeValueToBytes(drxtune2, 0x373B0296L, LEN_DRX_TUNE2);
        } else {
            // TODO proper error/warning handling
        }
    } else {
        // TODO proper error/warning handling
    }
    // DRX_TUNE4H
    if (dwm->preambleLength == TX_PREAMBLE_LEN_64) {
        writeValueToBytes(drxtune4H, 0x0010, LEN_DRX_TUNE4H);
    } else {
        writeValueToBytes(drxtune4H, 0x0028, LEN_DRX_TUNE4H);
    }
    // RF_RXCTRLH
    if (dwm->channel != CHANNEL_4 && dwm->channel != CHANNEL_7) {
        writeValueToBytes(rfrxctrlh, 0xD8, LEN_RF_RXCTRLH);
    } else {
        writeValueToBytes(rfrxctrlh, 0xBC, LEN_RF_RXCTRLH);
    }
    // RX_TXCTRL
    if (dwm->channel == CHANNEL_1) {
        writeValueToBytes(rftxctrl, 0x00005C40L, LEN_RF_TXCTRL);
    } else if (dwm->channel == CHANNEL_2) {
        writeValueToBytes(rftxctrl, 0x00045CA0L, LEN_RF_TXCTRL);
    } else if (dwm->channel == CHANNEL_3) {
        writeValueToBytes(rftxctrl, 0x00086CC0L, LEN_RF_TXCTRL);
    } else if (dwm->channel == CHANNEL_4) {
        writeValueToBytes(rftxctrl, 0x00045C80L, LEN_RF_TXCTRL);
    } else if (dwm->channel == CHANNEL_5) {
        writeValueToBytes(rftxctrl, 0x001E3FE0L, LEN_RF_TXCTRL);
    } else if (dwm->channel == CHANNEL_7) {
        writeValueToBytes(rftxctrl, 0x001E7DE0L, LEN_RF_TXCTRL);
    } else {
        // TODO proper error/warning handling
    }
    // TC_PGDELAY
    if (dwm->channel == CHANNEL_1) {
        writeValueToBytes(tcpgdelay, 0xC9, LEN_TC_PGDELAY);
    } else if (dwm->channel == CHANNEL_2) {
        writeValueToBytes(tcpgdelay, 0xC2, LEN_TC_PGDELAY);
    } else if (dwm->channel == CHANNEL_3) {
        writeValueToBytes(tcpgdelay, 0xC5, LEN_TC_PGDELAY);
    } else if (dwm->channel == CHANNEL_4) {
        writeValueToBytes(tcpgdelay, 0x95, LEN_TC_PGDELAY);
    } else if (dwm->channel == CHANNEL_5) {
        writeValueToBytes(tcpgdelay, 0xC0, LEN_TC_PGDELAY);
    } else if (dwm->channel == CHANNEL_7) {
        writeValueToBytes(tcpgdelay, 0x93, LEN_TC_PGDELAY);
    } else {
        // TODO proper error/warning handling
    }
    // FS_PLLCFG and FS_PLLTUNE
    if (dwm->channel == CHANNEL_1) {
        writeValueToBytes(fspllcfg, 0x09000407L, LEN_FS_PLLCFG);
        writeValueToBytes(fsplltune, 0x1E, LEN_FS_PLLTUNE);
    } else if (dwm->channel == CHANNEL_2 || dwm->channel == CHANNEL_4) {
        writeValueToBytes(fspllcfg, 0x08400508L, LEN_FS_PLLCFG);
        writeValueToBytes(fsplltune, 0x26, LEN_FS_PLLTUNE);
    } else if (dwm->channel == CHANNEL_3) {
        writeValueToBytes(fspllcfg, 0x08401009L, LEN_FS_PLLCFG);
        writeValueToBytes(fsplltune, 0x5E, LEN_FS_PLLTUNE);
    } else if (dwm->channel == CHANNEL_5 || dwm->channel == CHANNEL_7) {
        writeValueToBytes(fspllcfg, 0x0800041DL, LEN_FS_PLLCFG);
        writeValueToBytes(fsplltune, 0xA6, LEN_FS_PLLTUNE);
    } else {
        // TODO proper error/warning handling
    }
    // LDE_CFG1
    writeValueToBytes(ldecfg1, 0xD, LEN_LDE_CFG1);
    // LDE_CFG2
    if (dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
        writeValueToBytes(ldecfg2, 0x1607, LEN_LDE_CFG2);
    } else if (dwm->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
        writeValueToBytes(ldecfg2, 0x0607, LEN_LDE_CFG2);
    } else {
        // TODO proper error/warning handling
    }
    // LDE_REPC
    if (dwm->preambleCode == PREAMBLE_CODE_16MHZ_1 || dwm->preambleCode == PREAMBLE_CODE_16MHZ_2) {
        if (dwm->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x5998 >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x5998, LEN_LDE_REPC);
        }
    } else if (dwm->preambleCode == PREAMBLE_CODE_16MHZ_3 || dwm->preambleCode == PREAMBLE_CODE_16MHZ_8) {
        if (dwm->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x51EA >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x51EA, LEN_LDE_REPC);
        }
    } else if (dwm->preambleCode == PREAMBLE_CODE_16MHZ_4) {
        if (dwm->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x428E >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x428E, LEN_LDE_REPC);
        }
    } else if (dwm->preambleCode == PREAMBLE_CODE_16MHZ_5) {
        if (dwm->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x451E >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x451E, LEN_LDE_REPC);
        }
    } else if (dwm->preambleCode == PREAMBLE_CODE_16MHZ_6) {
        if (dwm->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x2E14 >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x2E14, LEN_LDE_REPC);
        }
    } else if (dwm->preambleCode == PREAMBLE_CODE_16MHZ_7) {
        if (dwm->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x8000 >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x8000, LEN_LDE_REPC);
        }
    } else if (dwm->preambleCode == PREAMBLE_CODE_64MHZ_9) {
        if (dwm->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x28F4 >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x28F4, LEN_LDE_REPC);
        }
    } else if (dwm->preambleCode == PREAMBLE_CODE_64MHZ_10 || dwm->preambleCode == PREAMBLE_CODE_64MHZ_17) {
        if (dwm->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x3332 >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x3332, LEN_LDE_REPC);
        }
    } else if (dwm->preambleCode == PREAMBLE_CODE_64MHZ_11) {
        if (dwm->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x3AE0 >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x3AE0, LEN_LDE_REPC);
        }
    } else if (dwm->preambleCode == PREAMBLE_CODE_64MHZ_12) {
        if (dwm->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x3D70 >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x3D70, LEN_LDE_REPC);
        }
    } else if (dwm->preambleCode == PREAMBLE_CODE_64MHZ_18 || dwm->preambleCode == PREAMBLE_CODE_64MHZ_19) {
        if (dwm->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x35C2 >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x35C2, LEN_LDE_REPC);
        }
    } else if (dwm->preambleCode == PREAMBLE_CODE_64MHZ_20) {
        if (dwm->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x47AE >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x47AE, LEN_LDE_REPC);
        }
    } else {
        // TODO proper error/warning handling
    }
    // TX_POWER (enabled smart transmit power control)
    if (dwm->channel == CHANNEL_1 || dwm->channel == CHANNEL_2) {
        if (dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            if (dwm->smartPower) {
                writeValueToBytes(txpower, 0x15355575L, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x75757575L, LEN_TX_POWER);
            }
        } else if (dwm->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            if (dwm->smartPower) {
                writeValueToBytes(txpower, 0x07274767L, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x67676767L, LEN_TX_POWER);
            }
        } else {
            // TODO proper error/warning handling
        }
    } else if (dwm->channel == CHANNEL_3) {
        if (dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            if (dwm->smartPower) {
                writeValueToBytes(txpower, 0x0F2F4F6FL, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x6F6F6F6FL, LEN_TX_POWER);
            }
        } else if (dwm->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            if (dwm->smartPower) {
                writeValueToBytes(txpower, 0x2B4B6B8BL, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x8B8B8B8BL, LEN_TX_POWER);
            }
        } else {
            // TODO proper error/warning handling
        }
    } else if (dwm->channel == CHANNEL_4) {
        if (dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            if (dwm->smartPower) {
                writeValueToBytes(txpower, 0x1F1F3F5FL, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x5F5F5F5FL, LEN_TX_POWER);
            }
        } else if (dwm->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            if (dwm->smartPower) {
                writeValueToBytes(txpower, 0x3A5A7A9AL, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x9A9A9A9AL, LEN_TX_POWER);
            }
        } else {
            // TODO proper error/warning handling
        }
    } else if (dwm->channel == CHANNEL_5) {
        if (dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            if (dwm->smartPower) {
                writeValueToBytes(txpower, 0x0E082848L, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x48484848L, LEN_TX_POWER);
            }
        } else if (dwm->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            if (dwm->smartPower) {
                writeValueToBytes(txpower, 0x25456585L, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x85858585L, LEN_TX_POWER);
            }
        } else {
            // TODO proper error/warning handling
        }
    } else if (dwm->channel == CHANNEL_7) {
        if (dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            if (dwm->smartPower) {
                writeValueToBytes(txpower, 0x32527292L, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x92929292L, LEN_TX_POWER);
            }
        } else if (dwm->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            if (dwm->smartPower) {
                writeValueToBytes(txpower, 0x5171B1D1L, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0xD1D1D1D1L, LEN_TX_POWER);
            }
        } else {
            // TODO proper error/warning handling
        }
    } else {
        // TODO proper error/warning handling
    }

    //OTP tuning!
    uint8_t buf_otp[4];
    readBytesOTP(0x01E, buf_otp);

    if (buf_otp[0] == 0) {
        // No trim value available from OTP, use midrange value of 0x10
        writeValueToBytes(fsxtalt, ((0x10 & 0x1F) | 0x60), LEN_FS_XTALT);
    } else {
        writeValueToBytes(fsxtalt, ((buf_otp[0] & 0x1F) | 0x60), LEN_FS_XTALT);
    }

    // write configuration back to chip
    dwWrite(AGC_TUNE, AGC_TUNE1_SUB, agctune1, LEN_AGC_TUNE1);
    dwWrite(AGC_TUNE, AGC_TUNE2_SUB, agctune2, LEN_AGC_TUNE2);
    dwWrite(AGC_TUNE, AGC_TUNE3_SUB, agctune3, LEN_AGC_TUNE3);
    dwWrite(DRX_TUNE, DRX_TUNE0b_SUB, drxtune0b, LEN_DRX_TUNE0b);
    dwWrite(DRX_TUNE, DRX_TUNE1a_SUB, drxtune1a, LEN_DRX_TUNE1a);
    dwWrite(DRX_TUNE, DRX_TUNE1b_SUB, drxtune1b, LEN_DRX_TUNE1b);
    dwWrite(DRX_TUNE, DRX_TUNE2_SUB, drxtune2, LEN_DRX_TUNE2);
    dwWrite(DRX_TUNE, DRX_TUNE4H_SUB, drxtune4H, LEN_DRX_TUNE4H);
    dwWrite(LDE_IF, LDE_CFG1_SUB, ldecfg1, LEN_LDE_CFG1);
    dwWrite(LDE_IF, LDE_CFG2_SUB, ldecfg2, LEN_LDE_CFG2);
    dwWrite(LDE_IF, LDE_REPC_SUB, lderepc, LEN_LDE_REPC);
    dwWrite(TX_POWER, txpower, LEN_TX_POWER);
    dwWrite(RF_CONF, RF_RXCTRLH_SUB, rfrxctrlh, LEN_RF_RXCTRLH);
    dwWrite(RF_CONF, RF_TXCTRL_SUB, rftxctrl, LEN_RF_TXCTRL);
    dwWrite(TX_CAL, TC_PGDELAY_SUB, tcpgdelay, LEN_TC_PGDELAY);
    dwWrite(FS_CTRL, FS_PLLTUNE_SUB, fsplltune, LEN_FS_PLLTUNE);
    dwWrite(FS_CTRL, FS_PLLCFG_SUB, fspllcfg, LEN_FS_PLLCFG);

    dwWrite(FS_CTRL, FS_XTALT_SUB, fsxtalt, LEN_FS_XTALT);
}

//K
void AP_DwmDriverBackend::dwEnableClock(dwClock_t clock) {
    uint8_t pmscctrl0[LEN_PMSC_CTRL0];
    memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
    dwRead(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    if (clock == dwClockAuto) {
        spidev->set_speed(AP_HAL::Device::SPEED_LOW);
        pmscctrl0[0] = dwClockAuto;
        pmscctrl0[1] &= 0xFE;
    } else if (clock == dwClockXti) {
        spidev->set_speed(AP_HAL::Device::SPEED_LOW);
        pmscctrl0[0] &= 0xFC;
        pmscctrl0[0] |= dwClockXti;
    } else if (clock == dwClockPll) {
        spidev->set_speed(AP_HAL::Device::SPEED_HIGH);
        pmscctrl0[0] &= 0xFC;
        pmscctrl0[0] |= dwClockPll;
    } else {
        // TODO deliver proper warning
    }
    dwWrite(PMSC, PMSC_CTRL0_SUB, pmscctrl0, 1);
    dwWrite(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
}

void AP_DwmDriverBackend::readBytesOTP(uint16_t address, uint8_t data[]) {
    uint8_t addressBytes[LEN_OTP_ADDR];

    // p60 - 6.3.3 Reading a value from OTP memory
    // bytes of address
    addressBytes[0] = (address & 0xFF);
    addressBytes[1] = ((address >> 8) & 0xFF);
    // set address
    dwWrite(OTP_IF, OTP_ADDR_SUB, addressBytes, LEN_OTP_ADDR);
    // switch into read mode
    dwSpiWrite8(OTP_IF, OTP_CTRL_SUB, 0x03);  // OTPRDEN | OTPREAD
    dwSpiWrite8(OTP_IF, OTP_CTRL_SUB, 0x01);  // OTPRDEN
    // read value/block - 4 bytes
    dwRead(OTP_IF, OTP_RDAT_SUB, data, LEN_OTP_RDAT);
    // end read mode
    dwSpiWrite8(OTP_IF, OTP_CTRL_SUB, 0x00);
}

//K
void AP_DwmDriverBackend::dwSetAntenaDelay(dwTime_t delay) {
	dwm->antennaDelay.full = delay.full;
}


// transmit functions

void AP_DwmDriverBackend::sendPower() {
    dwNewTransmit();
    //dwSetDefaults(dev); //unused (empty if TX_MODE)

    //TODO txPacket??

    //dwSetData(dwDev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

    dwStartTransmit();
}

//K
void AP_DwmDriverBackend::dwNewTransmit() {
    dwIdle();
    memset(dwm->sysctrl, 0, LEN_SYS_CTRL);
    dwClearTransmitStatus();
    dwm->deviceMode = TX_MODE;
}

void AP_DwmDriverBackend::dwClearTransmitStatus() {
    // clear latched TX bits
    uint8_t reg[LEN_SYS_STATUS] = {0};
    setBit(reg, LEN_SYS_STATUS, AAT_BIT, true);
    setBit(reg, LEN_SYS_STATUS, TXFRB_BIT, true);
    setBit(reg, LEN_SYS_STATUS, TXPRS_BIT, true);
    setBit(reg, LEN_SYS_STATUS, TXPHS_BIT, true);
    setBit(reg, LEN_SYS_STATUS, TXFRS_BIT, true);

    dwWrite(SYS_STATUS, reg, LEN_SYS_STATUS);
}

void AP_DwmDriverBackend::dwSetData(uint8_t data[], unsigned int n) {
    if (dwm->frameCheck) {
        n += 2;  // two bytes CRC-16
    }
    if (n > LEN_EXT_UWB_FRAMES) {
        return;  // TODO proper error handling: frame/buffer size
    }
    if (n > LEN_UWB_FRAMES && !dwm->extendedFrameLength) {
        return;  // TODO proper error handling: frame/buffer size
    }
    // transmit data and length
    dwWrite(TX_BUFFER, data, n);
    dwm->txfctrl[0] = (uint8_t)(n & 0xFF);  // 1 byte (regular length + 1 bit)
    dwm->txfctrl[1] &= 0xE0;
    dwm->txfctrl[1] |= (uint8_t)((n >> 8) & 0x03);  // 2 added bits if extended length
}

void AP_DwmDriverBackend::dwStartTransmit() {
    dwWriteTransmitFrameControlRegister();
    setBit(dwm->sysctrl, LEN_SYS_CTRL, SFCST_BIT, !dwm->frameCheck);
    setBit(dwm->sysctrl, LEN_SYS_CTRL, TXSTRT_BIT, true);
    dwWrite(SYS_CTRL, dwm->sysctrl, LEN_SYS_CTRL);
    if (dwm->permanentReceive) {
        memset(dwm->sysctrl, 0, LEN_SYS_CTRL);
        dwm->deviceMode = RX_MODE;
        dwStartReceive();
    } else if (dwm->wait4resp) {
        dwm->deviceMode = RX_MODE;
    } else {
        dwm->deviceMode = IDLE_MODE;
    }
}



// receive functions

void AP_DwmDriverBackend::dwStartReceive() {
    setBit(dwm->sysctrl, LEN_SYS_CTRL, SFCST_BIT, !dwm->frameCheck);
    setBit(dwm->sysctrl, LEN_SYS_CTRL, RXENAB_BIT, true);
    dwWrite(SYS_CTRL, dwm->sysctrl, LEN_SYS_CTRL);
}

unsigned int AP_DwmDriverBackend::dwGetDataLength() {
    unsigned int len = 0;
    if (dwm->deviceMode == TX_MODE) {
        // 10 bits of TX frame control register
        len = ((((unsigned int)dwm->txfctrl[1] << 8) | (unsigned int)dwm->txfctrl[0]) & 0x03FF);
    } else if (dwm->deviceMode == RX_MODE) {
        // 10 bits of RX frame control register
        uint8_t rxFrameInfo[LEN_RX_FINFO];
        dwRead(RX_FINFO, rxFrameInfo, LEN_RX_FINFO);
        len = ((((unsigned int)rxFrameInfo[1] << 8) | (unsigned int)rxFrameInfo[0]) & 0x03FF);
    }
    if (dwm->frameCheck && len > 2) {
        return len - 2;
    }
    return len;
}

void AP_DwmDriverBackend::dwGetData(uint8_t data[], unsigned int n) {
    if (n <= 0) {
        return;
    }
    dwRead(RX_BUFFER, data, n);
}


//K
void AP_DwmDriverBackend::dwGetTransmitTimestamp(dwTime_t* time) {
	dwRead(TX_TIME, TX_STAMP_SUB, time->raw, LEN_TX_STAMP);
}
//K
void AP_DwmDriverBackend::dwGetReceiveTimestamp(dwTime_t* time) {
	time->full = 0;
	dwRead(RX_TIME, RX_STAMP_SUB, time->raw, LEN_RX_STAMP);
	// correct timestamp (i.e. consider range bias)
	dwCorrectTimestamp(time);
}
//K
void AP_DwmDriverBackend::dwGetRawReceiveTimestamp(dwTime_t* time) {
	time->full = 0;
	dwRead(RX_TIME, RX_STAMP_SUB, time->raw, LEN_RX_STAMP);
}
//K
void AP_DwmDriverBackend::dwCorrectTimestamp(dwTime_t* timestamp) {
	// base line dBm, which is -61, 2 dBm steps, total 18 data points (down to -95 dBm)
	float rxPowerBase = -(dwGetReceivePower() + 61.0f) * 0.5f;
	if (!isfinite(rxPowerBase)) {
		return;
	}
	int rxPowerBaseLow = (int)rxPowerBase;
	int rxPowerBaseHigh = rxPowerBaseLow + 1;
	if(rxPowerBaseLow <= 0) {
		rxPowerBaseLow = 0;
		rxPowerBaseHigh = 0;
	} else if(rxPowerBaseHigh >= 17) {
		rxPowerBaseLow = 17;
		rxPowerBaseHigh = 17;
	}
	// select range low/high values from corresponding table
	int rangeBiasHigh = 0;
	int rangeBiasLow = 0;
	if(dwm->channel == CHANNEL_4 || dwm->channel == CHANNEL_7) {
		// 900 MHz receiver bandwidth
		if(dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_900_16_ZERO ? -BIAS_900_16[rxPowerBaseHigh] : BIAS_900_16[rxPowerBaseHigh]);
			rangeBiasHigh <<= 1;
			rangeBiasLow = (rxPowerBaseLow < BIAS_900_16_ZERO ? -BIAS_900_16[rxPowerBaseLow] : BIAS_900_16[rxPowerBaseLow]);
			rangeBiasLow <<= 1;
		} else if(dwm->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_900_64_ZERO ? -BIAS_900_64[rxPowerBaseHigh] : BIAS_900_64[rxPowerBaseHigh]);
			rangeBiasHigh <<= 1;
			rangeBiasLow = (rxPowerBaseLow < BIAS_900_64_ZERO ? -BIAS_900_64[rxPowerBaseLow] : BIAS_900_64[rxPowerBaseLow]);
			rangeBiasLow <<= 1;
		} else {
			// TODO proper error handling
		}
	} else {
		// 500 MHz receiver bandwidth
		if(dwm->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_500_16_ZERO ? -BIAS_500_16[rxPowerBaseHigh] : BIAS_500_16[rxPowerBaseHigh]);
			rangeBiasLow = (rxPowerBaseLow < BIAS_500_16_ZERO ? -BIAS_500_16[rxPowerBaseLow] : BIAS_500_16[rxPowerBaseLow]);
		} else if(dwm->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_500_64_ZERO ? -BIAS_500_64[rxPowerBaseHigh] : BIAS_500_64[rxPowerBaseHigh]);
			rangeBiasLow = (rxPowerBaseLow < BIAS_500_64_ZERO ? -BIAS_500_64[rxPowerBaseLow] : BIAS_500_64[rxPowerBaseLow]);
		} else {
			// TODO proper error handling
		}
	}
	// linear interpolation of bias values
	float rangeBias = rangeBiasLow + (rxPowerBase - rxPowerBaseLow) * (rangeBiasHigh - rangeBiasLow);
	// range bias [mm] to timestamp modification value conversion
	dwTime_t adjustmentTime;
	adjustmentTime.full = (int)(rangeBias * SPEED_OF_LIGHT_ON_COUNTER_FREQ_INV * 0.001f);
	// apply correction
	timestamp->full -= adjustmentTime.full;
}

void AP_DwmDriverBackend::dwGetSystemTimestamp(dwTime_t* time) {
	dwRead(SYS_TIME, time->raw, LEN_SYS_TIME);
}

bool AP_DwmDriverBackend::dwIsTransmitDone() {
	return getBit(dwm->sysstatus, LEN_SYS_STATUS, TXFRS_BIT);
}

//K
bool AP_DwmDriverBackend::dwIsReceiveTimestampAvailable() {
	return getBit(dwm->sysstatus, LEN_SYS_STATUS, LDEDONE_BIT);
}

//K
float AP_DwmDriverBackend::dwGetReceiveQuality() {
	uint8_t noiseBytes[LEN_STD_NOISE];
	uint8_t fpAmpl2Bytes[LEN_FP_AMPL2];
	unsigned int noise, f2;
	dwRead(RX_FQUAL, STD_NOISE_SUB, noiseBytes, LEN_STD_NOISE);
	dwRead(RX_FQUAL, FP_AMPL2_SUB, fpAmpl2Bytes, LEN_FP_AMPL2);
	noise = (unsigned int)noiseBytes[0] | ((unsigned int)noiseBytes[1] << 8);
	f2 = (unsigned int)fpAmpl2Bytes[0] | ((unsigned int)fpAmpl2Bytes[1] << 8);
	return (float)f2 / noise;
}
//K
float AP_DwmDriverBackend::spiReadRxInfo() {
	uint8_t rxFrameInfo[LEN_RX_FINFO];
	dwRead(RX_FINFO, rxFrameInfo, LEN_RX_FINFO);
	return (float)((((unsigned int)rxFrameInfo[2] >> 4) & 0xFF) | ((unsigned int)rxFrameInfo[3] << 4));
}
//K
float AP_DwmDriverBackend::calculatePower(float base, float N, uint8_t pulseFrequency) {
	float A, corrFac;

	if(TX_PULSE_FREQ_16MHZ == pulseFrequency) {
		A = 113.77f;
		corrFac = 2.3334f;
	} else {
		A = 121.74f;
		corrFac = 1.1667f;
	}

	float estFpPwr = 10.0f * log10f(base / (N * N)) - A;

	if(estFpPwr <= -88) {
		return estFpPwr;
	} else {
		// approximation of Fig. 22 in user manual for dbm correction
		estFpPwr += (estFpPwr + 88) * corrFac;
	}

	return estFpPwr;
}
//K
float AP_DwmDriverBackend::dwGetFirstPathPower() {
	float f1 = (float)dwSpiRead16(RX_TIME, FP_AMPL1_SUB);
	float f2 = (float)dwSpiRead16(RX_FQUAL, FP_AMPL2_SUB);
	float f3 = (float)dwSpiRead16(RX_FQUAL, FP_AMPL3_SUB);
	float N = spiReadRxInfo();

	return calculatePower(f1 * f1 + f2 * f2 + f3 * f3, N, dwm->pulseFrequency);
}
//K/
float AP_DwmDriverBackend::dwGetReceivePower() {
	float C = (float)dwSpiRead16(RX_FQUAL, CIR_PWR_SUB);
	float N = spiReadRxInfo();

	float twoPower17 = 131072.0f;

	return calculatePower(C * twoPower17, N, dwm->pulseFrequency);
}

//K
bool AP_DwmDriverBackend::dwIsReceiveDone() {
	if(dwm->frameCheck) {
		return getBit(dwm->sysstatus, LEN_SYS_STATUS, RXFCG_BIT);
	}
	return getBit(dwm->sysstatus, LEN_SYS_STATUS, RXDFR_BIT);
}

// DW1000 register read/write
//K
void AP_DwmDriverBackend::dwReadSystemConfigurationRegister() {
	dwWrite(SYS_CFG, dwm->syscfg, LEN_SYS_CFG);
}

//K
void AP_DwmDriverBackend::dwWriteSystemConfigurationRegister() {
	dwWrite(SYS_CFG, dwm->syscfg, LEN_SYS_CFG);
}

//K
void AP_DwmDriverBackend::dwReadSystemEventStatusRegister() {
	dwRead(SYS_STATUS, dwm->sysstatus, LEN_SYS_STATUS);
}

//K
void AP_DwmDriverBackend::dwReadNetworkIdAndDeviceAddress() {
	dwRead(PANADR, dwm->networkAndAddress, LEN_PANADR);
}

//K
void AP_DwmDriverBackend::dwWriteNetworkIdAndDeviceAddress() {
	dwWrite(PANADR, dwm->networkAndAddress, LEN_PANADR);
}

//K
void AP_DwmDriverBackend::dwReadSystemEventMaskRegister() {
	dwRead(SYS_MASK, dwm->sysmask, LEN_SYS_MASK);
}

//K
void AP_DwmDriverBackend::dwWriteSystemEventMaskRegister() {
	dwWrite(SYS_MASK, dwm->sysmask, LEN_SYS_MASK);
}

//K
void AP_DwmDriverBackend::dwReadChannelControlRegister() {
	dwRead(CHAN_CTRL, dwm->chanctrl, LEN_CHAN_CTRL);
}

//K
void AP_DwmDriverBackend::dwWriteChannelControlRegister() {
	dwWrite(CHAN_CTRL, dwm->chanctrl, LEN_CHAN_CTRL);
}

//K
void AP_DwmDriverBackend::dwReadTransmitFrameControlRegister() {
	dwRead(TX_FCTRL, dwm->txfctrl, LEN_TX_FCTRL);
}

//K
void AP_DwmDriverBackend::dwWriteTransmitFrameControlRegister() {
	dwWrite(TX_FCTRL, dwm->txfctrl, LEN_TX_FCTRL);
}


// ------

//K
void AP_DwmDriverBackend::dwSetReceiveWaitTimeout(uint16_t timeout) {
    uint8_t* timeout_ptr = (uint8_t*) &timeout; //TODO check
	dwWrite(RX_FWTO, timeout_ptr, 2);
	setBit(dwm->syscfg, LEN_SYS_CFG, RXWTOE_BIT, timeout!=0);
}
//K
void AP_DwmDriverBackend::dwSetFrameFilter(bool val) {
	setBit(dwm->syscfg, LEN_SYS_CFG, FFEN_BIT, val);
}
//K
void AP_DwmDriverBackend::dwSetFrameFilterBehaveCoordinator(bool val) {
	setBit(dwm->syscfg, LEN_SYS_CFG, FFBC_BIT, val);
}
//K
void AP_DwmDriverBackend::dwSetFrameFilterAllowBeacon(bool val) {
	setBit(dwm->syscfg, LEN_SYS_CFG, FFAB_BIT, val);
}
//K
void AP_DwmDriverBackend::dwSetFrameFilterAllowData(bool val) {
	setBit(dwm->syscfg, LEN_SYS_CFG, FFAD_BIT, val);
}
//K
void AP_DwmDriverBackend::dwSetFrameFilterAllowAcknowledgement(bool val) {
	setBit(dwm->syscfg, LEN_SYS_CFG, FFAA_BIT, val);
}
//K
void AP_DwmDriverBackend::dwSetFrameFilterAllowMAC(bool val) {
	setBit(dwm->syscfg, LEN_SYS_CFG, FFAM_BIT, val);
}
//K
void AP_DwmDriverBackend::dwSetFrameFilterAllowReserved(bool val) {
	setBit(dwm->syscfg, LEN_SYS_CFG, FFAR_BIT, val);
}
//K
void AP_DwmDriverBackend::dwSetDoubleBuffering(bool val) {
	setBit(dwm->syscfg, LEN_SYS_CFG, DIS_DRXB_BIT, !val);
}
//K
void AP_DwmDriverBackend::dwSetInterruptPolarity(bool val) {
	setBit(dwm->syscfg, LEN_SYS_CFG, HIRQ_POL_BIT, val);
}
//K
void AP_DwmDriverBackend::dwSetReceiverAutoReenable(bool val) {
	setBit(dwm->syscfg, LEN_SYS_CFG, RXAUTR_BIT, val);
}
//K
void AP_DwmDriverBackend::dwInterruptOnSent(bool val) {
	setBit(dwm->sysmask, LEN_SYS_MASK, TXFRS_BIT, val);
}
//K
void AP_DwmDriverBackend::dwInterruptOnReceived(bool val) {
	setBit(dwm->sysmask, LEN_SYS_MASK, RXDFR_BIT, val);
	setBit(dwm->sysmask, LEN_SYS_MASK, RXFCG_BIT, val);
}
//K
void AP_DwmDriverBackend::dwInterruptOnReceiveFailed(bool val) {
	setBit(dwm->sysmask, LEN_SYS_STATUS, LDEERR_BIT, val);
	setBit(dwm->sysmask, LEN_SYS_STATUS, RXFCE_BIT, val);
	setBit(dwm->sysmask, LEN_SYS_STATUS, RXPHE_BIT, val);
	setBit(dwm->sysmask, LEN_SYS_STATUS, RXRFSL_BIT, val);
	setBit(dwm->sysmask, LEN_SYS_MASK, RXSFDTO_BIT, val);
	setBit(dwm->sysmask, LEN_SYS_MASK, AFFREJ_BIT, val);
}
//K
void AP_DwmDriverBackend::dwInterruptOnReceiveTimeout(bool val) {
	setBit(dwm->sysmask, LEN_SYS_MASK, RXRFTO_BIT, val);
	setBit(dwm->sysmask, LEN_SYS_MASK, RXPTO_BIT, val);
}
//K
void AP_DwmDriverBackend::dwInterruptOnReceiveTimestampAvailable(bool val) {
	setBit(dwm->sysmask, LEN_SYS_MASK, LDEDONE_BIT, val);
}
//K
void AP_DwmDriverBackend::dwInterruptOnAutomaticAcknowledgeTrigger(bool val) {
	setBit(dwm->sysmask, LEN_SYS_MASK, AAT_BIT, val);
}





// data handling functions

void AP_DwmDriverBackend::writeValueToBytes(uint8_t data[], long val, unsigned int n) {
    unsigned int i;
    for (i = 0; i < n; i++) {
        data[i] = ((val >> (i * 8)) & 0xFF);
    }
}

void AP_DwmDriverBackend::setBit(uint8_t data[], unsigned int n, unsigned int bit, bool val) {
    unsigned int idx;
    unsigned int shift;

    idx = bit / 8;
    if (idx >= n) {
        return;  // TODO proper error handling: out of bounds
    }
    uint8_t *targetByte = &data[idx];
    shift = bit % 8;
    if (val) {
        *targetByte |= (1 << shift);
    } else {
        *targetByte &= ~(1 << shift);
    }
}

bool AP_DwmDriverBackend::getBit(uint8_t data[], unsigned int n, unsigned int bit) {
    unsigned int idx;
    unsigned int shift;

    idx = bit / 8;
    if (idx >= n) {
        return false;  // TODO proper error handling: out of bounds
    }
    uint8_t targetByte = data[idx];
    shift = bit % 8;

    return (targetByte >> shift) & 0x01;
}


// SPI read/write functions

//static uint8_t spiBufferRx[128];
//static uint8_t spiBufferTx[128]; TODO use for transfer

void AP_DwmDriverBackend::dwRead(uint8_t reg, uint8_t *val, uint8_t len) {
    uint8_t head[1];
    head[0] = reg & 0x3F;
    spidev->transfer(head, 1, val, len);
}

void AP_DwmDriverBackend::dwRead(uint8_t reg, uint16_t sub, uint8_t *val, uint8_t len) {
    if (sub == 0) {
        dwRead(reg, val, len);
        return;
    }
    uint8_t head[3];
    head[0] = (reg & 0x3F) | 0x40;
    if (sub <= 0x7F) {
        head[1] = sub;
        spidev->transfer(head, 2, val, len);
    } else {
        head[1] = (sub & 0x7F) | 0x80;
        sub >>= 7;
        head[2] = sub & 0xff;
        spidev->transfer(head, 3, val, len);
    }
}


uint16_t AP_DwmDriverBackend::dwSpiRead16(uint8_t regid, uint32_t address) {
	uint16_t data;
	dwRead(regid, address, (uint8_t *)&data, sizeof(data));
	return data;
}

void AP_DwmDriverBackend::dwSpiWrite8(uint8_t regid, uint32_t address, uint8_t data) {
    dwWrite(regid, address, &data, sizeof(data));
}

void AP_DwmDriverBackend::dwWrite(uint8_t reg, uint8_t *val, uint8_t len) {
    uint8_t *packets;
    packets = new uint8_t[len + 1];
    packets[0] = reg | 0x80;
    memcpy((uint8_t *)&packets[1], val, len);
    spidev->transfer(packets, len + 1, nullptr, 0);
    delete[] packets;
}

void AP_DwmDriverBackend::dwWrite(uint8_t reg, uint16_t sub, uint8_t *val, uint8_t len) {
    if (sub == NO_SUB) {
        dwWrite(reg, val, len);
        return;
    }
    uint8_t *packets;
    if (sub <= 0x7F) {
        packets = new uint8_t[len + 2];
        packets[0] = reg | 0xC0;
        packets[1] = sub;
        memcpy((uint8_t *)&packets[2], val, len);
        spidev->transfer(packets, len + 2, nullptr, 0);
    } else {
        packets = new uint8_t[len + 3];
        packets[0] = reg | 0xC0;
        packets[1] = (sub & 0x7F) | 0x80;
        sub >>= 7;
        packets[2] = sub & 0xff;
        memcpy((uint8_t *)&packets[3], val, len);
        spidev->transfer(packets, len + 3, nullptr, 0);
    }
    delete[] packets;
}

bool AP_DwmDriverBackend::set_sys_reg() {
    // value to set sys_cfg register
    uint8_t send1[5] = {0x84, 0x00, 0x12, 0x00, 0x20};
    // value to set sys_ctrl register.
    // Can't check to verify as it's automatically cleared to 0 by the DWM1000 - 0x 0000 0100
    uint8_t send2[5] = {0x8D, 0x00, 0x01, 0x00, 0x00};
    if (!spidev->transfer(send1, 5, nullptr, 0)) {
        return false;  // conf_dwm write error
    }
    if (!spidev->transfer(send2, 5, nullptr, 0)) {
        return false;  // conf_dwm write error
    }
    return true;
}

