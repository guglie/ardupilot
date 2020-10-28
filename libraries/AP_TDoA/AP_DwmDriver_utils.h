#pragma once

/* ###########################################################################
 * #### DW1000 register read/write ###########################################
 * ######################################################################### */

void dwReadSystemConfigurationRegister(dwDevice_t* dev) {
	dwSpiRead(dev, SYS_CFG, NO_SUB, dev->syscfg, LEN_SYS_CFG);
}

void dwWriteSystemConfigurationRegister(dwDevice_t* dev) {
	dwSpiWrite(dev, SYS_CFG, NO_SUB, dev->syscfg, LEN_SYS_CFG);
}

void dwReadSystemEventStatusRegister(dwDevice_t* dev) {
	dwSpiRead(dev, SYS_STATUS, NO_SUB, dev->sysstatus, LEN_SYS_STATUS);
}

void dwReadNetworkIdAndDeviceAddress(dwDevice_t* dev) {
	dwSpiRead(dev, PANADR, NO_SUB, dev->networkAndAddress, LEN_PANADR);
}

void dwWriteNetworkIdAndDeviceAddress(dwDevice_t* dev) {
	dwSpiWrite(dev, PANADR, NO_SUB, dev->networkAndAddress, LEN_PANADR);
}

void dwReadSystemEventMaskRegister(dwDevice_t* dev) {
	dwSpiRead(dev, SYS_MASK, NO_SUB, dev->sysmask, LEN_SYS_MASK);
}

void dwWriteSystemEventMaskRegister(dwDevice_t* dev) {
	dwSpiWrite(dev, SYS_MASK, NO_SUB, dev->sysmask, LEN_SYS_MASK);
}

void dwReadChannelControlRegister(dwDevice_t* dev) {
	dwSpiRead(dev, CHAN_CTRL, NO_SUB, dev->chanctrl, LEN_CHAN_CTRL);
}

void dwWriteChannelControlRegister(dwDevice_t* dev) {
	dwSpiWrite(dev, CHAN_CTRL, NO_SUB, dev->chanctrl, LEN_CHAN_CTRL);
}

void dwReadTransmitFrameControlRegister(dwDevice_t* dev) {
	dwSpiRead(dev, TX_FCTRL, NO_SUB, dev->txfctrl, LEN_TX_FCTRL);
}

void dwWriteTransmitFrameControlRegister(dwDevice_t* dev) {
	dwSpiWrite(dev, TX_FCTRL, NO_SUB, dev->txfctrl, LEN_TX_FCTRL);
}


//

bool dwIsReceiveDone(dwDevice_t* dev) {
	if(dev->frameCheck) {
		return getBit(dev->sysstatus, LEN_SYS_STATUS, RXFCG_BIT);
	}
	return getBit(dev->sysstatus, LEN_SYS_STATUS, RXDFR_BIT);
}




// custom


// main loop handler

void dwHandleInterrupt(dwDevice_t *dev) {
	// read current status and handle via callbacks
	dwReadSystemEventStatusRegister(dev);
	if(dwIsClockProblem(dev) /* TODO and others */ && dev->handleError != 0) {
		(*dev->handleError)(dev);
	}
	if(dwIsTransmitDone(dev) && dev->handleSent != 0) {
		dwClearTransmitStatus(dev);
		(*dev->handleSent)(dev);
	}
	if(dwIsReceiveTimestampAvailable(dev) && dev->handleReceiveTimestampAvailable != 0) {
		dwClearReceiveTimestampAvailableStatus(dev);
		(*dev->handleReceiveTimestampAvailable)(dev);
	}
	if(dwIsReceiveFailed(dev)) { //K
		dwClearReceiveStatus(dev);
		dwRxSoftReset(dev); // Needed due to error in the RX auto-re-enable functionality. See page 35 of DW1000 manual, v2.13.
		if(dev->handleReceiveFailed != 0) {
			dev->handleReceiveFailed(dev);
			if(dev->permanentReceive) {
				dwNewReceive(dev);
				dwStartReceive(dev);
			}
		}
	} else if(dwIsReceiveTimeout(dev)) { // K
		dwClearReceiveStatus(dev);
		dwRxSoftReset(dev); // Needed due to error in the RX auto-re-enable functionality. See page 35 of DW1000 manual, v2.13.
		if(dev->handleReceiveTimeout != 0) {
			(*dev->handleReceiveTimeout)(dev);
			if(dev->permanentReceive) {
				dwNewReceive(dev);
				dwStartReceive(dev);
			}
		}
	} else if(dwIsReceiveDone(dev) && dev->handleReceived != 0) {
		dwClearReceiveStatus(dev);
		(*dev->handleReceived)(dev);
		if(dev->permanentReceive) {
			dwNewReceive(dev);
			dwStartReceive(dev);
		}
	}
}




