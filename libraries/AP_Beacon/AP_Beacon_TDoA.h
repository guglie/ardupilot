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
 * TDoA Tag by Guglielmo Cassinelli
 * 
 */
#pragma once

#include "AP_Beacon_Backend.h"
#include <AP_TDoA/AP_DwmDriverBackend.h>

//#include <stdint.h>


class AP_Beacon_TDoA : public AP_Beacon_Backend
{
public:
    // constructor
    AP_Beacon_TDoA(AP_Beacon &frontend, AP_SerialManager &serial_manager);

    // update
    void update() override;

    bool healthy() override;

    double get_tdoa(uint8_t id) {
    	//return this->_tdoa->getLastDistDiff(id);
        return 0;
    }

private:

    uint32_t _last_estimation_update;
    uint32_t last_update_ms = 0;

    static AP_DwmDriverBackend * _dwm_driver;
};
