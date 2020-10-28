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
#include "AP_Beacon_TDoA.h"
#include <ctype.h>
#include <stdio.h>
#include <AP_Math/AP_Math.h>

//for fake gps
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_GPS/AP_GPS.h>

//logs
#include <AP_Logger/AP_Logger.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

//AP_DwmDriver *AP_Beacon_TDoA::_dwm_driver;
AP_DwmDriverBackend *AP_Beacon_TDoA::_dwm_driver;


// constructor
AP_Beacon_TDoA::AP_Beacon_TDoA(AP_Beacon &frontend, AP_SerialManager &serial_manager) :
    AP_Beacon_Backend(frontend)
{
    if(_dwm_driver == nullptr) {

    	//Location origin_loc;
    	//_frontend.get_origin(origin_loc);
    	//float origin_alt_meters = origin_loc.alt/100.0;

        _dwm_driver = new AP_DwmDriverBackend();
		_dwm_driver->driver_init();
    }
}

bool AP_Beacon_TDoA::healthy()
{
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

// update the state of the sensor
void AP_Beacon_TDoA::update(void)
{
	
	
    //uint32_t curr_estimation_update = this->_tdoa->getLastPosTimestamp() / 1000;   //this->_tdoa->getLastUpdate();

	//
    /*if(this->_last_estimation_update + FAKE_GPS_UPDATE_INTERVAL < curr_estimation_update) { // TODO CHECK
    	this->_last_estimation_update = curr_estimation_update;

		Vector3f estimated_position = this->_tdoa->getEstimatedPosition();
		float position_error = this->_tdoa->getPosFilterErrorLevel();
		uint64_t pkt0_time = this->_tdoa->getLastPosTimestamp();
		//set_beacon_tdoa_dist(estimated_position);

		Vector3f position_ned = this->correct_for_orient_yaw(estimated_position);

		Location origin_loc;
		this->_frontend.get_origin(origin_loc);

		origin_loc.offset(position_ned.x, position_ned.y);

		float position_alt = position_ned.z + origin_loc.alt/100.0;


		// correct offboard timestamp to be in local ms since boot
		uint32_t timeStamp_ms;
		uint32_t resetTime_ms = 0;
    	const Vector3f sensor_offset = {};
		const Vector3f position = {x, y, z};

		AP::ahrs().writeExtNavData(sensor_offset, estimated_position, Quaternion(), 0.15f, 100, timeStamp_ms, resetTime_ms);

	}*/

	//TODO set last_update only when position is updated
	//last_update_ms = AP_HAL::millis();
}
