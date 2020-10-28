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
 * UWB Localization with Time Difference of Arrival by Guglielmo Cassinelli
 * 
 */
#pragma once


#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include "AP_DwmDriverBackend.h"

#include "AP_TdoaStorage.h"
#include "AP_Tdoa3Tag.h"






class AP_DwmDriver {
private:


    AP_DwmDriverBackend driver;

    static AP_DwmDriver* _singleton;


    static AP_TdoaStorage storage;
    static AP_Tdoa3Tag tag;


public:
    AP_DwmDriver() {
        _singleton = this;

        driver = AP_DwmDriverBackend();

        // initialize storage singleton
        storage = AP_TdoaStorage();
    

        tag = AP_Tdoa3Tag();


        
        
    }

    static AP_DwmDriver* get_singleton() { return _singleton; }

};



