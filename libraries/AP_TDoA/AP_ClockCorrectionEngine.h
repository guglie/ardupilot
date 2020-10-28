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
#pragma once

#include <AP_HAL/AP_HAL.h>

namespace AP_ClockCorrectionEngine {

typedef struct {
    double clockCorrection;
    unsigned int clockCorrectionBucket;
} clockCorrectionStorage_t;

double get(const clockCorrectionStorage_t* storage);
double calculate(const uint64_t new_t_in_cl_reference, const uint64_t old_t_in_cl_reference, const uint64_t new_t_in_cl_x, const uint64_t old_t_in_cl_x, const uint64_t mask);
bool update(clockCorrectionStorage_t* storage, const double clockCorrectionCandidate);

};  // namespace AP_ClockCorrectionEngine
