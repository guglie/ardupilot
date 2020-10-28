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

#include "AP_ClockCorrectionEngine.h"

using namespace AP_ClockCorrectionEngine;

#define MAX_CLOCK_DEVIATION_SPEC 10e-6
#define CLOCK_CORRECTION_SPEC_MIN (1.0 - MAX_CLOCK_DEVIATION_SPEC * 2)
#define CLOCK_CORRECTION_SPEC_MAX (1.0 + MAX_CLOCK_DEVIATION_SPEC * 2)

#define CLOCK_CORRECTION_ACCEPTED_NOISE 0.03e-6
#define CLOCK_CORRECTION_FILTER 0.1
#define CLOCK_CORRECTION_BUCKET_MAX 4

/**
 Obtains the clock correction from a clockCorrectionStorage_t object. This is the recommended public API to obtain the clock correction, instead of getting it directly from the storage object.
 */
double AP_ClockCorrectionEngine::get(const clockCorrectionStorage_t* storage) {
  return storage->clockCorrection;
}

/**
 Truncates a timestamp to the number of bits of the mask. This truncation ensures that the value returned is a valid time event, even if the time counter wrapped around.
 */
static uint64_t truncateTimeStamp(uint64_t timeStamp, uint64_t mask) {
  return timeStamp & mask;
}

/**
 Implementation of the leaky bucket algorithm. See: https://en.wikipedia.org/wiki/Leaky_bucket
 */
static void fillClockCorrectionBucket(clockCorrectionStorage_t* storage) {
  if (storage->clockCorrectionBucket < CLOCK_CORRECTION_BUCKET_MAX) {
    storage->clockCorrectionBucket++;
  }
}

/**
 Implementation of the leaky bucket algorithm. See: https://en.wikipedia.org/wiki/Leaky_bucket
 */
static bool emptyClockCorrectionBucket(clockCorrectionStorage_t* storage) {
  if (storage->clockCorrectionBucket > 0) {
    storage->clockCorrectionBucket--;
    return false;
  }

  return true;
}

/**
 Calculates the clock correction between a reference clock and another clock (x).

 @param new_t_in_cl_reference The newest time of occurrence for an event (t), measured by the clock of reference
 @param old_t_in_cl_reference An older time of occurrence for an event (t), measured by the clock of reference
 @param new_t_in_cl_x The newest time of occurrence for an event (t), measured by clock x
 @param old_t_in_cl_x The previous time of occurrence for an event (t), measured by clock x
 @param mask A mask as long as the number of bits used to represent the timestamps. Used to calculate a valid timestamp, even if wrapped arounds of the time counter happened at some point
 @return The necessary clock correction to apply to timestamps measured by clock x, in order to obtain their value like if the measurement was done by the reference clock. Or -1 if it was not possible to perform the computation. Example: timestamp_in_cl_reference = clockCorrection * timestamp_in_cl_x
 */
double AP_ClockCorrectionEngine::calculate(const uint64_t new_t_in_cl_reference, const uint64_t old_t_in_cl_reference, const uint64_t new_t_in_cl_x, const uint64_t old_t_in_cl_x, const uint64_t mask) {
  const uint64_t tickCount_in_cl_reference = truncateTimeStamp(new_t_in_cl_reference - old_t_in_cl_reference, mask);
  const uint64_t tickCount_in_cl_x = truncateTimeStamp(new_t_in_cl_x - old_t_in_cl_x, mask);

  if (tickCount_in_cl_x == 0) {
    return -1;
  }

  return (double)tickCount_in_cl_reference / (double)tickCount_in_cl_x;
}

/**
 Updates the clock correction only if the provided value follows certain conditions. This is used to discard wrong clock correction measurements.
 @return True if the provided clock correction sample ir reliable, false otherwise. A sample is reliable when it is in the accepted noise level (which means that we already have two or more samples that are similar) and has been LP filtered.
 */
bool AP_ClockCorrectionEngine::update(clockCorrectionStorage_t* storage, const double clockCorrectionCandidate) {
  bool sampleIsReliable = false;

  const double currentClockCorrection = storage->clockCorrection;
  const double difference = clockCorrectionCandidate - currentClockCorrection;

  if (-CLOCK_CORRECTION_ACCEPTED_NOISE < difference && difference < CLOCK_CORRECTION_ACCEPTED_NOISE) {
    // Simple low pass filter
    const double newClockCorrection = currentClockCorrection * CLOCK_CORRECTION_FILTER + clockCorrectionCandidate * (1.0 - CLOCK_CORRECTION_FILTER);

    sampleIsReliable = true;
    fillClockCorrectionBucket(storage);
    storage->clockCorrection = newClockCorrection;
  } else {
    const bool shouldAcceptANewClockReference = emptyClockCorrectionBucket(storage);
    if (shouldAcceptANewClockReference) {
      if (CLOCK_CORRECTION_SPEC_MIN < clockCorrectionCandidate && clockCorrectionCandidate < CLOCK_CORRECTION_SPEC_MAX) {
        // We do not fill the bucket and accept the clock correction sample as reliable: a sample is reliable when it is in the accepted noise level (which means that we already have two or more samples that are similar) and has been LP filtered. See: https://github.com/bitcraze/crazyflie-firmware/pull/328
        storage->clockCorrection = clockCorrectionCandidate;
      }
    }
  }

  return sampleIsReliable;
}
