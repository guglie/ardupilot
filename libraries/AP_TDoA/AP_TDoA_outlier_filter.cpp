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
 * TDoA Outlier Filter by Guglielmo Cassinelli
 * 
 * ported to ArduPilot from Crazyflie firmware by Bitcraze AB
 */

#include "AP_TDoA_outlier_filter.h"

#include <AP_Logger/AP_Logger.h>




bool AP_TDOA_OutlierFilter::validateTdoaSimple(float tdoa, Vector3f anc1, Vector3f anc0) {
	return isDistanceDiffSmallerThanDistanceBetweenAnchors(tdoa, anc1, anc0);
}

bool AP_TDOA_OutlierFilter::validateTdoaSteps(float tdoa, Vector3f anc1, Vector3f anc0, const float error, const Vector3f jacobian, const Vector3f estPos) {
  bool sampleIsGood = false;

  if (isDistanceDiffSmallerThanDistanceBetweenAnchors(tdoa, anc1, anc0)) {
    float errorBaseDistance = norm(jacobian.x, jacobian.y, jacobian.z);
    errorDistance = fabsf(error / errorBaseDistance);

    int filterIndex = updateBuckets();

    if (filterIndex > previousFilterIndex) {
      filterCloseDelayCounter = FILTER_CLOSE_DELAY_COUNT;
    } else if (filterIndex < previousFilterIndex) {
      if (filterCloseDelayCounter > 0) {
        filterCloseDelayCounter--;
        filterIndex = previousFilterIndex;
      }
    }
    previousFilterIndex = filterIndex;

    if (filterIndex == FILTER_NONE) {
      // Lost tracking, open up to let the kalman filter converge
      acceptanceLevel = 100.0;
      sampleIsGood = true;
    } else {
      acceptanceLevel = filterLevels[filterIndex].acceptanceLevel;
      if (errorDistance < acceptanceLevel) {
        sampleIsGood = true;
      }
    }
  }

  return sampleIsGood;
}

void AP_TDOA_OutlierFilter::reset() {
  // Nothing here
}

bool AP_TDOA_OutlierFilter::isDistanceDiffSmallerThanDistanceBetweenAnchors(float tdoa, Vector3f anc1, Vector3f anc0) {
  float anchorDistanceSq = distanceSq(anc1, anc0);
  float distanceDiffSq = sq(tdoa);
  return (distanceDiffSq < anchorDistanceSq);
}

float AP_TDOA_OutlierFilter::distanceSq(const Vector3f a, const Vector3f b) {
	Vector3f diffVect = a - b;
	return norm(diffVect.x, diffVect.y, diffVect.z);
}


void AP_TDOA_OutlierFilter::addToBucket(filterLevel_t* filter) {
  if (filter->bucket < MAX_BUCKET_FILL) {
    filter->bucket++;
  }
}

void AP_TDOA_OutlierFilter::removeFromBucket(filterLevel_t* filter) {
  if (filter->bucket > 0) {
    filter->bucket--;
  }
}

int AP_TDOA_OutlierFilter::updateBuckets() {
  int filterIndex = FILTER_NONE;

  for (int i = FILTER_LEVELS - 1; i >= 0; i--) {
    filterLevel_t* filter = &filterLevels[i];

    if (errorDistance < filter->acceptanceLevel) {
      removeFromBucket(filter);
    } else {
      addToBucket(filter);
    }

    if (filter->bucket < BUCKET_ACCEPTANCE_LEVEL) {
      filterIndex = i;
    }
  }

#if BUCKET_LOG
  AP::logger().Write("TDAF", "TimeUS,filter", "Qf", AP_HAL::micros64(), filterLevels[filterIndex].acceptanceLevel);
#endif

  return filterIndex;
}



