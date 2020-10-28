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

#pragma once


#define BUCKET_LOG true

#define BUCKET_ACCEPTANCE_LEVEL 5
#define MAX_BUCKET_FILL 12
#define FILTER_CLOSE_DELAY_COUNT 30

#define FILTER_LEVELS 5
#define FILTER_NONE FILTER_LEVELS


#define INITIAL_LEVEL FILTER_LEVELS/2
#define INITIAL_PREV_LEVEL INITIAL_LEVEL-1

#include <AP_Math/AP_Math.h>
#include <AP_Math/VectorN.h>


typedef struct {
  float acceptanceLevel;
  int bucket;
} filterLevel_t;


class AP_TDOA_OutlierFilter {
public:

	filterLevel_t filterLevels[FILTER_LEVELS] = {
			{.acceptanceLevel = 0.4},
			{.acceptanceLevel = 0.8},
			{.acceptanceLevel = 1.2},
			{.acceptanceLevel = 1.6},
			{.acceptanceLevel = 2.0},
	};

	float acceptanceLevel = filterLevels[INITIAL_LEVEL].acceptanceLevel; //0.0f;
	float errorDistance;
	int filterCloseDelayCounter = 0;
	int previousFilterIndex = INITIAL_PREV_LEVEL;


	bool validateTdoaSimple(float tdoa, Vector3f anc1, Vector3f anc0);
	bool validateTdoaSteps(float tdoa, Vector3f anc1, Vector3f anc0, const float error, const Vector3f jacobian, const Vector3f estPos);

	void reset();

private:

	bool isDistanceDiffSmallerThanDistanceBetweenAnchors(float tdoa, Vector3f anc1, Vector3f anc0);
	float distanceSq(const Vector3f a, const Vector3f b);
	float sq(float a) {return a * a;}
	void addToBucket(filterLevel_t* filter);
	void removeFromBucket(filterLevel_t* filter);
	int updateBuckets();
};

