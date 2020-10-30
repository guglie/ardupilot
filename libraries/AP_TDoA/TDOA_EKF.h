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
 * TDoA EKF by Guglielmo Cassinelli
 * 
 * Thanks to Michael Hamer, Mark Mueller, Kristoffer Richardsson
 * 
 */

#pragma once


#include <AP_HAL/AP_HAL.h>
//#include <DataFlash/DataFlash.h>


#define ALLOW_DOUBLE_TRIG_FUNCTIONS  //enable double math
#define ALLOW_DOUBLE_MATH_FUNCTIONS


#if CONFIG_HAL_BOARD != HAL_BOARD_SITL

#ifndef ARM_MATH_CM4
	#define ARM_MATH_CM4
#endif

#include "arm_math.h"


#endif



#include <AP_Math/AP_Math.h>
#include <AP_Math/VectorN.h>

#include <limits>

#include "AP_TDoA_outlier_filter.h"


#define TDOA_EKF_DEBUG true


/*************
 * PARAMETERS
 */
#define ANCHOR_MAX_NUM 256
#define DEFAULT_POSITION_STDEV 0.1f // variance of the estimated position in cm

#define INITIAL_POS_VARIANCE_XY 2.0f
#define INITIAL_POS_VARIANCE_Z 2.0f
#define INITIAL_VEL_VARIANCE 0.1f // assuming vehicle starts steady

#define ACC_NOISE_XY 0.0f
#define ACC_NOISE_Z 0.0f
#define VEL_NOISE_XY 4.0f
#define VEL_NOISE_Z 3.0f
#define POS_NOISE 0.0f


// costants
#define MAX_COVARIANCE 10000.0f
#define MIN_COVARIANCE 0.000001f


//TODO set as parameters
#define INITIAL_POS_STATE Vector3f(10,10,1)

#define ANC_UPDATE_ALPHA 0.02 //0.05
#define ANC_UPDATE_ALPHA_INV  (1. - ANC_UPDATE_ALPHA)

#define OUTLIER_FILTER_ENABLE true
#define USE_ALT_ZERO_WHEN_DISARMED false
#define USE_ANCHOR_ERROR_COMBO false // use average anchor error (instead of default TDoA St.Dev = 0.15)
/**
 * The internally-estimated state is:
 * - X, Y, Z: the quad's position in the global frame
 * - VX, VY, VZ: the quad's velocity in the global frame
 */
typedef enum {
  STATE_X = 0, STATE_Y, STATE_Z, N_SIZE
} stateIdx_t;

extern const AP_HAL::HAL &hal;


/*class MyTest {
private:
	arm_matrix_instance_f32 Hm;// = {1, N, (float *)H};

public:
	MyTest() {
		static float H[N] = {0};
		arm_mat_init_f32(&Hm, 1, N, (float *)H);
	}

	void test() {
		Hm.pData[0] = 0.1f;
        Hm.pData[1] = 0.2f;
        Hm.pData[2] = 0.3f;

		//this toy example works
		if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.MyTest() %f %f %f must be equal to %f %f %f \n", Hm.pData[0], Hm.pData[1], Hm.pData[2], 0.1f, 0.2f, 0.3f);
	}
};*/


class TDOA_EKF {
private:

	float x[N_SIZE];      // state vector
	float P[N_SIZE * N_SIZE];  // The covariance matrix
	float Q[N_SIZE * N_SIZE];  // process noise covariance matrix
	float F[N_SIZE * N_SIZE];  // update matrix
	float H[N_SIZE];      // extraction (column) vector
	float K[N_SIZE];      // kalman gain vector
	float I[N_SIZE * N_SIZE];  // identity matrix, initialize all to 0

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL //check device id only if not in SITL

	arm_matrix_instance_f32 xm;// = {N, 1, (float *)x};
	arm_matrix_instance_f32 Pm;// = {N, N, (float *)P};
	arm_matrix_instance_f32 Qm;// = {N, N, (float *)Q};
	arm_matrix_instance_f32 Fm;// = {N, N, (float *)F};
	arm_matrix_instance_f32 Hm;// = {1, N, (float *)H};
	arm_matrix_instance_f32 Km;// = {N, 1, (float *)K};
	arm_matrix_instance_f32 Im;// = {N, N, (float *)I};
#endif

	static float anchor_error_avg[ANCHOR_MAX_NUM]; //TODO use the tdoa storage

	float origin_alt = 0.0f;

	float firstObservationDone = false;
	float dt = 0; //time elapsed from last update (set by set_delta_time(), used in state_transition_f())

	void set_process_noise_Q();

	void set_transition_matrix_F();

	void state_transition_f();

	void stateEstimatorScalarUpdate(float error, float stdMeasNoise);

	static bool outlierFilterVaildateTdoaSteps(float tdoa, float error, Vector3f jacobian, Vector3f pos, Vector3f anc1, Vector3f anc0);

	static bool isDistanceDiffSmallerThanDistanceBetweenAnchors(float tdoa, Vector3f anc1, Vector3f anc0);

	void set_delta_time(float delta_time);

	//main function to update estimation with one TDOA
	void tdoa_scalar_step(float tdoa, Vector3f xyz1, Vector3f xyz0, float tdoa_stdev, uint8_t anc1_id, uint8_t anc0_id);



	void initialize_filter();

	void initialize_state_x();


#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
	bool contains_nan(arm_matrix_instance_f32 * m);
	bool nan_value_needs_reset(arm_matrix_instance_f32 * m);
#endif


	void update_anchor_error(float error, Vector3f jacobian, uint8_t anc1_id, uint8_t anc0_id) {
		float errorBaseDistance = norm(jacobian.x, jacobian.y, jacobian.z);
		float errorDistance = fabsf(error / errorBaseDistance);

		anchor_error_avg[anc1_id] = anchor_error_avg[anc1_id] * ANC_UPDATE_ALPHA_INV + errorDistance * ANC_UPDATE_ALPHA;
		anchor_error_avg[anc0_id] = anchor_error_avg[anc0_id] * ANC_UPDATE_ALPHA_INV + errorDistance * ANC_UPDATE_ALPHA;
	}

	float get_anchor_error_combo(uint8_t anc1_id, uint8_t anc0_id) {
		return (anchor_error_avg[anc1_id] + anchor_error_avg[anc0_id]) / 2.0f; //average of estimated errors of the 2 anchors of a TDoA
	}



	//static void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst) { arm_mat_trans_f32(pSrc, pDst); }
	//static void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst) { configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); }
	//static void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst) { configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }

public:
	AP_TDOA_OutlierFilter outlierFilter;
	bool needReset = false;
	float lastObservationTime = 0;

    // constructor
	TDOA_EKF(float origin_altitude = 0);

	//main update
	void update(uint64_t usec_timestamp, float tdoa, Vector3f anc1, Vector3f anc0, float tdoa_stdev, uint8_t anc1_id, uint8_t anc0_id);

	//getters
	Vector3f getPosition() {
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
		return Vector3f(xm.pData[STATE_X], xm.pData[STATE_Y], xm.pData[STATE_Z]);
		//return Vector3f(xm.pData[0], xm.pData[1], xm.pData[2]);
#else
		return Vector3f(0.1,0.1,0.1);
#endif
	}

	void test() {
    	//static float H[N] = {0};
		//arm_mat_init_f32(&Hm, 1, N, (float *)H);

		Hm.pData[STATE_X] = 0.1f;
        Hm.pData[STATE_Y] = 0.2f;
        Hm.pData[STATE_Z] = 0.3f;

		if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.test()   %f %f %f must be equal to %f %f %f \n", Hm.pData[0], Hm.pData[1], Hm.pData[2], 0.1f, 0.2f, 0.3f);
	}

	float getPosFilterErrorLevel() {
//#if OUTLIER_FILTER_ENABLE
//		return outlierFilter.acceptanceLevel / 2.;
//#else
		return DEFAULT_POSITION_STDEV;
//#endif
	}

	float getPosX() {
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
		return xm.pData[STATE_X];
#else
		return 0;
#endif
	}
	float getPosY() {
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
		return xm.pData[STATE_Y];
#else
		return 0;
#endif
	}
	float getPosZ() {
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
		return xm.pData[STATE_Z];
#else
		return 0;
#endif
	}


	Vector3f getVelocity() { return Vector3f(0.,0.,0.); }

	Vector3f getPosVariance() {
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
		return Vector3f(Pm.pData[0], Pm.pData[1 * N_SIZE + 1], Pm.pData[2 * N_SIZE + 2]);
#else
		return Vector3f(0,0,0);
#endif
	}

	float get_anchor_error(uint8_t anc) {
		return anchor_error_avg[anc];
	}

	void resetFilter();



};
