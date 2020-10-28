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

#include "TDOA_EKF.h"




/* PUBLIC METHODS */

float TDOA_EKF::anchor_error_avg[ANCHOR_MAX_NUM] = {0.};


TDOA_EKF::TDOA_EKF(float origin_altitude) {
    if(TDOA_EKF_DEBUG) hal.console->printf(" \n TDOA_EKF.TDOA_EKF(): constructor \n");

    origin_alt = origin_altitude;

    outlierFilter = AP_TDOA_OutlierFilter();

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL

    /*float* x = new float[N_SIZE] {0};      // state vector
	float* P = new float[N_SIZE * N_SIZE] {0};  // The covariance matrix
	float* Q = new float[N_SIZE * N_SIZE] {0};  // process noise covariance matrix
	float* F = new float[N_SIZE * N_SIZE] {0};  // update matrix
	float* H = new float[N_SIZE] {0};      // extraction (column) vector
	float* K = new float[N_SIZE] {0};      // kalman gain vector
	float* I = new float[N_SIZE * N_SIZE] {0};  // identity matrix, initialize all to 0*/

    static float x[N_SIZE] = {0};      // state vector
	static float P[N_SIZE * N_SIZE] = {0};  // The covariance matrix
	static float Q[N_SIZE * N_SIZE] = {0};  // process noise covariance matrix
	static float F[N_SIZE * N_SIZE] = {0};  // update matrix
	static float H[N_SIZE] = {0};      // extraction (column) vector
	static float K[N_SIZE] = {0};      // kalman gain vector
	static float I[N_SIZE * N_SIZE] = {0};  // identity matrix, initialize all to 0

    arm_mat_init_f32(&xm, N_SIZE, 1, (float *) x);
    arm_mat_init_f32(&Pm, N_SIZE, N_SIZE, (float *) P);
    arm_mat_init_f32(&Qm, N_SIZE, N_SIZE, (float *) Q);
    arm_mat_init_f32(&Fm, N_SIZE, N_SIZE, (float *) F);
    arm_mat_init_f32(&Hm, 1, N_SIZE, (float *) H);
    arm_mat_init_f32(&Km, N_SIZE, 1, (float *) K);
    arm_mat_init_f32(&Im, N_SIZE, N_SIZE, (float *) I);

    //initialize identity
    for (int i = 0; i < N_SIZE; i++) {
        for (int j = 0; j < N_SIZE; j++) {
            Im.pData[i * N_SIZE + j] = (i == j ? 1.0 : 0.0);
        }
    }

#endif

    initialize_filter();

    set_delta_time(0);
}

void TDOA_EKF::update(uint64_t usec_timestamp, float tdoa, Vector3f anc1, Vector3f anc0, float tdoa_stdev, uint8_t anc1_id, uint8_t anc0_id) {
    float timeSec = usec_timestamp / 1000000.0;

    if (!firstObservationDone) {
        firstObservationDone = true;  //dt=0
    } else {
        set_delta_time(timeSec - lastObservationTime);
    }

    lastObservationTime = timeSec;

    tdoa_scalar_step(tdoa, anc1, anc0, tdoa_stdev, anc1_id, anc0_id);
}

/* PRIVATE METHODS */

void TDOA_EKF::set_delta_time(float delta_time) {
    this->dt = delta_time;

    set_transition_matrix_F();
    set_process_noise_Q();
}

void TDOA_EKF::tdoa_scalar_step(float tdoa, Vector3f anc1, Vector3f anc0, float tdoa_stdev, uint8_t anc1_id, uint8_t anc0_id) {
    if (needReset) {
        resetFilter();
    }

    if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.tdoa_scalar_step() 1   tdoa %f    dt %f \n", tdoa, dt);

    // PREDICT

    state_transition_f();

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    bool success = true;

    // P = F dot P dot F.T + Q
    static float F_T[N_SIZE * N_SIZE];  //size must be transpose of F size
    static arm_matrix_instance_f32 F_Tm = {N_SIZE, N_SIZE, (float *)F_T};

    static float FP[N_SIZE * N_SIZE];  //size must be (F rows * P cols)
    static arm_matrix_instance_f32 FPm = {N_SIZE, N_SIZE, (float *)FP};

    static float FPFT[N_SIZE * N_SIZE];  //size must be (FP rows * F_T cols)
    static arm_matrix_instance_f32 FPFTm = {N_SIZE, N_SIZE, (float *)FPFT};

    success = success && ARM_MATH_SUCCESS == arm_mat_trans_f32(&Fm, &F_Tm);          // F_T = transpose F
    success = success && ARM_MATH_SUCCESS == arm_mat_mult_f32(&Fm, &Pm, &FPm);       // FP = F * P
    success = success && ARM_MATH_SUCCESS == arm_mat_mult_f32(&FPm, &F_Tm, &FPFTm);  // FPFT = FP * F_T

    if (nan_value_needs_reset(&FPFTm) || nan_value_needs_reset(&Qm)) return;

    if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.tdoa_scalar_step() 2\n");

    success = success && ARM_MATH_SUCCESS == arm_mat_add_f32(&FPFTm, &Qm, &Pm);  // P = FPFT + Q   =    F * P * Ft + Q

    //TODO ensure P boundedness and symmetry
    //TODO: Why would it hit these bounds? Needs to be investigated.
    for (int i = 0; i < N_SIZE; i++) {
        for (int j = i; j < N_SIZE; j++) {
            float p = 0.5f * Pm.pData[i * N_SIZE + j] + 0.5f * Pm.pData[j * N_SIZE + i];  // add measurement noise
            if (isnan(p) || p > MAX_COVARIANCE) {
                Pm.pData[i * N_SIZE + j] = Pm.pData[j * N_SIZE + i] = MAX_COVARIANCE;
            } else if (i == j && p < MIN_COVARIANCE) {
                Pm.pData[i * N_SIZE + j] = Pm.pData[j * N_SIZE + i] = MIN_COVARIANCE;
            } else {
                Pm.pData[i * N_SIZE + j] = Pm.pData[j * N_SIZE + i] = p;
            }
        }
    }

    if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.tdoa_scalar_step() 3 measurement pred  H = %f %f %f  N = %d\n", Hm.pData[STATE_X], Hm.pData[STATE_Y], Hm.pData[STATE_Z], N_SIZE);


    // measurement prediction

    /*for (int i = 0; i < N; i++) {
        Hm.pData[i] = 0.0;
    }*/

    Vector3f pos = getPosition();  //estimated pos

    Vector3f vect_d1 = pos - anc1;
    Vector3f vect_d0 = pos - anc0;
    float d1 = norm(vect_d1.x, vect_d1.y, vect_d1.z);
    float d0 = norm(vect_d0.x, vect_d0.y, vect_d0.z);

    float predicted = d1 - d0;
    float error = tdoa - predicted;

    if (d1 != 0.0 && d0 != 0.0) {
        Hm.pData[0] = 0.5f; //((pos.x - anc1.x) / d1 - (pos.x - anc0.x) / d0);  //NOT WORKING! WHY?
        Hm.pData[1] = 0.6f; //((pos.y - anc1.y) / d1 - (pos.x - anc0.y) / d0);
        Hm.pData[2] = 0.7f; //((pos.z - anc1.z) / d1 - (pos.x - anc0.z) / d0);

        if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.tdoa_scalar_step() 3.4: wait \n");

        xm.pData[0] = 0.55f;
        xm.pData[1] = 0.66f;
        xm.pData[2] = 0.77f;

        Km.pData[0] = 0.555f;
        Km.pData[1] = 0.666f;
        Km.pData[2] = 0.777f;


        if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.tdoa_scalar_step() 3.5: H should have been set to %f %f %f but is  %f %f %f  and x =  %f %f %f  and K = %f %f %f \n",
                            ((pos.x - anc1.x) / d1 - (pos.x - anc0.x) / d0),
                            ((pos.y - anc1.y) / d1 - (pos.x - anc0.y) / d0),
                            ((pos.z - anc1.z) / d1 - (pos.x - anc0.z) / d0),
                            Hm.pData[0], Hm.pData[1], Hm.pData[2], xm.pData[0], xm.pData[1], xm.pData[2], Km.pData[0], Km.pData[1], Km.pData[2] );

#if USE_ALT_ZERO_WHEN_DISARMED
        bool soft_armed = hal.util->get_soft_armed();

        if (!soft_armed) {
            Hm.pData[STATE_Z] = -origin_alt;
        }
#endif

        if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.tdoa_scalar_step() 4: H = %f %f %f   d1 %f  d0 %f   pos  %f %f %f  pred %f   anc1Pos   %f %f %f   anc0Pos   %f %f %f  \n", Hm.pData[STATE_X], Hm.pData[STATE_Y], Hm.pData[STATE_Z], d1, d0, pos.x, pos.y, pos.z, predicted, anc1.x, anc1.y, anc1.z, anc0.x, anc0.y, anc0.z);

        Vector3f jacobian(Hm.pData[STATE_X], Hm.pData[STATE_Y], Hm.pData[STATE_Z]);

        //TODO consider using position from arducopter EKF (using gyro and accelerometer data)
        //instead of pos from this TDOA filter

#if OUTLIER_FILTER_ENABLE
        bool sampleIsGood = outlierFilter.validateTdoaSteps(tdoa, anc1, anc0, error, jacobian, pos);
#else
        bool sampleIsGood = outlierFilter.validateTdoaSimple(tdoa, anc1, anc0);
#endif

        if (sampleIsGood) {
            if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.tdoa_scalar_step() 5 sample is good\n");

#if USE_ANCHOR_ERROR_COMBO
            update_anchor_error(error, jacobian, anc1_id, anc0_id);
            tdoa_stdev = get_anchor_error_combo(anc1_id, anc0_id);
#endif

            stateEstimatorScalarUpdate(error, tdoa_stdev);
        }
    }

#endif
}

void TDOA_EKF::set_process_noise_Q() {
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL

    if (dt > 0) {
        Qm.pData[STATE_X * N_SIZE + STATE_X] = sq(ACC_NOISE_XY * sq(dt) + VEL_NOISE_XY * dt + POS_NOISE);
        Qm.pData[STATE_Y * N_SIZE + STATE_Y] = sq(ACC_NOISE_XY * sq(dt) + VEL_NOISE_XY * dt + POS_NOISE);
        Qm.pData[STATE_Z * N_SIZE + STATE_Z] = sq(ACC_NOISE_Z * sq(dt) + VEL_NOISE_Z * dt + POS_NOISE);
    } else {
        Qm.pData[STATE_X * N_SIZE + STATE_X] = 0;
        Qm.pData[STATE_Y * N_SIZE + STATE_Y] = 0;
        Qm.pData[STATE_Z * N_SIZE + STATE_Z] = 0;
    }
#endif
}

void TDOA_EKF::set_transition_matrix_F() {
    //prepare matrix F to have:

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL

    //state = 1 * current_state
    for (int i = 0; i < N_SIZE; i++) {
        Fm.pData[i * N_SIZE + i] = 1.;
    }

#endif
}

void TDOA_EKF::state_transition_f() {
}

void TDOA_EKF::stateEstimatorScalarUpdate(float error, float stdMeasNoise) {
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    bool success = true;  //TODO remove

	if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.stateEstimatorScalarUpdate() 1   H %f %f %f \n", Hm.pData[0], Hm.pData[1], Hm.pData[2]);


    if (nan_value_needs_reset(&Hm)) return;

    static float H_T[N_SIZE];
    static arm_matrix_instance_f32 H_Tm = {N_SIZE, 1, (float *)H_T};  //DONT reuse H data

    //copy H data (no need to transpose)
    for (int i = 0; i < N_SIZE; i++) {
        H_Tm.pData[i] = Hm.pData[i];
    }

    //set K to zeros
    /*for(int i=0; i<N; i++) {
		K[i] = 0.0; //set K to zeros
	}*/

    //if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.stateEstimatorScalarUpdate(): H_Tm = [%f %f %f]\n", H_Tm.pData[0], H_Tm.pData[1], H_Tm.pData[2]);

    static float PHT[N_SIZE];
    static arm_matrix_instance_f32 PHTm = {N_SIZE, 1, (float *)PHT};

    success = success && ARM_MATH_SUCCESS == arm_mat_mult_f32(&Pm, &H_Tm, &PHTm);  //PHT = P * H_T
    if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.stateEstimatorScalarUpdate() 2  P:\n");
    for (int i = 0; i < N_SIZE; i++) {
        if(TDOA_EKF_DEBUG) hal.console->printf("TDOA_EKF  ");
        for (int j = 0; j < N_SIZE; j++) {
            if(TDOA_EKF_DEBUG) hal.console->printf("%f ", Pm.pData[i * N_SIZE + j]);
        }
        if(TDOA_EKF_DEBUG) hal.console->printf("\n");
    }
	if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.stateEstimatorScalarUpdate() 2  P * H_T [%f %f %f]  =  PHT [%f %f %f] \n", H_Tm.pData[0], H_Tm.pData[1], H_Tm.pData[2], PHTm.pData[0], PHTm.pData[1], PHTm.pData[2]);
	if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.stateEstimatorScalarUpdate() 2  P *  H_T [%f %f %f]  =  PHT [%f %f %f] \n", H_Tm.pData[0], H_Tm.pData[1], H_Tm.pData[2], PHTm.pData[0], PHTm.pData[1], PHTm.pData[2]);


    if (nan_value_needs_reset(&PHTm)) {
        /*AP::logger().Write("TDAX", "TimeUS,1,2,3,4,5,6,7,8,9,h1,h2,h3,r1,r2,r3", "Qfffffffffffffff",
				AP_HAL::micros64(), Pm.pData[0], Pm.pData[1], Pm.pData[2], Pm.pData[3], Pm.pData[4], Pm.pData[5], Pm.pData[6], Pm.pData[7], Pm.pData[8],
				H_Tm.pData[0], H_Tm.pData[1], H_Tm.pData[2], PHT[0], PHT[1], PHT[2]
		);*/
        return;
    }

    float r = sq(stdMeasNoise);
    float HPHR = r;  //accumulate here H * P * H_T + r

    for (int i = 0; i < N_SIZE; i++) {             //Add the element of HPH' to the above
        HPHR += Hm.pData[i] * PHTm.pData[i];  //this obviously only works if the update is scalar (as in this function)
    }

    // ====== MEASUREMENT UPDATE ====== //

	if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.stateEstimatorScalarUpdate() 3   HPHR %f   ->   pos should be = X + PHT / HPHR = %f %f %f\n", HPHR,
                    xm.pData[0] + PHTm.pData[0] / HPHR, xm.pData[1] + PHTm.pData[1] / HPHR, xm.pData[2] + PHTm.pData[2] / HPHR);


    for (int i = 0; i < N_SIZE; i++) {            //Add the element of HPH' to the above
        Km.pData[i] = PHTm.pData[i] / HPHR;  // kalman gain = (PH' (HPH' + R ) ^ -1)
        xm.pData[i] += Km.pData[i] * error;  // state update

        if (isnan(xm.pData[i])) {
            needReset = true;
            return;
        }
    }


	if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.stateEstimatorScalarUpdate() 4    pos %f %f %f\n", xm.pData[STATE_X], xm.pData[STATE_Y], xm.pData[STATE_Z]);

    // ====== COVARIANCE UPDATE ====== //
    // (KH - I) * P * (KH - I)'

    static float KH[N_SIZE * N_SIZE];
    static arm_matrix_instance_f32 KHm = {N_SIZE, N_SIZE, (float *)KH};

    static float KH_1[N_SIZE * N_SIZE];
    static arm_matrix_instance_f32 KH_1m = {N_SIZE, N_SIZE, (float *)KH_1};

    static float KH_1_T[N_SIZE * N_SIZE];
    static arm_matrix_instance_f32 KH_1_Tm = {N_SIZE, N_SIZE, (float *)KH_1_T};

    //KH_1 = K * H - I;

    success = success && ARM_MATH_SUCCESS == arm_mat_mult_f32(&Km, &Hm, &KHm);
    success = success && ARM_MATH_SUCCESS == arm_mat_sub_f32(&KHm, &Im, &KH_1m);

    success = success && ARM_MATH_SUCCESS == arm_mat_trans_f32(&KH_1m, &KH_1_Tm);

    static float KH_1_P[N_SIZE * N_SIZE];
    static arm_matrix_instance_f32 KH_1_Pm = {N_SIZE, N_SIZE, (float *)KH_1_P};

    success = success && ARM_MATH_SUCCESS == arm_mat_mult_f32(&KH_1m, &Pm, &KH_1_Pm);

    success = success && ARM_MATH_SUCCESS == arm_mat_mult_f32(&KH_1_Pm, &KH_1_Tm, &Pm);  //	P = KH_1 * P * KH_1t

	if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.stateEstimatorScalarUpdate() 5\n");


    // add the measurement variance and ensure boundedness and symmetry
    for (int i = 0; i < N_SIZE; i++) {
        for (int j = i; j < N_SIZE; j++) {
            float v = Km.pData[i] * r * Km.pData[j];
            float p = 0.5f * Pm.pData[i * N_SIZE + j] + 0.5f * Pm.pData[j * N_SIZE + i] + v;  // add measurement noise

            if (isnan(p) || p > MAX_COVARIANCE) {
                Pm.pData[i * N_SIZE + j] = Pm.pData[j * N_SIZE + i] = MAX_COVARIANCE;
            } else if (i == j && p < MIN_COVARIANCE) {
                Pm.pData[i * N_SIZE + j] = Pm.pData[j * N_SIZE + i] = MIN_COVARIANCE;
            } else {
                Pm.pData[i * N_SIZE + j] = Pm.pData[j * N_SIZE + i] = p;
            }
        }
    }

#endif
}

void TDOA_EKF::resetFilter() {
    initialize_filter();

	if(TDOA_EKF_DEBUG) hal.console->printf("\n TDOA_EKF.resetFilter() ++++++++++++++++++++++++\n");

    needReset = false;
}

void TDOA_EKF::initialize_filter() {
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL

    //COVARIANCE P
    for (int i = 0; i < N_SIZE; i++) {
        for (int j = 0; j < N_SIZE; j++) {
            Pm.pData[i * N_SIZE + j] = 0.0;
        }
    }

    Pm.pData[STATE_X * N_SIZE + STATE_X] = Pm.pData[STATE_Y * N_SIZE + STATE_Y] = INITIAL_POS_VARIANCE_XY;
    Pm.pData[STATE_Z * N_SIZE + STATE_Z] = INITIAL_POS_VARIANCE_Z;

    //STATE
    //initialize position
    //Vector3f initialPositionGuess(1,1,1); //TODO input as function parameter
    xm.pData[STATE_X] = INITIAL_POS_STATE.x;
    xm.pData[STATE_Y] = INITIAL_POS_STATE.y;
    xm.pData[STATE_Z] = INITIAL_POS_STATE.z;

#endif
}

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL

bool TDOA_EKF::contains_nan(arm_matrix_instance_f32 *m) {
    for (int i = 0; i < m->numRows; i++) {
        for (int j = 0; j < m->numCols; j++) {
            if (isnan(m->pData[i * m->numCols + j])) {
                return true;
            }
        }
    }

    return false;
}

bool TDOA_EKF::nan_value_needs_reset(arm_matrix_instance_f32 *m) {
    needReset = contains_nan(m);

    return needReset;
}

#endif
