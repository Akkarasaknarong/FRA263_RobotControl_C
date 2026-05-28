/*
 * kalman_filter.c
 *
 *  Created on: May 27, 2026
 *      Author: Akkarasaknarong
 */

#include "kalman_filter.h"

// Define Matrix -------------------------------------
static float32_t qpos = 0;
static float32_t qvel = 0;
static float32_t qload = 0;
static float32_t qi = 0;
static float32_t r_noise = 0.0f;

static float32_t A_f32[16] = {
		1.0000, 0.0002, -0.0000, 0.0001,
		0,      0.8493, -0.1923, 0.5235,
		0,      0,       1.0000, 0,
		0,     -0.4704, 0.0502, 0.7254 };
static float32_t B_f32[4] = {
		0.0000,
		0.0250,
		0,
		0.0802 };
static float32_t C_f32[4] = {
		1.0000, 0, 0, 0 };
static float32_t Q_f32[16] = {0};
static float32_t W_f32[4] = {
		0,
		0,
		1,
		0 };
static float32_t X_f32[4] = {0};
static float32_t X_prev_f32[4] = {0};
static float32_t P_f32[16] =
		{1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1};
static float32_t P_prev_f32[16] =
		{1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1};
static float32_t At_f32[16] ;
static float32_t Ct_f32[4];
// X_pred_f32
static float32_t AmX_prev_f32[16];
static float32_t BmV_f32[4];
static float32_t X_pred_f32[4];
// P_pred_f32
static float32_t AmP_prev_f32[16];
static float32_t AmPm_prevmAt_f32[16];
static float32_t P_pred_f32[16];
// K_f32
static float32_t PmCt_f32[4];
static float32_t CmPmCt_f32[1];
static float32_t K_f32[4];
// error
static float32_t Cx_pred_f32[1];
static float32_t kalman_error = 0.0f;
// X_est & P_est
static float32_t Kmerror_f32[4];
static float32_t Eye4_f32[16] = {
		1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1};
static float32_t KmC_f32[16];
static float32_t Eye4mKmC_f32[16];
// ----------------------------------------------------

// Mapping Matrix -------------------------------------
static arm_matrix_instance_f32 A;
static arm_matrix_instance_f32 B;
static arm_matrix_instance_f32 C;
static arm_matrix_instance_f32 Q;
static arm_matrix_instance_f32 W;
static arm_matrix_instance_f32 X;
static arm_matrix_instance_f32 X_prev;
static arm_matrix_instance_f32 P;
static arm_matrix_instance_f32 P_prev;

static arm_matrix_instance_f32 At;
static arm_matrix_instance_f32 Ct;

// X_pred
static arm_matrix_instance_f32 AmX_prev;
static arm_matrix_instance_f32 BmV;
static arm_matrix_instance_f32 X_pred;
// P_pred
static arm_matrix_instance_f32 AmP_prev;
static arm_matrix_instance_f32 AmPm_prevmAt;
static arm_matrix_instance_f32 P_pred;
// K
static arm_matrix_instance_f32 PmCt;
static arm_matrix_instance_f32 CmPmCt;
static arm_matrix_instance_f32 K_mat;
// error
static arm_matrix_instance_f32 Cx_pred_mat;
// X_est & P_est
static arm_matrix_instance_f32 Kmerror_mat;
static arm_matrix_instance_f32 Eye4_mat;
static arm_matrix_instance_f32 KmC_mat;
static arm_matrix_instance_f32 Eye4mKmC_mat;
// ----------------------------------------------------

void Motor_Kalman_Init() {
	Q_f32[0] = qpos;
	Q_f32[5] = qvel;
	Q_f32[10] = qload;
	Q_f32[15] = qi;

	arm_mat_init_f32(&A, 4, 4, (float32_t*) &A_f32);
	arm_mat_init_f32(&B, 4, 1, (float32_t*) &B_f32);
	arm_mat_init_f32(&C, 1, 4, (float32_t*) &C_f32);
	arm_mat_init_f32(&Q, 4, 4, (float32_t*) &Q_f32);
	arm_mat_init_f32(&W, 4, 1, (float32_t*) &W_f32);
	arm_mat_init_f32(&X, 4, 1, (float32_t*) &X_f32);
	arm_mat_init_f32(&X_prev, 4, 1, (float32_t*) &X_prev_f32);
	arm_mat_init_f32(&P, 4, 4, (float32_t*) &P_f32);
	arm_mat_init_f32(&P_prev, 4, 4, (float32_t*) &P_prev_f32);
	arm_mat_init_f32(&At, 4, 4, (float32_t*) &At_f32);
	arm_mat_init_f32(&Ct, 4, 1, (float32_t*) &Ct_f32);
	arm_mat_trans_f32(&A,&At);
	arm_mat_trans_f32(&C,&Ct);

	// X_pred
	arm_mat_init_f32(&AmX_prev, 4, 1, (float32_t*) &AmX_prev_f32);
	arm_mat_init_f32(&BmV, 4, 1, (float32_t*) &BmV_f32);
	arm_mat_init_f32(&X_pred, 4, 1, (float32_t*) &X_pred_f32);
	// P_pred
	arm_mat_init_f32(&AmP_prev, 4, 4, (float32_t*) &AmP_prev_f32);
	arm_mat_init_f32(&AmPm_prevmAt, 4, 4, (float32_t*) &AmPm_prevmAt_f32);
	arm_mat_init_f32(&P_pred, 4, 4, (float32_t*) &P_pred_f32);
	// K
	arm_mat_init_f32(&PmCt, 4, 1, (float32_t*) &PmCt_f32);
	arm_mat_init_f32(&CmPmCt, 1, 1, (float32_t*) &CmPmCt_f32);
	arm_mat_init_f32(&K_mat, 4, 1, (float32_t*) &K_f32);
	// error
	arm_mat_init_f32(&Cx_pred_mat, 1, 1, (float32_t*) &Cx_pred_f32);
	// X_est & P_est
	arm_mat_init_f32(&Kmerror_mat, 4, 1, (float32_t*) &Kmerror_f32);
	arm_mat_init_f32(&Eye4_mat, 4, 4, (float32_t*) &Eye4_f32);
	arm_mat_init_f32(&KmC_mat, 4, 4, (float32_t*) &KmC_f32);
	arm_mat_init_f32(&Eye4mKmC_mat, 4, 4, (float32_t*) &Eye4mKmC_f32);
}

void Motor_Kalman_Tunning(float32_t q_pos, float32_t q_vel, float32_t q_load , float32_t q_i, float32_t r){
	Q_f32[0]  = q_pos;
	Q_f32[5]  = q_vel;
	Q_f32[10] = q_load;
	Q_f32[15] = q_i;
	r_noise = r ;
}

void Motor_Kalman_Update(float32_t v, float32_t z,
		float32_t *q_est,float32_t *qd_est, float32_t *load_est, float32_t *i_est) {
	// State 1: Prediction state
	// x_pred = (Ad * x_prev) + (Bd * v)
	arm_mat_mult_f32(&A, &X_prev, &AmX_prev);
	arm_mat_scale_f32(&B, v, &BmV);
	arm_mat_add_f32(&AmX_prev, &BmV, &X_pred);

	// p_pred = (Ad*p_prev*Ad') + Qd
	arm_mat_mult_f32(&A, &P_prev, &AmP_prev);
	arm_mat_mult_f32(&AmP_prev, &At, &AmPm_prevmAt);
	arm_mat_add_f32(&AmPm_prevmAt, &Q, &P_pred);

	// State 2: Correction State
	// K = (p_pred * Cd') / (Cd * p_pred * Cd' + r)
	arm_mat_mult_f32(&P_pred, &Ct, &PmCt);
	arm_mat_mult_f32(&C, &PmCt, &CmPmCt);
	float32_t denominator = CmPmCt_f32[0] + r_noise;
	float32_t inv_denominator = 1.0f / denominator;
	arm_scale_f32(PmCt_f32, inv_denominator, K_f32, 4);

	// error = z - Cd * x_pred
	arm_mat_mult_f32(&C, &X_pred, &Cx_pred_mat);
	kalman_error = z - Cx_pred_f32[0];

	// x_est = x_pred + (K * error)
	arm_scale_f32(K_f32, kalman_error, Kmerror_f32, 4);
	arm_mat_add_f32(&X_pred, &Kmerror_mat, &X_prev);

	// p_est = (eye(4) - K*Cd)*p_pred
	arm_mat_mult_f32(&K_mat, &C, &KmC_mat);
	arm_mat_sub_f32(&Eye4_mat, &KmC_mat, &Eye4mKmC_mat);
	arm_mat_mult_f32(&Eye4mKmC_mat, &P_pred, &P_prev);

	*q_est = X_prev_f32[0]; // Position
	*qd_est = X_prev_f32[1]; // Velocity
	*load_est = X_prev_f32[2]; // Load
	*i_est = X_prev_f32[3]; // Current
}


