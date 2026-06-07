/*
 * cascade.c
 *
 *  Created on: May 28, 2026
 *      Author: Akkarasaknarong
 */

#include "cascade.h"
extern refTarget_t REFdata ;
extern QEIstruct_t QEIdata ;
extern KALMANstruct_t ESTdata;
extern PIDDebug_t PIDDebug;

#define PID_LUT_STEP     5.0f
#define PID_LUT_SIZE     73    // 0-360 deg ทุก 5 องศา
#define PID_TIME_SLOW    3.25f
#define PID_TIME_FAST    1.25f

typedef struct {
    float kp_pos;
    float ki_pos;
    float kd_pos;
    float kp_vel;
    float ki_vel;
    float kd_vel;
} PID_LUT_Entry_t;

static const PID_LUT_Entry_t PID_LUT_SLOW[PID_LUT_SIZE] = {
//   kp_pos  ki_pos  kd_pos  kp_vel  ki_vel  kd_vel
    {800,    50,     0,      400,    1000,   0},  // [0]  0-4 deg
    {800,    50,     0,      400,    1000,   0},  // [1]  5-9 deg
    {800,    50,     0,      400,    1000,   0},  // [2]  10-14 deg
    {800,    50,     0,      400,    1000,   0},  // [3]  15-19 deg
    {800,    50,     0,      400,    1000,   0},  // [4]  20-24 deg
    {800,    50,     0,      400,    1000,   0},  // [5]  25-29 deg
    {800,    50,     0,      400,    1000,   0},  // [6]  30-34 deg
    {800,    50,     0,      400,    1000,   0},  // [7]  35-39 deg
    {800,    50,     0,      400,    1000,   0},  // [8]  40-44 deg
    {800,    50,     0,      400,    1000,   0},  // [9]  45-49 deg
    {800,    50,     0,      400,    1000,   0},  // [10] 50-54 deg
    {800,    50,     0,      400,    1000,   0},  // [11] 55-59 deg
    {800,    50,     0,      400,    1000,   0},  // [12] 60-64 deg
    {800,    50,     0,      400,    1000,   0},  // [13] 65-69 deg
    {800,    50,     0,      400,    1000,   0},  // [14] 70-74 deg
    {800,    50,     0,      400,    1000,   0},  // [15] 75-79 deg
    {800,    50,     0,      400,    1000,   0},  // [16] 80-84 deg
    {800,    50,     0,      400,    1000,   0},  // [17] 85-89 deg
    {800,    50,     0,      400,    1000,   0},  // [18] 90-94 deg
    {800,    50,     0,      400,    1000,   0},  // [19] 95-99 deg
    {800,    50,     0,      400,    1000,   0},  // [20] 100-104 deg
    {800,    50,     0,      400,    1000,   0},  // [21] 105-109 deg
    {800,    50,     0,      400,    1000,   0},  // [22] 110-114 deg
    {800,    50,     0,      400,    1000,   0},  // [23] 115-119 deg
    {800,    50,     0,      400,    1000,   0},  // [24] 120-124 deg
    {800,    50,     0,      400,    1000,   0},  // [25] 125-129 deg
    {800,    50,     0,      400,    1000,   0},  // [26] 130-134 deg
    {800,    50,     0,      400,    1000,   0},  // [27] 135-139 deg
    {800,    50,     0,      400,    1000,   0},  // [28] 140-144 deg
    {800,    50,     0,      400,    1000,   0},  // [29] 145-149 deg
    {800,    50,     0,      400,    1000,   0},  // [30] 150-154 deg
    {800,    50,     0,      400,    1000,   0},  // [31] 155-159 deg
    {800,    50,     0,      400,    1000,   0},  // [32] 160-164 deg
    {800,    50,     0,      400,    1000,   0},  // [33] 165-169 deg
    {800,    50,     0,      400,    1000,   0},  // [34] 170-174 deg
    {800,    50,     0,      400,    1000,   0},  // [35] 175-179 deg
    {800,    50,     0,      400,    200,   0},  // [36] 180-184 deg
    {800,    50,     0,      400,    200,   0},  // [37] 185-189 deg
    {800,    50,     0,      400,    500,   0},  // [38] 190-194 deg
    {800,    50,     0,      400,    500,   0},  // [39] 195-199 deg
    {800,    50,     0,      400,    500,   0},  // [40] 200-204 deg
    {800,    50,     0,      400,    500,   0},  // [41] 205-209 deg
    {800,    50,     0,      400,    500,   0},  // [42] 210-214 deg
    {800,    50,     0,      400,    500,   0},  // [43] 215-219 deg
    {800,    50,     0,      400,    1000,   0},  // [44] 220-224 deg
    {800,    50,     0,      400,    1000,   0},  // [45] 225-229 deg
    {800,    50,     0,      400,    1000,   0},  // [46] 230-234 deg
    {800,    50,     0,      400,    1000,   0},  // [47] 235-239 deg
    {800,    50,     0,      400,    1000,   0},  // [48] 240-244 deg
    {800,    50,     0,      400,    1000,   0},  // [49] 245-249 deg
    {800,    50,     0,      400,    1000,   0},  // [50] 250-254 deg
    {800,    50,     0,      400,    1000,   0},  // [51] 255-259 deg
    {800,    50,     0,      400,    1000,   0},  // [52] 260-264 deg
    {800,    50,     0,      400,    1000,   0},  // [53] 265-269 deg
    {800,    50,     0,      400,    1000,   0},  // [54] 270-274 deg
    {800,    50,     0,      400,    1000,   0},  // [55] 275-279 deg
    {800,    50,     0,      400,    1000,   0},  // [56] 280-284 deg
    {800,    50,     0,      400,    1000,   0},  // [57] 285-289 deg
    {800,    50,     0,      400,    1000,   0},  // [58] 290-294 deg
    {800,    50,     0,      400,    1000,   0},  // [59] 295-299 deg
    {800,    50,     0,      400,    1000,   0},  // [60] 300-304 deg
    {800,    50,     0,      400,    1000,   0},  // [61] 305-309 deg
    {800,    50,     0,      400,    1000,   0},  // [62] 310-314 deg
    {800,    50,     0,      400,    1000,   0},  // [63] 315-319 deg
    {800,    50,     0,      400,    1000,   0},  // [64] 320-324 deg
    {800,    50,     0,      400,    1000,   0},  // [65] 325-329 deg
    {800,    50,     0,      400,    1000,   0},  // [66] 330-334 deg
    {800,    50,     0,      400,    1000,   0},  // [67] 335-339 deg
    {800,    50,     0,      400,    1000,   0},  // [68] 340-344 deg
    {800,    50,     0,      400,    1000,   0},  // [69] 345-349 deg
    {800,    50,     0,      400,    1000,   0},  // [70] 350-354 deg
    {800,    50,     0,      400,    1000,   0},  // [71] 355-359 deg
    {800,    50,     0,      400,    1000,   0},  // [72] 360 deg
};

static const PID_LUT_Entry_t PID_LUT_FAST[PID_LUT_SIZE] = {
//   kp_pos  ki_pos  kd_pos  kp_vel  ki_vel  kd_vel
    {800,    50,     0,      400,    1000,   0},  // [0]  0-4 deg
    {800,    50,     0,      400,    1000,   0},  // [1]  5-9 deg
    {800,    50,     0,      400,    1000,   0},  // [2]  10-14 deg
    {800,    50,     0,      400,    1000,   0},  // [3]  15-19 deg
    {800,    50,     0,      400,    1000,   0},  // [4]  20-24 deg
    {800,    50,     0,      400,    1000,   0},  // [5]  25-29 deg
    {800,    50,     0,      400,    1000,   0},  // [6]  30-34 deg
    {800,    50,     0,      400,    1000,   0},  // [7]  35-39 deg
    {800,    50,     0,      400,    1000,   0},  // [8]  40-44 deg
    {800,    50,     0,      400,    1000,   0},  // [9]  45-49 deg
    {800,    50,     0,      400,    1000,   0},  // [10] 50-54 deg
    {800,    50,     0,      400,    1000,   0},  // [11] 55-59 deg
    {800,    50,     0,      400,    1000,   0},  // [12] 60-64 deg
    {800,    50,     0,      400,    1000,   0},  // [13] 65-69 deg
    {800,    50,     0,      400,    1000,   0},  // [14] 70-74 deg
    {800,    50,     0,      400,    1000,   0},  // [15] 75-79 deg
    {800,    50,     0,      400,    1000,   0},  // [16] 80-84 deg
    {800,    50,     0,      400,    1000,   0},  // [17] 85-89 deg
    {800,    50,     0,      400,    1000,   0},  // [18] 90-94 deg
    {800,    50,     0,      400,    1000,   0},  // [19] 95-99 deg
    {800,    50,     0,      400,    1000,   0},  // [20] 100-104 deg
    {800,    50,     0,      400,    1000,   0},  // [21] 105-109 deg
    {800,    50,     0,      400,    1000,   0},  // [22] 110-114 deg
    {800,    50,     0,      400,    1000,   0},  // [23] 115-119 deg
    {800,    50,     0,      400,    1000,   0},  // [24] 120-124 deg
    {800,    50,     0,      400,    1000,   0},  // [25] 125-129 deg
    {800,    50,     0,      400,    1000,   0},  // [26] 130-134 deg
    {800,    50,     0,      400,    1000,   0},  // [27] 135-139 deg
    {800,    50,     0,      400,    1000,   0},  // [28] 140-144 deg
    {800,    50,     0,      400,    1000,   0},  // [29] 145-149 deg
    {800,    50,     0,      400,    1000,   0},  // [30] 150-154 deg
    {800,    50,     0,      400,    1000,   0},  // [31] 155-159 deg
    {800,    50,     0,      400,    1000,   0},  // [32] 160-164 deg
    {800,    50,     0,      400,    1000,   0},  // [33] 165-169 deg
    {800,    50,     0,      400,    1000,   0},  // [34] 170-174 deg
    {800,    50,     0,      400,    1000,   0},  // [35] 175-179 deg
    {800,    50,     0,      400,    1000,   0},  // [36] 180-184 deg
    {800,    50,     0,      400,    1000,   0},  // [37] 185-189 deg
    {800,    50,     0,      400,    1000,   0},  // [38] 190-194 deg
    {800,    50,     0,      400,    1000,   0},  // [39] 195-199 deg
    {800,    50,     0,      400,    1000,   0},  // [40] 200-204 deg
    {800,    50,     0,      400,    1000,   0},  // [41] 205-209 deg
    {800,    50,     0,      400,    1000,   0},  // [42] 210-214 deg
    {800,    50,     0,      400,    1000,   0},  // [43] 215-219 deg
    {800,    50,     0,      400,    1000,   0},  // [44] 220-224 deg
    {800,    50,     0,      400,    1000,   0},  // [45] 225-229 deg
    {800,    50,     0,      400,    1000,   0},  // [46] 230-234 deg
    {800,    50,     0,      400,    1000,   0},  // [47] 235-239 deg
    {800,    50,     0,      400,    1000,   0},  // [48] 240-244 deg
    {800,    50,     0,      400,    1000,   0},  // [49] 245-249 deg
    {800,    50,     0,      400,    1000,   0},  // [50] 250-254 deg
    {800,    50,     0,      400,    1000,   0},  // [51] 255-259 deg
    {800,    50,     0,      400,    1000,   0},  // [52] 260-264 deg
    {800,    50,     0,      400,    1000,   0},  // [53] 265-269 deg
    {800,    50,     0,      400,    1000,   0},  // [54] 270-274 deg
    {800,    50,     0,      400,    1000,   0},  // [55] 275-279 deg
    {800,    50,     0,      400,    1000,   0},  // [56] 280-284 deg
    {800,    50,     0,      400,    1000,   0},  // [57] 285-289 deg
    {800,    50,     0,      400,    1000,   0},  // [58] 290-294 deg
    {800,    50,     0,      400,    1000,   0},  // [59] 295-299 deg
    {800,    50,     0,      400,    1000,   0},  // [60] 300-304 deg
    {800,    50,     0,      400,    1000,   0},  // [61] 305-309 deg
    {800,    50,     0,      400,    1000,   0},  // [62] 310-314 deg
    {800,    50,     0,      400,    1000,   0},  // [63] 315-319 deg
    {800,    50,     0,      400,    1000,   0},  // [64] 320-324 deg
    {800,    50,     0,      400,    1000,   0},  // [65] 325-329 deg
    {800,    50,     0,      400,    1000,   0},  // [66] 330-334 deg
    {800,    50,     0,      400,    1000,   0},  // [67] 335-339 deg
    {800,    50,     0,      400,    1000,   0},  // [68] 340-344 deg
    {800,    50,     0,      400,    1000,   0},  // [69] 345-349 deg
    {800,    50,     0,      400,    1000,   0},  // [70] 350-354 deg
    {800,    50,     0,      400,    1000,   0},  // [71] 355-359 deg
    {800,    50,     0,      400,    1000,   0},  // [72] 360 deg
};

void PID_LUT_Apply(float distance_deg, float time_sec) {
    if (distance_deg < 0)   distance_deg = 0;
    if (distance_deg > 360) distance_deg = 360;

    int idx = (int)(distance_deg / PID_LUT_STEP);
    if (idx >= PID_LUT_SIZE) idx = PID_LUT_SIZE - 1;

    const PID_LUT_Entry_t *e;

    if (time_sec <= 1.25f) {
        e = &PID_LUT_FAST[idx];
    } else {
        e = &PID_LUT_SLOW[idx];
    }

    Pos_ctrl_Tunning(e->kp_pos, e->ki_pos, e->kd_pos);
    Vel_ctrl_Tunning(e->kp_vel, e->ki_vel, e->kd_vel);
}

static float error_pos_prev = 0.0f;
static float error_pos_sum  = 0.0f;
static float error_vel_prev = 0.0f;
static float error_vel_sum  = 0.0f;
static float PID_pos        = 0.0f;
float Gain_R = 1.0f;
float Gain_L = 1.0f;

static PIDParam_t PIDparam = {0};

void Pos_ctrl_Init() {
	error_pos_prev = 0;
}
void Vel_ctrl_Init() {
	error_vel_prev = 0;
}

void Pos_ctrl_Tunning(float p_pos, float i_pos, float d_pos) {
    PIDparam.kp_pos = p_pos;
    PIDparam.ki_pos = i_pos;
    PIDparam.kd_pos = d_pos;
}
void Vel_ctrl_Tunning(float p_vel, float i_vel, float d_vel) {
    PIDparam.kp_vel = p_vel;
    PIDparam.ki_vel = i_vel;
    PIDparam.kd_vel = d_vel;
}

void Pos_ctrl_Compute(float ref_pos, float cur_pos) {
    float error_pos = ref_pos - cur_pos;

    if ((error_pos > 0 && error_pos_prev < 0) ||
		(error_pos < 0 && error_pos_prev > 0)) {
		error_pos_sum = 0.0f;
	}

    float P_pos = PIDparam.kp_pos * error_pos;
    float I_pos = PIDparam.ki_pos * error_pos_sum;
    float D_pos = PIDparam.kd_pos * (error_pos - error_pos_prev) / POSITION_CONTROL_FREQ;
    PID_pos = P_pos + I_pos + D_pos;

    error_pos_sum += error_pos * POSITION_CONTROL_FREQ;
    // Anti-windup
	if      (error_pos_sum >  I_POS_LIMIT) error_pos_sum =  I_POS_LIMIT;
	else if (error_pos_sum < -I_POS_LIMIT) error_pos_sum = -I_POS_LIMIT;

    error_pos_prev = error_pos;

    PIDDebug.pid_pos = PID_pos ;
    PIDDebug.p_pos = P_pos ;
    PIDDebug.i_pos = I_pos ;
    PIDDebug.d_pos = D_pos ;
}

void Vel_ctrl_Compute(float ref_vel, float cur_vel, float *PWM_PID_out) {
    if      (PID_pos >  MAX_VELOCITY) PID_pos =  MAX_VELOCITY;
    else if (PID_pos < -MAX_VELOCITY) PID_pos = -MAX_VELOCITY;

    float error_vel = PID_pos + ref_vel - cur_vel;

    if ((error_vel > 0 && error_vel_prev < 0) ||
        (error_vel < 0 && error_vel_prev > 0)) {
        error_vel_sum = 0.0f;
    }

    float P_vel = PIDparam.kp_vel * error_vel;
    float I_vel = PIDparam.ki_vel * error_vel_sum;
    float D_vel = PIDparam.kd_vel * (error_vel - error_vel_prev) / VELOCITY_CONTROL_FREQ;

    float pos_error   = REFdata.ref_q_deg - QEIdata.q_deg;

    float cur_vel_abs = fabsf(ESTdata.qd_est);

    uint8_t joint_stuck = (cur_vel_abs < FFW_VEL_GATE) &&
                          (fabsf(pos_error) < FFW_MAX_DEG);

     if (!joint_stuck) {
         error_vel_sum += error_vel * VELOCITY_CONTROL_FREQ;
     }

    error_vel_sum += error_vel * VELOCITY_CONTROL_FREQ;

    if      (error_vel_sum >  I_VEL_LIMIT) error_vel_sum =  I_VEL_LIMIT;
    else if (error_vel_sum < -I_VEL_LIMIT) error_vel_sum = -I_VEL_LIMIT;

    error_vel_prev = error_vel;

    float Output = P_vel + I_vel + D_vel;
    if      (Output > 0) Output = Output * Gain_L;
    else if (Output < 0) Output = Output * Gain_R;

    float pwm_friction_ffw = 0.0f;
    if (cur_vel_abs < FFW_VEL_GATE) {
        if (pos_error > FFW_DEAD_DEG && pos_error < FFW_MAX_DEG) {
            float ratio = (pos_error - FFW_DEAD_DEG) / (FFW_MAX_DEG - FFW_DEAD_DEG);
            pwm_friction_ffw = PWM_FRICTION_COMP * ratio;
        }
        else if (pos_error < -FFW_DEAD_DEG && pos_error > -FFW_MAX_DEG) {
            float ratio = (-pos_error - FFW_DEAD_DEG) / (FFW_MAX_DEG - FFW_DEAD_DEG);
            pwm_friction_ffw = -PWM_FRICTION_COMP * ratio;
        }
    }

    *PWM_PID_out = Output + pwm_friction_ffw;

    PIDDebug.pid_vel    = Output;
    PIDDebug.p_vel      = P_vel;
    PIDDebug.i_vel      = I_vel;
    PIDDebug.d_vel      = D_vel;
    PIDDebug.friction_ffw = pwm_friction_ffw;
}
