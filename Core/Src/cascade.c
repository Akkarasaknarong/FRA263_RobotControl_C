/*
 * cascade.c
 *
 *  Created on: May 28, 2026
 *      Author: Akkarasaknarong
 */

#include "cascade.h"

static float error_pos_prev = 0.0f;
static float error_pos_sum  = 0.0f;
static float error_vel_prev = 0.0f;
static float error_vel_sum  = 0.0f;
static float PID_pos        = 0.0f;
float Gain_R = 1.0f;
float Gain_L = 1.0f;

typedef struct {
	float kp_pos;
	float kd_pos ;
	float ki_pos ;
	float kp_vel;
	float kd_vel ;
	float ki_vel ;
} PIDParam_t;
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
    float P_pos = PIDparam.kp_pos * error_pos;
    float I_pos = PIDparam.ki_pos * error_pos_sum;
    float D_pos = PIDparam.kd_pos * (error_pos - error_pos_prev) / POSITION_CONTROL_FREQ;
    PID_pos = P_pos + I_pos + D_pos;
    error_pos_sum += error_pos * POSITION_CONTROL_FREQ;
    error_pos_prev = error_pos;
}

void Vel_ctrl_Compute(float ref_vel, float cur_vel, float *PWM_PID_out) {
    float error_vel = PID_pos + ref_vel - cur_vel;
    float P_vel = PIDparam.kp_vel * error_vel;
    float I_vel = PIDparam.ki_vel * error_vel_sum;
    float D_vel = PIDparam.kd_vel * (error_vel - error_vel_prev) / VELOCITY_CONTROL_FREQ;
    error_vel_sum += error_vel * VELOCITY_CONTROL_FREQ;
    error_vel_prev = error_vel;

    float Output = P_vel + I_vel + D_vel ;

    if (Output > 0){ // Rotate Left
    	Output = Output * Gain_L ;
    }
    else if (Output < 0) { // Rotate Right
		Output = Output * Gain_R ;
	}
    else if (Output == 0) { // Rotate Right
		Output = Output;
	}

    *PWM_PID_out = Output;
}

