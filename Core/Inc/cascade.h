/*
 * cascade.h
 *
 *  Created on: May 27, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_CASCADE_H_
#define INC_CASCADE_H_

#define POSITION_CONTROL_FREQ 0.004f // 250Hz
#define VELOCITY_CONTROL_FREQ 0.0004 // 2500Hz
#define MAX_VELOCITY 4.4 // approximately
#define I_POS_LIMIT     0.01f    // rad/s
#define I_VEL_LIMIT      6500.0f  // PWM

#define PWM_FRICTION_COMP  15000 // PWM
#define FFW_VEL_GATE 0.25
#define FFW_DEAD_DEG 0.05
#define FFW_MAX_DEG 1.8

#include "datatype.h"

extern float Gain_L ;
extern float Gain_R ;
extern refTarget_t REFdata ;

void Pos_ctrl_Init();
void Pos_ctrl_Tunning(float p_pos , float i_pos, float d_pos);
void Pos_ctrl_Compute(float ref_pos , float cur_pos);
void Vel_ctrl_Init();
void Vel_ctrl_Tunning(float p_vel , float i_vel, float d_vel);
void Vel_ctrl_Compute(float ref_vel, float cur_vel, float *PWM_PID_out);
void PID_LUT_Apply(float distance_deg, float time_sec);

#endif /* INC_CASCADE_H_ */
