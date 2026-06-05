/*
 * cascade.h
 *
 *  Created on: May 27, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_CASCADE_H_
#define INC_CASCADE_H_

#define POSITION_CONTROL_FREQ 0.001 // 1000Hz
#define VELOCITY_CONTROL_FREQ 0.0004 // 2500Hz
#define MAX_VELOCITY 3.8 // approximately
#define I_POS_LIMIT      4.0f    // rad/s
#define I_VEL_LIMIT      20000.0f  // PWM

extern float Gain_L ;
extern float Gain_R ;

void Pos_ctrl_Init();
void Pos_ctrl_Tunning(float p_pos , float i_pos, float d_pos);
void Pos_ctrl_Compute(float ref_pos , float cur_pos);
void Vel_ctrl_Init();
void Vel_ctrl_Tunning(float p_vel , float i_vel, float d_vel);
void Vel_ctrl_Compute(float ref_vel, float cur_vel, float *PWM_PID_out);

#endif /* INC_CASCADE_H_ */
