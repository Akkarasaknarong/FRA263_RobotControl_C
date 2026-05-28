/*
 * disturbance_feedforward.h
 *
 *  Created on: May 28, 2026
 *      Author: Akkarasaknarong
 */

#include "disturbance_feedforward.h"

static float motor_R = 1.59;
static float motor_Eff = 0.49833;
static float motor_Ke = 0.081664;

void Motor_Disturbance_feedforward_Update(float ref_load, float *_V_DFFW){
	*_V_DFFW = (motor_R / (motor_Ke * motor_Eff))*ref_load;
}

