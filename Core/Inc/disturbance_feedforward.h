/*
 * disturbance_feedforward.h
 *
 *  Created on: May 28, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_DISTURBANCE_FEEDFORWARD_H_
#define INC_DISTURBANCE_FEEDFORWARD_H_

void Motor_Disturbance_feedforward_Init();
void Motor_Disturbance_feedforward_Update(float ref_load, float *_V_DFFW);

#endif /* INC_DISTURBANCE_FEEDFORWARD_H_ */
