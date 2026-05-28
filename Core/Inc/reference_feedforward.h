/*
 * refferene_ffw.h
 *
 *  Created on: May 27, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_REFERENCE_FEEDFORWARD_H_
#define INC_REFERENCE_FEEDFORWARD_H_

void Motor_Ref_feedforward_Init();
void Motor_Ref_feedforward_Update(float ref_vel, float *V);

#endif /* INC_REFERENCE_FEEDFORWARD_H_ */
