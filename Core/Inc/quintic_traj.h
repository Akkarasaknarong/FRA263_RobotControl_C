/*
 * quinticTraj.h
 *
 *  Created on: May 28, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_QUINTIC_TRAJ_H_
#define INC_QUINTIC_TRAJ_H_

void QuinticTraj_Init(float start_point, float end_point , float time);
void QuinticTraj_Compute(float curr_time ,float* q ,float* qd ,float* qdd );

#endif /* INC_QUINTIC_TRAJ_H_ */
