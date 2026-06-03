/*
 * quinticTraj.h
 *
 *  Created on: May 28, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_QUINTIC_TRAJ_H_
#define INC_QUINTIC_TRAJ_H_

#include "datatype.h"
extern refTarget_t REFdata ;

uint8_t QuinticTraj_P2P(float q_start, float q_final, float total_time, float current_time);

#endif /* INC_QUINTIC_TRAJ_H_ */
