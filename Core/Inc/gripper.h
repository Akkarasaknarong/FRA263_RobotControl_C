/*
 * gripper.h
 *
 *  Created on: May 30, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_GRIPPER_H_
#define INC_GRIPPER_H_
#include "datatype.h"

void Gripper_Group(int up, int down, int close, int open);
void Gripper_Control(int _Command);

#endif /* INC_GRIPPER_H_ */
