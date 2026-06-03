/*
 * gripper.c
 *
 *  Created on: May 30, 2026
 *      Author: Akkarasaknarong
 */

#include "gripper.h"

void Gripper_Group(int up, int down, int close, int open) {
	HAL_GPIO_WritePin(Gripper_Up_GPIO_Port, Gripper_Up_Pin, up);
	HAL_GPIO_WritePin(Gripper_Down_GPIO_Port, Gripper_Down_Pin, down);
	HAL_GPIO_WritePin(Gripper_Close_GPIO_Port, Gripper_Close_Pin, close);
	HAL_GPIO_WritePin(Gripper_Open_GPIO_Port, Gripper_Open_Pin, open);
}

void Gripper_Control(int _Command) {
	// 0 : Place
	// 1 : Pick
	// -1 : Reset / Idle

	static int prev_command = -1;
	static uint32_t start_time = 0;

	if (_Command != prev_command) {
		start_time = HAL_GetTick();
		prev_command = _Command;
	}

	float internal_time = (HAL_GetTick() - start_time) / 1000.0f;

	if (_Command == 0) {
		if (internal_time <= 0.8f) {
			Gripper_Group(0, 1, 0, 0);
		} else if (internal_time <= 1.2f) {
			Gripper_Group(0, 0, 0, 1);
		} else {
			Gripper_Group(1, 0, 0, 1);
		}
	}

	else if (_Command == 1) {
		if (internal_time <= 0.8f) {
			Gripper_Group(0, 1, 0, 0);
		} else if (internal_time <= 1.2f) {
			Gripper_Group(0, 0, 1, 0);
		} else  {
			Gripper_Group(1, 0, 1, 0);
		}
	}
	else {
		prev_command = -1;
	}
}


