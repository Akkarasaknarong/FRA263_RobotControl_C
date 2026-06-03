/*
 * Robot_Run.c
 *
 *  Created on: Jun 1, 2026
 *      Author: Akkarasaknarong
 */

#include "basesystem_decode.h"
#include "robot_worker.h"
#include "gripper.h"
#include "cascade.h"

uint8_t Non_Blocking_Timer(uint32_t wait_time) {
	static uint32_t start_time = 0;
	static uint8_t is_running = 0;
	if (wait_time == 0) {
		is_running = 0;
		return 1;
	}
	if (is_running == 0) {
		start_time = HAL_GetTick();
		is_running = 1;
		return 0;
	}
	if (HAL_GetTick() - start_time >= wait_time) {
		is_running = 0;
		return 1;
	}
	return 0;
}

void Auto_Worker(){

}
void Manual_Worker() {
	uint32_t required_time = 0;
	// Jogging
	if (Robot.Manualmode_data.Jog_value != 0){

	}

	// Gripper
	else if (Robot.Manualmode_data.gripper_sequence == Pick) {
		Gripper_Control(1);
		required_time = 2000;
	}
	else if (Robot.Manualmode_data.gripper_sequence == Place) {
		Gripper_Control(0);
		required_time = 2000;
	}
	else {
		Gripper_Control(-1);
		required_time = 800;

		if (Robot.Manualmode_data.gripper_state == Up)
			Gripper_Group(1, 0, 0, 0);
		else if (Robot.Manualmode_data.gripper_state == Down)
			Gripper_Group(0, 1, 0, 0);
		else if (Robot.Manualmode_data.gripper_state == Close)
			Gripper_Group(0, 0, 1, 0);
		else if (Robot.Manualmode_data.gripper_state == Open)
			Gripper_Group(0, 0, 0, 1);
		else {
			Gripper_Group(0, 0, 0, 0);
			required_time = 0;
		}
	}

	// Non-Blocking Timer
	if (Non_Blocking_Timer(required_time) == 1) {
		Basesystem_Reset_register();
		Robot.Robot_Status = READY;
		Robot.Manualmode_data.gripper_state = idle_gripper_state;
		Robot.Manualmode_data.gripper_sequence = idle_gripper_sequence ;
	}
}

void Robot_Worker() {
	if (Robot.Robot_Status == NOT_READY){
		if (Robot.Mode == Mode_MANUAL){
				Manual_Worker();
		}
	}

}




