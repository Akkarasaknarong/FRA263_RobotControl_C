/*
 * Robot_Run.c
 *
 *  Created on: Jun 1, 2026
 *      Author: Akkarasaknarong
 */

#include "basesystem_decode.h"
#include "robot_worker.h"
#include "gripper.h"
#include "quintic_traj.h"

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
	static int state = 0;
	static float start_q, target_q;
	static uint32_t start_tick, wait_time;

	switch (state) {
		case 0:
			if (Robot.Manualmode_data.Jog_value != 0) {
				start_q = REFdata.ref_q_deg;
				target_q = start_q + (float)Robot.Manualmode_data.Jog_value;
				Timer = 0;
				state = 1;
			}
			else if (Robot.Manualmode_data.gripper_sequence == Pick) {
				Gripper_Control(1);
				start_tick = HAL_GetTick(); wait_time = 2000;
				state = 2;
			}
			else if (Robot.Manualmode_data.gripper_sequence == Place) {
				Gripper_Control(0);
				start_tick = HAL_GetTick(); wait_time = 2000;
				state = 2;
			}
			else if (Robot.Manualmode_data.gripper_state != idle_gripper_state) {
				if (Robot.Manualmode_data.gripper_state == Up) Gripper_Group(1, 0, 0, 0);
				else if (Robot.Manualmode_data.gripper_state == Down) Gripper_Group(0, 1, 0, 0);
				else if (Robot.Manualmode_data.gripper_state == Close) Gripper_Group(0, 0, 1, 0);
				else if (Robot.Manualmode_data.gripper_state == Open) Gripper_Group(0, 0, 0, 1);

				start_tick = HAL_GetTick(); wait_time = 800;
				state = 2;
			}
			break;
		case 1:
			QuinticTraj_P2P(start_q, target_q, 2.5f, Timer);
			if (Timer >= 2.5f) {
				state = 99;
			}
			break;
		case 2:
			if (Robot.Manualmode_data.gripper_sequence == Pick) Gripper_Control(1);
			else if (Robot.Manualmode_data.gripper_sequence == Place) Gripper_Control(0);

			if (HAL_GetTick() - start_tick >= wait_time) {
				state = 99;
			}
			break;
		case 99:
			Basesystem_Reset_register();
			Robot.Robot_Status = READY;
			Robot.Manualmode_data.gripper_state = idle_gripper_state;
			Robot.Manualmode_data.gripper_sequence = idle_gripper_sequence;
			Robot.Manualmode_data.Jog_value = 0;

			state = 0;
			break;
	}
}

void Robot_Worker() {
	if (Robot.Robot_Status == NOT_READY){
		if (Robot.Mode == Mode_MANUAL){
				Manual_Worker();
		}
	}

}




