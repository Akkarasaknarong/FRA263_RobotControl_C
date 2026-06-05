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
#include <math.h>
extern PIDParam_t PIDparam;

void Apply_PID_Tuning(float target_deg) {
//    float mod_deg = fmodf(target_deg, 360.0f);
//    if (mod_deg < 0) mod_deg += 360.0f;
//
//    PIDparam.kd_pos = 0.0f;
//    PIDparam.ki_pos = 0.0f;
//    PIDparam.kp_vel = 0.6f;
//    PIDparam.kd_vel = 0.0f;
//    PIDparam.ki_vel = 0.0f;
//
//    if (mod_deg <= 90.0f) {
//        PIDparam.kp_pos = 1000.0f;
//    }
//    else if (mod_deg <= 180.0f) {
//        PIDparam.kp_pos = 1000.0f;
//    }
//    else {
//        PIDparam.kp_pos = 1000.0f;
//    }
}

float Remap_CW_CCW(Direction_t _Direction, float _Target_q_deg, float _Start_q_deg) {
    float current_mod = fmodf(_Start_q_deg, 360.0f);
    if (current_mod < 0) {
        current_mod += 360.0f;
    }

    float diff = _Target_q_deg - current_mod;
    if (_Direction == CCW) {
        if (diff < 0.0f) {
            diff += 360.0f;
        }
    }
    else if (_Direction == CW) {
        if (diff > 0.0f) {
            diff -= 360.0f;
        }
    }
    return _Start_q_deg + diff;
}

void Auto_Worker() {
	static int state_auto = -1;
	static int current_PickPlace = 0;
	static float target_t = 0;
	static float start_q = 0, target_q = 0;
	static uint32_t gripper_start_tick = 0;

	switch (state_auto) {
	case -1:
		if (Robot.Automode_data.P2P.p2p_value != 0){
			if(Robot.Automode_data.P2P.unit == deg){
				start_q = REFdata.ref_q_deg;
				target_q = start_q + (float)Robot.Automode_data.P2P.p2p_value;
			} else if (Robot.Automode_data.P2P.unit == idx){
				start_q = REFdata.ref_q_deg;
				target_q = start_q + IndexToDegree((float)Robot.Automode_data.P2P.p2p_value);
			}

			Apply_PID_Tuning(target_q);
			Timer = 0;
			state_auto = 88;
		}
		else {
			state_auto = 0 ;
		}
		break;
	case 0:	// Init Trajactory State
			if (Robot.Automode_data.PickPlace.Number_of_target == 0) {
				current_PickPlace = 0;
				state_auto = 99;
				break;
			}
			else if (current_PickPlace >= (Robot.Automode_data.PickPlace.Number_of_target * 2)) state_auto = 99;
			else if (current_PickPlace < (Robot.Automode_data.PickPlace.Number_of_target * 2)) {
				start_q = REFdata.ref_q_deg;
				float raw_target = IndexToDegree((float)Robot.Automode_data.PickPlace.Gripper_sequence[current_PickPlace]);
				if (current_PickPlace == 0) {
					float current_mod = fmodf(start_q, 360.0f);
					if (current_mod < 0) current_mod += 360.0f;

					float diff = raw_target - current_mod;
					if (diff > 180.0f) diff -= 360.0f;
					else if (diff < -180.0f) diff += 360.0f;

					target_q = start_q + diff;
				}
				else {
					target_q = Remap_CW_CCW(Robot.Automode_data.PickPlace.Direction[current_PickPlace], raw_target, start_q);
				}
				if (current_PickPlace % 2 == 0)
					target_t = 1.3f; // 1.25f sec
				else
					target_t = 3.25f; // 3.25f sec

				Apply_PID_Tuning(target_q);
				Timer = 0;
				state_auto = 1;
			}
			break;

		case 1: // Trajactory State
			QuinticTraj_P2P(start_q, target_q, target_t, Timer);
			if (Timer > target_t) {
				if (Robot.Automode_data.PickPlace.Gripper_Ena == GRIPPER_ENABLE) {
					if (current_PickPlace % 2 == 0)
						Gripper_Control(1); // Pick
					else
						Gripper_Control(0); // Place
					gripper_start_tick = HAL_GetTick();
					state_auto = 2;
				}
				else {
					current_PickPlace++;
					state_auto = 0;
				}
			}
			break;
		case 2: // Gripper State
			if (current_PickPlace % 2 == 0) Gripper_Control(1);
			else Gripper_Control(0);
			if (HAL_GetTick() - gripper_start_tick >= 2000) {
				current_PickPlace++;
				state_auto = 0;
			}
			break;
		case 88: // P2P State
			QuinticTraj_P2P(start_q, target_q, 2.5f, Timer);
			if (Timer >= 2.5f) {
				state_auto = 99;
			}
			break;
		case 99: // End State Auto
			Basesystem_Reset_register();
			Robot.Robot_Status = READY;
			current_PickPlace = 0;
			Robot.Automode_data.PickPlace.Number_of_target = 0;
			state_auto = -1;
			break;
	}
}

void Manual_Worker() {
	static int state = 0;
	static float start_q, target_q;
	static uint32_t start_tick, wait_time;

	switch (state) {
		case 0: // Check State JOG and Gripper
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
		case 1: // JOG state
			QuinticTraj_P2P(start_q, target_q, 2.5f, Timer);
			if (Timer >= 2.5f) {
				state = 99;
			}
			break;
		case 2:  // Gripper state
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
		else if (Robot.Mode == Mode_AUTO) {
			Auto_Worker();
		}
	}
	Basesystem_Feedback();
}




