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
#include "cascade.h"
#include <math.h>
//extern u16u8_t reg[MODBUS_REGISTER_COUNT];

// PID tuning is now handled by PID_LUT_Apply() in cascade.c

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
	case -1: // Check P2P mode
		if (Robot.Automode_data.P2P.p2p_value != 0){
			if(Robot.Automode_data.P2P.unit == deg){
				start_q = REFdata.ref_q_deg;
				target_q = start_q + (float)Robot.Automode_data.P2P.p2p_value;
			} else if (Robot.Automode_data.P2P.unit == idx){
				start_q = REFdata.ref_q_deg;
				target_q = start_q + IndexToDegree((float)Robot.Automode_data.P2P.p2p_value);
			}

			PID_LUT_Apply(fabsf(target_q - start_q), 2.5f);
			Timer = 0;
			state_auto = 88;
		}
		else { // Check Auto sequence mode
			state_auto = 0 ;
		}
		break;
		case 0:	// Automode sequence : Init Trajactory State
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
					target_t = 1.4f; // 1.25f sec
				else
					target_t = 3.25f; // 3.25f sec

				PID_LUT_Apply(fabsf(target_q - start_q), target_t);
				Timer = 0;
				state_auto = 1;
			}
			break;

		case 1: // Automode sequence : Trajactory State
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
				PID_LUT_Apply(fabsf(target_q - start_q), 2.5f);
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

void Test_Worker(){
	/*
		case 0: init and check Testmode Type
		case 1: Precision Test (Go to init_pos)
		case 2: wait (t_wait) sec
		case 3: Precision Test (Init position to Target Position)
		case 4: wait (t_wait) sec
		case 5: Precision Test (Target Position to Init position)
		case 6: wait (t_wait) sec

		case 21: Perfornance Test
		case 22:
		case 23:
		case 24:
		case 25:

		case 99: End Switch caseห
	 */

	static uint8_t state_test = 0 ;
	static int repeat ;
	static int repeat_count = 0 ;
	static float init_pos ;
	static float targ_pos ;
	static float start_q ;

	static float t_go_init = 2.0f ;
	static float t_go_front = 3.25f ;
	static float t_go_back = 3.25f ;
	static float t_wait = 1.0f ;

	switch (state_test) {
	case 0:
		if (Robot.Testmode_data.Testtype == Precision_test) {
			if (Robot.Testmode_data.Precision.repeat != 0) {
				start_q = REFdata.ref_q_deg;

				if (Robot.Testmode_data.Precision.unit == idx) {
					init_pos = IndexToDegree(Robot.Testmode_data.Precision.Init_pos);
					targ_pos = IndexToDegree(Robot.Testmode_data.Precision.Targ_pos);
				} else if (Robot.Testmode_data.Precision.unit == deg) {
					init_pos = Robot.Testmode_data.Precision.Init_pos;
					targ_pos = Robot.Testmode_data.Precision.Targ_pos;
				}
				repeat = Robot.Testmode_data.Precision.repeat;
				repeat_count = 0;
				PID_LUT_Apply(fabsf(init_pos - start_q), t_go_init);
				Timer = 0;
				state_test = 1;
			}
			} else if (Robot.Testmode_data.Testtype == Performance_test) {
				start_q = REFdata.ref_q_deg;
				PID_LUT_Apply(fabsf(start_q), t_go_init);
				Timer = 0;
				state_test = 21;
			}
			else {
				state_test = 99 ;
			}
			break;
		case 1:
			QuinticTraj_P2P(start_q, init_pos, t_go_init, Timer);
			if (Timer > t_go_init){
				Timer = 0 ;
				state_test = 2 ;
			}
			break;
		case 2:
			if (Timer > t_wait){
				PID_LUT_Apply(fabsf(targ_pos - init_pos), t_go_front);
				Timer = 0 ;
				state_test = 3 ;
			}
			break;
		case 3:
			QuinticTraj_P2P(init_pos, targ_pos, t_go_front, Timer);
			if (Timer > t_go_front){
				Timer = 0 ;
				state_test = 4 ;
			}
			break;
		case 4:
			if (Timer > t_wait){
				PID_LUT_Apply(fabsf(targ_pos - init_pos), t_go_back);
				Timer = 0 ;
				state_test = 5 ;
			}
			break ;
		case 5 :
			QuinticTraj_P2P(targ_pos, init_pos, t_go_back, Timer);
			if (Timer > t_go_back){
				Timer = 0 ;
				state_test = 6 ;
			}
			break ;
		case 6:
			if (Timer > t_wait){
				Timer = 0 ;
				repeat_count++;
				if (repeat_count < repeat){
					PID_LUT_Apply(fabsf(targ_pos - init_pos), t_go_front);
					state_test = 3;
				} else {
					state_test = 99;
				}
			}
			break ;



		case 21:
			QuinticTraj_P2P(start_q, 0, t_go_init, Timer);
			if (Timer > t_go_init){
				Timer = 0 ;
				state_test = 22 ;
			}
			break;
		case 22:
			if (Timer > t_wait){
				PID_LUT_Apply(360.0f, 1.4f);
				Timer = 0 ;
				state_test = 23 ;
			}
			break ;
		case 23:
			QuinticTraj_P2P(0, 360, 1.4f, Timer);
			if (Timer > 1.4){
				Timer = 0 ;
				state_test = 24 ;
			}
			break;
		case 24:
			if (Timer > t_wait){
				PID_LUT_Apply(360.0f, 1.4f);
				Timer = 0 ;
				state_test = 25 ;
			}
			break ;
		case 25:
			QuinticTraj_P2P(360, 0, 1.4f, Timer);
			if (Timer > 1.4){
				Timer = 0 ;
				state_test = 26 ;
			}
			break;
		case 26:
			if (Timer > t_wait){
				Timer = 0 ;
				state_test = 99 ;
			}
			break ;

		case 99:
			Basesystem_Reset_register();
			Robot.Robot_Status = READY;
			state_test = 0 ;
			repeat_count = 0;
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
		else if (Robot.Mode == Mode_TEST) {
			Test_Worker();
		}
	}
	Basesystem_Feedback();
}




