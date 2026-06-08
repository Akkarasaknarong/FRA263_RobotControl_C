/*
 * basesystem_interface.c
 *
 *  Created on: May 30, 2026
 *      Author: Akkarasaknarong
 */

#include "stdlib.h"
#include "basesystem_decode.h"
#include "modbus.h"

extern Robot_t Robot ;
extern refTarget_t REFdata ;
extern KALMANstruct_t ESTdata ;
static u16u8_t* local_reg = {0};

// Mode Selector Decode
typedef enum {
	_auto_mode = 4 ,
	_jog_mode = 2,
	_test_mode = 16
} mode_selector_t ;
// Manual mode Decode
typedef enum {
	_up = 0 ,
	_down = 1,
	_open = 2,
	_close = 4,
	_gripper_state_idle = 99
} gripper_state_t ;
typedef enum {
	_pick = 1 ,
	_place = 2
} gripper_sequence_t ;
// Test mode Decode
typedef enum {
	_Precision_test = 0 ,
	_Performance_test = 1
} testtype_t ;
// Auto mode Decode
typedef enum {
	_GRIPPER_ENABLE = 1,
	_GRIPPER_DISENABLE = 0
}gripperENA_t ;

float map_to_0_360(float angle_infinity) {
    float angle_mod = fmodf(angle_infinity, 360.0f);
    if (angle_mod < 0.0f) {
        angle_mod += 360.0f;
    }
    return angle_mod;
}

void decode_modeselector(){
	int8_t mode_check = local_reg[MODE_SELECTER_REGISTER].I16;
	if (mode_check == _auto_mode)
		Robot.Mode = Mode_AUTO ;
	else if (mode_check == _jog_mode)
		Robot.Mode = Mode_MANUAL ;
	else if (mode_check == _test_mode)
		Robot.Mode = Mode_TEST ;
}

void decode_automode(){
	// CW / CCW
	Robot.Automode_data.PickPlace.Gripper_sequence[0] = local_reg[SEQ1_REGISTER].I16;
	Robot.Automode_data.PickPlace.Gripper_sequence[1] = local_reg[SEQ2_REGISTER].I16;
	Robot.Automode_data.PickPlace.Gripper_sequence[2] = local_reg[SEQ3_REGISTER].I16;
	Robot.Automode_data.PickPlace.Gripper_sequence[3] = local_reg[SEQ4_REGISTER].I16;
	Robot.Automode_data.PickPlace.Gripper_sequence[4] = local_reg[SEQ5_REGISTER].I16;
	Robot.Automode_data.PickPlace.Gripper_sequence[5] = local_reg[SEQ6_REGISTER].I16;
	Robot.Automode_data.PickPlace.Gripper_sequence[6] = local_reg[SEQ7_REGISTER].I16;
	Robot.Automode_data.PickPlace.Gripper_sequence[7] = local_reg[SEQ8_REGISTER].I16;
	Robot.Automode_data.PickPlace.Gripper_sequence[8] = local_reg[SEQ9_REGISTER].I16;
	Robot.Automode_data.PickPlace.Gripper_sequence[9] = local_reg[SEQ10_REGISTER].I16;
	for (int i = 0; i < 10; i++) {
	    int16_t seq_val = local_reg[SEQ1_REGISTER + i].I16;

	    if (seq_val == 0) {
	        Robot.Automode_data.PickPlace.Direction[i] = CW;
	    }
	    else if (seq_val < 0) {
	        Robot.Automode_data.PickPlace.Direction[i] = CW;
	    }
	    else {
	        Robot.Automode_data.PickPlace.Direction[i] = CCW;
	    }
	}

	// Sequence
	Robot.Automode_data.PickPlace.Gripper_sequence[0] = abs(local_reg[SEQ1_REGISTER].I16);
	Robot.Automode_data.PickPlace.Gripper_sequence[1] = abs(local_reg[SEQ2_REGISTER].I16);
	Robot.Automode_data.PickPlace.Gripper_sequence[2] = abs(local_reg[SEQ3_REGISTER].I16);
	Robot.Automode_data.PickPlace.Gripper_sequence[3] = abs(local_reg[SEQ4_REGISTER].I16);
	Robot.Automode_data.PickPlace.Gripper_sequence[4] = abs(local_reg[SEQ5_REGISTER].I16);
	Robot.Automode_data.PickPlace.Gripper_sequence[5] = abs(local_reg[SEQ6_REGISTER].I16);
	Robot.Automode_data.PickPlace.Gripper_sequence[6] = abs(local_reg[SEQ7_REGISTER].I16);
	Robot.Automode_data.PickPlace.Gripper_sequence[7] = abs(local_reg[SEQ8_REGISTER].I16);
	Robot.Automode_data.PickPlace.Gripper_sequence[8] = abs(local_reg[SEQ9_REGISTER].I16);
	Robot.Automode_data.PickPlace.Gripper_sequence[9] = abs(local_reg[SEQ10_REGISTER].I16);

	// N-Pare
	Robot.Automode_data.PickPlace.Number_of_target =  local_reg[N_PARE_REGISTER].I16;
	// Gripper
	int8_t gpp_ena = local_reg[GRIPPER_CHECKBOX_REGISTER].I16;
	if (gpp_ena == _GRIPPER_ENABLE)
		Robot.Automode_data.PickPlace.Gripper_Ena = GRIPPER_ENABLE;
	else if (gpp_ena == _GRIPPER_DISENABLE)
		Robot.Automode_data.PickPlace.Gripper_Ena = GRIPPER_DISENABLE;


	// P2P
	int8_t p2p_unit_check =  local_reg[P2P_UNIT_REGISTER].I16;
	if (p2p_unit_check == 1)
		Robot.Automode_data.P2P.unit = idx ;
	else if (p2p_unit_check == 0)
			Robot.Automode_data.P2P.unit = deg ;
	Robot.Automode_data.P2P.p2p_value = local_reg[P2P_VALUE_REGISTER].I16;
}

void decode_manualmode(){
	// Gripper state
	if (local_reg[GRIPPER_STATE_REGISTER].U16 == _up){
		Robot.Manualmode_data.gripper_state = Up ;
	} else if (local_reg[GRIPPER_STATE_REGISTER].U16 == _down){
		Robot.Manualmode_data.gripper_state = Down ;
	} else if (local_reg[GRIPPER_STATE_REGISTER].U16 == _open){
		Robot.Manualmode_data.gripper_state = Open ;
	} else if (local_reg[GRIPPER_STATE_REGISTER].U16 == _close){
		Robot.Manualmode_data.gripper_state = Close ;
	} else {
		Robot.Manualmode_data.gripper_state = idle_gripper_state;
	}


	// Gripper sequences
	if (local_reg[GRIPPER_SEQUENCE_REGISTER].U16 == _pick){
		Robot.Manualmode_data.gripper_sequence = Pick ;
	}
	else if (local_reg[GRIPPER_SEQUENCE_REGISTER].U16 == _place){
		Robot.Manualmode_data.gripper_sequence = Place ;
	}

	// Jog value
	Robot.Manualmode_data.Jog_value = local_reg[JOG_VALUE_REGISTER].I16 ;
}

void decode_testmode(){
	if (local_reg[TESTMODE_SELECMODE_REGISTER].I16 == _Precision_test){
		Robot.Testmode_data.Testtype = Precision_test ;
	} else if (local_reg[TESTMODE_SELECMODE_REGISTER].I16 == _Performance_test){
		Robot.Testmode_data.Testtype = Performance_test ;
	}

	// Perfonmance Value
	Robot.Testmode_data.Performance.Speed = local_reg[TEST_SPEED_REGISTER].I16 ;
	Robot.Testmode_data.Performance.Accel = local_reg[TEST_ACCEL_REGISTER].I16 ;

	// Precision Value
	Robot.Testmode_data.Precision.Init_pos = local_reg[TEST_INIT_POS_REGISTER].I16 ;
	Robot.Testmode_data.Precision.Targ_pos = local_reg[TEST_TARG_POS_REGISTER].I16 ;

	Robot.Testmode_data.Precision.repeat = local_reg[TEST_REPEAT_REGISTER].I16 ;
	if (Robot.Testmode_data.Precision.repeat < 0)
		Robot.Testmode_data.Precision.unit =  idx;
	else if (Robot.Testmode_data.Precision.repeat > 0)
		Robot.Testmode_data.Precision.unit =  deg;

	Robot.Testmode_data.Precision.repeat = abs(local_reg[TEST_REPEAT_REGISTER].I16) ;
}

void Basesystem_decode_Init(){

}

void Basesystem_Reset_register() {
	reg[JOG_VALUE_REGISTER].I16 = 0;
	reg[GRIPPER_SEQUENCE_REGISTER].U16 = 0;
	reg[GRIPPER_STATE_REGISTER].U16 = 99;

	reg[P2P_VALUE_REGISTER].I16 = 0;
	reg[N_PARE_REGISTER].I16 = 0;

	reg[TEST_REPEAT_REGISTER].I16 = 0;

//    for (int i = 0x02; i <= 0x25; i++) {
//        reg[i].U16 = 0;
//    }
//    reg[GRIPPER_STATE_REGISTER].U16 = 99;
}

void Basesystem_decode_Update(){
	local_reg = reg ;
	decode_modeselector();
	decode_manualmode();
	decode_testmode();
	decode_automode();
	Robot.Homing = local_reg[0x01].U16;
}

void Basesystem_Feedback(){
	reg[POSITION_FEEDBACK_REGISTER].I16 = (int16_t)(REFdata.ref_q_deg * 10.0f);
	reg[VELOCITY_FEEDBACK_REGISTER].I16 = (int16_t)(ESTdata.qd_est* 10.0f);
	reg[ACCELERATION_FEEDBACK_REGISTER].I16 = (int16_t)(RadianToDegree(ESTdata.q_est) * 10.0f);

}


