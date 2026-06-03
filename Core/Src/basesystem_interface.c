/*
 * basesystem_interface.c
 *
 *  Created on: May 30, 2026
 *      Author: Akkarasaknarong
 */

#include "basesystem_interface.h"
#include "modbus.h"

Basesystem_t Basesystem_Data = {0};
static u16u8_t* local_reg = {0};

typedef enum {
	decode_up  = 0,
	decode_down = 1,
	decode_open = 2,
	decode_close = 4
} gp_state_decoce_t ;

typedef enum {
	decode_pick = 1,
	decode_place = 2
} gp_command_decoce_t ;

void Gripper_Decode(){
	uint16_t gripper_state_decode = local_reg[REG_GRIPPER_MANUAL].U16 ;
	uint16_t gripper_command_decode = local_reg[REG_GRIPPER_SEQ].U16 ;

	if (gripper_state_decode == decode_up){
		Robot.Feedback_data.Gripper = Up ;
	} else if (gripper_state_decode == decode_down){
		Robot.Feedback_data.Gripper = Down ;
	} else if (gripper_state_decode == decode_open){
		Robot.Feedback_data.Gripper = Open;
	} else if (gripper_state_decode == decode_close){
		Robot.Feedback_data.Gripper = Close;
	}

	if (gripper_command_decode == decode_pick){
//		Robot.Gripper_sequence = Pick;
	} else if (gripper_command_decode == decode_place){
//		Robot.Gripper_sequence = Place;
	}
}

void Manual_Decode(void){
	Gripper_Decode();
}

void Auto_Mode(void){
	Basesystem_Data._Auto.Sequence[0] = local_reg[18].U16;
	Basesystem_Data._Auto.Sequence[1] = local_reg[19].U16;
	Basesystem_Data._Auto.Sequence[2] = local_reg[20].U16;
	Basesystem_Data._Auto.Sequence[3] = local_reg[21].U16;
	Basesystem_Data._Auto.Sequence[4] = local_reg[22].U16;
	Basesystem_Data._Auto.Sequence[5] = local_reg[23].U16;
	Basesystem_Data._Auto.Sequence[6] = local_reg[24].U16;
	Basesystem_Data._Auto.Sequence[7] = local_reg[25].U16;
	Basesystem_Data._Auto.Sequence[8] = local_reg[32].U16;
	Basesystem_Data._Auto.Sequence[9] = local_reg[33].U16;
	Basesystem_Data._Auto.N_pare = local_reg[34].U16;
	Basesystem_Data._Auto.Gripper_status = local_reg[4].U16;

	Basesystem_Data._Auto.P2P.unit = local_reg[35].U16;
	Basesystem_Data._Auto.P2P.value = local_reg[36].U16;
}

void Test_Mode(void){

}

void Basesystem_Interface_Init(){
	for (int i = 0; i < 10; ++i) {
		Basesystem_Data._Auto.Sequence[i] = 0;
	}
}

void Basesystem_Interface_Decode(u16u8_t* holding_register){
	if (Basesystem_Data.has_new_data == 1) {
		// Decode
		local_reg = holding_register;
		Gripper_Decode();

		uint16_t Current_Mode = local_reg[1].U16;
		if (Current_Mode == 0) {
			Basesystem_Data.Mode = Mode_IDLE;
			Robot.Mode = Mode_IDLE;
		} else if (Current_Mode == 2) {
			Basesystem_Data.Mode = Mode_MANUAL;
			Robot.Mode = Mode_MANUAL;
			Manual_Decode();
		} else if (Current_Mode == 4) {
			Basesystem_Data.Mode = Mode_AUTO;
			Robot.Mode = Mode_AUTO;
			Auto_Mode();
		} else if (Current_Mode == 16) {
			Basesystem_Data.Mode = Mode_TEST;
			Robot.Mode = Mode_TEST;
			Test_Mode();
		}
	}
}

void Basesystem_Interface_Feedback() {
	reg[REG_POSITION].U16 = (uint16_t) ((int16_t) Scalex10(QEIdata.q_deg));
	reg[REG_VELOCITY].U16 = (uint16_t) ((int16_t) Scalex10(ESTdata.qd_est));
	reg[REG_ACCELERATION].U16 = (uint16_t) ((int16_t) Scalex10(99));

	if (Robot.Feedback_data.Emergency == EMERGENCY_PRESS){
		reg[REG_EMERGENCY].U16 = 0x0001 ;
	} else if (Robot.Feedback_data.Emergency == EMERGENCY_DO_NOT_PRESS){
		reg[REG_EMERGENCY].U16 = 0x0000 ;
	}
}


