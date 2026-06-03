/*
 * Robot_Run.c
 *
 *  Created on: Jun 1, 2026
 *      Author: Akkarasaknarong
 */

#include "Robot_Worker.h"
#include "basesystem_interface.h"

extern int state_machine ;
extern void Automode_Update();
extern void Quintic_List_Basesystem(int N_pare);
extern void Basesystem_Interface_Feedback();

void Robot_Worker() {
	if (Robot.Robot_Status == Ready_recieve_Basesystem && Robot.Data_from_Basesystem == have_data_Basesystem){
		Robot.Robot_Status = Not_Ready_recieve_Basesystem ;
		Robot.Robot_Processing = Robot_Processing ;
		Timer = 0.0f;
		state_machine = 1;
		Robot.Data_from_Basesystem = Do_not_have_data_Basesystem;
	}

	if (Robot.Robot_Processing == Robot_Processing){
		if (Robot.Mode == Mode_AUTO){
			if (Basesystem_Data._Auto.N_pare != 0){
				Automode_Update(); // If this function run complete will (Robot.Robot_Processing = Robot_Complete) && Auto reset (Timer = 0)
			}
		}
		if (Robot.Mode == Mode_MANUAL){
			Robot.Robot_Processing = Robot_Complete ;
		}
	}

	if (Robot.Robot_Processing == Robot_Complete){
		Robot.Robot_Status = Ready_recieve_Basesystem ;
	}
	Basesystem_Interface_Feedback();
}

void reset_sequence(){
	reg[34].U16 = 0 ;
}




