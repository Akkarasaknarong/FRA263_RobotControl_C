/*
 * User_Interface.c
 *
 *  Created on: May 31, 2026
 *      Author: Akkarasaknarong
 */

#include "User_Interface.h"

Cabinet_State_t Cabinet_State = {0};
uint32_t Local_Cabinet_Timer = 0;
Robot_Ready_t Robot_Ready = READY ;
wait_Reset_t FLAG_Wait_reset = HAVE_RESET;

void JoyStick_Update(){
	uint8_t Homing = HAL_GPIO_ReadPin(JS_HOME_GPIO_Port, JS_HOME_Pin);
	uint8_t Emergency = HAL_GPIO_ReadPin(JS_EMERGENCY_GPIO_Port, JS_EMERGENCY_Pin);
	uint8_t Reset = HAL_GPIO_ReadPin(JS_RESET_GPIO_Port, JS_RESET_Pin);
	uint8_t UpDown = HAL_GPIO_ReadPin(JS_Updown_GPIO_Port, JS_Updown_Pin);
	uint8_t OpenClose = HAL_GPIO_ReadPin(JS_OpenClose_GPIO_Port, JS_OpenClose_Pin);
	uint8_t RotateLeft = HAL_GPIO_ReadPin(JS_Rotate_Left_GPIO_Port, JS_Rotate_Left_Pin);
	uint8_t RotateRight = HAL_GPIO_ReadPin(JS_Rotate_Right_GPIO_Port, JS_Rotate_Right_Pin);

	if (Homing == 0){
		Robot.Joystick = JOY_Homing ;
	} else if (Emergency == 0){
		Robot.Joystick = JOY_Emergency ;
	} else if (Reset == 0){
		Robot.Joystick = JOY_Reset ;
	} else if (UpDown == 0){
		Robot.Joystick = JOY_Toggle_UpDown ;
	} else if (OpenClose == 0){
		Robot.Joystick = JOY_Toggle_OpenClose ;
	} else if (RotateLeft == 0){
		Robot.Joystick = JOY_Rotate_Left ;
	} else if (RotateRight == 0){
		Robot.Joystick = JOY_Rotate_Right ;
	}
}

void User_Interface_Start(){
	Cabinet_State.emer_button= HAL_GPIO_ReadPin(E_STOP_GPIO_Port, E_STOP_Pin);
	Cabinet_State.reset_button= HAL_GPIO_ReadPin(BUTTON_RESET_GPIO_Port, BUTTON_RESET_Pin);

	if (Cabinet_State.emer_button == 0){
		Robot_Ready = NOT_READY;
		FLAG_Wait_reset = DON_HAVE_RESET;
	}
	if (Cabinet_State.emer_button == 1){
		Robot_Ready = READY;
	}
	if (Cabinet_State.reset_button == 0){
		FLAG_Wait_reset = HAVE_RESET;
	}

	// Not Ready
	if (Robot_Ready == NOT_READY && FLAG_Wait_reset == HAVE_RESET){
		Cabinet_State.prox_state = 0 ;
		Cabinet_State.rotary_sw = 0 ;
		HAL_GPIO_WritePin(RELAY_MODE_GPIO_Port, RELAY_MODE_Pin, 0);
		HAL_GPIO_WritePin(RELAY_READY_GPIO_Port, RELAY_READY_Pin, 0);
	}
	else if (Robot_Ready == NOT_READY && FLAG_Wait_reset == DON_HAVE_RESET){
		Cabinet_State.prox_state = 0 ;
		Cabinet_State.rotary_sw = 0 ;
		HAL_GPIO_WritePin(RELAY_MODE_GPIO_Port, RELAY_MODE_Pin, 0);
		HAL_GPIO_WritePin(RELAY_READY_GPIO_Port, RELAY_READY_Pin, 0);

	// Ready
	} else if (Robot_Ready == READY && FLAG_Wait_reset == HAVE_RESET){
		Cabinet_State.rotary_sw = HAL_GPIO_ReadPin(MODE_SELECTOR_GPIO_Port,MODE_SELECTOR_Pin);
		Cabinet_State.prox_state = HAL_GPIO_ReadPin(PROX_GPIO_Port, PROX_Pin);

		if (Cabinet_State.rotary_sw == 0) {
			HAL_GPIO_WritePin(RELAY_MODE_GPIO_Port, RELAY_MODE_Pin, 1);
		} else {
			HAL_GPIO_WritePin(RELAY_MODE_GPIO_Port, RELAY_MODE_Pin, 0);
		}
		if (Cabinet_State.emer_button == 0) {
			HAL_GPIO_WritePin(RELAY_READY_GPIO_Port, RELAY_READY_Pin, 0);
		} else {
			HAL_GPIO_WritePin(RELAY_READY_GPIO_Port, RELAY_READY_Pin, 1);
		}

		JoyStick_Update();
	}

	// Monitor
	if (Cabinet_State.emer_button == 0){
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
	}
	if (Cabinet_State.emer_button == 1){
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
		}

	// Feedback to Robot
	if (Robot_Ready == READY){
		Robot.Feedback_data.Emergency = EMERGENCY_DO_NOT_PRESS ;
	} else if (Robot_Ready == NOT_READY){
		Robot.Feedback_data.Emergency = EMERGENCY_PRESS ;
	}

}




