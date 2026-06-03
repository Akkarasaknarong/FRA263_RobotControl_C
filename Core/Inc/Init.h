/*
 * Init.h
 *
 *  Created on: May 31, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_INIT_H_
#define INC_INIT_H_

#include "stm32g4xx_hal.h"
#include "datatype.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern UART_HandleTypeDef huart2;
extern SerialFrame_t STLINK_UART_frame;
extern u16u8_t reg[MODBUS_REGISTER_COUNT];
extern ModbusHandleTypedef hmodbus;
extern Robot_t Robot ;

#define ENCODER_TIM       htim3
#define micors_TIM       htim5
#define PWM_CH1_TIM       htim8
#define UART_TIM          htim15
#define POS_CTRL_TIM      htim6
#define VEL_CTRL_TIM      htim16
#define QEI_UPDATE_TIM    htim17
#define MODBUS_TIM    htim7
#define STLINK_UART huart2

void Init_TIMER(){
	HAL_TIM_Encoder_Start(&ENCODER_TIM, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start(&PWM_CH1_TIM);
	HAL_TIM_PWM_Start(&PWM_CH1_TIM, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&POS_CTRL_TIM);
	HAL_TIM_Base_Start_IT(&VEL_CTRL_TIM);
	HAL_TIM_Base_Start_IT(&QEI_UPDATE_TIM);
	HAL_TIM_Base_Start_IT(&UART_TIM);
}

void Init_Library(){
	// Init Cabinet State
	Cabinet_State.emer_button = 0;
	Cabinet_State.prox_state = 0;
	Cabinet_State.reset_button = 0;
	Cabinet_State.rotary_sw = 0;

	// Init Base System
	Basesystem_Interface_Init();

	// Init Motor
	MD20A_Init(&PWM_CH1_TIM, DIR_GPIO_Port, DIR_Pin);

	// Init Refference FFW
	Motor_Ref_feedforward_Init();

	// Init Refference Kalman
	Motor_Kalman_Init();
	Motor_Kalman_Tunning(1, 1e-01, 10, 1e-01, 1e-20);

	// Init QEI
	QEI_Init(&htim3);

	// Init Cascade
	Pos_ctrl_Init();
	Pos_ctrl_Tunning(100000, 5000, 5000);
	Vel_ctrl_Init();
	Vel_ctrl_Tunning(1, 0, 0);

	// Init Serial Frame
	SerialFrame_Init(&STLINK_UART_frame, &STLINK_UART);

	// Init modbus
	memset(reg, 0, sizeof(reg));
	reg[REG_HEARTBEAT].U16 = HEARTBEAT_ROBOT;
	hmodbus.huart = &STLINK_UART;
	hmodbus.htim = &MODBUS_TIM;
	hmodbus.slaveAddress = MODBUS_SLAVE_ADDRESS;
	hmodbus.RegisterSize = MODBUS_REGISTER_COUNT;
	Modbus_init(&hmodbus, reg);
}

#endif /* INC_INIT_H_ */
