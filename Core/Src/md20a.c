/*
 * md20a.c
 *
 *  Created on: May 30, 2026
 *      Author: Akkarasaknarong
 */

#include "md20a.h"
static MD20AParam motorDriver;

void MD20A_Init(TIM_HandleTypeDef* pwm_tim, GPIO_TypeDef* dir_port, uint16_t dir_pin) {
	motorDriver._tim_pwm = pwm_tim;
	motorDriver._dir_port = dir_port;
	motorDriver._dir_pin = dir_pin;
}

void MD20A_Control(int pwm) {
	// Satutation
	if (pwm > 65535) {
		pwm = 65535;
	} else if (pwm < -65535) {
		pwm = -65535;
	}

	// Remap Direction
	if (pwm > 0) {
		HAL_GPIO_WritePin(motorDriver._dir_port, motorDriver._dir_pin, 0);
		__HAL_TIM_SET_COMPARE(motorDriver._tim_pwm, TIM_CHANNEL_1, pwm);
	} else if (pwm < 0) {
		HAL_GPIO_WritePin(motorDriver._dir_port, motorDriver._dir_pin, 1);
		__HAL_TIM_SET_COMPARE(motorDriver._tim_pwm, TIM_CHANNEL_1, -pwm);
	} else {
		HAL_GPIO_WritePin(motorDriver._dir_port, motorDriver._dir_pin, 0);
		__HAL_TIM_SET_COMPARE(motorDriver._tim_pwm, TIM_CHANNEL_1, 0);
	}
}


