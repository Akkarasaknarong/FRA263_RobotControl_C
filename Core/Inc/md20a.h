/*
 * md20a.h
 *
 *  Created on: May 30, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_MD20A_H_
#define INC_MD20A_H_
#include "datatype.h"

typedef struct {
	TIM_HandleTypeDef* _tim_pwm;
	GPIO_TypeDef* _dir_port;
	uint16_t _dir_pin;
}MD20AParam;

void MD20A_Init(TIM_HandleTypeDef* pwm_tim, GPIO_TypeDef* dir_port, uint16_t dir_pin);
void MD20A_Control(int pwm);

#endif /* INC_MD20A_H_ */
