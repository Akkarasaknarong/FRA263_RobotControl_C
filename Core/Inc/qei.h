/*
 * qei.h
 *
 *  Created on: May 28, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_QEI_H_
#define INC_QEI_H_

#include "stm32g4xx_hal.h"
#include <arm_math.h>

#define QEI_1TURN 8192
#define QEI_MULTITURN 57344
#define QEI_UPDATE_FREQ 0.0002

void QEI_Init(TIM_HandleTypeDef *tim);
void QEI_Update(float *qei_q,float *qei_qd);

#endif /* INC_QEI_H_ */
