/*
 * qei.h
 *
 *  Created on: May 28, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_QEI_H_
#define INC_QEI_H_

#include "datatype.h"

#define QEI_1TURN 8192
#define QEI_MULTITURN 57344
#define QEI_UPDATE_FREQ 0.0002
extern QEIstruct_t QEIdata ;

void QEI_Init(TIM_HandleTypeDef *tim);
void QEI_Update();

#endif /* INC_QEI_H_ */
