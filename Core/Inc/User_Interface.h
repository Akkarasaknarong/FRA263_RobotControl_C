/*
 * User_Interface.h
 *
 *  Created on: May 31, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_USER_INTERFACE_H_
#define INC_USER_INTERFACE_H_

#include "stm32g4xx_hal.h"
#include "main.h"
#include "datatype.h"

// Button state struct
typedef enum {
	HAVE_RESET,
    DON_HAVE_RESET
} wait_Reset_t;

typedef struct {
	uint8_t emer_button ;
	uint8_t reset_button;
	uint8_t rotary_sw;
	uint8_t prox_state ;
} Cabinet_State_t ;

extern Robot_t Robot ;
extern Cabinet_State_t Cabinet_State ;
extern Robot_Ready_t Robot_Ready ;
extern wait_Reset_t FLAG_Wait_reset;

void User_Interface_Start();

#endif /* INC_USER_INTERFACE_H_ */
