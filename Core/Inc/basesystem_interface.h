/*
 * basesystem_interface.h
 *
 *  Created on: May 30, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_BASESYSTEM_INTERFACE_H_
#define INC_BASESYSTEM_INTERFACE_H_
#include "datatype.h"

typedef enum {
    AUTO_TYPE_NONE = 0,
    AUTO_TYPE_SEQUENCE,
    AUTO_TYPE_P2P
} AutoType_t;

typedef struct {
	int unit ;
	int value ;
} P2PVal_Struct ;

typedef struct {
	uint8_t N_pare ;
	uint8_t Gripper_status ;
	int16_t  Sequence[10];

	AutoType_t Type;
	P2PVal_Struct P2P ;
} AutoMode_Struct ;

typedef struct {
	int Gripper_pin_state ;
	int Gripper_command ;
	int Jog_val ;
} ManualMode_Struct ;

typedef struct {
	RobotMode_t Mode ;
	AutoMode_Struct _Auto;
	ManualMode_Struct _Manual ;
	int has_new_data ;
} Basesystem_t;

extern Basesystem_t Basesystem_Data ;
extern QEIstruct_t QEIdata ;
extern KALMANstruct_t ESTdata ;
extern Robot_t Robot ;
extern u16u8_t reg[50];

void Basesystem_Interface_Init();
void Basesystem_Interface_Decode(u16u8_t* holding_register);
void Basesystem_Interface_Feedback();

#endif /* INC_BASESYSTEM_INTERFACE_H_ */
