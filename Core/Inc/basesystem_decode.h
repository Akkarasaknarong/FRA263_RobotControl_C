/*
 * basesystem_interface.h
 *
 *  Created on: May 30, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_BASESYSTEM_DECODE_H_
#define INC_BASESYSTEM_DECODE_H_
#include "datatype.h"
extern u16u8_t reg[50];

// Dectect Mode
#define MODE_SELECTER_REGISTER 0x01

// Manual Mode Register
#define GRIPPER_STATE_REGISTER 0x02
#define GRIPPER_SEQUENCE_REGISTER 0x03
#define JOG_VALUE_REGISTER 0x05

// Test Mode Register
#define TESTMODE_SELECMODE_REGISTER 0x06
#define TEST_SPEED_REGISTER 0x07
#define TEST_ACCEL_REGISTER 0x08
#define TEST_INIT_POS_REGISTER 0x09
#define TEST_TARG_POS_REGISTER 0x10
#define TEST_REPEAT_REGISTER 0x11

// Auto Mode Register
// Sequence
#define SEQ1_REGISTER 0x12
#define SEQ2_REGISTER 0x13
#define SEQ3_REGISTER 0x14
#define SEQ4_REGISTER 0x15
#define SEQ5_REGISTER 0x16
#define SEQ6_REGISTER 0x17
#define SEQ7_REGISTER 0x18
#define SEQ8_REGISTER 0x19
#define SEQ9_REGISTER 0x20
#define SEQ10_REGISTER 0x21
#define N_PARE_REGISTER 0x22
#define GRIPPER_CHECKBOX_REGISTER 0x4

// P2P
#define P2P_UNIT_REGISTER 0x23
#define P2P_VALUE_REGISTER 0x24

// Feedback Register
#define POSITION_FEEDBACK_REGISTER 0x28
#define VELOCITY_FEEDBACK_REGISTER 0x29
#define ACCELERATION_FEEDBACK_REGISTER 0x30

void Basesystem_decode_Init();
void Basesystem_decode_Update();
void Basesystem_Feedback();
void Basesystem_Reset_register();
float map_to_0_360(float angle_infinity);

#endif /* INC_BASESYSTEM_DECODE_H_ */
