/*
 * datatype.h
 *
 *  Created on: May 30, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_DATATYPE_H_
#define INC_DATATYPE_H_

#include "stm32g4xx_hal.h"
#include <arm_math.h>
#include <stdint.h>
#include <string.h>
#include "main.h"

// ======================================================================= //
// Robot Status
// ======================================================================= //
// Current Task
typedef enum {
	Homing,
	Go_Pick,
	Go_Place,
	Go_Point,
	Idle
} CurrentTask_t;

// Emergency State
typedef enum {
	EMERGENCY_PRESS,
	EMERGENCY_DO_NOT_PRESS
} Emergency_t;

// Ready / Not Ready
typedef enum {
	READY,
	NOT_READY
} Robot_Ready_t;

// Curent Mode
typedef enum {
    Mode_IDLE    = 0,
	Mode_MANUAL  = 2,
	Mode_AUTO    = 4,
	Mode_TEST    = 16
} RobotMode_t;

// Gripper State
typedef enum {
	idle_gripper_state,
	Up,
	Down,
	Close,
	Open,
}Gripper_t;

// Gripper Sequenc
typedef enum {
	idle_gripper_sequence,
	Pick,
	Place
}Gripper_sequence_t;

// Joystick
typedef enum {
	JOY_Emergency,
	JOY_Reset,
	JOY_Toggle_UpDown,
	JOY_Toggle_OpenClose,
	JOY_Rotate_Left,
	JOY_Rotate_Right,
	JOY_Homing,
} Joytick_t;

// Joystick
typedef enum {
	deg,
	idx
} unit_t;
// ======================================================================= //
// Cascade Control
// ======================================================================= //
// Refference  Target
extern float Timer;
typedef struct {
	float ref_q ;
	float ref_qd ;
	float ref_qdd ;
	float ref_q_deg;
	float ref_q_index;
} refTarget_t;

// Current Status
typedef struct {
	float q ;
	float qd ;
	float q_deg ;
	float q_index ;
} QEIstruct_t;

// Steadystate Error
typedef struct {
	float q_ss ;
	float qd_ss ;
	float qd_deg_ss;
	float qd_index_ss;
} SSErrorstruct_t;

// Kalman Estimate
typedef struct {
	float q_est ;
	float qd_est ;
	float load_est ;
	float i_est ;
} KALMANstruct_t;

// PID Parameter
typedef struct {
	float kp_pos;
	float kd_pos ;
	float ki_pos ;
	float kp_vel;
	float kd_vel ;
	float ki_vel ;
} PIDParam_t;


// ======================================================================= //
// Basesystem Deocde
// ======================================================================= //
// Datatype U16 U8
typedef union {
    uint16_t U16;
    int16_t  I16;
    uint8_t  U8[2];
} u16u8_t;

// Ready to Receive basesystem
typedef enum {
	Ready_recieve_Basesystem,
	Not_Ready_recieve_Basesystem
} Robot_Status_t ;

// Have data from basesystem
typedef enum {
	Do_not_have_data_Basesystem,
	have_data_Basesystem
} Data_from_Basesystem_t ;


// Robot Processing state
typedef enum {
	Robot_Idle,
	Robot_Processing,
	Robot_Complete
} Robot_Process_t ;





// ======================================================================= //
// Combine all data in Robot
// ======================================================================= //
// Automode struct
typedef enum {
	CCW ,
	CW
} Direction_t ;
typedef enum {
	GRIPPER_ENABLE = 0,
	GRIPPER_DISENABLE = 1
}GripperENA_t ;
// Pick and Place
typedef struct {
	Direction_t Direction[9] ;
	uint8_t Number_of_target ;
	GripperENA_t Gripper_Ena;
	int8_t Gripper_sequence[10] ; // Gripper Sequence
} Pick_Place_t ;
// P2P
typedef struct {
	unit_t unit;
	uint16_t p2p_value ;
} P2P_t ;
typedef struct {
	Pick_Place_t PickPlace;
	P2P_t P2P ;
} Automode_t ;
// ======================================================================= //
// Manualmode struct
typedef struct {
	Gripper_t gripper_state; // up down close open
	Gripper_sequence_t gripper_sequence ; // pick place
	int16_t Jog_value; // unit degeree
} Manualmode_t ;
// ======================================================================= //
// Testmode struct
typedef enum {
	Precision_test = 0,
	 Performance_test = 1,
	 idle_testmode
} Testtype_t ;

typedef struct {
	int8_t Speed ;
	int8_t Accel ;
} Performance_t ;
typedef struct {
	unit_t unit ;
	uint16_t Init_pos ;
	uint16_t Targ_pos ;
	int8_t repeat ;
} Precision_t ;
typedef struct {
	Testtype_t Testtype ;
	Performance_t Performance ;
	Precision_t Precision;
} Testmode_t ;
// ======================================================================= //
// Feedback data
typedef struct {
	CurrentTask_t Current_Task;
	Emergency_t Emergency;
	Gripper_t Gripper;
} Feedback_t ;
// ======================================================================= //
// Monitor data
typedef struct {
	QEIstruct_t *QEI;
	refTarget_t *REF;
	SSErrorstruct_t *SSErr;
} Monitors_t ;
// ======================================================================= //




// ======================================================================= //
// Combine Robot struct
typedef struct {
	Robot_Ready_t Robot_Status ; // Ready to Receive basesystem

	RobotMode_t Mode;
	Automode_t Automode_data ;
	Manualmode_t Manualmode_data;
	Testmode_t Testmode_data ;
	Feedback_t Feedback_data ;
	Monitors_t Monitor_data ;
	Joytick_t Joystick;
} Robot_t;

// Convert unit
static inline float IndexToDegree(int index) {
    return (float)index * 5.0f;
}
static inline int DegreeToIndex(float degree) {
    return (int)roundf(degree / 5.0f);
}
static inline float IndexToRadian(int index) {
    return (float)index * (PI / 36.0f);
}
static inline int RadianToIndex(float radian) {
    return (int)roundf(radian * (36.0f / PI));
}
static inline float DegreeToRadian(float degree) {
    return degree * (PI / 180.0f);
}
static inline float RadianToDegree(float radian) {
    return radian * (180.0f / PI);
}
static inline float Scalex10(float raw) {
    return raw * 10.0f;
}
#endif /* INC_DATATYPE_H_ */
