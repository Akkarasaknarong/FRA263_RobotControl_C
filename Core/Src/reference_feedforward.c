/*
 * refference_feedforward.c
 *
 *  Created on: May 27, 2026
 *      Author: Akkarasaknarong
 */

#include "reference_feedforward.h"

#define FFW_NUM0    ( 0.4324f)
#define FFW_NUM1    (-0.4025f)
#define FFW_NUM2    ( 0.3450f)
#define FFW_DEN1    ( 1.4954f)
#define FFW_DEN2    (-0.5590f)

static float u_prev1 = 0 ;
static float u_prev2 = 0 ;
static float y_prev1 = 0 ;
static float y_prev2 = 0 ;

void Motor_Ref_feedforward_Init()
{
    u_prev1 = 0.0f;
    u_prev2 = 0.0f;
    y_prev1 = 0.0f;
    y_prev2 = 0.0f;
}

void Motor_Ref_feedforward_Update(float ref_vel,float* V)
{
    float y;
    y =   FFW_DEN1 * y_prev1
        + FFW_DEN2 * y_prev2
        + FFW_NUM0 * ref_vel
        + FFW_NUM1 * u_prev1
        + FFW_NUM2 * u_prev2;
    u_prev2 = u_prev1;
    u_prev1 = ref_vel;
    y_prev2 = y_prev1;
    y_prev1 = y;

	*V = y;
}



