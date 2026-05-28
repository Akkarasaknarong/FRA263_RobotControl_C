/*
 * quintic_traj.c
 *
 *  Created on: May 28, 2026
 *      Author: Akkarasaknarong
 */

#include "quintic_traj.h"

static float a0 = 0 ;
static float a1 = 0 ;
static float a2 = 0 ;
static float a3 = 0 ;
static float a4 = 0 ;
static float a5 = 0 ;

static float q_0 = 0 ;
static float q_f = 0 ;
static float t_end = 0 ;

void QuinticTraj_Init(float q0, float qf , float tend){
	float t3 = tend*tend*tend ;
	float t4 = tend*tend*tend*tend ;
	float t5 = tend*tend*tend*tend*tend ;

	a0 = q0 ;
	a1 = 0.0f ;
	a2 = 0.0f ;
	a3 = ((20.0f * qf) - (20.0f * q0)) / (2.0f * t3);
	a4 = ((30.0f * q0) - (30.0f * qf)) / (2.0f * t4);
	a5 = ((12.0f*qf)-(12.0f*q0))/(2*t5);

	q_0 = q0 ;
	q_f = qf ;
	t_end = tend ;
}

void QuinticTraj_Compute(float t ,float* q ,float* qd ,float* qdd ){
	if (t == 0.0f){
		*q = q_0 ;
		*qd = 0.0f ;
		*qdd = 0.0f ;
		return;
	}
	if (t >= t_end){
		*q = q_f;
		*qd = 0.0f ;
		*qdd = 0.0f ;
		return ;
	}

	float t2 = t*t ;
	float t3 = t*t*t ;
	float t4 = t*t*t*t ;
	float t5 = t*t*t*t*t ;

	*q = (a0) + (a1*t) + (a2*t2) + (a3*t3) + (a4*t4) + (a5*t5);
	*qd = (a1) + (2.0f*a2*t) + (3.0f*a3*t2) + (4.0f*a4*t3) + (5.0f*a5*t4);
	*qdd = (2.0f*a2) + (6.0f*a3*t) + (12.0f*a4*t2) + (20.0f*a5*t3);
}


