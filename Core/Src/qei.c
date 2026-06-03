/*
 * qei.c
 *
 *  Created on: May 28, 2026
 *      Author: Akkarasaknarong
 */

#include "qei.h"

static uint32_t q_cur_tick;
static uint32_t q_lst_tick;
static int32_t q_conti_tick;
static float q_cur_rad;
static float q_lst_rad;
static float q_conti_rad;
static float qd_cur_rad;
static TIM_HandleTypeDef *qei_htim; ;

void QEI_Init(TIM_HandleTypeDef *tim){
	qei_htim = tim ;
	q_cur_tick = 0 ;
	q_lst_tick = 0 ;
	q_conti_tick = 0 ;
	q_cur_rad = 0 ;
	q_lst_rad = 0 ;
	q_conti_rad = 0 ;
	qd_cur_rad = 0 ;
}

void QEI_Update() {
	q_cur_tick = __HAL_TIM_GET_COUNTER(qei_htim);
	int32_t diff_position_tick = q_cur_tick - q_lst_tick;

	if (diff_position_tick > (QEI_MULTITURN / 2))
		diff_position_tick -= QEI_MULTITURN;
	else if (diff_position_tick < -(QEI_MULTITURN / 2))
		diff_position_tick += QEI_MULTITURN;

	q_conti_tick += diff_position_tick;
	q_conti_rad = (q_conti_tick / 8192.0f) * 2 * PI;

	float diff_position_rad = (diff_position_tick / 8192.0f) * 2 * PI;
	qd_cur_rad = diff_position_rad / QEI_UPDATE_FREQ;

	q_lst_tick = q_cur_tick;

	QEIdata.q = q_conti_rad ;
	QEIdata.qd = qd_cur_rad ;
	QEIdata.q_deg = (q_conti_tick / 8192.0f) * 360.0f ;
}



