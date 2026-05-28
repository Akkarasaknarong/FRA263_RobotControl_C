/*
 * kalman_filter.h
 *
 *  Created on: May 27, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_KALMAN_FILTER_H_
#define INC_KALMAN_FILTER_H_
#include <arm_math.h>

typedef struct {
    float est_pos_rad;
    float est_vel_rad;
    float est_load;
    float est_current;
} KalmanStruct_t;

void Motor_Kalman_Init();
void Motor_Kalman_Tunning(float32_t q_pos, float32_t q_vel, float32_t q_load , float32_t q_i, float32_t r);
void Motor_Kalman_Update(float32_t v, float32_t z, float32_t *q_est,
		float32_t *qd_est, float32_t *load_est, float32_t *i_est);

#endif /* INC_KALMAN_FILTER_H_ */
