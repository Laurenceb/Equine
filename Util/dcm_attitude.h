#pragma once

#include "stm32f10x.h"

void main_filter(float DCM[3][3], float magno[3], float accel[3], float euler_out[3], float gyro[3], float d_t);
void init_controller(float PI_limit[3]);
void run_3_pi(float out[3], float I[3], float PI[3][2], float I_limit[3], float error[3], float d_t);
void accel_correction(float out[2], float accel[3]);
float magno_correction(float magno[3]);
void cross_product(float out[3], float in_one[3], float in_two[3]);
float normalize_vector(float out[3], float in[3]);
void vector_by_matrix(float out[3], float in[3], float matrix[3][3]);
void normalize_DCM(float DCM[3][3]);
void propogate_gyro(float DCM[3][3], float gyro[3], float delta_t);
void DCM_to_quaternion(float quat[4], float DCM[3][3]);
void DCM_to_euler(float euler[3], float DCM[3][3]);
