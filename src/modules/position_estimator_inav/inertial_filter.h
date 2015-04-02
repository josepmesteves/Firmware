/*
 * inertial_filter.h
 *
 *   Copyright (C) 2013 Anton Babushkin. All rights reserved.
 *   Author: 	Anton Babushkin	<rk3dov@gmail.com>
 */

#include <stdbool.h>
#include <drivers/drv_hrt.h>

void inertial_filter_predict(float dt, float x[3], float acc);

void inertial_filter_predict2(float dt, float x[3], float acc, float acc_prev, bool use_vel);

void inertial_filter_correct(float e, float dt, float x[3], int i, float w);

void baro_inertial_filter_correct(float e_pos, float e_vel, float z[2], float w_pos, float w_vel);
