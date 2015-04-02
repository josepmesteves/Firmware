/*
 * inertial_filter.c
 *
 *   Copyright (C) 2013 Anton Babushkin. All rights reserved.
 *   Author: 	Anton Babushkin	<rk3dov@gmail.com>
 */

#include <math.h>

#include "inertial_filter.h"

void inertial_filter_predict(float dt, float x[2], float acc)
{
	if (isfinite(dt)) {
		if (!isfinite(acc)) {
			acc = 0.0f;
		}

		x[0] += x[1] * dt + acc * dt * dt / 2.0f;
		x[1] += acc * dt;
	}
}


void inertial_filter_predict2(float dt, float x[2], float acc, float acc_prev, bool use_vel)
{
	if (isfinite(dt)) {
		if (!isfinite(acc)) {
			acc = 0.0f;
		}
		
		if (use_vel) { // use velocity and laws of movement
			x[0] += x[1] * dt + acc * dt * dt / 2.0f;
			x[1] += acc * dt;
		} else { // only use the double integration of acceleration for position
			x[0] += x[1]*dt + (acc+acc_prev) * dt * dt / 4.0f;
			x[1] += (acc + acc_prev)*dt/2.0f;
		}
	}
}


void inertial_filter_correct(float e, float dt, float x[2], int i, float w)
{
	if (isfinite(e) && isfinite(w) && isfinite(dt)) {
		float ewdt = e * w * dt;
		x[i] += ewdt;

		if (i == 0) {
			x[1] += w * ewdt;
		}
	}
}


void baro_inertial_filter_correct(float e_pos, float e_vel, float z[2], float w_pos, float w_vel)
{
	if (isfinite(e_pos) && isfinite(w_pos) && isfinite(e_vel) && isfinite(w_vel)) {
		float ewdt_pos = e_pos * w_pos;
		float ewdt_vel = e_vel * w_vel;

		z[0] += ewdt_pos;
		z[1] += ewdt_vel;
	}
}

