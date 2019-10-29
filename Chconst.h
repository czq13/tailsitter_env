/*
 * Chconst.h
 *
 *  Created on: Oct 22, 2019
 *      Author: czq
 */

#ifndef CHCONST_H_
#define CHCONST_H_

struct vehicle_attitude_s {
	double timestamp;
	double pre_timestamp;
	float rollspeed;
	float pitchspeed;
	float yawspeed;
	float q[4];
};

struct vehicle_attitude_setpoint_s {
	double timestamp; // required for logger
	double pre_timestamp;
	float roll_body;
	float pitch_body;
	float yaw_body;
	float yaw_sp_move_rate;
	float q_d[4];
	float thrust;
	float landing_gear;
	bool q_d_valid;
	bool roll_reset_integral;
	bool pitch_reset_integral;
	bool yaw_reset_integral;
	bool fw_control_yaw;
	bool disable_mc_yaw_control;
	bool apply_flaps;
};

struct vehicle_local_position_s {
	double timestamp; // required for logger
	double pre_timestamp;
	float x;
	float y;
	float z;
	float delta_xy[2];
	float delta_z;
	float vx;
	float vy;
	float vz;
	float z_deriv;
	float delta_vxy[2];
	float delta_vz;
	float ax;
	float ay;
	float az;
	float yaw;

};
#define M_E_F			2.71828183f
#define M_LOG2E_F		1.44269504f
#define M_LOG10E_F		0.43429448f
#define M_LN2_F			0.69314718f
#define M_LN10_F		2.30258509f
#define M_PI			3.14159265f
#define M_TWOPI_F		6.28318531f
#define M_PI_2_F		1.57079632f
#define M_PI_4_F		0.78539816f
#define M_3PI_4_F		2.35619449f
#define M_SQRTPI_F		1.77245385f
#define M_1_PI_F		0.31830989f
#define M_2_PI_F		0.63661977f
#define M_2_SQRTPI_F		1.12837917f
#define M_DEG_TO_RAD_F		0.0174532925f
#define M_RAD_TO_DEG_F		57.2957795f
#define M_SQRT2_F		1.41421356f
#define M_SQRT1_2_F		0.70710678f
#define M_LN2LO_F		1.90821484E-10f
#define M_LN2HI_F		0.69314718f
#define M_SQRT3_F		1.73205081f
#define M_IVLN10_F		0.43429448f	// 1 / log(10)
#define M_LOG2_E_F		0.69314718f
#define M_INVLN2_F		1.44269504f	// 1 / log(2)

#include "MC_CONST.h"

#endif /* CHCONST_H_ */
