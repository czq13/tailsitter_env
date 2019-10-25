/*
 * Chconst.h
 *
 *  Created on: Oct 22, 2019
 *      Author: czq
 */

#ifndef CHCONST_H_
#define CHCONST_H_

struct vehicle_attitude_s {
	long long timestamp;
	long long pre_timestamp;
	float rollspeed;
	float pitchspeed;
	float yawspeed;
	float q[4];
};

struct vehicle_attitude_setpoint_s {
	long long timestamp; // required for logger
	long long pre_timestamp;
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
	long long timestamp; // required for logger
	long long pre_timestamp;
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

#include "MC_CONST.h"

#endif /* CHCONST_H_ */
