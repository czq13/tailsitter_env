/*
 * Chconst.h
 *
 *  Created on: Oct 22, 2019
 *      Author: czq
 */

#ifndef CHCONST_H_
#define CHCONST_H_

struct vehicle_attitude_s {
	float rollspeed;
	float pitchspeed;
	float yawspeed;
	float q[4];
};

struct vehicle_attitude_setpoint_s {
	long long timestamp; // required for logger
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

#include "MC_CONST.h"

#endif /* CHCONST_H_ */
