/*
 * tailsitter.h
 *
 *  Created on: Oct 21, 2019
 *      Author: czq
 */

#ifndef TAILSITTER_H_
#define TAILSITTER_H_

#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>

#include "ecl_pitch_controller.h"
#include "ecl_roll_controller.h"
#include "ecl_yaw_controller.h"
#include "ecl_controller.h"
#include "mcattctrl.h"
#include "Chconst.h"

class tailsitter {
public:
	tailsitter();
	virtual ~tailsitter();
	gazebo::physics::WorldPtr world;
	gazebo::physics::JointPtr rotor1,rotor2,rotor3,rotor0,left_elevon,right_elevon;
	gazebo::physics::ModelPtr model;
	int step_size;
	void run_world();

	MC::mc_att_ctrl * mc_ctrl;
	struct vehicle_attitude_s _v_att;
	struct vehicle_local_position_s _local_pos;
	struct vehicle_attitude_setpoint_s _v_att_sp;
	matrix::Vector3f mc_att_control;
	matrix::Vector3f fw_att_control;
	matrix::Vector3f ts_att_control;
	double rotor[4],left_ele,right_ele;
	double pitch,roll,yaw,fwpitch,fwroll,fwyaw,speed,thrust_sp,pitch_weight,roll_weight,yaw_weight;
	double ts_roll,ts_pitch;
	double vxb,vyb,vzb;

	struct ECL_ControlData control_input = {};
	ECL_RollController	_roll_ctrl;
	ECL_PitchController	_pitch_ctrl;
	ECL_YawController	_yaw_ctrl;

	FILE * logfile;

	void get_mc_ctrl();
	void get_fx_ctrl();
	void get_ts_ctrl();
	void init_fw_state();
	void update_info();
	void apply_ctrl();
	void fill_mc_actuator_outputs();
	void fill_fw_actuator_outputs();
	void fill_ts_actuator_outputs();
	void log();
	void mc_ctrl_h();
	void fw_ctrl_speed();
};

#endif /* TAILSITTER_H_ */
