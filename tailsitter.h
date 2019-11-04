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
	matrix::Vector3f mc_att_control;
	matrix::Vector3f fw_att_control;
	double rotor[4],left_ele,right_ele;
	double pitch,roll,yaw;

	FILE * logfile;

	void update_info();
	void apply_ctrl();
	void fill_actuator_outputs();
	void log();
	void ctrl_h();
};

#endif /* TAILSITTER_H_ */
