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

class tailsitter {
public:
	tailsitter();
	virtual ~tailsitter();
	gazebo::physics::WorldPtr world;
	gazebo::physics::JointPtr rotor1,rotor2,rotor3,rotor0,left_elevon,right_elevon;
	gazebo::physics::ModelPtr model;
	int step_size;
	void run_world();
};

#endif /* TAILSITTER_H_ */
