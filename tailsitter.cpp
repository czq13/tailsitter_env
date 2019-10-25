/*
 * tailsitter.cpp
 *
 *  Created on: Oct 21, 2019
 *      Author: czq
 */

#include "tailsitter.h"


tailsitter::tailsitter() {
	// TODO Auto-generated constructor stub
	std::string file_name = "/home/czq/chWorkspace/tailsitter_env/gazebo_model/tailsitter.world";
	std::string model_name = "tailsitter";
	try {
		std::cerr << "loading world..." << std::endl;
		this->world = gazebo::loadWorld(file_name);
		if (this->world == NULL) {std::cerr << "load world failed!" << std::endl;return;}
		else std::cerr << "load world success\n";
		this->model = this->world->ModelByName(model_name);
		if (this->model == NULL) {std::cerr << "load model failed!\n";return;}
		else std::cerr << "load model success\n";
		std::string rotor0Name = "rotor_0_joint";
		std::string rotor1Name = "rotor_1_joint";
		std::string rotor2Name = "rotor_2_joint";
		std::string rotor3Name = "rotor_3_joint";
		std::string left_elevon_name = "left_elevon_joint";
		std::string right_elevon_name = "right_elevon_joint";
		this->rotor0 = this->model->GetJoint(rotor0Name);
		this->rotor1 = this->model->GetJoint(rotor1Name);
		this->rotor2 = this->model->GetJoint(rotor2Name);
		this->rotor3 = this->model->GetJoint(rotor3Name);
		this->left_elevon = this->model->GetJoint(left_elevon_name);
		this->right_elevon = this->model->GetJoint(right_elevon_name);
		if (this->rotor0 == NULL ||
				this->rotor1 == NULL ||
				this->rotor2 == NULL ||
				this->rotor3 == NULL ||
				this->left_elevon == NULL ||
				this->right_elevon == NULL) {
					std::cerr << "load joint failed!\n";
		}
		else std:: cerr << "load joint success\n";
		this->step_size = 10;
		logfile = fopen("log.txt","w");
		mc_ctrl = new MC::mc_att_ctrl();
		_v_att.pre_timestamp = _v_att.timestamp = 0;
		_local_pos.pre_timestamp = _local_pos.timestamp = 0;
	} catch(gazebo::common::Exception &e) {
		std::cerr << "ERROR: " << e << std::endl;
		return;
	}
}

tailsitter::~tailsitter() {
	// TODO Auto-generated destructor stub
}
void tailsitter::run_world() {
	//run ctrl
	update_info();
	mc_ctrl->_v_att = _v_att;
	double dt = _v_att.timestamp-_v_att.pre_timestamp;
	mc_ctrl->control_attitude(dt);
	mc_ctrl->control_attitude_rates(dt);
	mc_att_control = mc_ctrl->_att_control;
	fill_actuator_outputs();
	apply_ctrl();
	gazebo::runWorld(this->world,this->step_size);
	log();
}
void tailsitter::update_info() {
	ignition::math::Pose3d pose = this->model->WorldPose();
	ignition::math::Vector3d angaccel = this->model->WorldAngularAccel();
	ignition::math::Vector3d angvel = this->model->WorldAngularVel();
	ignition::math::Vector3d linearaccel = this->model->WorldLinearAccel();
	ignition::math::Vector3d linearvel = this->model->WorldLinearVel();

	ignition::math::Quaterniond ang = pose.Rot();
	ignition::math::Vector3d pos = pose.Pos();
	_v_att.rollspeed = angaccel[0];
	_v_att.pitchspeed = angaccel[1];
	_v_att.yawspeed = angaccel[2];
	_v_att.q[0] = ang.W();
	_v_att.q[1] = ang.X();
	_v_att.q[2] = ang.Y();
	_v_att.q[3] = ang.Z();
	_v_att.pre_timestamp = _v_att.timestamp;
	_v_att.timestamp = (this->world->SimTime()).Double();


	_local_pos.ax = linearaccel[0];
	_local_pos.ay = linearaccel[1];
	_local_pos.az = linearaccel[2];
	_local_pos.vx = linearvel[0];
	_local_pos.vy = linearvel[1];
	_local_pos.vz = linearvel[2];
	_local_pos.x = pos[0];
	_local_pos.y = pos[1];
	_local_pos.z = pos[2];
	_local_pos.pre_timestamp = _local_pos.timestamp;
	_local_pos.timestamp = (this->world->SimTime()).Double();
	roll = ang.Roll();
	pitch = ang.Pitch();
	yaw = ang.Yaw();
}
void tailsitter::apply_ctrl() {
	this->rotor0->SetVelocity(0,rotor[0] * 120);
	this->rotor1->SetVelocity(0,rotor[1] * 120);
	this->rotor2->SetVelocity(0,rotor[2] * 120);
	this->rotor3->SetVelocity(0,rotor[3] * 120);
	this->left_elevon->SetPosition(0,left_ele);
	this->right_elevon->SetPosition(0,right_ele);
}
void tailsitter::fill_actuator_outputs(){
	rotor[0] = 0.753 + mc_att_control[0] + mc_att_control[1];
	rotor[1] = 0.753 - mc_att_control[0] - mc_att_control[1];
	rotor[2] = 0.753 + mc_att_control[0] - mc_att_control[1];
	rotor[3] = 0.753 - mc_att_control[0] + mc_att_control[1];
	left_ele = mc_att_control[2];
	right_ele = -mc_att_control[2];
}
void tailsitter::log() {
	if (!logfile) printf("log fail!\n");
	else {
		fprintf(logfile,"%f %f %f ",_local_pos.x,_local_pos.y,_local_pos.z);
		fprintf(logfile,"%f %f %f ",_local_pos.vx,_local_pos.vy,_local_pos.vz);
		fprintf(logfile,"%lf %lf %lf ",roll,pitch,yaw);
		fprintf(logfile,"\n");
	}
}
