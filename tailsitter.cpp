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
	gazebo::runWorld(this->world,this->step_size);
}


