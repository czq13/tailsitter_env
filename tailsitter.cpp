/*
 * tailsitter.cpp
 *
 *  Created on: Oct 21, 2019
 *      Author: czq
 */

#include "tailsitter.h"


tailsitter::tailsitter() {
	// TODO Auto-generated constructor stub
	std::string file_name = "/home/czq/chWorkspace/tailsitter_env/fw/gazebo_model/tailsitter.world";
	//std::string file_name = "/home/czq/eclipse-workspace/tailsitter_env/fw/gazebo_model/tailsitter.world";
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
		this->step_size = 1;
		logfile = fopen("log.txt","w");
		mc_ctrl = new MC::mc_att_ctrl();
		_v_att.pre_timestamp = _v_att.timestamp = 0;
		_local_pos.pre_timestamp = _local_pos.timestamp = 0;
	} catch(gazebo::common::Exception &e) {
		std::cerr << "ERROR: " << e << std::endl;
		return;
	}
	_pitch_ctrl.set_time_constant(FW_P_TC);
	_pitch_ctrl.set_k_p(FW_PR_P);
	_pitch_ctrl.set_k_i(FW_PR_I);
	_pitch_ctrl.set_k_ff(FW_PR_FF);
	_pitch_ctrl.set_integrator_max(FW_PR_IMAX);

	/* roll control parameters */
	_roll_ctrl.set_time_constant(FW_R_TC);
	_roll_ctrl.set_k_p(FW_RR_P);
	_roll_ctrl.set_k_i(FW_RR_I);
	_roll_ctrl.set_k_ff(FW_RR_FF);
	_roll_ctrl.set_integrator_max(FW_RR_IMAX);

		/* yaw control parameters */
	_yaw_ctrl.set_k_p(FW_YR_P);
	_yaw_ctrl.set_k_i(FW_YR_I);
	_yaw_ctrl.set_k_ff(FW_YR_FF);
	_yaw_ctrl.set_integrator_max(FW_YR_IMAX);
	_roll_ctrl.set_max_rate(math::radians(FW_R_RMAX));
	_pitch_ctrl.set_max_rate_pos(math::radians(FW_P_RMAX_POS));
	_pitch_ctrl.set_max_rate_neg(math::radians(FW_P_RMAX_NEG));
	_yaw_ctrl.set_max_rate(math::radians(FW_Y_RMAX));

}

tailsitter::~tailsitter() {
	// TODO Auto-generated destructor stub
}
void tailsitter::run_world() {
	//run ctrl
	update_info();
	fw_ctrl_speed();
	get_fx_ctrl();
	fill_fw_actuator_outputs();
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
	_v_att.rollspeed = angvel[0];
	_v_att.pitchspeed = angvel[1];
	_v_att.yawspeed = angvel[2];
	_v_att.q[0] = ang.W();
	_v_att.q[1] = ang.X();
	_v_att.q[2] = ang.Y();
	_v_att.q[3] = ang.Z();
	//printf("q[0]=%f,[1]=%f,[2]=%f,[3]=%f\n",_v_att.q[0],_v_att.q[1],_v_att.q[2],_v_att.q[3]);
	_v_att.pre_timestamp = _v_att.timestamp;
	_v_att.timestamp = (this->world->SimTime()).Double();
	//printf("angaccel(0)=%f,(1)=%f,(2)=%f\n",angaccel[0],angaccel[1],angaccel[2]);

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

	matrix::Dcmf R = matrix::Quatf(_v_att.q);
	matrix::Dcmf R_adapted = R;		//modified rotation matrix

	/* move z to x */
	R_adapted(0, 0) = R(0, 2);
	R_adapted(1, 0) = R(1, 2);
	R_adapted(2, 0) = R(2, 2);

	/* move x to z */
	R_adapted(0, 2) = R(0, 0);
	R_adapted(1, 2) = R(1, 0);
	R_adapted(2, 2) = R(2, 0);

	/* change direction of pitch (convert to right handed system) */
	R_adapted(0, 0) = -R_adapted(0, 0);
	R_adapted(1, 0) = -R_adapted(1, 0);
	R_adapted(2, 0) = -R_adapted(2, 0);

	/* fill in new attitude data */
	R = R_adapted;
	matrix::Eulerf euler_angles(R);
	fwroll = euler_angles.phi();
	fwpitch = euler_angles.theta();
	fwyaw = euler_angles.psi();
	speed = sqrt(linearvel[0]*linearvel[0]+linearvel[1]*linearvel[1]+linearvel[2]*linearvel[2]);
}
void tailsitter::mc_ctrl_h() {
	mc_ctrl->thrust_sp = 0.75 + 0.5 * (0-_local_pos.vz);
	printf("thrust_sp=%f",mc_ctrl->thrust_sp);
}
void tailsitter::fw_ctrl_speed() {
	fwthrust = 0.5 + (22 - (speed));

}
void tailsitter::apply_ctrl() {
	this->rotor0->SetVelocity(0,math::constrain(rotor[0] * 120,0.0,120.0));
	this->rotor1->SetVelocity(0,math::constrain(rotor[1] * 120,0.0,120.0));
	this->rotor2->SetVelocity(0,math::constrain(rotor[2] * 120,0.0,120.0));
	this->rotor3->SetVelocity(0,math::constrain(rotor[3] * 120,0.0,120.0));
	this->left_elevon->SetPosition(0,left_ele);
	this->right_elevon->SetPosition(0,right_ele);
}
void tailsitter::fill_mc_actuator_outputs(){
	rotor[0] = mc_ctrl->thrust_sp - mc_att_control(0) - mc_att_control(1);
	rotor[1] = mc_ctrl->thrust_sp + mc_att_control(0) + mc_att_control(1);
	rotor[2] = mc_ctrl->thrust_sp + mc_att_control(0) - mc_att_control(1);
	rotor[3] = mc_ctrl->thrust_sp - mc_att_control(0) + mc_att_control(1);
	//printf("rotor0=%f,1=%f,2=%f,3=%f\n",rotor[0],rotor[1],rotor[2],rotor[3]);
	left_ele = -mc_att_control(2);
	right_ele = mc_att_control(2);
}
void tailsitter::fill_fw_actuator_outputs() {
	rotor[0] = fwthrust;
	rotor[1] = fwthrust;
	rotor[2] = fwthrust;
	rotor[3] = fwthrust;
	left_ele = fw_att_control(1)+fw_att_control(0);
	right_ele = fw_att_control(1)-fw_att_control(0);
	printf("left_ele=%lf,right_ele=%lf\n",left_ele,right_ele);
}
void tailsitter::get_mc_ctrl() {
	mc_ctrl->_v_att = _v_att;
	mc_ctrl->_v_att_sp.q_d[0] = 1.0f;mc_ctrl->_v_att_sp.q_d[1] = 0.0f;mc_ctrl->_v_att_sp.q_d[2] = 0.0f;mc_ctrl->_v_att_sp.q_d[3] = 0.0f;
	double dt = math::max(_v_att.timestamp-_v_att.pre_timestamp,1.0e-6);
	mc_ctrl->control_attitude(dt);
	mc_ctrl->control_attitude_rates(dt);
	mc_att_control = mc_ctrl->_att_control;
}
void tailsitter::get_fx_ctrl() {
	control_input.roll = fwroll;
	control_input.pitch = fwpitch;
	control_input.yaw = fwyaw;
	control_input.body_x_rate = _v_att.rollspeed;
	control_input.body_y_rate = _v_att.pitchspeed;
	control_input.body_z_rate = _v_att.yawspeed;
	control_input.roll_setpoint = 5.0/57.3;
	control_input.pitch_setpoint = 10.0/57.3;
	control_input.yaw_setpoint = fwyaw;
	control_input.airspeed_min = 14;
	control_input.airspeed_max = 30;
	control_input.airspeed = speed;
	control_input.lock_integrator = false;
	control_input.groundspeed = speed;
	float airspeed_scaling = 16.0f / ((speed < 14.0f) ? 14.0f: speed);
	control_input.scaler = airspeed_scaling;
	_roll_ctrl.control_attitude(control_input);
	_pitch_ctrl.control_attitude(control_input);
	_yaw_ctrl.control_attitude(control_input); //runs last, because is depending on output of roll and pitch attitude

	/* Update input data for rate controllers */
	control_input.roll_rate_setpoint = _roll_ctrl.get_desired_rate();
	control_input.pitch_rate_setpoint = _pitch_ctrl.get_desired_rate();
	printf("pitch_rate_sp=%f\n",control_input.pitch_rate_setpoint);
	control_input.yaw_rate_setpoint = _yaw_ctrl.get_desired_rate();

	/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
	float roll_u = _roll_ctrl.control_euler_rate(control_input);
	fw_att_control(0) = (std::isfinite(roll_u)) ? roll_u + 0 : 0;

	if (!std::isfinite(roll_u)) {
		_roll_ctrl.reset_integrator();
	}

	float pitch_u = _pitch_ctrl.control_euler_rate(control_input);
	printf("pitch_u=%f\n",pitch_u);
	fw_att_control(1) = (std::isfinite(pitch_u)) ? pitch_u + 0 : 0;

	if (!std::isfinite(pitch_u)) {
		_pitch_ctrl.reset_integrator();
	}

	float yaw_u = 0.0f;

	yaw_u = _yaw_ctrl.control_euler_rate(control_input);
	fw_att_control(2) = (std::isfinite(yaw_u)) ? yaw_u + 0 : 0;
}
void tailsitter::init_fw_state(){
	ignition::math::Pose3d pose;
	pose.Set(0.0,0.0,0.0,0.0,-1.57,0.0);
	this->model->SetWorldPose(pose);
	ignition::math::Vector3d vel;
	vel[0] = -15.0;
	vel[1] = 0.0;
	vel[2] = 0.0;
	this->model->SetLinearVel(vel);
}
void tailsitter::log() {
	if (!logfile) printf("log fail!\n");
	else {
		fprintf(logfile,"%lf %lf %lf ",_local_pos.vx,_local_pos.vy,_local_pos.vz);
		//fprintf(logfile,"%lf %lf %lf ",roll,pitch,yaw);
		fprintf(logfile,"%lf %lf %lf ",fwroll,fwpitch,fwyaw);
		fprintf(logfile,"%f %f %f ",mc_ctrl->_att_control(0),mc_ctrl->_att_control(1),mc_ctrl->_att_control(2));
		fprintf(logfile,"\n");
	}
}
