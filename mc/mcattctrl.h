/*
 * mcattctrl.h
 *
 *  Created on: Oct 22, 2019
 *      Author: czq
 */

#ifndef MC_MCATTCTRL_H_
#define MC_MCATTCTRL_H_

#include "Chconst.h"
#include <matrix/math.hpp>
#include <math/filter/LowPassFilter2p.hpp>
#include <math/Limits.hpp>
#include <math/Functions.hpp>
namespace MC {

class mc_att_ctrl {
public:
	mc_att_ctrl();
	virtual ~mc_att_ctrl();

	struct vehicle_attitude_setpoint_s _v_att_sp;
	struct vehicle_attitude_s _v_att;
	matrix::Vector3f _att_control;			/**< attitude control vector */

	void control_attitude(float dt);
	matrix::Vector3f pid_attenuations(float tpa_breakpoint, float tpa_rate);
	void control_attitude_rates(float dt);

	matrix::Vector3f _attitude_p;		/**< P gain for attitude control */
	matrix::Vector3f _rate_p;		/**< P gain for angular rate error */
	matrix::Vector3f _rate_i;		/**< I gain for angular rate error */
	matrix::Vector3f _rate_int_lim;		/**< integrator state limit for rate loop */
	matrix::Vector3f _rate_d;		/**< D gain for angular rate error */
	matrix::Vector3f _rate_ff;		/**< Feedforward gain for desired rates */

	matrix::Vector3f _mc_rate_max;		/**< attitude rate limits in stabilized modes */
	matrix::Vector3f _auto_rate_max;	/**< attitude rate limits in auto modes */
	matrix::Vector3f _acro_rate_max;	/**< max attitude rates in acro mode */

	math::LowPassFilter2p _lp_filters_d[3];
	static constexpr const float initial_update_rate_hz = 250.f; /**< loop update rate used for initialization */
	float _loop_update_rate_hz{initial_update_rate_hz};          /**< current rate-controller loop update rate in [Hz] */
	float _d_term_cutoff_freq,thrust_sp;
	matrix::Vector3f _rates_prev;			/**< angular rates on previous step */
	matrix::Vector3f _rates_prev_filtered;		/**< angular rates on previous step (low-pass filtered) */
	matrix::Vector3f _rates_sp;			/**< angular rates setpoint */
	matrix::Vector3f _rates_int;			/**< angular rates integral error */
};

} /* namespace MC */

#endif /* MC_MCATTCTRL_H_ */
