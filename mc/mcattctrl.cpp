/*
 * mcattctrl.cpp
 *
 *  Created on: Oct 22, 2019
 *      Author: czq
 */

#include "mcattctrl.h"

namespace MC {
#define MIN_TAKEOFF_THRUST    0.2f
#define TPA_RATE_LOWER_LIMIT 0.05f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

mc_att_ctrl::mc_att_ctrl() : _lp_filters_d{
	{initial_update_rate_hz, 50.f},
	{initial_update_rate_hz, 50.f},
	{initial_update_rate_hz, 50.f}}
{
	_rates_prev.zero();
	_rates_prev_filtered.zero();
	_rates_sp.zero();
	_rates_int.zero();
	_att_control.zero();
	// TODO Auto-generated constructor stub
	/* roll gains */
	_attitude_p(0) = MC_ROLL_P;
	_rate_p(0) = MC_ROLLRATE_P;
	_rate_i(0) = MC_ROLLRATE_I;
	_rate_int_lim(0) = MC_RR_INT_LIM;
	_rate_d(0) = MC_ROLLRATE_D;
	_rate_ff(0) = MC_ROLLRATE_FF;

	/* pitch gains */
	_attitude_p(1) = MC_PITCH_P;
	_rate_p(1) = MC_PITCHRATE_P;
	_rate_i(1) = MC_PITCHRATE_I;
	_rate_int_lim(1) = MC_PR_INT_LIM;
	_rate_d(1) = MC_PITCHRATE_D;
	_rate_ff(1) = MC_PITCHRATE_FF;

	/* yaw gains */
	_attitude_p(2) = MC_YAW_P;
	_rate_p(2) = MC_YAWRATE_P;
	_rate_i(2) = MC_YAWRATE_I;
	_rate_int_lim(2) = MC_YR_INT_LIM;
	_rate_d(2) = MC_YAWRATE_D;
	_rate_ff(2) = MC_YAWRATE_FF;
	_d_term_cutoff_freq = MC_DTERM_CUTOFF;

	if (fabsf(_lp_filters_d[0].get_cutoff_freq() - _d_term_cutoff_freq) > 0.01f) {
		_lp_filters_d[0].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq);
		_lp_filters_d[1].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq);
		_lp_filters_d[2].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq);
		_lp_filters_d[0].reset(_rates_prev(0));
		_lp_filters_d[1].reset(_rates_prev(1));
		_lp_filters_d[2].reset(_rates_prev(2));
	}

	/* angular rate limits */
	_mc_rate_max(0) = math::radians(MC_ROLLRATE_MAX);
	_mc_rate_max(1) = math::radians(MC_PITCHRATE_MAX);
	_mc_rate_max(2) = math::radians(MC_YAWRATE_MAX);

	/* auto angular rate limits */
	_auto_rate_max(0) = math::radians(MC_ROLLRATE_MAX);
	_auto_rate_max(1) = math::radians(MC_PITCHRATE_MAX);
	_auto_rate_max(2) = math::radians(MC_YAWRAUTO_MAX);

	/* manual rate control acro mode rate limits and expo */
	_acro_rate_max(0) = math::radians(MC_ACRO_R_MAX);
	_acro_rate_max(1) = math::radians(MC_ACRO_P_MAX);
	_acro_rate_max(2) = math::radians(MC_ACRO_Y_MAX);

}

mc_att_ctrl::~mc_att_ctrl() {
	// TODO Auto-generated destructor stub
}
void
mc_att_ctrl::control_attitude(float dt)
{
	//_thrust_sp = att_set.thrust;

	/* prepare yaw weight from the ratio between roll/pitch and yaw gains */
	matrix::Vector3f attitude_gain = _attitude_p;
	const float roll_pitch_gain = (attitude_gain(0) + attitude_gain(1)) / 2.f;
	const float yaw_w = math::constrain(attitude_gain(2) / roll_pitch_gain, 0.f, 1.f);
	attitude_gain(2) = roll_pitch_gain;

	/* get estimated and desired vehicle attitude */
	matrix::Quatf q(_v_att.q);
	matrix::Quatf qd(_v_att_sp.q_d);
	//printf("in mc_att_ctrl:q(0)=%f,(1)=%f,(2)=%f,(3)=%f\n",q(0),q(1),q(2),q(3));
	//printf("in mc_att_ctrl:qd(0)=%f,(1)=%f,(2)=%f,(3)=%f\n",qd(0),qd(1),qd(2),qd(3));
	/* ensure input quaternions are exactly normalized because acosf(1.00001) == NaN */
	q.normalize();
	qd.normalize();

	/* calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch */
	matrix::Vector3f e_z = q.dcm_z();
	matrix::Vector3f e_z_d = qd.dcm_z();
	matrix::Quatf qd_red(e_z, e_z_d);


	if (abs(qd_red(1)) > (1.f - 1e-5f) || abs(qd_red(2)) > (1.f - 1e-5f)) {
		/* In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		 * full attitude control anyways generates no yaw input and directly takes the combination of
		 * roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable. */
		qd_red = qd;

	} else {
		/* transform rotation from current to desired thrust vector into a world frame reduced desired attitude */
		qd_red *= q;
	}
	//printf("in mc_att_ctrl:qd_red(0)=%f,(1)=%f,(2)=%f,(3)=%f\n",qd_red(0),qd_red(1),qd_red(2),qd_red(3));
	/* mix full and reduced desired attitude */
	matrix::Quatf q_mix = qd_red.inversed() * qd;
	q_mix *= math::signNoZero(q_mix(0));
	/* catch numerical problems with the domain of acosf and asinf */
	q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
	q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);
	qd = qd_red * matrix::Quatf(cosf(yaw_w * acosf(q_mix(0))), 0, 0, sinf(yaw_w * asinf(q_mix(3))));

	/* quaternion attitude control law, qe is rotation from q to qd */
	matrix::Quatf qe = q.inversed() * qd;
	//printf("in mc_att_ctrl:qe(0)=%f,(1)=%f,(2)=%f,(3)=%f\n",qe(0),qe(1),qe(2),qe(3));
	/* using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	 * also taking care of the antipodal unit quaternion ambiguity */
	matrix::Vector3f eq = 2.f * math::signNoZero(qe(0)) * qe.imag();

	/* calculate angular rates setpoint */
	_rates_sp = eq.emult(attitude_gain);
	//printf("in mc_att_ctrl:_rates(1)=%f,(2)=%f,(3)=%f\n",_rates_sp(0),_rates_sp(1),_rates_sp(2));
	/* Feed forward the yaw setpoint rate.
	 * The yaw_feedforward_rate is a commanded rotation around the world z-axis,
	 * but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	 * Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
	 * and multiply it by the yaw setpoint rate (yaw_sp_move_rate) and gain (_yaw_ff).
	 * This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
	 * such that it can be added to the rates setpoint.
	 */
	matrix::Vector3f yaw_feedforward_rate = q.inversed().dcm_z();
	yaw_feedforward_rate *= 0.0f * MC_YAW_FF;
	_rates_sp += yaw_feedforward_rate;
	//printf("in mc_att_ctrl:yaw_feedforward_rate(1)=%f,(2)=%f,(3)=%f\n",yaw_feedforward_rate(0),yaw_feedforward_rate(1),yaw_feedforward_rate(2));
	printf("in mc_att_ctrl:_rates_sp(1)=%f,(2)=%f,(3)=%f\n",_rates_sp(0),_rates_sp(1),_rates_sp(2));
	/* limit rates */
	for (int i = 0; i < 3; i++) {
		_rates_sp(i) = math::constrain(_rates_sp(i), -_auto_rate_max(i), _auto_rate_max(i));
	}

}

/*
 * Throttle PID attenuation
 * Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
 * Input: 'tpa_breakpoint', 'tpa_rate', '_thrust_sp'
 * Output: 'pidAttenuationPerAxis' vector
 */
matrix::Vector3f
mc_att_ctrl::pid_attenuations(float tpa_breakpoint, float tpa_rate)
{
	/* throttle pid attenuation factor */
	float tpa = 1.0f - tpa_rate * (fabsf(thrust_sp) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
	tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

	matrix::Vector3f pidAttenuationPerAxis;
	pidAttenuationPerAxis(0) = tpa;
	pidAttenuationPerAxis(1) = tpa;
	pidAttenuationPerAxis(2) = 1.0;

	return pidAttenuationPerAxis;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
mc_att_ctrl::control_attitude_rates(float dt)
{
	// get the raw gyro data and correct for thermal errors
	matrix::Vector3f rates;

	rates(0) = _v_att.rollspeed;
	rates(1) = _v_att.pitchspeed;
	rates(2) = _v_att.yawspeed;

	matrix::Vector3f rates_p_scaled = _rate_p.emult(pid_attenuations(MC_TPA_BREAK_P, MC_TPA_RATE_P));
	matrix::Vector3f rates_i_scaled = _rate_i.emult(pid_attenuations(MC_TPA_BREAK_I, MC_TPA_RATE_I));
	matrix::Vector3f rates_d_scaled = _rate_d.emult(pid_attenuations(MC_TPA_BREAK_D, MC_TPA_RATE_I));

	/* angular rates error */
	printf("rates(0)=%f,(1)=%f,(2)=%f\n",rates(0),rates(1),rates(2));
	matrix::Vector3f rates_err = _rates_sp - rates;
	printf("rates_err[0]=%f,[1]=%f,[2]=%f\n",rates_err(0),rates_err(1),rates_err(2));

	/* apply low-pass filtering to the rates for D-term */
	matrix::Vector3f rates_filtered(
		_lp_filters_d[0].apply(rates(0)),
		_lp_filters_d[1].apply(rates(1)),
		_lp_filters_d[2].apply(rates(2)));
	//printf("rates_filtered[0]=%f,[1]=%f,[2]=%f,dt=%f\n",rates_filtered(0),rates_filtered(1),rates_filtered(2),dt);
	printf("rates_p_scaled(0)=%f,(1)=%f,(2)=%f\n",rates_p_scaled(0),rates_p_scaled(1),rates_p_scaled(2));
	matrix::Vector3f v1 = rates_p_scaled.emult(rates_err);
	matrix::Vector3f v2 = _rates_int;
	matrix::Vector3f v3 =  rates_d_scaled.emult(rates_filtered - _rates_prev_filtered) / dt;
	matrix::Vector3f v4 = _rate_ff.emult(_rates_sp);
	printf("v1(0)=%f,(1)=%f,(2)=%f\n",v1(0),v1(1),v1(2));
	printf("v2(0)=%f,(1)=%f,(2)=%f\n",v2(0),v2(1),v2(2));
	printf("v3(0)=%f,(1)=%f,(2)=%f\n",v3(0),v3(1),v3(2));
	printf("v4(0)=%f,(1)=%f,(2)=%f\n",v4(0),v4(1),v4(2));
	_att_control = rates_p_scaled.emult(rates_err) +
		       _rates_int -
		       rates_d_scaled.emult(rates_filtered - _rates_prev_filtered) / dt +
		       _rate_ff.emult(_rates_sp);
	printf("_att_control(0)=%f,(1)=%f,(2)=%f\n",_att_control(0),_att_control(1),_att_control(2));
	_rates_prev = rates;
	_rates_prev_filtered = rates_filtered;

	/* update integral only if motors are providing enough thrust to be effective */
	if (thrust_sp > MIN_TAKEOFF_THRUST) {
		for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {

			// Perform the integration using a first order method and do not propagate the result if out of range or invalid
			float rate_i = _rates_int(i) + rates_i_scaled(i) * rates_err(i) * dt;

			if (std::isfinite(rate_i) && rate_i > -_rate_int_lim(i) && rate_i < _rate_int_lim(i)) {
				_rates_int(i) = rate_i;

			}
		}
	}

	/* explicitly limit the integrator state */
	for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
		_rates_int(i) = math::constrain(_rates_int(i), -_rate_int_lim(i), _rate_int_lim(i));

	}
}

} /* namespace MC */
