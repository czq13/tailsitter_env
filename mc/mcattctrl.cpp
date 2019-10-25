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
	Vector3f attitude_gain = _attitude_p;
	const float roll_pitch_gain = (attitude_gain(0) + attitude_gain(1)) / 2.f;
	const float yaw_w = math::constrain(attitude_gain(2) / roll_pitch_gain, 0.f, 1.f);
	attitude_gain(2) = roll_pitch_gain;

	/* get estimated and desired vehicle attitude */
	Quatf q(_v_att.q);
	Quatf qd(_v_att_sp.q_d);

	/* ensure input quaternions are exactly normalized because acosf(1.00001) == NaN */
	q.normalize();
	qd.normalize();

	/* calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch */
	Vector3f e_z = q.dcm_z();
	Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (abs(qd_red(1)) > (1.f - 1e-5f) || abs(qd_red(2)) > (1.f - 1e-5f)) {
		/* In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		 * full attitude control anyways generates no yaw input and directly takes the combination of
		 * roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable. */
		qd_red = qd;

	} else {
		/* transform rotation from current to desired thrust vector into a world frame reduced desired attitude */
		qd_red *= q;
	}

	/* mix full and reduced desired attitude */
	Quatf q_mix = qd_red.inversed() * qd;
	q_mix *= math::signNoZero(q_mix(0));
	/* catch numerical problems with the domain of acosf and asinf */
	q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
	q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);
	qd = qd_red * Quatf(cosf(yaw_w * acosf(q_mix(0))), 0, 0, sinf(yaw_w * asinf(q_mix(3))));

	/* quaternion attitude control law, qe is rotation from q to qd */
	Quatf qe = q.inversed() * qd;

	/* using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	 * also taking care of the antipodal unit quaternion ambiguity */
	Vector3f eq = 2.f * math::signNoZero(qe(0)) * qe.imag();

	/* calculate angular rates setpoint */
	_rates_sp = eq.emult(attitude_gain);

	/* Feed forward the yaw setpoint rate.
	 * The yaw_feedforward_rate is a commanded rotation around the world z-axis,
	 * but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	 * Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
	 * and multiply it by the yaw setpoint rate (yaw_sp_move_rate) and gain (_yaw_ff).
	 * This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
	 * such that it can be added to the rates setpoint.
	 */
	Vector3f yaw_feedforward_rate = q.inversed().dcm_z();
	yaw_feedforward_rate *= _v_att_sp.yaw_sp_move_rate * _yaw_ff.get();
	_rates_sp += yaw_feedforward_rate;


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
Vector3f
mc_att_ctrl::pid_attenuations(float tpa_breakpoint, float tpa_rate)
{
	/* throttle pid attenuation factor */
	float tpa = 1.0f - tpa_rate * (fabsf(_v_rates_sp.thrust) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
	tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

	Vector3f pidAttenuationPerAxis;
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
	Vector3f rates;

	rates(0) = _v_att.rollspeed;
	rates(1) = _v_att.pitchspeed;
	rates(2) = _v_att.yawspeed;

	Vector3f rates_p_scaled = _rate_p.emult(pid_attenuations(MC_TPA_BREAK_P, MC_TPA_RATE_P));
	Vector3f rates_i_scaled = _rate_i.emult(pid_attenuations(MC_TPA_BREAK_I, MC_TPA_RATE_I));
	Vector3f rates_d_scaled = _rate_d.emult(pid_attenuations(MC_TPA_BREAK_D, MC_TPA_RATE_I));

	/* angular rates error */
	Vector3f rates_err = _rates_sp - rates;

	/* apply low-pass filtering to the rates for D-term */
	Vector3f rates_filtered(
		_lp_filters_d[0].apply(rates(0)),
		_lp_filters_d[1].apply(rates(1)),
		_lp_filters_d[2].apply(rates(2)));

	_att_control = rates_p_scaled.emult(rates_err) +
		       _rates_int -
		       rates_d_scaled.emult(rates_filtered - _rates_prev_filtered) / dt +
		       _rate_ff.emult(_rates_sp);

	_rates_prev = rates;
	_rates_prev_filtered = rates_filtered;

	/* update integral only if motors are providing enough thrust to be effective */
	if (_thrust_sp > MIN_TAKEOFF_THRUST) {
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
