/*
 * MC_CONST.h
 *
 *  Created on: Oct 22, 2019
 *      Author: czq
 */

#ifndef MC_CONST_H_
#define MC_CONST_H_

#define MC_ROLL_P 6.0f
#define MC_ROLLRATE_P 0.2f
#define MC_ROLLRATE_I 0.05f
#define MC_RR_INT_LIM 0.30f
#define MC_ROLLRATE_D 0.003f
#define MC_ROLLRATE_FF 0.0f
#define MC_PITCH_P 6.0f
#define MC_PITCHRATE_P 0.2f
#define MC_PITCHRATE_I 0.05f
#define MC_PR_INT_LIM 0.30f
#define MC_PITCHRATE_D 0.003f
#define MC_PITCHRATE_FF 0.0f
#define MC_YAW_P 2.8f
#define MC_YAWRATE_P 0.2f
#define MC_YAWRATE_I 0.1f
#define MC_YR_INT_LIM 0.30f
#define MC_YAWRATE_D 0.0f
#define MC_YAWRATE_FF 0.0f
#define MC_YAW_FF 0.5f
#define MC_ROLLRATE_MAX 220.0f
#define MC_PITCHRATE_MAX 220.0f
#define MC_YAWRATE_MAX 200.0f
#define MC_YAWRAUTO_MAX 45.0f
#define MC_ACRO_R_MAX 720.0f
#define MC_ACRO_P_MAX 720.0f
#define MC_ACRO_Y_MAX 540.0f
#define MC_ACRO_EXPO 0.69f
#define MC_ACRO_EXPO_Y 0.69f
#define MC_ACRO_SUPEXPO 0.7f
#define MC_ACRO_SUPEXPOY 0.7f
#define MC_RATT_TH 0.8f
#define MC_BAT_SCALE_EN 0
#define MC_TPA_BREAK_P 1.0f
#define MC_TPA_BREAK_I 1.0f
#define MC_TPA_BREAK_D 1.0f
#define MC_TPA_RATE_P 0.0f
#define MC_TPA_RATE_I 0.0f
#define MC_TPA_RATE_D 0.0f
#define MC_DTERM_CUTOFF 30.f
#define MC_AIRMODE 0

#define FW_R_TC 0.4f
#define FW_P_TC 0.4f
#define FW_PR_P 0.08f
#define FW_PR_I 0.02f
#define FW_P_RMAX_POS 60.0f
#define FW_P_RMAX_NEG 60.0f
#define FW_PR_IMAX 0.4f
#define FW_RR_P 0.05f
#define FW_RR_I 0.01f
#define FW_RR_IMAX 0.2f
#define FW_R_RMAX 70.0f
#define FW_YR_P 0.05f
#define FW_YR_I 0.01f
#define FW_YR_IMAX 0.2f
#define FW_Y_RMAX 50.0f
#define FW_RLL_TO_YAW_FF 0.0f
#define FW_W_EN 0
#define FW_WR_P 0.5f
#define FW_WR_I 0.1f
#define FW_WR_IMAX 1.0f
#define FW_W_RMAX 30.0f
#define FW_RR_FF 0.5f
#define FW_PR_FF 0.5f
#define FW_YR_FF 0.3f
#define FW_WR_FF 0.2f
#define FW_RSP_OFF 0.0f
#define FW_PSP_OFF 0.0f
#define FW_MAN_R_MAX 45.0f
#define FW_MAN_P_MAX 45.0f
#define FW_FLAPS_SCL 1.0f
#define FW_FLAPERON_SCL 0.0f
#define FW_ARSP_MODE 0
#define FW_MAN_R_SC 1.0f
#define FW_MAN_P_SC 1.0f
#define FW_MAN_Y_SC 1.0f
#define FW_BAT_SCALE_EN 0
#define FW_ACRO_X_MAX 90
#define FW_ACRO_Y_MAX 90
#define FW_ACRO_Z_MAX 45
#define FW_RATT_TH 0.8f
#define FW_DTRIM_R_VMIN 0.0f
#define FW_DTRIM_P_VMIN 0.0f
#define FW_DTRIM_Y_VMIN 0.0f
#define FW_DTRIM_R_VMAX 0.0f
#define FW_DTRIM_P_VMAX 0.0f
#define FW_DTRIM_Y_VMAX 0.0f
#define FW_DTRIM_R_FLPS 0.0f
#define FW_DTRIM_P_FLPS 0.0f

#endif /* MC_CONST_H_ */
