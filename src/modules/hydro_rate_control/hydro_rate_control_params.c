/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file hydro_rate_control_params.c
 *
 * Parameters defined by the fixed-wing rate control task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/**
 * Pitch rate proportional gain.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_PR_P, 0.08f);

/**
 * Pitch rate derivative gain.
 *
 * Pitch rate differential gain.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_PR_D, 0.f);

/**
 * Pitch rate integrator gain.
 *
 * @unit %/rad
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_PR_I, 0.1f);

/**
 * Pitch rate integrator limit
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_PR_IMAX, 0.4f);

/**
 * Roll rate proportional gain
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_RR_P, 0.05f);

/**
 * Roll rate derivative gain
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_RR_D, 0.0f);

/**
 * Roll rate integrator gain
 *
 * @unit %/rad
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.01
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_RR_I, 0.1f);

/**
 * Roll integrator limit
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_RR_IMAX, 0.2f);

/**
 * Yaw rate proportional gain
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_YR_P, 0.05f);

/**
 * Yaw rate derivative gain
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_YR_D, 0.0f);

/**
 * Yaw rate integrator gain
 *
 * @unit %/rad
 * @min 0.0
 * @max 10
 * @decimal 1
 * @increment 0.5
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_YR_I, 0.1f);

/**
 * Yaw rate integrator limit
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_YR_IMAX, 0.2f);

/**
 * Roll rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_RR_FF, 0.5f);

/**
 * Pitch rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_PR_FF, 0.5f);

/**
 * Yaw rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_YR_FF, 0.3f);

/**
 * Acro body roll max rate setpoint
 *
 * @min 10
 * @max 720
 * @unit deg
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_ACRO_X_MAX, 90);

/**
 * Acro body pitch max rate setpoint
 *
 * @min 10
 * @max 720
 * @unit deg
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_ACRO_Y_MAX, 90);

/**
 * Acro body yaw max rate setpoint
 *
 * @min 10
 * @max 720
 * @unit deg
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_ACRO_Z_MAX, 45);

/**
 * Enable throttle scale by battery level
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery.
 *
 * @boolean
 * @group Hydro Rate Control
 */
PARAM_DEFINE_INT32(HY_BAT_SCALE_EN, 0);

/**
 * Enable airspeed scaling
 *
 * This enables a logic that automatically adjusts the output of the rate controller to take
 * into account the real torque produced by an aerodynamic control surface given
 * the current deviation from the trim airspeed (HY_AIRSPD_TRIM).
 *
 * Enable when using aerodynamic control surfaces (e.g.: plane)
 * Disable when using rotor wings (e.g.: autogyro)
 *
 * @boolean
 * @group Hydro Rate Control
 */
PARAM_DEFINE_INT32(HY_ARSP_SCALE_EN, 1);

/**
* Roll trim increment at minimum airspeed
*
* This increment is added to TRIM_ROLL when airspeed is HY_AIRSPD_MIN.
 *
 * @group Hydro Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(HY_DTRIM_R_VMIN, 0.0f);

/**
* Pitch trim increment at minimum airspeed
*
* This increment is added to TRIM_PITCH when airspeed is HY_AIRSPD_MIN.
 *
 * @group Hydro Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(HY_DTRIM_P_VMIN, 0.0f);

/**
* Yaw trim increment at minimum airspeed
*
* This increment is added to TRIM_YAW when airspeed is HY_AIRSPD_MIN.
 *
 * @group Hydro Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(HY_DTRIM_Y_VMIN, 0.0f);

/**
* Roll trim increment at maximum airspeed
*
* This increment is added to TRIM_ROLL when airspeed is HY_AIRSPD_MAX.
 *
 * @group Hydro Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(HY_DTRIM_R_VMAX, 0.0f);

/**
* Pitch trim increment at maximum airspeed
*
* This increment is added to TRIM_PITCH when airspeed is HY_AIRSPD_MAX.
 *
 * @group Hydro Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(HY_DTRIM_P_VMAX, 0.0f);

/**
* Yaw trim increment at maximum airspeed
*
* This increment is added to TRIM_YAW when airspeed is HY_AIRSPD_MAX.
 *
 * @group Hydro Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(HY_DTRIM_Y_VMAX, 0.0f);

/**
 * Manual roll scale
 *
 * Scale factor applied to the desired roll actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_MAN_R_SC, 1.0f);

/**
 * Manual pitch scale
 *
 * Scale factor applied to the desired pitch actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_MAN_P_SC, 1.0f);

/**
 * Manual yaw scale
 *
 * Scale factor applied to the desired yaw actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_MAN_Y_SC, 1.0f);

/**
 * Roll control to yaw control feedforward gain.
 *
 * This gain can be used to counteract the "adverse yaw" effect for fixed wings.
 * When the plane enters a roll it will tend to yaw the nose out of the turn.
 * This gain enables the use of a yaw actuator to counteract this effect.
 *
 * @min 0.0
 * @decimal 1
 * @increment 0.01
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_RLL_TO_YAW_FF, 0.0f);

/**
 * Enable yaw rate controller in Acro
 *
 * If this parameter is set to 1, the yaw rate controller is enabled in Fixed-wing Acro mode.
 * Otherwise the pilot commands directly the yaw actuator.
 * It is disabled by default because an active yaw rate controller will fight against the
 * natural turn coordination of the plane.
 *
 * @boolean
 * @group Hydro Rate Control
 */
PARAM_DEFINE_INT32(HY_ACRO_YAW_EN, 0);

/**
 * Thrust to pit control feedforward gain.
 *
 * Thrust to pit control feedforward gain.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_THR_TO_PIT_FF, 0.0f);


/**
 * Minimum Airspeed (CAS)
 *
 * The minimal airspeed (calibrated airspeed) the user is able to command.
 * Further, if the airspeed falls below this value, the TECS controller will try to
 * increase airspeed more aggressively.
 * Has to be set according to the vehicle's stall speed (which should be set in HYDRO_AIRSPD_STALL),
 * with some margin between the stall speed and minimum airspeed.
 * This value corresponds to the desired minimum speed with the default load factor (level flight, default weight),
 * and is automatically adpated to the current load factor (calculated from roll setpoint and WEIGHT_GROSS/WEIGHT_BASE).
 *
 * @unit m/s
 * @min 0.5
 * @decimal 1
 * @increment 0.5
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_AIRSPD_MIN, 10.0f); //最小空速

/**
 * Maximum manually added yaw rate
 *
 * This is the maximally added yaw rate setpoint from the yaw stick in any attitude controlled flight mode.
 * It is added to the yaw rate setpoint generated by the controller for turn coordination.
 *
 * @unit deg/s
 * @min 0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_MAN_YR_MAX, 30.f);
