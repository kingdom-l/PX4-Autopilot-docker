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
 * @file hydro_att_control_params.c
 *
 * Parameters defined by the fixed-wing attitude control task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/**
 * Attitude Roll Time Constant
 *
 * This defines the latency between a roll step input and the achieved setpoint
 * (inverse to a P gain). Smaller systems may require smaller values.
 *
 * @unit s
 * @min 0.2
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_R_TC, 0.4f);

/**
 * Attitude pitch time constant
 *
 * This defines the latency between a pitch step input and the achieved setpoint
 * (inverse to a P gain). Smaller systems may require smaller values.
 *
 * @unit s
 * @min 0.2
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_P_TC, 0.2f);

/**
 * Maximum positive / up pitch rate setpoint
 *
 * @unit deg/s
 * @min 0.0
 * @max 180
 * @decimal 1
 * @increment 0.5
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_P_RMAX_POS, 120.0f);

/**
 * Maximum negative / down pitch rate setpoint
 *
 * @unit deg/s
 * @min 0.0
 * @max 180
 * @decimal 1
 * @increment 0.5
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_P_RMAX_NEG, 120.0f);

/**
 * Maximum roll rate setpoint
 *
 * @unit deg/s
 * @min 0.0
 * @max 180
 * @decimal 1
 * @increment 0.5
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_R_RMAX, 70.0f);

/**
 * Maximum yaw rate setpoint
 *
 * @unit deg/s
 * @min 0.0
 * @max 180
 * @decimal 1
 * @increment 0.5
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_Y_RMAX, 50.0f);

/**
 * Maximum Airspeed (CAS)
 *
 * The maximal airspeed (calibrated airspeed) the user is able to command.
 *
 * @unit m/s
 * @min 0.5
 * @decimal 1
 * @increment 0.5
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_AIRSPD_MAX, 20.0f);

/**
 * Stall Airspeed (CAS)
 *
 * The stall airspeed (calibrated airspeed) of the vehicle.
 * It is used for airspeed sensor failure detection and for the control
 * surface scaling airspeed limits.
 *
 * @unit m/s
 * @min 0.5
 * @decimal 1
 * @increment 0.5
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_AIRSPD_STALL, 7.0f);

/**
 * Trim (Cruise) Airspeed
 *
 * The trim CAS (calibrated airspeed) of the vehicle. If an airspeed controller is active,
 * this is the default airspeed setpoint that the controller will try to achieve.
 * This value corresponds to the trim airspeed with the default load factor (level flight, default weight).
 *
 * @unit m/s
 * @min 0.5
 * @decimal 1
 * @increment 0.5
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_AIRSPD_TRIM, 15.0f);

/**
 * Pitch setpoint offset (pitch at level flight)
 *
 * An airframe specific offset of the pitch setpoint in degrees, the value is
 * added to the pitch setpoint and should correspond to the pitch at
 * typical cruise speed of the airframe.
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_PSP_OFF, 0.0f);

/**
 * Maximum manual pitch angle
 *
 * Applies to both directions in all manual modes with attitude stabilization but without altitude control
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_MAN_P_MAX, 30.0f);

/**
 * Maximum manual roll angle
 *
 * Applies to both directions in all manual modes with attitude stabilization
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_MAN_R_MAX, 45.0f);

/**
 * Dive pitch angle down
 *
 * Dive pitch angle down
 *
 * @unit deg
 * @min -80.0
 * @max 0.0
 * @decimal 1
 * @increment 0.5
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_DIVE_DN_DEG, -30.0f);

/**
 * Dive thrust down
 *
 * Dive thrust down
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_DIVE_DN_THR, 0.5f);

/**
 * Dive time down
 *
 * Dive time down
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_DIVE_DN_SEC, 0.2f);

/**
 * Dive pitch angle cruise
 *
 * Dive pitch angle cruise
 *
 * @unit deg
 * @min -40.0
 * @max 40.0
 * @decimal 1
 * @increment 0.5
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_DIVE_CRU_DEG, 0.0f);

/**
 * Dive thrust cruise
 *
 * Dive thrust cruise
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_DIVE_CRU_THR, 0.5f);

/**
 * Dive time cruise
 *
 * Dive time cruise
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_DIVE_CRU_SEC, 0.2f);

/**
 * Dive pitch angle up
 *
 * Dive pitch angle up
 *
 * @unit deg
 * @min 0.0
 * @max 80.0
 * @decimal 1
 * @increment 0.5
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_DIVE_UP_DEG, 30.0f);

/**
 * Dive thrust up
 *
 * Dive thrust up
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_DIVE_UP_THR, 0.5f);

/**
 * Dive time up
 *
 * Dive time up
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_DIVE_UP_SEC, 0.4f);
