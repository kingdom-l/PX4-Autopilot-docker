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
 * @file hydro_control_allocator_params.c
 *
 * Parameters defined by the hydro control allocator task
 *
 */

/**
 * Hydro Airspeed Trim
 *
 * This defines airspeed trim of hydro in water
 *
 * @unit m/s
 * @min 0.2
 * @max 10.0
 * @decimal 2
 * @increment 0.5
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_AIRAPEED_TRIM, 4.0);

/**
 * Hydro Angle of Attack Trim
 *
 * This defines AoA trim of hydro in water
 *
 * @unit m/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.5
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_ALPHA_TRIM, 0.0);

/**
 * Hydro Airspeed Source Selection
 *
 * Set this to 0 if there is no airspeed measurement.
 *
 * @boolean
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_INT32(HY_SPEED_SELECT, 0);

/**
 * Maximum thrust of a single underwater thruster
 *
 *
 * @unit N
 * @min 0
 * @max 15
 * @decimal 2
 * @increment 0.1
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_THRUST_MAX, 10);

/**
 * Maximum thrust Gain of a single underwater thruster
 *
 *
 * @unit
 * @min 0
 * @max 10
 * @decimal 2
 * @increment 0.1
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_TH_MAX_GAIN, 1.5);

/**
 * Right hydrofoil lift coefficient
 *
 *
 * @unit
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.0001
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_RHF_CL, 0.14);

/**
 * Right hydrofoil zero-lift coefficient
 *
 *
 * @unit
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.0001
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_RHF_CL0, 0);

/**
 * Right hydrofoil drag coefficient
 *
 *
 * @unit
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.0001
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_RHF_CD, 0.0034);

/**
 * Right hydrofoil zero-lift drag coefficient
 *
 *
 * @unit
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.0001
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_RHF_CD0, 0.0048);

/**
 * Wing area of  Right hydrofoil
 *
 *
 * @unit
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.0001
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_RHF_AREA, 0.015);

/**
 * Horizontal tail lift coefficient
 *
 *
 * @unit
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.0001
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_HTAIL_CL, 0.09);

/**
 * Horizontal tail zero-lift coefficient
 *
 *
 * @unit
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.0001
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_HTAIL_CL0, 0);

/**
 * Horizontal tail drag coefficient
 *
 *
 * @unit
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.0001
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_HTAIL_CD, 0.0048);

/**
 * Horizontal tail zero-lift drag coefficient
 *
 *
 * @unit
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.0001
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_HTAIL_CD0, 0.0048);

/**
 * Wing area of Horizontal tail
 *
 *
 * @unit
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.0001
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_HTAIL_AREA, 0.02);

/**
 * Deflection angle of the control surface
 *
 *
 * @unit rad
 * @min -0.7
 * @max 0.7
 * @decimal 2
 * @increment 0.1
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_WING_ANG_MAX, 0.7);

/**
 * Index of Hydro Right Motor in Actuators Setup
 *
 * hy_motro_l motor1      hy_motor_r motor2
 *
 *
 * @boolean
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_INT32(HY_LMOTOR_IDX, 1);

/**
 * Index of Hydro Left Motor in Actuators Setup
 *
 *
 *
 * @boolean
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_INT32(HY_RMOTOR_IDX, 2);

/**
 * Index of Hydro Horizontal Tail Servo in Actuators Setup
 *
 * servo_r servo1
 * servo_l servo2
 * htail_servo servo3
 * hy_servo_r servo4
 * hy_servo_l servo5
 *
 * @boolean
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_INT32(HY_HTAIL_SV_IDX, 3);

/**
 * Index of Hydrofoil Right Servo in Actuators Setup
 *
 *
 *
 * @boolean
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_INT32(HY_R_SV_IDX, 4);

/**
 * Index of Hydrofoil Left Servo in Actuators Setup
 *
 *
 *
 * @boolean
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_INT32(HY_L_SV_IDX, 5);

/**
 * Structure info
 *
 *
 * @unit m
 * @min 0.
 * @max 1.
 * @decimal 2
 * @increment 0.1
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_ST_INFO_X2, 0.03);

/**
 * Structure info
 *
 *
 * @unit m
 * @min 0.
 * @max 1.
 * @decimal 2
 * @increment 0.1
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_ST_INFO_Y2, 0.);

/**
 * Structure info
 *
 *
 * @unit m
 * @min 0.
 * @max 1.
 * @decimal 2
 * @increment 0.1
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_ST_INFO_Z2, 0.1);

/**
 * Structure info
 *
 * hydrofoil tail moment weight
 *
 * Roll moment arm
 *
 * @unit m
 * @min 0.
 * @max 1.
 * @decimal 2
 * @increment 0.1
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_ST_INFO_YT, 0.3);

/**
 * Structure info
 *
 * Yaw moment arm
 *
 * @unit m
 * @min 0.
 * @max 1.
 * @decimal 2
 * @increment 0.1
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_ST_INFO_YY, 0.1);

/**
 * Structure info
 *
 *
 * @unit
 * @min 0.
 * @max 5.
 * @decimal 2
 * @increment 0.1
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_ST_INFO_HT, 0.8);

/**
 * Pitch moment weight for fxr
 *
 *
 * @unit
 * @min 0.
 * @max 50.
 * @decimal 2
 * @increment 0.1
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_FXR_WEIGHT, 0);

/**
 * Pitch moment weight for fzr
 *
 *
 * @unit
 * @min -50.
 * @max 0.
 * @decimal 2
 * @increment 0.1
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_FZR_WEIGHT, -10);

/**
 * Pitch moment weight for fxl
 *
 *
 * @unit
 * @min 0.
 * @max 50.
 * @decimal 2
 * @increment 0.1
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_FXL_WEIGHT, 0);

/**
 * Pitch moment weight for fzl
 *
 *
 * @unit
 * @min -50.
 * @max 0.
 * @decimal 2
 * @increment 0.1
 * @group Hydro Control Allocator
 */
PARAM_DEFINE_FLOAT(HY_FZL_WEIGHT, -10);
