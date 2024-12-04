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
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file hydro_pos_control_params.c
 *
 * Parameters defined by the hydro position control task
 *
 */

/**
 * Hydro high proportional gain.
 *
 * @unit m/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_HIGH_P, 0.08f);

/**
 * Hydro high integrator gain.
 *
 * @unit m
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_HIGH_I, 0.0f);

/**
 * Hydro high derivative gain.
 *
 *
 * @unit m/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_HIGH_D, 0.0f);

/**
 * Maximum pitch angle for hydro depth control output
 *
 * The maximum pitch angle setpoint setpoint for a height-rate or altitude controlled mode.
 *
 * @unit deg
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @increment 0.5
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_P_LIM, 30.0f);

/**
 * Maximum roll angle for manual control setpoint
 *
 * The maximum roll angle setpoint for setpoint for a height-rate or altitude controlled mode.
 *
 * @unit deg
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @increment 0.5
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_R_LIM, 30.0f);

/**
 * Hydro depth saturate max value
 *
 * The depth saturate max value for saturate_function.
 *
 * @unit
 * @min 0.0
 * @max 15.0
 * @decimal 1
 * @increment 1
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DEPSAT_MAX, 5.f);

/**
 * Hydro depth saturate slope
 *
 * Th edepth saturate slope for saturate_function.
 *
 * @unit
 * @min 0.0
 * @max 3.0
 * @decimal 1
 * @increment 0.5
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DEPSAT_K, 1.f);

/**
 * Hydro depth setpoint
 *
 * The depth setpoint for an altitude controlled mode.
 *
 * @unit m
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @increment 0.1
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DEPTH_SP, 0.f);

/**
 * Hydro position tracking differentiator params h
 *
 *
 * @unit
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_POS_TD_H, 0.01f);

/**
 * Hydro position tracking differentiator params r0
 *
 *
 * @unit
 * @min 30.0
 * @max 600.0
 * @decimal 1
 * @increment 0.1
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_POS_TD_R0, 100.f);

/**
 * Hydro position tracking differentiator params h0
 *
 *
 * @unit
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_POS_TD_H0, 0.07f);
