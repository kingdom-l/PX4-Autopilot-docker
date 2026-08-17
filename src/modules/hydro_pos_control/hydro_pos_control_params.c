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
 * @unit
 * @min 0.0
 * @max 600
 * @decimal 3
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DEP_P, 0.08f);

/**
 * Hydro high integrator gain.
 *
 * @unit
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DEP_I, 0.0f);

/**
 * Hydro high feedforward gain.
 *
 *
 * @unit
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DEP_FF, 0.0f);

/**
 * Max z-axis force for the depth control output.
 *
 *
 * @unit
 * @min 0
 * @max 20
 * @decimal 2
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DEP_LIM, 16.0f);

/**
 * Maximum pitch angle for manual control setpoint
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
PARAM_DEFINE_FLOAT(HY_D_PMAX, 30.0f);

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
PARAM_DEFINE_FLOAT(HY_D_RMAX, 30.0f);

/**
 * Hydro depth saturate max value
 *
 * The depth saturate max value for saturate_function and z-axis force.
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
 * @min -3.0
 * @max 3.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DEPTH_SP, 0.f);

/**
 * Hydro velocity feedback for depth control
 *
 *
 * @unit m
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_VELFB_P, 0.f);

/**
 * Hydro vel proportional gain.
 *
 * @unit
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_VA_P, 0.08f);

/**
 * Hydro vel integrator gain.
 *
 * @unit
 * @min 0.0
 * @max 1
 * @decimal 3
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_VA_I, 0.0f);

/**
 * Hydro vel forward feedback gain.
 *
 * @unit
 * @min 0.0
 * @max 1
 * @decimal 3
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_VA_FF, 0.153f);

/**
 * Hydro Va setpoint
 *
 * The Va setpoint for an velocity controlled mode.
 *
 * @unit m
 * @min 0.0
 * @max 3.0
 * @decimal 1
 * @increment 0.1
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_VA_SP, 1.5f);

/**
 * Hydro vel controller resolution
 *
 *
 * @unit
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_VE_RES, 0.05f);

/**
 * Hydro vel controller output 1/slope
 *
 *
 * @unit
 * @min 0.0
 * @max 5000.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_VFX_SP_SLOPE, 300.0f);

/**
 * Hydro vel error a
 *
 *
 * @unit m/s
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_VE_A, 0.05f);

/**
 * Hydro vel error b
 *
 *
 * @unit m/s
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_VE_B, 1.1f);

/**
 * Hydro vel integretor output limit
 *
 * @unit m/s
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_VE_ILIMIT, 0.3f);

/**
 * Hydro depth error a
 *
 *
 * @unit m
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DE_A, 0.02f);

/**
 * Hydro depth error b
 *
 *
 * @unit m
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DE_B, 0.02f);

/**
 * Hydro depth integretor output limit
 *
 * @unit m/s
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DE_ILIMIT, 0.3f);

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
PARAM_DEFINE_FLOAT(HY_POS_TD_R0, 70.f);

/**
 * Hydro position tracking differentiator params h0
 *
 *
 * @unit
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_POS_TD_H0, 0.01f);

/**
 * Hydro depth eso params beta1
 *
 *
 * @unit
 * @min 10.0
 * @max 600.0
 * @decimal 1
 * @increment 1
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_D_ESO_BETA1, 100.f);

/**
 * Hydro depth eso params beta2
 *
 *
 * @unit
 * @min 10.0
 * @max 1000
 * @decimal 1
 * @increment 1
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_D_ESO_BETA2, 280.f);

/**
 * Hydro depth eso params beta3
 *
 *
 * @unit
 * @min 10.0
 * @max 10000
 * @decimal 1
 * @increment 1
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_D_ESO_BETA3, 900.f);

/**
 * Hydro depth eso params b0 backwards
 *
 * b0 = 1/HY_D_ESO_B0_INV
 *
 * @unit
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.1
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_D_ESO_B0_INV, 2.3f);

/**
 * Hydro depth eso params h
 *
 *
 * @unit
 * @min 0.0
 * @max 1
 * @decimal 3
 * @increment 0.001
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_D_ESO_H, 0.008f);

/**
 * Hydro high adrc proportional gain.
 *
 * @unit
 * @min 0.0
 * @max 600
 * @decimal 3
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DEP_ADRC_P, 0.08f);

/**
 * Hydro high adrc derivative gain.
 *
 * @unit
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DEP_ADRC_D, 0.0f);

/**
 * Hydro high adrc feedforward gain.
 *
 *
 * @unit
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DEP_FF_ADRC, 1.f);

/**
 * Hydro high eso total disturbance comp factor.
 *
 *
 * @unit
 * @min 0.0
 * @max 1
 * @decimal 3
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DEP_KCOMP_ESO, 1.f);

/**
 * Max z-axis force for the depth adrc control output.
 *
 *
 * @unit
 * @min 0
 * @max 15
 * @decimal 2
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DEP_LIM_ADRC, 10.0f);

/**
 * Hydro vel eso params beta1
 *
 *
 * @unit
 * @min 10.0
 * @max 600.0
 * @decimal 1
 * @increment 1
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_V_ESO_BETA1, 100.f);

/**
 * Hydro vel eso params beta2
 *
 *
 * @unit
 * @min 10.0
 * @max 1000
 * @decimal 1
 * @increment 1
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_V_ESO_BETA2, 300.f);

/**
 * Hydro vel eso params b0 backwards
 *
 * b0 = 1/HY_V_ESO_B0_INV
 *
 * @unit
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.1
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_V_ESO_B0_INV, 2.3f);

/**
 * Hydro vel eso params h
 *
 *
 * @unit
 * @min 0.0
 * @max 1
 * @decimal 3
 * @increment 0.001
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_V_ESO_H, 0.008f);

/**
 * Hydro vel adrc proportional gain.
 *
 * @unit
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_VA_ADRC_P, 0.08f);

/**
 * Hydro vel adrc feedforward gain.
 *
 *
 * @unit
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_VA_FF_ADRC, 2.f);

/**
 * Max x-axis force for the vel adrc control output.
 *
 *
 * @unit
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_VA_ADRC_LIM, 1.0f);

/**
 * Hydro vel adrc controller resolution
 *
 *
 * @unit
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_VE_RES_ADRC, 0.05f);

/**
 * Hydro vel adrc controller output 1/slope
 *
 *
 * @unit
 * @min 0.0
 * @max 5000.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_VFX_SLPADRC, 300.0f);

/**
 * Enable ADRC for Depth and Vel
 *
 * Enable when using PID for Depth and Vel
 * Disable when using ADRC for Depth and Vel
 *
 * @boolean
 * @group Hydro Position Control
 */
PARAM_DEFINE_INT32(HY_DEPVA_PID_EN, 1);

/**
 * Hydro depth lowpass filter params
 *
 * cutoff_freq < sample_freq / 2
 *
 * @unit
 * @min 10.0
 * @max 1000
 * @decimal 1
 * @increment 1
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DEP_SAMFREQ, 800.f);

/**
 * Hydro depth lowpass filter params
 *
 * cutoff_freq < sample_freq / 2
 *
 * @unit
 * @min 10.0
 * @max 1000
 * @decimal 1
 * @increment 1
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DEP_CUTFREQ, 40.f);
