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
 * @max 100
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
 * @max 100
 * @decimal 3
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_VA_I, 0.0f);

/**
 * Hydro mocap position jump threshold
 *
 * The x, y and z axes are checked and accepted independently. An axis is
 * rejected when it changes by more than this distance from that axis' last
 * accepted position. No y/z offset and no velocity-dependent allowance are
 * applied. Set to 0 to disable finite-position jump rejection.
 *
 * @unit m
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_DBG_JUMP, 0.2f);

/**
 * Position-derivative sliding-window length
 *
 * Number of accepted samples used by TimeDerivativeCalc. Odd values are
 * rounded up to the next even value because the algorithm uses two
 * half-windows.
 *
 * @min 4
 * @max 100
 * @group Hydro Position Control
 */
PARAM_DEFINE_INT32(HY_VEL_WIN, 10);

/**
 * Consecutive consistent samples required for jump reacquisition
 *
 * After an axis rejects a step, this many mutually consistent finite samples
 * are required before that axis accepts the new position. Reacquisition resets
 * only that axis' derivative estimator.
 *
 * @min 2
 * @max 20
 * @group Hydro Position Control
 */
PARAM_DEFINE_INT32(HY_JMP_REJ_N, 3);

/**
 * Jump-filter reacquisition timeout
 *
 * After this interval without an accepted sample, two consecutive consistent
 * finite samples may reinitialize that axis. The corresponding derivative
 * estimator is reset and must warm up again.
 *
 * @unit s
 * @min 0.05
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_JMP_REAC_T, 0.30f);

/**
 * Position-feedback validity timeout
 *
 * A rejected or missing axis sample may use its last accepted position only
 * for this interval. SACT adaptation and ESO/HRP are paused while derivatives
 * re-warm; the base controller remains active. After this timeout stale
 * feedback is removed from control.
 *
 * @unit s
 * @min 0.10
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_POS_TIMEOUT, 0.50f);

/**
 * Stale-feedback fallback ramp time
 *
 * When feedback times out while armed, forward output moves smoothly to zero
 * and vertical output moves smoothly to feedforward-only control. Disarming
 * and emergency throttle cut still clear outputs immediately.
 *
 * @unit s
 * @min 0.05
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Position Control
 */
PARAM_DEFINE_FLOAT(HY_FB_RAMP, 0.50f);

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
 * Hydro depth eso params b0 inv
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
 * Hydro vel eso params b0 inv
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
 * Depth and velocity controller
 *
 * The values 0 and 1 preserve the behavior of existing parameter files.
 *
 * @value 0 ADRC
 * @value 1 PID
 * @value 2 eADRC-HRP
 * @value 3 SACT+
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

/**
 * HRP depth proportional gain
 *
 * This is the single startup gain; there is no separate run gain.
 *
 * @min 0
 * @max 600
 * @decimal 2
 * @increment 1
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_D_KP, 18.f);

/**
 * HRP depth derivative gain
 *
 * @min 0
 * @max 600
 * @decimal 2
 * @increment 1
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_D_KD, 6.f);

/**
 * HEP depth feedforward gain.
 *
 *
 * @unit
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.05
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_DEP_FF, 3.f);

/**
 * HRP depth ESO params b0 inv
 *
 * b0 = 1/HY_HR_D_B0_INV
 *
 * @unit
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.1
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_D_B0_INV, 2.3f);

/**
 * HRP depth ESO bandwidth
 *
 * @unit rad/s
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 0.5
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_D_WO, 0.f);

/**
 * HRP velocity proportional gain
 *
 * This is the single startup gain; there is no separate run gain.
 *
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.1
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_V_KP, 4.f);

/**
 * HRP velocity feedforward gain.
 *
 *
 * @unit
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.05
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_VA_FF, 0.f);

/**
 * HRP velocity ESO params b0 inv
 *
 * b0 = 1/HY_HR_V_B0_INV
 *
 * @unit
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.1
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_V_B0_INV, 2.3f);

/**
 * HRP velocity ESO bandwidth
 *
 * @unit rad/s
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 0.5
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_V_WO, 3.f);

/**
 * HRP depth prediction gain
 *
 * @min 0
 * @max 10
 * @decimal 2
 * @increment 0.1
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_D_ALP, 0.f);

/**
 * HRP velocity prediction gain
 *
 * @min 0
 * @max 10
 * @decimal 2
 * @increment 0.1
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_V_ALP, 0.f);



/**
 * HRP forgetting factor
 *
 * @min 0.5
 * @max 1
 * @decimal 3
 * @increment 0.005
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_FORGET, 0.99f);

/**
 * HRP ridge regularization
 *
 * @min 0.000001
 * @max 10
 * @decimal 6
 * @increment 0.001
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_RIDGE, 0.01f);

/**
 * HRP residual filter coefficient
 *
 * Equivalent coefficient at a 100 Hz update rate. The implementation makes
 * the filter time-constant consistent when dt varies.
 *
 * @min 0
 * @max 0.999
 * @decimal 3
 * @increment 0.01
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_RLPF, 0.95f);

/**
 * HRP depth residual prediction limit
 *
 * @unit m/s^2
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.1
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_D_RLIM, 8.f);

/**
 * HRP velocity residual prediction limit
 *
 * @unit m/s^2
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.1
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_V_RLIM, 4.f);

/**
 * HRP depth residual noise standard deviation
 *
 * Used only by the causal prediction-confidence calculation.
 *
 * @unit m/s^2
 * @min 0
 * @max 10
 * @decimal 3
 * @increment 0.01
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_D_SIG, 0.05f);

/**
 * HRP velocity residual noise standard deviation
 *
 * Used only by the causal prediction-confidence calculation.
 *
 * @unit m/s^2
 * @min 0
 * @max 10
 * @decimal 3
 * @increment 0.01
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_V_SIG, 0.02f);

/**
 * HRP confidence memory time constant
 *
 * The causal reliability forgetting factor is exp(-dt/HY_HR_C_TC).
 *
 * @unit s
 * @min 0.01
 * @max 10
 * @decimal 2
 * @increment 0.05
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_C_TC, 0.40f);

/**
 * HRP observer compensation ramp time
 *
 * @unit s
 * @min 0
 * @max 5
 * @decimal 2
 * @increment 0.05
 * @group Hydro eADRC-HRP
 */
PARAM_DEFINE_FLOAT(HY_HR_RAMP, 0.15f);

/**
 * SACT+ depth proportional scale
 *
 * Single startup value; no separate run value is used.
 *
 * @min 0
 * @max 600
 * @decimal 2
 * @increment 1
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_D_SP, 8.f);

/**
 * SACT+ depth derivative scale
 *
 * @min 0
 * @max 600
 * @decimal 2
 * @increment 1
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_D_SD, 3.f);

/**
 * SACT+ depth proportional boundary
 *
 * @unit m
 * @min 0.0001
 * @max 10
 * @decimal 4
 * @increment 0.005
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_D_DP, 0.065f);

/**
 * SACT+ depth derivative boundary
 *
 * @unit m/s
 * @min 0.0001
 * @max 10
 * @decimal 4
 * @increment 0.01
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_D_DD, 0.5f);

/**
 * SACT+ depth feedforward gain.
 *
 *
 * @unit
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.05
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_DEP_FF, 1.f);

/**
 * SACT+ depth b0 inv
 *
 * @unit
 * @min 0.0001
 * @max 10
 * @decimal 4
 * @increment 0.01
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_D_B0_INV, 2.3f);

/**
 * SACT+ depth proportional exponent
 *
 * @min 0.05
 * @max 1.5
 * @decimal 3
 * @increment 0.05
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_D_DLP, 0.7f);

/**
 * SACT+ depth derivative exponent
 *
 * @min 0.05
 * @max 1.5
 * @decimal 3
 * @increment 0.05
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_D_DLD, 0.9f);

/**
 * SACT+ depth alpha1
 *
 * @min 0
 * @max 10
 * @decimal 3
 * @increment 0.01
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_D_A1, 0.0f);

/**
 * SACT+ depth alpha2
 *
 * @min 0
 * @max 10
 * @decimal 3
 * @increment 0.01
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_D_A2, 0.0f);

/**
 * SACT+ depth adaptation gain
 *
 * @min 0
 * @max 100
 * @decimal 3
 * @increment 0.05
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_D_GAM, 0.f);

/**
 * SACT+ depth nominal disturbance gain
 *
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.5
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_D_DNM, 16.f);

/**
 * SACT+ depth theta limit
 *
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.5
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_D_TLM, 1.f);

/**
 * SACT+ depth Lambda limit
 *
 * Limits adaptive-state windup during actuator saturation.
 *
 * @min 0
 * @max 1000
 * @decimal 2
 * @increment 0.5
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_D_LLM, 1.f);

/**
 * SACT+ velocity proportional scale
 *
 * Single startup value; no separate run value is used.
 *
 * @min 0
 * @max 600
 * @decimal 2
 * @increment 1
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_V_SP, 4.f);

/**
 * SACT+ velocity derivative scale
 *
 * @min 0
 * @max 600
 * @decimal 2
 * @increment 0.5
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_V_SD, 0.7f);

/**
 * SACT+ velocity proportional boundary
 *
 * @unit m/s
 * @min 0.0001
 * @max 10
 * @decimal 4
 * @increment 0.01
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_V_DP, 1.f);

/**
 * SACT+ velocity derivative boundary
 *
 * @unit m/s^2
 * @min 0.0001
 * @max 10
 * @decimal 4
 * @increment 0.01
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_V_DD, 2.f);

/**
 * SACT+ velocity feedforward gain.
 *
 *
 * @unit
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.05
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_VA_FF, 2.f);

/**
 * SACT+ velocity b0 inv
 *
 * @unit
 * @min 0.0001
 * @max 10
 * @decimal 4
 * @increment 0.01
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_V_B0_INV, 2.3f);

/**
 * SACT+ velocity proportional exponent
 *
 * @min 0.05
 * @max 1.5
 * @decimal 3
 * @increment 0.05
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_V_DLP, 0.8f);

/**
 * SACT+ velocity derivative exponent
 *
 * @min 0.05
 * @max 1.5
 * @decimal 3
 * @increment 0.05
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_V_DLD, 0.8f);

/**
 * SACT+ velocity alpha1
 *
 * @min 0
 * @max 10
 * @decimal 3
 * @increment 0.01
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_V_A1, 0.0f);

/**
 * SACT+ velocity alpha2
 *
 * @min 0
 * @max 10
 * @decimal 3
 * @increment 0.01
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_V_A2, 0.0f);

/**
 * SACT+ velocity adaptation gain
 *
 * @min 0
 * @max 100
 * @decimal 3
 * @increment 0.05
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_V_GAM, 0.0f);

/**
 * SACT+ velocity nominal disturbance gain
 *
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.5
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_V_DNM, 3.f);

/**
 * SACT+ velocity theta limit
 *
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.5
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_V_TLM, 10.f);

/**
 * SACT+ velocity Lambda limit
 *
 * Limits adaptive-state windup during actuator saturation.
 *
 * @min 0
 * @max 1000
 * @decimal 2
 * @increment 0.5
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_V_LLM, 10.f);

/**
 * SACT+ derivative filter time constant
 *
 * A value of 0.027 s is approximately the MATLAB coefficient 0.73 at 100 Hz.
 *
 * @unit s
 * @min 0
 * @max 2
 * @decimal 3
 * @increment 0.005
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_DER_TC, 0.05f);

/**
 * SACT+ compensation ramp time
 *
 * @unit s
 * @min 0
 * @max 10
 * @decimal 2
 * @increment 0.05
 * @group Hydro SACT+
 */
PARAM_DEFINE_FLOAT(HY_SA_RAMP, 1.4f);
