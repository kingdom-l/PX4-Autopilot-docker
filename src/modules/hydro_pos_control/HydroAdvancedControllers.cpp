/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "HydroAdvancedControllers.hpp"

#include <cmath>
#include <float.h>

void HrpPredictor::reset()
{
	for (uint8_t row = 0; row < WindowLength; ++row) {
		for (uint8_t column = 0; column < MaxFeatureCount; ++column) {
			_features[row][column] = 0.f;
		}

		_targets[row] = 0.f;
	}

	for (uint8_t i = 0; i < MaxFeatureCount; ++i) {
		_theta[i] = 0.f;
	}

	_confidence = 0.f;
	_prediction_error_power = 0.f;
	_residual_mean = 0.f;
	_residual_power = 0.f;
	_last_prediction = 0.f;
	_head = 0;
	_count = 0;
	_model_valid = false;
	_prediction_pending = false;
	_statistics_ready = false;
}

void HrpPredictor::configure(const Config &config)
{
	_config.feature_count = math::constrain(config.feature_count, static_cast<uint8_t>(1), MaxFeatureCount);
	_config.forgetting_factor = math::constrain(config.forgetting_factor, 0.5f, 1.f);
	_config.ridge = math::max(config.ridge, 1e-7f);
	_config.prediction_limit = math::max(config.prediction_limit, 0.f);
	_config.noise_sigma = math::max(config.noise_sigma, 0.f);
}

uint8_t HrpPredictor::chronologicalIndex(uint8_t sample) const
{
	// fit() is called only for a full window; _head then points to the oldest row.
	return (_head + sample) % WindowLength;
}

void HrpPredictor::addSample(const float feature[MaxFeatureCount], float target, float reliability_forgetting)
{
	if (!PX4_ISFINITE(target)) {
		return;
	}

	// The target available now realizes the one-step-ahead prediction issued
	// on the preceding update. This is the causal reliability calculation used
	// by the current MATLAB implementation; it deliberately does not use the
	// regression window's in-sample fitting error.
	if (_prediction_pending) {
		const float lambda = math::constrain(reliability_forgetting, 0.f, 1.f);
		const float one_minus_lambda = 1.f - lambda;
		const float prediction_error = target - _last_prediction;
		const float residual_mean_old = _residual_mean;
		_prediction_error_power = lambda * _prediction_error_power
					  + one_minus_lambda * prediction_error * prediction_error;
		_residual_mean = lambda * _residual_mean + one_minus_lambda * target;
		const float centered_residual = target - residual_mean_old;
		_residual_power = lambda * _residual_power
				  + one_minus_lambda * centered_residual * centered_residual;
		_statistics_ready = true;
		_prediction_pending = false;

		const float noise_variance = _config.noise_sigma * _config.noise_sigma;
		const float signal_energy = _residual_power + noise_variance;
		const float denominator = signal_energy + _prediction_error_power;
		_confidence = denominator > 1e-12f ? math::constrain(signal_energy / denominator, 0.f, 1.f) : 0.f;

		if (!PX4_ISFINITE(_confidence)) {
			_confidence = 0.f;
			_statistics_ready = false;
		}
	}

	for (uint8_t i = 0; i < _config.feature_count; ++i) {
		if (!PX4_ISFINITE(feature[i])) {
			return;
		}

		_features[_head][i] = feature[i];
	}

	for (uint8_t i = _config.feature_count; i < MaxFeatureCount; ++i) {
		_features[_head][i] = 0.f;
	}

	_targets[_head] = target;
	_head = (_head + 1) % WindowLength;
	_count = math::min(static_cast<uint8_t>(_count + 1), WindowLength);

	if (_count >= WindowLength) {
		fit();
	}
}

float HrpPredictor::predict(const float feature[MaxFeatureCount])
{
	if (!_model_valid) {
		_prediction_pending = false;
		return 0.f;
	}

	float prediction = 0.f;

	for (uint8_t i = 0; i < _config.feature_count; ++i) {
		if (!PX4_ISFINITE(feature[i])) {
			_prediction_pending = false;
			return 0.f;
		}

		prediction += _theta[i] * feature[i];
	}

	if (!PX4_ISFINITE(prediction)) {
		_prediction_pending = false;
		return 0.f;
	}

	_last_prediction = math::constrain(prediction, -_config.prediction_limit, _config.prediction_limit);
	_prediction_pending = true;
	return _last_prediction;
}

void HrpPredictor::fit()
{
	if (_count < WindowLength) {
		_model_valid = false;
		return;
	}

	float normal[MaxFeatureCount][MaxFeatureCount]{};
	float rhs[MaxFeatureCount]{};
	float weight = 1.f;

	// Avoid powf() in the control loop. M is fixed and small, so nine
	// multiplications produce exactly forgetting_factor^(M-1).
	for (uint8_t i = 1; i < WindowLength; ++i) {
		weight *= _config.forgetting_factor;
	}

	for (uint8_t sample = 0; sample < WindowLength; ++sample) {
		const uint8_t index = chronologicalIndex(sample);

		for (uint8_t row = 0; row < _config.feature_count; ++row) {
			rhs[row] += weight * _features[index][row] * _targets[index];

			for (uint8_t column = 0; column < _config.feature_count; ++column) {
				normal[row][column] += weight * _features[index][row] * _features[index][column];
			}
		}

		weight /= _config.forgetting_factor;
	}

	for (uint8_t i = 0; i < _config.feature_count; ++i) {
		normal[i][i] += _config.ridge;
	}

	for (uint8_t i = 0; i < MaxFeatureCount; ++i) {
		_theta[i] = 0.f;
	}

	float condition_proxy = FLT_MAX;

	if (!solveCholesky(normal, rhs, _theta, _config.feature_count, condition_proxy) || condition_proxy > 1e8f) {
		_model_valid = false;
		return;
	}

	_model_valid = true;
}

bool HrpPredictor::solveCholesky(float matrix[MaxFeatureCount][MaxFeatureCount],
				 const float rhs[MaxFeatureCount], float solution[MaxFeatureCount],
				 uint8_t feature_count, float &condition_proxy) const
{
	float lower[MaxFeatureCount][MaxFeatureCount]{};
	float minimum_diagonal = FLT_MAX;
	float maximum_diagonal = 0.f;

	for (uint8_t row = 0; row < feature_count; ++row) {
		for (uint8_t column = 0; column <= row; ++column) {
			float sum = matrix[row][column];

			for (uint8_t k = 0; k < column; ++k) {
				sum -= lower[row][k] * lower[column][k];
			}

			if (row == column) {
				if (!PX4_ISFINITE(sum) || sum <= 1e-10f) {
					return false;
				}

				lower[row][column] = sqrtf(sum);
				minimum_diagonal = math::min(minimum_diagonal, lower[row][column]);
				maximum_diagonal = math::max(maximum_diagonal, lower[row][column]);

			} else {
				lower[row][column] = sum / lower[column][column];
			}
		}
	}

	float intermediate[MaxFeatureCount]{};

	for (uint8_t row = 0; row < feature_count; ++row) {
		float sum = rhs[row];

		for (uint8_t column = 0; column < row; ++column) {
			sum -= lower[row][column] * intermediate[column];
		}

		intermediate[row] = sum / lower[row][row];
	}

	for (int row = static_cast<int>(feature_count) - 1; row >= 0; --row) {
		float sum = intermediate[row];

		for (uint8_t column = static_cast<uint8_t>(row + 1); column < feature_count; ++column) {
			sum -= lower[column][row] * solution[column];
		}

		solution[row] = sum / lower[row][row];

		if (!PX4_ISFINITE(solution[row])) {
			return false;
		}
	}

	const float diagonal_ratio = maximum_diagonal / math::max(minimum_diagonal, 1e-12f);
	condition_proxy = diagonal_ratio * diagonal_ratio;
	return PX4_ISFINITE(condition_proxy);
}

void EadrcHrpController::reset(float depth_error, float depth_error_rate, float velocity_error,
		float depth_disturbance_initial)
{
	_depth_predictor.reset();
	_velocity_predictor.reset();
	_depth_z1 = depth_error;
	_depth_z2 = depth_error_rate;
	_depth_z3 = depth_disturbance_initial;
	_velocity_z1 = velocity_error;
	_velocity_z2 = 0.f;
	_depth_error_previous = depth_error;
	_depth_error_rate_previous = depth_error_rate;
	_velocity_error_previous = velocity_error;
	_depth_rate_state_previous = depth_error_rate;
	_depth_disturbance_previous = depth_disturbance_initial;
	_velocity_disturbance_previous = 0.f;
	_depth_force_previous = 0.f;
	_velocity_force_previous = 0.f;
	_depth_residual = 0.f;
	_velocity_residual = 0.f;
	_depth_residual_1 = 0.f;
	_depth_residual_2 = 0.f;
	_velocity_residual_1 = 0.f;
	_velocity_residual_2 = 0.f;
	_depth_prediction = 0.f;
	_velocity_prediction = 0.f;
	_depth_residual_history_count = 0;
	_velocity_residual_history_count = 0;
	_elapsed = 0.f;
	_initialized = true;
}

void EadrcHrpController::setAppliedForces(float fx_force, float fz_force)
{
	_velocity_force_previous = PX4_ISFINITE(fx_force) ? fx_force : 0.f;
	_depth_force_previous = PX4_ISFINITE(fz_force) ? fz_force : 0.f;
}

float EadrcHrpController::lpfCoefficient(float coefficient_at_100_hz, float dt)
{
	const float reference_coefficient = math::constrain(coefficient_at_100_hz, 0.f, 0.9999f);
	const float time_constant = reference_coefficient * 0.01f / math::max(1.f - reference_coefficient, 1e-4f);
	return time_constant / (time_constant + dt);
}

void EadrcHrpController::updateObservers(float dt, float depth_error, float velocity_error,
					 const EadrcHrpParams &params)
{
	// Limit the Euler step to 10 ms. At lower measurement rates this costs at
	// most five very small observer iterations and prevents bandwidth-induced
	// numerical instability.
	const uint8_t substeps = math::constrain(static_cast<uint8_t>(ceilf(dt / 0.01f)),
				 static_cast<uint8_t>(1), static_cast<uint8_t>(5));
	const float observer_dt = dt / static_cast<float>(substeps);
	const float depth_b0 = 1.f / math::max(params.depth_b0_inverse, 1e-3f);
	const float velocity_b0 = 1.f / math::max(params.velocity_b0_inverse, 1e-3f);
	const float depth_wo = math::max(params.depth_observer_bandwidth, 0.f);
	const float velocity_wo = math::max(params.velocity_observer_bandwidth, 0.f);
	const float depth_beta1 = 3.f * depth_wo;
	const float depth_beta2 = 3.f * depth_wo * depth_wo;
	const float depth_beta3 = depth_wo * depth_wo * depth_wo;
	const float velocity_beta1 = 2.f * velocity_wo;
	const float velocity_beta2 = velocity_wo * velocity_wo;

	for (uint8_t i = 0; i < substeps; ++i) {
		const float depth_innovation = depth_error - _depth_z1;
		const float depth_z1_next = _depth_z1 + observer_dt * (_depth_z2 + depth_beta1 * depth_innovation);
		const float depth_z2_next = _depth_z2 + observer_dt *
					    (_depth_z3 - depth_b0 * _depth_force_previous + depth_beta2 * depth_innovation);
		const float depth_z3_next = _depth_z3 + observer_dt * depth_beta3 * depth_innovation;
		_depth_z1 = depth_z1_next;
		_depth_z2 = depth_z2_next;
		_depth_z3 = depth_z3_next;

		const float velocity_innovation = velocity_error - _velocity_z1;
		const float velocity_z1_next = _velocity_z1 + observer_dt *
					       (_velocity_z2 - velocity_b0 * _velocity_force_previous
						+ velocity_beta1 * velocity_innovation);
		const float velocity_z2_next = _velocity_z2 + observer_dt * velocity_beta2 * velocity_innovation;
		_velocity_z1 = velocity_z1_next;
		_velocity_z2 = velocity_z2_next;
	}
}

EadrcHrpController::Output EadrcHrpController::update(float dt, float depth_error, float depth_error_rate,
		float velocity_error, const EadrcHrpParams &params)
{
	dt = math::constrain(dt, 1e-3f, 0.05f);

	if (!_initialized) {
		reset(depth_error, depth_error_rate, velocity_error, params.depth_disturbance_initial);
	}

	_depth_predictor.configure(params.depth_predictor);
	_velocity_predictor.configure(params.velocity_predictor);
	updateObservers(dt, depth_error, velocity_error, params);

	if (!PX4_ISFINITE(_depth_z1) || !PX4_ISFINITE(_depth_z2) || !PX4_ISFINITE(_depth_z3)
	    || !PX4_ISFINITE(_velocity_z1) || !PX4_ISFINITE(_velocity_z2)) {
		reset(depth_error, depth_error_rate, velocity_error, params.depth_disturbance_initial);
	}

	const float depth_b0 = 1.f / math::max(params.depth_b0_inverse, 1e-3f);
	const float velocity_b0 = 1.f / math::max(params.velocity_b0_inverse, 1e-3f);
	const float depth_acceleration = (depth_error_rate - _depth_error_rate_previous) / dt;
	const float velocity_error_rate = (velocity_error - _velocity_error_previous) / dt;
	const float depth_residual_raw = depth_acceleration + depth_b0 * _depth_force_previous
					 - _depth_disturbance_previous;
	const float velocity_residual_raw = velocity_error_rate + velocity_b0 * _velocity_force_previous
					    - _velocity_disturbance_previous;
	const float residual_filter = lpfCoefficient(params.residual_lpf, dt);
	_depth_residual = residual_filter * _depth_residual + (1.f - residual_filter) * depth_residual_raw;
	_velocity_residual = residual_filter * _velocity_residual + (1.f - residual_filter) * velocity_residual_raw;

	// The freshly reconstructed residual belongs to the preceding sample
	// because it uses the preceding applied force and ESO disturbance. Pair it
	// with that sample's error/observer state, then predict the current residual
	// using the two newest residuals. This reproduces the MATLAB j=k-1 ordering.
	const float confidence_time_constant = math::max(params.confidence_time_constant, 1e-3f);
	const float reliability_forgetting = expf(-dt / confidence_time_constant);

	if (_depth_residual_history_count >= 2) {
		const float depth_training_feature[HrpPredictor::MaxFeatureCount] {
			1.f, _depth_residual_1, _depth_residual_2, _depth_error_previous, _depth_rate_state_previous
		};
		_depth_predictor.addSample(depth_training_feature, _depth_residual, reliability_forgetting);
	}

	if (_velocity_residual_history_count >= 2) {
		const float velocity_training_feature[HrpPredictor::MaxFeatureCount] {
			1.f, _velocity_residual_1, _velocity_residual_2, _velocity_error_previous, 0.f
		};
		_velocity_predictor.addSample(velocity_training_feature, _velocity_residual, reliability_forgetting);
	}

	const float depth_prediction_feature[HrpPredictor::MaxFeatureCount] {
		1.f, _depth_residual, _depth_residual_1, depth_error, _depth_z2
	};
	const float velocity_prediction_feature[HrpPredictor::MaxFeatureCount] {
		1.f, _velocity_residual, _velocity_residual_1, velocity_error, 0.f
	};
	_depth_prediction = _depth_predictor.predict(depth_prediction_feature);
	_velocity_prediction = _velocity_predictor.predict(velocity_prediction_feature);

	_elapsed += dt;
	const float ramp = params.compensation_ramp_time > 1e-4f ?
			   math::min(_elapsed / params.compensation_ramp_time, 1.f) : 1.f;
	const float depth_compensation = params.depth_alpha * _depth_predictor.confidence() * _depth_prediction;
	const float velocity_compensation = params.velocity_alpha * _velocity_predictor.confidence() * _velocity_prediction;
	const float depth_base = params.depth_b0_inverse
				 * (params.depth_kp * depth_error + params.depth_kd * depth_error_rate)
				 - params.depth_feedforward;
	const float velocity_base = params.velocity_b0_inverse * params.velocity_kp * velocity_error
				    + params.velocity_feedforward;
	const float depth_robust_compensation = params.depth_b0_inverse * (_depth_z3 + depth_compensation);
	const float velocity_robust_compensation = params.velocity_b0_inverse
					 * (_velocity_z2 + velocity_compensation);

	Output output{};
	// Reuse HY_HR_RAMP as an enable ramp for the complete control command.
	// This removes the arming step while keeping the raw Kp/Kd output available
	// for the disarmed static-parameter diagnostic in HydroPositionControl.
	output.fz_force = depth_base + ramp * (depth_robust_compensation);
	output.fx_force = velocity_base + ramp * (velocity_robust_compensation);
	output.fz_force = math::constrain(output.fz_force, -fabsf(params.depth_force_limit),
			  fabsf(params.depth_force_limit));
	output.fx_force = math::constrain(output.fx_force, 0.f,
			  fabsf(params.velocity_force_limit));

	if (!PX4_ISFINITE(output.fz_force) || !PX4_ISFINITE(output.fx_force)) {
		reset(depth_error, depth_error_rate, velocity_error, params.depth_disturbance_initial);
		return {};
	}

	_depth_residual_2 = _depth_residual_1;
	_depth_residual_1 = _depth_residual;
	_velocity_residual_2 = _velocity_residual_1;
	_velocity_residual_1 = _velocity_residual;
	_depth_residual_history_count = math::min(static_cast<uint8_t>(_depth_residual_history_count + 1),
						 static_cast<uint8_t>(2));
	_velocity_residual_history_count = math::min(static_cast<uint8_t>(_velocity_residual_history_count + 1),
						    static_cast<uint8_t>(2));
	_depth_error_previous = depth_error;
	_depth_error_rate_previous = depth_error_rate;
	_velocity_error_previous = velocity_error;
	_depth_rate_state_previous = _depth_z2;
	_depth_disturbance_previous = _depth_z3;
	_velocity_disturbance_previous = _velocity_z2;
	return output;
}

void SactPlusController::reset(float velocity_error)
{
	resetAdaptiveStates();
	_depth_error_rate_filtered = 0.f;
	_velocity_error_rate_filtered = 0.f;
	_velocity_error_previous = velocity_error;
	_initialized = true;
}

void SactPlusController::resetAdaptiveStates()
{
	_depth_lambda = 0.f;
	_velocity_lambda = 0.f;
	_depth_theta = 0.f;
	_velocity_theta = 0.f;
	_elapsed = 0.f;
}

float SactPlusController::variableSaturation(float error, float scale, float boundary, float exponent)
{
	const float safe_boundary = math::max(fabsf(boundary), 1e-4f);
	const float magnitude = math::max(fabsf(error), safe_boundary);
	const float safe_exponent = math::constrain(exponent, 0.05f, 1.5f);
	return scale * powf(magnitude, safe_exponent - 1.f) * error;
}

SactPlusController::Output SactPlusController::update(float dt, float depth_error, float depth_error_rate,
		float velocity_error, const SactPlusParams &params, bool adaptation_enabled)
{
	dt = math::constrain(dt, 1e-3f, 0.05f);

	if (!_initialized) {
		reset(velocity_error);
	}

	const float velocity_error_rate = (velocity_error - _velocity_error_previous) / dt;
	const float filter_time_constant = math::max(params.derivative_filter_time_constant, 0.f);
	const float filter_coefficient = filter_time_constant / (filter_time_constant + dt);
	_depth_error_rate_filtered = filter_coefficient * _depth_error_rate_filtered
				     + (1.f - filter_coefficient) * depth_error_rate;
	_velocity_error_rate_filtered = filter_coefficient * _velocity_error_rate_filtered
					+ (1.f - filter_coefficient) * velocity_error_rate;

	const float depth_virtual_control = variableSaturation(depth_error, params.depth.proportional_scale,
					    params.depth.proportional_boundary, params.depth.proportional_exponent)
					  + variableSaturation(_depth_error_rate_filtered, params.depth.derivative_scale,
					    params.depth.derivative_boundary, params.depth.derivative_exponent);
	const float velocity_virtual_control = variableSaturation(velocity_error, params.velocity.proportional_scale,
					       params.velocity.proportional_boundary, params.velocity.proportional_exponent)
					     + variableSaturation(_velocity_error_rate_filtered, params.velocity.derivative_scale,
					       params.velocity.derivative_boundary, params.velocity.derivative_exponent);

	float depth_adaptive_compensation = 0.f;
	float velocity_adaptive_compensation = 0.f;

	if (adaptation_enabled) {
		_depth_lambda = math::constrain(_depth_lambda - dt * depth_virtual_control,
				-fabsf(params.depth.lambda_limit), fabsf(params.depth.lambda_limit));
		_velocity_lambda = math::constrain(_velocity_lambda - dt * velocity_virtual_control,
				-fabsf(params.velocity.lambda_limit), fabsf(params.velocity.lambda_limit));
		const float depth_augmented_error = _depth_error_rate_filtered + params.depth.alpha2 * depth_error;
		const float velocity_augmented_error = _velocity_error_rate_filtered + params.velocity.alpha2 * velocity_error;
		_depth_theta = math::constrain(_depth_theta + dt * params.depth.gamma * depth_augmented_error,
				-fabsf(params.depth.theta_limit), fabsf(params.depth.theta_limit));
		_velocity_theta = math::constrain(_velocity_theta + dt * params.velocity.gamma * velocity_augmented_error,
				-fabsf(params.velocity.theta_limit), fabsf(params.velocity.theta_limit));

		_elapsed += dt;
		const float ramp = params.compensation_ramp_time > 1e-4f ?
				   math::min(_elapsed / params.compensation_ramp_time, 1.f) : 1.f;
		depth_adaptive_compensation = ramp
					      * (params.depth.nominal_disturbance * (-params.depth.alpha1 * _depth_lambda)
						 + _depth_theta);
		velocity_adaptive_compensation = ramp
						 * (params.velocity.nominal_disturbance * (-params.velocity.alpha1 * _velocity_lambda)
						    + _velocity_theta);

	} else {
		// Keep the nonlinear PD and fixed feedforward path alive, but make sure
		// Lambda/Theta and the compensation ramp cannot accumulate while disabled.
		resetAdaptiveStates();
	}

	Output output{};
	output.fz_base_raw = params.depth_b0_inverse * depth_virtual_control - params.depth_feedforward;
	output.fx_base_raw = params.velocity_b0_inverse * velocity_virtual_control + params.velocity_feedforward;
	output.fz_force = output.fz_base_raw + depth_adaptive_compensation;
	output.fx_force = output.fx_base_raw + velocity_adaptive_compensation;
	output.fz_force = math::constrain(output.fz_force, -fabsf(params.depth_force_limit),
			  fabsf(params.depth_force_limit));
	output.fx_force = math::constrain(output.fx_force, -fabsf(params.velocity_force_limit),
			  fabsf(params.velocity_force_limit));

	if (!PX4_ISFINITE(output.fz_force) || !PX4_ISFINITE(output.fx_force)) {
		reset(velocity_error);
		return {};
	}

	_velocity_error_previous = velocity_error;
	return output;
}
