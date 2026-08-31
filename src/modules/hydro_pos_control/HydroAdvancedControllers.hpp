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

#pragma once

#include <lib/mathlib/mathlib.h>

#include <cstdint>

/**
 * Fixed-memory historical-residual predictor.
 *
 * The MATLAB reference recomputes a weighted, standardized ridge regression
 * from an M-sample window at every control step. This implementation retains
 * that model but caps M at 40 and refits only every N samples. It uses no heap,
 * Eigen, pseudoinverse, or condition-number decomposition.
 */
class HrpPredictor
{
public:
	static constexpr uint8_t WindowLength = 10;
	static constexpr uint8_t MaxFeatureCount = 5;
	static constexpr uint8_t DepthFeatureCount = 5;
	static constexpr uint8_t VelocityFeatureCount = 4;

	struct Config {
		uint8_t feature_count{DepthFeatureCount};
		float forgetting_factor{0.99f};
		float ridge{1e-2f};
		float prediction_limit{8.f};
		float noise_sigma{0.05f};
	};

	void reset();
	void configure(const Config &config);
	void addSample(const float feature[MaxFeatureCount], float target, float reliability_forgetting);
	float predict(const float feature[MaxFeatureCount]);

	float confidence() const { return _statistics_ready ? _confidence : 0.f; }
	uint8_t sampleCount() const { return _count; }

private:
	void fit();
	bool solveCholesky(float matrix[MaxFeatureCount][MaxFeatureCount], const float rhs[MaxFeatureCount],
			   float solution[MaxFeatureCount], uint8_t feature_count, float &condition_proxy) const;
	uint8_t chronologicalIndex(uint8_t sample) const;

	Config _config{};
	float _features[WindowLength][MaxFeatureCount]{};
	float _targets[WindowLength]{};
	float _theta[MaxFeatureCount]{};
	float _confidence{0.f};
	float _prediction_error_power{0.f};
	float _residual_mean{0.f};
	float _residual_power{0.f};
	float _last_prediction{0.f};
	uint8_t _head{0};
	uint8_t _count{0};
	bool _model_valid{false};
	bool _prediction_pending{false};
	bool _statistics_ready{false};
};

struct EadrcHrpParams {
	float depth_b0_inverse{2.3f};
	float velocity_b0_inverse{2.3f};
	float depth_kp{2.f};
	float depth_kd{5.f};
	float depth_observer_bandwidth{8.f};
	float velocity_kp{1.f};
	float velocity_observer_bandwidth{5.f};
	float depth_alpha{0.f};
	float velocity_alpha{0.f};
	float residual_lpf{0.95f};
	float confidence_time_constant{0.40f};
	float compensation_ramp_time{0.5f};
	float depth_feedforward{0.f};
	float velocity_feedforward{0.f};
	float depth_force_limit{16.f};
	float velocity_force_limit{52.f};
	HrpPredictor::Config depth_predictor{};
	HrpPredictor::Config velocity_predictor{HrpPredictor::VelocityFeatureCount, 0.99f, 1e-2f, 4.f, 0.02f};
};

class EadrcHrpController
{
public:
	struct Output {
		float fx_force{0.f};
		float fz_force{0.f};
	};

	void reset(float depth_error, float depth_error_rate, float velocity_error);
	Output update(float dt, float depth_error, float depth_error_rate, float velocity_error,
		      const EadrcHrpParams &params);
	void setAppliedForces(float fx_force, float fz_force);

	float depthState() const { return _depth_z1; }
	float depthRateState() const { return _depth_z2; }
	float depthDisturbance() const { return _depth_z3; }
	float velocityState() const { return _velocity_z1; }
	float velocityDisturbance() const { return _velocity_z2; }
	float depthPrediction() const { return _depth_prediction; }
	float velocityPrediction() const { return _velocity_prediction; }
	float depthConfidence() const { return _depth_predictor.confidence(); }
	float velocityConfidence() const { return _velocity_predictor.confidence(); }

private:
	static float lpfCoefficient(float coefficient_at_100_hz, float dt);
	void updateObservers(float dt, float depth_error, float velocity_error, const EadrcHrpParams &params);

	HrpPredictor _depth_predictor{};
	HrpPredictor _velocity_predictor{};

	float _depth_z1{0.f};
	float _depth_z2{0.f};
	float _depth_z3{0.f};
	float _velocity_z1{0.f};
	float _velocity_z2{0.f};
	float _depth_error_previous{0.f};
	float _depth_error_rate_previous{0.f};
	float _velocity_error_previous{0.f};
	float _depth_rate_state_previous{0.f};
	float _depth_disturbance_previous{0.f};
	float _velocity_disturbance_previous{0.f};
	float _depth_force_previous{0.f};
	float _velocity_force_previous{0.f};
	float _depth_residual{0.f};
	float _velocity_residual{0.f};
	float _depth_residual_1{0.f};
	float _depth_residual_2{0.f};
	float _velocity_residual_1{0.f};
	float _velocity_residual_2{0.f};
	float _depth_prediction{0.f};
	float _velocity_prediction{0.f};
	uint8_t _depth_residual_history_count{0};
	uint8_t _velocity_residual_history_count{0};
	float _elapsed{0.f};
	bool _initialized{false};
};

struct SactChannelParams {
	float proportional_scale{1.f};
	float derivative_scale{1.f};
	float proportional_boundary{1.f};
	float derivative_boundary{1.f};
	float proportional_exponent{0.5f};
	float derivative_exponent{0.5f};
	float alpha1{0.1f};
	float alpha2{0.1f};
	float gamma{1.f};
	float nominal_disturbance{1.f};
	float theta_limit{1.f};
	float lambda_limit{20.f};
};

struct SactPlusParams {
	float depth_b0_inverse{2.3f};
	float velocity_b0_inverse{2.3f};
	float derivative_filter_time_constant{0.027f};
	float compensation_ramp_time{1.4f};
	float depth_feedforward{0.f};
	float velocity_feedforward{0.f};
	float depth_force_limit{16.f};
	float velocity_force_limit{52.f};
	SactChannelParams depth{};
	SactChannelParams velocity{};
};

class SactPlusController
{
public:
	struct Output {
		float fx_force{0.f};
		float fz_force{0.f};
		float fx_base_raw{0.f};
		float fz_base_raw{0.f};
	};

	void reset(float velocity_error);
	void resetAdaptiveStates();
	Output update(float dt, float depth_error, float depth_error_rate, float velocity_error,
		      const SactPlusParams &params, bool adaptation_enabled = false);

	float depthLambda() const { return _depth_lambda; }
	float velocityLambda() const { return _velocity_lambda; }
	float depthTheta() const { return _depth_theta; }
	float velocityTheta() const { return _velocity_theta; }

private:
	static float variableSaturation(float error, float scale, float boundary, float exponent);

	float _depth_lambda{0.f};
	float _velocity_lambda{0.f};
	float _depth_theta{0.f};
	float _velocity_theta{0.f};
	float _depth_error_rate_filtered{0.f};
	float _velocity_error_rate_filtered{0.f};
	float _velocity_error_previous{0.f};
	float _elapsed{0.f};
	bool _initialized{false};
};
