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

#pragma once

#include <lib/rate_control/rate_control.hpp>

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRate.hpp>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>

#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/depth_estimated.h>

#include <lib/mathlib/math/filter/MedianFilter.hpp>
#include <lib/mathlib/mathlib.h>

// using matrix::Eulerf;
// using matrix::Quatf;

// using uORB::SubscriptionData;

using namespace time_literals;
using namespace math;

class DepthEstimator final : public ModuleBase<DepthEstimator>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	DepthEstimator();
	~DepthEstimator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _sensor_baro_sub{this, ORB_ID(sensor_baro)};
	uORB::SubscriptionCallbackWorkItem _adc_report_sub{this, ORB_ID(adc_report)};

	uORB::Publication<depth_estimated_s>		_depth_estimated_pub{ORB_ID(depth_estimated)};

	struct DepthRawData
	{
		uint64_t timestamp_sample;
		float    depth_origin;
	};

	float _depth_estimated{0};

	MedianFilter<float, 15> _medfilter_depth_pr;

	//这里数组的下标既代表type，又代表index，因为默认了每种传感器都只能有一个
	DepthRawData 	_depth_raw_data[depth_estimated_s::DEPTH_TYPE_NUM];
	bool		_has_depth_raw_data[depth_estimated_s::DEPTH_TYPE_NUM]{false};

	bool get_depth_raw_data_pr(DepthRawData& raw_data);
	bool get_depth_raw_data_lv(DepthRawData& raw_data);

	void update_depth_pr(DepthRawData raw_data);
	void update_depth_lv(DepthRawData raw_data);

	perf_counter_t _loop_perf;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::HY_DE_PR_CTRL>) _param_hy_de_pr_ctrl,
		(ParamInt<px4::params::HY_DE_LV_CTRL>) _param_hy_de_lv_ctrl,
		(ParamFloat<px4::params::HY_DE_PR_P0>) _param_hy_de_pr_p0,
		(ParamFloat<px4::params::HY_DE_PR_RHO>) _param_hy_de_pr_rho,
		(ParamFloat<px4::params::HY_DE_PR_G>) _param_hy_de_pr_g,
		(ParamFloat<px4::params::HY_DE_PR_K>) _param_hy_de_pr_k,
		(ParamFloat<px4::params::HY_DE_PR_MAXD>) _param_hy_de_pr_maxd
	)

};
