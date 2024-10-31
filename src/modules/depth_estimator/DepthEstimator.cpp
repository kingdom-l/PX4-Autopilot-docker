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

#include "DepthEstimator.hpp"

#include <math.h>

using namespace time_literals;
using namespace matrix;

DepthEstimator::DepthEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	/* fetch initial parameter values */
	updateParams();
}

DepthEstimator::~DepthEstimator()
{
	perf_free(_loop_perf);
}

bool
DepthEstimator::init()
{
	if (!_sensor_baro_sub.registerCallback() || !_adc_report_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void DepthEstimator::Run()
{
	if (should_exit()) {
		_sensor_baro_sub.unregisterCallback();
		_adc_report_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only update parameters if they changed
	bool params_updated = _parameter_update_sub.updated();

	// check for parameter updates
	if (params_updated) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	while(true)
	{
		//如果缓存中没有原始数据，则尝试获取原始数据
		for(int i=0; i<depth_estimated_s::DEPTH_TYPE_NUM; i++)
		{
			if(!_has_depth_raw_data[i])
			{
				if(i == depth_estimated_s::DEPTH_TYPE_PR && _param_hy_de_pr_ctrl.get())
				{
					_has_depth_raw_data[i] = get_depth_raw_data_pr(_depth_raw_data[i]);
				}
				else if(i == depth_estimated_s::DEPTH_TYPE_LV && _param_hy_de_lv_ctrl.get())
				{
					_has_depth_raw_data[i] = get_depth_raw_data_lv(_depth_raw_data[i]);
				}
			}
		}

		//判断当前应该用哪个原始数据进行更新，如果原始数据已经用完，则break
		uint8_t update_type;
		if(_has_depth_raw_data[0] && _has_depth_raw_data[1])
		{
			if(_depth_raw_data[0].timestamp_sample < _depth_raw_data[1].timestamp_sample)
			{
				update_type = 0;
			}
			else
			{
				update_type = 1;
			}
		}
		else if(_has_depth_raw_data[0])
		{
			update_type = 0;
		}
		else if(_has_depth_raw_data[1])
		{
			update_type = 1;
		}
		else
		{
			break;
		}

		//执行更新
		if(update_type == depth_estimated_s::DEPTH_TYPE_PR)
		{
			update_depth_pr(_depth_raw_data[update_type]);
			_has_depth_raw_data[update_type] = false;
		}
		else if(update_type == depth_estimated_s::DEPTH_TYPE_LV)
		{
			update_depth_lv(_depth_raw_data[update_type]);
			_has_depth_raw_data[update_type] = false;
		}
	}

	// backup schedule
	ScheduleDelayed(20_ms);

	perf_end(_loop_perf);
}

bool DepthEstimator::get_depth_raw_data_pr(DepthRawData& raw_data)
{
	sensor_baro_s sensor_baro;
	if(_sensor_baro_sub.update(&sensor_baro))
	{
		raw_data.timestamp_sample = sensor_baro.timestamp_sample;
		raw_data.depth_origin = (sensor_baro.pressure - _param_hy_de_pr_p0.get()) /\
					(_param_hy_de_pr_rho.get() * _param_hy_de_pr_g.get());
		return true;
	}
	return false;
}

bool DepthEstimator::get_depth_raw_data_lv(DepthRawData& raw_data)
{
	adc_report_s adc_report;
	if(_adc_report_sub.update(&adc_report))
	{
		//水位计的原始数据获取暂不实现
		return false;
	}
	return false;
}

void DepthEstimator::update_depth_pr(DepthRawData raw_data)
{
	depth_estimated_s de;

	float med = _medfilter_depth_pr.apply(raw_data.depth_origin);
	float maxd = _param_hy_de_pr_maxd.get();
	float depth_origin = raw_data.depth_origin;

	if(isInRange(depth_origin - med, -maxd, maxd))
	{
		_depth_estimated += _param_hy_de_pr_k.get()*(depth_origin - _depth_estimated);
		de.depth_invalid_pr = NAN;
	}
	else
	{
		de.depth_invalid_pr = depth_origin;
	}

	de.type = depth_estimated_s::DEPTH_TYPE_PR;

	de.timestamp_sample_pr = raw_data.timestamp_sample;
	de.timestamp_sample_lv = 0;

	de.depth_estimated = _depth_estimated;

	de.depth_origin_pr = depth_origin;
	de.depth_origin_lv = NAN;

	de.depth_medfilted_pr = med;

	de.depth_invalid_lv = NAN;

	de.timestamp = hrt_absolute_time();

	_depth_estimated_pub.publish(de);
}

void DepthEstimator::update_depth_lv(DepthRawData raw_data)
{
	//水位计的更新暂不实现
	return;
}

int DepthEstimator::task_spawn(int argc, char *argv[])
{
	DepthEstimator *instance = new DepthEstimator();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int DepthEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DepthEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
depth_estimator module is used to estimate the current depth.

)DESCR_STR");

	return 0;
}

extern "C" __EXPORT int depth_estimator_main(int argc, char *argv[])
{
	return DepthEstimator::main(argc, argv);
}
