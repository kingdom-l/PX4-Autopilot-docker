/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#include "VehicleDepthData.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/events.h>
#include <lib/geo/geo.h>
#include <lib/atmosphere/atmosphere.h>


namespace sensors
{

using namespace matrix;
using namespace atmosphere;

static constexpr uint32_t SENSOR_TIMEOUT{300_ms};

VehicleDepthData::VehicleDepthData() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_vehicle_depth_data_pub.advertise();

	_voter.set_timeout(SENSOR_TIMEOUT);
}

VehicleDepthData::~VehicleDepthData()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleDepthData::Start()
{
	ScheduleNow();
	return true;
}

void VehicleDepthData::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}
}

void VehicleDepthData::DepthTemperatureUpdate()
{
	differential_pressure_s differential_pressure;

	static constexpr float temperature_min_celsius = -20.f;
	static constexpr float temperature_max_celsius = 35.f;

	// update air temperature if data from differential pressure sensor is finite and not exactly 0
	// limit the range to max 35°C to limt the error due to heated up airspeed sensors prior flight
	if (_differential_pressure_sub.update(&differential_pressure) && PX4_ISFINITE(differential_pressure.temperature)
	    && fabsf(differential_pressure.temperature) > FLT_EPSILON) {

		_depth_temperature_celsius = math::constrain(differential_pressure.temperature, temperature_min_celsius,
					   temperature_max_celsius);
		PX4_INFO("here DepthTemperatureUpdate");
	}
}

bool VehicleDepthData::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		// update priority
		for (int instance = 0; instance < MAX_SENSOR_COUNT; instance++) {

			const int32_t priority_old = _calibration[instance].priority();

			_calibration[instance].ParametersUpdate();

			const int32_t priority_new = _calibration[instance].priority();

			if (priority_old != priority_new) {
				if (_priority[instance] == priority_old) {
					_priority[instance] = priority_new;

				} else {
					// change relative priority to incorporate any sensor faults
					int priority_change = priority_new - priority_old;
					_priority[instance] = math::constrain(_priority[instance] + priority_change, 1, 100);
				}
			}
		}

		return true;
	}

	return false;
}

void VehicleDepthData::Run()
{
	perf_begin(_cycle_perf);

	const hrt_abstime time_now_us = hrt_absolute_time();

	const bool parameter_update = ParametersUpdate();

	DepthTemperatureUpdate();
	PX4_INFO("here, depth");

	bool updated[MAX_SENSOR_COUNT] {};

	for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {

		const bool was_advertised = _advertised[uorb_index];

		if (!_advertised[uorb_index]) {
			// use data's timestamp to throttle advertisement checks
			if ((_last_publication_timestamp[uorb_index] == 0)
			    || (time_now_us > _last_publication_timestamp[uorb_index] + 1_s)) {

				if (_sensor_sub[uorb_index].advertised()) {
					_advertised[uorb_index] = true;

				} else {
					_last_publication_timestamp[uorb_index] = time_now_us;
				}
			}
		}

		if (_advertised[uorb_index]) {
			int sensor_sub_updates = 0;
			sensor_baro_s report;
			while ((sensor_sub_updates < sensor_baro_s::ORB_QUEUE_LENGTH) && _sensor_sub[uorb_index].update(&report)) {
				sensor_sub_updates++;

				if (_calibration[uorb_index].device_id() != report.device_id) {
					_calibration[uorb_index].set_device_id(report.device_id);
					_priority[uorb_index] = _calibration[uorb_index].priority();
				}

				if (_calibration[uorb_index].enabled()) {

					if (!was_advertised) {
						if (uorb_index > 0) {
							/* the first always exists, but for each further sensor, add a new validator */
							if (!_voter.add_new_validator()) {
								PX4_ERR("failed to add validator for %s %i", _calibration[uorb_index].SensorString(), uorb_index);
							}
						}

						if (_selected_sensor_sub_index < 0) {
							_sensor_sub[uorb_index].registerCallback();
						}

						ParametersUpdate(true);
					}

					// pressure corrected with offset (if available)
					//这一部分对气压进行了校正
					_calibration[uorb_index].SensorCorrectionsUpdate();
					const float pressure_corrected = _calibration[uorb_index].Correct(report.pressure);
					sensor_data[uorb_index][sensor_sub_updates-1]=pressure_corrected;//添加进数组
					const float pressure_sealevel_pa = _param_sens_depth_qnh.get() * 100.f;
					float data_array[3];
					if  (_param_sens_depth_medium.get() == Medium::Air){
					//float data_array[3] {pressure_corrected, report.temperature, getAltitudeFromPressure(pressure_corrected, pressure_sealevel_pa)};
						data_array[0]=pressure_corrected;
						data_array[1]=report.temperature;
						data_array[2]=getAltitudeFromPressure(pressure_corrected, pressure_sealevel_pa);
					}
					else{
						if (_param_sens_depth_medium.get() == Medium::FreshWater){
							float rho = 997.0474;
							data_array[0]=pressure_corrected;
							data_array[1]=report.temperature;
							data_array[2]=getDepthFromPressure(pressure_corrected, pressure_sealevel_pa,rho);
						}
						else{
							float rho = 1023.6;
							data_array[0]=pressure_corrected;
							data_array[1]=report.temperature;
							data_array[2]=getDepthFromPressure(pressure_corrected, pressure_sealevel_pa,rho);
						}
					}
					_voter.put(uorb_index, report.timestamp, data_array, report.error_count, _priority[uorb_index]);

					_timestamp_sample_sum[uorb_index] += report.timestamp_sample;
					_data_sum[uorb_index] += pressure_corrected;
					_temperature_sum[uorb_index] += report.temperature;
					_data_sum_count[uorb_index]++;

					_last_data[uorb_index] = pressure_corrected;

					updated[uorb_index] = true;
				}
			}
		}
		//for (int i = 0; i < 6; i++){
			//for (int j = 0; j < 5 - i; j++){
				//if (sensor_data[uorb_index][j] > sensor_data[uorb_index][j + 1]){
					//float temp = sensor_data[uorb_index][j];
					//sensor_data[uorb_index][j] = sensor_data[uorb_index][j + 1];
					//sensor_data[uorb_index][j + 1] = temp;
				//}
			//}
		//}
		//_data_sum[uorb_index] = _data_sum[uorb_index]-sensor_data[uorb_index][0]-sensor_data[uorb_index][5];
	}

	// check for the current best sensor
	int best_index = 0;
	_voter.get_best(time_now_us, &best_index);

	if (best_index >= 0) {
		// handle selection change (don't process on same iteration as parameter update)
		if ((_selected_sensor_sub_index != best_index) && !parameter_update) {
			// clear all registered callbacks
			for (auto &sub : _sensor_sub) {
				sub.unregisterCallback();
			}

			if (_selected_sensor_sub_index >= 0) {
				PX4_INFO("%s switch from #%" PRId8 " -> #%d", _calibration[_selected_sensor_sub_index].SensorString(),
					 _selected_sensor_sub_index, best_index);
			}

			_selected_sensor_sub_index = best_index;
			_sensor_sub[_selected_sensor_sub_index].registerCallback();
		}
	}

	// Publish
	if (_param_sens_depth_rate.get() > 0) {
		int interval_us = 1e6f / _param_sens_depth_rate.get();

		for (int instance = 0; instance < MAX_SENSOR_COUNT; instance++) {
			if (updated[instance] && (_data_sum_count[instance] > 0)) {

				const hrt_abstime timestamp_sample = _timestamp_sample_sum[instance] / _data_sum_count[instance];

				if (timestamp_sample >= _last_publication_timestamp[instance] + interval_us) {

					bool publish = (time_now_us <= timestamp_sample + 1_s);

					if (publish) {
						publish = (_selected_sensor_sub_index >= 0)
							  && (instance == _selected_sensor_sub_index)
							  && (_voter.get_sensor_state(_selected_sensor_sub_index) == DataValidator::ERROR_FLAG_NO_ERROR);
					}

					if (publish) {
						const float pressure_pa = _data_sum[instance] / (_data_sum_count[instance]);//这里采用了累加求平均的方式
						const float temperature = _temperature_sum[instance] / _data_sum_count[instance];

						const float pressure_sealevel_pa = _param_sens_depth_qnh.get() * 100.f;
						float altitude;
						if (_param_sens_depth_medium.get() == Medium::Air)
						{
							altitude = getAltitudeFromPressure(pressure_pa, pressure_sealevel_pa);//这里需要修改计算水深的公式
						}
						else
						{
							//const float p1 = _param_sens_depth_qnh.get() * 100.f;
							//const float p0 = pressure_pa;
							float rho;//代表液体密度
							if(_param_sens_depth_medium.get() == Medium::FreshWater){
								rho = 997.0474;
								altitude =getDepthFromPressure(pressure_pa, pressure_sealevel_pa,rho);
								//altitude =_data_sum[instance];
							}
							else{
								rho = 1023.6;
								altitude =getDepthFromPressure(pressure_pa, pressure_sealevel_pa,rho);
							}
							//altitude =(p0-p1) / (rho * CONSTANTS_ONE_G)*100;
						}
						// calculate air density
						const float depth_density = getDensityFromPressureAndTemp(pressure_pa, temperature);

						// populate vehicle_air_data with and publish
						vehicle_air_data_s out{};
						out.timestamp_sample = timestamp_sample;
						out.baro_device_id = _calibration[instance].device_id();
						out.baro_alt_meter = altitude;//压力计计算高度
						out.baro_temp_celcius = temperature;
						out.baro_pressure_pa = pressure_pa;
						out.rho = depth_density;
						out.eas2tas = sqrtf(kAirDensitySeaLevelStandardAtmos / math::max(depth_density, FLT_EPSILON));
						out.calibration_count = _calibration[instance].calibration_count();
						out.timestamp = hrt_absolute_time();

						_vehicle_depth_data_pub.publish(out);
					}

					_last_publication_timestamp[instance] = timestamp_sample;

					// reset
					_timestamp_sample_sum[instance] = 0;
					_data_sum[instance] = 0;
					_temperature_sum[instance] = 0;
					_data_sum_count[instance] = 0;
					for (int i = 0; i < 6; i++){
						sensor_data[instance][i]=0.0f;
					}
				}
			}
		}
	}

	if (!parameter_update) {
		CheckFailover(time_now_us);
	}

	UpdateStatus();

	// reschedule timeout
	ScheduleDelayed(50_ms);

	perf_end(_cycle_perf);
}

void VehicleDepthData::CheckFailover(const hrt_abstime &time_now_us)
{
	// check failover and report (save failover report for a cycle where parameters didn't update)
	if (_last_failover_count != _voter.failover_count()) {
		uint32_t flags = _voter.failover_state();
		int failover_index = _voter.failover_index();

		if (flags != DataValidator::ERROR_FLAG_NO_ERROR) {
			if (failover_index >= 0 && failover_index < MAX_SENSOR_COUNT) {

				if (time_now_us > _last_error_message + 3_s) {
					mavlink_log_emergency(&_mavlink_log_pub, "%s #%i failed: %s%s%s%s%s!\t",
							      _calibration[failover_index].SensorString(),
							      failover_index,
							      ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " OFF" : ""),
							      ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " STALE" : ""),
							      ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " TIMEOUT" : ""),
							      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " ERR CNT" : ""),
							      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " ERR DNST" : ""));

					events::px4::enums::sensor_failover_reason_t failover_reason{};

					if (flags & DataValidator::ERROR_FLAG_NO_DATA) { failover_reason = failover_reason | events::px4::enums::sensor_failover_reason_t::no_data; }

					if (flags & DataValidator::ERROR_FLAG_STALE_DATA) { failover_reason = failover_reason | events::px4::enums::sensor_failover_reason_t::stale_data; }

					if (flags & DataValidator::ERROR_FLAG_TIMEOUT) { failover_reason = failover_reason | events::px4::enums::sensor_failover_reason_t::timeout; }

					if (flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) { failover_reason = failover_reason | events::px4::enums::sensor_failover_reason_t::high_error_count; }

					if (flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) { failover_reason = failover_reason | events::px4::enums::sensor_failover_reason_t::high_error_density; }

					/* EVENT
					 * @description
					 * Land immediately and check the system.
					 */
					events::send<uint8_t, events::px4::enums::sensor_failover_reason_t>(
						events::ID("sensor_failover_baro"), events::Log::Emergency, "Baro sensor #{1} failure: {2}", failover_index,
						failover_reason);

					_last_error_message = time_now_us;
				}

				// reduce priority of failed sensor to the minimum
				_priority[failover_index] = 1;
			}
		}

		_last_failover_count = _voter.failover_count();
	}
}

void VehicleDepthData::UpdateStatus()
{
	if (_selected_sensor_sub_index >= 0) {
		sensors_status_s sensors_status{};
		sensors_status.device_id_primary = _calibration[_selected_sensor_sub_index].device_id();

		float mean{};
		int sensor_count = 0;

		for (int sensor_index = 0; sensor_index < MAX_SENSOR_COUNT; sensor_index++) {
			if ((_calibration[sensor_index].device_id() != 0) && (_calibration[sensor_index].enabled())) {
				sensor_count++;
				mean += _last_data[sensor_index];
			}
		}

		if (sensor_count > 0) {
			mean /= sensor_count;
		}

		for (int sensor_index = 0; sensor_index < MAX_SENSOR_COUNT; sensor_index++) {
			if (_calibration[sensor_index].device_id() != 0) {

				_sensor_diff[sensor_index] = 0.95f * _sensor_diff[sensor_index] + 0.05f * (_last_data[sensor_index] - mean);

				sensors_status.device_ids[sensor_index] = _calibration[sensor_index].device_id();
				sensors_status.inconsistency[sensor_index] = _sensor_diff[sensor_index];
				sensors_status.healthy[sensor_index] = (_voter.get_sensor_state(sensor_index) == DataValidator::ERROR_FLAG_NO_ERROR);
				sensors_status.priority[sensor_index] = _voter.get_sensor_priority(sensor_index);
				sensors_status.enabled[sensor_index] = _calibration[sensor_index].enabled();
				sensors_status.external[sensor_index] = _calibration[sensor_index].external();

			} else {
				sensors_status.inconsistency[sensor_index] = NAN;
			}
		}

		sensors_status.timestamp = hrt_absolute_time();
		_sensors_status_depth_pub.publish(sensors_status);
	}
}

void VehicleDepthData::PrintStatus()
{
	if (_selected_sensor_sub_index >= 0) {
		PX4_INFO_RAW("[vehicle_air_data] selected %s: %" PRIu32 " (%" PRId8 ")\n",
			     _calibration[_selected_sensor_sub_index].SensorString(),
			     _calibration[_selected_sensor_sub_index].device_id(), _selected_sensor_sub_index);
	}

	_voter.print();

	for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
		if (_advertised[i] && (_priority[i] > 0)) {
			_calibration[i].PrintStatus();
		}
	}
}

}; // namespace sensors
