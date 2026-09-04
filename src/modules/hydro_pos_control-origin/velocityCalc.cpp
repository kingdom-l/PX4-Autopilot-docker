/*
 * @Date: 2025-06-16 17:00:21
 * @LastEditTime: 2025-06-18 21:31:24
 * @LastEditors: wangwenshuai
 * @description:
 */
#include "velocityCalc.hpp"

#include <lib/mathlib/mathlib.h>

/**
 * @brief 重置滑动窗口的全部内部状态。
 *
 * @usage 输入时间倒退、位置非法、传感器断流或外部希望重新初始化速度估计器时调用。
 */
void VelocityCalc::reset()
{
	_velocity.zero();
	_velocity_updated = false;
	_sample_accepted = false;

	for (uint8_t axis = 0; axis < 3; axis++) {
		_axis_sample_accepted[axis] = false;
		resetAxis(axis);
	}
}

/**
 * @brief 写入并限幅当前滑动窗口参数。
 *
 * @param params 外部传入的滑窗样本数和跳点门限。
 * @usage 每次 update() 前调用，以支持运行中调参。
 */
void VelocityCalc::setParams(const Params &params)
{
	_params = params;
	_params.sliding_window = math::constrain(params.sliding_window, static_cast<uint8_t>(2), static_cast<uint8_t>(100));
	_params.outlier_min_gate_xy =
		(PX4_ISFINITE(params.outlier_min_gate_xy) && (params.outlier_min_gate_xy > 0.f)) ?
		params.outlier_min_gate_xy : DEFAULT_OUTLIER_MIN_GATE_XY;
	_params.outlier_min_gate_z =
		(PX4_ISFINITE(params.outlier_min_gate_z) && (params.outlier_min_gate_z > 0.f)) ?
		params.outlier_min_gate_z : DEFAULT_OUTLIER_MIN_GATE_Z;
	_params.maximum_rate_xy =
		(PX4_ISFINITE(params.maximum_rate_xy) && (params.maximum_rate_xy > 0.f)) ?
		params.maximum_rate_xy : DEFAULT_MAXIMUM_RATE_XY;
	_params.maximum_rate_z =
		(PX4_ISFINITE(params.maximum_rate_z) && (params.maximum_rate_z > 0.f)) ?
		params.maximum_rate_z : DEFAULT_MAXIMUM_RATE_Z;
}

/**
 * @brief 使用三维位置更新滑动窗口速度估计器。
 *
 * @param position 世界系位置，单位 m。
 * @param timestamp 位置采样时间戳，单位 us。
 * @param params 滑窗样本数和跳点门限。
 * @return true 表示缓存速度仍可使用；false 表示尚未初始化或内部状态已被重置。
 * @usage 仅在新的位置样本到达时调用；孤立跳点不撤销最近一次有效输出。
 */
bool VelocityCalc::update(const matrix::Vector3f &position, hrt_abstime timestamp, const Params &params)
{
	// 该标志只描述本次调用；即使旧速度仍有效，也不能跨周期保留 true。
	_velocity_updated = false;
	_sample_accepted = false;

	for (bool &axis_accepted : _axis_sample_accepted) {
		axis_accepted = false;
	}

	setParams(params);

	// Each axis validates its own sample so a single-axis NaN or jump does not
	// discard valid data from the other two axes.
	return updateSlidingWindow3D(position, timestamp, _params);
}

/**
 * @brief 使用三个独立的一维滑动窗口估计世界系三维速度。
 *
 * @param position 当前世界系三维位置，单位 m。
 * @param timestamp 当前样本时间戳，单位 us。
 * @param params 速度估计参数。
 * @return true 表示缓存三轴速度仍有效；false 表示至少一个轴尚未就绪或已经被重置。
 * @usage 由 update() 调用，输出通过 velocity() 读取。
 */
bool VelocityCalc::updateSlidingWindow3D(const matrix::Vector3f &position, hrt_abstime timestamp,
		const Params &params)
{
	bool ready = true; // 三个轴是否都已经填满窗口并成功拟合。
	bool sample_accepted = true; // 当前三轴位置是否都通过跳点检测并写入滑窗。

	for (uint8_t axis = 0; axis < 3; axis++) {
		bool axis_sample_accepted = false;
		const float outlier_min_gate = (axis == 2) ? params.outlier_min_gate_z : params.outlier_min_gate_xy;
		const float maximum_rate = (axis == 2) ? params.maximum_rate_z : params.maximum_rate_xy;
		const bool axis_ready = axisOneDimensionCalc(axis, params.sliding_window, position(axis), timestamp,
					outlier_min_gate, maximum_rate, axis_sample_accepted);
		_axis_sample_accepted[axis] = axis_sample_accepted;
		ready = axis_ready && ready;
		sample_accepted = axis_sample_accepted && sample_accepted;
	}

	_sample_accepted = sample_accepted;

	if (ready) {
		_velocity(0) = _axis_velocity[0];
		_velocity(1) = _axis_velocity[1];
		_velocity(2) = _axis_velocity[2];
		_velocity_valid = true;
		_velocity_updated = true;
	}

	// 单个跳点只代表本帧没有更新速度，不代表上一帧速度立即失效；resetAxis() 会在真正重置时撤销有效性。
	return _velocity_valid;
}

/**
 * @brief 重置三轴滑窗中指定轴的状态。
 *
 * @param axis 轴编号，0=x，1=y，2=z。
 * @usage 某一轴时间戳异常、输入非法或连续跳点过多时调用。
 */
void VelocityCalc::resetAxis(uint8_t axis)
{
	if (axis >= 3) {
		return;
	}

	// 任一轴被重置后，三轴速度不再处于同一完整窗口，必须重新等待全部轴就绪。
	_velocity_valid = false;
	_window3[axis] = 0;
	_sample_count3[axis] = 0;
	_index3[axis] = 0;
	_consecutive_rejects3[axis] = 0;
	_last_time3[axis] = 0;
	_axis_velocity[axis] = 0.f;
	_axis_velocity_valid[axis] = false;
	_axis_sample_accepted[axis] = false;
}

/**
 * @brief 更新指定轴的一维滑动窗口速度估计。
 *
 * @param axis 轴编号，0=x，1=y，2=z。
 * @param times 滑动窗口样本数，合法范围 2..100。
 * @param position 当前轴位置样本，单位 m。
 * @param timestamp 当前样本时间戳，单位 us。
 * @param outlier_min_gate 当前轴跳点剔除最小门限，单位 m。
 * @param maximum_rate 当前轴最大可信运动速率，单位 m/s。
 * @param[out] sample_accepted true 表示当前轴样本已写入滑窗；false 表示输入非法或被跳点检测拒绝。
 * @return true 表示该轴速度有效；false 表示样本不足、输入非法或样本被拒绝。
 * @usage updateSlidingWindow3D() 对三个轴分别调用该函数。
 */
bool VelocityCalc::axisOneDimensionCalc(uint8_t axis, uint8_t times, float position, hrt_abstime timestamp,
					float outlier_min_gate, float maximum_rate, bool &sample_accepted)
{
	sample_accepted = false;

	if ((axis >= 3) || (times < 2) || (times > 100) || !PX4_ISFINITE(position)) {
		resetAxis(axis);
		return false;
	}

	const hrt_abstime now = (timestamp != 0) ? timestamp :
				hrt_absolute_time(); // 当前位置样本时间戳，单位 us。

	if (_window3[axis] != times) {
		resetAxis(axis);
		_window3[axis] = times;
	}

	if (_sample_count3[axis] > 0) {
		if (now <= _last_time3[axis]) {
			resetAxis(axis);
			_window3[axis] = times;

		} else {
			const float dt = (now - _last_time3[axis]) * 1e-6f; // 当前样本与上一帧的间隔，单位 s。

			if (dt > MAX_SAMPLE_INTERVAL) {
				resetAxis(axis);
				_window3[axis] = times;
			}
		}
	}

	if (axisIsOutlier(axis, position, now, outlier_min_gate, maximum_rate)) {
		_consecutive_rejects3[axis]++;

		if (_consecutive_rejects3[axis] >= MAX_CONSECUTIVE_REJECTS) {
			resetAxis(axis);
			_window3[axis] = times;
		}

		return false;
	}

	sample_accepted = true;
	_pos3[axis][_index3[axis]] = position;
	_time3[axis][_index3[axis]] = now;
	_last_time3[axis] = now;
	_index3[axis] = (_index3[axis] + 1) % _window3[axis];
	_consecutive_rejects3[axis] = 0;

	if (_sample_count3[axis] < _window3[axis]) {
		_sample_count3[axis]++;
	}

	if (_sample_count3[axis] < _window3[axis]) {
		return false;
	}

	double slope = 0.0;     // 当前轴线性拟合斜率，即速度，单位 m/s。
	double intercept = 0.0; // 当前轴线性拟合截距，单位 m。
	double rms = 0.0;       // 当前轴线性拟合残差 RMS，单位 m。

	if (!axisFitLine(axis, _sample_count3[axis], slope, intercept, rms)) {
		return false;
	}

	_axis_velocity[axis] = static_cast<float>(slope);
	_axis_velocity_valid[axis] = true;
	return true;
}

/**
 * @brief 对指定轴窗口样本做最小二乘直线拟合。
 *
 * @param axis 轴编号，0=x，1=y，2=z。
 * @param count 参与拟合的样本数。
 * @param[out] slope 当前轴速度，单位 m/s。
 * @param[out] intercept 当前轴拟合截距，单位 m。
 * @param[out] rms 当前轴拟合残差 RMS，单位 m。
 * @return true 表示拟合成功；false 表示输入非法或时间分布退化。
 * @usage 三轴速度输出和跳点检测内部调用。
 */
bool VelocityCalc::axisFitLine(uint8_t axis, uint8_t count, double &slope, double &intercept, double &rms) const
{
	if ((axis >= 3) || (count < 2) || (_window3[axis] == 0)) {
		return false;
	}

	const uint8_t start = (_sample_count3[axis] < _window3[axis]) ? 0 : _index3[axis]; // 当前窗口最老样本。
	const hrt_abstime t0 = _time3[axis][start]; // 拟合相对时间零点，单位 us。

	double sum_t = 0.0;
	double sum_p = 0.0;
	double sum_tt = 0.0;
	double sum_tp = 0.0;

	for (uint8_t i = 0; i < count; i++) {
		const uint8_t sample_index = (start + i) % _window3[axis];
		const double t = static_cast<double>(_time3[axis][sample_index] - t0) * 1e-6;
		const double p = static_cast<double>(_pos3[axis][sample_index]);
		sum_t += t;
		sum_p += p;
		sum_tt += t * t;
		sum_tp += t * p;
	}

	const double n = static_cast<double>(count);
	const double denominator = n * sum_tt - sum_t * sum_t;

	if (denominator <= 1e-9) {
		return false;
	}

	slope = (n * sum_tp - sum_t * sum_p) / denominator;
	intercept = (sum_p - slope * sum_t) / n;

	double residual_square_sum = 0.0;

	for (uint8_t i = 0; i < count; i++) {
		const uint8_t sample_index = (start + i) % _window3[axis];
		const double t = static_cast<double>(_time3[axis][sample_index] - t0) * 1e-6;
		const double p = static_cast<double>(_pos3[axis][sample_index]);
		const double residual = p - (intercept + slope * t);
		residual_square_sum += residual * residual;
	}

	rms = sqrt(residual_square_sum / n);
	return true;
}

/**
 * @brief 判断指定轴的新位置样本是否为跳点。
 *
 * @param axis 轴编号，0=x，1=y，2=z。
 * @param position 当前轴新位置样本，单位 m。
 * @param timestamp 当前样本时间戳，单位 us。
 * @param outlier_min_gate 当前轴跳点剔除最小门限，单位 m。
 * @param maximum_rate 当前轴最大可信运动速率，单位 m/s。
 * @return true 表示新样本偏离当前趋势过大，应拒绝；false 表示可接收。
 * @usage 在写入滑窗前调用，减少位置跳变对速度估计的冲击。
 */
bool VelocityCalc::axisIsOutlier(uint8_t axis, float position, hrt_abstime timestamp,
				 float outlier_min_gate, float maximum_rate) const
{
	if ((axis >= 3) || (_sample_count3[axis] == 0) || !PX4_ISFINITE(position)
	    || !PX4_ISFINITE(maximum_rate) || (maximum_rate <= 0.f)) {
		return false;
	}

	const double min_gate = (PX4_ISFINITE(outlier_min_gate) && (outlier_min_gate > 0.0f)) ?
				static_cast<double>(outlier_min_gate) :
				static_cast<double>((axis == 2) ? DEFAULT_OUTLIER_MIN_GATE_Z : DEFAULT_OUTLIER_MIN_GATE_XY);
	const uint8_t last_index = (_index3[axis] + _window3[axis] - 1) % _window3[axis];
	const float dt = static_cast<float>(timestamp - _last_time3[axis]) * 1e-6f;
	const double step_gate = min_gate + static_cast<double>(maximum_rate * dt);

	// This physical-rate gate is active from the second sample, including the
	// regression warm-up period, so an early gross jump cannot seed the window.
	if (!PX4_ISFINITE(dt) || (dt <= 0.f)
	    || fabs(static_cast<double>(position - _pos3[axis][last_index])) > step_gate) {
		return true;
	}

	if (_sample_count3[axis] < MIN_OUTLIER_SAMPLES) {
		return false;
	}

	double slope = 0.0;
	double intercept = 0.0;
	double rms = 0.0;

	if (!axisFitLine(axis, _sample_count3[axis], slope, intercept, rms)) {
		return false;
	}

	const uint8_t start = (_sample_count3[axis] < _window3[axis]) ? 0 : _index3[axis];
	const double t = static_cast<double>(timestamp - _time3[axis][start]) * 1e-6;
	const double expected_position = intercept + slope * t;
	const double residual = fabs(static_cast<double>(position) - expected_position);
	const double gate = fmax(min_gate, static_cast<double>(OUTLIER_SIGMA_GATE) * rms);
	return residual > gate;
}
