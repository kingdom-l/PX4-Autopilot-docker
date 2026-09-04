/*
 * @Date: 2025-06-16 17:00:28
 * @LastEditTime: 2025-06-18 11:10:20
 * @LastEditors: wangwenshuai
 * @description:
 */
#pragma once

#include <drivers/drv_hrt.h>
#include <matrix/math.hpp>

#include <stdint.h>

/**
 * @brief 基于带时间戳位置样本的三轴滑动窗口速度估计器。
 *
 * @usage 新位置样本到达时调用 update()；返回 true 后通过 velocity() 读取世界系速度。
 */
class VelocityCalc
{
public:
	struct Params {
		uint8_t sliding_window{10};       ///< 滑动窗口样本数，范围 2..100。
		float outlier_min_gate_xy{0.10f}; ///< x/y 跳点剔除最小门限，单位 m。
		float outlier_min_gate_z{0.025f}; ///< z（深度）跳点剔除最小门限，单位 m。
		float maximum_rate_xy{5.0f};      ///< x/y 最大可信运动速率，单位 m/s。
		float maximum_rate_z{1.0f};       ///< z 最大可信运动速率，单位 m/s。
	};

	/**
	 * @brief 重置滑动窗口的全部内部状态。
	 *
	 * @usage 输入时间倒退、位置非法、传感器断流或外部希望重新初始化速度估计器时调用。
	 */
	void reset();

	/**
	 * @brief 使用三维位置更新滑动窗口速度估计器。
	 *
	 * @param position 世界系位置，单位 m。
	 * @param timestamp 位置采样时间戳，单位 us。
	 * @param params 滑窗样本数和跳点门限。
	 * @return true 表示缓存速度仍可使用；false 表示尚未初始化或内部状态已被重置。
	 *
	 * @usage 仅在新的位置样本到达时调用；孤立跳点被拒绝时保持最近有效速度，连续拒绝或时间异常时失效。
	 */
	bool update(const matrix::Vector3f &position, hrt_abstime timestamp, const Params &params);

	/**
	 * @brief 返回最近一次有效的三维速度估计。
	 *
	 * @return 世界系速度引用，单位 m/s。
	 * @usage 在 update() 返回 true 后读取；样本间隔期间可继续使用缓存值。
	 */
	const matrix::Vector3f &velocity() const { return _velocity; }

	/**
	 * @brief 判断最近一次 update() 是否用当前物理样本重新计算了完整三轴速度。
	 *
	 * @param 无。
	 * @return true 表示当前样本的三轴位置都被接受且产生了新的完整速度；false 表示仍在填窗、
	 *         样本被拒绝、估计器失效或当前只是在继续使用旧速度缓存。
	 * @usage 调用 update() 后立即读取；控制器用它区分“缓存速度仍有效”和“本周期确有新速度测量”。
	 */
	bool velocity_updated() const { return _velocity_updated; }

	/**
	 * @brief 判断最近一次 update() 的三轴位置样本是否全部进入了各自滑窗。
	 *
	 * @param 无。
	 * @return true 表示三轴样本均被接受；false 表示至少一轴输入非法或被跳点检测拒绝。
	 * @usage 调用 update() 后立即读取；即使窗口尚未填满，该值也能用于决定是否提交本帧位置给控制器。
	 */
	bool sample_accepted() const { return _sample_accepted; }

	/**
	 * @brief 判断最近一次 update() 中指定轴的位置样本是否进入了该轴滑窗。
	 *
	 * @param axis 轴编号，0=x，1=y，2=z。
	 * @return true 表示指定轴样本已接受；轴编号越界、输入非法或跳点被拒绝时返回 false。
	 * @usage 调用 update() 后立即读取；控制器据此只提交本周期通过筛选的轴。
	 */
	bool axis_sample_accepted(uint8_t axis) const { return (axis < 3) ? _axis_sample_accepted[axis] : false; }

	/**
	 * @brief 判断指定轴是否已有可用的缓存速度。
	 *
	 * @param axis 轴编号，0=x，1=y，2=z。
	 * @return true 表示该轴滑窗已完成拟合且此后未被重置。
	 * @usage 单轴跳点或重置不会阻断其他轴的速度反馈。
	 */
	bool axis_velocity_valid(uint8_t axis) const { return (axis < 3) ? _axis_velocity_valid[axis] : false; }

	/**
	 * @brief 返回指定轴最近一次有效的速度估计。
	 *
	 * @param axis 轴编号，0=x，1=y，2=z。
	 * @return 指定轴速度，单位 m/s；调用前应先检查 axis_velocity_valid()。
	 */
	float axis_velocity(uint8_t axis) const { return (axis < 3) ? _axis_velocity[axis] : 0.f; }

private:
	void setParams(const Params &params);
	bool updateSlidingWindow3D(const matrix::Vector3f &position, hrt_abstime timestamp, const Params &params);
	void resetAxis(uint8_t axis);
	bool axisOneDimensionCalc(uint8_t axis, uint8_t times, float position, hrt_abstime timestamp,
				  float outlier_min_gate, float maximum_rate, bool &sample_accepted);
	bool axisFitLine(uint8_t axis, uint8_t count, double &slope, double &intercept, double &rms) const;
	bool axisIsOutlier(uint8_t axis, float position, hrt_abstime timestamp,
			   float outlier_min_gate, float maximum_rate) const;

	float _pos3[3][100] {};                     ///< 三轴滑窗位置环形缓冲，[axis][sample]，单位 m。
	hrt_abstime _time3[3][100] {};              ///< 三轴滑窗位置采样时间戳，[axis][sample]，单位 us。
	uint8_t _window3[3] {};                     ///< 三轴滑窗每个轴的窗口长度。
	uint8_t _sample_count3[3] {};               ///< 三轴滑窗每个轴的有效样本数。
	uint8_t _index3[3] {};                      ///< 三轴滑窗每个轴的下一个写入位置。
	uint8_t _consecutive_rejects3[3] {};        ///< 三轴滑窗每个轴的连续跳点拒绝次数。
	hrt_abstime _last_time3[3] {};              ///< 三轴滑窗每个轴的上一帧时间戳，单位 us。
	float _axis_velocity[3] {};                 ///< 三轴滑窗拟合得到的世界系速度，单位 m/s。
	bool _axis_velocity_valid[3] {};            ///< 每个轴的缓存速度是否有效。

	Params _params{};                           ///< 当前生效的滑动窗口参数。
	matrix::Vector3f _velocity{};               ///< 最近一次有效的世界系速度估计，单位 m/s。
	bool _velocity_valid{false};                 ///< 缓存速度是否已经由完整三轴滑窗生成且未被内部重置。
	bool _velocity_updated{false};               ///< 最近一次 update() 是否生成了新的完整三轴速度。
	bool _sample_accepted{false};                ///< 最近一次 update() 的三轴位置是否全部进入了滑窗。
	bool _axis_sample_accepted[3] {};             ///< 最近一次 update() 的逐轴样本接受标志。

	static constexpr float MAX_SAMPLE_INTERVAL{0.5f};          ///< 允许的最大相邻采样间隔，单位 s。
	static constexpr float DEFAULT_OUTLIER_MIN_GATE_XY{0.10f}; ///< 默认 x/y 最小跳点门限，单位 m。
	static constexpr float DEFAULT_OUTLIER_MIN_GATE_Z{0.025f}; ///< 默认 z 最小跳点门限，单位 m。
	static constexpr float DEFAULT_MAXIMUM_RATE_XY{5.0f};      ///< 默认 x/y 最大可信速率，单位 m/s。
	static constexpr float DEFAULT_MAXIMUM_RATE_Z{1.0f};       ///< 默认 z 最大可信速率，单位 m/s。
	static constexpr float OUTLIER_SIGMA_GATE{5.0f};           ///< 滑窗残差 RMS 的倍数门限。
	static constexpr uint8_t MIN_OUTLIER_SAMPLES{4};           ///< 开始趋势检测前需要的最少样本数。
	static constexpr uint8_t MAX_CONSECUTIVE_REJECTS{3};       ///< 连续拒绝达到该次数后重置滑窗。
};
