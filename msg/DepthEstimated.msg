uint64 timestamp                # time since system start (microseconds)
uint8 type			# 这一帧的类型(如果是多个传感器，则还需要有index，此处假设所有传感器都只有一个，因此type可以代表index)
uint8 DEPTH_TYPE_PR = 0     	# 深度计
uint8 DEPTH_TYPE_LV = 1     	# 水位计
uint8 DEPTH_TYPE_NUM = 2
uint64 timestamp_sample_pr	# 深度计采样时间 (microseconds)
uint64 timestamp_sample_lv      # 水位计采样时间 (microseconds)
float32 depth_estimated		# 最终输出的估计值
float32 depth_origin_pr		# 原始值
float32 depth_origin_lv
float32 depth_medfilted_pr	# 中值滤波的结果
float32 depth_invalid_pr	# 不合法的值
float32 depth_invalid_lv
# TOPICS depth_estimated
