# DISTANCE_SENSOR_MATRIX message data

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id		# unique device ID for the sensor that does not change between power cycles

uint8 resolution		# 16 or 64

uint8 RESOLUTION_4x4		= 16
uint8 RESOLUTION_8x8		= 64

float32 min_distance		# Minimum distance the sensor can measure (in m)
float32 max_distance		# Maximum distance the sensor can measure (in m)
float32[64] current_distance	# Current distance reading (in m)
float32[64] variance		# Measurement variance (in m^2), 0 for unknown / invalid readings
int8[64] signal_quality		# Signal quality in percent (0...100%), where 0 = invalid signal, 100 = perfect signal, and -1 = unknown signal quality.

float32 h_fov # Sensor horizontal field of view (rad)
float32 v_fov # Sensor vertical field of view (rad)

uint8 orientation		# Direction the sensor faces from MAV_SENSOR_ORIENTATION enum

uint8 ROTATION_YAW_0		= 0
uint8 ROTATION_YAW_30		= 1
uint8 ROTATION_YAW_60		= 2
uint8 ROTATION_YAW_90		= 3
uint8 ROTATION_YAW_120		= 4
uint8 ROTATION_YAW_150		= 5
uint8 ROTATION_YAW_180		= 6
uint8 ROTATION_YAW_210		= 7
uint8 ROTATION_YAW_240		= 8
uint8 ROTATION_YAW_270		= 9
uint8 ROTATION_YAW_300		= 10
uint8 ROTATION_YAW_330		= 11
