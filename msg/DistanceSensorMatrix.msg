# DISTANCE_SENSOR_MATRIX message data

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id		# unique device ID for the sensor that does not change between power cycles

uint8 seq			# message sequence

uint8 resolution		# 16 or 64

uint8 RESOLUTION_4x4		= 16
uint8 RESOLUTION_8x8		= 64

int8 temperature		# Sensor temperature in deg C
#float32[64] distances		# Current distances reading (in m)
int16[64] distance_raw		# Current distances reading (RAW / 4 = mm)
#float32[64] variance		# Measurement variance (in m^2), 0 for unknown / invalid readings
uint16[64] variance_raw		# Measurement variance (RAW / 128 = mm)
uint8[64] signal		# Signal strength, capped at 0xFF
uint8[64] ambient		# Ambient noise, capped at 0xFF
uint8[64] status		# Measurement status

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
