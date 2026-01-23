uint64 timestamp
uint64 timestamp_sample

int32 dt_us			# dt[us]

uint8 sensor_id

# KF data
float32 z_m
float32 vz_m_s			# m/s
float32 var_z_m2
float32 var_vz_m2s2

# Raw sensor data
int64 x_sum
int64 y_sum
int16 x_delta
int16 y_delta
uint8 squal
uint8 squal2
uint8 rawdata_sum
uint8 rawdata_min
uint8 rawdata_max
uint16 shutter
uint16 x_resolution		# CPI (count per inch)
uint16 y_resolution		# CPI
bool new_data
