uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id		# unique device ID for the sensor that does not change between power cycles

uint8 seq			# message sequence

uint16 distance_mm		# Measured distance in millimeters
uint16 sigma_mm			# Estimated measurements std deviation in mm
uint32 ambient_rate_kcps	# Ambient noise in kcps
uint32 ambient_per_spad_kcps	# Ambient noise in kcps/SPAD
uint32 signal_rate_kcps		# Measured signal of the target in kcps
uint32 signal_per_spad_kcps	# Measured signal of the target in kcps/SPAD
uint16 number_of_spad		# Number of SPADs enabled
uint8 range_status		# Status of measurements. If the status is equal to 0, the data are valid

uint8 axis			# Axis along which the sensor is measuring distance

uint8 AXIS_X			= 0
uint8 AXIS_Y			= 1
uint8 AXIS_UNKNOWN		= 255
