uint64 timestamp
uint64 timestamp_sample

uint32 sequence

# Measurement and innovation details for the event. These fields can be used for detailed analysis of the filter's performance and to understand the reasons behind health status and event outcomes.
float32 measurement
float32 prediction
float32 innovation
float32 innovation_var
float32 nis
float32 r

# Compact measurement row representation.
# For VL_X/VL_Y: h0/h1 are coefficients of [x_N, y_E].
# For other event types the model is implied by event_type.
float32 h0
float32 h1

float32[7] k
float32[7] state_after
float32[7] p_diag_after
float32[3] p_v_diag_after

# This message is intended to capture all events related to the shaft Kalman filter, including measurement updates, predictions, and health check results. It is designed to be published at a high rate (e.g. every time the filter runs) and can be used for debugging, monitoring, and offline analysis of the filter's performance and behavior.
uint8 EVENT_NONE	= 0
uint8 EVENT_VL_X	= 1
uint8 EVENT_VL_Y	= 2
uint8 EVENT_PAT_Z	= 3
uint8 EVENT_PAT_DYAW	= 4
uint8 EVENT_BARO_Z	= 5
uint8 EVENT_XY_ZUPT_X	= 6
uint8 EVENT_XY_ZUPT_Y	= 7
uint8 EVENT_Z_ZUPT	= 8

uint8 event_type

# Health status of the measurement that triggered the event. This is not only faults; it can indicate degradation, rejection, or data-source faults.
uint8 HEALTH_UNKNOWN	= 0
uint8 HEALTH_OK		= 1
uint8 HEALTH_DEGRADED	= 2
uint8 HEALTH_REJECTED	= 3
uint8 HEALTH_FAULT	= 4
uint8 HEALTH_DISABLED	= 5

uint8 health


# Reasons produced by health/plausibility checks. These are not only faults;
# they can indicate degradation, rejection, or data-source faults.
uint16 CHECK_NIS		= 1
uint16 CHECK_RANGE_STATUS	= 2
uint16 CHECK_HIGH_SIGMA		= 4
uint16 CHECK_LOW_SIGNAL		= 8
uint16 CHECK_HIGH_AMBIENT	= 16
uint16 CHECK_RAW_VELOCITY	= 32
uint16 CHECK_RAW_ACCELERATION	= 64
uint16 CHECK_RADIAL_LIMIT	= 128
uint16 CHECK_SQUAL2		= 256
uint16 CHECK_FREEZE		= 512
uint16 CHECK_SETPOINT_MISMATCH	= 1024
uint16 CHECK_TIMEOUT		= 2048
uint16 CHECK_SHUTTER		= 4096

uint16 check_flags
