uint64 timestamp
uint64 timestamp_sample

# State in local NED frame, except dyaw which is relative yaw from PAT rope tangential motion.
float32 x
float32 y
float32 z
float32 vx
float32 vy
float32 vz
float32 dyaw

# Diagonal covariance entries.
float32 var_x
float32 var_y
float32 var_z
float32 var_vx
float32 var_vy
float32 var_vz
float32 var_dyaw

float32 yaw_ekf
