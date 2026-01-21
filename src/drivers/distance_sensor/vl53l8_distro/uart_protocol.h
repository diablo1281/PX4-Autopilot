/*
 * uart_protocol.h
 *
 *  Created on: Jun 1, 2025
 *      Author: diabl
 */

#ifndef INC_UART_PROTOCOL_H_
#define INC_UART_PROTOCOL_H_

#include <stdint.h>

#define UART_PROT_MSG_HEADER_1	0xD3
#define UART_PROT_MSG_HEADER_2	0xAC

#define UART_PROT_MSG_HEADER_LEN	4
#define UART_PROT_MSG_CRC_LEN		2

#define UART_PROT_PAYLOAD_MIN_SIZE	(sizeof(CMD_short_s) - UART_PROT_MSG_HEADER_LEN)
#define UART_PROT_PAYLOAD_MAX_SIZE	(sizeof(VL_Range_Data_s<64>) - UART_PROT_MSG_HEADER_LEN)

#define UART_PROT_MSG_MIN_SIZE		(UART_PROT_MSG_HEADER_LEN + UART_PROT_PAYLOAD_MIN_SIZE)
#define UART_PROT_MSG_MAX_SIZE		(UART_PROT_MSG_HEADER_LEN + UART_PROT_PAYLOAD_MAX_SIZE)

#define UART_PROT_CMD_IS_ALIVE			0x00
#define UART_PROT_CMD_SENSOR_INIT		0x01
#define UART_PROT_CMD_SENSOR_RESET		0x02
#define UART_PROT_CMD_RNG_START			0x03
#define UART_PROT_CMD_RNG_STOP			0x04
#define UART_PROT_CMD_RNG_SINGLE		0x05

#define UART_PROT_CMD_OUT_SENSOR_RES		0x11
#define UART_PROT_CMD_OUT_TIMING_BUDGET		0x12
#define UART_PROT_CMD_OUT_FREQUENCY		0x13
#define UART_PROT_CMD_OUT_TARGET_ORD		0x14

#define UART_PROT_CMD_IN_SENSOR_RES		0x15
#define UART_PROT_CMD_IN_TIMING_BUDGET		0x16
#define UART_PROT_CMD_IN_FREQUENCY		0x17
#define UART_PROT_CMD_IN_TARGET_ORD		0x18

#define UART_PROT_TARGET_ORDER_CLOSEST		((uint8_t) 1U)
#define UART_PROT_TARGET_ORDER_STRONGEST	((uint8_t) 2U)

#define UART_PROT_CMD_TIMESYNC		0x21

#define UART_PROT_CMD_DATA_READY	0x30

#define UART_PROT_CMD_L4_X_CALIB_OFFSET		0x50
#define UART_PROT_CMD_L4_Y_CALIB_OFFSET		0x51
#define UART_PROT_CMD_L4_RNG_TIME_BUDGET	0x52

#define UART_PROT_CMD_TKF_OUTPUT_RATE			0x60
#define UART_PROT_CMD_TKF_X_CENTER_OFFSET		0x61
#define UART_PROT_CMD_TKF_Y_CENTER_OFFSET		0x62
#define UART_PROT_CMD_TKF_COG_LEVER_OFFSET		0x63
#define UART_PROT_CMD_TKF_PROCESS_NOISE_Q		0x64
#define UART_PROT_CMD_TKF_MIN_SIGMA			0x65
#define UART_PROT_CMD_TKF_USE_NIS_GATE			0x66
#define UART_PROT_CMD_TKF_NIS_GATE_THRESHOLD		0x67
#define UART_PROT_CMD_TKF_USE_SIGNAL_DEGRADE		0x68
#define UART_PROT_CMD_TKF_SIGNAL_DEGRADE_FACTOR		0x69


#define UART_PROT_CMD_STATUS_ACK	0xA1
#define UART_PROT_CMD_STATUS_ERROR	0xF0

#define UART_PROT_CMD_INVALID		0xFF

#define UART_PROT_CMD_MAX_SIZE		(34 * 50)

#define UART_PROT_ERROR_INIT		0x01
#define UART_PROT_ERROR_NOT_INIT	0x02

#define UART_PROT_ERROR_UNKNOWN		0xCF


#define VL53L8_RESOLUTION_4x4		16
#define VL53L8_RESOLUTION_8x8		64

/*
 * CMD:
 * - header (1+1)
 * - cmd_len (1) - CMD + payload[]
 * - CMD
 * - payload[]
 * - CRC (2)
 */

uint16_t calculate_crc(const uint8_t *data, size_t length);

#ifdef __cplusplus

struct __attribute__((__packed__)) CMD_short_s {
	uint8_t		header_1 = UART_PROT_MSG_HEADER_1;		// ->
	uint8_t		header_2 = UART_PROT_MSG_HEADER_2;		// - > HEADER
	uint16_t	packet_len = 2 + UART_PROT_MSG_CRC_LEN;	// ->			only payload, with CRC
	uint8_t		cmd;
	uint8_t		value;
	uint16_t	crc;

	uint16_t calculate_crc(bool overwrite = false) {
		uint16_t tmp = ::calculate_crc((uint8_t *)&packet_len, packet_len - UART_PROT_MSG_CRC_LEN);
		if(overwrite) crc = tmp;
		return tmp;
	}
};

struct __attribute__((__packed__)) CMD_multi_s {
	uint8_t		header_1 = UART_PROT_MSG_HEADER_1;		// ->
	uint8_t		header_2 = UART_PROT_MSG_HEADER_2;		// - > HEADER
	uint16_t	packet_len = 2 + UART_PROT_MSG_CRC_LEN;	// ->			only payload, with CRC
	uint8_t		cmd;
	union {
		uint32_t	value_u;
		int32_t		value_i;
		float		value_f;
		char		value_c[4];
	};
	uint16_t	crc;

	uint16_t calculate_crc(bool overwrite = false) {
		uint16_t tmp = ::calculate_crc((uint8_t *)&packet_len, packet_len - UART_PROT_MSG_CRC_LEN);
		if(overwrite) crc = tmp;
		return tmp;
	}
};

struct __attribute__((__packed__)) CMD_long_s {
	uint8_t		header_1 = UART_PROT_MSG_HEADER_1;		// ->
	uint8_t		header_2 = UART_PROT_MSG_HEADER_2;		// - > HEADER
	uint16_t	packet_len = 9 + UART_PROT_MSG_CRC_LEN;	// ->			only payload, with CRC
	uint8_t		cmd;
	uint64_t	value;
	uint16_t	crc;

	uint16_t calculate_crc(bool overwrite = false) {
		uint16_t tmp = ::calculate_crc((uint8_t *)&packet_len, packet_len - UART_PROT_MSG_CRC_LEN);
		if(overwrite) crc = tmp;
		return tmp;
	}
};

// struct __attribute__((__packed__)) Visual_Odometry_Data_s {
// 	uint8_t		header_1 = UART_PROT_MSG_HEADER_1;		// ->
// 	uint8_t		header_2 = UART_PROT_MSG_HEADER_2;		// - > HEADER
// 	uint16_t	packet_len = 70 + UART_PROT_MSG_CRC_LEN;	// ->			only payload, with CRC
// 	uint64_t	timestamp;
// 	float		x_m;	// distance to center line from center of the drone [m] (FRD)
// 	float		y_m;
// 	float		vx_mps;	// velocity from the center line (FRD) in m/s
// 	float		vy_mps;	// velocity from the center line (FRD) in m/s
// 	float		roll_rad;	// roll angle in radians
// 	float		pitch_rad;	// pitch angle in radians
// 	float		var_x_m2;
// 	float		var_y_m2;
// 	float		var_vx_m2s2;
// 	float		var_vy_m2s2;
// 	float		var_roll_rad2;
// 	float		var_pitch_rad2;
// 	float		rho_m;
// 	uint16_t	calculation_time_ms;
// 	uint16_t	inliers_total;	// liczba inlierów użytych do globalnej PCA
// 	uint16_t	n1;				// liczba punktów z sensora 1 po filtrze Z->XYZ
// 	uint16_t	n2;				// to samo dla sensora 2
// 	uint8_t		ok;				// 1 jeśli wynik sensowny (N>=2)
// 	uint8_t		ok_prior;		// 1 jeśli rho w [prior.rho0±prior.halfspan]
// 	uint16_t	crc;

// 	uint16_t calculate_crc(bool overwrite = false) {
// 		uint16_t tmp = ::calculate_crc((uint8_t *)&packet_len, packet_len - UART_PROT_MSG_CRC_LEN);
// 		if(overwrite) crc = tmp;
// 		return tmp;
// 	}
// };

struct __attribute__((__packed__)) Visual_Odometry_Data2_s {
	uint8_t		header_1 = UART_PROT_MSG_HEADER_1;		// ->
	uint8_t		header_2 = UART_PROT_MSG_HEADER_2;		// - > HEADER
	uint16_t	packet_len = 53 + UART_PROT_MSG_CRC_LEN;	// ->			only payload, with CRC
	uint64_t	timestamp;
	float		x_m;	// distance to center line from center of the drone [m] (FRD)
	float		y_m;
	float		vx_mps;	// velocity from the center line (FRD) in m/s
	float		vy_mps;	// velocity from the center line (FRD) in m/s
	float		var_x_m2;
	float		var_y_m2;
	float		var_vx_m2s2;
	float		var_vy_m2s2;
	uint32_t	calculation_time_us;
	uint32_t	rejected_x;
	uint32_t	rejected_y;
	uint8_t		frame;	// 0 = FRD, 1 = NED | 0b10 = attitude not found
	uint16_t	crc;

	uint16_t calculate_crc(bool overwrite = false) {
		uint16_t tmp = ::calculate_crc((uint8_t *)&packet_len, packet_len - UART_PROT_MSG_CRC_LEN);
		if(overwrite) crc = tmp;
		return tmp;
	}
};

struct __attribute__((__packed__)) Vehicle_Attitude_s {
	uint8_t		header_1 = UART_PROT_MSG_HEADER_1;		// ->
	uint8_t		header_2 = UART_PROT_MSG_HEADER_2;		// - > HEADER
	uint16_t	packet_len = 32 + UART_PROT_MSG_CRC_LEN;	// ->			only payload, with CRC
	uint64_t	timestamp;
	uint64_t	timestamp_sample;
	float		roll_rad;
	float		pitch_rad;
	float		yaw_rad;
	uint16_t	crc;

	uint16_t calculate_crc(bool overwrite = false) {
		uint16_t tmp = ::calculate_crc((uint8_t *)&packet_len, packet_len - UART_PROT_MSG_CRC_LEN);
		if(overwrite) crc = tmp;
		return tmp;
	}
};

struct __attribute__((__packed__)) VL_L4_Range_Data_s {
	uint8_t		header_1 = UART_PROT_MSG_HEADER_1;		// ->
	uint8_t		header_2 = UART_PROT_MSG_HEADER_2;		// - > HEADER
	uint16_t	packet_len = 33 + UART_PROT_MSG_CRC_LEN;	// ->	only payload, with CRC
	uint8_t		sensor_id;
	uint8_t		seq;
	uint64_t	timestamp;
	uint16_t	distance_mm;			/* Measured distance in millimeters */
	uint16_t	sigma_mm;				/* Estimated measurements std deviation in mm */
	uint32_t	ambient_rate_kcps;		/* Ambient noise in kcps */
	uint32_t	ambient_per_spad_kcps;	/* Ambient noise in kcps/SPAD */
	uint32_t	signal_rate_kcps;		/* Measured signal of the target in kcps */
	uint32_t	signal_per_spad_kcps;	/* Measured signal of the target in kcps/SPAD */
	uint16_t	number_of_spad;			/* Number of SPADs enabled */
	uint8_t		range_status;			/* Status of measurements. If the status is equal to 0, the data are valid*/
	uint16_t	crc;

	int16_t calculate_crc(bool overwrite = false) {
		uint16_t tmp = ::calculate_crc((uint8_t *)&packet_len, packet_len - UART_PROT_MSG_CRC_LEN);
		if(overwrite) crc = tmp;
		return tmp;
	}
};

template <size_t M>
struct __attribute__((__packed__)) VL_Range_Data_s {
	uint8_t		header_1 = UART_PROT_MSG_HEADER_1;		// ->
	uint8_t		header_2 = UART_PROT_MSG_HEADER_2;		// - > HEADER
	uint16_t	packet_len = 12 + (M * 7) + UART_PROT_MSG_CRC_LEN;	// ->	only payload, with CRC
	uint8_t		sensor_id;
	uint8_t		seq;
	uint8_t		resolution = M;
	uint64_t	timestamp;
	int8_t		silicon_temp;		// deg C
//	uint8_t		targets[M];		// number of valid targets detected per zone
	int16_t		distance[M];		// Distance to target						(RAW / 4) = mm
	uint16_t	range_sigma[M];		// Sigma of measured distances					(RAW / 128) = mm
//	uint8_t		reflectance[M];		// Estimated reflectance in %					RAW /= 2
//	uint32_t	ambient[M];		// Ambient noise in kcps/spads					RAW /= 2048
	uint8_t		ambient[M];		// Ambient noise in kcps/spads					(RAW /= 2048) - max 0xFF
//	uint32_t	signal[M];		// Signal returned to the sensor in kcps/spads			RAW /= 2048
	uint8_t		signal[M];		// Signal returned to the sensor in kcps/spads			(RAW /= 2048) - max 0xFF
	uint8_t		status[M];		// Status of measurment:  5 & 9 are OK; 255 if nothing
	uint16_t	crc;

	uint16_t calculate_crc(bool overwrite = false) {
		uint16_t tmp = ::calculate_crc((uint8_t *)&packet_len, packet_len - UART_PROT_MSG_CRC_LEN);
		if(overwrite) crc = tmp;
		return tmp;
	}
};

#endif	// _cplusplus


//uint16_t calculate_crc(const uint8_t *data, size_t length) {
//	uint16_t crc = 0xFFFF;
//	for(size_t i = 0; i < length; i++) {
//		crc ^= (uint16_t)data[i] << 8;
//		for(uint8_t j = 0; j < 8; j++) {
//			if(crc & 0x8000) crc = (crc << 1) ^ 0x1021;
//			else crc <<= 1;
//		}
//	}
//	return crc;
//}

#endif /* INC_UART_PROTOCOL_H_ */
