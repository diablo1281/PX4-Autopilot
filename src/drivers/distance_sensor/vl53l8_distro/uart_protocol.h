/*
 * uart_protocol.h
 *
 *  Created on: Jun 1, 2025
 *      Author: diabl
 */

#ifndef INC_UART_PROTOCOL_H_
#define INC_UART_PROTOCOL_H_

#include <stdint.h>

#define UART_PROT_MSG_HEADER_1	0xAA
#define UART_PROT_MSG_HEADER_2	0x55

#define UART_PROT_MSG_HEADER_LEN	4
#define UART_PROT_MSG_CRC_LEN		2

#define UART_PROT_CMD_IS_ALIVE		0x00
#define UART_PROT_CMD_SENSOR_INIT	0x01
#define UART_PROT_CMD_SENSOR_RESET	0x02
#define UART_PROT_CMD_RNG_START		0x03
#define UART_PROT_CMD_RNG_STOP		0x04
#define UART_PROT_CMD_RNG_SINGLE	0x05
#define UART_PROT_CMD_SENSOR_RES	0x11
#define UART_PROT_CMD_TARGET_ORD	0x12
#define UART_PROT_CMD_TIMESYNC		0x21
#define UART_PROT_CMD_STATUS_ACK	0xA1
#define UART_PROT_CMD_STATUS_ERROR	0xF0

#define UART_PROT_CMD_INVALID		0xFF

#define UART_PROT_CMD_MAX_SIZE		(32 * 20)

#define UART_PROT_ERROR_INIT		0x01
#define UART_PROT_ERROR_NOT_INIT	0x02

#define UART_PROT_ERROR_UNKNOWN		0xCF

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

	uint16_t calculate_crc() {
		crc = ::calculate_crc((uint8_t *)&packet_len, packet_len - UART_PROT_MSG_CRC_LEN);
		return crc;
	}
};

struct __attribute__((__packed__)) CMD_long_s {
	uint8_t		header_1 = UART_PROT_MSG_HEADER_1;		// ->
	uint8_t		header_2 = UART_PROT_MSG_HEADER_2;		// - > HEADER
	uint16_t	packet_len = 9 + UART_PROT_MSG_CRC_LEN;	// ->			only payload, with CRC
	uint8_t		cmd;
	union {
		uint64_t	value;
		struct {
			uint8_t val_1;
			uint8_t val_2;
			uint8_t val_3;
			uint8_t val_4;
			uint8_t val_5;
			uint8_t val_6;
			uint8_t val_7;
			uint8_t val_8;
		};
	};
	uint16_t	crc;

	uint16_t calculate_crc() {
		crc = ::calculate_crc((uint8_t *)&packet_len, packet_len - UART_PROT_MSG_CRC_LEN);
		return crc;
	}
};

template <size_t M>
struct __attribute__((__packed__)) VL_Range_Data_s {
	uint8_t		header_1 = UART_PROT_MSG_HEADER_1;		// ->
	uint8_t		header_2 = UART_PROT_MSG_HEADER_2;		// - > HEADER
	uint16_t	packet_len = 11 + (M * 14) + UART_PROT_MSG_CRC_LEN;	// ->	only payload, with CRC
	uint64_t	timestamp;
	uint8_t		sensor_id;
	uint8_t		resolution = M;
	int8_t		silicon_temp;	// deg C
//	uint8_t		targets[M];		// number of valid targets detected per zone
	int16_t		distance[M];	// Distance to target in mm								RAW /= 4
	uint16_t	range_sigma[M];	// Sigma of measured distances im mm					RAW /= 128
	uint8_t		reflectance[M];	// Estimated reflectance in %							RAW /= 2
	uint32_t	ambient[M];		// Ambient noise in kcps/spads							RAW /= 2048
	uint32_t	signal[M];		// Signal returned to the sensor in kcps/spads			RAW /= 2048
	uint8_t		status[M];		// Status of measurment:  5 & 9 are OK; 255 if nothing
	uint16_t	crc;

	// void fill_data(VL53L8CX_ResultsData *data) {
	// 	silicon_temp = data->silicon_temp_degc;
	// 	memcpy(distance, data->distance_mm, sizeof(int16_t) * M);
	// 	memcpy(range_sigma, data->distance_mm, sizeof(uint16_t) * M);
	// 	memcpy(reflectance, data->distance_mm, sizeof(uint8_t) * M);
	// 	memcpy(ambient, data->distance_mm, sizeof(uint32_t) * M);
	// 	memcpy(signal, data->distance_mm, sizeof(uint32_t) * M);
	// 	memcpy(status, data->distance_mm, sizeof(uint8_t) * M);
	// }

	uint16_t calculate_crc() {
		crc = ::calculate_crc((uint8_t *)&packet_len, packet_len - UART_PROT_MSG_CRC_LEN);
		return crc;
	}
};



//struct __attribute__((__packed__)) VL_Range_Data_16_s {
//	uint8_t		header_1 = UART_PROT_MSG_HEADER_1;		// ->
//	uint8_t		header_2 = UART_PROT_MSG_HEADER_2;		// - > HEADER
//	uint16_t	packet_len = 11 + (16 * 15) + UART_PROT_MSG_CRC_LEN;	// ->	only payload, with CRC
//	uint64_t	timestamp;
//	uint8_t		sensor_id;
//	uint8_t		resolution;
//	int8_t		silicon_temp;	// deg C
//	uint8_t		targets[16];		// number of valid targets detected per zone
//	int16_t		distance[16];	// Distance to target in mm								RAW /= 4
//	uint16_t	range_sigma[16];	// Sigma of measured distances im mm					RAW /= 128
//	uint8_t		reflectance[16];	// Estimated reflectance in %							RAW /= 2
//	uint32_t	ambient[16];		// Ambient noise in kcps/spads							RAW /= 2048
//	uint32_t	signal[16];		// Signal returned to the sensor in kcps/spads			RAW /= 2048
//	uint8_t		status[16];		// Status of measurment:  5 & 9 are OK; 255 if nothing
//	uint16_t	crc;
//};
//
//struct __attribute__((__packed__)) VL_Range_Data_64_s {
//	uint8_t		header_1 = UART_PROT_MSG_HEADER_1;		// ->
//	uint8_t		header_2 = UART_PROT_MSG_HEADER_2;		// - > HEADER
//	uint16_t	packet_len = 11 + (64 * 15) + UART_PROT_MSG_CRC_LEN;	// ->	only payload, with CRC
//	uint64_t	timestamp;
//	uint8_t		sensor_id;
//	uint8_t		resolution;
//	int8_t		silicon_temp;	// deg C
//	uint8_t		targets[64];		// number of valid targets detected per zone
//	int16_t		distance[64];	// Distance to target in mm								RAW /= 4
//	uint16_t	range_sigma[64];	// Sigma of measured distances im mm					RAW /= 128
//	uint8_t		reflectance[64];	// Estimated reflectance in %							RAW /= 2
//	uint32_t	ambient[64];		// Ambient noise in kcps/spads							RAW /= 2048
//	uint32_t	signal[64];		// Signal returned to the sensor in kcps/spads			RAW /= 2048
//	uint8_t		status[64];		// Status of measurment:  5 & 9 are OK; 255 if nothing
//	uint16_t	crc;
//};

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
