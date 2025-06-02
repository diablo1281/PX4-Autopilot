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

#define UART_PROT_CMD_SENSOR_RESET	0x01
#define UART_PROT_CMD_RNG_START		0x02
#define UART_PROT_CMD_RNG_STOP		0x03
#define UART_PROT_CMD_RNG_SINGLE	0x04
#define UART_PROT_CMD_SENSOR_RES	0x11
#define UART_PROT_CMD_TARGET_ORD	0x12
#define UART_PROT_CMD_TIMESYNC		0x21
#define UART_PROT_CMD_STATUS_ACK	0xA1
#define UART_PROT_CMD_STATUS_ERROR	0xF0

#define UART_PROT_CMD_INVALID		0xFF

#define UART_PROT_CMD_MAX_SIZE		(32 * 20)

#define UART_PROT_ERROR_INIT		0x01
#define UART_PROT_ERROR_NOT_INIT	0x02

/*
 * CMD:
 * - header (1+1)
 * - cmd_len (1) - CMD + payload[]
 * - CMD
 * - payload[]
 * - CRC (2)
 */

struct __attribute__((__packed__)) CMD_short_s {
	uint8_t		header_1 = UART_PROT_MSG_HEADER_1;		// ->
	uint8_t		header_2 = UART_PROT_MSG_HEADER_2;		// - > HEADER
	uint16_t	packet_len = 2 + UART_PROT_MSG_CRC_LEN;	// ->			only payload, with CRC
	uint8_t		cmd;
	uint8_t		value;
	uint16_t	crc;
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
};

struct __attribute__((__packed__)) VL_Range_Data_16_s {
	uint8_t		header_1 = UART_PROT_MSG_HEADER_1;		// ->
	uint8_t		header_2 = UART_PROT_MSG_HEADER_2;		// - > HEADER
	uint16_t	packet_len = 74 + UART_PROT_MSG_CRC_LEN;	// ->			only payload, with CRC
	uint64_t	timestamp;
	uint8_t		sensor_id;
	uint8_t		resolution;
	uint16_t	distance[16];
	uint16_t	status[16];
	uint16_t	ambient[16];
	uint16_t	signal[16];
	uint16_t	crc;

};

struct __attribute__((__packed__)) VL_Range_Data_64_s {
	uint8_t		header_1 = UART_PROT_MSG_HEADER_1;			// ->
	uint8_t		header_2 = UART_PROT_MSG_HEADER_2;			// - > HEADER
	uint16_t	packet_len = 266 + UART_PROT_MSG_CRC_LEN;	// ->			only payload, with CRC
	uint64_t	timestamp;
	uint8_t		sensor_id;
	uint8_t		resolution;
	uint16_t	distance[64];
	uint16_t	status[64];
	uint16_t	ambient[64];
	uint16_t	signal[64];
	uint16_t	crc;
};

uint16_t calculate_crc(const uint8_t *data, size_t length) {
	uint16_t crc = 0xFFFF;
	for(size_t i = 0; i < length; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for(uint8_t j = 0; j < 8; j++) {
			if(crc & 0x8000) crc = (crc << 1) ^ 0x1021;
			else crc <<= 1;
		}
	}
	return crc;
}

#endif /* INC_UART_PROTOCOL_H_ */
