/**
 * @file VL53L8_Distro.hpp
 *
 * Driver for the ST VL53L8 ToF Sensor Distro Board connected via Serial.
 */

#pragma once

#include <termios.h>

#include <px4_log.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <px4_platform_common/Serial.hpp>

#include "uart_protocol.h"

using namespace device;
using namespace time_literals;

class VL53L8_Distro : public px4::ScheduledWorkItem
{
private:

enum PacketType : uint8_t {
	CMD_Short = 0,
	CMD_Long,
	MSG_RangeData_16,
	MSG_RangeData_64,
	INVALID = 255
};

public:
	/**
	 * Default Constructor
	 * @param serial_port The serial port to open for communicating with the sensor.
	 * @param rotation The sensor rotation relative to the vehicle body.
	 */
	VL53L8_Distro(const char *path);
	~VL53L8_Distro() override;

	int init();
	void print_info();

private:
	/**
	 * Reads data from serial UART and places it into a buffer.
	 */
	int collect(uint32_t timeout_us = 110_ms);

	/**
	 * Perform a reading cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void Run() override;

	/**
	 * Initialise the automatic measurement state machine and start it.
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/**
	 * Stops the automatic measurement state machine.
	 */
	void stop();

	/**
	 * Opens and configures the UART serial communications port.
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 */
	int open_serial_port(speed_t speed = 0);

	int initialize_sensor();

	int get_sensors_resolution();

	int measure(uint8_t command);

	int read_packet(PacketType &packet_type, uint32_t timeout_us = 0);

	int read_data(uint32_t timeout_us = 0);

	char 	_port[20]{};
	Serial	_uart{};
	speed_t _port_baudrate{1000000};

	bool _task_should_exit{false};

	bool _is_initialized{false};

	bool _ranging_in_progress{false}; // Flag to indicate if a ranging operation is in progress

	uint8_t _sensors_resolution{16}; // Default resolution for VL53L8
	uint8_t _sensors_count{0}; // Default number of sensors
	uint8_t _buffer[sizeof(VL_Range_Data_s<64>) * 6];
	const uint16_t _buffer_size{sizeof(_buffer)};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};
