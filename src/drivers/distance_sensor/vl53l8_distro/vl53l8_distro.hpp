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

#include "uart_protocol.h"

using namespace time_literals;

class VL53L8_Distro : public px4::ScheduledWorkItem
{
public:
	/**
	 * Default Constructor
	 * @param serial_port The serial port to open for communicating with the sensor.
	 * @param rotation The sensor rotation relative to the vehicle body.
	 */
	VL53L8_Distro(const char *serial_port);
	~VL53L8_Distro() override;

	int init();
	void print_info();

private:
	/**
	 * Reads data from serial UART and places it into a buffer.
	 */
	int collect();

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
	int open_serial_port(const speed_t speed = B1000000);

	int initialize_sensor();

	int get_sensors_resolution();

	int measure_single();

	bool parse_command(const CMD_short_s &cmd, uint8_t expected_cmd);
	bool parse_command(const CMD_long_s &cmd, uint8_t expected_cmd);

	const char *_serial_port{nullptr};
	int _port_fd{-1};

	bool _task_should_exit{false};

	bool _is_initialized{false};

	uint8_t _sensors_resolution{64}; // Default resolution for VL53L8

	uint8_t _buffer[sizeof(VL_Range_Data_s<64>)];
	uint16_t _buffer_len{sizeof(VL_Range_Data_s<64>)};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};
