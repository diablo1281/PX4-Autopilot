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

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/distance_sensor_single.h>
#include <uORB/topics/distance_sensor_matrix.h>
#include <uORB/topics/optical_navigation_horizontal.h>

#include <px4_platform_common/Serial.hpp>

#include "ring_buffer.hpp"

#include "uart_protocol.h"

using namespace device;
using namespace time_literals;

#define VL53L8_DISTRO_PORT_COUNT 10
#define VL53L8_DISTRO_MAX_SENSOR_COUNT	8
#define VL53L8_DISTRO_L4_MAX_SENSOR_COUNT 2
#define VL53L8_DISTRO_L4_FIRST_INDEX (VL53L8_DISTRO_PORT_COUNT - VL53L8_DISTRO_L4_MAX_SENSOR_COUNT + 1)

class VL53L8_Distro : public px4::ScheduledWorkItem
{
private:

enum PacketType : uint8_t {
	CMD_Short = 0,
	CMD_Multi,
	CMD_Long,
	MSG_VisualOdometry,
	MSG_VisualOdometry2,
	MSG_RangeData_16,
	MSG_RangeData_64,
	MSG_RangeData_L4,
	INVALID = 255
};

public:
	/**
	 * Default Constructor
	 * @param path The serial port to open for communicating with the sensor.
	 * @param baudrate Baudrate for selected serial port.
	 */
	VL53L8_Distro(const char *path, int baudrate);
	~VL53L8_Distro() override;

	int init();
	void print_info();

private:
	/**
	 * Reads data from serial UART and places it into a buffer.
	 */
	int collect(uint32_t timeout_us = 110_ms);
	// Wariant strumieniowy oparty o ring buffer (polecany do pracy ciągłej):
	int collect_streaming(uint32_t timeout_us = 110_ms);

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

	int measure(uint8_t command, bool ack = true);

	int send_timesync();

	int read_packet(PacketType &packet_type, uint32_t timeout_us = 0);
	// Pompuje UART -> ring buffer małymi porcjami
	size_t pump_uart_to_ring(uint32_t slice_timeout_us = 1000);

	int send_command(uint8_t cmd, uint8_t &value, bool ack = false, uint32_t timeout_us = 100_ms);
	int send_command(uint8_t cmd, uint32_t &value, bool ack = false, uint32_t timeout_us = 100_ms);

	int read_ACK(uint8_t &value, uint32_t timeout_us = 100_ms);
	int read_ACK(uint32_t &value, uint32_t timeout_us = 100_ms);

	int read_data(uint32_t timeout_us = 0);

	template <size_t M>
	bool parse_and_fill(VL_Range_Data_s<M> *data, distance_sensor_matrix_s &msg);

	bool parse_and_fill_L4(VL_L4_Range_Data_s *data, distance_sensor_single_s &msg);

	bool parse_and_fill_visual_odometry(Visual_Odometry_Data2_s *data);

	char 	_port[20]{};
	Serial	_uart{};
	speed_t _port_baudrate{1000000};

	// RX ring — wystarcza na kilka pakietów 8x8 + zapas
	static constexpr size_t RX_RB_SIZE = 8192;
	RingBuffer<RX_RB_SIZE> _rx;

	bool _task_should_exit{false};

	uint32_t _task_interval{100_ms};

	bool _is_initialized{false};

	bool _ranging_in_progress{false}; // Flag to indicate if a ranging operation is in progress

#if VL53L8_DISTRO_L4_MAX_SENSOR_COUNT > 0
	int16_t _sensor_L4_X_calib_offset_mm{-10};
	int16_t _sensor_L4_Y_calib_offset_mm{-14};
	uint16_t _sensor_L4_Y_range_budget_ms{20};

	uint16_t	_tkf_output_rate_ms{100};
	uint16_t	_tkf_X_center_offset_mm{61};
	uint16_t	_tkf_Y_center_offset_mm{62};
	float		_tkf_CoG_lever_height_m{0.05};
	float		_tkf_process_noise_Q{0.6};
	float		_tkf_min_sigma_R{0.005};
	bool		_tkf_use_NIS_GATE{true};
	float		_tkf_NIS_GATE_treshold{16.0};
	bool		_tkf_use_signal_degrade{true};
	float		_tkf_signal_degrade_factor{4.0};
#endif

	uint8_t _sensors_rotation[VL53L8_DISTRO_MAX_SENSOR_COUNT + VL53L8_DISTRO_L4_MAX_SENSOR_COUNT]{};
	uint8_t _sensors_out_resolution{VL53L8_RESOLUTION_4x4}; // Default resolution for VL53L8
	uint8_t _sensors_in_resolution{VL53L8_RESOLUTION_4x4}; // Default resolution for VL53L8
	uint8_t _sensors_out_frequency{10}; // Default frequency for VL53L8
	uint8_t _sensors_in_frequency{10}; // Default frequency for VL53L8
	uint8_t _sensors_out_target_order{UART_PROT_TARGET_ORDER_STRONGEST}; // Default target order for VL53L8
	uint8_t _sensors_in_target_order{UART_PROT_TARGET_ORDER_STRONGEST}; // Default target order for VL53L8
	uint8_t _sensors_count{0};
	uint16_t _sensors_active_mask{0};
	uint32_t _sensors_device_id[VL53L8_DISTRO_MAX_SENSOR_COUNT + VL53L8_DISTRO_L4_MAX_SENSOR_COUNT]{};
	uint8_t _buffer[UART_PROT_MSG_MAX_SIZE * 2];
	const uint16_t _buffer_size{sizeof(_buffer)};

	hrt_abstime _last_sync_time{};

	uORB::PublicationMulti<distance_sensor_matrix_s> _distance_sensor_pub[VL53L8_DISTRO_MAX_SENSOR_COUNT] {
		ORB_ID(distance_sensor_matrix)
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 1
		, ORB_ID(distance_sensor_matrix)
#endif
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 2
		, ORB_ID(distance_sensor_matrix)
#endif
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 3
		, ORB_ID(distance_sensor_matrix)
#endif
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 4
		, ORB_ID(distance_sensor_matrix)
#endif
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 5
		, ORB_ID(distance_sensor_matrix)
#endif
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 6
		, ORB_ID(distance_sensor_matrix)
#endif
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 7
		, ORB_ID(distance_sensor_matrix)
#endif
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 8
		, ORB_ID(distance_sensor_matrix)
#endif
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 9
		, ORB_ID(distance_sensor_matrix)
#endif
	};

	uORB::PublicationMulti<distance_sensor_single_s> _distance_sensor_L4_pub[VL53L8_DISTRO_L4_MAX_SENSOR_COUNT] {
		ORB_ID(distance_sensor_single),
		ORB_ID(distance_sensor_single)
	};

	uORB::Publication<optical_navigation_horizontal_s> _optical_navigation_pub{ORB_ID(optical_navigation_horizontal)};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};
