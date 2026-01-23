#include "vl53l8_distro.hpp"

#include <cerrno>

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include <lib/drivers/device/Device.hpp>

#include <matrix/matrix/Euler.hpp>
#include <matrix/matrix/Quaternion.hpp>

#include <parameters/param.h>

#include "crc.h"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

VL53L8_Distro::VL53L8_Distro(const char *path, int baudrate) :
    ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(path))
{
    /* store port name */
	strncpy(_port, path, sizeof(_port) - 1);
    /* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

    _port_baudrate = baudrate;

    device::Device::DeviceId device_id;
    device_id.devid_s.devtype = DRV_DIST_DEVTYPE_VL53L8_DISTRO;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

    // Advertise all topics to have consistent instance ID for sensors
    for(uint8_t i = 0; i < (VL53L8_DISTRO_MAX_SENSOR_COUNT); i++) {
        _distance_sensor_pub[i].advertise();
        device_id.devid_s.address = i + 1;
        _sensors_device_id[i] = device_id.devid;
    }

    for(uint8_t i = 0; i < VL53L8_DISTRO_L4_MAX_SENSOR_COUNT; i++) {
        _distance_sensor_L4_pub[i].advertise();
        device_id.devid_s.address = i + VL53L8_DISTRO_L4_FIRST_INDEX;
        _sensors_device_id[i + VL53L8_DISTRO_MAX_SENSOR_COUNT] = device_id.devid;
    }
}

VL53L8_Distro::~VL53L8_Distro()
{
    stop();

    perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int VL53L8_Distro::init()
{
    int32_t param = 0;
    param_get(param_find("VL_D_OUT_RES"), &param);
    if(param > 0) {
        _sensors_out_resolution = (uint8_t) param;
    } else {
        PX4_ERR("Error reading `VL_D_OUT_RES` parameter!");
    }
    param_get(param_find("VL_D_IN_RES"), &param);
    if(param > 0) {
        _sensors_in_resolution = (uint8_t) param;
    } else {
        PX4_ERR("Error reading `VL_D_IN_RES` parameter!");
    }

    param_get(param_find("VL_D_OUT_FREQ"), &param);
    if(param > 0) {
        _sensors_out_frequency = (uint8_t) param;
        _task_interval = 1_s / _sensors_out_frequency;
    } else {
        PX4_ERR("Error reading `VL_D_OUT_FREQ` parameter!");
    }
    param_get(param_find("VL_D_IN_FREQ"), &param);
    if(param > 0) {
        _sensors_in_frequency = (uint8_t) param;
    } else {
        PX4_ERR("Error reading `VL_D_IN_FREQ` parameter!");
    }

    param_get(param_find("VL_D_OUT_TARGET"), &param);
    if(param > 0) {
        _sensors_out_target_order = (uint8_t) param;
    } else {
        PX4_ERR("Error reading `VL_D_OUT_TARGET` parameter!");
    }
    param_get(param_find("VL_D_IN_TARGET"), &param);
    if(param > 0) {
        _sensors_in_target_order = (uint8_t) param;
    } else {
        PX4_ERR("Error reading `VL_D_IN_TARGET` parameter!");
    }

    int32_t orientation = 0;

    param_get(param_find("VL_D_1_ORIENT"), &orientation); _sensors_rotation[0] = (uint8_t)orientation;
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 1
    param_get(param_find("VL_D_2_ORIENT"), &orientation); _sensors_rotation[1] = (uint8_t)orientation;
#endif
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 2
    param_get(param_find("VL_D_3_ORIENT"), &orientation); _sensors_rotation[2] = (uint8_t)orientation;
#endif
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 3
    param_get(param_find("VL_D_4_ORIENT"), &orientation); _sensors_rotation[3] = (uint8_t)orientation;
 #endif
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 4
    param_get(param_find("VL_D_5_ORIENT"), &orientation); _sensors_rotation[4] = (uint8_t)orientation;
#endif
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 5
    param_get(param_find("VL_D_6_ORIENT"), &orientation); _sensors_rotation[5] = (uint8_t)orientation;
#endif
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 6
    param_get(param_find("VL_D_7_ORIENT"), &orientation); _sensors_rotation[6] = (uint8_t)orientation;
#endif
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 7
    param_get(param_find("VL_D_8_ORIENT"), &orientation); _sensors_rotation[7] = (uint8_t)orientation;
#endif
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 8
    param_get(param_find("VL_D_9_ORIENT"), &orientation); _sensors_rotation[8] = (uint8_t)orientation;
#endif
#if VL53L8_DISTRO_MAX_SENSOR_COUNT > 9
    param_get(param_find("VL_D_10_ORIENT"), &orientation); _sensors_rotation[9] = (uint8_t)orientation;
#endif

#if VL53L8_DISTRO_L4_MAX_SENSOR_COUNT > 0
    param_get(param_find(" VL_D_L4_9_AXIS"), &orientation); _sensors_rotation[VL53L8_DISTRO_L4_FIRST_INDEX - 1] = (uint8_t)orientation;
#endif
#if VL53L8_DISTRO_L4_MAX_SENSOR_COUNT > 1
    param_get(param_find("VL_D_L4_10_AXIS"), &orientation); _sensors_rotation[VL53L8_DISTRO_L4_FIRST_INDEX] = (uint8_t)orientation;
#endif

#if VL53L8_DISTRO_L4_MAX_SENSOR_COUNT > 0
    float param_f = 0.0f;

    param_get(param_find("VL_D_L4_OFF_X"), &param);
    if(param > 0) _sensor_L4_X_calib_offset_mm = (int16_t) param;
    else PX4_ERR("Error reading `VL_D_L4_OFF_X` parameter!");

    param_get(param_find("VL_D_L4_OFF_Y"), &param);
    if(param > 0) _sensor_L4_Y_calib_offset_mm = (int16_t) param;
    else PX4_ERR("Error reading `VL_D_L4_OFF_Y` parameter!");

    param_get(param_find("VL_D_L4_RNGTIME"), &param);
    if(param > 0) _sensor_L4_Y_range_budget_ms = (uint16_t) param;
    else PX4_ERR("Error reading `VL_D_L4_RNGTIME` parameter!");

    param_get(param_find("VL_TKF_OUT_RATE"), &param);
    if(param > 0) _tkf_output_rate_ms = (uint16_t) param;
    else PX4_ERR("Error reading `VL_TKF_OUT_RATE` parameter!");

    param_get(param_find("VL_TKF_X_CTR_OFF"), &param);
    if(param > 0) _tkf_X_center_offset_mm = (uint16_t) param;
    else PX4_ERR("Error reading `VL_TKF_X_CTR_OFF` parameter!");

    param_get(param_find("VL_TKF_Y_CTR_OFF"), &param);
    if(param > 0) _tkf_Y_center_offset_mm = (uint16_t) param;
    else PX4_ERR("Error reading `VL_TKF_Y_CTR_OFF` parameter!");

    param_get(param_find("VL_TKF_COG_LEVER"), &param_f);
    if(param_f > 0) _tkf_CoG_lever_height_m = param_f;
    else PX4_ERR("Error reading `VL_TKF_COG_LEVER` parameter!");

    param_get(param_find("VL_TKF_Q_NOISE"), &param_f);
    if(param_f > 0) _tkf_process_noise_Q = param_f;
    else PX4_ERR("Error reading `VL_TKF_Q_NOISE` parameter!");

    param_get(param_find("VL_TKF_MIN_SIGMA"), &param_f);
    if(param_f > 0) _tkf_min_sigma_R = param_f;
    else PX4_ERR("Error reading `VL_TKF_MIN_SIGMA` parameter!");

    param_get(param_find("VL_TKF_USE_NGATE"), &param);
    if(param > 0) _tkf_use_NIS_GATE = (bool)param;
    else PX4_ERR("Error reading `VL_TKF_USE_NGATE` parameter!");

    param_get(param_find("VL_TKF_NGATE_THR"), &param_f);
    if(param_f > 0) _tkf_NIS_GATE_treshold = param_f;
    else PX4_ERR("Error reading `VL_TKF_NGATE_THR` parameter!");

    param_get(param_find("VL_TKF_USE_SIGDG"), &param);
    if(param > 0) _tkf_use_signal_degrade = (bool)param;
    else PX4_ERR("Error reading `VL_TKF_USE_SIGDG` parameter!");

    param_get(param_find("VL_TKF_SD_FACTOR"), &param_f);
    if(param_f > 0) _tkf_signal_degrade_factor = param_f;
    else PX4_ERR("Error reading `VL_TKF_SD_FACTOR` parameter!");

#endif

    start();
    return PX4_OK;
}

void VL53L8_Distro::print_info()
{
    perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

int VL53L8_Distro::collect(uint32_t timeout_us)
{
    perf_begin(_sample_perf);

    int bytes_read = 0;

    if ((bytes_read = read_data(timeout_us)) <= (int)sizeof(VL_Range_Data_s<VL53L8_RESOLUTION_4x4>)) {
        PX4_ERR("Failed to read data from sensors");
        perf_count(_comms_errors);
        perf_end(_sample_perf);
        return PX4_ERROR;
    }
    uint8_t data_received = 0;
    uint16_t read_idx = 0;
    bool is_data_valid = false;
    // uint8_t seq_check = 0;
    // bool seq_found = false;
    distance_sensor_matrix_s msg = {};
    msg.timestamp = hrt_absolute_time();

    while((data_received < _sensors_count) && (read_idx < (bytes_read - 1))) {
        is_data_valid = false;
        while (read_idx < (bytes_read - 1)) {
            if(_buffer[read_idx] == UART_PROT_MSG_HEADER_1 && _buffer[read_idx + 1] == UART_PROT_MSG_HEADER_2) {
                is_data_valid = true;
                break; // Found a valid packet header
            } else {
                read_idx++;
            }
        }

        if (!is_data_valid) {
            PX4_ERR("No valid data found in the buffer");
            perf_count(_comms_errors);
            perf_end(_sample_perf);
            return PX4_ERROR;
        }

        const size_t packet_size = (_sensors_out_resolution == VL53L8_RESOLUTION_8x8)
                         ? sizeof(VL_Range_Data_s<VL53L8_RESOLUTION_8x8>)
                         : sizeof(VL_Range_Data_s<VL53L8_RESOLUTION_4x4>);
        if ((read_idx + packet_size) > (uint16_t)bytes_read) {
            PX4_WARN("Incomplete packet in buffer [ %u ]", ((VL_Range_Data_s<VL53L8_RESOLUTION_4x4> *)&_buffer[read_idx])->sensor_id);
            read_idx++;
            break;
        }

        // if(!seq_found) {
        //     seq_check = ((VL_Range_Data_s<VL53L8_RESOLUTION_4x4> *)&_buffer[read_idx])->seq;
        //     seq_found = true;
        // } else if (seq_check != ((VL_Range_Data_s<VL53L8_RESOLUTION_4x4> *)&_buffer[read_idx])->seq) {
        //     PX4_WARN("Desync detected: [ %u - %u ]", ((VL_Range_Data_s<VL53L8_RESOLUTION_4x4> *)&_buffer[read_idx])->seq, seq_check);
        // }

        if(((VL_Range_Data_s<VL53L8_RESOLUTION_4x4> *)&_buffer[read_idx])->resolution != _sensors_out_resolution) {
            PX4_ERR("Wrong resolution: [ %u != %u ]", ((VL_Range_Data_s<VL53L8_RESOLUTION_4x4> *)&_buffer[read_idx])->resolution, _sensors_out_resolution);
            perf_count(_comms_errors);
            read_idx++;
            continue;
        }

        if(((VL_Range_Data_s<VL53L8_RESOLUTION_8x8> *)&_buffer[read_idx])->resolution == VL53L8_RESOLUTION_8x8) {
            auto *data = reinterpret_cast<VL_Range_Data_s<VL53L8_RESOLUTION_8x8> *>(&_buffer[read_idx]);
            if(!parse_and_fill<VL53L8_RESOLUTION_8x8>(data, msg)) {
                read_idx++;
                continue;
            }
            // PX4_INFO("Sensor data: timestamp: %llu, sensor_id: %d, resolution: %d", data->timestamp, data->sensor_id, data->resolution);
            read_idx += packet_size;
        } else {
            auto *data = reinterpret_cast<VL_Range_Data_s<VL53L8_RESOLUTION_4x4> *>(&_buffer[read_idx]);
            if(!parse_and_fill<VL53L8_RESOLUTION_4x4>(data, msg)) {
                read_idx++;
                continue;
            }
            // PX4_INFO("Sensor data: timestamp: %llu, sensor_id: %d, resolution: %d", data->timestamp, data->sensor_id, data->resolution);
            read_idx += packet_size;
        }

        data_received++;
        is_data_valid = false; // Reset for the next sensor data
    }

    perf_end(_sample_perf);

    return (data_received == _sensors_count) ? PX4_OK : PX4_ERROR;
}

template <size_t M>
bool VL53L8_Distro::parse_and_fill(VL_Range_Data_s<M> *data, distance_sensor_matrix_s &msg) {
    if (data->crc != data->calculate_crc(false)) {
		PX4_ERR("CRC mismatch: received: %04X, expected: %04X", data->crc, data->calculate_crc(false));
        perf_count(_comms_errors);
		return false;
	}

	const uint8_t sid = data->sensor_id;
	if (sid == 0 || sid > VL53L8_DISTRO_MAX_SENSOR_COUNT) {
		PX4_ERR("Invalid sensor_id: %u", sid);
        perf_count(_comms_errors);
		return false;
	}

	msg.timestamp_sample = data->timestamp;
	msg.device_id = _sensors_device_id[sid - 1];
	msg.seq = data->seq;
	msg.resolution = data->resolution;
	msg.temperature = data->silicon_temp;
	msg.orientation = _sensors_rotation[sid - 1];

    memcpy(msg.distance_raw, data->distance, msg.resolution * sizeof(int16_t));
    memcpy(msg.variance_raw, data->range_sigma, msg.resolution * sizeof(uint16_t));
    memcpy(msg.signal, data->signal, msg.resolution * sizeof(uint8_t));
    memcpy(msg.ambient, data->ambient, msg.resolution * sizeof(uint8_t));
    memcpy(msg.status, data->status, msg.resolution * sizeof(uint8_t));

	// for (size_t i = 0; i < data->resolution; i++) {
	// 	msg.current_distance[i] = ((float)data->distance[i]) / 4000.0f;
	// 	msg.variance[i]         = ((float)data->range_sigma[i]) / 128000.0f;
	// 	msg.signal[i]           = data->signal[i];
	// 	msg.ambient[i]          = data->ambient[i];
	// 	msg.status[i]           = data->status[i];
	// }

    if(!_distance_sensor_pub[sid - 1].publish(msg)) {
        PX4_WARN("Failed to publish [ %u ]", sid);
    };

	return true;
}

bool VL53L8_Distro::parse_and_fill_L4(VL_L4_Range_Data_s *data, distance_sensor_single_s &msg) {
    if (data->crc != data->calculate_crc(false)) {
        PX4_ERR("CRC mismatch: received: %04X, expected: %04X", data->crc, data->calculate_crc(false));
        perf_count(_comms_errors);
        return false;
    }

    const uint8_t sid = data->sensor_id - VL53L8_DISTRO_L4_FIRST_INDEX;

    if (sid >= VL53L8_DISTRO_L4_MAX_SENSOR_COUNT) {
        PX4_ERR("Invalid sensor_id: %u", data->sensor_id);
        perf_count(_comms_errors);
        return false;
    }

    msg.timestamp_sample = data->timestamp;
    msg.device_id = _sensors_device_id[data->sensor_id - 1];
    msg.seq = data->seq;
    msg.distance_mm = data->distance_mm;
    msg.sigma_mm = data->sigma_mm;
    msg.ambient_rate_kcps = data->ambient_rate_kcps;
    msg.ambient_per_spad_kcps = data->ambient_per_spad_kcps;
    msg.signal_rate_kcps = data->signal_rate_kcps;
    msg.signal_per_spad_kcps = data->signal_per_spad_kcps;
    msg.number_of_spad = data->number_of_spad;
    msg.range_status = data->range_status;
    msg.axis = _sensors_rotation[data->sensor_id - 1];

    if(!_distance_sensor_L4_pub[sid].publish(msg)) {
        PX4_WARN("Failed to publish L4 data");
    } else {
        // PX4_INFO("Published L4 data from sensor ID %d", data->sensor_id);
    };

    return true;
}

bool VL53L8_Distro::parse_and_fill_visual_odometry(Visual_Odometry_Data2_s *data) {
    if (data->crc != data->calculate_crc(false)) {
        PX4_ERR("CRC mismatch: received: %04X, expected: %04X", data->crc, data->calculate_crc(false));
        perf_count(_comms_errors);
        return false;
    }

    optical_navigation_horizontal_s msg = {};
    msg.timestamp           = data->timestamp;
    msg.x_m                 = data->x_m;
    msg.y_m                 = data->y_m;
    msg.vx_m_s              = data->vx_mps;
    msg.vy_m_s              = data->vy_mps;
    // msg.roll_rad            = data->roll_rad;
    // msg.pitch_rad           = data->pitch_rad;
    msg.var_x_m2            = data->var_x_m2;
    msg.var_y_m2            = data->var_y_m2;
    msg.var_vx_m2s2         = data->var_vx_m2s2;
    msg.var_vy_m2s2         = data->var_vy_m2s2;
    // msg.var_roll_rad2       = data->var_roll_rad2;
    // msg.var_pitch_rad2      = data->var_pitch_rad2;
    // msg.rho_m               = data->rho_m;
    msg.calculation_time_us = data->calculation_time_us;
    // msg.inliers_total       = data->inliers_total;
    // msg.n1                  = data->n1;
    // msg.n2                  = data->n2;
    // msg.ok                  = data->ok;
    // msg.ok_prior            = data->ok_prior;
    msg.rejected_x          = data->rejected_x;
    msg.rejected_y          = data->rejected_y;
    msg.frame               = data->frame;
    _optical_navigation_pub.publish(msg);

    return true;
}

void VL53L8_Distro::Run()
{
	perf_begin(_sample_perf);

	if (_task_should_exit) {
		PX4_INFO("VL53L8_Distro task exit requested");
		perf_cancel(_sample_perf);
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	// Ensure the serial port is open.
	if (open_serial_port() != PX4_OK) {
		PX4_ERR("Failed to open serial port");
		stop();
		ScheduleDelayed(500_ms);
		perf_end(_sample_perf);
		return;
	}

	// One-time init sequence (can block briefly, before periodic TX starts)
	if (!_is_initialized) {

		if (initialize_sensor() != PX4_OK) {
			PX4_ERR("Failed to initialize VL53L8_Distro sensor");
			stop();
			perf_end(_sample_perf);
			return;
		}

		if (get_sensors_resolution() != PX4_OK) {
			PX4_ERR("Failed to get sensors resolution");
			stop();
			perf_end(_sample_perf);
			return;
		}

        // Needed for VL53L8
		// Perform a single measurement to verify data path
		// if (measure(UART_PROT_CMD_RNG_SINGLE) != PX4_OK) {
		// 	PX4_ERR("Failed to perform initial measurement");
		// 	stop();
		// 	perf_end(_sample_perf);
		// 	return;
		// }

		// // Collect at least something (ok to wait here)
		// collect_streaming(1_s);

		// // Start streaming
		// if (measure(UART_PROT_CMD_RNG_START) != PX4_OK) {
		// 	PX4_ERR("Failed to start ranging");
		// 	stop();
		// 	perf_end(_sample_perf);
		// 	return;
		// }

		// Initialize periodic deadlines
		_next_attitude = now + ATT_PERIOD_US;
		_next_timesync = now + TS_PERIOD_US;

		ScheduleNow();
		perf_end(_sample_perf);
		return;
	}

	// --- RX: non-blocking, time-budgeted ---
	const hrt_abstime rx_start = now;

	// pump once before parsing
	pump_uart_to_ring(100);

	PacketType type{};
	while ((hrt_absolute_time() - rx_start) < RX_BUDGET_US) {
		if (!try_read_packet(type)) {
			break;
		}
		handle_packet(type);
	}

	// --- TX: attitude 50Hz ---
	if ((int64_t)(now - _next_attitude) >= 0) {
		send_attitude();
		advance_deadline(_next_attitude, ATT_PERIOD_US, now);
	}

	// --- TX: timesync 1Hz (message unchanged) ---
	if ((int64_t)(now - _next_timesync) >= 0) {
		send_timesync();
		advance_deadline(_next_timesync, TS_PERIOD_US, now);
	}

	// --- Schedule next wakeup (nearest deadline or RX tick) ---
	hrt_abstime next = _next_attitude;
	if (_next_timesync < next) {
		next = _next_timesync;
	}

	const hrt_abstime rx_wakeup = now + RX_TICK_US;
	if (rx_wakeup < next) {
		next = rx_wakeup;
	}

	ScheduleAt(next);

	perf_end(_sample_perf);
}

void VL53L8_Distro::start()
{
	PX4_INFO("Starting VL53L8_Distro thread");
	const hrt_abstime now = hrt_absolute_time();
	_next_attitude = now + ATT_PERIOD_US;
	_next_timesync = now + TS_PERIOD_US;
	ScheduleNow();
}

void VL53L8_Distro::stop()
{
    PX4_INFO("Stopping VL53L8_Distro measurements");
    _task_should_exit = true;



    // Ensure the serial port is closed.
	_uart.close();
    // Clear the work queue schedule.
	ScheduleClear();
}

int VL53L8_Distro::initialize_sensor() {
    if(!_is_initialized) {
        PX4_INFO("VL53L8_Distro not initialized, starting initialization");

        uint8_t cmd_value = 0;
        uint8_t retry = 0;
        uint8_t max_retries = 5;

        while (retry < max_retries)
        {
            //Send the "Is Alive" command and wait for ACK
            cmd_value = 0;
            if(send_command(UART_PROT_CMD_IS_ALIVE, cmd_value, true, 10_ms) == PX4_OK) {
                break; // Exit the loop if the sensor is alive
            };

            PX4_ERR("Sensor not responding, retrying... [%d]", retry + 1);

            retry++;

            if(retry >= max_retries) {
                PX4_ERR("Max retries reached, sensor not responding");
                return PX4_ERROR;
            }

            px4_sleep(1); // Wait for a second before retrying
        }

        send_timesync();

        cmd_value = 0x00; // Dummy value
        if(send_command(UART_PROT_CMD_SENSOR_RESET, cmd_value, true, 1_s) != PX4_OK) {
            PX4_ERR("Failed to stop any ongoing measurement");
            return PX4_ERROR;
        } else {
            PX4_INFO("Sensors reset done");
        }

        // Set OUT sensors
        cmd_value = _sensors_out_resolution;
        if(send_command(UART_PROT_CMD_OUT_SENSOR_RES, cmd_value, true, 1_s) != PX4_OK) {
            PX4_ERR("Failed to set OUT resolution");
            return PX4_ERROR;
        } else if(cmd_value != _sensors_out_resolution) {
            PX4_ERR("Failed to set ranging resolution to %d [%d] for OUT", _sensors_out_resolution, cmd_value);
            return PX4_ERROR;
        } else {
            PX4_INFO("Ranging resolution set to %d for OUT", _sensors_out_resolution);
        }

        cmd_value = _sensors_out_frequency;
        if(send_command(UART_PROT_CMD_OUT_FREQUENCY, cmd_value, true, 1_s) != PX4_OK) {
            PX4_ERR("Failed to set OUT frequency");
            return PX4_ERROR;
        } else if(cmd_value != _sensors_out_frequency) {
            PX4_ERR("Failed to set ranging frequency to %d Hz [%d] for OUT", _sensors_out_frequency, cmd_value);
            return PX4_ERROR;
        } else {
            PX4_INFO("Ranging frequency set to %d Hz for OUT", _sensors_out_frequency);
        }

        cmd_value = _sensors_out_target_order;
        if(send_command(UART_PROT_CMD_OUT_TARGET_ORD, cmd_value, true, 1_s) != PX4_OK) {
            PX4_ERR("Failed to set OUT target order");
            return PX4_ERROR;
        } else if(cmd_value != _sensors_out_target_order) {
            PX4_ERR("Failed to set target order to %d [%d] for OUT", _sensors_out_target_order, cmd_value);
            return PX4_ERROR;
        } else {
            PX4_INFO("Target order set to %s for OUT", (_sensors_out_target_order == UART_PROT_TARGET_ORDER_STRONGEST) ? "STRONGEST" : "CLOSEST");
        }

        // Set IN sensors
        cmd_value = _sensors_in_resolution;
        if(send_command(UART_PROT_CMD_IN_SENSOR_RES, cmd_value, true, 1_s) != PX4_OK) {
            PX4_ERR("Failed to set IN resolution");
            return PX4_ERROR;
        } else if(cmd_value != _sensors_in_resolution) {
            PX4_ERR("Failed to set ranging resolution to %d [%d] for IN", _sensors_in_resolution, cmd_value);
            return PX4_ERROR;
        } else {
            PX4_INFO("Ranging resolution set to %d for IN", _sensors_in_resolution);
        }

        cmd_value = _sensors_in_frequency;
        if(send_command(UART_PROT_CMD_IN_FREQUENCY, cmd_value, true, 1_s) != PX4_OK) {
            PX4_ERR("Failed to set IN frequency");
            return PX4_ERROR;
        } else if(cmd_value != _sensors_in_frequency) {
            PX4_ERR("Failed to set ranging frequency to %d Hz [%d] for IN", _sensors_in_frequency, cmd_value);
            return PX4_ERROR;
        } else {
            PX4_INFO("Ranging frequency set to %d Hz for IN", _sensors_in_frequency);
        }

        cmd_value = _sensors_in_target_order;
        if(send_command(UART_PROT_CMD_IN_TARGET_ORD, cmd_value, true, 1_s) != PX4_OK) {
            PX4_ERR("Failed to set IN target order");
            return PX4_ERROR;
        } else if(cmd_value != _sensors_in_target_order) {
            PX4_ERR("Failed to set target order to %d [%d] for IN", _sensors_in_target_order, cmd_value);
            return PX4_ERROR;
        } else {
            PX4_INFO("Target order set to %s for IN", (_sensors_in_target_order == UART_PROT_TARGET_ORDER_STRONGEST) ? "STRONGEST" : "CLOSEST");
        }

        // TKF parameters for L4 sensor(s)
#if VL53L8_DISTRO_L4_MAX_SENSOR_COUNT > 0
        CMD_multi_s msg_multi{};

        msg_multi.cmd = UART_PROT_CMD_L4_X_CALIB_OFFSET;
        msg_multi.value_i = _sensor_L4_X_calib_offset_mm;
        msg_multi.calculate_crc(true);
        int ret = _uart.write((const void *)&msg_multi, sizeof(msg_multi));
        if (ret <= 0) { PX4_ERR("write failed: %d (%s)", errno, strerror(errno)); perf_count(_comms_errors); return PX4_ERROR; }

        px4_udelay(50);

        msg_multi.cmd = UART_PROT_CMD_L4_Y_CALIB_OFFSET;
        msg_multi.value_i = _sensor_L4_Y_calib_offset_mm;
        msg_multi.calculate_crc(true);
        ret = _uart.write((const void *)&msg_multi, sizeof(msg_multi));
        if (ret <= 0) { PX4_ERR("write failed: %d (%s)", errno, strerror(errno)); perf_count(_comms_errors); return PX4_ERROR; }

        px4_udelay(50);

        msg_multi.cmd = UART_PROT_CMD_L4_RNG_TIME_BUDGET;
        msg_multi.value_u = _sensor_L4_Y_range_budget_ms;
        msg_multi.calculate_crc(true);
        ret = _uart.write((const void *)&msg_multi, sizeof(msg_multi));
        if (ret <= 0) { PX4_ERR("write failed: %d (%s)", errno, strerror(errno)); perf_count(_comms_errors); return PX4_ERROR; }

        px4_udelay(50);

        msg_multi.cmd = UART_PROT_CMD_TKF_OUTPUT_RATE;
        msg_multi.value_u = _tkf_output_rate_ms;
        msg_multi.calculate_crc(true);
        ret = _uart.write((const void *)&msg_multi, sizeof(msg_multi));
        if (ret <= 0) { PX4_ERR("write failed: %d (%s)", errno, strerror(errno)); perf_count(_comms_errors); return PX4_ERROR; }

        px4_udelay(50);

        msg_multi.cmd = UART_PROT_CMD_TKF_X_CENTER_OFFSET;
        msg_multi.value_u = _tkf_X_center_offset_mm;
        msg_multi.calculate_crc(true);
        ret = _uart.write((const void *)&msg_multi, sizeof(msg_multi));
        if (ret <= 0) { PX4_ERR("write failed: %d (%s)", errno, strerror(errno)); perf_count(_comms_errors); return PX4_ERROR; }

        px4_udelay(50);

        msg_multi.cmd = UART_PROT_CMD_TKF_Y_CENTER_OFFSET;
        msg_multi.value_u = _tkf_Y_center_offset_mm;
        msg_multi.calculate_crc(true);
        ret = _uart.write((const void *)&msg_multi, sizeof(msg_multi));
        if (ret <= 0) { PX4_ERR("write failed: %d (%s)", errno, strerror(errno)); perf_count(_comms_errors); return PX4_ERROR; }

        px4_udelay(50);

        msg_multi.cmd = UART_PROT_CMD_TKF_COG_LEVER_OFFSET;
        msg_multi.value_f = _tkf_CoG_lever_height_m;
        msg_multi.calculate_crc(true);
        ret = _uart.write((const void *)&msg_multi, sizeof(msg_multi));
        if (ret <= 0) { PX4_ERR("write failed: %d (%s)", errno, strerror(errno)); perf_count(_comms_errors); return PX4_ERROR; }

        px4_udelay(50);

        msg_multi.cmd = UART_PROT_CMD_TKF_PROCESS_NOISE_Q;
        msg_multi.value_f = _tkf_process_noise_Q;
        msg_multi.calculate_crc(true);
        ret = _uart.write((const void *)&msg_multi, sizeof(msg_multi));
        if (ret <= 0) { PX4_ERR("write failed: %d (%s)", errno, strerror(errno)); perf_count(_comms_errors); return PX4_ERROR; }

        px4_udelay(50);

        msg_multi.cmd = UART_PROT_CMD_TKF_MIN_SIGMA;
        msg_multi.value_f = _tkf_min_sigma_R;
        msg_multi.calculate_crc(true);
        ret = _uart.write((const void *)&msg_multi, sizeof(msg_multi));
        if (ret <= 0) { PX4_ERR("write failed: %d (%s)", errno, strerror(errno)); perf_count(_comms_errors); return PX4_ERROR; }

        px4_udelay(50);

        msg_multi.cmd = UART_PROT_CMD_TKF_USE_NIS_GATE;
        msg_multi.value_u = _tkf_use_NIS_GATE;
        msg_multi.calculate_crc(true);
        ret = _uart.write((const void *)&msg_multi, sizeof(msg_multi));
        if (ret <= 0) { PX4_ERR("write failed: %d (%s)", errno, strerror(errno)); perf_count(_comms_errors); return PX4_ERROR; }

        px4_udelay(50);

        msg_multi.cmd = UART_PROT_CMD_TKF_NIS_GATE_THRESHOLD;
        msg_multi.value_f = _tkf_NIS_GATE_treshold;
        msg_multi.calculate_crc(true);
        ret = _uart.write((const void *)&msg_multi, sizeof(msg_multi));
        if (ret <= 0) { PX4_ERR("write failed: %d (%s)", errno, strerror(errno)); perf_count(_comms_errors); return PX4_ERROR; }

        px4_udelay(50);

        msg_multi.cmd = UART_PROT_CMD_TKF_USE_SIGNAL_DEGRADE;
        msg_multi.value_u = _tkf_use_signal_degrade;
        msg_multi.calculate_crc(true);
        ret = _uart.write((const void *)&msg_multi, sizeof(msg_multi));
        if (ret <= 0) { PX4_ERR("write failed: %d (%s)", errno, strerror(errno)); perf_count(_comms_errors); return PX4_ERROR; }

        px4_udelay(50);

        msg_multi.cmd = UART_PROT_CMD_TKF_SIGNAL_DEGRADE_FACTOR;
        msg_multi.value_f = _tkf_signal_degrade_factor;
        msg_multi.calculate_crc(true);
        ret = _uart.write((const void *)&msg_multi, sizeof(msg_multi));
        if (ret <= 0) { PX4_ERR("write failed: %d (%s)", errno, strerror(errno)); perf_count(_comms_errors); return PX4_ERROR; }

        px4_udelay(50);

        PX4_INFO("TKF parameters for L4 sensor(s) set successfully");
#endif


        // Send the sensor initialization command
        uint32_t cmd_value32 = 0x00; // Use resolution set before for sensor initialization
        if(send_command(UART_PROT_CMD_SENSOR_INIT, cmd_value32, true, 20_s) != PX4_OK) {
            PX4_ERR("Failed to initialize sensors");
            return PX4_ERROR;
        }

        if(cmd_value32 == 0 || __builtin_popcount(cmd_value32) > (VL53L8_DISTRO_MAX_SENSOR_COUNT + VL53L8_DISTRO_L4_MAX_SENSOR_COUNT)) {
            PX4_ERR("Invalid number of sensors detected: %d", __builtin_popcount(cmd_value32));
            return PX4_ERROR;
        }

        _sensors_active_mask = cmd_value32;

        _sensors_count = __builtin_popcount(_sensors_active_mask);
        PX4_INFO("VL53L8_Distro initialized successfully on port: %s [sensors: %d]", _port, _sensors_count);
        this->_is_initialized = true;
    } else {
        PX4_INFO("VL53L8_Distro already initialized on port: %s", _port);
    }

    return PX4_OK;
}

int VL53L8_Distro::get_sensors_resolution() {
    if(!_is_initialized) {
        PX4_ERR("VL53L8_Distro not initialized, cannot get sensors resolution");
        return PX4_ERROR;
    }

    uint8_t cmd_value = 0;

    if(send_command(UART_PROT_CMD_OUT_SENSOR_RES, cmd_value, true, 1_s) != PX4_OK) {
        PX4_ERR("Failed to get OUT sensors resolution");
        return PX4_ERROR;
    }

    _sensors_out_resolution = cmd_value;
    PX4_INFO("Sensors resolution: %d", _sensors_out_resolution);

    return PX4_OK;
}

int VL53L8_Distro::measure(uint8_t command, bool ack) {
    if(!_is_initialized) {
        PX4_ERR("VL53L8_Distro not initialized, cannot perform measurement");
        return PX4_ERROR;
    }

    if(command != UART_PROT_CMD_RNG_START &&
       command != UART_PROT_CMD_RNG_STOP &&
       command != UART_PROT_CMD_RNG_SINGLE) {
        PX4_ERR("Invalid command for measurement: 0x%02X", command);
        return PX4_ERROR;
    }

    if(!_ranging_in_progress && command == UART_PROT_CMD_RNG_STOP) {
        PX4_ERR("Ranging already stopped");
        return PX4_ERROR;
    }

    uint8_t cmd_value = 0;

    if(send_command(command, cmd_value, ack, 100_ms) != PX4_OK) {
        PX4_ERR("Failed to send measurement command: 0x%02X", command);
        return PX4_ERROR;
    }

    if(command == UART_PROT_CMD_RNG_START) {
        _ranging_in_progress = true;
        PX4_INFO("Ranging started successfully");
    } else if(command == UART_PROT_CMD_RNG_STOP) {
        _ranging_in_progress = false;
        PX4_INFO("Ranging stopped successfully");
    } else if(command == UART_PROT_CMD_RNG_SINGLE) {
        _ranging_in_progress = false; // Single measurement does not keep ranging active
        PX4_INFO("Single measurement commanded successfully");
    }

    return PX4_OK;
}

int VL53L8_Distro::send_timesync() {
    CMD_long_s msg_long{};
    msg_long.cmd = UART_PROT_CMD_TIMESYNC;
    msg_long.value = hrt_absolute_time() + 1_ms; // Use current time for synchronization
    msg_long.calculate_crc(true);
    _last_sync_time = msg_long.value;
    return _uart.write((const void *)&msg_long, sizeof(msg_long));
}

int VL53L8_Distro::open_serial_port(speed_t speed) {
    if(_uart.isOpen()) {
        // PX4_INFO("Serial port %s already open", _port);
        return PX4_OK;
    }

    // Open the serial port
    if(!_uart.setPort(_port)) {
        PX4_ERR("Error configuring serial device on port %s", _port);
        return PX4_ERROR;
    }

    // Configure the desired baudrate if one was specified by the user.
    if(speed == 0) {
        PX4_INFO("Using default baudrate for %s [%lu]", _port, _port_baudrate);
        speed = _port_baudrate; // Use the default baudrate if not specified
    } else {
        PX4_INFO("Using baudrate for %s [%lu]", _port, _port_baudrate);
        _port_baudrate = speed; // Update the baudrate to the specified value
    }

    if(_port_baudrate > 0) {
        if (!_uart.setBaudrate(_port_baudrate)) {
            PX4_ERR("Error setting baudrate to %lu on %s", _port_baudrate, _port);
            return PX4_ERROR;
        }
    } else {
        PX4_ERR("Invalid baudrate specified for %s [%lu]", _port, _port_baudrate);
        return PX4_ERROR;
    }

    if(!_uart.open()) {
        PX4_ERR("Error opening serial device %s", _port);
        return PX4_ERROR;
    }

    PX4_INFO("Serial port %s opened successfully with baudrate %lu", _port, _port_baudrate);

	return PX4_OK;
}

int VL53L8_Distro::send_command(uint8_t cmd, uint8_t &value, bool ack, uint32_t timeout_us) {
    CMD_short_s msg{};
    msg.cmd = cmd;
    msg.value = value; // Set the command value
    msg.calculate_crc(true);

    int ret = _uart.write((const void *)&msg, sizeof(msg));
    // PX4_INFO("Wrote %d bytes to port %s for command 0x%02X", ret, _port, cmd);
    if (ret <= 0) {
        PX4_ERR("write failed: %d (%s)", errno, strerror(errno));
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    ret = read_ACK(value, timeout_us);
    if(ack) {
        // Wait for the ACK response
        if(ret != PX4_OK) {
            PX4_ERR("Failed to read ACK response for command 0x%02X", cmd);
            perf_count(_comms_errors);
            return PX4_ERROR;
        }
        // else if(value != msg.value) {
        //     PX4_ERR("ACK response value mismatch for command 0x%02X: sent %d, received %d", cmd, msg.value, value);
        //     perf_count(_comms_errors);
        //     return PX4_ERROR;
        // }
    }

    return PX4_OK;
}

int VL53L8_Distro::send_command(uint8_t cmd, uint32_t &value, bool ack, uint32_t timeout_us) {
    CMD_multi_s msg{};
    msg.cmd = cmd;
    msg.value_u = value; // Set the command value
    msg.calculate_crc(true);

    int ret = _uart.write((const void *)&msg, sizeof(msg));
    // PX4_INFO("Wrote %d bytes to port %s for command 0x%02X", ret, _port, cmd);
    if (ret <= 0) {
        PX4_ERR("write failed: %d (%s)", errno, strerror(errno));
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    ret = read_ACK(value, timeout_us);
    if(ack) {
        // Wait for the ACK response
        if(ret != PX4_OK) {
            PX4_ERR("Failed to read ACK response for command 0x%02X", cmd);
            perf_count(_comms_errors);
            return PX4_ERROR;
        }
        // else if(value != msg.value) {
        //     PX4_ERR("ACK response value mismatch for command 0x%02X: sent %d, received %d", cmd, msg.value, value);
        //     perf_count(_comms_errors);
        //     return PX4_ERROR;
        // }
    }

    return PX4_OK;
}

int VL53L8_Distro::read_ACK(uint8_t &value, uint32_t timeout_us) {
    PacketType packet_type;
    if (read_packet(packet_type, timeout_us) < 0) {
        PX4_ERR("Failed to read ACK response: %d (%s)", errno, strerror(errno));
        perf_count(_comms_errors);
        return PX4_ERROR;
    } else if (packet_type != PacketType::CMD_Short && (((CMD_short_s *)&_buffer[0])->cmd != UART_PROT_CMD_STATUS_ACK)) {
        PX4_ERR("No ACK response received");
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    value = ((CMD_short_s *)&_buffer[0])->value; // Update the value with the response


    // ssize_t bytes_read = _uart.readAtLeast((uint8_t *)&msg, sizeof(CMD_short_s), sizeof(CMD_short_s), timeout_us);

    // if(bytes_read < 0) {
    //     PX4_ERR("Failed to read ACK from UART: %d (%s)", errno, strerror(errno));
    //     perf_count(_comms_errors);
    //     return PX4_ERROR;
    // } else if (bytes_read == 0) {
    //     PX4_ERR("No ACK read from UART within timeout");
    //     perf_count(_comms_errors);
    //     return PX4_ERROR;
    // } else if (bytes_read != sizeof(CMD_short_s)) {
    //     PX4_ERR("Read %zd bytes, expected %zu bytes for ACK", bytes_read, sizeof(CMD_short_s));
    //     perf_count(_comms_errors);
    //     return PX4_ERROR;
    // }

    // if(msg.crc != msg.calculate_crc(false)) {
    //     PX4_ERR("CRC mismatch for ACK: received: %04X, expected: %04X", msg.crc, msg.calculate_crc(false));
    //     perf_count(_comms_errors);
    //     return PX4_ERROR;
    // }

    return PX4_OK; // Successfully read ACK
}

int VL53L8_Distro::read_ACK(uint32_t &value, uint32_t timeout_us) {
    PacketType packet_type;
    if (read_packet(packet_type, timeout_us) < 0) {
        PX4_ERR("Failed to read ACK response: %d (%s)", errno, strerror(errno));
        perf_count(_comms_errors);
        return PX4_ERROR;
    } else if (packet_type != PacketType::CMD_Multi && (((CMD_multi_s *)&_buffer[0])->cmd != UART_PROT_CMD_STATUS_ACK)) {
        PX4_ERR("No ACK response received");
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    value = ((CMD_multi_s *)&_buffer[0])->value_u; // Update the value with the response
    return PX4_OK; // Successfully read ACK
}

int VL53L8_Distro::read_data(uint32_t timeout_us) {
    uint16_t read_size = ((_sensors_out_resolution == VL53L8_RESOLUTION_8x8) ? sizeof(VL_Range_Data_s<VL53L8_RESOLUTION_8x8>) : sizeof(VL_Range_Data_s<VL53L8_RESOLUTION_4x4>)) * _sensors_count;
    ssize_t bytes_read = _uart.readAtLeast(_buffer, read_size, read_size, timeout_us);
    if (bytes_read < 0) {
        PX4_ERR("Failed to read data from UART: %d (%s)", errno, strerror(errno));
        perf_count(_comms_errors);
        return PX4_ERROR;
    } else if (bytes_read == 0) {
        PX4_ERR("No data read from UART within timeout");
        perf_count(_comms_errors);
        // return PX4_ERROR;
    } else if (bytes_read != read_size) {
        PX4_ERR("Read %zd bytes, expected %d bytes", bytes_read, read_size);
        perf_count(_comms_errors);
        // return PX4_ERROR;
    }

    // PX4_INFO("Data read: [ %d / %u ]", bytes_read, read_size);

    return (int)bytes_read; // Successfully read data
}

size_t VL53L8_Distro::pump_uart_to_ring(uint32_t slice_timeout_us)
{
    // Czytamy małą porcję bez blokowania na długo (tu 512 B)
    uint8_t tmp[512];
    ssize_t n = _uart.readAtLeast(tmp, sizeof(tmp), 1, slice_timeout_us);
    if (n > 0) {
        _rx.write(tmp, static_cast<size_t>(n), /*allow_overflow=*/true);
        return static_cast<size_t>(n);
    }
    return 0;
}


bool VL53L8_Distro::try_read_packet(PacketType &packet_type)
{
	// Non-blocking: assume _rx already contains whatever is available
	auto crc_over_ring = [&](size_t start_off, size_t nbytes) -> uint16_t {
		uint16_t crc = 0xFFFF;
		uint8_t chunk[64];
		size_t done = 0;
		while (done < nbytes) {
			size_t take = (nbytes - done > sizeof(chunk)) ? sizeof(chunk) : (nbytes - done);
			size_t got = _rx.peek(start_off + done, chunk, take);
			for (size_t i = 0; i < got; ++i) {
				uint8_t tbl_idx = (uint8_t)((crc >> 8) ^ chunk[i]);
				crc = (uint16_t)((crc << 8) ^ CRC16_CCITT_TABLE[tbl_idx]);
			}
			done += got;
		}
		return crc;
	};

	// 1) find header 0xD3 0xAC
	if (_rx.size() < UART_PROT_MSG_HEADER_LEN) {
		return false;
	}

	size_t offset = 0;
	if (!_rx.find2(UART_PROT_MSG_HEADER_1, UART_PROT_MSG_HEADER_2, offset)) {
		// keep 1 byte, drop the rest to avoid unbounded growth
		if (_rx.size() > 1) {
			_rx.drop(_rx.size() - 1);
		}
		return false;
	}

	if (offset) {
		_rx.drop(offset);
	}

	if (_rx.size() < UART_PROT_MSG_HEADER_LEN) {
		return false;
	}

	// 2) read packet_len
	uint16_t packet_len_le = 0;
	_rx.peek(2, reinterpret_cast<uint8_t *>(&packet_len_le), 2);
	const uint16_t packet_len = packet_len_le;

	if (packet_len < UART_PROT_PAYLOAD_MIN_SIZE || packet_len > UART_PROT_PAYLOAD_MAX_SIZE) {
		_rx.drop(1);
		return true; // consumed a byte, allow caller to continue
	}

	const uint16_t frame_len = UART_PROT_MSG_HEADER_LEN + packet_len;
	if (_rx.size() < frame_len) {
		return false; // not enough yet
	}

	// 3) CRC check over [packet_len..payload] excluding CRC
	const size_t crc_region_off = 2;
	const size_t crc_region_len = packet_len - UART_PROT_MSG_CRC_LEN;

	uint16_t received_crc_le = 0;
	_rx.peek(UART_PROT_MSG_HEADER_LEN + packet_len - UART_PROT_MSG_CRC_LEN,
		 reinterpret_cast<uint8_t *>(&received_crc_le), 2);

	const uint16_t expected_crc = crc_over_ring(crc_region_off, crc_region_len);

	if (received_crc_le != expected_crc) {
		perf_count(_comms_errors);
		_rx.drop(1);
		return true;
	}

	// 4) success: copy full frame out and drop from ring
	_rx.read(_buffer, frame_len);

	switch (frame_len) {
	case sizeof(CMD_short_s):
		packet_type = PacketType::CMD_Short;
		break;
	case sizeof(CMD_multi_s):
		packet_type = PacketType::CMD_Multi;
		break;
	case sizeof(CMD_long_s):
		packet_type = PacketType::CMD_Long;
		break;
	case sizeof(Visual_Odometry_Data2_s):
		packet_type = PacketType::MSG_VisualOdometry2;
		break;
	case sizeof(VL_Range_Data_s<VL53L8_RESOLUTION_4x4>):
		packet_type = PacketType::MSG_RangeData_16;
		break;
	case sizeof(VL_Range_Data_s<VL53L8_RESOLUTION_8x8>):
		packet_type = PacketType::MSG_RangeData_64;
		break;
	case sizeof(VL_L4_Range_Data_s):
		packet_type = PacketType::MSG_RangeData_L4;
		break;
	default:
		packet_type = PacketType::INVALID;
		PX4_ERR("Invalid packet size: %u", frame_len);
		return true;
	}

	return true;
}

void VL53L8_Distro::handle_packet(PacketType type)
{
    // PX4_INFO("Handling packet of type: %d", static_cast<int>(type));
	switch (type) {
	case PacketType::MSG_RangeData_64: {
		auto *d = reinterpret_cast<VL_Range_Data_s<VL53L8_RESOLUTION_8x8> *>(&_buffer[0]);
		_sensors_active_mask |= (1u << (d->sensor_id - 1));
		distance_sensor_matrix_s msg{};
		msg.timestamp = hrt_absolute_time();
		parse_and_fill<VL53L8_RESOLUTION_8x8>(d, msg);
        // PX4_INFO("Received 64-point range data from sensor ID %d", d->sensor_id);
	} break;

	case PacketType::MSG_RangeData_16: {
		auto *d = reinterpret_cast<VL_Range_Data_s<VL53L8_RESOLUTION_4x4> *>(&_buffer[0]);
		_sensors_active_mask |= (1u << (d->sensor_id - 1));
		distance_sensor_matrix_s msg{};
		msg.timestamp = hrt_absolute_time();
		parse_and_fill<VL53L8_RESOLUTION_4x4>(d, msg);
        // PX4_INFO("Received 16-point range data from sensor ID %d", d->sensor_id);
	} break;

	case PacketType::MSG_RangeData_L4: {
		auto *d = reinterpret_cast<VL_L4_Range_Data_s *>(&_buffer[0]);
		_sensors_active_mask |= (1u << (d->sensor_id - 1));
		distance_sensor_single_s msg{};
		msg.timestamp = hrt_absolute_time();
		parse_and_fill_L4(d, msg);
        // PX4_INFO("Received L4 range data from sensor ID %d", d->sensor_id);
	} break;

	case PacketType::MSG_VisualOdometry2: {
		auto *odom = reinterpret_cast<Visual_Odometry_Data2_s *>(&_buffer[0]);
		parse_and_fill_visual_odometry(odom);
        // PX4_INFO("Received visual odometry data");
	} break;

	default:
		// CMD_* / INVALID ignored here; command path handles ACKs separately
		break;
	}
}

int VL53L8_Distro::send_attitude()
{
	vehicle_attitude_s att{};
	if (_att_sub.update(&att)) {
		_att_last = att;
		_att_have_last = true;
	}

	if (!_att_have_last) {
		return PX4_ERROR;
	}

	const matrix::Quatf q(_att_last.q);
	const matrix::Eulerf e(q);

	Vehicle_Attitude_s pkt{};
	pkt.timestamp = hrt_absolute_time();
	pkt.timestamp_sample = _att_last.timestamp_sample;
	pkt.roll_rad = e.phi();
	pkt.pitch_rad = e.theta();
	pkt.yaw_rad = e.psi();
	pkt.calculate_crc(true);

	return _uart.write((const void *)&pkt, sizeof(Vehicle_Attitude_s));
}

int VL53L8_Distro::read_packet(PacketType &packet_type, uint32_t timeout_us)
{
	const hrt_abstime start_time_us = hrt_absolute_time();

	while (hrt_elapsed_time(&start_time_us) < timeout_us) {

		// pump a small slice (may block up to slice_timeout_us)
		pump_uart_to_ring(1_ms);

		if (try_read_packet(packet_type)) {
			// if it produced a valid packet type we accept it; INVALID means "keep scanning"
			if (packet_type != PacketType::INVALID) {
				return PX4_OK;
			}
		}
	}

	return PX4_ERROR;
}

int VL53L8_Distro::collect_streaming(uint32_t timeout_us)
{
	// Legacy helper: allows waiting for at least one valid packet (used during init).
	const hrt_abstime start_time_us = hrt_absolute_time();

	PacketType type{};
	bool got_any = false;

	while (hrt_elapsed_time(&start_time_us) < timeout_us) {

		// pump and then use the timeout-enabled reader (short slice)
		if (read_packet(type, 5_ms) != PX4_OK) {
			continue;
		}

		handle_packet(type);
		got_any = true;

		// During init we don't need a full sensor set, just proof that the link works.
		break;
	}

	return got_any ? PX4_OK : PX4_ERROR;
}

