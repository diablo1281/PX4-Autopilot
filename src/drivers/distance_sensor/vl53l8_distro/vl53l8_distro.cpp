#include "vl53l8_distro.hpp"

#include <cerrno>

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include <lib/drivers/device/Device.hpp>

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
    for(uint8_t i = 0; i < VL53L8_DISTRO_MAX_SENSOR_COUNT; i++) {
        _distance_sensor_pub[i].advertise();
        device_id.devid_s.address = i + 1;
        _sensors_device_id[i] = device_id.devid;
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

bool VL53L8_Distro::parse_and_fill_visual_odometry(Visual_Odometry_Data_s *data) {
    if (data->crc != data->calculate_crc(false)) {
        PX4_ERR("CRC mismatch: received: %04X, expected: %04X", data->crc, data->calculate_crc(false));
        perf_count(_comms_errors);
        return false;
    }

    optical_navigation_horizontal_s msg = {};
    msg.timestamp = data->timestamp;
    msg.x_m                 = data->x_m;
    msg.y_m                 = data->y_m;
    msg.vx_m_s              = data->vx_mps;
    msg.vy_m_s              = data->vy_mps;
    msg.roll_rad            = data->roll_rad;
    msg.pitch_rad           = data->pitch_rad;
    msg.var_x_m2            = data->var_x_m2;
    msg.var_y_m2            = data->var_y_m2;
    msg.var_vx_m2s2         = data->var_vx_m2s2;
    msg.var_vy_m2s2         = data->var_vy_m2s2;
    msg.var_roll_rad2       = data->var_roll_rad2;
    msg.var_pitch_rad2      = data->var_pitch_rad2;
    msg.rho_m               = data->rho_m;
    msg.calculation_time_ms = data->calculation_time_ms;
    msg.inliers_total       = data->inliers_total;
    msg.n1                  = data->n1;
    msg.n2                  = data->n2;
    msg.ok                  = data->ok;
    msg.ok_prior            = data->ok_prior;
    _optical_navigation_pub.publish(msg);

    return true;
}

void VL53L8_Distro::Run()
{
    perf_begin(_sample_perf);

    if(_task_should_exit) {
        PX4_INFO("VL53L8_Distro task exit requested");
        // TODO: Handle cleanup if necessary
        perf_cancel(_sample_perf);
        return;
    }

    // Ensure the serial port is open.
	if(open_serial_port() != PX4_OK) {
        PX4_ERR("Failed to open serial port");
        stop();
        ScheduleDelayed(500_ms); // Retry after a short delay
        return;
    }

    // Check if the sensor is alive and initialize it
    if (!_is_initialized) {
        if(initialize_sensor() != PX4_OK) {
            PX4_ERR("Failed to initialize VL53L8_Distro sensor");
            perf_cancel(_sample_perf);
            stop();
            return;
        }

        // Get the sensors resolution
        if(get_sensors_resolution() != PX4_OK) {
            PX4_ERR("Failed to get sensors resolution");
            perf_cancel(_sample_perf);
            stop();
            return;
        }

        // Perform a single measurement
        if(measure(UART_PROT_CMD_RNG_SINGLE) != PX4_OK) {
            PX4_ERR("Failed to perform initial measurement");
            perf_cancel(_sample_perf);
            stop();
            return;
        }

        // if(collect(1_s) != PX4_OK) {
        if(collect_streaming(1_s) != PX4_OK) {
            PX4_ERR("Failed to collect initial measurement data");
            perf_cancel(_sample_perf);
            stop();
            return;
        }

        // Start the automatic measurement
        if(measure(UART_PROT_CMD_RNG_START) != PX4_OK) {
            PX4_ERR("Failed to start ranging");
            perf_cancel(_sample_perf);
            stop();
            return;
        }

        // ScheduleDelayed(120_ms); // Schedule the next reading cycle
        // ScheduleOnInterval(_task_interval, _task_interval); // Schedule the next reading cycle
        ScheduleNow();
        return;
    }

    if(hrt_elapsed_time(&_last_sync_time) > 10_s) {
        send_timesync();
    }

    // if(collect(500_ms) != PX4_OK) {
    if(collect_streaming(2_s) != PX4_OK) {
        // PX4_WARN("Desync detected...");
        // measure(UART_PROT_CMD_RNG_STOP, false);
        // px4_usleep(200_ms);
        // int bytes_read = 1;
        // while(bytes_read > 0)
        //     bytes_read = _uart.readAtLeast(_buffer, _buffer_size, _buffer_size, 10_ms);
        // PX4_INFO("Restarting...");
        // open_serial_port();
        // send_timesync();
        // measure(UART_PROT_CMD_RNG_START);
        // _rx.clear();
        send_timesync();
        // ScheduleDelayed(20_ms); // Schedule the next reading cycle
        // perf_end(_sample_perf);
        // return;
    }

    ScheduleNow();

    perf_end(_sample_perf);
}

void VL53L8_Distro::start()
{
    PX4_INFO("Starting VL53L8_Distro thread");
    // ScheduleNow();
    ScheduleDelayed(1_s);
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

        // Send the sensor initialization command
        cmd_value = 0x00; // Use resolution set before for sensor initialization
        if(send_command(UART_PROT_CMD_SENSOR_INIT, cmd_value, true, 20_s) != PX4_OK) {
            PX4_ERR("Failed to initialize sensors");
            return PX4_ERROR;
        }

        if(cmd_value == 0 || cmd_value > VL53L8_DISTRO_MAX_SENSOR_COUNT) {
            PX4_ERR("Invalid number of sensors detected: %d", cmd_value);
            return PX4_ERROR;
        }

        _sensors_count = cmd_value;
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

int VL53L8_Distro::read_packet(PacketType &packet_type, uint32_t timeout_us)
{
    const hrt_abstime start_time_us = hrt_absolute_time();

    auto crc_over_ring = [&](size_t start_off, size_t nbytes) -> uint16_t {
        // Liczymy CRC nad n bajtami zaczynając od offsetu start_off w ringu,
        // w porcjach (żeby nie alokować tymczasowych buforów dużych rozmiarów).
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

    while (hrt_elapsed_time(&start_time_us) < timeout_us) {
        // 1) dolej bajty z UART do ringu
        pump_uart_to_ring(1_ms);

        // 2) szukamy nagłówka 0xD3 0xAC
        if (_rx.size() < UART_PROT_MSG_HEADER_LEN) continue;

        // wyrównanie do nagłówka
        size_t offset = 0;
        if (!_rx.find2(UART_PROT_MSG_HEADER_1, UART_PROT_MSG_HEADER_2, offset)) {
            // brak nagłówka — zostaw 1 bajt na przyszłość, resztę zrzuć
            if (_rx.size() > 1) _rx.drop(_rx.size() - 1);
            continue;
        }
        if (offset) _rx.drop(offset); // dosuń header do początku

        // 3) mamy header; odczytaj długość pakietu (pole packet_len za headerem)
        if (_rx.size() < UART_PROT_MSG_HEADER_LEN) continue;
        uint16_t packet_len_le = 0;
        _rx.peek(2, reinterpret_cast<uint8_t*>(&packet_len_le), 2);
        const uint16_t packet_len = packet_len_le;

        // sanity check długości (payload + CRC)
        if (packet_len < UART_PROT_PAYLOAD_MIN_SIZE || packet_len > UART_PROT_PAYLOAD_MAX_SIZE) {
            _rx.drop(1); // zły length — przesuń o 1 i próbuj dalej
            continue;
        }

        const uint16_t frame_len = UART_PROT_MSG_HEADER_LEN + packet_len; // pełna ramka
        if (_rx.size() < frame_len) {
            // czekamy aż dopłyną brakujące bajty
            continue;
        }

        // 4) CRC: liczymy po polu packet_len..(przed CRC) = (packet_len - CRC_LEN) bajtów
        // Start offset = 2 (bo omijamy header_1, header_2)
        const size_t crc_region_off = 2;
        const size_t crc_region_len = packet_len - UART_PROT_MSG_CRC_LEN;

        // pobierz CRC z końca ramki (2 bajty LE)
        uint16_t received_crc_le = 0;
        _rx.peek(UART_PROT_MSG_HEADER_LEN + packet_len - UART_PROT_MSG_CRC_LEN,
                 reinterpret_cast<uint8_t*>(&received_crc_le), 2);
        const uint16_t received_crc = received_crc_le;

        // policz oczekiwane CRC bez kopiowania
        const uint16_t expected_crc = crc_over_ring(crc_region_off, crc_region_len);

        if (received_crc != expected_crc) {
            perf_count(_comms_errors);
            _rx.drop(1);        // zły CRC — przesuń o 1 bajt i kontynuuj skan
            continue;
        }

        // 5) Sukces: przenieś całą ramkę do _buffer (jednorazowo), zjedz ją z ringu
        _rx.read(_buffer, frame_len);

        // 6) Rozpoznaj typ po całkowitym rozmiarze ramki (jak wcześniej)
        switch (frame_len) {
            case sizeof(CMD_short_s):             packet_type = PacketType::CMD_Short;       break;
            case sizeof(CMD_long_s):              packet_type = PacketType::CMD_Long;        break;
            case sizeof(Visual_Odometry_Data_s):  packet_type = PacketType::MSG_VisualOdometry; break;
            case sizeof(VL_Range_Data_s<VL53L8_RESOLUTION_4x4>):  packet_type = PacketType::MSG_RangeData_16; break;
            case sizeof(VL_Range_Data_s<VL53L8_RESOLUTION_8x8>):  packet_type = PacketType::MSG_RangeData_64; break;
            // jeśli dodałeś odometrię:
            // case sizeof(ODOM_Packet_s):           packet_type = PacketType::MSG_Odometry;    break;
            default:
                packet_type = PacketType::INVALID;
                PX4_ERR("Invalid packet size: %u", frame_len);
                continue;
        }

        return PX4_OK;
    }

    return PX4_ERROR; // timeout
}

int VL53L8_Distro::collect_streaming(uint32_t timeout_us)
{
    perf_begin(_sample_perf);

    const hrt_abstime start_time_us = hrt_absolute_time();
    // const uint16_t FULL_MASK =
    //     (_sensors_count >= 16) ? 0xFFFFu : ((1u << _sensors_count) - 1u);
    bool init_mask = (_sensors_active_mask == 0);
    uint16_t received_mask = 0;
    uint8_t seq_ref = 0xFF;

    distance_sensor_matrix_s msg{};
    msg.timestamp = hrt_absolute_time();

    while (hrt_elapsed_time(&start_time_us) < timeout_us && (init_mask || (received_mask != _sensors_active_mask))) {
        PacketType type;
        if (read_packet(type, 5_ms) != PX4_OK) {
            continue; // nic nie przyszło — jeszcze chwilę czekamy
        }

        if (type == PacketType::MSG_RangeData_64) {
            auto *d = reinterpret_cast<VL_Range_Data_s<VL53L8_RESOLUTION_8x8>*>(&_buffer[0]);
            // PX4_INFO("Sensor data: timestamp: %llu, sensor_id: %d, seq: %d", d->timestamp, d->sensor_id, d->seq);
            if (init_mask) _sensors_active_mask |= (1u << (d->sensor_id - 1));
            if (seq_ref == 0xFF) seq_ref = d->seq;
            if (d->seq != seq_ref) { received_mask = 0; seq_ref = d->seq; }
            if (parse_and_fill<VL53L8_RESOLUTION_8x8>(d, msg)) {
                received_mask |= (1u << (d->sensor_id - 1));
            }
        } else if (type == PacketType::MSG_RangeData_16) {
            auto *d = reinterpret_cast<VL_Range_Data_s<VL53L8_RESOLUTION_4x4>*>(&_buffer[0]);
            if (init_mask) _sensors_active_mask |= (1u << (d->sensor_id - 1));
            if (seq_ref == 0xFF) seq_ref = d->seq;
            if (d->seq != seq_ref) { received_mask = 0; seq_ref = d->seq; }
            if (parse_and_fill<VL53L8_RESOLUTION_4x4>(d, msg)) {
                received_mask |= (1u << (d->sensor_id - 1));
            }
        } else if(type == PacketType::MSG_VisualOdometry) {
            auto *odom = reinterpret_cast<Visual_Odometry_Data_s*>(&_buffer[0]);
            // PX4_INFO("Odometry data: timestamp: %llu", odom->timestamp);
            parse_and_fill_visual_odometry(odom);
        } else {
            // CMD_Short / CMD_Long — ACK, timesync itp. — ignorujemy tu
            // PX4_INFO("Ignoring packet of type %d", static_cast<int>(type));
        }
    }
    if(received_mask != _sensors_active_mask) {
        // PX4_WARN("Collected data from %u sensors (mask 0b%c%c" BYTE_TO_BINARY_PATTERN ")", __builtin_popcount(received_mask), ((received_mask) & 0x200 ? '1' : '0'), ((received_mask) & 0x100 ? '1' : '0'), BYTE_TO_BINARY(received_mask));
    }
    perf_end(_sample_perf);
    return (received_mask == _sensors_active_mask) ? PX4_OK : PX4_ERROR;
}

