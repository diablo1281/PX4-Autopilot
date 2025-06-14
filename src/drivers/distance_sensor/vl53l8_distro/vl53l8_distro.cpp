#include "vl53l8_distro.hpp"

#include <cerrno>

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include <lib/drivers/device/Device.hpp>

#include <parameters/param.h>

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
    int32_t resolution = 0;
    param_get(param_find("VL_DISTRO_RES"), &resolution);

    if(resolution > 0) {
        _sensors_resolution = (uint8_t) resolution;
    } else {
        PX4_ERR("Error reading `VL_DISTRO_RES` parameter!");
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

        const size_t packet_size = (_sensors_resolution == VL53L8_RESOLUTION_8x8)
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

        if(((VL_Range_Data_s<VL53L8_RESOLUTION_4x4> *)&_buffer[read_idx])->resolution != _sensors_resolution) {
            PX4_ERR("Wrong resolution: [ %u != %u ]", ((VL_Range_Data_s<VL53L8_RESOLUTION_4x4> *)&_buffer[read_idx])->resolution, _sensors_resolution);
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

        if(collect(1_s) != PX4_OK) {
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

        ScheduleDelayed(120_ms); // Schedule the next reading cycle
        return;
    }

    if(hrt_elapsed_time(&_last_sync_time) > 1_s) {
        send_timesync();
    }

    if(collect(500_ms) != PX4_OK) {
        PX4_WARN("Desync detected...");
        measure(UART_PROT_CMD_RNG_STOP, false);
        px4_usleep(200_ms);
        int bytes_read = 1;
        while(bytes_read > 0)
            bytes_read = _uart.readAtLeast(_buffer, _buffer_size, _buffer_size, 10_ms);
        PX4_INFO("Restarting...");
        open_serial_port();
        send_timesync();
        measure(UART_PROT_CMD_RNG_START);
        ScheduleDelayed(50_ms); // Schedule the next reading cycle
        perf_end(_sample_perf);
        return;
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

        CMD_short_s msg{};
        int ret;
        uint8_t retry = 0;

        while (retry < 5)
        {
            msg.cmd = UART_PROT_CMD_IS_ALIVE;
            msg.value = 0; // No specific value needed for this command
            msg.calculate_crc(true);
            ret = _uart.write((const void *)&msg, sizeof(msg));
            // PX4_INFO("Wrote %d bytes to port %s for checking if it'a alive", ret, _port);
            if (ret <= 0) {
                PX4_ERR("write failed: %d (%s)", errno, strerror(errno));
                perf_count(_comms_errors);
                return PX4_ERROR;
            }

            // Wait for the ACK response
            PacketType packet_type;
            ret = read_packet(packet_type, 10_ms);
            if (ret < 0) {
                PX4_ERR("Failed to read ACK response: %d (%s)", errno, strerror(errno));
                perf_count(_comms_errors);
                // return PX4_ERROR;
            } else if (packet_type != PacketType::CMD_Short && (((CMD_short_s *)&_buffer[0])->cmd != UART_PROT_CMD_STATUS_ACK)) {
                PX4_ERR("No ACK response received");
                perf_count(_comms_errors);
                // return PX4_ERROR;
            } else {
                PX4_INFO("VL53L8_Distro is alive on port: %s", _port);
                break; // Exit the loop if the sensor is alive
            }

            retry++;
            px4_sleep(1); // Wait for a second before retrying
        }

        send_timesync();

        msg.cmd = UART_PROT_CMD_SENSOR_INIT;
        msg.value = _sensors_resolution; // Set the resolution for sensor initialization
        msg.calculate_crc(true);
        // Send the sensor initialization command
        ret = _uart.write((const void *)&msg, sizeof(msg));
        // PX4_INFO("Wrote %d bytes to port %s for sensor initialization", ret, _port);
        if (ret <= 0) {
            PX4_ERR("write failed: %d (%s)", errno, strerror(errno));
            perf_count(_comms_errors);
            return PX4_ERROR;
        }

        // Wait for the ACK response from initialization command
        PacketType packet_type;
        ret = read_packet(packet_type, 10_s);
        if (ret < 0) {
            PX4_ERR("Failed to read ACK response: %d (%s)", errno, strerror(errno));
            perf_count(_comms_errors);
            return PX4_ERROR;
        } else if (packet_type != PacketType::CMD_Short && (((CMD_short_s *)&_buffer[0])->cmd != UART_PROT_CMD_STATUS_ACK)) {
            PX4_ERR("No ACK response received");
            perf_count(_comms_errors);
            return PX4_ERROR;
        }

        _sensors_count = ((CMD_short_s *)&_buffer[0])->value;
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

    CMD_short_s msg{};
    msg.cmd = UART_PROT_CMD_SENSOR_RES;
    msg.value = 0; // Value 0x00 for reading the sensors resolution
    msg.calculate_crc(true);

    int ret = _uart.write((const void *)&msg, sizeof(msg));
    // PX4_INFO("Wrote %d bytes to port %s for getting sensors resolution", ret, _port);
    if (ret <= 0) {
        PX4_ERR("write failed: %d (%s)", errno, strerror(errno));
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    // Wait for the ACK response
    PacketType packet_type;
    ret = read_packet(packet_type, 5_ms);
    if (ret < 0) {
        PX4_ERR("Failed to read ACK response: %d (%s)", errno, strerror(errno));
        perf_count(_comms_errors);
        return PX4_ERROR;
    } else if (packet_type != PacketType::CMD_Short && (((CMD_short_s *)&_buffer[0])->cmd != UART_PROT_CMD_STATUS_ACK)) {
        PX4_ERR("No ACK response received");
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    _sensors_resolution = ((CMD_short_s *)&_buffer[0])->value; // Assuming value contains the resolution
    PX4_INFO("Sensors resolution: %d", _sensors_resolution);

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

    CMD_short_s msg{};
    msg.cmd = command;
    msg.value = 0; // No specific value needed for this command
    msg.calculate_crc(true);

    int ret = _uart.write((const void *)&msg, sizeof(msg));
    // PX4_INFO("Wrote %d bytes to port %s for start measurement", ret, _port);
    if (ret <= 0) {
        PX4_ERR("write failed: %d (%s)", errno, strerror(errno));
        perf_count(_comms_errors);
        return PX4_ERROR;
    }


    if(ack) {
        // Wait for the ACK response
        PacketType packet_type;
        ret = read_packet(packet_type, 5_ms);
        if (ret < 0) {
            PX4_ERR("Failed to read ACK response: %d (%s)", errno, strerror(errno));
            perf_count(_comms_errors);
            return PX4_ERROR;
        } else if (packet_type != PacketType::CMD_Short && (((CMD_short_s *)&_buffer[0])->cmd != UART_PROT_CMD_STATUS_ACK)) {
            PX4_ERR("No ACK response received");
            perf_count(_comms_errors);
            return PX4_ERROR;
        }
    } else {
        _uart.readAtLeast(_buffer, _buffer_size, UART_PROT_MSG_MIN_SIZE, 5_ms);
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

int VL53L8_Distro::read_ACK(CMD_short_s &msg, uint32_t timeout_us) {
    ssize_t bytes_read = _uart.readAtLeast((uint8_t *)&msg, sizeof(CMD_short_s), sizeof(CMD_short_s), timeout_us);

    if(bytes_read < 0) {
        PX4_ERR("Failed to read ACK from UART: %d (%s)", errno, strerror(errno));
        perf_count(_comms_errors);
        return PX4_ERROR;
    } else if (bytes_read == 0) {
        PX4_ERR("No ACK read from UART within timeout");
        perf_count(_comms_errors);
        return PX4_ERROR;
    } else if (bytes_read != sizeof(CMD_short_s)) {
        PX4_ERR("Read %zd bytes, expected %zu bytes for ACK", bytes_read, sizeof(CMD_short_s));
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    if(msg.crc != msg.calculate_crc(false)) {
        PX4_ERR("CRC mismatch for ACK: received: %04X, expected: %04X", msg.crc, msg.calculate_crc(false));
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    return PX4_OK; // Successfully read ACK
}

int VL53L8_Distro::read_data(uint32_t timeout_us) {
    uint16_t read_size = ((_sensors_resolution == VL53L8_RESOLUTION_8x8) ? sizeof(VL_Range_Data_s<VL53L8_RESOLUTION_8x8>) : sizeof(VL_Range_Data_s<VL53L8_RESOLUTION_4x4>)) * _sensors_count;
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

int VL53L8_Distro::read_packet(PacketType &packet_type, uint32_t timeout_us) {
    hrt_abstime start_time = hrt_absolute_time();
    ssize_t bytes_read = 0;
    uint16_t i;
    uint16_t packet_len = 0;
    bool msg_found = false;
    // const uint16_t max_packet_size = (_sensors_resolution == 64) ? sizeof(VL_Range_Data_s<64>) : sizeof(VL_Range_Data_s<16>);
    // uint16_t remain_to_read = max_packet_size;

    while (hrt_elapsed_time(&start_time) < timeout_us) {
        // Read header
        bytes_read = _uart.readAtLeast(&_buffer[0],
            UART_PROT_MSG_MIN_SIZE,
            UART_PROT_MSG_HEADER_LEN, timeout_us / 10);
        if(bytes_read < UART_PROT_MSG_HEADER_LEN) { PX4_ERR("Read too little: %d", bytes_read); continue; }// No data read or error

        // PX4_INFO("Read %d bytes from UART [packet size: %u]", bytes_read, max_packet_size);

        while(hrt_elapsed_time(&start_time) < timeout_us) {
            if (_buffer[0] != UART_PROT_MSG_HEADER_1 || _buffer[1] != UART_PROT_MSG_HEADER_2) {
                return PX4_ERROR; // Invalid header, return error
                for(i = 0; i < (bytes_read - 1); i++) {
                    if(_buffer[i] == UART_PROT_MSG_HEADER_1 && _buffer[i + 1] == UART_PROT_MSG_HEADER_2) {
                        // Found the start of a packet
                        if(i == 0) break; // Header is already at the start of the buffer
                        memmove(&_buffer[0], &_buffer[i], bytes_read - i); // Move the found header to the start of the buffer
                        bytes_read -= i; // Adjust bytes_read to reflect the new position
                        // remain_to_read = max_packet_size - bytes_read; // Calculate remaining bytes to read
                        break;
                    }
                }

                // If no valid header found, continue reading
                if (_buffer[0] != UART_PROT_MSG_HEADER_1 || _buffer[1] != UART_PROT_MSG_HEADER_2) {
                    PX4_ERR("Invalid packet header: 0x%02X 0x%02X", _buffer[0], _buffer[1]);
                    break; // Skip to the next byte
                }
            }

            if (bytes_read < UART_PROT_MSG_HEADER_LEN) {
                ssize_t tmp = _uart.read(_buffer + bytes_read, UART_PROT_MSG_HEADER_LEN - bytes_read);
                if(tmp <= 0) {
                    // PX4_ERR("Failed to read rest of header: %d (%s)", errno, strerror(errno));
                    perf_count(_comms_errors);
                    break; // Exit the loop if no more data is available
                }
            }

            packet_len = (((CMD_short_s *)_buffer)->packet_len);

            if (packet_len < UART_PROT_PAYLOAD_MIN_SIZE || packet_len > UART_PROT_PAYLOAD_MAX_SIZE) {
                PX4_ERR("Invalid packet length: %d", packet_len);
                bytes_read -= 2; // Remove the header bytes to look for next one
                memmove(&_buffer[0], &_buffer[2], bytes_read);
                continue;
            }

            // PX4_INFO("Packet length: %d bytes , read: %d", packet_len + UART_PROT_MSG_HEADER_LEN, bytes_read);

            ssize_t tmp = 0;
            uint8_t *buffer_ptr = &_buffer[bytes_read]; // Use the internal buffer
            // px4_usleep(300_us);
            while(bytes_read < (packet_len + UART_PROT_MSG_HEADER_LEN)) {
                size_t remaining_bytes = (packet_len + UART_PROT_MSG_HEADER_LEN) - bytes_read;
                tmp = _uart.readAtLeast(buffer_ptr, remaining_bytes, remaining_bytes, timeout_us / 2);
                if(tmp <= 0) {
                    // PX4_ERR("Failed to read packet data: %d (%s)", errno, strerror(errno));
                    px4_usleep(250_us);
                    continue; // No data read or error, continue to read more
                } else {
                    bytes_read += tmp; // Update bytes read
                    buffer_ptr += tmp; // Move the buffer pointer forward
                    // PX4_INFO("Read %d bytes, total bytes read: [ %d / %d ]", tmp, bytes_read, packet_len + UART_PROT_MSG_HEADER_LEN);
                }
            }

            // PX4_INFO("Total bytes read for packet: %d", bytes_read);
            msg_found = true;
            break; // Exit the while loop if we have read enough bytes
        }

        // PX4_INFO("First 4 bytes: %02X %02X %02X %02X", _buffer[2], _buffer[3], _buffer[4], _buffer[5]);
        // PX4_INFO("Last 4 bytes before CRC: %02X %02X %02X %02X", _buffer[2 + packet_len - 4], _buffer[2 + packet_len - 3], _buffer[2 + packet_len - 2], _buffer[2 + packet_len - 1]);
        // PX4_INFO("CRC bytes: %02X %02X", _buffer[4 + packet_len - 2], _buffer[4 + packet_len - 1]);

        if(!msg_found) {
            // PX4_ERR("No valid packet found in the buffer");
            // perf_count(_comms_errors);
            return PX4_ERROR; // No valid packet found
        }

        uint16_t expected_crc = 0;
        uint16_t received_crc = 0;
        bool crc_valid = false;
        switch (bytes_read) {
        case (sizeof(CMD_short_s)): {
            packet_type = PacketType::CMD_Short;
            CMD_short_s *packet = (CMD_short_s *)&_buffer[0];
            expected_crc = packet->calculate_crc(false);
            received_crc = packet->crc;
            crc_valid = (received_crc == expected_crc);
            break;
        }

        case (sizeof(CMD_long_s)): {
            packet_type = PacketType::CMD_Long;
            CMD_long_s *packet = (CMD_long_s *)&_buffer[0];
            expected_crc = packet->calculate_crc(false);
            received_crc = packet->crc;
            crc_valid = (received_crc == expected_crc);
            break;
        }

        case (sizeof(VL_Range_Data_s<VL53L8_RESOLUTION_4x4>)): {
            packet_type = PacketType::MSG_RangeData_16;
            VL_Range_Data_s<VL53L8_RESOLUTION_4x4> *packet = (VL_Range_Data_s<VL53L8_RESOLUTION_4x4> *)&_buffer[0];
            expected_crc = packet->calculate_crc(false);
            received_crc = packet->crc;
            crc_valid = (received_crc == expected_crc);
            break;
        }

        case (sizeof(VL_Range_Data_s<VL53L8_RESOLUTION_8x8>)): {
            packet_type = PacketType::MSG_RangeData_64;
            VL_Range_Data_s<VL53L8_RESOLUTION_8x8> *packet = (VL_Range_Data_s<VL53L8_RESOLUTION_8x8> *)&_buffer[0];
            expected_crc = packet->calculate_crc(false);
            received_crc = packet->crc;
            crc_valid = (received_crc == expected_crc);
            break;
        }

        default:
            packet_type = PacketType::INVALID;
            PX4_ERR("Invalid packet size: %d bytes", bytes_read);
            return PX4_ERROR; // Invalid packet size
        }

        // PX4_INFO("Received packet type: %d, size: %d bytes", (int)packet_type, bytes_read + UART_PROT_MSG_HEADER_LEN);

        if(!crc_valid) {
            PX4_ERR("CRC mismatch in received packet [0x%04X != 0x%04X]", received_crc, expected_crc);
            perf_count(_comms_errors);
            return PX4_ERROR;   // TODO: Search for the next packet in the buffer
        }

        return PX4_OK; // Successfully read a packet
    }

    return PX4_ERROR;   // timeout
}

// int VL53L8_Distro::read_packet(PacketType &packet_type, uint32_t timeout_us) {
//     ssize_t bytes_read = 0;
//     hrt_abstime start_time = hrt_absolute_time();
//     uint8_t *buffer_ptr = (uint8_t *)&_buffer[0]; // Use the internal buffer
//     uint16_t buffer_remaining = _buffer_size;

//     // FIXME: readAtLeast is reading more than one byte at a time, which is not expected.

//     while (hrt_elapsed_time(&start_time) < timeout_us) {
//         bytes_read = _uart.readAtLeast(buffer_ptr, 1, 1, timeout_us / 10);
//         if (bytes_read < 0) {
//             PX4_ERR("Failed to read from UART: %d (%s)", errno, strerror(errno));
//             perf_count(_comms_errors);
//             return PX4_ERROR;
//         } else if (bytes_read == 0) {
//             PX4_ERR("No data read from UART within timeout");
//             continue;
//         } else if(*buffer_ptr != UART_PROT_MSG_HEADER_1) {
//             PX4_ERR("Invalid header byte: 0x%02X", *buffer_ptr);
//             continue; // Skip to the next byte
//         } else if (bytes_read > 1) {
//             PX4_ERR("Read more than one byte when expecting header byte only [%u]", bytes_read);
//         }

//         buffer_remaining -= bytes_read;
//         buffer_ptr += bytes_read;

//         bytes_read = _uart.readAtLeast(buffer_ptr, 1, 1, 100_us);
//         if (bytes_read < 0) {
//             PX4_ERR("Failed to read second header byte: %d (%s)", errno, strerror(errno));
//             perf_count(_comms_errors);
//             return PX4_ERROR;
//         } else if (bytes_read == 0) {
//             PX4_ERR("No data read for second header byte within timeout");
//             buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
//             buffer_remaining = _buffer_size;
//             continue; // Skip to the next byte
//         } else if(*buffer_ptr != UART_PROT_MSG_HEADER_2) {
//             PX4_ERR("Invalid second header byte: 0x%02X", *buffer_ptr);
//             buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
//             buffer_remaining = _buffer_size;
//             continue; // Skip to the next byte
//         }

//         buffer_remaining -= bytes_read;
//         buffer_ptr += bytes_read;

//         bytes_read = _uart.readAtLeast(buffer_ptr, 2, 2, 100_us);
//         if (bytes_read < 0) {
//             PX4_ERR("Failed to read packet length: %d (%s)", errno, strerror(errno));
//             perf_count(_comms_errors);
//             return PX4_ERROR;
//         } else if (bytes_read == 0) {
//             PX4_ERR("No data read for packet length within timeout");
//             buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
//             buffer_remaining = _buffer_size;
//             continue; // Skip to the next byte
//         }

//         const uint16_t packet_len = (((CMD_short_s *)&_buffer[0])->packet_len);
//         // PX4_INFO("Packet length: %d", packet_len);

//         if (packet_len < UART_PROT_PAYLOAD_MIN_SIZE || packet_len > UART_PROT_PAYLOAD_MAX_SIZE) {
//             PX4_ERR("Invalid packet length: %d", packet_len);
//             buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
//             buffer_remaining = _buffer_size;
//             continue; // Skip to the next byte
//         }

//         buffer_remaining -= bytes_read;
//         buffer_ptr += bytes_read;

//         uint32_t timeout_for_payload = (10 * packet_len) * 1e6 / _uart.getBaudrate(); // Calculate timeout based on baud rate
//         timeout_for_payload *= 2; // Add 200% margin
//         uint16_t max_single_read = 256;

//         if(packet_len <= max_single_read) {
//             bytes_read = _uart.readAtLeast(buffer_ptr, packet_len, packet_len, timeout_for_payload);
//             if (bytes_read < 0) {
//                 PX4_ERR("Failed to read packet payload: %d (%s)", errno, strerror(errno));
//                 perf_count(_comms_errors);
//                 return PX4_ERROR;
//             } else if (bytes_read == 0) {
//                 PX4_ERR("No data read for packet payload within timeout");
//                 buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
//                 buffer_remaining = _buffer_size;
//                 continue; // Skip to the next byte
//             } else if (bytes_read != packet_len) {
//                 PX4_ERR("Read %d bytes for packet payload, expected %d bytes", bytes_read, packet_len);
//                 buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
//                 buffer_remaining = _buffer_size;
//                 continue; // Skip to the next byte
//             }
//         } else {
//             uint16_t remaining_bytes = packet_len;
//             ssize_t part_bytes_read = 0;
//             bytes_read = 0;

//             while(remaining_bytes > 0) {
//                 part_bytes_read = _uart.readAtLeast(buffer_ptr, buffer_remaining, max_single_read, timeout_for_payload);
//                 if (part_bytes_read < 0) {
//                     PX4_ERR("Failed to read packet payload: %d (%s)", errno, strerror(errno));
//                     perf_count(_comms_errors);
//                     return PX4_ERROR;
//                 } else if (part_bytes_read == 0) {
//                     PX4_ERR("No data read for packet payload within timeout");
//                     buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
//                     buffer_remaining = _buffer_size;
//                     continue; // Skip to the next byte
//                 } else {
//                     remaining_bytes -= part_bytes_read;
//                     buffer_remaining -= part_bytes_read;
//                     buffer_ptr += part_bytes_read;
//                     bytes_read += part_bytes_read;

//                     // PX4_INFO("Read [ %d / %u ] bytes for packet payload, remaining bytes: %u", part_bytes_read, max_single_read, remaining_bytes);

//                     if (remaining_bytes < max_single_read) {
//                         max_single_read = remaining_bytes; // Adjust max_single_read for the next iteration
//                     }
//                 }
//             }
//         }

//         uint16_t expected_crc = 0;
//         uint16_t received_crc = 0;
//         bool crc_valid = false;
//         switch (bytes_read + UART_PROT_MSG_HEADER_LEN) {
//         case (sizeof(CMD_short_s)): {
//             packet_type = PacketType::CMD_Short;
//             CMD_short_s *packet = (CMD_short_s *)&_buffer[0];
//             expected_crc = packet->calculate_crc(false);
//             received_crc = packet->crc;
//             crc_valid = (received_crc == expected_crc);
//             break;
//         }

//         case (sizeof(CMD_long_s)): {
//             packet_type = PacketType::CMD_Long;
//             CMD_long_s *packet = (CMD_long_s *)&_buffer[0];
//             expected_crc = packet->calculate_crc(false);
//             received_crc = packet->crc;
//             crc_valid = (received_crc == expected_crc);
//             break;
//         }

//         case (sizeof(VL_Range_Data_s<16>)): {
//             packet_type = PacketType::MSG_RangeData_16;
//             VL_Range_Data_s<16> *packet = (VL_Range_Data_s<16> *)&_buffer[0];
//             expected_crc = packet->calculate_crc(false);
//             received_crc = packet->crc;
//             crc_valid = (received_crc == expected_crc);
//             break;
//         }

//         case (sizeof(VL_Range_Data_s<64>)): {
//             packet_type = PacketType::MSG_RangeData_64;
//             VL_Range_Data_s<64> *packet = (VL_Range_Data_s<64> *)&_buffer[0];
//             expected_crc = packet->calculate_crc(false);
//             received_crc = packet->crc;
//             crc_valid = (received_crc == expected_crc);
//             break;
//         }

//         default:
//             packet_type = PacketType::INVALID;
//             PX4_ERR("Invalid packet size: %d bytes", bytes_read);
//             buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
//             buffer_remaining = _buffer_size;
//             continue; // Skip to the next byte
//             break;
//         }

//         // PX4_INFO("Received packet type: %d, size: %d bytes", (int)packet_type, bytes_read + UART_PROT_MSG_HEADER_LEN);

//         if(!crc_valid) {
//             PX4_ERR("CRC mismatch in received packet [0x%04X != 0x%04X]", received_crc, expected_crc);
//             perf_count(_comms_errors);
//             buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
//             buffer_remaining = _buffer_size;
//             continue; // Skip to the next byte
//         }

//         return PX4_OK; // Successfully read a packet
//     }

//     return PX4_ERROR; // Timeout waiting for a packet
// }
