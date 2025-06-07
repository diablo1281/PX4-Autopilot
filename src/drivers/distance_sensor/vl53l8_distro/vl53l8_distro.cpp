#include "vl53l8_distro.hpp"

#include <cerrno>

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include <lib/drivers/device/Device.hpp>

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

VL53L8_Distro::VL53L8_Distro(const char *path) :
    ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(path))
{
    /* store port name */
	strncpy(_port, path, sizeof(_port) - 1);
    /* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';



    device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
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

    PX4_INFO("Collecting data from sensor...");

    PacketType packet_type;
    if(read_packet(packet_type, timeout_us) != PX4_OK) {
        PX4_ERR("Failed to read packet from sensor");
        perf_count(_comms_errors);
        perf_end(_sample_perf);
        return PX4_ERROR;
    }

    if(packet_type == PacketType::MSG_RangeData_16) {
        VL_Range_Data_s<16> *data = (VL_Range_Data_s<16> *)(_buffer);
        PX4_INFO("Sensor data collected: timestamp: %llu, sensor %d, resolution: %d", data->timestamp, data->sensor_id, data->resolution);
    } else if (packet_type == PacketType::MSG_RangeData_64) {
        VL_Range_Data_s<64> *data = (VL_Range_Data_s<64> *)(_buffer);
        PX4_INFO("Sensor data collected: timestamp: %llu, sensor %d, resolution: %d", data->timestamp, data->sensor_id, data->resolution);
    } else {
        PX4_ERR("Failed to collect data, invalid packet type: %d", packet_type);
        perf_count(_comms_errors);
        perf_end(_sample_perf);
        return PX4_ERROR;
    }

    perf_end(_sample_perf);

    return PX4_OK;
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
        if(initialize_sensor() != PX4_OK && get_sensors_resolution() != PX4_OK) {
            PX4_ERR("Failed to initialize VL53L8_Distro sensor");
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

        uint8_t data_received = 0;
        for(uint8_t i = 0; i < _sensors_count; i++) {
            if(collect(500_ms) != PX4_OK) {
                PX4_ERR("Failed to collect data for sensor %d", i);
            } else {
                PX4_INFO("Successfully collected data for sensor %d", i);
                data_received++;
            }
        }

        if(data_received == _sensors_count) {
            PX4_INFO("Successfully performed initial measurement for all sensors: %d", _sensors_count);
        } else {
            PX4_ERR("Failed to collect data from some sensors: %d/%d", data_received, _sensors_count);
            perf_cancel(_sample_perf);
            stop();
            return;
        }
    }

    uint8_t data_received = 0;
    for(uint8_t i = 0; i < _sensors_count; i++) {
        if (collect() != PX4_OK) {
            PX4_ERR("Failed to collect data from sensor %d", i);
            // stop();
            // return;
        } else {
            data_received++;
        }
    }

    if(data_received == _sensors_count) {
        PX4_INFO("Successfully collected data from all sensors: %d", _sensors_count);
    } else {
        PX4_ERR("Failed to collect data from some sensors: %d/%d", data_received, _sensors_count);
    }

    PX4_INFO("VL53L8_Distro task exit requested");

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
            PX4_INFO("Wrote %d bytes to port %s for checking if it'a alive", ret, _port);
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
                return PX4_ERROR;
            } else if (packet_type != PacketType::CMD_Short && (((CMD_short_s *)&_buffer[0])->cmd != UART_PROT_CMD_STATUS_ACK)) {
                PX4_ERR("No ACK response received");
                perf_count(_comms_errors);
                return PX4_ERROR;
            } else {
                PX4_INFO("VL53L8_Distro is alive on port: %s", _port);
                break; // Exit the loop if the sensor is alive
            }

            retry++;
            px4_sleep(1); // Wait for a second before retrying
        }

        CMD_long_s msg_long{};
        msg_long.cmd = UART_PROT_CMD_TIMESYNC;
        msg_long.value = hrt_abstime(); // Use current time for synchronization
        msg_long.calculate_crc(true);
        ret = _uart.write((const void *)&msg_long, sizeof(msg_long));
        PX4_INFO("Wrote %d bytes to port %s for time synchronization [%llu]", ret, _port, msg_long.value);

        msg.cmd = UART_PROT_CMD_SENSOR_INIT;
        msg.value = 0; // No specific value needed for this command
        msg.calculate_crc(true);
        // Send the sensor initialization command
        ret = _uart.write((const void *)&msg, sizeof(msg));
        PX4_INFO("Wrote %d bytes to port %s for sensor initialization", ret, _port);
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
    PX4_INFO("Wrote %d bytes to port %s for getting sensors resolution", ret, _port);
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

    _sensors_resolution = msg.value; // Assuming value contains the resolution
    PX4_INFO("Sensors resolution: %d", _sensors_resolution);

    return PX4_OK;
}

int VL53L8_Distro::measure(uint8_t command) {
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

    if(_ranging_in_progress && command != UART_PROT_CMD_RNG_STOP) {
        PX4_ERR("Ranging already in progress, cannot start new measurement");
        return PX4_ERROR;
    }

    CMD_short_s msg{};
    msg.cmd = command;
    msg.value = 0; // No specific value needed for this command
    msg.calculate_crc(true);

    int ret = _uart.write((const void *)&msg, sizeof(msg));
    PX4_INFO("Wrote %d bytes to port %s for single measurement", ret, _port);
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

    if(command == UART_PROT_CMD_RNG_START) {
        _ranging_in_progress = true;
        PX4_INFO("Ranging started successfully");
    } else if(command == UART_PROT_CMD_RNG_STOP) {
        _ranging_in_progress = false;
        PX4_INFO("Ranging stopped successfully");
    } else if(command == UART_PROT_CMD_RNG_SINGLE) {
        _ranging_in_progress = false; // Single measurement does not keep ranging active
        PX4_INFO("Single measurement completed successfully");
    }

    return PX4_OK;
}

int VL53L8_Distro::open_serial_port(speed_t speed) {
    if(_uart.isOpen()) {
        PX4_INFO("Serial port %s already open", _port);
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

int VL53L8_Distro::read_packet(PacketType &packet_type, uint32_t timeout_us) {
    ssize_t bytes_read = 0;
    hrt_abstime start_time = hrt_absolute_time();
    uint8_t *buffer_ptr = (uint8_t *)&_buffer[0]; // Use the internal buffer
    uint16_t buffer_remaining = _buffer_size;

    // FIXME: readAtLeast is reading more than one byte at a time, which is not expected.

    while (hrt_elapsed_time(&start_time) < timeout_us) {
        bytes_read = _uart.readAtLeast(buffer_ptr, 1, 1, timeout_us / 10);
        if (bytes_read < 0) {
            PX4_ERR("Failed to read from UART: %d (%s)", errno, strerror(errno));
            perf_count(_comms_errors);
            return PX4_ERROR;
        } else if (bytes_read == 0) {
            PX4_ERR("No data read from UART within timeout");
            continue;
        } else if(*buffer_ptr != UART_PROT_MSG_HEADER_1) {
            PX4_ERR("Invalid header byte: 0x%02X", *buffer_ptr);
            continue; // Skip to the next byte
        } else if (bytes_read > 1) {
            PX4_ERR("Read more than one byte when expecting header byte only [%u]", bytes_read);
        }

        buffer_remaining -= bytes_read;
        buffer_ptr += bytes_read;

        bytes_read = _uart.readAtLeast(buffer_ptr, 1, 1, 100_us);
        if (bytes_read < 0) {
            PX4_ERR("Failed to read second header byte: %d (%s)", errno, strerror(errno));
            perf_count(_comms_errors);
            return PX4_ERROR;
        } else if (bytes_read == 0) {
            PX4_ERR("No data read for second header byte within timeout");
            buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
            buffer_remaining = _buffer_size;
            continue; // Skip to the next byte
        } else if(*buffer_ptr != UART_PROT_MSG_HEADER_2) {
            PX4_ERR("Invalid second header byte: 0x%02X", *buffer_ptr);
            buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
            buffer_remaining = _buffer_size;
            continue; // Skip to the next byte
        }

        buffer_remaining -= bytes_read;
        buffer_ptr += bytes_read;

        bytes_read = _uart.readAtLeast(buffer_ptr, 2, 2, 100_us);
        if (bytes_read < 0) {
            PX4_ERR("Failed to read packet length: %d (%s)", errno, strerror(errno));
            perf_count(_comms_errors);
            return PX4_ERROR;
        } else if (bytes_read == 0) {
            PX4_ERR("No data read for packet length within timeout");
            buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
            buffer_remaining = _buffer_size;
            continue; // Skip to the next byte
        }

        uint16_t packet_len = (((CMD_short_s *)&_buffer[0])->packet_len);

        if (packet_len < UART_PROT_PAYLOAD_MIN_SIZE || packet_len > UART_PROT_PAYLOAD_MAX_SIZE) {
            PX4_ERR("Invalid packet length: %d", packet_len);
            buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
            buffer_remaining = _buffer_size;
            continue; // Skip to the next byte
        }

        buffer_remaining -= bytes_read;
        buffer_ptr += bytes_read;

        uint32_t timeout_for_payload = (10 * packet_len) * 1e6 / _uart.getBaudrate(); // Calculate timeout based on baud rate
        timeout_for_payload *= 2; // Add 200% margin

        bytes_read = _uart.readAtLeast(buffer_ptr, packet_len, packet_len, timeout_for_payload);
        if (bytes_read < 0) {
            PX4_ERR("Failed to read packet payload: %d (%s)", errno, strerror(errno));
            perf_count(_comms_errors);
            return PX4_ERROR;
        } else if (bytes_read == 0) {
            PX4_ERR("No data read for packet payload within timeout");
            buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
            buffer_remaining = _buffer_size;
            continue; // Skip to the next byte
        } else if (bytes_read != packet_len) {
            PX4_ERR("Read %d bytes for packet payload, expected %d bytes", bytes_read, packet_len);
            buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
            buffer_remaining = _buffer_size;
            continue; // Skip to the next byte
        }

        uint16_t expected_crc = 0;
        uint16_t received_crc = 0;
        bool crc_valid = false;
        switch (bytes_read + UART_PROT_MSG_HEADER_LEN) {
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

        case (sizeof(VL_Range_Data_s<16>)): {
            packet_type = PacketType::MSG_RangeData_16;
            VL_Range_Data_s<16> *packet = (VL_Range_Data_s<16> *)&_buffer[0];
            expected_crc = packet->calculate_crc(false);
            received_crc = packet->crc;
            crc_valid = (received_crc == expected_crc);
            break;
        }

        case (sizeof(VL_Range_Data_s<64>)): {
            packet_type = PacketType::MSG_RangeData_64;
            VL_Range_Data_s<64> *packet = (VL_Range_Data_s<64> *)&_buffer[0];
            expected_crc = packet->calculate_crc(false);
            received_crc = packet->crc;
            crc_valid = (received_crc == expected_crc);
            break;
        }

        default:
            packet_type = PacketType::INVALID;
            PX4_ERR("Invalid packet size: %d bytes", bytes_read);
            buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
            buffer_remaining = _buffer_size;
            continue; // Skip to the next byte
            break;
        }

        PX4_INFO("Received packet type: %d, size: %d bytes", (int)packet_type, bytes_read + UART_PROT_MSG_HEADER_LEN);

        if(!crc_valid) {
            PX4_ERR("CRC mismatch in received packet [0x%04X != 0x%04X]", received_crc, expected_crc);
            perf_count(_comms_errors);
            buffer_ptr = (uint8_t *)&_buffer[0]; // Reset buffer pointer
            buffer_remaining = _buffer_size;
            continue; // Skip to the next byte
        }

        return PX4_OK; // Successfully read a packet
    }

    return PX4_ERROR; // Timeout waiting for a packet
}
