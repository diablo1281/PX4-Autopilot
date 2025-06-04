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

VL53L8_Distro::VL53L8_Distro(const char *serial_port) :
    ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serial_port))
{
    _serial_port = strdup(serial_port);

    device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_serial_port[strlen(_serial_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

}

VL53L8_Distro::~VL53L8_Distro()
{
    stop();

    free((char *)_serial_port);
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

int VL53L8_Distro::collect()
{
    perf_begin(_sample_perf);

    PX4_INFO("Collecting data from sensor...");

    tcflush(_port_fd, TCIFLUSH);

    // Placeholder for UART data collection logic
    CMD_long_s cmd_long = CMD_long_s();
    cmd_long.cmd = UART_PROT_CMD_TIMESYNC; // Example command
    cmd_long.value = 0x0102030405060708; // Example value
    cmd_long.crc = calculate_crc((uint8_t *)&cmd_long.packet_len, cmd_long.packet_len);
    int bytes_written = ::write(_port_fd, (uint8_t *)&cmd_long, sizeof(cmd_long));
    PX4_INFO("Wrote %d bytes to port %s", bytes_written, _serial_port);
    if(bytes_written <= 0) {
        PX4_ERR("write failed: %d (%s)", errno, strerror(errno));
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
    if (!_is_initialized && initialize_sensor() != PX4_OK) {
        PX4_ERR("Failed to initialize VL53L8_Distro sensor");
        perf_cancel(_sample_perf);
        stop();
        return;
    }

    get_sensors_resolution(); // Get the sensors resolution

    measure_single(); // Perform a single measurement

    // if (collect() != PX4_OK) {
    //     PX4_ERR("Failed to collect data");
    //     stop();
    //     return;
    // }

    ScheduleDelayed(2_s); // Schedule the next run after 2 seconds

    perf_end(_sample_perf);
}

void VL53L8_Distro::start()
{
    PX4_INFO("Starting VL53L8_Distro thread");
    // ScheduleNow();
    ScheduleNow(); // Start the scheduled work immediately
}

void VL53L8_Distro::stop()
{
    PX4_INFO("Stopping VL53L8_Distro measurements");
    _task_should_exit = true;



    // Ensure the serial port is closed.
	::close(_port_fd);
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
            msg.calculate_crc();
            ret = ::write(_port_fd, (uint8_t *)&msg, sizeof(msg));
            PX4_INFO("Wrote %d bytes to port %s for checking if it'a alive", ret, _serial_port);
            if (ret <= 0) {
                PX4_ERR("write failed: %d (%s)", errno, strerror(errno));
                perf_count(_comms_errors);
                return PX4_ERROR;
            }

            px4_usleep(200_us);

            ret = ::read(_port_fd, (uint8_t *)&msg, sizeof(msg));
            if (ret <= 0) {
                PX4_ERR("read failed: %d (%s)", errno, strerror(errno));
                perf_count(_comms_errors);
                // return PX4_ERROR;

            } else if (parse_command(msg, UART_PROT_CMD_STATUS_ACK) == false) {
                perf_count(_comms_errors);
                // return PX4_ERROR;
            } else {
                PX4_INFO("VL53L8_Distro is alive on port: %s", _serial_port);
                break; // Exit the loop if the sensor is alive
            }
            retry++;
            px4_sleep(1); // Wait for a second before retrying
        }

        msg.cmd = UART_PROT_CMD_SENSOR_INIT;
        msg.value = 0; // No specific value needed for this command
        msg.calculate_crc();
        // Send the sensor initialization command
        ret = ::write(_port_fd, (uint8_t *)&msg, sizeof(msg));
        PX4_INFO("Wrote %d bytes to port %s for sensor initialization", ret, _serial_port);
        if (ret <= 0) {
            PX4_ERR("write failed: %d (%s)", errno, strerror(errno));
            perf_count(_comms_errors);
            return PX4_ERROR;
        }

        retry = 0;

        while(true) {
            px4_sleep(1); // Allow some time for the sensor to initialize

            // Read the acknowledgment from the sensor
            ret = ::read(_port_fd, (uint8_t *)&msg, sizeof(msg));
            if (ret <= 0) {
                PX4_ERR("read failed: %d (%s)", errno, strerror(errno));
                perf_count(_comms_errors);
                // return PX4_ERROR;
            } else if (parse_command(msg, UART_PROT_CMD_STATUS_ACK) == false) {
                perf_count(_comms_errors);
                // return PX4_ERROR;
            } else {
                break; // Exit the loop if the sensor is initialized successfully
            }

            if(retry >= 5) {
                PX4_ERR("VL53L8_Distro initialization failed after multiple retries");
                perf_count(_comms_errors);
                return PX4_ERROR;
            }

            retry++;
        }

        PX4_INFO("VL53L8_Distro initialized successfully on port: %s [sensors: %d]", _serial_port, msg.value);
        this->_is_initialized = true;
    } else {
        PX4_INFO("VL53L8_Distro already initialized on port: %s", _serial_port);
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
    msg.calculate_crc();

    int ret = ::write(_port_fd, (uint8_t *)&msg, sizeof(msg));
    PX4_INFO("Wrote %d bytes to port %s for getting sensors resolution", ret, _serial_port);
    if (ret <= 0) {
        PX4_ERR("write failed: %d (%s)", errno, strerror(errno));
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    px4_usleep(200_us);

    // Read the sensors resolution result
    ret = ::read(_port_fd, (uint8_t *)&msg, sizeof(msg));
    if (ret <= 0) {
        PX4_ERR("read failed: %d (%s)", errno, strerror(errno));
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    if (parse_command(msg, UART_PROT_CMD_STATUS_ACK) == false) {
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    _sensors_resolution = msg.value; // Assuming value contains the resolution
    PX4_INFO("Sensors resolution: %d", _sensors_resolution);

    return PX4_OK;
}

int VL53L8_Distro::measure_single() {
    if(!_is_initialized) {
        PX4_ERR("VL53L8_Distro not initialized, cannot perform measurement");
        return PX4_ERROR;
    }

    CMD_short_s msg{};
    msg.cmd = UART_PROT_CMD_RNG_SINGLE;
    msg.value = 0; // No specific value needed for this command
    msg.calculate_crc();

    int ret = ::write(_port_fd, (uint8_t *)&msg, sizeof(msg));
    PX4_INFO("Wrote %d bytes to port %s for single measurement", ret, _serial_port);
    if (ret <= 0) {
        PX4_ERR("write failed: %d (%s)", errno, strerror(errno));
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    px4_usleep(200_us);

    // Read the measurement result
    ret = ::read(_port_fd, (uint8_t *)&msg, sizeof(msg));
    if (ret <= 0) {
        PX4_ERR("read failed: %d (%s)", errno, strerror(errno));
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    if (parse_command(msg, UART_PROT_CMD_STATUS_ACK) == false) {
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    // Process the measurement data here
    // ...

    return PX4_OK;
}

int VL53L8_Distro::open_serial_port(const speed_t speed) {
    if(_port_fd > 0) {
        // PX4_INFO("Port already open");
        return PX4_OK;
    }

    // Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

    // Open the serial port.
	_port_fd = ::open(_serial_port, flags);

    if (_port_fd < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

    if (!isatty(_port_fd)) {
        PX4_ERR("Port %s is not a valid TTY (not a typewriter)", _serial_port);
        ::close(_port_fd);
        _port_fd = -1;
        return PX4_ERROR;
    }

    termios uart_config;

	// Store the current port configuration. attributes.
	if (tcgetattr(_port_fd, &uart_config)) {
		PX4_ERR("Unable to get termios from %s.", _serial_port);
		::close(_port_fd);
		_port_fd = -1;
		return PX4_ERROR;
	}

	// Clear: data bit size, two stop bits, parity, hardware flow control.
	uart_config.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);

	// Set: 8 data bits, enable receiver, ignore modem status lines.
	uart_config.c_cflag |= (CS8 | CREAD | CLOCAL);

	// Clear: echo, echo new line, canonical input and extended input.
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	// Set the input baud rate in the uart_config struct.
	int termios_state = cfsetispeed(&uart_config, speed);

    if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_port_fd);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_port_fd);
		return PX4_ERROR;
	}

	// Apply the modified port attributes.
	termios_state = tcsetattr(_port_fd, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_port_fd);
		return PX4_ERROR;
	}

    // Flush the hardware buffers.
	tcflush(_port_fd, TCIOFLUSH);

	PX4_INFO("successfully opened UART port %s (%d)", _serial_port, _port_fd);
	return PX4_OK;
}

bool VL53L8_Distro::parse_command(const CMD_short_s &cmd, uint8_t expected_cmd) {
    if(cmd.header_1 != UART_PROT_MSG_HEADER_1 || cmd.header_2 != UART_PROT_MSG_HEADER_2) {
        PX4_ERR("Invalid command header");
        return false;
    }

    if(cmd.cmd != expected_cmd) {
        PX4_ERR("Unexpected command: 0X%02X, expected: 0X%02X", cmd.cmd, expected_cmd);
        return false;
    }

    if(cmd.crc != calculate_crc((uint8_t *)&cmd.packet_len, cmd.packet_len - UART_PROT_MSG_CRC_LEN)) {
        PX4_ERR("CRC mismatch, expected: 0x%04X, received: 0x%04X", cmd.crc, calculate_crc((uint8_t *)&cmd.packet_len, cmd.packet_len - UART_PROT_MSG_CRC_LEN));
        return false;
    }

    return true;
}

bool VL53L8_Distro::parse_command(const CMD_long_s &cmd, uint8_t expected_cmd) {
    if(cmd.header_1 != UART_PROT_MSG_HEADER_1 || cmd.header_2 != UART_PROT_MSG_HEADER_2) {
        PX4_ERR("Invalid command header");
        return false;
    }

    if(cmd.cmd != expected_cmd) {
        PX4_ERR("Unexpected command: 0X%02X, expected: 0X%02X", cmd.cmd, expected_cmd);
        return false;
    }

    if(cmd.crc != calculate_crc((uint8_t *)&cmd.packet_len, cmd.packet_len - UART_PROT_MSG_CRC_LEN)) {
        PX4_ERR("CRC mismatch, expected: 0x%04X, received: 0x%04X", cmd.crc, calculate_crc((uint8_t *)&cmd.packet_len, cmd.packet_len - UART_PROT_MSG_CRC_LEN));
        return false;
    }

    return true;
}
