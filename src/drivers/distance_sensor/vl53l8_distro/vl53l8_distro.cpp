#include "vl53l8_distro.hpp"

#include "uart_protocol.h"

#include <cerrno>

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include <lib/drivers/device/Device.hpp>

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
    PX4_INFO("Initializing VL53L8_Distro on port: %s", _serial_port);
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
    PX4_INFO("Running VL53L8_Distro loop...");
    // Ensure the serial port is open.
	if(open_serial_port() != PX4_OK) {
        PX4_ERR("Failed to open serial port");
        stop();
        return;
    }


    if (collect() != PX4_OK) {
        PX4_ERR("Failed to collect data");
        stop();
        return;
    }
    // Schedule next run if needed
    // ScheduleDelayed(1000_ms);
}

void VL53L8_Distro::start()
{
    PX4_INFO("Starting VL53L8_Distro measurements");
    // ScheduleNow();
    ScheduleOnInterval(1000_ms, 0);
}

void VL53L8_Distro::stop()
{
    PX4_INFO("Stopping VL53L8_Distro measurements");
    // Ensure the serial port is closed.
	::close(_port_fd);
    // Clear the work queue schedule.
	ScheduleClear();
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
