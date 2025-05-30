#include "vl53l8_distro.hpp"

#include <lib/drivers/device/Device.hpp>

VL53L8_Distro::VL53L8_Distro(const char *port) :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
{
    if (port) {
	// Store the port name.
        strncpy(_port, port, sizeof(_port) - 1);
	// Enforce null termination.
        _port[sizeof(_port) - 1] = '\0';
    } else {
        _port[0] = '\0';
    }

    device::Device::DeviceId device_id;
    device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
    uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'
    if (bus_num < 10) {
	device_id.devid_s.bus = bus_num;
    }
}

VL53L8_Distro::~VL53L8_Distro()
{stop();

    perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int VL53L8_Distro::init()
{
    PX4_INFO("Initializing VL53L8_Distro on port: %s", _port);
    start();
    return PX4_OK;
}

void VL53L8_Distro::print_info()
{
    PX4_INFO("VL53L8_Distro info:");
    PX4_INFO("  Port: %s", _port);
    // Add more info as needed
}

int VL53L8_Distro::collect()
{
    // Placeholder for UART data collection logic
    PX4_DEBUG("Collecting data from sensor...");
    return PX4_OK;
}

void VL53L8_Distro::Run()
{
    if (collect() != PX4_OK) {
        PX4_ERR("Failed to collect data");
        stop();
        return;
    }
    // Schedule next run if needed
    ScheduleDelayed(1000_ms);
}

void VL53L8_Distro::start()
{
    PX4_INFO("Starting VL53L8_Distro measurements");
    ScheduleNow();
}

void VL53L8_Distro::stop()
{
    PX4_INFO("Stopping VL53L8_Distro measurements");
    // Clear the work queue schedule.
	ScheduleClear();
}

void VL53L8_Distro::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}
