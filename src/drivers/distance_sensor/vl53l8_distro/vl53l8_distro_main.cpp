#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>

#include "vl53l8_distro.hpp"

/**
 * Local functions in support of the shell command.
 */
namespace vl53l8_distro
{

VL53L8_Distro	*g_dev;

int reset(const char *port, int baudrate);
int start(const char *port, int baudrate);
int status();
int stop();
int usage();

/**
 * Reset the driver.
 */
int
reset(const char *port, int baudrate)
{
	if (stop() == PX4_OK) {
		return start(port, baudrate);
	}

	return PX4_ERROR;
}

/**
 * Start the driver.
 */
int
start(const char *port, int baudrate)
{
	if (port == nullptr) {
		PX4_ERR("invalid port");
		return PX4_ERROR;
	}

	if (g_dev != nullptr) {
		PX4_INFO("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new VL53L8_Distro(port, baudrate);

	if (g_dev == nullptr) {
		PX4_ERR("object instantiate failed");
		return PX4_ERROR;
	}

	if (g_dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Print the driver status.
 */
int
status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->print_info();

	return PX4_OK;
}

/**
 * Stop the driver
 */
int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
}

int
usage()
{
	PX4_INFO("usage: vl53l8_distro command [options]");
	PX4_INFO("command:");
	PX4_INFO("\treset|start|status|stop");
	PX4_INFO("options:");
	PX4_INFO("\t-d --device_path");
	return PX4_OK;
}

} // namespace vl53l8_distro


/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int vl53l8_distro_main(int argc, char *argv[])
{
	const char *device_path = nullptr;
	int baudrate = 0;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	bool error_flag = false;

	while ((ch = px4_getopt(argc, argv, "b:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			if(px4_get_parameter_value(myoptarg, baudrate) != 0) {
				PX4_ERR("baudrate parsing failed");
				error_flag = true;
			}
			break;

		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option!");
			return vl53l8_distro::usage();
		}
	}

	if(error_flag) {
		return vl53l8_distro::usage();
	}

	if (myoptind >= argc) {
		return vl53l8_distro::usage();
	}

	// Reset the driver.
	if (!strcmp(argv[myoptind], "reset")) {
		return vl53l8_distro::reset(device_path, baudrate);
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		return vl53l8_distro::start(device_path, baudrate);
	}

	// Print driver information.
	if (!strcmp(argv[myoptind], "status")) {
		return vl53l8_distro::status();
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return vl53l8_distro::stop();
	}

	return vl53l8_distro::usage();
}

