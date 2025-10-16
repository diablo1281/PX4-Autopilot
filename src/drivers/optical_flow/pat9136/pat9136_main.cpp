#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include "PAT9136.hpp"

I2CSPIDriverBase *PAT9136_I2C::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
	PAT9136_I2C *instance = new PAT9136_I2C(config);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	return instance;
}

void PAT9136_I2C::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for the PAT9136 optical navigation sensor.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pat9136", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("optical_navigation");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(PAT9136_I2C_ADDRESS);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

}

extern "C" __EXPORT int pat9136_main(int argc, char *argv[])
{
	using ThisDriver = PAT9136_I2C;

	BusCLIArguments cli{true, false};
	cli.i2c_address = PAT9136_I2C_ADDRESS;
	cli.default_i2c_frequency = 400000;
	cli.custom1 = 1;

	const char *verb = cli.parseDefaultArguments(argc, argv);
	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_FLOW_DEV_TYPE_PAT9136);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
