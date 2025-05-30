/**
 * @file VL53L8_Distro.hpp
 *
 * Driver for the ST VL53L8 ToF Sensor Distro Board connected via Serial.
 */

#pragma once

#include <px4_log.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/SubscriptionInterval.hpp>

using namespace time_literals;

class VL53L8_Distro : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	/**
	 * Default Constructor
	 * @param port The serial port to open for communicating with the sensor.
	 * @param rotation The sensor rotation relative to the vehicle body.
	 */
	VL53L8_Distro(const char *port);
	~VL53L8_Distro() override;

	int init();
	void print_info();

private:
	/**
	 * Reads data from serial UART and places it into a buffer.
	 */
	int collect();

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

	void parameters_update();

	char _port[20] {};


	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::SENS_EN_VL53L8D>) _sens_en_vl53l8d,
		(ParamInt<px4::params::VL_D_RATE>) _vl_d_rate,
		(ParamInt<px4::params::VL_D_ORIENT>) _vl_d_orient,
		(ParamInt<px4::params::VL_D_YAW_ANGLE>) _vl_d_yaw_angle
	)

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
};
