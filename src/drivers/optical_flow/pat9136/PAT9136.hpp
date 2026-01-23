/**
 * @file PAT9136.hpp
 * @author Jędrzej Szczepaniak <je.szczepaniak@gmail.com>
 *
 * Driver for the PixArt PAT9136 connected by I2C with custom Carrier Board.
 */

#pragma once

#include <px4_log.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/device/i2c.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/optical_navigation_vertical.h>
#include <uORB/SubscriptionInterval.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
// #include <uORB/PublicationMulti.hpp>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/matrix/matrix/Vector2.hpp>
#include <lib/matrix/matrix/Matrix.hpp>

using namespace time_literals;


#define PAT9136_I2C_RES_X	10000
#define PAT9136_I2C_RES_Y	10000

#define PAT9136_I2C_ADDRESS	(0x4F << 0)

#define PAT9136_I2C_PRODUCT_ID		0x4F

#define PAT9136_I2C_CMD_PRODUCT_ID	0x00

#define PAT9136_I2C_CMD_GET_MOTION_DATA	0x01	// TBD

#define PAT9136_I2C_CMD_SET_RESOLUTION	0x03	// [CMD, X_H, X_L, Y_H, Y_L]
#define PAT9136_I2C_CMD_GET_RESOLUTION	0x04	// [CMD, X_H, X_L, Y_H, Y_L]
#define PAT9136_I2C_RESOLUTION_CPI_MIN	100
#define PAT9136_I2C_RESOLUTION_CPI_MAX	20000

#define PAT9136_I2C_CMD_ORIENTATION_CHANGE	0x05	// [CMD, BYTE]
#define PAT9136_I2C_CMD_GET_ORIENTATION		0x06	// [CMD]

#define PAT9136_I2C_BYTE_ORIENTATION_SWAP_XY	(1 << 7)
#define PAT9136_I2C_BYTE_ORIENTATION_INV_Y		(1 << 6)
#define PAT9136_I2C_BYTE_ORIENTATION_INV_X		(1 << 5)
#define PAT9136_I2C_BYTE_ORIENTATION_DEFAULT	0

#define PAT9136_I2C_CMD_POWER_UP_RESET		0x5A
#define PAT9136_I2C_CMD_SHUTDOWN			0xB6



#define PAT9136_I2C_CMD_MATTE_TEXTURED_MODE	0x7F

#define PAT9136_I2C_CPI_TO_CPM(_x)		((_x) / 0.0254f)

class PAT9136_I2C : device::I2C, public ModuleParams, public I2CSPIDriver<PAT9136_I2C> {
public:
	PAT9136_I2C(const I2CSPIDriverConfig &config);
	virtual ~PAT9136_I2C();

	void parameters_update();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	void	RunImpl();

	int 	init() override;

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void	print_status() override;

protected:
	int	probe() override;

private:
	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void	start();

	int	measure();
	int	collect();

	int power_up_reset();
	int shutdown_sensor();

	int set_matte_textured_mode();

	int get_resolution();
	int set_resolution(uint16_t x_cpi, uint16_t y_cpi);

	int get_orientation(uint8_t &orientation);
	int set_orientation(uint8_t orientation);

	int write(uint8_t reg, uint8_t *data, size_t size = 1);
	int write(uint8_t reg, uint8_t data) { return this->write(reg, &data, 1); }
	int read(uint8_t reg, uint8_t *data, size_t size = 1);

	bool publish() { auto out = this->_optical_nav; return this->_pub.publish(out); }

	// KF
	void kf_init();
	void kf_set_process_noise(float sigma_a) { this->_sigma_a = sigma_a; }
	void kf_predict(float dt);
	bool kf_update_pos(float z, float R);
	bool kf_update_vel(float v, float R);
	float compute_Rz(uint8_t squal,
                        float z_ned_m,          // absolutne z w NED
                        float v_est_ned_mps,
                        float sigma_z0_m,
                        float rv_percent,       // 0.0f jeśli nie używasz RV
                        float delay_s,          // 0.0f jeśli nie używasz delay
                        uint8_t squal_min);

	float z() const { return _x(0); }
    float v() const { return _x(1); }
	float var_z() const { return _P(0,0); }
	float var_v() const { return _P(1,1); }

	matrix::Vector2f			_x{};	// [z; vz]
	matrix::Matrix<float,2,2>	_P{};	// covariance matrix

	float	_sigma_a{1.5f};	// process noise acceleration stddev
	constexpr static float MOTION_THR_PX = 5.0f; // motion threshold in pixels
	float	_ux{0.0f};	// X axis contribution factor
	float	_uy{0.0f};	// Y axis contribution factor


	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t 		_collection_errors;
	perf_counter_t 		_measure_errors;

	bool	_initialized{false};
	bool	_should_exit{false};
	uint8_t	_error_counter{0};
	hrt_abstime _last_run{0};
	uint32_t	_device_id{0};

	union {
		struct {
			uint16_t	_x_resolution_cpi;
			uint16_t	_y_resolution_cpi;
		};
		uint8_t			_resolution_buffer_cpi[4];
	};

	float		_x_resolution_cpm_inv{-1.0f};
	float		_y_resolution_cpm_inv{-1.0f};

	uint8_t			_instance_id{0};
	optical_navigation_vertical_s _optical_nav{};
	uORB::Publication<optical_navigation_vertical_s> _pub{ORB_ID(optical_navigation_vertical)};

#pragma pack(push,1)
	struct navigation_data_s {
		int64_t x_sum;
		int64_t y_sum;
		int16_t x_delta;
		int16_t y_delta;
		uint16_t shutter;
		uint8_t squal;
		uint8_t squal2;
		uint8_t rawdata_sum;
		uint8_t rawdata_min;
		uint8_t rawdata_max;
		bool	new_data;
	};
#pragma pack(pop)

	union {
		navigation_data_s _nav_data;
		uint8_t _nav_data_buff[sizeof(navigation_data_s)];
	};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_PAT9136>) _param_sens_en,
		(ParamFloat<px4::params::PAT9136_RATE_HZ>) _param_rate_hz,
		(ParamFloat<px4::params::PAT9136_X_CPI>) _param_x_cpi,
		(ParamFloat<px4::params::PAT9136_Y_CPI>) _param_y_cpi,
		(ParamInt<px4::params::PAT9136_ORIENT>) _param_orient,
		(ParamBool<px4::params::PAT9136_TEXTURE>) _param_texture,
		(ParamFloat<px4::params::PAT9136_ANGLE>) _param_sensor_angle
	)

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
};
