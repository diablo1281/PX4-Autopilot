/**
 * @file PAT9136.cpp
 * @author Jędrzej Szczepaniak <je.szczepaniak@gmail.com>
 *
 * Driver for the PixArt PAT9136 connected by I2C with custom Carrier Board.
 */

#include "PAT9136.hpp"

PAT9136_I2C::PAT9136_I2C(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config),
	_sample_perf(perf_alloc(PC_ELAPSED, "pat9136_read")),
	_comms_errors(perf_alloc(PC_COUNT, "pat9136_com_err")),
	_collection_errors(perf_alloc(PC_COUNT, "pat9136_collection_err")),
	_measure_errors(perf_alloc(PC_COUNT, "pat9136_measurement_err"))
{
	_pub.advertise();

	memset(&this->_optical_nav, 0, sizeof(this->_optical_nav));
}

PAT9136_I2C::~PAT9136_I2C()
{
	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_collection_errors);
	perf_free(_measure_errors);
}

void PAT9136_I2C::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}

int PAT9136_I2C::write(uint8_t reg, uint8_t *data, size_t size) {
	uint8_t buf[size + 1] = { reg };
	for (; size > 0; size--)
		buf[size] = data[size - 1];

	int ret = transfer(buf, sizeof(buf), nullptr, 0);

	if(ret != PX4_OK) {
		// PX4_ERR("write transfer returned %d, reg: 0x%02X", ret, reg);
		perf_count(_comms_errors);
	}

	return ret;
}

int PAT9136_I2C::read(uint8_t reg, uint8_t *data, size_t size) {
	int ret = transfer(&reg, 1, data, size);

	if(ret != PX4_OK) {
		// PX4_ERR("read transfer returned %d, data: 0x%02X", ret, data[0]);
		perf_count(_comms_errors);
	} else {
		// PX4_INFO("read: 0x%02X", data[0]);
	}

	return ret;
}

int PAT9136_I2C::init() {
	uint8_t WAI = 0xFF;
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != PX4_OK) {
		return ret;
	}

	ret = this->read(PAT9136_I2C_CMD_PRODUCT_ID, &WAI);

	if(ret != PX4_OK) return ret;

	if(WAI != PAT9136_I2C_PRODUCT_ID) {
		return PX4_ERROR;
	}

	ret = this->power_up_reset();
	if(ret != PX4_OK) return ret;

	px4_usleep(1_s);

	_x_resolution_cpi = (uint16_t)_param_x_cpi.get();
	_y_resolution_cpi = (uint16_t)_param_y_cpi.get();

	ret = this->set_resolution(_x_resolution_cpi, _y_resolution_cpi);
	if(ret != PX4_OK) return ret;
	px4_usleep(5_ms);
	ret = this->set_orientation((uint8_t)_param_orient.get());
	if(ret != PX4_OK) return ret;
	px4_usleep(5_ms);
	ret = this->get_resolution();
	if(ret != PX4_OK) return ret;

	if(_param_texture.get()) {
		px4_usleep(5_ms);
		ret = this->set_matte_textured_mode();
		if(ret != PX4_OK) return ret;
	}

	_initialized = true;

	start();

	return ret;
}

int PAT9136_I2C::collect() {
	return this->read(PAT9136_I2C_CMD_GET_MOTION_DATA,
			this->_nav_data_buff,
			sizeof(this->_nav_data_buff));
}

int PAT9136_I2C::power_up_reset() {
	return this->write(PAT9136_I2C_CMD_POWER_UP_RESET, nullptr, 0);
}

int PAT9136_I2C::shutdown_sensor() {
	return this->write(PAT9136_I2C_CMD_SHUTDOWN, nullptr, 0);
}

int PAT9136_I2C::set_matte_textured_mode() {
	return this->write(PAT9136_I2C_CMD_MATTE_TEXTURED_MODE, nullptr, 0);
}

int PAT9136_I2C::get_resolution() {
	int ret = this->read(PAT9136_I2C_CMD_GET_RESOLUTION,
			this->_resolution_buffer_cpi,
			sizeof(this->_resolution_buffer_cpi));

	if(ret == PX4_OK) {
		this->_x_resolution_cpm_inv = 1.0f / PAT9136_I2C_CPI_TO_CPM((float)this->_x_resolution_cpi);
		this->_y_resolution_cpm_inv = 1.0f / PAT9136_I2C_CPI_TO_CPM((float)this->_y_resolution_cpi);
	}

	return ret;
}

int PAT9136_I2C::set_resolution(uint16_t x_cpi, uint16_t y_cpi) {
	this->_x_resolution_cpi = x_cpi;
	this->_y_resolution_cpi = y_cpi;
	this->_x_resolution_cpm_inv = 1.0f / PAT9136_I2C_CPI_TO_CPM((float)x_cpi);
	this->_y_resolution_cpm_inv = 1.0f / PAT9136_I2C_CPI_TO_CPM((float)y_cpi);
	return this->write(PAT9136_I2C_CMD_SET_RESOLUTION,
			this->_resolution_buffer_cpi,
			sizeof(this->_resolution_buffer_cpi));
}

int PAT9136_I2C::get_orientation(uint8_t &orientation) {
	int ret = this->read(PAT9136_I2C_CMD_GET_ORIENTATION, &orientation);
	return ret;
}

int PAT9136_I2C::set_orientation(uint8_t orientation) {
	return this->write(PAT9136_I2C_CMD_ORIENTATION_CHANGE, orientation);
}

void PAT9136_I2C::start() {
	ScheduleClear();
	_should_exit = false;
	uint32_t interval_us = (uint32_t)(1000000.0f / _param_rate_hz.get());
	PX4_INFO("Starting with interval %lu ms", interval_us / 1000);
	PX4_INFO("Resolution: %u x %u CPI", _x_resolution_cpi, _y_resolution_cpi);
	PX4_INFO("Orientation: 0x%02X", (uint8_t)_param_orient.get());
	PX4_INFO("Texture mode: %s", _param_texture.get() ? "ON" : "OFF");
	ScheduleOnInterval(interval_us);
}

void PAT9136_I2C::print_status() {
	I2CSPIDriverBase::print_status();

	if (_initialized) {
		perf_print_counter(_sample_perf);
		perf_print_counter(_comms_errors);

		// printf("poll interval:  %u \n", _measure_interval);
	} else {
		PX4_INFO("Device not initialized.");
	}
}

int PAT9136_I2C::probe() {
	uint8_t WAI = 0xFF;
	if(this->read(PAT9136_I2C_CMD_PRODUCT_ID, &WAI) != PX4_OK) {
		// PX4_ERR("Failed to read WAI");
		return PX4_ERROR;
	}
	if(WAI != PAT9136_I2C_PRODUCT_ID) {
		// PX4_ERR("WAI mismatch: 0x%02X", WAI);
		return PX4_ERROR;
	}
	// PX4_INFO("WAI: 0x%02X", WAI);
	return PX4_OK;
}

void PAT9136_I2C::RunImpl() {
	if(this->_should_exit) {
		PX4_INFO("Shutting down task");
		ScheduleClear();
		return;
	}

	if(!_initialized) {
		PX4_INFO("Initializing sensor");
		if(this->init() != PX4_OK) {
			PX4_INFO("Device not found");
			this->_should_exit = true;
			return;
		} else {
			PX4_INFO("Sensor found [0x%2X]", PAT9136_I2C_PRODUCT_ID);
			_initialized = true;
		}
	}

	if(this->collect() != PX4_OK) {
		_error_counter++;
	} else {
		_error_counter = 0;
		hrt_abstime now = hrt_absolute_time();
		this->_optical_nav.dt_us = now - this->_optical_nav.timestamp;
		float dt_s_inv = 1.0f / (this->_optical_nav.dt_us * 1.0e-6f);
		this->_optical_nav.x_speed_m_s = (float)(this->_nav_data.x_sum - this->_optical_nav.x_sum) * this->_x_resolution_cpm_inv * dt_s_inv;
		this->_optical_nav.y_speed_m_s = (float)(this->_nav_data.y_sum - this->_optical_nav.y_sum) * this->_y_resolution_cpm_inv * dt_s_inv;
		this->_optical_nav.x_sum = this->_nav_data.x_sum;
		this->_optical_nav.y_sum = this->_nav_data.y_sum;
		this->_optical_nav.squal = this->_nav_data.squal;
		this->_optical_nav.squal2 = this->_nav_data.squal2;
		this->_optical_nav.rawdata_min = this->_nav_data.rawdata_min;
		this->_optical_nav.rawdata_max = this->_nav_data.rawdata_max;
		this->_optical_nav.rawdata_sum = this->_nav_data.rawdata_sum;
		this->_optical_nav.shutter = this->_nav_data.shutter;
		this->_optical_nav.x_resolution = this->_x_resolution_cpi;
		this->_optical_nav.y_resolution = this->_y_resolution_cpi;
		this->_optical_nav.sensor_id = PAT9136_I2C_PRODUCT_ID;

		this->_optical_nav.timestamp = now;

		this->publish();
	}

	if(_error_counter >= 5) {
		PX4_ERR("Communication error - suspending task");
		this->_should_exit = true;
		return;
	}
}
