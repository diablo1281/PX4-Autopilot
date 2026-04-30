/**
 * @file PAT9136.cpp
 * @author Jędrzej Szczepaniak <je.szczepaniak@gmail.com>
 *
 * Driver for the PixArt PAT9136 connected by I2C with custom Carrier Board.
 */

#include "PAT9136.hpp"
#include <lib/drivers/device/Device.hpp>

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

	device::Device::DeviceId device_id;
    device_id.devid_s.devtype = DRV_FLOW_DEV_TYPE_PAT9136;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_I2C;
	device_id.devid_s.address = PAT9136_I2C_PRODUCT_ID;
	this->_device_id = device_id.devid;

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

	this->_ux = sinf(math::radians(_param_sensor_angle.get()));
	this->_uy = cosf(math::radians(_param_sensor_angle.get()));

	this->kf_init();

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

		if(this->_optical_nav.timestamp == 0) {
			this->_optical_nav.new_data = true;
		} else {
			this->_optical_nav.new_data = this->_nav_data.new_data;
		}

		this->_optical_nav.dt_us = now - this->_optical_nav.timestamp;
		this->_optical_nav.sensor_id = this->_device_id;

		this->_optical_nav.x_sum = this->_nav_data.x_sum;
		this->_optical_nav.y_sum = this->_nav_data.y_sum;
		this->_optical_nav.x_delta = this->_nav_data.x_delta;
		this->_optical_nav.y_delta = this->_nav_data.y_delta;
		this->_optical_nav.squal = this->_nav_data.squal;
		this->_optical_nav.squal2 = this->_nav_data.squal2;
		this->_optical_nav.rawdata_min = this->_nav_data.rawdata_min;
		this->_optical_nav.rawdata_max = this->_nav_data.rawdata_max;
		this->_optical_nav.rawdata_sum = this->_nav_data.rawdata_sum;
		this->_optical_nav.shutter = this->_nav_data.shutter;
		this->_optical_nav.x_resolution = this->_x_resolution_cpi;
		this->_optical_nav.y_resolution = this->_y_resolution_cpi;



		// KF
		float dt_s = (float)(this->_optical_nav.dt_us) * 1e-6f;
		dt_s = math::constrain(dt_s, 0.8f/_param_rate_hz.get(), 1.2f/_param_rate_hz.get()); // constrain dt to reasonable values
		this->kf_predict(dt_s);

		if(this->_nav_data.new_data) {
			// position update
			const float px = (float)(this->_nav_data.x_sum) * this->_x_resolution_cpm_inv;
			const float py = (float)(this->_nav_data.y_sum) * this->_y_resolution_cpm_inv;

			const float z_ned = this->_ux * px + this->_uy * py;

			const float Rz = compute_Rz(this->_nav_data.squal,
										z_ned,
										this->v(),
										0.005f,          // sigma_z0_m
										0.0f,           // rv_percent
										0.0f,           // delay_s
										40);            // squal_min

			this->kf_update_pos(z_ned, Rz);
		} else {
			// velocity = 0 update, as no new data available
			const float kx = this->_ux * this->_x_resolution_cpm_inv;
			const float ky = this->_uy * this->_y_resolution_cpm_inv;
			const float z_thr = MOTION_THR_PX * sqrtf(kx * kx + ky * ky);
			const float v_thr = z_thr / dt_s;
			const float sigma_v = v_thr / 3.0f; // 3-sigma

			const float Rv = sigma_v * sigma_v;

			this->kf_update_vel(0.0f, Rv);
		}

		this->_optical_nav.z_m = this->z();
		this->_optical_nav.vz_m_s = this->v();
		this->_optical_nav.var_z_m2 = this->var_z();
		this->_optical_nav.var_vz_m2s2 = this->var_v();

		this->_optical_nav.timestamp = now;

		this->publish();
	}

	if(_error_counter >= 5) {
		PX4_ERR("Communication error - suspending task");
		this->_should_exit = true;
		return;
	}
}

void PAT9136_I2C::kf_init() {
	this->_x.zero();
	this->_P.setIdentity();
    this->_P(0,0) = 0.05f * 0.05f;   // np. 5 cm^2 startowo (konserwatywnie)
    this->_P(1,1) = 0.2f  * 0.2f;    // np. (0.2 m/s)^2
    this->_P(0,1) = 0.0f;
    this->_P(1,0) = 0.0f;
}

void PAT9136_I2C::kf_predict(float dt) {
	matrix::Matrix<float,2,2> F;
	F.setIdentity();
	F(0,1) = dt;

	const float dt2 = dt * dt;
	const float dt3 = dt2 * dt;
	const float dt4 = dt2 * dt2;
	const float sa2 = this->_sigma_a * this->_sigma_a;

	matrix::Matrix<float,2,2> Q;
	Q(0,0) = 0.25f * dt4 * sa2;
	Q(0,1) = 0.5f  * dt3 * sa2;
	Q(1,0) = 0.5f  * dt3 * sa2;
	Q(1,1) = dt2 * sa2;

	this->_x = F * this->_x;
	this->_P = F * this->_P * F.transpose() + Q;
}

bool PAT9136_I2C::kf_update_pos(float z, float R) {
	const float nu = z - this->_x(0);
	const float S  = this->_P(0,0) + R;
	if (S < 1e-9f) return false;

	matrix::Vector2f K;
	K(0) = this->_P(0,0) / S;
	K(1) = this->_P(1,0) / S;

	this->_x += K * nu;

	matrix::Matrix<float,2,2> I; I.setIdentity();
	matrix::Matrix<float,2,2> KH; KH.setZero();
	KH(0,0) = K(0);
	KH(1,0) = K(1);

	this->_P = (I - KH) * this->_P;
	return true;
}

bool PAT9136_I2C::kf_update_vel(float v, float R) {
	const float nu = v - this->_x(1);
	const float S  = this->_P(1,1) + R;
	if (S < 1e-9f) return false;

	matrix::Vector2f K;
	K(0) = this->_P(0,1) / S;
	K(1) = this->_P(1,1) / S;

	this->_x += K * nu;

	matrix::Matrix<float,2,2> I; I.setIdentity();
	matrix::Matrix<float,2,2> KH; KH.setZero();
	KH(0,1) = K(0);
	KH(1,1) = K(1);

	this->_P = (I - KH) * this->_P;
	return true;
}

float PAT9136_I2C::compute_Rz(uint8_t squal,
                        float z_ned_m,          // absolutne z w NED
                        float v_est_ned_mps,
                        float sigma_z0_m,
                        float rv_percent,       // 0.0f jeśli nie używasz RV
                        float delay_s,          // 0.0f jeśli nie używasz delay
                        uint8_t squal_min)
{
    if (squal < squal_min) {
        return 1e3f; // praktycznie pomija update
    }

    // SQUAL -> NoF
    const float NoF = (float)squal * 4.f;
    const float Nmin = 16.f;
    const float Nref = 256.f;

    const float sigma_noise = sigma_z0_m * sqrtf(Nref / fmaxf(NoF, Nmin));

    // RV: sigma_scale = alpha * |z|
    const float alpha = rv_percent / 200.f; // peak-to-peak % -> ~ +/-RV/2
    const float sigma_scale = alpha * fabsf(z_ned_m);

    // delay: sigma_delay = |v| * tau
    const float sigma_delay = fabsf(v_est_ned_mps) * fmaxf(delay_s, 0.f);

    const float sigma_total = sqrtf(sigma_noise*sigma_noise +
                                    sigma_scale*sigma_scale +
                                    sigma_delay*sigma_delay);

    return sigma_total * sigma_total;
}
