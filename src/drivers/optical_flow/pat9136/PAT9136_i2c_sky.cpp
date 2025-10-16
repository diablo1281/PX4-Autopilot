/*
 * PAT9136_i2c.cpp
 *
 *  Created on: 23.02.2024
 *      Author: JeSz
 */

////////////////////////////////
//	NIE UDOSTĘPNIAĆ!!!!!!!
////////////////////////////////

#include <main.h>
#include "PAT9136_i2c.h"
#include <public_data.h>
#include <string.h>

TaskFunction_t PAT9136_I2C_task(void* param);

PAT9136_I2C::PAT9136_I2C(I2C_driver *hi2c): _drv(hi2c) {
	if(this->_drv == nullptr) Error_Handler();
	this->_x_resolution_cpi = UINT16_MAX;
	this->_y_resolution_cpi = UINT16_MAX;
	memset(this->_nav_data_buff, 0, sizeof(navigation_data_s));
}

PAT9136_I2C::~PAT9136_I2C() {
	if(this->_htread != nullptr)
		vTaskDelete(this->_htread);
}

int PAT9136_I2C::os_start() {
	if((this->_pub = skynav::pub_optical_navigation->create(this->_instance_id)) == nullptr) Error_Handler();

	if(xTaskCreate((TaskFunction_t)PAT9136_I2C_task, "PAT9136_I2C", PAT9136_I2C_TASK_STACKSIZE, (void *)this, PAT9136_I2C_TASK_PRIORITY, &this->_htread) != pdPASS)
		return SKY_ERROR;

	return SKY_OK;
}

drv_status PAT9136_I2C::init() {
	uint8_t WAI = 0xFF;
	drv_status ret = drv_status_ERR;
	ret = this->_drv->read(PAT9136_I2C_CMD_PRODUCT_ID, &WAI, PAT9136_I2C_ADDRESS);

	if(ret != drv_status_OK) return ret;

	if(WAI != PAT9136_I2C_PRODUCT_ID) {
		return drv_status_ERR;
	}

	ret = this->power_up_reset();
	if(ret != drv_status_OK) return ret;

	vTaskDelay(1000);

	// TODO: set from params
	ret = this->set_resolution(PAT9136_I2C_RES_X, PAT9136_I2C_RES_Y);
	if(ret != drv_status_OK) return ret;
	ret = this->set_orientation(PAT9136_I2C_BYTE_ORIENTATION_DEFAULT);
	if(ret != drv_status_OK) return ret;
	ret = this->get_resolution();
	if(ret != drv_status_OK) return ret;

	return ret;
}

drv_status PAT9136_I2C::collect() {
	drv_status ret = this->_drv->read(PAT9136_I2C_CMD_GET_MOTION_DATA,
			this->_nav_data_buff,
			sizeof(this->_nav_data_buff),
			PAT9136_I2C_ADDRESS);
	return ret;
}

drv_status PAT9136_I2C::power_up_reset() {
	return this->write(PAT9136_I2C_CMD_POWER_UP_RESET, nullptr, 0);
}

drv_status PAT9136_I2C::shutdown_sensor() {
	return this->write(PAT9136_I2C_CMD_SHUTDOWN, nullptr, 0);
}

drv_status PAT9136_I2C::set_matte_textured_mode() {
	return this->write(PAT9136_I2C_CMD_MATTE_TEXTURED_MODE, nullptr, 0);
}

drv_status PAT9136_I2C::get_resolution() {
	return this->_drv->read(PAT9136_I2C_CMD_SET_RESOLUTION,
			this->_resolution_buffer_cpi,
			sizeof(this->_resolution_buffer_cpi),
			PAT9136_I2C_ADDRESS);
}

drv_status PAT9136_I2C::set_resolution_from_params() {
	return drv_status_NOT_IMPLEMENTED;
}

drv_status PAT9136_I2C::set_resolution(uint16_t x_cpi, uint16_t y_cpi) {
	this->_x_resolution_cpi = x_cpi;
	this->_y_resolution_cpi = y_cpi;
	this->_x_resolution_cpm_inv = 1.0f / PAT9136_I2C_CPI_TO_CPM((float)x_cpi);
	this->_y_resolution_cpm_inv = 1.0f / PAT9136_I2C_CPI_TO_CPM((float)y_cpi);
	return this->write(PAT9136_I2C_CMD_SET_RESOLUTION,
			this->_resolution_buffer_cpi,
			sizeof(this->_resolution_buffer_cpi));
}

drv_status PAT9136_I2C::get_orientation(uint8_t &orientation) {
	drv_status ret = this->_drv->read(PAT9136_I2C_CMD_ORIENTATION_CHANGE, &orientation, PAT9136_I2C_ADDRESS);
	if(ret == drv_status_OK) {
		this->_x_resolution_cpm_inv = 1.0f / PAT9136_I2C_CPI_TO_CPM((float)this->_x_resolution_cpi);
		this->_y_resolution_cpm_inv = 1.0f / PAT9136_I2C_CPI_TO_CPM((float)this->_y_resolution_cpi);
	}
	return ret;
}

drv_status PAT9136_I2C::set_orientation(uint8_t orientation) {
	return this->write(PAT9136_I2C_CMD_ORIENTATION_CHANGE, orientation);
}

drv_status PAT9136_I2C::write(uint8_t reg, uint8_t *data, size_t size) {
	uint8_t buf[size + 1] = { reg };
	for (; size > 0; size--)
		buf[size] = data[size - 1];

	return this->_drv->write(buf, sizeof(buf), PAT9136_I2C_ADDRESS);
}

void* PAT9136_I2C::thread(void *arg) {
	if(this->init() != drv_status_OK) {
		SKY_SEND_INFO("Device not found");
		vTaskSuspend(this->_htread);
	}

	SKY_SEND_INFO("Sensor found [0x%2X]", PAT9136_I2C_PRODUCT_ID);

	uint8_t error_counter = 0;
	TickType_t last_wake = xTaskGetTickCount();
	while(true) {
		if(this->collect() != drv_status_OK) {
			error_counter++;
		} else {
			error_counter = 0;
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

			optical_navigation_s nav = this->_optical_nav;
			this->_pub->publish(&nav);
		}

		if(error_counter >= 5) {
			SKY_SEND_INFO("Communication error - suspending task");
			vTaskSuspend(this->_htread);
		}

		vTaskDelayUntil(&last_wake, 20);
	}
}

TaskFunction_t PAT9136_I2C_task(void* param) {
	static_cast<PAT9136_I2C *>(param)->thread(0);
	return 0;
}
