/*
 * PAT9136_i2c.h
 *
 *  Created on: 23.02.2024
 *      Author: JeSz
 */

////////////////////////////////
//	NIE UDOSTĘPNIAĆ!!!!!!!
////////////////////////////////

#ifndef DRIVERS_OPTICAL_FLOW_PAT9136_PAT9136_I2C_H_
#define DRIVERS_OPTICAL_FLOW_PAT9136_PAT9136_I2C_H_

#include <data_st/optical_navigation.h>
#include <drivers/general/i2c_driver.h>
#include <public_data_instance.h>


#define PAT9136_I2C_TASK_PRIORITY	4
#define PAT9136_I2C_TASK_STACKSIZE	(4*configMINIMAL_STACK_SIZE)

#define PAT9136_I2C_RES_X	10000
#define PAT9136_I2C_RES_Y	10000

#define PAT9136_I2C_ADDRESS	(0x4F << 1)

#define PAT9136_I2C_PRODUCT_ID		0x4F

#define PAT9136_I2C_CMD_PRODUCT_ID	0x00

#define PAT9136_I2C_CMD_GET_MOTION_DATA	0x01	// TBD

#define PAT9136_I2C_CMD_SET_RESOLUTION	0x02	// [CMD, X_H, X_L, Y_H, Y_L]
#define PAT9136_I2C_RESOLUTION_CPI_MIN	100
#define PAT9136_I2C_RESOLUTION_CPI_MAX	20000

#define PAT9136_I2C_CMD_POWER_UP_RESET		0x5A
#define PAT9136_I2C_CMD_SHUTDOWN			0xB6

#define PAT9136_I2C_CMD_ORIENTATION_CHANGE		0x5B	// [CMD, BYTE]
#define PAT9136_I2C_BYTE_ORIENTATION_SWAP_XY	(1 << 7)
#define PAT9136_I2C_BYTE_ORIENTATION_INV_Y		(1 << 6)
#define PAT9136_I2C_BYTE_ORIENTATION_INV_X		(1 << 5)
#define PAT9136_I2C_BYTE_ORIENTATION_DEFAULT	0

#define PAT9136_I2C_CMD_MATTE_TEXTURED_MODE	0x7F

#define PAT9136_I2C_CPI_TO_CPM(_x)		((_x) / 0.0254f)

class PAT9136_I2C {
public:
	PAT9136_I2C(I2C_driver *hi2c);
	virtual ~PAT9136_I2C();

	int os_start();
	void* thread(void *arg);

private:
	drv_status init();

	drv_status collect();

	drv_status power_up_reset();
	drv_status shutdown_sensor();

	drv_status set_matte_textured_mode();

	drv_status get_resolution();
	drv_status set_resolution_from_params();
	drv_status set_resolution(uint16_t x_cpi, uint16_t y_cpi);

	drv_status get_orientation(uint8_t &orientation);
	drv_status set_orientation(uint8_t orientation);

	drv_status write(uint8_t reg, uint8_t *data, size_t size = 1);
	drv_status write(uint8_t reg, uint8_t data) { return this->write(reg, &data, 1); }

	bool publish() { auto out = this->_optical_nav; return this->_pub->publish(&out); }

	TaskHandle_t	_htread{nullptr};

	I2C_driver	*_drv{nullptr};

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
	optical_navigation_s _optical_nav{};
	skynav::PublicDataInstance *_pub{nullptr};

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
	};
#pragma pack(pop)

	union {
		navigation_data_s _nav_data;
		uint8_t _nav_data_buff[sizeof(navigation_data_s)];
	};
};

#endif /* DRIVERS_OPTICAL_FLOW_PAT9136_PAT9136_I2C_H_ */
