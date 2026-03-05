#ifndef BMP280_H_
#define BMP280_H_

#include "driver/i2c_master.h"
#include "datastruct.h"
#include <stdbool.h>

bool bmp280_init(i2c_master_dev_handle_t bmp280_handle, bmp280_calib_data_t* calib_data);
void bmp280_getEvent(i2c_master_dev_handle_t bmp280_handle, sensor_data_t* data);

#endif /* BMP280_H_ */