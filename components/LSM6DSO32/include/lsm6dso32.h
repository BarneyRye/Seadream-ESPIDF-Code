#ifndef LSM6DSO32_H_
#define LSM6DSO32_H_

#include "driver/i2c_master.h"
#include "datastruct.h"
#include <stdbool.h>

bool lsm6dso32_init(i2c_master_dev_handle_t lsm6dso32_handle);
void lsm6dso32_getEvent(i2c_master_dev_handle_t lsm6dso32_handle, sensor_data_t* data);

#endif /* LSM6DSO32_H_ */