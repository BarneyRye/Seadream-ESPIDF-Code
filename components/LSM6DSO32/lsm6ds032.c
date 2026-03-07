#include "lsm6dso32.h"
#include "lsm6dso32_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

bool lsm6dso32_checkID(i2c_master_dev_handle_t lsm6dso32_handle) {
    uint8_t id;
    uint8_t id_reg = IMU_WHO_AM_I;
    i2c_master_transmit_receive(lsm6dso32_handle, &id_reg, 1, &id, 1, -1);
    return (id == IMU_WHO_AM_I_ID);
}

void lsm6dso32_config(i2c_master_dev_handle_t lsm6dso32_handle) {
    uint8_t ctrl1_xl[2] = {IMU_CTRL1_XL,
        IMU_CTRL1_LPF2_XL_EN_OFF | IMU_CTRL1_FS_XL_32g | IMU_CTRL1_ODR_XL_3333Hz
    };
    uint8_t ctrl2_g[2] = {IMU_CTRL2_G,
        IMU_CTRL2_FS_125_OFF | IMU_CTRL2_FS_G_2000dps | IMU_CTRL2_ODR_G_3333Hz
    };
    uint8_t ctrl3_c[2] = {IMU_CTRL3_C,
        IMU_CTRL3_BDU_ON | IMU_CTRL3_IF_INC_ON
    };
    uint8_t ctrl10_c[2] = {IMU_CTRL10_C,IMU_TIMESTAMP_EN_ON};
    i2c_master_transmit(lsm6dso32_handle, ctrl1_xl, 2, -1);
    i2c_master_transmit(lsm6dso32_handle, ctrl2_g, 2, -1);
    i2c_master_transmit(lsm6dso32_handle, ctrl3_c, 2, -1);
    i2c_master_transmit(lsm6dso32_handle, ctrl10_c, 2, -1);
}

bool lsm6dso32_init(i2c_master_dev_handle_t lsm6dso32_handle) {
    uint8_t reset_cmd[2] = {IMU_CTRL3_C, IMU_CTRL3_SW_RESET_ON};
    i2c_master_transmit(lsm6dso32_handle, reset_cmd, 2, -1);
    vTaskDelay(pdMS_TO_TICKS(100));
    lsm6dso32_config(lsm6dso32_handle);
    return lsm6dso32_checkID(lsm6dso32_handle);
}

void lsm6dso32_getEvent(i2c_master_dev_handle_t lsm6dso32_handle, sensor_data_t* data) {
    uint8_t data_start_reg = IMU_OUTX_L_G;
    uint8_t raw_data[12];
    if (i2c_master_transmit_receive(lsm6dso32_handle, &data_start_reg, 1, raw_data, 12, 20) == ESP_OK) {
        data->gyro_x = (int16_t)(raw_data[1] << 8 | raw_data[0]);
        data->gyro_y = (int16_t)(raw_data[3] << 8 | raw_data[2]);
        data->gyro_z = (int16_t)(raw_data[5] << 8 | raw_data[4]);
        data->acc_x = (int16_t)(raw_data[7] << 8 | raw_data[6]);
        data->acc_y = (int16_t)(raw_data[9] << 8 | raw_data[8]);
        data->acc_z = (int16_t)(raw_data[11] << 8 | raw_data[10]);
    }
    else {
        data->gyro_x = 0;
        data->gyro_y = 0;
        data->gyro_z = 0;
        data->acc_x = 0;
        data->acc_y = 0;
        data->acc_z = 0;
    }
}