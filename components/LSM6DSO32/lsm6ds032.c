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
    uint8_t fifo_ctrl3[2] = {IMU_FIFO_CTRL3,
        IMU_FIFO_CTRL3_BDR_GY_ODR_833Hz |
        IMU_FIFO_CTRL3_BDR_XL_ODR_833Hz
    };
    uint8_t fifo_ctrl4[2] ={IMU_FIFO_CTRL4, 
        IMU_FIFO_CTRL4_DEC_TS_BATCH_1 |
        IMU_FIFO_CTRL4_ODR_T_BATCH_OFF  |
        IMU_FIFO_CTRL4_FIFO_MODE_CONTINOUS
    };
    uint8_t ctrl1_xl[2] = {IMU_CTRL1_XL,
        IMU_CTRL1_LPF2_XL_EN_OFF |
        IMU_CTRL1_FS_XL_32g |
        IMU_CTRL1_ODR_XL_833Hz

    };
    uint8_t ctrl2_g[2] = {IMU_CTRL2_G,
        IMU_CTRL2_FS_G_2000dps |
        IMU_CTRL2_ODR_G_833Hz
    };
    uint8_t ctrl3_c[2] = {IMU_CTRL3_C,
        IMU_CTRL3_BDU_ON |
        IMU_CTRL3_IF_INC_ON
    };
    uint8_t ctrl10_c[2] = {IMU_CTRL10_C,
        IMU_TIMESTAMP_EN_ON
    };
    i2c_master_transmit(lsm6dso32_handle, fifo_ctrl3, 2, -1);
    i2c_master_transmit(lsm6dso32_handle, fifo_ctrl4, 2, -1);
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
    uint8_t attempts = 0;
    uint8_t fifo_reg = IMU_FIFO_DATA_OUT_TAG;
    uint8_t raw[7];
    bool got_xl = false, got_gy = false, got_ts = false;
    data->time_us = 0;
    while (!(got_xl && got_gy && got_ts)) {
        if (i2c_master_transmit_receive(lsm6dso32_handle, &fifo_reg, 1, raw, 7, -1) != ESP_OK) return;
        uint8_t tag = raw[0] >> 3;
        if (tag == 0x00) vTaskDelay(pdMS_TO_TICKS(1));
        if (tag == 0x02) {
            data->acc_x = (int16_t)((raw[2] << 8) | raw[1]);
            data->acc_y = (int16_t)((raw[4] << 8) | raw[3]);
            data->acc_z = (int16_t)((raw[6] << 8) | raw[5]);
            got_xl = true;
        } else if (tag == 0x01) {
            data->gyro_x = (int16_t)((raw[2] << 8) | raw[1]);
            data->gyro_y = (int16_t)((raw[4] << 8) | raw[3]);
            data->gyro_z = (int16_t)((raw[6] << 8) | raw[5]);
            got_gy = true;
        } else if (tag == 0x04) {
            uint32_t ticks = ((uint32_t)raw[4] << 24) | ((uint32_t)raw[3] << 16) | 
                             ((uint32_t)raw[2] << 8)  | raw[1];
            data->time_us = (uint64_t)ticks * 25;
            got_ts = true; 
        }
        if (attempts++ >100) {
            data->acc_x = data->acc_y = data->acc_z = 0;
            data->gyro_x = data->gyro_y = data->gyro_z = 0;
            data->time_us = 0;
            return;
        }
    }
}