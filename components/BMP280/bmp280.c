#include "bmp280.h"
#include "bmp280_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

bool bmp280_checkID(i2c_master_dev_handle_t bmp280_handle) {
    uint8_t id;
    uint8_t id_reg = BMP280_ID_REG;
    i2c_master_transmit_receive(bmp280_handle, &id_reg, 1, &id, 1, -1);
    return (id == BMP280_ID);
}

void bmp280_config(i2c_master_dev_handle_t bmp280_handle) {
    uint8_t ctrl_data[2] = {BMP280_CTRL_MEAS_REG, 
                           BMP280_CTRL_MEAS_MODE_NORMAL | BMP280_CTRL_MEAS_OSRS_P_X1 | BMP280_CTRL_MEAS_OSRS_T_X1};
    i2c_master_transmit(bmp280_handle, ctrl_data, 2, -1);
    uint8_t conf_data[2] = {BMP280_CONFIG_REG, 
                           BMP280_CONFIG_FILTER_OFF | BMP280_CONFIG_STANDBY_0_5_MS};
    i2c_master_transmit(bmp280_handle, conf_data, 2, -1); 
}

void bmp280_getCalibData(i2c_master_dev_handle_t bmp280_handle, bmp280_calib_data_t* calib_data) {
uint8_t reg = BMP280_CALIB00_REG;
    i2c_master_transmit_receive(bmp280_handle, &reg, 1, (uint8_t*)calib_data, 24, -1);
}

bool bmp280_init(i2c_master_dev_handle_t bmp280_handle, bmp280_calib_data_t* calib_data) {
    uint8_t reset_cmd[2] = {BMP280_RESET_REG, BMP280_RESET_VAL};
    i2c_master_transmit(bmp280_handle, reset_cmd, 2, -1);
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    bmp280_config(bmp280_handle);
    bmp280_getCalibData(bmp280_handle, calib_data);
    
    return bmp280_checkID(bmp280_handle);
}

void bmp280_getEvent(i2c_master_dev_handle_t bmp280_handle, sensor_data_t* data) {
    uint8_t reg = BMP280_PRESS_MSB_REG;
    uint8_t raw[6];
    
    if (i2c_master_transmit_receive(bmp280_handle, &reg, 1, raw, 6, -1) == ESP_OK) {
        data->pressure = (uint32_t)((raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4));
        data->temperature  = (uint32_t)((raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4));
    }
}