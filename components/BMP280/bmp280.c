#include "bmp280.h"
#include "bmp280_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

bool bmp280_checkID(i2c_master_dev_handle_t bmp280_handle) {
    /*
    Reads BMP280 ID
    If matches expected value, returns true. Otherwise, returns false.
    */
    uint8_t id;
    uint8_t id_reg = BMP280_ID_REG;
    i2c_master_transmit_receive(bmp280_handle, &id_reg, 1, &id, 1, -1);
    return (id == BMP280_ID);
}

void bmp280_config(i2c_master_dev_handle_t bmp280_handle) {
    /*
    Sets config for BMP280
        - Normal mode
        - Pressure and temperature oversampling x1
        - Filter off
        - Standby time 0.5 ms
    */
    uint8_t ctrl_data[2] = {BMP280_CTRL_MEAS_REG, 
                           BMP280_CTRL_MEAS_MODE_NORMAL | BMP280_CTRL_MEAS_OSRS_P_X1 | BMP280_CTRL_MEAS_OSRS_T_X1};
    i2c_master_transmit(bmp280_handle, ctrl_data, 2, -1);
    uint8_t conf_data[2] = {BMP280_CONFIG_REG, 
                           BMP280_CONFIG_FILTER_OFF | BMP280_CONFIG_STANDBY_0_5_MS};
    i2c_master_transmit(bmp280_handle, conf_data, 2, -1); 
}

void bmp280_getCalibData(i2c_master_dev_handle_t bmp280_handle, bmp280_calib_data_t* calib_data) {
    /*
    Reads calibration data from BMP280
    Stores data in provided bmp280_calib_data_t struct pointer
    */
    uint8_t reg = BMP280_CALIB00_REG;
    uint8_t buf[24];
    i2c_master_transmit_receive(bmp280_handle, &reg, 1, buf, 24, -1);

    calib_data->dig_T1 = (uint16_t)(buf[1] << 8 | buf[0]);
    calib_data->dig_T2 = (int16_t)(buf[3] << 8 | buf[2]);
    calib_data->dig_T3 = (int16_t)(buf[5] << 8 | buf[4]);
    calib_data->dig_P1 = (uint16_t)(buf[7] << 8 | buf[6]);
    calib_data->dig_P2 = (int16_t)(buf[9] << 8 | buf[8]);
    calib_data->dig_P3 = (int16_t)(buf[11] << 8 | buf[10]);
    calib_data->dig_P4 = (int16_t)(buf[13] << 8 | buf[12]);
    calib_data->dig_P5 = (int16_t)(buf[15] << 8 | buf[14]);
    calib_data->dig_P6 = (int16_t)(buf[17] << 8 | buf[16]);
    calib_data->dig_P7 = (int16_t)(buf[19] << 8 | buf[18]);
    calib_data->dig_P8 = (int16_t)(buf[21] << 8 | buf[20]);
    calib_data->dig_P9 = (int16_t)(buf[23] << 8 | buf[22]);
}

bool bmp280_init(i2c_master_dev_handle_t bmp280_handle, bmp280_calib_data_t* calib_data) {
    /*
    Initializes BMP280
        - Resets device
        - Configures settings
        - Reads calibration data
        - Checks device ID
    */
    uint8_t reset_cmd[2] = {BMP280_RESET_REG, BMP280_RESET_VAL};
    i2c_master_transmit(bmp280_handle, reset_cmd, 2, -1);
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    bmp280_config(bmp280_handle);
    bmp280_getCalibData(bmp280_handle, calib_data);
    
    return bmp280_checkID(bmp280_handle);
}

void bmp280_getEvent(i2c_master_dev_handle_t bmp280_handle, sensor_data_t* data) {
    /*
    Reads pressure and temperature data from BMP280
    Stores data in provided sensor_data_t struct pointer
    */
    uint8_t reg = BMP280_PRESS_MSB_REG;
    uint8_t raw[6];
    
    if (i2c_master_transmit_receive(bmp280_handle, &reg, 1, raw, 6, 20) == ESP_OK) {
        data->pressure = (uint32_t)((raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4));
        data->temperature  = (uint32_t)((raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4));
    }
}