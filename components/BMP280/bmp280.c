#include "bmp280.h"
#include "bmp280_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define BMP280_I2C_TIMEOUT_MS 50

#define BMP280_I2C_ADDR_PWR 0x77
#define BMP280_I2C_ADDR_GND 0x76

bool bmp280_addr;

bool bmp280_checkID(i2c_master_dev_handle_t bmp280_handle) {
    /*
    Reads BMP280 ID
    If matches expected value, returns true. Otherwise, returns false.
    */
    uint8_t id;
    uint8_t id_reg = BMP280_ID_REG;
    if (i2c_master_transmit_receive(bmp280_handle, &id_reg, 1, &id, 1, BMP280_I2C_TIMEOUT_MS) != ESP_OK) {
        return false;
    }
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
    i2c_master_transmit(bmp280_handle, ctrl_data, 2, BMP280_I2C_TIMEOUT_MS);
    uint8_t conf_data[2] = {BMP280_CONFIG_REG, 
                           BMP280_CONFIG_FILTER_OFF | BMP280_CONFIG_STANDBY_0_5_MS};
    i2c_master_transmit(bmp280_handle, conf_data, 2, BMP280_I2C_TIMEOUT_MS); 
}

void bmp280_getCalibData(i2c_master_dev_handle_t bmp280_handle, bmp280_calib_data_t* calib_data) {
    /*
    Reads calibration data from BMP280
    Stores data in provided bmp280_calib_data_t struct pointer
    */
    uint8_t reg = BMP280_CALIB00_REG;
    uint8_t buf[24];
    if (i2c_master_transmit_receive(bmp280_handle, &reg, 1, buf, 24, BMP280_I2C_TIMEOUT_MS) != ESP_OK) {
        memset(calib_data, 0, sizeof(*calib_data));
        return;
    }

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

void check_bmp_address(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *bmp280_handle) {
    if (bmp280_checkID(*bmp280_handle)) {
        return;
    }
    if (bmp280_addr) {
        i2c_master_bus_rm_device(*bmp280_handle);
        i2c_device_config_t bmp280_dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = BMP280_I2C_ADDR_PWR,
            .scl_speed_hz = 400000,
        };
        i2c_master_bus_add_device(bus_handle, &bmp280_dev_config, bmp280_handle);
    }
    else {
        i2c_master_bus_rm_device(*bmp280_handle);
        i2c_device_config_t bmp280_dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = BMP280_I2C_ADDR_GND,
            .scl_speed_hz = 400000,
        };
        i2c_master_bus_add_device(bus_handle, &bmp280_dev_config, bmp280_handle);
    }
}

bool bmp280_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *bmp280_handle, bmp280_calib_data_t *calib_data) {
    /*
    BMP280 initialization function
    Tries to find BMP280 on both possible I2C addresses. If found, configures device and reads calibration data.
    Returns true if device found and initialized successfully, false otherwise.
    */
    bmp280_addr = false;
    uint8_t reset_cmd[2] = {BMP280_RESET_REG, BMP280_RESET_VAL};
    i2c_master_transmit(*bmp280_handle, reset_cmd, 2, BMP280_I2C_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(100));

    if (!bmp280_checkID(*bmp280_handle)) {
        i2c_master_bus_rm_device(*bmp280_handle);

        i2c_device_config_t bmp280_dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = BMP280_I2C_ADDR_GND,
            .scl_speed_hz = 400000,
        };

        i2c_master_bus_add_device(bus_handle, &bmp280_dev_config, bmp280_handle);

        i2c_master_transmit(*bmp280_handle, reset_cmd, 2, BMP280_I2C_TIMEOUT_MS);
        vTaskDelay(pdMS_TO_TICKS(100));
        bmp280_addr = true;
    }
    if (bmp280_checkID(*bmp280_handle)) {
        bmp280_config(*bmp280_handle);
        bmp280_getCalibData(*bmp280_handle, calib_data);
    }
    if (bmp280_addr) {
        calib_data->address = BMP280_I2C_ADDR_GND;
    }
    else {
        calib_data->address = BMP280_I2C_ADDR_PWR;
    }
    return bmp280_checkID(*bmp280_handle);
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