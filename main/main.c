/* Includes */
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "datastruct.h"
#include "bmp280.h"
#include "lsm6dso32.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <unistd.h>

/* I2C Defines */
#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 13
#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 1000000

/* SPI Defines */
#define SD_MOUNT_POINT "/sdcard"
#define SPI_SCK_PIN 2
#define SPI_MOSI_PIN 3
#define SPI_MISO_PIN 1
#define SPI_CS_PIN 4

/* I2C device addresses*/
#define BMP280_I2C_ADDR_PWR 0x77
#define BMP280_I2C_ADDR_GND 0x76
#define LSM6DSO32_I2C_ADDR 0x6A

/* I2C Handles */
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t bmp280_dev_handle;
i2c_master_dev_handle_t lsm6dso32_dev_handle;

/* Buffer size for sensor data logging and calib structure*/
#define BUFFER_SIZE (512/sizeof(sensor_data_t)) // 512/32 = 16, to fit SD RAM buffer size
bmp280_calib_data_t bmp280_calib_data;

/* SD card struc and filename */
char filename[32];
sdmmc_card_t *card;

/* Queue Handle */
QueueHandle_t sensor_queue;

/* Function prototypes */
void i2c_init(void);
void sd_card_init(void);
void getNextFilename(char *filename, char *calib_filename);
void logCalibData(const char *calib_filename, bmp280_calib_data_t *calib_data);
void sensor_task(void *pvParameters);
void logging_task(void *pvParameters);

/* Main application entry point */
void app_main(void){
    /*
    Initializes I2C, SD card, sensors, and creates FreeRTOS tasks for sensor reading and SD logging
    Also stores BMP280 calibration data to SD card and gets next available filename for logging
    */
    i2c_init();

    bmp280_init(bus_handle, &bmp280_dev_handle, &bmp280_calib_data);
    lsm6dso32_init(lsm6dso32_dev_handle);

    sd_card_init();
    char calib_filename[32];
    getNextFilename(filename, calib_filename);
    logCalibData(calib_filename, &bmp280_calib_data);

    sensor_queue = xQueueCreate(2, sizeof(sensor_data_t)*BUFFER_SIZE);

    xTaskCreatePinnedToCore(sensor_task, "Sensor task", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(logging_task, "SD logging task", 4096, NULL, 5, NULL, 1);
}

/* FreeRTOS Tasks */
void sensor_task(void *pvParameters){
    /*
    Sensor task to read data and send over queue
    Also handles BMP280 data reading for the first buffer element
    */
    uint32_t lognum = 0;
    static sensor_data_t sensor_buffer[BUFFER_SIZE];
    while(1){
        for (uint8_t i = 0; i<BUFFER_SIZE; i++) {
            lsm6dso32_getEvent(lsm6dso32_dev_handle, &sensor_buffer[i]);
            sensor_buffer[i].time_us = esp_timer_get_time();
            if (i == 0) {
                bmp280_getEvent(bmp280_dev_handle, &sensor_buffer[i]);
            }
            else {
                sensor_buffer[i].pressure = sensor_buffer[0].pressure;
                sensor_buffer[i].temperature = sensor_buffer[0].temperature;
            }
            sensor_buffer[i].lognum = lognum++;
        }
        xQueueSend(sensor_queue, sensor_buffer, portMAX_DELAY);
    }
}

void logging_task(void *pvParameters){
    /*
    Logging task to write sensor data from queue to SD card
    Also handles periodic syncing to ensure data integrity
    */
    FILE *f = fopen(filename, "wb");
    if (f == NULL) {
        vTaskDelete(NULL);
        return;
    }
    static sensor_data_t logging_buffer[BUFFER_SIZE];
    uint32_t last_sync = xTaskGetTickCount();

    while (1) {
        if (xQueueReceive(sensor_queue, logging_buffer, portMAX_DELAY) == pdPASS) {
            fwrite(logging_buffer, sizeof(sensor_data_t), BUFFER_SIZE, f);
            if (xTaskGetTickCount() - last_sync >= pdMS_TO_TICKS(100)) {
                fsync(fileno(f));
                last_sync = xTaskGetTickCount();
            }
        }
    }
}

/* Helper functions */
void i2c_init(){
    /* Initializes I2C master bus and adds BMP280 and LSM6DSO32 devices */
    i2c_master_bus_config_t i2c_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_config, &bus_handle));
    
    i2c_device_config_t bmp280_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = BMP280_I2C_ADDR_PWR,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &bmp280_dev_config, &bmp280_dev_handle));

    i2c_device_config_t lsm6dso32_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = LSM6DSO32_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &lsm6dso32_dev_config, &lsm6dso32_dev_handle));
}

void sd_card_init(){
    /* Initializes SD card and mounts filesystem */
    esp_err_t ret;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 2,
        .allocation_unit_size = 32*1024
    };
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SPI_CS_PIN;
    slot_config.host_id = SPI2_HOST;

    ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    ESP_ERROR_CHECK(ret);
}

void getNextFilename(char *filename, char *calib_filename){
    /* Gets the next available filename for logging and calibration data */
    uint8_t file_index = 0;
    while (file_index < 99 ){
        char temp_filename[32];
        sprintf(temp_filename, SD_MOUNT_POINT "/log%02d.bin", file_index);
        FILE *f = fopen(temp_filename, "r");
        if (f == NULL) {
            strcpy(filename, temp_filename);
            sprintf(calib_filename, SD_MOUNT_POINT "/calib%02d.csv", file_index);
            return;
        }
        fclose(f);
        file_index++;
    }
    strcpy(filename, SD_MOUNT_POINT "/log99.bin");
    strcpy(calib_filename, SD_MOUNT_POINT "/calib99.csv");
    return;
}

void logCalibData(const char *calib_filename, bmp280_calib_data_t *calib_data){
    /* Logs BMP280 calibration data to specified CSV file */
    FILE *f = fopen(calib_filename, "w");
    if (f == NULL) return;
    fprintf(f, "%04hx,%04hx,%04hx\n", 
            calib_data->dig_T1, calib_data->dig_T2, calib_data->dig_T3);
            
    fprintf(f, "%04hx,%04hx,%04hx,%04hx,%04hx,%04hx,%04hx,%04hx,%04hx\n",
            calib_data->dig_P1, calib_data->dig_P2, calib_data->dig_P3,
            calib_data->dig_P4, calib_data->dig_P5, calib_data->dig_P6,
            calib_data->dig_P7, calib_data->dig_P8, calib_data->dig_P9);
    
    fclose(f);
}