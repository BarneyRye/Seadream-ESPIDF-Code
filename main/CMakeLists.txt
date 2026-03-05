idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_timer esp_driver_i2c esp_driver_spi fatfs sdmmc Datastruct BMP280 LSM6DSO32)