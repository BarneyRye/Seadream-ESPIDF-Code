idf_component_register(SRCS "bmp280.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_driver_i2c Datastruct freertos)