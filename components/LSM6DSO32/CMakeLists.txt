idf_component_register(SRCS "lsm6ds032.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_driver_i2c Datastruct freertos)