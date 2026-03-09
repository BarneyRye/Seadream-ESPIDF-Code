#ifndef DATASTRUCT_H_
#define DATASTRUCT_H_

#include <stdint.h>

/* Structs for sensor data and calibration data */
typedef struct __attribute__((packed)){
    uint32_t lognum;
    uint64_t time_us;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint32_t pressure;
    uint32_t temperature;
} sensor_data_t;

typedef struct __attribute__((packed)){
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t address;
} bmp280_calib_data_t;

#endif /* DATASTRUCT_H_ */