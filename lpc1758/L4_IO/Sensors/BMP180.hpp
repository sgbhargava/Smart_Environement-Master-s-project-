/*
 * BMP180.h
 *
 *  Created on: Jun 5, 2016
 *      Author: Christopher
 */

#ifndef L4_IO_SENSORS_BMP180_H_
#define L4_IO_SENSORS_BMP180_H_
#include <stdint.h>
#include "SensorData.hpp"

void bmp180_get_cal_param(uint8_t deviceAddr, int16_t * AC1, int16_t * AC2, int16_t * AC3,
                            uint16_t * AC4, uint16_t * AC5, uint16_t * AC6,
                            int16_t * B1, int16_t * B2, int16_t * MB,
                            int16_t * MC, int16_t * MD);

void bmp180_get_ut(uint8_t deviceAddr, int32_t * UT);
void bmp180_get_up(uint8_t deviceAddr, int32_t * UP);
void bmp180_get_temperature(int32_t UT, uint16_t AC6, uint16_t AC5,
                            int16_t MC, int16_t MD, int32_t * B5);

void bmp180_get_pressure(int16_t B1, int16_t B2, int32_t B5,
                        int16_t AC1, int16_t AC2, int16_t AC3,
                        uint16_t AC4, int32_t UP);

void bmp180_service(uint8_t deviceAddr, int16_t AC1, int16_t AC2, int16_t AC3,
                            uint16_t AC4, uint16_t AC5, uint16_t AC6,
                            int16_t B1, int16_t B2, int16_t MB,
                            int16_t MC, int16_t MD, TemperatureData_s * tempData);

#endif /* L4_IO_SENSORS_BMP180_H_ */
