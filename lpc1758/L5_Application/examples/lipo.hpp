/*
 * lipo.hpp
 *
 *  Created on: Jul 17, 2016
 *      Author: SPIRIT
 */

#ifndef L5_APPLICATION_EXAMPLES_LIPO_HPP_
#define L5_APPLICATION_EXAMPLES_LIPO_HPP_
#include "stdlib.h"
#include "stdio.h"
#include <stdint.h>
#include "i2c2.hpp"
#include "examples/common_includes.hpp"
#include "Sensors/SensorData.hpp"
#define VOLTAGE_CONVERTER 0.00125
const uint8_t FuelGaugeReadAddr = 0x6d;
const uint8_t FuelGaugeWriteAddr = 0x6c;
void lipo_monitor_init();
void configMAX17043(uint8_t percent);
float percentMAX17043();
void qsMAX17043();
float vcellMAX17043();


#endif /* L5_APPLICATION_EXAMPLES_LIPO_HPP_ */
