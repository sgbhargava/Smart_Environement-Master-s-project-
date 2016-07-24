/*
 * common_includes.h
 *
 *  Created on: Jul 17, 2016
 *      Author: SPIRIT
 */

#ifndef L5_APPLICATION_EXAMPLES_COMMON_INCLUDES_HPP_
#define L5_APPLICATION_EXAMPLES_COMMON_INCLUDES_HPP_
#include "string"
#include "tasks.hpp"
#include "temperature_sensor.hpp"
#include "examples/lipo.hpp"

extern const char DEVICE_ID;
extern SemaphoreHandle_t UVSem;
extern SemaphoreHandle_t humiditySem;
extern SemaphoreHandle_t pressureSem;
extern SemaphoreHandle_t TXSem;
extern SemaphoreHandle_t GPSSem;
extern SemaphoreHandle_t healthSem;
#endif /* L5_APPLICATION_EXAMPLES_COMMON_INCLUDES_HPP_ */
