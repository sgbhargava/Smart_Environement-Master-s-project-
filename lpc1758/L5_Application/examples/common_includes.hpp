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
typedef struct sysStatStruct
{
	float deviceTemp;
	float deviceBat;
	float deviceCPU;
	float deviceMem;
	float deviceTemperature;
	float deviceVoltage;
	bool  batCriticalFlag;
}sysStat;
extern sysStatStruct sys_stat;
extern QueueHandle_t sysStatQh;
extern const char DEVICE_ID;

#endif /* L5_APPLICATION_EXAMPLES_COMMON_INCLUDES_HPP_ */
