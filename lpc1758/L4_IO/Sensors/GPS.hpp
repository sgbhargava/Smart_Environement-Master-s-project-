/*
 * GPS.hpp
 *
 *  Created on: Jul 10, 2016
 *      Author: Christopher
 */

#ifndef L4_IO_SENSORS_GPS_HPP_
#define L4_IO_SENSORS_GPS_HPP_
#include "SensorData.hpp"

void GPS_init();

void GPS_Read(GPSData_s * gps);

#endif /* L4_IO_SENSORS_GPS_HPP_ */
