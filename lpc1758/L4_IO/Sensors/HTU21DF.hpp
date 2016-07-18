/*
 * HTU21DF.hpp
 *
 *  Created on: Jun 26, 2016
 *      Author: Christopher
 */

#ifndef L4_IO_SENSORS_HTU21DF_HPP_
#define L4_IO_SENSORS_HTU21DF_HPP_

#define HTU21DF_I2CADDR       0x80
#define HTU21DF_READTEMP      0xE3
#define HTU21DF_READHUM       0xE5
#define HTU21DF_WRITEREG       0xE6
#define HTU21DF_READREG       0xE7
#define HTU21DF_RESET       0xFE

void HTU21DF_init();

void HTU21DF_Humidity(float * humidity);

void HTU21DF_Temperature(float * temperature);

#endif /* L4_IO_SENSORS_HTU21DF_HPP_ */
