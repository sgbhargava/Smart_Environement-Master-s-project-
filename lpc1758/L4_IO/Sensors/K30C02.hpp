/*
 * K30C02.hpp
 *
 *  Created on: Jul 6, 2016
 *      Author: Christopher
 */

#ifndef L4_IO_SENSORS_K30C02_HPP_
#define L4_IO_SENSORS_K30C02_HPP_
#include <stdint.h>

#define DEVICEADDRESS       0XFE
#define C02READ             0x44
#define ADDRESSMSB          0x00
#define ADDRESSLSB          0x08
#define READLEN             0x02
#define CHECKSUM_MSB        0x9F
#define CHECKSUM_LSB        0x25

void K30_init();
void K30_ReadC02();


#endif /* L4_IO_SENSORS_K30C02_HPP_ */
