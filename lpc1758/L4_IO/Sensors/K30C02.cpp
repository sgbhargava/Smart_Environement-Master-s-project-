/*
 * K30C02.cpp
 *
 *  Created on: Jul 6, 2016
 *      Author: Christopher
 */
#include <Sensors/SensorData.hpp>
#include "K30C02.hpp"
#include "uart2.hpp"
#include "utilities.h"
#include "stdio.h"
#include "SensorData.hpp"

const uint8_t readC02[] = {DEVICEADDRESS, C02READ, ADDRESSMSB, ADDRESSLSB,
                        READLEN, CHECKSUM_MSB, CHECKSUM_LSB};

int readCO2size = sizeof(readC02)/sizeof(uint8_t);

const int size = 16;
char C02buffer[size];

Uart2& C02_uart2 = Uart2::getInstance();
extern SensorData_s SensorData;

void K30_init()
{
    C02_uart2.init(9600);
}

void K30_ReadC02()
{
    delay_ms(2000); // 2 second sample time
    for(int i = 0; i < readCO2size; i++)
    {
        C02_uart2.putChar(readC02[i], 100);
    }
    C02_uart2.gets(C02buffer, size, 100);
    SensorData.CO2 = ((C02buffer[3]<<8) + C02buffer[4]);
}

