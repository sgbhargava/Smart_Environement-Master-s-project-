/*
 * K30C02.cpp
 *
 *  Created on: Jul 6, 2016
 *      Author: Christopher
 */
#include "K30C02.hpp"
#include "uart3.hpp"
#include "utilities.h"
#include "stdio.h"

const uint8_t readC02[] = {DEVICEADDRESS, C02READ, ADDRESSMSB, ADDRESSLSB,
                        READLEN, CHECKSUM_MSB, CHECKSUM_LSB};
const uint8_t initC02[] = {DEVICEADDRESS, C02INIT, INITADDRESSMSB, INITADDRESSLSB,
                        INITLEN, INITCHECKSUM_MSB, INITCHECKSUM_LSB};

int readCO2size = sizeof(readC02)/sizeof(uint8_t);
int initCO2size = sizeof(initC02)/sizeof(uint8_t);

const int size = 16;
char C02buffer[size];

Uart3& C02_uart3 = Uart3::getInstance();

void K30_init()
{
    C02_uart3.init(9600);
    for(int i = 0; i < initCO2size; i++)
    {
        C02_uart3.putChar(initC02[i], 1000);
    }
}

void K30_ReadC02(float * co2Data)
{
    float tmp;
    for(int j = 0; j < 10; j++)
    {
        delay_ms(20000);
        for(int i = 0; i < readCO2size; i++)
        {
            C02_uart3.putChar(readC02[i], 1000);
        }
        C02_uart3.gets(C02buffer, size, 1000);
        tmp = ((C02buffer[3]<<8) + C02buffer[4]);
        if(tmp > 0 && tmp < 1000)
        {
            *co2Data = tmp;
            printf("co2Data = %f\n", *co2Data);
            return;
        }
    }
}

