/*
 * HTU21DF.cpp
 *
 *  Created on: Jun 26, 2016
 *      Author: Christopher
 */
#include "HTU21DF.hpp"
#include <stdio.h>
#include <utilities.h>
#include <i2c2.hpp>

void HTU21DF_init(){
    I2C2::getInstance().writeReg(HTU21DF_I2CADDR, HTU21DF_RESET, 0);
    delay_ms(25);
}

void HTU21DF_Humidity(float * humidity)
{
    uint8_t buffer[2] = { 0 };
    float tmp;
    I2C2::getInstance().readRegisters(HTU21DF_I2CADDR,HTU21DF_READHUM, &buffer[0], 2);
    tmp = -6.0+125.0*(((buffer[0]<<8)+buffer[1])/65536.0);
    *humidity = tmp;
}

void HTU21DF_Temperature(float * temperature)
{
    uint8_t buffer[2] = { 0 };
    float tmp;
    I2C2::getInstance().readRegisters(HTU21DF_I2CADDR,HTU21DF_READTEMP, &buffer[0], 2);
    tmp = ((buffer[0]<<8)+buffer[1]);
    tmp *= 175.72;
    tmp /= 65536;
    tmp -= 46.85;
    *temperature = tmp;
}
