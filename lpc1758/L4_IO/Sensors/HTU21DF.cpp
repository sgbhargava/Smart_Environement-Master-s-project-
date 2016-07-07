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
#include "SensorData.hpp"

extern SensorData_s SensorData;
void HTU21DF_init(){
    I2C2::getInstance().writeReg(HTU21DF_I2CADDR, HTU21DF_RESET, 0);
    delay_ms(15);
}

void HTU21DF_Humidity()
{
    uint8_t buffer[2] = { 0 };
    float tmp;
    I2C2::getInstance().readRegisters(HTU21DF_I2CADDR,HTU21DF_READHUM, &buffer[0], 2);
    tmp = -6.0+125.0*(((buffer[0]<<8)+buffer[1])/65536.0);
    SensorData.humidity = tmp;
}


