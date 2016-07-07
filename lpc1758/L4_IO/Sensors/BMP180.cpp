/*
 * BMP180.c
 *
 *  Created on: Jun 5, 2016
 *      Author: Christopher
 */

#ifndef BMP180_C_
#define BMP180_C_

#include <Sensors/BMP180.hpp>
#include <stdio.h>
#include <utilities.h>
#include <i2c2.hpp>
#include "SensorData.hpp"

extern SensorData_s SensorData;

void bmp180_get_cal_param(uint8_t deviceAddr, int16_t * AC1, int16_t * AC2, int16_t * AC3,
                            uint16_t * AC4, uint16_t * AC5, uint16_t * AC6,
                            int16_t * B1, int16_t * B2, int16_t * MB,
                            int16_t * MC, int16_t * MD)
{
    uint8_t msb, lsb;
    delay_ms(1000);
    //EEPROM
    msb = I2C2::getInstance().readReg(deviceAddr, 0xAA);
    lsb = I2C2::getInstance().readReg(deviceAddr, 0xAB);
    *AC1 = (msb << 8) | lsb;

    msb = I2C2::getInstance().readReg(deviceAddr, 0xAC);
    lsb = I2C2::getInstance().readReg(deviceAddr, 0xAD);
    *AC2 = (msb << 8) | lsb;

    msb = I2C2::getInstance().readReg(deviceAddr, 0xAE);
    lsb = I2C2::getInstance().readReg(deviceAddr, 0xAF);
    *AC3 = (msb << 8) | lsb;

    msb = I2C2::getInstance().readReg(deviceAddr, 0xB0);
    lsb = I2C2::getInstance().readReg(deviceAddr, 0xB1);
    *AC4 = (msb << 8) | lsb;

    msb = I2C2::getInstance().readReg(deviceAddr, 0xB2);
    lsb = I2C2::getInstance().readReg(deviceAddr, 0xB3);
    *AC5 = (msb << 8) | lsb;

    msb = I2C2::getInstance().readReg(deviceAddr, 0xB4);
    lsb = I2C2::getInstance().readReg(deviceAddr, 0xB5);
    *AC6 = (msb << 8) | lsb;

    msb = I2C2::getInstance().readReg(deviceAddr, 0xB6);
    lsb = I2C2::getInstance().readReg(deviceAddr, 0xB7);
    *B1 = (msb << 8) | lsb;

    msb = I2C2::getInstance().readReg(deviceAddr, 0xB8);
    lsb = I2C2::getInstance().readReg(deviceAddr, 0xB9);
    *B2 = (msb << 8) | lsb;

    msb = I2C2::getInstance().readReg(deviceAddr, 0xBA);
    lsb = I2C2::getInstance().readReg(deviceAddr, 0xBB);
    *MB = (msb << 8) | lsb;

    msb = I2C2::getInstance().readReg(deviceAddr, 0xBC);
    lsb = I2C2::getInstance().readReg(deviceAddr, 0xBD);
    *MC = (msb << 8) | lsb;

    msb = I2C2::getInstance().readReg(deviceAddr, 0xBE);
    lsb = I2C2::getInstance().readReg(deviceAddr, 0xBF);
    *MD = (msb << 8) | lsb;
 }

void bmp180_get_ut(uint8_t deviceAddr, int32_t * ut)
{
    uint8_t msb, lsb;
    I2C2::getInstance().writeReg(deviceAddr, 0xF4, 0x2E);
    delay_ms(10);
    msb = I2C2::getInstance().readReg(deviceAddr, 0xF6);
    lsb = I2C2::getInstance().readReg(deviceAddr, 0xF7);
    *ut = (msb << 8) | lsb;
}

void bmp180_get_up(uint8_t deviceAddr, int32_t * UP)
{
    uint8_t msb, lsb, xlsb;
    I2C2::getInstance().writeReg(deviceAddr, 0xF4, 0x34);
    delay_ms(10);
    msb = I2C2::getInstance().readReg(deviceAddr, 0xF6);
    lsb = I2C2::getInstance().readReg(deviceAddr, 0xF7);
    xlsb = I2C2::getInstance().readReg(deviceAddr, 0xF8);
    *UP = (((msb << 16) | (lsb << 8) | xlsb) >> 8);
}
void bmp180_get_temperature(int32_t UT, uint16_t AC6, uint16_t AC5,
                            int16_t MC, int16_t MD, int32_t * B5)
{
    int32_t X1, X2, temperature;
    X1 = (UT-AC6) * AC5 / 32768;
    X2 = MC * 2048 / (X1 + MD);
    *B5 = X1 + X2;
    temperature = ((X1 + X2) + 8) / 16;
    SensorData.temperature = ((temperature/10.0)*(9.0/5.0)+32);
}

void bmp180_get_pressure(int16_t B1, int16_t B2, int32_t B5,
                        int16_t AC1, int16_t AC2, int16_t AC3,
                        uint16_t AC4, int32_t UP )
{
    int32_t X1, X2, X3, B3, B6, p;
    uint32_t B4, B7;
    B6 = B5 - 4000;
    X1 = (B2 * (B6 * B6 / 4096)) / 2048;
    X2 = AC2 * B6 / 2048;
    X3 = X1 + X2;
    B3 = (((AC1 * 4 + X3)) + 2) / 4;
    X1 = AC3 * B6 / 8192;
    X2 = (B1 * (B6 * B6 / 4096)) / 65536;
    X3 = ((X1 + X2) + 2) / 4;
    B4 = AC4 * (unsigned)(X3 + 32768) / 32768;
    B7 = ((unsigned)UP - B3) * (50000);
    if(B7 < 0x80000000){
        p = (B7 * 2) / B4;
     }
    else {
        p = (B7/B4) * 2;
    }
    X1 = (p/256) * (p/256);
    X1 = (X1 * 3038)/ 65536;
    X2 = (-7357 * p)/65536;
    p = p + (X1 + X2 + 3791) / 16;
    SensorData.pressure = p;
}

void bmp180_service(uint8_t deviceAddr, int16_t AC1, int16_t AC2, int16_t AC3,
                            uint16_t AC4, uint16_t AC5, uint16_t AC6,
                            int16_t B1, int16_t B2, int16_t MB,
                            int16_t MC, int16_t MD)
{
    int32_t     ut, up, b5;
    bmp180_get_ut(deviceAddr, &ut);
    bmp180_get_up( deviceAddr, &up);
    bmp180_get_temperature( ut,  AC6,  AC5, MC,  MD,  &b5);
    bmp180_get_pressure( B1,  B2,  b5, AC1,  AC2,  AC3, AC4, up);
}

#endif /* BMP180_C_ */
