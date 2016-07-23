/*
 * SI1145.cpp
 *
 *  Created on: Jun 12, 2016
 *      Author: Christopher
 */

#ifndef SI1145_C_
#define SI1145_C_
#include "SI1145.hpp"
#include <stdio.h>
#include <utilities.h>
#include <i2c2.hpp>

void reset()
{

    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_MEASRATE0, 0);
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_MEASRATE1, 0);
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_IRQEN, 0);
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_IRQMODE1, 0);
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_IRQMODE2, 0);
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_INTCFG, 0);
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_IRQSTAT, 0xff);
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_COMMAND, SI1145_RESET);
    delay_ms(25);

    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_HWKEY, 0x17);
    delay_ms(25);
}

void SI1145_init()
{
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_UCOEFF0, 0x29);
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_UCOEFF1, 0x89);
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_UCOEFF2, 0x02);
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_UCOEFF3, 0x00);

    //enable UV sensor, IR, Light
    writeParam(SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV |
            SI1145_PARAM_CHLIST_ENALSIR | SI1145_PARAM_CHLIST_ENALSVIS);
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_INTCFG, SI1145_REG_INTCFG_INTOE);
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_IRQEN, SI1145_REG_IRQEN_ALSEVERYSAMPLE);

    //Set high range
    writeParam(SI1145_PARAM_ALSIRADCMISC, SI1145_PARAM_ALSIRADCMISC_RANGE);
    writeParam(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE);

    //Measurement rate for auto
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_MEASRATE0, 0xff);

    //Auto Run
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_COMMAND, SI1145_PSALS_AUTO);
}
void readUV(float * uv){
    uint8_t tmp8 = 0;
    uint16_t tmp16 = 0;
    delay_ms(20);
    tmp8 = I2C2::getInstance().readReg(SI1145_ADDR, SI1145_REG_UVINDEX0 );
    tmp16 = I2C2::getInstance().readReg(SI1145_ADDR, SI1145_REG_UVINDEX1);
    *uv = (((tmp16 << 8) + tmp8)/100.0);
}
void readIR()
{
#if 0
    uint8_t tmp8 = 0;
    uint16_t tmp16 = 0;
    delay_ms(20);
    tmp8 = I2C2::getInstance().readReg(SI1145_ADDR, SI1145_REG_ALSIRDATA0 );
    tmp16 = I2C2::getInstance().readReg(SI1145_ADDR, SI1145_REG_ALSIRDATA1);
#endif
}
void readVisible()
{
#if 0
    uint8_t tmp8 = 0;
    uint16_t tmp16 = 0;
    delay_ms(20);
    tmp8 = I2C2::getInstance().readReg(SI1145_ADDR, SI1145_REG_ALSVISDATA0 );
    tmp16 = I2C2::getInstance().readReg(SI1145_ADDR, SI1145_REG_ALSVISDATA1);
#endif
}

void writeParam(uint8_t p, uint8_t v)
{
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_PARAMWR, v);
    I2C2::getInstance().writeReg(SI1145_ADDR, SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
}

#endif
