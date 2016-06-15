/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * This contains the period callback functions for the periodic scheduler
 *
 * @warning
 * These callbacks should be used for hard real-time system, and the priority of these
 * tasks are above everything else in the system (above the PRIORITY_CRITICAL).
 * The period functions SHOULD NEVER block and SHOULD NEVER run over their time slot.
 * For example, the 1000Hz take slot runs periodically every 1ms, and whatever you
 * do must be completed within 1ms.  Running over the time slot will reset the system.
 */

#include <stdint.h>
#include "io.hpp"
#include "periodic_callback.h"
#include "stdio.h"
#define VOLTAGE_CONVERTER 0.00125
I2C2& i2c = I2C2::getInstance();
float batVoltage = 0.0;
float batPercentage = 0.0;
bool alertStatus = true;
const uint8_t FuelGaugeReadAddr = 0x6d;
const uint8_t FuelGaugeWriteAddr = 0x6c;

/*
configMAX17043(byte percent) configures the config register of
the MAX170143, specifically the alert threshold therein. Pass a
value between 1 and 32 to set the alert threshold to a value between
1 and 32%. Any other values will set the threshold to 32%.
*/
void configMAX17043(uint8_t percent)
{
  if ((percent >= 32)||(percent == 0))  // Anything 32 or greater will set to 32%
    i2c.writeReg(FuelGaugeWriteAddr,0x0C, 0x9700);
  else
  {
    uint8_t percentBits = 32 - percent;
    i2c.writeReg(FuelGaugeWriteAddr,0x0C, (0x9700 | percentBits));
  }
}

/*
qsMAX17043() issues a quick-start command to the MAX17043.
A quick start allows the MAX17043 to restart fuel-gauge calculations
in the same manner as initial power-up of the IC. If an application's
power-up sequence is very noisy, such that excess error is introduced
into the IC's first guess of SOC, the Arduino can issue a quick-start
to reduce the error.
*/

/*
percentMAX17043() returns a float value of the battery percentage
reported from the SOC register of the MAX17043.
*/
float percentMAX17043()
{
	uint8_t temp_reg[2];
	float percent = 0.0;

	//Read SOC register of MAX17043
	i2c.readRegisters(FuelGaugeReadAddr,0x04,temp_reg,2);
	percent =  temp_reg[0];  // High byte of SOC is percentage
	percent += (float)(temp_reg[1])/256.0;  // Low byte is 1/256%

	  return percent;
}

/*
qsMAX17043() issues a quick-start command to the MAX17043.
A quick start allows the MAX17043 to restart fuel-gauge calculations
in the same manner as initial power-up of the IC. If an application's
power-up sequence is very noisy, such that excess error is introduced
into the IC's first guess of SOC, the Arduino can issue a quick-start
to reduce the error.
*/
void qsMAX17043()
{
  i2c.writeReg(FuelGaugeWriteAddr,0x06, 0x4000);

}

/*
vcellMAX17043() returns a 12-bit ADC reading of the battery voltage,
as reported by the MAX17043's VCELL register.
This does not return a voltage value. To convert this to a voltage,
multiply by 5 and divide by 4096.
*/
float vcellMAX17043()
{
  uint8_t temp_reg[2];
  uint16_t vcell_temp = 0;
  float vcell = 0;

 i2c.readRegisters(FuelGaugeReadAddr, 0x02, temp_reg,2);
  // last 4 bits of vcell are nothing
 vcell_temp = ( ( temp_reg[0] << 8 ) |  temp_reg[1] );
 vcell_temp = ( vcell_temp >> 4 );
 vcell =( ( float )vcell_temp * VOLTAGE_CONVERTER );

  return vcell;
}

/// This is the stack size used for each of the period tasks
const uint32_t PERIOD_TASKS_STACK_SIZE_BYTES = (512 * 4);

/// Called once before the RTOS is started, this is a good place to initialize things once
bool period_init(void)
{
	//set p0.0 as input for Alert pin of FuelGuage
	LPC_GPIO0->FIODIR &= !(1 << 0);
	configMAX17043(32);  // Configure the MAX17043's alert percentage
	qsMAX17043();  // restart fuel-gauge calculations

    return true; // Must return true upon success
}

/// Register any telemetry variables
bool period_reg_tlm(void)
{
    // Make sure "SYS_CFG_ENABLE_TLM" is enabled at sys_config.h to use Telemetry
    return true; // Must return true upon success
}



void period_1Hz(void)
{
	static float v_cell = 0;
	static float percentage_cell = 0;
	//test fuel guage functionality
	if (LPC_GPIO0->FIOPIN & (1 << 0))
	{
		//Alert is 1, battery percentage is above threshold.
		printf(" Battery above threshold\n");
	}
	else
	{
		printf("Battery low\n");
	}
	v_cell = vcellMAX17043();
	percentage_cell = percentMAX17043();
	printf (" Battery voltage = %02f\t precentage left = %02f\%\n",v_cell, percentage_cell );

}

void period_10Hz(void)
{
    LE.toggle(2);
}

void period_100Hz(void)
{
    LE.toggle(3);
}

void period_1000Hz(void)
{
    LE.toggle(4);
}
