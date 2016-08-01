/*
 * lipo.cpp
 *
 *  Created on: Jul 17, 2016
 *      Author: SPIRIT
 */
#include "lipo.hpp"
void lipo_monitor_init()
{
	//set p0.0 as input for Alert pin of FuelGuage
	LPC_GPIO0->FIODIR &= !(1 << 0);
	configMAX17043(32);  // Configure the MAX17043's alert percentage
	qsMAX17043();  // restart fuel-gauge calculations

}
/*
configMAX17043(byte percent) configures the config register of
the MAX170143, specifically the alert threshold therein. Pass a
value between 1 and 32 to set the alert threshold to a value between
1 and 32%. Any other values will set the threshold to 32%.
*/
void configMAX17043(uint8_t percent)
{
	I2C2& i2c = I2C2::getInstance();
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
	I2C2& i2c = I2C2::getInstance();
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
	I2C2& i2c = I2C2::getInstance();
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
 I2C2& i2c = I2C2::getInstance();
 i2c.readRegisters(FuelGaugeReadAddr, 0x02, temp_reg,2);
  // last 4 bits of vcell are nothing
 vcell_temp = ( ( temp_reg[0] << 8 ) |  temp_reg[1] );
 vcell_temp = ( vcell_temp >> 4 );
 vcell =( ( float )vcell_temp * VOLTAGE_CONVERTER );

  return vcell;
}

void fuel_guage_task(SystemHealth_s *sys_stat)
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
	v_cell 								= vcellMAX17043();
	percentage_cell 					= percentMAX17043();
	sys_stat->deviceBatteryPercent 		= percentage_cell;
	sys_stat->deviceVoltage				= v_cell;
	TemperatureSensor &tempSensor 		= TemperatureSensor::getInstance();
	sys_stat->deviceTemp				= tempSensor.getFarenheit();
}


