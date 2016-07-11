/*
 * SensorData.h
 *
 *  Created on: Jun 5, 2016
 *      Author: Christopher
 */

#ifndef SENSORDATA_H_
#define SENSORDATA_H_

typedef struct SensorDataStruct{
        float temperature;
        float pressure;
        float humidity;
        float UVIndex;
        float CO2;
        float Latitude;
        float Longitude;
        float Altitude;
} SensorData_s;


#endif /* L4_IO_SENSORS_SENSORDATA_H_ */
