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

typedef struct GPSData_s{
        float Latitude;
        float Longitude;
        float Altitude;
} GPSData;

typedef struct TemperatureData_s{
        float temperature;
        float pressure;
}TemperatureData;

typedef struct HumidityData_s{
        float temperature;
        float humidity;
}HumidityData;

typedef struct SystemHealth_s{
        float deviceMemUsage;
        int deviceCPUUsage;
        float deviceBatteryPercent;
        float deviceTemp;
        float deviceVoltage;
}SystemData;

typedef struct SunTracker_s {
        int ch0;
        int ch1;
        int ch2;
        int ch3;
}SunTracker;
#endif /* L4_IO_SENSORS_SENSORDATA_H_ */
