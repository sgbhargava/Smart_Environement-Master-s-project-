/*
 * GPS.cpp
 *
 *  Created on: Jul 10, 2016
 *      Author: Christopher
 */

#ifndef L4_IO_SENSORS_GPS_CPP_
#define L4_IO_SENSORS_GPS_CPP_
#include "GPS.hpp"
#include "uart3.hpp"
#include "stdio.h"
#include "scheduler_task.hpp"

Uart3& Gps_uart3 = Uart3::getInstance();


void GPS_init()
{
    Gps_uart3.init(4800, 256, 32);
}

void GPS_Read(GPSData_s * gps)
{
    char buffer[1024];
    float utcTime, latitude, longitude, HDOP, Altitude;
    int PositionFix, SatellitesUsed;
    char NSIndicator, EWIndicator;

    Gps_uart3.gets(buffer, 1024);
    int status = sscanf(buffer,"$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f",
            &utcTime, &latitude, &NSIndicator, &longitude, &EWIndicator,
            &PositionFix, &SatellitesUsed, &HDOP,&Altitude);
    if(status > 0)
    {
        if(!PositionFix)
        {
            utcTime = 0;
            latitude = 0;
            longitude = 0;
        }
        else
        {
            if(NSIndicator == 'S') latitude *= -1;
            if(EWIndicator == 'W') longitude *= -1;
            gps->Latitude = latitude;
            gps->Longitude = longitude;
            gps->Altitude = Altitude;
        }
    }

}



#endif /* L4_IO_SENSORS_GPS_CPP_ */
