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
 * @brief Contains FreeRTOS Tasks
 */
#ifndef TASKS_HPP_
#define TASKS_HPP_

#include "scheduler_task.hpp"
#include "soft_timer.hpp"
#include "command_handler.hpp"
#include "wireless.h"
#include "Sensors/BMP180.hpp"
#include "Sensors/SI1145.hpp"
#include "Sensors/HTU21DF.hpp"
#include "Sensors/K30C02.hpp"
#include "Sensors/SensorData.hpp"
#include "Sensors/GPS.hpp"
#include "char_dev.hpp"
#include "utilities.h"
#include "stdio.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "examples/common_includes.hpp"

extern SensorData_s SensorData;
extern SemaphoreHandle_t UVSem;
extern SemaphoreHandle_t humiditySem;
extern SemaphoreHandle_t pressureSem;
extern SemaphoreHandle_t TXSem;
extern SemaphoreHandle_t GPSSem;

/**
 * Terminal task is our UART0 terminal that handles our commands into the board.
 * This also saves and restores the "disk" telemetry.  Disk telemetry variables
 * are automatically saved and restored across power-cycles to help us preserve
 * any non-volatile information.
 */
class terminalTask : public scheduler_task
{
    public:
        terminalTask(uint8_t priority);     ///< Constructor
        bool regTlm(void);                  ///< Registers telemetry
        bool taskEntry(void);               ///< Registers commands.
        bool run(void *p);                  ///< The main loop

    private:
        // Command channels device and input command str
        typedef struct {
            CharDev *iodev; ///< The IO channel
            str *cmdstr;    ///< The command string
            bool echo;      ///< If input should be echo'd back
        } cmdChan_t;

        VECTOR<cmdChan_t> mCmdIface;   ///< Command interfaces
        CommandProcessor mCmdProc;     ///< Command processor
        uint16_t mCommandCount;        ///< terminal command count
        uint16_t mDiskTlmSize;         ///< Size of disk variables in bytes
        char *mpBinaryDiskTlm;         ///< Binary disk telemetry
        SoftTimer mCmdTimer;           ///< Command timer

        cmdChan_t getCommand(void);
        void addCommandChannel(CharDev *channel, bool echo);
        void handleEchoAndBackspace(cmdChan_t *io, char c);
        bool saveDiskTlm(void);
};

/**
 * Remote task is the task that monitors the IR remote control signals.
 * It can "learn" remote control codes by typing "learn" into the UART0 terminal.
 * Thereafter, if a user enters a 2-digit number through a remote control, then
 * your function handleUserEntry() is called where you can take an action.
 */
class remoteTask : public scheduler_task
{
    public:
        remoteTask(uint8_t priority);   ///< Constructor
        bool init(void);                ///< Inits the task
        bool regTlm(void);              ///< Registers non-volatile variables
        bool taskEntry(void);           ///< One time entry function
        bool run(void *p);              ///< The main loop

    private:
        /** This function is called when a 2-digit number is decoded */
        void handleUserEntry(int num);
        
        /**
         * @param code  The IR code
         * @param num   The matched number 0-9 that mapped the IR code.
         * @returns true if the code has been successfully mapped to the num
         */
        bool getNumberFromCode(uint32_t code, uint32_t& num);

        uint32_t mNumCodes[10];      ///< IR Number codes
        uint32_t mIrNumber;          ///< Current IR number we're decoding
        SemaphoreHandle_t mLearnSem; ///< Semaphore to enable IR code learning
        SoftTimer mIrNumTimer;       ///< Time-out for user entry for 1st and 2nd digit
};

/**
 * Nordic wireless task to participate in the mesh network and handle retry logic
 * such that packets are resent if an ACK has not been received
 */
class wirelessTask : public scheduler_task
{
    public:
        wirelessTask(uint8_t priority) :
            scheduler_task("wireless", 512, priority)
        {
            /* Nothing to init */
        }

        bool run(void *p)
        {
            wireless_service(); ///< This is a non-polling function if FreeRTOS is running.
            return true;
        }
};

/**
 * Periodic callback dispatcher task
 * This task gives the semaphores that end up calling functions at periodic_callbacks.cpp
 */
class periodicSchedulerTask : public scheduler_task
{
    public:
        periodicSchedulerTask(void);
        bool run(void *p);

    private:
        bool handlePeriodicSemaphore(const uint8_t index, const uint8_t frequency);
};

/**
 * Nordic wireless task to participate in the mesh network and handle retry logic
 * such that packets are resent if an ACK has not been received
 */
class TemperaturePressureSensorTask : public scheduler_task
{
    public:
        TemperaturePressureSensorTask(uint8_t priority) :
            scheduler_task("Temperature_Pressure", 2048, priority)
        {
            sensor_Temperature_data_q = xQueueCreate(1, sizeof(TempertureData_q));
            addSharedObject("Temperature_queue", sensor_Temperature_data_q);
            bmp180Addr = 0xEE;
            AC1 = 0; AC2 = 0; AC3 = 0; AC4 = 0; AC5 = 0; AC6 = 0;
            B1 = 0 ; B2 = 0; MB = 0; MC = 0; MD = 0;
        }
        bool init()
        {
            bmp180_get_cal_param(bmp180Addr, &AC1, &AC2, &AC3, &AC4, &AC5, &AC6, &B1, &B2, &MB, &MC, &MD);
            return true;
        }

        bool run(void *p)
        {
        	if (xSemaphoreTake(pressureSem, portMAX_DELAY))
        	{
        		printf ("=================Got pressureSem\n");
				bmp180_service(bmp180Addr, AC1, AC2, AC3, AC4, AC5, AC6, B1, B2, MB, MC, MD, &TempertureData_q);
				xQueueOverwrite( sensor_Temperature_data_q, &TempertureData_q);
				xSemaphoreGive(UVSem);
        	}
            return true;
        }
    private:
        uint8_t     bmp180Addr;
        int16_t     AC1, AC2, AC3, B1, B2, MB, MC, MD;
        uint16_t    AC4, AC5, AC6;
        QueueHandle_t sensor_Temperature_data_q;
        TemperatureData_s TempertureData_q;
};

class UVLightIRSensorTask : public scheduler_task
{
    public:
        UVLightIRSensorTask(uint8_t priority) :
            scheduler_task("UVLight_IR", 2048, priority)
        {
            sensor_UVLight_data_q = xQueueCreate(1, sizeof(float));
            addSharedObject("UVLight_queue", sensor_UVLight_data_q);
            uv = -1;
        }
        bool init()
        {
            reset();
            SI1145_init();
            return true;
        }
        bool run(void *p)
        {
        	if (xSemaphoreTake(UVSem, portMAX_DELAY))
        	{
        		printf ("=================Got UVSem\n");
        		readUV(&uv);
        		xQueueOverwrite(sensor_UVLight_data_q, &uv);
        		xSemaphoreGive(humiditySem);
        	}
            return true;
        }
    private:
        QueueHandle_t sensor_UVLight_data_q;
        SensorData_s SensorData_q;
        float uv;

};

class HumiditySensorTask : public scheduler_task
{
    public:
        HumiditySensorTask(uint8_t priority) :
            scheduler_task("Humidity", 2048, priority)
        {
            sensor_Humidity_data_q = xQueueCreate(1, sizeof(humidty_temperature));
            addSharedObject("Humidity_queue", sensor_Humidity_data_q);
            humidity = -1;
            temperature = -1;
        }
        bool init()
        {
            HTU21DF_init();
            return true;
        }
        bool run(void *p)
        {
        	if (xSemaphoreTake(humiditySem, portMAX_DELAY))
        	{
        		printf ("=================Got humiditySem\n");

				delay_ms(5000);
				HTU21DF_Humidity(&humidity);
				HTU21DF_Temperature(&temperature);
				humidty_temperature.humidity = humidity;
				humidty_temperature.temperature = ((temperature)*(9.0/5.0)+32);
				xQueueOverwrite(sensor_Humidity_data_q, &humidty_temperature);
				xSemaphoreGive(GPSSem);
        	}
			return true;
        }
    private:
        QueueHandle_t sensor_Humidity_data_q;
        HumidityData_s humidty_temperature;
        float humidity;
        float temperature;

};

class C02SensorTask : public scheduler_task
{
    public:
        C02SensorTask(uint8_t priority) :
            scheduler_task("C02", 2048, priority)
        {
            sensor_c02_data_q = xQueueCreate(1, sizeof(float));
            addSharedObject("CO2_queue", sensor_c02_data_q);
            co2_data = -1;
        }
        bool init()
        {
            K30_init();
            return true;
        }
        bool run(void *p)
        {
            K30_ReadC02(&co2_data);
            xQueueOverwrite(sensor_c02_data_q, &co2_data);
            return true;
        }
    private:
        QueueHandle_t sensor_c02_data_q;
        float co2_data;
};

class GPSTask : public scheduler_task
{
    public:
        GPSTask(uint8_t priority) :
            scheduler_task("GPS", 4096, priority)
        {
            sensor_gps_data_q = xQueueCreate(1, sizeof(GPSData_s));
            addSharedObject("gps_queue", sensor_gps_data_q);
        }
        bool init()
        {
            GPS_init();
            return true;
        }
        bool run(void *p)
        {
        	if (xSemaphoreTake(GPSSem, portMAX_DELAY))
			{
				printf ("=================Got GPSSem\n");
				GPS_Read(&gps_q);
				xQueueOverwrite(sensor_gps_data_q, &gps_q);
				xSemaphoreGive(TXSem);
			}
            return true;
        }
    private:
        QueueHandle_t sensor_gps_data_q;
        GPSData_s gps_q;
};

class PrintSensorTask : public scheduler_task
{
    public:
        PrintSensorTask(uint8_t priority) :
            scheduler_task("Print", 2048, priority)
        {
            sensor_gps_data_q = getSharedObject("gps_queue");
            sensor_c02_data_q = getSharedObject("CO2_queue");
            sensor_Humidity_data_q = getSharedObject("Humidity_queue");
            sensor_UVLight_data_q = getSharedObject("UVLight_queue");
            sensor_Temperature_data_q = getSharedObject("Temperature_queue");
            co2Data = 0;
            humidity = 0;
            uv = 0;
        }

        bool run(void *p)
        {
            delay_ms(5000);
            if(xQueueReceive(sensor_Temperature_data_q, &TempertureData_q, 0))
            {

                printf("Temperature(BMP) = %lf\n", TempertureData_q.temperature );
                printf("Pressure(BMP) = %lf\n", TempertureData_q.pressure);
            }
            if(xQueueReceive(sensor_gps_data_q, &gps_q, 0))
            {
                printf("Latitude = %lf\n", gps_q.Latitude);
                printf("Longitude = %lf\n", gps_q.Longitude);
                printf("Altitude = %lf\n\n\n", gps_q.Altitude);
            }
            if(xQueueReceive(sensor_c02_data_q, &co2Data, 0))
            {
                printf("CO2 = %lf\n", co2Data);
            }
            if(xQueueReceive(sensor_Humidity_data_q, &Humidity_q, 0))
            {
                printf("Humidity = %lf\n", Humidity_q.humidity);
                printf("Temperature = %lf\n", Humidity_q.temperature);
            }
            if(xQueueReceive(sensor_UVLight_data_q, &uv, 0))
            {
                printf("UV Light = %lf\n", uv);
            }

            return true;
        }
    private:
        QueueHandle_t sensor_gps_data_q, sensor_c02_data_q, sensor_Humidity_data_q,
        sensor_UVLight_data_q, sensor_Temperature_data_q;
        GPSData_s gps_q;
        TemperatureData_s TempertureData_q;
        HumidityData_s Humidity_q;
        float co2Data, humidity, uv;
        SensorData_s SensorData_q;
};

#endif /* TASKS_HPP_ */
