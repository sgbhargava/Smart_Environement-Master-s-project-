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

extern SensorData_s SensorData;

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
            bmp180Addr = 0xEF;
            AC1 = 0; AC2 = 0; AC3 = 0; AC4 = 0; AC5 = 0; AC6 = 0;
            B1 = 0 ; B2 = 0; MB = 0; MC = 0; MD = 0;
            bmp180_get_cal_param(bmp180Addr, &AC1, &AC2, &AC3, &AC4, &AC5, &AC6, &B1, &B2, &MB, &MC, &MD);
        }

        bool run(void *p)
        {
            bmp180_service(bmp180Addr, AC1, AC2, AC3, AC4, AC5, AC6, B1, B2, MB, MC, MD);
            return true;
        }
    private:
        uint8_t     bmp180Addr;
        int16_t     AC1, AC2, AC3, B1, B2, MB, MC, MD;
        uint16_t    AC4, AC5, AC6;
};

class UVLightIRSensorTask : public scheduler_task
{
    public:
        UVLightIRSensorTask(uint8_t priority) :
            scheduler_task("UVLight_IR", 2048, priority)
        {
            reset();
            SI1145_init();
        }
        bool run(void *p)
        {
            readUV();
            readIR();
            readVisible();
            return true;
        }

};

class HumiditySensorTask : public scheduler_task
{
    public:
        HumiditySensorTask(uint8_t priority) :
            scheduler_task("Humidity", 2048, priority)
        {
            HTU21DF_init();
        }
        bool run(void *p)
        {
            HTU21DF_Humidity();
            return true;
        }

};

class C02SensorTask : public scheduler_task
{
    public:
        C02SensorTask(uint8_t priority) :
            scheduler_task("C02", 2048, priority)
        {

        }
        bool init()
        {
            K30_init();
            return true;
        }
        bool run(void *p)
        {
            K30_ReadC02();
            return true;
        }

};

class GPSTask : public scheduler_task
{
    public:
        GPSTask(uint8_t priority) :
            scheduler_task("GPS", 4096, priority)
        {

        }
        bool init()
        {
            GPS_init();
            return true;
        }
        bool run(void *p)
        {
            GPS_Read();
            return true;
        }

};

class PrintSensorTask : public scheduler_task
{
    public:
        PrintSensorTask(uint8_t priority) :
            scheduler_task("C02", 2048, priority)
        {

        }

        bool run(void *p)
        {
            printf("C02 = %lf\n", SensorData.CO2);
            printf("UV = %lf\n", SensorData.UVIndex);
            printf("Humidity = %lf\n", SensorData.humidity);
            printf("Temperature = %lf\n", SensorData.temperature);
            printf("Pressure = %lf\n", SensorData.pressure);
            printf("Latitude = %lf\n", SensorData.Latitude);
            printf("Longitude = %lf\n", SensorData.Longitude);
            printf("Altitude = %lf\n\n\n", SensorData.Altitude);
            delay_ms(10000);
            return true;
        }

};

#endif /* TASKS_HPP_ */
