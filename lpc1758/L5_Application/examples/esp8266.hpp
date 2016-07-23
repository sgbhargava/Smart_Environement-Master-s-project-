/*
 * esp8266.hpp
 *
 *  Created on: Jul 15, 2016
 *      Author: Bhargav
 */
/**
 * @file
 * @brief Contains esp8266 task to connect esp8266 Wifly module to ESP8266
 */
#ifndef L5_APPLICATION_EXAMPLES_ESP8266_HPP_
#define L5_APPLICATION_EXAMPLES_ESP8266_HPP_
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "scheduler_task.hpp"
#include "uart_dev.hpp"
#include "string"
#include "examples/common_includes.hpp"
/**
 * Web Request Type.
 * See example below on how to use it.
 */
typedef struct {
    std::string payload;
    char success;             ///< Changed to true if HTTP request was successful
    std::string req;
    SemaphoreHandle_t req_done_signal; ///< After web-request is made, this semaphore is given (if not zero)
} web_req_type;

#define ESP8266_PORT       "80"  ///< Port number to configure for TCP/IP Server
#define ESP8266_BAUD_RATE  115200  ///< Baud rate you wish to use (it will auto-detect and change to this)
#define ESP8266_SSID       "Through Silence"  ///< Your SSID
#define ESP8266_KEY        "jjjjmm16"   ///< Your WPA2 pass-phrase
#define ESP8266_RXQ_SIZE   512     ///< Size of UART's RXQ
#define ESP8266_TXQ_SIZE   512     ///< Size of UART's TXQ
#define ESP8266_SHR_OBJ    "webrq" ///< The shared object name of this task's Web reqeust queue




/**
 * This is a RN-XV "ESP8266" task responsible to connect to ESP8266 Network. It can also allow
 * other tasks to perform HTTP GET requests or POST data to a web-server.
 *
 * User can provide which UART shall be used for the ESP8266Task and RN-XV. User can use that
 * UART whenever Uart.isReady() is true. If Uart.isReady() returns false, then this task is
 * using the RN-XV and you shouldn't use it.
 *
 * The objective of this task is to connect your RN-XV to ESP8266, and provide the method to make
 * a WEB request.  Other than that, you can use the same UART as given to this task for your
 * communication based on isReady() method.  If you only want to connect, you can borrow this
 * code, or run this task and then suspend it once connection is made.
 *
 * To make a web request, populate the web request structure and do this in another task :
 *
 * @code
 * char myBuff[128] = { 0 };
 * web_req_type webreq;
 * webreq.http_ip_host = "www.google.com";
 * webreq.http_get_request = "index.html";
 * webreq.http_discard_until = 0;
 * webreq.http_response = &myBuff[0];
 * webreq.http_response_size = sizeof(myBuff);
 * webreq.req_done_signal = 0; // Don't use signal
 *
 * void *q = getSharedObject(ESP8266_SHR_OBJ);
 * xQueueSend(q, &webreq, portMAX_DELAY);
 *
 * // Wait 30 sec (or more) or if you don't want to poll, use a semaphore as signal
 * // webreq.req_done_signal semaphore is given after web request terminates
 * if (webreq.success) {
 *     // Your data will be at myBuff
 * }
 * @endcode
 *
 * If SYS_CFG_ENABLE_TLM is enabled, then the ESP8266 SSID and Passphrase is saved
 * to disk, which allows you to change the settings during run-time and these
 * settings are preserved across power cycle.  To change these keys, you can
 * use terminal command :
 *  "telemetry disk mESP8266Ssid mySsid"
 *  "telemetry disk mESP8266Key mykey"
 */
class esp8266Task : public scheduler_task
{
public:
        /**
         * Task constructor
         * @param uartForESP8266  The UART for your RN-XV, such as: "Uart3::getInstance()"
         * @param priority     The task's priority
         */
		esp8266Task(UartDev& uartForESP8266, uint8_t priority);

        bool run(void *p);    ///< Services web requests
        bool init(void);      ///< Inits stuff (obviously)
        bool taskEntry(void); ///< Auto-detects baud-rate and sets it on RN-XV
        bool regTlm(void);    ///< Registers "disk" variables

private:
        bool ESP8266InitBaudRate(void);
        void ESP8266SendTestCmd(void);
        bool ESP8266HandleHttpReq(web_req_type request);

        void ESP8266Flush(void);
        void ESP8266SendCmd(const char* pCmd, const char* pParam=0);
        //void ESP8266EnterCmdMode(void);
        bool ESP8266Connect(void);
        bool ESP8266IsConnected(void);

        UartDev& mESP8266;               ///< The uart to use for RN-XV
        uint32_t mESP8266BaudRate;       ///< The baud rate of the ESP8266
        QueueHandle_t mHttpReqQueue;  ///< Queue handle of web request

        /** @{ Disk telemetry variables */
        bool mESP8266Echo;     ///< If true, ESP8266 echo is printed using printf()
        char mESP8266Ssid[24]; ///< SSID is saved here
        char mESP8266Key[24];  ///< ESP8266 phrase is saved here
        QueueHandle_t sensor_gps_data_q, sensor_c02_data_q, sensor_Humidity_data_q,\
					sensor_UVLight_data_q, sensor_Temperature_data_q;
		GPSData_s gps_q;
		TemperatureData_s TempertureData_q;
		HumidityData_s Humidity_q;
		float co2Data, humidity, uv;
		SensorData_s SensorData_q;
        /** @} */
};


#endif /* L5_APPLICATION_EXAMPLES_ESP8266_HPP_ */
