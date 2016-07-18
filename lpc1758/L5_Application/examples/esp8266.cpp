/*
 * esp8266.cpp
 *
 *  Created on: Jul 15, 2016
 *      Author: Bhargav
 */
#include <examples/common_includes.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <string>
#include "esp8266.hpp"
#include "file_logger.h"
#include "tlm/c_tlm_comp.h"
#include "tlm/c_tlm_var.h"
#include "utilities.h"
#include "str.hpp"
extern QueueHandle_t sysStatQh;
void esp8266Task::ESP8266Flush(void)
{
    char c = 0;
    while(mESP8266.getChar(&c, OS_MS(500))) {
        if(mESP8266Echo) {
            putchar(c);
        }
    }
}

void esp8266Task::ESP8266SendCmd(const char* pCmd, const char* pParam)
{
    mESP8266.put(pCmd);
    if(0 != pParam) {
        mESP8266.put(pParam);
    }
    mESP8266.put("\r\n");

    ESP8266Flush();
}

/*
void esp8266Task::ESP8266EnterCmdMode(void)
{
     Exit out of command mode just in case
    mESP8266.putline("exit");

    vTaskDelayMs(260);
    mESP8266.put("$$$");
    vTaskDelayMs(260);

    ESP8266Flush();
}
*/

bool esp8266Task::ESP8266HandleHttpReq(web_req_type request)
{
	STR_ON_STACK(rsp, 128);
    bool success = false;
    printf("\n\n%s\n", request.payload.c_str());
    printf("size is %d", request.payload.length());
    ESP8266SendCmd("AT+CIPSTART=\"TCP\",\"192.168.1.228\",80");
    std::string to_be_sent;
    to_be_sent = request.req + request.payload;
    std::string send_len = "AT+CIPSEND=";
    int len_send = to_be_sent.length();
    char len_send_c[3];
    sprintf(len_send_c, "%d", len_send);
    send_len += len_send_c;
    send_len += "\r\n";
    printf("\n\n\n\%s\n%d", to_be_sent.c_str(),to_be_sent.length());
	ESP8266SendCmd(send_len.c_str());
	ESP8266SendCmd(to_be_sent.c_str());
	mESP8266.gets((char*)rsp(), rsp.getCapacity(), 1000);
	printf("%s\n",rsp());
    ESP8266Flush();

    ESP8266SendCmd("AT+CIPCLOSE");

    return success;
}

bool esp8266Task::ESP8266Connect(void)
{
	STR_ON_STACK(rsp, 128);
	mESP8266.putline("AT+CWMODE=3");
	mESP8266.gets((char*)rsp(), rsp.getCapacity(), 1000);
	printf("\t%s\n",rsp());
	mESP8266.gets((char*)rsp(), rsp.getCapacity(), 1000);
	printf("\t%s\n",rsp());
	if(!rsp.beginsWithIgnoreCase("ok"))
	{
		printf("Unable to configure device as AP and station\n");
	}
	ESP8266Flush();
	mESP8266.putline("AT+CWJAP=\"Through Silence\",\"jjjjmmyy16\"");
	mESP8266.gets((char*)rsp(), rsp.getCapacity(), 1000);
	while(!(rsp.beginsWithIgnoreCase("ok")))
	{
		mESP8266.gets((char*)rsp(), rsp.getCapacity(), 1000);
		if((rsp.beginsWithIgnoreCase("ERROR")))
				{
					return false;
				}
		printf("\t%s\n",rsp());
	}

	ESP8266IsConnected();
	/*while(mESP8266.gets((char*)rsp(), rsp.getCapacity(), 1000));
	printf("\t%s\n",rsp());
	while(mESP8266.gets((char*)rsp(), rsp.getCapacity(), 1000));
	printf("\t%s\n",rsp());
	if(!rsp.beginsWithIgnoreCase("wifi connected"))
	{
		printf("Wifi connection failed\n");
		return false;
	}
	while(mESP8266.gets((char*)rsp(), rsp.getCapacity(), 1000));
	printf("\t%s\n",rsp());
	if(!rsp.beginsWithIgnoreCase("wifi got IP"))
	{
		printf("Wifi did not get IP\n");
		return false;
	}*/
    return true;
}

bool esp8266Task::ESP8266IsConnected(void)
{
    //ESP8266EnterCmdMode();
    mESP8266.putline("AT+CWJAP?");

    /**
     * Get response, which could either be : "show connection\n####" (uart echo)
     * or the response could be just "####" (status)
     */
    STR_ON_STACK(rsp, 128);
    mESP8266.gets((char*)rsp(), rsp.getCapacity(), 1000);
    printf("\t%s\n",rsp());
    mESP8266.gets((char*)rsp(), rsp.getCapacity(), 1000);
    if(!rsp.beginsWithIgnoreCase("+CWJAP:\"Through Silence\"")) {
    	return false;
    }


    printf("==========Connected!==============\n");
    return true;
}

void esp8266Task::ESP8266SendTestCmd(void)
{
    ESP8266Flush();
    //ESP8266EnterCmdMode();
    ESP8266Flush();
    mESP8266.putline("AT");
}

bool esp8266Task::ESP8266InitBaudRate(void)
{
    STR_ON_STACK(s, 128);

    /* If we can communicate right now, we've got baud rate match so return out of here */
    printf("    ESP8266 attempt communication @ %i bps\n", (int)mESP8266BaudRate);
    mESP8266.setBaudRate(mESP8266BaudRate);
    //vTaskDelayMs(5000);
    ESP8266SendTestCmd();

    /* ESP8266 should either echo back "ver" or send response with "wifly" in it */
    s.clear();
    mESP8266.gets((char*)s(), s.getCapacity(), 1000);
    printf("	recieved ini %s\n",s());
    mESP8266.gets((char*)s(), s.getCapacity(), 1000);
    printf("	recieved ini %s\n",s());
    mESP8266.gets((char*)s(), s.getCapacity(), 1000);
    printf("	recieved ini %s\n",s());
    if (s.containsIgnoreCase("OK")) {
        printf("    ESP8266 Baud Rate confirmed @ %i\n", (int)mESP8266BaudRate);
        //ESP8266SendCmd("exit");
        return true;
    }

    /* We cannot communicate, so try different baud rates */
    int baudRatesToTry[] = {9600, 38400, 115200, 230400, 460800};
    const int num_baud_rates = sizeof(baudRatesToTry) / sizeof(baudRatesToTry[0]);
    for (int i = 0; i < num_baud_rates; i++) {
        const int baudRateToTry = baudRatesToTry[i];

        printf("    ESP8266 attempt communication @ %i bps\n", baudRateToTry);
        ESP8266Flush();
        mESP8266.setBaudRate(baudRateToTry);
        ESP8266SendTestCmd();


        /* If we can enter command mode now, then we've found the baud rate!
         * So set/change the desired baud rate on the ESP8266 module
         */
        s.clear();
        if (mESP8266.gets((char*)s(), s.getCapacity(), 1000)) {
        	printf("	recieved %s\n",s());
            if(s.containsIgnoreCase("OK")) {
                printf("    ESP8266 Baud Rate is: %i bps\n", baudRateToTry);
/*
                printf("    Changing ESP8266 to %i bps\n", (int)mESP8266BaudRate);
                s.printf("set uart baudrate %i", mESP8266BaudRate);
                ESP8266SendCmd(s());
                ESP8266SendCmd("save");
                ESP8266SendCmd("reboot");

                ESP8266Flush();*/
                return true;
            }
            else {
                printf("    ESP8266 bad response: %s\n", s());
            }
        }
    }

    mESP8266.setBaudRate(mESP8266BaudRate);
    printf("    ESP8266 Baud Rate is UNKNOWN.  Set baud rate back to %i\n", (int) mESP8266BaudRate);
    return false;
}

esp8266Task::esp8266Task(UartDev& uartForESP8266, uint8_t priority) :
        scheduler_task("esp8266", 512*8, priority),
        mESP8266(uartForESP8266),
        mESP8266BaudRate(ESP8266_BAUD_RATE),
        mESP8266Echo(true)
{
    mHttpReqQueue = xQueueCreate(1, sizeof(web_req_type*));
    memset(mESP8266Ssid, 0, sizeof(mESP8266Ssid));
    memset(mESP8266Key, 0, sizeof(mESP8266Key));

    /* Default values, can be changed/saved during runtime */
    strncpy(mESP8266Ssid, ESP8266_SSID, sizeof(mESP8266Ssid) - 1);
    strncpy(mESP8266Key,  ESP8266_KEY , sizeof(mESP8266Key) - 1);
}

bool esp8266Task::init()
{
    return addSharedObject(ESP8266_SHR_OBJ, mHttpReqQueue);
}

bool esp8266Task::regTlm()
{
#if SYS_CFG_ENABLE_TLM
    /* User can change this during run-time by setting telemetry variable */
    tlm_component *disk = tlm_component_get_by_name(SYS_CFG_DISK_TLM_NAME);
    TLM_REG_VAR(disk, mESP8266Ssid, tlm_string);
    TLM_REG_VAR(disk, mESP8266Key,  tlm_string);
    TLM_REG_VAR(disk, mESP8266Echo, tlm_bit_or_bool);
    TLM_REG_VAR(disk, mESP8266BaudRate, tlm_uint);
#endif

    return true;
}

bool esp8266Task::taskEntry()
{
    // Not ready until changed otherwise
    mESP8266.setReady(false);

    /* If we cannot detect baud rate, error out from here, but return true
     * so that this task doesn't halt the whole system due to this error.
     */
    printf("Caling init baud rate\n");
    if(!ESP8266InitBaudRate()) {
        return true;
    }

    printf("calling isConnected\n");
    if(!ESP8266IsConnected()) {
        puts("    ESP8266 not connected");
        if(ESP8266Connect()) {
            puts("    ESP8266 is now connected!");
        }
        else {
            puts("    ESP8266 ERROR connecting");
        }
    }

    /* Display the IP address
     * AT+CIFSR

	+CIFSR:APIP,"192.168.4.1"
	+CIFSR:APMAC,"5e:cf:7f:82:f7:43"
	+CIFSR:STAIP,"192.168.0.126"
	+CIFSR:STAMAC,"5c:cf:7f:82:f7:43"

	OK
     *  */
    do {

        mESP8266.putline("AT+CIFSR");
        char buffer[128] = { 0 };
        mESP8266.gets(buffer, sizeof(buffer), 1000); /* AT+CIFSR */
        mESP8266.gets(buffer, sizeof(buffer), 1000); /* +CIFSR:APIP,"<IP>" */
        mESP8266.gets(buffer, sizeof(buffer), 1000); /* mac*/
        mESP8266.gets(buffer, sizeof(buffer), 1000); /* ip */
        printf("\t%s\n",buffer);
        mESP8266.gets(buffer, sizeof(buffer), 1000); /*mac */
        mESP8266.gets(buffer, sizeof(buffer), 1000);
    } while (0) ;

    ESP8266Flush();
    mESP8266.setReady(true);

    return true;
}

bool esp8266Task::run(void* p)
{
   /* web_req_type *request = 0;
    if (xQueueReceive(mHttpReqQueue, &(request), portMAX_DELAY)) {
        mESP8266.setReady(false);
        {
            request->success = ESP8266HandleHttpReq(request);
            if(request->req_done_signal) {
                xSemaphoreGive( (request->req_done_signal));
            }
        }
        mESP8266.setReady(true);
    }*/
	printf("running webreq\n");
	sysStat sys_stat;
	sys_stat.deviceID = "2";
	sys_stat.deviceTemp = "70";
	sys_stat.deviceBat = "10";
	sys_stat.deviceCPU = "20";
	sys_stat.deviceMem = "30";

	//printf("%s\n %s\n", sys_stat.deviceBat.c_str(), sys_stat.deviceCPU.c_str());
	 xQueueSend(sysStatQh, &sys_stat, portMAX_DELAY);
	 sysStat status;
	 web_req_type request;

	 // Wait 30 sec (or more) or if you don't want to poll, use a semaphore as signal
	 // webreq.req_done_signal semaphore is given after web request terminates
	 if (xQueueReceive(sysStatQh, &status, portMAX_DELAY)) {
	         mESP8266.setReady(false);
		 {
	       int len;
	       std:: string req("POST /deviceInfo HTTP/1.1\r\nHost: 192.168.1.228\r\ncontent-type: application/json\r\ncontent-length: ");
	       std:: string data;
	       data = "{\r\n  \"deviceID\": " + status.deviceID;
	       data += ",\r\n  \"deviceTemp\": ";
	       data += status.deviceTemp;
	       data += ",\r\n  \"deviceBatteryPercent\": ";
	       data +=  status.deviceBat;
	       data += ",\r\n  \"deviceCPUUsage\": ";
	       data += status.deviceCPU;
	       data += ",\r\n  \"deviceMemUsage\": ";
	       data += status.deviceMem;
	       data += "\r\n}";

	       len = data.length();
	       char a[3];
	       sprintf(a, "%d", len);
	       req = req + a + "\r\n\r\n";
	       request.payload = data;
	       request.req = req;
	       ESP8266HandleHttpReq(request);
	       vTaskDelay(5000);

		 }
	 }
    return true;
}