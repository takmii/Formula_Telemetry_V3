#ifndef LIBS_H
#define LIBS_H

//#include <SdFat.h>
#include <Wire.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <pins.h>
#include <sensorsSetup.h>
#include <variables.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <timeSetup.h>
#include <PubSubClient.h>
#include <CAN.h>
#include <CAN_Messages.h>


#if __has_include("home_settings.h")
    #include "home_settings.h"
    #define D_WIFI false
#else
    #define D_WIFI true
#endif

#define SD_FREQ_MHZ(mhz) ((uint32_t)(1000000UL * (mhz)))

#define SD_TASK_TIMER CAN_TASK_TIMER
#define SD_FLUSH_TIMER 1000

#define CAN_TX_PIN GPIO_NUM_22
#define CAN_RX_PIN GPIO_NUM_21

#define TIMEBASE 100

#define BUFFER_LENGTH TIMEBASE/SD_TASK_TIMER
#define BUFFER_NUMBER 2
#define MAX_SENSORS 42
#define BUFFER_SIZE 7




void writeCSV(const char * filename, __u8 size);
void writeHeader(const char * filename, __u8 size);
void createDirectory(const char * dir);
String verifyFilename(String filename);

void sdTask(void *parameter);
void sdFlush(void *parameter);
void CAN_receiveTask(void *parameter);
void disableBluetooth();
void getRTC();
void setRTC();

void writeSDCard();


void CAN_setSensor(const __u8 *canData, __u8 canPacketSize,__u32 canId);


void fn_Messages(__u8 data[MESSAGES_DLC]);
void fn_Data_01(__u8 data[DATA_01_DLC]);
void fn_Data_02(__u8 data[DATA_02_DLC]);
void fn_Data_03(__u8 data[DATA_03_DLC]);
void fn_Data_04(__u8 data[DATA_04_DLC]);
void fn_Data_05(__u8 data[DATA_05_DLC]);
void fn_Data_06(__u8 data[DATA_06_DLC]);
void fn_Data_07(__u8 data[DATA_07_DLC]);
void fn_Data_08(__u8 data[DATA_08_DLC]);
void fn_Data_09(__u8 data[DATA_09_DLC]);
void fn_Debug(__u8 data[DEBUG_DLC]);




WiFiClientSecure espClient;
PubSubClient client(espClient);


const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = (-3) * 3600;         
const int   daylightOffset_sec = 0;

const char* mqtt_login = "Gateway";
const char* mqtt_pwd = "#0FormulaUTFPR";



template <typename T>
void sensorUpdate(T value, __u8 index);



#endif