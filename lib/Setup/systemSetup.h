#ifndef LIBS_H
#define LIBS_H

//#include <SdFat.h>
#include <Formula_SIM7600G.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <RTClib.h>
//#include <I2Cdev.h>
//#include <MPU6050.h>
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
//#include <CAN.h>
#include "driver/twai.h"
#include <CAN_Messages.h>


#if __has_include("home_settings.h")
    #include "home_settings.h"
    #define D_WIFI false
#else
    #define D_WIFI true
#endif

#define SD_FREQ_MHZ(mhz) ((uint32_t)(1000000UL * (mhz)))

#define SD_TASK_TIMER CAN_TASK_TIMER
#define RPM_TIMER CAN_TASK_TIMER
#define RPM_ZERO_TIMER 15000
#define SD_FLUSH_TIMER 1000
#define SD_VERIFY_FILESIZE_TIMER 60000
#define SD_VERIFY_TIMER 5000
#define MESSAGES_TIMER 10000
#define TEMP_TIMER 250
#define SIM_SETUP_TIMER 500
#define SIM_TIMER 100
#define CALIBRACAO_TIMER 500
#define ACC_TIMER 5

#define CAN_TX_PIN GPIO_NUM_22
#define CAN_RX_PIN GPIO_NUM_21

#define RX_PIN GPIO_NUM_16
#define TX_PIN GPIO_NUM_17

#define TIMEBASE 100

#define BUFFER_LENGTH TIMEBASE/SD_TASK_TIMER
#define BUFFER_NUMBER 2
#define MAX_SENSORS 85
#define BUFFER_SIZE 7

#define RPM_PIN 32

#define GPIO_LEDRPM GPIO_NUM_0

#define GY906_ADD 0x5A

void writeCSV(const char * filename, __u8 size);
void writeHeader(const char * filename, __u8 size);
void createDirectory(const char * dir);
String verifyFilename(String filename);

void sdTask(void *parameter);
void sdFlush(void *parameter);
//void sdFile(void *parameter);
void Calibracao(void *parameter);
void sdVerify(void *parameter);
void CAN_receiveTask(void *parameter);
void RPM_task(void *parameter);
void AccelGyro_task1(void *parameter);
void TempTask(void *parameter);
void SIM_Task(void *parameter);
void MessagesFN(void *parameter);
void disableBluetooth();
void getRTC();
void setRTC();

void writeSDCard();


void CAN_setSensor(const __u8 *canData, __u8 canPacketSize,__u32 canId);
void sendCANMessage(uint8_t id, uint8_t *data, uint8_t dlc);


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
void fn_Data_10(__u8 data[DATA_10_DLC]);
void fn_Buffer_Ack(__u8 data[BUFFER_ACK_DLC]);
void fn_Debug(__u8 data[DEBUG_DLC]);
void fn_Temp(__u8 data[TEMP_DLC]);

void fn_Group_0(__u8 data[GROUP0_DLC]);
void fn_Group_1(__u8 data[GROUP1_DLC]);
void fn_Group_2(__u8 data[GROUP2_DLC]);
void fn_Group_3(__u8 data[GROUP3_DLC]);
void fn_Group_7(__u8 data[GROUP7_DLC]);
void fn_Group_8(__u8 data[GROUP8_DLC]);
void fn_Group_9(__u8 data[GROUP9_DLC]);
void fn_Group_15(__u8 data[GROUP15_DLC]);

void IRAM_ATTR handleEdge();

void init_twai();

uint8_t readRegister(uint8_t reg);

TwoWire I2C_MPU6050 = TwoWire(1);
const uint8_t MPU_addr = 0x68;
//MPU6050 mpu(MPU_addr,&I2C_MPU6050);


Adafruit_MLX90614 mlx = Adafruit_MLX90614();

WiFiClientSecure espClient;
PubSubClient client(espClient);


const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = (-3) * 3600;         
const int   daylightOffset_sec = 0;

const char* mqtt_login = "Gateway";
const char* mqtt_pwd = "#0FormulaUTFPR";

const float FtC = 0.55;


template <typename T>
void sensorUpdate(T value, __u8 index);



#endif