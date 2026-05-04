#ifndef FORMULA_SIM7600G_H
#define FORMULA_SIM7600G_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "MQTT_credentials.h"

#define PAYLOAD_SIZE 256

extern HardwareSerial simCOM;
extern SemaphoreHandle_t xMqttMutex;

unsigned char sendAT(const char *at, unsigned long timeout=2000, const char *expected_answer = "OK", const char *error_answer = "ERROR");
unsigned char readSIM(unsigned long timeout, const char *expected_answer = "OK", const char *error_answer = "ERROR");
//bool waitForModuleReady(unsigned long timeout = 10000);
//bool waitResponse(const char* expected, unsigned long timeout);

void SIM7600_Init_Manager();
unsigned char SIM7600_Network_Manager(unsigned char set_order = 255);

bool mqtt_start(bool first_time); 
bool mqtt_close(bool first_time);
bool mqtt_publish(const char* topic, const char* payload);

void setPayload(unsigned char byte,unsigned short data);
void setPayload32(unsigned char byte,unsigned int data);
void clearPayload();
void clearRxBuffer();

extern char mqtt_payload[PAYLOAD_SIZE];
extern unsigned short payload_index;

extern unsigned char time_byte;
extern unsigned char rpm_byte;
extern String serialBuffer;

#endif