#ifndef FORMULA_SIM7600G_H
#define FORMULA_SIM7600G_H

#include <HardwareSerial.h>
//#include <Formula_FreeRTOS.h>

#define PAYLOAD_SIZE 256

extern HardwareSerial simCOM;

extern SemaphoreHandle_t xMqttMutex;

String sendAT(const char* at, unsigned long timeout=1000);
String readSIM(unsigned long timeout=1000);
unsigned char connection_val();
bool mqtt_start(bool first_time);
bool mqtt_close(bool first_time);
bool mqtt_publish(const char* topic, const char* payload);
void simulate_network_loss();
void clearRxBuffer();

void setPayload(unsigned char byte,unsigned short data);
void setPayload32(unsigned char byte,unsigned int data);
void clearPayload();

extern char mqtt_payload[PAYLOAD_SIZE];
extern char ATpayload[PAYLOAD_SIZE + 32];
extern unsigned short payload_index;

String extrairCGACT(String resposta);
String extrairOK(String resposta);

extern unsigned char time_byte;
extern unsigned char rpm_byte;

extern unsigned char mqtt_rate_count;

extern String serialBuffer;
extern bool networkDown;
extern unsigned long lastSimulate;



#endif