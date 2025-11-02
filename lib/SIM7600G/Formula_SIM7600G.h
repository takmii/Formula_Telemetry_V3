#ifndef FORMULA_SIM7600G_H
#define FORMULA_SIM7600G_H

#include <HardwareSerial.h>

extern HardwareSerial simCOM;

String sendAT(const char* at, unsigned long timeout=1000);
bool connection_val();
void mqtt_start();
void mqtt_close();
void simulate_network_loss();

String extrairCGACT(String resposta);

extern String serialBuffer;
extern bool networkDown;
extern unsigned long lastSimulate;



#endif