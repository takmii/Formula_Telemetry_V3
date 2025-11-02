#include "Formula_SIM7600G.h"

HardwareSerial simCOM(2);

String serialBuffer = " ";
bool networkDown = false;
unsigned long lastSimulate = 0;


String sendAT(const char* at, unsigned long timeout){ // Function to send AT commands and get the output from the router 
  static String msg;
  msg = "";
  simCOM.println(at);
  
  unsigned long start = millis(); 

  while (millis() - start < timeout){

    if (simCOM.available()) { 
        msg+=(char)simCOM.read();
    }    
  }
  return msg;
}

bool connection_val(){ // Connection and RF signal verification routine // Pendency - Insert error checking and correction (future)
  bool setup =0;
  static String data;
  data=sendAT("ATI"); // SimCOM module information 
  
  data=sendAT("AT+CGDCONT=1,\"IP\",\"tim.br\""); // Configuring the APN, may need some change depending on the operator 
  data=sendAT("AT+CGDCONT?"); // Verifying if APN was correctly configured 
  data=sendAT("AT+CGACT=1,1");

  data=sendAT("AT+CGACT?"); // Verifying logical connection between the modem and the mobile network 
  if(extrairCGACT(data)=="+CGACT: 1,1"){
    setup=1;
  }
  data=sendAT("AT+CGPADDR=1"); // Verifying IP Adresses acquired

  data=sendAT("AT+CPSI=0");

  data=sendAT("AT+CPSI?"); // RF signal parameters

  return setup;
}

void mqtt_start(){ // Start MQTT connection
  static String data;

  data=sendAT("AT+CMQTTSTART", 3000); // Start matt session

  data=sendAT("AT+CMQTTACCQ=0,\"esp32test1913\"", 3000); // Set the client ID
 
  data=sendAT("AT+CMQTTCONNECT=0,\"tcp://test.mosquitto.org:1883\",60,1", 3000); // Broker
  Serial.println(data); 

}

void mqtt_close(){ // Funcion to close MQTT connection
static String data;
 data=sendAT("AT+CMQTTDISC=0,60"); // Disconnect Client

 data=sendAT("AT+CMQTTREL=0"); // Release Client

 data=sendAT("AT+CMQTTSTOP"); // Interrupt MQTT session

}

void simulate_network_loss() { // Function to simulate loss signal  
  unsigned long now = millis();
  static String data;
  if (!networkDown && now - lastSimulate >= 50000) {
    Serial.println("\n[SYSTEM] Simulando perda de rede (AT+CGACT=0,1)");
    data=sendAT("AT+CFUN=0", 2000); // Enable the Airplane mode simulation connection interruption 
    networkDown = true;
    lastSimulate = now;
  }

  if (networkDown && now - lastSimulate >= 20000) {
    Serial.println("\n[SYSTEM] Reativando rede (AT+CGACT=1,1)");
    data=sendAT("AT+CFUN=1", 3000); // Disable the Airplane mode 
    networkDown = false;
    lastSimulate = now;
  }
}

String extrairCGACT(String resposta) {
  int inicio = resposta.indexOf("+CGACT:");
  if (inicio == -1) return ""; // não encontrou

  int fim = resposta.indexOf('\n', inicio);
  if (fim == -1) fim = resposta.length(); // caso não tenha quebra de linha

  String linha = resposta.substring(inicio, fim);
  linha.trim(); // remove \r ou espaços
  return linha;
}