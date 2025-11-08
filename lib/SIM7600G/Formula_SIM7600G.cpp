#include "Formula_SIM7600G.h"

HardwareSerial simCOM(2);

String serialBuffer = " ";
bool networkDown = false;
unsigned long lastSimulate = 0;

char mqtt_payload[PAYLOAD_SIZE];
char ATpayload[PAYLOAD_SIZE + 32];

unsigned short payload_index = 0;

unsigned char time_byte = 'a';
unsigned char rpm_byte = 'b';

unsigned char mqtt_rate_count = 5;

String sendAT(const char *at, unsigned long timeout)
{ // Function to send AT commands and get the output from the router
  clearRxBuffer();
  simCOM.println(at);
  return readSIM(timeout);
}

String readSIM(unsigned long timeout)
{ // Function to send AT commands and get the output from the router
  String msg= "";

  TickType_t startTick = xTaskGetTickCount();
  TickType_t timeoutTicks = pdMS_TO_TICKS(timeout);
  TickType_t lastDataTick = startTick;

  bool get = 0;
  unsigned long timeoff = 0;
  while (xTaskGetTickCount() - startTick < timeoutTicks)
  {

    if (simCOM.available())
    {
      char c = simCOM.read();
      msg += c;
      get = true;
      lastDataTick = xTaskGetTickCount();
    }
    if (get && (xTaskGetTickCount() - lastDataTick) >= pdMS_TO_TICKS(200))
    {
      break;
    }
    vTaskDelay(1);
  }
  return msg;
}

unsigned char connection_val()
{ // Connection and RF signal verification routine // Pendency - Insert error checking and correction (future)
  Serial.println("Chegou 0");
  static unsigned char order = 0;
  bool setup = 0;
  static bool first_time = true;
  static String data;

  if (order == 0)
  {
    data = sendAT("AT"); // Basic communication test
    Serial.println(data);
    if (data.indexOf("OK") != -1)
    {
      data = sendAT("AT+IPR=115200"); // Setting the baud rate to 115200
      Serial.println(data);
      data = sendAT("AT&W"); // salva configuração
      Serial.println(data);
      data = sendAT("AT+IFC=0,0"); // Desabilita controle de fluxo
      Serial.println(data);
      data = sendAT("AT&W"); // salva configuração
      Serial.println(data);
      order = 1;
    }
  }

  if (order == 1)
  {
    data = sendAT("ATI", 5000); // SimCOM module information
    Serial.println(data);
    if (data.indexOf("SIM7600G") != -1)
    {
      order = 2;
    }
  }

  if (order == 2)
  {
    data = sendAT("AT+CPIN?"); // SIM card status verification
    Serial.println(data);
    if (data.indexOf("READY") != -1)
    {
      order = 3;
    }
  }
  if (order == 3)
  {
    data = sendAT("AT+CGDCONT=1,\"IP\",\"zap.vivo.com.br\""); // Configuring the APN, may need some change depending on the operator
    Serial.println(data);
    if (data.indexOf("OK") != -1)
    {
      order = 4;
    }
  }
  if (order == 4)
  {
    data = sendAT("AT+CGDCONT?"); // Verifying if APN was correctly configured
    Serial.println(data);
    if (data.indexOf("OK") != -1)
    {
      order = 5;
    }
  }

  if (order == 5)
  {
    data = sendAT("AT+CGACT=1,1", 5000); // Activating the data connection
    Serial.println(data);
    if (data.indexOf("OK") != -1)
    {
      order = 6;
    }
  }

  if (order == 6)
  {
    data = sendAT("AT+CGACT?"); // Verifying logical connection between the modem and the mobile network
    Serial.println(data);
    if (data.indexOf("+CGACT: 1,1") != -1)
    {
      order = 7;
    }
  }

  if (order == 7)
  {
    data=sendAT("AT+CGPADDR=1"); // Verifying IP Adresses acquired
    Serial.println(data);
    if (data.indexOf("OK") != -1)
    {
      order = 8;
    }
  }

  if (order == 8)
  {
    data=sendAT("AT+NETCLOSE"); // Closing TCP/IP context if previously opened
    Serial.println(data);
    if (data.indexOf("OK") != -1 || data.indexOf("+NETCLOSE: 2") != -1)
    {
      order = 9;
    }
  }

  if (order == 9)
  {
    data = sendAT("AT+NETOPEN", 10000); // Open TCP/IP context
    Serial.println(data);
    if (data.indexOf("OK") != -1)
    {
      order = 10;
      setup = 1;
    }
  }

  /*
  data = sendAT("AT+NETOPEN", 10000); // Open TCP/IP context
  Serial.println(data);

  if (data.indexOf("OK") != -1 || data.indexOf("+NETOPEN: 0") != -1) {
    setup = 1;
  }

  data=sendAT("AT+CPSI=0");

  data=sendAT("AT+CPSI?"); // RF signal parameters*/

  return setup;
}

bool mqtt_start(bool first_time)
{ // Start MQTT connection
  bool status = 0;
  static unsigned char order;
  if (first_time){
    order = 0;
  }
  static String data;

  if (order == 0)
  {
    data = sendAT("AT+CMQTTSTART", 8000); // Start mqtt session
    Serial.println(data);
    if (data.indexOf("OK") != -1 || data.indexOf("+CMQTTSTART: 0") != -1)
    {
      order = 1;
    }
  }

  if (order == 1)
  {
    data = sendAT("AT+CMQTTACCQ=0,\"Formula_GTW\"", 2000); // Set the client ID
    Serial.println(data);
    if (data.indexOf("OK") != -1)
    {
      order = 2;
    }
  }

  if (order == 2)
  {
    data = sendAT("AT+CMQTTCONNECT=0,\"tcp://broker.hivemq.com:1883\",60,1", 8000); // Broker
    Serial.println(data);
    if (data.indexOf("OK") != -1)
    {
      order = 3;
    }
  }

  if (order == 3)
  {
    data = readSIM(10000); // Broker
    Serial.println(data);
    if (data.indexOf("+CMQTTCONNECT: 0,0") != -1)
    {
      vTaskDelay(pdMS_TO_TICKS(500));
      status = 1;
    }
  }


  return status;
}

bool mqtt_close(bool first_time)
{ // Funcion to close MQTT connection
  bool status = 0;
  static unsigned char order;
  if (first_time){
    order = 0;
  }
  static String data;

  if (order == 0)
  {
    data = sendAT("AT+CMQTTDISC=0,60"); // Disconnect Client
    Serial.println(data);
    if (data.indexOf("OK") != -1 || data.indexOf("+CMQTTDISC: 0,0") != -1)
    {
      order = 1;
    }
  }

  if (order == 1)
  {
    data = sendAT("AT+CMQTTREL=0"); // Release Client
    Serial.println(data);
    if (data.indexOf("OK") != -1)
    {
      order = 2;
    }
  }

  if (order == 2)
  {
    data = sendAT("AT+CMQTTSTOP"); // Interrupt MQTT session
    Serial.println(data);
    if (data.indexOf("OK") != -1)
    {
      status = 1;
    }
  }
  return status;
}

void simulate_network_loss()
{ // Function to simulate loss signal
  unsigned long now = millis();
  static String data;
  if (!networkDown && now - lastSimulate >= 50000)
  {
    Serial.println("\n[SYSTEM] Simulando perda de rede (AT+CGACT=0,1)");
    data = sendAT("AT+CFUN=0", 2000); // Enable the Airplane mode simulation connection interruption
    networkDown = true;
    lastSimulate = now;
  }

  if (networkDown && now - lastSimulate >= 20000)
  {
    Serial.println("\n[SYSTEM] Reativando rede (AT+CGACT=1,1)");
    data = sendAT("AT+CFUN=1", 3000); // Disable the Airplane mode
    networkDown = false;
    lastSimulate = now;
  }
}

String extrairCGACT(String resposta)
{
  int inicio = resposta.indexOf("+CGACT:");
  if (inicio == -1)
    return ""; // não encontrou

  int fim = resposta.indexOf('\n', inicio);
  if (fim == -1)
    fim = resposta.length(); // caso não tenha quebra de linha

  String linha = resposta.substring(inicio, fim);
  linha.trim(); // remove \r ou espaços
  return linha;
}

String extrairOK(String resposta)
{
  int inicio = resposta.indexOf("OK");
  if (inicio == -1)
    return ""; // não encontrou

  int fim = resposta.indexOf('\n', inicio);
  if (fim == -1)
    fim = resposta.length(); // caso não tenha quebra de linha

  String linha = resposta.substring(inicio, fim);
  linha.trim(); // remove \r ou espaços
  return linha;
}

bool mqtt_publish(const char *topic, const char *payload)
{ // Publish the payload
  bool send = 0;
  char localPayload[PAYLOAD_SIZE+1]; // buffer interno, protegido
  if (xSemaphoreTake(xMqttMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    strncpy(localPayload, payload, sizeof(localPayload) - 1);
    localPayload[sizeof(localPayload) - 1] = '\0';
    xSemaphoreGive(xMqttMutex);
  }

  bool status = 0;
  String data;
  bool sent = false;
  char ATtopic[64];

  Serial.print("PL: ");
  Serial.println(localPayload);
  Serial.println(topic);

  sprintf(ATtopic, "AT+CMQTTTOPIC=0,%d", strlen(topic)); // Topic
  data = sendAT(ATtopic, 2000);
  Serial.println(data);
  if (data.indexOf(">") == -1){
    return false;
  }

  simCOM.print(topic);
  simCOM.write(0X1A);
  data = readSIM(2000);
  Serial.println(data);
  if (data.indexOf("OK") == -1) return false;

  sprintf(ATpayload, "AT+CMQTTPAYLOAD=0,%d", strlen(localPayload));
  data = sendAT(ATpayload, 2000);
  Serial.println(data);
  if (data.indexOf(">") == -1){
    return false;
  }
  simCOM.print(localPayload);
  simCOM.write(0X1A);
  data = readSIM(2000);
  Serial.println(data);
  if (data.indexOf("OK") == -1) return false;
  
  data = sendAT("AT+CMQTTPUB=0,0,60", 7000);
  if (data.indexOf("+CMQTTPUB: 0,0") != -1)
  {
    clearPayload();
    return true;
  }
  return false;
}

void clearPayload()
{
  if (xSemaphoreTake(xMqttMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
  memset(mqtt_payload, 0, PAYLOAD_SIZE);
  payload_index = 0;
  xSemaphoreGive(xMqttMutex);
  }
}

void setPayload(unsigned char byte, unsigned short data)
{
  if (xSemaphoreTake(xMqttMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
  if (payload_index + 6 <= PAYLOAD_SIZE)
  {
    mqtt_payload[payload_index++] = byte;
    mqtt_payload[payload_index++] = '~';
    mqtt_payload[payload_index++] = 0x20 + ((data >> 10) & 0x3F);
    mqtt_payload[payload_index++] = 0x20 + ((data >> 5) & 0x1F);
    mqtt_payload[payload_index++] = 0x20 + (data & 0x1F);
  }
  xSemaphoreGive(xMqttMutex);
  }
}

void setPayload32(unsigned char byte, unsigned int data)
{
  if (xSemaphoreTake(xMqttMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
  if (payload_index + 6 <= PAYLOAD_SIZE)
  {
    mqtt_payload[payload_index++] = byte;
    mqtt_payload[payload_index++] = '~';
    mqtt_payload[payload_index++] = 0x20 + ((data >> 18) & 0x3F);
    mqtt_payload[payload_index++] = 0x20 + ((data >> 12) & 0x3F);
    mqtt_payload[payload_index++] = 0x20 + ((data >> 6) & 0x3F);
    mqtt_payload[payload_index++] = 0x20 + ((data) & 0x3F);
  }
  xSemaphoreGive(xMqttMutex);
  }
}

bool waitForModuleReady(unsigned long timeout = 10000)
{
  String msg = "";
  unsigned long start = millis();
  bool ready = false;

  while (millis() - start < timeout)
  {
    while (simCOM.available())
    {
      char c = simCOM.read();
      msg += c;

      // Verifica se já recebeu "PB DONE"
      if (msg.indexOf("PB DONE") != -1)
      {
        ready = true;
        break;
      }
    }
    if (ready)
      break;
  }

  if (ready)
  {
    Serial.println("✅ Módulo pronto para comandos AT!");
  }
  else
  {
    Serial.println("⚠️ Timeout esperando o módulo iniciar.");
  }

  return ready;
}

void clearRxBuffer() {
  while (simCOM.available()) {
    simCOM.read();
  }
}
