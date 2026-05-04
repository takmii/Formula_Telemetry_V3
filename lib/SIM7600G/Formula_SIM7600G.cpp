/*bool mqtt_start(bool first_time)
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
    if (data.indexOf("OK") != -1 || data.indexOf("+CMQTTSTART: 0") != -1)
    {
      order = 1;
    }
  }

  if (order == 1)
  {
    data = sendAT("AT+CMQTTACCQ=0,\"Formula_GTW\"", 2000); // Set the client ID
    if (data.indexOf("OK") != -1)
    {
      order = 2;
    }
  }

  if (order == 2)
  {
    data = sendAT("AT+CMQTTCONNECT=0,\"tcp://broker.hivemq.com:1883\",60,1", 8000); // Broker
    if (data.indexOf("OK") != -1)
    {
      order = 3;
    }
  }

  if (order == 3)
  {
    data = readSIM(10000); // Broker
    if (data.indexOf("+CMQTTCONNECT: 0,0") != -1)
    {
      vTaskDelay(pdMS_TO_TICKS(500));
      status = 1;
    }
  }


  return status;
}

bool mqtt_start(bool first_time)
{ 
  bool status = 0;
  static unsigned char order = 0;
  
  // Se for primeira vez ou reinício, reseta o passo
  if (first_time) order = 0;
  
  String data;

  if (order == 0)
  {
    // Para qualquer sessão anterior pendente
    sendAT("AT+CMQTTSTOP"); 
    vTaskDelay(pdMS_TO_TICKS(500));
    
    data = sendAT("AT+CMQTTSTART", 5000);
    if (data.indexOf("OK") != -1 || data.indexOf("+CMQTTSTART: 0") != -1)
    {
       order = 1;
    }
  }

  if (order == 1)
  {
    // Define o Client ID (Apenas o ID, sem senha aqui)
    // O ID pode ser fixo ou dinâmico. Vou usar um fixo genérico ou você pode criar um define.
    char cmd[64];
    sprintf(cmd, "AT+CMQTTACCQ=0,\"Formula_Car_01\",0"); 
    data = sendAT(cmd, 2000);
    if (data.indexOf("OK") != -1)
    {
       order = 2;
    }
  }

  if (order == 2)
  {
    char cmd[256]; // Buffer maior para caber URL + User + Pass
    
    
    sprintf(cmd, "AT+CMQTTCONNECT=0,\"%s:%s\",60,1,\"%s\",\"%s\"", 
            mqtt_url,   // "tcp://mrrpformula.ddns.net"
            mqtt_port,  // "1883"
            mqtt_login, // "tlmty_module"
            mqtt_pwd    // "eeeformulatouro"
    );
    
    Serial.print("Conectando: ");
    Serial.println(cmd);
    
    simCOM.println(cmd); 
    
    // Espera até 15s pela resposta "+CMQTTCONNECT: 0,0" (0,0 significa sucesso)
    if (waitResponse("+CMQTTCONNECT: 0,0", 15000)) { 
        Serial.println("MQTT Conectado e Autenticado!");
        status = 1;
        // Se conectou, não precisa avançar 'order', mantemos o estado pronto.
    } else {
        Serial.println("Falha na conexão/autenticação MQTT");
        // Se falhar (ex: erro 0,5 ou timeout), voltamos order=0 para tentar resetar tudo no próximo loop
        order = 0; 
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
    if (data.indexOf("OK") != -1 || data.indexOf("+CMQTTDISC: 0,0") != -1)
    {
      order = 1;
    }
  }

  if (order == 1)
  {
    data = sendAT("AT+CMQTTREL=0"); // Release Client
    if (data.indexOf("OK") != -1)
    {
      order = 2;
    }
  }

  if (order == 2)
  {
    data = sendAT("AT+CMQTTSTOP"); // Interrupt MQTT session
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

bool mqtt_publish(const char *topic, const char *payload)
{
  // 1. Copia Payload de forma segura (como você já fazia)
  char localPayload[PAYLOAD_SIZE+1];
  if (xSemaphoreTake(xMqttMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    strncpy(localPayload, payload, sizeof(localPayload) - 1);
    localPayload[sizeof(localPayload) - 1] = '\0';
    xSemaphoreGive(xMqttMutex);
  } else {
    return false; // Não conseguiu pegar o mutex
  }

  // Verifica tamanhos
  int lenTopic = strlen(topic);
  int lenPayload = strlen(localPayload);
  
  if (lenTopic == 0 || lenPayload == 0) return false;

  // --- PASSO 1: DEFINIR TÓPICO ---
  char cmd[64];
  sprintf(cmd, "AT+CMQTTTOPIC=0,%d", lenTopic);
  simCOM.println(cmd);
  
  // ESPERA O PROMPT '>'
  if (!waitResponse(">", 2000)) {
      Serial.println("Erro: Timeout esperando > para Topico");
      return false;
  }
  
  // Envia Tópico
  simCOM.print(topic);
  // Não usamos println aqui, apenas os bytes crus. 
  // O SIM7600 espera o Ctrl+Z pra confirmar ou o tamanho exato? 
  // No modo MQTT ele geralmente espera tamanho exato OU Ctrl+Z. Vamos garantir.
  // simCOM.write(0x1A); <--- Em alguns firmwares não precisa se o tamanho for exato, mas no geral precisa confirmar.
  // Vamos enviar o Ctrl+Z para garantir.
  simCOM.write(0x1A);

  if (!waitResponse("OK", 2000)) {
      Serial.println("Erro: Topic não aceito");
      return false;
  }

  // --- PASSO 2: DEFINIR PAYLOAD ---
  sprintf(cmd, "AT+CMQTTPAYLOAD=0,%d", lenPayload);
  simCOM.println(cmd);

  // ESPERA O PROMPT '>'
  if (!waitResponse(">", 2000)) {
      Serial.println("Erro: Timeout esperando > para Payload");
      return false;
  }

  // Envia Payload
  simCOM.print(localPayload);
  simCOM.write(0x1A); // Confirma envio

  if (!waitResponse("OK", 2000)) {
      Serial.println("Erro: Payload não aceito");
      return false;
  }

  // --- PASSO 3: PUBLICAR ---
  // Publica pacote 0, QoS 0, Timeout 60s
  simCOM.println("AT+CMQTTPUB=0,0,60");
  
  if (waitResponse("+CMQTTPUB: 0,0", 5000)) {
      clearPayload(); // Limpa só se deu sucesso
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
*/

#include "Formula_SIM7600G.h"

HardwareSerial serialCOM(2);

String serialBuffer = " ";
bool networkDown = false;
unsigned long lastSimulate = 0;

char mqtt_payload[PAYLOAD_SIZE];
char ATpayload[PAYLOAD_SIZE + 32];

unsigned short payload_index = 0;

unsigned char time_byte = 'a';
unsigned char rpm_byte = 'b';

unsigned char mqtt_rate_count = 5;

unsigned char sendAT(const char *at, unsigned long timeout, const char *expected_answer, const char *error_answer)
{ // Function to send AT commands and get the output from the router
  clearRxBuffer();
  serialCOM.println(at);
  return readSIM(timeout, expected_answer, error_answer);
}

unsigned char readSIM(unsigned long timeout, const char *expected_answer, const char *error_answer)
{ // Function to send AT commands and get the output from the router
  String msg= "";

  TickType_t startTick = xTaskGetTickCount();
  TickType_t timeoutTicks = pdMS_TO_TICKS(timeout);

  //unsigned long timer = millis();

  bool get = 0;
  while (xTaskGetTickCount() - startTick < timeoutTicks)
  //while(millis()-timer<timeout)
  {

    if (serialCOM.available())
    {
      char c = serialCOM.read();
      msg += c;
      get = true;
      
    }
    if (get){
      if (msg.indexOf(expected_answer) != -1 || msg.indexOf("OK") != -1)
      {
        return 0;
      }
      else if(msg.indexOf(">") != -1) 
      {
        return 3; 
      }
      else if (msg.indexOf(error_answer) != -1 || msg.indexOf("ERROR") != -1) 
      {
        return 1; 
      }
    }
  }
  return 2;
}

void SIM7600_Init_Manager(){
  unsigned char response = 2;
  bool setup = false;
  vTaskDelay(pdMS_TO_TICKS(500));
  while(serialCOM.available()) serialCOM.read();
  while (!setup){
    if (serialCOM.available()) {
        response += (char)serialCOM.read();
    }
   response = sendAT("AT");
   Serial.println(response);
   if (!response){
    setup = true;
   }
   if(!setup){
    vTaskDelay(pdMS_TO_TICKS(1000));
   }
  }
}

unsigned char SIM7600_Network_Manager(unsigned char set_order)
{
  static unsigned char order = 0;
  if (set_order != 255)
  {
    order = set_order;
  }
  Serial.print("NM");
  Serial.println(order);
  bool setup = 1;

  if (order == 0)
  {
     // Basic communication test
    if (!sendAT("AT",1000))
    {
      order = 1;
    }
  }

  if (order == 1)
  {
    // SimCOM module information
    if (!sendAT("ATI", 5000,"SIM7600G"))
    {
      order = 2;
    } 
  }

  if (order == 2)
  {
     // SIM card status verification
    if (!sendAT("AT+CPIN?",1000,"READY"))
    {
      order = 3;
    }
  }
  if (order == 3)
  {
     // Configuring the APN, may need some change depending on the operator
    if (!sendAT("AT+CGDCONT=1,\"IP\",\"zap.vivo.com.br\"",1000))
    {
      order = 4;
    }
  }
  if (order == 4)
  {
     // Verifying if APN was correctly configured
    if (!sendAT("AT+CGDCONT?",1000))
    {
      order = 5;
    }
  }

  if (order == 5)
  {
     // Activating the data connection
    if (!sendAT("AT+CGACT=1,1", 5000))
    {
      order = 6;
    }
  }

  if (order == 6)
  {
     // Verifying logical connection between the modem and the mobile network
    if (!sendAT("AT+CGACT?",1000,"+CGACT: 1,1"))
    {
      order = 7;
    }
  }

  if (order == 7)
  {
     // Verifying IP Adresses acquired
    if (!sendAT("AT+CGPADDR=1",1000))
    {
      order = 8;
    }
  }

  if (order == 8)
  {
     // Closing TCP/IP context if previously opened
    if (!sendAT("AT+NETCLOSE",1000,"+NETCLOSE: 2"))
    {
      order = 9;
    }
  }

  if (order == 9)
  {
     // Open TCP/IP context
    if (!sendAT("AT+NETOPEN", 10000))
    {
      order = 10;
      setup = 0;
    }
  }
  return setup;
}

bool mqtt_start(bool first_time)
{ 
  bool status = 1;
  static unsigned char order;
  if (first_time) {
    order = 0;
  }
  unsigned char connection_response;
  char mqtt_connect_cmd[256];
  Serial.print("MQ_S");
  Serial.println(order);

  if (order == 0)
  {
    if (!sendAT("AT+CMQTTSTART", 8000,"+CMQTTSTART: 0"))
    {
      order = 1;
    }
  }

  if (order == 1)
  {
    if (!sendAT("AT+CMQTTACCQ=0,\"Formula_GTW\"", 2000))
    {
      order = 2;
    }
  }

  // Configurar usuário MQTT
  if (order == 2)
  {
    char user_cmd[32];
    sprintf(user_cmd, "AT+CMQTTUSERNAME=0,\"%s\"", mqtt_login);
    if (!sendAT(user_cmd, 2000))
    {
      order = 3;
    }
  }

  // Configurar senha MQTT
  if (order == 3)
  {
    char pwd_cmd[32];
    sprintf(pwd_cmd, "AT+CMQTTPASSWORD=0,\"%s\"", mqtt_pwd);
    if (!sendAT(pwd_cmd, 2000))
    {
      order = 4;
    }
  }

  // Conectar ao broker usando as credenciais
  if (order == 4)
  {
    // Construir a URL completa do broker
    sprintf(mqtt_connect_cmd, "AT+CMQTTCONNECT=0,\"%s:%s\",60,1", mqtt_url, mqtt_port);
    if (!sendAT(mqtt_connect_cmd, 8000))
    {
      order = 5;
    }
  }

  if (order == 5)
  {
    connection_response = readSIM(10000,"+CMQTTCONNECT: 0,0", "+CMQTTCONNECT: 0,1"); 
    if (!connection_response)
    {
      Serial.println("✅ MQTT conectado com sucesso!");
      vTaskDelay(pdMS_TO_TICKS(500));
      status = 0;
    }
    else if (connection_response==1)
    {
      Serial.println("Erro na conexão MQTT");
        status = 1;
      }
    }

  return status;
}

bool mqtt_close(bool first_time)
{ 
  bool status = 0;
  static unsigned char order;
  if (first_time) {
    order = 0;
  }
  static String data;

  if (order == 0)
  {
    data = sendAT("AT+CMQTTDISC=0,60"); 
    if (data.indexOf("OK") != -1 || data.indexOf("+CMQTTDISC: 0,0") != -1)
    {
      order = 1;
    }
  }

  if (order == 1)
  {
    data = sendAT("AT+CMQTTREL=0"); 
    if (data.indexOf("OK") != -1)
    {
      order = 2;
    }
  }

  if (order == 2)
  {
    data = sendAT("AT+CMQTTSTOP"); 
    if (data.indexOf("OK") != -1)
    {
      status = 1;
    }
  }
  return status;
}

void simulate_network_loss()
{ 
  unsigned long now = millis();
  static String data;
  if (!networkDown && now - lastSimulate >= 50000)
  {
    Serial.println("\n[SYSTEM] Simulando perda de rede (AT+CGACT=0,1)");
    data = sendAT("AT+CFUN=0", 2000); 
    networkDown = true;
    lastSimulate = now;
  }

  if (networkDown && now - lastSimulate >= 20000)
  {
    Serial.println("\n[SYSTEM] Reativando rede (AT+CGACT=1,1)");
    data = sendAT("AT+CFUN=1", 3000); 
    networkDown = false;
    lastSimulate = now;
  }
}

bool mqtt_publish(const char *topic, const char *payload)
{ 
  bool send = 0;
  char localPayload[PAYLOAD_SIZE + 1]; 
  
  if (xSemaphoreTake(xMqttMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    strncpy(localPayload, payload, sizeof(localPayload) - 1);
    localPayload[sizeof(localPayload) - 1] = '\0';
    xSemaphoreGive(xMqttMutex);
  } else {
    Serial.println("❌ Falha ao obter mutex para publish");
    return false;
  }

  bool status = 0;
  String data;
  bool sent = false;
  char ATtopic[64];

  Serial.print("Publicando no tópico: ");
  Serial.println(topic);
  Serial.print("Payload: ");
  Serial.println(localPayload);

  // Configurar o tópico
  sprintf(ATtopic, "AT+CMQTTTOPIC=0,%d", strlen(topic)); 
  data = sendAT(ATtopic, 2000);
  Serial.println(data);
  if (data.indexOf(">") == -1) {
    Serial.println("❌ Erro ao configurar tópico - não recebeu '>'");
    return false;
  }

  simCOM.print(topic);
  simCOM.write(0X1A);
  data = readSIM(2000);
  Serial.println(data);
  if (data.indexOf("OK") == -1) {
    Serial.println("❌ Erro ao enviar tópico - não recebeu OK");
    return false;
  }

  // Configurar o payload
  sprintf(ATpayload, "AT+CMQTTPAYLOAD=0,%d", strlen(localPayload));
  data = sendAT(ATpayload, 2000);
  Serial.println(data);
  if (data.indexOf(">") == -1) {
    Serial.println("❌ Erro ao configurar payload - não recebeu '>'");
    return false;
  }
  
  simCOM.print(localPayload);
  simCOM.write(0X1A);
  data = readSIM(2000);
  Serial.println(data);
  if (data.indexOf("OK") == -1) {
    Serial.println("❌ Erro ao enviar payload - não recebeu OK");
    return false;
  }
 
  // Publicar
  data = sendAT("AT+CMQTTPUB=0,0,60", 7000);
  Serial.println(data);
  
  if (data.indexOf("+CMQTTPUB: 0,0") != -1)
  {
    Serial.println("✅ Mensagem publicada com sucesso!");
    clearPayload();
    return true;
  }
  else if (data.indexOf("+CMQTTPUB: 0,") != -1)
  {
    Serial.println("❌ Erro ao publicar mensagem");
    // Extrair código de erro
    int errorStart = data.indexOf("+CMQTTPUB: 0,");
    if (errorStart != -1) {
      String errorCode = data.substring(errorStart + 13, errorStart + 15);
      Serial.print("Código de erro: ");
      Serial.println(errorCode);
    }
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

bool waitForModuleReady(unsigned long timeout)
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

void clearRxBuffer() 
{
  while (simCOM.available()) {
    simCOM.read();
  }
}