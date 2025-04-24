#include <Arduino.h>
#include <systemSetup.h>


SPIClass SD_SPI;
String* sensorValues;
String printValues;
File oFile;
String dir_ = "/";
__u8 n = 1;

String file_;
__u8 sensorLength;
TickType_t now;






void setup() {
  Serial.begin(115200);
  while (!Serial){};
  CAN.setPins(CAN_RX_PIN, CAN_TX_PIN);
  while (!CAN.begin(500E3)){};

  disableBluetooth();
  if (D_WIFI){
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
  else{
    WiFi.begin(returnSSID(), returnPWD());
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      Serial.println("");
    }
    Serial.println("WiFi connected.");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  }
  timeBase espTime = getTimeBase();

  Serial.print(espTime.day);
    Serial.print("/");
    Serial.print(espTime.month);
    Serial.print("/");
    Serial.print(espTime.year);
    Serial.print("  ");
    Serial.print(espTime.hour);
    Serial.print(":");
    Serial.print(espTime.minute);
    Serial.print(":");
    Serial.println(espTime.second);
  sensorLength = indexSetup();
  
  sensorValues=setSensorValues(sensorLength);
  setSensorName();
  
  SD_SPI.begin(ESP32_SCK, ESP32_MISO, ESP32_MOSI, SD_CS);
  if (!SD.begin(SD_CS,SD_SPI)) {
    Serial.println("Falha ao inicializar o cartão SD!");
  }
  else{
    Serial.println("Cartão SD inicializado com sucesso.");
  }
  dir_ = dir_ + String(espTime.year);
  createDirectory(dir_.c_str());
  dir_ = dir_ + "/" + month[espTime.month-1];
  createDirectory(dir_.c_str());
  dir_ = dir_ +"/"+ String(espTime.day) +"_" + String(espTime.month);
  createDirectory(dir_.c_str());
  file_ = dir_ + "/" + "test";

  file_= verifyFilename(file_);

  oFile = SD.open(file_, FILE_APPEND);

  writeHeader(file_.c_str(),sensorLength);

  xTaskCreatePinnedToCore(
    sdTask,          // Function to implement the task
    "SD Task",     // Name of the task
    4096,            // Stack size in words
    NULL,            // Task input parameter
    1,               // Priority of the task
    NULL,            // Task handle
    1                // Core where the task should run (0 or 1)
  );

  xTaskCreatePinnedToCore(
    sdFlush,          // Function to implement the task
    "SD Flush",     // Name of the task
    2048,            // Stack size in words
    NULL,            // Task input parameter
    2,               // Priority of the task
    NULL,            // Task handle
    1                // Core where the task should run (0 or 1)
  );


    xTaskCreatePinnedToCore(
    CAN_receiveTask,          // Function
    "CAN RX Task",           // Name
    4096,                     // Stack size
    NULL,                     // Params
    3,                        // Priority
    NULL,                     // Task handle
    1                        // Core (0 or 1)
  );





}

void loop() {

}

template <typename T>
void sensorUpdate(T value, uint8_t index) {
  sensorValues[index] = String(value);
}

String* setSensorValues(__u8 size){
  String* values = new String[size];
  for (__u8 i=0; i<size; i++){
    values[i]="";
  }
  return values;
}

void writeHeader(const char * filename, __u8 size){
  if (oFile){
    for (__u8 i=0; i<size; i++){
      oFile.print(sensorIndex[i] -> name);
      oFile.print(";");
    }
    oFile.print("Tempo\n");
  }
}


void createDirectory(const char * dir){
  if (SD.exists(dir)) {
    Serial.println("Directory already exists: " + String(dir));
  } else {
    if (SD.mkdir(dir)) {
      Serial.println("Directory " + String(dir) + " created successfully");
    } else {
      Serial.println("Failed to create directory");
    }
  }
}

String verifyFilename(String filename){
  __u16 n = 0;
  String fn;
  do{
    n++;
    fn = filename;
  if (n<10){
    fn = fn+"0";
  }
    fn = fn + n + ".csv";
  }while (SD.exists((fn.c_str())));
  return fn;
}






void sdTask(void *parameter){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(SD_TASK_TIMER);
  for (;;) {
    if (oFile){
      oFile.println(printValues);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void sdFlush(void *parameter){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(SD_FLUSH_TIMER);
  for (;;) {
    oFile.flush();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void CAN_receiveTask(void *parameter){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(CAN_TASK_TIMER);
  
  __u32 id;
  
  for (;;) {
    
    __u8 packetSize = CAN.parsePacket();
    if (packetSize) {

      id = CAN.packetId();
      uint8_t *data = (uint8_t*)malloc(packetSize * sizeof(uint8_t));

      if (data != nullptr) {
      __u8 i=0;
      while (CAN.available()) {
        data[i] = (uint8_t)CAN.read();
        i++;
      }

      CAN_setSensor(data,packetSize,id);
      free(data);
    }
  }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void disableBluetooth() {
  btStop();
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
}

timeBase getTimeBase()
{
    struct tm timeinfo;
    timeBase tb;

    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return tb; // Will return zeros if failed
    }

    tb.year = timeinfo.tm_year + 1900;
    tb.month = timeinfo.tm_mon + 1;
    tb.day = timeinfo.tm_mday;
    tb.hour = timeinfo.tm_hour;
    tb.minute = timeinfo.tm_min;
    tb.second = timeinfo.tm_sec;

    return tb;
}

void CAN_setSensor(const __u8 *canData, __u8 canPacketSize,__u32 canId){
  __u8 pSize = canPacketSize;
  __u8 data[pSize];
  __u32 id = canId;
  memcpy(data, canData, pSize);

  Serial.print("0x");
  Serial.print(id,HEX);
  Serial.print(": ");
  for (__u8 i=0; i<pSize; i++){
    Serial.print(data[i]);
  }
  Serial.println("");

  switch (canId) {
    case MESSAGES_ID:
      fn_Messages(data);
      break;
    case DATA_01_ID:
      fn_Data_01(data);
      break;
    case DATA_02_ID:
      fn_Data_02(data);
      break;
    case DATA_03_ID:
      fn_Data_03(data);
      break;
    case DATA_04_ID:
      fn_Data_04(data);
      break;
    case DATA_05_ID:
      fn_Data_05(data);
      break;
    case DATA_06_ID:
      fn_Data_06(data);
      break;
    case DATA_07_ID:
      fn_Data_07(data);
      break;
    case DATA_08_ID:
      fn_Data_08(data);
      break;
    case DATA_09_ID:
      fn_Data_09(data);
      break;






    default:
      Serial.println("Not Recognized");
      break;
  }
}


void fn_Messages(__u8 data[MESSAGES_DLC]){


}

void fn_Data_01(__u8 data[DATA_01_DLC]){
  __u16 r_Voltage = ((data[1]&0x0F)<<8)+data[0];
  __u16 r_intTemp = (data[2]<<4) + ((data[1]>>4)&0x0F);
  __u16 r_Vref = ((data[4]&0x0F)<<8)+data[3];
  __u8 r_Gear = (data[4]>>4)&0x0F;

  sensorUpdate(Gear_Pos(r_Gear),Gear_Pos_Sens.index);
  sensorUpdate(LinearSensor(r_Voltage,0.004942,-0.029654),Voltage_Sensor.index);
}

void fn_Data_02(__u8 data[DATA_02_DLC]){

}

void fn_Data_03(__u8 data[DATA_03_DLC]){

}

void fn_Data_04(__u8 data[DATA_04_DLC]){

}

void fn_Data_05(__u8 data[DATA_05_DLC]){

}

void fn_Data_06(__u8 data[DATA_06_DLC]){

}

void fn_Data_07(__u8 data[DATA_07_DLC]){

}

void fn_Data_08(__u8 data[DATA_08_DLC]){

}

void fn_Data_09(__u8 data[DATA_09_DLC]){

}















