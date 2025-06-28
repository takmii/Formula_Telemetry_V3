#include <Arduino.h>
#include <systemSetup.h>

bool time_get = 0;
String printValues();

SPIClass SD_SPI;
#define SD_CONFIG SdSpiConfig(SD_CS, DEDICATED_SPI, 4000000, &SD_SPI);
char sensorValues[BUFFER_NUMBER][BUFFER_LENGTH][MAX_SENSORS][BUFFER_SIZE];
uint32_t timeValues[BUFFER_NUMBER][BUFFER_LENGTH];

SemaphoreHandle_t sdMutex;

/*SdFat SD;
SdFile oFile;*/
File oFile;
String dir_ = "/";
__u8 n = 1;

String file_;
String file_test;

__u8 sensorLength;
TickType_t now;
volatile bool flag_sd = 0;
bool sd_started = 0;

bool string_flag;
String can_msg = "";

volatile bool buffer_write = 0;
volatile bool buffer_read;

uint8_t row_write = 0;
uint8_t row_read = 0;

void fn_Debug(__u8 data[DEBUG_DLC]);

void setup()
{
  memset(sensorValues, 0, sizeof(sensorValues));
  sdMutex = xSemaphoreCreateRecursiveMutex();
  timeBase espTime;
  string_flag = 0;
  Serial.begin(115200);
  while (!Serial)
  {
  };
  CAN.setPins(CAN_RX_PIN, CAN_TX_PIN);
  while (!CAN.begin(500E3))
  {
  };

  disableBluetooth();

  uint16_t init_wifi_time = millis();
  WiFi.begin(returnSSID(), returnPWD());
  if (WiFi.status() != WL_CONNECTED)
  {
    while (WiFi.status() != WL_CONNECTED && millis() - init_wifi_time < 2500)
    {
      delay(100);
    }
    Serial.println("WiFi connected.");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    time_get = 1;
    espTime = getTimeBase();
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
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }

  sensorLength = indexSetup();

  setSensorName();

  SD_SPI.begin(ESP32_SCK, ESP32_MISO, ESP32_MOSI, SD_CS);

  /*if (!SD.begin(SdSpiConfig(SD_CS, DEDICATED_SPI, 1000000, &SD_SPI)))
  {
    Serial.println("Falha ao inicializar o cartão SD!");
  }*/
  if (!SD.begin(SD_CS, SD_SPI))
  {
    Serial.println("Falha ao inicializar o cartão SD!");
  }
  else
  {
    sd_started = 1;
    Serial.println("Cartão SD inicializado com sucesso.");
  }
  if (sd_started)
  {
    if (time_get)
    {
      dir_ = dir_ + String(espTime.year);
      createDirectory(dir_.c_str());
      dir_ = dir_ + "/" + espTime.month + "_" + String(espTime.day);
      createDirectory(dir_.c_str());
    }
    else
    {
      dir_ = dir_ + "_No_Date";
      createDirectory(dir_.c_str());
    }
    file_ = dir_ + "/" + "test";
    file_ = verifyFilename(file_);

    /*if (!oFile.open(file_.c_str(), O_WRITE | O_CREAT | O_APPEND))
    {
      Serial.println("Falha ao abrir arquivo");
    }*/
    oFile = SD.open(file_.c_str(), FILE_WRITE);

    if (!oFile)
    {
      Serial.println("Falha ao abrir arquivo");
    }

    writeHeader(file_.c_str(), sensorLength);
  }

  xTaskCreatePinnedToCore(
      sdTask,    // Function to implement the task
      "SD Task", // Name of the task
      4096,      // Stack size in words
      NULL,      // Task input parameter
      1,         // Priority of the task
      NULL,      // Task handle
      1          // Core where the task should run (0 or 1)
  );

  xTaskCreatePinnedToCore(
      sdFlush,    // Function to implement the task
      "SD Flush", // Name of the task
      2048,       // Stack size in words
      NULL,       // Task input parameter
      1,          // Priority of the task
      NULL,       // Task handle
      0           // Core where the task should run (0 or 1)
  );

  xTaskCreatePinnedToCore(
      CAN_receiveTask, // Function
      "CAN RX Task",   // Name
      4096,            // Stack size
      NULL,            // Params
      3,               // Priority
      NULL,            // Task handle
      1                // Core (0 or 1)
  );
}

void loop()
{
}

void sensorUpdate(float value, uint8_t index)
{
  snprintf(sensorValues[buffer_write][row_write][index], 7, "%6.2f", value);
}

void sensorUpdate(uint8_t value, uint8_t index)
{
  snprintf(sensorValues[buffer_write][row_write][index], 7, "%d", value);
}

void sensorUpdate(uint16_t value, uint8_t index)
{
  snprintf(sensorValues[buffer_write][row_write][index], 7, "%d", value);
}

void sensorUpdate(uint32_t value, uint8_t index)
{
  snprintf(sensorValues[buffer_write][row_write][index], 7, "%d", value);
}

void sensorUpdate(uint64_t value, uint8_t index)
{
  snprintf(sensorValues[buffer_write][row_write][index], 7, "%llu", value);
}

void sensorUpdate(String value, uint8_t index)
{
  value.toCharArray(sensorValues[buffer_write][row_write][index], 7);
}

void writeHeader(const char *filename, __u8 size)
{

  if (oFile)
  {
    for (__u8 i = 0; i < size; i++)
    {
      oFile.print(sensorIndex[i]->name);
      oFile.print(";");
    }
    oFile.print("Tempo\n");
  }
  oFile.flush();
}

void createDirectory(const char *dir)
{
  if (dir != nullptr)
  {
    if (SD.exists(dir))
    {
      Serial.println("Directory already exists: " + String(dir));
    }
    else
    {
      if (SD.mkdir(dir))
      {
        Serial.println("Directory " + String(dir) + " created successfully");
      }
      else
      {
        Serial.println("Failed to create directory");
      }
    }
  }
  else
  {
    Serial.println("Null Dir");
  }
}

String verifyFilename(String filename)
{
  __u16 n = 0;
  String fn;
  do
  {
    n++;
    fn = filename;
    if (n < 10)
    {
      fn = fn + "0";
    }
    fn = fn + n + ".csv";
  } while (SD.exists((fn.c_str())));
  return fn;
}

void sdTask(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(SD_TASK_TIMER);
  for (;;)
  {
    if (sd_started)
    {
      if (flag_sd)
      {
        vTaskDelay(pdMS_TO_TICKS(1));
        buffer_read = !buffer_write;
        writeSDCard();
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void sdFlush(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(SD_FLUSH_TIMER);
  for (;;)
  {
    if (sd_started){
    oFile.flush();
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void CAN_receiveTask(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(CAN_TASK_TIMER);

  __u32 id;

  for (;;)
  {

    __u8 packetSize = CAN.parsePacket();
    if (packetSize)
    {
      id = CAN.packetId();
      Serial.println(id);
      static uint8_t data[8];

      if (data != nullptr)
      {
        __u8 i = 0;
        while (CAN.available() && i < packetSize)
        {
          data[i++] = (uint8_t)CAN.read();
        }

        CAN_setSensor(data, packetSize, id);
      }
    }
    timeValues[buffer_write][row_write] = (xTaskGetTickCount() * 1000) / configTICK_RATE_HZ;
    row_write++;
    if (row_write >= BUFFER_LENGTH)
    {
      buffer_write = !buffer_write;
      flag_sd = 1;
      row_write = 0;
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void disableBluetooth()
{
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

void CAN_setSensor(const __u8 *canData, __u8 canPacketSize, __u32 canId)
{
  __u8 pSize = canPacketSize;
  __u8 data[pSize];
  __u32 id = canId;
  memcpy(data, canData, pSize);

  /*Serial.print("0x");
  Serial.print(id,HEX);
  Serial.print(": ");
  for (__u8 i=0; i<pSize; i++){
    Serial.print(data[i]);
  }
  Serial.println("");*/

  switch (canId)
  {
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

  case DEBUG_ID:
    fn_Debug(data);
    break;

  default:
    Serial.print("ID: ");
    Serial.println(canId);
    Serial.println("Not Recognized");
    break;
  }
}

void fn_Messages(__u8 data[MESSAGES_DLC])
{
}

void fn_Data_01(__u8 data[DATA_01_DLC])
{
  __u16 r_Voltage = ((data[1] & 0x0F) << 8) + data[0];
  __u16 r_intTemp = (data[2] << 4) + ((data[1] >> 4) & 0x0F);
  __u16 r_Vref = ((data[4] & 0x0F) << 8) + data[3];
  __u8 r_Gear = (data[4] >> 4) & 0x0F;

  sensorUpdate(Gear_Pos(r_Gear), Gear_Pos_Sens.index);
  sensorUpdate(LinearSensor(r_Vref, 0.000805861, 0), V_Ref_Sensor.index);
  /*Serial.print(r_Gear);
  Serial.print(" ");*/
  // Serial.println((xTaskGetTickCount() * 1000) / configTICK_RATE_HZ);
}

void fn_Data_02(__u8 data[DATA_02_DLC])
{
  Serial.println(data[0]);
}

void fn_Data_03(__u8 data[DATA_03_DLC])
{
}

void fn_Data_04(__u8 data[DATA_04_DLC])
{
}

void fn_Data_05(__u8 data[DATA_05_DLC])
{
}

void fn_Data_06(__u8 data[DATA_06_DLC])
{
}

void fn_Data_07(__u8 data[DATA_07_DLC])
{
}

void fn_Data_08(__u8 data[DATA_08_DLC])
{
}

void fn_Data_09(__u8 data[DATA_09_DLC])
{
}

void fn_Debug(__u8 data[DEBUG_DLC])
{
  if (!string_flag)
  {
    for (__u8 i = 0; i < DEBUG_DLC; i++)
    {
      if (data[i] == 0)
      {
        string_flag = true;
        break;
      }
      else
      {
        can_msg += char(data[i]);
      }
    }
    if (string_flag)
    {
      // Serial.print("_: ");
      // Serial.println(can_msg);
      can_msg = "";
      string_flag = false;
    }
  }
}

void writeSDCard()
{
  if (oFile)
  {
    String linha;
    for (int l = 0; l < BUFFER_LENGTH; l++)
    {
      linha.clear();
      for (int s = 0; s < sensorLength; s++)
      {
        char *val = sensorValues[buffer_read][l][s];
        if (val[0] != '\0')
        {
          linha += val;
        }
        // oFile.print(4095);
        linha += ";"; // separador CSV
      }
      linha += timeValues[buffer_read][l];
      oFile.println(linha);
    }
    // oFile.flush(); // força gravação no cartão SD
  }
  flag_sd = false; // sinaliza que terminou a escrita
}