#include <Arduino.h>
#include <systemSetup.h>

bool time_get = 0;
String printValues();

SPIClass SD_SPI;
#define SD_CONFIG SdSpiConfig(SD_CS, DEDICATED_SPI, 4000000, &SD_SPI);
char sensorValues[BUFFER_NUMBER][BUFFER_LENGTH][MAX_SENSORS][BUFFER_SIZE];
uint32_t timeValues[BUFFER_NUMBER][BUFFER_LENGTH];

SemaphoreHandle_t sdMutex;

volatile uint8_t n_rpm = 0;
volatile bool rpm_ready = 0;
volatile bool rpm_flag = 0;
volatile uint32_t rpm_time_first;
volatile uint32_t rpm_time_last;
volatile uint32_t rpm_time_zero;

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

uint32_t verify_Filesize_timer;

bool sd_open = 0;
bool sd_hold = 0;

bool string_flag;
String can_msg = "";

volatile bool buffer_write = 0;
volatile bool buffer_read;

uint8_t row_write = 0;
uint8_t row_read = 0;

bool rtc_exists = 0;
bool rtc_setup = 0;

const uint8_t MPU_addr = 0x68;
bool acc_start=0;

QueueHandle_t can_rx_queue;

void fn_Debug(__u8 data[DEBUG_DLC]);

RTC_DS3231 rtc;

void setup()
{
  memset(sensorValues, 0, sizeof(sensorValues));
  sdMutex = xSemaphoreCreateMutex();
  string_flag = 0;
  Wire.begin(25, 26);
  Wire.setClock(400000);
  I2C_MPU6050.begin(4,5);
  I2C_MPU6050.setClock(400000);

  Serial.begin(115200);
  while (!Serial)
  {
  }
  I2C_MPU6050.beginTransmission(MPU_addr);
  I2C_MPU6050.write(0x6B);  // PWR_MGMT_1
  I2C_MPU6050.write(0);     // Coloca 0 para acordar
  if(I2C_MPU6050.endTransmission(true)==0){
    acc_start=1;
    Serial.println("MPU6050 conectado");
  }
  if (!rtc.begin())
  {
    Serial.println("RTC DS3231 não encontrado.");
  }
  else
  {
    rtc_exists = 1;
  }
  /*CAN.setPins(CAN_RX_PIN, CAN_TX_PIN);
  while (!CAN.begin(500E3))
  {
  };*/

  init_twai();

  disableBluetooth();

  uint16_t init_wifi_time = millis();
  if(rtc_setup){
  //WiFi.begin(returnSSID(), returnPWD());
  if (WiFi.status() != WL_CONNECTED)
  {
    while (WiFi.status() != WL_CONNECTED && millis() - init_wifi_time < 5000)
    {
      delay(100);
    }
    if (WiFi.status() == WL_CONNECTED) {
  Serial.println("WiFi connected.");
  if (rtc_setup && rtc_exists)
  {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    time_get = 1;
    espTime = getTimeBase();
    setRTC();
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
}
  }
}

  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }

  if (!rtc_setup && rtc_exists)
  {
    getRTC();
    time_get = 1;
  }

  sensorLength = indexSetup();

  setSensorName();

  SD_SPI.begin(ESP32_SCK, ESP32_MISO, ESP32_MOSI, SD_CS);

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

    oFile = SD.open(file_.c_str(), FILE_APPEND);

    if (!oFile)
    {
      Serial.println("Falha ao abrir arquivo");
    }
    else
    {
      sd_open = 1;
    }

    writeHeader(file_.c_str(), sensorLength);
    verify_Filesize_timer=millis();
    
  }
  pinMode(GPIO_LEDRPM,OUTPUT);


  rpm_time_zero=millis();

  pinMode(RPM_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), handleEdge, FALLING);

  xTaskCreatePinnedToCore(
      sdTask,    // Function to implement the task
      "SD Task", // Name of the task
      8192,      // Stack size in words
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
      2,          // Priority of the task
      NULL,       // Task handle
      1           // Core where the task should run (0 or 1)
  );

   /* xTaskCreatePinnedToCore(
      sdFile,    // Function to implement the task
      "SD File", // Name of the task
      2048,       // Stack size in words
      NULL,       // Task input parameter
      1,          // Priority of the task
      NULL,       // Task handle
      1           // Core where the task should run (0 or 1)
  );*/


    xTaskCreatePinnedToCore(
      Calibracao,    // Function to implement the task
      "Calibracao", // Name of the task
      2048,       // Stack size in words
      NULL,       // Task input parameter
      1,          // Priority of the task
      NULL,       // Task handle
      1           // Core where the task should run (0 or 1)
  );

  xTaskCreatePinnedToCore(
      sdVerify,    // Function to implement the task
      "SD Verify", // Name of the task
      1024,        // Stack size in words
      NULL,        // Task input parameter
      2,           // Priority of the task
      NULL,        // Task handle
      1            // Core where the task should run (0 or 1)
  );

  xTaskCreatePinnedToCore(
      CAN_receiveTask, // Function
      "CAN RX Task",   // Name
      4096,            // Stack size
      NULL,            // Params
      3,               // Priority
      NULL,            // Task handle
      0                // Core (0 or 1)
  );

  xTaskCreatePinnedToCore(
      RPM_task,    // Function to implement the task
      "RPM Task", // Name of the task
      2048,       // Stack size in words
      NULL,       // Task input parameter
      5,          // Priority of the task
      NULL,       // Task handle
      0           // Core where the task should run (0 or 1)
  );

  xTaskCreatePinnedToCore(
      AccelGyro_task1,    // Function to implement the task
      "Accel Gyro Task", // Name of the task
      2048,       // Stack size in words
      NULL,       // Task input parameter
      2,          // Priority of the task
      NULL,       // Task handle
      0           // Core where the task should run (0 or 1)
  );
}

void loop()
{
}

void sensorUpdate(float value, uint8_t index)
{
  snprintf(sensorValues[buffer_write][row_write][index], 7, "%.2f", value);
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

void sensorUpdate(int8_t value, uint8_t index)
{
  snprintf(sensorValues[buffer_write][row_write][index], 7, "%d", value);
}

void sensorUpdate(int16_t value, uint8_t index)
{
  snprintf(sensorValues[buffer_write][row_write][index], 7, "%d", value);
}

void sensorUpdate(int32_t value, uint8_t index)
{
  snprintf(sensorValues[buffer_write][row_write][index], 7, "%ld", (long)value);
}

void sensorUpdate(int64_t value, uint8_t index)
{
  snprintf(sensorValues[buffer_write][row_write][index], 7, "%lld", (long long)value);
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
        if (xSemaphoreTake(sdMutex, portMAX_DELAY)) {
        writeSDCard();
        xSemaphoreGive(sdMutex);
        }
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
    if (sd_started)
    {
      if (xSemaphoreTake(sdMutex, portMAX_DELAY)) {
      oFile.flush();
      xSemaphoreGive(sdMutex);
        }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/*void sdFile(void *parameter){
  const uint64_t FILE_LIMIT = 3500000000ULL;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(SD_VERIFY_FILESIZE_TIMER);
  for (;;)
  {
    if (millis()-verify_Filesize_timer>SD_VERIFY_FILESIZE_TIMER){
      verify_Filesize_timer = millis();
      if ((uint64_t)oFile.size()>FILE_LIMIT){
        if (xSemaphoreTake(sdMutex, portMAX_DELAY)) {
        oFile.close();
        file_ = dir_ + "/" + "test";
        file_ = verifyFilename(file_);
        oFile = SD.open(file_.c_str(), FILE_APPEND);
        writeHeader(file_.c_str(), sensorLength);
        xSemaphoreGive(sdMutex);
        }
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}*/

void sdVerify(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(SD_VERIFY_TIMER);
  for (;;)
  {
    if (sd_started){
    if (!sd_hold)
    {
      if (xSemaphoreTake(sdMutex, portMAX_DELAY)) {
      if (!oFile)
      {
        oFile = SD.open(file_.c_str(), FILE_APPEND);
      }
      xSemaphoreGive(sdMutex);
        }
    }
  }
  vTaskDelayUntil(&xLastWakeTime, xFrequency);
}
}

void CAN_receiveTask(void *parameter)
{
  twai_message_t message;
  uint32_t alerts;

  for (;;)
  {
    while (twai_receive(&message, pdMS_TO_TICKS(1)) == ESP_OK)
    {
      CAN_setSensor(message.data, message.data_length_code, message.identifier);
    }
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

void setRTC()
{
  rtc.adjust(DateTime(
      espTime.year,
      espTime.month,
      espTime.day,
      espTime.hour,
      espTime.minute,
      espTime.second));
}

void getRTC()
{
  DateTime now = rtc.now();
  struct tm t;
  t.tm_year = now.year() - 1900;
  t.tm_mon = now.month() - 1;
  t.tm_mday = now.day();
  t.tm_hour = now.hour();
  t.tm_min = now.minute();
  t.tm_sec = now.second();
  t.tm_isdst = 0;

  time_t timeSinceEpoch = mktime(&t);

  struct timeval tv = {.tv_sec = timeSinceEpoch, .tv_usec = 0};
  settimeofday(&tv, nullptr);

  espTime.year = now.year();
  espTime.month = now.month();
  espTime.day = now.day();
  espTime.hour = now.hour();
  espTime.minute = now.minute();
  espTime.second = now.second();
}

void sendCANMessage(uint8_t id, uint8_t *data, uint8_t dlc){
  twai_message_t message;
  message.identifier = id;
  message.flags = 0;
  message.data_length_code = dlc;
    for (int i = 0; i < dlc; i++) {
        message.data[i] = data[i];
    }
    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(10));
}



void CAN_setSensor(const __u8 *canData, __u8 canPacketSize, __u32 canId)
{
  __u8 pSize = canPacketSize;
  __u8 data[pSize];
  __u32 id = canId;
  memcpy(data, canData, pSize);

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

  case BUFFER_ACK_ID:
    fn_Buffer_Ack(data);
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
  __u16 r_vBat = ((data[1] & 0x0F) << 8) + data[0];
  __u16 r_intTemp = (data[2] << 4) + ((data[1] >> 4) & 0x0F);
  __u16 r_vRef = ((data[4] & 0x0F) << 8) + data[3];
  __u8 r_Gear = (data[4] >> 4) & 0x0F;

  float vBat = vBatSensor(r_vBat);
  float vRef = vRefSensor(r_vRef);
  String Gear = Gear_Pos(r_Gear);

  sensorUpdate(vBat, Voltage_Sensor.index);
  sensorUpdate(vRef, V_Ref_Sensor.index);
  sensorUpdate(Gear, Gear_Pos_Sens.index);

  /*Serial.print(r_Gear);
  Serial.print(" ");*/
  // Serial.println((xTaskGetTickCount() * 1000) / configTICK_RATE_HZ);
}

void fn_Data_02(__u8 data[DATA_02_DLC])
{
  __u16 r_Susp_FR = ((data[1] & 0x0F) << 8) + data[0];
  __u16 r_Susp_FL = (data[2] << 4) + ((data[1] >> 4) & 0x0F);
  __u16 r_Susp_RR = ((data[4] & 0x0F) << 8) + data[3];
  __u16 r_Susp_RL = (data[5] << 4) + ((data[4] >> 4) & 0x0F);
  __u16 r_WheelAngle = ((data[7] & 0x0F) << 8) + data[6];

  float Susp_FR = suspSensor(r_Susp_FR);
  float Susp_FL = suspSensor(r_Susp_FL);
  float Susp_RR = suspSensor(r_Susp_RR);
  float Susp_RL = suspSensor(r_Susp_RL);
  float WheelAngle = (r_WheelAngle);

  sensorUpdate(Susp_FR, Susp_Pos_FR_Sensor.index);
  sensorUpdate(Susp_FL, Susp_Pos_FL_Sensor.index);
  sensorUpdate(Susp_RR, Susp_Pos_RR_Sensor.index);
  sensorUpdate(Susp_RL, Susp_Pos_RL_Sensor.index);
  sensorUpdate(WheelAngle, SteerWheel_Pos_Sensor.index);
}

void fn_Data_03(__u8 data[DATA_03_DLC])
{

  __u16 r_Hall_FR = ((data[1] & 0x0F) << 8) + data[0];
  __u16 r_Hall_FL = (data[2] << 4) + ((data[1] >> 4) & 0x0F);
  __u16 r_Hall_RR = ((data[4] & 0x0F) << 8) + data[3];
  __u16 r_Hall_RL = (data[5] << 4) + ((data[4] >> 4) & 0x0F);

  float Hall_FR = (r_Hall_FR);
  float Hall_FL = (r_Hall_FL);
  float Hall_RR = (r_Hall_RR);
  float Hall_RL = (r_Hall_RL);

  sensorUpdate(Hall_FR, Wheel_Spd_FR_Sensor.index);
  sensorUpdate(Hall_FL, Wheel_Spd_FL_Sensor.index);
  sensorUpdate(Hall_RR, Wheel_Spd_RR_Sensor.index);
  sensorUpdate(Hall_RL, Wheel_Spd_RL_Sensor.index);
}

void fn_Data_04(__u8 data[DATA_04_DLC])
{
}

void fn_Data_05(__u8 data[DATA_05_DLC])
{
}

void fn_Data_06(__u8 data[DATA_06_DLC])
{
  __u16 r_F_BrakelinePress = ((data[4] & 0x0F) << 8) + data[3];
  __u16 r_R_BrakelinePress = (data[5] << 4) + ((data[4] >> 4) & 0x0F);
  float F_BrakelinePress = (r_F_BrakelinePress);
  float R_BrakelinePress = (r_R_BrakelinePress);
  sensorUpdate(F_BrakelinePress, F_Brakeline_Pressure.index);
  sensorUpdate(R_BrakelinePress, R_Brakeline_Pressure.index);
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

void fn_Buffer_Ack(__u8 data[BUFFER_ACK_DLC])
{
  if (data[0]=='1'){
    timeValues[buffer_write][row_write] = (xTaskGetTickCount() * 1000) / configTICK_RATE_HZ;
    row_write++;
    if (row_write >= BUFFER_LENGTH)
    {
      buffer_write = !buffer_write;
      flag_sd = 1;
      row_write = 0;
    }
  }

}

void fn_Debug(__u8 data[DEBUG_DLC])
{
  if (!string_flag)
  {
    for (__u8 i = 0; i < DEBUG_DLC; i++)
    {
      if (data[i] == '$')
      {
        Serial.println(can_msg);
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
    sd_hold = 1;
    for (int l = 0; l < BUFFER_LENGTH; l++)
    {
      linha.clear();
      for (int s = 0; s < sensorLength; s++)
      {
        char *val = sensorValues[buffer_read][l][s];
        if (val[0] != '\0')
        {
          linha += val;
          val[0] = '\0';
        }
        // oFile.print(4095);
        linha += ";"; // separador CSV
      }
      linha += timeValues[buffer_read][l];
      linha += ";";
      linha += String(buffer_read);
      oFile.println(linha);
    }
    // oFile.flush(); // força gravação no cartão SD
    sd_hold = 0;
  }
  flag_sd = false; // sinaliza que terminou a escrita
}

void init_twai()
{
  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = {
      .mode = TWAI_MODE_NORMAL,
      .tx_io = CAN_TX_PIN,
      .rx_io = CAN_RX_PIN,
      .clkout_io = TWAI_IO_UNUSED,
      .bus_off_io = TWAI_IO_UNUSED,
      .tx_queue_len = 5,
      .rx_queue_len = 5,
      .alerts_enabled = TWAI_ALERT_NONE,
      .clkout_divider = 0};

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    printf("Driver installed\n");
  }
  else
  {
    printf("Failed to install driver\n");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK)
  {
    printf("Driver started\n");
  }
  else
  {
    printf("Failed to start driver\n");
    return;
  }
}

void Calibracao(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(CALIBRACAO_TIMER);
  for (;;)
  {
    //Serial.println(xPortGetFreeHeapSize());

    uint8_t index = RPM_Sensor.index;
    bool print =0;
    uint8_t time=0;
    if (row_write==0){
      while(row_write==0&&time<5){
        vTaskDelay(pdMS_TO_TICKS(1));
        time++;
      }
    }
    if (row_write!=0){
      print =1;
    }
    if (print){
      char *value = sensorValues[buffer_write][row_write-1][index];
      if (value[0]!='\0'){
      Serial.println(value);
      }
    }
    vTaskDelay(xFrequency);
  }
}

void IRAM_ATTR handleEdge(){
  uint32_t now = micros();
  Serial.println(""); 
  if (n==0){
    rpm_time_first= now;
    rpm_ready=1;
  }
  if (rpm_ready&&n<10){
   
  n++;
  }
  if(n==10){
  rpm_time_last= now;
  rpm_flag = 1;
  }
}

void RPM_task(void *parameter){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(RPM_TIMER);
  uint32_t time_rpm;
  float r_RPM;
  uint16_t RPM;

  for (;;)
  {
    time_rpm=millis();
    bool update = 0;
    bool flag;
    noInterrupts();
    flag = rpm_flag;
    interrupts();
    
    if (flag){
    
    r_RPM = 60000000.0/(rpm_time_last-rpm_time_first);
    RPM = (uint16_t)r_RPM;
    n = 0;
    rpm_flag = 0;
    update=1;
    rpm_ready=0;
    }
    else if(time_rpm-rpm_time_zero>RPM_ZERO_TIMER){
      RPM=0;
      n=0;
      rpm_ready=0;
      rpm_time_zero = time_rpm;
      update=1;
      rpm_ready=0;
    }
    if (update){
      sensorUpdate(RPM, RPM_Sensor.index);
      uint8_t RPM_data[2]={0,0};
      RPM_data[0] = RPM & 0xFF;
      RPM_data[1] = (RPM >> 8) & 0x3F;
      sendCANMessage(RPM_ID, RPM_data, RPM_DLC);
    }
    if(RPM>10000){
      digitalWrite(GPIO_LEDRPM, HIGH);
    }
    else{
      digitalWrite(GPIO_LEDRPM, LOW);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
}

}

void AccelGyro_task1(void *parameter){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(ACC_TIMER);

  int16_t AcX, AcY, AcZ;
  int16_t GyX, GyY, GyZ;
  uint8_t AccData[8];
  uint8_t GyroData[8];
  for (;;)
  {
    if (acc_start){
      I2C_MPU6050.beginTransmission(MPU_addr);
      I2C_MPU6050.write(0x3B); // Endereço do primeiro registrador de dados
    I2C_MPU6050.endTransmission(false);
    I2C_MPU6050.requestFrom(MPU_addr, 14, true);

    AcX = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
    sensorUpdate(AcX,Accel_X.index);
    AcY = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
    sensorUpdate(AcY,Accel_Y.index);
    AcZ = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
    sensorUpdate(AcZ,Accel_Z.index);
    I2C_MPU6050.read(); I2C_MPU6050.read(); // Temperatura (ignorada)
    GyX = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
    sensorUpdate(GyX,Gyro_X.index);
    GyY = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
    sensorUpdate(GyY,Gyro_Y.index);
    GyZ = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
    sensorUpdate(GyZ,Gyro_Z.index);

      AccData[0] = AcX & 0xFF;
      AccData[1] = (AcX >> 8) & 0xFF;
      AccData[2] = AcY & 0xFF;
      AccData[3] = (AcY >> 8) & 0xFF;
      AccData[4] = AcZ & 0xFF;
      AccData[5] = (AcZ >> 8) & 0xFF;
      AccData[6] = 0;
      AccData[7] = 0;

      sendCANMessage(ACC_ID, AccData, ACC_DLC);

      GyroData[0] = GyX & 0xFF;
      GyroData[1] = (GyX >> 8) & 0xFF;
      GyroData[2] = GyY & 0xFF;
      GyroData[3] = (GyY >> 8) & 0xFF;
      GyroData[4] = GyZ & 0xFF;
      GyroData[5] = (GyZ >> 8) & 0xFF;
      GyroData[6] = 0;
      GyroData[7] = 0;

      sendCANMessage(GYRO_ID, GyroData, GYRO_DLC);
    }
    vTaskDelay(xFrequency);
  }
}