#include <Arduino.h>
#include <systemSetup.h>

bool time_get = 0;
String printValues();

SPIClass SD_SPI;
#define SD_CONFIG SdSpiConfig(SD_CS, DEDICATED_SPI, 4000000, &SD_SPI);
char sensorValues[BUFFER_NUMBER][BUFFER_LENGTH][MAX_SENSORS][BUFFER_SIZE];
uint32_t timeValues[BUFFER_NUMBER][BUFFER_LENGTH];


SemaphoreHandle_t sdMutex;
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t xMqttMutex;

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


uint8_t sd_status=0;
uint8_t accgyro_status=0;

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

bool time_set_ack = 0;

bool acc_start=0;

QueueHandle_t can_rx_queue;

void fn_Debug(__u8 data[DEBUG_DLC]);

RTC_DS3231 rtc;

void setup()
{
  memset(sensorValues, 0, sizeof(sensorValues));
  sdMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();
  xMqttMutex = xSemaphoreCreateMutex();
  string_flag = 0;
  Wire.begin(25, 26);
  Wire.setClock(400000);
  I2C_MPU6050.begin(4,5);
  I2C_MPU6050.setClock(100000);
  I2C_MPU6050.setTimeout(1000);

  Serial.begin(921600);
  while (!Serial)
  {
  }
  /*simCOM.setRxBufferSize(2048);
  simCOM.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  while(!simCOM)
  {
  }*/

  I2C_MPU6050.beginTransmission(0x68);
  if(I2C_MPU6050.endTransmission()==0){
    acc_start=1;
    Serial.println("MPU6050 conectado");

    I2C_MPU6050.beginTransmission(MPU_addr);
    I2C_MPU6050.write(0x6B); // PWR_MGMT_1
    I2C_MPU6050.write(0x01); 
    I2C_MPU6050.endTransmission();

    I2C_MPU6050.beginTransmission(MPU_addr);
    I2C_MPU6050.write(0x1A);
    I2C_MPU6050.write(0x05);
    I2C_MPU6050.endTransmission();

    I2C_MPU6050.beginTransmission(MPU_addr);
    I2C_MPU6050.write(0x1B);
    I2C_MPU6050.write(0x08);
    I2C_MPU6050.endTransmission();

    I2C_MPU6050.beginTransmission(MPU_addr);
    I2C_MPU6050.write(0x1C);
    I2C_MPU6050.write(0x10);
    I2C_MPU6050.endTransmission();
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
    sd_status = 1;
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

  //pinMode(RPM_PIN,INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(RPM_PIN), handleEdge, FALLING);

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
      1024,       // Stack size in words
      NULL,       // Task input parameter
      1,          // Priority of the task
      NULL,       // Task handle
      1           // Core where the task should run (0 or 1)
  );

      xTaskCreatePinnedToCore(
      MessagesFN,    // Function to implement the task
      "Message FN", // Name of the task
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

  /*xTaskCreatePinnedToCore(
      RPM_task,    // Function to implement the task
      "RPM Task", // Name of the task
      2048,       // Stack size in words
      NULL,       // Task input parameter
      5,          // Priority of the task
      NULL,       // Task handle
      0           // Core where the task should run (0 or 1)
  );*/

  xTaskCreatePinnedToCore(
      AccelGyro_task1,    // Function to implement the task
      "Accel Gyro Task", // Name of the task
      2048,       // Stack size in words
      NULL,       // Task input parameter
      2,          // Priority of the task
      NULL,       // Task handle
      0           // Core where the task should run (0 or 1)
  );

   xTaskCreatePinnedToCore(
      TempTask,    // Function to implement the task
      "Temp Task", // Name of the task
      2048,       // Stack size in words
      NULL,       // Task input parameter
      2,          // Priority of the task
      NULL,       // Task handle
      0           // Core where the task should run (0 or 1)
  );

  /*xTaskCreatePinnedToCore(
      SIM_Task,    // Function to implement the task
      "SIM Task", // Name of the task
      8192,       // Stack size in words
      NULL,       // Task input parameter
      1,          // Priority of the task
      NULL,       // Task handle
      1           // Core where the task should run (0 or 1)
  );*/

  /*xTaskCreatePinnedToCore(
      MQTT_Time_Task,    // Function to implement the task
      "MQTT Time Task", // Name of the task
      1024,       // Stack size in words
      NULL,       // Task input parameter
      1,          // Priority of the task
      NULL,       // Task handle
      1           // Core where the task should run (0 or 1)
  );*/
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
        sd_status = 1;
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
  case DATA_10_ID:
    fn_Data_10(data);
    break;

  case BUFFER_ACK_ID:
    fn_Buffer_Ack(data);
    break;

  case TIMESET_ACK_ID:
    if (data[0]==1){
      time_set_ack = 1;
    }
    break;

  case DEBUG_ID:
    fn_Debug(data);
    break;

  case TEMP_ID:
    fn_Temp(data);
    break;

  case BASE_ID + GROUP0_ID:
    fn_Group_0(data);
    break;

    case BASE_ID + GROUP1_ID:
    fn_Group_1(data);
    break;

    case BASE_ID + GROUP2_ID:
    fn_Group_2(data);
    break;

    case BASE_ID + GROUP3_ID:
    fn_Group_3(data);
    break;

        case BASE_ID + GROUP7_ID:
    fn_Group_7(data);
    break;

        case BASE_ID + GROUP8_ID:
    fn_Group_8(data);
    break;

        case BASE_ID + GROUP9_ID:
    fn_Group_9(data);
    break;

        case BASE_ID + GROUP15_ID:
    fn_Group_15(data);
    break;

  default:
    //Serial.print("0x");
    //Serial.println(canId,HEX);
    //Serial.println(" NR");
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
  float intTemp = internalTemp(r_intTemp);
  String Gear = Gear_Pos(r_Gear);

  sensorUpdate(vBat, Voltage_Sensor.index);
  sensorUpdate(intTemp, Internal_Temperature_Sensor.index);
  sensorUpdate(vRef, V_Ref_Sensor.index);
  sensorUpdate(Gear, Gear_Pos_Sens.index);
}

void fn_Data_02(__u8 data[DATA_02_DLC])
{

    const static float Susp_FR_Center = 0;
  const static float Susp_FL_Center = 74.9;
  const static float Susp_RR_Center = 82.1;
  const static float Susp_RL_Center = 23.1;

  __u16 r_Susp_FR = ((data[1] & 0x0F) << 8) + data[0];
  __u16 r_Susp_FL = (data[2] << 4) + ((data[1] >> 4) & 0x0F);
  __u16 r_Susp_RR = ((data[4] & 0x0F) << 8) + data[3];
  __u16 r_Susp_RL = (data[5] << 4) + ((data[4] >> 4) & 0x0F);
  __u16 r_WheelAngle = ((data[7] & 0x0F) << 8) + data[6];

  float Susp_FR = suspSensor(r_Susp_FR,0,Susp_FR_Center);
  float Susp_FL = suspSensor(r_Susp_FL,1,Susp_FL_Center);
  float Susp_RR = suspSensor(r_Susp_RR,0,Susp_RR_Center);
  float Susp_RL = suspSensor(r_Susp_RL,1,Susp_RL_Center);
  String WheelAngle = SteeringWheel.steeringWheelValue(r_WheelAngle);

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
  //__u16 r_Oil_Pressure = ((data[1] & 0x0F) << 8) + data[0];
  //__u16 r_Oil_Temp = (data[2] << 4) + ((data[1] >> 4) & 0x0F);

  //float Oil_Pressure = (r_Oil_Pressure);
  //float Oil_Temp = (r_Oil_Temp);
  
  //sensorUpdate(Oil_Pressure, Oil_Pressure_Sensor.index);
  //sensorUpdate(Oil_Temp, Oil_Temperature_Sensor.index);
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

void fn_Data_10(__u8 data[DATA_10_DLC])
{
  static const double a = 0.00130654;
  static const double b = 0.000259714;
  static const double c = -1.08959E-08;

  __u16 r_MAP1=((data[1] & 0x0F) << 8) + data[0];
  __u16 r_MAP2=(data[2] << 4) + ((data[1] >> 4) & 0x0F);
  __u16 r_MAF=((data[4] & 0x0F) << 8) + data[3];
  __u16 r_OilTemp=(data[5] << 4) + ((data[4] >> 4) & 0x0F);

  float MAP1 = mapSensor(r_MAP1);
  float MAP2 = mapSensor(r_MAP2);
  float MAF = mafSensor(r_MAF);
  float OilTemp = tempOilSensor(r_OilTemp,a,b,c);

  sensorUpdate(MAP1,MAP1_Sensor.index);
  sensorUpdate(MAP2,MAP2_Sensor.index);
  sensorUpdate(MAF,MAF_Sensor.index);
  sensorUpdate(OilTemp,Oil_Temperature_Sensor.index);
}

void fn_Temp(__u8 data[TEMP_DLC]){
  __u16 r_DT_FR = data[0] | ((data[1] & 0x07) << 8);
  __u16 r_DT_FL = ((data[1] >> 3) & 0x1F) | ((data[2] & 0x3F) << 5);
  __u16 r_DT_RR = ((data[2] >> 6) & 0x03) | (data[3] << 2) | ((data[4] & 0x01) << 10);
  __u16 r_DT_RL = ((data[4] >> 1) & 0x7F) | ((data[5] & 0x0F) << 7);
  __u16 r_DT_F1 = ((data[5] >> 4) & 0x0F) | ((data[6] & 0x1F) << 4);
  __u16 r_DT_F2 = ((data[6] >> 5) & 0x07) | (data[7] << 3);


  float DT_FR = U16toFloat(r_DT_FR,2);
  float DT_FL = U16toFloat(r_DT_FL,2);
  float DT_RR = U16toFloat(r_DT_RR,2);
  float DT_RL = U16toFloat(r_DT_RL,2);
  float DT_F1 = U16toFloat(r_DT_F1,2);
  float DT_F2 = U16toFloat(r_DT_F2,2);

  sensorUpdate(DT_FR, Disk_Temp_FR_Sensor.index);
  sensorUpdate(DT_FL, Disk_Temp_FL_Sensor.index);
  sensorUpdate(DT_RR, Disk_Temp_RR_Sensor.index);
  sensorUpdate(DT_RL, Disk_Temp_RL_Sensor.index);
  sensorUpdate(DT_F1, Firewall_Temperature1_Sensor.index);
  sensorUpdate(DT_F2, Firewall_Temperature2_Sensor.index);
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
    sd_status = 2;
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

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
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
  static uint8_t TimeSet_Data[TIMESET_DLC];
  for (;;)
  {
    //Serial.println(xPortGetFreeHeapSize());

    if (!time_set_ack){
      TimeSet_Data[0] = espTime.day&0x1F;
      TimeSet_Data[0]|= ((espTime.month&0x0F)<<5);
      TimeSet_Data[1] = (espTime.month>>3)&0x01;
      TimeSet_Data[1]|= (espTime.year&0X7F)<<1;
      TimeSet_Data[2] = (espTime.year>>7)&0x1F;
      TimeSet_Data[2] |= (espTime.hour&0x07)<<5;
      TimeSet_Data[3] = (espTime.hour>>3)&0x03;
      TimeSet_Data[3]|= (espTime.minute&0x3F)<<2;
      TimeSet_Data[4] = espTime.second&0x3F;

      sendCANMessage(TIMESET_ID, TimeSet_Data, TIMESET_DLC);
    }
    vTaskDelay(xFrequency);
  }
}

/*void IRAM_ATTR handleEdge(){
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
}*/

/*void RPM_task(void *parameter){
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

}*/

/*void AccelGyro_task1(void *parameter){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(ACC_TIMER);

  int16_t AcX, AcY, AcZ;
  int16_t AcXf, AcYf, AcZf;
  AcXf=-819;
  AcYf=164;
  AcZf=819;
  int16_t AcMod;
  int16_t GyX, GyY, GyZ;
  uint8_t AccData[8];
  uint8_t GyroData[8];
  int16_t roll_rate = -92;
  int16_t pitch_rate = 121;
  int16_t yaw_rate = -121;
  float RateRoll,RatePitch,RateYaw;
  float aX,aY,aZ;
  float aMod;

  for (;;)
  {
    if (acc_start){
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
    uint8_t pwr_mgmt_1 = readRegister(0x6B);
    if (pwr_mgmt_1 & 0x40) { // bit SLEEP ativo
      I2C_MPU6050.beginTransmission(MPU_addr);
      I2C_MPU6050.write(0x6B);
      I2C_MPU6050.write(0x01); // reativa e define GyroX como clock
      I2C_MPU6050.endTransmission();
      Serial.println("MPU6050 reativado do modo sleep");
    }
    
    I2C_MPU6050.beginTransmission(MPU_addr);
    I2C_MPU6050.write(0x3B); // Endereço do primeiro registrador de dados
    if (!I2C_MPU6050.endTransmission(false)){
      accgyro_status = 2;
    }
    else{
      accgyro_status = 1;
    }
    I2C_MPU6050.requestFrom(MPU_addr, 14, true);

    AcX = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
    AcY = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
    AcZ = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
    
    I2C_MPU6050.read(); I2C_MPU6050.read(); // Temperatura (ignorada)
    GyX = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
    GyY = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
    GyZ = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();

    xSemaphoreGive(i2cMutex);
    }
    AcX+=AcXf;
    AcY+=AcYf;
    AcZ+=AcZf;

    GyX-=roll_rate;
    GyY-=pitch_rate;
    GyZ-=yaw_rate;

    RateRoll=(float)GyX/65.5;
    RatePitch=(float)GyY/65.5;
    RateYaw=(float)GyZ/65.5;


    aX = (float)AcX/16384;
    aY = (float)AcY/16384;
    aZ = (float)AcZ/16384;

    AcMod = (int16_t)sqrt((double)((int32_t)AcX * AcX + (int32_t)AcY * AcY + (int32_t)AcZ * AcZ));
    aMod = (float)AcMod/16384;

    Serial.print("AcX: "); Serial.print(aX); Serial.print(" ");
    Serial.print(" AcY: "); Serial.print(aY); Serial.print(" ");
    Serial.print(" AcZ: "); Serial.print(aZ); Serial.print(" ");
    Serial.print(" AcMod: "); Serial.print(aMod); Serial.print(" ");
    Serial.print(" GyX: "); Serial.print(RateRoll); Serial.print(" ");
    Serial.print(" GyY: "); Serial.print(RatePitch); Serial.print(" ");
    Serial.print(" GyZ: "); Serial.println(RateYaw);


    sensorUpdate(aX,Accel_X.index);
    sensorUpdate(aY,Accel_Y.index);
    sensorUpdate(aZ,Accel_Z.index);
    sensorUpdate(aMod,Accel.index);
    sensorUpdate(RateRoll,Gyro_X.index);
    sensorUpdate(RatePitch,Gyro_Y.index);
    sensorUpdate(RateYaw,Gyro_Z.index);

      AccData[0] = AcX & 0xFF;
      AccData[1] = (AcX >> 8) & 0xFF;
      AccData[2] = AcY & 0xFF;
      AccData[3] = (AcY >> 8) & 0xFF;
      AccData[4] = AcZ & 0xFF;
      AccData[5] = (AcZ >> 8) & 0xFF;
      AccData[6] = AcMod & 0xFF;
      AccData[7] = (AcMod >> 8) & 0xFF;

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
}*/

void AccelGyro_task1(void *parameter){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(ACC_TIMER);

  int16_t AcX, AcY, AcZ;
  //int16_t AcXf = -819;
  //int16_t AcYf = 164;
  //int16_t AcZf = 819;
  int16_t AcXf = -175;
  int16_t AcYf = 25;
  int16_t AcZf = 225;
  int16_t AcMod;
  int16_t GyX, GyY, GyZ;
  uint8_t AccData[8];
  uint8_t GyroData[8];
  int16_t roll_rate = -92;
  int16_t pitch_rate = 121;
  int16_t yaw_rate = -121;
  float RateRoll, RatePitch, RateYaw;
  float aX, aY, aZ;
  float aMod;
  
  uint8_t errorCount = 0;
  uint8_t sleepCheckCounter = 0; // Só verifica sleep a cada N ciclos

  for (;;)
  {
    if (acc_start) {
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        
        bool dataValid = false;
        
        // ⚠️ VERIFICA SLEEP APENAS A CADA 20 LEITURAS (evita sobrecarga I2C)
        if (sleepCheckCounter >= 20) {
          sleepCheckCounter = 0;
          
          I2C_MPU6050.beginTransmission(MPU_addr);
          I2C_MPU6050.write(0x6B);
          uint8_t error = I2C_MPU6050.endTransmission(false);
          
          if (error == 0) {
            uint8_t bytesReceived = I2C_MPU6050.requestFrom(MPU_addr, (uint8_t)1, (uint8_t)true);
            if (bytesReceived == 1) {
              uint8_t pwr_mgmt_1 = I2C_MPU6050.read();
              
              if (pwr_mgmt_1 & 0x40) {
                //Serial.println("MPU6050 em SLEEP! Reativando...");
                I2C_MPU6050.beginTransmission(MPU_addr);
                I2C_MPU6050.write(0x6B);
                I2C_MPU6050.write(0x01);
                I2C_MPU6050.endTransmission();
                delay(50); // Aguarda acordar
              }
            }
          }
        }
        sleepCheckCounter++;
        
        // ⚠️ LÊ DADOS DO ACELERÔMETRO E GIROSCÓPIO
        I2C_MPU6050.beginTransmission(MPU_addr);
        I2C_MPU6050.write(0x3B);
        uint8_t error = I2C_MPU6050.endTransmission(false);
        
        if (error == 0) {
          uint8_t bytesReceived = I2C_MPU6050.requestFrom(MPU_addr, (uint8_t)14, (uint8_t)true);
          
          if (bytesReceived == 14) {
            AcX = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
            AcY = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
            AcZ = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
            
            I2C_MPU6050.read(); I2C_MPU6050.read(); // Temperatura
            
            GyX = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
            GyY = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
            GyZ = I2C_MPU6050.read() << 8 | I2C_MPU6050.read();
            
            dataValid = true;
            accgyro_status = 2;
            errorCount = 0;
          } else {
            accgyro_status = 1;
            errorCount++;
            //Serial.printf("Erro: esperava 14 bytes, recebeu %d\n", bytesReceived);
            
            // Limpa buffer se houver dados pendentes
            while (I2C_MPU6050.available()) {
              I2C_MPU6050.read();
            }
          }
        } else {
          accgyro_status = 1;
          errorCount++;
          //Serial.printf(" Erro I2C: %d\n", error);
        }
        
        xSemaphoreGive(i2cMutex);
        
        // ⚠️ REINICIA MPU SE MUITOS ERROS
        if (errorCount > 10) {
          //Serial.println(" Muitos erros! Reiniciando MPU6050...");
          if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            // Reset completo
            I2C_MPU6050.beginTransmission(MPU_addr);
            I2C_MPU6050.write(0x6B);
            I2C_MPU6050.write(0x80); // Reset bit
            I2C_MPU6050.endTransmission();
            vTaskDelay(pdMS_TO_TICKS(100));
            
            // Reconfigura
            I2C_MPU6050.beginTransmission(MPU_addr);
            I2C_MPU6050.write(0x6B);
            I2C_MPU6050.write(0x01);
            I2C_MPU6050.endTransmission();
            
            xSemaphoreGive(i2cMutex);
          }
          errorCount = 0;
          vTaskDelay(pdMS_TO_TICKS(200));
          continue;
        }
        
        //  SÓ PROCESSA SE DADOS VÁLIDOS
        if (dataValid) {
          AcX += AcXf;
          AcY += AcYf;
          AcZ += AcZf;

          GyX -= roll_rate;
          GyY -= pitch_rate;
          GyZ -= yaw_rate;

          RateRoll = (float)GyX / 65.5;
          RatePitch = (float)GyY / 65.5;
          RateYaw = (float)GyZ / 65.5;

          aX = (float)AcX / 4096;
          aY = (float)AcY / 4096;
          aZ = (float)AcZ / 4096;

          AcMod = (int16_t)sqrt((double)((int32_t)AcX * AcX + (int32_t)AcY * AcY + (int32_t)AcZ * AcZ));
          aMod = (float)AcMod / 4096;

          //Serial.print("AcX: "); Serial.print(aX);
          //Serial.print(" AcY: "); Serial.print(aY);
          //Serial.print(" AcZ: "); Serial.print(aZ);
          //Serial.print(" AcMod: "); Serial.print(aMod);
          //Serial.print(" GyX: "); Serial.print(RateRoll);
          //Serial.print(" GyY: "); Serial.print(RatePitch);
          //Serial.print(" GyZ: "); Serial.println(RateYaw);

          sensorUpdate(aX, Accel_X.index);
          sensorUpdate(aY, Accel_Y.index);
          sensorUpdate(aZ, Accel_Z.index);
          sensorUpdate(aMod, Accel.index);
          sensorUpdate(RateRoll, Gyro_X.index);
          sensorUpdate(RatePitch, Gyro_Y.index);
          sensorUpdate(RateYaw, Gyro_Z.index);

          AccData[0] = AcX & 0xFF;
          AccData[1] = (AcX >> 8) & 0xFF;
          AccData[2] = AcY & 0xFF;
          AccData[3] = (AcY >> 8) & 0xFF;
          AccData[4] = AcZ & 0xFF;
          AccData[5] = (AcZ >> 8) & 0xFF;
          AccData[6] = AcMod & 0xFF;
          AccData[7] = (AcMod >> 8) & 0xFF;

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
      }
    }
    vTaskDelay(xFrequency);
  }
}

void fn_Group_0(__u8 data[GROUP0_DLC])
{
  __u16 r_seconds = word(data[0],data[1]);
  __u16 r_pw1 = word(data[2],data[3]);
  __u16 r_pw2 = word(data[4],data[5]);
  __u16 r_RPM = word(data[6],data[7]);

  __u16 seconds = MS2_U16_Calibration(r_seconds,MS2_1_cal,MS2_1_cal);
  float pw1 = MS2_Float_Calibration(r_pw1,MS2_1_cal,MS2_1000_cal);
  float pw2 = MS2_Float_Calibration(r_pw2,MS2_1_cal,MS2_1000_cal);
  __u16 RPM = MS2_U16_Calibration(r_RPM,MS2_1_cal,MS2_1_cal);
  

  //setPayload(rpm_byte,RPM);

  sensorUpdate(seconds, MS2_Sec.index);
  sensorUpdate(pw1, MS2_Bank1.index);
  sensorUpdate(pw2, MS2_Bank2.index);
  sensorUpdate(RPM, RPM_Sensor.index);
}

void fn_Group_1(__u8 data[GROUP1_DLC])
{
  __s16 r_Fin_Ign_Sprk_Adv = word(data[0],data[1]);
  __u8 r_BatchFire_Inj_Events = data[2];
  __u8 r_EngineStatus = data[3];
  __u8 r_Bank1_AFR_Tgt = data[4];
  __u8 r_Bank2_AFR_Tgt = data[5];

float Fin_Ign_Sprk_Adv = MS2_Float_Calibration(r_Fin_Ign_Sprk_Adv,MS2_1_cal,MS2_10_cal);
__u8 BatchFire_Inj_Events = MS2_U8_Calibration(r_BatchFire_Inj_Events,MS2_1_cal,MS2_1_cal);
__u8 EngineStatus = MS2_U8_Calibration(r_EngineStatus,MS2_1_cal,MS2_1_cal);
float Bank1_AFR_Tgt = MS2_Float_Calibration(r_Bank1_AFR_Tgt,MS2_1_cal,MS2_10_cal);
float Bank2_AFR_Tgt = MS2_Float_Calibration(r_Bank2_AFR_Tgt,MS2_1_cal,MS2_10_cal);

sensorUpdate(Fin_Ign_Sprk_Adv, MS2_Fin_Ign_Sprk_Adv.index);
sensorUpdate(BatchFire_Inj_Events, MS2_BatchFire_Inj_Events.index);
sensorUpdate(EngineStatus, MS2_EngineStatus.index);
sensorUpdate(Bank1_AFR_Tgt, MS2_Bank1_AFR_Tgt.index);
sensorUpdate(Bank2_AFR_Tgt, MS2_Bank2_AFR_Tgt.index);

}

void fn_Group_2(__u8 data[GROUP2_DLC])
{
  
  __s16 r_Baro = word(data[0],data[1]);
  __s16 r_MAP = word(data[2],data[3]);
  __s16 r_MAT = word(data[4],data[5]);
  __s16 r_CLT = word(data[6],data[7]);


  float Baro = MS2_Float_Calibration(r_Baro,MS2_1_cal,MS2_10_cal);
  float MAP = MS2_Float_Calibration(r_MAP,MS2_1_cal,MS2_10_cal);
  float MAT = (MS2_Float_Calibration(r_MAT,MS2_1_cal,MS2_10_cal)-32)*FtC;
  float CLT = (MS2_Float_Calibration(r_CLT,MS2_1_cal,MS2_10_cal)-32)*FtC;
  

  sensorUpdate(Baro, MS2_Baro_Press.index);
  sensorUpdate(MAP, MS2_MAP.index);
  sensorUpdate(MAT, MS2_MAT.index);
  sensorUpdate(CLT, MS2_CLT.index);
}

void fn_Group_3(__u8 data[GROUP3_DLC])
{
  __s16 r_TPS = word(data[0],data[1]);
  __s16 r_Voltage = word(data[2],data[3]);
  __s16 r_AFR1 = word(data[4],data[5]);
  __s16 r_AFR2 = word(data[6],data[7]);


  __s16 TPS = MS2_S16_Calibration(r_TPS,MS2_1_cal,MS2_10_cal);
  float Voltage = MS2_Float_Calibration(r_Voltage,MS2_1_cal,MS2_10_cal);
  float AFR1 = MS2_Float_Calibration(r_AFR1,MS2_1_cal,MS2_10_cal);
  float AFR2 = MS2_Float_Calibration(r_AFR2,MS2_1_cal,MS2_10_cal);
  

  sensorUpdate(TPS, MS2_TPS.index);
  sensorUpdate(Voltage, MS2_Voltage.index);
  sensorUpdate(AFR1, MS2_AFR1.index);
  sensorUpdate(AFR2, MS2_AFR2.index);
}

/*void fn_Group_4(__u8 data[GROUP4_DLC])
{
  __s16 r_knockIn = word(data[0],data[1]);
  __s16 r_egoCor1 = word(data[2],data[3]);
  __s16 r_egoCor2 = word(data[4],data[5]);
  __s16 r_aircor = word(data[6],data[7]);


  __s16 knockIn = MS2_S16_Calibration(r_knockIn,MS2_1_cal,MS2_10_cal);
  __s16 egoCor1 = MS2_Float_Calibration(r_egoCor1,MS2_1_cal,MS2_10_cal);
  __s16 egoCor2 = MS2_Float_Calibration(r_egoCor2,MS2_1_cal,MS2_10_cal);
  __s16 aircor = MS2_Float_Calibration(r_aircor,MS2_1_cal,MS2_10_cal);
  

  sensorUpdate(knockIn, MS2_knockIn.index);
  sensorUpdate(egoCor1, MS2_egoCorr1.index);
  sensorUpdate(egoCor1, MS2_egoCorr2.index);
  sensorUpdate(aircor, MS2_aircor.index);
}*/

void fn_Group_7(__u8 data[GROUP7_DLC])
{
  __s16 r_cold_Adv = word(data[0],data[1]);
  __s16 r_TPS_rate = word(data[2],data[3]);
  __s16 r_MAP_rate = word(data[4],data[5]);
  __s16 r_RPM_rate = word(data[6],data[7]);

  float cold_Adv = MS2_Float_Calibration(r_cold_Adv,MS2_1_cal,MS2_10_cal);
  float TPS_rate = MS2_Float_Calibration(r_TPS_rate,MS2_1_cal,MS2_10_cal);
  float MAP_rate = MS2_Float_Calibration(r_MAP_rate,MS2_1_cal,MS2_1_cal);
  __s16 RPM_rate = MS2_S16_Calibration(r_RPM_rate,MS2_10_cal,MS2_1_cal);

  sensorUpdate(cold_Adv, MS2_Cold_Adv.index);
  sensorUpdate(TPS_rate, MS2_TPS_Rate.index);
  sensorUpdate(MAP_rate, MS2_MAP_Rate.index);
  sensorUpdate(RPM_rate, MS2_RPM_Rate.index);
}

void fn_Group_8(__u8 data[GROUP8_DLC])
{
  __s16 r_MAF_Load = word(data[0],data[1]);
  __s16 r_Fuel_Load = word(data[2],data[3]);
  __s16 r_Fuel_Correction = word(data[4],data[5]);
  __s16 r_MAF = word(data[6],data[7]);

  float MAF_Load = MS2_Float_Calibration(r_MAF_Load,MS2_1_cal,MS2_10_cal);
  float Fuel_Load = MS2_Float_Calibration(r_Fuel_Load,MS2_1_cal,MS2_10_cal);
  float Fuel_Correction = MS2_Float_Calibration(r_Fuel_Correction,MS2_1_cal,MS2_10_cal);
  float MAF = MS2_Float_Calibration(r_MAF,MS2_1_cal,MS2_100_cal);

  sensorUpdate(MAF_Load, MS2_MAF_Load.index);
  sensorUpdate(Fuel_Load, MS2_Fuel_Load.index);
  sensorUpdate(Fuel_Correction, MS2_Fuel_Correction.index);
  sensorUpdate(MAF, MS2_MAF.index);
}

void fn_Group_9(__u8 data[GROUP9_DLC])
{
  __s16 r_O2_V1= word(data[0],data[1]);
  __s16 r_O2_V2 = word(data[2],data[3]);
  __u16 r_Main_Dwell = word(data[4],data[5]);
  __u16 r_Trailing_Dwell = word(data[6],data[7]);

  float O2_V1 = MS2_Float_Calibration(r_O2_V1,MS2_1_cal,MS2_100_cal);
  float O2_V2 = MS2_Float_Calibration(r_O2_V2,MS2_1_cal,MS2_100_cal);
  float Main_Dwell = MS2_Float_Calibration(r_Main_Dwell,MS2_1_cal,MS2_10_cal);
  float Trailing_Dwell = MS2_Float_Calibration(r_Trailing_Dwell,MS2_1_cal,MS2_10_cal);

  sensorUpdate(O2_V1, MS2_O2_V1.index);
  sensorUpdate(O2_V2, MS2_O2_V2.index);
  sensorUpdate(Main_Dwell, MS2_Main_Ign_Dwell.index);
  sensorUpdate(Trailing_Dwell, MS2_Trailing_Ign_Dwell.index);
}

void fn_Group_15(__u8 data[GROUP15_DLC])
{
  __s16 r_OilPress= word(data[0],data[1]);
  //__s16 r_O2_V2 = word(data[2],data[3]);

  float OilPress = MS2_Float_Calibration(r_OilPress,MS2_1_cal,MS2_10_cal);

  sensorUpdate(OilPress, Oil_Pressure_Sensor.index);
}

void MessagesFN(void *parameter){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(MESSAGES_TIMER);
  static __u8 data[8];
  for (;;)
  {
    data[0] = sd_status&0b11;
    data[0] |= (accgyro_status&0b11)<<2;
    sendCANMessage(MESSAGES_ID,data,MESSAGES_DLC);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }

}

uint8_t readRegister(uint8_t reg) {
  I2C_MPU6050.beginTransmission(MPU_addr);
  I2C_MPU6050.write(reg);
  I2C_MPU6050.endTransmission(false);
  I2C_MPU6050.requestFrom(MPU_addr, 1, true);
  if (I2C_MPU6050.available()) {
    return I2C_MPU6050.read();
  }
  return 0xFF; // valor inválido se falhar
}

void TempTask(void *parameter){
    TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(TEMP_TIMER);

  static const unsigned char Register = 0x07;
  double data;
  for (;;)
  {
    /*if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      I2C_MPU6050.beginTransmission(GY906_ADD);
      I2C_MPU6050.write(0x3B); // Endereço do primeiro registrador de dados
    if (!I2C_MPU6050.endTransmission(false)){
    }
    I2C_MPU6050.requestFrom(GY906_ADD, 3, true);
    if (I2C_MPU6050.available() >= 3) {
    uint8_t lsb = I2C_MPU6050.read();
    uint8_t msb = I2C_MPU6050.read();
    uint8_t pec = I2C_MPU6050.read(); // byte de verificação, pode ser ignorado
    data = (msb << 8) | lsb;
  } else {
    Serial.println("Error"); // erro de leitura
  }
  xSemaphoreGive(i2cMutex);
    }
  // Conversão conforme datasheet
    Serial.println((data * 0.02) - 273.15);*/
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


/*void SIM_Task(void *parameter){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static TickType_t FirstTime = xTaskGetTickCount();
  const TickType_t xFrequency1 = pdMS_TO_TICKS(SIM_SETUP_TIMER);
  const TickType_t xFrequency2 = pdMS_TO_TICKS(SIM_TIMER);
  static bool sim_setup = 0;
  static bool mqtt_started = 0;
  static bool init = 0;
  static bool reset = 0;
  bool first_close = 1;
  bool first_open = 1;
  uint16_t data;
  for (;;)
  {
    if(!sim_setup){
      if (!init){
      String msg = "";
      bool m1=0;
      bool m2=0;
      TickType_t startTick = xTaskGetTickCount();
      TickType_t lastDataTick = startTick;
      bool init_get = 0;
      msg = readSIM(30000); //espera 20 segundos para iniciar 
        if (msg.indexOf("+CPIN: READY") != -1 && !m1) {
        m1=true;
        Serial.println("SIM OK");
      }
      if (msg.indexOf("SMS DONE") != -1 && !m2){
        m2=true;
        Serial.println("SMS OK");
      }
      if (msg.indexOf("PB DONE") != -1) {
        Serial.println("SIM7600 Initialized");
        init=1;
      }
      simCOM.println("AT");
      
      
      Serial.println(msg);
      msg = "";
      vTaskDelay(pdMS_TO_TICKS(10000));
      //init=1;
      }
      unsigned char val_flag = connection_val();
      if(val_flag==1){
        reset = 0;
        sim_setup=1;
      }
      vTaskDelayUntil(&xLastWakeTime, xFrequency1);
    }

    else if (sim_setup){
      
      if (!mqtt_started){
        if (mqtt_start(first_open)){
          mqtt_started = 1;
          first_open = 1;
        }
        else{
          first_open = 0;
        }
      }
      if(mqtt_started){
        if(mqtt_payload[0]!=0){
        if(!mqtt_publish("Test2142151",mqtt_payload)){
          
          while(!mqtt_close(first_close)){
            first_close = 0;
          }
          vTaskDelay(pdMS_TO_TICKS(100));
          mqtt_started=0;
        }
        else{
          first_close = 1;
        }
        }
      }
      vTaskDelayUntil(&xLastWakeTime, xFrequency2);
    }
  }
}

void MQTT_Time_Task(void *parameter){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(MQTT_TIME_TIMER);
  for (;;)
  {
    while(row_write!=0){}
    while(timeValues[buffer_write]<timeValues[buffer_write-1]){}
    setPayload32(time_byte, (unsigned int)timeValues[buffer_write]);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}*/