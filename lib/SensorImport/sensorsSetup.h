#ifndef SENSORS_SETUP_H
#define SENSORS_SETUP_H
#include <Arduino.h>
#include <variables.h>

#define V_4095 3.3
#define A_20V 24818
#define A_5_5V 6825
const float VRefMax = 5.1615;

class Sensor
{
public:
  String value = "$";
  String name;
  __u8 index;
};



extern Sensor Voltage_Sensor;
extern Sensor Internal_Temperature_Sensor;
extern Sensor V_Ref_Sensor;
extern Sensor Gear_Pos_Sens;
extern Sensor RPM_Sensor;
extern Sensor Susp_Pos_FR_Sensor;
extern Sensor Susp_Pos_FL_Sensor;
extern Sensor Susp_Pos_RR_Sensor;
extern Sensor Susp_Pos_RL_Sensor;
extern Sensor SteerWheel_Pos_Sensor;
extern Sensor Accel_X;
extern Sensor Accel_Y;
extern Sensor Accel_Z;
extern Sensor Accel;
extern Sensor Gyro_X;
extern Sensor Gyro_Y;
extern Sensor Gyro_Z;
extern Sensor Wheel_Spd_FR_Sensor;
extern Sensor Wheel_Spd_FL_Sensor;
extern Sensor Wheel_Spd_RR_Sensor;
extern Sensor Wheel_Spd_RL_Sensor;
extern Sensor Disk_Temp_FR_Sensor;
extern Sensor Disk_Temp_FL_Sensor;
extern Sensor Disk_Temp_RR_Sensor;
extern Sensor Disk_Temp_RL_Sensor;
extern Sensor F_Brakeline_Pressure;
extern Sensor R_Brakeline_Pressure;
extern Sensor Caliper_Pressure_FR_Sensor;
extern Sensor Caliper_Pressure_FL_Sensor;
extern Sensor Caliper_Pressure_RR_Sensor;
extern Sensor Caliper_Pressure_RL_Sensor;
extern Sensor Throttle_Position_Sensor;
extern Sensor Brake_Position_Sensor;
extern Sensor Fuel_Pressure_Sensor;
extern Sensor Fuel_Temperature_Sensor;
extern Sensor Oil_Temperature_Sensor;
extern Sensor Oil_Pressure_Sensor;
extern Sensor Intercooler_Temperature_Sensor;
extern Sensor Intercooler_Pressure_Sensor;
extern Sensor In_Cooling_Temperature_Sensor;
extern Sensor Out_Cooling_Temperature_Sensor;
extern Sensor MAP_Sensor;
extern Sensor MAF_Sensor;
extern Sensor Cylinder_1_Pressure_Sensor;
extern Sensor Cylinder_2_Pressure_Sensor;
extern Sensor MS2_Sec;
extern Sensor MS2_Bank1;
extern Sensor MS2_Bank2;
extern Sensor MS2_Baro_Press;
extern Sensor MS2_MAP;
extern Sensor MS2_MAT;
extern Sensor MS2_CLT;
extern Sensor MS2_TPS;
extern Sensor MS2_Voltage;
extern Sensor MS2_AFR1;
extern Sensor MS2_AFR2;
extern Sensor MS2_Lambda;
extern Sensor MS2_Cold_Adv;
extern Sensor MS2_TPS_Rate;
extern Sensor MS2_MAP_Rate;
extern Sensor MS2_RPM_Rate;
extern Sensor MS2_MAF_Load;
extern Sensor MS2_Fuel_Load;
extern Sensor MS2_Fuel_Correction;
extern Sensor MS2_MAF;
extern Sensor MS2_O2_V1;
extern Sensor MS2_O2_V2;
extern Sensor MS2_Main_Ign_Dwell;
extern Sensor MS2_Trailing_Ign_Dwell;
extern Sensor MS2_Fin_Ign_Sprk_Adv;
extern Sensor MS2_BatchFire_Inj_Events;
extern Sensor MS2_EngineStatus;
extern Sensor MS2_Bank1_AFR_Tgt;
extern Sensor MS2_Bank2_AFR_Tgt;
extern Sensor Firewall_Temperature1_Sensor;
extern Sensor Firewall_Temperature2_Sensor;
extern Sensor Wing_Extensometer_1_Sensor;
extern Sensor Wing_Extensometer_2_Sensor;
extern Sensor Wing_Extensometer_3_Sensor;
extern Sensor Wing_Extensometer_4_Sensor;

// Formato para adicionar mais sensores " extern Sensor SensorVariable "

extern Sensor *sensorIndex[];

__u8 indexSetup();
void setSensorName();

float vRef_Proportion(uint16_t data);
String Gear_Pos(__u8 value);
float LinearSensor(__u16 value, float a, float b);
float TempSensor(__u16 value, __u32 R1, __u32 R2, float c1, float c2, float c3);
float vBatSensor(__u16 value);
float vRefSensor(__u16 value);
float internalTemp(__u16 value);
float suspSensor(__u16 value);
//float wheelAngleSensor(__u16 value);
unsigned short degreesofPrecision(uint16_t data, float max_Value, float decimal);
signed short AccAxisCalibration();
signed short GyroAxisCalibration();
float MS2_Calibration(float value, unsigned short m, unsigned short d);

float U16toFloat(uint16_t value, uint8_t precision_bits);

class SW_Settings
{
private:
  unsigned char dValue;
  float center;
  float c_Region1;
  signed short r_Region1;
  float c_Region2;
  signed short r_Region2;
  float c_Region5;
  signed short r_Region5;
  float c_Region6;
  signed short r_Region6;
  unsigned char regions[3] = {0, 0, 0};

public:
  unsigned char mValue;

  SW_Settings(float centerDeg, uint8_t mVal)
      : mValue(mVal), center(centerDeg)
  {
    dValue = 180 - mValue;
    r_Region2 = -dValue + 1;
    c_Region2 = fmodf((center + r_Region2), 360);
    r_Region5 = dValue - 1;
    c_Region5 = fmodf((center + r_Region5), 360);
    r_Region1 = r_Region2 - dValue;
    c_Region1 = fmodf((center + r_Region1), 360);
    r_Region6 = r_Region5 + dValue;
    c_Region6 = fmodf((center + r_Region6), 360);
  }
  unsigned char getOldRegion()
  {
    unsigned char sum = 0;
    unsigned char n = 0;
    for (unsigned char i = 0; i < 3; i++)
    {
      if (regions[i] != 0)
      {
        sum += regions[i];
        n++;
      }
    }
    return sum / n;
  }
  void setOldRegion(unsigned char value)
  {
    for (unsigned char i = 2; i > 0; i--)
    {
      regions[i] = regions[i - 1];
    }
    regions[0] = value;
  }
  
  String steeringWheelValue(unsigned short r_value)
  {
    float prop = vRef_Proportion(r_value);
    float middle =0;
    float value = (prop*360);

    static bool hypothetical = 1;
    static unsigned char current_region = 0;
    static unsigned char old_region;
    float conValue;
    String retValue = "";
    float test_value = fmodf(((value - center) + 540), 360) - 180;

    if (hypothetical)
    {
      if (test_value > r_Region2 && test_value < r_Region5)
      {
        hypothetical = 0;
        if (test_value > r_Region2 && test_value < 0)
        {
          current_region = 3;
          conValue = test_value;
        }
        else
        {
          current_region = 4;
          conValue = test_value;
        }
      }
      else if (test_value <= r_Region2 && test_value >= r_Region1)
      {
        current_region = 2;
        conValue = fmodf(((value - c_Region2) + 540), 360) - 180 + r_Region2;
      }
      else if (test_value < r_Region1)
      {
        current_region = 1;
        conValue = fmodf(((value - c_Region1) + 540), 360) - 180 + r_Region1;
      }
    }
    else
    {
      old_region = getOldRegion();
      if (old_region == 1)
      {
        test_value = fmodf(((value - c_Region1) + 540), 360) - 180 + r_Region1;
      }
      else if (old_region == 2)
      {
        test_value = fmodf(((value - c_Region2) + 540), 360) - 180 + r_Region2;
      }
      else if (old_region == 5)
      {
        test_value = fmodf(((value - c_Region5) + 540), 360) - 180 + r_Region5;
      }
      else if (old_region == 6)
      {
        test_value = fmodf(((value - c_Region6) + 540), 360) - 180 + r_Region6;
      }
      else
      {
        test_value = fmodf(((value - center) + 540), 360) - 180;
      }

      if (test_value > r_Region2 && test_value < 0)
      {
        current_region = 3;
        conValue = test_value;
      }
      else if (test_value >= 0 && test_value < r_Region5)
      {
        current_region = 4;
      }
      else if (test_value <= r_Region1 && (old_region == 1 || old_region == 2))
      {
        current_region = 1;
      }
      else if (test_value <= r_Region2 && (old_region == 1 || old_region == 2 || old_region == 3))
      {
        current_region = 2;
      }
      else if (test_value >= r_Region6 && (old_region == 5 || old_region == 6))
      {
        current_region = 6;
      }
      else if (test_value >= r_Region5 && (old_region == 4 || old_region == 5 || old_region == 6))
      {
        current_region = 5;
      }

      else
      {
        hypothetical = 1;
      }
    }
    retValue = String(test_value / 2);
    if (hypothetical)
    {
      retValue = retValue + "?";
    }
    setOldRegion(current_region);
    return retValue;
  }
};

extern SW_Settings SteeringWheel;

#endif