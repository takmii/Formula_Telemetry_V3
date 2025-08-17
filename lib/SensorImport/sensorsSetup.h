#ifndef SENSORS_SETUP_H
#define SENSORS_SETUP_H
#include <Arduino.h>
#include <variables.h>

#define V_4095 3.3
#define A_20V 24818
#define A_5_5V 6825

class Sensor {
    public:
      String value ="$";
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
extern Sensor Intercooler_Temperature_Sensor;
extern Sensor Intercooler_Pressure_Sensor;
extern Sensor In_Cooling_Temperature_Sensor;
extern Sensor Out_Cooling_Temperature_Sensor;
extern Sensor MAP_Sensor;
extern Sensor MAF_Sensor;
extern Sensor Cylinder_1_Pressure_Sensor;
extern Sensor Cylinder_2_Pressure_Sensor;
extern Sensor Firewall_Temperature_Sensor;
extern Sensor Wing_Extensometer_1_Sensor;
extern Sensor Wing_Extensometer_2_Sensor;
extern Sensor Wing_Extensometer_3_Sensor;
extern Sensor Wing_Extensometer_4_Sensor;

// Formato para adicionar mais sensores " extern Sensor SensorVariable "

extern Sensor* sensorIndex[];

__u8 indexSetup();
void setSensorName();

float vRef_Proportion(uint16_t data);
String Gear_Pos(__u8 value);
float LinearSensor(__u16 value, float a, float b);
float TempSensor (__u16 value, __u32 R1, __u32 R2, float c1, float c2, float c3);
float vBatSensor(__u16 value);
float vRefSensor(__u16 value);
float internalTemp(__u16 value);
float suspSensor(__u16 value);
float wheelAngleSensor(__u16 value);
unsigned short degreesofPrecision(uint16_t data, float max_Value, float decimal);
signed short AccAxisCalibration();
signed short GyroAxisCalibration();

#endif