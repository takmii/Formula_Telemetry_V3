#include "sensorsSetup.h"

Sensor Voltage_Sensor;
Sensor Internal_Temperature_Sensor;
Sensor V_Ref_Sensor;
Sensor Gear_Pos_Sens;
Sensor RPM_Sensor;
Sensor Susp_Pos_FR_Sensor;
Sensor Susp_Pos_FL_Sensor;
Sensor Susp_Pos_RR_Sensor;
Sensor Susp_Pos_RL_Sensor;
Sensor SteerWheel_Pos_Sensor;
Sensor Accel_X;
Sensor Accel_Y;
Sensor Accel_Z;
Sensor Gyro_X;
Sensor Gyro_Y;
Sensor Gyro_Z;
Sensor Wheel_Spd_FR_Sensor;
Sensor Wheel_Spd_FL_Sensor;
Sensor Wheel_Spd_RR_Sensor;
Sensor Wheel_Spd_RL_Sensor;
Sensor Disk_Temp_FR_Sensor;
Sensor Disk_Temp_FL_Sensor;
Sensor Disk_Temp_RR_Sensor;
Sensor Disk_Temp_RL_Sensor;
Sensor Caliper_Pressure_FR_Sensor;
Sensor Caliper_Pressure_FL_Sensor;
Sensor Caliper_Pressure_RR_Sensor;
Sensor Caliper_Pressure_RL_Sensor;
Sensor F_Brakeline_Pressure;
Sensor R_Brakeline_Pressure;
Sensor Throttle_Position_Sensor;
Sensor Brake_Position_Sensor;
Sensor Fuel_Pressure_Sensor;
Sensor Fuel_Temperature_Sensor;
Sensor Oil_Temperature_Sensor;
Sensor Intercooler_Temperature_Sensor;
Sensor Intercooler_Pressure_Sensor;
Sensor In_Cooling_Temperature_Sensor;
Sensor Out_Cooling_Temperature_Sensor;
Sensor MAP_Sensor;
Sensor MAF_Sensor;
Sensor Cylinder_1_Pressure_Sensor;
Sensor Cylinder_2_Pressure_Sensor;
Sensor Firewall_Temperature_Sensor;
Sensor Wing_Extensometer_1_Sensor;
Sensor Wing_Extensometer_2_Sensor;
Sensor Wing_Extensometer_3_Sensor;
Sensor Wing_Extensometer_4_Sensor;
// Formato para adicionar mais sensores " Sensor SensorVariable "

Sensor *sensorIndex[] = {

    &Voltage_Sensor,
    &Internal_Temperature_Sensor,
    &V_Ref_Sensor,
    &Gear_Pos_Sens,
    &RPM_Sensor,
    &Susp_Pos_FR_Sensor,
    &Susp_Pos_FL_Sensor,
    &Susp_Pos_RR_Sensor,
    &Susp_Pos_RL_Sensor,
    &SteerWheel_Pos_Sensor,
    &Accel_X,
    &Accel_Y,
    &Accel_Z,
    &Gyro_X,
    &Gyro_Y,
    &Gyro_Z,
    &Wheel_Spd_FR_Sensor,
    &Wheel_Spd_FL_Sensor,
    &Wheel_Spd_RR_Sensor,
    &Wheel_Spd_RL_Sensor,
    &Disk_Temp_FR_Sensor,
    &Disk_Temp_FL_Sensor,
    &Disk_Temp_RR_Sensor,
    &Disk_Temp_RL_Sensor,
    &Caliper_Pressure_FR_Sensor,
    &Caliper_Pressure_FL_Sensor,
    &Caliper_Pressure_RR_Sensor,
    &Caliper_Pressure_RL_Sensor,
    &F_Brakeline_Pressure,
    &R_Brakeline_Pressure,
    &Throttle_Position_Sensor,
    &Brake_Position_Sensor,
    &Fuel_Pressure_Sensor,
    &Fuel_Temperature_Sensor,
    &Oil_Temperature_Sensor,
    &Intercooler_Temperature_Sensor,
    &Intercooler_Pressure_Sensor,
    &In_Cooling_Temperature_Sensor,
    &Out_Cooling_Temperature_Sensor,
    &MAP_Sensor,
    &MAF_Sensor,
    &Cylinder_1_Pressure_Sensor,
    &Cylinder_2_Pressure_Sensor,
    &Firewall_Temperature_Sensor,
    &Wing_Extensometer_1_Sensor,
    &Wing_Extensometer_2_Sensor,
    &Wing_Extensometer_3_Sensor,
    &Wing_Extensometer_4_Sensor

    // Formato para adicionar mais sensores " &SensorVariable "
};
// A posiçao do Sensor implica na ordem do arquivo CSV

__u8 indexSetup()
{
    for (__u8 i = 0; i < sizeof(sensorIndex) / sizeof(sensorIndex[0]); i++)
    {
        sensorIndex[i]->index = i;
    }
    return sizeof(sensorIndex) / sizeof(sensorIndex[0]);
}

void setSensorName()
{
    Voltage_Sensor.name = "Bat_Voltage (V)";
    Internal_Temperature_Sensor.name = "Int_Temp (ºC)";
    V_Ref_Sensor.name = "Ref_Voltage (V)";
    Gear_Pos_Sens.name = "Gear";
    RPM_Sensor.name = "RPM";
    Susp_Pos_FR_Sensor.name = "Susp_Pos_FR (º)";
    Susp_Pos_FL_Sensor.name = "Susp_Pos_FL (º)";
    Susp_Pos_RR_Sensor.name = "Susp_Pos_RR (º)";
    Susp_Pos_RL_Sensor.name = "Susp_Pos_RL (º)";
    SteerWheel_Pos_Sensor.name = "SteerWheel (º)";
    Accel_X.name = "Accel (X)";
    Accel_Y.name = "Accel (Y)";
    Accel_Z.name = "Accel (Z)";
    Gyro_X.name = "Gyro (X)";
    Gyro_Y.name = "Gyro (Y)";
    Gyro_Z.name = "Gyro (Z)";
    Wheel_Spd_FR_Sensor.name = "Wheel_SPD_FR (km/h)";
    Wheel_Spd_FL_Sensor.name = "Wheel_SPD_FL (km/h)";
    Wheel_Spd_RR_Sensor.name = "Wheel_SPD_RR (km/h)";
    Wheel_Spd_RL_Sensor.name = "Wheel_SPD_RL (km/h)";
    Disk_Temp_FR_Sensor.name = "Disk_Temp_FR (ºC)";
    Disk_Temp_FL_Sensor.name = "Disk_Temp_FL (ºC)";
    Disk_Temp_RR_Sensor.name = "Disk_Temp_RR (ºC)";
    Disk_Temp_RL_Sensor.name = "Disk_Temp_RL (ºC)";
    Caliper_Pressure_FR_Sensor.name = "Caliper_Press_FR (kPa)";
    Caliper_Pressure_FL_Sensor.name = "Caliper_Press_FL (kPa)";
    Caliper_Pressure_RR_Sensor.name = "Caliper_Press_RR (kPa)";
    Caliper_Pressure_RL_Sensor.name = "Caliper_Press_RL (kPa)";
    F_Brakeline_Pressure.name = "Brakeline_Pressure_F (kPa)";
    R_Brakeline_Pressure.name = "Brakeline_Pressure_F (kPa)";
    Throttle_Position_Sensor.name = "TPS (%)";
    Brake_Position_Sensor.name = "Brake (%)";
    Fuel_Pressure_Sensor.name = "Voltage (V)";
    Fuel_Temperature_Sensor.name = "Fuel_Temp (ºC)";
    Oil_Temperature_Sensor.name = "Oil_Temp (ºC)";
    Intercooler_Temperature_Sensor.name = "Intercooler_In_Temp (ºC)";
    Intercooler_Pressure_Sensor.name = "Intercooler_Out_Temp (ºC)";
    In_Cooling_Temperature_Sensor.name = "Cooling_In_Temp (ºC)";
    Out_Cooling_Temperature_Sensor.name = "Cooling_Out_Temp (ºC)";
    MAP_Sensor.name = "MAP (kPa)";
    MAF_Sensor.name = "MAF (mm^3/s)";
    Cylinder_1_Pressure_Sensor.name = "Cylinder_1_Press (kPa)";
    Cylinder_2_Pressure_Sensor.name = "Cylinder_2_Press (kPa)";
    Firewall_Temperature_Sensor.name = "Firewall_Temp (ºC)";
    Wing_Extensometer_1_Sensor.name = "Wing_Ext_1 (mm)";
    Wing_Extensometer_2_Sensor.name = "Wing_Ext_2 (mm)";
    Wing_Extensometer_3_Sensor.name = "Wing_Ext_3 (mm)";
    Wing_Extensometer_4_Sensor.name = "Wing_Ext_4 (mm)";
}