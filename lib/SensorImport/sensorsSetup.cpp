#include "sensorsSetup.h"

SW_Settings SteeringWheel(341.5, 135);

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
Sensor Accel;
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
Sensor Oil_Pressure_Sensor;
Sensor Intercooler_Temperature_Sensor;
Sensor Intercooler_Pressure_Sensor;
Sensor In_Cooling_Temperature_Sensor;
Sensor Out_Cooling_Temperature_Sensor;
Sensor MAP_Sensor;
Sensor MAF_Sensor;
Sensor Cylinder_1_Pressure_Sensor;
Sensor Cylinder_2_Pressure_Sensor;
Sensor MS2_Sec;
Sensor MS2_Bank1;
Sensor MS2_Bank2;
Sensor MS2_Baro_Press;
Sensor MS2_MAP;
Sensor MS2_MAT;
Sensor MS2_CLT;
Sensor MS2_TPS;
Sensor MS2_Voltage;
Sensor MS2_AFR1;
Sensor MS2_AFR2;
Sensor MS2_Lambda;
Sensor MS2_Cold_Adv;
Sensor MS2_TPS_Rate;
Sensor MS2_MAP_Rate;
Sensor MS2_RPM_Rate;
Sensor MS2_MAF_Load;
Sensor MS2_Fuel_Load;
Sensor MS2_Fuel_Correction;
Sensor MS2_MAF;
Sensor MS2_O2_V1;
Sensor MS2_O2_V2;
Sensor MS2_Main_Ign_Dwell;
Sensor MS2_Trailing_Ign_Dwell;
Sensor MS2_Fin_Ign_Sprk_Adv;
Sensor MS2_BatchFire_Inj_Events;
Sensor MS2_EngineStatus;
Sensor MS2_Bank1_AFR_Tgt;
Sensor MS2_Bank2_AFR_Tgt;
Sensor Firewall_Temperature1_Sensor;
Sensor Firewall_Temperature2_Sensor;
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
    &Accel,
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
    &Oil_Pressure_Sensor,
    &Intercooler_Temperature_Sensor,
    &Intercooler_Pressure_Sensor,
    &In_Cooling_Temperature_Sensor,
    &Out_Cooling_Temperature_Sensor,
    &MAP_Sensor,
    &MAF_Sensor,
    &Cylinder_1_Pressure_Sensor,
    &Cylinder_2_Pressure_Sensor,
    &MS2_Sec,
    &MS2_Bank1,
    &MS2_Bank2,
    &MS2_Baro_Press,
    &MS2_MAP,
    &MS2_MAT,
    &MS2_CLT,
    &MS2_TPS,
    &MS2_Voltage,
    &MS2_AFR1,
    &MS2_Bank1_AFR_Tgt,
    &MS2_AFR2,
    &MS2_Bank2_AFR_Tgt,
    &MS2_Lambda,
    &MS2_Cold_Adv,
    &MS2_TPS_Rate,
    &MS2_MAP_Rate,
    &MS2_RPM_Rate,
    &MS2_MAF_Load,
    &MS2_Fuel_Load,
    &MS2_Fuel_Correction,
    &MS2_MAF,
    &MS2_O2_V1,
    &MS2_O2_V2,
    &MS2_Main_Ign_Dwell,
    &MS2_Trailing_Ign_Dwell,
    &MS2_Fin_Ign_Sprk_Adv,
    &MS2_BatchFire_Inj_Events,
    &MS2_EngineStatus,
    &Firewall_Temperature1_Sensor,
    &Firewall_Temperature2_Sensor,
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
    Accel_X.name = "Accel_X (m/s^2)";
    Accel_Y.name = "Accel_Y (m/s^2)";
    Accel_Z.name = "Accel_Z (m/s^2)";
    Accel.name = "Accel_Mod (m/s^2)";
    Gyro_X.name = "Roll (°/s)";
    Gyro_Y.name = "Pitch (°/s)";
    Gyro_Z.name = "Yaw (°/s)";
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
    Oil_Pressure_Sensor.name = "Oil_Press (bar)";
    Intercooler_Temperature_Sensor.name = "Intercooler_In_Temp (ºC)";
    Intercooler_Pressure_Sensor.name = "Intercooler_Out_Temp (ºC)";
    In_Cooling_Temperature_Sensor.name = "Cooling_In_Temp (ºC)";
    Out_Cooling_Temperature_Sensor.name = "Cooling_Out_Temp (ºC)";
    MAP_Sensor.name = "MAP (kPa)";
    MAF_Sensor.name = "MAF (mm^3/s)";
    Cylinder_1_Pressure_Sensor.name = "Cylinder_1_Press (kPa)";
    Cylinder_2_Pressure_Sensor.name = "Cylinder_2_Press (kPa)";
    MS2_Sec.name = "MS2_Seconds(s)";
    MS2_Bank1.name = "Bank_1 (ms)";
    MS2_Bank2.name = "Bank_2 (ms)";
    MS2_Baro_Press.name = "Barometer_Press (kPa)";
    MS2_MAP.name = "MAP (kPa)";
    MS2_MAT.name = "MAT (ºC)";
    MS2_CLT.name = "CLT (ºC)";
    MS2_TPS.name = "TPS (%)";
    MS2_Voltage.name = "MS2_Voltage (V)";
    MS2_AFR1.name = "AFR_1";
    MS2_Bank1_AFR_Tgt.name = "Bank1_AFR_Target (AFR)";
    MS2_AFR2.name = "AFR_2";
    MS2_Bank2_AFR_Tgt.name = "Bank2_AFR_Target (AFR)";
    MS2_Lambda.name = "Lambda";
    MS2_Cold_Adv.name = "Cold Advance (º)";
    MS2_TPS_Rate.name = "TPS_Rate (%/s)";
    MS2_MAP_Rate.name = "MAP_Rate (kPa/s)";
    MS2_RPM_Rate.name = "RPM_Rate (RPM/s)";
    MS2_MAF_Load.name = "MAF_Load (%)";
    MS2_Fuel_Load.name = "Fuel_Load (%)";
    MS2_Fuel_Correction.name = "Fuel_Correction (%)";
    MS2_MAF.name = "MAF (g/s)";
    MS2_O2_V1.name = "O2_Voltage_1 (V)";
    MS2_O2_V2.name = "O2_Voltage_2 (V)";
    MS2_Main_Ign_Dwell.name = "Main_Ign_Dwell (ms)";
    MS2_Trailing_Ign_Dwell.name = "Trailing_Ign_Dwell (ms)";
    MS2_Fin_Ign_Sprk_Adv.name = "Final_Ign_Sprk_Adv (º)";
    MS2_BatchFire_Inj_Events.name = "BatchFire_Inj_Events";
    MS2_EngineStatus.name = "EngineStatus";
    Firewall_Temperature1_Sensor.name = "Firewall_Temp_1 (ºC)";
    Firewall_Temperature2_Sensor.name = "Firewall_Temp_2 (ºC)";
    Wing_Extensometer_1_Sensor.name = "Wing_Ext_1 (mm)";
    Wing_Extensometer_2_Sensor.name = "Wing_Ext_2 (mm)";
    Wing_Extensometer_3_Sensor.name = "Wing_Ext_3 (mm)";
    Wing_Extensometer_4_Sensor.name = "Wing_Ext_4 (mm)";
}
