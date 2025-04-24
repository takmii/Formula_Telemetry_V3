#include "sensorsSetup.h"

Sensor Voltage_Sensor;
Sensor Internal_Temperature_Sensor;
Sensor V_Ref_Sensor;
Sensor Gear_Pos_Sens;
Sensor Susp_Pos_FR_Sensor;
Sensor Susp_Pos_FL_Sensor;
Sensor Susp_Pos_RR_Sensor;
Sensor Susp_Pos_RL_Sensor;
Sensor SteerWheel_Pos_Sensor;
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
Sensor Throttle_Position_Sensor;
Sensor Brake_Position_Sensor;
Sensor Fuel_Pressure_Sensor;
Sensor Fuel_Temperature_Sensor;
Sensor Oil_Temperature_Sensor;
Sensor Intercooler_Temperature_Sensor;
Sensor Intercooler_Pressure_Sensor;
Sensor In_Cooling_Temperature_Sensor;
Sensor Out_Cooling_Temperature_Sensor;
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
    &Susp_Pos_FR_Sensor,
    &Susp_Pos_FL_Sensor,
    &Susp_Pos_RR_Sensor,
    &Susp_Pos_RL_Sensor,
    &SteerWheel_Pos_Sensor,
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
    &Throttle_Position_Sensor,
    &Brake_Position_Sensor,
    &Fuel_Pressure_Sensor,
    &Fuel_Temperature_Sensor,
    &Oil_Temperature_Sensor,
    &Intercooler_Temperature_Sensor,
    &Intercooler_Pressure_Sensor,
    &In_Cooling_Temperature_Sensor,
    &Out_Cooling_Temperature_Sensor,
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
    Voltage_Sensor.name = "Voltage (V)";
    Internal_Temperature_Sensor.name = "Voltage (V)";
    V_Ref_Sensor.name = "Voltage (V)";
    Gear_Pos_Sens.name = "Voltage (V)";
    Susp_Pos_FR_Sensor.name = "Voltage (V)";
    Susp_Pos_FL_Sensor.name = "Voltage (V)";
    Susp_Pos_RR_Sensor.name = "Voltage (V)";
    Susp_Pos_RL_Sensor.name = "Voltage (V)";
    SteerWheel_Pos_Sensor.name = "Voltage (V)";
    Wheel_Spd_FR_Sensor.name = "Voltage (V)";
    Wheel_Spd_FL_Sensor.name = "Voltage (V)";
    Wheel_Spd_RR_Sensor.name = "Voltage (V)";
    Wheel_Spd_RL_Sensor.name = "Voltage (V)";
    Disk_Temp_FR_Sensor.name = "Voltage (V)";
    Disk_Temp_FL_Sensor.name = "Voltage (V)";
    Disk_Temp_RR_Sensor.name = "Voltage (V)";
    Disk_Temp_RL_Sensor.name = "Voltage (V)";
    Caliper_Pressure_FR_Sensor.name = "Voltage (V)";
    Caliper_Pressure_FL_Sensor.name = "Voltage (V)";
    Caliper_Pressure_RR_Sensor.name = "Voltage (V)";
    Caliper_Pressure_RL_Sensor.name = "Voltage (V)";
    Throttle_Position_Sensor.name = "Voltage (V)";
    Brake_Position_Sensor.name = "Voltage (V)";
    Fuel_Pressure_Sensor.name = "Voltage (V)";
    Fuel_Temperature_Sensor.name = "Voltage (V)";
    Oil_Temperature_Sensor.name = "Voltage (V)";
    Intercooler_Temperature_Sensor.name = "Voltage (V)";
    Intercooler_Pressure_Sensor.name = "Voltage (V)";
    In_Cooling_Temperature_Sensor.name = "Voltage (V)";
    Out_Cooling_Temperature_Sensor.name = "Voltage (V)";
    Cylinder_1_Pressure_Sensor.name = "Voltage (V)";
    Cylinder_2_Pressure_Sensor.name = "Voltage (V)";
    Firewall_Temperature_Sensor.name = "Voltage (V)";
    Wing_Extensometer_1_Sensor.name = "Voltage (V)";
    Wing_Extensometer_2_Sensor.name = "Voltage (V)";
    Wing_Extensometer_3_Sensor.name = "Voltage (V)";
    Wing_Extensometer_4_Sensor.name = "Voltage (V)";
}