#include <sensorsSetup.h>
#include <variables.h>

float LinearSensor(__u16 value, float a, float b){
  return a*float(value)+b;
}

float TempSensor (__u16 value, __u32 R1, __u32 R2, float c1, float c2, float c3){
  float a = float(value)/4095;
  float R = a*((R1+R2)/(1-a));
  return c1 + c2*log(R)+c3*pow(log(R),3);
}

float vBatSensor(__u16 value){
  __u16 dop = degreesofPrecision(value,3.3,0.1);
  float vBat = (float) dop * A_20V / 4095;
  return vBat/A_20V * 20;
}

float vRefSensor(__u16 value){
  __u16 dop = degreesofPrecision(value,3.3,0.1);
  float vRef = (float) dop * A_5_5V / 4095;
  return vRef/A_5_5V * 5.5;
}

float suspSensor(__u16 value){
  float a = 0.0446;
  float b = -25.17;
  return ((float)value)*a+b;
}

String Gear_Pos(__u8 value){
    switch(value){
        case 7: 
        return "N";
        break;
        case 1: 
        return "1";
        break;
        case 2: 
        return "2";
        break;
        case 3: 
        return "3";
        break;
        case 4: 
        return "4";
        break;
        case 5: 
        return "5";
        break;
        case 6: 
        return "6";
        break;
    }
    return "ERROR";
}

unsigned short degreesofPrecision(uint16_t data, float max_Value, float decimal){
  float factor = decimal/max_Value;
  uint16_t arred = 4095.0 * factor;
  return (data/arred)*arred;
}