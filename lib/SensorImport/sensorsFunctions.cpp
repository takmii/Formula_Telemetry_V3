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

String Gear_Pos(__u8 value){
    switch(value){
        case 0: 
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
}