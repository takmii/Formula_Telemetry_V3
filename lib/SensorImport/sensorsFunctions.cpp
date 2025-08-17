#include <sensorsSetup.h>
#include <variables.h>

__u16 vRef_=1;

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
  vRef_ = dop;
  float vRef = (float) dop * A_5_5V / 4095;
  return vRef/A_5_5V * 5.5;
}

float internalTemp(__u16 value){
  static const double Acoef = 8.6883e-4;
  static const double Bcoef = 2.5472e-4;
  static const double Ccoef = 1.7806e-7;
  static const double R = 1000.0;
  double Rntc = R * ((double)value / (4095.0 - (double)value));
  double lnR = log(Rntc);
  double invT = Acoef + Bcoef * lnR + Ccoef * lnR * lnR * lnR;
  double T = 1.0 / invT;    // Kelvin
  float tempC = (float)(T - 273.15); // Celsius
  tempC = roundf(tempC * 10.0f) / 10.0f; // Arredonda para 1 decimal
  return tempC;
}

float suspSensor(__u16 value){
  float prop = vRef_Proportion(value);
  float middle =0;
  return roundf(((prop*120) - middle) * 10.0f) / 10.0f;
}

float wheelAngleSensor(__u16 value){
  float prop = vRef_Proportion(value);
  float middle =0;
  return roundf(((prop*360) - middle) * 10.0f) / 10.0f;
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

float vRef_Proportion(uint16_t data){
  return (float)data/vRef_;
}