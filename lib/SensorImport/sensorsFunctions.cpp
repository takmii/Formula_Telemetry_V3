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
  float vRefProp = (float) dop / 4095;
  return vRefProp * VRefMax;
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

float suspSensor(__u16 value, bool direction, float center){
  int8_t dir = direction==1?1:-1;
  float prop = vRef_Proportion(value);
  return roundf(((prop*120) - center) * 10.0f * dir) / 10.0f;
}

float mapSensor(__u16 value){
  static const double a= 5*54.12;
  static const double b= -1.65;
  float prop = vRef_Proportion(value);
  return roundf((a*prop + b) * 10.0f) / 10.0f;
}

float mafSensor(__u16 value){
  const static float a = 13.49;
  const static float b = -31.78;
  const static float c = 61.578;
  const static float d = -43.707;
  float prop = vRef_Proportion(value);
  float pA = pow(5*prop,3);
  float pB = pow(5*prop,2);
  float pC = 5*prop;
  return roundf(((a*pA) + (b*pB) + (c*pC) + d) * 10.0f) / 10.0f;
}

float tempSensor(__u16 value,double a,double b,double c){
  const static unsigned short R1 = 2200;
  const static unsigned short R2 = 3900;

  float inv_prop = 1/vRef_Proportion(value);
  float R = (float)inv_prop*R2 - R2 - R1;
  float lR = log(R);
  float lR3 = pow(lR,3);
  return roundf((1/((a) + (b*lR) + (c*lR3))-273.15) * 10.0f) / 10.0f;
}

float tempOilSensor(__u16 value,double a,double b,double c){
  const static unsigned short R1 = 2200;
  const static unsigned short R2 = 3900;
  const static unsigned short R3 = 10000;
  const static float Req = (float) ((R1+R2)*R3)/(R1+R2+R3);

  float inv_prop = 1/vRef_Proportion(value);
  float R = (float)inv_prop*R2 - Req;
  float lR = log(R);
  float lR3 = pow(lR,3);
  return roundf((1/((a) + (b*lR) + (c*lR3))-273.15) * 10.0f) / 10.0f;
}


/*float wheelAngleSensor(__u16 value){
  float prop = vRef_Proportion(value);
  float middle =0;
  return roundf(((prop*360) - middle) * 10.0f) / 10.0f;
}*/

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
  return ((float)data / vRef_ >= 1.0f) ? 1.0f : (float)data / vRef_;
}

float U16toFloat(uint16_t value, uint8_t precision_bits){
    uint16_t scale = 1 << precision_bits;                // 2^precision_bits
    uint16_t intPart = value >> precision_bits;          // parte inteira
    uint16_t fracPart = value & (scale - 1);             // parte fracionária em "steps"
    return (float)intPart + ((float)fracPart / scale);   // reconstrução
}

float MS2_Float_Calibration(unsigned short value, unsigned short m, unsigned short d){
  return (float)value*m/d;
}

unsigned short MS2_U16_Calibration(unsigned short value, unsigned short m, unsigned short d){
  float v=(float)value*m/d;
  return (unsigned short)v;
}

unsigned char MS2_U8_Calibration(unsigned short value, unsigned short m, unsigned short d){
  float v=(float)value*m/d;
  return (unsigned char)v;
}

signed short MS2_S16_Calibration(unsigned short value, unsigned short m, unsigned short d){
  float v=(float)value*m/d;
  return (signed short)v;
}

signed char MS2_S8_Calibration(unsigned short value, unsigned short m, unsigned short d){
  float v=(float)value*m/d;
  return (signed char)v;
}
