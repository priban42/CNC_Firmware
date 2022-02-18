#ifndef Thermistor_h
#define Thermistor_h
#include <Arduino.h>

class Thermistor{
  public:
    Thermistor(int pin, float resistor);
    float getTemp();
  private:
  int ThermistorPin;
  int Vo;
  float R1;
  float logR2, R2, T, Tc, Tf;
  float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
  
};

#endif 
