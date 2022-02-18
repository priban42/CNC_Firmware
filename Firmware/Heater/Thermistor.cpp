#include "Arduino.h"
#include "Thermistor.h"

Thermistor::Thermistor(int pin, float resistor){
  ThermistorPin = pin;
  R1 = resistor / 10;
}

float Thermistor::getTemp(){
  Vo = analogRead(ThermistorPin);
  
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  return(T - 273.15);
}
