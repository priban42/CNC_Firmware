#include "Thermistor.h"
#include "PID_temp.h"
#include <SoftwareSerial.h>

SoftwareSerial megaSerial(7, 8);

int MillPin = 3;
int FanPin = 9;
int HotendPin = 10;
int HeatbedPin = 11;

Thermistor hotendMeter = Thermistor(A3, 8200);
Thermistor heatbedMeter = Thermistor(A2, 8200);
                          //int pin, float P, float I, float D, float V
PID_temp hotendHeater = PID_temp(HotendPin, 15, 2.5, 70, 0.85, 8); // HotendPin, 10, 2, 70, 0.85
PID_temp heatbedHeater = PID_temp(HeatbedPin, 20, 2, 50, 2.85, 15);// 30, 3, 70, 2.85

byte readByteSafe(){//waits until a byte arrives to buffer, than returns it
  while (not megaSerial.available()){}
  return megaSerial.read();
}

long readValue(byte bytes){//returns a number composed of several [bytes] in buffer
  long vraceni = 0;
  for (byte x = 0; x < bytes - 1; x++){
    vraceni += readByteSafe();
    vraceni *= 256;
  }
  vraceni += readByteSafe();
  return vraceni; 
}

void setup() {
  Serial.begin(115200);
  megaSerial.begin(38400);
  pinMode(FanPin, OUTPUT);
  pinMode(MillPin, OUTPUT);
}

int fanValue = 0;
int lastFanValue = 0;
void loop() {
  hotendHeater.refresh(hotendMeter.getTemp());
  heatbedHeater.refresh(heatbedMeter.getTemp());

  Serial.print(hotendMeter.getTemp());
  Serial.print("/");
  Serial.print(hotendHeater.tempature);
  Serial.print("/");
  Serial.print(hotendHeater.strength);
  Serial.print(" ");
  Serial.print(heatbedMeter.getTemp());
  Serial.print("/");
  Serial.print(heatbedHeater.tempature);
  Serial.print("/");
  Serial.println(heatbedHeater.strength);
  
  if (hotendHeater.tempReached()){
    megaSerial.write((1));
  }
  if (heatbedHeater.tempReached()){
    megaSerial.write((1));
  }
  
  if (megaSerial.available()){
    byte command = readValue(1);
    //Serial.println(command);
    switch(command){
      case 21:
        fanValue = int(readValue(2)*0.7);
        analogWrite(FanPin, fanValue);
        hotendHeater.i_sum += (fanValue-lastFanValue) * 1.1;
        lastFanValue = fanValue;
        break;
      case 22:
        analogWrite(FanPin, 0);
        break;
      case 23:
        heatbedHeater.setTemp(readValue(2));
        break;
      case 24:
        heatbedHeater.setTemp(readValue(2));
        heatbedHeater.sentDone = false;
        break;
      case 25:
        hotendHeater.setTemp(readValue(2));
        break;
      case 26:
        hotendHeater.setTemp(readValue(2));
        hotendHeater.sentDone = false;
        break;
      case 30:
        megaSerial.print(heatbedMeter.getTemp());
        megaSerial.print(" ");
        megaSerial.println(hotendMeter.getTemp());
        break;
    }
  }

}
