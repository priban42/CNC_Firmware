#include "Arduino.h"
#include "PID_temp.h"

PID_temp::PID_temp(int pin, float P, float I, float D, float V, float offsetIndex){
  p = P;
  i = I;
  d = D;
  v = V;
  HeaterPin = pin;
  pinMode(HeaterPin, OUTPUT);
  last_time = micros();
  last_D = millis();
}

void PID_temp::setTemp(int temp){
  tempature = temp;
}

float PID_temp::signum(float num){
  if (num > 0){
    return 1.0;
  }
  return -1.0;
}

boolean PID_temp::tempReached(){
  if (not sentDone){
    sentDone = done;
    return (done);
  }
  return false;
   
}

void PID_temp::refresh(float currentTemp){
  diff = tempature - currentTemp;
  v_sum = float(tempature)*v - 8*v;
  p_sum = p*diff;
  i_sum += signum(diff)*(1/(abs(diff)+5))*i*(micros() - last_time)/1000000;
  if (i_sum < 0){
    i_sum = 0;
  }
  last_time = micros();

  if ((millis() - last_D) > D_interval){
    last_D = millis();
    d_sum = (lastTemp - currentTemp) * d;
    lastTemp = currentTemp;
  }
  strength = (v_sum + p_sum + i_sum + d_sum);
  if (strength > 255){
    strength = 255;
  }
  if (strength < 0){
     strength = 0;
  }
  if (abs(diff) < 2){
    done = true;
  }else{
    done = false;
  }

  
  analogWrite(HeaterPin, strength);
  /*
  Serial.print(" temp:");
  Serial.print(currentTemp);
  Serial.print(" v:");
  Serial.print(v_sum);
  Serial.print(" p:");
  Serial.print(p_sum);
  Serial.print(" i:");
  Serial.print(i_sum);
  Serial.print(" d:");
  Serial.print(d_sum);
  Serial.print(" strengt:");
  Serial.print(strength);
  Serial.println();
  */
}
/*
byte PID_temp::refresh(int currentTemp){
  strength = float(tempature)*0.87 - 18;
  //strength = float(temp)*2.85 - 93 - 20; // heatbed
  long diff = (tempature - currentTemp);
  current_time = micros();
  if ((diff) < 5 and diff > -5)
  I += diff*i*(current_time - last_time)/1000000;
  if (I > 30){
    I = 30;
  }
  if (I<-5){
    I = -5;
  }
  last_time = current_time;
  long mezi = (diff)*abs(diff)*0.2 + diff*2 + I;
  if (mezi < 0){
    mezi = mezi/2;
  }
  strength += mezi;
  if (strength > 255){
    strength = 255;
  }
  if (strength < 0){
     strength = 0;
  }
  analogWrite(HeaterPin, strength);
  if (abs(diff)<3){
    done = true;
  }else{
    done = false;
  }
  
  return byte(strength);
}
*/
