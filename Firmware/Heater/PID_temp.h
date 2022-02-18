#ifndef PID_temp_h
#define PID_temp_h
#include <Arduino.h>

class PID_temp{
  public:
    PID_temp(int pin, float P, float I, float D, float V, float offsetIndex);
    void setTemp(int temp);
    void refresh(float currentTemp);
    boolean tempReached();
    float tempature = 0;
    long strength;
    boolean sentDone = true;


    float i_sum = 0;
  private:

    boolean done = false;
    
    
    unsigned long last_time;
    float signum(float num);
    
    int HeaterPin;
    
    
    float diff;
      
    float p = 10;
    float i = 2;
    float d = 70;
    float v = 0.87;
    float offset_index = 10;

    float v_sum;
    float p_sum;
    
    float d_sum = 0;
    float lastTemp = 30;
    unsigned long D_interval = 2000;//ms
    unsigned long last_D;
};

#endif 
