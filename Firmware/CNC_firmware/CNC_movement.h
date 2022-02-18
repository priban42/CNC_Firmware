#ifndef CNC_movement_h
#define CNC_movement_h
#include <SpeedyStepper.h>
#include <Arduino.h>

#define dirE 52
#define stepE 50

#define dirX 48
#define stepX 46

#define dirY 44
#define stepY 42

#define dirZ 40
#define stepZ 38

#define enableE 36
#define enableXYZ 34

#define limit_switch_X 2
#define limit_switch_Y 3
#define limit_switch_Z 4

class CNC_movement{

  public:
    CNC_movement();
    void setOldCoordinates();
    
    void moveUniversal();
    void moveXYZ_noAcc();
    void moveXYE();
    void moveXYE_start_acc();
    void moveXYE_noAcc();
    void moveXYE_end_acc();
    void moveZ();
    void disableMotors();
    void enableMotors();
    void calibrate();

    void resetE();
    
    void set_EZ_Bases(byte byte_z, byte byte_e);
    
    float max_Speed = 30.0; // mm/s
    float acceleration = 700.0;
    float shortAccelerationMultiplication = 25;
    float minDistanceForAcceleration = 2;//mm
    float accIndex = 1;
    
    float max_SpeedE = 10.0; // mm/s
    float max_SpeedZ = 3.0; // mm/s
    
    float spmm_X = 80.0;//79.8; // 42.4 steps per mm
    float spmm_Y = 80.0;//79.8;
    float spmm_Z = 4000;//3200.0;
    float spmm_E = 1481.6;//370.4;//240.76;//120.38; //240.76;// 66.45676767*4
    
    long maxdX = 230*spmm_X;
    long maxdY = 230*spmm_Y;
    long maxdZ = 200*spmm_Z;

    long next_X = 0;
    long next_Y = 0;
    long next_Z = 0;
    long next_E = 0;
    
  private:
    
    SpeedyStepper stepperX;
    SpeedyStepper stepperY;
    SpeedyStepper stepperZ;
    SpeedyStepper stepperE;
    
    long current_X = 0;
    long current_Y = 0;
    long current_Z = 0;
    long current_E = 0;

    long distanceToGo_X;
    long distanceToGo_Y;
    long distanceToGo_Z;
    long distanceToGo_E;


    int directionX = -1;
    int directionY = -1;
    int directionZ = 1;
    int directionE = 1;
    
    

    byte byteX = 2;
    byte byteY = 2;
    
    byte byteZ = 3;
    byte byteE = 3;

    long baseX = long(1)<<(8*byteX - 1);
    long baseY = long(1)<<(8*byteY - 1);
    long baseZ = long(1)<<(8*byteZ - 1);
    long baseE = long(1)<<(8*byteE - 1);


    float Base;
    float meziSpeed;
    float meziAcceleration;

    long long_null = -2147483648;
  
};

#endif
