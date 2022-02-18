#include "Arduino.h"
#include "CNC_movement.h"
#include "CNC_stepper.h"
#include <SpeedyStepper.h>

SpeedyStepper stepperX;
SpeedyStepper stepperY;
SpeedyStepper stepperZ;
SpeedyStepper stepperE;

CNC_stepper CNC_Stepper;

CNC_movement::CNC_movement(){
  
  CNC_stepper CNC_Stepper;
  CNC_Stepper.init();
  
  CNC_Stepper.set_stepsPerMM_X(spmm_X);
  CNC_Stepper.set_stepsPerMM_Y(spmm_Y);
  CNC_Stepper.set_stepsPerMM_Z(spmm_Z);
  CNC_Stepper.set_stepsPerMM_E(spmm_E);
  
  stepperX.connectToPins(stepX, dirX);
  stepperY.connectToPins(stepY, dirY);
  stepperZ.connectToPins(stepZ, dirZ);
  stepperE.connectToPins(stepE, dirE);
  
  pinMode(enableE, OUTPUT); 
  pinMode(enableXYZ, OUTPUT);

  pinMode(limit_switch_X, INPUT);
  pinMode(limit_switch_Y, INPUT);
  pinMode(limit_switch_Z, INPUT);
  
}

void CNC_movement::disableMotors(){
  digitalWrite(enableE, HIGH);
  digitalWrite(enableXYZ, HIGH);
}
void CNC_movement::enableMotors(){
  digitalWrite(enableE, LOW);
  digitalWrite(enableXYZ, LOW);
}

void CNC_movement::resetE(){
  current_E = 0;
  next_E = 0;
  stepperE.setCurrentPositionInSteps(0);
  CNC_Stepper.set_current_E(0);
  CNC_Stepper.set_next_E(0);
}

void CNC_movement::set_EZ_Bases(byte byte_z, byte byte_e){
  byteZ = byte_z;
  byteE = byte_e;
  baseZ = pow(2, 8*byteZ-1);
  baseE = pow(2, 8*byteE-1);
}

void CNC_movement::setOldCoordinates(){
  current_X = next_X;
  current_Y = next_Y;
  current_Z = next_Z;
  current_E = next_E;
}

float Q_rsqrt(float number){
    long i;
    float x2, y;
    const float threehalfs = 1.5F;
    x2 = number * 0.5F;
    y  = number;
    i  = * ( long * ) &y;    // evil floating point bit level hacking
    i  = 0x5f3759df - ( i >> 1 );               // what the fuck? 
    y  = * ( float * ) &i;
    y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
    return y;
}

void CNC_movement::moveUniversal(){
  distanceToGo_X = abs(current_X - next_X);
  distanceToGo_Y = abs(current_Y - next_Y);
  distanceToGo_Z = abs(current_Z - next_Z);
  distanceToGo_E = abs(current_E - next_E);

  stepperX.setCurrentPositionInSteps(current_X);
  stepperY.setCurrentPositionInSteps(current_Y);
  stepperZ.setCurrentPositionInSteps(-current_Z);
  stepperE.setCurrentPositionInSteps(current_E);
  
  if (distanceToGo_X + distanceToGo_Y + distanceToGo_Z == 0){
    stepperE.setSpeedInStepsPerSecond(max_SpeedE*spmm_E);
    stepperE.setAccelerationInStepsPerSecondPerSecond(1000*spmm_E);
    stepperE.moveToPositionInSteps(next_E);
    while(!stepperE.motionComplete()){
      stepperE.processMovement();
    }
  }else{
    
    Base = Q_rsqrt(sq(distanceToGo_X/spmm_X)+sq(distanceToGo_Y/spmm_Y)+sq(distanceToGo_Z)/spmm_Z);
    
    meziSpeed = Base*max_Speed;
    meziAcceleration = Base*acceleration;
    
    stepperX.setSpeedInStepsPerSecond(distanceToGo_X*meziSpeed);
    stepperY.setSpeedInStepsPerSecond(distanceToGo_Y*meziSpeed);
    stepperZ.setSpeedInStepsPerSecond(distanceToGo_Z*meziSpeed);
    stepperE.setSpeedInStepsPerSecond(distanceToGo_E*meziSpeed);

    stepperX.setAccelerationInStepsPerSecondPerSecond(distanceToGo_X*meziAcceleration);
    stepperY.setAccelerationInStepsPerSecondPerSecond(distanceToGo_Y*meziAcceleration);
    stepperZ.setAccelerationInStepsPerSecondPerSecond(distanceToGo_Z*meziAcceleration);
    stepperE.setAccelerationInStepsPerSecondPerSecond(distanceToGo_E*meziAcceleration);
    
    stepperX.setupMoveInSteps(next_X);
    stepperY.setupMoveInSteps(next_Y);
    stepperZ.setupMoveInSteps(-next_Z);
    stepperE.setupMoveInSteps(next_E);

    while(!stepperX.motionComplete() || !stepperY.motionComplete() || !stepperZ.motionComplete() || !stepperE.motionComplete()){ 
      stepperX.processMovement();
      stepperY.processMovement();
      stepperZ.processMovement();
      stepperE.processMovement();
    }
  }
}

void CNC_movement::moveZ(){
  stepperZ.setSpeedInStepsPerSecond(max_SpeedZ*spmm_Z);
  stepperZ.setAccelerationInStepsPerSecondPerSecond(100*spmm_Z);
  stepperZ.setupMoveInSteps(-next_Z);
  while(!stepperZ.motionComplete()){ 
    stepperZ.processMovement();
  }
}

void CNC_movement::moveXYZ_noAcc(){
  CNC_Stepper.set_current_X(current_X);
  CNC_Stepper.set_current_Y(current_Y);
  CNC_Stepper.set_current_Z(current_Z);
  
  CNC_Stepper.set_next_X(next_X);
  CNC_Stepper.set_next_Y(next_Y);
  CNC_Stepper.set_next_Z(next_Z);
  
  CNC_Stepper.set_speed(max_Speed);
  CNC_Stepper.set_acceleration(acceleration);
  CNC_Stepper.moveXYZ_noAcc();
}

void CNC_movement::moveXYE(){
  CNC_Stepper.set_current_X(current_X);
  CNC_Stepper.set_current_Y(current_Y);
  CNC_Stepper.set_current_E(current_E);
  
  CNC_Stepper.set_next_X(next_X);
  CNC_Stepper.set_next_Y(next_Y);
  CNC_Stepper.set_next_E(next_E);
  
  CNC_Stepper.set_speed(max_Speed);
  CNC_Stepper.set_acceleration(acceleration);
  CNC_Stepper.moveXYE();
}

void CNC_movement::moveXYE_start_acc(){
  CNC_Stepper.set_current_X(current_X);
  CNC_Stepper.set_current_Y(current_Y);
  CNC_Stepper.set_current_E(current_E);
  
  CNC_Stepper.set_next_X(next_X);
  CNC_Stepper.set_next_Y(next_Y);
  CNC_Stepper.set_next_E(next_E);
  
  CNC_Stepper.set_speed(max_Speed);
  CNC_Stepper.set_acceleration(acceleration);
  CNC_Stepper.moveXYE_start_acc();
}

void CNC_movement::moveXYE_noAcc(){
  CNC_Stepper.set_current_X(current_X);
  CNC_Stepper.set_current_Y(current_Y);
  CNC_Stepper.set_current_E(current_E);
  
  CNC_Stepper.set_next_X(next_X);
  CNC_Stepper.set_next_Y(next_Y);
  CNC_Stepper.set_next_E(next_E);
  
  CNC_Stepper.set_speed(max_Speed);
  CNC_Stepper.set_acceleration(acceleration);
  CNC_Stepper.moveXYE_noAcc();
}

void CNC_movement::moveXYE_end_acc(){
  CNC_Stepper.set_current_X(current_X);
  CNC_Stepper.set_current_Y(current_Y);
  CNC_Stepper.set_current_E(current_E);
  
  CNC_Stepper.set_next_X(next_X);
  CNC_Stepper.set_next_Y(next_Y);
  CNC_Stepper.set_next_E(next_E);
  
  CNC_Stepper.set_speed(max_Speed);
  CNC_Stepper.set_acceleration(acceleration);
  CNC_Stepper.moveXYE_end_acc();
}

void CNC_movement::calibrate(){

  stepperZ.setSpeedInStepsPerSecond(max_SpeedZ*spmm_Z);
  stepperZ.setAccelerationInStepsPerSecondPerSecond(1000*spmm_Z);
  
  stepperZ.setupRelativeMoveInSteps(-10*spmm_Z);
  while(!stepperZ.motionComplete()){ 
    stepperZ.processMovement();
  }
  
  stepperX.setSpeedInStepsPerSecond(80*spmm_X);
  stepperX.setAccelerationInStepsPerSecondPerSecond(4000*spmm_X);
  stepperX.setupRelativeMoveInSteps((maxdX + 20*spmm_X));

  stepperY.setSpeedInStepsPerSecond(80*spmm_Y);
  stepperY.setAccelerationInStepsPerSecondPerSecond(4000*spmm_Y);
  stepperY.setupRelativeMoveInSteps((maxdY + 20*spmm_Y));
  
  while(!stepperX.motionComplete() || !stepperY.motionComplete()){ 
    for (int x = 0; x<20; x++){
      stepperX.processMovement();
      stepperY.processMovement();
    }
    if (digitalRead(limit_switch_X) == 1){
      stepperX.setupStop();
      stepperX.setupRelativeMoveInSteps(0);
    }
    if (digitalRead(limit_switch_Y) == 1){
      stepperY.setupStop();
      stepperY.setupRelativeMoveInSteps(0);
    }
  }

  stepperX.setCurrentPositionInSteps(0);
  stepperY.setCurrentPositionInSteps(0);
  current_X = 0;
  current_Y = 0;
  
  stepperZ.setupRelativeMoveInSteps(maxdZ);
  while(!stepperZ.motionComplete()){
    for (int x = 0; x<10; x++){
      stepperZ.processMovement();
    }
    if (digitalRead(limit_switch_Z) == 1){
      stepperZ.setupRelativeMoveInSteps(0);
      stepperZ.setupStop();
      while(!stepperZ.motionComplete()){
        stepperZ.processMovement();
      }
      break;
    }
  }
  stepperZ.setupRelativeMoveInSteps(-5*spmm_Z);
  while(!stepperZ.motionComplete()){ 
    stepperZ.processMovement();
    if (digitalRead(limit_switch_Z) == 0){
      stepperZ.setupRelativeMoveInSteps(0);
      stepperZ.setupStop();
      while(!stepperZ.motionComplete()){
        stepperZ.processMovement();
      }
    }
  }
  stepperZ.setupRelativeMoveInSteps(-3*spmm_Z);
  while(!stepperZ.motionComplete()){ 
    stepperZ.processMovement();
  }
  stepperZ.setCurrentPositionInSteps(0);
  current_Z = 0;
  
}
    
