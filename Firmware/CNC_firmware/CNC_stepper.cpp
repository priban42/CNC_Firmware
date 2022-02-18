#include "Arduino.h"
#include "CNC_stepper.h"
#include <digitalWriteFast.h>

CNC_stepper::CNC_stepper(){

}
void CNC_stepper::init(){
    pinMode(pin_dirX, OUTPUT);
    pinMode(pin_stepX, OUTPUT);
    pinMode(pin_dirY, OUTPUT);
    pinMode(pin_stepY, OUTPUT);
    pinMode(pin_dirZ, OUTPUT);
    pinMode(pin_stepZ, OUTPUT);
    pinMode(pin_dirE, OUTPUT);
    pinMode(pin_stepE, OUTPUT);
}


//////////////////////////////////////////////////////////

void CNC_stepper::moveXYE_noAcc(){
  setDirectionsXYZE();
  stepsToGo_X = abs(next_X - current_X);
  stepsToGo_Y = abs(next_Y - current_Y);
  stepsToGo_E = abs(next_E - current_E);

  Base = Q_rsqrt(sq(stepsToGo_X/steps_per_mm_X) + sq(stepsToGo_Y/steps_per_mm_Y));
  Base_speed = (Base*max_Speed);

  speed_X = Base_speed*stepsToGo_X;
  speed_Y = Base_speed*stepsToGo_Y;
  speed_E = Base_speed*stepsToGo_E;

  step_count_X = 0;
  step_count_Y = 0;
  step_count_E = 0;

  done_X = false;
  done_Y = false;
  done_E = false;
  
  mezera_X = 1000000/(speed_X);
  mezera_Y = 1000000/(speed_Y);
  mezera_E = 1000000/(speed_E);

  mezera_sum_X = 0;
  mezera_sum_Y = 0;
  mezera_sum_E = 0;
  
  first_step = micros();
  while (!done_X || !done_Y || !done_E){
    if (step_count_X < stepsToGo_X){
      if ((micros() - first_step) >= (mezera_sum_X)){
        digitalWriteFast(pin_stepX, HIGH);
        digitalWriteFast(pin_stepX, LOW);
        mezera_sum_X += mezera_X;
        step_count_X++;
      }
    }else{
      done_X = true;
    }
    if (step_count_Y < stepsToGo_Y){
      if ((micros() - first_step) >= (mezera_sum_Y)){
        digitalWriteFast(pin_stepY, HIGH);
        digitalWriteFast(pin_stepY, LOW);
        mezera_sum_Y += mezera_Y;
        step_count_Y++;
      }
    }else{
      done_Y = true;
    }
    if (step_count_E < stepsToGo_E){
      if ((micros() - first_step) >= (mezera_sum_E)){
        digitalWriteFast(pin_stepE, HIGH);
        digitalWriteFast(pin_stepE, LOW);
        mezera_sum_E += mezera_E;
        step_count_E++;
      }
    }else{
      done_E = true;
    }
  }

  current_X = next_X;
  current_Y = next_Y;
  current_E = next_E;
}

//////////////////////////////////////////////////////////

void CNC_stepper::moveXYZ_noAcc(){
  setDirectionsXYZE();
  stepsToGo_X = abs(next_X - current_X);
  stepsToGo_Y = abs(next_Y - current_Y);
  stepsToGo_Z = abs(next_Z - current_Z);

  Base = Q_rsqrt(sq(stepsToGo_X/steps_per_mm_X) + sq(stepsToGo_Y/steps_per_mm_Y) + sq(stepsToGo_Z/steps_per_mm_Z));
  Base_speed = (Base*max_Speed);

  speed_X = Base_speed*stepsToGo_X;
  speed_Y = Base_speed*stepsToGo_Y;
  speed_Z = Base_speed*stepsToGo_Z;

  step_count_X = 0;
  step_count_Y = 0;
  step_count_Z = 0;

  done_X = false;
  done_Y = false;
  done_Z = false;
  
  mezera_X = 1000000/(speed_X);
  mezera_Y = 1000000/(speed_Y);
  mezera_Z = 1000000/(speed_Z);

  mezera_sum_X = 0;
  mezera_sum_Y = 0;
  mezera_sum_Z = 0;
  
  first_step = micros();
  while (!done_X || !done_Y || !done_Z){
    if (step_count_X < stepsToGo_X){
      if ((micros() - first_step) >= (mezera_sum_X)){
        digitalWriteFast(pin_stepX, HIGH);
        digitalWriteFast(pin_stepX, LOW);
        mezera_sum_X += mezera_X;
        step_count_X++;
      }
    }else{
      done_X = true;
    }
    if (step_count_Y < stepsToGo_Y){
      if ((micros() - first_step) >= (mezera_sum_Y)){
        digitalWriteFast(pin_stepY, HIGH);
        digitalWriteFast(pin_stepY, LOW);
        mezera_sum_Y += mezera_Y;
        step_count_Y++;
      }
    }else{
      done_Y = true;
    }
    if (step_count_Z < stepsToGo_Z){
      if ((micros() - first_step) >= (mezera_sum_Z)){
        digitalWriteFast(pin_stepZ, HIGH);
        digitalWriteFast(pin_stepZ, LOW);
        mezera_sum_Z += mezera_Z;
        step_count_Z++;
      }
    }else{
      done_Z = true;
    }
  }

  current_X = next_X;
  current_Y = next_Y;
  current_Z = next_Z;
}

//////////////////////////////////////////////////////////

void CNC_stepper::moveXYE(){
  setDirectionsXYZE();
  stepsToGo_X = abs(next_X - current_X);
  stepsToGo_Y = abs(next_Y - current_Y);
  stepsToGo_E = abs(next_E - current_E);
  
  Base = Q_rsqrt(sq(stepsToGo_X/steps_per_mm_X) + sq(stepsToGo_Y/steps_per_mm_Y));
  Base_speed = (Base*max_Speed);
  Base_acceleration = (Base*acceleration);

  speed_X = Base_speed*stepsToGo_X;
  speed_Y = Base_speed*stepsToGo_Y;
  speed_E = Base_speed*stepsToGo_E;
  
  acceleration_X = Base_acceleration*stepsToGo_X;
  acceleration_Y = Base_acceleration*stepsToGo_Y;
  acceleration_E = Base_acceleration*stepsToGo_E;
  
  acceleration_distance_X = sq(speed_X)/(2.0*acceleration_X);
  acceleration_distance_Y = sq(speed_Y)/(2.0*acceleration_Y);
  acceleration_distance_E = sq(speed_E)/(2.0*acceleration_E);
  if (2*acceleration_distance_X > stepsToGo_X || 2*acceleration_distance_Y > stepsToGo_Y || 2*acceleration_distance_E > stepsToGo_E){
    acceleration_distance_X = (stepsToGo_X/2);
    speed_distance_X = stepsToGo_X%2;
    
    acceleration_distance_Y = (stepsToGo_Y/2);
    speed_distance_Y = stepsToGo_Y%2;
    
    acceleration_distance_E = (stepsToGo_E/2);
    speed_distance_E = stepsToGo_E%2;
  }else{
    
    speed_distance_X = stepsToGo_X - 2*acceleration_distance_X;
    speed_distance_Y = stepsToGo_Y - 2*acceleration_distance_Y;
    speed_distance_E = stepsToGo_E - 2*acceleration_distance_E;
  }

  acc_resolution = (max_Speed/acceleration)*acc_resolution_index + 1;

  distance_part_X = acceleration_distance_X/((acc_resolution+1)*acc_resolution/2.0);
  distance_part_Y = acceleration_distance_Y/((acc_resolution+1)*acc_resolution/2.0);
  distance_part_E = acceleration_distance_E/((acc_resolution+1)*acc_resolution/2.0);

  speed_part_X = speed_X/acc_resolution;
  speed_part_Y = speed_Y/acc_resolution;
  speed_part_E = speed_E/acc_resolution;

  step_count_X = 0;
  step_count_Y = 0;
  step_count_E = 0;

  Distance_X = 0;
  Distance_Y = 0;
  Distance_E = 0;

  distance_acc_sum_X = 0;
  distance_acc_sum_Y = 0;
  distance_acc_sum_E = 0;

  distance_acc_sum_sum_X = 0;
  distance_acc_sum_sum_Y = 0;
  distance_acc_sum_sum_E = 0;

  speed_sum_X = 0;
  speed_sum_Y = 0;
  speed_sum_E = 0;
  
  for (int x = 0; x < acc_resolution; x++){
    done_X = false;
    done_Y = false;
    done_E = false;
    
    distance_acc_sum_X += distance_part_X;
    distance_acc_sum_Y += distance_part_Y;
    distance_acc_sum_E += distance_part_E;
    
    distance_acc_sum_sum_X += distance_acc_sum_X;
    distance_acc_sum_sum_Y += distance_acc_sum_Y;
    distance_acc_sum_sum_E += distance_acc_sum_E;
    
    Distance_X = distance_acc_sum_sum_X + 0.5;
    Distance_Y = distance_acc_sum_sum_Y + 0.5;
    Distance_E = distance_acc_sum_sum_E + 0.5;
    
    speed_sum_X += speed_part_X;
    speed_sum_Y += speed_part_Y;
    speed_sum_E += speed_part_E;
    
    mezera_X = 1000000/(speed_sum_X);
    mezera_Y = 1000000/(speed_sum_Y);
    mezera_E = 1000000/(speed_sum_E);
    
    mezera_sum_X = 0;
    mezera_sum_Y = 0;
    mezera_sum_E = 0;
    
    first_step = micros();
    while (!done_X || !done_Y || !done_E){

      if (step_count_X < Distance_X){
        if ((micros() - first_step) >= (mezera_sum_X)){
          digitalWriteFast(pin_stepX, HIGH);
          digitalWriteFast(pin_stepX, LOW);
          mezera_sum_X += mezera_X;
          step_count_X++;
        }
      }else{
        done_X = true;
      }
      if (step_count_Y < Distance_Y){
        if ((micros() - first_step) >= (mezera_sum_Y)){
          digitalWriteFast(pin_stepY, HIGH);
          digitalWriteFast(pin_stepY, LOW);
          mezera_sum_Y += mezera_Y;
          step_count_Y++;
        }
      }else{
        done_Y = true;
      }
      if (step_count_E < Distance_E){
        if ((micros() - first_step) >= (mezera_sum_E)){
          digitalWriteFast(pin_stepE, HIGH);
          digitalWriteFast(pin_stepE, LOW);
          mezera_sum_E += mezera_E;
          step_count_E++;
        }
      }else{
        done_E = true;
      }
    }
  }

  done_X = false;
  done_Y = false;
  done_E = false;

  Distance_X += speed_distance_X;
  Distance_Y += speed_distance_Y;
  Distance_E += speed_distance_E;

  mezera_sum_X = 0;
  mezera_sum_Y = 0;
  mezera_sum_E = 0;
  
  first_step = micros();
  while (!done_X || !done_Y || !done_E){
    if (step_count_X < Distance_X){
      if ((micros() - first_step) >= (mezera_sum_X)){
        digitalWriteFast(pin_stepX, HIGH);
        digitalWriteFast(pin_stepX, LOW);
        mezera_sum_X += mezera_X;
        step_count_X++;
      }
    }else{
      done_X = true;
    }
    if (step_count_Y < Distance_Y){
      if ((micros() - first_step) >= (mezera_sum_Y)){
        digitalWriteFast(pin_stepY, HIGH);
        digitalWriteFast(pin_stepY, LOW);
        mezera_sum_Y += mezera_Y;
        step_count_Y++;
      }
    }else{
      done_Y = true;
    }
    if (step_count_E < Distance_E){
      if ((micros() - first_step) >= (mezera_sum_E)){
        digitalWriteFast(pin_stepE, HIGH);
        digitalWriteFast(pin_stepE, LOW);
        mezera_sum_E += mezera_E;
        step_count_E++;
      }
    }else{
      done_E = true;
    }
  }

  distance_acc_sum_sum_X = Distance_X + distance_acc_sum_X;
  distance_acc_sum_sum_Y = Distance_Y + distance_acc_sum_Y;
  distance_acc_sum_sum_E = Distance_E + distance_acc_sum_E;

  Distance_X = distance_acc_sum_sum_X + 0.5;
  Distance_Y = distance_acc_sum_sum_Y + 0.5;
  Distance_E = distance_acc_sum_sum_E + 0.5;

  for (int x = 0; x < acc_resolution; x++){
    done_X = false;
    done_Y = false;
    done_E = false;
    
    mezera_sum_X = 0;
    mezera_sum_Y = 0;
    mezera_sum_E = 0;
    
    first_step = micros();
    while (!done_X || !done_Y || !done_E){
      if (step_count_X < Distance_X){
        if ((micros() - first_step) >= (mezera_sum_X)){
          digitalWriteFast(pin_stepX, HIGH);
          digitalWriteFast(pin_stepX, LOW);
          mezera_sum_X += mezera_X;
          step_count_X++;
        }
      }else{
        done_X = true;
      }
      if (step_count_Y < Distance_Y){
        if ((micros() - first_step) >= (mezera_sum_Y)){
          digitalWriteFast(pin_stepY, HIGH);
          digitalWriteFast(pin_stepY, LOW);
          mezera_sum_Y += mezera_Y;
          step_count_Y++;
        }
      }else{
        done_Y = true;
      }
      if (step_count_E < Distance_E){
        if ((micros() - first_step) >= (mezera_sum_E)){
          digitalWriteFast(pin_stepE, HIGH);
          digitalWriteFast(pin_stepE, LOW);
          mezera_sum_E += mezera_E;
          step_count_E++;
        }
      }else{
        done_E = true;
      }
    }

    distance_acc_sum_X -= distance_part_X;
    distance_acc_sum_Y -= distance_part_Y;
    distance_acc_sum_E -= distance_part_E;
    
    distance_acc_sum_sum_X += distance_acc_sum_X;
    distance_acc_sum_sum_Y += distance_acc_sum_Y;
    distance_acc_sum_sum_E += distance_acc_sum_E;

    Distance_X = distance_acc_sum_sum_X + 0.5;
    Distance_Y = distance_acc_sum_sum_Y + 0.5;
    Distance_E = distance_acc_sum_sum_E + 0.5;

    speed_sum_X -= speed_part_X;
    speed_sum_Y -= speed_part_Y;
    speed_sum_E -= speed_part_E;
    
    mezera_X = 1000000/(speed_sum_X);
    mezera_Y = 1000000/(speed_sum_Y);
    mezera_E = 1000000/(speed_sum_E);
    
  }
  current_X = next_X;
  current_Y = next_Y;
  current_E = next_E;
}

//////////////////////////////////////////////////////////

void CNC_stepper::moveXYE_start_acc(){
  setDirectionsXYZE();
  stepsToGo_X = abs(next_X - current_X);
  stepsToGo_Y = abs(next_Y - current_Y);
  stepsToGo_E = abs(next_E - current_E);

  Base = Q_rsqrt(sq(stepsToGo_X/steps_per_mm_X) + sq(stepsToGo_Y/steps_per_mm_Y));
  Base_speed = (Base*max_Speed);
  Base_acceleration = (Base*acceleration);

  speed_X = Base_speed*stepsToGo_X;
  speed_Y = Base_speed*stepsToGo_Y;
  speed_E = Base_speed*stepsToGo_E;
  
  acceleration_X = Base_acceleration*stepsToGo_X;
  acceleration_Y = Base_acceleration*stepsToGo_Y;
  acceleration_E = Base_acceleration*stepsToGo_E;
  
  acceleration_distance_X = sq(speed_X)/(2.0*acceleration_X);
  acceleration_distance_Y = sq(speed_Y)/(2.0*acceleration_Y);
  acceleration_distance_E = sq(speed_E)/(2.0*acceleration_E);
  if (acceleration_distance_X > stepsToGo_X || acceleration_distance_Y > stepsToGo_Y || acceleration_distance_E > stepsToGo_E){
    acceleration_distance_X = (stepsToGo_X);
    speed_distance_X = 0;
    
    acceleration_distance_Y = (stepsToGo_Y);
    speed_distance_Y = 0;
    
    acceleration_distance_E = (stepsToGo_E);
    speed_distance_E = 0;
  }else{
    
    speed_distance_X = stepsToGo_X - acceleration_distance_X;
    speed_distance_Y = stepsToGo_Y - acceleration_distance_Y;
    speed_distance_E = stepsToGo_E - acceleration_distance_E;
  }

  acc_resolution = (max_Speed/acceleration)*acc_resolution_index + 1;

  distance_part_X = acceleration_distance_X/((acc_resolution+1)*acc_resolution/2.0);
  distance_part_Y = acceleration_distance_Y/((acc_resolution+1)*acc_resolution/2.0);
  distance_part_E = acceleration_distance_E/((acc_resolution+1)*acc_resolution/2.0);
  
  speed_part_X = speed_X/acc_resolution;
  speed_part_Y = speed_Y/acc_resolution;
  speed_part_E = speed_E/acc_resolution;

  step_count_X = 0;
  step_count_Y = 0;
  step_count_E = 0;

  Distance_X = 0;
  Distance_Y = 0;
  Distance_E = 0;

  distance_acc_sum_X = 0;
  distance_acc_sum_Y = 0;
  distance_acc_sum_E = 0;

  distance_acc_sum_sum_X = 0;
  distance_acc_sum_sum_Y = 0;
  distance_acc_sum_sum_E = 0;

  speed_sum_X = 0;
  speed_sum_Y = 0;
  speed_sum_E = 0;
  
  unsigned long soucet = 0;
  for (int x = 0; x < acc_resolution; x++){
    done_X = false;
    done_Y = false;
    done_E = false;
    
    distance_acc_sum_X += distance_part_X;
    distance_acc_sum_Y += distance_part_Y;
    distance_acc_sum_E += distance_part_E;
    
    distance_acc_sum_sum_X += distance_acc_sum_X;
    distance_acc_sum_sum_Y += distance_acc_sum_Y;
    distance_acc_sum_sum_E += distance_acc_sum_E;
    
    Distance_X = distance_acc_sum_sum_X + 0.5;
    Distance_Y = distance_acc_sum_sum_Y + 0.5;
    Distance_E = distance_acc_sum_sum_E + 0.5;
    
    speed_sum_X += speed_part_X;
    speed_sum_Y += speed_part_Y;
    speed_sum_E += speed_part_E;
    
    mezera_X = 1000000/(speed_sum_X);
    mezera_Y = 1000000/(speed_sum_Y);
    mezera_E = 1000000/(speed_sum_E);
    
    mezera_sum_X = 0;
    mezera_sum_Y = 0;
    mezera_sum_E = 0;

    first_step = micros();
    while (!done_X || !done_Y || !done_E){
      if (step_count_X < Distance_X){
        if ((micros() - first_step) >= (mezera_sum_X)){
          digitalWriteFast(pin_stepX, HIGH);
          digitalWriteFast(pin_stepX, LOW);
          mezera_sum_X += mezera_X;
          step_count_X++;
          soucet++;
        }
      }else{
        done_X = true;
      }
      if (step_count_Y < Distance_Y){
        if ((micros() - first_step) >= (mezera_sum_Y)){
          digitalWriteFast(pin_stepY, HIGH);
          digitalWriteFast(pin_stepY, LOW);
          mezera_sum_Y += mezera_Y;
          step_count_Y++;
          soucet++;
        }
      }else{
        done_Y = true;
      }
      if (step_count_E < Distance_E){
        if ((micros() - first_step) >= (mezera_sum_E)){
          digitalWriteFast(pin_stepE, HIGH);
          digitalWriteFast(pin_stepE, LOW);
          mezera_sum_E += mezera_E;
          step_count_E++;
          soucet++;
        }
      }else{
        done_E = true;
      }
    }
  }

  done_X = false;
  done_Y = false;
  done_E = false;

  Distance_X += speed_distance_X;
  Distance_Y += speed_distance_Y;
  Distance_E += speed_distance_E;

  mezera_sum_X = 0;
  mezera_sum_Y = 0;
  mezera_sum_E = 0;
  
  first_step = micros();
  while (!done_X || !done_Y || !done_E){
    if (step_count_X < Distance_X){
      if ((micros() - first_step) >= (mezera_sum_X)){
        digitalWriteFast(pin_stepX, HIGH);
        digitalWriteFast(pin_stepX, LOW);
        mezera_sum_X += mezera_X;
        step_count_X++;
      }
    }else{
      done_X = true;
    }
    if (step_count_Y < Distance_Y){
      if ((micros() - first_step) >= (mezera_sum_Y)){
        digitalWriteFast(pin_stepY, HIGH);
        digitalWriteFast(pin_stepY, LOW);
        mezera_sum_Y += mezera_Y;
        step_count_Y++;
      }
    }else{
      done_Y = true;
    }
    if (step_count_E < Distance_E){
      if ((micros() - first_step) >= (mezera_sum_E)){
        digitalWriteFast(pin_stepE, HIGH);
        digitalWriteFast(pin_stepE, LOW);
        mezera_sum_E += mezera_E;
        step_count_E++;
      }
    }else{
      done_E = true;
    }
  }
  current_X = next_X;
  current_Y = next_Y;
  current_E = next_E;
}

//////////////////////////////////////////////////////////

void CNC_stepper::moveXYE_end_acc(){
  setDirectionsXYZE();
  stepsToGo_X = abs(next_X - current_X);
  stepsToGo_Y = abs(next_Y - current_Y);
  stepsToGo_E = abs(next_E - current_E);

  Base = Q_rsqrt(sq(stepsToGo_X/steps_per_mm_X) + sq(stepsToGo_Y/steps_per_mm_Y));
  Base_speed = (Base*max_Speed);
  Base_acceleration = (Base*acceleration);

  speed_X = Base_speed*stepsToGo_X;
  speed_Y = Base_speed*stepsToGo_Y;
  speed_E = Base_speed*stepsToGo_E;
  
  acceleration_X = Base_acceleration*stepsToGo_X;
  acceleration_Y = Base_acceleration*stepsToGo_Y;
  acceleration_E = Base_acceleration*stepsToGo_E;
  
  acceleration_distance_X = sq(speed_X)/(2.0*acceleration_X);
  acceleration_distance_Y = sq(speed_Y)/(2.0*acceleration_Y);
  acceleration_distance_E = sq(speed_E)/(2.0*acceleration_E);
  if (acceleration_distance_X > stepsToGo_X || acceleration_distance_Y > stepsToGo_Y || acceleration_distance_E > stepsToGo_E){
    acceleration_distance_X = (stepsToGo_X);
    speed_distance_X = 0;
    
    acceleration_distance_Y = (stepsToGo_Y);
    speed_distance_Y = 0;
    
    acceleration_distance_E = (stepsToGo_E);
    speed_distance_E = 0;
  }else{
    speed_distance_X = stepsToGo_X - acceleration_distance_X;
    speed_distance_Y = stepsToGo_Y - acceleration_distance_Y;
    speed_distance_E = stepsToGo_E - acceleration_distance_E;
  }
    
  acc_resolution = (max_Speed/acceleration)*acc_resolution_index + 1;

  distance_part_X = acceleration_distance_X/((acc_resolution+1)*acc_resolution/2.0);
  distance_part_Y = acceleration_distance_Y/((acc_resolution+1)*acc_resolution/2.0);
  distance_part_E = acceleration_distance_E/((acc_resolution+1)*acc_resolution/2.0);
  
  speed_part_X = speed_X/acc_resolution;
  speed_part_Y = speed_Y/acc_resolution;
  speed_part_E = speed_E/acc_resolution;

  speed_sum_X = speed_X;
  speed_sum_Y = speed_Y;
  speed_sum_E = speed_E;

  step_count_X = 0;
  step_count_Y = 0;
  step_count_E = 0;

  Distance_X = speed_distance_X;
  Distance_Y = speed_distance_Y;
  Distance_E = speed_distance_E;

  distance_acc_sum_X = acc_resolution * distance_part_X;
  distance_acc_sum_Y = acc_resolution * distance_part_Y;
  distance_acc_sum_E = acc_resolution * distance_part_E;

  done_X = false;
  done_Y = false;
  done_E = false;

  mezera_X = 1000000/(speed_sum_X);
  mezera_Y = 1000000/(speed_sum_Y);
  mezera_E = 1000000/(speed_sum_E);

  mezera_sum_X = 0;
  mezera_sum_Y = 0;
  mezera_sum_E = 0;

  first_step = micros();
  while (!done_X || !done_Y || !done_E){

    if (step_count_X < Distance_X){
      if ((micros() - first_step) >= (mezera_sum_X)){
        digitalWriteFast(pin_stepX, HIGH);
        digitalWriteFast(pin_stepX, LOW);
        mezera_sum_X += mezera_X;
        step_count_X++;
      }
    }else{
      done_X = true;
    }
    if (step_count_Y < Distance_Y){
      if ((micros() - first_step) >= (mezera_sum_Y)){
        digitalWriteFast(pin_stepY, HIGH);
        digitalWriteFast(pin_stepY, LOW);
        mezera_sum_Y += mezera_Y;
        step_count_Y++;
      }
    }else{
      done_Y = true;
    }
    if (step_count_E < Distance_E){
      if ((micros() - first_step) >= (mezera_sum_E)){
        digitalWriteFast(pin_stepE, HIGH);
        digitalWriteFast(pin_stepE, LOW);
        mezera_sum_E += mezera_E;
        step_count_E++;
      }
    }else{
      done_E = true;
    }
  }

  distance_acc_sum_sum_X = Distance_X + distance_acc_sum_X;
  distance_acc_sum_sum_Y = Distance_Y + distance_acc_sum_Y;
  distance_acc_sum_sum_E = Distance_E + distance_acc_sum_E;

  Distance_X = distance_acc_sum_sum_X + 0.5;
  Distance_Y = distance_acc_sum_sum_Y + 0.5;
  Distance_E = distance_acc_sum_sum_E + 0.5;

  
  for (int x = 0; x < acc_resolution; x++){
    done_X = false;
    done_Y = false;
    done_E = false;
    
    mezera_sum_X = 0;
    mezera_sum_Y = 0;
    mezera_sum_E = 0;
    
    first_step = micros();
    while (!done_X || !done_Y || !done_E){
      if (step_count_X < Distance_X){
        if ((micros() - first_step) >= (mezera_sum_X)){
          digitalWriteFast(pin_stepX, HIGH);
          digitalWriteFast(pin_stepX, LOW);
          mezera_sum_X += mezera_X;
          step_count_X++;
        }
      }else{
        done_X = true;
      }
      if (step_count_Y < Distance_Y){
        if ((micros() - first_step) >= (mezera_sum_Y)){
          digitalWriteFast(pin_stepY, HIGH);
          digitalWriteFast(pin_stepY, LOW);
          mezera_sum_Y += mezera_Y;
          step_count_Y++;
        }
      }else{
        done_Y = true;
      }
      if (step_count_E < Distance_E){
        if ((micros() - first_step) >= (mezera_sum_E)){
          digitalWriteFast(pin_stepE, HIGH);
          digitalWriteFast(pin_stepE, LOW);
          mezera_sum_E += mezera_E;
          step_count_E++;
        }
      }else{
        done_E = true;
      }
    }

    distance_acc_sum_X -= distance_part_X;
    distance_acc_sum_Y -= distance_part_Y;
    distance_acc_sum_E -= distance_part_E;
    
    distance_acc_sum_sum_X += distance_acc_sum_X;
    distance_acc_sum_sum_Y += distance_acc_sum_Y;
    distance_acc_sum_sum_E += distance_acc_sum_E;

    Distance_X = distance_acc_sum_sum_X + 0.5;
    Distance_Y = distance_acc_sum_sum_Y + 0.5;
    Distance_E = distance_acc_sum_sum_E + 0.5;

    speed_sum_X -= speed_part_X;
    speed_sum_Y -= speed_part_Y;
    speed_sum_E -= speed_part_E;
    
    mezera_X = 1000000/(speed_sum_X);
    mezera_Y = 1000000/(speed_sum_Y);
    mezera_E = 1000000/(speed_sum_E);
    
  }

  current_X = next_X;
  current_Y = next_Y;
  current_E = next_E;
}

//////////////////////////////////////////////////////////

void CNC_stepper::set_next_X(long coordinate){
  next_X = coordinate;
}
void CNC_stepper::set_next_Y(long coordinate){
  next_Y = coordinate;
}
void CNC_stepper::set_next_Z(long coordinate){
  next_Z = coordinate;
}
void CNC_stepper::set_next_E(long coordinate){
  next_E = coordinate;
}
/////////////////////////
void CNC_stepper::set_stepsPerMM_X(float stepsPerMM){
  steps_per_mm_X = stepsPerMM;
}
void CNC_stepper::set_stepsPerMM_Y(float stepsPerMM){
  steps_per_mm_Y = stepsPerMM;
}
void CNC_stepper::set_stepsPerMM_Z(float stepsPerMM){
  steps_per_mm_Z = stepsPerMM;
}
void CNC_stepper::set_stepsPerMM_E(float stepsPerMM){
  steps_per_mm_E = stepsPerMM;
}
/////////////////////////
void CNC_stepper::set_speed(float Speed){
  max_Speed = Speed;
}
void CNC_stepper::set_acceleration(float Acceleration){
  acceleration = Acceleration;
}
////////////////////////
void CNC_stepper::set_current_X(long coordinate){
  current_X = coordinate;
}
void CNC_stepper::set_current_Y(long coordinate){
  current_Y = coordinate;
}
void CNC_stepper::set_current_Z(long coordinate){
  current_Z = coordinate;
}
void CNC_stepper::set_current_E(long coordinate){
  current_E = coordinate;
}
///////////////////////


void CNC_stepper::setDirectionsXYZE(){
  if (next_X > current_X){
    digitalWriteFast(pin_dirX, LOW);
  }else{
    digitalWriteFast(pin_dirX, HIGH);
  }
  if (next_Y > current_Y){
    digitalWriteFast(pin_dirY, LOW);
  }else{
    digitalWriteFast(pin_dirY, HIGH);
  }
  if (next_Z > current_Z){
    digitalWriteFast(pin_dirZ, LOW);
  }else{
    digitalWriteFast(pin_dirZ, HIGH);
  }
  if (next_E > current_E){
    digitalWriteFast(pin_dirE, LOW);
  }else{
    digitalWriteFast(pin_dirE, HIGH);
  }
}

float CNC_stepper::Q_rsqrt(float number){
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
