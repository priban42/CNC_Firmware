#ifndef CNC_stepper_h
#define CNC_stepper_h
#include <Arduino.h>
#include <digitalWriteFast.h>


#define pin_dirX 48
#define pin_stepX 46

#define pin_dirY 44
#define pin_stepY 42

#define pin_dirZ 40
#define pin_stepZ 38

#define pin_dirE 52
#define pin_stepE 50


class CNC_stepper{
  public:
    CNC_stepper();
    void moveXYZ_noAcc();
    void moveXYE();
    void moveXYE_noAcc();
    void moveXYE_start_acc();
    void moveXYE_end_acc();
    
    void init();

    void set_next_X(long coordinate);
    void set_next_Y(long coordinate);
    void set_next_Z(long coordinate);
    void set_next_E(long coordinate);

    void set_current_X(long coordinate);
    void set_current_Y(long coordinate);
    void set_current_Z(long coordinate);
    void set_current_E(long coordinate);

    void set_stepsPerMM_X(float stepsPerMM);
    void set_stepsPerMM_Y(float stepsPerMM);
    void set_stepsPerMM_Z(float stepsPerMM);
    void set_stepsPerMM_E(float stepsPerMM);

    void set_speed(float Speed);
    void set_acceleration(float Acceleration);

  private:

    void setDirectionsXYZE();
    
    float Q_rsqrt(float number);
    
    float max_Speed = 50.0;
    float acceleration = 100.0;

    float steps_per_mm_X = 80.0;
    float steps_per_mm_Y = 80.0;
    float steps_per_mm_Z = 4000; //3200.0;
    float steps_per_mm_E = 1481.6;

    long directionX = -1; //1 or -1
    long directionY = -1;
    long directionZ = -1;

    long current_X = 0;
    long current_Y = 0;
    long current_Z = 0;
    long current_E = 0;

    long next_X = 0;
    long next_Y = 0;
    long next_Z = 0;
    long next_E = 0;

    long stepsToGo_X;
    long stepsToGo_Y;
    long stepsToGo_Z;
    long stepsToGo_E;

    long int acc_resolution;
    unsigned int acc_resolution_index = 40;//80

    float Base;
    float Base_speed;
    float Base_acceleration;

    boolean done_X;
    boolean done_Y;
    boolean done_Z;
    boolean done_E;

    unsigned long first_step;

    unsigned long step_count;
    
    unsigned long acceleration_distance_X;
    unsigned long speed_distance_X;
    float speed_X;
    float acceleration_X;
    float speed_part_X;
    float speed_sum_X;
    float distance_acc_sum_X;
    float distance_acc_sum_sum_X;
    unsigned long Distance_X;
    float distance_part_X;
    unsigned long step_count_X;
    float mezera_X;
    float mezera_sum_X;

    unsigned long acceleration_distance_Y;
    unsigned long speed_distance_Y;
    float speed_Y;
    float acceleration_Y;
    float speed_part_Y;
    float speed_sum_Y;
    float distance_acc_sum_Y;
    float distance_acc_sum_sum_Y;
    unsigned long Distance_Y;
    float distance_part_Y;
    unsigned long step_count_Y;
    float mezera_Y;
    float mezera_sum_Y;

    unsigned long acceleration_distance_Z;
    unsigned long speed_distance_Z;
    float speed_Z;
    float acceleration_Z;
    float speed_part_Z;
    float speed_sum_Z;
    float distance_acc_sum_Z;
    float distance_acc_sum_sum_Z;
    unsigned long Distance_Z;
    float distance_part_Z;
    unsigned long step_count_Z;
    float mezera_Z;
    float mezera_sum_Z;

    unsigned long acceleration_distance_E;
    unsigned long speed_distance_E;
    float speed_E;
    float acceleration_E;
    float speed_part_E;
    float speed_sum_E;
    float distance_acc_sum_E;
    float distance_acc_sum_sum_E;
    unsigned long Distance_E;
    float distance_part_E;
    unsigned long step_count_E;
    float mezera_E;
    float mezera_sum_E;
    
};
#endif
