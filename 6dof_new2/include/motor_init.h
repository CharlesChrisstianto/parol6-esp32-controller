#pragma once
#include <Arduino.h>

struct MotorStruct
{   
    int standby_position;
    int homed_position;
    int motor_max_current; //max current in mA

    int joint_range_positive_steps;  // Positive motion limit in steps
    int joint_range_negative_steps;  // Negative motion limit in steps

    int reduction_ratio; // Gear reduction ratio
    int position;
    int speed;

    int current; // current current of the joint
    int microstep; // microstep step setting
    int irun;  //scale factor for running current based on set rms current
    int ihold; //scale factor for holding current based on set rms current  
    float hold_multiplier;

    int DIR;
    int STEP;
    int LIMIT;
    int limit_switch_trigger; // Active level for limit switch

    int direction_reversed;
    bool driver_initialized;
    bool homing;  // Whether joint is currently homing
    int homed;   // Whether joint has been homed

    int diag0;
    int version;
    int ioin;

    int commaned_mode;
    int commanded_position;
    int commanded_velocity;
    int commanded_current;

    
};

void Init_motor_struct(struct MotorStruct *Joint__);
void Init_Joint_1(struct MotorStruct *Joint__);
void Init_Joint_2(struct MotorStruct *Joint__);
void Init_Joint_3(struct MotorStruct *Joint__);
void Init_Joint_4(struct MotorStruct *Joint__);
void Init_Joint_5(struct MotorStruct *Joint__);
void Init_Joint_6(struct MotorStruct *Joint__);


