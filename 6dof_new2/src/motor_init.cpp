#include <Arduino.h>
#include "motor_init.h" 
#include "parameter.h"
#include "gpio.h"
#include "TMCStepper.h"
#include "AccelStepper.h"


void Init_Joint_1(MotorStruct *Joint__)
{
    Joint__->standby_position = 10240;
    Joint__->homed_position = 8600; //13500
    Joint__->motor_max_current = MOTOR1_MAX_CURRENT;
    Joint__->joint_range_positive_steps = 14000;
    Joint__->joint_range_negative_steps = -14000;
    Joint__->microstep = MICROSTEP;
    Joint__->limit_switch_trigger = 0; // temp since sensor is faulty?
    Joint__->reduction_ratio = 1; //6.4; // 96 / 15  
    Joint__->position = -200;
    Joint__->speed = -100;
    Joint__->current = 0;
    Joint__->LIMIT = LIMIT1;
    Joint__->DIR = DIR1;
    Joint__->STEP = STEP1;
    Joint__->irun = 16;
    Joint__->ihold = 16;
    Joint__->hold_multiplier = 0.7;
    Joint__->direction_reversed = 1;
    Joint__->homed = 1;
}

void Init_Joint_2(MotorStruct *Joint__)
{
    Joint__->standby_position = -32000; 
    Joint__->homed_position = 19588;
    Joint__->motor_max_current = MOTOR2_MAX_CURRENT;
    Joint__->joint_range_positive_steps = -1200;
    Joint__->joint_range_negative_steps = -51587;
    Joint__->microstep = MICROSTEP;
    Joint__->limit_switch_trigger = 0; // temp since sensor is faulty?
    Joint__->reduction_ratio = 20; 
    Joint__->position = 0;
    Joint__->speed = 0;
    Joint__->current = 0;
    Joint__->LIMIT = LIMIT2;
    Joint__->DIR = DIR2;
    Joint__->STEP = STEP2;
    Joint__->irun = 16;
    Joint__->ihold = 16;
    Joint__->hold_multiplier = 0.8;
    Joint__->direction_reversed = 0;
    Joint__->homed = 0;
}

void Init_Joint_3(MotorStruct *Joint__)
{
    Joint__->standby_position = 57905;
    Joint__->homed_position = 23020;
    Joint__->motor_max_current = MOTOR3_MAX_CURRENT;
    Joint__->joint_range_positive_steps = 92000;
    Joint__->joint_range_negative_steps = 34700;
    Joint__->microstep = MICROSTEP;
    Joint__->limit_switch_trigger = 0; // temp since sensor is faulty?
    Joint__->reduction_ratio = 18.0952381;  
    Joint__->position = 0;
    Joint__->speed = 0;
    Joint__->current = 0;
    Joint__->LIMIT = LIMIT3;
    Joint__->DIR = DIR3;
    Joint__->STEP = STEP3;
    Joint__->irun = 16;
    Joint__->ihold = 16;
    Joint__->hold_multiplier = 0.8;
    Joint__->direction_reversed = 1;
    Joint__->homed = 0;
}


void Init_Joint_4(MotorStruct *Joint__)
{
    Joint__->standby_position = 0;
    Joint__->homed_position = -10200;
    Joint__->motor_max_current = MOTOR4_MAX_CURRENT;
    Joint__->joint_range_positive_steps = 7500;
    Joint__->joint_range_negative_steps = -7500;
    Joint__->microstep = MICROSTEP;
    Joint__->limit_switch_trigger = 0; // temp since sensor is faulty?
    Joint__->reduction_ratio = 4;  
    Joint__->position = 0;
    Joint__->speed = 0;
    Joint__->current = 0;
    Joint__->LIMIT = LIMIT4;
    Joint__->DIR = DIR4;
    Joint__->STEP = STEP4;
    Joint__->irun = 16;
    Joint__->ihold = 16;
    Joint__->hold_multiplier = 0.8;
    Joint__->direction_reversed = 0;
    Joint__->homed = 0;
}

void Init_Joint_5(MotorStruct *Joint__)
{
    Joint__->standby_position = 0;
    Joint__->homed_position = 8900;
    Joint__->motor_max_current = MOTOR5_MAX_CURRENT;
    Joint__->joint_range_positive_steps = 6400;
    Joint__->joint_range_negative_steps = -6400;
    Joint__->microstep = MICROSTEP;
    Joint__->limit_switch_trigger = 0; // temp since sensor is faulty?
    Joint__->reduction_ratio = 4;  
    Joint__->position = 0;
    Joint__->speed = 0;
    Joint__->current = 0;
    Joint__->LIMIT = LIMIT5;
    Joint__->DIR = DIR5;
    Joint__->STEP = STEP5;
    Joint__->irun = 16;
    Joint__->ihold = 16;
    Joint__->hold_multiplier = 0.8;
    Joint__->direction_reversed = 0;
    Joint__->homed = 0;
}

void Init_Joint_6(MotorStruct *Joint__)
{
    Joint__->standby_position = 32000;
    Joint__->homed_position = 15900;
    Joint__->motor_max_current = MOTOR6_MAX_CURRENT;
    Joint__->joint_range_positive_steps = 64000;
    Joint__->joint_range_negative_steps = 0;
    Joint__->microstep = MICROSTEP;
    Joint__->limit_switch_trigger = 0; // temp since sensor is faulty?
    Joint__->reduction_ratio = 10;  
    Joint__->position = 0;
    Joint__->speed = 0;
    Joint__->current = 0;
    Joint__->LIMIT = LIMIT6;
    Joint__->DIR = DIR6;
    Joint__->STEP = STEP6;
    Joint__->irun = 16;
    Joint__->ihold = 16;
    Joint__->hold_multiplier = 0.88;
    Joint__->direction_reversed = 1;
    Joint__->homed = 0;
}