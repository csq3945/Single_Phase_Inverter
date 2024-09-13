#ifndef __myself_PID_h
#define __myself_PID_h

#include <math.h>
#include <stdlib.h>

typedef struct 
{
    float set_value;
    float actual_value;
    float error;
    float error_previous;
    float kp;
    float ki;
    float kd;
    float integral;
    float control_variable;
}PidPosition;

typedef struct
{
    float set_value;
    float actual_value;
    float error;
    float error_previous_01;
    float error_previous_02;
    float kp;
    float ki;
    float kd;
    float control_variable;
}PidIncremant;

typedef struct 
{
    float set_value;
    float actual_value;
    float error;
    float error_previous;
    float kp;
    float ki;
    float kd;
    float error_limit;
    float integral;
    float control_variable;
}PidSeparateIntegral;

typedef struct 
{
    float set_value;
    float actual_value;
    float error;
    float error_previous;
    float kp;
    float ki;
    float kd;
    float error_limit;
    float integral;
    float control_variable;
    float actual_value_max;
    float actual_value_min;
}PidAntisaturateIntegral;

typedef struct 
{
    float set_value;
    float actual_value;
    float error;
    float error_previous;
    float kp;
    float ki;
    float kd;
    float error_limit;
    float integral;
    float control_variable;
    float actual_value_max;
    float actual_value_min;
}PidTrapezoidIntegral;

typedef struct 
{
    float set_value;
    float actual_value;
    float error;
    float error_previous;
    float kp;
    float ki;
    float kd;
    float error_limit;
    float error_stage_01;
    float error_stage_02;
    float integral;
    float control_variable;
}PID_alterable_integral;

typedef struct
{
    float set_value;
    float actual_value;
    float error;
    float error_previous_01;
    float error_previous_02;
    float kp;
    float ki;
    float kd;
    float integral;
    float control_variable;
}PidIncompletionDifferential;


float Limit_Value(float value, float max, float min);

void PID_Position_Init(PidPosition *pid, float set_value, float kp, float ki, float kd);
float PID_Position_Run(PidPosition *pid, float measured_value);

void PID_Incremant_Init(PidIncremant *pid, float set_value, float kp, float ki, float kd);
float PID_Incremant_Run(PidIncremant *pid, float measured_value);

void PID_Separate_Integral_Init(PidSeparateIntegral *pid, float set_value, float kp, float ki, float kd, float error_limit);
float PID_Separate_Integral_Run(PidSeparateIntegral *pid, float measured_value);

void PID_Antisaturate_Integral_Init(PidAntisaturateIntegral *pid, float set_value, float kp, float ki, float kd,float error_limit, float actual_value_max, float actual_value_min);
float PID_Antisaturate_Integral_Run(PidAntisaturateIntegral *pid, float measured_value);

void PID_Trapezoid_Integral_Init(PidTrapezoidIntegral *pid, float set_value, float kp, float ki, float kd,float error_limit, float actual_value_max, float actual_value_min);
float PID_Trapezoid_Integral_Run(PidTrapezoidIntegral *pid, float measured_value);

void PID_Alterable_Integral_Init(PID_alterable_integral *pid, float set_value, float kp, float ki, float kd, float error_limit, float error_stage_01);
float PID_Alterable_Integral_Run(PID_alterable_integral *pid, float measured_value);

void PID_Incompletion_Differential_Init(PidIncompletionDifferential *pid, float set_value, float kp, float ki, float kd);
float PID_Incompletion_Differential_Run(PidIncompletionDifferential *pid, float measured_value);


#endif

