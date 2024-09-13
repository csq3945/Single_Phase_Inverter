#include "myself_PID.h"

inline float Limit_Value(float value, float max, float min)
{
    if (value > max)
    {
        return max;
    }
    if (value < min)
    {
        return min;
    }
    return value;
}

void PID_Position_Init(PidPosition *pid, float set_value, float kp, float ki, float kd)
{
    pid->set_value = set_value;
    pid->actual_value = 0;
    pid->error = 0;
    pid->error_previous = 0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->control_variable = 0;
}
float PID_Position_Run(PidPosition *pid, float measured_value)
{
    pid->actual_value = measured_value;
    pid->error = pid->set_value - pid->actual_value;
    pid->integral += pid->error;
    pid->control_variable = pid->kp*pid->error + pid->ki*pid->integral + pid->kd*(pid->error-pid->error_previous);
    pid->error_previous = pid->error;
    return pid->control_variable;
}

void PID_Incremant_Init(PidIncremant *pid, float set_value, float kp, float ki, float kd)
{
    pid->set_value = set_value;
    pid->actual_value = 0;
    pid->error = 0;
    pid->error_previous_01 = 0;
    pid->error_previous_02 = 0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->control_variable = 0;
}
float PID_Incremant_Run(PidIncremant *pid, float measured_value)
{
    pid->actual_value = measured_value;
    pid->error = pid->set_value - pid->actual_value;
    pid->control_variable = pid->kp*(pid->error-pid->error_previous_01) + pid->ki*pid->error + pid->kd*(pid->error-2*pid->error_previous_01+pid->error_previous_02);
    pid->error_previous_02 = pid->error_previous_01;
    pid->error_previous_01 = pid->error;
    return pid->control_variable;
}

void PID_Separate_Integral_Init(PidSeparateIntegral *pid, float set_value, float kp, float ki, float kd, float error_limit)
{
    pid->set_value = set_value;
    pid->actual_value = 0;
    pid->error = 0;
    pid->error_previous = 0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error_limit = error_limit;
    pid->integral = 0;
    pid->control_variable = 0;
}
float PID_Separate_Integral_Run(PidSeparateIntegral *pid, float measured_value)
{
    unsigned char index;
    pid->actual_value = measured_value;
    pid->error = pid->set_value - pid->actual_value;
    if (abs(pid->error) > pid->error_limit)
    {
        index = 0;
    }
    else
    {
        index = 1;
        pid->integral += pid->error;
    }
    pid->control_variable = pid->kp*pid->error + index*pid->ki*pid->integral + pid->kd*(pid->error-pid->error_previous);
    pid->error_previous = pid->error;
    return pid->control_variable;
}

void PID_Antisaturate_Integral_Init(PidAntisaturateIntegral *pid, float set_value, float kp, float ki, float kd,float error_limit, float actual_value_max, float actual_value_min)
{
    pid->set_value = set_value;
    pid->actual_value = 0;
    pid->error = 0;
    pid->error_previous = 0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error_limit = error_limit;
    pid->integral = 0;
    pid->control_variable = 0;
    pid->actual_value_max = actual_value_max;
    pid->actual_value_min = actual_value_min;
}
float PID_Antisaturate_Integral_Run(PidAntisaturateIntegral *pid, float measured_value)
{
    unsigned char index;
    pid->actual_value = measured_value;
    pid->error = pid->set_value - pid->actual_value;
    if (pid->actual_value > pid->actual_value_max)
    {
        if (abs(pid->error) > pid->error_limit)
        {
            index = 0;
        }
        else
        {
            index = 1;
        }
    }
    else if (pid->actual_value < pid->actual_value_min)
    {
        if (abs(pid->error) > pid->error_limit)
        {
            index = 0;
        }
        else
        {
            index = 1;
            if (pid->error > 0)
            {
                pid->integral += pid->error;
            }
        }
    }
    else
    {
        if (abs(pid->error) > pid->error_limit)
        {
            index = 0;
        }
        else
        {
            index = 1;
            pid->integral += pid->error;
        }
    }
    pid->control_variable = pid->kp*pid->error + index*pid->ki*pid->integral + pid->kd*(pid->error-pid->error_previous);
    pid->error_previous = pid->error;
    return pid->control_variable;
}

void PID_Trapezoid_Integral_Init(PidTrapezoidIntegral *pid, float set_value, float kp, float ki, float kd,float error_limit, float actual_value_max, float actual_value_min)
{
    pid->set_value = set_value;
    pid->actual_value = 0;
    pid->error = 0;
    pid->error_previous = 0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error_limit = error_limit;
    pid->integral = 0;
    pid->control_variable = 0;
    pid->actual_value_max = actual_value_max;
    pid->actual_value_min = actual_value_min;
}
float PID_Trapezoid_Integral_Run(PidTrapezoidIntegral *pid, float measured_value)
{
    unsigned char index;
    pid->actual_value = measured_value;
    pid->error = pid->set_value - pid->actual_value;
    if (pid->actual_value > pid->actual_value_max)
    {
        if (abs(pid->error) > pid->error_limit)
        {
            index = 0;
        }
        else
        {
            index = 1;
        }
    }
    else if (pid->actual_value < pid->actual_value_min)
    {
        if (abs(pid->error) > pid->error_limit)
        {
            index = 0;
        }
        else
        {
            index = 1;
            if (pid->error > 0)
            {
                pid->integral += pid->error;
            }
        }
    }
    else
    {
        if (abs(pid->error) > pid->error_limit)
        {
            index = 0;
        }
        else
        {
            index = 1;
            pid->integral += ((pid->error+pid->error_previous)/2);
        }
    }
    pid->control_variable = pid->kp*pid->error + index*pid->ki*pid->integral + pid->kd*(pid->error-pid->error_previous);
    pid->error_previous = pid->error;
    return pid->control_variable;
}

void PID_Alterable_Integral_Init(PID_alterable_integral *pid, float set_value, float kp, float ki, float kd, float error_limit, float error_stage_01)
{
    pid->set_value = set_value;
    pid->actual_value = 0;
    pid->error = 0;
    pid->error_previous = 0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error_limit = error_limit;
    pid->error_stage_01 = error_stage_01;
    pid->error_stage_02 = 0;
    pid->integral = 0;
    pid->control_variable = 0;
}
float PID_Alterable_Integral_Run(PID_alterable_integral *pid, float measured_value)
{
    float index;
    pid->actual_value = measured_value;
    pid->error = pid->set_value - pid->actual_value;
    if (abs(pid->error) > pid->error_limit)
    {
        index = 0;
    }
    else if (abs(pid->error) < pid->error_stage_01)
    {
        index = 1;
        pid->integral += pid->error;
    }
    else
    {
        index = (pid->error_limit - abs(pid->error)) / 10;
        pid->integral += pid->error;
    }
    pid->control_variable = pid->kp*pid->error + index*pid->ki*pid->integral + pid->kd*(pid->error-pid->error_previous);
    pid->error_previous = pid->error;
    return pid->control_variable;
}

void PID_Incompletion_Differential_Init(PidIncompletionDifferential *pid, float set_value, float kp, float ki, float kd)
{
    pid->set_value = set_value;
    pid->actual_value = 0;
    pid->error = 0;
    pid->error_previous_01 = 0;
    pid->error_previous_02 = 0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->control_variable = 0;
}
float PID_Incompletion_Differential_Run(PidIncompletionDifferential *pid, float measured_value)
{
    pid->actual_value = measured_value;
    pid->error = pid->set_value - pid->actual_value;
    pid->integral += pid->error;
    pid->control_variable = pid->kp*pid->error + pid->ki*pid->integral + pid->kd*(pid->error-(pid->error_previous_01*2)+pid->error_previous_02);
    pid->error_previous_02 = pid->error_previous_01;
    pid->error_previous_01 = pid->error;
    return pid->control_variable;
}



