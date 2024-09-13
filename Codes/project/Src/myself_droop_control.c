#include "myself_droop_control.h"

void Single_Phase_Power_Init(SinglePhasePower *power)
{
    SOGI_Init(&(power->sogi_i));
    SOGI_Init(&(power->sogi_u));
    power->power_pq.p = 0;
    power->power_pq.q = 0;
}

inline void Single_Phase_Calculated_Power(SinglePhasePower *power, SinglePhase u, SinglePhase i)
{
    SOGI_Run(&(power->sogi_i), i);
    SOGI_Run(&(power->sogi_u), u);
    power->power_pq.p = ((power->sogi_i.clarke.alpha * power->sogi_u.clarke.alpha)\
                        +(power->sogi_i.clarke.beta * power->sogi_u.clarke.beta))*0.5f;
    power->power_pq.q = ((power->sogi_i.clarke.alpha * power->sogi_u.clarke.beta)\
                        -(power->sogi_i.clarke.beta * power->sogi_u.clarke.alpha))*0.5f;
}

void Droop_Control_Init(DroopControl *droop)
{
    droop->amplitude = 1;
    droop->w = 2*PI/400;
    droop->m = 0.0001;
    droop->n = 0.000000001;

    droop->out_w = 0;
    droop->out_theta = 0;
}

/**
 * @brief Droop control
 * @param *droop Pointer to DroopControl struct
 * @param power_pq The power of p and q
 * @retval set value sinusoidal waveform
 */
inline float Droop_Control_Run(DroopControl *droop, PowerPqs power_pq)
{
    droop->out_w = droop->w - (power_pq.p * droop->m);
    droop->out_theta += droop->out_w;
    droop->out_theta = fmod(droop->out_theta, 2*PI);
    return (droop->amplitude-(power_pq.q*droop->n))*cosf(droop->out_theta);
}















