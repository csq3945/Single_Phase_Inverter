#ifndef __myself_droop_control_h
#define __myself_droop_control_h

#include "myself_spll_sogi.h"

typedef struct
{
    float p;
    float q;
    // float s;
}PowerPqs;

typedef struct
{
    Sogi sogi_i;
    Sogi sogi_u;
    PowerPqs power_pq;
}SinglePhasePower;

typedef struct 
{
    float w;
    float amplitude;
    float m;
    float n;

    float out_w;
    float out_theta;
}DroopControl;


void Single_Phase_Power_Init(SinglePhasePower *power);
void Single_Phase_Calculated_Power(SinglePhasePower *power, SinglePhase u, SinglePhase i);

void Droop_Control_Init(DroopControl *droop);
float Droop_Control_Run(DroopControl *droop, PowerPqs power_pq);


#endif



