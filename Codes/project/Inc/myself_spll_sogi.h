#ifndef __myself_spll_sogi_h
#define __myself_spll_sogi_h

#include "myself_power_transform.h"
#include "myself_pid.h"

typedef struct 
{
    float value;
    float w;        // angular frequency, 2*pi*f
}SinglePhase;

typedef struct 
{
    Clarke clarke_last;
    Clarke clarke;
    float gain;     //0.5~1
    float temp_a;
    float temp_ai;
    float temp_bi;
}Sogi;

typedef struct
{
    Park park;
    PidPosition pid;    // ki is tiny < 1e-6
    float lock_theta;   // transforming electrical angle(rad)
    float lock_w;       // angular frequency, 2*pi*f
}Pll;

typedef struct 
{
    SinglePhase singlephase;
    Sogi sogi;
    Pll pll;
}Spll;

void SOGI_Init(Sogi *sogi);
void SOGI_Run(Sogi *sogi, SinglePhase singlephase);

void PLL_Init(Pll *pll);
void PLL_Run(Pll *pll, Clarke calrke, float w);

void SPLL_Init(Spll *spll);
float Sample_Phase_locked_loop(Spll *spll, SinglePhase input);

#endif

