#ifndef __myself_power_transform_h
#define __myself_power_transform_h


#include <math.h>

#define     PI          3.14159265358979323846f

typedef struct
{
    float a;
    float b;
    float c;
}ThreePhase;

typedef struct
{
    float alpha;
    float beta;
}Clarke;

typedef struct
{
    float d;
    float q;
}Park;

void Clarke_Transform(ThreePhase threephase, Clarke *clarke);
void Clarke_Transform_Inverse(Clarke clarke, ThreePhase *threephase);

void Park_Transform(Clarke clarke, Park *park, float theta);
void Park_Transform_Inverse(Park park, Clarke *clarke, float theta);

#endif















