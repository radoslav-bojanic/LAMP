#include <stdio.h>
#include "lampControl.h"

typedef struct LampState
{
    uint8_t bigLamp;
    uint8_t smallLamp;
} LampState_ts;

static LampState_ts LampState_s = 
{
    .bigLamp = LAMP_OFF,
    .smallLamp = LAMP_OFF
};

void LampControl_Init(void)
{
    LampState_s.bigLamp = LAMP_OFF;
    LampState_s.smallLamp = LAMP_OFF;
}

void LampControl_SetBigLamp(uint8_t LampState)
{
    LampState_s.bigLamp = LampState;
}

uint8_t LampControl_GetBigLamp(void)
{
    return LampState_s.bigLamp;
}

void LampControl_SetSmallLamp(uint8_t LampState)
{
    LampState_s.smallLamp = LampState;
}

uint8_t LampControl_GetSmallLamp(void)
{
    return LampState_s.smallLamp;
}
