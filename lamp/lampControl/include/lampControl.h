#ifndef LAMP_CONTROL_H
#define LAMP_CONTROL_H

#include <stdint.h>

#define LAMP_UNUSED (0xFFu)

#define LAMP_OFF (0u)
#define LAMP_ON  (1u)

#define NO_RANGING                     (0x00u)
#define ENABLE_RANGING_BIG_LAMP_BIT    (0x01u)
#define ENABLE_RANGING_SMALL_LAMP_BIT  (0x02u)

typedef enum
{
    BigLamp,
    SmallLamp,
    NumOfLamps,
    LampInvalid = 0xFF
} LampType_e;

extern void LampControl_Init(void);

extern void LampControl_SetBigLamp(uint8_t LampState);
extern uint8_t LampControl_GetBigLamp(void);

extern void LampControl_SetSmallLamp(uint8_t LampState);
extern uint8_t LampControl_GetSmallLamp(void);

#endif

