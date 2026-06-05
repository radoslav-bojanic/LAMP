#include <stdint.h>

#define LAMP_OFF (0u)
#define LAMP_ON  (1u)

static void LampControl_Init(void);

static void LampControl_SetBigLamp(uint8_t LampState);
static uint8_t LampControl_GetBigLamp(void);

static void LampControl_SetSmallLamp(uint8_t LampState);
static uint8_t LampControl_GetSmallLamp(void);

