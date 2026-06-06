#ifndef APP_H
#define APP_H

#include <stdint.h>
#include "lampControl.h"


extern void App_Init(void);
extern void App_SetLampState(LampType_e LampType, uint8_t LampValue);
extern void App_SetLampStateFromISR(LampType_e LampType, uint8_t LampValue);
extern void App_SetLampStateFromISR(LampType_e LampType, uint8_t LampValue);
extern void App_EnableRanging(uint8_t EnableRangingBitField);

#endif
