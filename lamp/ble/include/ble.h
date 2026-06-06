#ifndef BLE_H
#define BLE_H
#include <stdbool.h>
#include <stdint.h>

extern void Ble_Init();

extern bool Ble_GetRssi(int8_t* Rssi);
extern void Ble_NotifyBigLamp(uint8_t state);
extern void Ble_NotifySmallLamp(uint8_t state);

#endif
