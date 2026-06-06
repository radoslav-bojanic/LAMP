#include <string.h>

#include "lampControl.h"
#include "app.h"
#include "ble.h"




void app_main(void)
{
    LampControl_Init();
    App_Init();
    Ble_Init();
}