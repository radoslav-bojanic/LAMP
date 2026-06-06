#include <stdio.h>
#include "lampControl.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "ble.h"

#define GPIO_BIG_LAMP    GPIO_NUM_6
#define GPIO_SMALL_LAMP  GPIO_NUM_7

#define LOG_TAG "LampControl"

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

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_BIG_LAMP) | (1ULL << GPIO_SMALL_LAMP),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /* default off */
    gpio_set_level(GPIO_BIG_LAMP,   0);
    gpio_set_level(GPIO_SMALL_LAMP, 0);
}

void LampControl_SetBigLamp(uint8_t LampState)
{
    LampState_s.bigLamp = LampState;
    if(LampState_s.bigLamp == LAMP_OFF)
    {
        gpio_set_level(GPIO_BIG_LAMP,   0);
        ESP_LOGI(LOG_TAG, "BIG LAMP 1");
    }
    else
    {
        gpio_set_level(GPIO_BIG_LAMP,   1);
        ESP_LOGI(LOG_TAG, "BIG LAMP 0");
    }
    Ble_NotifyBigLamp(LampState);
}

uint8_t LampControl_GetBigLamp(void)
{
    return LampState_s.bigLamp;
}

void LampControl_SetSmallLamp(uint8_t LampState)
{
    LampState_s.smallLamp = LampState;
    if(LampState_s.smallLamp == LAMP_OFF)
    {
        gpio_set_level(GPIO_SMALL_LAMP,   0);
    }
    else
    {
        gpio_set_level(GPIO_SMALL_LAMP,   1);
    }
    Ble_NotifySmallLamp(LampState);
}

uint8_t LampControl_GetSmallLamp(void)
{
    return LampState_s.smallLamp;
}
