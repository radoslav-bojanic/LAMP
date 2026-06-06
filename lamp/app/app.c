#include <stdio.h>
#include "app.h"
#include "ble.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <math.h>

/* Definitions */
#define APP_TASK_STACK_SIZE (4096u)
#define APP_TASK_PRIO       (20u)
#define APP_TASK_NAME       "AppTask"
#define APP_QUEUE_LENGTH    (10u)

#define RSSI_TX_POWER   (-48)   // calibrate: measure RSSI while standing 1m from ESP
#define RSSI_PATH_LOSS  (3.0f)  // 2.0 free space, 2.5-3.0 indoors

typedef struct LampStateChangedNotif
{
    LampType_e LampType;
    uint8_t LampValue;
    uint8_t EnableRanging;
} LampStateChangedNotif_ts;

typedef struct
{
    float q;  // process noise covariance  (how much RSSI can jump between samples, tune this)
    float r;  // measurement noise covariance (how noisy the raw RSSI is, tune this)
    float p;  // estimation error covariance (initialized once)
    float x;  // current estimate
} KalmanFilter_ts;

uint8_t EnableRanging = NO_RANGING;

/* Static variables */
static QueueHandle_t LampStateChangedQueue = NULL;  // private, static
static KalmanFilter_ts s_kalman;
static bool            s_rssi_init = false;
static uint8_t user_present = 0u;

/* Static functions */
static void AppTask(void *pvParameters);
static void ProcessRssi(void);
static void Kalman_Init(KalmanFilter_ts *kf, float initial_rssi);
static float Kalman_Update(KalmanFilter_ts *kf, float measurement);
static float Rssi_ToDistance(int8_t rssi);

static void AppTask(void *pvParameters)
{
    (void)pvParameters;
    LampStateChangedNotif_ts LampStateChangedNotif_s;
    for(;;)
    {
        if (xQueueReceive(LampStateChangedQueue, &LampStateChangedNotif_s, pdMS_TO_TICKS(80)) == pdTRUE)
        {
            switch(LampStateChangedNotif_s.LampType)
            {
                case BigLamp:
                {
                    LampControl_SetBigLamp(LampStateChangedNotif_s.LampValue);
                    break;
                }
                case SmallLamp:
                {
                    LampControl_SetSmallLamp(LampStateChangedNotif_s.LampValue);
                    break;
                }
                default:
                {
                    /* No lamp change case */
                    break;
                }
            }

            if(LampStateChangedNotif_s.EnableRanging != LAMP_UNUSED)
            {
                EnableRanging = LampStateChangedNotif_s.EnableRanging;
                ESP_LOGI(APP_TASK_NAME, "Ranging changed to %d", LampStateChangedNotif_s.EnableRanging);
            }
            ESP_LOGI(APP_TASK_NAME, "LampStateChangedReceived");
        }

        ProcessRssi();
    }
}

static void ProcessRssi(void)
{

    if ((EnableRanging == LAMP_UNUSED) || (EnableRanging == NO_RANGING))
    {
        return;
    }

    int8_t raw;
    if (!Ble_GetRssi(&raw))
    {
        s_rssi_init = false;
        return;
    }

    if (!s_rssi_init)
    {
        Kalman_Init(&s_kalman, (float)raw);
        s_rssi_init = true;
    }

    float filtered = Kalman_Update(&s_kalman, (float)raw);
    int8_t rssi    = (int8_t)filtered;

    if((Rssi_ToDistance(rssi) < 3) && (user_present == 0))
    {
        ESP_LOGI(APP_TASK_NAME, "User close");
        user_present = 1;
        App_SetLampState(BigLamp, LAMP_ON);
    }
    else if((Rssi_ToDistance(rssi) > 7) && (user_present == 1))
    {
        ESP_LOGI(APP_TASK_NAME, "User far");
        user_present = 0;
        App_SetLampState(BigLamp, LAMP_OFF);
    }
    else
    {
        /* Hysteresis not reached*/
    }
}

static void Kalman_Init(KalmanFilter_ts *kf, float initial_rssi)
{
    kf->q = 0.1f;           // low = trust model, high = trust measurement
    kf->r = 2.0f;           // typical BLE RSSI noise variance
    kf->p = 1.0f;           // initial uncertainty
    kf->x = initial_rssi;
}

static float Kalman_Update(KalmanFilter_ts *kf, float measurement)
{
    /* predict */
    kf->p = kf->p + kf->q;

    /* update */
    float k = kf->p / (kf->p + kf->r);   // Kalman gain
    kf->x   = kf->x + k * (measurement - kf->x);
    kf->p   = (1.0f - k) * kf->p;

    return kf->x;
}

static float Rssi_ToDistance(int8_t rssi)
{
    return powf(10.0f, ((float)(RSSI_TX_POWER - rssi)) / (10.0f * RSSI_PATH_LOSS));
}

void App_Init(void)
{
    TaskHandle_t xHandle = NULL;

    LampStateChangedQueue = xQueueCreate(APP_QUEUE_LENGTH, sizeof(LampStateChangedNotif_ts));
    (void)xTaskCreate(
                    AppTask,       /* Function that implements the task. */
                    APP_TASK_NAME,          /* Text name for the task. */
                    APP_TASK_STACK_SIZE,      /* Stack size in words, not bytes. */
                    NULL,    /* Parameter passed into the task. */
                    APP_TASK_PRIO,/* Priority at which the task is created. */
                    &xHandle );
}

void App_SetLampState(LampType_e LampType, uint8_t LampValue)
{
    LampStateChangedNotif_ts LampStateChanged_s = 
    {
        .LampType = LampType,
        .LampValue = LampValue,
        .EnableRanging = LAMP_UNUSED
    };

    xQueueSend(LampStateChangedQueue, &LampStateChanged_s, pdMS_TO_TICKS(100)); 
}

void App_SetLampStateFromISR(LampType_e LampType, uint8_t LampValue)
{
    LampStateChangedNotif_ts LampStateChanged_s = 
    {
        .LampType = LampType,
        .LampValue = LampValue,
        .EnableRanging = LAMP_UNUSED
    };

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xQueueSendFromISR(LampStateChangedQueue, &LampStateChanged_s, &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void App_EnableRanging(uint8_t EnableRangingBitField)
{
    LampStateChangedNotif_ts LampStateChanged_s = 
    {
        .LampType = LAMP_UNUSED,
        .LampValue = LAMP_UNUSED,
        .EnableRanging = EnableRangingBitField
    };

    xQueueSend(LampStateChangedQueue, &LampStateChanged_s, pdMS_TO_TICKS(100)); 
}
