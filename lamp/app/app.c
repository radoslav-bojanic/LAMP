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

#define RSSI_SAMPLE_INTERVAL_MS     (100u)
#define PRESENCE_CONFIRM_COUNT      (5u)    /* N consecutive samples to confirm state change */
#define DISTANCE_CLOSE_M            (3.0f)
#define DISTANCE_FAR_M              (6.0f)  /* tighter gap — was 3m to 7m */
#define RSSI_MISSED_MAX             (3u)    /* tolerate N missed reads before resetting */


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
    LampStateChangedNotif_ts notif;

    for (;;)
    {
        /* drain queue without blocking so RSSI samples at fixed rate */
        while (xQueueReceive(LampStateChangedQueue, &notif, 0) == pdTRUE)
        {
            switch (notif.LampType)
            {
            case BigLamp:   LampControl_SetBigLamp(notif.LampValue);   break;
            case SmallLamp: LampControl_SetSmallLamp(notif.LampValue); break;
            default: break;
            }

            if (notif.EnableRanging != LAMP_UNUSED)
            {
                EnableRanging = notif.EnableRanging;
                ESP_LOGI(APP_TASK_NAME, "Ranging changed to %d", notif.EnableRanging);
            }
        }

        ProcessRssi();
        vTaskDelay(pdMS_TO_TICKS(20));  /* ~50Hz loop, RSSI_SAMPLE_INTERVAL controls actual rate */
    }
}

static void ProcessRssi(void)
{
    static TickType_t   last_sample   = 0;
    static uint8_t      close_count   = 0u;
    static uint8_t      far_count     = 0u;
    static uint8_t      missed_count  = 0u;

    if (EnableRanging == NO_RANGING || EnableRanging == LAMP_UNUSED)
    {
        return;
    }

    /* enforce fixed sample interval */
    TickType_t now = xTaskGetTickCount();
    if ((now - last_sample) < pdMS_TO_TICKS(RSSI_SAMPLE_INTERVAL_MS))
    {
        return;
    }
    last_sample = now;

    int8_t raw;
    if (!Ble_GetRssi(&raw))
    {
        missed_count++;
        if (missed_count >= RSSI_MISSED_MAX)
        {
            /* phone genuinely gone, reset filter on next valid read */
            s_rssi_init  = false;
            missed_count = 0u;
            close_count  = 0u;
            far_count    = 0u;
        }
        return;
    }
    missed_count = 0u;

    if (!s_rssi_init)
    {
        Kalman_Init(&s_kalman, (float)raw);
        s_rssi_init = true;
        return;  /* skip this sample, let filter settle */
    }

    float   filtered = Kalman_Update(&s_kalman, (float)raw);
    float   distance = Rssi_ToDistance((int8_t)filtered);

    ESP_LOGD(APP_TASK_NAME, "RSSI raw=%d filtered=%.1f dist=%.2fm", raw, filtered, distance);

    if (user_present == 0u)
    {
        /* looking for close */
        if (distance < DISTANCE_CLOSE_M)
        {
            close_count++;
            far_count = 0u;
            if (close_count >= PRESENCE_CONFIRM_COUNT)
            {
                close_count  = 0u;
                user_present = 1u;
                ESP_LOGI(APP_TASK_NAME, "User close dist=%.2fm", distance);
                if (EnableRanging & ENABLE_RANGING_BIG_LAMP_BIT)   App_SetLampState(BigLamp,   LAMP_ON);
                if (EnableRanging & ENABLE_RANGING_SMALL_LAMP_BIT) App_SetLampState(SmallLamp, LAMP_ON);
            }
        }
        else
        {
            close_count = 0u;
        }
    }
    else
    {
        /* looking for far */
        if (distance > DISTANCE_FAR_M)
        {
            far_count++;
            close_count = 0u;
            if (far_count >= PRESENCE_CONFIRM_COUNT)
            {
                far_count    = 0u;
                user_present = 0u;
                ESP_LOGI(APP_TASK_NAME, "User far dist=%.2fm", distance);
                if (EnableRanging & ENABLE_RANGING_BIG_LAMP_BIT)   App_SetLampState(BigLamp,   LAMP_OFF);
                if (EnableRanging & ENABLE_RANGING_SMALL_LAMP_BIT) App_SetLampState(SmallLamp, LAMP_OFF);
            }
        }
        else
        {
            far_count = 0u;
        }
    }
}

static void Kalman_Init(KalmanFilter_ts *kf, float initial_rssi)
{
    kf->q = 0.5f;           // low = trust model, high = trust measurement
    kf->r = 5.0f;           // typical BLE RSSI noise variance
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
