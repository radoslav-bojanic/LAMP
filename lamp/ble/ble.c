#include <stdio.h>
#include "ble.h"
#include "ble_cfg.h"
#include "app.h"


#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include "host/ble_gatt.h"
#include "nvs_flash.h"

/* Definitions and constants */
#define TAG "BLE app"
#define NOTIFY_QUEUE_LENGTH (5u)

typedef struct
{
    uint16_t attr_handle;
    uint8_t  state;
} NotifyEvent_ts;


/* Static variables */
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint8_t big_lamp_state = LAMP_OFF;
static uint8_t small_lamp_state = LAMP_OFF;
static uint8_t ranging_enabled = 0u;
static QueueHandle_t s_notify_queue = NULL;
static uint16_t big_lamp_val_handle;
static uint16_t small_lamp_val_handle;
static bool isPaired = false;
/* ---- notify task, runs in NimBLE context via callout ---- */
static struct ble_npl_callout s_notify_callout;

/* Static funcions prototypes */
static void start_advertising(void);
static int gap_event(struct ble_gap_event *event, void *arg);
static int lamp_access_cb(uint16_t conn_handle,
                          uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt,
                          void *arg);
static void ble_app_on_sync(void);
static void Ble_NotifyCallout(struct ble_npl_event *event);
static void Ble_ScheduleNotify(uint16_t attr_handle, uint8_t state);

static const struct ble_gatt_svc_def gatt_svcs[] = {
{
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = BLE_UUID16_DECLARE(LAMP_GATT_SERVICE),

    .characteristics = (struct ble_gatt_chr_def[]) { {

        .uuid      = BLE_UUID16_DECLARE(BIG_LAMP_GATT_CHAR),
        .access_cb = lamp_access_cb,
        .val_handle = &big_lamp_val_handle,
        .flags     = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,

    }, {

        .uuid      = BLE_UUID16_DECLARE(SMALL_LAMP_GATT_CHAR),
        .access_cb = lamp_access_cb,
        .val_handle = &small_lamp_val_handle,
        .flags     = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,

    }, {

        .uuid      = BLE_UUID16_DECLARE(ENABLE_RANGING_CHAR),
        .access_cb = lamp_access_cb,
        .flags     = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,

    }, {
        0
    } }
},
{0}
};

/* Static functions implementation */
static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT: 
    {

        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "CONNECT failed status=%d",
                     event->connect.status);
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            start_advertising();
            return 0;
        }

        conn_handle = event->connect.conn_handle;
        isPaired = false;

        ESP_LOGI(TAG, "CONNECTED conn_handle=%d", conn_handle);

        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(conn_handle, &desc) == 0) {

            ESP_LOGI(TAG,
                "peer addr=%02X:%02X:%02X:%02X:%02X:%02X",
                desc.peer_id_addr.val[5],
                desc.peer_id_addr.val[4],
                desc.peer_id_addr.val[3],
                desc.peer_id_addr.val[2],
                desc.peer_id_addr.val[1],
                desc.peer_id_addr.val[0]);

            ESP_LOGI(TAG,
                "interval=%d latency=%d timeout=%d",
                desc.conn_itvl,
                desc.conn_latency,
                desc.supervision_timeout);
        }

        break;
    }

    case BLE_GAP_EVENT_DISCONNECT: 
    {

        ESP_LOGW(TAG,
            "DISCONNECTED reason=%d conn_handle=%d",
            event->disconnect.reason,
            event->disconnect.conn.conn_handle);

        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        isPaired = false;

        start_advertising();
        break;
    }

    case BLE_GAP_EVENT_CONN_UPDATE: 
    {

        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0) {

            ESP_LOGI(TAG,
                "CONN UPDATE interval=%d latency=%d timeout=%d",
                desc.conn_itvl,
                desc.conn_latency,
                desc.supervision_timeout);
        }

        break;
    }
    case BLE_GAP_EVENT_ADV_COMPLETE:
    {
        ESP_LOGI(TAG, "ADV complete, restarting");
        start_advertising();
        break;
    }
    case BLE_GAP_EVENT_PASSKEY_ACTION:
    {
        if (event->passkey.params.action == BLE_SM_IOACT_DISP)
        {
            /* generate and display passkey */
            struct ble_sm_io pkey = {0};
            pkey.action    = BLE_SM_IOACT_DISP;
            pkey.passkey   = 123456;  // hardcode or generate randomly

            ESP_LOGI(TAG, "Passkey: %06lu", pkey.passkey);

            ble_sm_inject_io(event->passkey.conn_handle, &pkey);
        }
        break;
    }
    case BLE_GAP_EVENT_ENC_CHANGE:
    {
        if (event->enc_change.status == 0)
        {
            ESP_LOGI(TAG, "Pairing complete, encryption enabled conn_handle=%d",
                    event->enc_change.conn_handle);
            isPaired = true;
        }
        else
        {
            ESP_LOGW(TAG, "Pairing failed status=%d", event->enc_change.status);
        }
        break;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING:
    {
        /* already bonded, delete old bond and re-pair */
        struct ble_gap_conn_desc desc;
        ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        ble_store_util_delete_peer(&desc.peer_id_addr);
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }
    default:
        break;
    }

    return 0;
}

static int lamp_access_cb(uint16_t conn_handle,
                          uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt,
                          void *arg)
{
    const ble_uuid_t *uuid = ctxt->chr->uuid;

    if (!isPaired)
    {
        ESP_LOGW(TAG, "GATT access rejected, not paired");
        ble_gap_terminate(conn_handle, BLE_ERR_AUTH_FAIL);
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }

    /* ---- BIG LAMP ---- */
    if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(BIG_LAMP_GATT_CHAR)) == 0)
    {
        switch (ctxt->op)
        {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            return os_mbuf_append(ctxt->om, &big_lamp_state, sizeof(big_lamp_state));

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            if (ctxt->om->om_len < 1) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            big_lamp_state = ctxt->om->om_data[0];
            ESP_LOGI("LAMP", "BIG_LAMP WRITE: %d", big_lamp_state);
            App_SetLampState(BigLamp, big_lamp_state);
            ble_gatts_chr_updated(attr_handle);
            return 0;

        default:
            return BLE_ATT_ERR_UNLIKELY;
        }
    }

    /* ---- SMALL LAMP ---- */
    if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(SMALL_LAMP_GATT_CHAR)) == 0)
    {
        switch (ctxt->op)
        {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            return os_mbuf_append(ctxt->om, &small_lamp_state, sizeof(small_lamp_state));

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            if (ctxt->om->om_len < 1) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            small_lamp_state = ctxt->om->om_data[0];
            ESP_LOGI("LAMP", "SMALL_LAMP WRITE: %d", small_lamp_state);
            App_SetLampState(SmallLamp, small_lamp_state);
            ble_gatts_chr_updated(attr_handle);
            return 0;

        default:
            return BLE_ATT_ERR_UNLIKELY;
        }
    }

    /* ---- ENABLE RANGING ---- */
    if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(ENABLE_RANGING_CHAR)) == 0)
    {
        switch (ctxt->op)
        {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            return os_mbuf_append(ctxt->om, &ranging_enabled, sizeof(ranging_enabled));

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            if (ctxt->om->om_len < 1) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            ranging_enabled = ctxt->om->om_data[0];
            ESP_LOGI("LAMP", "ENABLE_RANGING WRITE: %d", ranging_enabled);
            App_EnableRanging(ranging_enabled);
            return 0;

        default:
            return BLE_ATT_ERR_UNLIKELY;
        }
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static void ble_app_on_sync(void)
{
    ESP_LOGI(TAG, "BLE sync complete");

    start_advertising();
}

static void start_advertising(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;

    memset(&fields, 0, sizeof(fields));
    memset(&adv_params, 0, sizeof(adv_params));

    const char *name = "Lampa";

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.appearance = 0x07C0;          // Generic Light Source

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields failed rc=%d", rc);
        return;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(
        BLE_OWN_ADDR_PUBLIC,
        NULL,
        BLE_HS_FOREVER,
        &adv_params,
        gap_event,
        NULL);

    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start failed rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising started");
    }
}

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void Ble_NotifyCallout(struct ble_npl_event *event)
{
    NotifyEvent_ts ne;

    while (xQueueReceive(s_notify_queue, &ne, 0) == pdTRUE)
    {
        if (ne.attr_handle == big_lamp_val_handle)
        {
            big_lamp_state = ne.state;
        }
        else if (ne.attr_handle == small_lamp_val_handle)
        {
            small_lamp_state = ne.state;
        }

        ble_gatts_chr_updated(ne.attr_handle);
    }
}

static void Ble_ScheduleNotify(uint16_t attr_handle, uint8_t state)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE)
    {
        return;
    }

    NotifyEvent_ts ne = { .attr_handle = attr_handle, .state = state };

    if (xQueueSend(s_notify_queue, &ne, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        ESP_LOGW(TAG, "notify queue full");
        return;
    }

    /* trigger callout to drain queue from NimBLE context */
    ble_npl_callout_reset(&s_notify_callout, 0);
}

/* Extern functions implementation */
void Ble_Init()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nimble_port_init());

    s_notify_queue = xQueueCreate(NOTIFY_QUEUE_LENGTH, sizeof(NotifyEvent_ts));

    ble_npl_callout_init(&s_notify_callout,
                         nimble_port_get_dflt_eventq(),
                         Ble_NotifyCallout,
                         NULL);

    ble_hs_cfg.sync_cb = ble_app_on_sync;

    ble_hs_cfg.sm_io_cap         = BLE_HS_IO_DISPLAY_ONLY;  // Just Works
    ble_hs_cfg.sm_bonding        = 1;   // store keys after pairing
    ble_hs_cfg.sm_mitm           = 1;   // no MITM protection needed for Just Works
    ble_hs_cfg.sm_sc             = 1;   // Secure Connections (BLE 4.2+)
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    /* setup GATT */
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    nimble_port_freertos_init(ble_host_task);
}

bool Ble_GetRssi(int8_t* Rssi)
{
    bool retVal = false;
    if(conn_handle != BLE_HS_CONN_HANDLE_NONE)
    {
        if (ble_gap_conn_rssi(conn_handle, Rssi) == 0)
        {
            retVal = true;
        }
    }

    return retVal;
}

void Ble_NotifyBigLamp(uint8_t state)
{
    Ble_ScheduleNotify(big_lamp_val_handle, state);
}

void Ble_NotifySmallLamp(uint8_t state)
{
    Ble_ScheduleNotify(small_lamp_val_handle, state);
}