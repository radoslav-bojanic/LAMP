#include <stdio.h>
#include "ble.h"
#include "ble_cfg.h"
#include "app.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "store/config/ble_store_config.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include "host/ble_gatt.h"
#include "nvs_flash.h"

#define BLE_MAX_CONNECTIONS     CONFIG_BT_NIMBLE_MAX_CONNECTIONS
#define CONNECTION_ID_INVALID   (0xFFu)

#define TAG "BLE app"
#define NOTIFY_QUEUE_LENGTH (5u)

typedef struct
{
    uint16_t attr_handle;
    uint8_t  state;
} NotifyEvent_ts;

typedef struct
{
    uint8_t  connectionId;
    uint16_t connectionHandle;
    bool     isPaired;
} ConnectionInfo_ts;

/* Static variables */
static uint8_t big_lamp_state   = LAMP_OFF;
static uint8_t small_lamp_state = LAMP_OFF;
static uint8_t ranging_enabled  = NO_RANGING;

static QueueHandle_t s_notify_queue = NULL;
static struct ble_npl_callout s_notify_callout;

static uint16_t big_lamp_val_handle;
static uint16_t small_lamp_val_handle;
static uint16_t ranging_val_handle;
static uint16_t device_model_val_handle;

static ConnectionInfo_ts ConnectionInfo_s[BLE_MAX_CONNECTIONS];

/* ---------- connection table helpers ---------- */

static ConnectionInfo_ts* Conn_Find(uint16_t handle)
{
    for (uint8_t i = 0u; i < BLE_MAX_CONNECTIONS; i++)
    {
        if (ConnectionInfo_s[i].connectionHandle == handle)
        {
            return &ConnectionInfo_s[i];
        }
    }
    return NULL;
}

static ConnectionInfo_ts* Conn_Alloc(uint16_t handle)
{
    for (uint8_t i = 0u; i < BLE_MAX_CONNECTIONS; i++)
    {
        if (ConnectionInfo_s[i].connectionId == CONNECTION_ID_INVALID)
        {
            ConnectionInfo_s[i].connectionId     = i;
            ConnectionInfo_s[i].connectionHandle = handle;
            ConnectionInfo_s[i].isPaired         = false;
            return &ConnectionInfo_s[i];
        }
    }
    return NULL;  /* table full */
}

static void Conn_Free(uint16_t handle)
{
    ConnectionInfo_ts *conn = Conn_Find(handle);
    if (conn != NULL)
    {
        conn->connectionId     = CONNECTION_ID_INVALID;
        conn->connectionHandle = BLE_HS_CONN_HANDLE_NONE;
        conn->isPaired         = false;
    }
}

static bool Conn_AnyActive(void)
{
    for (uint8_t i = 0u; i < BLE_MAX_CONNECTIONS; i++)
    {
        if (ConnectionInfo_s[i].connectionId != CONNECTION_ID_INVALID)
        {
            return true;
        }
    }
    return false;
}

/* ---------- forward declarations ---------- */

static void start_advertising(void);
static int  gap_event(struct ble_gap_event *event, void *arg);
static int  lamp_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);
static int  device_model_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg);
static void ble_app_on_sync(void);
static void Ble_NotifyCallout(struct ble_npl_event *event);
static void Ble_ScheduleNotify(uint16_t attr_handle, uint8_t state);

/* ---------- GATT table ---------- */

static const struct ble_gatt_svc_def gatt_svcs[] = {
{
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = BLE_UUID16_DECLARE(LAMP_GATT_SERVICE),
    .characteristics = (struct ble_gatt_chr_def[]) { {
        .uuid       = BLE_UUID16_DECLARE(BIG_LAMP_GATT_CHAR),
        .access_cb  = lamp_access_cb,
        .val_handle = &big_lamp_val_handle,
        .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
    }, {
        .uuid       = BLE_UUID16_DECLARE(SMALL_LAMP_GATT_CHAR),
        .access_cb  = lamp_access_cb,
        .val_handle = &small_lamp_val_handle,
        .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
    }, {
        .uuid      = BLE_UUID16_DECLARE(ENABLE_RANGING_CHAR),
        .access_cb = lamp_access_cb,
        .val_handle = &ranging_val_handle,
        .flags     = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
    }, {
        0
    } }
},
{
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = BLE_UUID16_DECLARE(DEVICE_MODEL_SERVICE),
    .characteristics = (struct ble_gatt_chr_def[]) { {
        .uuid       = BLE_UUID16_DECLARE(DEVICE_MODEL_CHAR),
        .access_cb  = device_model_access_cb,
        .val_handle = &device_model_val_handle,
        .flags      = BLE_GATT_CHR_F_READ,
    }, {
        0
    } }
},
{0}
};

/* ---------- GAP event ---------- */

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
    {
        if (event->connect.status != 0)
        {
            ESP_LOGW(TAG, "CONNECT failed status=%d", event->connect.status);
            start_advertising();
            return 0;
        }

        uint16_t handle = event->connect.conn_handle;
        ConnectionInfo_ts *conn = Conn_Alloc(handle);

        if (conn == NULL)
        {
            ESP_LOGW(TAG, "Connection table full, terminating conn_handle=%d", handle);
            ble_gap_terminate(handle, BLE_ERR_CONN_LIMIT);
            return 0;
        }

        ESP_LOGI(TAG, "CONNECTED conn_handle=%d id=%d", handle, conn->connectionId);

        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(handle, &desc) == 0)
        {
            ESP_LOGI(TAG, "peer addr=%02X:%02X:%02X:%02X:%02X:%02X",
                desc.peer_id_addr.val[5], desc.peer_id_addr.val[4],
                desc.peer_id_addr.val[3], desc.peer_id_addr.val[2],
                desc.peer_id_addr.val[1], desc.peer_id_addr.val[0]);

            ESP_LOGI(TAG, "interval=%d latency=%d timeout=%d",
                desc.conn_itvl, desc.conn_latency, desc.supervision_timeout);
        }

        /* keep advertising so other clients can connect */
        start_advertising();
        break;
    }

    case BLE_GAP_EVENT_DISCONNECT:
    {
        uint16_t handle = event->disconnect.conn.conn_handle;

        ESP_LOGW(TAG, "DISCONNECTED reason=%d conn_handle=%d",
            event->disconnect.reason, handle);

        Conn_Free(handle);
        if(!Conn_AnyActive())
        {
            ranging_enabled = NO_RANGING;
            App_EnableRanging(ranging_enabled);
            Ble_ScheduleNotify(ranging_val_handle, ranging_enabled);
        }
        start_advertising();
        break;
    }

    case BLE_GAP_EVENT_CONN_UPDATE:
    {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0)
        {
            ESP_LOGI(TAG, "CONN UPDATE interval=%d latency=%d timeout=%d",
                desc.conn_itvl, desc.conn_latency, desc.supervision_timeout);
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
            struct ble_sm_io pkey = {0};
            pkey.action  = BLE_SM_IOACT_DISP;
            pkey.passkey = 123456;

            ESP_LOGI(TAG, "Passkey: %06lu", pkey.passkey);
            ble_sm_inject_io(event->passkey.conn_handle, &pkey);
        }
        break;
    }

    case BLE_GAP_EVENT_ENC_CHANGE:
    {
        if (event->enc_change.status == 0)
        {
            uint16_t handle = event->enc_change.conn_handle;
            ConnectionInfo_ts *conn = Conn_Find(handle);

            if (conn != NULL)
            {
                conn->isPaired = true;
                ESP_LOGI(TAG, "Pairing complete conn_handle=%d id=%d",
                    handle, conn->connectionId);
            }
        }
        else
        {
            ESP_LOGW(TAG, "Pairing failed status=%d", event->enc_change.status);
        }
        break;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING:
    {
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

/* ---------- GATT callbacks ---------- */

static int lamp_access_cb(uint16_t conn_handle,
                          uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt,
                          void *arg)
{
    if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
    {
        ConnectionInfo_ts *conn = Conn_Find(conn_handle);

        if (conn == NULL || !conn->isPaired)
        {
            ESP_LOGW(TAG, "GATT access rejected, not paired conn_handle=%d", conn_handle);
            ble_gap_terminate(conn_handle, BLE_ERR_AUTH_FAIL);
            return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
        }
    }

    const ble_uuid_t *uuid = ctxt->chr->uuid;

    /* ---- BIG LAMP ---- */
    if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(BIG_LAMP_GATT_CHAR)) == 0)
    {
        switch (ctxt->op)
        {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            return os_mbuf_append(ctxt->om, &big_lamp_state, sizeof(big_lamp_state));

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            if (ctxt->om->om_len < 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            big_lamp_state = ctxt->om->om_data[0];
            ESP_LOGI("LAMP", "BIG_LAMP WRITE: %d", big_lamp_state);
            ranging_enabled &= ~ENABLE_RANGING_BIG_LAMP_BIT;
            App_SetLampState(BigLamp, big_lamp_state);
            App_EnableRanging(ranging_enabled);
            Ble_ScheduleNotify(big_lamp_val_handle, big_lamp_state);
            Ble_ScheduleNotify(ranging_val_handle, ranging_enabled);
            return 0;

        default: return BLE_ATT_ERR_UNLIKELY;
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
            if (ctxt->om->om_len < 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            small_lamp_state = ctxt->om->om_data[0];
            ESP_LOGI("LAMP", "SMALL_LAMP WRITE: %d", small_lamp_state);
            ranging_enabled &= ~ENABLE_RANGING_SMALL_LAMP_BIT;
            App_EnableRanging(ranging_enabled);
            App_SetLampState(SmallLamp, small_lamp_state);
            Ble_ScheduleNotify(small_lamp_val_handle, small_lamp_state);
            Ble_ScheduleNotify(ranging_val_handle, ranging_enabled);
            return 0;

        default: return BLE_ATT_ERR_UNLIKELY;
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
            if (ctxt->om->om_len < 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            ranging_enabled = ctxt->om->om_data[0];
            ESP_LOGI("LAMP", "ENABLE_RANGING WRITE: %d", ranging_enabled);
            Ble_ScheduleNotify(ranging_val_handle, ranging_enabled);
            App_EnableRanging(ranging_enabled);
            return 0;

        default: return BLE_ATT_ERR_UNLIKELY;
        }
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static int device_model_access_cb(uint16_t conn_handle,
                                   uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt,
                                   void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    static const char model[] = DEVICE_MODEL_JSON;
    return os_mbuf_append(ctxt->om, model, sizeof(model) - 1);
}

/* ---------- advertising ---------- */

static void ble_app_on_sync(void)
{
    ESP_LOGI(TAG, "BLE sync complete");
    start_advertising();
}

static void start_advertising(void)
{
    /* NimBLE stops advertising on connect automatically,
       so always try to restart; ignore rc if already advertising */
    if (ble_gap_adv_active())
    {
        return;
    }

    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;

    memset(&fields,     0, sizeof(fields));
    memset(&adv_params, 0, sizeof(adv_params));

    const char *name = "Lampa";

    fields.flags            = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name             = (uint8_t *)name;
    fields.name_len         = strlen(name);
    fields.name_is_complete = 1;
    fields.appearance       = 0x07C0;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "adv_set_fields failed rc=%d", rc);
        return;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event, NULL);

    if (rc != 0)
    {
        ESP_LOGE(TAG, "adv_start failed rc=%d", rc);
    }
    else
    {
        ESP_LOGI(TAG, "Advertising started");
    }
}

/* ---------- notify ---------- */

static void Ble_NotifyCallout(struct ble_npl_event *event)
{
    NotifyEvent_ts ne;

    while (xQueueReceive(s_notify_queue, &ne, 0) == pdTRUE)
    {
        /* update local state mirror */
        if (ne.attr_handle == big_lamp_val_handle)
        {
            big_lamp_state = ne.state;
        }
        else if (ne.attr_handle == small_lamp_val_handle)
        {
            small_lamp_state = ne.state;
        }

        /* notify all paired connections */
        for (uint8_t i = 0u; i < BLE_MAX_CONNECTIONS; i++)
        {
            if (ConnectionInfo_s[i].connectionId != CONNECTION_ID_INVALID &&
                ConnectionInfo_s[i].isPaired)
            {
                ble_gatts_notify(ConnectionInfo_s[i].connectionHandle, ne.attr_handle);
            }
        }
    }
}

static void Ble_ScheduleNotify(uint16_t attr_handle, uint8_t state)
{
    if (!Conn_AnyActive())
    {
        return;
    }

    NotifyEvent_ts ne = { .attr_handle = attr_handle, .state = state };

    if (xQueueSend(s_notify_queue, &ne, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        ESP_LOGW(TAG, "notify queue full");
        return;
    }

    ble_npl_callout_reset(&s_notify_callout, 0);
}

/* ---------- public API ---------- */

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void Ble_Init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nimble_port_init());

    for (uint8_t i = 0u; i < BLE_MAX_CONNECTIONS; i++)
    {
        ConnectionInfo_s[i].connectionId     = CONNECTION_ID_INVALID;
        ConnectionInfo_s[i].connectionHandle = BLE_HS_CONN_HANDLE_NONE;
        ConnectionInfo_s[i].isPaired         = false;
    }

    s_notify_queue = xQueueCreate(NOTIFY_QUEUE_LENGTH, sizeof(NotifyEvent_ts));

    ble_npl_callout_init(&s_notify_callout,
                         nimble_port_get_dflt_eventq(),
                         Ble_NotifyCallout,
                         NULL);

    ble_hs_cfg.sync_cb           = ble_app_on_sync;
    ble_hs_cfg.sm_io_cap         = BLE_HS_IO_DISPLAY_ONLY;
    ble_hs_cfg.sm_bonding        = 1;
    ble_hs_cfg.sm_mitm           = 1;
    ble_hs_cfg.sm_sc             = 1;
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.store_status_cb   = ble_store_util_status_rr;

    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    nimble_port_freertos_init(ble_host_task);
}

bool Ble_GetRssi(int8_t *Rssi)
{
    int8_t best = INT8_MIN;
    bool found = false;

    for (uint8_t i = 0u; i < BLE_MAX_CONNECTIONS; i++)
    {
        if (ConnectionInfo_s[i].connectionId == CONNECTION_ID_INVALID) continue;

        int8_t rssi = 0;
        if (ble_gap_conn_rssi(ConnectionInfo_s[i].connectionHandle, &rssi) == 0)
        {
            if (rssi > best)
            {
                best = rssi;
                found = true;
            }
        }
    }

    if (found) *Rssi = best;
    if(!found) ESP_LOGI(TAG, "RSSI read failed conn_handle=%d", 
              ConnectionInfo_s[0].connectionHandle);
    return found;
}

void Ble_NotifyBigLamp(uint8_t state)
{
    Ble_ScheduleNotify(big_lamp_val_handle, state);
}

void Ble_NotifySmallLamp(uint8_t state)
{
    Ble_ScheduleNotify(small_lamp_val_handle, state);
}