#include <string.h>

#include "lampControl.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include "host/ble_gatt.h"

#define LAMP_GATT_SERVICE (0x1815)
#define LAMP_GATT_CHAR    (0x2BE2)

static const char *TAG = "BLE";
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint8_t lamp_state = 0u;

static void start_advertising(void);
static int gap_event(struct ble_gap_event *event, void *arg);
static int lamp_access_cb(uint16_t conn_handle,
                          uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt,
                          void *arg);

static const struct ble_gatt_svc_def gatt_svcs[] = {
{
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = BLE_UUID16_DECLARE(LAMP_GATT_SERVICE),   // custom service

    .characteristics = (struct ble_gatt_chr_def[]) { {

        .uuid = BLE_UUID16_DECLARE(LAMP_GATT_CHAR), // LAMP_STATE
        .access_cb = lamp_access_cb,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,

    }, {
        0
    } }
},
{0}
};


static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT: {

        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "CONNECT failed status=%d",
                     event->connect.status);
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            start_advertising();
            return 0;
        }

        conn_handle = event->connect.conn_handle;

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

    case BLE_GAP_EVENT_DISCONNECT: {

        ESP_LOGW(TAG,
            "DISCONNECTED reason=%d conn_handle=%d",
            event->disconnect.reason,
            event->disconnect.conn.conn_handle);

        conn_handle = BLE_HS_CONN_HANDLE_NONE;

        start_advertising();
        break;
    }

    case BLE_GAP_EVENT_CONN_UPDATE: {

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
        ESP_LOGI(TAG, "ADV complete, restarting");
        start_advertising();
        break;

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
    switch (ctxt->op) {

    case BLE_GATT_ACCESS_OP_READ_CHR:
        return os_mbuf_append(ctxt->om,
                              &lamp_state,
                              sizeof(lamp_state));

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (ctxt->om->om_len < 1) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        lamp_state = ctxt->om->om_data[0];

        ESP_LOGI("LAMP", "WRITE received: %d", lamp_state);

        // optional: update hardware here
        // lamp_set(lamp_state);

        // notify all subscribed clients
        ble_gatts_chr_updated(attr_handle);

        return 0;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
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

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.sync_cb = ble_app_on_sync;

    /* setup GATT */
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    nimble_port_freertos_init(ble_host_task);
}