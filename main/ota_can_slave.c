#include "ota_can_slave.h"
#include "ota_can_protocol.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"

static const char *TAG = "OTA_SLAVE";

// ─── TWAI init ────────────────────────────────────────────────────────────────

void ota_can_init(void)
{
    twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN,
                                                               TWAI_MODE_NORMAL);
    twai_timing_config_t  t_cfg = CAN_BITRATE;
    twai_filter_config_t  f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_cfg, &t_cfg, &f_cfg));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI started  TX=%d RX=%d", CAN_TX_PIN, CAN_RX_PIN);
}

// ─── helpers ─────────────────────────────────────────────────────────────────

static void send_ack(uint16_t seq, bool ok)
{
    twai_message_t msg = {
        .identifier     = CAN_ID_ACK,
        .data_length_code = 3,
        .data = {
            (seq >> 8) & 0xFF,
            seq        & 0xFF,
            ok ? 0x00 : 0xFF,
        }
    };
    twai_transmit(&msg, pdMS_TO_TICKS(CAN_TX_TIMEOUT_MS));
}

static void send_ctrl(uint32_t id)
{
    twai_message_t msg = {
        .identifier       = id,
        .data_length_code = 0,
    };
    twai_transmit(&msg, pdMS_TO_TICKS(CAN_TX_TIMEOUT_MS));
}

// ─── trigger detection ────────────────────────────────────────────────────────

static bool wait_for_trigger(void)
{
    ESP_LOGI(TAG, "Waiting for OTA trigger...");
    while (1) {
        twai_message_t msg;
        /* Short timeout so other tasks can transmit between checks */
        esp_err_t ret = twai_receive(&msg, pdMS_TO_TICKS(50));
        if (ret == ESP_ERR_TIMEOUT) continue;
        if (ret != ESP_OK) continue;
        if (msg.identifier != CAN_ID_TRIGGER) continue;
        if (msg.data_length_code < 4) continue;
        if (msg.data[0] == OTA_TRIGGER_BYTE0 &&
            msg.data[1] == OTA_TRIGGER_BYTE1 &&
            msg.data[2] == OTA_TRIGGER_BYTE2 &&
            msg.data[3] == OTA_TRIGGER_BYTE3) {
            ESP_LOGI(TAG, "OTA trigger received!");
            return true;
        }
    }
}

// ─── OTA receive ─────────────────────────────────────────────────────────────

static bool receive_and_apply_ota(void)
{
    /* Wait for OTA_START */
    ESP_LOGI(TAG, "Waiting for OTA_START...");
    twai_message_t msg;
    uint32_t total_size  = 0;
    uint32_t total_frames = 0;

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(10000);
    while (xTaskGetTickCount() < deadline) {
        if (twai_receive(&msg, pdMS_TO_TICKS(1000)) != ESP_OK) continue;
        if (msg.identifier == CAN_ID_OTA_START && msg.data_length_code == 8) {
            total_size   = ((uint32_t)msg.data[0] << 24) | ((uint32_t)msg.data[1] << 16) |
                           ((uint32_t)msg.data[2] <<  8) |  (uint32_t)msg.data[3];
            total_frames = ((uint32_t)msg.data[4] << 24) | ((uint32_t)msg.data[5] << 16) |
                           ((uint32_t)msg.data[6] <<  8) |  (uint32_t)msg.data[7];
            break;
        }
    }
    if (total_size == 0) {
        ESP_LOGE(TAG, "Timeout waiting for OTA_START");
        return false;
    }
    ESP_LOGI(TAG, "OTA_START: %lu bytes, %lu frames",
             (unsigned long)total_size, (unsigned long)total_frames);

    /* Prepare OTA partition */
    const esp_partition_t *update_part = esp_ota_get_next_update_partition(NULL);
    if (!update_part) {
        ESP_LOGE(TAG, "No OTA partition found");
        send_ctrl(CAN_ID_ABORT);
        return false;
    }
    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update_part, total_size, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        send_ctrl(CAN_ID_ABORT);
        return false;
    }

    /* ACK the start */
    send_ack(0xFFFF, true);

    /* Receive data frames */
    uint32_t received_bytes = 0;
    uint16_t expected_seq   = 0;
    int      retries        = 0;

    while (received_bytes < total_size) {
        if (twai_receive(&msg, pdMS_TO_TICKS(OTA_ACK_TIMEOUT_MS)) != ESP_OK) {
            ESP_LOGW(TAG, "Timeout waiting for frame seq=%u", expected_seq);
            if (++retries > OTA_MAX_RETRIES) {
                ESP_LOGE(TAG, "Too many timeouts, aborting");
                send_ctrl(CAN_ID_ABORT);
                esp_ota_abort(ota_handle);
                return false;
            }
            send_ack(expected_seq, false);
            continue;
        }

        /* OTA_END received */
        if (msg.identifier == CAN_ID_OTA_END) {
            ESP_LOGI(TAG, "OTA_END received");
            break;
        }

        /* Abort from master */
        if (msg.identifier == CAN_ID_ABORT) {
            ESP_LOGE(TAG, "Abort received from master");
            esp_ota_abort(ota_handle);
            return false;
        }

        if (msg.identifier != CAN_ID_DATA || msg.data_length_code < 3) continue;

        uint16_t seq       = ((uint16_t)msg.data[0] << 8) | msg.data[1];
        uint8_t  chunk_len = msg.data[2];

        if (chunk_len == 0 || chunk_len > CAN_PAYLOAD_BYTES) {
            ESP_LOGW(TAG, "Bad chunk_len=%u seq=%u", chunk_len, seq);
            send_ack(expected_seq, false);
            continue;
        }

        /* Duplicate: re-ACK without writing */
        if (seq != expected_seq) {
            ESP_LOGW(TAG, "Unexpected seq %u (want %u), re-ACK", seq, expected_seq);
            send_ack(seq, true);
            continue;
        }

        err = esp_ota_write(ota_handle, &msg.data[3], chunk_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            send_ctrl(CAN_ID_ABORT);
            esp_ota_abort(ota_handle);
            return false;
        }

        received_bytes += chunk_len;
        retries = 0;
        send_ack(seq, true);
        expected_seq++;

        if (expected_seq % 200 == 0) {
            ESP_LOGI(TAG, "Progress: %lu / %lu bytes",
                     (unsigned long)received_bytes, (unsigned long)total_size);
        }
    }

    ESP_LOGI(TAG, "All data received (%lu bytes). Finalizing...",
             (unsigned long)received_bytes);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        send_ctrl(CAN_ID_OTA_FAIL);
        return false;
    }
    err = esp_ota_set_boot_partition(update_part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        send_ctrl(CAN_ID_OTA_FAIL);
        return false;
    }

    send_ctrl(CAN_ID_OTA_OK);
    ESP_LOGI(TAG, "OTA success! Rebooting in 2 seconds...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return true;
}

// ─── slave task ───────────────────────────────────────────────────────────────

static void slave_task(void *arg)
{
    while (1) {
        if (wait_for_trigger()) {
            if (!receive_and_apply_ota()) {
                ESP_LOGW(TAG, "OTA failed — continuing with current firmware");
            }
        }
    }
}

void ota_can_slave_start(void)
{
    xTaskCreate(slave_task, "ota_slave", 8192, NULL, 5, NULL);
}
