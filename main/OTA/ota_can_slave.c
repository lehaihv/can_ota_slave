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

void ota_can_init(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    twai_reconfigure_alerts(TWAI_ALERT_BUS_OFF, NULL);
    ESP_LOGI(TAG, "TWAI started @500kbps");
}

static void send_ack(uint16_t seq, bool ok) {
    twai_message_t msg = { .identifier = CAN_ID_ACK, .data_length_code = 3,
                           .data = { (seq>>8)&0xFF, seq&0xFF, ok?0x00:0xFF } };
    twai_transmit(&msg, pdMS_TO_TICKS(CAN_TX_TIMEOUT_MS));
}

static void send_window_ack(uint16_t start_seq, bool ok) {
    twai_message_t msg = { .identifier = CAN_ID_WINDOW_ACK, .data_length_code = 3,
                           .data = { (start_seq>>8)&0xFF, start_seq&0xFF, ok?0x00:0xFF } };
    twai_transmit(&msg, pdMS_TO_TICKS(CAN_TX_TIMEOUT_MS));
}

static void send_ctrl(uint32_t id) {
    twai_message_t msg = { .identifier = id, .data_length_code = 0 };
    twai_transmit(&msg, pdMS_TO_TICKS(CAN_TX_TIMEOUT_MS));
}

static bool wait_for_trigger(void) {
    ESP_LOGI(TAG, "Waiting for OTA trigger...");
    while (1) {
        twai_message_t msg;
        if (twai_receive(&msg, pdMS_TO_TICKS(50)) != ESP_OK) continue;
        if (msg.identifier != CAN_ID_TRIGGER || msg.data_length_code < 4) continue;
        if (msg.data[0]==OTA_TRIGGER_BYTE0 && msg.data[1]==OTA_TRIGGER_BYTE1 &&
            msg.data[2]==OTA_TRIGGER_BYTE2 && msg.data[3]==OTA_TRIGGER_BYTE3) {
            ESP_LOGI(TAG, "✓ Trigger received");
            vTaskDelay(pdMS_TO_TICKS(50));
            return true;
        }
    }
}

static bool receive_and_apply_ota(void) {
    if (!wait_for_trigger()) return false;
    twai_message_t dummy; while (twai_receive(&dummy, 0) == ESP_OK) {}

    ESP_LOGI(TAG, "Waiting for OTA_START...");
    twai_message_t msg;
    uint32_t total_size = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(10000);
    while (xTaskGetTickCount() < deadline) {
        if (twai_receive(&msg, pdMS_TO_TICKS(1000)) != ESP_OK) continue;
        if (msg.identifier == CAN_ID_OTA_START && msg.data_length_code == 8) {
            total_size = (msg.data[0]<<24)|(msg.data[1]<<16)|(msg.data[2]<<8)|msg.data[3];
            break;
        }
    }
    if (!total_size) { ESP_LOGE(TAG, "✗ OTA_START timeout"); return false; }

    const esp_partition_t *upd = esp_ota_get_next_update_partition(NULL);
    if (!upd || total_size > upd->size) { send_ctrl(CAN_ID_ABORT); return false; }

    esp_ota_handle_t ota_handle;
    if (esp_ota_begin(upd, total_size, &ota_handle) != ESP_OK) {
        send_ctrl(CAN_ID_ABORT); return false;
    }

    send_ack(0xFFFF, true);

    uint32_t received_bytes = 0;
    uint16_t expected_seq = 0;
    int retries = 0;

    while (received_bytes < total_size) {
        if (twai_receive(&msg, pdMS_TO_TICKS(OTA_ACK_TIMEOUT_MS)) != ESP_OK) {
            if (++retries > OTA_MAX_RETRIES) {
                ESP_LOGE(TAG, "✗ Timeout at seq %u", expected_seq);
                send_ctrl(CAN_ID_ABORT); esp_ota_abort(ota_handle); return false;
            }
            send_window_ack(expected_seq, false);
            continue;
        }

        if (msg.identifier == CAN_ID_ABORT) { esp_ota_abort(ota_handle); return false; }
        
        if (msg.identifier == CAN_ID_WINDOW_SYNC) {
            send_window_ack(expected_seq, true);
            continue;
        }

        if (msg.identifier == CAN_ID_OTA_END) {
            ESP_LOGI(TAG, "✓ OTA_END received early");
            break;
        }

        if (msg.identifier != CAN_ID_DATA || msg.data_length_code < CAN_HEADER_BYTES) continue;

        uint16_t seq = ((uint16_t)msg.data[0] << 8) | msg.data[1];
        uint8_t chunk_len = msg.data_length_code - CAN_HEADER_BYTES;
        if (chunk_len == 0 || chunk_len > CAN_MAX_FIRMWARE_BYTES) continue;

        if (seq == expected_seq) {
            if (esp_ota_write(ota_handle, &msg.data[CAN_HEADER_BYTES], chunk_len) != ESP_OK) {
                send_ctrl(CAN_ID_ABORT); esp_ota_abort(ota_handle); return false;
            }
            received_bytes += chunk_len;
            expected_seq++;
            retries = 0;
        } else if (seq < expected_seq) {
            // Duplicate: ignore safely
            continue;
        } else {
            // Out-of-order: NACK window start
            send_window_ack(expected_seq, false);
            continue;
        }

        if (expected_seq % PROGRESS_LOG_INTERVAL == 0) {
            ESP_LOGI(TAG, "Progress: %lu / %lu", (unsigned long)received_bytes, (unsigned long)total_size);
        }
    }

    ESP_LOGI(TAG, "All data received (%lu bytes). Waiting for OTA_END...", (unsigned long)received_bytes);
    
    // Wait for master's OTA_END signal (with 5s timeout to handle network drops/reboots)
    TickType_t end_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
    bool got_end = false;
    while (xTaskGetTickCount() < end_deadline) {
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            if (msg.identifier == CAN_ID_OTA_END) { got_end = true; break; }
            if (msg.identifier == CAN_ID_ABORT)   { esp_ota_abort(ota_handle); return false; }
        }
    }
    if (!got_end) ESP_LOGW(TAG, "OTA_END timeout, finalizing anyway");

    esp_err_t err = esp_ota_end(ota_handle);
    if (err != ESP_OK) { ESP_LOGE(TAG, "✗ esp_ota_end failed: %s", esp_err_to_name(err)); send_ctrl(CAN_ID_OTA_FAIL); return false; }
    
    err = esp_ota_set_boot_partition(upd);
    if (err != ESP_OK) { ESP_LOGE(TAG, "✗ set_boot_partition failed: %s", esp_err_to_name(err)); send_ctrl(CAN_ID_OTA_FAIL); return false; }

    send_ctrl(CAN_ID_OTA_OK);
    ESP_LOGI(TAG, "✓ OTA success! Rebooting in 2 seconds...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return true;
}

static void slave_task(void *arg) {
    while (1) {
        receive_and_apply_ota();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void ota_can_slave_start(void) {
    xTaskCreate(slave_task, "ota_slave", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "OTA slave started");
}