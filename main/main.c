#include "esp_log.h"
#include "esp_ota_ops.h"
#include "driver/twai.h"
#include "OTA/ota_can_slave.h"
#include "OTA/app_tasks.h"

static const char *TAG = "MAIN";

/* Returns true if the CAN bus initialised and is not in a fault state */
static bool diagnostic(void)
{
    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK) {
        ESP_LOGE(TAG, "Diagnostic: failed to read TWAI status");
        return false;
    }
    if (status.state == TWAI_STATE_BUS_OFF) {
        ESP_LOGE(TAG, "Diagnostic: TWAI bus-off detected");
        return false;
    }
    if (status.state == TWAI_STATE_STOPPED) {
        ESP_LOGE(TAG, "Diagnostic: TWAI not started");
        return false;
    }
    ESP_LOGI(TAG, "Diagnostic: TWAI OK (state=%d tx_err=%lu rx_err=%lu)",
             status.state,
             (unsigned long)status.tx_error_counter,
             (unsigned long)status.rx_error_counter);
    return true;
}

void app_main(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "Running partition: %s  (offset 0x%08lx)",
             running->label, (unsigned long)running->address);

    ESP_LOGI(TAG, "Role: SLAVE  (CAN OTA receiver)");
    ota_can_init();        // install TWAI driver once, shared by all tasks

    /* --- OTA rollback diagnostic pattern (ESP-IDF recommended) --- */
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            if (diagnostic()) {
                ESP_LOGI(TAG, "Diagnostics passed — firmware marked valid");
                esp_ota_mark_app_valid_cancel_rollback();
            } else {
                ESP_LOGE(TAG, "Diagnostics failed — rolling back to previous firmware");
                esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
    }

    // /* --- Old pattern (unconditional mark valid — kept for reference) ---
    // esp_err_t ota_mark_err = esp_ota_mark_app_valid_cancel_rollback();
    // if (ota_mark_err != ESP_OK) {
    //     ESP_LOGE(TAG, "mark_app_valid failed: %s", esp_err_to_name(ota_mark_err));
    // } else {
    //     ESP_LOGI(TAG, "Firmware marked valid, rollback cancelled");
    // }
    // */

    app_tasks_start();
    ota_can_slave_start();
}
