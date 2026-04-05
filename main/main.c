#include "esp_log.h"
#include "esp_ota_ops.h"
#include "OTA/ota_can_slave.h"
#include "OTA/app_tasks.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "Running partition: %s  (offset 0x%08lx)",
             running->label, (unsigned long)running->address);

    // Confirm this firmware is healthy — cancels automatic rollback to previous partition
    esp_err_t ota_mark_err = esp_ota_mark_app_valid_cancel_rollback();
    if (ota_mark_err != ESP_OK) {
        ESP_LOGE(TAG, "mark_app_valid failed: %s", esp_err_to_name(ota_mark_err));
    } else {
        ESP_LOGI(TAG, "Firmware marked valid, rollback cancelled");
    }

    ESP_LOGI(TAG, "Role: SLAVE  (CAN OTA receiver)");
    ota_can_init();        // install TWAI driver once, shared by all tasks
    app_tasks_start();
    ota_can_slave_start();
}
