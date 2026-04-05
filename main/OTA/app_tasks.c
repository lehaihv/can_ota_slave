#include "app_tasks.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "APP";

#define PWM_GPIO        GPIO_NUM_4
#define PWM_FREQ_HZ     1000
#define PWM_RESOLUTION  LEDC_TIMER_13_BIT
#define PWM_MAX_DUTY    ((1 << 13) - 1)
#define PWM_CHANNEL     LEDC_CHANNEL_0
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_MODE        LEDC_LOW_SPEED_MODE

static void pwm_task(void *arg)
{
    ledc_timer_config_t timer = {
        .speed_mode      = PWM_MODE,
        .timer_num       = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .speed_mode = PWM_MODE,
        .channel    = PWM_CHANNEL,
        .timer_sel  = PWM_TIMER,
        .gpio_num   = PWM_GPIO,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
    ESP_LOGI(TAG, "PWM started on GPIO%d  %d Hz", PWM_GPIO, PWM_FREQ_HZ);

    int duty = 0, step = 64;
    bool going_up = true;

    while (1) {
        ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
        ledc_update_duty(PWM_MODE, PWM_CHANNEL);
        if (going_up) {
            duty += step;
            if (duty >= PWM_MAX_DUTY) { duty = PWM_MAX_DUTY; going_up = false; }
        } else {
            duty -= step;
            if (duty <= 0) { duty = 0; going_up = true; }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_tasks_start(void)
{
    xTaskCreate(pwm_task, "pwm", 2048, NULL, 3, NULL);
    // Add more tasks here for v2, v3 …
}
