// std
#include <stdbool.h>
#include <math.h>

// freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// esp
#include "esp_log.h"

// app
#include "motor_control.h"
#include "at_handlers.h"


static const char *TAG = "app_main";

extern "C" void app_main(void);

static void motor_print_task(void *p)
{
    ESP_LOGI(TAG, "start motor_print_task");

    while (true) {
        float current = get_motor_current();
        float velocity = get_motor_velocity() * 180.0 / M_PI;
        float position = get_motor_position() * 180.0 / M_PI;

        int32_t pulse = get_motor_pulse();
        float pwm = get_motor_pwm();

        // ESP_LOGI(TAG, "c: %.2fA, v: %.1fdeg/s, p: %.1fdeg, pwm: %.1f", current, velocity, position, pwm);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    uint32_t current_frequency = 20000;
    uint32_t velocity_frequency = 1000;
    uint32_t position_frequency = 1000;

    bool init = init_motor_control(current_frequency, velocity_frequency, position_frequency);
    if (!init) {
        ESP_LOGE(TAG, "motor control init fail");
    } else {
        ESP_LOGI(TAG, "at command init");
        xTaskCreatePinnedToCore((TaskFunction_t)at_command_task, "at_command_task", 4 * 1024, NULL, 7, NULL, 0);

        ESP_LOGI(TAG, "motor print init");
        xTaskCreatePinnedToCore(motor_print_task, "motor_print_task", 4096, NULL, 6, NULL, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
