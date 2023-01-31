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
#include "app_loadcell.h"
#include "app_opticalflow.h"


static const char *TAG = "app_main";

extern "C" void app_main(void);

void app_main(void)
{
    // uint32_t current_frequency = 20000;
    // uint32_t velocity_frequency = 1000;
    // uint32_t position_frequency = 1000;

    // ESP_LOGI(TAG, "motor control init");
    // bool init = init_motor_control(current_frequency, velocity_frequency, position_frequency);
    // if (!init) {
    //     ESP_LOGE(TAG, "motor control init fail");
    // }

    // ESP_LOGI(TAG, "at command init");
    // xTaskCreatePinnedToCore((TaskFunction_t)at_command_task, "at_command_task", 4 * 1024, NULL, 7, NULL, 0);

    // while (true) {
    //     vTaskDelay(pdMS_TO_TICKS(100));
    // }

    // loadcell_init();

    // while (true) {
    //     int32_t loadcell = loadcell_read();

    //     ESP_LOGI(TAG, "loadcell: %d", loadcell);

    //     vTaskDelay(pdMS_TO_TICKS(100));
    // }

    opticalflow_init();

    while (true) {
        int16_t delta_x = 0;
        int16_t delta_y = 0;
        opticalflow_read(&delta_x, &delta_y);

        ESP_LOGI(TAG, "delta_x: %d, delta_y: %d", delta_x, delta_y);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
