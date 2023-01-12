// std
#include <stdbool.h>

// freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// esp
#include "esp_log.h"

// components
#include "rotary_encoder.h"


static const char *TAG = "app_main";

void app_main(void)
{
    // Rotary encoder underlying device is represented by a PCNT unit in this example
    uint32_t pcnt_unit = 0;

    // Create rotary encoder instance
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, 18, 19);
    rotary_encoder_t *encoder = NULL;
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder->set_glitch_filter(encoder, 1));

    // Start encoder
    ESP_ERROR_CHECK(encoder->start(encoder));

    // Report counter value
    while (true) {
        ESP_LOGI(TAG, "Encoder value: %d", encoder->get_counter_value(encoder));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
