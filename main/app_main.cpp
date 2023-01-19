// std
#include <stdbool.h>
#include <math.h>
#include <string.h>

// freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// esp
#include "esp_log.h"

#include "lqr_steer_control.h"

// app
#include "define.h"
#include "motor_control.h"
#include "at_handlers.h"


static const char *TAG = "app_main";

extern "C" void app_main(void);


enum GOLFCAR_COMMAND
{
    COMMAND_NONE = 0,
    COMMAND_SETTING_PATH = 1,
    COMMAND_RENEW_GOLFCAR_STATE = 2,
};

enum GOLFCAR_STATE
{
    STATE_INIT = 0,
    STATE_PATH_TRACKING = 1,
};

static int command;
static int cycle_state;

static void cycle_update_task(void *arg) {
    static bool first_call = true;

    double dt = 0.01;

    std::vector<WayPoint> waypoints;
    lqr_steer_control golfcar_control;
    ControlState current_state;

    memset((void*)&current_state, 0, sizeof(current_state));

    while (true) {
        gpio_set_level(DEBUG_PIN_0, 1);
        switch (command) {
            case COMMAND_NONE:

            break;
            case COMMAND_SETTING_PATH:
                ESP_LOGI(TAG, "path planning start");
                waypoints.push_back(WayPoint(0.00, 0.00));
                waypoints.push_back(WayPoint(1.70, 2.70));
                waypoints.push_back(WayPoint(4.60, -0.20));
                waypoints.push_back(WayPoint(8.40, 2.70));
                waypoints.push_back(WayPoint(11.20, 5.30));
                waypoints.push_back(WayPoint(11.00, 8.10));
                waypoints.push_back(WayPoint(12.10, 10.20));
                waypoints.push_back(WayPoint(9.40, 12.80));
                waypoints.push_back(WayPoint(7.40, 11.30));
                waypoints.push_back(WayPoint(3.90, 14.10));
                waypoints.push_back(WayPoint(2.00, 12.10));
                waypoints.push_back(WayPoint(-0.50, 8.80));
                waypoints.push_back(WayPoint(-3.40, 7.50));
                waypoints.push_back(WayPoint(-6.50, 7.10));
                waypoints.push_back(WayPoint(-9.20, 3.80));
                waypoints.push_back(WayPoint(-9.70, 1.80));
                waypoints.push_back(WayPoint(-10.10, -0.70));
                waypoints.push_back(WayPoint(-11.50, -3.10));
                waypoints.push_back(WayPoint(-8.80, -4.30));
                waypoints.push_back(WayPoint(-6.30, -5.70));
                waypoints.push_back(WayPoint(-3.70, -6.80));
                waypoints.push_back(WayPoint(-0.80, -6.00));
                waypoints.push_back(WayPoint(1.60, -4.30));
                waypoints.push_back(WayPoint(0.00, 0.00));
                ESP_LOGI(TAG, "way point setup finish");
                golfcar_control.generate_spline(current_state, waypoints, 10.0 / 3.6, 1);
                ESP_LOGI(TAG, "path planning finish");
                command = COMMAND_NONE;
                cycle_state = STATE_PATH_TRACKING;
            break;
            case COMMAND_RENEW_GOLFCAR_STATE: {
                // GPS 값 또는 기타 등으로 포지션 수정
                golfcar_control.set_state(current_state);
                command = COMMAND_NONE;
            } break;
        }

        switch (cycle_state) {
            case STATE_INIT:

            break;
            case STATE_PATH_TRACKING: {
                if (golfcar_control.update(dt)) {
                    ESP_LOGI(TAG,"finish");
                    cycle_state = STATE_INIT;
                } else {
                    current_state = golfcar_control.get_state();
                    ESP_LOGI(TAG,"current state = (x = %lf), (y = %lf), (yaw = %lf), (v = %lf)",
                                current_state.x, current_state.y, current_state.yaw, current_state.v);
                    // motor_control_set_position_target(current_state.yaw);
                }
            } break;
        }
        gpio_set_level(DEBUG_PIN_0, 0);
        vTaskDelay(pdMS_TO_TICKS((int)(dt * 1000)));
    }
}

void app_main(void)
{
    gpio_config_t gpio_conf;
    memset(&gpio_conf, 0, sizeof(gpio_config_t));
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = (1ULL << DEBUG_PIN_0) | (1ULL << DEBUG_PIN_1) | (1ULL << DEBUG_PIN_2);
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    uint32_t current_frequency = 20000;
    uint32_t velocity_frequency = 1000;
    uint32_t position_frequency = 1000;

    ESP_LOGI(TAG, "motor control init");
    bool init = init_motor_control(current_frequency, velocity_frequency, position_frequency);
    if (!init) {
        ESP_LOGE(TAG, "motor control init fail");
    }

    ESP_LOGI(TAG, "at command init");
    xTaskCreatePinnedToCore((TaskFunction_t)at_command_task, "at_command_task", 4 * 1024, NULL, 7, NULL, 0);

    xTaskCreatePinnedToCore(cycle_update_task, "cycle", 4096, NULL, 3, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(5000));
    command = COMMAND_SETTING_PATH;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
