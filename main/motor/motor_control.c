#include "motor_control.h"

// std
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>
#include <math.h>

// freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// esp
#include "esp_system.h"
#include "esp_log.h"
#include "driver/timer.h"
#include "driver/mcpwm.h"
#include "driver/pcnt.h"
#include "driver/adc.h"
#include "driver/gpio.h"

// app
#include "timer_interrupt.h"


static const char* TAG = "motor_control";

motor_control_t motor_control;

// timer event
static QueueHandle_t timer_event_queue = NULL;

// pwm
static const gpio_num_t GPIO_PWM0A_OUT = GPIO_NUM_15;
static const gpio_num_t GPIO_PWM0B_OUT = GPIO_NUM_16;
static const uint32_t MOTOR_CTRL_PWM_FREQUENCY = 20000;
static const mcpwm_unit_t MOTOR_CTRL_MCPWM_UNIT = MCPWM_UNIT_0;
static const mcpwm_timer_t MOTOR_CTRL_MCPWM_TIMER = MCPWM_TIMER_0;
static const float MOTOR_CTRL_MCPWM_MIN_DUTY = 50.0f;
static const float MOTOR_CTRL_MCPWM_MAX_DUTY = 100.0f;

// encoder
static const gpio_num_t GPIO_PCNT_PINA = GPIO_NUM_18;
static const gpio_num_t GPIO_PCNT_PINB = GPIO_NUM_19;
static rotary_encoder_t *encoder_module = NULL;

// adc for current sensor
static esp_adc_cal_characteristics_t adc_chars;

static esp_err_t motor_control_encoder_init()
{
    // encoder
    uint32_t pcnt_unit = 0;
    rotary_encoder_config_t encoder_config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, GPIO_PCNT_PINA, GPIO_PCNT_PINB);

    esp_err_t ret = rotary_encoder_new_ec11(&encoder_config, &encoder_module);
    ESP_RETURN_ON_ERROR(ret, TAG, "rotary_encoder init failed");

    /* Filter out glitch (1us) */
    ret = encoder_module->set_glitch_filter(encoder_module, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "rotary_encoder glitch_filter failed");

    /* Start encoder */
    ret = encoder_module->start(encoder_module);
    ESP_RETURN_ON_ERROR(ret, TAG, "rotary_encoder start failed");

    pcnt_counter_clear((pcnt_unit_t)pcnt_unit);

    return ESP_OK;
}

static esp_err_t motor_control_pwn_init()
{
    mcpwm_gpio_init(MOTOR_CTRL_MCPWM_UNIT, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MOTOR_CTRL_MCPWM_UNIT, MCPWM0B, GPIO_PWM0B_OUT);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = MOTOR_CTRL_PWM_FREQUENCY * 2;
    pwm_config.cmpr_a = 50;
    pwm_config.cmpr_b = 50;
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    return mcpwm_init(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, &pwm_config);
}

static esp_err_t motor_control_adc_init()
{
    esp_err_t ret = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP_FIT);

    ESP_RETURN_ON_ERROR(ret, TAG, "adc init failed");

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc_chars);

    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_11);

    return ESP_OK;
}

static bool motor_timer_callback_function(void)
{
    BaseType_t high_task_awoken = pdFALSE;

    int32_t pulse = encoder_module->get_counter_value(encoder_module);

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_event_queue, &pulse, &high_task_awoken);

    // return whether we need to yield at the end of ISR
    return high_task_awoken == pdTRUE;
}

static esp_err_t motor_control_timer_init(uint32_t frequency)
{
    esp_err_t ret = ESP_OK;

    timer_event_queue = xQueueCreate(10, sizeof(int32_t));

    timer_interrupt_init(motor_timer_callback_function, frequency);

    return ret;
}

static void run_motor()
{
    float pwm = motor_control.pwm;

    if (pwm > 100.0f) {
        pwm = 100.0f;
    } else if (pwm < -100.0f) {
        pwm = -100.0f;
    }

    float duty = fabs(pwm) * (MOTOR_CTRL_MCPWM_MAX_DUTY - MOTOR_CTRL_MCPWM_MIN_DUTY) / 100.0f + MOTOR_CTRL_MCPWM_MIN_DUTY;

    if (duty > MOTOR_CTRL_MCPWM_MAX_DUTY) {
        duty = MOTOR_CTRL_MCPWM_MAX_DUTY;
    } else if (duty < MOTOR_CTRL_MCPWM_MIN_DUTY) {
        duty = MOTOR_CTRL_MCPWM_MIN_DUTY;
    }

    if (pwm >= 0) {
        mcpwm_set_duty(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_A, 100.0f - duty);
        mcpwm_set_duty(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_B, duty);
    } else {
        mcpwm_set_duty(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_A, duty);
        mcpwm_set_duty(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_B, 100.0f - duty);
    }
}

static void calculate_pwm(float v)
{
    float duty = v / VDC * 100.0f;
    if (duty > 100.0f) {
        duty = 100.0f;
    } else if (duty < -100.0f) {
        duty = -100.0f;
    }

    motor_control.pwm = duty;
}

static float get_adc_voltage()
{
    int raw = adc1_get_raw(ADC1_CHANNEL_2);
    float voltage = (float)esp_adc_cal_raw_to_voltage(raw, &adc_chars) / 1000.0f;
    return voltage;
}

static void motor_control_task(void *args)
{
    int32_t velocity_cnt = 0;
    int32_t position_cnt = 0;

    int32_t position_tick = motor_control.frequency.current / motor_control.frequency.position;
    int32_t velocity_tick = motor_control.frequency.current / motor_control.frequency.velocity;

    float position_past = 0.0f;

    while (true) {
        bool is_velocity_frequency = false;
        bool is_position_frequency = false;
        int32_t pulse = 0;

        // wait for timer interrupt
        xQueueReceive(timer_event_queue, &pulse, portMAX_DELAY);

        if (position_cnt % position_tick == 0) {
            position_cnt = 0;
            is_position_frequency = true;
        }
        if (velocity_cnt % velocity_tick == 0) {
            velocity_cnt = 0;
            is_velocity_frequency = true;
        }

        // encoder pulse
        motor_control.pulse = pulse;

        // calculate position
        motor_control.position = motor_control.pulse * PULSE_TO_RADIAN;

        // calculate velocity
        if (is_velocity_frequency) {
            float dt = 1.0f / motor_control.frequency.velocity;
            float diff = motor_control.position - position_past;

            if (diff > M_PI) {
                diff -= 2 * M_PI;
            } else if (diff < -M_PI) {
                diff += 2 * M_PI;
            }
            float radian = diff / dt;
            motor_control.velocity = motor_control.velocity * 0.9f + radian * 0.1f;

            position_past = motor_control.position;
        }

        // calculate current
        float voltage = get_adc_voltage();
        float current = (-(ADC_VOLTAGE_OFFSET - voltage) * ADC_VOLTAGE_TO_CURRENT) - motor_control.current_offset;
        motor_control.current = motor_control.current * 0.9f + current * 0.1f;

        switch (motor_control.mode) {
            case MOTOR_CONTROL_MODE_INIT: {
                calculate_pwm(0);
                run_motor();
            } break;
            case MOTOR_CONTROL_MODE_POSITION: {
                if (is_position_frequency) {
                    motor_control_position();
                }
                motor_control_current();
                run_motor();
            } break;
            case MOTOR_CONTROL_MODE_VELOCITY: {
                if (is_velocity_frequency) {
                    motor_control_velocity();
                }
                motor_control_current();
                run_motor();
            } break;
            case MOTOR_CONTROL_MODE_CURRENT: {
                motor_control_current();
                run_motor();
            } break;
            case MOTOR_CONTROL_MODE_PWM: {
                run_motor();
            } break;
            case MOTOR_CONTROL_MODE_EMERGENCY: {
                calculate_pwm(0);
                run_motor();
            } break;
            case MOTOR_CONTROL_MODE_CURRENT_OFFSET: {
                calculate_pwm(0);
                run_motor();

                float current_cali = -(ADC_VOLTAGE_OFFSET - voltage) * ADC_VOLTAGE_TO_CURRENT;
                motor_control.current_calibration_sum += current_cali;
                motor_control.current_calibration_cnt++;

                if (motor_control.current_calibration_cnt >= 100) {
                    motor_control.current_offset = motor_control.current_calibration_sum / 100.0f;

                    motor_control.current_calibration_sum = 0.0f;
                    motor_control.current_calibration_cnt = 0;
                    motor_control.mode = MOTOR_CONTROL_MODE_INIT;
                }
            } break;
        }

        velocity_cnt++;
        position_cnt++;
    }
}

bool init_motor_control(uint32_t current_frequency, uint32_t velocity_frequency, uint32_t position_frequency)
{
    memset(&motor_control, 0, sizeof(motor_control));

    motor_control.frequency.current = current_frequency;
    motor_control.frequency.velocity = velocity_frequency;
    motor_control.frequency.position = position_frequency;

    // pid
    motor_pid_gain_t current_pid_gain = {
        .p = 10.0f,
        .i = 0.001f,
        .d = 0.0f
    };
    motor_control_set_current_pid_gain(current_pid_gain);

    motor_pid_gain_t position_pid_gain = {
        .p = 6.0f,
        .i = 0.0001f,
        .d = 0.2f
    };
    motor_control_set_position_pid_gain(position_pid_gain);
    motor_control.pid.position._ErrSumLimit = 10.0f;

    // limit
    motor_limit_parameters_t limit = {
        .current = MOTOR_CONTROL_CURRENT_LIMIT_MAX,
        .velocity = MOTOR_CONTROL_VELOCITY_LIMIT_REF,
        .velocity_gain = 1.0,
    };
    motor_control.limit = limit;

    // path
    path_init(&motor_control.position_path, 0.0f);

    // current sensor
    if (motor_control_adc_init() != ESP_OK) {
        motor_control.err_state = MOTOR_CONTROL_ERR_STATE_MOTOR_FAIL;
        return false;
    }

    // motor
    if (motor_control_encoder_init() != ESP_OK || motor_control_pwn_init() != ESP_OK) {
        motor_control.err_state = MOTOR_CONTROL_ERR_STATE_MOTOR_FAIL;
        return false;
    }

    // control timer
    if (motor_control_timer_init(motor_control.frequency.current) != ESP_OK) {
        motor_control.err_state = MOTOR_CONTROL_ERR_STATE_MOTOR_FAIL;
        return false;
    }

    xTaskCreatePinnedToCore(motor_control_task, "motor_control_task", 8192, NULL, 3, NULL, 1);

    motor_control.err_state = MOTOR_CONTROL_ERR_STATE_OK;

    vTaskDelay(pdMS_TO_TICKS(100));

    motor_control_calibrate_current();

    return true;
}

void motor_control_current()
{
    float current = motor_control.current;

    float target_current = motor_control.pid.current._Target;

    // float velocity_cut_max = (motor_control.limit.velocity - motor_control.velocity) * motor_control.limit.velocity_gain;
    // float velocity_cut_min = (-motor_control.limit.velocity - motor_control.velocity) * motor_control.limit.velocity_gain;

    // if (motor_control.pid.current._Target > velocity_cut_max) {
    //     motor_control.pid.current._Target = velocity_cut_max;
    // } else if (motor_control.pid.current._Target < velocity_cut_min) {
    //     motor_control.pid.current._Target = velocity_cut_min;
    // }

    // set target
    if (motor_control.pid.current._Target > motor_control.limit.current) {
        motor_control.pid.current._Target = motor_control.limit.current;
    } else if (motor_control.pid.current._Target < -motor_control.limit.current) {
        motor_control.pid.current._Target = -motor_control.limit.current;
    }

    // pid control for current
    float v_output = pid_control_cur(&motor_control.pid.current, current);
    if (v_output > VDC) {
        v_output = VDC;
    } else if(v_output < -VDC) {
        v_output = -VDC;
    }
    motor_control.pid.current._Output = v_output;
    motor_control.pid.current._Target = target_current;

    calculate_pwm(v_output);
}

void motor_control_velocity()
{
    motor_control.pid.current._Target = pid_control_vel(&motor_control.pid.velocity, motor_control.velocity);
}

void motor_control_position()
{
    path_calcuate(&motor_control.position_path, 1.0f / motor_control.frequency.position);
    motor_control.pid.position._Target = motor_control.position_path.desired_q;
    motor_control.pid.current._Target = pid_control(&motor_control.pid.position, motor_control.position);
}

void motor_control_set_emergency_mode()
{
    if (motor_control.mode != MOTOR_CONTROL_MODE_EMERGENCY) {
        motor_control.mode = MOTOR_CONTROL_MODE_EMERGENCY;
        // set free wheel mode
        calculate_pwm(0);
        run_motor();
    }
}

uint8_t motor_control_clear_emergency_mode()
{
    if (motor_control.err_state == MOTOR_CONTROL_ERR_STATE_EMERGENCY) {
        motor_control.mode = MOTOR_CONTROL_MODE_INIT;

        pid_control_reset(&motor_control.pid.current);
        pid_control_reset(&motor_control.pid.velocity);
        pid_control_reset(&motor_control.pid.position);

        // set free wheel mode
        calculate_pwm(0);
        run_motor();

        motor_control.err_state = MOTOR_CONTROL_ERR_STATE_OK;
    }

    return motor_control.err_state;
}

uint8_t motor_control_stop()
{
    if (motor_control.err_state == MOTOR_CONTROL_ERR_STATE_OK) {
        pid_control_reset(&motor_control.pid.current);
        pid_control_reset(&motor_control.pid.velocity);
        pid_control_reset(&motor_control.pid.position);
        motor_control.mode = MOTOR_CONTROL_MODE_INIT;
    }

    return motor_control.err_state;
}

uint8_t motor_control_set_current_target(float target)
{
    if (motor_control.err_state == MOTOR_CONTROL_ERR_STATE_OK) {
        motor_control.pid.current._Target = target;
        motor_control.mode = MOTOR_CONTROL_MODE_CURRENT;
    }

    return motor_control.err_state;
}

uint8_t motor_control_set_velocity_target(float target)
{
    if (motor_control.err_state == MOTOR_CONTROL_ERR_STATE_OK) {
        if (motor_control.mode == MOTOR_CONTROL_MODE_CURRENT) {
            pid_control_reset(&motor_control.pid.velocity);
        }

        motor_control.pid.velocity._Target = target;
        if (motor_control.pid.velocity._Target > MOTOR_CONTROL_VELOCITY_LIMIT_REF) {
            motor_control.pid.velocity._Target = MOTOR_CONTROL_VELOCITY_LIMIT_REF;
        } else if(motor_control.pid.velocity._Target < -MOTOR_CONTROL_VELOCITY_LIMIT_REF) {
            motor_control.pid.velocity._Target = -MOTOR_CONTROL_VELOCITY_LIMIT_REF;
        }
        motor_control.mode = MOTOR_CONTROL_MODE_VELOCITY;
    }

    return motor_control.err_state;
}

uint8_t motor_control_set_position_target(float target)
{
    if (motor_control.err_state == MOTOR_CONTROL_ERR_STATE_OK) {
        if (motor_control.mode == MOTOR_CONTROL_MODE_CURRENT || motor_control.mode == MOTOR_CONTROL_MODE_VELOCITY) {
            pid_control_reset(&motor_control.pid.position);
            path_init(&motor_control.position_path, motor_control.position);
        }

        path_set_target(&motor_control.position_path, target, MOTOR_CONTROL_VELOCITY_LIMIT_REF * 0.7f);
        motor_control.mode = MOTOR_CONTROL_MODE_POSITION;
    }

    return motor_control.err_state;
}

uint8_t motor_control_set_pwm_target(float target)
{
    if (motor_control.err_state == MOTOR_CONTROL_ERR_STATE_OK) {

        if (target > 100.0f) {
            target = 100.0f;
        } else if (target < -100.0f) {
            target = -100.0f;
        }

        motor_control.pwm = target;
        motor_control.mode = MOTOR_CONTROL_MODE_PWM;
    }

    return motor_control.err_state;
}

uint8_t motor_control_calibrate_current()
{
    if (motor_control.err_state == MOTOR_CONTROL_ERR_STATE_OK) {
        motor_control.current_calibration_cnt = 0;
        motor_control.current_calibration_sum = 0.0f;
        motor_control.mode = MOTOR_CONTROL_MODE_CURRENT_OFFSET;
    }

    return motor_control.err_state;
}

motor_pid_gain_t motor_control_get_current_pid_gain()
{
    motor_pid_gain_t gain = {
        .p = motor_control.pid.current._Kp,
        .i = motor_control.pid.current._Ki,
        .d = motor_control.pid.current._Kd,
    };
    return gain;
}

void motor_control_set_current_pid_gain(motor_pid_gain_t gain)
{
    motor_control.pid.current._Kp = gain.p;
    motor_control.pid.current._Ki = gain.i;
    motor_control.pid.current._Kd = gain.d;
    motor_control.pid.current._Ka = 1.0 / motor_control.pid.current._Kp / motor_control.pid.current._Ki / 100.0;
}

motor_pid_gain_t motor_control_get_velocity_pid_gain()
{
    return (motor_pid_gain_t) {
        .p = motor_control.pid.velocity._Kp,
        .i = motor_control.pid.velocity._Ki,
        .d = motor_control.pid.velocity._Kd,
    };
}

void motor_control_set_velocity_pid_gain(motor_pid_gain_t gain)
{
    motor_control.pid.velocity._Kp = gain.p;
    motor_control.pid.velocity._Ki = gain.i;
    motor_control.pid.velocity._Kd = gain.d;
    motor_control.pid.velocity._Ka = 1.0 / motor_control.pid.velocity._Kp / motor_control.pid.velocity._Ki / 100.0;
}

motor_pid_gain_t motor_control_get_position_pid_gain()
{
    return (motor_pid_gain_t) {
        .p = motor_control.pid.position._Kp,
        .i = motor_control.pid.position._Ki,
        .d = motor_control.pid.position._Kd,
    };
}

void motor_control_set_position_pid_gain(motor_pid_gain_t gain)
{
    motor_control.pid.position._Kp = gain.p;
    motor_control.pid.position._Ki = gain.i;
    motor_control.pid.position._Kd = gain.d;
    motor_control.pid.position._Ka = 0.0f;
}

uint8_t motor_control_get_mode()
{
    return motor_control.mode;
}

uint8_t motor_control_get_err_state()
{
    return motor_control.err_state;
}

float get_motor_current()
{
    return motor_control.current;
}

float get_motor_velocity()
{
    return motor_control.velocity;
}

float get_motor_position()
{
    return motor_control.position;
}

float get_motor_pwm()
{
    return motor_control.pwm;
}

int32_t get_motor_pulse()
{
    return motor_control.pulse;
}

