#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

// std
#include <stdint.h>
#include <stdbool.h>

// freertos
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// esp
#include "esp_check.h"
#include "esp_adc_cal.h"

// components
#include "rotary_encoder.h"
#include "path_interpolation.h"

// app
#include "pid_control.h"


// motor control
#define MOTOR_CONTROL_MODE_DEBUG                0
#define MOTOR_CONTROL_MODE_INIT                 0
#define MOTOR_CONTROL_MODE_POSITION             1
#define MOTOR_CONTROL_MODE_VELOCITY             2
#define MOTOR_CONTROL_MODE_CURRENT              3
#define MOTOR_CONTROL_MODE_SET_ORIGIN           4       // reserve
#define MOTOR_CONTROL_MODE_SENSORLESS           5       // reserve
#define MOTOR_CONTROL_MODE_PWM                  6
#define MOTOR_CONTROL_MODE_CURRENT_OFFSET       20
#define MOTOR_CONTROL_MODE_EMERGENCY            0xFF

#define MOTOR_CONTROL_CURRENT_LIMIT_REF         0.5f
#define MOTOR_CONTROL_CURRENT_LIMIT_MIN         0.2f
#define MOTOR_CONTROL_CURRENT_LIMIT_MAX         1.0f
#define MOTOR_CONTROL_VELOCITY_LIMIT_REF        20.0f

#define MOTOR_CONTROL_WS_REF                    0.05
#define MOTOR_CONTROL_WC_REF                    0.025

#define MOTOR_CONTROL_ERR_STATE_OK              0
#define MOTOR_CONTROL_ERR_STATE_MOTOR_FAIL      1
#define MOTOR_CONTROL_ERR_STATE_ORIGIN_FAIL     2       // reserve
#define MOTOR_CONTROL_ERR_STATE_EMERGENCY       3
#define MOTOR_CONTROL_ERR_STATE_INVALID_VALUE   4
#define MOTOR_CONTROL_ERR_STATE_BUSY            5

#define MOTOR_CONTROL_VELOCITY_CUT_DEFAULT_CUTLINE  10.0f // rad/s
#define MOTOR_CONTROL_VELOCITY_CUT_DEFAULT_GAIN     1

// motor
#define VDC                                     6.0f
#define MOTOR_INDUCTANCE                        0.0015f
#define MOTOR_RESISTANCE                        4.2f
#define PULSE_TO_RADIAN                         (2 * M_PI / 348.0f)     // ratio: 1/29, encoder: 12 [P/R] -> 29 * 12 = 348 [P/R]
#define ADC_VOLTAGE_OFFSET                      1.65f
#define ADC_VOLTAGE_TO_CURRENT                  (1.0f / 0.055f)         // [A/V]

typedef struct motor_pid_gain {
    float p;
    float i;
    float d;
} motor_pid_gain_t;

typedef struct motor_limit_parameters {
    float current;
    float velocity;
    float velocity_gain;
} motor_limit_parameters_t;

typedef struct motor_control {
    uint8_t mode;
    uint8_t err_state;

    float pwm;
    int32_t pulse;

    float current;
    float velocity;
    float position;

    struct {
        uint32_t current;
        uint32_t velocity;
        uint32_t position;
    } frequency;

    struct {
        pid_control_t current;
        pid_control_t velocity;
        pid_control_t position;
    } pid;

    motor_limit_parameters_t limit;

    // path interpolation
    path_interpolation position_path;

    // current calibration
    uint32_t current_calibration_cnt;
    float current_calibration_sum;
    float current_offset;
} motor_control_t;

bool init_motor_control(uint32_t current_frequency, uint32_t velocity_frequency, uint32_t position_frequency);

void run_motor_control();
void motor_control_current();
void motor_control_velocity();
void motor_control_position();
void motor_control_set_emergency_mode();
uint8_t motor_control_clear_emergency_mode();

uint8_t motor_control_stop();
uint8_t motor_control_set_current_target(float target);
uint8_t motor_control_set_velocity_target(float target);
uint8_t motor_control_set_position_target(float target);
uint8_t motor_control_set_pwm_target(float target);
uint8_t motor_control_calibrate_current();

motor_pid_gain_t motor_control_get_current_pid_gain();
void motor_control_set_current_pid_gain(motor_pid_gain_t gain);
motor_pid_gain_t motor_control_get_velocity_pid_gain();
void motor_control_set_velocity_pid_gain(motor_pid_gain_t gain);
motor_pid_gain_t motor_control_get_position_pid_gain();
void motor_control_set_position_pid_gain(motor_pid_gain_t gain);

uint8_t motor_control_get_mode();
uint8_t motor_control_get_err_state();

float get_motor_current();
float get_motor_velocity();
float get_motor_position();

float get_motor_pwm();
int32_t get_motor_pulse();

#ifdef __cplusplus
}
#endif

#endif
