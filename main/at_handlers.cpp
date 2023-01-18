#include "at_handlers.h"

// std
#include <string>
#include <cmath>

// freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// esp
#include "esp_system.h"

// components
#include "ei_at_server.h"
#include "ei_classifier_porting.h"

// app
#include "motor_control.h"


static inline bool check_args_num(const int &required, const int &received)
{
    if (received < required) {
        ei_printf("Too few arguments! Required: %d\n", required);
        return false;
    }

    return true;
}

bool read_pwm_handler(void)
{
    float pwm = get_motor_pwm();

    ei_printf("read pwm: %.2f%\n", pwm);

    return true;
}

bool write_pwm_handler(const char **argv, const int argc)
{
    check_args_num(1, argc);

    float target_pwm = atof(argv[0]);

    ei_printf("write pwm: %.2f%\n", target_pwm);
    motor_control_set_pwm_target(target_pwm);

    return true;
}

bool read_current_handler(void)
{
    float current = get_motor_current();

    ei_printf("read current: %.3fA\n", current);

    return true;
}

bool write_current_handler(const char **argv, const int argc)
{
    check_args_num(1, argc);

    float target_current = atof(argv[0]);

    ei_printf("write current: %.3fA\n", target_current);
    motor_control_set_current_target(target_current);

    return true;
}

bool read_velocity_handler(void)
{
    float velocity = get_motor_velocity();

    ei_printf("read velocity: %.3frad/s\n", velocity);

    return true;
}

bool write_velocity_handler(const char **argv, const int argc)
{
    check_args_num(1, argc);

    float target_velocity = atof(argv[0]);

    ei_printf("write velocity: %.3frad/s\n", target_velocity);
    motor_control_set_velocity_target(target_velocity);

    return true;
}

bool read_position_handler(void)
{
    float position = get_motor_position();

    ei_printf("read position: %.3frad\n", position);

    return true;
}

bool write_position_handler(const char **argv, const int argc)
{
    check_args_num(1, argc);

    float target_position = atof(argv[0]);

    ei_printf("write position: %.3frad\n", target_position);
    motor_control_set_position_target(target_position);

    return true;
}

bool read_current_pid_handler(void)
{
    motor_pid_gain_t gain = motor_control_get_current_pid_gain();
    ei_printf("read current pid - p: %.5f, i: %.5f, d: %.5f\n", gain.p, gain.i, gain.d);
    return true;
}

bool write_current_pid_handler(const char **argv, const int argc)
{
    check_args_num(3, argc);

    float p = atof(argv[0]);
    float i = atof(argv[1]);
    float d = atof(argv[2]);

    motor_pid_gain_t gain = {
        .p = p,
        .i = i,
        .d = d
    };

    ei_printf("write current pid - p: %.5f, i: %.5f, d: %.5f\n", p, i, d);
    motor_control_set_current_pid_gain(gain);

    return true;
}

bool read_position_pid_handler(void)
{
    motor_pid_gain_t gain = motor_control_get_position_pid_gain();

    ei_printf("read position pid - p: %.5f, i: %.5f, d: %.5f\n", gain.p, gain.i, gain.d);

    return true;
}

bool write_position_pid_handler(const char **argv, const int argc)
{
    check_args_num(3, argc);

    float p = atof(argv[0]);
    float i = atof(argv[1]);
    float d = atof(argv[2]);

    motor_pid_gain_t gain = {
        .p = p,
        .i = i,
        .d = d
    };

    ei_printf("write position pid - p: %.5f, i: %.5f, d: %.5f\n", p, i, d);
    motor_control_set_position_pid_gain(gain);

    return true;
}

TaskHandle_t print_task_handle = NULL;
static bool print_on = false;

static bool print_motor_parameter(void)
{
    float current = get_motor_current();
    float velocity = get_motor_velocity();
    float position = get_motor_position();

    float pwm = get_motor_pwm();

    ei_printf("current: %.2fA, velocity: %.2frad/s, position: %.3frad, pwm: %.1f%\n", current, velocity, position, pwm);

    return true;
}

static void print_motor_parameter_task(void *args)
{
    while (true) {
        if (print_on) {
            print_motor_parameter();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

bool print_motor_parameter_handler(const char **argv, const int argc)
{
    check_args_num(1, argc);

    std::string on(argv[0]);

    if (on == "ON") {
        print_on = true;
        if (!print_task_handle) {
            xTaskCreatePinnedToCore(print_motor_parameter_task, "print_motor_parameter_task", 4096, NULL, 6, &print_task_handle, 1);
        }
    } else if (on == "OFF") {
        print_on = false;
    } else {
        ei_printf("%s is not supported command\n", on.c_str());
    }

    return true;
}

ATServer *ei_at_init()
{
    ATServer *at;

    at = ATServer::get_instance();

    // register_command(command(AT+command), description, run, read(?), write(=), write_description)
    at->register_command(
        "PWM",
        "Read/Write PWM",
        nullptr,
        read_pwm_handler,
        write_pwm_handler,
        "Target PWM[%]");

    at->register_command(
        "CUR",
        "Read/Write Current",
        nullptr,
        read_current_handler,
        write_current_handler,
        "Target Current[A]");

    at->register_command(
        "VEL",
        "Read/Write Velocity",
        nullptr,
        read_velocity_handler,
        write_velocity_handler,
        "Target Velocity[rad/s]");

    at->register_command(
        "POS",
        "Read/Write Position",
        nullptr,
        read_position_handler,
        write_position_handler,
        "Target Position[rad]");

    at->register_command(
        "PID_C",
        "Read/Write Current PID Gain",
        nullptr,
        read_current_pid_handler,
        write_current_pid_handler,
        "P,I,D");

    at->register_command(
        "PID_P",
        "Read/Write Position PID Gain",
        nullptr,
        read_position_pid_handler,
        write_position_pid_handler,
        "P,I,D");

    at->register_command(
        "PRINT",
        "Print Motor Parameters",
        print_motor_parameter,
        nullptr,
        print_motor_parameter_handler,
        "ON/OFF");

    return at;
}

void at_command_task(void *args)
{
    ATServer *at = ei_at_init();
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    at->print_prompt();

    while (true) {
        /* handle command comming from uart */
        char data = ei_get_serial_byte();

        while (data != 0xFF) {
            at->handle(data);
            data = ei_get_serial_byte();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
