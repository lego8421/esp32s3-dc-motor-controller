#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct pid_control_
{
    float _Kp;
    float _Kd;
    float _Ki;
    float _Ka;
    float _UnderOfpoint;
    float _Target;
    float _ErrSumLimit;
    float _Err;
    float _Err_Temp;
    float _PPost_Err;
    float _Post_Err;
    float _ErrSum;
    float _Input;
    float _Output;
    float _Output_Temp;
    float _Output_Sat;
} pid_control_t;

// 2 * pi * current frequency / 10
#define PID_WC (2 * 3.141592 * 20000 / 10)

// 2 * pi * velocity frequency / 20
#define PID_WS (2 * 3.141592 * 1000 / 20)

float pid_control(pid_control_t* dst , float data_input);
float pid_control_cur(pid_control_t* dst, float data_input);
float pid_control_vel(pid_control_t* dst, float data_input);
float pid_control_pos(pid_control_t* dst, float data_input);
void pid_control_init_current(pid_control_t* dst);
void pid_control_init_velocity(pid_control_t* dst);
void pid_control_init_position(pid_control_t* dst);
void pid_control_reset(pid_control_t* dst);

#ifdef __cplusplus
}

#endif
#endif
