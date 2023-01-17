#include "pid_control.h"

/*
    Current._Kp = Motor Inductance * Wc * line to line const;
    Current._Ki = Motor Resistance * Wc * line to line const * 1/control frequency;
    Velocity._Kp = Motor Inertia * Ws / Torque const;
    Velocity._Ki = Velocity._Kp * Ws / line to line const * 1/control frequency;
*/
void pid_control_init_position(pid_control_t* dst)
{
    dst->_Err = 0.0f;
    dst->_ErrSum = 0.0f;
    dst->_Input = 0.0f;
    dst->_Kp = 700.0f;
    dst->_Ki = 0.0f;
    dst->_Kd = 200.0f;
    dst->_Ka = 0.0f;
    dst->_Output = 0.0f;
    dst->_Output_Temp = 0.0f;
    dst->_Target = 0.0f;
    dst->_Post_Err = 0.0f;
    dst->_PPost_Err = 0.0f;
    dst->_ErrSumLimit = 0.0f;
    dst->_Output_Sat = 0.0f;
}

void pid_control_init_current(pid_control_t* dst)
{
    dst->_Err = 0.0f;
    dst->_ErrSum = 0.0f;
    dst->_Input = 0.0f;
    dst->_Kp = 0.00046f * PID_WC * 0.5f;
    dst->_Ki = 0.85f * PID_WC * 0.5f * 0.00002f;
    dst->_Kd = 0.0f;
    dst->_Ka = 1.0f / dst->_Kp / dst->_Ki / 100.0f;
    dst->_Output = 0.0f;
    dst->_Output_Temp = 0.0f;
    dst->_Target = 0.0f;
    dst->_UnderOfpoint = 1.0f;
    dst->_Post_Err = 0.0f;
    dst->_ErrSumLimit = 0.0f;
}

void pid_control_init_velocity(pid_control_t* dst)
{
    dst->_Err = 0.0f;
    dst->_ErrSum = 0.0f;
    dst->_Input = 0.0f;
    dst->_Kp = 0.000016f * PID_WS / 0.08f;
    dst->_Ki = dst->_Kp * PID_WS / 5.0f * 0.00002f;
    dst->_Kd = 0.0f;
    dst->_Ka = 1.0f / dst->_Kp / dst->_Ki / 100.0f;
    dst->_Output = 0.0f;
    dst->_Output_Temp = 0.0f;
    dst->_Target = 0.0f;
    dst->_Post_Err = 0.0f;
    dst->_PPost_Err = 0.0f;
    dst->_ErrSumLimit = 0.0f;
    dst->_Output_Sat = 3.0f;
}

float pid_control_cur(pid_control_t* dst, float DataInput)
{
    dst->_Input = DataInput;
    dst->_Err = dst->_Target - dst->_Input;

    dst->_Output += dst->_Kp * ( dst->_Err - dst->_Post_Err ) + dst->_Ki * dst->_Err;
    dst->_Post_Err = dst->_Err;

    return dst->_Output;
}

float pid_control_vel(pid_control_t* dst, float DataInput)
{
    dst->_Input = DataInput;
    dst->_Err = dst->_Target - dst->_Input;

    dst->_Output += -dst->_Kp * ( dst->_Input - dst->_Post_Err ) + dst->_Ki * dst->_Err;
    dst->_Post_Err = dst->_Input;

    return dst->_Output;
}

float pid_control_pos(pid_control_t* dst, float DataInput)
{
    dst->_Input = DataInput;
    dst->_Err = dst->_Target - dst->_Input;

    dst->_Output = (dst->_Kp * ( dst->_Err )) + (dst->_Kd * ( dst->_Err - dst->_Post_Err ) );
    dst->_Post_Err = dst->_Err;

    return dst->_Output;
}

float pid_control(pid_control_t* dst , float DataInput )
{
    dst->_Input = DataInput;

    dst->_Err = dst->_Target - dst->_Input;
    dst->_ErrSum += dst->_Err;

    if (dst->_ErrSum > dst->_ErrSumLimit) dst->_ErrSum = dst->_ErrSumLimit;
    else if(dst->_ErrSum < -dst->_ErrSumLimit) dst->_ErrSum = -dst->_ErrSumLimit;

    dst->_Output = (dst->_Kp * ( dst->_Err ))
                   + (dst->_Kd * ( dst->_Err - dst->_Post_Err ) )
                   + (dst->_Ki * ( dst->_ErrSum ));

    dst->_Post_Err = dst->_Err;

    return dst->_Output;
}

void pid_control_reset(pid_control_t* dst)
{
    dst->_Target        = 0;
    dst->_Err           = 0;
    dst->_Err_Temp      = 0;
    dst->_PPost_Err     = 0;
    dst->_Post_Err      = 0;
    dst->_ErrSum        = 0;
    dst->_Input         = 0;
    dst->_Output        = 0;
    dst->_Output_Temp   = 0;
}
