#include "pid.h"

#define I_CHANGE 1//是否启用变积分

static float kic = 1.0;//变积分参数

// 常规PID
float pid_solve(pid_param_t *pid, float error)
{
    pid->out_d = (error - pid->out_p) * pid->low_pass + pid->out_d * (1 - pid->low_pass);
    pid->out_p = error;
    pid->out_i += error;

    #if I_CHANGE
    if(abs(error) > 100)
    {
      kic = 0.0;
    }
    else if(abs(error) > 90 && abs(error) <=100)
    {
      kic = 0.3;
    }
    else if(abs(error) > 75 && abs(error) <=90)
    {
      kic = 0.5;
    }
    else if(abs(error) > 50 && abs(error) <=75)
    {
      kic = 0.8;
    }
    else
    {
      kic = 1.0;
    }
    #endif
  
    if (pid->ki != 0)
        pid->out_i = MINMAX(pid->out_i, -pid->i_max / pid->ki, pid->i_max / pid->ki);//积分限幅

    return MINMAX(pid->kp * pid->out_p + kic * pid->ki * pid->out_i + pid->kd * pid->out_d, -PWM_DUTY_MAX, PWM_DUTY_MAX);
}

// 增量式PID
float increment_pid_solve(pid_param_t *pid, float error)
{
    pid->out_d = MINMAX(pid->kd * (error - 2 * pid->pre_error + pid->pre_pre_error), -pid->d_max, pid->d_max);
    pid->out_p = MINMAX(pid->kp * (error - pid->pre_error), -pid->p_max, pid->p_max);
    pid->out_i = MINMAX(pid->ki * error, -pid->i_max, pid->i_max);

    pid->pre_pre_error = pid->pre_error;
    pid->pre_error = error;

    return pid->out_p + pid->out_i + pid->out_d;
}
