#include "pid.h"

#define USED 1
#define UNUSED 0
#define I_CHANGE USED //是否启用变积分
#define D_AHEAD USED //是否启用微分先行

static float kic = 1.0; //变积分参数

float pid_solve_dah(pid_param_t *pid, float error)
{
  float pid_output;

#if D_AHEAD

  float c1, c2, c3, temp;

  temp = pid->gama * pid->kd + pid->kp;
  
  c3 = pid->kd / temp;
  c2 = (pid->kd + pid->kp) / temp;
  c1 = pid->gama * c3;
  
  pid->out_d = c1 * pid->out_d + c2 * error + c3 * pid->pre_error;

#elif !D_AHEAD

  pid->out_d = (error - pid->out_p) * pid->low_pass + pid->out_d * (1 - pid->low_pass);

#endif

  pid->out_p = error;
  pid->out_i += error;

#if I_CHANGE

#if USED
  if (abs(error) > 75)
    kic = 0.0;
  else if (abs(error) > 60 && abs(error) <= 75)
    kic = 0.3;
  else if (abs(error) > 45 && abs(error) <= 60)
    kic = 0.5;
  else if (abs(error) > 30 && abs(error) <= 45)
    kic = 0.8;
  else
    kic = 1.0;
#elif UNUSED  
  
#endif

#endif

  if (pid->ki != 0)
    pid->out_i = MINMAX(pid->out_i, -pid->i_max / pid->ki, pid->i_max / pid->ki); //积分限幅

#if D_AHEAD

  pid_output = MINMAX(pid->kp * pid->out_p + kic * pid->ki * pid->out_i + pid->out_d, -PWM_DUTY_MAX, PWM_DUTY_MAX); //输出限幅
  pid->pre_error = error;

#elif !D_AHEAD

  pid_output = MINMAX(pid->kp * pid->out_p + kic * pid->ki * pid->out_i + pid->kd * pid->out_d, -PWM_DUTY_MAX, PWM_DUTY_MAX); //输出限幅

#endif

  return pid_output;
}

// 常规PID
float pid_solve_nomal(pid_param_t *pid, float error)
{
  float pid_output;

#if UNUSED

  float c1, c2, c3, temp;

  temp = pid->gama * pid->kd + pid->kp;
  
  c3 = pid->kd / temp;
  c2 = (pid->kd + pid->kp) / temp;
  c1 = pid->gama * c3;
  
  pid->out_d = c1 * pid->out_d + c2 * error + c3 * pid->pre_error;

#elif USED

  pid->out_d = (error - pid->out_p) * pid->low_pass + pid->out_d * (1 - pid->low_pass);

#endif

  pid->out_p = error;
  pid->out_i += error;

#if I_CHANGE

  if (abs(error) > 75)
    kic = 0.0;
  else if (abs(error) > 60 && abs(error) <= 75)
    kic = 0.3;
  else if (abs(error) > 45 && abs(error) <= 60)
    kic = 0.5;
  else if (abs(error) > 30 && abs(error) <= 45)
    kic = 0.8;
  else
    kic = 1.0;

#endif

  if (pid->ki != 0)
    pid->out_i = MINMAX(pid->out_i, -pid->i_max / pid->ki, pid->i_max / pid->ki); //积分限幅

#if UNUSED

  pid_output = MINMAX(pid->kp * pid->out_p + kic * pid->ki * pid->out_i + pid->out_d, -PWM_DUTY_MAX, PWM_DUTY_MAX); //输出限幅
  pid->pre_error = error;

#elif USED

  pid_output = MINMAX(pid->kp * pid->out_p + kic * pid->ki * pid->out_i + pid->kd * pid->out_d, -PWM_DUTY_MAX, PWM_DUTY_MAX); //输出限幅

#endif

  return pid_output;
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

float change_kib = 4;

//变积分PID，e大i小
float changable_pid_solve(pid_param_t *pid, float error) {
    pid->out_d = pid->kd * (error - 2 * pid->pre_error + pid->pre_pre_error);
    pid->out_p = pid->kp * (error - pid->pre_error);
    float ki_index = pid->ki;
    if (error + pid->pre_error > 0) {
        ki_index = (pid->ki) - (pid->ki) / (1 + exp(change_kib - 0.2 * fabs(error)));    //变积分控制
    }

    pid->out_i = ki_index * error;
    pid->pre_pre_error = pid->pre_error;
    pid->pre_error = error;

    return MINMAX(pid->out_p, -pid->p_max, pid->p_max)
         + MINMAX(pid->out_i, -pid->i_max, pid->i_max)
         + MINMAX(pid->out_d, -pid->d_max, pid->d_max);
}
