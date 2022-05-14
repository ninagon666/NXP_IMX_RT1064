#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "headfile.h"
#include "pid.h"

#define DIR_1 D1
#define DIR_2 D0
#define DIR_3 D12
#define DIR_4 D13
#define PWM_1 PWM2_MODULE3_CHB_D3
#define PWM_2 PWM2_MODULE3_CHA_D2
#define PWM_3 PWM1_MODULE1_CHB_D15
#define PWM_4 PWM1_MODULE1_CHA_D14

#define MOTOR_P_MAX 20000
#define MOTOR_I_MAX 10000
#define MOTOR_D_MAX 20000

typedef struct motor_param_t
{
    float total_encoder;  //编码器总值 total = raw + speed 此值用于记录总的运行距离 不需要清零
    //float target_encoder; //编码器目标值 ？不知有何用处 暂不用
    float encoder_raw;    //编码器原始数据 raw = half of编码器读取值
    float encoder_speed;  //测量速度 speed是raw一阶低通滤波出来的值
    float target_speed;   //目标速度
    int32_t duty; //Motor PWM duty

    pid_param_t pid;       //Motor PID param
    pid_param_t stop_pid; //stop Motor PID param
} motor_param_t;

extern motor_param_t motor_1, motor_2, motor_3, motor_4;

#define MOTOR_CREATE(ts, kp, ki, kd, low_pass, p_max, i_max, d_max) \
    {                                                                                             \
        .total_encoder = 0,                                                                       \
        .target_speed = ts,                                                                       \
        .pid = PID_CREATE(kp, ki, kd, low_pass, p_max, i_max, d_max),                             \
    }


void motor_duty(PWMCH_enum pwmch, int32 DUTY, PIN_enum PIN);
void motor_init();
int64_t get_total_encoder();
void all_wheels_set(float ats);
    
#endif
    