#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "headfile.h"
#include "pid.h"

#define MOTOR1_A   PWM1_MODULE1_CHB_D15   //定义1电机正转PWM引脚
#define MOTOR1_B   PWM1_MODULE0_CHA_D12   //定义1电机反转PWM引脚
                        
#define MOTOR2_A   PWM2_MODULE3_CHB_D3   //定义2电机正转PWM引脚
#define MOTOR2_B   PWM1_MODULE3_CHB_D1   //定义2电机反转PWM引脚
    
#define MOTOR3_A   PWM1_MODULE0_CHB_D13  //定义3电机正转PWM引脚
#define MOTOR3_B   PWM1_MODULE1_CHA_D14  //定义3电机反转PWM引脚
    
#define MOTOR4_A   PWM2_MODULE3_CHA_D2  //定义4电机正转PWM引脚
#define MOTOR4_B   PWM1_MODULE3_CHA_D0  //定义4电机反转PWM引脚

#define MOTOR_P_MAX 20000
#define MOTOR_I_MAX 10000
#define MOTOR_D_MAX 20000

typedef enum{
  motor_1_ = 0,
  motor_2_,
  motor_3_,
  motor_4_
}motor_label;

typedef struct motor_param_t
{
    float total_encoder;  //编码器总值 total = raw + speed 此值用于记录总行程
    //float target_encoder; //编码器目标值 ？不知有何用处 暂不用
    float encoder_raw;    //编码器原始数据 raw = half of编码器读取值
    float encoder_speed;  //测量速度 speed是raw一阶低通滤波出来的值
    float target_speed;   //目标速度
    rt_int32_t duty; //Motor PWM duty

    pid_param_t pid;       //Motor PID param
    pid_param_t stop_pid; //stop Motor PID param
} motor_param_t;

extern motor_param_t motor_1, motor_2, motor_3, motor_4;

#define MOTOR_CREATE(ts, kp, ki, kd, low_pass, _gama, p_max, i_max, d_max) \
    {                                                                                             \
        .total_encoder = 0,                                                                       \
        .target_speed = ts,                                                                       \
        .pid = PID_CREATE(kp, ki, kd, low_pass, _gama, p_max, i_max, d_max),                             \
    }


void motor_duty(motor_label motor_x_,int32 DUTY);
void motor_init(void);
int64_t get_total_encoder(void);
void total_encoder_clear(void);
void all_wheels_set(float ats);
    
#endif
    