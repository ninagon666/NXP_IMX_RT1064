#include "motor.h"
//
//motor_param_t motor_1 = MOTOR_CREATE(0, 1100, 18, 5000, 1, 0.5, MOTOR_P_MAX, MOTOR_I_MAX, MOTOR_D_MAX);
//motor_param_t motor_2 = MOTOR_CREATE(0, 760, 18, 6600, 1, 0.5, MOTOR_P_MAX, MOTOR_I_MAX, MOTOR_D_MAX);
//motor_param_t motor_3 = MOTOR_CREATE(0, 850, 18, 7500, 1, 0.5, MOTOR_P_MAX, MOTOR_I_MAX, MOTOR_D_MAX);
//motor_param_t motor_4 = MOTOR_CREATE(0, 900, 18, 5000, 1, 0.5, MOTOR_P_MAX, MOTOR_I_MAX, MOTOR_D_MAX);
motor_param_t motor_1 = MOTOR_CREATE(0, 650,  8, 30000, 1, 0.5, MOTOR_P_MAX, MOTOR_I_MAX, MOTOR_D_MAX);
motor_param_t motor_2 = MOTOR_CREATE(0, 1100, 7, 30000, 1, 0.5, MOTOR_P_MAX, MOTOR_I_MAX, MOTOR_D_MAX);
motor_param_t motor_3 = MOTOR_CREATE(0, 600,  8, 30000, 1, 0.5, MOTOR_P_MAX, MOTOR_I_MAX, MOTOR_D_MAX);
motor_param_t motor_4 = MOTOR_CREATE(0, 960,  8, 30000, 1, 0.5, MOTOR_P_MAX, MOTOR_I_MAX, MOTOR_D_MAX);

void all_wheels_set(float ats)
{
    motor_1.target_speed = ats;
    motor_2.target_speed = ats;
    motor_3.target_speed = ats;
    motor_4.target_speed = ats;
}

void motor_duty(motor_label motor_x_, int32 DUTY)
{
    if (motor_x_ == motor_1_)
    {
        if (0 <= DUTY) //电机1   正转 设置占空比为 百分之 (1000/TIMER1_PWM_DUTY_MAX*100)
        {
            pwm_duty(MOTOR1_A, DUTY);
            pwm_duty(MOTOR1_B, 0);
        }
        else //电机1   反转
        {
            pwm_duty(MOTOR1_A, 0);
            pwm_duty(MOTOR1_B, -DUTY);
        }
    }
    else if (motor_x_ == motor_2_)
    {
        if (0 <= DUTY) //电机2   正转
        {
            pwm_duty(MOTOR2_A, DUTY);
            pwm_duty(MOTOR2_B, 0);
        }
        else //电机2   反转
        {
            pwm_duty(MOTOR2_A, 0);
            pwm_duty(MOTOR2_B, -DUTY);
        }
    }
    else if (motor_x_ == motor_3_)
    {
        if (0 <= DUTY) //电机3   正转
        {
            pwm_duty(MOTOR3_A, DUTY);
            pwm_duty(MOTOR3_B, 0);
        }
        else //电机3   反转
        {
            pwm_duty(MOTOR3_A, 0);
            pwm_duty(MOTOR3_B, -DUTY);
        }
    }
    else if (motor_x_ == motor_4_)
    {
        if (0 <= DUTY) //电机4   正转
        {
            pwm_duty(MOTOR4_A, DUTY);
            pwm_duty(MOTOR4_B, 0);
        }
        else //电机4   反转
        {
            pwm_duty(MOTOR4_A, 0);
            pwm_duty(MOTOR4_B, -DUTY);
        }
    }
}

void motor_init(void)
{
    // PWM_DUTY_MAX 50000
    pwm_init(MOTOR1_A, 17000, 0);
    pwm_init(MOTOR1_B, 17000, 0);
    pwm_init(MOTOR2_A, 17000, 0);
    pwm_init(MOTOR2_B, 17000, 0);
    pwm_init(MOTOR3_A, 17000, 0);
    pwm_init(MOTOR3_B, 17000, 0);
    pwm_init(MOTOR4_A, 17000, 0);
    pwm_init(MOTOR4_B, 17000, 0);

    // 电机停转PID控制 稍微超调增快响应
    motor_1.stop_pid.kp = 1100;
    motor_1.stop_pid.ki = 30;
    motor_1.stop_pid.kd = 8000;

    motor_2.stop_pid.kp = 1100;
    motor_2.stop_pid.ki = 30;
    motor_2.stop_pid.kd = 8000;

    motor_3.stop_pid.kp = 1100;
    motor_3.stop_pid.ki = 30;
    motor_3.stop_pid.kd = 8000;

    motor_4.stop_pid.kp = 1100;
    motor_4.stop_pid.ki = 30;
    motor_4.stop_pid.kd = 8000;

    //位置
    // 3  4
    // 1  2
}

void total_encoder_clear(void)
{
    motor_1.total_encoder = 0;
    motor_2.total_encoder = 0;
    motor_3.total_encoder = 0;
    motor_4.total_encoder = 0;
}

int64_t get_total_encoder(void)
{
    return (int64_t)((motor_1.total_encoder +
                      motor_2.total_encoder +
                      motor_3.total_encoder +
                      motor_4.total_encoder) /
                     4);
}
