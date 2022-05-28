#include "motor.h"
//
motor_param_t motor_1 = MOTOR_CREATE(0, 900, 25, 6000, 1, 0.5, MOTOR_P_MAX, MOTOR_I_MAX, MOTOR_D_MAX);
motor_param_t motor_2 = MOTOR_CREATE(0, 580, 25, 7600, 1, 0.5, MOTOR_P_MAX, MOTOR_I_MAX, MOTOR_D_MAX);
motor_param_t motor_3 = MOTOR_CREATE(0, 510, 25, 8500, 1, 0.5, MOTOR_P_MAX, MOTOR_I_MAX, MOTOR_D_MAX);
motor_param_t motor_4 = MOTOR_CREATE(0, 900, 25, 6000, 1, 0.5, MOTOR_P_MAX, MOTOR_I_MAX, MOTOR_D_MAX);

void all_wheels_set(float ats)
{
  motor_1.target_speed = ats;
  motor_2.target_speed = ats;
  motor_3.target_speed = ats;
  motor_4.target_speed = ats;
}

void motor_duty(PWMCH_enum pwmch, int32 DUTY, PIN_enum PIN)
{
    //四个if判断着写的原因是避免电机接口接错，实际不方便改，而用软件方式改正
    if (pwmch == PWM_1)
    {
        if (DUTY > 0)
        {
            gpio_set(PIN, 1);
            pwm_duty(pwmch, DUTY);
        }
        else
        {
            gpio_set(PIN, 0);
            pwm_duty(pwmch, -DUTY);
        }
    }
    else if (pwmch == PWM_2)
    {
        if (DUTY > 0)
        {
            gpio_set(PIN, 1);
            pwm_duty(pwmch, DUTY);
        }
        else
        {
            gpio_set(PIN, 0);
            pwm_duty(pwmch, -DUTY);
        }
    }
    else if (pwmch == PWM_3)
    {
        if (DUTY > 0)
        {
            gpio_set(PIN, 0);
            pwm_duty(pwmch, DUTY);
        }
        else
        {
            gpio_set(PIN, 1);
            pwm_duty(pwmch, -DUTY);
        }
    }
    else if (pwmch == PWM_4)
    {
        if (DUTY > 0)
        {
            gpio_set(PIN, 1);
            pwm_duty(pwmch, DUTY);
        }
        else
        {
            gpio_set(PIN, 0);
            pwm_duty(pwmch, -DUTY);
        }
    }
}

void motor_init(void)
{
    gpio_init(DIR_1, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(DIR_2, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(DIR_3, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(DIR_4, GPO, 0, GPIO_PIN_CONFIG);
    // PWM_DUTY_MAX 50000
    pwm_init(PWM_1, 17000, 0);
    pwm_init(PWM_2, 17000, 0);
    pwm_init(PWM_3, 17000, 0);
    pwm_init(PWM_4, 17000, 0);

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
    // pwm_duty(PWM_1, 0);
    // pwm_duty(PWM_2, 0);
    // pwm_duty(PWM_3, 0);
    // pwm_duty(PWM_4, 0);
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
