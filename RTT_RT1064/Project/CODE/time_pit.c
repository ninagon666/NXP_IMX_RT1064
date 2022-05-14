#include "time_pit.h"
#include "encoder.h"
#include "pid.h"
#include "angle.h"
#include "MahonyAHRS.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))                   //输出小的值
#define MAX(a, b) (((a) > (b)) ? (a) : (b))                   //输出大的值
#define MINMAX(input, low, upper) MIN(MAX(input, low), upper) //限幅

uint32 flag1 = 0;

void timer1_pit_entry(void *parameter)
{
    float angle_error = 0.0;
    static uint32 time;
    time++;
    
    rt_enter_critical();
    Mahony_computeAngles();
    rt_exit_critical();

    encoder_get();
    if(Angle.run_mode == MODE_RUN)
    {
        motor_duty(PWM_1, MINMAX(pid_solve(&motor_1.pid, motor_1.target_speed - motor_1.encoder_speed), -15000, 15000), DIR_1);
        motor_duty(PWM_2, MINMAX(pid_solve(&motor_2.pid, motor_2.target_speed - motor_2.encoder_speed), -15000, 15000), DIR_2);
        motor_duty(PWM_3, MINMAX(pid_solve(&motor_3.pid, motor_3.target_speed - motor_3.encoder_speed), -15000, 15000), DIR_3);
        motor_duty(PWM_4, MINMAX(pid_solve(&motor_4.pid, motor_4.target_speed - motor_4.encoder_speed), -15000, 15000), DIR_4);
    }
    else if (Angle.run_mode == MODE_TURN)
    {
      angle_error = arhs_data.yaw - Angle.target_angle;
      if(angle_error > 180)
      {
          angle_error = angle_error - 360.0;
      }
      else if(angle_error < -180)
      {
          angle_error = angle_error + 360.0;
      }
      
      motor_1.target_speed = MINMAX(pid_solve(&Angle.pid, angle_error), -30, 30);
      motor_2.target_speed = -MINMAX(pid_solve(&Angle.pid, angle_error), -30, 30);
      motor_3.target_speed = MINMAX(pid_solve(&Angle.pid, angle_error), -30, 30);
      motor_4.target_speed = -MINMAX(pid_solve(&Angle.pid, angle_error), -30, 30);
      
      motor_duty(PWM_1, MINMAX(pid_solve(&motor_1.pid, motor_1.target_speed - motor_1.encoder_speed), -10000, 10000), DIR_1);
      motor_duty(PWM_2, MINMAX(pid_solve(&motor_2.pid, motor_2.target_speed - motor_2.encoder_speed), -10000, 10000), DIR_2);
      motor_duty(PWM_3, MINMAX(pid_solve(&motor_3.pid, motor_3.target_speed - motor_3.encoder_speed), -10000, 10000), DIR_3);
      motor_duty(PWM_4, MINMAX(pid_solve(&motor_4.pid, motor_4.target_speed - motor_4.encoder_speed), -10000, 10000), DIR_4);
    }
    else if (Angle.run_mode == MODE_STOP)
    {
        motor_duty(PWM_1, MINMAX(pid_solve(&motor_1.pid, - motor_1.encoder_speed), -15000, 15000), DIR_1);
        motor_duty(PWM_2, MINMAX(pid_solve(&motor_2.pid, - motor_2.encoder_speed), -15000, 15000), DIR_2);
        motor_duty(PWM_3, MINMAX(pid_solve(&motor_3.pid, - motor_3.encoder_speed), -15000, 15000), DIR_3);
        motor_duty(PWM_4, MINMAX(pid_solve(&motor_4.pid, - motor_4.encoder_speed), -15000, 15000), DIR_4);
    }
}

void timer_pit_init(void)
{
    rt_timer_t timer;

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

    imu963ra_init();
    imu_offset_init();

    //创建一个定时器 周期运行
    timer = rt_timer_create("timer1", timer1_pit_entry, RT_NULL, 5, RT_TIMER_FLAG_PERIODIC);

    //启动定时器
    if (RT_NULL != timer)
    {
        rt_timer_start(timer);
    }
}