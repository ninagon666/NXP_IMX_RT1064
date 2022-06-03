#include "time_pit.h"
#include "encoder.h"
#include "pid.h"
#include "angle.h"
#include "MahonyAHRS.h"

#define PI 3.14159265358

#define MIN(a, b) (((a) < (b)) ? (a) : (b))                   //输出小的值
#define MAX(a, b) (((a) > (b)) ? (a) : (b))                   //输出大的值
#define MINMAX(input, low, upper) MIN(MAX(input, low), upper) //限幅

#define AGL_SP_LIM 20 //角度环差速限制
#define AGL_RUN_LIM 6 //直行角度环限制
#define AGL_TIM_LIM 55 //角度环控制时间限制

#define DISTANCE_K 204.2f //距离系数K，换算单位为cm，参考值206

rt_uint32_t run_gap = 0;//行驶距离

extern rt_sem_t turn_sem, run_sem;

static rt_uint8_t turn_ticks = 0;

void timer1_pit_entry(void *parameter)
{
    float angle_error = 0.0;
//    static rt_uint32_t time;
//    time++;
    
    rt_enter_critical();
    Mahony_computeAngles();
    angle_error = arhs_data.yaw - Angle.target_angle;
    if (angle_error > 180)//误差限制，使其单次不会转过超过
    {
        angle_error = angle_error - 360.0;
    }
    else if (angle_error < -180)
    {
        angle_error = angle_error + 360.0;
    }
    rt_exit_critical();
    encoder_get();
    if(Angle.run_mode == MODE_RUN)
    {
        motor_duty(motor_1_, MINMAX(pid_solve_dah(&motor_1.pid, motor_1.target_speed - motor_1.encoder_speed), -15000, 15000));
        motor_duty(motor_2_, MINMAX(pid_solve_dah(&motor_2.pid, motor_2.target_speed - motor_2.encoder_speed), -15000, 15000));
        motor_duty(motor_3_, MINMAX(pid_solve_dah(&motor_3.pid, motor_3.target_speed - motor_3.encoder_speed), -15000, 15000));
        motor_duty(motor_4_, MINMAX(pid_solve_dah(&motor_4.pid, motor_4.target_speed - motor_4.encoder_speed), -15000, 15000));

        if (abs(angle_error) > 0.5)//行进过程中超过偏差超过该角度进行角度环控制
        {
            motor_duty(motor_1_, MINMAX(pid_solve_dah(&motor_1.pid, motor_1.target_speed - motor_1.encoder_speed 
            - MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_RUN_LIM, AGL_RUN_LIM)), -15000, 15000));
            motor_duty(motor_2_, MINMAX(pid_solve_dah(&motor_2.pid, motor_2.target_speed - motor_2.encoder_speed 
            + MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_RUN_LIM, AGL_RUN_LIM)), -15000, 15000));
            motor_duty(motor_3_, MINMAX(pid_solve_dah(&motor_3.pid, motor_3.target_speed - motor_3.encoder_speed 
            - MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_RUN_LIM, AGL_RUN_LIM)), -15000, 15000));
            motor_duty(motor_4_, MINMAX(pid_solve_dah(&motor_4.pid, motor_4.target_speed - motor_4.encoder_speed 
            + MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_RUN_LIM, AGL_RUN_LIM)), -15000, 15000));
        }
        //if(get_total_encoder() >= (run_gap * 1024 * 111 / (10 * 55 * PI)))//里程控制
        if(get_total_encoder() >= (run_gap * DISTANCE_K / PI))//里程控制
        {
          rt_sem_release(run_sem);
          //Angle.run_mode = MODE_STOP;
        }
        //PRINTF("%.0lf,%.0lf,%.0lf,%.0lf\n", motor_2.target_speed, motor_2.encoder_speed, motor_3.target_speed, motor_3.encoder_speed);
    }
    else if (Angle.run_mode == MODE_TURN)
    {
      //角度环
      motor_1.target_speed = -MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_SP_LIM, AGL_SP_LIM);
      motor_2.target_speed = MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_SP_LIM, AGL_SP_LIM);
      motor_3.target_speed = -MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_SP_LIM, AGL_SP_LIM);
      motor_4.target_speed = MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_SP_LIM, AGL_SP_LIM);
      //输入到速度内环
      motor_duty(motor_1_, MINMAX(pid_solve_dah(&motor_1.pid, motor_1.target_speed - motor_1.encoder_speed), -10000, 10000));
      motor_duty(motor_2_, MINMAX(pid_solve_dah(&motor_2.pid, motor_2.target_speed - motor_2.encoder_speed), -10000, 10000));
      motor_duty(motor_3_, MINMAX(pid_solve_dah(&motor_3.pid, motor_3.target_speed - motor_3.encoder_speed), -10000, 10000));
      motor_duty(motor_4_, MINMAX(pid_solve_dah(&motor_4.pid, motor_4.target_speed - motor_4.encoder_speed), -10000, 10000));

      if(abs(angle_error) < 0.2)//小于一定误差后开始计数 大于阈值停止角度环控制
      {
         turn_ticks++;
         if(turn_ticks >= AGL_TIM_LIM)
         {
           turn_ticks = 0;
           rt_sem_release(turn_sem);
           //Angle.run_mode = MODE_STOP;
         }
      }
    }
    else if (Angle.run_mode == MODE_STOP)
    {
        motor_duty(motor_1_, MINMAX(pid_solve_dah(&motor_1.pid, -motor_1.encoder_speed), -30000, 30000));
        motor_duty(motor_2_, MINMAX(pid_solve_dah(&motor_2.pid, -motor_2.encoder_speed), -30000, 30000));
        motor_duty(motor_3_, MINMAX(pid_solve_dah(&motor_3.pid, -motor_3.encoder_speed), -30000, 30000));
        motor_duty(motor_4_, MINMAX(pid_solve_dah(&motor_4.pid, -motor_4.encoder_speed), -30000, 30000));
    }
}

void timer_pit_init(void)
{
    rt_timer_t timer;
  
    Angle.run_mode = MODE_STOP;
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
