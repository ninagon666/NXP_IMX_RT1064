#include "time_pit.h"
#include "encoder.h"
#include "pid.h"
#include "angle.h"
#include "MahonyAHRS.h"
#include "target_location.h"

#define PI 3.14159265358

#define MIN(a, b) (((a) < (b)) ? (a) : (b))                   //输出小的值
#define MAX(a, b) (((a) > (b)) ? (a) : (b))                   //输出大的值
#define MINMAX(input, low, upper) MIN(MAX(input, low), upper) //限幅

#define AGL_SP_LIM 20 //角度环差速限制
#define AGL_RUN_LIM 6 //直行角度环限制
#define AGL_TIM_LIM 55 //角度环控制时间限制

#define DISTANCE_K 204.2f //距离系数K，换算单位为cm，参考值206

rt_uint32_t run_gap = 0;//行驶距离

extern rt_sem_t turn_sem, run_sem, ag_fix_sem, lr_fix_sem, ud_fix_sem;
extern float lr_offset, ud_offest;
extern int8_t rx_array[5];

static rt_uint8_t turn_ticks = 0;

void timer1_pit_entry(void *parameter)
{
    float angle_error = 0.0;
    static rt_uint32_t time;
    time++;
    if(time == 20)
    {
      time = 0;
      if(rx_array[1] == 0)
      {
        PRINTF("apple\n");
      }
      else if(rx_array[1] == 1)
      {
        PRINTF("banana\n");
      }
      else if(rx_array[1] == 2)
      {
        PRINTF("bus\n");
      }
      else if(rx_array[1] == 3)
      {
        PRINTF("car\n");
      }
      else if(rx_array[1] == 4)
      {
        PRINTF("cat\n");
      }
      else if(rx_array[1] == 5)
      {
        PRINTF("cow\n");
      }
      else if(rx_array[1] == 6)
      {
        PRINTF("dog\n");
      }
      else if(rx_array[1] == 7)
      {
        PRINTF("durian\n");
      }
      else if(rx_array[1] == 8)
      {
        PRINTF("grapes\n");
      }
      else if(rx_array[1] == 9)
      {
        PRINTF("hourse\n");
      }
      else if(rx_array[1] == 10)
      {
        PRINTF("oranges\n");
      }
      else if(rx_array[1] == 11)
      {
        PRINTF("pig\n");
      }
      else if(rx_array[1] == 12)
      {
        PRINTF("plane\n");
      }
      else if(rx_array[1] == 13)
      {
        PRINTF("ship\n");
      }
      else if(rx_array[1] == 14)
      {
        PRINTF("train\n");
      }
    }
    
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
    encoder_get();
    rt_exit_critical();
    Threshold(mt9v03x_csi_image[0], mt9v03x_thres_image[0], MT9V03X_CSI_W, MT9V03X_CSI_H, 180);
    
    if(Angle.run_mode == MODE_RUN)
    {
        motor_duty(motor_1_, MINMAX(pid_solve(&motor_1.pid, motor_1.target_speed - motor_1.encoder_speed), -15000, 15000));
        motor_duty(motor_2_, MINMAX(pid_solve(&motor_2.pid, motor_2.target_speed - motor_2.encoder_speed), -15000, 15000));
        motor_duty(motor_3_, MINMAX(pid_solve(&motor_3.pid, motor_3.target_speed - motor_3.encoder_speed), -15000, 15000));
        motor_duty(motor_4_, MINMAX(pid_solve(&motor_4.pid, motor_4.target_speed - motor_4.encoder_speed), -15000, 15000));

        if (abs(angle_error) > 0.3)//行进过程中超过偏差超过该角度进行角度环控制
        {
            motor_duty(motor_1_, MINMAX(pid_solve(&motor_1.pid, motor_1.target_speed - motor_1.encoder_speed 
            - MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_RUN_LIM, AGL_RUN_LIM)), -15000, 15000));
            motor_duty(motor_2_, MINMAX(pid_solve(&motor_2.pid, motor_2.target_speed - motor_2.encoder_speed 
            + MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_RUN_LIM, AGL_RUN_LIM)), -15000, 15000));
            motor_duty(motor_3_, MINMAX(pid_solve(&motor_3.pid, motor_3.target_speed - motor_3.encoder_speed 
            - MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_RUN_LIM, AGL_RUN_LIM)), -15000, 15000));
            motor_duty(motor_4_, MINMAX(pid_solve(&motor_4.pid, motor_4.target_speed - motor_4.encoder_speed 
            + MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_RUN_LIM, AGL_RUN_LIM)), -15000, 15000));
        }
        //if(get_total_encoder() >= (run_gap * 1024 * 111 / (10 * 55 * PI)))//里程控制
        if(get_total_encoder() >= (run_gap * DISTANCE_K / (PI * 1.5f)))//里程控制
        {
          rt_uint32_t white_points = 0;
          for (int i = 0; i < MT9V03X_CSI_H; ++i)
          {
              for (int j = 0; j < MT9V03X_CSI_W; ++j)
              {
                  if(mt9v03x_thres_image[i][j] == 255){
                    white_points++;
                  }
              }
          }
          if(white_points >= 750)
          {
            all_wheels_set(0);
            rt_sem_release(run_sem);
            Angle.run_mode = MODE_AG_FIX;
          }
        }
        //PRINTF("%.0lf,%.0lf,%.0lf,%.0lf\n", motor_2.target_speed, motor_2.encoder_speed, motor_3.target_speed, motor_3.encoder_speed);
    }
    else if (Angle.run_mode == MODE_AG_FIX)
    {
      if(Angle.fix_mode == MODE_LEFTUP || Angle.fix_mode == MODE_RIGHTDOWN)
      {
        //角度环
        motor_1.target_speed = -MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_SP_LIM, AGL_SP_LIM);
        motor_2.target_speed = MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_SP_LIM, AGL_SP_LIM);
        motor_3.target_speed = 0;
        motor_4.target_speed = MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_SP_LIM, AGL_SP_LIM);
      }
      else if(Angle.fix_mode == MODE_RIGHTUP || Angle.fix_mode == MODE_LEFTDOWN)
      {
        //角度环
        motor_1.target_speed = -MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_SP_LIM, AGL_SP_LIM);
        motor_2.target_speed = 0;
        motor_3.target_speed = -MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_SP_LIM, AGL_SP_LIM);
        motor_4.target_speed =  MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_SP_LIM, AGL_SP_LIM);
      }
      //输入到速度内环
      motor_duty(motor_1_, MINMAX(pid_solve(&motor_1.pid, motor_1.target_speed - motor_1.encoder_speed), -10000, 10000));
      motor_duty(motor_2_, MINMAX(pid_solve(&motor_2.pid, motor_2.target_speed - motor_2.encoder_speed), -10000, 10000));
      motor_duty(motor_3_, MINMAX(pid_solve(&motor_3.pid, motor_3.target_speed - motor_3.encoder_speed), -10000, 10000));
      motor_duty(motor_4_, MINMAX(pid_solve(&motor_4.pid, motor_4.target_speed - motor_4.encoder_speed), -10000, 10000));

      if(abs(angle_error) < 0.2)//小于一定误差后开始计数 大于阈值停止角度环控制
      {
         turn_ticks++;
         if(turn_ticks >= AGL_TIM_LIM)
         {
           turn_ticks = 0;
           rt_sem_release(ag_fix_sem);
         }
      }
    }
    else if (Angle.run_mode == MODE_LR_FIX)
    {
      rt_uint32_t left_white = 0, right_white = 0;
      
      for (int j = 0; j < 93; ++j)
      {
          for (int i = 0; i < MT9V03X_CSI_H; ++i)
          {
              if(mt9v03x_thres_image[i][j] == 255){
                left_white++;
              }
          }
      }
      for (int j = 93; j < MT9V03X_CSI_W; ++j)
      {
          for (int i = 0; i < MT9V03X_CSI_H; ++i)
          {
              if(mt9v03x_thres_image[i][j] == 255){
                right_white++;
              }
          }
      }
      
      
      if((right_white + left_white) < 500)
      {
        if(Angle.fix_mode == MODE_LEFTUP || Angle.fix_mode == MODE_RIGHTDOWN)
        {
          motor_duty(motor_1_, MINMAX(pid_solve(&motor_1.pid,  8 - motor_1.encoder_speed), -8000, 8000));
          motor_duty(motor_2_, MINMAX(pid_solve(&motor_2.pid, -8 - motor_2.encoder_speed), -8000, 8000));
          motor_duty(motor_3_, MINMAX(pid_solve(&motor_3.pid, -8 - motor_3.encoder_speed), -8000, 8000));
          motor_duty(motor_4_, MINMAX(pid_solve(&motor_4.pid,  8 - motor_4.encoder_speed), -8000, 8000));
        }
        else if(Angle.fix_mode == MODE_RIGHTUP || Angle.fix_mode == MODE_LEFTDOWN)
        {
          motor_duty(motor_1_, MINMAX(pid_solve(&motor_1.pid, -8 - motor_1.encoder_speed), -8000, 8000));
          motor_duty(motor_2_, MINMAX(pid_solve(&motor_2.pid,  8 - motor_2.encoder_speed), -8000, 8000));
          motor_duty(motor_3_, MINMAX(pid_solve(&motor_3.pid,  8 - motor_3.encoder_speed), -8000, 8000));
          motor_duty(motor_4_, MINMAX(pid_solve(&motor_4.pid, -8 - motor_4.encoder_speed), -8000, 8000));
        }
      }
      else
      {
        if(abs(right_white - left_white) <= 80)
        {
          motor_duty(motor_1_, 0);
          motor_duty(motor_2_, 0);
          motor_duty(motor_3_, 0);
          motor_duty(motor_4_, 0);
          Angle.run_mode = MODE_STOP;
          
          rt_sem_release(lr_fix_sem);
        } 
        else if(right_white - left_white > 80)
        {   
          if(Angle.fix_mode == MODE_LEFTUP || Angle.fix_mode == MODE_RIGHTDOWN)
          {
            motor_duty(motor_1_, MINMAX(pid_solve(&motor_1.pid,  12 - motor_1.encoder_speed), -8000, 8000));
            motor_duty(motor_2_, MINMAX(pid_solve(&motor_2.pid, -12 - motor_2.encoder_speed), -8000, 8000));
            motor_duty(motor_3_, MINMAX(pid_solve(&motor_3.pid, -12 - motor_3.encoder_speed), -8000, 8000));
            motor_duty(motor_4_, MINMAX(pid_solve(&motor_4.pid,  12 - motor_4.encoder_speed), -8000, 8000));
          }
          else if(Angle.fix_mode == MODE_RIGHTUP || Angle.fix_mode == MODE_LEFTDOWN)
          {
            motor_duty(motor_1_, MINMAX(pid_solve(&motor_1.pid, -12 - motor_1.encoder_speed), -8000, 8000));
            motor_duty(motor_2_, MINMAX(pid_solve(&motor_2.pid,  12 - motor_2.encoder_speed), -8000, 8000));
            motor_duty(motor_3_, MINMAX(pid_solve(&motor_3.pid,  12 - motor_3.encoder_speed), -8000, 8000));
            motor_duty(motor_4_, MINMAX(pid_solve(&motor_4.pid, -12 - motor_4.encoder_speed), -8000, 8000));
          }
        }
        else if(left_white - right_white > 80)
        {        
          if(Angle.fix_mode == MODE_LEFTUP)
          {
            motor_duty(motor_1_, MINMAX(pid_solve(&motor_1.pid, -12 - motor_1.encoder_speed), -8000, 8000));
            motor_duty(motor_2_, MINMAX(pid_solve(&motor_2.pid,  12 - motor_2.encoder_speed), -8000, 8000));
            motor_duty(motor_3_, MINMAX(pid_solve(&motor_3.pid,  12 - motor_3.encoder_speed), -8000, 8000));
            motor_duty(motor_4_, MINMAX(pid_solve(&motor_4.pid, -12 - motor_4.encoder_speed), -8000, 8000));
          }
          else if(Angle.fix_mode == MODE_RIGHTUP)
          {
            motor_duty(motor_1_, MINMAX(pid_solve(&motor_1.pid,  12 - motor_1.encoder_speed), -8000, 8000));
            motor_duty(motor_2_, MINMAX(pid_solve(&motor_2.pid, -12 - motor_2.encoder_speed), -8000, 8000));
            motor_duty(motor_3_, MINMAX(pid_solve(&motor_3.pid, -12 - motor_3.encoder_speed), -8000, 8000));
            motor_duty(motor_4_, MINMAX(pid_solve(&motor_4.pid,  12  - motor_4.encoder_speed), -8000, 8000));
          }
        }
      }        
    }
    else if (Angle.run_mode == MODE_UD_FIX)
    {
      rt_uint32_t up_white = 0, down_white = 0;
      
      for (int i = 0; i < 75; ++i)
      {
          for (int j = 15; j < MT9V03X_CSI_W - 15; ++j)
          {
              if(mt9v03x_thres_image[i][j] == 255){
                up_white++;
              }
          }
      }
      for (int i = 75; i < MT9V03X_CSI_H; ++i)
      {
          for (int j = 15; j < MT9V03X_CSI_W - 15; ++j)
          {
              if(mt9v03x_thres_image[i][j] == 255){
                down_white++;
              }
          }
      }
      
      if(abs(up_white - down_white) <= 80)
      {
        motor_duty(motor_1_, 0);
        motor_duty(motor_2_, 0);
        motor_duty(motor_3_, 0);
        motor_duty(motor_4_, 0);
        Angle.run_mode = MODE_STOP;
        rt_sem_release(ud_fix_sem);
      } 
      else if(up_white > down_white)
      {        
        motor_duty(motor_1_, MINMAX(pid_solve(&motor_1.pid, 5 - motor_1.encoder_speed), -8000, 8000));
        motor_duty(motor_2_, MINMAX(pid_solve(&motor_2.pid, 5 - motor_2.encoder_speed), -8000, 8000));
        motor_duty(motor_3_, MINMAX(pid_solve(&motor_3.pid, 5 - motor_3.encoder_speed), -8000, 8000));
        motor_duty(motor_4_, MINMAX(pid_solve(&motor_4.pid, 5 - motor_4.encoder_speed), -8000, 8000));
      }
      else if(down_white > up_white)
      {        
        motor_duty(motor_1_, MINMAX(pid_solve(&motor_1.pid, -5 - motor_1.encoder_speed), -8000, 8000));
        motor_duty(motor_2_, MINMAX(pid_solve(&motor_2.pid, -5 - motor_2.encoder_speed), -8000, 8000));
        motor_duty(motor_3_, MINMAX(pid_solve(&motor_3.pid, -5 - motor_3.encoder_speed), -8000, 8000));
        motor_duty(motor_4_, MINMAX(pid_solve(&motor_4.pid, -5 - motor_4.encoder_speed), -8000, 8000));
      }
    }
    else if (Angle.run_mode == MODE_TURN)
    {
      //角度环
      motor_1.target_speed = -MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_SP_LIM, AGL_SP_LIM);
      motor_2.target_speed =  MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_SP_LIM, AGL_SP_LIM);
      motor_3.target_speed = -MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_SP_LIM, AGL_SP_LIM);
      motor_4.target_speed =  MINMAX(pid_solve_nomal(&Angle.pid, angle_error), -AGL_SP_LIM, AGL_SP_LIM);
      //输入到速度内环
      motor_duty(motor_1_, MINMAX(pid_solve(&motor_1.pid, motor_1.target_speed - motor_1.encoder_speed), -10000, 10000));
      motor_duty(motor_2_, MINMAX(pid_solve(&motor_2.pid, motor_2.target_speed - motor_2.encoder_speed), -10000, 10000));
      motor_duty(motor_3_, MINMAX(pid_solve(&motor_3.pid, motor_3.target_speed - motor_3.encoder_speed), -10000, 10000));
      motor_duty(motor_4_, MINMAX(pid_solve(&motor_4.pid, motor_4.target_speed - motor_4.encoder_speed), -10000, 10000));

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
        motor_duty(motor_1_, MINMAX(pid_solve(&motor_1.pid, -motor_1.encoder_speed), -20000, 30000));
        motor_duty(motor_2_, MINMAX(pid_solve(&motor_2.pid, -motor_2.encoder_speed), -30000, 30000));
        motor_duty(motor_3_, MINMAX(pid_solve(&motor_3.pid, -motor_3.encoder_speed), -30000, 30000));
        motor_duty(motor_4_, MINMAX(pid_solve(&motor_4.pid, -motor_4.encoder_speed), -30000, 30000));
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
