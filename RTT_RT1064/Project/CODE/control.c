#include "control.h"
#include "time_pit.h"
#include "road.h"
#include "angle.h"
#include "motor.h"
#include "MahonyAHRS.h"

rt_sem_t next_control_sem = RT_NULL;
rt_sem_t turn_sem = RT_NULL;
rt_sem_t run_sem = RT_NULL;
rt_sem_t ag_fix_sem = RT_NULL;
rt_sem_t lr_fix_sem = RT_NULL;
rt_sem_t ud_fix_sem = RT_NULL;

void control_entry(void *parameter)
{
    for (int i = 0; i <= ture_points_num + 1; ++i)
    {
        
        rt_sem_take(next_control_sem, RT_WAITING_FOREVER);
        //rt_kprintf("next points\n");
        Angle.target_angle = turn_angle[i];
        Angle.run_mode = MODE_TURN;
      
        rt_sem_take(turn_sem, RT_WAITING_FOREVER);
        total_encoder_clear();
        all_wheels_set(0);
        run_gap = map_gap[i];
        all_wheels_set(15);
        Angle.run_mode = MODE_RUN;
      
        rt_sem_take(run_sem, RT_WAITING_FOREVER);
        rt_thread_mdelay(100); 
        Angle.run_mode = MODE_AG_FIX;
        total_encoder_clear();
        if(Angle.target_angle >= 0.0f && Angle.target_angle <= 90.0f)
        {
          Angle.fix_mode = MODE_LEFTUP;
          Angle.target_angle = 90.0f;
        }
        else if(Angle.target_angle >= 90.0f && Angle.target_angle <= 180.0f)
        {
          Angle.fix_mode = MODE_RIGHTUP;
          Angle.target_angle = 90.0f;
        }
        else if(Angle.target_angle >= 180.0f && Angle.target_angle <= 270.0f)
        {
          Angle.fix_mode = MODE_RIGHTDOWN;
          Angle.target_angle = 270.0f;
        }
        else if(Angle.target_angle >= 270.0f && Angle.target_angle < 360.0f)
        {
          Angle.fix_mode = MODE_LEFTDOWN;
          Angle.target_angle = 270.0f;
        }
        
        rt_sem_take(ag_fix_sem, RT_WAITING_FOREVER);
        total_encoder_clear();
        Angle.run_mode = MODE_LR_FIX;
        all_wheels_set(0);
      
        rt_sem_take(lr_fix_sem, RT_WAITING_FOREVER);
        Angle.run_mode = MODE_UD_FIX;
        all_wheels_set(0);
        
        rt_sem_take(ud_fix_sem, RT_WAITING_FOREVER);
        all_wheels_set(0);
        if(Angle.fix_mode == MODE_LEFTUP || Angle.fix_mode == MODE_RIGHTUP)
        {
          Angle.target_angle = 90.0f;
        }
        else if(Angle.fix_mode == MODE_RIGHTDOWN || Angle.fix_mode == MODE_LEFTDOWN)
        {
          Angle.target_angle = 270.0f;
        }
        Angle.run_mode = MODE_TURN;
        rt_sem_take(turn_sem, RT_WAITING_FOREVER);
        Angle.run_mode = MODE_STOP;
        
        uart_putchar(USART_1, '1');
        //rt_sem_release(openart_sem);
        //rt_sem_release(next_control_sem);
    }
       
    return ;
}

void control_init(void)
{
    rt_thread_t tid;

    //创建控制线程 优先级设置为11
    tid = rt_thread_create("control", control_entry, RT_NULL, 512, 11, 3);
  
    next_control_sem = rt_sem_create("next control", 1, RT_IPC_FLAG_PRIO);
    if (next_control_sem == RT_NULL)
    {
        rt_kprintf("create next control semaphore failed.\n");
        return ;
    }
    
    turn_sem = rt_sem_create("turn sem", 0, RT_IPC_FLAG_PRIO);
    if (turn_sem == RT_NULL)
    {
        rt_kprintf("create turn semaphore failed.\n");
        return ;
    }
    
    run_sem = rt_sem_create("run sem", 0, RT_IPC_FLAG_PRIO);
    if (run_sem == RT_NULL)
    {
        rt_kprintf("create run semaphore failed.\n");
        return ;
    }
    
    ag_fix_sem = rt_sem_create("run sem", 0, RT_IPC_FLAG_PRIO);
    if (ag_fix_sem == RT_NULL)
    {
        rt_kprintf("create run semaphore failed.\n");
        return ;
    }
    
    lr_fix_sem = rt_sem_create("run sem", 0, RT_IPC_FLAG_PRIO);
    if (lr_fix_sem == RT_NULL)
    {
        rt_kprintf("create run semaphore failed.\n");
        return ;
    }
    
    ud_fix_sem = rt_sem_create("run sem", 0, RT_IPC_FLAG_PRIO);
    if (ud_fix_sem == RT_NULL)
    {
        rt_kprintf("create run semaphore failed.\n");
        return ;
    }

    //启动控制线程
    if (RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}