#include "control.h"
#include "time_pit.h"
#include "road.h"
#include "angle.h"
#include "motor.h"
#include "MahonyAHRS.h"

rt_sem_t next_control_sem = RT_NULL;
rt_sem_t turn_sem = RT_NULL;
rt_sem_t run_sem = RT_NULL;

void control_entry(void *parameter)
{
    for (int i = 0; i <= ture_points_num + 1; ++i)
    {
        
        rt_sem_take(next_control_sem, RT_WAITING_FOREVER);
        //rt_kprintf("next points\n");
        Angle.target_angle = turn_angle[i]; 
        PRINTF("Angle %.2f\n", Angle.target_angle);
        Angle.run_mode = MODE_TURN;
      
        rt_sem_take(turn_sem, RT_WAITING_FOREVER);
        total_encoder_clear();
        all_wheels_set(0);
        run_gap = map_gap[i];
        PRINTF("run_gap %d\n", run_gap);
        all_wheels_set(15);
        Angle.run_mode = MODE_RUN;
      
        rt_sem_take(run_sem, RT_WAITING_FOREVER);
        Angle.run_mode = MODE_STOP;
        rt_thread_mdelay(500);
        rt_sem_release(next_control_sem);
    }
       
    return ;
}

void control_init(void)
{
    rt_thread_t tid;

    //创建控制线程 优先级设置为11
    tid = rt_thread_create("control", control_entry, RT_NULL, 512, 11, 5);
  
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

    //启动控制线程
    if (RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}