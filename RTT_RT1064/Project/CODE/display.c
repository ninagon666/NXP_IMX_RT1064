#include "encoder.h"
#include "motor.h"
#include "display.h"
#include <stdio.h>
#include "MahonyAHRS.h"
#include "angle.h"

uint8 test1[25], test2[25], test3[25], test4[25], test5[25], test6[25], test7[25], test8[25], test9[25];

void display_entry(void *parameter)
{
    lcd_clear(WHITE);
    
    lcd_showstr(0, 0, (int8 *)"pitch:");
    lcd_showstr(0, 1, (int8 *)"roll:");
    lcd_showstr(0, 2, (int8 *)"yaw:");
    lcd_showstr(0, 3, (int8 *)"w1_ecs:");
    lcd_showstr(0, 4, (int8 *)"w2_ecs:");
    lcd_showstr(0, 5, (int8 *)"w3_ecs:");
    lcd_showstr(0, 6, (int8 *)"w4_ecs:");
    lcd_showstr(0, 7, (int8 *)"Tag_ag1:");
    lcd_showstr(0, 8, (int8 *)"alw_tec:");
    
    while(1)
    {        
        lcd_showfloat(8 * 6, 0, arhs_data.pitch, 3, 5);
        lcd_showfloat(8 * 5, 1, arhs_data.roll, 3, 5);
        lcd_showfloat(8 * 4, 2, arhs_data.yaw, 3, 5);
        lcd_showfloat(8 * 7, 3, motor_1.encoder_speed, 6, 1);
        lcd_showfloat(8 * 7, 4, motor_2.encoder_speed, 6, 1);
        lcd_showfloat(8 * 7, 5, motor_3.encoder_speed, 6, 1);
        lcd_showfloat(8 * 7, 6, motor_4.encoder_speed, 6, 1);
        lcd_showfloat(8 * 8, 7, Angle.target_angle, 3, 1);
        lcd_showint32(8 * 8, 8, (int32)get_total_encoder(), 7);
        if(Angle.run_mode == MODE_RUN)
        {
          lcd_showstr(0, 9, (int8 *)"MODE:RUN ");
        }
        else if(Angle.run_mode == MODE_STOP)
        {
          lcd_showstr(0, 9, (int8 *)"MODE:STOP");
        }
        else if(Angle.run_mode == MODE_TURN)
        {
          lcd_showstr(0, 9, (int8 *)"MODE:TURN");
        }
    }
}

void display_init(void)
{
    rt_thread_t tid;

    //初始化屏幕
    lcd_init();
    //寻找坐标时已经初始化
    
    //创建显示线程 优先级设置为31
    tid = rt_thread_create("display", display_entry, RT_NULL, 1024, 31, 10);

    //启动显示线程
    if (RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}
