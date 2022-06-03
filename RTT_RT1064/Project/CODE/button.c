#include "buzzer.h"
#include "button.h"

#define KEY_UP C27
#define KEY_DOWN D4
#define KEY_LEFT C31
#define KEY_RIGHT C4
#define KEY_ENTER C26

#define KEY_1 C31
#define KEY_2 C27
#define KEY_3 C26
#define KEY_4 C4

//开关状态变量
uint8 key_1_status = 1;
uint8 key_2_status = 1;
uint8 key_3_status = 1;
uint8 key_4_status = 1;

//上一次开关状态变量
uint8 key_1_last_status;
uint8 key_2_last_status;
uint8 key_3_last_status;
uint8 key_4_last_status;

//开关信号量
rt_sem_t key1_sem;
rt_sem_t key2_sem;
rt_sem_t key3_sem;
rt_sem_t key4_sem;

void button_entry(void *parameter)
{
    //保存按键状态
    key_1_last_status = key_1_status;
    key_2_last_status = key_2_status;
    key_3_last_status = key_3_status;
    key_4_last_status = key_4_status;
    
    //读取当前按键状态
    key_1_status = gpio_get(KEY_1);
    key_2_status = gpio_get(KEY_2);
    key_3_status = gpio_get(KEY_3);
    key_4_status = gpio_get(KEY_4);
    
    //检测到按键按下之后并放开 释放一次信号量
    if(key_1_status && !key_1_last_status)    
    {
        rt_sem_release(key1_sem);
        rt_mb_send(buzzer_mailbox, 50);
    }
    if(key_2_status && !key_2_last_status)    
    {
        rt_sem_release(key2_sem);
        rt_mb_send(buzzer_mailbox, 100);
    }
    if(key_3_status && !key_3_last_status)    
    {
        rt_sem_release(key3_sem);
        rt_mb_send(buzzer_mailbox, 150);
    }
    if(key_4_status && !key_4_last_status)    
    {
        rt_sem_release(key4_sem);
        rt_mb_send(buzzer_mailbox, 200);
    }
}

void button_init(void)
{
    rt_timer_t timer1;

    // 初始化为GPIO浮空输入 默认上拉高电平
    gpio_init(KEY_1, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);			
    gpio_init(KEY_2, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);
    gpio_init(KEY_3, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);
    gpio_init(KEY_4, GPI, GPIO_HIGH, GPIO_PIN_CONFIG);
    
    //创建按键的信号量，当按键按下就释放信号量，在需要使用按键的地方获取信号量即可
    key1_sem = rt_sem_create("key1", 0, RT_IPC_FLAG_FIFO);		
    key2_sem = rt_sem_create("key2", 0, RT_IPC_FLAG_FIFO);  
    key3_sem = rt_sem_create("key3", 0, RT_IPC_FLAG_FIFO);  
    key4_sem = rt_sem_create("key4", 0, RT_IPC_FLAG_FIFO);  
    
    timer1 = rt_timer_create("button", button_entry, RT_NULL, 20, RT_TIMER_FLAG_PERIODIC);

    if(RT_NULL != timer1) 
    {
        rt_timer_start(timer1);
    }
}