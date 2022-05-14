#include "encoder.h"

// ENCODER1
#define ENCODER1_QTIMER QTIMER_1
#define ENCODER1_A QTIMER1_TIMER2_C2
#define ENCODER1_B QTIMER1_TIMER3_C24
// ENCODER2
#define ENCODER2_QTIMER QTIMER_1
#define ENCODER2_A QTIMER1_TIMER0_C0
#define ENCODER2_B QTIMER1_TIMER1_C1
// ENCODER3
#define ENCODER3_QTIMER QTIMER_2
#define ENCODER3_A QTIMER2_TIMER0_C3
#define ENCODER3_B QTIMER2_TIMER3_C25
// ENCODER4
#define ENCODER4_QTIMER QTIMER_3
#define ENCODER4_A QTIMER3_TIMER2_B18
#define ENCODER4_B QTIMER3_TIMER3_B19

void encoder_init(void)
{
    qtimer_quad_init(ENCODER1_QTIMER, ENCODER1_A, ENCODER1_B);
    qtimer_quad_init(ENCODER2_QTIMER, ENCODER2_A, ENCODER2_B);
    qtimer_quad_init(ENCODER3_QTIMER, ENCODER3_A, ENCODER3_B);
    qtimer_quad_init(ENCODER4_QTIMER, ENCODER4_A, ENCODER4_B);
}

float low_pass = 0.25;  //低通滤波系数(0-1) 越高灵敏度越低但是平稳 反之不稳定但是灵敏度高

void encoder_get(void)
{
    //debug
//    motor_1.encoder_raw = -qtimer_quad_get(ENCODER1_QTIMER, ENCODER1_A);
//    motor_2.encoder_raw = qtimer_quad_get(ENCODER2_QTIMER, ENCODER2_A);
//    motor_3.encoder_raw = qtimer_quad_get(ENCODER3_QTIMER, ENCODER3_A);
//    motor_4.encoder_raw = -qtimer_quad_get(ENCODER4_QTIMER, ENCODER4_A);
//    PRINTF("%.0lf,%.0lf,%.0lf,%.0lf\n", motor_1.encoder_raw, motor_2.encoder_raw, motor_3.encoder_raw, motor_4.encoder_raw);
    
    //获取原始数据
    motor_1.encoder_raw = qtimer_quad_get(ENCODER1_QTIMER, ENCODER1_A) / 2.;
    motor_2.encoder_raw = -qtimer_quad_get(ENCODER2_QTIMER, ENCODER2_A) / 2.;
    motor_3.encoder_raw = qtimer_quad_get(ENCODER3_QTIMER, ENCODER3_A) / 2.;
    motor_4.encoder_raw = -qtimer_quad_get(ENCODER4_QTIMER, ENCODER4_A) / 2.;

    //编码器 低通滤波
    motor_1.encoder_speed = motor_1.encoder_speed * low_pass + motor_1.encoder_raw * (1. - low_pass);
    motor_2.encoder_speed = motor_2.encoder_speed * low_pass + motor_2.encoder_raw * (1. - low_pass);
    motor_3.encoder_speed = motor_3.encoder_speed * low_pass + motor_3.encoder_raw * (1. - low_pass);
    motor_4.encoder_speed = motor_4.encoder_speed * low_pass + motor_4.encoder_raw * (1. - low_pass);

    //计算
    motor_1.total_encoder += motor_1.encoder_raw;
    motor_2.total_encoder += motor_2.encoder_raw;
    motor_3.total_encoder += motor_3.encoder_raw;
    motor_4.total_encoder += motor_4.encoder_raw;

    //清空编码器读数
    qtimer_quad_clear(ENCODER1_QTIMER, ENCODER1_A);
    qtimer_quad_clear(ENCODER2_QTIMER, ENCODER2_A);
    qtimer_quad_clear(ENCODER3_QTIMER, ENCODER3_A);
    qtimer_quad_clear(ENCODER4_QTIMER, ENCODER4_A);
}
