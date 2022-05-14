/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		IAR 8.3 or MDK 5.28
 * @Target core		NXP RT1064DVL6A
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 ********************************************************************************************************************/

//�����Ƽ�IO�鿴Projecct�ļ����µ�TXT�ı�

//���µĹ��̻��߹����ƶ���λ�����ִ�����²���
//��һ�� �ر��������д򿪵��ļ�
//�ڶ��� project  clean  �ȴ��·�����������

//���ش���ǰ������Լ�ʹ�õ��������ڹ���������������Ϊ�Լ���ʹ�õ�

#include "headfile.h"

// User include
#include "display.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "time_pit.h"
#include "target_location.h"
#include "angle.h"

int main(void)
{
  // usb_cdc_init();
  timer_pit_init();
  //Target_Location_Init();
  display_init();
  encoder_init();
  motor_init();
  Angle.run_mode = MODE_RUN;
//  rt_thread_mdelay(5000);
//  while(1)
//  {
//      Angle.target_angle = 0.0;
//      rt_thread_mdelay(2000);
//      Angle.target_angle = 45.0;
//      rt_thread_mdelay(2000);
//      Angle.target_angle = 90.0;
//      rt_thread_mdelay(2000);
//      Angle.target_angle = 135.0;
//      rt_thread_mdelay(2000);
//      Angle.target_angle = 180.0;
//      rt_thread_mdelay(2000);
//      Angle.target_angle = 225.0;
//      rt_thread_mdelay(2000);
//      Angle.target_angle = 270.0;
//      rt_thread_mdelay(2000);
//      Angle.target_angle = 315.0;
//      rt_thread_mdelay(2000);
//  }

  EnableGlobalIRQ(0);
  
  return 0;
}
