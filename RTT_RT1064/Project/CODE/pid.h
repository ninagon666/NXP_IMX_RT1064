#ifndef __PID_H__
#define __PID_H__

#include "headfile.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))                   //输出小的值
#define MAX(a, b) (((a) > (b)) ? (a) : (b))                   //输出大的值
#define MINMAX(input, low, upper) MIN(MAX(input, low), upper) //限幅

typedef struct
{
    float kp;    //P
    float ki;    //I
    float kd;    //D
    float p_max; //p_integrator_max
    float i_max; //i_integrator_max
    float d_max; //d_integrator_max

    float low_pass;//一阶低通滤波系数
    float gama;    //微分先行滤波系数

    float out_p;
    float out_i;
    float out_d;

    float error;
    float pre_error;     //前次误差
    float pre_pre_error; //前前次误差

    //float last_output;//前次输出
} pid_param_t;

/*
 * name : PID_CREATE
 * description : create a struct & init 
 * how to use : pid_param_t XX = PID_CREATE(kp, ki, kd, low_pass, max_p, max_i, max_d);
 */

#define PID_CREATE(_kp, _ki, _kd, _low_pass, _gama, max_p, max_i, max_d) \
    {                                                             \
        .kp = _kp,                                                \
        .ki = _ki,                                                \
        .kd = _kd,                                                \
        .low_pass = _low_pass,                                    \
        .out_p = 0,                                               \
        .out_i = 0,                                               \
        .out_d = 0,                                               \
        .p_max = max_p,                                           \
        .i_max = max_i,                                           \
        .d_max = max_d,                                           \
        .gama = _gama,                                            \
    }

float pid_solve_dah(pid_param_t *pid, float error);
float pid_solve_nomal(pid_param_t *pid, float error);    
float increment_pid_solve(pid_param_t *pid, float error);
float changable_pid_solve(pid_param_t *pid, float error);

#endif /* _PID_H_ */
    