#ifndef __ANGLE_H__
#define __ANGLE_H__

#include "headfile.h"
#include "pid.h"

#define ANGLR_MAX 10

typedef struct angle_param_t
{
    float target_angle;
    float raw;
  
    enum
    {
        MODE_NORMAL,
        MODE_TURN,
        MODE_RUN,
        MODE_AG_FIX,
        MODE_LR_FIX,
        MODE_UD_FIX,
        MODE_STOP
        // MODE_BANGBANG,
        // MODE_SOFT,
        // MODE_POSLOOP,
    } run_mode; //运行状态枚举
    
    enum
    {
        MODE_NONE,
        MODE_LEFTUP,
        MODE_RIGHTUP,
        MODE_LEFTDOWN,
        MODE_RIGHTDOWN
    } fix_mode; //运行状态枚举
    
    pid_param_t pid;       //PID param
} angle_param_t;

extern angle_param_t Angle;

#define ANGLE_CREATE(ta, kp, ki, kd, low_pass, _gama, p_max, i_max, d_max) \
    {                                                                                             \
        .fix_mode = MODE_NONE,                                                                    \
        .run_mode = MODE_NORMAL,                                                                  \
        .target_angle = ta,                                                                       \
        .pid = PID_CREATE(kp, ki, kd, low_pass, _gama, p_max, i_max, d_max),                      \
    }

#endif