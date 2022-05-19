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
        MODE_STOP
        // MODE_BANGBANG,
        // MODE_SOFT,
        // MODE_POSLOOP,
    } run_mode; //ÔËÐÐ×´Ì¬Ã¶¾Ù
    
    pid_param_t pid;       //PID param
} angle_param_t;

extern angle_param_t Angle;

#define ANGLE_CREATE(ta, kp, ki, kd, low_pass, p_max, i_max, d_max) \
    {                                                                                             \
        .run_mode = MODE_NORMAL,                                                                  \
        .target_angle = ta,                                                                       \
        .pid = PID_CREATE(kp, ki, kd, low_pass, p_max, i_max, d_max),                             \
    }

#endif