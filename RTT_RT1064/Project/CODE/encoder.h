#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "headfile.h"
#include "motor.h"

extern float x_total, y_total;

void encoder_init(void);

void encoder_get(void);

#endif
