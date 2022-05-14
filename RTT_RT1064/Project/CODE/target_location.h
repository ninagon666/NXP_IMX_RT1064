#ifndef __TARGET_LOCATION_H__
#define __TARGET_LOCATION_H__
#include "headfile.h"

extern uint8 (*mt9v03x_thres_image)[MT9V03X_CSI_W];

void Threshold(uint8 *sp, uint8 *tp, uint16 width, uint16 height, uint8 thres);
void AdaptiveThreshold(uint8 *img_data, uint8 *output_data, int width, int height, int block, uint8 clip_value);
void Target_Location_Init(void);

typedef struct
{
    uint32 x_sum;
    uint32 y_sum;
    uint8 nums;
    uint8 row;
    uint8 column;
} Point_place;

#endif