#ifndef __ROAD_H__
#define __ROAD_H__

#include "headfile.h"

extern rt_uint8_t ture_points_num;

extern int true_map[22][2];//转换后的坐标点位置
extern int points_map[22][2];
extern int points_label[22];
extern double turn_angle[22];//转向角度
extern double map_gap[22];//旅程

void find_road(int in_map[][2], int *out_label);
void map_change(int input[][2], int output[][2], int pts_nums, int *pts_label, int yr, int xb);
void map_calculate(int input[][2], double *angle, double *gap, const int pts_nums, double x_length, double y_length);

#endif