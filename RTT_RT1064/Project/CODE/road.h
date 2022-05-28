#ifndef __ROAD_H__
#define __ROAD_H__

#include "headfile.h"

extern uint8 ture_points_num;
extern int points_map[21][2];

void find_road(int **map, int *label);

#endif