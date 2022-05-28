#include "road.h"
#include "math.h"

uint8 ture_points_num = 0;
int points_map[21][2] = {0};

void find_road(int **map, int *label) {
    int point_total = 12;
    int point_now = 0;           //当前指向点
    double points_gap[21] = {0}; //用于存放点到点之间的距离
    int points_label[21] = {0};  //路径序列
    int road_best = 0;           //局部最优点号

    for (int i = 0; i < point_total; ++i) {
        for (int j = 0; j < point_total; ++j) {
            points_gap[j] = sqrt(pow((map[point_now][0] - map[j][0]), 2) + pow((map[point_now][1] - map[j][1]), 2)); //计算距离
            if (road_best == point_now)
                road_best = j; //排除从出发点指向出发点
            else if (points_gap[j] < points_gap[road_best]) {
                //表示当前状态 0 - 未经过重复点 1 - 经过重复点
                char tag = 0; 
                //查询已经去过的点
                for (int k = 0; k < i; ++k) {
                    if (points_label[k] == j)
                        tag = 1;
                }

                if (tag == 0) {
                    if (points_gap[j] != 0)
                        road_best = j;
                }
            }
        }
        point_now = road_best;
        points_label[i + 1] = road_best;
    }
    
    label = points_label;//把路径传到序列指针
}