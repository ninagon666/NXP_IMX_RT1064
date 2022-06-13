#include "road.h"
#include "math.h"

#define PI 3.14159265358

rt_uint8_t ture_points_num = 0;

int points_map[22][2] = {0};//拍摄到的坐标点位置
int true_map[22][2] = {0};//转换后的坐标点位置
int points_label[22] = {0};//路径序列
double turn_angle[22] = {0};//转向角度
double map_gap[22] = {0};//旅程

void find_road(int in_map[][2], int *out_label) {
    double points_gap[21] = {0}; //用于存放点到点之间的距离
    int point_now = 0;           //当前指向点
    int pts_label[21] = {0};     //路径序列
    int road_best = 0;           //局部最优点号

    for (int i = 0; i <= ture_points_num; ++i) {
        for (int j = 0; j <= ture_points_num; ++j) {
            points_gap[j] = sqrt(pow((in_map[point_now][0] - in_map[j][0]), 2)
                                 + pow((in_map[point_now][1] - in_map[j][1]), 2)); //计算距离
        }
        for (int j = 0; j <= ture_points_num; ++j) {
            unsigned char tag = 0;
            if (road_best == point_now && j != point_now) {
                for (int k = 0; k < i; ++k)
                    if (pts_label[k] == j) tag = 1;
                if (!tag) road_best = j;
            }
            else if (points_gap[j] < points_gap[road_best] && j != point_now) {
                for (int k = 0; k < i; ++k)
                    if (pts_label[k] == j) tag = 1;
                if (!tag) road_best = j;
            }
        }
        point_now = road_best;
        pts_label[i + 1] = road_best;
    }

    rt_memcpy(out_label, pts_label, (ture_points_num+1)*sizeof(int *));//把路径传到序列指针
}

//转换坐标
void map_change(int input[][2], int output[][2], int pts_nums, int *pts_label, int yr, int xb) {
    for (int i = 0; i <= pts_nums; ++i) {
        output[i][0] = input[pts_label[i]][0];
        output[i][1] = input[pts_label[i]][1];
    }//重新排序

    for (int i = 0; i <= pts_nums; ++i) {
        output[i][0] = abs(output[i][0] - yr);
        output[i][1] = abs(output[i][1] - xb);
    }//重新映射坐标系
    
    return ;
}

//转换坐标值
void map_calculate(int input[][2], double *angle, double *gap, const int pts_nums, double x_length, double y_length) {

//    x_length = (double)(Yr - Yl) / 100.0f;
//    y_length = (double)(Xb - Xt) / 100.0f;

    for (int i = 0; i <= pts_nums; ++i) {
        angle[i] = atan2(input[i+1][1] - input[i][1],
                         input[i+1][0] - input[i][0]) * 180.0f / PI;
        if(angle[i] < 0)
            angle[i] = 360.0f + angle[i];
        
//        for (int j = 1; j <= pts_nums; ++j) {
//          input[j][0] = input[j][0] - (x_length * 10.0f);
//          input[j][1] = input[j][1] + (y_length * 8.0f);
//        }
        
        gap[i] = sqrt(pow((((double) input[i][0] / x_length * 7.0f) -
                       ((double) input[i + 1][0] / x_length * 7.0f)), 2)
                       + pow((((double) input[i][1] / y_length * 5.0f) -
                       ((double) input[i + 1][1] / y_length * 5.0f)), 2));
    }
}
