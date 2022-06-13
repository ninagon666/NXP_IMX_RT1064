#include "target_location.h"
#include "road.h"
#include "time_pit.h"
#include "buzzer.h"
#include <stdio.h>

#define KEY_UP C27
#define KEY_DOWN D4
#define KEY_LEFT C31
#define KEY_RIGHT C4
#define KEY_ENTER C26

//定义拨码开关引脚
#define SW2 D27

//开关状态变量
uint8 key_up_status = 1;
uint8 key_down_status = 1;
uint8 key_left_status = 1;
uint8 key_right_status = 1;
uint8 key_enter_status = 1;

//上一次开关状态变量
uint8 key_up_last_status;
uint8 key_down_last_status;
uint8 key_left_last_status;
uint8 key_right_last_status;
uint8 key_enter_last_status;

//开关标志位
uint8 key_up_flag;
uint8 key_down_flag;
uint8 key_left_flag;
uint8 key_right_flag;
uint8 key_enter_flag;

//边缘点坐标 1~4顺时针
uint8 edge_point1_x = 0, edge_point1_y = 0; //左上
uint8 edge_point2_x = 0, edge_point2_y = 0; //右上
uint8 edge_point3_x = 0, edge_point3_y = 0; //右下
uint8 edge_point4_x = 0, edge_point4_y = 0; //左下

//边缘线坐标 Xt 上边缘X坐标 Xb 下边缘X坐标 Yl 左边缘Y坐标 Yr 右边缘Y坐标
uint8 Xt = 10, Xb = 35, Yl = 10, Yr = 45;

//二值化阈值 显示字符串
uint8 thres = 100, thres_str[20];
uint8 clip_value = 8;

//总点数
uint8 total_point = 0;

//边框补偿
#define ROW_CLIP 5
#define COLUMN_CLIP 5

//点数阈值
#define POINT_THRESHOLD 5

//寻找坐标点时数组
static Point_place Points_arr[30] = {0};

//坐标点数组
extern int points_map[22][2];

//二值化图像指针
uint8 (*mt9v03x_thres_image)[MT9V03X_CSI_W];

// lcd显示字符串
static uint8 point1_str[20], point2_str[20], point3_str[20], point4_str[20];

//状态标志
static uint8 MODE_TAG = 0;

//固定阈值二值化
void Threshold(uint8 *sp, uint8 *tp, uint16 width, uint16 height, uint8 thres)
{
    uint32 i, j;

    for (j = 0; j < height; ++j)
    {
        for (i = 0; i < width; ++i)
        {
            (*(tp + j * width + i)) = (*(sp + j * width + i)) > thres ? 255 : 0;
        }
    }
}

//自适应阈值二值化
AT_ITCM_SECTION_INIT(void AdaptiveThreshold(uint8 *img_data, uint8 *output_data, int width, int height, int block, uint8 clip_value))
{
    assert(block % 2 == 1); // block必须为奇数
    int half_block = block / 2;
    for (int y = half_block; y < height - half_block; ++y)
    {
        for (int x = half_block; x < width - half_block; ++x)
        {
            // 计算局部阈值
            int thres = 0;
            for (int dy = -half_block; dy <= half_block; ++dy)
            {
                for (int dx = -half_block; dx <= half_block; ++dx)
                {
                    thres += img_data[(x + dx) + (y + dy) * width];
                }
            }
            thres = thres / (block * block) - clip_value;
            // 进行二值化
            output_data[x + y * width] = img_data[x + y * width] > thres ? 255 : 0;
        }
    }
}

//**寻找边界**//

#define COLUMN_SCOPE 70 //列扫描范围 向左 & 向右 扫描
#define ROW_SCOPE 50    //行扫描范围 向上 & 向下 扫描

#define COLUMN_OFFSET 50 //列偏执 向左 & 向右 扫描时从中央分别 向上 & 向下 判定多少列
#define ROW_OFFSET 50    //行偏执 向上 & 向下 扫描时从中央分别 向左 & 向右 判定多少行

#define COLUMN_TAR 60 //列判定阈值
#define ROW_TAR 55    //行判定阈值

#define DOUBLE_COLUNN_OFFSET 100 //两倍列偏执 用于设定存相邻三行放特征值的数组
#define DOUBLE_ROW_OFFSET 100    //两倍行偏执 用于设定存放相邻三列特征值的数组

void Find_Edge_1(void)
{
    uint8 aspect = 255;   //相邻3 行|列 压缩特征值
    uint8 black_nums = 0; //记录当前扫描行黑点数值

    //上半区域 从中间向上搜索 逐行
    for (uint8 i = 60; i > 60 - ROW_SCOPE; --i)
    {
        black_nums = 0;
        //逐行搜索 每次把搜索点所处列上下点的数据融合
        for (uint8 j = 94 - COLUMN_OFFSET; j < 94 + COLUMN_OFFSET; ++j)
        {
            aspect = mt9v03x_thres_image[i - 1][j] &
                     mt9v03x_thres_image[i][j] & mt9v03x_thres_image[i + 1][j];
            if (aspect == 0)
                black_nums++;
        }
        if (black_nums > COLUMN_TAR)
        {
            Xt = i;
            break;
        }
    }

    //右半区域 从中间向右搜索 逐列
    for (uint8 j = 94; j < 94 + COLUMN_SCOPE; ++j)
    {
        black_nums = 0;
        //逐行搜索 每次把搜索点所处行上下点的数据融合
        for (uint8 i = 60 - ROW_OFFSET; i < 60 + ROW_OFFSET; ++i)
        {
            aspect = mt9v03x_thres_image[i][j - 1] &
                     mt9v03x_thres_image[i][j] & mt9v03x_thres_image[i][j + 1];
            if (aspect == 0)
                black_nums++;
        }
        if (black_nums > ROW_TAR)
        {
            Yr = j;
            break;
        }
    }

    //下半区域 从中间向下搜索 逐行
    for (uint8 i = 60; i < 60 + ROW_SCOPE; ++i)
    {
        black_nums = 0;
        //逐行搜索 每次把搜索点所处列上下点的数据融合
        for (uint8 j = 94 - COLUMN_OFFSET; j < 94 + COLUMN_OFFSET; ++j)
        {
            aspect = mt9v03x_thres_image[i - 1][j] &
                     mt9v03x_thres_image[i][j] & mt9v03x_thres_image[i + 1][j];
            if (aspect == 0)
                black_nums++;
        }
        if (black_nums > COLUMN_TAR)
        {
            Xb = i;
            break;
        }
    }

    //左半区域 从中间向左搜索 逐列
    for (uint8 j = 94; j > 94 - COLUMN_SCOPE; --j)
    {
        black_nums = 0;
        //逐行搜索 每次把搜索点所处行上下点的数据融合
        for (uint8 i = 60 - ROW_OFFSET; i < 60 + ROW_OFFSET; ++i)
        {
            aspect = mt9v03x_thres_image[i][j - 1] &
                     mt9v03x_thres_image[i][j] & mt9v03x_thres_image[i][j + 1];
            if (aspect == 0)
                black_nums++;
        }
        if (black_nums > ROW_TAR)
        {
            Yl = j;
            break;
        }
    }
}

void Find_Edge_2(void)
{
    uint8 black_MAX = 0; //记录历史扫描行黑点最大值
    uint8 black_NUM = 0; //记录当前扫描行黑点数值

    black_MAX = 0;
    for (uint8 i = 10; i < 60; ++i)
    {
        black_NUM = 0;
        for (uint8 j = 0; j < 188; ++j)
        {
            if (mt9v03x_thres_image[i][j] == 0)
                black_NUM++;
        }
        if (black_NUM > black_MAX)
        {
            Xt = i;
            black_MAX = black_NUM;
        }
    } //上半区域 从顶部向中间搜索 逐行

    black_MAX = 0;
    for (uint8 i = 110; i > 60; --i)
    {
        black_NUM = 0;
        for (uint8 j = 0; j < 188; ++j)
        {
            if (mt9v03x_thres_image[i][j] == 0)
                black_NUM++;
        }
        if (black_NUM > black_MAX)
        {
            Xb = i;
            black_MAX = black_NUM;
        }
    } //下半区域

    black_MAX = 0;
    for (uint8 j = 10; j < 94; ++j)
    {
        black_NUM = 0;
        for (uint8 i = 0; i < 120; ++i)
        {
            if (mt9v03x_thres_image[i][j] == 0)
                black_NUM++;
        }
        if (black_NUM > black_MAX)
        {
            Yl = j;
            black_MAX = black_NUM;
        }
    } //左半区域

    black_MAX = 0;
    for (uint8 j = 178; j > 94; --j)
    {
        black_NUM = 0;
        for (uint8 i = 0; i < 120; ++i)
        {
            if (mt9v03x_thres_image[i][j] == 0)
                black_NUM++;
        }
        if (black_NUM > black_MAX)
        {
            Yr = j;
            black_MAX = black_NUM;
        }
    } //右半区域
}

void point_clean(Point_place *pointx)
{
    pointx->row_sum = 0;
    pointx->column_sum = 0;
    pointx->nums = 0;
    pointx->row = 0;
    pointx->column = 0;
}

void point_add(Point_place *pointx, uint8 x, uint8 y)
{
    pointx->row_sum = pointx->row_sum + x;
    pointx->column_sum = pointx->column_sum + y;
    pointx->nums++;
    pointx->row = pointx->row_sum / pointx->nums;
    pointx->column = pointx->column_sum / pointx->nums;
}

/*
 * 问题：一但画面畸变导致边框误判产生黑色断线会被判定为一个点
 *       判定阈值不准确，推测判断条件没能生效，原因可能是判断位置有问题
 *       统一最后判断可能会导致数组越界
 */

void Find_Point_1(uint8 Xt, uint8 Xb, uint8 Yl, uint8 Yr)
{
    Point_place *NEXT = NULL;

    for (uint16 i = Xt + 5; i < Xb - 5; ++i)
    {
        for (uint8 j = Yl + 5; j < Yr - 5; ++j)
        {

            if (mt9v03x_thres_image[i][j] == 0) //寻找到黑点
            {
                if (total_point == 0) //该点为第一个黑点
                {
                    NEXT = (&Points_arr[0]);
                    point_add(NEXT, i, j);
                    total_point++;
                }
                else
                {
                    uint8 flag = 0; //标记该点在列表内是否存在

                    for (uint8 k = 0; k < total_point; ++k) //遍历列表里所以点坐标 判断该点是否能放入已有列表
                    {
                        if (abs(Points_arr[k].row - i) < 5 && abs(Points_arr[k].column - j) < 5) //判断条件 列表里的点与其偏差在5*5的一个矩阵内
                        {
                            NEXT = (&Points_arr[k]);
                            point_add(NEXT, i, j);
                            flag = 1;
                            break;
                        }
                    }
                    if (flag == 0) //若不存在
                    {
                        NEXT = (&Points_arr[total_point - 1]);
                        if ((*NEXT).nums < POINT_THRESHOLD) //判断上一个点内黑点值是否超过阈值 小于则删除是一个点
                        {

                            point_clean(NEXT);
                            point_add(NEXT, i, j); //将新搜索到的黑点存放入列表
                        }
                        else
                        {
                            NEXT = (&Points_arr[total_point]);
                            point_add(NEXT, i, j);
                            total_point++;
                        }
                    }
                }
            }
        }
    }
    if (Points_arr[total_point - 1].nums < POINT_THRESHOLD) //最后一个点单独判断其黑点值是否超过阈值
    {
        point_clean(&Points_arr[total_point - 1]);
        total_point--;
    }
}

void Find_Point_2(uint8 Xt, uint8 Xb, uint8 Yl, uint8 Yr)
{
    Point_place *NEXT = NULL;

    for (uint16 i = Xt + 5; i < Xb - 5; ++i)
    {
        for (uint8 j = Yl + 5; j < Yr - 5; ++j)
        {

            if (mt9v03x_thres_image[i][j] == 0) //寻找到黑点
            {
                if (total_point == 0) //该点为第一个黑点
                {
                    NEXT = (&Points_arr[0]);
                    point_add(NEXT, i, j);
                    total_point++;
                }
                else
                {
                    uint8 flag = 0; //标记该点在列表内是否存在

                    for (uint8 k = 0; k < total_point; ++k) //遍历列表里所以点坐标 判断该点是否能放入已有列表
                    {
                        if (abs(Points_arr[k].row - i) < 4 && abs(Points_arr[k].column - j) < 4) //判断条件 列表里的点与其偏差在4*4的一个矩阵内
                        {
                            NEXT = (&Points_arr[k]);
                            point_add(NEXT, i, j);
                            flag = 1;
                            break;
                        }
                    }
                    if (flag == 0) //若不存在
                    {
                        NEXT = (&Points_arr[total_point]);
                        point_add(NEXT, i, j);
                        total_point++;
                    }
                }
            }
        }
    }
}

void Target_Location_entry()
{
    uint8 remake = 0;

    while (gpio_get(SW2))
    {
        //保存按键状态
        key_up_last_status = key_up_status;
        key_down_last_status = key_down_status;
        key_left_last_status = key_left_status;
        key_right_last_status = key_right_status;
        key_enter_last_status = key_enter_status;

        //读取当前按键状态
        key_up_status = gpio_get(KEY_UP);
        key_down_status = gpio_get(KEY_DOWN);
        key_left_status = gpio_get(KEY_LEFT);
        key_right_status = gpio_get(KEY_RIGHT);
        key_enter_status = gpio_get(KEY_ENTER);

        //检测到按键按下之后  并放开置位标志位
        if (key_up_status && !key_up_last_status)
            key_up_flag = 1;
        if (key_down_status && !key_down_last_status)
            key_down_flag = 1;
        if (key_right_status && !key_right_last_status)
            key_right_flag = 1;
        if (key_right_status && !key_right_last_status)
            key_right_flag = 1;
        if (key_enter_status && !key_enter_last_status)
            key_enter_flag = 1;

        //标志位置位之后，可以使用标志位执行自己想要做的事件
        if (key_up_flag)
        {
            key_up_flag = 0; //使用按键之后，应该清除标志位
            thres++;
            clip_value++;
        }

        if (key_down_flag)
        {
            key_down_flag = 0; //使用按键之后，应该清除标志位
            thres--;
            clip_value--;
        }

        if (key_right_flag)
        {
            key_right_flag = 0; //使用按键之后，应该清除标志位
        }

        if (key_right_flag)
        {
            key_right_flag = 0; //使用按键之后，应该清除标志位
        }

        if (key_enter_flag)
        {
            key_enter_flag = 0; //使用按键之后，应该清除标志位
            if (MODE_TAG == 0)
                MODE_TAG = 1;
            else if (MODE_TAG == 2)
                remake = 1;
        }
        
        //在TFT上显示测试变量
        sprintf((char *)thres_str, "clip_value:%d ", clip_value);
        sprintf((char *)point1_str, "Xt:%d  Xb:%d  ", Xt, Xb);
        sprintf((char *)point2_str, "Yl:%d  Yr:%d  ", Yl, Yr);
        sprintf((char *)point3_str, "Points:%d MODE:%d", ture_points_num, MODE_TAG);
        sprintf((char *)point4_str, "%d %d %d %d %d %d", points_label[0], points_label[1], points_label[2], 
                                                         points_label[3], points_label[4], points_label[5]);
        lcd_showstr(0, 5, (int8 *)thres_str);
        lcd_showstr(0, 6, (int8 *)point1_str);
        lcd_showstr(0, 7, (int8 *)point2_str);
        lcd_showstr(0, 8, (int8 *)point3_str);
        lcd_showstr(0, 9, (int8 *)point4_str);

        if (MODE_TAG == 0)
        {
            if (mt9v03x_csi_finish_flag)
            {
                mt9v03x_csi_finish_flag = 0;
                rt_enter_critical();// 调度器上锁，上锁后将不再切换到其他线程，仅响应中断
                // 以下进入临界区
                // 临界区代码执行不会被其他线程抢占
                //二值化
                AdaptiveThreshold(mt9v03x_csi_image[0], mt9v03x_thres_image[0], MT9V03X_CSI_W, MT9V03X_CSI_H, 7, clip_value);
                //Threshold(mt9v03x_csi_image[0], mt9v03x_thres_image[0], MT9V03X_CSI_W, MT9V03X_CSI_H, 180);
                Find_Edge_1();
                rt_exit_critical();// 调度器解锁
                edge_point1_x = Xt;
                edge_point1_y = Yl;
                edge_point2_x = Xt;
                edge_point2_y = Yr;
                edge_point3_x = Xb;
                edge_point3_y = Yr;
                edge_point4_x = Xb;
                edge_point4_y = Yl;

                //画边框
                lcd_drawLine(edge_point1_y * 82 / MT9V03X_CSI_H, edge_point1_x * 128 / MT9V03X_CSI_W, edge_point2_y * 82 / MT9V03X_CSI_H, edge_point2_x * 128 / MT9V03X_CSI_W, RED);
                lcd_drawLine(edge_point2_y * 82 / MT9V03X_CSI_H, edge_point2_x * 128 / MT9V03X_CSI_W, edge_point3_y * 82 / MT9V03X_CSI_H, edge_point3_x * 128 / MT9V03X_CSI_W, RED);
                lcd_drawLine(edge_point3_y * 82 / MT9V03X_CSI_H, edge_point3_x * 128 / MT9V03X_CSI_W, edge_point4_y * 82 / MT9V03X_CSI_H, edge_point4_x * 128 / MT9V03X_CSI_W, RED);
                lcd_drawLine(edge_point4_y * 82 / MT9V03X_CSI_H, edge_point4_x * 128 / MT9V03X_CSI_W, edge_point1_y * 82 / MT9V03X_CSI_H, edge_point1_x * 128 / MT9V03X_CSI_W, RED);

                // 使用缩放显示函数，根据原始图像大小 以及设置需要显示的大小自动进行缩放或者放大显示
                lcd_displayimage032_zoom(mt9v03x_thres_image[0], MT9V03X_CSI_W, MT9V03X_CSI_H, 128, 82);
            }
        }
        else if (MODE_TAG == 1)
        {
            //画边框
            lcd_drawLine(edge_point1_y * 82 / MT9V03X_CSI_H, edge_point1_x * 128 / MT9V03X_CSI_W, edge_point2_y * 82 / MT9V03X_CSI_H, edge_point2_x * 128 / MT9V03X_CSI_W, RED);
            lcd_drawLine(edge_point2_y * 82 / MT9V03X_CSI_H, edge_point2_x * 128 / MT9V03X_CSI_W, edge_point3_y * 82 / MT9V03X_CSI_H, edge_point3_x * 128 / MT9V03X_CSI_W, RED);
            lcd_drawLine(edge_point3_y * 82 / MT9V03X_CSI_H, edge_point3_x * 128 / MT9V03X_CSI_W, edge_point4_y * 82 / MT9V03X_CSI_H, edge_point4_x * 128 / MT9V03X_CSI_W, RED);
            lcd_drawLine(edge_point4_y * 82 / MT9V03X_CSI_H, edge_point4_x * 128 / MT9V03X_CSI_W, edge_point1_y * 82 / MT9V03X_CSI_H, edge_point1_x * 128 / MT9V03X_CSI_W, RED);

            Find_Point_2(Xt, Xb, Yl, Yr);
            ture_points_num = 0;
            points_map[0][0] = Yr;
            points_map[0][1] = Xb;
            for (int i = 0; i < total_point; ++i) //消除噪点
            {
                if (Points_arr[i].nums > POINT_THRESHOLD)
                {
                    ture_points_num++;
                    points_map[ture_points_num][0] = Points_arr[i].column;
                    points_map[ture_points_num][1] = Points_arr[i].row;
                }
            }
            find_road(points_map, points_label); //路径规划
            //画地图
            for (int i = 0; i <= ture_points_num; ++i)
            { //画坐标点
                lcd_drawLine(points_map[i][0] * 82 / MT9V03X_CSI_H - 2, points_map[i][1] * 128 / MT9V03X_CSI_W,
                             points_map[i][0] * 82 / MT9V03X_CSI_H + 2, points_map[i][1] * 128 / MT9V03X_CSI_W, RED);
                lcd_drawLine(points_map[i][0] * 82 / MT9V03X_CSI_H, points_map[i][1] * 128 / MT9V03X_CSI_W - 2,
                             points_map[i][0] * 82 / MT9V03X_CSI_H, points_map[i][1] * 128 / MT9V03X_CSI_W + 2, RED);
                if (i < ture_points_num)
                {
                    lcd_drawLine(points_map[points_label[i]][0] * 82 / MT9V03X_CSI_H, points_map[points_label[i]][1] * 128 / MT9V03X_CSI_W,
                                 points_map[points_label[i + 1]][0] * 82 / MT9V03X_CSI_H, points_map[points_label[i + 1]][1] * 128 / MT9V03X_CSI_W, BLUE);
                } //画路径
            }    
            map_change(points_map, true_map, ture_points_num, points_label, Yr, Xb);//转换坐标系
            map_calculate(true_map, turn_angle, map_gap, ture_points_num, 
                         (double)(Yr - Yl) / 100.0f, (double)(Xb - Xt) / 100.0f);
            //            for(int i = 0; i < MT9V03X_CSI_H; ++i) {
            //              for(int j = 0; j < MT9V03X_CSI_W; ++j)
            //                PRINTF("%u  ", mt9v03x_thres_image[i][j] / 255);
            //              PRINTF("\n");
            //            }
            MODE_TAG = 2;
        }
        else if (MODE_TAG == 2)
        {
            if (remake)
            {
                Point_place *NEXT = NULL;

                for (; total_point > 0; --total_point)
                {
                    NEXT = &Points_arr[total_point - 1];
                    point_clean(NEXT);
                    points_label[total_point - 1] = 0;
                    true_map[total_point - 1][0] = 0;
                    true_map[total_point - 1][1] = 0;
                    turn_angle[total_point - 1] = 0;
                    map_gap[total_point - 1] = 0;
                }

                MODE_TAG = 0;
                remake = 0;
            }
        }
    }
    
    turn_angle[0] = turn_angle[0] - 5.0f;
    turn_angle[1] = turn_angle[1] - 4.0f;
    
    rt_mb_send(buzzer_mailbox, 233);
    lcd_clear(WHITE);

    return;
}

void Target_Location_Init(void)
{
    rt_thread_t tid;

    //初始化
    lcd_init();
    mt9v03x_csi_init(); //初始化摄像头 使用CSI接口
    gpio_init(KEY_UP, GPI, 0, GPIO_PIN_CONFIG);
    gpio_init(KEY_DOWN, GPI, 0, GPIO_PIN_CONFIG);
    gpio_init(KEY_LEFT, GPI, 0, GPIO_PIN_CONFIG);
    gpio_init(KEY_RIGHT, GPI, 0, GPIO_PIN_CONFIG);
    gpio_init(KEY_ENTER, GPI, 0, GPIO_PIN_CONFIG);
    gpio_init(SW2, GPI, 0, GPIO_PIN_CONFIG);

    //创建寻找坐标线程 优先级设置为8
    tid = rt_thread_create("Target_location", Target_Location_entry, RT_NULL, 4096, 8, 100);
    //优先级比主线程高，刚开始就执行让其卡死在此线程，找到坐标后改线程return回收

    //启动寻找坐标线程
    if (RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}
