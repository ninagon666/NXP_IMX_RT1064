#include "target_location.h"
#include <stdio.h>

//定义按键引脚
#define KEY1 C25
#define KEY2 C26
#define KEY3 C3
#define KEY4 C4

//定义拨码开关引脚
#define SW2 D27

//图像像素查找表
/*************
 * 188 * 120 pixels
 * (93,59)为中心
 * ////y轴坐标与标号相同时
 * 仅记录x轴
 *************/
//左上
// uint8 left_top_x[] = {};    i   0 - 59
const uint8 left_top_y[60] = {0, 1, 3, 4, 6, 7, 9, 10, 12, 14, 15,
                              17, 18, 20, 21, 23, 25, 26, 28, 29, 31,
                              32, 34, 36, 37, 39, 40, 42, 43, 45, 47,
                              48, 50, 51, 53, 54, 56, 57, 59, 61, 62,
                              64, 65, 67, 68, 70, 72, 73, 75, 76, 78,
                              79, 81, 83, 84, 86, 87, 89, 90, 92};
//右上
// uint8 right_top_x[] = {};    i   0 - 59
const uint8 right_top_y[60] = {186, 184, 183, 181, 180, 178, 177, 175, 173, 172,
                               170, 169, 167, 166, 164, 162, 161, 159, 158, 156,
                               155, 153, 151, 150, 148, 147, 145, 144, 142, 141,
                               139, 137, 136, 134, 133, 131, 130, 128, 126, 125,
                               123, 122, 120, 119, 117, 115, 114, 112, 111, 109,
                               108, 106, 104, 103, 101, 100, 98, 97, 95, 94};
//左下
// uint8 left_bottom_x[] = {}; 120 - i  i   0 - 59
const uint8 left_bottom_y[60] = {0, 1, 3, 4, 6, 7, 9, 10, 12, 14, 15,
                                 17, 18, 20, 21, 23, 25, 26, 28, 29, 31,
                                 32, 34, 36, 37, 39, 40, 42, 43, 45, 47,
                                 48, 50, 51, 53, 54, 56, 57, 59, 61, 62,
                                 64, 65, 67, 68, 70, 72, 73, 75, 76, 78,
                                 79, 81, 83, 84, 86, 87, 89, 90, 92};
//右下
// uint8 right_bottom_x[] = {}; 120 - i  i   0 - 59
const uint8 right_bottom_y[60] = {186, 184, 183, 181, 180, 178, 177, 175, 173, 172,
                                  170, 169, 167, 166, 164, 162, 161, 159, 158, 156,
                                  155, 153, 151, 150, 148, 147, 145, 144, 142, 141,
                                  139, 137, 136, 134, 133, 131, 130, 128, 126, 125,
                                  123, 122, 120, 119, 117, 115, 114, 112, 111, 109,
                                  108, 106, 104, 103, 101, 100, 98, 97, 95, 94};
//边缘点坐标 1~4顺时针
uint8 edge_point1_x = 0, edge_point1_y = 0; //左上
uint8 edge_point2_x = 0, edge_point2_y = 0; //右上
uint8 edge_point3_x = 0, edge_point3_y = 0; //右下
uint8 edge_point4_x = 0, edge_point4_y = 0; //左下

//边缘线坐标 Xt 上边缘X坐标 Xb 下边缘X坐标 Yl 左边缘Y坐标 Yr 右边缘Y坐标
uint8 Xt = 0, Xb = 0, Yl = 0, Yr = 0;
uint8 black_MAX = 0; //记录历史扫描行黑点最大值
uint8 black_NUM = 0; //记录当前扫描行黑点数值

//开关状态变量
uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;

//上一次开关状态变量
uint8 key1_last_status;
uint8 key2_last_status;
uint8 key3_last_status;
uint8 key4_last_status;

//开关标志位
uint8 key1_flag;
uint8 key2_flag;
uint8 key3_flag;
uint8 key4_flag;

//二值化阈值 显示字符串
uint8 thres = 100, thres_str[20];
uint8 clip_value = 2;

//边框补偿
#define ROW_CLIP 5
#define COLUMN_CLIP 5

//点数阈值
#define POINT_THRESHOLD 3

//总点数
uint8 total_point = 0;

//坐标点数组
Point_place Points_arr[20] = {0};

//二值化图像指针
uint8 (*mt9v03x_thres_image)[MT9V03X_CSI_W];

// lcd显示字符串
uint8 point1_str[20], point2_str[20], point3_str[20], point4_str[20];

//目标点结构体数组
Point_place Points_str[20];

//状态标志
uint8 MODE_TAG = 0;

//固定阈值二值化
void Threshold(uint8 *sp, uint8 *tp, uint16 width, uint16 height, uint8 thres)
{
    uint32 i, j;

    for (j = 0; j < height; j++)
    {
        for (i = 0; i < width; i++)
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
    for (int y = half_block; y < height - half_block; y++)
    {
        for (int x = half_block; x < width - half_block; x++)
        {
            // 计算局部阈值
            int thres = 0;
            for (int dy = -half_block; dy <= half_block; dy++)
            {
                for (int dx = -half_block; dx <= half_block; dx++)
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

void Find_Edge_1()
{
    for (uint8 i = 10; i < 50; i++)
    {
        if (mt9v03x_thres_image[i][left_top_y[i]] == 0)
        {
            edge_point1_x = i;
            edge_point1_y = left_top_y[i];
            break;
        }
    } //左上区域

    for (uint8 i = 10; i < 50; i++)
    {
        if (mt9v03x_thres_image[i][right_top_y[i]] == 0)
        {
            edge_point2_x = i;
            edge_point2_y = right_top_y[i];
            break;
        }
    } //右上区域

    for (uint8 i = 10; i < 50; i++)
    {
        if (mt9v03x_thres_image[i][right_bottom_y[i]] == 0)
        {
            edge_point3_x = 120 - i;
            edge_point3_y = right_bottom_y[i];
            break;
        }
    } //右下区域

    for (uint8 i = 10; i < 50; i++)
    {
        if (mt9v03x_thres_image[i][left_bottom_y[i]] == 0)
        {
            edge_point4_x = 120 - i;
            edge_point4_y = left_bottom_y[i];
            break;
        }
    } //左下区域
}

void Find_Edge_2()
{
    black_MAX = 0;
    for (uint8 i = 0; i < 60; i++)
    {
        black_NUM = 0;
        for (uint8 j = 0; j < 188; j++)
        {
            if (mt9v03x_thres_image[i][j] == 0)
                black_NUM++;
        }
        if (black_NUM > black_MAX)
        {
            Xt = i;
            black_MAX = black_NUM;
        }
    } //上半区域

    black_MAX = 0;
    for (uint8 i = 120; i > 60; i--)
    {
        black_NUM = 0;
        for (uint8 j = 0; j < 188; j++)
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
    for (uint8 j = 0; j < 94; j++)
    {
        black_NUM = 0;
        for (uint8 i = 0; i < 120; i++)
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
    for (uint8 j = 188; j > 94; j--)
    {
        black_NUM = 0;
        for (uint8 i = 0; i < 120; i++)
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
    pointx->x_sum = 0;
    pointx->y_sum = 0;
    pointx->nums = 0;
    pointx->row = 0;
    pointx->column = 0;
}

void point_add(Point_place *pointx, uint8 x, uint8 y)
{
    pointx->x_sum = pointx->x_sum + x;
    pointx->y_sum = pointx->y_sum + y;
    pointx->nums++;
    pointx->row = pointx->x_sum / pointx->nums;
    pointx->column = pointx->y_sum / pointx->nums;
}

void Find_Point(uint8 Xt, uint8 Xb, uint8 Yl, uint8 Yr)
{
    Point_place *NEXT = NULL;
    for (uint16 i = Xt + 5; i < Xb - 5; i++)
    {
        for (uint8 j = Yl + 5; j < Yr - 5; j++)
        {

            if (mt9v03x_thres_image[i][j] == 0)
            {
                if (total_point == 0)
                {
                    NEXT = (&Points_arr[0]);
                    point_add(NEXT, i, j);
                    total_point++;
                }
                else
                {
                    uint8 flag = 0;

                    for (uint8 k = 0; k < total_point; k++)
                    {
                        if (abs(Points_arr[k].row - i) < 5 && abs(Points_arr[k].column - j) < 5)
                        {
                            NEXT = (&Points_arr[k]);
                            point_add(NEXT, i, j);
                            flag = 1;
                            break;
                        }
                    }
                    if (flag == 0)
                    {
                        NEXT = (&Points_arr[total_point - 1]);
                        if ((*NEXT).nums < POINT_THRESHOLD)
                        {

                            point_clean(NEXT);
                            point_add(NEXT, i, j);
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
    if (Points_arr[total_point - 1].nums < POINT_THRESHOLD)
    {
        point_clean(&Points_arr[total_point - 1]);
        total_point--;
    }
}

void Target_Location_entry()
{
    while (gpio_get(SW2))
    {
        //保存按键状态
        key1_last_status = key1_status;
        key2_last_status = key2_status;
        key3_last_status = key3_status;
        key4_last_status = key4_status;
        //读取当前按键状态
        key1_status = gpio_get(KEY1);
        key2_status = gpio_get(KEY2);
        key3_status = gpio_get(KEY3);
        key4_status = gpio_get(KEY4);

        //检测到按键按下之后  并放开置位标志位
        if (key1_status && !key1_last_status)
            key1_flag = 1;
        if (key2_status && !key2_last_status)
            key2_flag = 1;
        if (key3_status && !key3_last_status)
            key3_flag = 1;
        if (key4_status && !key4_last_status)
            key4_flag = 1;

        //标志位置位之后，可以使用标志位执行自己想要做的事件
        if (key1_flag)
        {
            key1_flag = 0; //使用按键之后，应该清除标志位
            if (MODE_TAG == 2)
                MODE_TAG = 1;
            else if (MODE_TAG == 1)
                MODE_TAG = 2;
        }

        if (key2_flag)
        {
            key2_flag = 0; //使用按键之后，应该清除标志位
            if (MODE_TAG == 0)
                MODE_TAG = 1;
            else if (MODE_TAG == 1)
                MODE_TAG = 0;
        }

        if (key3_flag)
        {
            key3_flag = 0; //使用按键之后，应该清除标志位
            thres++;
            clip_value++;
        }

        if (key4_flag)
        {
            key4_flag = 0; //使用按键之后，应该清除标志位
            thres--;
            clip_value--;
        }

        //在TFT上显示测试变量
        sprintf((char *)thres_str, "clip_value: %d", clip_value);
        sprintf((char *)point1_str, "Xt:%d Xb:%d", Xt, Xb);
        sprintf((char *)point2_str, "Yl:%d Yr:%d", Yl, Yr);
        sprintf((char *)point3_str, "Points: %d", total_point);
        sprintf((char *)point4_str, "MODE: %d", MODE_TAG);
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
                // rt_enter_critical();// 调度器上锁，上锁后将不再切换到其他线程，仅响应中断
                // 以下进入临界区
                // 临界区代码执行不会被其他线程抢占
                //二值化
                AdaptiveThreshold(mt9v03x_csi_image[0], mt9v03x_thres_image[0], MT9V03X_CSI_W, MT9V03X_CSI_H, 7, clip_value);
                Find_Edge_2();
                // rt_exit_critical();// 调度器解锁
                edge_point1_x = Xt;
                edge_point1_y = Yl;
                edge_point2_x = Xt;
                edge_point2_y = Yr;
                edge_point3_x = Xb;
                edge_point3_y = Yr;
                edge_point4_x = Xb;
                edge_point4_y = Yl;

                //
                lcd_drawLine(edge_point1_y * 82 / MT9V03X_CSI_H, edge_point1_x * 128 / MT9V03X_CSI_W, edge_point2_y * 82 / MT9V03X_CSI_H, edge_point2_x * 128 / MT9V03X_CSI_W, RED);
                lcd_drawLine(edge_point2_y * 82 / MT9V03X_CSI_H, edge_point2_x * 128 / MT9V03X_CSI_W, edge_point3_y * 82 / MT9V03X_CSI_H, edge_point3_x * 128 / MT9V03X_CSI_W, RED);
                lcd_drawLine(edge_point3_y * 82 / MT9V03X_CSI_H, edge_point3_x * 128 / MT9V03X_CSI_W, edge_point4_y * 82 / MT9V03X_CSI_H, edge_point4_x * 128 / MT9V03X_CSI_W, RED);
                lcd_drawLine(edge_point4_y * 82 / MT9V03X_CSI_H, edge_point4_x * 128 / MT9V03X_CSI_W, edge_point1_y * 82 / MT9V03X_CSI_H, edge_point1_x * 128 / MT9V03X_CSI_W, RED);
                
                // 使用缩放显示函数，根据原始图像大小 以及设置需要显示的大小自动进行缩放或者放大显示
                lcd_displayimage032_zoom(mt9v03x_thres_image[0], MT9V03X_CSI_W, MT9V03X_CSI_H, 128, 82);
            }
        }
        else if(MODE_TAG == 1)
        {
            lcd_drawLine(edge_point1_y * 82 / MT9V03X_CSI_H, edge_point1_x * 128 / MT9V03X_CSI_W, edge_point2_y * 82 / MT9V03X_CSI_H, edge_point2_x * 128 / MT9V03X_CSI_W, RED);
            lcd_drawLine(edge_point2_y * 82 / MT9V03X_CSI_H, edge_point2_x * 128 / MT9V03X_CSI_W, edge_point3_y * 82 / MT9V03X_CSI_H, edge_point3_x * 128 / MT9V03X_CSI_W, RED);
            lcd_drawLine(edge_point3_y * 82 / MT9V03X_CSI_H, edge_point3_x * 128 / MT9V03X_CSI_W, edge_point4_y * 82 / MT9V03X_CSI_H, edge_point4_x * 128 / MT9V03X_CSI_W, RED);
            lcd_drawLine(edge_point4_y * 82 / MT9V03X_CSI_H, edge_point4_x * 128 / MT9V03X_CSI_W, edge_point1_y * 82 / MT9V03X_CSI_H, edge_point1_x * 128 / MT9V03X_CSI_W, RED);
        }
        else if(MODE_TAG == 2)
        {
            Find_Point(Xt, Xb, Yl, Yr);
            for (int i = 0; i < total_point; ++i)
            {
                lcd_drawLine(Points_arr[i].column * 82 / MT9V03X_CSI_H - 2, Points_arr[i].row * 128 / MT9V03X_CSI_W, Points_arr[i].column * 82 / MT9V03X_CSI_H + 2, Points_arr[i].row * 128 / MT9V03X_CSI_W, RED);
                lcd_drawLine(Points_arr[i].column * 82 / MT9V03X_CSI_H, Points_arr[i].row * 128 / MT9V03X_CSI_W - 2, Points_arr[i].column * 82 / MT9V03X_CSI_H, Points_arr[i].row * 128 / MT9V03X_CSI_W + 2, RED);
            }
            MODE_TAG = 1;
        }
    }

    if (gpio_get(SW2) == 1)
    {
        lcd_clear(WHITE);
        return;
    }
}

void Target_Location_Init(void)
{
    rt_thread_t tid;

    //初始化
    lcd_init();
    mt9v03x_csi_init(); //初始化摄像头 使用CSI接口
    gpio_init(KEY3, GPI, 0, GPIO_PIN_CONFIG);
    gpio_init(KEY4, GPI, 0, GPIO_PIN_CONFIG);
    gpio_init(SW2, GPI, 0, GPIO_PIN_CONFIG);

    //创建寻找坐标线程 优先级设置为31
    tid = rt_thread_create("Target_location", Target_Location_entry, RT_NULL, 4096, 8, 100);
    //优先级比主线程高，刚开始就执行让其卡死在此线程，找到坐标后改线程return回收

    //启动寻找坐标线程
    if (RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}