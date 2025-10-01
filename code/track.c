//  简介:八邻域图像处理

//------------------------------------------------------------------------------------------------------------------
#include "image.h"
#include "common.h"
#include "global.h"
#include "motor.h"
#include "pid.h"
#include "zf_device_tft180.h"
#include "track.h"
#include "server.h"
// RGB565颜色定义
#define RGB565_RED 0xF800    // 红色
#define RGB565_GREEN 0x07E0  // 绿色
#define RGB565_BLUE 0x001F   // 蓝色
#define RGB565_YELLOW 0xFFE0 // 黄色
#define RGB565_PURPLE 0xF81F // 紫色
#define RGB565_CYAN 0x07FF   // 青色
#define RGB565_WHITE 0xFFFF  // 白色
#define RGB565_BLACK 0x0000  // 黑色

/**
 * @brief 最小二乘法
 * @param uint8 begin				输入起点
 * @param uint8 end					输入终点
 * @param uint8 *border				输入需要计算斜率的边界首地址
 *  @see CTest		Slope_Calculate(start, end, border);//斜率
 * @return 返回说明
 *     -<em>false</em> fail
 *     -<em>true</em> succeed
 */
float Slope_Calculate(uint8 begin, uint8 end, uint8 *border)
{
    float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
    int16 i = 0;
    float result = 0;
    static float resultlast;

    for (i = begin; i < end; i++)
    {
        xsum += i;
        ysum += border[i];
        xysum += i * (border[i]);
        x2sum += i * i;
    }
    if ((end - begin) * x2sum - xsum * xsum) // 判断除数是否为零
    {
        result = ((end - begin) * xysum - xsum * ysum) / ((end - begin) * x2sum - xsum * xsum);
        resultlast = result;
    }
    else
    {
        result = resultlast;
    }
    return result;
}

/*
函数名称：int my_abs(int value)
功能说明：求绝对值
参数说明：
函数返回：绝对值
修改时间：2022年9月8日
备    注：
example：  my_abs( x)；
 */
int my_abs(int value)
{
    if (value >= 0)
        return value;
    else
        return -value;
}

int16 limit_a_b(int16 x, int a, int b)
{
    if (x < a)
        x = a;
    if (x > b)
        x = b;
    return x;
}

/*
函数名称：int16 limit(int16 x, int16 y)
功能说明：求x,y中的最小值
参数说明：
函数返回：返回两值中的最小值
修改时间：2022年9月8日
备    注：
example：  limit( x,  y)
 */
int16 limit1(int16 x, int16 y)
{
    if (x > y)
        return y;
    else if (x < -y)
        return -y;
    else
        return x;
}

/*
函数名称：uint8 get_start_point(uint8 start_row)
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数
函数返回：找到起点返回1，未找到返回0
修改时间：2024年3月20日
备    注：优化版本 - 包含多行列搜索、起始点验证、历史信息预测
 */
uint8 start_point_l[2] = {0}; // 左边起点的x，y值
uint8 start_point_r[2] = {0}; // 右边起点的x，y值
uint8 last_valid_l[2] = {0};  // 上一帧有效左边界点
uint8 last_valid_r[2] = {0};  // 上一帧有效右边界点
uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0, l_found = 0, r_found = 0;
    uint8 search_range = 3; // 搜索行数范围
    uint8 confidence_l = 0; // 左边界点置信度
    uint8 confidence_r = 0;
    uint8 best_l_row = start_row; // 最佳左边界行
    uint8 best_r_row = start_row; // 最佳右边界行
    uint8 best_l_x = 0;           // 最佳左边界x坐标
    uint8 best_r_x = 0;           // 最佳右边界x坐标

    // 清零
    start_point_l[0] = 0; // x
    start_point_l[1] = 0; // y

    start_point_r[0] = 0; // x
    start_point_r[1] = 0; // y

    // 优先在指定行搜索，如果找不到则扩展到相邻行
    for (int8 search_offset = 0; search_offset <= search_range; search_offset++)
    {
        uint8 current_row = start_row - search_offset;

        // 确保行号有效
        if (current_row < 0)
            continue;

        // 尝试使用历史信息预测起始点位置，提高搜索效率
        uint8 left_search_start = IMAGE_WIDTH / 2;
        uint8 right_search_start = IMAGE_WIDTH / 2;

        // 如果有上一帧的有效信息，从历史位置附近开始搜索
        if (last_valid_l[0] > 0 && last_valid_l[1] > 0)
        {
            left_search_start = last_valid_l[0];
        }
        if (last_valid_r[0] > 0 && last_valid_r[1] > 0)
        {
            right_search_start = last_valid_r[0];
        }

        // 从预测位置开始向左搜索左边界
        if (!l_found)
        {
            // 先从预测位置向左搜索
            for (i = left_search_start; i > border_min; i--)
            {
                if (bin_image[current_row][i] == 255 && bin_image[current_row][i - 1] == 0)
                {
                    // 计算置信度：位置接近中心、边界清晰
                    uint8 current_confidence = 10 - (my_abs(i - IMAGE_WIDTH / 2) / 5);
                    if (current_confidence > confidence_l)
                    {
                        confidence_l = current_confidence;
                        best_l_x = i;
                        best_l_row = current_row;
                    }
                    l_found = 1;
                    break;
                }
            }

            // 如果从预测位置没找到，再从中心向左搜索
            if (!l_found)
            {
                for (i = IMAGE_WIDTH / 2; i > border_min; i--)
                {
                    if (bin_image[current_row][i] == 255 && bin_image[current_row][i - 1] == 0)
                    {
                        uint8 current_confidence = 10 - (my_abs(i - IMAGE_WIDTH / 2) / 5);
                        if (current_confidence > confidence_l)
                        {
                            confidence_l = current_confidence;
                            best_l_x = i;
                            best_l_row = current_row;
                        }
                        l_found = 1;
                        break;
                    }
                }
            }
        }

        // 从预测位置开始向右搜索右边界
        if (!r_found)
        {
            // 先从预测位置向右搜索
            for (i = right_search_start; i < border_max; i++)
            {
                if (bin_image[current_row][i] == 255 && bin_image[current_row][i + 1] == 0)
                {
                    // 计算置信度：位置接近中心、边界清晰
                    uint8 current_confidence = 10 - (my_abs(i - IMAGE_WIDTH / 2) / 5);
                    if (current_confidence > confidence_r)
                    {
                        confidence_r = current_confidence;
                        best_r_x = i;
                        best_r_row = current_row;
                    }
                    r_found = 1;
                    break;
                }
            }

            // 如果从预测位置没找到，再从中心向右搜索
            if (!r_found)
            {
                for (i = IMAGE_WIDTH / 2; i < border_max; i++)
                {
                    if (bin_image[current_row][i] == 255 && bin_image[current_row][i + 1] == 0)
                    {
                        uint8 current_confidence = 10 - (my_abs(i - IMAGE_WIDTH / 2) / 5);
                        if (current_confidence > confidence_r)
                        {
                            confidence_r = current_confidence;
                            best_r_x = i;
                            best_r_row = current_row;
                        }
                        r_found = 1;
                        break;
                    }
                }
            }
        }

        // 如果左右边界都找到了，可以提前退出搜索
        if (l_found && r_found)
            break;
    }

    // 设置找到的最佳起始点
    if (l_found)
    {
        start_point_l[0] = best_l_x;
        start_point_l[1] = best_l_row;
        // 保存有效的历史信息
        last_valid_l[0] = best_l_x;
        last_valid_l[1] = best_l_row;
    }

    if (r_found)
    {
        start_point_r[0] = best_r_x;
        start_point_r[1] = best_r_row;
        // 保存有效的历史信息
        last_valid_r[0] = best_r_x;
        last_valid_r[1] = best_r_row;
    }

    // 起始点验证：确保左右边界间距合理
    if (l_found && r_found)
    {
        uint8 width = start_point_r[0] - start_point_l[0];
        // 如果左右边界间距在合理范围内，认为找到了有效起始点
        if (width >= 30 && width <= 100)
        {
            return 1;
        }
        else
        {
            // 间距不合理，可能是噪声干扰，返回失败
            return 0;
        }
    }
    else
    {
        // printf("未找到起点\n");
        return 0;
    }
}

/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[IMAGE_WIDTH],uint16 *l_stastic, uint16 *r_stastic,
                        uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r            ：最多需要循环的次数
(*image)[IMAGE_WIDTH]    ：需要进行找点的图像数组，必须是二值图,填入数组名称即可
                       特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic                ：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic                ：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x                ：左边起点横坐标
l_start_y                ：左边起点纵坐标
r_start_x                ：右边起点横坐标
r_start_y                ：右边起点纵坐标
hightest                ：循环结束所得到的最高高度
函数返回：无
修改时间：2022年9月25日
备    注：
example：
    search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */
#define USE_num IMAGE_HEIGHT * 3 // 定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

// 存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = {{0}}; // 左线
uint16 points_r[(uint16)USE_num][2] = {{0}}; // 右线
uint16 dir_r[(uint16)USE_num] = {0};         // 用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = {0};         // 用来存储左边生长方向
uint16 data_stastics_l = 0;                  // 统计左边找到点的个数
uint16 data_stastics_r = 0;                  // 统计右边找到点的个数
uint8 hightest = 0;                          // 最高点
void search_l_r(uint16 break_flag, uint8 (*image)[IMAGE_WIDTH], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8 *hightest)
{

    uint8 i = 0, j = 0;

    // 左边变量
    uint8 search_filds_l[8][2] = {{0}};
    uint8 index_l = 0;
    uint8 temp_l[8][2] = {{0}};
    uint8 center_point_l[2] = {0};
    uint16 l_data_statics; // 统计左边

    // 定义八个邻域
    static int8 seeds_l[8][2] = {
        {0, 1},
        {-1, 1},
        {-1, 0},
        {-1, -1},
        {0, -1},
        {1, -1},
        {1, 0},
        {1, 1},
    };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},         {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    // 这个是顺时针

    // 右边变量
    uint8 search_filds_r[8][2] = {{0}};
    uint8 center_point_r[2] = {0}; // 中心坐标点
    uint8 index_r = 0;             // 索引下标
    uint8 temp_r[8][2] = {{0}};
    uint16 r_data_statics; // 统计右边
    // 定义八个邻域
    static int8 seeds_r[8][2] = {
        {0, 1},
        {1, 1},
        {1, 0},
        {1, -1},
        {0, -1},
        {-1, -1},
        {-1, 0},
        {-1, 1},
    };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},         {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    // 这个是逆时针

    l_data_statics = *l_stastic; // 统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = *r_stastic; // 统计找到了多少个点，方便后续把点全部画出来

    // 第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x; // x
    center_point_l[1] = l_start_y; // y
    center_point_r[0] = r_start_x; // x
    center_point_r[1] = r_start_y; // y

    // 开启邻域循环
    while (break_flag--)
    {

        // 左边
        for (i = 0; i < 8; i++) // 传递8F坐标
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0]; // x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1]; // y
        }
        // 中心坐标点填充到已经找到的点内
        points_l[l_data_statics][0] = center_point_l[0]; // x
        points_l[l_data_statics][1] = center_point_l[1]; // y
        l_data_statics++;                                // 索引加一

        // 右边
        for (i = 0; i < 8; i++) // 传递8F坐标
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0]; // x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1]; // y
        }
        // 中心坐标点填充到已经找到的点内
        points_r[r_data_statics][0] = center_point_r[0]; // x
        points_r[r_data_statics][1] = center_point_r[1]; // y

        index_l = 0; // 先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0; // 先清零，后使用
            temp_l[i][1] = 0; // 先清零，后使用
        }

        // 左边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0 && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i); // 记录生长方向
            }

            if (index_l)
            {
                // 更新坐标点
                center_point_l[0] = temp_l[0][0]; // x
                center_point_l[1] = temp_l[0][1]; // y
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0]; // x
                        center_point_l[1] = temp_l[j][1]; // y
                    }
                }
            }
        }
        if ((points_r[r_data_statics][0] == points_r[r_data_statics - 1][0] && points_r[r_data_statics][0] == points_r[r_data_statics - 2][0] && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1]) || (points_l[l_data_statics - 1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics - 1][0] == points_l[l_data_statics - 3][0] && points_l[l_data_statics - 1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics - 1][1] == points_l[l_data_statics - 3][1]))
        {
            // printf("三次进入同一个点，退出\n");
            break;
        }
        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2 && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2))
        {
            // printf("\n左右相遇退出\n");
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1; // 取出最高点
            // printf("\n在y=%d处退出\n",*hightest);
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
            // printf("\n如果左边比右边高了，左边等待右边\n");
            continue; // 如果左边比右边高了，左边等待右边
        }
        if (dir_l[l_data_statics - 1] == 7 && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1])) // 左边比右边高且已经向下生长了
        {
            // printf("\n左边开始向下了，等待右边，等待中... \n");
            center_point_l[0] = points_l[l_data_statics - 1][0]; // x
            center_point_l[1] = points_l[l_data_statics - 1][1]; // y
            l_data_statics--;
        }
        r_data_statics++; // 索引加一

        index_r = 0; // 先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0; // 先清零，后使用
            temp_r[i][1] = 0; // 先清零，后使用
        }

        // 右边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0 && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                temp_r[index_r][0] = search_filds_r[i][0];
                temp_r[index_r][1] = search_filds_r[i][1];

                // 计算候选点权重
                // 1. 基于历史方向的权重
                uint8 dir_diff = my_abs(i - last_dir_r);
                dir_diff = dir_diff > 4 ? 8 - dir_diff : dir_diff; // 计算最小角度差
                temp_r_weight[index_r] = dir_weight_r[i] * (5 - dir_diff);

                // 2. 优先选择上方的点（高y值）
                if (temp_r[index_r][1] > center_point_r[1])
                {
                    temp_r_weight[index_r] += 3; // 给上方的点额外权重
                }

                // 3. 避免过于靠近中线
                uint8 mid_line = (IMAGE_WIDTH) / 2;
                if (my_abs(temp_r[index_r][0] - mid_line) > 20)
                {
                    temp_r_weight[index_r] += 2; // 远离中线的点更可能是边界
                }

                index_r++;
                dir_r[r_data_statics - 1] = i; // 记录生长方向
            }
        }

        // 根据权重选择最佳点
        if (index_r)
        {
            // 找到权重最大的点
            uint8 best_index = 0;
            for (j = 1; j < index_r; j++)
            {
                if (temp_r_weight[j] > temp_r_weight[best_index])
                {
                    best_index = j;
                }
                else if (temp_r_weight[j] == temp_r_weight[best_index])
                {
                    // 权重相同时，优先选择上方的点
                    if (temp_r[j][1] > temp_r[best_index][1])
                    {
                        best_index = j;
                    }
                }
            }

            // 更新坐标点
            center_point_r[0] = temp_r[best_index][0]; // x
            center_point_r[1] = temp_r[best_index][1]; // y

            // 更新方向统计和权重表
            uint8 current_dir = dir_r[r_data_statics - 1];
            if (current_dir == last_dir_r)
            {
                same_dir_count_r++;
                if (same_dir_count_r > MAX_CONSECUTIVE_SAME_DIRECTION)
                {
                    // 连续多次同一方向，增加相邻方向的权重，避免陷入局部最优
                    dir_weight_r[(current_dir + 1) & 7] += 1;
                    dir_weight_r[(current_dir - 1) & 7] += 1;
                }
            }
            else
            {
                same_dir_count_r = 1;
                // 新方向，增加其权重
                dir_weight_r[current_dir] += 1;
                // 衰减旧方向的权重
                if (dir_weight_r[last_dir_r] > 1)
                {
                    dir_weight_r[last_dir_r] -= 1;
                }
            }
            last_dir_r = current_dir;
        }
    }

    // 取出循环次数
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}

/*
函数名称：void get_left(uint16 total_L)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_L    ：找到的点的总数
函数返回：无
修改时间：2024年3月20日
备    注：优化版本 - 包含平滑处理、缺失点补全和异常点过滤
 */
#define SMOOTH_WINDOW_SIZE 3 // 平滑窗口大小
#define MAX_JUMP_THRESHOLD 5 // 相邻行边界点的最大跳跃阈值

uint8 l_border[IMAGE_HEIGHT];    // 左线数组
uint8 r_border[IMAGE_HEIGHT];    // 右线数组
uint8 center_line[IMAGE_HEIGHT]; // 中线数组
void get_left(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    uint8 valid_points[IMAGE_HEIGHT] = {0}; // 记录每行是否有有效点

    // 初始化
    for (i = 0; i < IMAGE_HEIGHT; i++)
    {
        l_border[i] = border_min;
        valid_points[i] = 0;
    }

    // 第一次遍历：记录所有有效点
    for (j = 0; j < total_L; j++)
    {
        if (points_l[j][1] >= 0 && points_l[j][1] < IMAGE_HEIGHT)
        {
            // 每行可能有多个点，取中间位置或最靠近上一行的点
            if (!valid_points[points_l[j][1]] ||
                (valid_points[points_l[j][1]] && points_l[j][1] > h))
            {
                l_border[points_l[j][1]] = points_l[j][0] + 1;
                valid_points[points_l[j][1]] = 1;
            }
        }
    }

    // 第二次遍历：补全缺失的点（基于线性插值）
    uint8 last_valid_y = IMAGE_HEIGHT - 1;
    uint8 next_valid_y = 0;

    // 从底部开始查找第一个有效点
    for (h = IMAGE_HEIGHT - 1; h >= 0; h--)
    {
        if (valid_points[h])
        {
            last_valid_y = h;
            break;
        }
    }

    // 向上补全缺失点
    for (h = last_valid_y - 1; h >= 0; h--)
    {
        if (valid_points[h])
        {
            // 找到下一个有效点，进行线性插值
            next_valid_y = h;

            if (last_valid_y - next_valid_y > 1)
            {
                float slope = (float)(l_border[next_valid_y] - l_border[last_valid_y]) /
                              (float)(next_valid_y - last_valid_y);

                for (uint8 k = next_valid_y + 1; k < last_valid_y; k++)
                {
                    l_border[k] = l_border[last_valid_y] + slope * (k - last_valid_y);
                    // 边界限幅
                    if (l_border[k] < border_min)
                        l_border[k] = border_min;
                    if (l_border[k] > border_max)
                        l_border[k] = border_max;
                }
            }

            last_valid_y = next_valid_y;
        }
    }

    // 第三次遍历：平滑处理
    uint8 temp_border[IMAGE_HEIGHT];
    for (i = 0; i < IMAGE_HEIGHT; i++)
    {
        temp_border[i] = l_border[i];
    }

    for (i = SMOOTH_WINDOW_SIZE; i < IMAGE_HEIGHT - SMOOTH_WINDOW_SIZE; i++)
    {
        // 中值滤波结合均值滤波，提高鲁棒性
        uint8 window[SMOOTH_WINDOW_SIZE * 2 + 1];
        for (j = 0; j < SMOOTH_WINDOW_SIZE * 2 + 1; j++)
        {
            uint8 idx = i - SMOOTH_WINDOW_SIZE + j;
            window[j] = temp_border[idx];
        }

        // 冒泡排序计算中值
        for (j = 0; j < SMOOTH_WINDOW_SIZE * 2; j++)
        {
            for (uint8 k = j + 1; k < SMOOTH_WINDOW_SIZE * 2 + 1; k++)
            {
                if (window[j] > window[k])
                {
                    uint8 temp = window[j];
                    window[j] = window[k];
                    window[k] = temp;
                }
            }
        }

        // 取中值作为滤波结果
        l_border[i] = window[SMOOTH_WINDOW_SIZE];
    }

    // 第四次遍历：检测并修正异常跳跃
    for (i = 1; i < IMAGE_HEIGHT; i++)
    {
        if (my_abs(l_border[i] - l_border[i - 1]) > MAX_JUMP_THRESHOLD)
        {
            // 跳跃过大，认为是异常点，使用前一行的值或平滑值
            if (i > 1)
            {
                l_border[i] = (l_border[i - 1] + l_border[i - 2]) / 2;
            }
            else
            {
                l_border[i] = l_border[i - 1];
            }
        }
    }
}
/*
函数名称：void get_right(uint16 total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2024年3月20日
备    注：优化版本 - 包含平滑处理、缺失点补全和异常点过滤
 */
void get_right(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    uint8 valid_points[IMAGE_HEIGHT] = {0}; // 记录每行是否有有效点

    // 初始化
    for (i = 0; i < IMAGE_HEIGHT; i++)
    {
        r_border[i] = border_max; // 右边线初始化放到最右边
        valid_points[i] = 0;
    }

    // 第一次遍历：记录所有有效点
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] >= 0 && points_r[j][1] < IMAGE_HEIGHT)
        {
            // 每行可能有多个点，取中间位置或最靠近上一行的点
            if (!valid_points[points_r[j][1]] ||
                (valid_points[points_r[j][1]] && points_r[j][1] > h))
            {
                r_border[points_r[j][1]] = points_r[j][0] - 1;
                valid_points[points_r[j][1]] = 1;
            }
        }
    }

    // 第二次遍历：补全缺失的点（基于线性插值）
    uint8 last_valid_y = IMAGE_HEIGHT - 1;
    uint8 next_valid_y = 0;

    // 从底部开始查找第一个有效点
    for (h = IMAGE_HEIGHT - 1; h >= 0; h--)
    {
        if (valid_points[h])
        {
            last_valid_y = h;
            break;
        }
    }

    // 向上补全缺失点
    for (h = last_valid_y - 1; h >= 0; h--)
    {
        if (valid_points[h])
        {
            // 找到下一个有效点，进行线性插值
            next_valid_y = h;

            if (last_valid_y - next_valid_y > 1)
            {
                float slope = (float)(r_border[next_valid_y] - r_border[last_valid_y]) /
                              (float)(next_valid_y - last_valid_y);

                for (uint8 k = next_valid_y + 1; k < last_valid_y; k++)
                {
                    r_border[k] = r_border[last_valid_y] + slope * (k - last_valid_y);
                    // 边界限幅
                    if (r_border[k] < border_min)
                        r_border[k] = border_min;
                    if (r_border[k] > border_max)
                        r_border[k] = border_max;
                }
            }

            last_valid_y = next_valid_y;
        }
    }

    // 第三次遍历：平滑处理
    uint8 temp_border[IMAGE_HEIGHT];
    for (i = 0; i < IMAGE_HEIGHT; i++)
    {
        temp_border[i] = r_border[i];
    }

    for (i = SMOOTH_WINDOW_SIZE; i < IMAGE_HEIGHT - SMOOTH_WINDOW_SIZE; i++)
    {
        // 中值滤波结合均值滤波，提高鲁棒性
        uint8 window[SMOOTH_WINDOW_SIZE * 2 + 1];
        for (j = 0; j < SMOOTH_WINDOW_SIZE * 2 + 1; j++)
        {
            uint8 idx = i - SMOOTH_WINDOW_SIZE + j;
            window[j] = temp_border[idx];
        }

        // 冒泡排序计算中值
        for (j = 0; j < SMOOTH_WINDOW_SIZE * 2; j++)
        {
            for (uint8 k = j + 1; k < SMOOTH_WINDOW_SIZE * 2 + 1; k++)
            {
                if (window[j] > window[k])
                {
                    uint8 temp = window[j];
                    window[j] = window[k];
                    window[k] = temp;
                }
            }
        }

        // 取中值作为滤波结果
        r_border[i] = window[SMOOTH_WINDOW_SIZE];
    }

    // 第四次遍历：检测并修正异常跳跃
    for (i = 1; i < IMAGE_HEIGHT; i++)
    {
        if (my_abs(r_border[i] - r_border[i - 1]) > MAX_JUMP_THRESHOLD)
        {
            // 跳跃过大，认为是异常点，使用前一行的值或平滑值
            if (i > 1)
            {
                r_border[i] = (r_border[i - 1] + r_border[i - 2]) / 2;
            }
            else
            {
                r_border[i] = r_border[i - 1];
            }
        }
    }
}

// 定义膨胀和腐蚀的阈值区间
#define threshold_max 255 * 5                      // 此参数可根据自己的需求调节
#define threshold_min 255 * 2                      // 此参数可根据自己的需求调节
void image_filter(uint8 (*bin_image)[IMAGE_WIDTH]) // 形态学滤波，简单来说就是膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;

    for (i = 1; i < IMAGE_HEIGHT - 1; i++)
    {
        for (j = 1; j < (IMAGE_WIDTH - 1); j++)
        {
            // 统计八个方向的像素值
            num =
                bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1] + bin_image[i][j - 1] + bin_image[i][j + 1] + bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];

            if (num >= threshold_max && bin_image[i][j] == 0)
            {
                bin_image[i][j] = 255; // 白  可以搞成宏定义，方便更改
            }
            if (num <= threshold_min && bin_image[i][j] == 255)
            {
                bin_image[i][j] = 0; // 黑
            }
        }
    }
}

/*
函数名称：void image_draw_rectan(uint8(*image)[IMAGE_WIDTH])
功能说明：给图像画一个黑框
参数说明：uint8(*image)[IMAGE_WIDTH]    图像首地址
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_draw_rectan(bin_image);
 */
void image_draw_rectan(uint8 (*image)[IMAGE_WIDTH])
{
    uint8 i = 0;
    for (i = 0; i < IMAGE_HEIGHT; i++)
    {
        image[i][0] = 0;
        image[i][1] = 0;
        image[i][IMAGE_WIDTH - 1] = 0;
        image[i][IMAGE_WIDTH - 2] = 0;
    }
    for (i = 0; i < IMAGE_WIDTH; i++)
    {
        image[0][i] = 0;
        image[1][i] = 0;
        // image[IMAGE_HEIGHT-1][i] = 0;
    }
}

// 路径偏差值
int16 track_deviation = 0;
// 线检测标志
uint8 line_detected = 0;

/*
函数名称：void calculate_deviation(void)
功能说明：计算路径偏差值，用于控制小车行驶
参数说明：无
函数返回：无
 */
void calculate_deviation(void)
{
    //    uint8 i;
    uint16 middle_row = IMAGE_HEIGHT * 3 / 4; // 取图像下半部分的中间行作为控制行
    int16 target_center = IMAGE_WIDTH / 2;    // 目标中点

    // 检查是否检测到有效的路径
    if (l_border[middle_row] > border_min && r_border[middle_row] < border_max)
    {
        // 计算当前中线位置
        center_line[middle_row] = (l_border[middle_row] + r_border[middle_row]) >> 1;

        // 计算偏差值
        track_deviation = center_line[middle_row] - target_center;

        // 限制偏差值范围
        track_deviation = limit_a_b(track_deviation, -30, 30);

        line_detected = 1;
    }
    else
    {
        // 未检测到有效路径
        track_deviation = 0;
        line_detected = 0;
    }
}

// /*
// 函数名称：void track_control(void)
// 功能说明：根据路径偏差值控制小车行驶
// 参数说明：无
// 函数返回：无
//  */
// void track_control(void)
// {
//     if (line_detected)
//     {
//         // 根据偏差值调整左右轮速度
//         int16 left_adjust = track_deviation * 0.8;   // 左轮调整系数
//         int16 right_adjust = -track_deviation * 0.8; // 右轮调整系数

//         // 应用调整并限制范围
//         target_left = limit_a_b(speedleft + left_adjust, 0, 100);
//         target_right = limit_a_b(speedright + right_adjust, 0, 100);
//     }
//     else
//     {
//         // 未检测到路径，减速并尝试寻找
//         target_left = limit_a_b(speedleft * 0.6, 0, 100);
//         target_right = limit_a_b(speedright * 0.6, 0, 100);
//     }
// }

/*
函数名称：void track_process(void)
功能说明：巡线处理主函数
参数说明：无
函数返回：无
 */
void track_process(void)
{
    uint16 i;
    uint8 hightest = 0; // 定义一个最高行，tip：这里的最高指的是y值的最小

    // 二值化图像已经在cpu0_main.c中通过otsuThreshold函数处理

    // 应用逆透视变换（如果项目中已实现）
    // Inv_Perspective_Map_Image(bin_image, map_image);

    // 滤波和预处理
    image_filter(bin_image);
    image_draw_rectan(bin_image);
    // 清零计数
    data_stastics_l = 0;
    data_stastics_r = 0;

    if (get_start_point(IMAGE_HEIGHT - 2)) // 找到起点了，再执行八领域，没找到就一直找
    {
        // printf("正在开始八领域\n");
        search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0],
                   start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
        // printf("八邻域已结束\n");

        // 从爬取的边界线内提取边线，这个才是最终有用的边线
        get_left(data_stastics_l);
        get_right(data_stastics_r);

        // 执行十字补线处理
        cross_fill(bin_image, l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r);

        // 计算中线
        for (i = hightest; i < IMAGE_HEIGHT - 1; i++)
        {
            if (l_border[i] > border_min && r_border[i] < border_max)
            {
                center_line[i] = (l_border[i] + r_border[i]) >> 1; // 求中线
            }
        }
    }
    else
    {
        // 未找到起点
        line_detected = 0;
        track_deviation = 0;
    }

    // 显示处理后的图像
    tft180_displayimage03x((const uint8 *)bin_image, 160, 128);

    for (uint16 i = 0; i < data_stastics_l; i++)
    {
        // 使用与tft180_show_gray_image相同的坐标缩放逻辑，确保与二值化图像显示对齐
        uint16 x = limit_a_b(((points_l[i][0] + 2) * 160) / 188, 0, 159);
        uint16 y = limit_a_b((points_l[i][1] * 128) / 120, 0, 127);
        tft180_draw_point(x, y, RGB565_BLUE); // 显示左边起点
    }
    for (i = 0; i < data_stastics_r; i++)
    {
        // 使用与tft180_show_gray_image相同的坐标缩放逻辑，确保与二值化图像显示对齐
        uint16 x = limit_a_b(((points_r[i][0] - 2) * 160) / 188, 0, 159);
        uint16 y = limit_a_b((points_r[i][1] * 128) / 120, 0, 127);
        tft180_draw_point(x, y, RGB565_RED); // 显示右边起点
    }

    // 显示中线和左右边界，限制在显示屏高度范围内(0-127)
    for (i = hightest; i < 128; i++)
    {
        center_line[i] = (l_border[i] + r_border[i]) >> 1; // 求中线
        // 求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
        // 当然也有多组边线的找法，但是个人感觉很繁琐，不建议

        // 使用与tft180_show_gray_image相同的坐标缩放逻辑，确保与二值化图像显示对齐
        // 注意：原始图像大小为188×120，显示大小为160×128
        uint16 center_x = limit_a_b((center_line[i] * 160) / 188, 0, 159);
        uint16 l_x = limit_a_b((l_border[i] * 160) / 188, 0, 159);
        uint16 r_x = limit_a_b((r_border[i] * 160) / 188, 0, 159);
        uint16 y = limit_a_b((i * 128) / 120, 0, 127);
        // 根据环岛状态设置中线颜色
        if (circle_state != CIRCLE_NONE)
        {
            tft180_draw_point(center_x, y, RGB565_PURPLE); // 环岛内显示紫色中线
        }
        else
        {
            tft180_draw_point(center_x, y, RGB565_CYAN); // 非环岛显示青色中线
        }
        tft180_draw_point(l_x, y, RGB565_GREEN); // 显示左边线
        tft180_draw_point(r_x, y, RGB565_GREEN); // 显示右边线
    }
}

// 环岛处理相关代码

// 环岛相关全局变量
enum CIRCLE_STATE circle_state = CIRCLE_NONE; // 当前环岛状态
uint8 circle_entry_count = 0;                 // 进入环岛计数
uint8 circle_exit_count = 0;                  // 离开环岛计数
uint8 circle_exit_detected = 0;               // 出口检测标志
uint8 flag_r_cir = 0;                         // 右环岛标志
uint8 flag_l_cir = 0;                         // 左环岛标志
uint8_t cir_stage = 0;                        // 环岛阶段标志
uint8_t cir_x = 0, cir_y = 0;                 // 环岛中心点坐标
uint8 target_speed = 50;

// 十字相关全局变量
int16_t length = 0;                     // 赛道长度
int r_lose_judge[IMAGE_HEIGHT] = {0};   // 右边界丢线判断
int l_lose_judge[IMAGE_HEIGHT] = {0};   // 左边界丢线判断
int16_t r_lose_num = 0, l_lose_num = 0; // 左右边界丢线计数
uint8_t flag_cross = 0;                 // 十字标志位
uint8_t cross_stage = 0;                // 十字阶段
uint8_t cross_timer = 0;                // 十字定时器
uint8 lose_num = 0, nolose_num = 0;     // 丢线和非丢线计数

/**
 * @brief 判断右环岛
 * @return uint8 1表示检测到右环岛，0表示未检测到
 */
uint8 Cir1_judge(void)
{
    uint8 i, j;
    uint8 count = 0;

    // 检测图像上部区域是否有右环岛特征
    for (i = 5; i < 20; i++)
    {
        for (j = IMAGE_WIDTH / 2; j < IMAGE_WIDTH - 5; j++)
        {
            if (bin_image[i][j] == 0 && bin_image[i + 1][j] == 255 && bin_image[i - 1][j] == 255)
            {
                count++;
            }
        }
    }

    if (count > 10) // 阈值可调整
    {
        return 1;
    }
    return 0;
}

/**
 * @brief 判断左环岛
 * @return uint8 1表示检测到左环岛，0表示未检测到
 */
uint8 Cir2_judge(void)
{
    uint8 i, j;
    uint8 count = 0;

    // 检测图像上部区域是否有左环岛特征
    for (i = 5; i < 20; i++)
    {
        for (j = 5; j < IMAGE_WIDTH / 2; j++)
        {
            if (bin_image[i][j] == 0 && bin_image[i + 1][j] == 255 && bin_image[i - 1][j] == 255)
            {
                count++;
            }
        }
    }

    if (count > 10) // 阈值可调整
    {
        return 1;
    }
    return 0;
}

/**
 * @brief 计算环岛中心点
 */
void cir_point(void)
{
    uint8_t i = 0;
    float sum_x = 0, sum_y = 0;
    uint8_t num = 0;

    for (i = 0; i < IMAGE_HEIGHT - 30; i++)
    {
        if (l_border[i] != border_min && r_border[i] != border_max)
        {
            sum_x = sum_x + (l_border[i] + r_border[i]) / 2;
            sum_y = sum_y + i;
            num++;
        }
    }

    if (num != 0)
    {
        cir_x = (sum_x) / (num);
        cir_y = (sum_y) / (num);
    }
}

/**
 * @brief 环岛补线处理
 */
void circle_line_complement(void)
{
    uint8 i = 0;

    for (i = 0; i < IMAGE_HEIGHT - 10; i++)
    {
        // 当左边界丢失时，根据右边界和预设宽度推算左边界
        if (l_border[i] <= border_min)
        {
            l_border[i] = r_border[i] - 60; // 预设宽度可调整
            if (l_border[i] < border_min)
                l_border[i] = border_min;
        }

        // 当右边界丢失时，根据左边界和预设宽度推算右边界
        if (r_border[i] >= border_max)
        {
            r_border[i] = l_border[i] + 60; // 预设宽度可调整
            if (r_border[i] > border_max)
                r_border[i] = border_max;
        }
    }
}

/**
 * @brief 右环岛处理
 */
void Cir_r_handle(void)
{
    uint8 i = 0;

    // 7阶段右环岛状态机
    switch (cir_stage)
    {
    case 0:
        // 初始阶段：检测到右环岛
        if (Cir1_judge())
        {
            flag_r_cir = 1;
            cir_stage = 1;
        }
        break;

    case 1:
        // 进入环岛：减速并调整方向
        Target_Speed_Control(35, 45); // 轻微右偏
        if (circle_state == CIRCLE_ON_TRACK)
            cir_stage = 2;
        break;

    case 2:
        // 环岛内稳定行驶
        Target_Speed_Control(40, 50);
        if (detect_circle_exit())
            cir_stage = 3;
        break;

    case 3:
        // 检测到出口：准备转向
        Target_Speed_Control(35, 45);
        cir_stage = 4;
        break;

    case 4:
        // 出口转向
        Target_Speed_Control(30, 50);
        if (!detect_circle_exit())
            cir_stage = 5;
        break;

    case 5:
        // 离开环岛：恢复直线行驶
        Target_Speed_Control(45, 45);
        cir_stage = 6;
        break;

    case 6:
        // 重置状态
        flag_r_cir = 0;
        cir_stage = 0;
        break;
    }
}

/**
 * @brief 左环岛处理
 */
void Cir_l_handle(void)
{
    uint8 i = 0;

    // 7阶段左环岛状态机
    switch (cir_stage)
    {
    case 0:
        // 初始阶段：检测到左环岛
        if (Cir2_judge())
        {
            flag_l_cir = 1;
            cir_stage = 1;
        }
        break;

    case 1:
        // 进入环岛：减速并调整方向
        Target_Speed_Control(45, 35); // 轻微左偏
        if (circle_state == CIRCLE_ON_TRACK)
            cir_stage = 2;
        break;

    case 2:
        // 环岛内稳定行驶
        Target_Speed_Control(50, 40);
        if (detect_circle_exit())
            cir_stage = 3;
        break;

    case 3:
        // 检测到出口：准备转向
        Target_Speed_Control(45, 35);
        cir_stage = 4;
        break;

    case 4:
        // 出口转向
        Target_Speed_Control(50, 30);
        if (!detect_circle_exit())
            cir_stage = 5;
        break;

    case 5:
        // 离开环岛：恢复直线行驶
        Target_Speed_Control(45, 45);
        cir_stage = 6;
        break;

    case 6:
        // 重置状态
        flag_l_cir = 0;
        cir_stage = 0;
        break;
    }
}

/**
 * @brief 基于像素模式检测环岛入口
 * @return uint8 1表示检测到入口，0表示未检测到
 */
uint8 detect_circle_entry(void)
{
    // 1. 基于特定像素模式检测
    uint8 i = 0, j = 0;
    uint8 count = 0;

    // 检测是否存在环岛特征：查找 "黑-白-黑" 的垂直像素排列模式
    for (i = 5; i < 20; i++)
    {
        for (j = 5; j < IMAGE_WIDTH - 5; j++)
        {
            if (bin_image[i][j] == 0 && bin_image[i + 1][j] == 255 && bin_image[i - 1][j] == 255)
            {
                count++;
            }
        }
    }

    // 2. 结合边界宽度特征检测
    uint8 width_sum = 0;
    for (uint8 y = CIRCLE_BOTTOM_LINE; y < IMAGE_HEIGHT; y++)
    {
        uint8 line_width = r_border[y] - l_border[y];
        if (line_width > CIRCLE_ENTRY_WIDTH_THRESHOLD)
        {
            width_sum++;
        }
    }

    // 两种检测方法结合，提高可靠性
    if ((count > 15 || width_sum > (IMAGE_HEIGHT - CIRCLE_BOTTOM_LINE) * 0.7))
    {
        circle_entry_count++;
        if (circle_entry_count >= CIRCLE_ENTRY_THRESHOLD)
        {
            // 区分左右环岛
            if (Cir1_judge())
                flag_r_cir = 1;
            else if (Cir2_judge())
                flag_l_cir = 1;
            return 1;
        }
    }
    else
    {
        circle_entry_count = 0;
    }

    return 0;
}

/**
 * @brief 基于像素模式检测环岛出口
 * @return uint8 1表示检测到出口，0表示未检测到
 */
uint8 detect_circle_exit(void)
{
    // 如果已经检测到出口，直接返回
    if (circle_exit_detected)
    {
        return 1;
    }

    // 1. 基于特定像素模式检测
    uint8 i = 0, j = 0;
    uint8 count = 0;

    // 检测是否存在环岛出口特征：查找 "白-黑-白" 的垂直像素排列模式
    for (i = 5; i < 20; i++)
    {
        for (j = 5; j < IMAGE_WIDTH - 5; j++)
        {
            if (bin_image[i][j] == 255 && bin_image[i + 1][j] == 0 && bin_image[i - 1][j] == 0)
            {
                count++;
            }
        }
    }

    // 2. 结合边界宽度特征检测
    uint8 bottom_narrow_count = 0;
    uint8 top_wide_count = 0;

    // 检查底部区域宽度是否变窄
    for (uint8 y = CIRCLE_BOTTOM_LINE; y < IMAGE_HEIGHT; y++)
    {
        uint8 line_width = r_border[y] - l_border[y];
        if (line_width < CIRCLE_EXIT_WIDTH_THRESHOLD)
        {
            bottom_narrow_count++;
        }
    }

    // 检查顶部区域是否保持宽线
    for (uint8 y = 0; y < CIRCLE_TOP_LINE; y++)
    {
        uint8 line_width = r_border[y] - l_border[y];
        if (line_width > CIRCLE_ENTRY_WIDTH_THRESHOLD)
        {
            top_wide_count++;
        }
    }

    // 两种检测方法结合，提高可靠性
    if (count > 15 || (bottom_narrow_count > (IMAGE_HEIGHT - CIRCLE_BOTTOM_LINE) * 0.7 &&
                       top_wide_count > CIRCLE_TOP_LINE * 0.7))
    {
        circle_exit_count++;
        if (circle_exit_count >= CIRCLE_EXIT_THRESHOLD)
        {
            circle_exit_detected = 1;
            return 1;
        }
    }
    else
    {
        circle_exit_count = 0;
    }

    return 0;
}

// 十字补线法
/**
 * @brief 计算丢线情况和最长直道长度
 * @param 无
 * @return 无
 */
void lose_line()
{
    r_lose_num = 0;
    l_lose_num = 0;
    length = 0;
    uint16_t i = 0;

    // 判断左边界丢线情况
    for (i = IMAGE_HEIGHT - 1; i > 1; i--)
    {
        if (left_bor[i] <= 3 && left_bor[i] > 0)
        {
            l_lose_judge[i] = 1;
            l_lose_num++;
        }
        else
            l_lose_judge[i] = 0; // 丢线记为1
    }

    // 判断右边界丢线情况
    for (i = IMAGE_HEIGHT - 1; i > 1; i--)
    {
        if (right_bor[i] >= 156 && right_bor[i] < 160)
        {
            r_lose_judge[i] = 1;
            r_lose_num++;
        }
        else
            r_lose_judge[i] = 0; // 丢线记为1
    }

    // 计算最长直道长度（从底部开始找线）
    for (i = IMAGE_HEIGHT - 1; i > 0; i--)
    {
        if (bin_image[i][80] != 0 && bin_image[i - 1][80] != 0)
            length++;
    }
}

/**
 * @brief 判断特定区域内的丢线情况
 * @param type - 类型（1：左边界，2：右边界）
 * @param startline - 起始行
 * @param endline - 结束行
 * @return 无
 */
void lose_location_test(uint8_t type, uint8_t startline, uint8_t endline)
{
    lose_num = 0, nolose_num = 0;
    uint8_t y = startline;

    // 检查左边界丢线情况
    if (type == 1)
    {
        for (y = startline; y > endline; y--)
        {
            lose_num += l_lose_judge[y];
            nolose_num = (startline - endline + 1) - lose_num;
        }
    }
    // 检查右边界丢线情况
    if (type == 2)
    {
        for (y = startline; y > endline; y--)
        {
            lose_num += r_lose_judge[y];
            nolose_num = (startline - endline + 1) - lose_num;
        }
    }
}

/**
 * @brief 计算斜率截距
 * @param uint8 start			输入起点
 * @param uint8 end			输入终点
 * @param uint8 *border			输入需要计算斜率的边界
 * @param float *slope_rate			输入斜率地址
 * @param float *intercept			输入截距地址
 *  @see CTest		calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
 * @return 返回说明
 *     -<em>false</em> fail
 *     -<em>true</em> succeed
 */
void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept)
{
    uint16 i, num = 0;
    uint16 xsum = 0, ysum = 0;
    float y_average, x_average;

    num = 0;
    xsum = 0;
    ysum = 0;
    y_average = 0;
    x_average = 0;
    for (i = start; i < end; i++)
    {
        xsum += i;
        ysum += border[i];
        num++;
    }

    // 计算各个平均数
    if (num)
    {
        x_average = (float)(xsum / num);
        y_average = (float)(ysum / num);
    }

    /*计算斜率*/
    *slope_rate = Slope_Calculate(start, end, border);  // 斜率
    *intercept = y_average - (*slope_rate) * x_average; // 截距
}

/**
 * @brief 十字路口判断
 * @param 无
 * @return
 *   0 - 未检测到十字
 *   1 - 检测到类型1十字
 *   2 - 检测到类型2十字
 */
u8 cross_judge(void)
{
    uint16 i = 0;
    uint8 num1 = 0;
    uint8 num2 = 0, num3 = 0, num4 = 0;

    // 检测类型1十字
    if (l_l_judge() && length > 110)
    {
        for (i = 0; i < data_stastics_l; i++)
        {
            if (dir_r[i] == 6)
                num1++;
            if (dir_l[i] == 6)
                num2++;
        }
        // 判断是否满足类型1十字的方向特征
        if (num2 > 5 && num1 > 5) // 简化版本，去掉了r_dir_sum和l_dir_sum的检查
        {
            num1 = 0;
            num2 = 0;
            return 1;
        }
    }

    // 检测类型2十字
    if (l_l_judge() == 2 && length > 110)
    {
        for (i = 0; i < data_stastics_l; i++)
        {
            if (dir_r[i] == 2)
                num4++;
            if (dir_l[i] == 2)
                num3++;
        }
        // 判断是否满足类型2十字的方向特征
        if (num3 > 5 && num4 > 4) // 简化版本
        {
            num3 = 0;
            num4 = 0;
            return 2;
        }
    }

    return 0;
}

/**
 * @brief 左边界十字补线
 * @param total_L - 左边界点总数
 * @return 无
 */
void L_border_cross(uint16 total_L)
{
    uint16 i = total_L - 1;
    uint8 h = 0;
    uint8 count = 0;
    uint8 x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    float k_l = 0;
    uint8 h2 = 0;

    // 初始化左边界数组
    for (h = 0; h < IMAGE_HEIGHT; h++)
    {
        left_bor[h] = 0;
    }

    // 处理十字情况1
    if (flag_cross == 1)
    {
        while (i > 0)
        {
            h = points_l[i][1];
            if (h2 > h)
            {
                i--;
                continue;
            }
            h2 = 0;

            // 检测十字特征
            if (dir_l[i - 2] == 6 && dir_l[i - 1] == 6 && dir_l[i] == 6 && dir_l[i + 1] != 6 && i - 2 > 1 && h > 3)
            {
                // 计算斜率
                x1 = points_l[i - 7][0];
                y1 = points_l[i - 7][1];
                x2 = points_l[i - 11][0];
                y2 = points_l[i - 11][1];
                k_l = abs((y1 - y2) / (x1 - x2));

                // 补线处理
                while (count < 100 && h - count > 0)
                {
                    if (points_l[i][0] - ((float)count / 4) < 2)
                    {
                        left_bor[h - count] = 2;
                    }
                    else
                        left_bor[h - count] = points_l[i][0] - ((float)count / 4);
                    count++;
                }
                h2 = h - count + 1;
                count = 0;
            }
            else if (points_l[i][0] > left_bor[h])
            {
                left_bor[h] = points_l[i][0];
            }
            i--;
        }
    }
    // 处理十字情况2
    else if (flag_cross == 2)
    {
        for (i = 0; i < data_stastics_l; i++)
        {
            if (dir_l[i] == 2 && dir_l[i - 1] == 4 && dir_l[i + 1] == 2)
            {
                x1 = points_l[i][0];
                y1 = points_l[i][1];
                x2 = i;
                break;
            }
        }
        // 补线处理
        for (i = 24; i < 127; i++)
        {
            left_bor[i] = x1 - (i - x2) / 4 - 10;
        }
        x1 = 0;
        y1 = 0;
        x2 = 0;
    }
}

/**
 * @brief 右边界十字补线
 * @param total_R - 右边界点总数
 * @return 无
 */
void R_border_cross(uint16 total_R)
{
    uint16 i = total_R - 1;
    uint8 h = 0;
    uint8 count = 0;
    uint8 x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    float k_r = 0;
    uint8 h2 = 0;

    // 初始化右边界数组
    for (h = 0; h < IMAGE_HEIGHT; h++)
    {
        right_bor[h] = 200;
    }

    // 处理十字情况1
    if (flag_cross == 1)
    {
        while (i > 0)
        {
            h = points_r[i][1];
            if (h2 > h)
            {
                i--;
                continue;
            }
            h2 = 0;

            // 检测十字特征
            if (dir_r[i - 2] == 6 && dir_r[i - 1] == 6 && dir_r[i] == 6 && dir_r[i + 1] != 6 && i - 2 > 1 && h > 3)
            {
                // 计算斜率
                x1 = points_r[i + 7][0];
                y1 = points_r[i + 7][1];
                x2 = points_r[i + 11][0];
                y2 = points_r[i + 11][1];
                k_r = abs((y1 - y2) / (x1 - x2));

                // 补线处理
                while (count < 100 && h + count < 127)
                {
                    if (points_r[i][0] + ((float)count / 4) > 158)
                    {
                        right_bor[h + count] = 158;
                    }
                    else
                        right_bor[h + count] = points_r[i][0] + ((float)count / 4);
                    count++;
                }
                h2 = h + count - 1;
                count = 0;
            }
            else if (points_r[i][0] < right_bor[h])
            {
                right_bor[h] = points_r[i][0];
            }
            i--;
        }
    }
    // 处理十字情况2
    else if (flag_cross == 2)
    {
        for (i = 0; i < data_stastics_r; i++)
        {
            if (dir_r[i] == 2 && dir_r[i + 1] == 2 && (dir_r[i - 1] == 4 || dir_r[i - 1] == 5) && points_r[i][0] > 70)
            {
                x1 = points_r[i][0];
                y1 = points_r[i][1];
                x2 = i;
                break;
            }
        }
        // 根据x1位置决定补线方式
        if (x1 < 70)
        {
            for (i = 24; i < 127; i++)
            {
                right_bor[i] = left_bor[i] + 50; // 假设的标准宽度
            }
        }
        else
        {
            for (i = 24; i < 127; i++)
            {
                right_bor[i] = x1 + (i - x2) / 4;
            }
        }
        x1 = 0;
        y1 = 0;
        x2 = 0;
    }
}

/**
 * @brief 十字补线函数
 * @param uint8(*image)[IMAGE_WIDTH]		输入二值图像
 * @param uint8 *l_border			输入左边界首地址
 * @param uint8 *r_border			输入右边界首地址
 * @param uint16 total_num_l			输入左边循环总次数
 * @param uint16 total_num_r			输入右边循环总次数
 * @param uint16 *dir_l			输入左边生长方向首地址
 * @param uint16 *dir_r			输入右边生长方向首地址
 * @param uint16(*points_l)[2]		输入左边轮廓首地址
 * @param uint16(*points_r)[2]		输入右边轮廓首地址
 *  @see CTest		cross_fill(image,l_border, r_border, data_statics_l, data_statics_r, dir_l, dir_r, points_l, points_r);
 * @return 返回说明
 *     -<em>false</em> fail
 *     -<em>true</em> succeed
 */
void cross_fill(uint8 (*image)[IMAGE_WIDTH], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r, uint16 *dir_l, uint16 *dir_r, uint16 (*points_l)[2], uint16 (*points_r)[2])
{
    // 计算丢线情况和最长直道长度
    lose_line();

    // 十字判断
    uint8 cross_type = cross_judge();
    if (cross_type > 0)
    {
        flag_cross = cross_type;
        cross_timer = 0;

        // 根据十字类型进行补线
        L_border_cross(total_num_l);
        R_border_cross(total_num_r);

        // 计算中线
        for (uint8 i = 0; i < IMAGE_HEIGHT; i++)
        {
            center_line[i] = (left_bor[i] + right_bor[i]) / 2;
        }
    }
    else
    {
        // 现有的补线逻辑作为备用
        uint8 i;
        uint8 break_num_l = 0;
        uint8 break_num_r = 0;
        uint8 start, end;
        float slope_l_rate = 0, intercept_l = 0;
        // 出十字
        for (i = 1; i < total_num_l; i++)
        {
            if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
            {
                break_num_l = points_l[i][1]; // 传递y坐标
                break;
            }
        }
        for (i = 1; i < total_num_r; i++)
        {
            if (dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] == 6 && dir_r[i + 5] == 6 && dir_r[i + 7] == 6)
            {
                break_num_r = points_r[i][1]; // 传递y坐标
                break;
            }
        }
        if (break_num_l && break_num_r && image[IMAGE_HEIGHT - 1][4] && image[IMAGE_HEIGHT - 1][IMAGE_WIDTH - 4]) // 两边生长方向都符合条件
        {
            // 计算斜率
            start = break_num_l - 15;
            start = limit_a_b(start, 0, IMAGE_HEIGHT);
            end = break_num_l - 5;
            calculate_s_i(start, end, l_border, &slope_l_rate, &intercept_l);
            for (i = break_num_l - 5; i < IMAGE_HEIGHT - 1; i++)
            {
                l_border[i] = slope_l_rate * (i) + intercept_l;               // y = kx+b
                l_border[i] = limit_a_b(l_border[i], border_min, border_max); // 限幅
            }

            // 计算斜率
            start = break_num_r - 15;                  // 起点
            start = limit_a_b(start, 0, IMAGE_HEIGHT); // 限幅
            end = break_num_r - 5;                     // 终点
            calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
            for (i = break_num_r - 5; i < IMAGE_HEIGHT - 1; i++)
            {
                r_border[i] = slope_l_rate * (i) + intercept_l;
                r_border[i] = limit_a_b(r_border[i], border_min, border_max);
            }
        }

        // 清除十字标志
        if (flag_cross > 0)
        {
            cross_timer++;
            if (cross_timer > 50) // 超时清除
            {
                flag_cross = 0;
                cross_timer = 0;
            }
        }
    }
}

/**
 * @brief 环岛处理主函数
 */
void circle_process(void)
{
    static uint16 circle_timer = 0;

    // 计算环岛中心点
    cir_point();

    // 进行环岛补线处理
    circle_line_complement();

    switch (circle_state)
    {
    case CIRCLE_NONE:
        // 未检测到环岛
        if (detect_circle_entry())
        {
            circle_state = CIRCLE_ENTRY;
            circle_timer = 0;
            // 进入环岛减速
            Target_Speed_Control(40, 40);
        }
        break;

    case CIRCLE_ENTRY:
        // 进入环岛
        circle_timer++;
        if (circle_timer > 50) // 等待一段时间确认进入环岛
        {
            circle_state = CIRCLE_ON_TRACK;
            circle_timer = 0;
        }
        break;

    case CIRCLE_ON_TRACK:
        // 环岛内行驶
        // 根据左右环岛进行差异化处理
        if (flag_r_cir)
        {
            Cir_r_handle();
        }
        else if (flag_l_cir)
        {
            Cir_l_handle();
        }
        else
        {
            // 通用环岛处理
            Target_Speed_Control(50, 50);
        }

        // 检测出口
        if (detect_circle_exit())
        {
            circle_state = CIRCLE_EXIT;
            circle_timer = 0;
        }
        break;

    case CIRCLE_EXIT:
        // 离开环岛
        circle_timer++;
        if (circle_timer > 100) // 等待一段时间确认离开环岛
        {
            // 重置环岛状态
            circle_state = CIRCLE_NONE;
            circle_entry_count = 0;
            circle_exit_count = 0;
            circle_exit_detected = 0;
            flag_r_cir = 0;
            flag_l_cir = 0;
            cir_stage = 0;
            // 恢复正常行驶速度
            Target_Speed_Control(60, 60);
        }
        break;

    default:
        circle_state = CIRCLE_NONE;
        break;
    }
}

/**
 * @brief 带环岛处理的巡线函数
 */
void track_process_with_circle(void)
{
    // 先执行常规巡线处理
    track_process();

    // 根据环岛状态进行处理
    circle_process();
}
