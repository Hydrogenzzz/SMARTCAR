
#ifndef _TRACK_H_
#define _TRACK_H_

#include "zf_common_headfile.h"
#include "image.h"

// 复用现有的图像尺寸定义
#define IMAGE_HEIGHT MT9V03X_H // 图像高度
#define IMAGE_WIDTH MT9V03X_W  // 图像宽度
// 为了保持兼容性，添加别名
#define image_h IMAGE_HEIGHT
#define image_w IMAGE_WIDTH

// 颜色定义
#define uesr_RED 0XF800   // 红色
#define uesr_GREEN 0X07E0 // 绿色
#define uesr_BLUE 0X001F  // 蓝色
#define RGB565_RED 0xF800    // 红色
#define RGB565_GREEN 0x07E0  // 绿色
#define RGB565_BLUE 0x001F   // 蓝色
#define RGB565_YELLOW 0xFFE0 // 黄色
#define RGB565_PURPLE 0xF81F // 紫色
#define RGB565_CYAN 0x07FF   // 青色
#define RGB565_WHITE 0xFFFF  // 白色
#define RGB565_BLACK 0x0000  // 黑色
// 边界定义
#define border_max (IMAGE_WIDTH - 2) // 边界最大值
#define border_min 1             // 边界最小值
/*变量声明*/
#define use_num 1 // 1就是不压缩，2就是压缩一倍
// 环岛检测参数
#define CIRCLE_ENTRY_THRESHOLD 3        // 进入环岛判断阈值
#define CIRCLE_EXIT_THRESHOLD 3         // 离开环岛判断阈值
#define CIRCLE_ENTRY_WIDTH_THRESHOLD 40 // 环岛入口宽度阈值
#define CIRCLE_EXIT_WIDTH_THRESHOLD 30  // 环岛出口宽度阈值
#define CIRCLE_BOTTOM_LINE 90           // 图像底部检测线
#define CIRCLE_TOP_LINE 30              // 图像顶部检测线
// 函数声明
extern void track_process(void);   // 巡线主处理函数
extern uint8 l_border[IMAGE_HEIGHT];    // 左边界数组
extern uint8 r_border[IMAGE_HEIGHT];    // 右边界数组
extern uint8 center_line[IMAGE_HEIGHT]; // 中心线数组

extern uint8 original_image[IMAGE_HEIGHT][IMAGE_WIDTH];

extern int16 track_deviation; // 路径偏差值（-100到100）

extern uint8 line_detected; // 线是否找到标志



// 环岛状态机定义
enum CIRCLE_STATE
{
    CIRCLE_NONE = 0,    // 无环岛状态
    CIRCLE_ENTRY,       // 进入环岛状态
    CIRCLE_ON_TRACK,    // 在环岛内正常行驶
    CIRCLE_EXIT_DETECT, // 检测环岛出口
    CIRCLE_EXIT         // 离开环岛状态
};

// 环岛相关全局变量
extern enum CIRCLE_STATE circle_state; // 当前环岛状态
extern uint8 circle_entry_count;       // 进入环岛计数
extern uint8 circle_exit_count;        // 离开环岛计数
extern uint8 circle_exit_detected;     // 出口检测标志
extern uint8 flag_r_cir;               // 右环岛标志
extern uint8 flag_l_cir;               // 左环岛标志
extern uint8_t cir_stage;              // 环岛阶段标志
extern uint8_t cir_x, cir_y;           // 环岛中心点坐标

// 环岛检测参数
extern uint8 target_speed; // 目标速度

// 环岛处理函数声明
extern uint8 detect_circle_entry(void);      // 检测环岛入口
extern uint8 detect_circle_exit(void);       // 检测环岛出口
extern void circle_process(void);            // 环岛处理主函数
extern void track_process_with_circle(void); // 带环岛处理的巡线函数
extern uint8 Cir1_judge(void);               // 判断右环岛
extern uint8 Cir2_judge(void);               // 判断左环岛
extern void cir_point(void);                 // 计算环岛中心点
extern void circle_line_complement(void);    // 环岛补线处理
extern void Cir_r_handle(void);              // 右环岛处理
extern void Cir_l_handle(void);              // 左环岛处理

// 十字处理函数声明
extern void lose_line(void);                 // 计算丢线情况和最长直道长度
extern void lose_location_test(uint8_t type, uint8_t startline, uint8_t endline); // 判断特定区域内的丢线情况
extern u8 cross_judge(void);                 // 十字路口判断
extern void L_border_cross(uint16 total_L);  // 左边界十字补线
extern void R_border_cross(uint16 total_R);  // 右边界十字补线
extern void cross_fill(uint8 (*image)[IMAGE_WIDTH], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r, uint16 *dir_l, uint16 *dir_r, uint16 (*points_l)[2], uint16 (*points_r)[2]); // 十字补线主函数

#endif /*_TRACK_H_*/
