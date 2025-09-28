#include "global.h"
#include "zf_common_headfile.h"

// 左轮实际速度
float speedleft = 0;
// 右轮实际速度
float speedright = 0;
// 左轮目标速度
float target_left = 0;
// 右轮目标速度
float target_right = 0;
// 二值化图像（未初始化，仅声明）
float target_angle = 0;
uint8_t bin_image;

/*
函数名称：void allinitialize(void)
功能说明：智能车系统初始化函数，初始化所有外设和控制参数
参数说明：无
函数返回：无
*/
void allinitialize(void)
{
    // 初始化按键列表，参数1表示支持长按功能
    key_list_init(1);
    // 初始化1.8寸TFT显示屏
    tft180_init();
    // 初始化菜单系统
    menu_init(); // 菜单初始化
    // 初始化编码器
    encoder_init();
    // 初始化舵机控制
    server_init();
    // 初始化电机控制
    Motor_Init();
    // 初始化左轮增量式PID控制器（P=40, I=5, D=13, 输出限幅5000）
    Incremental_PID_Init(&left, 40, 5, 13, 5000);
    // 初始化右轮增量式PID控制器（P=40, I=5, D=13, 输出限幅5000）
    Incremental_PID_Init(&right, 40, 5, 13, 5000);
}
