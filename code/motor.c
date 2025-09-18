#include "motor.h"
#include "zf_common_headfile.h"
#include "global.h"

/*
函数名称：Motor_Init
功能说明：初始化电机控制相关的PWM和GPIO引脚
参数说明：无
函数返回：无
*/
void Motor_Init(void)
{
    pwm_init(ATOM0_CH7_P02_7, 17000, 0);  // 初始化左轮PWM
    pwm_init(ATOM0_CH5_P02_5, 17000, 0);  // 初始化右轮PWM
    gpio_init(P02_4, GPO, 0, GPO_PUSH_PULL);  // 初始化右轮方向控制引脚
    gpio_init(P02_6, GPO, 0, GPO_PUSH_PULL);  // 初始化左轮方向控制引脚
}
// ...e
/*void Motor_Control(uint8_t dir, float duty) // dir涓烘柟鍚戯紝duty涓哄崰绌烘瘮
{
    if (dir == 1)
    {
        gpio_set_level(P02_4, 1);
        gpio_set_level(P02_6, 1);
        pwm_set_duty(ATOM0_CH7_P02_7,duty);
        pwm_set_duty(ATOM0_CH5_P02_5,duty);
    }
    else
    {
        gpio_set_level(P02_4, 0);
        gpio_set_level(P02_6, 0);
        pwm_set_duty(ATOM0_CH7_P02_7,duty);
        pwm_set_duty(ATOM0_CH5_P02_5,duty);
    }
}*/
/*
函数名称：void Target_Speed_Control(float targetleft, float targetright)
功能说明：设置左右轮目标速度，更新全局速度变量
参数说明：
    targetleft - 左轮目标速度值
    targetright - 右轮目标速度值
函数返回：无
*/
void Target_Speed_Control(float targetleft, float targetright)
{
    // 更新全局目标速度变量
    target_left = targetleft; // 前为在global中定义的全局target速度，后为输出target速度
    target_right = targetright;
}
/*
函数名称：Motor_Control
功能说明：控制电机的转速和方向
参数说明：
    speedleft - 左轮速度值（正值为正转，负值为反转）
    speedright - 右轮速度值（正值为正转，负值为反转）
函数返回：无
*/
void Motor_Control(int32 speedleft, int32 speedright)
{
    // Motor_Limit(&speedleft, &speedright);
    // 对左轮速度进行限幅
    if (speedleft > 5000)
    {
        speedleft = 5000;
    }
    if (speedleft < -5000)
    {
        speedleft = -5000;
    }
    // 对右轮速度进行限幅
    if (speedright > 5000)
    {
        speedright = 5000;
    }
    if (speedright < -5000)
    {
        speedright = -5000;
    }
    
    // 控制左轮的方向和占空比
    if (speedleft >= 0)
    {
        gpio_set_level(P02_6, 1);  // 正转
        pwm_set_duty(ATOM0_CH7_P02_7, (uint32)speedleft); // 0..8000
    }
    else
    {
        gpio_set_level(P02_6, 0);  // 反转
        pwm_set_duty(ATOM0_CH7_P02_7, (uint32)(-speedleft));
    }

    // 控制右轮的方向和占空比
    if (speedright >= 0)
    {
        gpio_set_level(P02_4, 1);  // 正转
        pwm_set_duty(ATOM0_CH5_P02_5, (uint32)speedright);
    }
    else
    {
        gpio_set_level(P02_4, 0);  // 反转
        pwm_set_duty(ATOM0_CH5_P02_5, (uint32)(-speedright));
    }
}
