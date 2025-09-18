#include "motor.h"
#include "zf_common_headfile.h"
#include "global.h"

void Motor_Init(void)
{
    pwm_init(ATOM0_CH7_P02_7, 17000, 0);
    pwm_init(ATOM0_CH5_P02_5, 17000, 0);
    gpio_init(P02_4, GPO, 0, GPO_PUSH_PULL);
    gpio_init(P02_6, GPO, 0, GPO_PUSH_PULL);
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
void Target_Speed_Control(float targetleft, float targetright)
{
    target_left = targetleft; // 前为在global中定义的全局target速度，后为输出target速度
    target_right = targetright;
}
void Motor_Control(int32 speedleft, int32 speedright)
{
    // Motor_Limit(&speedleft, &speedright);
    if (speedleft > 5000)
    {
        speedleft = 5000;
    }
    if (speedleft < -5000)
    {
        speedleft = -5000;
    }
    if (speedright > 5000)
    {
        speedright = 5000;
    }
    if (speedright < -5000)
    {
        speedright = -5000;
    } // 闄愬箙
    if (speedleft >= 0)
    {
        gpio_set_level(P02_6, 1);
        // 姝ｈ浆
        pwm_set_duty(ATOM0_CH7_P02_7, (uint32)speedleft); // 0..8000
    }
    else
    {
        gpio_set_level(P02_6, 0); // 鍙嶈浆
        pwm_set_duty(ATOM0_CH7_P02_7, (uint32)(-speedleft));
    }

    // 鍙崇數鏈烘柟鍚戜笌鍗犵┖姣�
    if (speedright >= 0)
    {
        gpio_set_level(P02_4, 1);
        pwm_set_duty(ATOM0_CH5_P02_5, (uint32)speedright);
    }
    else
    {
        gpio_set_level(P02_4, 0);
        pwm_set_duty(ATOM0_CH5_P02_5, (uint32)(-speedright));
    }
}
