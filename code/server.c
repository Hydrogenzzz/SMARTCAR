#include "server.h"
#include "track.h"
// #include "global.h"
#include "zf_common_headfile.h"

/*
函数名称：void server_init()
功能说明：初始化舵机控制PWM
参数说明：无
函数返回：无
*/
void server_init()
{
    // 初始化PWM通道，频率50Hz，初始占空比768（对应舵机中位）
    pwm_init(ATOM0_CH1_P33_9, 50, 768);
}

/*
函数名称：void steer_control(float PWM)
功能说明：设置舵机PWM占空比，控制舵机转向
参数说明：
    PWM - 目标PWM占空比值（范围通常为568-968）
函数返回：无
*/
void steer_control(float PWM)
{
    // 设置舵机PWM通道的占空比
    pwm_set_duty(ATOM0_CH1_P33_9, PWM);
}
void pid_steer_control()
{
    if (line_detected)
    {
        float pid_output = Positional_PID_Calc(&angle, 0, track_deviation);
        float base_pwm = 768f;
        float target_pwm = base_pwm + pid_output;
        target_pwm = limit_a_b(target_pwm, 568.0f, 968.0f);
        // 舵机控制
        steer_control(target_pwm);
    }
}