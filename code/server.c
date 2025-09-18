#include "server.h"
//#include "global.h"
#include "zf_common_headfile.h"

void server_init()
{
    pwm_init(ATOM0_CH1_P33_9,50,768);
}

void steer_control(float PWM)
{
    pwm_set_duty(ATOM0_CH1_P33_9,PWM);
}
