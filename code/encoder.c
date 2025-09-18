/*
 * encoder.c
 *
 *  Created on: 2024��8��26��
 *      Author: WindSnow
 */
#include "encoder.h"

/*
函数名称：void encoder_init(void)
功能说明：初始化编码器接口和相关定时器
参数说明：无
函数返回：无
*/
void encoder_init(void)
{
    // 初始化左编码器（TIM5）
    encoder_quad_init(ENCODER_LEFT, ENCODER_LEFT_A, ENCODER_LEFT_B);
    // 初始化右编码器（TIM6）
    encoder_quad_init(ENCODER_RIGHT, ENCODER_RIGHT_A, ENCODER_RIGHT_B);
    // 初始化PIT定时器，用于定时读取编码器值，周期30ms
    pit_ms_init(CCU60_CH0, 30);
}


