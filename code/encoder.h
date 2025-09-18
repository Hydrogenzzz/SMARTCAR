/*
 * encoder.h
 *
 *  Created on: 2024年8月26日
 *      Author: WindSnow
 */

#ifndef CODE_ENCODER_H_
#define CODE_ENCODER_H_

#include "zf_common_headfile.h"


#define ENCODER_LEFT                 (TIM5_ENCODER)                         // 正交编码器对应使用的编码器接口
#define ENCODER_LEFT_A               (TIM5_ENCODER_CH1_P10_3)               // A 相对应的引脚
#define ENCODER_LEFT_B               (TIM5_ENCODER_CH2_P10_1)               // B 相对应的引脚
#define ENCODER_RIGHT                (TIM6_ENCODER)                         // 正交编码器对应使用的编码器接口
#define ENCODER_RIGHT_A              (TIM6_ENCODER_CH1_P20_3)               // A 相对应的引脚
#define ENCODER_RIGHT_B              (TIM6_ENCODER_CH2_P20_0)               // B 相对应的引脚




void encoder_init(void);

#endif /* CODE_ENCODER_H_ */
