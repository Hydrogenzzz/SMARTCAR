/*
 * encoder.h
 *
 *  Created on: 2024��8��26��
 *      Author: WindSnow
 */

#ifndef CODE_ENCODER_H_
#define CODE_ENCODER_H_

#include "zf_common_headfile.h"


#define ENCODER_LEFT                 (TIM5_ENCODER)                         // ������������Ӧʹ�õı������ӿ�
#define ENCODER_LEFT_A               (TIM5_ENCODER_CH1_P10_3)               // A ���Ӧ������
#define ENCODER_LEFT_B               (TIM5_ENCODER_CH2_P10_1)               // B ���Ӧ������
#define ENCODER_RIGHT                (TIM6_ENCODER)                         // ������������Ӧʹ�õı������ӿ�
#define ENCODER_RIGHT_A              (TIM6_ENCODER_CH1_P20_3)               // A ���Ӧ������
#define ENCODER_RIGHT_B              (TIM6_ENCODER_CH2_P20_0)               // B ���Ӧ������




void encoder_init(void);

#endif /* CODE_ENCODER_H_ */
