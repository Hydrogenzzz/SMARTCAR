/*
 * encoder.c
 *
 *  Created on: 2024Äê8ÔÂ26ÈÕ
 *      Author: WindSnow
 */
#include "encoder.h"



void encoder_init(void)
{
    encoder_quad_init(ENCODER_LEFT, ENCODER_LEFT_A, ENCODER_LEFT_B);
    encoder_quad_init(ENCODER_RIGHT, ENCODER_RIGHT_A, ENCODER_RIGHT_B);
    pit_ms_init(CCU60_CH0, 30);

}


