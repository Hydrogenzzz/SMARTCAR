#ifndef _motor_h_
#define _motor_h_
#include "zf_common_headfile.h"
#include "global.h"

#define RIGHT_PWM_PIN (ATOM0_CH5_P02_5)
#define LEFT_PWM_PIN (ATOM0_CH7_P02_7)
#define RIGHT_DIR_PIN (P02_4)
#define LEFT_DIR_PIN (P02_6)
void Motor_Init(void);
// void Motor_Limit(int32 *motorA, int32 *motorB);
void Motor_Control(int32 speedleft, int32 speedright);
void Target_Speed_Control(float targetleft, float targetright);
#endif // !_motor_h_
