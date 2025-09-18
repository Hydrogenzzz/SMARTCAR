#ifndef __PID_H
#define __PID_H

#include "zf_common_headfile.h" //���ɶ�Ӧ��headfile

typedef struct
{
    float kp, ki, kd; // ����ϵ��
    float kp_2, kd_G;
    float error, lastError, lastlastError; // ���ϴ������ϴ����
    float integral, maxIntegral;           // ���֡������޷�
    float output, maxOutput;               // ���������޷�
} PID;

void Incremental_PID_Init(PID *pid, float p, float i, float d, float maxOutput);
void Incremental_PID_Calc(PID *pid, float set_value, float get_value);

void Positional_PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOutput);
void Positional_PID_Calc(PID *pid, float set_value, float get_value);

void Double_pd_Positional_PID_Init(PID *pid, float p, float d, float kp_2, float kd_G, float maxOutput);
void Double_pd_Positional_PID_Calc(PID *pid, float se_value, float get_value, float gyro_z);

extern PID left;
extern PID right;

#endif
