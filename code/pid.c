#include "pid.h"

PID left,right,angle;
/**
 * ************************************************************************
 * @brief ����ʽPID�����ĳ�ʼ��
 * 
 * @param[in] pid  pidָ��
 * @param[in] p  ��ʼ���趨��p
 * @param[in] i  ��ʼ���趨��i
 * @param[in] d  ��ʼ���趨��d
 * @param[in] maxOutput  ����޷�ֵ
 * 
 * ************************************************************************
 */
void Incremental_PID_Init(PID *pid, float p, float i, float d, float maxOutput)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->maxOutput = maxOutput;
}

/**
 * ************************************************************************
 * @brief ����ʽPID������
 *          
 * @param[in] pid  pidָ��
 * @param[in] set_value  Ŀ��ֵ
 * @param[in] get_value  ����ֵ
 * 
 * ************************************************************************
 */
void Incremental_PID_Calc(PID *pid, float set_value,float get_value)//d,p,i
{
	pid->error = set_value - get_value;      									//����ƫ��
	pid->output += pid->kp*(pid->error - pid->lastError) + pid->ki*pid->error + \
				pid->kd*(pid->error - 2*pid->lastError + pid->lastlastError);			//����ʽPI������
	pid->lastlastError = pid->lastError;    											//�������ϴ����
	pid->lastError = pid->error;	           											//������һ��ƫ��

	if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput)
        pid->output = -pid->maxOutput;													//����޷�
}



/**
 * ************************************************************************
 * @brief λ��ʽPID�����ĳ�ʼ��
 * 
 * @param[in] pid  pidָ��
 * @param[in] p  ��ʼ���趨��p
 * @param[in] i  ��ʼ���趨��i
 * @param[in] d  ��ʼ���趨��d
 * @param[in] maxI  �����޷�ֵ
 * @param[in] maxOutput  ����޷�ֵ
 * 
 * ************************************************************************
 */
void Positional_PID_Init (PID *pid, float p, float i, float d, float maxI, float maxOutput)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOutput;
}

/**
 * ************************************************************************
 * @brief λ��ʽPID������
 * 
 * @param[in] pid  pid�ṹ��
 * @param[in] set_value  Ŀ��ֵ
 * @param[in] get_value  ����ֵ
 * 
 * ************************************************************************
 */
void Positional_PID_Calc (PID *pid,float set_value, float get_value)
{
	float dout,pout;
    //��������
    pid->lastError = pid->error; 							//����error������
    pid->error = set_value - get_value; 					//������error

    //����΢��
    dout = (pid->error - pid->lastError) * pid->kd;
    //�������
    pout = pid->error * pid->kp;
    //�������
    pid->integral += pid->error * pid->ki;

    //�����޷�
    if (pid->integral > pid->maxIntegral)
        pid->integral = pid->maxIntegral;
    else if (pid->integral < -pid->maxIntegral)
        pid->integral = -pid->maxIntegral;
    //�������
    pid->output = pout + dout + pid->integral;
    //����޷�
    if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput)
        pid->output = -pid->maxOutput;
}

/**
 * ************************************************************************
 * @brief ˫pdλ��ʽPID�����ĳ�ʼ��
 *
 * @param[in] pid  pidָ��
 * @param[in] p  ��ʼ���趨��p
 * @param[in] d  ��ʼ���趨��d
 * @param[in] kp_2  ��ʼ���趨��kp_2
 * @param[in] kd_G  ��ʼ���趨��kd_G
 * @param[in] maxOutput  ����޷�ֵ
 *
 * ************************************************************************
 */

void Double_pd_Positional_PID_Init (PID *pid, float p, float d, float kp_2, float kd_G, float maxOutput)
{
    pid->kp = p;
    pid->kd = d;
    pid->kp_2 = kp_2;
    pid->kd_G = kd_G;
    pid->maxOutput = maxOutput;
}

/**
 * ************************************************************************
 * @brief λ��ʽPID������
 *
 * @param[in] pid  pid�ṹ��
 * @param[in] set_value  Ŀ��ֵ
 * @param[in] get_value  ����ֵ
 *
 * ************************************************************************
 */
//���������ǵ�ƫ��pid�����ֵ = error*KP + error * abs(error)*kp_2 + (error - lasterror)*kd + imu660ra_gyro_z * kd_G
void Double_pd_Positional_PID_Calc (PID *pid,float set_value, float get_value , float gyro_z)
{
    float zout;
    //��������
    pid->lastError = pid->error;                            //����error������
    pid->error = set_value - get_value;                     //������error

    //�������������
    zout = gyro_z * pid->kd_G;

    //�������
    pid->output = pid->error*pid->kp + pid->error * abs(pid->error)*pid->kp_2 + (pid->error - pid->lastError)*pid->kd + zout;
    //����޷�
    if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput)
        pid->output = -pid->maxOutput;
}

