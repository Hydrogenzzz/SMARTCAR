#include "pid.h"

PID left,right,angle;
/**
 * ************************************************************************
 * @brief 增量式PID参数的初始化
 * 
 * @param[in] pid  pid指针
 * @param[in] p  初始化设定的p
 * @param[in] i  初始化设定的i
 * @param[in] d  初始化设定的d
 * @param[in] maxOutput  输出限幅值
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
 * @brief 增量式PID控制器
 *          
 * @param[in] pid  pid指针
 * @param[in] set_value  目标值
 * @param[in] get_value  反馈值
 * 
 * ************************************************************************
 */
void Incremental_PID_Calc(PID *pid, float set_value,float get_value)//d,p,i
{
	pid->error = set_value - get_value;      									//计算偏差
	pid->output += pid->kp*(pid->error - pid->lastError) + pid->ki*pid->error + \
				pid->kd*(pid->error - 2*pid->lastError + pid->lastlastError);			//增量式PI控制器
	pid->lastlastError = pid->lastError;    											//保存上上次误差
	pid->lastError = pid->error;	           											//保存上一次偏差

	if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput)
        pid->output = -pid->maxOutput;													//输出限幅
}



/**
 * ************************************************************************
 * @brief 位置式PID参数的初始化
 * 
 * @param[in] pid  pid指针
 * @param[in] p  初始化设定的p
 * @param[in] i  初始化设定的i
 * @param[in] d  初始化设定的d
 * @param[in] maxI  积分限幅值
 * @param[in] maxOutput  输出限幅值
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
 * @brief 位置式PID控制器
 * 
 * @param[in] pid  pid结构体
 * @param[in] set_value  目标值
 * @param[in] get_value  反馈值
 * 
 * ************************************************************************
 */
void Positional_PID_Calc (PID *pid,float set_value, float get_value)
{
	float dout,pout;
    //更新数据
    pid->lastError = pid->error; 							//将旧error存起来
    pid->error = set_value - get_value; 					//计算新error

    //计算微分
    dout = (pid->error - pid->lastError) * pid->kd;
    //计算比例
    pout = pid->error * pid->kp;
    //计算积分
    pid->integral += pid->error * pid->ki;

    //积分限幅
    if (pid->integral > pid->maxIntegral)
        pid->integral = pid->maxIntegral;
    else if (pid->integral < -pid->maxIntegral)
        pid->integral = -pid->maxIntegral;
    //计算输出
    pid->output = pout + dout + pid->integral;
    //输出限幅
    if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput)
        pid->output = -pid->maxOutput;
}

/**
 * ************************************************************************
 * @brief 双pd位置式PID参数的初始化
 *
 * @param[in] pid  pid指针
 * @param[in] p  初始化设定的p
 * @param[in] d  初始化设定的d
 * @param[in] kp_2  初始化设定的kp_2
 * @param[in] kd_G  初始化设定的kd_G
 * @param[in] maxOutput  输出限幅值
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
 * @brief 位置式PID控制器
 *
 * @param[in] pid  pid结构体
 * @param[in] set_value  目标值
 * @param[in] get_value  反馈值
 *
 * ************************************************************************
 */
//加了陀螺仪的偏差pid，输出值 = error*KP + error * abs(error)*kp_2 + (error - lasterror)*kd + imu660ra_gyro_z * kd_G
void Double_pd_Positional_PID_Calc (PID *pid,float set_value, float get_value , float gyro_z)
{
    float zout;
    //更新数据
    pid->lastError = pid->error;                            //将旧error存起来
    pid->error = set_value - get_value;                     //计算新error

    //计算陀螺仪输出
    zout = gyro_z * pid->kd_G;

    //计算输出
    pid->output = pid->error*pid->kp + pid->error * abs(pid->error)*pid->kp_2 + (pid->error - pid->lastError)*pid->kd + zout;
    //输出限幅
    if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput)
        pid->output = -pid->maxOutput;
}

