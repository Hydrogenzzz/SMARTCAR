/*********************************************************************************************************************
 * TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
 * Copyright (c) 2022 SEEKFREE ��ɿƼ�
 *
 * ���ļ��� TC264 ��Դ���һ����
 *
 * TC264 ��Դ�� ���������
 * �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
 * �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
 *
 * ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
 * ����û�������������Ի��ʺ��ض���;�ı�֤
 * ����ϸ����μ� GPL
 *
 * ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
 * ���û�У������<https://www.gnu.org/licenses/>
 *
 * ����ע����
 * ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
 * ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
 * ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
 * ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
 *
 * �ļ�����          isr
 * ��˾����          �ɶ���ɿƼ����޹�˾
 * �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
 * ��������          ADS v1.9.4
 * ����ƽ̨          TC264D
 * ��������          https://seekfree.taobao.com/
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2022-09-15       pudding            first version
 ********************************************************************************************************************/

#include "isr_config.h"
#include "isr.h"
#include "global.h"
// ����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ�� interrupt_global_enable(0); �������ж�Ƕ��
// �򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ������� interrupt_global_disable(); ���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ����� interrupt_global_enable(0); �������жϵ���Ӧ��

// **************************** PIT中断处理 ****************************
/*
中断名称：cc60_pit_ch0_isr
功能说明：CCU6_0的PIT通道0中断处理函数，用于按键扫描
优先级：CCU6_0_CH0_ISR_PRIORITY (30)
服务核心：CPU0
*/
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0); // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);  // 清除PIT中断标志位
    key_handler();              // 调用按键处理函数
}

/*
中断名称：cc60_pit_ch1_isr
功能说明：CCU6_0的PIT通道1中断处理函数，用于电机速度PID控制
优先级：CCU6_0_CH1_ISR_PRIORITY (31)
服务核心：CPU0
*/
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0); // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);  // 清除PIT中断标志位

    // 读取编码器计数并计算实际速度
    speedleft = -(int)encoder_get_count(TIM5_ENCODER);
    speedright = (int)encoder_get_count(TIM6_ENCODER);

    // 进行PID计算
    Incremental_PID_Calc(&left, target_left, speedleft);
    Incremental_PID_Calc(&right, target_right, speedright);

    // 控制电机
    Motor_Control(left.output, right.output);

    // 清除编码器计数，准备下次读取
    encoder_clear_count(TIM5_ENCODER);
    encoder_clear_count(TIM6_ENCODER);
}

/*
中断名称：cc61_pit_ch0_isr
功能说明：CCU6_1的PIT通道0中断处理函数
优先级：CCU6_1_CH0_ISR_PRIORITY (32)
服务核心：CPU0
*/
IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0); // 开启中断嵌套
    pit_clear_flag(CCU61_CH0);  // 清除PIT中断标志位
    // Positional_PID_Calc(&angle,0,distance );
            // 计算路径偏差
        calculate_deviation();
        // 控制小车行驶
        pid_steer_control();
}

/*
中断名称：cc61_pit_ch1_isr
功能说明：CCU6_1的PIT通道1中断处理函数
优先级：CCU6_1_CH1_ISR_PRIORITY (33)
服务核心：CPU0
*/
IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0); // 开启中断嵌套
    pit_clear_flag(CCU61_CH1);  // 清除PIT中断标志位
}
// **************************** PIT�жϺ��� ****************************

// **************************** 外部中断处理 ****************************
/*
中断名称：exti_ch0_ch4_isr
功能说明：ERU通道0和通道4的外部中断处理函数
优先级：EXTI_CH0_CH4_INT_PRIO (40)
服务核心：CPU0
*/
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);            // 开启中断嵌套
    if (exti_flag_get(ERU_CH0_REQ0_P15_4)) // 通道0中断（P15_4引脚）
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
    }

    if (exti_flag_get(ERU_CH4_REQ13_P15_5)) // 通道4中断（P15_5引脚）
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);
    }
}

/*
中断名称：exti_ch1_ch5_isr
功能说明：ERU通道1和通道5的外部中断处理函数，处理ToF模块和无线模块中断
优先级：EXTI_CH1_CH5_INT_PRIO (41)
服务核心：CPU0
*/
IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0); // 开启中断嵌套

    if (exti_flag_get(ERU_CH1_REQ10_P14_3)) // 通道1中断（P14_3引脚）
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler(); // ToF 模块 INT 中断处理
    }

    if (exti_flag_get(ERU_CH5_REQ1_P15_8)) // 通道5中断（P15_8引脚）
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);

        wireless_module_spi_handler(); // SPI WIFI 中断回调函数
    }
}

// ��������ͷpclk����Ĭ��ռ���� 2ͨ�������ڴ���DMA��������ﲻ�ٶ����жϺ���
// IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // �����ж�Ƕ��
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // ͨ��2�ж�
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // ͨ��6�ж�
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }

IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);            // �����ж�Ƕ��
    if (exti_flag_get(ERU_CH3_REQ6_P02_0)) // ͨ��3�ж�
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler(); // ����ͷ�����ɼ�ͳһ�ص�����
    }
    if (exti_flag_get(ERU_CH7_REQ16_P15_1)) // ͨ��7�ж�
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);
    }
}
// **************************** �ⲿ�жϺ��� ****************************

// **************************** DMA�жϺ��� ****************************
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    camera_dma_handler();       // ����ͷ�ɼ����ͳһ�ص�����
}
// **************************** DMA�жϺ��� ****************************

// **************************** �����жϺ��� ****************************
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart0_handle);
}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart0_handle);

#if DEBUG_UART_USE_INTERRUPT   // ������� debug �����ж�
    debug_interrupr_handler(); // ���� debug ���ڽ��մ������� ���ݻᱻ debug ���λ�������ȡ
#endif                         // ����޸��� DEBUG_UART_INDEX ����δ�����Ҫ�ŵ���Ӧ�Ĵ����ж�ȥ
}
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart0_handle);
}

// ����1Ĭ�����ӵ�����ͷ���ô���
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart1_handle);
}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart1_handle);
    camera_uart_handler(); // ����ͷ��������ͳһ�ص�����
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart1_handle);
}

// ����2Ĭ�����ӵ�����ת����ģ��
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart2_handle);
}
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart2_handle);
    wireless_module_uart_handler(); // ����ģ��ͳһ�ص�����
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart2_handle);
}

IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart3_handle);
}
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart3_handle);

    gps_uart_callback();
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart3_handle);
}
// **************************** �����жϺ��� ****************************
