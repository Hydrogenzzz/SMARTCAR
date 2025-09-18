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
 * �ļ�����          cpu0_main
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
#pragma section all "cpu0_dsram"
#include "zf_common_headfile.h"
#include "server.h"
#include "global.h"
#include "track.h"

// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************
char BT_String[20];
uint8_t data = 25;
int core0_main(void)
{

    clock_init(); // ��ȡʱ��Ƶ��<��ر���>
    debug_init(); // ��ʼ��Ĭ�ϵ��Դ���
    // �˴���д�û����� ���������ʼ�������
    key_list_init(1);
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_init();
    menu_init(); // �˵���ʼ��
    encoder_init();
    server_init();
    Motor_Init();
    mt9v03x_init();

    mt9v03x_set_exposure_time(100);
    Incremental_PID_Init(&left, 1.5, 0.1, 0.05, 10000);
    Incremental_PID_Init(&right, 1.5, 0.1, 0.05, 10000);
    pit_ms_init(CCU60_CH1, 5); // ��������ȡ�ٶ�
    pit_ms_init(CCU60_CH0, 5); // ��������
    Target_Speed_Control(40, 40); // �������ٶ�
    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready(); // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
        // key_action();
        // menu_refresh(); // ������Ϊ�˵���ʼ���ı�Ҫ����
        //        Motor_Control(1000,1000);
        // steer_control(768);
        // printf("%f,%f\n",speedleft,speedright);
        if (mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            otsuThreshold(&mt9v03x_image, &bin_image);
            track_process(); // ִ��巡线处理
            // 图像显示已移至track.c中的track_process函数内
            // tft180_displayimage03x((const uint8 *)bin_image, 160, 128);
        }
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

#pragma section all restore

// **************************** �������� ****************************
