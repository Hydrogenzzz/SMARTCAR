#include "my_flash.h"

uint8 write_flag = 0,flash_flag = 0;
uint8 erase_flag = 0;//先1后0要记住 1-擦除and保存变量的初始化值 0-不擦除and保存变量
float speed=100;
void read_flash_to_buffer(void)
{
    //1-擦除
    if(erase_flag == 1)
    {
        flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX); // 擦除这一页
    }
    //0-不擦除 直接读取
    if(erase_flag == 0)
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);// 将数据从 flash 读取到缓冲区
        /////////////////////////////////////////////////////////////////////////
          speed = flash_union_buffer[0].float_type;
//        max_turn_speed = flash_union_buffer[1].float_type;
//        S3010.kp = flash_union_buffer[2].float_type;
//        S3010.kd = flash_union_buffer[3].float_type;
//        S3010_straight.kp = flash_union_buffer[4].float_type;
//        S3010_straight.kd = flash_union_buffer[5].float_type;
//        back_speed = flash_union_buffer[6].float_type;
//        multiple_v_diff = flash_union_buffer[7].float_type;
//        y = flash_union_buffer[8].uint8_type;
//        back_kp = flash_union_buffer[9].float_type;
//        front_kp = flash_union_buffer[10].float_type;
//        clip_speed = flash_union_buffer[11].float_type;
//        stop_d = flash_union_buffer[12].uint8_type;
//        car_x = flash_union_buffer[13].uint8_type;
//        multiple_v_diff_straight = flash_union_buffer[14].float_type;
//        servo_duty_max = flash_union_buffer[15].float_type;
//        servo_duty_min = flash_union_buffer[16].float_type;
        /////////////////////////////////////////////////////////////////////////
    }
}

void write_buffer_to_flash(void)
{
    //写入保存变量的初始化值
    if(erase_flag == 1 && write_flag == 1)
    {
        flash_buffer_clear();
        /////////////////////////////////////////////////////////////////////////
       flash_union_buffer[0].float_type = 100;//speed
//        flash_union_buffer[1].float_type = 160;//max_turn_speed
//        flash_union_buffer[2].float_type = 2.3;//S3010.kp
//        flash_union_buffer[3].float_type = 5;//S3010.kd
//        flash_union_buffer[4].float_type = 1;//S3010_straight.kp
//        flash_union_buffer[5].float_type = 1.5;//S3010_straight.kd
//        flash_union_buffer[6].float_type = 100;//back_speed
//        flash_union_buffer[7].float_type = 1;//multiple_v_diff
//        flash_union_buffer[8].uint8_type = 55;//y
//        flash_union_buffer[9].float_type = 80;//back_kp
//        flash_union_buffer[10].float_type = 50;//front_kp
//        flash_union_buffer[11].float_type = 40;//clip_speed
//        flash_union_buffer[12].uint8_type = 100;//stop_d
//        flash_union_buffer[13].uint8_type = 88;//car_x
//        flash_union_buffer[14].float_type = 1;//multiple_v_diff_straight
//        flash_union_buffer[15].float_type = 7.65 * 100;//servo_duty_max
//        flash_union_buffer[16].float_type = 6.05 * 100;//servo_duty_min
        /////////////////////////////////////////////////////////////////////////
        flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);// 向指定 Flash 扇区的页码写入缓冲区数据
        flash_buffer_clear();
        write_flag = 0;
        flash_flag++;
    }
    //保存变量
    if(erase_flag == 0 && write_flag == 1)
    {
        flash_buffer_clear();
        /////////////////////////////////////////////////////////////////////////
       flash_union_buffer[0].float_type = speed;
//        flash_union_buffer[1].float_type = max_turn_speed;
//        flash_union_buffer[2].float_type = S3010.kp;
//        flash_union_buffer[3].float_type = S3010.kd;
//        flash_union_buffer[4].float_type = S3010_straight.kp;
//        flash_union_buffer[5].float_type = S3010_straight.kd;
//        flash_union_buffer[6].float_type = back_speed;
//        flash_union_buffer[7].float_type = multiple_v_diff;
//        flash_union_buffer[8].uint8_type = y;
//        flash_union_buffer[9].float_type = back_kp;
//        flash_union_buffer[10].float_type = front_kp;
//        flash_union_buffer[11].float_type = clip_speed;
//        flash_union_buffer[12].uint8_type = stop_d;
//        flash_union_buffer[13].uint8_type = car_x;
//        flash_union_buffer[14].float_type = multiple_v_diff_straight;
//        flash_union_buffer[15].float_type = servo_duty_max;
//        flash_union_buffer[16].float_type = servo_duty_min;
        /////////////////////////////////////////////////////////////////////////
        flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);// 向指定 Flash 扇区的页码写入缓冲区数据
        flash_buffer_clear();
        write_flag = 0;
        flash_flag++;
    }
}

