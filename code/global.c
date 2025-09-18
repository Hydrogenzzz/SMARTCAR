#include "global.h"
#include "zf_common_headfile.h"

float speedleft = 0;
float speedright = 0;
float target_left = 0;
float target_right = 0;
uint8_t bin_image;
void allinitialize(void)
{
    key_list_init(1);
    tft180_init();
    menu_init(); // 鑿滃崟鍒濆鍖�
    encoder_init();
    server_init();
    Motor_Init();
    Incremental_PID_Init(&left, 40, 5, 13, 5000);
    Incremental_PID_Init(&right, 40, 5, 13, 5000);
}
