#ifndef _TRACK_H_
#define _TRACK_H_

#include "zf_common_headfile.h"
#include "image.h"

// 复用现有的图像尺寸定义
#define image_h	MT9V03X_H	//图像高度
#define image_w	MT9V03X_W	//图像宽度

// 兼容IMAGE_HEIGHT和IMAGE_WIDTH的定义
#define IMAGE_HEIGHT	image_h
#define IMAGE_WIDTH	image_w

// 颜色定义
#define uesr_RED	0XF800    //红色
#define uesr_GREEN	0X07E0    //绿色
#define uesr_BLUE	0X001F    //蓝色

// 边界定义
#define border_max	(image_w-2) //边界最大值
#define border_min	1	//边界最小值	

// 函数声明
extern void track_process(void); // 巡线主处理函数
extern uint8 l_border[image_h];  // 左边界数组
extern uint8 r_border[image_h];  // 右边界数组
extern uint8 center_line[image_h]; // 中心线数组

extern uint8 original_image[image_h][image_w];

extern int16 track_deviation;    // 路径偏差值（-100到100）

extern uint8 line_detected;      // 线是否找到标志

#endif /*_TRACK_H_*/

