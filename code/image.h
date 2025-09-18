/*
 * image.h
 *
 *  Created on: 2024Äê7ÔÂ13ÈÕ
 *      Author: liukai
 */

#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"

#define LINE_MAX_LENGTH (120)
typedef uint8_t pixel_t;


typedef struct{
    uint8_t *data;
    uint16_t width;
    uint16_t height;
}image;

typedef struct{
    uint16_t x;
    uint16_t y;
}point;

typedef struct line{
    int length;
    point data[LINE_MAX_LENGTH];
}line;

void MapImagshow_UpperComputer(const uint8_t (*map_image)[120][188]);
void otsuThreshold(const uint8_t (*mat)[120][188], uint8_t (*bin_img)[120][188]);
void Inv_Perspective_Map_init();
void Inv_Perspective_Map_point(const uint8_t i,const uint8_t j, int *x, int *y);
void Inv_Perspective_Map(const uint8_t (*image)[120][188], uint8_t (*map_img)[120][188]);
float Distance(point p1, point p2);
void line_clean(line *self);
void line_append(line *self, point p);
void draw_point(const point * self,const uint16_t color); 
void draw_line(const line * const self, uint16_t color);

extern uint8 bin_image[MT9V03X_H][MT9V03X_W];
extern uint8 map_image[MT9V03X_H][MT9V03X_W];
extern uint8 map_bin_image[MT9V03X_H][MT9V03X_W];


#endif /* CODE_IMAGE_H_ */
