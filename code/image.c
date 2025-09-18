/*
 * image.c
 *
 *  Created on: 2024��7��13��
 *      Author: liukai
 */
#include "image.h"
#include "zf_device_mt9v03x.h"
#include "menu.h"
extern const uint8_t black_val = 0, white_val = 255;
uint8 bin_image[MT9V03X_H][MT9V03X_W];
uint8 map_image[MT9V03X_H][MT9V03X_W];
uint8 map_bin_image[MT9V03X_H][MT9V03X_W];

line left_line_ori;
line right_line_ori;

uint8_t image_show[MT9V03X_H][MT9V03X_W];
line line_show1;
line line_show2;

double change_un_Mat[3][3] ={
        {-0.444041,0.340344,-6.435363},
        {-0.015644,0.051009,-15.582834},
        {-0.000137,0.004097,-0.573250}
};

//��򷨶�ֵ��
void otsuThreshold(const uint8_t (*mat)[120][188], uint8_t (*bin_img)[120][188])
{
    #define GrayScale 256
    int pixelCount[GrayScale] = {0}; // ÿ���Ҷ�ֵ��ռ���ظ���
    float pixelPro[GrayScale] = {0}; // ÿ���Ҷ�ֵ��ռ�����ر���
    int i, j;
    int Sumpix = 188 * 120; // �����ص�
    uint8_t threshold = 0; // ʹ��uint8_t��ȷ������ֵ������ȷ

    // ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
    for (i = 0; i < 120; i++)
    {
        for (j = 0; j < 188; j++)
        {
            pixelCount[(int)(*mat)[i][j]]++;
        }
    }
    float u = 0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / Sumpix; // ����ÿ������������ͼ���еı���
        u += i * pixelPro[i]; // ��ƽ���Ҷ�
    }

    float maxVariance = 0.0; // �����䷽��
    float w0 = 0, avgValue = 0; // w0 ǰ��������avgValue ǰ��ƽ���Ҷ�
    for (int i = 0; i < GrayScale; i++) // ÿһ��ѭ������һ��������䷽����� (����for����Ϊ1��)
    {
        w0 += pixelPro[i]; // ���赱ǰ�Ҷ�iΪ��ֵ, 0~i �Ҷ�������ռ����ͼ��ı�����ǰ������
        avgValue += i * pixelPro[i];

        float variance = pow((avgValue / w0 - u), 2) * w0 / (1 - w0); // ��䷽��
        if (variance > maxVariance)
        {
            maxVariance = variance;
            threshold = (uint8_t)i; // ��ȷת��Ϊuint8_t����
        }
    }

     for (int row = 0; row < 120; row++)
     {
         for (int col = 0; col < 188; col++)
         {
             (*bin_img)[row][col] = ((*mat)[row][col] > threshold) ? 255 : 0;
             if (row ==0 || row == 119 || col == 0 || col ==187) (*bin_img)[row][col] = 0;
         }
     }

}

//�ֲ���ֵ��
void adaptiveThreshold(const uint8_t (*mat)[120][188], uint8_t (*bin_img)[120][188], int block, uint8_t clip_value){
  zf_assert(block % 2 == 1); // block����Ϊ����
  int half_block = block / 2;
  for(int y=half_block; y<120-half_block; y++){
    for(int x=half_block; x<188-half_block; x++){
      // ����ֲ���ֵ
      int thres = 0;
      for(int dy=-half_block; dy<=half_block; dy++){
        for(int dx=-half_block; dx<=half_block; dx++){
          thres += (*mat)[y+dy][x+dx];
        }
      }
      thres = thres / (block * block) - clip_value;
      // ���ж�ֵ��
      (*bin_img)[y][x] = (*mat)[y][x]>thres ? 255 : 0;
    }
  }
}

/*
�������ƣ�MapImagshow_UpperComputer
����˵��������ͼͼ��ͨ��UART���͵���λ����ʾ
����˵����
          map_image - Ҫ���͵ĵ�ͼͼ������ָ��
�������أ���
*/
void MapImagshow_UpperComputer(const uint8_t (*map_image)[120][188]) {
    uart_write_byte(UART_0,0x00);
    uart_write_byte(UART_0,0xff);
    uart_write_byte(UART_0,0x01);
    uart_write_byte(UART_0,0x01);
    for (int i=0 ;i<120 ;i++) {
      for (int j=0 ;j<188 ;j++) {
          uart_write_byte(UART_0, (*map_image)[i][j]);
     }
    }
}

/*
�������ƣ�Inv_Perspective_Map_point
����˵�����Ե����������͸�ӱ任
����˵����
          i, j - ԭʼ������
          x, y - �任�����������ָ��
�������أ���
*/
void Inv_Perspective_Map_point(const uint8_t i,const uint8_t j, int *x, int *y) {
    *x = (int)((change_un_Mat[0][0] * i
       + change_un_Mat[0][1] * j + change_un_Mat[0][2])
       / (change_un_Mat[2][0] * i + change_un_Mat[2][1] * j
       + change_un_Mat[2][2]));
    *y = (int)((change_un_Mat[1][0] * i
       + change_un_Mat[1][1] * j + change_un_Mat[1][2])
       / (change_un_Mat[2][0] * i + change_un_Mat[2][1] * j
       + change_un_Mat[2][2]));
}

/*
�������ƣ�Inv_Perspective_Map
����˵����������ͼ�������͸�ӱ任
����˵����
          image - ԭʼͼ������ָ��
          map_img - �任��ͼ������ָ��
�������أ���
*/
void Inv_Perspective_Map(const uint8_t (*image)[120][188], uint8_t (*map_img)[120][188]) {
    for (int i = 0; i < 188; i++) {
       for (int j = 0; j < 120; j++) {
          int x,y;
          Inv_Perspective_Map_point(i, j, &x, &y);
          if (x < 0 || x >= 188) { continue; };
          if (y < 0 || y >= 120) { continue; };
          //���������͸��ͼ��ķ�Χ����ô�����п�����

          (*map_img)[j][i] = (*image)[y][x];

     }
   }
}

/*
�������ƣ�line_clean
����˵���������������
����˵����
          self - �����ṹ��ָ��
�������أ���
*/
void line_clean(line *self){
    self->length = 0;  // ������������Ϊ0
}

/*
�������ƣ�line_append
����˵��������������ӵ�
����˵����
          self - �����ṹ��ָ��
          p - Ҫ��ӵĵ�����
�������أ���
*/
void line_append(line *self, point p) {
    if(self->length < LINE_MAX_LENGTH){
        self->data[self->length] = p;  // ��ӵ㵽������������
        self->length++;                // �������ȼ�1
    }
}

/*
�������ƣ�draw_point
����˵������TFT��Ļ�ϻ��Ƶ�����
����˵����
          self - ��ṹ��ָ��
          color - �����ɫ��RGB565��ʽ��
�������أ���
*/
void draw_point(const point * self,const uint16_t color){
    uint16_t tar_row = 0,tar_col = 0;
    tar_row = (self->x) * 128 / 120;  // ����ת������Ӧ��Ļ�ߴ磩
    tar_col = (self->y) * 160 / 188;
    tft180_draw_point(tar_col,tar_row,color);  // ����Ļ�ϻ��Ƶ�
}

/*
�������ƣ�draw_line
����˵������TFT��Ļ�ϻ�������
����˵����
          self - �����ṹ��ָ��
          color - ������ɫ��RGB565��ʽ��
�������أ���
*/
void draw_line(const line * const self, uint16_t color){
    for(int cur = 0;cur < self->length;cur++){
        draw_point(&(self->data)[cur],color);  // ���λ��������е�ÿ����
    }
}
