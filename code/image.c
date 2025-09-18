/*
 * image.c
 *
 *  Created on: 2024年7月13日
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

//大津法二值化
void otsuThreshold(const uint8_t (*mat)[120][188], uint8_t (*bin_img)[120][188])
{
    #define GrayScale 256
    int pixelCount[GrayScale] = {0}; // 每个灰度值所占像素个数
    float pixelPro[GrayScale] = {0}; // 每个灰度值所占总像素比例
    int i, j;
    int Sumpix = 188 * 120; // 总像素点
    uint8_t threshold = 0; // 使用uint8_t以确保返回值类型正确

    // 统计灰度级中每个像素在整幅图像中的个数
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
        pixelPro[i] = (float)pixelCount[i] / Sumpix; // 计算每个像素在整幅图像中的比例
        u += i * pixelPro[i]; // 总平均灰度
    }

    float maxVariance = 0.0; // 最大类间方差
    float w0 = 0, avgValue = 0; // w0 前景比例，avgValue 前景平均灰度
    for (int i = 0; i < GrayScale; i++) // 每一次循环都是一次完整类间方差计算 (两个for叠加为1个)
    {
        w0 += pixelPro[i]; // 假设当前灰度i为阈值, 0~i 灰度像素所占整幅图像的比例即前景比例
        avgValue += i * pixelPro[i];

        float variance = pow((avgValue / w0 - u), 2) * w0 / (1 - w0); // 类间方差
        if (variance > maxVariance)
        {
            maxVariance = variance;
            threshold = (uint8_t)i; // 明确转换为uint8_t类型
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

//局部二值化
void adaptiveThreshold(const uint8_t (*mat)[120][188], uint8_t (*bin_img)[120][188], int block, uint8_t clip_value){
  zf_assert(block % 2 == 1); // block必须为奇数
  int half_block = block / 2;
  for(int y=half_block; y<120-half_block; y++){
    for(int x=half_block; x<188-half_block; x++){
      // 计算局部阈值
      int thres = 0;
      for(int dy=-half_block; dy<=half_block; dy++){
        for(int dx=-half_block; dx<=half_block; dx++){
          thres += (*mat)[y+dy][x+dx];
        }
      }
      thres = thres / (block * block) - clip_value;
      // 进行二值化
      (*bin_img)[y][x] = (*mat)[y][x]>thres ? 255 : 0;
    }
  }
}

/*
函数名称：MapImagshow_UpperComputer
功能说明：将地图图像通过UART发送到上位机显示
参数说明：
          map_image - 要发送的地图图像数组指针
函数返回：无
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
函数名称：Inv_Perspective_Map_point
功能说明：对单个点进行逆透视变换
参数说明：
          i, j - 原始点坐标
          x, y - 变换后点坐标的输出指针
函数返回：无
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
函数名称：Inv_Perspective_Map
功能说明：对整个图像进行逆透视变换
参数说明：
          image - 原始图像数组指针
          map_img - 变换后图像数组指针
函数返回：无
*/
void Inv_Perspective_Map(const uint8_t (*image)[120][188], uint8_t (*map_img)[120][188]) {
    for (int i = 0; i < 188; i++) {
       for (int j = 0; j < 120; j++) {
          int x,y;
          Inv_Perspective_Map_point(i, j, &x, &y);
          if (x < 0 || x >= 188) { continue; };
          if (y < 0 || y >= 120) { continue; };
          //如果超出逆透视图像的范围，那么不进行拷贝。

          (*map_img)[j][i] = (*image)[y][x];

     }
   }
}

/*
函数名称：line_clean
功能说明：清空线条数据
参数说明：
          self - 线条结构体指针
函数返回：无
*/
void line_clean(line *self){
    self->length = 0;  // 重置线条长度为0
}

/*
函数名称：line_append
功能说明：向线条中添加点
参数说明：
          self - 线条结构体指针
          p - 要添加的点坐标
函数返回：无
*/
void line_append(line *self, point p) {
    if(self->length < LINE_MAX_LENGTH){
        self->data[self->length] = p;  // 添加点到线条数据数组
        self->length++;                // 线条长度加1
    }
}

/*
函数名称：draw_point
功能说明：在TFT屏幕上绘制单个点
参数说明：
          self - 点结构体指针
          color - 点的颜色（RGB565格式）
函数返回：无
*/
void draw_point(const point * self,const uint16_t color){
    uint16_t tar_row = 0,tar_col = 0;
    tar_row = (self->x) * 128 / 120;  // 坐标转换（适应屏幕尺寸）
    tar_col = (self->y) * 160 / 188;
    tft180_draw_point(tar_col,tar_row,color);  // 在屏幕上绘制点
}

/*
函数名称：draw_line
功能说明：在TFT屏幕上绘制线条
参数说明：
          self - 线条结构体指针
          color - 线条颜色（RGB565格式）
函数返回：无
*/
void draw_line(const line * const self, uint16_t color){
    for(int cur = 0;cur < self->length;cur++){
        draw_point(&(self->data)[cur],color);  // 依次绘制线条中的每个点
    }
}
