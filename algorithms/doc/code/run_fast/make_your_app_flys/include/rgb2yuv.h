#ifdef _RGB_TO_YUY_H_
#define _RGB_TO_YUY_H_

#define XSIZE 640
#define YSIZE 480
#define IMGSIZE XSIZE * YSIZE  

typedef struct RGB
{
       unsigned char R;
       unsigned char G;
       unsigned char B;
}RGB;

struct RGB in[IMGSIZE];         // 需要计算的原始数据
unsigned char out[IMGSIZE];     // 计算后的结果


#endif