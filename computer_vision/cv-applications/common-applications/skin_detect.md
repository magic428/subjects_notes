# 图像直方图和反向投影的肤色检测

肤色的检测可以利用肤色的HSV模型，通过计算图像的 HSV 模型中肤色的 H 和 S 分量中的一个或两个的直方图，再用该直方图反向投影至原图像，定位出肤色。   

HSV颜色模型：   

RGB模型是图像处理中常用的颜色模型，多用于颜色显示和图像处理，三维坐标中H(Hue)代表色调，S(Saturation)代表饱和度，V代表(Value)明度，理解起来很容易，是一种针对用户观感的一种颜色模型，侧重于色彩表示，什么颜色、深浅如何、明暗如何，跟人眼的直观感受很契合。   

对于肤色的直方图，只计算H和S分量的直方图，所以肤色的明暗度对检测结果影响很小，提高了对不同灰度的鲁棒性。   

检测的基本步骤：   
1. 截取一部分肤色区域，作为检测目标的样本；   
2. 对样本做直方图计算，并归一化直方图；   
3. 利用归一化后的直方图，通过calcBackProject函数在原图像中检索，与样本具有表现一致的直方图区域将会被作为肤色检测出来；   

## 关键函数 - calcHist、normalize 、calcBackProject    

calcHist 函数之前已经做过介绍，函数原型及参数说明可以参考：[直方图该怎么画](https://blog.csdn.net/dcrmg/article/details/52049917)    

```cpp
void normalize( InputArray src, OutputArray dst, double alpha=1, double beta=0,  
                             int norm_type=NORM_L2, int dtype=-1, InputArray mask=noArray());  
```

normalize 函数作用是归一化计算出来的直方图。   

第一个参数：InputArray类型的Src，输入的直方图数组；  
第二个参数：OutputArray类型的dst，输出的归一化后的数组；  
第三个参数：double型的alpha，归一化数组的最小值；  
第五个参数：int（枚举）型的norm_type，归一化方法；  
第六个参数：int型的deype，值为-1， 指示归一化后的输出数组与输入数组类型相同    
第七个参数：Mat(): 可选的掩码.    

calcBackProject函数原型：    

```cpp
calcBackProject(&image,       //目标图像    
                1,            // 图像个数    
                channels,     // 通道数量    
                histogram,    // 进行反投影的直方图    
                result,       // 结果图像    
                ranges,       // 每个维度的阈值    
                255.0         // 放缩因子    
                );    
```

## 代码实现：    

```cpp
#include "core/core.hpp"  
#include "highgui/highgui.hpp"  
#include "imgproc/imgproc.hpp"  
  
using namespace cv;  
  
Mat image,imageHSV,imageHist,imageNorm,imagecalcBack;  
Mat imagegirl,imagegirlHSV;  
  
int histSize=1;  
float histR[]={0,255};  
const float *histRange=histR;  
  
int channels[]={0,1};  
  
void TrackBarFun(int ,void(*));  
  
int main(int argc,char *argv[])  
{  
    image=imread(argv[1]);  
    imagegirl=imread(argv[2]);  
    cvtColor(imagegirl,imagegirlHSV,CV_RGB2HSV);  
    if(!image.data)  
    {  
        return -1;  
    }  
    cvtColor(image,imageHSV,CV_RGB2HSV);  
    namedWindow("HSV");  
    createTrackbar("bins控制","HSV",&histSize,100,TrackBarFun);     
    TrackBarFun(0,0);  
    waitKey();  
}  
void TrackBarFun(int ,void(*))  
{  
    if(histSize==0)  
    {  
        histSize=1;  
    }  
    calcHist(&imageHSV,2,channels,Mat(),imageHist,1,&histSize,&histRange,true,false);  
    normalize(imageHist,imageNorm,0,255,NORM_MINMAX,-1,Mat());  
    calcBackProject(&imagegirlHSV,2,channels,imageNorm,imagecalcBack,&histRange,1,true);      
    imshow("Source",imagegirl);  
    imshow("HSV",imagegirlHSV);  
    imshow("CalcBack",imagecalcBack);  
}  
```
效果如下，拖动滑动条，在 bins 数目为 7 的时候，效果比较好：   


 