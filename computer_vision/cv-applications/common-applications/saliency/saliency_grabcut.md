# 在 saliency map 的基础上得到原图的前景图  

So here we smooth the back-projection image with mean shift, enhance the contrast of the saliency map with histogram equalization, and invert the image.


The goal is to produce a smooth saliency map where salient regions have bright pixels.


saliency.py 文件的实现.   

使用 Mean-Shift 算法进行 smooth,, 然后使用直方图均衡化对 saliency map 的对比度加强, 最后反转图像的像素. 

我们的目的是生成一张在显著特征区域有很亮像素的 saliency map.   

下面这段代码是使用 OpenCV 中的 GrabCut 算法获取图像中的前景像素.   

~~~cpp
cv::Mat bgmodel, fgmodel, mask;
cv::Rect rect = cv::Rect(cv::Point(xx,yy), cv::Point(xx+ww,yy+hh));
cv::grabCut(im, mask, rect, bgmodel, fgmodel, 5, cv::GC_INIT_WITH_RECT);
cv::compare(mask, cv::GC_PR_FGD, mask, cv::CMP_EQ);
cv::Mat foreground(im.size(), CV_8UC3, cv::Scalar(255, 255, 255));
im.copyTo(foreground, mask);
cv::imshow("saliency", mask);
cv::waitKey();
~~~

## 使用 GrabCut 的注意事项  

1) 矩形框必须包含所有的前景像素, 因为矩形框之外的像素会被当做背景像素处理.   



## 怎么获取轮廓   

### 1. 轮廓提取
findContours发现轮廓

~~~cpp
findContours(
    InputOutputArray  binImg,       // 输入8bit图像，0值像素值不变，非0的像素看成1；（变为二值图像）
    OutputArrayOfArrays  contours,  // 输出找到的轮廓对象
    OutputArray hierachy            // 图像的拓扑结构
    int mode,                       // 轮廓返回的模式(RETR_TREE等)
    int method,                     // 发现方法(CHAIN_APPROX_SIMPLE等)
    Point offset=Point()            // 轮廓像素的位移（默认没有位移（0, 0））
)
~~~

【报错问题】   

findContours() 有时会报告“已触发了一个断点”等错误，尝试过有效的解决方法有：  

1.为vector提前申请一定的空间，如

std::vector<std::vector<Point>> contours(500)

2.Debug版切换为Release版；

### 2. drawContours 绘制轮廓

OpenCV 同时提供了轮廓的绘制函数.   

~~~cpp
drawContours(
    InputOutputArray  binImg,       // 输出图像
    OutputArrayOfArrays  contours,  // 找到的全部轮廓对象
    Int contourIdx                  // 轮廓索引号
    const Scalar & color,           // 绘制颜色
    int  thickness,                 // 绘制线宽
    int  lineType ,                 // 线的类型（默认8）
    InputArray hierarchy,           // 拓扑结构图
    int maxlevel,                   // 最大层数（0只绘制当前的，1表示绘制绘制当前及其内嵌的轮廓）
    Point offset=Point()            // 轮廓位移
    )
~~~

示例：

~~~cpp
#include<opencv2/opencv.hpp>
using namespace cv;

int main()
{
    Mat src,dst;
    src = imread("E:/image/image/shape.jpg");
    if(src.empty())
    {
        printf("can not load image \n");
        return -1;
    }
    namedWindow("input", CV_WINDOW_AUTOSIZE);
    imshow("input", src);
    dst = Mat::zeros(src.size(), CV_8UC3);

    blur(src,src,Size(3,3));
    cvtColor(src,src,COLOR_BGR2GRAY);
    Canny(src, src, 20, 80, 3, false);
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(src, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    RNG rng(0);
    for(int i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
        drawContours(dst, contours, i, color, 2, 8, hierarchy, 0, Point(0,0));
    }
    namedWindow("output", CV_WINDOW_AUTOSIZE);
    imshow("output",dst);
    waitKey();
    return 0;
}
~~~

这里写图片描述这里写图片描述

使用opencv3时(测试用opencv3.1.0)发现，cv命名空间下没有了vector，而在opencv2中(测试用opencv2.4.10)还存在。后查看各自的头文件发现： 
opencv.hpp头文件中包含着core.hpp（#include “opencv2/core.hpp”）; 
而在opencv2的core.hpp中包含有

........
#include <map>
#include <new>
#include <string>
#include <vector>
.......
等头文件，但在opencv3的core.hpp中删去这些包含项。 
因此在使用opencv3时cv命名空间下没有了vector。

使用 opencv2.4.10 时可以写：

~~~cpp
#include<opencv2/opencv.hpp>
using namespace cv;

int main()
{
    Mat src,dst;
    src = imread("E:/image/image/shape.jpg");
    if(src.empty())
    {
        printf("can not load image \n");
        return -1;
    }
    namedWindow("input", CV_WINDOW_AUTOSIZE);
    imshow("input", src);
    dst = Mat::zeros(src.size(), CV_8UC3);
    blur(src,src,Size(5,5));
    Canny(src, src, 20, 80, 3, false);
    vector<vector<Point>>contours;
    vector<Vec4i> hierarchy;
    findContours(src, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    RNG rng(0);
    for(int i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
        drawContours(dst, contours, i, color, 2, 8, hierarchy, 0, Point(0,0));
    }
    namedWindow("output", CV_WINDOW_AUTOSIZE);
    imshow("output",dst);
    waitKey();
    return 0;
}
~~~

绘制轮廓外矩形框
常用绘制轮廓外形状的函数： 
cv::boundingRect(InputArray points)绘制一个矩形（轮廓周围最小矩形左上角点和右下角点） 
cv::minAreaRect(InputArray points)绘制轮廓周围最小旋转矩形 
cv::minEnclosingCircle(InputArray points, Point2f& center, float& radius)//绘制轮廓周围最小圆形 
cv::fitEllipse(InputArray points)绘制轮廓周围最小椭圆

绘制轮廓外矩形框：

~~~cpp
#include<opencv2/opencv.hpp>
using namespace cv;

int main()
{
    Mat src,dst;
    src = imread("E:/image/shape.jpg");
    if(src.empty())
    {
        printf("can not load image \n");
        return -1;
    }
    namedWindow("input", CV_WINDOW_AUTOSIZE);
    imshow("input", src);
    dst = Mat::zeros(src.size(), CV_8UC3);
    std::vector<std::vector<Point>>contours;
    std::vector<Vec4i> hierarchy;
    blur(src,src,Size(3,3));
    cvtColor(src,src,COLOR_BGR2GRAY);
    Canny(src, src, 20, 80, 3, false);
    findContours(src, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    RNG rng(0);
    std::vector<std::vector<Point>> contoursPloy(contours.size());
    std::vector<RotatedRect> minRects(contours.size());

    for(int i = 0; i < contours.size(); i++)
    {
        minRects[i] = minAreaRect(Mat(contours[i]));
        Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
        drawContours(dst, contoursPloy, i, color, 1,8,std::vector<Vec4i>(), 0, Point(0, 0));
        Point2f rectPoints[4];
        minRects[i].points(rectPoints);
        for (int j = 0; j < 4; j++)
        {
            line(dst, rectPoints[j], rectPoints[(j+1)%4], color, 1, 8, 0);
        }
    }

    namedWindow("output", CV_WINDOW_AUTOSIZE);
    imshow("output",dst);
    waitKey();
    return 0;
}
~~~

这里写图片描述这里写图片描述

轮廓筛选
图像处理中可以使用轮廓的面积和长度等特征对轮廓进行筛选。

~~~cpp
moments(
    InputArray  array,//输入数据
    bool   binaryImage=false //是否为二值图像
)
contourArea(
    InputArray  contour,//输入轮廓数据
    bool   oriented//返回绝对值（默认false）
)

arcLength(
    InputArray  curve,//输入轮廓
    bool   closed// 轮廓否是封闭曲线
)
~~~

轮廓筛选示例：

使用轮廓的面积和长度特征对轮廓进行筛选后用外接矩形将筛选后的轮廓框选出来。

~~~cpp
#include<opencv2/opencv.hpp>
using namespace cv;

void trackBar(int,void*);

Mat src,dst;
std::vector<std::vector<Point>>contours;
int area = 0, length = 0;
int main()
{

    src = imread("E:/image/shape.jpg");
    if(src.empty())
    {
        printf("can not load image \n");
        return -1;
    }
    namedWindow("input", CV_WINDOW_AUTOSIZE);
    imshow("input", src);
    dst = Mat::zeros(src.size(), CV_8UC3);

    std::vector<Vec4i> hierarchy;
    blur(src,dst,Size(3,3));
    cvtColor(dst,dst,COLOR_BGR2GRAY);
    Canny(dst, dst, 20, 80, 3, false);
    findContours(dst, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
    namedWindow("output", CV_WINDOW_AUTOSIZE);

    createTrackbar("area:", "output", &area,150,trackBar);
    createTrackbar("length:", "output", &length,150,trackBar);

    waitKey();
    return 0;
}
~~~

~~~cpp
void trackBar(int,void*)
{
    Mat src1 = src.clone();
    RNG rng(0);
    std::vector<std::vector<Point>> contoursPloy(contours.size());
    std::vector<RotatedRect> minRects(contours.size());

    for(int i = 0; i < contours.size(); i++)
    {
        if(contourArea(contours[i]) > area && arcLength(contours[i], false) > length)
        {
            minRects[i] = minAreaRect(Mat(contours[i]));
            Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
            //drawContours(dst, contoursPloy, i, color, 1,8,vector<Vec4i>(), 0, Point(0, 0));
            Point2f rectPoints[4];
            minRects[i].points(rectPoints);
            for (int j = 0; j < 4; j++)
            {
                line(src1, rectPoints[j], rectPoints[(j+1)%4], color, 2, 8, 0);
            }
        }
    }
    imshow("output",src1);
    src1 = src;
}
~~~