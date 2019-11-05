# 暗通道先验去雾算法原理及C++代码实现 - 何凯明

在图像去雾这个领域, 几乎没有人不知道[《Single Image Haze Removal Using Dark Channel Prior》](https://www.robots.ox.ac.uk/~vgg/rg/papers/hazeremoval.pdf) 这篇文章是 2009 年 CVPR 最佳论文, 作者何凯明.  

关于何博士的一些资料和论文, 大家可以访问这里：http://kaiminghe.com/.   

本文主要是对《Single Image Haze Removal Using Dark Channel Prior》的翻译、整理、及部分解释、代码实现。如果您的英文水平好, 建议看原文可能来的更爽些。   

## 一、论文思想的简单描述 

首先看看暗通道先验是什么.

在绝大多数非天空的局部区域里, 某一些像素总会有至少一个颜色通道具有很低的值。换言之, 该区域光强度的最小值是个很小的数。我们给暗通道一个数学定义, 对于任意的输入图像 $J$, 其暗通道可以用下式表达：   

$$
J^{dark}(x) = \min_{y \in \Omega(x)}(\min_{C \in \{r,g,b\}}J^C(y))
$$

式中 $J^C$ 表示彩色图像的每个通道 , $\Omega(x)$ 表示以像素 $x$ 为中心的一个窗口。  

上式的意义用代码表达也很简单, 首先求出每个像素 RGB 分量中的最小值, 存入一幅和原始图像大小相同的灰度图中, 然后再对这幅灰度图进行最小值滤波, 滤波的半径由窗口大小决定, 一般有 WindowSize = 2 * Radius + 1;   

**暗通道先验的理论指出：** 

$$
J^{dark} \rightarrow 0  
$$

实际生活中造成暗通道中低像素值主要有三个因素：   
a) 汽车、建筑物和城市中玻璃窗户的阴影, 或者是树叶、树与岩石等自然景观的投影；
b) 色彩鲜艳的物体或表面, 在 RGB 的三个通道中有些通道的值很低（比如绿色的草地／树／植物, 红色或黄色的花朵／叶子, 或者蓝色的水面）；   
c) 颜色较暗的物体或者表面, 例如灰暗色的树干和石头。总之, 自然景物中到处都是阴影或者彩色, 这些景物的图像的暗原色总是很灰暗的。   

在作者的论文中, 统计了 5000 多副图像的特征, 也都基本符合这个先验, 因此, 我们可以认为这其实一条 `定理`。   

有了这个先验, 接着就需要进行一些数学方面的推导来最终解决问题。首先, 在计算机视觉和计算机图形中, 下述方程所描述的雾图模型被广泛使用：   

$$
I(x)  = J(x)t(x) + A·(1-t(x)) 
$$

其中, $I(x)$ 就是我们现在已经有的图像（待去雾的图像）, $J(x)$ 是我们要恢复的无雾的图像, $A$ 是全球大气光成分, $t(x)$ 为透射率。   

现在的已知条件就是: $I(x)$, 要求目标值 $J(x)$, 显然这个方程有无数解, 因此就需要一些先验了。   

将上式稍作处理, 变形为：   

$$
\frac{I^C(x)}{A^C}  = \frac{J^C(x)}{A^C} t(x) + 1-t(x) 
$$
如上所述, 上标 $C$ 表示 R/G/B 三个通道。   

首先假设在每一个窗口内透射率 $t(x)$ 为常数, 定义为 $\tilde{t}(x)$, 并且 $A$ 值已经给定, 然后对上式两边求两次最小值运算, 得到下式：   

$$
\min_{y \in \Omega(x)}(\min_{C \in \{r,g,b\}}\frac{I^C(y)}{A^C}) = \min_{y \in \Omega(x)}(\min_{C \in \{r,g,b\}}\frac{J^C(x)}{A^C})\tilde{t}(x) + 1-\tilde{t}(x)
$$

上式中, $J$ 是待求的无雾的图像, 根据前述的暗原色先验理论有：    

$$
J^{dark}(x) = \min_{y \in \Omega(x)}(\min_{C \in \{r,g,b\}}J^C(y)) = 0
$$

因此可得：  

$$
 \min_{y \in \Omega(x)}(\min_{C \in \{r,g,b\}}\frac{J^C(x)}{A^C}) = 0
$$

因此可得：  

$$
\tilde{t}(x) = 1-\min_{y \in \Omega(x)}(\min_{C \in \{r,g,b\}}\frac{I^C(y)}{A^C})
$$

这就得到了透射率 $\tilde{t}(x)$ 的预估值。   

在现实生活中, 即使是晴天白云, 空气中也存在着一些颗粒, 因此看远处的物体还是能感觉到雾的影响, 另外, 雾的存在让人类感到景深的存在, 因此有必要在去雾的时候保留一定程度的雾, 这可以通过在上式中引入一个在[0,1] 之间的因子, 则上式修正为：

$$
\tilde{t}(x) = 1-\omega\min_{y \in \Omega(x)}(\min_{C \in \{r,g,b\}}\frac{I^C(y)}{A^C})
$$                                              

本文中所有的测试结果依赖于： $\omega=0.95$。   

上述推论中都是假设全球达气光 $A$ 值是已知的, 但是其实这个值也是需要从有雾图像中计算得到. 具体步骤如下：   

1） 从暗通道图中按照亮度的大小取前 0.1% 的像素。
2） 在这些位置中, 在原始有雾图像 $I(x)$ 中寻找对应的具有最高亮度点的值, 作为 $A$ 值。

现在 $I,A,t$ 是已知条件, 我们就可以进行无雾图像 $J$ 的恢复了.   

当投射图 $t$ 的值很小时, 会导致 $J$ 的值偏大, 从而使得图像整体向白场过渡, 因此一般可设置一阈值 $t_0$, 当 $t$ 值小于 $t_0$ 时, 令 $t = t_0$, 本文中所有效果图均以 $t_0 = 0.1$ 为标准计算。   

因此, 最终的恢复公式如下：

$$
J(x)  = \frac{I(x) - A}{\text{max}(t(x), t_0)} + A
$$

当直接用上述理论进行恢复时, 去雾的效果其实也是很明显的, 比如下面一些例子：   

![](http://images.cnitblog.com/blog/349293/201308/23145239-17545837cecf4a1ea502db6d6c9d83e0.x-png)
<center>有雾图</center>

![](http://images.cnitblog.com/blog/349293/201308/23171710-270977890b794c6e96ba51aebd573f24.x-png)
<center>去雾图</center>

注意到第一幅图的原图两个字的周围明显有一块不协调的地方, 而第二图顶部水平方向似乎有一块没有进行去雾处理, 这些都是由于我们的`透射率图过于粗糙`了。  

要获得更为精细的透射率图, 何博士在文章中提出了了 soft matting 方法, 能得到非常细腻的结果。但是他的一个致命的弱点就是速度慢, 不使用于实际使用。在 2011 年, 何博士又出了一片论文, 其中提到了`导向滤波`的方式来获得较好的透射率图。该方法的主要过程集中于简单的方框模糊, 而方框模糊有多重合半径无关的快速算法。因此, 算法的实用性增强, 关于这个导向滤波算法大家在何博士的网站可以自己去研习下, 除了在去雾方面外, 还有其他多方面的应用。   

使用了导向滤波后的去雾效果：
![](http://images.cnitblog.com/blog/349293/201308/23173327-a9c620f3117a455db709bee42f22da7b.x-png)

## 二、各参数对去雾结果的影响   

### 1. 窗口的大小   

这个对结果来说是个关键的参数, 窗口越大, 其包含暗通道的概率越大, 暗通道也就越黑。我们不去从理论角度分析, 从实践的效果来看, 似乎窗口越大, 去雾的效果越不明显.

建议窗口大小在 `11-51` 之间, 即半径在 `5-25` 之间。   

### 2. $\omega$ 参数

公式中的 $\omega$ 具有着明显的意义, 其值越小, 去雾效果越不明显.   


## 三. 编码的步骤   

如果你仔细的分析了原文的细路, 加上适当的参考, 编码其实并不是很困难。   

### 1. 算法实现流程   
1） 根据原始图像求暗通道。   
2） 按文中所描述的算法自动获得全球大气光的值。   
这里说明一点, 原始论文中的A最终是取原始像素中的某一个点的像素, 我实际上是取的符合条件的所有点的平均值作为 A 的值, 我这样做是因为, 如果是取一个点, 则各通道的A值很有可能全部很接近 255, 这样的话会造成处理后的图像偏色和出现大量色斑。原文作者说这个算法对天空部分不需特备处理, 我实际发现该算法对有天空的图像的效果一般都不好。天空会出现明显的过渡区域。作为解决方案, 我增加了一个参数, 最大全球大气光值, 当计算的值大于该值时, 则就取该值。 　 
3） 按公式计算预估的透射率图。   
这里计算预估的透射率图时, 每个通道的数据都需要除以对应的 A 值, 即归一化, 这样做, 还存在一个问题, 由于A的选取过程, 并不能保证每个像素分量值除以 A 值后都小于 1, 从而导致t的值可能小于 0, 而这是不容许的, 原文作者并没有交代这一点是如何处理的。我在实际的编码中发现, 如果真的这样做了, 其效果也并不是很理想, 因此我最后的办法是在式（12）中, 不考虑 A 的计算。
4）计算导向滤波图。
这里可以直接用原始的图像做导向图, 当然也可以用其灰度图, 但是用RGB导向图在下一步的计算中会占用比较大的时间。
5）按照《Guided Image Filtering》论文中的公式（5）、（6）、（8）编码计算获得精细的透射率图。c++代码见上一篇文章。   
还有一点就是, 上述计算需要在 [0,1] 范围内进行, 也就是说导向图和预估的透射率图都必须从 [0,255] 先映射到 [0,1] 再进行计算。关于 guidedfilter 中半径 r 的值, 因为在前面进行最小值后暗通道的图像成一块一块的, 为了使透射率图更加精细, 建议这个 r 的取值不小于进行最小值滤波的半径的 4 倍. 当 r 比较小的时候, 在透射率图中基本看不到什么细节信息, 因此恢复处的图像边缘处不明显。  
参数 eps 的取值也有所讲究, 他主要是为了防止计算中除以 0 的错误以及为了使得某些计算结果不至于过大, 一般建议取值 0.001 或者更小。  
如果使用的彩色 RGB 图做导向图, 计算时间上会增加不少, 所的到的透射率图的边缘会比灰度图所处理的保留了更多的细节, 效果上略微比灰度图好。  
以 RGB 图为导向图的计算中, 涉及到 3*3 部分矩阵求逆的过程, 如果用非 matlab 语言写, 可以先借助于 matlab 的符号计算功能, 以及其中的符号计算命令 simple, 把计算结果算出来, 然后再再其他高级语言中实现。  
6) 按式（22）进行无雾图像的恢复。  

### 2. MFC 头文件   
```cpp
#pragma once  
#include "afxwin.h"  
#include <cv.h>  
#include "cxcore.h"    
#include "math.h"   
#include <highgui.h>  
#include<vector>  
#include <iostream>    
#include "opencv2/core/core.hpp"      
#include "opencv2/highgui/highgui.hpp"      
#include "opencv2/imgproc/imgproc.hpp"      
  
  
using namespace std;  
using namespace cv;  
  
class Ctry :  
    public CCmdTarget  
{  
public:  
    Ctry();  
    virtual ~Ctry();  
    DECLARE_MESSAGE_MAP()  
    afx_msg void OnTryTyr1();  
    afx_msg void OnTryPath();  
  
public:  
    Mat getimage(Mat &a);  
    Mat guidedFilter2(cv::Mat I, cv::Mat p, int r, double eps);  
    IplImage* getDarkChannel(IplImage* &src);  
    IplImage* getMinIcy(IplImage* dark,int w);  
    double getA(IplImage* dark, IplImage*hazeImage);  
    IplImage* getTransmission(IplImage* Icy, double Ac);  
  
    IplImage* getDehazedImage(IplImage* hazeImage, IplImage* guidedt,double Ac);  
  
public:  
  
  
  
  
};  
```

### 3. MFC 源文件 

```cpp
#include "stdafx.h"  
#include "Ctry.h"  
#include "Resource.h"  
#include<cv.h>  
  
#define PI 3.14159  
  
Ctry::Ctry()  
{  
}  
  
Ctry::~Ctry()  
{  
}  
  
BEGIN_MESSAGE_MAP(Ctry, CCmdTarget)  
    ON_COMMAND(ID_TRY_TYR1, &Ctry::OnTryTyr1)  
    ON_COMMAND(ID_TRY_PATH, &Ctry::OnTryPath)  
END_MESSAGE_MAP()  
  
  
void Ctry::OnTryTyr1()  
{  
    // TODO:  在此添加命令处理程序代码  
  
      
}  
  
void Ctry::OnTryPath()  
{  
    // TODO:  在此添加命令处理程序代码      
    IplImage* img = cvLoadImage("C:\\Users\\徐图之\\Desktop\\1.jpg");  
  
  
    IplImage* g = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);  
    g=getDarkChannel(img);  
    double A = getA(g, img);   //大气光强A  
  
    IplImage* Icy = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);  
    Icy = getMinIcy(g, 5);  
  
    //投射图t  
    IplImage* t = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);  
    t = getTransmission(Icy, A);  
  
    //获得guide image  
    Mat mt = cvarrToMat(t, true);  
    Mat image_src = cvarrToMat(img, true);  
    Mat image_gray(image_src.size(), CV_8UC1);  
    cvtColor(image_src, image_gray, CV_BGR2GRAY);  
    Mat guide = getimage(image_gray);  
    int r = 8;  
    double eps = 0.04;  
    Mat q = guidedFilter2(guide, mt, r, eps);  
    IplImage* guidedt = cvCloneImage(&(IplImage)q);  
  
  
    IplImage* dehazedImage = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);  
    dehazedImage = getDehazedImage(img, guidedt, A);  
  
  
    cvSaveImage("C:\\Users\\徐图之\\Desktop\\dark .jpg", g);  
    cvSaveImage("C:\\Users\\徐图之\\Desktop\\guidedt .jpg", guidedt);  
    cvSaveImage("C:\\Users\\徐图之\\Desktop\\t.jpg", t);  
    cvSaveImage("C:\\Users\\徐图之\\Desktop\\dehazedImage84.jpg", dehazedImage);  
      
}  
  
  
//convert image depth to CV_64F    
Mat  Ctry::getimage(Mat &a)  
{  
    int hei = a.rows;  
    int wid = a.cols;  
    Mat I(hei, wid, CV_64FC1);  
    //convert image depth to CV_64F    
    a.convertTo(I, CV_64FC1, 1.0 / 255.0);  
    return I;  
}  
  
  
  
Mat  Ctry::guidedFilter2(cv::Mat I, cv::Mat p, int r, double eps)  
{  
    /* 
    % GUIDEDFILTER   O(1) time implementation of guided filter. 
    % 
    %   - guidance image: I (should be a gray-scale/single channel image) 
    %   - filtering input image: p (should be a gray-scale/single channel image) 
    %   - local window radius: r 
    %   - regularization parameter: eps 
    */  
  
    cv::Mat _I;  
    I.convertTo(_I, CV_64FC1);  
    I = _I;  
  
    cv::Mat _p;  
    p.convertTo(_p, CV_64FC1);  
    p = _p;  
  
    //[hei, wid] = size(I);    
    int hei = I.rows;  
    int wid = I.cols;  
  
    //N = boxfilter(ones(hei, wid), r); % the size of each local patch; N=(2r+1)^2 except for boundary pixels.    
    cv::Mat N;  
    cv::boxFilter(cv::Mat::ones(hei, wid, I.type()), N, CV_64FC1, cv::Size(r, r));  
  
    //mean_I = boxfilter(I, r) ./ N;    
    cv::Mat mean_I;  
    cv::boxFilter(I, mean_I, CV_64FC1, cv::Size(r, r));  
  
    //mean_p = boxfilter(p, r) ./ N;    
    cv::Mat mean_p;  
    cv::boxFilter(p, mean_p, CV_64FC1, cv::Size(r, r));  
  
    //mean_Ip = boxfilter(I.*p, r) ./ N;    
    cv::Mat mean_Ip;  
    cv::boxFilter(I.mul(p), mean_Ip, CV_64FC1, cv::Size(r, r));  
  
    //cov_Ip = mean_Ip - mean_I .* mean_p; % this is the covariance of (I, p) in each local patch.    
    cv::Mat cov_Ip = mean_Ip - mean_I.mul(mean_p);  
  
    //mean_II = boxfilter(I.*I, r) ./ N;    
    cv::Mat mean_II;  
    cv::boxFilter(I.mul(I), mean_II, CV_64FC1, cv::Size(r, r));  
  
    //var_I = mean_II - mean_I .* mean_I;    
    cv::Mat var_I = mean_II - mean_I.mul(mean_I);  
  
    //a = cov_Ip ./ (var_I + eps); % Eqn. (5) in the paper;       
    cv::Mat a = cov_Ip / (var_I + eps);  
  
    //b = mean_p - a .* mean_I; % Eqn. (6) in the paper;    
    cv::Mat b = mean_p - a.mul(mean_I);  
  
    //mean_a = boxfilter(a, r) ./ N;    
    cv::Mat mean_a;  
    cv::boxFilter(a, mean_a, CV_64FC1, cv::Size(r, r));  
    mean_a = mean_a / N;  
  
    //mean_b = boxfilter(b, r) ./ N;    
    cv::Mat mean_b;  
    cv::boxFilter(b, mean_b, CV_64FC1, cv::Size(r, r));  
    mean_b = mean_b / N;  
  
    //q = mean_a .* I + mean_b; % Eqn. (8) in the paper;    
    cv::Mat q = mean_a.mul(I) + mean_b;  
  
    return q;  
}  
  
IplImage* Ctry::getDarkChannel(IplImage* &src)  
{  
    IplImage* temp = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);  
    CvScalar pixel;  
    double  px;  
    for (int i = 0; i < src->height; i++)  
    {  
        for (int j = 0; j < src->width; j++)  
        {  
            pixel = cvGet2D(src, i, j);  
            if (pixel.val[0]<pixel.val[1])  
            {  
                px = pixel.val[0];  
            }  
            else  
            {  
                px = pixel.val[1];  
            }  
  
            if (px >pixel.val[2])  
            {  
                px = pixel.val[2];  
            }  
            cvSetReal2D(temp, i, j, px);  
        }  
    }  
  
    return  temp;  
}  
  
double Ctry::getA(IplImage* dark, IplImage* hazeImage)  
{    
    double sum=0;   //像素点符合条件A的和  
    int pointNum = 0;   //满足要求的像素点数  
    double A;        //大气光强A  
    double pix;    //暗通道图中照亮度的前0.1%范围的像素值  
    CvScalar pixel;   //按图中符合A的点, 在雾图中对应的像素  
  
    float stretch_p[256], stretch_p1[256], stretch_num[256];  
    //清空三个数组,初始化填充数组元素为0      
    memset(stretch_p, 0, sizeof(stretch_p));  
    memset(stretch_p1, 0, sizeof(stretch_p1));  
    memset(stretch_num, 0, sizeof(stretch_num));  
  
    int nHeight = dark->height;  
    int nWidth = dark->width;  
    int i, j;  
    for (i = 0; i<nHeight; i++)  
    {  
        for (j = 0; j<nWidth; j++)  
        {  
            double  pixel0 = cvGetReal2D(dark, i, j);  
            int   pixel = (int)pixel0;  
            stretch_num[pixel]++;  
        }  
    }  
    //统计各个灰度级出现的概率    
    for (i = 0; i<256; i++)  
    {  
        stretch_p[i] = stretch_num[i] / (nHeight*nWidth);  
    }  
  
    //统计各个灰度级的概率,从暗通道图中按照亮度的大小取前0.1%的像素,pix为分界点  
    for (i = 0; i<256; i++)  
    {  
        for (j = 0; j <= i; j++)  
        {  
            stretch_p1[i] += stretch_p[j];  
            if (stretch_p1[i]>0.999)  
            {  
                pix = (double)i;  
                i = 256;  
                break;  
            }  
  
        }  
    }  
  
    for (i = 0; i< hazeImage->height; i++)  
    {  
        for (j = 0; j < hazeImage->width; j++)  
        {  
            double temp = cvGetReal2D(dark, i, j);  
            if (temp > pix)  
            {  
                pixel = cvGet2D(hazeImage, i, j);  
                pointNum++;  
                sum += pixel.val[0];  
                sum += pixel.val[1];  
                sum += pixel.val[2];  
  
            }  
        }  
    }  
    A = sum / (3 * pointNum);  
    if (A > 220.0)  
    {  
        A = 220.0;  
    }  
    return A;  
}  
  
//获取暗通道图像窗口中的最小值, 用于后续计算透射率t,参数w为窗口的大小  
IplImage* Ctry::getMinIcy(IplImage* dark, int w)  
{  
    IplImage* Icy = cvCreateImage(cvGetSize(dark), IPL_DEPTH_8U, 1);  
    int hei = dark->height;  
    int wid = dark->width;  
    int hw = hei / w;  
    int ww = wid / w;  
    for (int i = w; i < (hw - 1)*w; i += w)  
    {  
        for (int j = w; j < (ww - 1)*w; j += w)  
        {  
            double p = cvGetReal2D(dark, i-1, j-1);  //得到窗口最右下角的一个像素点  
            //得到窗口最小的像素值  
            for (int ii = i - w; ii < i; ii++)  
            {  
                for (int jj = j - w; jj < j; jj++)  
                {  
                    double newp = cvGetReal2D(dark, ii, jj);  
                    if (newp < p)  
                    {  
                        p = newp;  
                    }  
                }  
            }  
            //设置Icy的值  
            for (int ii = i - w; ii < i; ii++)  
            {  
                for (int jj = j - w; jj < j; jj++)  
                {  
                    cvSetReal2D(Icy, ii, jj, p);  
                }  
            }  
  
        }  
    }  
  
    //处理最右边一列  不包含最下一个子块  
    for (int i = w; i < (hw - 1)*w; i += w)  
    {  
        double p = cvGetReal2D(dark, i-1, wid-1);  //得到窗口最右下角的一个像素点  
        for (int ii = i - w; ii < i; ii++)  
        {  
  
            for (int j = (ww - 1)*w; j < wid; j++)  
            {  
                //得到窗口最小的像素值  
                double newp = cvGetReal2D(dark, ii, j);  
                if (newp < p)  
                {  
                    p = newp;  
                }  
            }  
        }     
  
        //设置Icy的值  
        for (int ii = i - w; ii < i; ii++)  
        {  
  
            for (int j = (ww - 1)*w; j < wid; j++)  
            {  
                cvSetReal2D(Icy, ii, j, p);  
            }  
        }  
    }  
  
  
    //处理最下一行 不包含最后一个子块  
    for (int j = w; j < (ww - 1)*w; j += w)  
    {  
        double p = cvGetReal2D(dark, hei-1, j);  //得到窗口最右下角的一个像素点  
        for (int i = (hw - 1)*w; i < hei; i++)  
        {  
            for (int jj = j - w; jj < j; jj++)  
            {  
                //得到窗口最小的像素值  
                double newp = cvGetReal2D(dark, i, jj);  
                if (newp < p)  
                {  
                    p = newp;  
                }  
            }  
        }  
  
        //设置Icy的值  
        for (int i = (hw - 1)*w; i < hei; i++)  
        {  
  
            for (int jj = j - w; jj < j; jj++)  
            {  
                cvSetReal2D(Icy, i, jj, p);  
            }  
        }  
  
    }  
  
    //处理最右下角的一个子块  
    double p = cvGetReal2D(dark, hei-1, wid-1);  //得到窗口最右下角的一个像素点  
    for (int i = (hw - 1)*w; i < hei; i++)  
    {  
        for (int j = (ww - 1)*w; j < wid; j++)  
        {  
            //得到窗口最小的像素值  
            double newp = cvGetReal2D(dark, i, j);  
            if (newp < p)  
            {  
                p = newp;  
            }  
  
        }  
    }  
    for (int i = (hw - 1)*w; i < hei; i++)  
    {  
        for (int j = (ww - 1)*w; j < wid; j++)  
        {  
            cvSetReal2D(Icy, i, j, p);  
  
        }  
    }  
  
    return Icy;  
}  
  
IplImage* Ctry::getTransmission(IplImage* Icy, double Ac)  
{  
    IplImage* t = cvCreateImage(cvGetSize(Icy), IPL_DEPTH_8U, 1);  
    for (int i = 0; i < t->height; i++)  
    {  
        for (int j = 0; j < t->width; j++)  
        {  
            double temp = cvGetReal2D(Icy, i, j);  
            double tempt = 1 - 0.95*temp / Ac;  
            cvSetReal2D(t, i, j, tempt*255);  
        }  
    }  
  
    return t;  
  
}  
  
IplImage* Ctry::getDehazedImage(IplImage* hazeImage, IplImage* guidedt, double Ac)  
{  
    IplImage* dehazedImage = cvCreateImage(cvGetSize(hazeImage), IPL_DEPTH_8U, 3);  
    IplImage* r = cvCreateImage(cvGetSize(hazeImage), IPL_DEPTH_8U, 1);  
    IplImage* g = cvCreateImage(cvGetSize(hazeImage), IPL_DEPTH_8U, 1);  
    IplImage* b = cvCreateImage(cvGetSize(hazeImage), IPL_DEPTH_8U, 1);  
  
    cvSplit(hazeImage, b, g, r, NULL);  
      
    IplImage* dehaze_r = cvCreateImage(cvGetSize(hazeImage), IPL_DEPTH_8U, 1);  
    IplImage* dehaze_g = cvCreateImage(cvGetSize(hazeImage), IPL_DEPTH_8U, 1);  
    IplImage* dehaze_b = cvCreateImage(cvGetSize(hazeImage), IPL_DEPTH_8U, 1);  
  
    for (int i = 0; i < r->height; i++)  
    {  
        for (int j = 0; j < r->width; j++)  
        {  
            double tempt = cvGetReal2D(guidedt, i, j);  
            if (tempt/255 < 0.1)  
            {  
                tempt = 25.5;  
            }  
  
            double I_r=cvGetReal2D(r, i, j);  
            double de_r = 255 * (I_r - Ac) / tempt + Ac;  
            cvSetReal2D(dehaze_r, i, j, de_r);  
  
            double I_g = cvGetReal2D(g, i, j);  
            double de_g = 255 * (I_g - Ac) / tempt + Ac;  
            cvSetReal2D(dehaze_g, i, j, de_g);  
  
            double I_b = cvGetReal2D(b, i, j);  
            double de_b = 255 * (I_b - Ac) / tempt + Ac;  
            cvSetReal2D(dehaze_b, i, j, de_b);  
  
        }  
    }  
  
    cvMerge(dehaze_b, dehaze_g, dehaze_r, 0, dehazedImage);  
  
    return dehazedImage;  
  
}  
```cpp

代码调试后, 得到的图片有点色差, 边缘有 2 条黑色的线, 我也懒得修改, 欢迎各位批评指正。  

在论文原文中, 有这样一段话：

Since the scene radiance is usually not as bright as the atmospheric light, the image after haze removal looks dim. So we increase the exposure of J(x) for
display.

意思就是说直接去雾后的图像会比原始的暗, 因此在处理完后需要进行一定的曝光增强, 但作者没有说明其是如何增强的, 因此这里的图和他论文的效果有所不同是正常的。一般在去雾处理后再用自动色剂之类的算法增强下会获得比较满意的结果。

去雾算法目前也有着众多其他的方式, 不过我所接触的, 很多都是以这个为基础, 因此, 先弄会这个为研究其他的去雾算法能奠定坚实的基础。
