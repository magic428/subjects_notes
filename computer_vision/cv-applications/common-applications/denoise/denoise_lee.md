# 基于局部均值方差统计信息的图像去噪 

[TOC]

## 论文核心思想   

在 1979 年 Lee 发表的论文[《Digital Image Enhancement and Noise Filtering by Use of Local Statistics》](http://www.sci.utah.edu/~gerig/CS7960-S2010/materials/Perona-Malik/Lee%20filter%20Digital%20image%20enhancement%20and%20noise%20filtering%20by%20using%20local%20statistics.pdf)中，提出了基于局部信息去除加性噪音、乘性噪音及加性乘性混合噪音的方法.   

经过仔细的学习和编码，发现其去除加性噪音的方法效果非常好，具有现在一些 EPF 算法类似的边缘保留功能，并且其运算并不复杂，可以应用到类似于磨皮这种项目中。   

简单的算法描述如下， 对于一幅 `N*M` 大小灰度图像，用 $x_{i,j}$ 表示 $(i,j)$ 位置处的像素值，那么在 `（2*n+1)*(2*m+1)` 窗口内部的局部平均值可表示为：  

$$
m_{ij} = \frac{1}{(2n+1)(2m+1)}\sum_{k=i-n}^{n+i}\sum_{l=j-m}^{m+j}x_{kl}  
$$
    
局部均方差可表示为:   

$$
v_{ij} = \frac{1}{(2n+1)(2m+1)}\sum_{k=i-n}^{n+i}\sum_{l=j-m}^{m+j}(x_{kl}-m_{ij})^2
$$ 
    

加性去噪后的结果为：    

$$
\bar{x_{ij}} = (1-k)m_{ij} + kx_{ij} 
$$

其中 $x_{ij}$ 为原图中的像素点, $k$ 为：    

$$
k = \frac{v_{ij}}{v_{ij} + \sigma}
$$

式中 $\sigma$ 为用户输入的参数。    

过程和简单，平滑图像的同时保持边缘基本不受影响，比如下图的结果：    

![](../snapshots/denoise_lee_result.jpg)   

这个优良的性质让其能在图像磨皮方面发挥一定的作用。

## 算法效率   

再来看看这个算法的效率如何。由上面的计算公式可以看到，其主要的计算量是局部均值以及均布均方差，均值的计算优化方式很多，比如典型的 `积分图`。而关于均布均方差的优化，推荐大家看这里：http://fiji.sc/Integral_Image_Filters，其实最后就是两个积分图, 其核心的推导公式为：    

$$
{\text{Var}}(X)={\frac  {1}{n}}\sum _{{i=1}}^{n}(x_{i}-\mu )^{2}\quad {\text{and}}\quad \mu ={\frac  {1}{n}}\sum _{{i=1}}^{n}x_{i} 
$$

展开为:   

$${\text{Var}}(x)={\frac  {1}{n}}\sum _{{i=1}}^{n}\left(x_{i}^{2}-2x_{i}\mu +\mu ^{2}\right)$$

$$={\frac  {1}{n}}\sum _{{i=1}}^{n}x_{i}^{2}-{\frac  {1}{n}}\sum _{{i=1}}^{n}2x_{i}\mu +\mu ^{2}$$

$$={\frac  {1}{n}}\sum _{{i=1}}^{n}x_{i}^{2}-{\frac  {1}{n}}\sum _{{i=1}}^{n}2x_{i}\mu +{\frac  {1}{n^{2}}}\left(\sum _{{i=1}}^{n}x_{i}\right)^{2}$$

$$={\frac  {1}{n}}\sum _{{i=1}}^{n}x_{i}^{2}-{\frac  {2\mu }{n}}\sum _{{i=1}}^{n}x_{i}+{\frac  {1}{n^{2}}}\left(\sum _{{i=1}}^{n}x_{i}\right)^{2}$$

$$={\frac  {1}{n}}\sum _{{i=1}}^{n}x_{i}^{2}-{\frac  {2}{n^{2}}}\left(\sum _{{i=1}}^{n}x_{i}\right)^{2}+{\frac  {1}{n^{2}}}\left(\sum _{{i=1}}^{n}x_{i}\right)^{2}$$

$$={\frac  {1}{n}}\sum _{{i=1}}^{n}x_{i}^{2}-{\frac  {1}{n^{2}}}\left(\sum _{{i=1}}^{n}x_{i}\right)^{2}$$

$$={\frac  {1}{n}}\sum _{{i=1}}^{n}x_{i}^{2}-\left({\frac  {1}{n}}\sum _{{i=1}}^{n}x_{i}\right)^{2}$$

$$={\frac  {1}{n}}\left(\sum _{{i=1}}^{n}x_{i}^{2}-{\frac  {1}{n}}\left(\sum _{{i=1}}^{n}x_{i}\right)^{2}\right)$$

所有的求和都可以通过两个积分图 $I({\vec  {x}})$ 和 $I({\vec  {x}})^{2}$ 计算得到.

经过这样的推导，可以看到局部均方差也可以通过 `积分图` 快速实现，同时程序的效率和局部的半径参数无关，因此，效率非常高。     

上述公式是针对灰度图像进行的，对于常见的 RGB 彩色图，只要对 R/G/B 三通道分别进行处理就 OK 了。   

有了上述基础，经过个人的摸索，对于磨皮应用，这个算法的两个参数:
(1) 半径：   
`max(Src->Width, Src->Height) * 0.02`   
(2) 求解 $k$ 所需的 $\sigma$:   
`10 + DenoiseLevel * DenoiseLevel * 5`  
其中`DenoiseLevel` 为磨皮的程度参数，范围从 1 到 10，越大磨皮越厉害。  
 
考虑到这个算法对每个像素的亮度值的改变并不是很大，对于彩色图像，如果能够转换到其他的包含亮度分量的颜色空间后，只对亮度进行处理，然后在转换回来，应该对视觉的影响不大，这样去除颜色空间的转换时间，可以提高三倍的速度，是相当可观的.  

常见的包含亮度的颜色空间有 LAB,HSV,YUV,YCbCr 等，其中 YCbCr 和 RGB 的转换公式非常简单，没有浮点计算，对整体的效率影响不大，因此可以选用这个空间。优化的步骤可以一并在特定的颜色空间中进行, 大大提高算法效率.   

下面是处理函数的流程：
```cpp
/// 实现图像的磨皮, 原图、目标图必须都是24位(3 chs * 8)的。
/// "Src": 需要处理的源图像的数据结构。 
/// "Dest": 需要处理的源图像的数据结构。 
/// "DenoiseMethod": 磨皮的算法，0为双边磨皮，1为均方差磨皮。 
/// "DenoiseLevel": 磨皮的程度，有效范围[1,10]，数据越大，磨皮越明显。 
/// "WhiteMethod": 美白的算法，0为Log曲线美白，1为色彩平衡磨皮。 
/// "NonSkinLevel": 美白的程度，有效范围[1,10]，数据越大，美白越明显。 

void SkinBeautification(cv::Mat *Src, cv::Mat *Dest, int DenoiseLevel, int WhiteMethod, int WhiteLevel)
{
    cv::Mat *Skin = NULL, *Y = NULL, *Cb = NULL, *Cr = NULL, *YY = NULL;
    unsigned char *Table = (unsigned char *)malloc(256*sizeof(unsigned char)); 
    
    // 分配皮肤区域的内存
    CreateMatrix(Src->Width, Src->Height, Src->Depth, 1, &Skin);   
    CreateMatrix(Src->Width, Src->Height, Src->Depth, 1, &Y);
    CreateMatrix(Src->Width, Src->Height, Src->Depth, 1, &Cb);
    CreateMatrix(Src->Width, Src->Height, Src->Depth, 1, &Cr);
    CreateMatrix(Src->Width, Src->Height, Src->Depth, 1, &YY);

    // 第一步： 将RGB转换到YCbCr空间
    RGBToYCbCr(Src, Y, Cb, Cr); 
 
    int SpaceError = 10 + DenoiseLevel * DenoiseLevel * 5; 

    // 第二步：对Y分量进行加性噪音的去除
    LeeAdditvieNoiseFilter(Y, YY, max(Src->Width, Src->Height) * 0.02, SpaceError);  

    // 将图像从YCbCr空间转换会RGB空间
    YCbCrToRGB(Src, Y, Cb, Cr);
   
    // 注意释放内存
    FreeMatrix(&Y);
    FreeMatrix(&Cb);
    FreeMatrix(&Cr);
    FreeMatrix(&YY);
    FreeMemory(Table);
}
```

这个算法最大的优点是整个过程的任何函数都没有浮点计算，这对于某些硬件来说是很重要的，但是一个缺点是优化后的算法不能并行，在我的 I3笔记本电脑上 30W 的像素处理时间 20 ms，完全能实现实时效果。    

## 算法实现    
```cpp
int main(int argc, char* argv[])
{
    if (argc < 4){
        std::cout << "Usage: integra_image input.png patch_size DenoiseLevel "<< std::endl;
        return -1;
    }

    Mat src = imread(argv[1]);
    Mat dst = src.clone();
    uchar* src_data = src.data;
    uchar* dst_data = dst.data;
    
    int DenoiseLevel = atoi(argv[2]);
    int sigmma = 100 + DenoiseLevel * DenoiseLevel * 5;
    SIZE = atoi(argv[3]);
    
    if ( 1 == src.channels() )
        lee_filter_1_chs(src, dst, sigmma);
    else
        lee_filter_3_chs(src, dst, sigmma);

    cv::imshow("src", src);
    cv::imshow("output", dst);

    if(27 == cv::waitKey(0))
        cv::destroyAllWindows();

    return 0;
}

void lee_filter_1_chs(const cv::Mat &src, cv::Mat &dst, int sigmma)
{
    int height = src.rows;
    int width = src.cols;
    int lineByte = src.step;
    uchar* src_data = src.data;
    uchar* dst_data = dst.data;
    double Sum, Sum2;
    int index;
    double Mean, Var, k;
    int size = SIZE*SIZE;
    cout << "Size = " << SIZE << ", sigmma = " << sigmma << endl;

    for ( int x = SIZE/2;x < height-SIZE/2; x++ ) {  
        for ( int y = SIZE/2;y < width-SIZE/2; y++ ) {  

            Sum = 0, Mean = 0, Sum2 = 0;
            index = x * lineByte + y;  
    
            for( int i = -SIZE/2; i<= SIZE/2; i++ ) {  
                for( int j = -SIZE/2; j<=SIZE/2; j++ ) {

                    uchar tmp = src_data[index + i*lineByte + j];  
                    Sum += tmp;  
                    Sum2 += tmp*tmp;  
                }  
            }  

            Mean = Sum/size;  
            Var = (Sum2-(Sum*Sum)/size)/size;  
            k = Var/(Var+sigmma);  

            dst_data[index] = CLIP255((int)((1.0-k)*Mean + k*src_data[index]));  
        }  
    }  
}

/**
 * lee_filter 过程
 *  注意和图像的 BGR 顺序还是 RGB 顺序无关.  
*/
void lee_filter_3_chs(const cv::Mat &src, cv::Mat &dst, int sigmma)
{
    int height = src.rows;
    int width = src.cols;
    int lineByte = src.step;
    uchar* src_data = src.data;
    uchar* dst_data = dst.data;
    float SumR2 = 0, SumG2 = 0, SumB2 = 0;
    float SumR = 0, SumG = 0, SumB = 0;
    float MeanR = 0, MeanG = 0, MeanB = 0;
    float VarR = 0,  VarG = 0, VarB = 0;
    float kr, kg, kb; 
    int index;
    int size = SIZE * SIZE;

    for( int i = SIZE/2; i < height-SIZE/2; i++ ) {
        for( int j = SIZE/2; j < width-SIZE/2; j++ ) {

            SumR2 = 0, SumG2 = 0, SumB2 = 0;
            SumR = 0, SumG = 0, SumB = 0; 
            MeanR = 0, MeanG = 0, MeanB = 0;
            VarR = 0, VarG = 0, VarB = 0;
            index = i * lineByte + 3 * j;   // 3 channels

            for( int m = -SIZE/2; m<= SIZE/2; m++ ) {
                for( int n = -SIZE/2; n<=SIZE/2; n++ ) {
                    SumB  += src_data[index+m*lineByte+n*3];
                    SumB2 += src_data[index+m*lineByte+n*3]*src_data[index+m*lineByte+n*3];

                    SumG  += src_data[index+1+m*lineByte+n*3];
                    SumG2 += src_data[index+1+m*lineByte+n*3]*src_data[index+1+m*lineByte+n*3];

                    SumR  += src_data[index+2+m*lineByte+n*3];
                    SumR2 += src_data[index+2+m*lineByte+n*3]*src_data[index+2+m*lineByte+n*3];
                }
            } 
            
            // 计算平均值
            MeanR = SumR/size;
            MeanG = SumG/size;
            MeanB = SumB/size;

            // 计算方差
            VarR = (SumR2-(SumR*SumR)/size)/size;
            VarG = (SumG2-(SumG*SumG)/size)/size;
            VarB = (SumB2-(SumB*SumB)/size)/size;

            // 计算系数
            kr = VarR /(VarR+sigmma);
            kg = VarG /(VarG+sigmma);
            kb = VarB /(VarB+sigmma);
            
            dst_data[index]   =  CLIP255((int)((1-kb)*MeanB+kb*src_data[index]));
            dst_data[index+1] =  CLIP255((int)((1-kg)*MeanG+kg*src_data[index+1]));
            dst_data[index+2] =  CLIP255((int)((1-kr)*MeanR+kr*src_data[index+2]));
        }
    }
}
```

每次都会计算像素块的累积值, 并没有使用积分图去做优化. 等有时间了再使用积分图去做优化.    

目前彩色图像(600*480)可以跑到 150 ms, 灰度图可以跑到 50 ms.   

积分图已经实现, 现在考虑用颜色空间进行处理.    

