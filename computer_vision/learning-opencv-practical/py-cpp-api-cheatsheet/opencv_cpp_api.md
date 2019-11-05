# OpenCV 的 CPP API  

## 1. 图片操作 

## 1.1 imwrite   

函数原型:  

```cpp
bool imwrite(const string& filename, 
             InputArray img, 
             const vector<int>& params=vector<int>() );

int cvSaveImage(const char* filename, 
                const CvArr* image, 
                const int* params=0 );
```

params – 指定图片的保存格式, 支持的格式有:  

(1) JPEG, CV_IMWRITE_JPEG_QUALITY 从 0 到 100, 默认值是 95.   
(2) PNG, CV_IMWRITE_PNG_COMPRESSION 表示压缩程度, 从 0 到 9. 默认值是 3.  

```cpp
Mat img(480, 640, CV_8UC4);

vector<int> compression_params;
compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
compression_params.push_back(9);

imwrite("alpha.png", img, compression_params);

// 或者 
imwrite("alpha.png", img, [CV_IMWRITE_PNG_COMPRESSION, 9]);
```

### 1.2 图像中显示文字   

```cpp
cv::putText(frame, capture, (10, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, (0, 255, 0))
```

### 1.3 监听用户按键动作   

```cpp
cv::waitKey(1)   
key == ord('c')
cv::destroyAllWindows()   
```

### 1.4 图像深拷贝   

```cpp
cv::Mat img = cv::imread(argv[1]);
cv::Mat convas = img.clone();
// 或者
img.copyTo(convas);

cv::Mat a;
cv::Mat b = a;             // 浅拷贝, 只拷贝矩阵头, 数据并没有复制
cv::Mat c(a);              // 浅拷贝, 只拷贝矩阵头, 数据并没有复制
cv::Mat b = a.clone();     // 深拷贝, 矩阵头和数据都拷贝了
cv::Mat c;
a.copyTo(c);               // 深拷贝, 矩阵头和数据都拷贝了3). cv::Mat 与 CvMat 和 IplImage 的互转
```

## 2. Video 读写相关  

### 2.1. Video 读取

```cpp
cv::VideoCapture = cap(filename);    

cap.isOpened();   

cap.get();   

cv::Mat frame = cap.read();  
// 或者 
cap >> frame;

cap.release();
```

### 2.2. Video 保存

```cpp
VideoWriter::VideoWriter(const string& filename, int fourcc, double fps, Size frameSize, bool isColor=true); 

// 通常 fourcc, fps, width, height 可以在原视频中使用 get() 函数获取  
cv::VideoWriter vw(filename, fourcc, fps, (width, height));
// fourcc 可以用以下宏定义生成, 占用空间最小的编码方式是 MPEG-4.2
CV_FOURCC('M', 'P', '4', '2');  

vw.write(frame);

vw.release();
```

get 函数中常用的参数:   

* CAP_PROP_FRAME_WIDTH;    
* CAP_PROP_FRAME_HEIGHT;    
* CAP_PROP_FRAME_COUNT;  // 获取视频总的帧数, 可用于遍历视频流  
* CAP_PROP_FPS;    
* CAP_PROP_FOURCC;    

所有支持的参数.   

```
CV_CAP_PROP_POS_MSEC Current position of the video file in milliseconds or video capture timestamp.
CV_CAP_PROP_POS_FRAMES 0-based index of the frame to be decoded/captured next.
CV_CAP_PROP_POS_AVI_RATIO Relative position of the video file: 0 - start of the film, 1 - end of the film.
CV_CAP_PROP_FRAME_WIDTH Width of the frames in the video stream.
CV_CAP_PROP_FRAME_HEIGHT Height of the frames in the video stream.
CV_CAP_PROP_FPS Frame rate.
CV_CAP_PROP_FOURCC 4-character code of codec.
CV_CAP_PROP_FRAME_COUNT Number of frames in the video file.
CV_CAP_PROP_FORMAT Format of the Mat objects returned by retrieve() .
CV_CAP_PROP_MODE Backend-specific value indicating the current capture mode.
CV_CAP_PROP_BRIGHTNESS Brightness of the image (only for cameras).
CV_CAP_PROP_CONTRAST Contrast of the image (only for cameras).
CV_CAP_PROP_SATURATION Saturation of the image (only for cameras).
CV_CAP_PROP_HUE Hue of the image (only for cameras).
CV_CAP_PROP_GAIN Gain of the image (only for cameras).
CV_CAP_PROP_EXPOSURE Exposure (only for cameras).
CV_CAP_PROP_CONVERT_RGB Boolean flags indicating whether images should be converted to RGB.
CV_CAP_PROP_WHITE_BALANCE_U The U value of the whitebalance setting (note: only supported by DC1394 v 2.x backend currently)
CV_CAP_PROP_WHITE_BALANCE_V The V value of the whitebalance setting (note: only supported by DC1394 v 2.x backend currently)
CV_CAP_PROP_RECTIFICATION Rectification flag for stereo cameras (note: only supported by DC1394 v 2.x backend currently)
CV_CAP_PROP_ISO_SPEED The ISO speed of the camera (note: only supported by DC1394 v 2.x backend currently)
CV_CAP_PROP_BUFFERSIZE Amount of frames stored in internal buffer memory (note: only supported by DC1394 v 2.x backend currently)
```


## 3. 矩阵和图像操作   

|         函数名称         |                             描述                   |
|:-----------------------|:---------------------------------------------------| 
| cv::abs (cvAbs)        | 计算矩阵或数组中各元素的绝对值 |
| cv::absdiff (cvAbsDiff) | 计算两个矩阵或数组中各元素之差的绝对值, 其中一个矩阵可以为标量 |
| cv::add (cvAdd) | 计算两个矩阵或数组中各元素之和, 其中一个矩阵可以为标量 |
| cv::addWeighted (cvAddWeighted) | 计算两个矩阵或数组中各元素的加权之和 |
| cv::calcCovarMatrix (cvCalcCovarMatrix) | 计算一组 n 维空间向量的协方差 |
| cartToPolar (cvCartToPolar) | 计算二维向量的幅值和极角 |
| compare (cvCmp) | 比较两个矩阵各元素的大小, CMP_EQ, CMP_GT, CMP_GE, CMP_LT, CMP_LE, CMP_NE|
| completeSymm (仅 CPP) | 将方阵上下对角线两侧的元素对调拷贝, 得到对称矩阵 |
| cv::convertScaleAbs (cvConvertScaleAbs) | 缩放, 求绝对值, 转换数据类型为 CV_8U |
| cv::countNonZero (cvCountNonZero) | 统计矩阵中的非零元素个数 |
| cv::cvarrToMat  (仅 CPP)| 将 CvMat, IplImage 或 CvMatND 数据类型转换为 Mat |
| cv::dct (cvDCT) | 对 1D 或 2D 矩阵进行离散余弦变换 |
| cv::dft (cvDFT) | 对 1D 或 2D 矩阵进行离散傅里叶变换 |
| cv::determinant (CvDet) | 计算浮点矩阵的行列式 |
| cv::divide (cvDiv) | 两个矩阵或数组的对应元素相除 |
| cv::eigen (cvEigenVV) | 计算对称矩阵的特征值和特征向量 |
| cv::exp (cvExp) | 计算矩阵各元素的指数值 |
| cv::flip (cvFlip) | 围绕固定的轴翻转矩阵元素, 水平(1), 垂直(0)或水平垂直一起翻转(-1) |
| cv::gemm (cvGEMM) | 矩阵乘法 |
| cv::invert (cvInvert) | 计算矩阵的逆矩阵或伪逆矩阵 |
| cv::log (cvLog) | 计算矩阵各元素的自然对数值 |
| cv::LUT (cvLUT) | 对矩阵应用 LUT, LUT 是针对像素值而言. dst(I) = lut(src(I) + d) |
| cv::magnitude (仅 CPP) | 计算 2D 向量的幅值,  dst(I) = sqrt(x(I)^2 + y(I)^2) |
| cv::Mahalanobis (cvMahalanobis) |计算两个向量的马氏距离 |
| cv::max (cvMax) | 计算两个矩阵或数组的对应元素的最大值 |
| cv::mean (cvMean) | 按通道计算某个矩阵或数组中所有元素的平均值. 输入矩阵有四个通道, 输出为 Scalar|
| cv::meanStdDev (cvMeanStdDev) | 计算某个矩阵或数组中所有元素的均值和方差 |
| cv::merge (cvMerge) | 将单通道矩阵或数组合并为多通道 |
| cv::min (cvMin) | 计算两个矩阵或数组的对应元素的最小值 |
| cv::minMaxLoc (cvMinMaxLoc) | 找到矩阵或数组中的最大值和最小值 |
| cv::multiply() | 两个矩阵或数组的对应元素相乘 |
| cv::mulTransposed (cvMulTransposed) | 计算矩阵和其转置矩阵的对应元素的乘积 |
| cv::norm (cvNorm) | 计算矩阵或向量的范数 |
| cv::normalize (仅 CPP) | 矩阵或向量的归一化操作 |
| cv::PCA | 主成分分析, PCA(), operator(), PCA::backProject(), PCA::project() |
| cv::pow (cvPow) | 计算矩阵各元素的幂 |
| cv::RNG | 随机数生成器, RNG(), RNG::next(), operator(), uniform(), gaussian(), fill() |
| cv::randu (仅 CPP) | 生成均匀分布的矩阵 |
| cv::randn (仅 CPP) | 生成正态分布的矩阵 |
| cv::randShuffle (仅 CPP) | 随机打乱矩阵中的元素 |
| cv::reduce (cvReduce) | 将矩阵缩减为向量(0, 缩减为一行, 1 缩减为一列), CV_REDUCE_(SUM, AVG, MAX, MIN) |
| cv::repeat (cvRepeat) | 使用给定的矩阵复制得到新的矩阵, ny 是沿 Y 轴复制的次数, nx 是沿 Y 轴复制的次数 |
| cv::scaleAdd (cvScaleAdd) | 矩阵和一个标量相加 |
| cv::setIdentity (cvSetIdentity) | 初始化一个本征矩阵 |
| cv::sort (仅 CPP) | 对矩阵的每一列或每一行进行排序 |
| cv::split (cvSplit) | 将多通道矩阵分割为单通道矩阵, 如将三通道彩色图像分为 B,G,R 三通道 |
| cv::sqrt (cvSqrt) | 计算矩阵中每个元素的平方根 |
| cv::subtract (cvSub) | 计算两个矩阵或数组中各元素之差, 其中一个矩阵可以为标量 |
| cv::SVD | 奇异值分解, SVD(), compute(), solveZ(), backSubst() |
| cv::sum (cvSum) | 按通道计算某个矩阵或数组中所有元素的累加和. 输入矩阵有四个通道, 输出为 Scalar |
| cv::theRNG (仅 CPP) | 返回默认的随机数生成器 |
| cv::trace (cvTrace) | 计算矩阵的迹 |
| cv::transpose (cvTranspose) | 矩阵转置 |


注意: 容易混淆的一些操作.  

* 矩阵乘法(cv::gemm())和矩阵各元素相乘(.* 运算, cv::Mat::mul());  
* 归一化(cv::normalize())和类型转换(cv::Mat::convertTo());  

凡是函数参数中有 dtype 参数的, 说明可以指定输出矩阵的元素数据类型, 如 CV_32F, CV8U 等. 如果没有指定, 则使用默认值 -1, 表示输出矩阵的元素数据类型和输入矩阵的元素数据类型相同.   

## 4. CvArr, CvMat, IplImage, cv::Mat, cv::InputArray 相互转换

弄清楚这几者的关系有助于 C 接口和 CPP 接口相互兼容.   

https://blog.csdn.net/bagboy_taobao_com/article/details/47048249  

### 4.1 CvArr   

```cpp
typedef void CvArr;   
```

可以认为它是万能指针, 例如某个函数的参数是 CvArr*, 在该函数内部会强制转换回该函数要求的数据类型的, 所以你调用该函数时, 传入的类型就必须与该函数要求的类型一致. 否则肯定会报错.   

### 4.2 cv::Mat   

我们可以理解为 cv::Mat 把向量, 矩阵, 图像等等都统一了操作. cv::Mat 有更强的矩阵运算能力, 支持常见的矩阵运算.   

对图像数据的运算, 将 CvMat 与 IplImage 类型转化为 cv::Mat 类型可大大提高运算效率.    

例如要计算时, 我们可以把 CvMat 或 IplImage 浅拷贝为 cv::Mat, 然后计算, 计算完再转回 CvMat 或 IplImage.  

cv::Mat 是一个多维的密集数据数组. 可以用来处理向量和矩阵, 图像, 直方图等等常见的多维数据.   

```cpp
class Mat
{
    ...
    // 看这里的构造函数
    //! converts old-style CvMat to the new matrix; the data is not copied by default
    Mat(const CvMat* m, bool copyData=false);
    //! converts old-style CvMatND to the new matrix; the data is not copied by default
    Mat(const CvMatND* m, bool copyData=false);
    //! converts old-style IplImage to the new matrix; the data is not copied by default
    Mat(const IplImage* img, bool copyData=false);


    // 看这里的括号操作符
    //! converts header to CvMat; no data is copied
    operator CvMat() const;
    //! converts header to IplImage; no data is copied
    operator IplImage() const;
    ... 
};
```

很多 OpenCV 的函数参数是 CvMat 或者是 IplImage, 使用 cv::Mat 来存储图像信息时, 这些函数都可以无缝的使用 cv::Mat.  

使用 IplImage 或者 CvMat 变量时, 当你需要使用 cv::Mat 的方法时, 则可以通过 cv::Mat 的构造函数把 CvMat 或 IplImage 转换成 cv::Mat (同时可以指定是否拷贝数据).  

### 4.3 cv::Mat 与 IplImage 互转

```cpp
// cv::Mat 转 IplImage
cv::Mat imgMat = imread("demo.jpg");

IplImage pImg1 = IplImage(imgMat);      // 浅拷贝, 只拷贝矩阵头, 数据并没有复制.
IplImage pImg2 = imgMat;                // 浅拷贝, 只拷贝矩阵头, 数据并没有复制.

// IplImage 转 cv::Mat
IplImage * img = cvLoadImage("C\\a.jpg");

cv::Mat imgMat1(img);                   // 浅拷贝, 只拷贝矩阵头, 数据并没有复制.
cv::Mat imgMat2(img, false);            // 浅拷贝, 只拷贝矩阵头, 数据并没有复制.
cv::Mat imgMat3 = img;                  // 浅拷贝, 只拷贝矩阵头, 数据并没有复制.
cv::Mat imgMat4(img, true);             // 深拷贝, 矩阵头和数据都拷贝了
```

### 4.4 cv::Mat 与 CvMat 互转  

```cpp
// cv::Mat 转 CvMat
cv::Mat imgMat = imread("Cdemo.jpg");
CvMat cvMat1 = CvMat(imgMat);           // 浅拷贝, 只拷贝矩阵头, 数据并没有复制.
CvMat cvMat2 = imgMat;                  // 浅拷贝, 只拷贝矩阵头, 数据并没有复制.

// CvMat 转 cv::Mat
CvMat* imgCvMat;
cv::Mat imgMat1(imgCvMat);                  // 浅拷贝, 只拷贝矩阵头, 数据并没有复制.
cv::Mat imgMat2(imgCvMat, false);           // 浅拷贝, 只拷贝矩阵头, 数据并没有复制.
cv::Mat imgMat3 = imgCvMat;                 // 浅拷贝, 只拷贝矩阵头, 数据并没有复制.
cv::Mat imgMat4 = cv::Mat(imgCvMat, true);  // 深拷贝, 矩阵头和数据都拷贝了
```

### 4.5 CvMat 与 IplImage 互转  

```cpp
// CvMat 转 IplImage
CvMat M;
IplImage* img = cvCreateImageHeader(M.size(), M.depth(), M.channels());
cvGetImage(&M, img);    // 深拷贝, 矩阵头和数据都拷贝了(返回也是 img)

IplImage* img = cvCreateImage(M.size(), M.depth(), M.channels());
cvConvert(&M, img);     // 深拷贝, 矩阵头和数据都拷贝了

// IplImage 转 CvMat
IplImage* img;
CvMat temp;
CvMat* mat = cvGetMat(img, &temp);    // 深拷贝, 矩阵头和数据都拷贝了
// 或者
CvMat *mat = cvCreateMat(img->height, img->width, CV_64FC3);  // 注意 height 和 width 的顺序
cvConvert(img, mat);                  // 深拷贝, 矩阵头和数据都拷贝了
```

### 4.6 cv::Mat 与 cv::InputArray

cv::InputArray 的定义是这样的: Proxy datatype for passing Mat's and vector<>'s as input parameters class _InputArray,  
我们看到很多函数的参数是 InputArray, 而调用的时候传入的是cv::Mat对象, 进入代码可以看到:  

```cpp
typedef const _InputArray& InputArray;

// Proxy datatype for passing Mat's and vector<>'s as 
// input parameters class CV_EXPORTS _InputArray
class _InputArray {
    ...
    _InputArray();
    _InputArray(const Mat& m);  // 看这个构造函数
    ...
};

_InputArray::_InputArray(const Mat& m) : flags(MAT), obj((void*)&m) {
    ...
}
```

根据上述构造函数可知, cv::Mat 可以隐式构造 _InputArray 对象.   


## 5. 使用 OpenCV API 时踩过的坑.   

### 5.1 使用 cv::normalize() 操作元素数据类型为 CV_8U 的矩阵

输出结果中只有 0 和 1, 即相当于二值化操作.   

**原因:** 在使用 cv::normalize() 函数时, 未指定 dtype 参数的值为 CV_32F. 

因此, 归一化之后的矩阵的元素数据类型仍然为 CV_8U, 它是无法表示 [0.0, 1.0] 之间的小数的, 因此得到的就只有 0 和 1.   

### 5.2 使用 cv::calcCovarMatrix() 得到的协方差矩阵和 MATLAB cov() 得到的协方差矩阵不同 

> https://blog.csdn.net/shaoxiaohu1/article/details/9304173   

> https://blog.csdn.net/fengbingchun/article/details/73558370  

按理说对于给定的矩阵 R (假设 n 维空间向量为列向量), 其协方差矩阵应该是相同的.    

**原因:**  MATLAB cov() 对最终的协方差矩阵进行了缩放 1.0/(R.cols - 1) 的操作. 尽管 OpenCV 的 cv::calcCovarMatrix() 提供了 CV_COVAR_SCALE 标志, 但是其缩放尺度是: 1.0/R.col, 因此不推荐直接使用, 可以在得到原始的协方差矩阵后乘以尺度因子 1.0/(R.cols - 1) 即可.  

OpenCV 具体操作如下:   

```cpp
    ...

    cv::Mat covMat, meanMat;
    cv::calcCovarMatrix( R, 
                         covMat, 
                         meanMat, 
                         CV_COVAR_NORMAL| CV_COVAR_COLS, 
                         CV_32FC1);  
    covMat = (1.0/(R.cols - 1))*covMat;  // 缩放操作  

    ...
```

### 5.3 使用 cv::SVD() 得到的奇异值分解结果和 MATLAB svd() 得到的结果不同   

二者的奇异值向量是相同的, 只是左矩阵和右矩阵元素的符号不一致(矩阵的最后一列为奇异值). 因此, 需要对 OpenCV 得到的奇异值矩阵做如下符号处理:  

```cpp
    // [U,S,V] = svd(mat); --- MATLAB 代码
    svd.compute(mat, S, U, VT);
    cv::transpose(VT, V);
    for(int i = 0; i < U.cols - 1; ++i) {  // 不处理最后一列
        U.col(i) = -U.col(i);
    }
```

### 5.4 使用 cv::Mat::convertTo() 函数时, 盲目使用缩放因子.  

其实, cv::Mat::convertTo() 函数将 CV_8U 类型转换为 CV_32F, 且采用 1./255 的缩放因子时, 可以等价的使用 cv::normalize() 函数.   
