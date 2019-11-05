# OpenCV 图像去噪算法 - 归一化,高斯,中值,双边滤波
## 1. 图像平滑

图像平滑与图像模糊是同一概念，主要用于图像的去噪。平滑要使用滤波器，为不改变图像的相位信息，一般使用线性滤波器。不同的核函数代表不同的滤波器，有不同的用途。   

常见的滤波器包括：    
1. 归一化滤波器，也是均值滤波器，用输出像素点核窗口内的像素均值代替输出点像素值。   
2. 高斯滤波器，实际中最常用的滤波器，高斯滤波是将输入数组的每一个像素点与高斯核卷积,将卷积得到的值当作输出像素值。    
3. 中值滤波器，中值滤波将图像的每个像素用邻域(以当前像素为中心的正方形区域)像素的中值代替。对椒盐噪声最有效，去除跳变点非常有效。    
4. 双边滤波器，为避免滤波器平滑图像去噪的同时使边缘也模糊，这种情况下使用双边滤波器。    

四种滤波器分别对应 4 个 OpenCV 函数，这些函数的前 2 个参数都是原图像和滤波后图像。    

归一化滤波函数原型:   
```cpp
C++: 
void blur(InputArray src, OutputArray dst, Size ksize, Point anchor=Point(-1,-1), int borderType=BORDER_DEFAULT )

Python: 
cv2.blur(src, ksize[, dst[, anchor[, borderType]]]) → dst
```

高斯滤波函数原型:    
```cpp
C++: 
void GaussianBlur(InputArray src, OutputArray dst, Size ksize, double sigmaX, double sigmaY=0, int borderType=BORDER_DEFAULT )

Python: 
cv2.GaussianBlur(src, ksize, sigmaX[, dst[, sigmaY[, borderType]]]) → dst
```

中值滤波函数原型:   
```cpp
C++: 
void medianBlur(InputArray src, OutputArray dst, int ksize)

Python: 
cv2.medianBlur(src, ksize[, dst]) → dst
```
双边滤波器的函数原型如下：     
```cpp
C++: 
void bilateralFilter( InputArray src, OutputArray dst, int d,
                      double sigmaColor, double sigmaSpace,
                      int borderType=BORDER_DEFAULT );

python:
cv2.bilateralFilter(src, d, sigmaColor, sigmaSpace[, dst[, borderType]]) → dst
```
双边滤波是一种非线性的滤波方法，是结合图像的空间邻近度和像素值相似度的一种折中处理，同时考虑空域信息和灰度相似性，达到保留图像的同时削弱噪声的效果。  

d: 表示在过滤过程中每个像素领域的直径。  
sigmaColor: 颜色空间滤波器的 sigma 值。值越大，表明该像素领域内有越宽广的颜色会被混合到一起。  
sigmaSpace: 坐标空间中滤波器的 sigma 值，坐标空间的标准差. 值越大，意味着越远的像素会相互影响，从而使更大的区域中足够相似的颜色获得相同的颜色。   

归一化滤波器 `blur()` 的第 3 个参数为滤波核窗口的大小，Size(i,i)表示 `ixi` 大小的窗口。   
高斯滤波器 `GaussianBlur()` 第 3 个参数也是滤波核窗口的大小，第 4、第 5 个参数分辨表示 x 方向和 y 方向的 δ。   
中值滤波器 `medianBlur()` 第 3 个参数是滤波器的长度，该滤波器的窗口为正方形。    

## 2. 图像代数运算 - 平均值去噪，减去背景     

代数运算，就是对两幅图像的点之间进行加、减、乘、除的运算。    

代数运算中比较常用的是图像相加和相减。图像相加常用来求平均值去除 addtive 噪声或者实现二次曝光。   

图像相减用于减去背景或周期噪声，污染等。   

### 图像相加    
OpenCV 中提供了相加的函数, 用于将一张图加到一个累加器上:    
```cpp
C:
void cvAcc(   
           const CvArr* image,//输入图像  
           CvArr* sum,  //累积图像   
           const CvArr* mask=NULL//可选的运算  
 );  

C++: 
void accumulate(InputArray src,              // 输入图像
                InputOutputArray dst,        // 累积图像
                InputArray mask=noArray() )  // 指定感兴趣区域

```

另外, 还需要用到一个线性变换转换函数来对相加的结果求平均:    
```cpp
C: 
void cvConvertScale(   
        const CvArr* src, //输入数组  
        CvArr* dst,//输出数组  
        double scale=1,//比例  
        double shift=0 //缩放比例，可选  
); 
``` 
曝光和去噪是一样的，也是对几幅图像求平均。    

### 图像相减
OpenCV 中用 cvAbsDiff 函数计算两数组的差的绝对值.   
Calculates the per-element absolute difference between two arrays or between an array and a scalar.

```cpp
C:
void cvAbsDiff(   
        const CvArr* src1,   // 第一个输入数组  
        const CvArr* src2,   // 第二个输入数组  
        CvArr* dst           //输出数组  
);  

C++:  
void absdiff(InputArray src1, 
             InputArray src2, 
             OutputArray dst)

Python: 
cv2.absdiff(src1, src2[, dst]) → dst
```
减去背景是通过两幅图像代数相减，可以判断出前景区域和运动区域，这是最简单（很多时候也是效果很好的）运动检测方法。

## 3. 线性滤波理论 - 方框滤波、均值滤波与高斯滤波     

### 平滑处理   

“平滑处理“（smoothing）也称“模糊处理”（bluring）。平滑处理的用途，最常见的是用来减少图像上的噪点或者失真。在涉及到降低图像分辨率时，平滑处理是非常好用的方法。   

### 滤波与滤波器   

滤波是将信号中特定波段频率滤除的操作，是抑制和防止干扰的一项重要措施。而滤波器就是建立的一个数学模型，通过这个模型来将图像数据进行能量转化，能量低的就排除掉，噪声就是属于低能量部分。    
一种形象的比喻法是：我们可以把滤波器想象成一个包含加权系数的窗口，当使用这个滤波器平滑处理图像时，就把这个窗口放到图像之上，透过这个窗口来看我们得到的图像。     

在新版本的 OpenCV 中，提供了如下五种常用的图像平滑处理操作方法，且他们分别被封装在单独的函数中:    
```
方框滤波——boxblur() 
均值滤波——blur() 
高斯滤波——GaussianBlur()
中值滤波——medianBlur()
双边滤波——bilateralFilter()
```

### 线性滤波器    

线性滤波器：线性滤波器经常用于剔除输入信号中不想要的频率或者从许多频率中选择一个想要的频率。   

几种常见的线性滤波器：    
允许低频率通过的 `低通滤波器` 。  
允许高频率通过的 `高通滤波器` 。  
允许一定范围频率通过的 `带通滤波器` 。  
阻止一定范围频率通过并且允许其它频率通过的 `带阻滤波器` 。  
允许所有频率通过、仅仅改变相位关系的 `全通滤波器` 。  
阻止一个狭窄频率范围通过的特殊 `带阻滤波器`, `陷波滤波器`（Band-stop filter）。   

### 关于滤波和模糊    

滤波是将信号中特定波段频率滤除的操作。

高斯滤波是指用高斯函数作为滤波函数的滤波操作，至于是不是模糊，要看是高斯低通还是高斯高通，低通就是模糊，高通就是锐化。   

### 线性滤波    

线性滤波是一种常用的邻域算子，像素的输出值取决于输入像素的加权和。

### 方框滤波（box Filter）    
```cpp
C++: 
void boxFilter(InputArray src,OutputArray dst, int ddepth, Size ksize, Point 
anchor=Point(-1,-1), boolnormalize=true, int borderType=BORDER_DEFAULT )
```
其中:    
src: 输入图像，即源图像，填Mat类的对象即可。该函数对通道是独立处理的，且可以处理任意通道数的图片，但需要注意，待处理的图片深度应该为 CV_8U, CV_16U, CV_16S, 
CV_32F 以及 CV_64F 中的一种。   
dst: 即目标图像，需要和源图片有一样的尺寸和类型。比如以源图 Mat::Clone() 后初始化目标图.   
ddepth: 输出图像的深度，-1 代表使用原图深度，即 src.depth()。   
ksize: 核的大小。一般这样写 Size( w,h ) 来表示, w 为像素宽度， h为像素高度。Size（3,3）就表示 3x3 的核大小，Size（5,5）就表示 5x5 的核大小.   
anchor: 表示锚点（即被平滑的那个点）, 默认值为 Point(-1,-1)。 
normalize: 是否使用区域归一化的核, 默认值为true。   
borderType: 用于图像边界 padding 的模式。默认值 BORDER_DEFAULT，一般不去管它。   

均值滤波是方框滤波归一化（normalized）后的特殊情况。其中，归一化就是把要处理的量都缩放到一个范围内,比如(0,1)，以便统一处理和直观量化。   

而非归一化（Unnormalized）的方框滤波用于计算每个像素邻域内的积分特性，比如密集光流算法
（dense optical flow algorithms）中用到的图像倒数的协方差矩阵（covariance matrices of image derivatives）.    

如果我们要在可变的窗口中计算像素总和，可以使用 integral() 函数。    

### 均值滤波    

均值滤波，是最简单的一种滤波操作，输出图像的每一个像素是核窗口内输入图像对应像素的像素的平均
值( 所有像素加权系数相等)，其实说白了它就是归一化后的方框滤波。    

均值滤波是典型的线性滤波算法，主要方法为邻域平均法，即用一片图像区域的各个像素的均值来代替原
图像中的各个像素值。一般需要在图像上对目标像素给出一个模板（核），该模板包括了其周围的临近
像素（比如以目标像素为中心的周围8（3x3-1）个像素，构成一个滤波模板，即去掉目标像素本身）。再
用模板中的全体像素的平均值来代替原来像素值。即对待处理的当前像素点（x，y），选择一个模板，该
模板由其近邻的若干像素组成，求模板中所有像素的均值，再把该均值赋予当前像素点（x，y），作为处
理后图像在该点上的灰度个g（x，y），即个g（x，y）=1/m ∑f（x，y） ，其中 m 为该模板中包含当前像素在内的像素总个数。

均值滤波的缺陷:   

它不能很好地保护图像细节，在图像去噪的同时也破坏了图像的细节部分，从而使图像变得模糊，不能很好地去除噪声点。

`blur()` 函数的原型：
```cpp
C++:  
void blur( InputArray src, OutputArraydst, 
           Size ksize, 
           Point anchor=Point(-1,-1), 
           int borderType=BORDER_DEFAULT )
```
其中:   
src: 输入图像，即源图像，填Mat类的对象即可。该函数对通道是独立处理的，且可以处理任意通道数的图片，但需要注意，待处理的图片深度应该为 CV_8U, CV_16U, CV_16S, 
CV_32F 以及 CV_64F 中的一种。   
dst: 即目标图像，需要和源图片有一样的尺寸和类型。比如以源图 Mat::Clone() 后初始化目标图.   
ksize: 核的大小。一般这样写 Size( w,h ) 来表示, w 为像素宽度， h为像素高度。Size（3,3）就表示 3x3 的核大小，Size（5,5）就表示 5x5 的核大小.   
anchor: 表示锚点（即被平滑的那个点）, 默认值为 Point(-1,-1)。 
borderType: 用于图像边界 padding 的模式。默认值 BORDER_DEFAULT，一般不去管它。   

### 高斯滤波    

高斯滤波是一种线性平滑滤波，适用于消除高斯噪声，广泛应用于图像处理的减噪过程。高斯滤波就是对
整幅图像进行加权平均的过程，每一个像素点的值，都由其本身和邻域内的其他像素值经过加
权平均后得到。高斯滤波的具体操作是：用一个模板（或称卷积、掩模）扫描图像中的每一个像素，用模
板确定的邻域内像素的加权平均灰度值去替代模板中心像素点的值。    

高斯模糊技术生成的图像，其视觉效果就像是经过一个半透明屏幕在观察图像，这与镜头焦外成像效果散
景以及普通照明阴影中的效果都明显不同。高斯平滑也用于计算机视觉算法中的预先处理阶段，以增强图
像在不同比例大小下的图像效果（参见尺度空间表示以及尺度空间实现）。从数学的角度来看，图像的高
斯模糊过程就是图像与正态分布做卷积。由于正态分布又叫作高斯分布，所以这项技术就叫作高斯模糊。
图像与圆形方框模糊做卷积将会生成更加精确的焦外成像效果。由于高斯函数的傅立叶变换是另外一个高
斯函数，所以高斯模糊对于图像来说就是一个低通滤波操作。    

高斯滤波器是一类根据高斯函数的形状来选择权值的线性平滑滤波器。高斯平滑滤波器对于抑制服从正
态分布的噪声非常有效。

对于图像处理来说，常用二维零均值离散高斯函数作平滑滤波器。

```cpp
C++:
void GaussianBlur( InputArray src,OutputArray dst, 
                   Size ksize, 
                   double sigmaX, double sigmaY=0, 
                   intborderType=BORDER_DEFAULT )
```
其中:   
src: 输入图像，即源图像，填Mat类的对象即可。该函数对通道是独立处理的，且可以处理任意通道数的图片，但需要注意，待处理的图片深度应该为 CV_8U, CV_16U, CV_16S, 
CV_32F 以及 CV_64F 中的一种。   
dst: 即目标图像，需要和源图片有一样的尺寸和类型。比如以源图 Mat::Clone() 后初始化目标图.   
ksize: 高斯核的大小。其中 ksize.width 和 ksize.height 可以不同，但他们都必须为正数和奇数。或者它们可以是零，然后由 sigmaX 和 sigmaY 计算而来。   
sigmaX: 表示高斯核函数在 X 方向的的标准偏差。
sigmaY: 表示高斯核函数在 Y 方向的的标准偏差。若 sigmaY 为零，就将它设为 sigmaX，如果sigmaX 和 sigmaY 都是 0，那么就由 ksize.width 和 ksize.height 计算得到。
为了结果的正确性着想，最好是把第三个参数 ksize，第四个参数 sigmaX 和第五个参数 sigmaY 全部指定到。    
borderType: 用于图像边界 padding 的模式。默认值 BORDER_DEFAULT，一般不去管它。   

## 4. 相关 OpenCV 源码实现流程分析    

分析 OpenCV 中线性滤波函数—— boxFilter，blur 和 GaussianBlur 函数以及涉及到的源码的实现流程。      

### boxFilter() 函数

实现的源码文件(OpenCV 源码版本：2.4.10):    
`/path/to/opencv/sources/modules/imgproc/src/smooth.cpp` 

算法实现流程:  

- 拷贝源图的形参 Mat 数据到临时变量，用于稍后的操作    
- 定义 int 型临时变量，代表源图深度的 sdepth，源图通道的引用 cn
- 处理 ddepth 小于零的情况
- 初始化目标图
- 拷贝目标图的形参 Mat 数据到临时变量，用于稍后的操作
- 处理 borderType 不为 BORDER_CONSTANT 且 normalize 为真的情况
- 若之前有过 HAVE_TEGRA_OPTIMIZATION 优化选项的定义，则执行宏体中的tegra 优化版函数并返回
- 调用 FilterEngine 滤波引擎创建一个BoxFilter，正式开始滤波操作

这里的 FilterEngine 是 OpenCV 图像滤波功能的核心引擎。   

### FilterEngine 类 - OpenCV图像滤波核心引擎

各种滤波函数比如blur， GaussianBlur，到头来其实是就是在函数末尾处定义了一个Ptr<FilterEngine>类型的f，然后f->apply( src, dst )了一下而已。
这个类可以把几乎是所有的滤波操作施加到图像上。它包含了所有必要的中间缓存器。有很多和滤波相关
的create系函数的返回值直接就是Ptr<FilterEngine>。

其中的Ptr是用来动态分配的对象的智能指针模板类，而上面的尖括号里面的模板参数就是FilterEngine。

使用FilterEngine类可以分块处理大量的图像，构建复杂的管线，其中就包含一些进行滤波阶段。如果我
们需要使用预先定义好的的滤波操作，cv::filter2D(), cv::erode(),以及cv::dilate()，可以选择，他
们不依赖于FilterEngine，在自己函数体内部就实现了FilterEngine提供的功能。不像其他的诸如blur系列函数，依赖于FilterEngine引擎。

FilterEngine类源代码流程
代码作用：FilterEngine类，OpenCV图像滤波功能的核心引擎
OpenCV源代码版本：2.4.8
源码路径：…\opencv\sources\modules\imgproc\include\opencv2\imgproc\imgproc.hpp
 
默认构造函数
完整的构造函数
默认析构函数
重新初始化引擎。释放之前滤波器申请的内存。
开始对指定了ROI区域和尺寸的图片进行滤波操作
开始对指定了ROI区域的图片进行滤波操作
处理图像的下一个srcCount行（函数的第三个参数）
对图像指定的ROI区域进行滤波操作，若srcRoi=(0,0,-1,-1)，则对整个图像进行滤波操作
如果滤波器可分离，则返回true
返回输入和输出行数
一些成员参数定义

### OpenCV 中 Size 类型剖析

`/path/to/opencv/modules/core/include/opencv2/core/core.hpp`   

```cpp
typedef Size_<int> Size2i;
typedef Size2i Size;
```
Size_ 是个模板类，在这里 Size_<int> 表示其类体内部的模板所代表的类型为int。   
首先给已知的数据类型 Size_<int> 起个新名字，叫 Size2i。然后又给已知的数据类型 Size2i 起个新名字，叫 Size。连起来就是，Size_<int>、Size2i、Size 这三个类型名等价。   

可以看到 Size_ 模板类的内部又重载了一些构造函数以满足我们的需要，其中，我们用得最多的是如下这个构造函数：    
```cpp
Size_(_Tp _width, _Tp _height);
```
另外，代码末尾定义了模板类型的宽度和高度：    
```
_Tp width, height; //宽度和高度
```
于是我们可以用 XXX. width 和 XXX.height 来分别表示其宽度和高度。

例如：`Size(5, 5)` 表示构造出的 Size 宽度和高度都为 5，即 XXX.width 和 XXX.height 都为 5.    

### blur() 函数源码剖析

实现的源码文件(OpenCV 源码版本：2.4.10):    
`/path/to/opencv/sources/modules/imgproc/src/smooth.cpp` 

算法实现流程:  

- 调用 boxFilter 函数进行处理    

可以看到在 blur() 内部就是调用了一个 boxFilter() ，且第六个参数为true，即上文所说的 normalize=true，即均值滤波是均一化后的方框滤波.    

### GaussianBlur() 函数流程

实现的源码文件(OpenCV 源码版本：2.4.10):    
`/path/to/opencv/sources/modules/imgproc/src/smooth.cpp` 

算法实现流程:  

- 拷贝形参 Mat 数据到临时变量，用于稍后的操作   
- 处理边界选项不为 BORDER_CONSTANT 时的情况   
- 若 ksize 长宽都为 1，将源图拷贝给目标图   
- 若之前有过 HAVE_TEGRA_OPTIMIZATION 优化选项的定义，则执行宏体中的tegra 优化版函数并返回   
- 如果 HAVE_IPP && (IPP_VERSION_MAJOR >= 7 为真，则执行宏体中语句   
- 调动滤波引擎，正式进行高斯滤波操作   

## 5. 线性滤波函数调用示例

调用代码示范：   

```cpp
// 载入原图
Mat image = imread("2.jpg");
Mat out;

// 进行滤波操作
boxFilter(image, out, -1, Size(5, 5));
blur(image, out, Size(7, 7), Point(-1,-1));
GaussianBlur(image, out, Size(5, 5), 0, 0);
```
 
## 总结 && 问题   

non-local
bm-3d

在实际处理过程中, 双边滤波在保留图像清晰度的前提下可以达到实时的处理效果.   

磨皮可以看成是去噪算法的应用。典型的去噪算法，比如均值模糊、高斯模糊、中值滤波都有很好的去噪效果，但是由于边缘也被模糊,视觉效果太差。能有效的用于磨皮算法的去噪方式主要是那些能够保留边缘的算法，典型的比如 `双边滤波`、`Non-Local` 以及 `BM3D` 之类的。` BM3D` 据说去噪效果最好。但是后两者到目前为止未看到具有实质意义的实时实现算法. 而双边滤波，有多篇论文已经提出了可行的加速方案。其实 Photoshop 中的表面模糊也可以看成是一种双边滤波，因此不少用 PS 磨皮的过程也大量使用了表面模糊算法的。   



