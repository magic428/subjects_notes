# 常用`matlab`工具箱介绍

1.Piotr's Computer Vision Matlab Toolbox
由UCSD的Piotr Dollar编写，侧重物体识别（Object Recognition）检测相关的特征提取和分类算法。这个工具箱属于专而精的类型，主要就是Dollar的几篇物体检测的论文的相关算法，如果做物体识别相关的研究，应该是很好用的。同时它的图像操作或矩阵操作函数也可以作为Matlab图像处理工具箱的补充，功能主要包括几个模块： 
* channels模块，图像特征提取，包括HOG等，Dollar的研究工作提出了一种Channel Feature的特征[2]，因此这个channels主要包括了提取这一特征需要的一些基本算法梯度、卷及等基本算法 
* classify模块，一些快速的分类相关算法，包括random ferns, RBF functions, PCA等 
* detector模块，与Channel Feature特征对应的检测算法1 
* filters模块，一些常规的图像滤波器 
* images模块，一些常规的图像、视频操作，有一些很实用的函数 
* matlab模块，一些常规的Matlab函数，包括矩阵计算、显示、变量操作等，很实用 
* videos模块，一些常规的视频操作函数等

2.VLFeat和Piotr’s Image & Video Matlab Toolbox


VLFeat和Piotr’s Image & Video Matlab Toolbox

首先要推荐`[Matlab计算机视觉/图像处理工具箱推荐](http://www.52ml.net/13272.html)`一文，对很多Matlab环境下的计算机视觉/图像处理工具箱进行了推荐介绍。本文是对其中提到的VLFeat和Piotr’s Image & Video Matlab Toolbox两个进行了介绍。

VLFeat

VLFeat介绍

VLFeat：著名而常用 
项目网站：http://www.vlfeat.org/ 
许可证：BSD 
著名的计算机视觉/图像处理开源项目，知名度应该不必OpenCV低太多，曾获ACM Open Source Software Competition 2010一等奖。使用C语言编写，提供C语言和Matlab两种接口。实现了大量计算机视觉算法，包括： 
* 常用图像处理功能，包括颜色空间变换、几何变换（作为Matlab的补充），常用机器学习算法，包括GMM、SVM、KMeans等，常用的图像处理的plot工具。 
* 特征提取，包括 Covariant detectors, HOG, SIFT,MSER等。VLFeat提供了一个vl_covdet() 函数作为框架，可以方便的统一所谓“co-variant feature detectors”，包括了DoG, Harris-Affine, Harris-Laplace并且可以提取SIFT或raw patches描述子。 
* 超像素（Superpixel）分割，包括常用的Quick shift, SLIC算法等 
* 高级聚类算法，比如整数KMeans：Integer k-means (IKM)、hierarchical version of integer k-means (HIKM)，基于互信息自动判定聚类类数的算法Agglomerative Information Bottleneck (AIB) algorithm等 
* 高维特曾匹配算法，随机KD树Randomized kd-trees
在Matlb中安装VLFeat

VLFeat使用C语言编写，提供C语言和Matlab两种接口，我在Matlb环境下使用VLFeat。

首先在VLFeat - Download可以下载VLFeat编译后的二进制包，也可以下载源码。windows平台的话，推荐使用二进制包。之后将所下载的二进制包解压缩到某个位置，如D:\VLFeat。

VLFeat - Download > Using from MATLAB这里有Matlab环境下VLFeat 的安装方法。VLFeat 有两种安装方式：临时安装(One-time setup)和永久安装(Permanent setup)。

临时安装VLFeat

在Matlab命令中输入

run(‘VLFEATROOT/toolbox/vl_setup’)
其中VLFEATROOT为VLFeat解压缩路径，如解压缩到D:\VLFeat时，安装命令则为：

run(‘D:\VLFeat\toolbox\vl_setup’)
此处要核对一下toolbox是否在VLFeat目录下，不在的话就在子目录下找一下。 
临时安装的VLFeat就是将路径加入到Path中，但只是临时添加，Matlab重启后会消失，所以是临时安装。

永久安装VLFeat

首先要在Matlab的Path已经包含了的目录下建立startup.m文件，有几种方案 
1. 打开matlab，将目录切换到任一Matlab已经包含了的目录下（如初始的path路径下），输入edit startup.m创建启动文件startup.m； 
2. 在任意位置创建startup.m，并将其剪切到任一Matlab已经包含了的目录下（如初始的path路径下）； 
3. 在任意位置创建startup.m，在ENVIRONMENT（主页）选项卡中，选择setPath（设置路径）选项，将startup.m文件所在的文件夹包含到PATH中。

后在startup.m中编辑发下内容：

run(‘VLFEATROOT/toolbox/vl_setup’)
其中VLFEATROOT为VLFeat解压缩路径，如解压缩到D:\VLFeat时，安装命令则为：

run(‘D:\VLFeat\toolbox\vl_setup’)
保存并关闭startup.m文件。

以后每次重新打开matlab程序，会自动将VLFeat的toolbox的目录加到Matlab的Path里面。

VLFeat安装验证

在matlab中输入path，可以发现在Path中保存了VLFeat的toolbox的目录；
在matlab中输入
vl_version verbose
可以得到类似于以下信息：

VLFeat version 0.9.17 
Static config: X64, little_endian, GNU C 40201 LP64, POSIX_threads, SSE2, OpenMP 
4 CPU(s): GenuineIntel MMX SSE SSE2 SSE3 SSE41 SSE42 
OpenMP: max threads: 4 (library: 4) 
Debug: yes 
SIMD enabled: yes
至此VLFeat已经安装成功。

Piotr’s Image & Video Matlab Toolbox

Piotr’s Image & Video Matlab Toolbox介绍


在Matlb中安装Piotr’s Image & Video Matlab Toolbox

Piotr’s Image & Video Matlab Toolbox的官方网站为Piotr’s Image & Video Matlab Toolbox 
在页面上可以找到最Toolbox的下载地址，最新版的存放在Github上，地址为pdollar/toolbox · GitHub

下载到Toolbox后，将其解压到任意目录下，如D:\toolbox-master

