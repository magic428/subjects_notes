% BM3D 图像去噪算法.  

# ABOUT

* Author    : Marc Lebrun <marc.lebrun@cmla.ens-cachan.fr>
* Copyright : (C) 2011 IPOL Image Processing On Line http://www.ipol.im/
* Licence   : GPL v3+, see GPLv3.txt

# OVERVIEW

This source code provides an implementation of the BM3D image denoising.

# UNIX/LINUX/MAC 用户指南  

代码对于 Unix/Linux 和 Mac OS 平台是直接兼容的.   

- Compilation. 
Automated compilation requires the cmake program.

- Libraries. 
This code requires the libpng libtiff libjpeg and libfftw librarers.

- Image formats. 
PNG, JPEG, TIFF (including floating point) formatis is supported. 
 
-------------------------------------------------------------------------:
使用方法:  

1. Download the code package and extract it. Go to that directory. 

2. Compile the source code (on Unix/Linux/Mac OS). 
mkdir build; cd build; cmake ..; make

3. Run BM3D image denoising.
./bm3d
The generic way to run the code is:

./bm3d input.png sigma ImDenoised.png [ImBasic.png ]
-tau_2d_hard 2DtransformStep1 -useSD_hard 
-tau_2d_wien 2DtransformStep2 -useSD_wien 
-color_space ColorSpace  

with :
- cinput.png 带噪声的图像;
- sigma is the value of the noise;
- ImBasic.png 算法第一阶段处理后的图像;
- ImDenoised.png 算法去噪后的输出图像;
- 2DtransformStep1: choice of the 2D transform which will be applied in the 
     second step of the algorithm. You can choose the DCT transform or the 
     Bior1.5 transform for the 2D transform in the step 1 (tau_2D_hard = dct or bior) 
     and/or the step 2. (tau_2d_wien = dct or bior).
- useSD_hard: for the first step, users can choose if they prefer to use
     standard variation for the weighted aggregation (useSD1 = 1)
- 2DtransformStep2: choice of the 2D transform which will be applied in the 
     second step of the algorithm. You can choose the DCT transform or the 
     Bior1.5 transform for the 2D transform in the step 1 (tau_2D_hard = dct or bior) 
     and/or the step 2. (tau_2d_wien = dct or bior).
- useSD_wien: for the second step, users can choose if they prefer to use
     standard variation for the weighted aggregation (useSD2 = 1)
- ColorSpace: 选择在哪个 color space 上处理这个图片. 
     you can choose the colorspace for both steps between : rgb, yuv, ycbcr and opp.
- patch_size: 图像块的大小
- nb_threads: 指定工作线程个数
- verbose: print additional information

Example, run
./bm3d cinput.png 40 ImDenoised.png ImBasic.png -useSD_wien \
   -tau_2d_hard bior -tau_2d_wien dct -color_space rgb

# ABOUT THIS FILE

Copyright 2011 IPOL Image Processing On Line http://www.ipol.im/

Copying and distribution of this file, with or without modification,
are permitted in any medium without royalty provided the copyright
notice and this notice are preserved.  This file is offered as-is,
without any warranty.
