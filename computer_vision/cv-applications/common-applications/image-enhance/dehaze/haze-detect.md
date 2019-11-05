# 视频去雾算法 - Video-Dehaze

论文的详细介绍和结果展示可以看[这里]("http://mcl.korea.ac.kr/projects/dehazing").    

## Author:
  Jin Hwan Kim

  Media Communications Laboratory, 
  School of Electrical Engineering,
  Korea University, Seoul, Korea
  
  E-mail: arite AT korea.ac.kr
  
  Version History: 1.0 (Initial Release)  12-Mar-2013 

## Dependencies
  Visual studio 2015 with windows 10
  Mininum OpenCV 2.3.1 is required < OpenCV C API >

## Usage 

0) 使用 cmake 生成工程文件

在工程目录下执行下面的命令：  

```bash
$ mkdir build
$ cd build/  
$ cmake -G "Visual Studio 14 2015 Win64" -D OpenCV_DIR="D:/workSpace/thirdparty20170624/OpenCV2411/build" ../
```

使用 vs2015 打开生成的 .sln 工程文件，编译工程。  

1) video  

```bash
dehazing input_filename output_filename [frames]
``` 
2) image

```bash
dehazing input_filename output_filename
```

you may modify the some parameters in dehazing constructor or modify the code to add parameters to excutable file.  

Example: 

```bash
  dehazing "sample\cross.avi" "outputs\cross_out.avi" 100
```

Output:
  programe makes an outfile, which is dehazed.

## 程序中的关键参数调整  

1). 下采样的图像尺寸；  
2). 时间连续性因子中检测相邻帧变化的概率模型的 sigma；  
3). 透射率优化过程中使用的步长参数；  
4). 导向滤波使用的 block-size；  
5). 估计透射率时使用的 block-size；  
6). 视频去雾算法中，应该将 PrevFlag 设置为 true， 这样可以减少相邻帧之间透射率的突变；  
7). 算法估计得到的透射率值越小，恢复后的图像颜色越暗，表现为大块的黑色区域；  

出现大面积黑色块的原因是：模拟像素突变概率模型的 sigma 过小，导致具有变化较大区域的权重系数无法体现真实的场景变化，从而在第二帧之后得到较低的透射率值， 去雾结果呈现大面积黑色块。  

解决办法， 设置合适的 sigma 值（500）即可.

## Citation 

[1] J.-H. Kim, W.-D. Jang, Y. Park, D.-H. Lee, J.-Y. Sim, C.-S. Kim, "Temporally
  coherent real-time video dehazing," in Proc. IEEE ICIP, 2012.
[2] J.-H. Kim, W.-D. Jang, J.-Y. Sim, C.-S. Kim, "Optimized Contrast Enhancement 
  for Real-Time Image and Video Dehazing," J. Vis. Commun. Image R. Vol. 24, No.3,  pp.410-425, Apr, 2013.

## License 

  The source files in ./src/ directory
  
  The guided filter is He et al.'s guided image filtering, and we implement in C.  
  
  The other part will be useful but WITHOUT ANY WARRANTY;  
  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    

  Permission is granted for research and educational use.  
 
  If you want to include this software in a commercial product, please contact 
  the author.  
