# 图像处理   

./dehazing_korea /home/klm/work/td_marco/images/video/coal_machine.avi output.avi 1000


## 计算机视觉基础   
- [计算机视觉基础1 - 视差与深度信息](./doc/computer_vision_basement/disparity_depth.md)  
- [计算机视觉基础2 - 相机成像的几何描述 ](./doc/computer_vision_basement/camera_geometry.md)  
- [计算机视觉基础3 - 内部参数描述](./doc/computer_vision_basement/camera_inner_params.md)  
- [计算机视觉基础4 - 对极几何](./doc/computer_vision_basement/epipolar_geometry.md)  
- [计算机视觉基础5 - 本质矩阵与基本矩阵](./doc/computer_vision_basement/fundamental_matrix.md)  

## 图像去噪   
- [图像去噪算法总结](./doc/denoise/denoise_sum.md)   
- [双边滤波算法原理及实现](./doc/denoise/denoise_bilaterFilter.md)   
- [基于局部均值方差统计信息的图像去噪](./doc/denoise/denoise_lee.md)   
- [OpenCV 图像去噪算法 - 归一化,高斯,中值,双边滤波](./doc/denoise/denoise_opencv.md)  

## 图像去雾算法   
- [ 暗通道先验去雾算法原理及C++代码实现 - 何凯明](./doc/defrog/haze_dark_channel.md)   

@article{JHKIM_JVCI_2013, 
author = {Jin-Hwan Kim and Won-Dong Jang and Jae-Young Sim and Chang-Su Kim}, 
title = {Optimized contrast enhancement for real-time image and video dehazing}, 
journal = {J. Vis. Commun. Image R.}, 
volume = {24}, 
number = {3}, 
pages = {410--425}, 
month = Feb, 
year = {2013} 
}

1: 只处理24位图像。    
2: Src和Dest可以相同，相同和不同时速度无差异。    
3: 采用了下采样优化算法，在保留去雾效果的同时，提高了算法的实时性。    
4: 下采样参数不易小于0.5，否则用于缩放所占用的时间可能会比小图计算透射率带来的时间收益还大，建议取值0.25。     

- [OpenCV implementation of "A Fast Semi-Inverse Approach to Detect and Remove the Haze from a Single Image - ACCV 2010", Restorration works almost perfectly, it ends up being a little darker in places due to the nature of the blending I used](https://github.com/eokeeffe/FastSemiInverse-Dehazing)   
- [A cross-platform image dehazing/defogging mobile app implemented with React Native, Djinni and OpenCV, based on dark channel prior and fast guided filter.](https://github.com/yenshih/Dehaze)   
- [This MATLAB code is an implementation of the single image dehazing algorithm proposed in the paper "A Fast Single Image Haze Removal Algorithm using Color Attenuation Prior" by Qingsong Zhu, Jiaming Mai and Ling Shao](https://github.com/JiamingMai/Color-Attenuation-Prior-Dehazing)     

## 图像增强   
- [OpenCV 图像增强算法实现 - 直方图均衡化、拉普拉斯、Log、Gamma ](./doc/enhance/enhance_opencv.md)  
- [Retinex 图像增强算法实现](./doc/enhance/enhance_retinex.md)  

## Others  
- [OpenCV 保存各种编码器下视频文件占用空间对比](./doc/video_codec_size.md)  
- [图像直方图和反向投影的肤色检测](./doc/skin_detect.md)   
