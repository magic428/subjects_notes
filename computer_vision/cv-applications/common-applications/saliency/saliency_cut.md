# SaliencyCut  

> 基于 GrubCut 算法改进而来.  

该方法在以下论文中提出:  

[1] MM Cheng, NJ Mitra, X Huang, PHS Torr SM Hu. Global Contrast based Salient 
    Region Detection. IEEE CVPR, p. 409-416, 2011.    

中提出的 SaliencyCut( 一种 Unsupervised salient object segmentation ) 而实现的算法.  

该算法原本是程明明教授团队实现的, 不过他们实现的是 Windows 版本, 我在这里做了移植, 将平台相关的代码用标准库(包括 boost 库) 替换, 使其具有跨平台性.   


## GrabCut   

GrabCut 通过一个简单的标注框来获取前景区域的 mask.  

我们在显著性检测算法中最终得到的结果一般是图像的显著特性图, 即 Saliency Map. 因此我们的目的是输入一张 Saliency Map 得到最终显著性区域的 mask. 而不是像 GrabCut 那样需要认为指定一个区域.    



## 算法流程  

SaliencyCut 对 GrabCut 中的两个部分做了改进, 分别是 "iterative refine" 和 "adaptive fitting".  

算法初始化:  

将显著性算法检测得到的 Saliency Map 进行阈值化(Tb)之后, 得到一个三分图(可能为前景或背景的区域, 确定为背景的区域)).  

阈值化操作中, 那些大于阈值 Tb 的像素块所在的最大连通区域被初始化为显著性目标所在的区域. 同时这个区域也称为未知区域.   

其余区域为背景区域.   

注意: 这里和 GrabCut 的不同, 我们并没有硬性指定哪里是前景区域, 而是指定了一部分未知区域, 在后续的迭代过程中确定前景像素.   

因此, 初始化过程确定了部分背景像素, 也就是 Saliency Map 阈值化之后的小于 Tb 的像素块所在的部分. 在后续的迭代过程中, 这部分区域是不会参与进来计算, 也就是说它是不会变的. 而未知区域部分是迭代过程中需要关心的, 算法的最终目的是将位置区域中属于前景和背景的像素分离开来.  

算法中使用参数来置信三分图中的背景区域. 因此我们用可以使前景 recall 率高的阈值来初始化 GrabCut 算法, 然后在迭代的过程中提高其 Precision.  

在作者给出的论文中, 在保证 recall 可以达到 95% 的前提下选择固定阈值, 将 saliency map 归一化到[0 - 255] 之间后, 选择阈值 Tb = 70.  

迭代过程:   

每次迭代结束后, 使用 dilate() 和 erode() 操作处理当前的分割结果, 将膨胀后得到的外侧区域设置为背景, 腐蚀后得到的内侧区域设置为前景, 剩下的为位置区域, 这样就得到了一个新的三分图.   


不同于 GrabCut 的是, SaliencyCut 每次迭代都会更新初始化的显著性区域. 

论文中连续进行 5 次腐蚀和膨胀操作.   

