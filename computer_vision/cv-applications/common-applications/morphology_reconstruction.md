
# OpenCV 实现 Matlab 的 imreconstruct 函数   

## 0. 形态学重构

形态学重构, 直观的将就是对一幅 marker 图像连续膨胀, 直到其轮廓和另一张 mask 图像重合. 形态学重构过程中, marker 标记图像的峰值点被扩散(spread out), 即膨胀.   


  

This figure illustrates this processing in 1-D. Each successive dilation is constrained to lie underneath the mask. When further dilation ceases to change the image, processing stops. The final dilation is the reconstructed image. (Note: the actual implementation of this operation in the toolbox is done much more efficiently. See the imreconstruct reference page for more details.) The figure shows the successive dilations of the marker.


![在这里插入图片描述](https://img-blog.csdnimg.cn/2018110709275552.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2d6ajIwMTM=,size_16,color_FFFFFF,t_70)

## 1. 形态学运算基础 

形态学中最基本的工具是结构元素. 结构元素定义为一个结构和原点(锚点). 形态学运算就是对图像的每个像素应用这个结构元素, 即每个像素和结构元素执行卷积运算.  

原则上说, 结构元素可以是任何形状, 但通常只使用简单的形状, 如方形, 圆形或菱形, 而原点位于中心位置.  

腐蚀操作是替换当前像素为在结构元素定义的像素集合中找到的最小像素值.  

膨胀操作是替换当前像素为在结构元素定义的像素集合中找到的最大像素值.  

形态学闭运算定义为对图像先膨胀, 再腐蚀. 可以将误分割成碎片的物体重新连接.  

形态学开运算定义为对图像先腐蚀, 再膨胀. 可以去除图像噪点引起的小像素块.  

## 2. MATLAB - imreconstruct 函数 

```matlab
imreconstruct(marker,mask,conn)
imreconstruct(marker,mask)
```

imreconstruct 图像重构是一种形态学变换, 运算涉及两张图和一个结构元素.  其中一张图是标记图像(marker), 标记了那些开始形态学变换的点; 另一张是掩模图像(mask), 约束参与形态学变换的区域. 结构元素用来定义连通性(connectivity). 默认使用 8-connectivity, 即使用 3 * 3 的矩阵, 中心位于 (2, 2). 

常见的有二值图像(binary images)和灰度图像(gray-scale)的形态学重构变换.  

### 2.1 二值图像(binary images)

对于掩模图像 $G$, 标记图像 $F$, $R_G(F)$ 为根据 $F$ 对 $G$ 重构后的图像. 通过下面给出的迭代流程计算 $R_G(F)$ :  

1. 用标记图像 $F$ 初始化 $h_1$;  
2. 创建结构元素: $B = ones(3)$;  
3. repeat:
        $\quad \quad h_{k + 1} = ( h_k \otimes  B ) \cap G$
    直到 $h_{k + 1} = h_k$.
4. $R_G(F) = h_{k+1}$ .

标记图像 $F$ 必须是掩模图像 $G$ 的真子集:   

$$F \subseteq G$$

图 10.21 中描述了图像重构的迭代过程. 尽管上述的公式便于解释形态学重构, 但是在实际应用中存在更快的算法. imreconstruct 使用 “fast hybrid reconstruction” 算法实现, 该算法是 Vincent 于 1993 年提出.  

![Fig-10.21](https://img-blog.csdnimg.cn/20181106145626561.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2d6ajIwMTM=,size_16,color_FFFFFF,t_70)

### 2.2 Opening by Reconstruction   

在形态学开运算中, 腐蚀操作可以有效地移除小像素块, 接下来的膨胀操作用于恢复当前保留下来的目标外形. 然而, 开运算对目标外形的恢复精度取决于外形和结构元素的相似程度.  

开运算和开运算重构.  

opening by reconstruction 方法, 可以恢复腐蚀后得到的目标的原始外形.

下面是一个比较形态学开运算和形态学重构开运算的例子, 这个例子用于提取那些在垂直方向上有长笔画的字符.  

开运算和开运算重构都包含腐蚀运算, 使用一个窄的垂直结构元素, 垂直长度为字符的高度.  

```matlab
f = imread('book_text_bw.tif');
fe = imerode(f, ones(51, 1));   % 腐蚀结果见 Figure 10.22(b). 
fo = imopen(f,ones(51, 1));  % imopen() 的结果见 Fig. 10.22(c).
fobr = imreconstruct(fe, f); % imreconstruct() 的结果见 Fig. 10.22(d).
```

从 Figure 10.22(c)-(d) 可以看出, imopen() 只得到了字符的垂直笔画, 字符的其余部分并没有恢复; 而 imreconstruct () 得到了包含垂直笔画的完整字符.  


![Fig-10.22](https://img-blog.csdnimg.cn/20181106145723360.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2d6ajIwMTM=,size_16,color_FFFFFF,t_70)   

填充空洞和去除边界在这不作研究.  

## 3. OpenCV 形态学重构的基本算法流程   

```cpp
/**
 * \brief: morpy-Reconstruct, 并不是纯粹的形态学重构算法, 并不满足 marker <= mask  
 * 
 * \param:  marker is the source img
 *          mask is label img
 * 
 * \return: dst is the output
 */ 
void Reconstruct(Mat marker, Mat mask, Mat& dst) 
{ 
    Mat se=getStructuringElement(MORPH_RECT,Size(3,3)); 
    Mat tmp1(marker.size(), marker.type()), tmp2(marker.size(), marker.type()); 
    cv::min(marker, mask, dst);   

    do { 
        dst.copyTo(tmp1); 
        dilate(dst, mask, se); 
        cv::min(marker, mask, dst); 
        tmp2=abs(tmp1-dst); 
    } while (sum(tmp2).val[0] != 0); 
}
// 或者下面这种实现方式
Mat morphReconstruct(Mat marker, Mat mask, Mat& dst)
{
    cv::min(marker, mask, dst);
    dilate(dst, dst, Mat());
    cv::min(dst, mask, dst);
    Mat temp1 = Mat(marker.size(), CV_8UC1);
    Mat temp2 = Mat(marker.size(), CV_8UC1);
    do
    {
        dst.copyTo(temp1);
        dilate(dst, dst, Mat());
        cv::min(dst, mask, dst);
        compare(temp1, dst, temp2, CV_CMP_NE);
    } while (sum(temp2).val[0] != 0);
}
```

这并不是最优的形态学重构算法, 但是它只使用了基本的图像运算, 并且可以取得很好的效果.


## 4. 形态学重构原理   

![在这里插入图片描述](https://img-blog.csdnimg.cn/2018110613375290.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2d6ajIwMTM=,size_16,color_FFFFFF,t_70)

## 5. 总结 

在形态学梯度图像的基础上，利用形态学的开闭重构运算对梯度图像进行重构，在保留重要区域伦敦的同时去除细节和噪声。

 分水岭变换存在过分割现象，原因在于检测的局部极值过多，造成极值过多的原因在于图像中的非规则灰度扰动和噪声。对于好的分水岭图像分割方法，不仅能消除过分割现象，而且应保证分割后的区域伦敦边缘具有较准确的定位能力。

方法1：利用形态学开闭重构运算对原始图像的形态学梯度图像进行滤波重构，在简化梯度图像的同时，保持伦敦分水线的准确定位，消除产生过分分割现象的根源。分割过程中需选择一个结构元素，以对图像进行形态学重构。该结构元素对图像应该有:处理后，图像中的灰度跃变急剧增强，消除梯度对边缘方向的依赖性，同时，结构元素半径较小，避免梯度图像产生过厚边缘造成的区域伦敦定位误差。由于噪声的影响，采用形态学开闭重构运算对梯度图像进行重构。消除噪声，保留重要的伦敦极值信息。随着结构元素的递增，图像中的局部极值会消除，而不会产生新的区域极值。

## 参考资料  

[1]: Vincent, L., "Morphological Grayscale Reconstruction in Image Analysis: Applications and Efficient Algorithms," IEEE Transactions on Image Processing, Vol. 2, No. 2, April, 1993, pp. 176-201.
[2]: http://blog.sina.com.cn/s/blog_67f77fc80101fyn1.html   