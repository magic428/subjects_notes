# 视频去雾算法的代码实现学习  

(1) 估计大气光值 

```cpp
void dehazing::AirlightEstimation(IplImage* imInput)
```

通过一个索引 nMaxIndex 来指示四分搜索的下一个待分割子区域。  

$$\parallel (I_r(p), I_g(p), I_b(p)) - (255, 255, 255)\parallel$$


```cpp
position = nY*nStep+nX*3;
nDistance = (255-img[position])^2) + (255-img[position+1])^2 +(255-img[position+2])^2;
```

既然这个距离只是用来相互比较以确定最小距离的像素点（并不是直接使用这个距离值），那就没必要开根号了。  

