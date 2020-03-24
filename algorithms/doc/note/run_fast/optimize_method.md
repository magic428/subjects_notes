# 常用的优化方法   

## 1. 近似计算    

## 2. 使用积分图    

## 3. CPU 并行指令   

## 4. 带显卡的 cuda 优化.   

总体的CUDA优化分以下几个策略：    
- 最大化并行执行来获得最大的利用率  
- 优化内存使用方法来获得最大的内存吞吐量  
- 优化指令使用方式来获得最大的指令吞吐量   

## 5. 避免计算开根号来提高效率    

## std::vector ? -> no   

std::vector 的效率很低, 尽量不要使用, 实际中使用数组.    

## 浮点数转换为整数运算, 整数运算转换为位运算.   

比如对于 RGB2YUV, 对于 Y 分量的计算:   
```cpp
Y = 0.299 R + 0.587 G + 0.114 B 
```
```cpp
// because: 19595 / (2 >> (16-1)) = 0.299

Y = (19595 * (uchar)imInput->imageData[nY*nStep+nX*3+2]
   + 38469 * (uchar)imInput->imageData[nY*nStep+nX*3+1] 
   + 7471 * (uchar)imInput->imageData[nY*nStep+nX*3]) >> 16;
```

## 查表运算   

## 并行运算    

