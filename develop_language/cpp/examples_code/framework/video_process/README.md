# Video Process 框架   

用于图像处理的框架, 使用时直接继承 FrameProcessor 类, 然后重写 process() 方法;  

```cpp
virtual void process(cv:: Mat &input, cv:: Mat &output) = 0;
```

## 框架中包含的文件  

### 1. 抽象基类   

videoprocessing.cpp  
videoprocessor.h  

Reading Video Sequences  
Processing the Video Frames  
Writing Video Sequences  

### 2. 目标跟踪实例   

featuretracker.h  
tracking.cpp  

Tracking Feature Points in Video  

### 3. 前景检测实例   

BGFGSegmentor.h  
foreground.cpp  

Extracting the Foreground Objects in Video  


cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
cv::cornerSubPix(gray, features, winSize, cv::Size(-1,-1), termcrit);