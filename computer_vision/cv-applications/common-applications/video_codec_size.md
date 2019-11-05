# OpenCV 保存各种编码器下视频文件占用空间对比   

打开视频文件或摄像头视频需要使用 OpenCV 中的 VideoCapture 类，保存视频或摄像头视频到本地磁盘，需要使用 OpenCV 中的 VideoWriter类，使用都很简单，这篇文章就记录一下 VideoWriter 类的用法，主要关注一下 VideoWriter 在不同编码方式下保存视频文件大小的区别。    

VideoWriter 类的一个常用构造方式如下：    

```cpp
VideoWriter(const string& filename, 
            int fourcc, 
            double fps,
            Size frameSize, 
            bool isColor=true);  
```
其中 fourcc 代表了所使用的编码方式，如果输入-1，则会在运行时候弹出选择对话框，可以选择编码器.    

其他的 int 型的 CV_FOURCC() 所代表的编码器如下：    
```
CV_FOURCC('P','I','M','1') = MPEG-1 codec
CV_FOURCC('M','J','P','G') = motion-jpeg codec
CV_FOURCC('M', 'P', '4', '2') = MPEG-4.2 codec
CV_FOURCC('D', 'I', 'V', '3') = MPEG-4.3 codec
CV_FOURCC('D', 'I', 'V', 'X') = MPEG-4 codec
CV_FOURCC('U', '2', '6', '3') = H263 codec
CV_FOURCC('I', '2', '6', '3') = H263I codec
CV_FOURCC('F', 'L', 'V', '1') = FLV1 codec
```

下边这个例子使用了以上所列的各种编码方式保存的摄像头视频，在我的机器上第 6、7 种方式，即 H263 和 H263I 两种方式不能使用，可能是本机上没有对应的编解码器。    

```cpp
#include <opencv2/highgui/highgui.hpp>    
#include <opencv2/imgproc/imgproc.hpp>    
#include <opencv2/core/core.hpp>   
  
using namespace cv;  
using namespace std;  
  
int main(int argc,char *argv[])    
{    
    VideoCapture videoInput(0);  
    if(!videoInput.isOpened()) {  
        return -1;  
    }  

    float fpsInput=24; //获取帧率  
    float pauseInput=1000/fpsInput;  //设置帧间隔  
    Mat frame;  
    Size videoSize=Size(videoInput.get(CV_CAP_PROP_FRAME_WIDTH),videoInput.get(CV_CAP_PROP_FRAME_HEIGHT));  
  
    string videoPath1="D:\\videoRecordPIM1.avi";  
    int fourcc1=CV_FOURCC('P','I','M','1');  
    VideoWriter videoOutput1(videoPath1,fourcc1,fpsInput,videoSize,true);  
  
    string videoPath2="D:\\videoRecordMJPG.avi";  
    int fourcc2=CV_FOURCC('M','J','P','G');  
    VideoWriter videoOutput2(videoPath2,fourcc2,fpsInput,videoSize,true);  
  
    string videoPath3="D:\\videoRecordMP42.avi";  
    int fourcc3=CV_FOURCC('M', 'P', '4', '2');  
    VideoWriter videoOutput3(videoPath3,fourcc3,fpsInput,videoSize,true);  
  
    string videoPath4="D:\\videoRecordDIV3.avi";  
    int fourcc4=CV_FOURCC('D', 'I', 'V', '3');  
    VideoWriter videoOutput4(videoPath4,fourcc4,fpsInput,videoSize,true);  
  
    string videoPath5="D:\\videoRecordDIVX.avi";  
    int fourcc5=CV_FOURCC('D', 'I', 'V', 'X');  
    VideoWriter videoOutput5(videoPath5,fourcc5,fpsInput,videoSize,true);     
  
    string videoPath8="D:\\videoRecordFLV1.avi";  
    int fourcc8=CV_FOURCC('F', 'L', 'V', '1');  
    VideoWriter videoOutput8(videoPath8,fourcc8,fpsInput,videoSize,true);  
  
    if(!videoOutput1.isOpened()) {  
        return -1;  
    }  
    if(!videoOutput2.isOpened()) {  
        return -1;  
    }  
    if(!videoOutput3.isOpened()) {  
        return -1;  
    }  
    if(!videoOutput4.isOpened()) {  
        return -1;  
    }  
    if(!videoOutput5.isOpened()) {  
        return -1;  
    }  
      
    if(!videoOutput8.isOpened()) {  
        return -1;  
    }  
  
    while(true) {  
        videoInput>>frame;  
        if(frame.empty()||waitKey(pauseInput)==27) {  
            break;  
        }  

        videoOutput1<<frame;  
        videoOutput2<<frame;  
        videoOutput3<<frame;  
        videoOutput4<<frame;  
        videoOutput5<<frame;  
        videoOutput8<<frame;  
        imshow("Video",frame);  
    }  
    waitKey();  
    return 0;    
}  
```


通过摄像头录制了长约 5 分钟的视频，在 D 盘下生成的文件如下：




生成文件占用空间最小的编码方式是 MPEG-4.2 codec，约为19MB。用红线段标识，对应在VideoWriter类的构造函数参数为CV_FOURCC('M', 'P', '4', '2') 。    

最大的是 MPEG-1 codec，用蓝线段标识，约为111MB。对应在 VideoWriter 类的构造函数参数为 CV_FOURCC('P','I','M','1') ，所占磁盘空间是前者的 5.7 倍。所以如果需要 24 小时全天候录制监控，基于磁盘空间的考虑，可以优先使用 MPEG-4.2 的编解码方式。    


