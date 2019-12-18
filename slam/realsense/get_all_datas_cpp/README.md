# Realsense D435i +Opencv 获取彩色、深度、IMU数据并对齐 

参考realsense官方文档和各位大佬的博客，在Ubuntu18.04 系统下，得到了Realsense D435i的所有数据（如果我没猜错的话），包括它的RGB图、左右红外摄像图、深度图、IMU数据，并且将深度图数据和RGB图进行对齐。

其中IMU数据获取得有点痛苦，最后还是google了才知道怎么去提取。

没什么好说的了…… 直接上代码吧，下面都有注释。

代码的功能就是循环显示所有获得的图像和IMU数据。

源码
```cpp
#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define width 640 
#define height 480 
#define fps 30


int main(int argc, char** argv) try
{
    // judge whether devices is exist or not 
	rs2::context ctx;
	auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
	if (list.size() == 0) 
		throw std::runtime_error("No device detected. Is it plugged in?");
	rs2::device dev = list.front();
    
    //
    rs2::frameset frames;
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;//创建一个通信管道//https://baike.so.com/doc/1559953-1649001.html pipeline的解释
    //Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg;//创建一个以非默认配置的配置用来配置管道
    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);//向配置添加所需的流
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16,fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
   
    // get depth scale 
    // float depth_scale = get_depth_scale(profile.get_device());

    // start stream 
    pipe.start(cfg);//指示管道使用所请求的配置启动流

    
    while(1)
    {
        frames = pipe.wait_for_frames();//等待所有配置的流生成框架

        // Align to depth 
        rs2::align align_to_depth(RS2_STREAM_DEPTH);
        frames = align_to_depth.process(frames);
    
        // Get imu data
        if (rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL))
        {
            rs2_vector accel_sample = accel_frame.get_motion_data();
            std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;
        }
        if (rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
        {
            rs2_vector gyro_sample = gyro_frame.get_motion_data();
            std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
        }
        
        //Get each frame
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();
        rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
        rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);
        
        
        // Creating OpenCV Matrix from a color image
        Mat color(Size(width, height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        Mat pic_right(Size(width,height), CV_8UC1, (void*)ir_frame_right.get_data());
        Mat pic_left(Size(width,height), CV_8UC1, (void*)ir_frame_left.get_data());
        Mat pic_depth(Size(width,height), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
        
        // Display in a GUI
        namedWindow("Display Image", WINDOW_AUTOSIZE );
        imshow("Display Image", color);
        waitKey(1);
        imshow("Display depth", pic_depth*15);
        waitKey(1);
        imshow("Display pic_left", pic_left);
        waitKey(1);
        imshow("Display pic_right",pic_right);
        waitKey(1);
    }
    return 0;
}

// error
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
```


CMakeLists.txt  

```js
cmake_minimum_required(VERSION 3.1.0)

project(alldata)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(realsense2 REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${realsense2_INCLUDE_DIRS})
add_executable(alldata GetAlldata.cpp)

target_link_libraries( alldata ${OpenCV_LIBS} ${realsense2_LIBRARY})
```

效果
1、从左到右从上到下分别是红外左、红外右、对齐后的图像显示、深度灰度图，是视频流，如果只想获取一帧图像的话可以把代码的 while 去掉。
在这里插入图片描述

2、IMU 数据是一直打印的，这里就不放出来了。

注意：Realsense D435i 直接读出的深度图是 16 位的图像，而我们所使用的 opencv 的 Mat 常用的是8位的，因此，在图像转化方面（Mat depthimageconvert(Mat input)这个函数） C++语言里我还没有找到一个合适的方法，原因有2：
<1> 直接从 depth_stream 中读出的深度图的像素值范围为 0-65535，而我们最终要使用的图像像素灰度值范围是 0-255，因此如果直接用 I（i,j）/ 255 或者 I（i,j）/65535*255 的话（其实二者原理是一样的），这样得到的结果就会是一片黑，要乘以一个系数，使整张图像的灰度值提升才能获得可视化的结果。但是如果这样做的话会使原数据不真实。  
<2> realsense D435i 默认的 depth 图像是远白近黑，但是我们经常使用的是远黑近白，可以直接用 255-I(i,j) 进行转换;  

## 参考  

https://github.com/IntelRealSense/librealsense/tree/master/examples  
https://www.google.com/amp/s/www.intelrealsense.com/how-to-getting-imu-data-from-d435i-and-t265/amp/  
https://dev.intelrealsense.com/docs/api-how-to  
https://blog.csdn.net/LongXiaoYue0/article/details/81059635  
