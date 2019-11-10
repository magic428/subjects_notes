# CMakeLists.txt 模板 

- OpenCV;  
- Boost;  
- FFMPEG;  

以下是使用 OpenCV 库的程序的 CMakeLists.txt.   

```cmake
cmake_minimum_required(VERSION 2.8)
project(opencv_demo)

SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
SET( CMAKE_BUILD_TYPE Release )

set( LINKER_LIBS "" )

set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories("include")

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND LINKER_LIBS ${OpenCV_LIBS})

# boost filesystem
find_package(Boost REQUIRED COMPONENTS
system
filesystem
)
list(APPEND LINKER_LIBS ${Boost_LIBRARIES})

# FFmpeg
set(FFMPEG_ROOT D:/workSpace/thirdparty20170624/ffmpeg-20191004)
include_directories(${FFMPEG_ROOT}/include)
LINK_DIRECTORIES(${FFMPEG_ROOT}/lib)
list(APPEND FFMPEG_LIBRARIES avcodec.lib)   # 音视频编码核心库, 用于音视频或图片编解码
list(APPEND FFMPEG_LIBRARIES avdevice.lib)  # 硬件采集, 加速, 显示库
list(APPEND FFMPEG_LIBRARIES avfilter.lib)  # 音视频滤波器
list(APPEND FFMPEG_LIBRARIES avformat.lib)  # 音视频封装格式生成或解析 
list(APPEND FFMPEG_LIBRARIES avutil.lib)    # 该库中封装了一些公共的工具函数
list(APPEND FFMPEG_LIBRARIES postproc.lib)  # 封装了同步, 时间计算的简单算法
list(APPEND FFMPEG_LIBRARIES swresample.lib)# 音视频编解码格式预设
list(APPEND FFMPEG_LIBRARIES swscale.lib)   # 原始视频格式转换库
list(APPEND LINKER_LIBS ${FFMPEG_LIBRARIES})

aux_source_directory(src SRC_FILES)

message("Source Files: " ${SRC_FILES} )
message("Header Files: " ${OpenCV_INCLUDE_DIRS} )
message("Lib Files: " ${LINKER_LIBS} )

# add dlclose() 所在的 dl 库 
list(APPEND LINKER_LIBS dl)

add_executable(${PROJECT_NAME} ${SRC_FILES} )
target_link_libraries(${PROJECT_NAME} ${LINKER_LIBS})
```
