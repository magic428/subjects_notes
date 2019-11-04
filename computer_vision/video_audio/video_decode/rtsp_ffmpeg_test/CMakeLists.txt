#####################################################
# cmake ../ -G "Visual Studio 14 2015 Win64"
#
# VS 开启 OpenMP: 
#    属性 | 配置属性 | C/C++ | 语言 | “OpenMP 支持” 选择 “是” 
#
#####################################################
cmake_minimum_required(VERSION 2.8)
project( rtsp_test )

if(MSVC)
    # 抑制生成 ZERO_CHECK 工程
    set(CMAKE_SUPPRESS_REGENERATION ON) 
    # set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wno-dev" )
    # warning C4996: 'sprintf': This function or variable may be unsafe.
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /utf-8  /wd4828 /wd4996")
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin )
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/bin )
elseif(CMAKE_COMPILER_IS_GNUCC)
    set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -O2" )
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin )
endif()

# linked libs collections
set( LINKER_LIBS "" )
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# OpenCV
set(OpenCV_DIR D:/workSpace/thirdparty20170624/OpenCV2411/build/)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND LINKER_LIBS ${OpenCV_LIBS})

# FFmpeg
set(FFMPEG_ROOT D:/workSpace/thirdparty20170624/ffmpeg-20191004)
set(FFMPEG_INCLUDEDIR ${FFMPEG_ROOT}/include)
LINK_DIRECTORIES(${FFMPEG_ROOT}/lib)
list(APPEND FFMPEG_LIBRARIES avcodec.lib) #音视频编码核心库,用于音视频或图片编解码
list(APPEND FFMPEG_LIBRARIES avdevice.lib) #硬件采集,加速,显示库
list(APPEND FFMPEG_LIBRARIES avfilter.lib) #音视频滤波器
list(APPEND FFMPEG_LIBRARIES avformat.lib) #音视频封装格式生成或解析 
list(APPEND FFMPEG_LIBRARIES avutil.lib) #该库中封装了一些公共的工具函数
list(APPEND FFMPEG_LIBRARIES postproc.lib) #封装了同步,时间计算的简单算法
list(APPEND FFMPEG_LIBRARIES swresample.lib) #音视频编解码格式预设
list(APPEND FFMPEG_LIBRARIES swscale.lib) #原始视频格式转换库
list(APPEND LINKER_LIBS ${FFMPEG_LIBRARIES})
include_directories(${FFMPEG_INCLUDEDIR})

# user defined
include_directories(include)
set(getopt_SRCS "")

if(MSVC)
    include_directories(thirdparty)  # getopt.h
    #list(REMOVE_ITEM SRCS thirdparty/getopt.c)
    set(getopt_SRCS thirdparty/getopt.c)
endif(MSVC)

# Target 1
add_executable(rtsp2img ${getopt_SRCS} src/ffmpeg_rtsp2image.cpp )
target_link_libraries(rtsp2img ${LINKER_LIBS}) 

# Target 2
add_executable(rtsp2video ${getopt_SRCS} src/ffmpeg_rtsp2video.cpp )
target_link_libraries(rtsp2video ${LINKER_LIBS}) 

# Target 3
add_executable(rtsp_img_thread src/stream_fetch.cpp src/sf_main.cpp)
target_link_libraries(rtsp_img_thread ${LINKER_LIBS}) 

message("LINKER_LIBS: ${LINKER_LIBS}")
