# 在 OpenCVConfig.cmake 中学习 CMake 实践  

通过阅读 OpenCV2413 官方 OpenCVConfig.cmake, 学习在工程中使用到的 CMake 高级用法.  

1) 获取 CMakeLists.txt 文件所在的路径  

```cpp
get_filename_component( OpenCV_CONFIG_PATH 
                        "${CMAKE_CURRENT_LIST_FILE}" 
                        PATH CACHE)
```

```cpp
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
```

回退到上一层目录, 类似于 `cd ../`.  

2) if 语句 

判断文件或目录是否存在   

```cpp
if(EXISTS "${OpenCV_LIB_PATH}/OpenCVConfig.cmake")
    ...
endif()
```

判断某个函数是否被定义.  

```cpp
if(NOT COMMAND find_host_package)
    message("ARGN:" ${ARGN})
    macro(find_host_package)
        find_package(${ARGN})
    endmacro()
endif()
```

判断某个变量是否被定义.  

```cpp
if(NOT DEFINED OpenCV_MODULES_SUFFIX)
  if(ANDROID)
    string(REPLACE - _ OpenCV_MODULES_SUFFIX "_${ANDROID_NDK_ABI_NAME}")
  else()
    set(OpenCV_MODULES_SUFFIX "")
  endif()
endif()
```

判断 target-name 是否是一个编译得到的目标.  

```cpp
if(TARGET target-name)
endif()
```

True if the given name is an existing logical target name created by a call to the add_executable(), add_library(), or add_custom_target() command that has already been invoked (in any directory).  



3) 


```cpp

get_filename_component( _OpenCV_LIB_PATH 
                        "${OpenCV_LIB_PATH}/../bin" 
                         ABSOLUTE)
file(TO_NATIVE_PATH "${_OpenCV_LIB_PATH}" _OpenCV_LIB_PATH)
```

4) 正则表达式匹配  

```cpp
if(NOT ";${OpenCV_LANGUAGES};" MATCHES ";CXX;")
    enable_language(CXX)
endif()

if(MSVC_VERSION EQUAL 1400)
    set(OpenCV_RUNTIME vc8)
elseif(MSVC_VERSION EQUAL 1500)
    set(OpenCV_RUNTIME vc9)
elseif(MSVC_VERSION EQUAL 1600)
    set(OpenCV_RUNTIME vc10)
elseif(MSVC_VERSION EQUAL 1700)
    set(OpenCV_RUNTIME vc11)
elseif(MSVC_VERSION EQUAL 1800)
    set(OpenCV_RUNTIME vc12)
elseif(MSVC_VERSION EQUAL 1900)
    set(OpenCV_RUNTIME vc14)
elseif(MSVC_VERSION MATCHES "^191[0-9]$")
    set(OpenCV_RUNTIME vc15)
endif()

if(OPENCV_GCC_TARGET_MACHINE MATCHES "amd64|x86_64|AMD64")
    set(MINGW64 1)
    set(OpenCV_ARCH x64)
else()
    set(OpenCV_ARCH x86)
endif()
```
 
4) 强行覆盖已有的 CACHE Entry   

```cpp
set( OpenCV_LIB_DIR_OPT 
     "${OpenCV_LIB_PATH}" 
     CACHE PATH "Path where release OpenCV libraries are located" FORCE)
```

PATH 意味着这个变量是必须指向磁盘上已有的一个路径; 在 cmake-gui(1) 中会弹出一个文件选择对话框.  

5) include  

Load and run CMake code from a file or module.  

```cpp
include("${OpenCV_LIB_PATH}/OpenCVConfig.cmake")  
```

6) set_target_properties  

设置 target 的属性.  

```cpp
set_target_properties(opencv_videostab PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_highgui;opencv_features2d;opencv_calib3d;opencv_ml;opencv_video;opencv_legacy;opencv_objdetect;opencv_photo;opencv_gpu;opencv_core;opencv_flann;opencv_imgproc;opencv_highgui;opencv_features2d;opencv_calib3d;opencv_ml;opencv_video;opencv_legacy;opencv_objdetect;opencv_photo;opencv_gpu"
)
```

7) 如果存在 .lib 文件, 则应该确保其对应的 .dll 文件也是存在的.  

```cpp
list(APPEND _IMPORT_CHECK_FILES_FOR_opencv_features2d "${_IMPORT_PREFIX}/x64/vc14/lib/opencv_features2d2413.lib" "${_IMPORT_PREFIX}/x64/vc14/bin/opencv_features2d2413.dll" )
```

