# 在工程中使用到的 CMake 自定义功能总结  

## 1. 自动添加子目录中的源文件  

先识别出子目录下的源文件, 然后使用 foreach() 指令遍历.  

```cmake
file(GLOB SUBDIRS "src/*")
foreach(DIR ${SUBDIRS})
    if(IS_DIRECTORY ${DIR})
        include_directories(${DIR})
        add_subdirectory(${DIR})
    endif()
endforeach()
```

## 2. 将其他目录中的源码编译为动态库供主程序调用  

1) Linux 生成动态库文件  

其 CMakeLists.txt 的内容如下:  

```cmake
project(demolib)

# 设置动态库的编译生成目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
add_library( # Sets the name of the library.
             ${PROJECT_NAME}
             # Sets the library as a shared library.
             SHARED
             # Provides a relative path to your source file(s).
             # Associated headers in the same location as their source
             # file are automatically included.
             ${SRCS} )
#set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "" OUTPUT_NAME ${PROJECT_NAME})
```

可执行文件的 CMakeLists.txt 内容如下:  

```cmake
# 设置动态库的搜索路径
LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/lib) 
set( LINKER_LIBS "" )
list(APPEND LINKER_LIBS "demolib")  # 库名为 libconst_enum_inline.so

aux_source_directory(src SRC_LIST)

# exec bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${LINKER_LIBS})
```

## 3. 编译控制选项  

控制编译动态库和静态库;   

控制编译平台相关的库;  

控制是否编译 Demo 程序;  

```
option(ENABLE_INT8 "use int8" OFF)
option(ENABLE_AVX2 "use avx2" OFF)
option(ENABLE_NEON "whether use neon, if use arm please set it on" OFF)
option(DEMO "build the demo" OFF)

SET(fdt_base_dir   ${PROJECT_SOURCE_DIR})
SET(fdt_src_dir    ${fdt_base_dir}/src)
SET(fdt_inc_dir    ${fdt_base_dir}/src)

SET(fdt_lib_name   facedetection)
SET(fdt_lib_static ${fdt_lib_name})
SET(fdt_lib_shared ${fdt_lib_name}_shared)

FILE(GLOB_RECURSE fdt_source_files ${fdt_src_dir}/*.cpp)
LIST(SORT         fdt_source_files)

if(ENABLE_INT8)
	message("using int8")
	add_definitions(-D_ENABLE_INT8)
endif()

if(ENABLE_AVX2)
	message("using avx2")
	add_definitions(-D_ENABLE_AVX2)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2 -mfma")
endif()

if(ENABLE_NEON)
	message("using arm")
	add_definitions(-D_ENABLE_NEON)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

INCLUDE_DIRECTORIES(${fdt_inc_dir})

# Create a static library (.a)
ADD_LIBRARY(${fdt_lib_static} STATIC ${fdt_source_files})

# Create a shared library (.so)
ADD_LIBRARY(${fdt_lib_shared} SHARED ${fdt_source_files})
SET_TARGET_PROPERTIES(${fdt_lib_shared} PROPERTIES OUTPUT_NAME "${fdt_lib_name}")
SET_TARGET_PROPERTIES(${fdt_lib_shared} PROPERTIES PREFIX "lib")

# Create demo. OpenCV is requred.
if (DEMO)
    #if(WIN32)
    #    set(OpenCV_DIR "D:/opencv343/build") # TODO
    #endif()
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})

    set(fdt_demo_files ${fdt_base_dir}/example/libfacedetectcnn-example.cpp)
    add_executable(fdt_demo ${fdt_demo_files})
    target_link_libraries(fdt_demo ${fdt_lib_static} ${OpenCV_LIBS})
endif()

if (GSTREAMER)
    find_package(OpenCV REQUIRED)

    include(FindPkgConfig)
    pkg_search_module(GSTREAMER REQUIRED gstreamer-1.0)
    pkg_search_module(GSTREAMER_BASE REQUIRED gstreamer-base-1.0)
    pkg_search_module(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0)

    add_library(gstfacedetect SHARED
        example/libfacedetect.cpp
    )

    include_directories(gstfacedetect PRIVATE
        ${GSTREAMER_INCLUDE_DIRS}
        ${GSTREAMER_BASE_INCLUDE_DIRS}
        ${GSTREAMER_VIDEO_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(gstfacedetect
        ${GSTREAMER_LIBRARIES}
        ${GSTREAMER_BASE_LIBRARIES}
        ${GSTREAMER_VIDEO_LIBRARIES}
        ${OpenCV_LIBS}
        ${fdt_lib_shared}
    )
endif()
```

## 二. issues

### 2.1 CMAKE_MODULE_PATH 变量并不能为 include() 函数提供搜索路径.   

```cmake
# set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/)
message(STATUS "cmake include: ${CMAKE_MODULE_PATH}")
```

