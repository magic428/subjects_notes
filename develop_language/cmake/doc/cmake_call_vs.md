# Windows 下 cmake 使用 

windows 下使用 cmake 时需要指定使用的编译器, 常见的是调用 VS 编译器. 通过 cmake 的命令行参数 '-G' 指定. 如:   

```bash
mkdir build && cd build
cmake ../ -G "Visual Studio 14 2015 Win64"
```

下面是调用 VS 时使用的 CMakeLists.txt, 由于 find_package() 需要定义搜索的 .cmake 文件,因此需要先设置库的搜索目录. 例如 OpenCV 库的目录和 Boost 库的目录.   

```cmake
#####################################################
# cmake ../ -G "Visual Studio 14 2015 Win64"
#####################################################

cmake_minimum_required(VERSION 2.8)
project(salmat)

#SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
#SET( CMAKE_BUILD_TYPE Release )

set( LINKER_LIBS "" )

set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories("include")

# opencv
set(OpenCV_DIR D:\\ggnbf\\thirdparty20170624\\OpenCV)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND LINKER_LIBS ${OpenCV_LIBS})

# boost filesystem  
set(BOOST_INCLUDEDIR D:/ggnbf/thirdparty20170624/Boost/include)
LINK_DIRECTORIES("D:/ggnbf/thirdparty20170624/Boost/lib64-msvc-14.0")
# manually add
list(APPEND Boost_LIBRARIES libboost_filesystem-vc140-mt-1_64.lib)
list(APPEND Boost_LIBRARIES libboost_system-vc140-mt-1_64.lib)
list(APPEND LINKER_LIBS ${Boost_LIBRARIES})
include_directories(${BOOST_INCLUDEDIR}) 

aux_source_directory(src SRC_FILES)
aux_source_directory(src/common SRC_FILES)
aux_source_directory(src/segmentation SRC_FILES)
aux_source_directory(src/saliency SRC_FILES)
aux_source_directory(src/matting SRC_FILES)
aux_source_directory(src/cluster SRC_FILES)
aux_source_directory(src/illustration SRC_FILES)

message("Source Files: " ${SRC_FILES} )
message("Header Files: " ${OpenCV_INCLUDE_DIRS} )
message("Lib Files: " ${LINKER_LIBS} ) 

# add dlclose()  
# list(APPEND LINKER_LIBS dl)

add_executable(${PROJECT_NAME} ${SRC_FILES} )
target_link_libraries(${PROJECT_NAME} ${LINKER_LIBS})
```