cmake_minimum_required(VERSION 2.8)  

project(gflag_test)
  
set(LINKER_LIBS "")  
#set(DIR_SRCS "")  

# opencv  
find_package(OpenCV REQUIRED)  
include_directories(${OpenCV_INCLUDE_DIRS})  
list(APPEND LINKER_LIBS ${OpenCV_LIBS})  
  
# glog  
# 这是从 caffe 中摘取的  
include("cmake/Modules/FindGlog.cmake")  
include_directories(SYSTEM ${GLOG_INCLUDE_DIRS})  
list(APPEND LINKER_LIBS ${GLOG_LIBRARIES})  
  
# gflags  
# 这是从caffe中获取的  
include("cmake/Modules/FindGFlags.cmake")  
include_directories(SYSTEM ${GFLAGS_INCLUDE_DIRS})  
list(APPEND LINKER_LIBS ${GFLAGS_LIBRARIES})  
  
# 指定包含 CMakeLists.txt 的子目录
# ---[ Subdirectories
# add_subdirectory(src/)

# 指定源码搜索路径 aux_source_directory(<dir> <variable>), 
# 把当前路径下 dir 目录下的所有源文件路径放到变量 variable 中
# 指定源文件目录为 "src"，并将其赋值给环境变量 "DIR_SRCS"
#AUX_SOURCE_DIRECTORY(../src, ${DIR_SRCS})
SET(DIR_SRCS "src/gflag_test.cpp")

message(STATUS "source dirs: ${DIR_SRCS}")

# 加入你自己编译成可执行文件的名称以及对应的代码 
add_executable(gflags_test ${DIR_SRCS})  

# 链接时候的库文件  
target_link_libraries(gflags_test ${LINKER_LIBS})  