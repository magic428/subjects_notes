cmake_minimum_required(VERSION 2.8)
set(LIB_SRC hello.cpp)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)  
#LIBRARY_OUTPUT_PATH 重新定义目标链接库文件的存放位置
#PROJECT_BINARY_DIR 运行cmake命令时的目录   

#add_compile_options(-std=c++11 -fPIC)
add_library(libhello SHARED ${LIB_SRC})
set_target_properties(libhello PROPERTIES OUTPUT_NAME "hello")
#SET_TARGET_PROPERTIES
#设置输出的名称,设置动态库的版本和API版本
