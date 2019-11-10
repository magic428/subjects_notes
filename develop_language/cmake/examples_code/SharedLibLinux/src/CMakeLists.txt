cmake_minimum_required(VERSION 2.8)
include_directories(
    ${PROJECT_SOURCE_DIR}/libhello
    ${PROJECT_SOURCE_DIR}/libhelloclass
                    )
#PROJECT_SOURCE_DIR 工程的根目录
#向工程添加多个特定的头文件搜索路径

set(SRC_list main.cpp) #宏定义

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)  
#EXECUTABLE_OUTPUT_PATH 重新定义目标二进制可执行文件的存放位置


link_directories(${PROJECT_BINARY_DIR}/lib)
add_executable(main ${SRC_list})  #生成可执行文件


target_link_libraries(main 
                        libhello
                        libhelloclass 
                        )
