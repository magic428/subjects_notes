# 使用 CMake 生成动态链接库后被程序调用   

## 将各个模块编译成动态库  

将代码封装为动态库供别人调用, 而不是直接提供源码.   

或者为了便于管理自己学习时创建的工程, 在 main.cpp 中调用每个模块中实现好的 demo() 函数.

请看 cmake 代码:  

```cmake
cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)


set(LIB_SRC hellocmake.cpp)  # 设置编译动态库所需的源文件
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib) # 设置动态库的保存路径  

add_library(libhellocmake SHARED ${LIB_SRC})  # 指定将源文件编译为动态库, SHARED 表示动态库
set_target_properties(libhellocmake PROPERTIES OUTPUT_NAME "hellocmake")  # 设置动态库名
```
详细说明: 

- LIBRARY_OUTPUT_PATH 和 PROJECT_BINARY_DIR 为 cmake 系统变量;  
- LIBRARY_OUTPUT_PATH (等价于 CMAKE_LIBRARY_OUTPUT_DIRECTORY), 用来设置动态库的保存路径;  
- PROJECT_BINARY_DIR (等价于 CMAKE_BINARY_DIR) 表示运行 cmake 时的路径;  
- add_library() 设置 libhellocmake 将会被编译为动态库;  
- set_target_properties 用于设动态库置库名; 默认库名的前缀为 "lib"; 使用 `PREFIX ""` 选项可以使编译出的动态库名没有 "lib" 前缀; 如 `set_target_properties(libhellocmake PROPERTIES PREFIX "" OUTPUT_NAME "hellocmake")` 编译出的动态库名为 'hellocmake.so';  

## main 程序中调用  

编译得到的库文件保存在特定的目录下, 通过在主程序中设置链接该编译好的动态库就可以调用其中的函数.  

```cmake
cmake_minimum_required(VERSION 2.8)

# PROJECT_SOURCE_DIR 工程的根目录, 即顶层 CMakeLists.txt 所在的目录
# 向工程添加多个特定的头文件搜索路径
include_directories(
    include
)

set(SRC_list main.cpp) # 宏定义, 表示可执行文件对应的源文件列表

# CMAKE_RUNTIME_OUTPUT_DIRECTORY 重新设置二进制可执行文件的存放位置
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 指定库文件路径
link_directories(${PROJECT_BINARY_DIR}/lib)

add_executable(main ${SRC_list})  #生成可执行文件

target_link_libraries( main 
                       libhello
                       libhellocmake )
```

源程序要想调用编译好的库文件, 只需要在 cmake 中指定链接需要用到的库即可, 比如 libhello 和 libhellocmake.  

## 比较动态库和可执行文件的 CMakeLists.txt 写法   

如果不使用链接动态库的方式, 而是将所有的文件全部写入一个总的 CMakeLists.txt 中, 这样就会使这个 CMakeLists.txt 看起来很复杂. 其他地方需要使用这些功能的时候需要将源文件拷贝过去, 比较繁琐. 动态链接库的使用可以方便我们管理各个模块, 在一定程度上保证代码的私密性.  

