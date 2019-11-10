# cmake 用法

## 基本入门  

[CMake 变量和指令函数 ](dev_tools/cmake/doc/variables_and_cmd.md)  
[CMake FILE 指令详解](/dev_tools/cmake/doc/FILE_command.md)  
[CMake FIND_xxx 指令详解](/dev_tools/cmake/doc/FILE_command.md)  

## 实践

[一个 OpenCV 程序的 CMakeLists.txt](/dev_tools/cmake/doc/opencv_CMakeLists.md)  
[在工程中使用到的功能总结](/dev_tools/cmake/doc/project_practice.md)  
[CMake 工程生成动态链接库](/dev_tools/cmake/doc/shared_link_lib.md)  
[Windows 下 CMake 调用 VS](/dev_tools/cmake/doc/cmake_call_vs.md)  
[CMake 如何自动查找库路径](/dev_tools/cmake/doc/how_cmake_find_libs_automatic.md)  
[模块的使用和自定义模块](/dev_tools/cmake/doc/user_module.md)  
[CMake 中使用 OpenMP](/dev_tools/cmake/doc/cmake_openomp.md)  
[CMake 中集成 proto 命令编译](/dev_tools/cmake/doc/cmake_protobuf.md)  

# CMakeLists.txt 模板 
# 在 CMakeLists.txt 文件之间共享变量  
# CMake 根据编译器平台设置不同的编译选项  
# CMake 控制指令 - IF/WHILE/FOREACH   

## issues  

CMAKE_MODULE_PATH 对于 include() 的搜索路径不起作用.  