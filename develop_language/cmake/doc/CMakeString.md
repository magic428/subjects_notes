# CMake 字符串操作  



```cmake
# 获取当前路径的 basename  
STRING( REGEX REPLACE ".*/(.*)" "\\1" CURRENT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR} ) 

# 获取上一级目录
STRING( REGEX REPLACE ".*/(.*)/.*" "\\1" CURRENT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR} ) 

# 匹配 SRC_FILES 中后缀为 .cpp 的源文件
string(REGEX MATCH ".*/*.cpp" CPP_Files ${SRC_FILES})
```
