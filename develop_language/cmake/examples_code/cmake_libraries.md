# cmake 常用链接库

## c++11 标准  
```cmake
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")
```

## 多个模块的添加方式   
```cmake
set(LINKER_LIBS "")  

list(APPEND LINKER_LIBS ${OpenCV_LIBS}) 
list(APPEND LINKER_LIBS ${GLOG_LIBRARIES})  
...

target_link_libraries(bin ${LINKER_LIBS})  
```

## OpenCV 模块   

```cmake
find_package( OpenCV REQUIRED )

include_directories(${OpenCV_INCLUDE_DIRS})  
target_link_libraries( calibrate ${OpenCV_LIBS} )
```

## Boost 模块  
```cmake
find_package(Boost REQUIRED COMPONENTS
# regex
filesystem
thread
)

if(NOT Boost_FOUND)
    message("error: Boost Not found, need install it!!")
endif()

include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(process ${Boost_LIBRARIES})
```

## glog && gflags   
```cmake
set(LINKER_LIBS "")  

# 从 caffe 中摘取的  
include("cmake/Modules/FindGlog.cmake")  
include_directories(SYSTEM ${GLOG_INCLUDE_DIRS})  
list(APPEND LINKER_LIBS ${GLOG_INCLUDE_DIRS})  
  
# gflags  
include("cmake/Modules/FindGFlags.cmake")  
include_directories(SYSTEM ${GFLAGS_INCLUDE_DIRS})  
list(APPEND LINKER_LIBS ${GFLAGS_LIBRARIES})  
  
target_link_libraries(gflags_test ${LINKER_LIBS})  
```
