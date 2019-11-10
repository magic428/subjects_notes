# CMake 根据编译器平台设置不同的编译选项  

## 1. 根据编译器平台设置不同的编译选项    

```cmake
# MSVC-VS
IF(MSVC)
    MESSAGE(STATUS "This is windows.") 
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /utf-8 /wd4828")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++14")
    # /arch:SSE and /arch:SSE2 are only available when you compile for the x86 platform.
    ADD_DEFINITIONS(/arch:SSE /arch:SSE2) 
# GCC
ELSEIF(CMAKE_COMPILER_IS_GNUCC) 
    MESSAGE(STATUS "This is unix.")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-switch-enum -Wno-switch -Wno-error=non-virtual-dtor")
    ADD_DEFINITIONS( -D__ANDROID__)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O3 -pthread")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
ENDIF()
```

或者  

```cmake
IF (${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
    SET(option WIN32)
    SET(win32_LIBRARIES comctl32.lib shlwapi.lib shell32.lib odbc32.lib odbccp32.lib  kernel32.lib user32.lib   gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib   odbccp32.lib)
    #SET(defs -DUNICODE -D_UNICODE)
ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
```

## 2. CMake 调用操作系统中的环境变量   

使用 $ENV{NAME} 指令就可以调用操作系统的环境变量了。   

比如获取 Windows 系统中的 Path 系统变量:  

```cmake
MESSAGE(STATUS "Path: $ENV{Path}")
```

设置环境变量的方式是:  

```cmake
SET(ENV{variable} value)
```