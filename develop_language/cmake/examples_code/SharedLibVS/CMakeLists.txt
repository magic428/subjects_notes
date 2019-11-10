#####################################################
# cmake ../ -G "Visual Studio 14 2015 Win64"
#
# VS 开启 OpenMP: 
#    属性 | 配置属性 | C/C++ | 语言 | “OpenMP 支持” 选择 “是” 
#
#####################################################

cmake_minimum_required(VERSION 2.8)
project(dll_lib)

if(MSVC)
    # 抑制生成 ZERO_CHECK 工程
    set(CMAKE_SUPPRESS_REGENERATION ON) 
    # set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wno-dev" )
    # warning C4996: 'sprintf': This function or variable may be unsafe.
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /utf-8  /wd4828 /wd4996")
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/lib )
    #set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin )
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin )
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/bin )
elseif(CMAKE_COMPILER_IS_GNUCC)
    set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -O2" )
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin )
endif()

include_directories(dll_lib/include)

# dll-lib 
file(GLOB dllLibFiles dll_lib/src/*.cpp)
file(GLOB dllLibHeaderFiles dll_lib/include/*.h)
source_group(TREE ${CMAKE_SOURCE_DIR} FILES ${dllLibFiles})
source_group(TREE ${CMAKE_SOURCE_DIR} FILES ${dllLibHeaderFiles})

add_library(${PROJECT_NAME} SHARED 
            ${dllLibFiles}
            ${dllLibHeaderFiles} )
#set_target_properties(BasicUsageEnvironment PROPERTIES OUTPUT_NAME "helloclass")
if(MSVC)
    add_definitions(-D_WINSOCK_DEPRECATED_NO_WARNINGS)
    # target_link_libraries(${PROJECT_NAME} ws2_32.lib)
    target_link_libraries(${PROJECT_NAME} ws2_32)
endif()

# bin  
file(GLOB srcFiles src/*.cpp)
source_group(TREE ${CMAKE_SOURCE_DIR} FILES ${srcFiles})
add_executable( test ${srcFiles} )
target_link_libraries(test lib/dll_lib)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")