cmake_minimum_required(VERSION 2.8)

PROJECT(tracker)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

aux_source_directory(src DIR_SRCS)
include_directories(include)
add_executable(tracker ${DIR_SRCS})


include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(tracker ${OpenCV_LIBS})
