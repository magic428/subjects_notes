cmake_minimum_required(VERSION 3.1.0)

project(filter_motion_data)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(PCL 1.6 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(realsense2 REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${realsense2_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} filter_motion_data.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${realsense2_LIBRARY} ${PCL_LIBRARIES} )