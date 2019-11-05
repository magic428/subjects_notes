cmake_minimum_required(VERSION 2.8)
project( calc_histogram )

SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
SET( CMAKE_BUILD_TYPE Release )

include_directories(src)
find_package( OpenCV REQUIRED )
add_executable( calc_histogram src/calc_histogram.cpp)
target_link_libraries( calc_histogram ${OpenCV_LIBS} )