cmake_minimum_required(VERSION 2.8)
project( quant )

SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
SET( CMAKE_BUILD_TYPE Release )

find_package( OpenCV REQUIRED )
add_executable( quant src/quantize.cpp)
target_link_libraries( quant ${OpenCV_LIBS} )