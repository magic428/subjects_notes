cmake_minimum_required(VERSION 2.8)
project( skin_detect )
find_package( OpenCV REQUIRED )
add_executable( skin_detect skin_detect.cpp)
target_link_libraries( skin_detect ${OpenCV_LIBS} )