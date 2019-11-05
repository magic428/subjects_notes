cmake_minimum_required(VERSION 2.8)
project( calibrate )

#SET(CMAKE_C_COMPILER g++)
SET(CMAKE_CXX_COMPILER g++)

find_package( OpenCV REQUIRED )
add_executable( calibrate calibrate.cpp CameraCalibrator.cpp CameraCalibrator.hpp)
target_link_libraries( calibrate ${OpenCV_LIBS} )