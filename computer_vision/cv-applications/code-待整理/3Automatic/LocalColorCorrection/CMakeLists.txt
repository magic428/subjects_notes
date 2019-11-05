cmake_minimum_required(VERSION 2.8)

project(color_correction)

SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2  -std=c99" )
 
add_executable(color_correction color_correction.c)
target_link_libraries( color_correction -lm) 
