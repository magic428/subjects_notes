cmake_minimum_required( VERSION 2.8 )
project( shared_ptr )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
add_executable( shared_ptr shared_ptr.cpp  )
