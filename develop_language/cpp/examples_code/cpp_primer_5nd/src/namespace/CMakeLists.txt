cmake_minimum_required(VERSION 2.8)
project( namespace )
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )

#SET( CMAKE_BUILD_TYPE Release )
SET( CMAKE_BUILD_TYPE Debug )

set(LINKER_LIBS "")  

# opencv  
find_package(OpenCV REQUIRED)  
include_directories(${OpenCV_INCLUDE_DIRS})  
list(APPEND LINKER_LIBS ${OpenCV_LIBS})  
  
# glog  
# 这是从 caffe 中摘取的  
include("${CMAKE_SOURCE_DIR}/cmake/FindGlog.cmake")  
include_directories(SYSTEM ${GLOG_INCLUDE_DIRS})  
list(APPEND LINKER_LIBS ${GLOG_LIBRARIES})  

# Add the source in project root directory
aux_source_directory(src DIRSRCS)
aux_source_directory(src/tools DIRSRCS)
aux_source_directory(. DIRSRCS)

# user defined
include_directories(include)

# Target
message("source files: ${DIRSRCS}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
add_executable( namespace ${DIRSRCS})
target_link_libraries( namespace ${LINKER_LIBS} -lcblas ) 