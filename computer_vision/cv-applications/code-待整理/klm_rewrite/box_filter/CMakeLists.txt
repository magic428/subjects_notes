cmake_minimum_required(VERSION 2.8)
project( box_filter )
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2  -std=c++11" )

SET( CMAKE_BUILD_TYPE Release )

set(LINKER_LIBS "")  
find_package( OpenCV REQUIRED )
list(APPEND LINKER_LIBS ${OpenCV_LIBS})

# Add the source in project root directory
aux_source_directory(src SRCS)

# user defined
include_directories(include)

message("source files: ${SRCS}")
message("LINKER_LIBS: ${LINKER_LIBS}")

# Target
add_executable( box_filter ${SRCS})
target_link_libraries( box_filter ${LINKER_LIBS}) 