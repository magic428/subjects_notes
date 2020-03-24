cmake_minimum_required(VERSION 2.8)
project( make_your_app_flys )
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2  -std=c++11" )

SET( CMAKE_BUILD_TYPE Release )

set(LINKER_LIBS "")  
find_package( OpenCV REQUIRED )
list(APPEND LINKER_LIBS ${OpenCV_LIBS})

# Add the source in project root directory
aux_source_directory(src SRCS)

# user defined
# include_directories(include)

message("source files: ${SRCS}")
message("LINKER_LIBS: ${LINKER_LIBS}")

# Target
add_executable( make_your_app_flys ${SRCS})
target_link_libraries( make_your_app_flys ${LINKER_LIBS}) 