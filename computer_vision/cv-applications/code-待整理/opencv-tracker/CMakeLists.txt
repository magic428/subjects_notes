cmake_minimum_required(VERSION 2.8)
project( tracker )
find_package( OpenCV REQUIRED )

include_directories(include)
aux_source_directory(src SOURCE_FILES)

message("source files: " ${SOURCE_FILES})
add_executable( tracker ${SOURCE_FILES})
target_link_libraries( tracker ${OpenCV_LIBS} )