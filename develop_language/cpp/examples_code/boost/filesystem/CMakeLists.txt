cmake_minimum_required(VERSION 2.8)
project( boost_filesys )
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET( CMAKE_BUILD_TYPE Release )
#SET( CMAKE_BUILD_TYPE Debug )

set(LINKER_LIBS "")  
find_package(Boost REQUIRED COMPONENTS
system
filesystem
)
list(APPEND LINKER_LIBS ${Boost_LIBRARIES})

# Add the source in project root directory
aux_source_directory(. DIRSRCS)

# user defined
include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})

# Target
message("source files: ${DIRSRCS}")
message("lib files: ${LINKER_LIBS}")

add_executable( boost_filesys ${DIRSRCS})
target_link_libraries( boost_filesys ${LINKER_LIBS} ) 