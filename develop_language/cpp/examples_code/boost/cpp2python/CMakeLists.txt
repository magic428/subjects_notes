
cmake_minimum_required(VERSION 2.8)
project( hello_ext )
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )

SET( CMAKE_BUILD_TYPE Release )
#SET( CMAKE_BUILD_TYPE Debug )
set(LINKER_LIBS "")  

find_package(PythonLibs)
include_directories (${PYTHON_INCLUDE_DIRS})
list(APPEND LINKER_LIBS ${PYTHON_LIBRARIES})

find_package(Boost COMPONENTS python-py34)  #find_package(Boost 1.45.0 COMPONENTS python)
include_directories (${Boost_INCLUDE_DIRS})
list(APPEND LINKER_LIBS ${Boost_LIBRARIES})

list(APPEND LINKER_LIBS boost_python)

# Add the source in project root directory
aux_source_directory(src/python DIRSRCS)

# user defined
include_directories(/usr/include/python3.4m)
include_directories(${Boost_INCLUDE_DIRS})

# Target
message("source files: ${DIRSRCS}")
message("LINKER_LIBS files: ${LINKER_LIBS}")

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./exercise/)
# add_executable( hello_ext ${DIRSRCS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../src/python)
add_library(hello_ext SHARED ${DIRSRCS})
target_link_libraries (hello_ext
    ${LINKER_LIBS}
)
set_target_properties(hello_ext PROPERTIES PREFIX "" OUTPUT_NAME "hello_ext")