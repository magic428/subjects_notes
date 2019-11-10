cmake_minimum_required( VERSION 2.8 )
project( bigtalk_design_patterns )

SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3" )
set( CMAKE_BUILD_TYPE "Release" )

# Add header file include directories
file(GLOB HEADER_DIRS "include/*")
foreach(HEADER_DIR ${HEADER_DIRS})
    if(IS_DIRECTORY ${HEADER_DIR})
        include_directories(${HEADER_DIR})
    endif()
endforeach()

# Add block directories
file(GLOB SRC_DIRS "src/*")
foreach(SRC_DIR ${SRC_DIRS})
    if(IS_DIRECTORY ${SRC_DIR})
        STRING( REGEX REPLACE ".*/(.*)" "\\1" LIB_NAME ${SRC_DIR} ) 
        list(APPEND LINKER_LIBS ${LIB_NAME})
        add_subdirectory(${SRC_DIR})
    endif()
endforeach()

LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/lib) 
MESSAGE(STATUS "Local Libs: " ${LINKER_LIBS})

# exec bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
aux_source_directory(src SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${LINKER_LIBS})
