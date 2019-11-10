cmake_minimum_required( VERSION 2.8 )
project( effective_cpp )

SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3" )
set( CMAKE_BUILD_TYPE "Release" )

# Add header file include directories
# Add block directories
file(GLOB SUBDIRS "src/*")
foreach(DIR ${SUBDIRS})
    if(IS_DIRECTORY ${DIR})
        include_directories(${DIR})
        add_subdirectory(${DIR})
    endif()
endforeach()

LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/lib) 
set( LINKER_LIBS "" )
list(APPEND LINKER_LIBS "const_enum_inline")
list(APPEND LINKER_LIBS "const")

# exec bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
aux_source_directory(src SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${LINKER_LIBS})
