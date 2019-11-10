cmake_minimum_required( VERSION 2.8 )
project( vector_operation )

SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3" )
set( CMAKE_BUILD_TYPE "Release" )

set( LINKER_LIBS "" )

# ---[ Includes
set(Caffe_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(Caffe_SRC_DIR ${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_BINARY_DIR})

# Add the source in project root directory
#aux_source_directory(. DIRSRCS)#包含主文件夹下的main.cpp文件

# Add header file include directories
# Add block directories
file(GLOB SUBDIRS "src/*")
foreach(DIR ${SUBDIRS})
    if(IS_DIRECTORY ${DIR})
        include_directories(${DIR})
        add_subdirectory(${DIR})
    endif()
endforeach()

# opencv
# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})
# list(APPEND LINKER_LIBS ${OpenCV_LIBS})

# boost filesystem
find_package(Boost REQUIRED COMPONENTS
system
filesystem
)
list(APPEND LINKER_LIBS ${Boost_LIBRARIES})

# seprate chapters 
#aux_source_directory(src/vector_operation SRC_FILES)
#aux_source_directory(src/default_paramter SRC_FILES)
#aux_source_directory(src/map_instance SRC_FILES)
#aux_source_directory(src/namespace SRC_FILES)
#aux_source_directory(src/shared_ptr SRC_FILES)
#aux_source_directory(src/string SRC_FILES)
#aux_source_directory(src/template SRC_FILES)

message("Source Files: " ${SRC_FILES} )
message("Lib Files: " ${LINKER_LIBS} )

# Target
#add_executable(${PROJECT_NAME} ${SRC_FILES} )
#target_link_libraries(${PROJECT_NAME} ${LINKER_LIBS})
#add_executable(${PROJECT_NAME} main.cpp) #上述方法有问题，这种方法才有效
#target_link_libraries(${PROJECT_NAME} hello world)# 将多目录下的 hello 库和 world 库链接上可执行文件 helloworld.o