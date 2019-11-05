cmake_minimum_required(VERSION 2.8)
project( process )

SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")
find_package(Boost REQUIRED COMPONENTS
# regex
filesystem
#thread
system
timer
date_time
chrono
atomic
noncopyable
)
if(NOT Boost_FOUND)
    message("Not found Boost")
endif()

include_directories(${Boost_INCLUDE_DIRS})
message("${Boost_INCLUDE_DIRS}")
message("${Boost_LIBRARIES}")


add_executable( process src/process.cpp )
target_link_libraries(process ${Boost_LIBRARIES})