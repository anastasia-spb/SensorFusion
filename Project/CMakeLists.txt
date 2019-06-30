cmake_minimum_required (VERSION 3.5)

set(PROJ_NAME "ExtendedKF")
project(${PROJ_NAME})


add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

file(GLOB EKF_SRC
    "src/*.h"
    "src/*.cpp"
)

add_executable(${PROJ_NAME} ${EKF_SRC})
