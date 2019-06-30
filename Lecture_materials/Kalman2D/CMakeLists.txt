project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

set(PROJ_NAME "Kalman_2D")
project(${PROJ_NAME})

add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

file(GLOB KF_SRC
    "src/*.h"
    "src/*.cpp"
)


add_executable(${PROJ_NAME} ${KF_SRC})

target_include_directories(${PROJ_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Eigen
)

