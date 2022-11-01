cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(ukf)

find_package(Eigen3 REQUIRED)
include_directories(${CMAKE_SOURCE_DIR})
link_directories(${Eigen_INCLUDE_DIRS})


add_executable (ukf src/main.cpp src/ukf.cpp)
target_link_libraries (ukf ${Eigen_LIBRARIES})

