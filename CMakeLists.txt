cmake_minimum_required(VERSION 3.16)

project(imu_filter_cpp VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_FIND_LIBRARY_SUFFIXES .a)

find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}_lib INTERFACE)
target_include_directories(${PROJECT_NAME}_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include/ ${EIGEN3_INCLUDE_DIR})
# target_link_libraries(${PROJECT_NAME}_lib INTERFACE )

add_executable(${PROJECT_NAME}_example examples/example.cpp)
target_link_libraries(${PROJECT_NAME}_example  ${PROJECT_NAME}_lib)