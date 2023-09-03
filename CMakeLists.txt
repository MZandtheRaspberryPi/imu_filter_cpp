cmake_minimum_required(VERSION 3.16)

project(imu_filter_cpp VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_FIND_LIBRARY_SUFFIXES .a)

# find_package(Eigen3 REQUIRED)
add_library(${PROJECT_NAME}_lib ${CMAKE_CURRENT_SOURCE_DIR}/src/filter_base.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/filter_saito_ekf.cpp)
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/ ${CMAKE_CURRENT_SOURCE_DIR}/eigen/)

add_executable(${PROJECT_NAME}_main ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME}_lib)
