cmake_minimum_required(VERSION 3.11)
project(KDTest)

set(CMAKE_CXX_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(kdtest kdtest.cc slam6d/kd.cc)
target_include_directories(kdtest PRIVATE include)
target_link_libraries(kdtest ${OpenCV_LIBS})