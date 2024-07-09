cmake_minimum_required(VERSION 3.11)
project(Normals)

set(CMAKE_CXX_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Newmat
add_subdirectory("newmat-10")
set(NEWMAT_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/newmat-10")
set(NEWMAT_LIBRARIES newmat)
include_directories(${NEWMAT_INCLUDE_DIRS})

add_executable(calcNormals calcNormals.cc normals.cc)
target_link_libraries(calcNormals ${OpenCV_LIBS} newmat)

add_executable(2 main.cpp normals.cc)
target_link_libraries(2 ${OpenCV_LIBS} newmat)