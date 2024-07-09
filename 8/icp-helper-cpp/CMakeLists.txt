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

include_directories("${PROJECT_SOURCE_DIR}/include")

add_executable(simulateICP src/simulateICP.cc src/generate.cc src/helper.cc)
target_link_libraries(simulateICP ${OpenCV_LIBS} newmat)