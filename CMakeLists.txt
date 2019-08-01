cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

set (CMAKE_CXX_STANDARD 11)

project(iterative_closest_point)

# PCL library
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# CSV parsing
include_directories(inc)

# Build
add_executable (iterative_closest_point src/iterative_closest_point.cpp)
target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})
