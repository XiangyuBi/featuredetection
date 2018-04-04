cmake_minimum_required(VERSION 2.8)
set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

project(X_ray_trial)

set(CMAKE_CXX_STANDARD 14)

file(GLOB_RECURSE sources src/*.cpp featuredetection/src/*.hpp)
add_executable(X_ray_trial main.cpp ${sources})

find_package(PCL REQUIRED)
if(PCL_FOUND)
    include_directories( ${PCL_INCLUDE_DIRS} )
    add_definitions( ${PCL_DEFINITIONS} )
    link_directories( ${PCL_LIBRARY_DIRS} )
    target_link_libraries(X_ray_trial ${PCL_LIBRARIES} )
endif()
