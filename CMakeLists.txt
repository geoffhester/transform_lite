cmake_minimum_required(VERSION 3.3)

project(transform_lite)

set(CMAKE_CXX_FLAGS "-stdlib=libc++")
add_definitions("-std=c++14")

set(cmake_build_type DEBUG)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Boost REQUIRED)
add_library(boost INTERFACE IMPORTED)
set_property(TARGET boost PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})

add_subdirectory(poena)
add_subdirectory(constexprstring)
add_subdirectory(transformlite)
add_subdirectory(examples)