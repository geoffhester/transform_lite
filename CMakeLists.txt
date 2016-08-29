cmake_minimum_required(VERSION 2.8)

project(TransformLite)

set(CMAKE_CXX_FLAGS "-stdlib=libc++")
add_definitions("-std=c++14")

set(cmake_build_type DEBUG)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Boost REQUIRED)
include_directories(include ${Boost_INCLUDE_DIR})

add_executable(TransformDemo main.cc)
