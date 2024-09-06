cmake_minimum_required(VERSION 3.11)
project(ConvexOptimizationAdapter)

set(CMAKE_MINSIZEREL_POSTFIX _msz)
set(CMAKE_RELEASE_POSTFIX _rel)
set(CMAKE_DEBUG_POSTFIX _dbg)

find_package(JNI)

add_subdirectory(csrc)
