#minimum CMake Version
cmake_minimum_required(VERSION 3.2)
#project name
project(nautilus_gui)

set(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-g++")
#g++ color output
SET(CMAKE_COLOR_MAKEFILE ON)
#Header directory
include_directories(include)

# Add all source files.
file(GLOB_RECURSE SOURCES src/*.cpp)

#compile flag
set(CMAKE_CXX_FLAGS "-Wall -std=c++17 -fdiagnostics-color -pthread")

# Use the package PkgConfig to detect GTK+ headers/library files - cross-compiled
find_package(PkgConfig REQUIRED)

set(ENV{PKG_CONFIG_PATH} "/usr/lib/arm-linux-gnueabihf/pkgconfig")
pkg_search_module(GTK REQUIRED gtkmm-3.0)
include_directories(${GTK_INCLUDE_DIRS})
link_directories(${GTK_LIBRARY_DIRS})

# Build executables
add_executable(nautilus_gui ${SOURCES})
target_link_libraries(nautilus_gui ${GTK_LIBRARIES})
