cmake_minimum_required(VERSION 3.20)
project(livox-mid40-app)

set(CMAKE_CXX_STANDARD 20)
option(USE_CLANG_TIDY "Use clang tidy" OFF)
if(USE_CLANG_TIDY)
message(STATUS "Using clang tidy")
set(CMAKE_CXX_CLANG_TIDY "clang-tidy")
endif()

find_library(LIVOX_LIB livox_sdk_static)

if(NOT LIVOX_LIB)
    message(FATAL_ERROR "Livox library not found!")
endif()

include(cmake/format.cmake)

add_library(mid src/mid40.cc src/mid360.cc)
target_include_directories(mid PUBLIC include)
target_link_libraries(mid livox_sdk_static livox_lidar_sdk_static)

add_subdirectory(grpc)

add_executable(app src/app.cc)
target_link_libraries(app mid grpc_server)

add_executable(app360 src/app360.cc)
target_link_libraries(app360 mid grpc_server)