cmake_minimum_required(VERSION 3.20)
project(livox-mid40-app)

find_library(LIVOX_LIB livox_sdk_static)

if(NOT LIVOX_LIB)
    message(FATAL_ERROR "Livox library not found!")
endif()

include(cmake/format.cmake)

add_library(mid40 src/mid40.cc)
target_include_directories(mid40 PUBLIC include)
target_link_libraries(mid40 livox_sdk_static)

add_executable(app src/app.cc)
target_link_libraries(app mid40)