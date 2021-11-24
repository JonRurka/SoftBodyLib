cmake_minimum_required(VERSION 3.17)
project(SoftBodyLib)

set(CMAKE_CXX_STANDARD 17)
set(CONAN_DISABLE_CHECK_COMPILER true)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS 1)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

if (UNIX)
    set(CMAKE_CXX_FLAGS -pthread)
endif ()

add_library(SoftBodyLib SHARED source/library/dll_main.cpp)
add_executable(SoftBodyLib_tester source/executable/exe_main.cpp)

TARGET_LINK_LIBRARIES(SoftBodyLib_tester SoftBodyLib)